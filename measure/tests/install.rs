//! The configuration path on the client's simulated servo: what lands, in
//! what order, what is refused before the first write, and what a refusal
//! after it leaves.

mod common;

use common::*;
use futures_lite::future::block_on;
use osc_client::data_state::{CONFIG_CORRUPT, STAMP_MISMATCH};
use osc_client::pos_lut;
use osc_client::stamp;
use osc_measure::install::{self, AFTER, STORED, Step, check, restore, save};
use osc_measure::set::SetFile;
use osc_measure::state::{self, Record};
use serde_json::json;

fn field_addr(name: &str) -> (u16, u16) {
    let d = descriptor();
    let f = d.field(name).unwrap_or_else(|| panic!("{name}"));
    (f.addr, f.width)
}

fn install_set(
    c: &mut osc_client::Client<Spy>,
    set: &SetFile,
    loosen: &[&str],
) -> (Result<(), install::Refused>, Vec<Step>) {
    let loosen: Vec<String> = loosen.iter().map(|s| s.to_string()).collect();
    let mut steps = Vec::new();
    let r = block_on(install::install(
        c,
        ID,
        &descriptor(),
        set,
        Some(&table().points),
        &loosen,
        &mut steps,
    ));
    (r, steps)
}

fn read_u16(c: &mut osc_client::Client<Spy>, name: &str) -> u16 {
    let (addr, _) = field_addr(name);
    let b = block_on(c.read(ID, addr, 2)).expect("read");
    u16::from_le_bytes([b[0], b[1]])
}

#[test]
fn install_writes_only_what_differs() {
    let (mut c, log) = on_record();
    let (r, steps) = install_set(&mut c, &record(), &[]);
    r.expect("the set it holds installs");
    assert_eq!(steps, []);
    assert_eq!(writes_below(&log, CONTROL), [], "nothing written");

    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(150.0);
    let set = rules_set(&i, &table());
    log.borrow_mut().clear();
    let (r, steps) = install_set(&mut c, &set, &[]);
    r.expect("one plant field installs");
    assert_eq!(wrote(&steps), ["velocity_limit_cps"]);
    assert!(matches!(steps.last(), Some(Step::Stamped(_))));
    assert_eq!(
        writes_below(&log, CONTROL),
        [field_addr("velocity_limit_cps"), field_addr("plant_stamp")]
    );
    assert_eq!(
        read_u16(&mut c, "velocity_limit_cps") as i64,
        set.value("velocity_limit_cps")
            .and_then(|v| v.as_i64())
            .expect("a number")
    );
    let checked = block_on(check(&mut c, ID, &descriptor(), &set)).expect("check");
    assert!(checked.clean(), "{:?}", checked.differences);
    let checked = block_on(check(&mut c, ID, &descriptor(), &record())).expect("check");
    assert_eq!(checked.differences.len(), 1, "{:?}", checked.differences);
    assert!(checked.differences[0].starts_with("velocity_limit_cps: servo 3019, set 4025"));
}

#[test]
fn every_write_is_read_back() {
    let (mut c, log) = seeded();
    let rec = record();
    let (r, steps) = install_set(
        &mut c,
        &rec,
        &every_protection(&rec)
            .iter()
            .map(String::as_str)
            .collect::<Vec<_>>(),
    );
    r.expect("the set of record installs on the seeded servo");
    let log = log.borrow();
    let mut checked = 0;
    for (k, e) in log.iter().enumerate() {
        let Ev::Write { addr, len } = *e else {
            continue;
        };
        if addr >= CONTROL {
            continue;
        }
        let next = log[k + 1..]
            .iter()
            .take_while(|e| !matches!(e, Ev::Write { .. }))
            .any(
                |e| matches!(*e, Ev::Read { addr: a, len: n } if a <= addr && addr + len <= a + n),
            );
        assert!(
            next,
            "the write at {addr:#05x} is not read back before the next write"
        );
        checked += 1;
    }
    assert!(checked > 40, "{checked} writes");
    assert!(wrote(&steps).iter().any(|w| w == "slots"));
    // The table's pages are read back by pos_lut's own install: every page
    // FETCHed after the COMMIT.
    assert!(wrote(&steps).iter().any(|w| w == "table"));
}

#[test]
fn the_table_lands_live_and_the_stamp_matches() {
    let (mut c, _) = seeded();
    let rec = record();
    let names = every_protection(&rec);
    let (r, steps) = install_set(
        &mut c,
        &rec,
        &names.iter().map(String::as_str).collect::<Vec<_>>(),
    );
    r.expect("installs");
    let d = descriptor();
    let lut = block_on(pos_lut::read(&mut c, ID, &d)).expect("lut");
    assert!(lut.live());
    assert_eq!(lut.points, table().points);
    let v = block_on(stamp::verdict(&mut c, ID, &d)).expect("verdict");
    assert!(v.matches());
    // The bench servo's stored set of record carries this stamp.
    assert_eq!(v.stored, 0xd1c4);
    assert_eq!(steps.last(), Some(&Step::Stamped(0xd1c4)));
    let checked = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert!(
        checked.clean(),
        "{:?} {}",
        checked.differences,
        checked.data
    );
    assert_eq!(checked.stamp, "0xd1c4 = live set");
}

#[test]
fn save_writes_the_backup_before_anything_else() {
    let (mut c, log) = seeded();
    let d = descriptor();
    let rec = record();
    let names = every_protection(&rec);
    let before = block_on(state::read(&mut c, ID, &d)).expect("state");
    log.borrow_mut().clear();

    let mut sink = LogSink {
        files: Default::default(),
        log: log.clone(),
        fail: true,
    };
    let mut steps = Vec::new();
    let e = block_on(save(
        &mut c,
        ID,
        &d,
        &rec,
        Some(&table().points),
        &names,
        &mut sink,
        &mut steps,
    ))
    .expect_err("no backup, no save");
    assert!(e.0.contains("the disk is full"), "{e}");
    assert_eq!(writes_below(&log, CONTROL), [], "nothing written");
    assert!(
        !log.borrow()
            .contains(&Ev::Mgmt(osc_client::MgmtOp::Save as u8))
    );
    assert_eq!(
        block_on(state::read(&mut c, ID, &d)).expect("state").raw,
        before.raw
    );

    log.borrow_mut().clear();
    sink.fail = false;
    let mut steps = Vec::new();
    block_on(save(
        &mut c,
        ID,
        &d,
        &rec,
        Some(&table().points),
        &names,
        &mut sink,
        &mut steps,
    ))
    .expect("the save lands");
    let log = log.borrow().clone();
    let at = |e: &Ev| log.iter().position(|x| x == e).unwrap_or(usize::MAX);
    let backup = at(&Ev::File(STORED.into()));
    let first_write = log
        .iter()
        .position(|e| matches!(*e, Ev::Write { addr, .. } if addr < CONTROL))
        .expect("a write");
    assert!(
        backup < first_write,
        "the backup is kept before the first write"
    );
    assert!(backup < at(&Ev::Mgmt(osc_client::MgmtOp::Save as u8)));
    assert!(
        log[..backup]
            .iter()
            .all(|e| !matches!(e, Ev::Write { addr, .. } if *addr < CONTROL))
    );
    assert!(matches!(steps.first(), Some(Step::Rebooted)));
    assert!(matches!(steps.get(1), Some(Step::Backup(p)) if p == STORED));

    // The backup is the state the servo had stored, one the notebooks'
    // restore reads, and the state after the SAVE is the set.
    let stored =
        Record::parse(std::str::from_utf8(&sink.files[STORED]).expect("utf8")).expect("record");
    assert_eq!(stored.doc["fields"]["raw_max"], json!(4095));
    assert_eq!(stored.doc["raw"], before.record(&d)["raw"]);
    assert!(sink.files.contains_key(AFTER));
    block_on(osc_client::mgmt::reboot(&mut c, ID)).expect("reboot");
    let checked = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert!(
        checked.clean(),
        "the set boots: {:?} {}",
        checked.differences,
        checked.data
    );

    // And the backup puts the old state back.
    let mut steps = Vec::new();
    block_on(restore(
        &mut c,
        ID,
        &d,
        std::str::from_utf8(&sink.files[STORED]).expect("utf8"),
        json!({"from": "test"}),
        &names,
        &mut steps,
    ))
    .expect("restore");
    assert_eq!(read_u16(&mut c, "raw_max"), 4095);
    assert_eq!(
        block_on(state::read(&mut c, ID, &d)).expect("state").raw,
        before.raw
    );
}

#[test]
fn a_refusal_after_the_first_write_reboots_to_the_saved_state() {
    let (mut c, _) = on_record();
    let d = descriptor();
    // New stops, and a table with a point past the new low stop: the
    // stops land, then the servo refuses the table.
    let mut i = inputs();
    if let Some(s) = i.measured.stops.as_mut() {
        s.raw_min = 90;
    }
    let mut t = table();
    t.stops.0 = 90;
    t.points[5] = -2;
    let set = rules_set(&i, &t);
    let mut steps = Vec::new();
    let e = block_on(install::install(
        &mut c,
        ID,
        &d,
        &set,
        Some(&t.points),
        &[],
        &mut steps,
    ))
    .expect_err("the table is refused");
    assert!(e.0.contains("REJECT_ENDS"), "{e}");
    assert!(
        e.0.contains("rebooted: the servo is back on its saved state"),
        "{e}"
    );
    assert_eq!(wrote(&steps).first().map(String::as_str), Some("raw_min"));
    assert_eq!(steps.last(), Some(&Step::Rebooted));
    assert_eq!(read_u16(&mut c, "raw_min"), 82);
    let checked = block_on(check(&mut c, ID, &d, &record())).expect("check");
    assert!(
        checked.clean(),
        "{:?} {}",
        checked.differences,
        checked.data
    );
}

#[test]
fn a_field_that_protects_less_needs_its_name() {
    let (mut c, log) = on_record();
    let looser = |f: &dyn Fn(&mut osc_measure::Inputs)| {
        let mut i = inputs();
        f(&mut i);
        rules_set(&i, &table())
    };

    let higher_cap = looser(&|i| i.nameplate.derate_other = Some(0.9));
    let (r, steps) = install_set(&mut c, &higher_cap, &[]);
    let e = r.expect_err("a higher cap needs its name");
    assert!(e.0.contains("current_limit_counts: servo 1043, set"), "{e}");
    assert!(e.0.contains("--loosen current_limit_counts"), "{e}");
    assert_eq!(steps, []);
    assert_eq!(writes_below(&log, CONTROL), []);
    let (r, steps) = install_set(&mut c, &higher_cap, &["current_limit_counts"]);
    r.expect("named, it installs");
    assert_eq!(wrote(&steps), ["current_limit_counts"]);

    // Back down is tighter: no name needed.
    let (r, _) = install_set(&mut c, &record(), &[]);
    r.expect("a lower cap installs unnamed");
    assert_eq!(read_u16(&mut c, "current_limit_counts"), 1043);

    // A judged field: any change is taken as protecting less.
    let fault =
        looser(&|i| i.policy.stall_response = Some(osc_measure::policy::StallResponse::Fault));
    let (r, _) = install_set(&mut c, &fault, &[]);
    let e = r.expect_err("a judged field needs its name");
    assert!(
        e.0.contains("stall_response: servo \"Yield (1)\", set \"Fault (0)\", judged"),
        "{e}"
    );

    // A name that is no protection field of the set is refused, not ignored.
    let (r, _) = install_set(&mut c, &record(), &["r_q12"]);
    assert!(
        r.expect_err("not a protection field")
            .0
            .contains("r_q12 is not a protection field")
    );
}

#[test]
fn torque_on_refuses_every_write_verb() {
    let (mut c, log) = on_record();
    let d = descriptor();
    let backup = serde_json::to_string(
        &block_on(state::read(&mut c, ID, &d))
            .expect("state")
            .record(&d),
    )
    .expect("json");
    let (torque, _) = field_addr("torque_enable");
    block_on(c.write(ID, torque, &[1])).expect("torque on");
    assert_eq!(block_on(c.read(ID, torque, 1)).expect("read"), [1]);
    log.borrow_mut().clear();

    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(150.0);
    let (r, _) = install_set(&mut c, &rules_set(&i, &table()), &[]);
    let refusals = [
        r.expect_err("install"),
        block_on(save(
            &mut c,
            ID,
            &d,
            &record(),
            Some(&table().points),
            &[],
            &mut osc_measure::sink::MemSink::default(),
            &mut Vec::new(),
        ))
        .expect_err("save"),
        block_on(restore(
            &mut c,
            ID,
            &d,
            &backup,
            json!({}),
            &[],
            &mut Vec::new(),
        ))
        .expect_err("restore"),
    ];
    for e in refusals {
        assert!(e.0.contains("torque_enable is on"), "{e}");
    }
    assert_eq!(writes_below(&log, CONTROL), []);
    assert!(
        log.borrow().iter().all(|e| !matches!(e, Ev::Mgmt(_))),
        "not even a reboot"
    );
    assert_eq!(block_on(c.read(ID, torque, 1)).expect("read"), [1]);
}

#[test]
fn the_tool_has_no_factory_path() {
    // No source of the library calls the wipe, the client's or its op's
    // (the CLI's own test says the same of its sources); the spy pipe
    // under every test of this file fails on a FACTORY on the bus.
    let mut sources = Vec::new();
    for dir in ["src", "src/rules", "src/capture", "src/steps", "src/fit"] {
        for e in std::fs::read_dir(path(dir)).unwrap_or_else(|e| panic!("{dir}: {e}")) {
            let p = e.expect("entry").path();
            if p.extension().is_some_and(|x| x == "rs") {
                sources.push(p);
            }
        }
    }
    assert!(sources.len() > 25);
    for p in sources {
        let s = std::fs::read_to_string(&p)
            .expect("read")
            .to_ascii_lowercase();
        for call in ["factory(", "mgmtop::factory", "factory as u8"] {
            assert!(!s.contains(call), "{} names {call}", p.display());
        }
    }

    // A servo whose saved settings are corrupt, the one state only that
    // wipe clears, is refused by every write verb, nothing written.
    let (mut c, log) = on_record();
    let d = descriptor();
    let backup = serde_json::to_string(
        &block_on(state::read(&mut c, ID, &d))
            .expect("state")
            .record(&d),
    )
    .expect("json");
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.mode.data_flags |= CONFIG_CORRUPT);
    let s = block_on(state::read(&mut c, ID, &d)).expect("state");
    assert_eq!(s.data.flags & CONFIG_CORRUPT, CONFIG_CORRUPT);
    log.borrow_mut().clear();
    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(150.0);
    let (r, _) = install_set(&mut c, &rules_set(&i, &table()), &[]);
    let refusals = [
        r.expect_err("install"),
        block_on(save(
            &mut c,
            ID,
            &d,
            &record(),
            Some(&table().points),
            &[],
            &mut osc_measure::sink::MemSink::default(),
            &mut Vec::new(),
        ))
        .expect_err("save"),
        block_on(restore(
            &mut c,
            ID,
            &d,
            &backup,
            json!({}),
            &[],
            &mut Vec::new(),
        ))
        .expect_err("restore"),
    ];
    for e in refusals {
        assert!(e.0.contains("`osc recover`"), "{e}");
    }
    assert_eq!(writes_below(&log, CONTROL), []);
    assert!(log.borrow().iter().all(|e| !matches!(e, Ev::Mgmt(_))));
    assert_eq!(s.data.flags & STAMP_MISMATCH, 0);
}
