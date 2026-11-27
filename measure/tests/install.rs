//! The configuration path on the client's simulated servo: what lands, in
//! what order, what is refused before the first write, and what a refusal
//! after it leaves.

mod common;

use common::*;
use futures_lite::future::block_on;
use osc_client::pos_lut;
use osc_client::store_state::{CALIB_DEFAULTS, CONFIG_DEFAULTS, UNSAVED};
use osc_measure::install::{self, AFTER, STORED, Step, check, restore, save};
use osc_measure::pyjson;
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
) -> (Result<(), install::Refused>, Vec<Step>) {
    let mut steps = Vec::new();
    let r = block_on(install::install(
        c,
        ID,
        &descriptor(),
        set,
        Some(&table().points),
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
    let (r, steps) = install_set(&mut c, &record());
    r.expect("the set it holds installs");
    // the record's undervolt floor has no field on this firmware
    assert_eq!(steps, [Step::Retired("v_undervolt_counts".to_owned())]);
    assert_eq!(writes_below(&log, CONTROL), [], "nothing written");

    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(150.0);
    let set = rules_set(&i, &table());
    log.borrow_mut().clear();
    let (r, steps) = install_set(&mut c, &set);
    r.expect("one plant field installs");
    assert_eq!(wrote(&steps), ["velocity_limit_cps"]);
    assert_eq!(
        writes_below(&log, CONTROL),
        [field_addr("velocity_limit_cps")],
        "the field alone: nothing is stamped"
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

/// Soft limits off (no margin share) install over limits that are on, and
/// back: each way the pair is one write, since 0 / 0 is no side alone.
#[test]
fn the_soft_limits_go_off_and_on_in_one_write() {
    let (mut c, log) = on_record();
    let pair = (field_addr("pos_min_soft_counts").0, 8);
    let mut i = inputs();
    i.policy.soft_margin_share = None;
    let off = rules_set(&i, &table());
    assert_eq!(
        off.value("pos_min_soft_counts").and_then(|v| v.as_i64()),
        Some(0)
    );
    assert_eq!(
        off.value("pos_max_soft_counts").and_then(|v| v.as_i64()),
        Some(0)
    );
    let (r, steps) = install_set(&mut c, &off);
    r.expect("the soft limits go off");
    assert_eq!(
        wrote(&steps),
        ["pos_min_soft_counts", "pos_max_soft_counts"]
    );
    assert_eq!(writes_below(&log, CONTROL)[0], pair);
    let soft = block_on(c.read(ID, pair.0, 8)).expect("read");
    assert_eq!(soft, [0; 8]);
    let checked = block_on(check(&mut c, ID, &descriptor(), &off)).expect("check");
    assert!(checked.clean(), "{:?}", checked.differences);

    log.borrow_mut().clear();
    let (r, steps) = install_set(&mut c, &record());
    r.expect("the soft limits come back on");
    assert_eq!(
        wrote(&steps),
        ["pos_min_soft_counts", "pos_max_soft_counts"]
    );
    assert_eq!(writes_below(&log, CONTROL)[0], pair);
    let soft = block_on(c.read(ID, pair.0, 8)).expect("read");
    assert_eq!(soft[..4], 314i32.to_le_bytes());
    assert_eq!(soft[4..], 3560i32.to_le_bytes());
}

#[test]
fn every_write_is_read_back() {
    let (mut c, log) = seeded();
    let rec = record();
    let (r, steps) = install_set(&mut c, &rec);
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

/// An install lands volatile: the table LIVE, the check clean, and the
/// servo unsaved. The check names the servo by its fingerprint, the hash
/// of its state record, which moves with any byte of the set.
#[test]
fn the_table_lands_live_and_the_install_reads_unsaved() {
    let (mut c, _) = seeded();
    let rec = record();
    let d = descriptor();
    let before = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert!(!before.unsaved, "the seed is saved");
    let (r, steps) = install_set(&mut c, &rec);
    r.expect("installs");
    assert!(wrote(&steps).iter().any(|w| w == "table"));
    let lut = block_on(pos_lut::read(&mut c, ID, &d)).expect("lut");
    assert!(lut.live());
    assert_eq!(lut.points, table().points);
    let checked = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert!(
        checked.clean(),
        "{:?} {}",
        checked.differences,
        checked.settings
    );
    assert!(checked.unsaved, "installed, not saved");
    let state = block_on(state::read(&mut c, ID, &d)).expect("state");
    assert_eq!(checked.servo, state.hash(&d));
    assert_eq!(checked.servo, state.record(&d)["hash"]);
    assert_eq!(checked.servo.len(), 64);
    assert_ne!(checked.servo, before.servo, "the set moved the bytes");
    let again = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert_eq!(again.servo, checked.servo, "a read moves nothing");
}

#[test]
fn save_writes_the_backup_before_anything_else() {
    let (mut c, log) = seeded();
    let d = descriptor();
    let rec = record();
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
        checked.settings
    );

    // And the backup puts the old state back.
    let mut steps = Vec::new();
    block_on(restore(
        &mut c,
        ID,
        &d,
        std::str::from_utf8(&sink.files[STORED]).expect("utf8"),
        json!({"from": "test"}),
        &mut steps,
    ))
    .expect("restore");
    assert_eq!(read_u16(&mut c, "raw_max"), 4095);
    assert_eq!(
        block_on(state::read(&mut c, ID, &d)).expect("state").raw,
        before.raw
    );
}

/// A servo never saved boots both images on their defaults, which only the
/// SAVE clears: the check before it is the set landed, and after it the
/// servo boots the set, both images loaded, nothing unsaved.
#[test]
fn save_on_a_factory_fresh_servo_saves_and_boots_clean() {
    let (mut c, log) = factory();
    let d = descriptor();
    let rec = record();
    let fresh = block_on(state::read(&mut c, ID, &d)).expect("state");
    assert_eq!(
        fresh.store.status & (CONFIG_DEFAULTS | CALIB_DEFAULTS),
        CONFIG_DEFAULTS | CALIB_DEFAULTS,
        "{}",
        fresh.settings_said()
    );
    let mut sink = LogSink {
        files: Default::default(),
        log: log.clone(),
        fail: false,
    };
    let mut steps = Vec::new();
    block_on(save(
        &mut c,
        ID,
        &d,
        &rec,
        Some(&table().points),
        &mut sink,
        &mut steps,
    ))
    .unwrap_or_else(|e| panic!("the save lands: {e}"));
    let saves = log
        .borrow()
        .iter()
        .filter(|e| **e == Ev::Mgmt(osc_client::MgmtOp::Save as u8))
        .count();
    assert_eq!(saves, 1);
    assert!(steps.contains(&Step::Saved), "{steps:?}");
    let stored: serde_json::Value = serde_json::from_slice(&sink.files[STORED]).expect("json");
    assert_eq!(stored["settings"], "defaults: CONFIG, CALIB");

    block_on(osc_client::mgmt::reboot(&mut c, ID)).expect("reboot");
    let checked = block_on(check(&mut c, ID, &d, &rec)).expect("check");
    assert!(
        checked.clean(),
        "the set boots: {:?} {}",
        checked.differences,
        checked.settings
    );
    assert_eq!(checked.settings, "loaded");
    assert!(!checked.unsaved);
    let after: serde_json::Value = serde_json::from_slice(&sink.files[AFTER]).expect("json");
    assert_eq!(checked.servo, after["hash"], "the state the SAVE left");
}

/// The check before the SAVE refuses a set that did not land, never a
/// stored state or a refused mode: both are information.
#[test]
fn the_check_before_a_save_refuses_only_a_set_that_did_not_land() {
    let checked = |differences: Vec<String>, settings: &str, refused: &str| install::Checked {
        differences,
        unowned: vec![],
        settings: settings.into(),
        refused: refused.into(),
        unsaved: true,
        servo: "0".repeat(64),
    };
    for (settings, refused) in [
        ("loaded", "none"),
        ("defaults: CONFIG, CALIB", "Current, Velocity, Position"),
        ("defaults: CONFIG", "none"),
    ] {
        let c = checked(vec![], settings, refused);
        assert!(c.clean(), "{settings}");
        assert_eq!(c.unsaveable(), None, "{settings}");
    }
    let c = checked(vec!["v_kp_q88: servo 70, set 97".into()], "loaded", "none");
    assert!(!c.clean());
    let why = c.unsaveable().expect("refused");
    assert!(why.ends_with(": v_kp_q88: servo 70, set 97"), "{why}");
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
        &mut steps,
    ))
    .expect_err("the table is refused");
    assert!(e.0.contains("REJECT_ENDS"), "{e}");
    assert!(
        e.0.contains("rebooted: the servo is back on its saved state"),
        "{e}"
    );
    assert_eq!(wrote(&steps).first().map(String::as_str), Some("raw_min"));
    assert!(
        !steps.contains(&Step::Rebooted),
        "the refusal says the reboot, once: {steps:?}"
    );
    assert_eq!(read_u16(&mut c, "raw_min"), 82);
    let checked = block_on(check(&mut c, ID, &d, &record())).expect("check");
    assert!(
        checked.clean(),
        "{:?} {}",
        checked.differences,
        checked.settings
    );
}

/// The tool makes no judgment of a set: one that moves a protection
/// field, either way, installs as any other, and its steps say each move.
#[test]
fn a_set_that_moves_a_protection_installs_and_says_the_move() {
    let (mut c, _) = on_record();
    let with = |f: &dyn Fn(&mut osc_measure::Inputs)| {
        let mut i = inputs();
        f(&mut i);
        rules_set(&i, &table())
    };
    let moved = |steps: &[Step], name: &str| {
        steps.iter().find_map(|s| match s {
            Step::Field { name: n, from, to } if n == name => Some((from.clone(), to.clone())),
            _ => None,
        })
    };

    let higher_cap = with(&|i| i.nameplate.derate_other = Some(0.9));
    let cap = higher_cap.value("current_limit_counts").cloned();
    let (r, steps) = install_set(&mut c, &higher_cap);
    r.expect("a higher cap installs");
    assert_eq!(wrote(&steps), ["current_limit_counts"]);
    assert_eq!(
        moved(&steps, "current_limit_counts"),
        Some((json!(1043), cap.expect("a cap")))
    );
    let said = steps[0].to_string();
    assert!(said.starts_with("current_limit_counts: 1043 -> "), "{said}");

    let slower = with(&|i| i.policy.stall_omega_max_deg_s = Some(10.0));
    let (r, steps) = install_set(&mut c, &slower);
    r.expect("a lower stall speed installs");
    let (from, to) = moved(&steps, "stall_omega_max_cps").expect("the move is a step");
    assert_eq!(from, json!(500));
    assert!(to.as_i64().expect("an int") < 500, "{to}");

    let fault =
        with(&|i| i.policy.stall_response = Some(osc_measure::policy::StallResponse::Fault));
    let (r, steps) = install_set(&mut c, &fault);
    r.expect("another stall response installs");
    assert_eq!(
        moved(&steps, "stall_response"),
        Some((json!("Yield (1)"), json!("Fault (0)")))
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
    let (r, _) = install_set(&mut c, &rules_set(&i, &table()));
    let refusals = [
        r.expect_err("install"),
        block_on(save(
            &mut c,
            ID,
            &d,
            &record(),
            Some(&table().points),
            &mut osc_measure::sink::MemSink::default(),
            &mut Vec::new(),
        ))
        .expect_err("save"),
        block_on(restore(&mut c, ID, &d, &backup, json!({}), &mut Vec::new()))
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

    // A servo whose saved settings are unreadable runs on its defaults and
    // is written like any: no verb refuses on the stored state, none wipes.
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
        .servo_table_mut(0, |t| t.telemetry.common.status_flags |= CONFIG_DEFAULTS);
    let s = block_on(state::read(&mut c, ID, &d)).expect("state");
    assert_eq!(s.store.status & CONFIG_DEFAULTS, CONFIG_DEFAULTS);
    log.borrow_mut().clear();
    let mut i = inputs();
    i.policy.velocity_limit_deg_s = Some(150.0);
    let (r, _) = install_set(&mut c, &rules_set(&i, &table()));
    r.expect("install");
    block_on(restore(&mut c, ID, &d, &backup, json!({}), &mut Vec::new())).expect("restore");
    assert!(log.borrow().iter().all(|e| !matches!(e, Ev::Mgmt(_))));
    let s = block_on(state::read(&mut c, ID, &d)).expect("state");
    assert_eq!(
        s.store.status & CONFIG_DEFAULTS,
        CONFIG_DEFAULTS,
        "information"
    );
    assert_eq!(
        s.store.status & UNSAVED,
        UNSAVED,
        "the restore wrote, unsaved"
    );
}

/// A record kept while servos carried a plant stamp (CALIB layout 7: a
/// `stamp` line and `plant_stamp` among its fields, both under its hash)
/// still restores: it reads under the hash it was written with, the
/// register no servo has any more is never written, and a register read
/// under its old name lands under today's.
#[test]
fn a_record_from_a_stamped_servo_still_restores() {
    let (mut c, log) = on_record();
    let d = descriptor();
    let mut doc = block_on(state::read(&mut c, ID, &d))
        .expect("state")
        .record(&d);
    let m = doc.as_object_mut().expect("object");
    m.remove("unsaved");
    m.remove("settings");
    m.remove("refused_modes");
    m.remove("hash");
    m.insert("stamp".into(), json!("0xd1c4 = live set"));
    m.insert("data_state".into(), json!("clean"));
    m["fields"]
        .as_object_mut()
        .expect("fields")
        .insert("plant_stamp".into(), json!(53700));
    m["writable"]
        .as_array_mut()
        .expect("writable")
        .push(json!("plant_stamp"));
    let fields = m["fields"].as_object_mut().expect("fields");
    let held = fields.remove("yield_current_counts").expect("held");
    fields.insert("i_cont_counts".into(), held.clone());
    for name in m["writable"].as_array_mut().expect("writable") {
        if name == "yield_current_counts" {
            *name = json!("i_cont_counts");
        }
    }
    let old_hash = pyjson::hash(&json!({
        "addr": doc["addr"],
        "raw": doc["raw"],
        "pos_table": {"state": doc["pos_table"]["state"], "points_sha1": doc["pos_table"]["points_sha1"]},
        "wide_codes": doc["wide_codes"],
        "stamp": doc["stamp"],
        "data_state": doc["data_state"],
    }));
    let mut bad = doc.clone();
    bad["hash"] = json!(state::state_hash(&doc));
    assert!(Record::parse(&bad.to_string()).is_err(), "not its hash");
    doc["hash"] = json!(old_hash);
    let r = Record::parse(&doc.to_string()).expect("the old record reads");
    let adopted = r.adopted(json!({})).expect("adopted");
    assert!(adopted.value("plant_stamp").is_none(), "never written");
    assert!(adopted.value("i_cont_counts").is_none());
    assert_eq!(
        adopted.value("yield_current_counts"),
        Some(&held),
        "today's name"
    );
    log.borrow_mut().clear();
    block_on(restore(
        &mut c,
        ID,
        &d,
        &doc.to_string(),
        json!({}),
        &mut Vec::new(),
    ))
    .expect("restores");
}
