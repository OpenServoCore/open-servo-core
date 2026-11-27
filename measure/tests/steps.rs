//! The rest step on the client's simulated servo at 3 Mbps: what it
//! accepts and refuses, the line it refuses before any write, the burst it
//! will not take as evidence, and the one way out every exit runs. Then
//! the probe step's ways out on the same servo: the simulated servo runs
//! no shunt burst, so the probe reaches its refusals before any write, a
//! burst that never completes, an arm the servo refuses, a bus error and
//! the interrupt flag, and no further.

mod common;

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use common::{ID, descriptor, record, table};
use futures_lite::future::block_on;
use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
use osc_client::pipe::{Pipe, PipeError};
use osc_client::{BaudRate, Client, Inst, Opcode, mgmt};
use osc_measure::capture::burst;
use osc_measure::capture::meta::gunzip;
use osc_measure::guard::Way;
use osc_measure::install;
use osc_measure::nameplate::Nameplate;
use osc_measure::sink::MemSink;
use osc_measure::steps::rest::{self, ROWS, Rested};
use osc_measure::steps::{Rig, Verdict};

const OSC_EXCHANGE: u8 = 0x20;
const TORQUE: u16 = 0x180;
const CHUNK0: u16 = 0x1e2;

/// A hook on every TEL arm, its number from 1: it may reach into the rig,
/// and fail the arm's own send.
type Hook = Box<dyn FnMut(usize, &mut FakePipe) -> bool>;

/// Every write on the bus, its address and bytes.
type Writes = Rc<RefCell<Vec<(u16, Vec<u8>)>>>;

/// A hook on every write, its address and bytes: it may reach into the
/// rig, and fail the write's own send.
type OnWrite = Box<dyn FnMut(u16, &[u8], &mut FakePipe) -> bool>;

/// The fake adapter, every write logged with its bytes.
struct Bus {
    inner: FakePipe,
    writes: Writes,
    arms: usize,
    hook: Hook,
    on_write: OnWrite,
}

impl Pipe for Bus {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        if let [
            _,
            _,
            OSC_EXCHANGE,
            _,
            _,
            _,
            _,
            inst,
            _,
            _,
            _,
            _,
            payload @ ..,
        ] = bytes
            && Inst(*inst).opcode() == Some(Opcode::Write)
            && payload.len() >= 2
        {
            let addr = u16::from_le_bytes([payload[0], payload[1]]);
            self.writes.borrow_mut().push((addr, payload[2..].to_vec()));
            if (self.on_write)(addr, &payload[2..], &mut self.inner) {
                return Err(PipeError::Io("the cable came out".into()));
            }
            if addr == CHUNK0 && payload.len() >= 12 {
                self.arms += 1;
                if (self.hook)(self.arms, &mut self.inner) {
                    return Err(PipeError::Io("the cable came out".into()));
                }
            }
        }
        self.inner.send(bytes).await
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        self.inner.recv().await
    }

    async fn pause(&mut self, d: Duration) {
        self.inner.pause(d).await
    }
}

/// A deterministic count of noise, uniform over `-w..=w`.
fn noise(i: u32, k: u32, w: i32) -> i32 {
    let mut h = i.wrapping_mul(0x9E37_79B9) ^ k.wrapping_mul(0x85EB_CA6B);
    h ^= h >> 15;
    h = h.wrapping_mul(0x2C1B_3C6D);
    h ^= h >> 12;
    (h % (2 * w as u32 + 1)) as i32 - w
}

/// A servo at rest on the bench: the pot at 2041, the zero at 1206, the
/// taps at the bias, a 5 V rail, the board at 26 C, each a count or so of
/// noise. `pot` and `zero_w` override the pot and the zero's noise.
fn track(pot: Option<u16>, zero_w: i32) -> Vec<TelSample> {
    (0..4096u32)
        .map(|i| {
            let n = |k: u32| noise(i, k, 1);
            let at = |base: i32, k: u32| (base + n(k)) as u16;
            TelSample {
                tick: TelemetryTick {
                    _scan: [0; 9],
                    pos: pot.unwrap_or_else(|| at(2041, 1)),
                    vbus_raw: at(1241, 3),
                    ntc_raw: at(2010, 4),
                    current_raw: (1206 + noise(i, 5, zero_w)) as u16,
                    vmotor_a: at(773, 6),
                    vmotor_b: at(772, 7),
                    duty_ticks: 0,
                    current_trough: at(1206, 8),
                    i_tick_counts: 0,
                    vdiff_counts: 0,
                    vmotor_a_trough: at(773, 9),
                    vmotor_b_trough: at(772, 10),
                    tick_flags: 0,
                    sample_tick: i,
                },
                vbus_counts: 2000,
            }
        })
        .collect()
}

/// The servo of record at `rate`: the set of record installed and saved,
/// resting on `track`, its boot reading of the reference 1490.
fn servo(rate: BaudRate, track: Vec<TelSample>, hook: Hook) -> (Client<Bus>, Writes) {
    servo_with_vref(rate, track, hook, 1490)
}

/// [`servo`] with the boot reading `vref_raw`; 0 is none.
fn servo_with_vref(
    rate: BaudRate,
    track: Vec<TelSample>,
    hook: Hook,
    vref_raw: u16,
) -> (Client<Bus>, Writes) {
    let mut inner = FakePipe::new(rate, &[1]);
    inner.seed_calibrated(0);
    let writes = Writes::default();
    let mut c = block_on(Client::connect(Bus {
        inner,
        writes: writes.clone(),
        arms: 0,
        hook: Box::new(|_, _| false),
        on_write: Box::new(|_, _, _| false),
    }))
    .expect("connect");
    let rec = record();
    block_on(install::install(
        &mut c,
        ID,
        &descriptor(),
        &rec,
        Some(&table().points),
        &mut Vec::new(),
    ))
    .expect("the set of record installs");
    block_on(mgmt::save(&mut c, ID)).expect("save");
    c.pipe_mut().inner.set_track(0, track);
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.refs.vref_raw = vref_raw);
    c.pipe_mut().hook = hook;
    writes.borrow_mut().clear();
    (c, writes)
}

fn nameplate() -> Nameplate {
    common::load("testdata/mg90-d/nameplate.json")
}

fn run(
    c: &mut Client<Bus>,
    stop: &AtomicBool,
    sink: &mut MemSink,
) -> Result<Rested, install::Refused> {
    let d = descriptor();
    let clock = || 1.7e9;
    let np = nameplate();
    let mut rig = Rig {
        c,
        id: ID,
        d: &d,
        sink,
        stop,
        clock: &clock,
    };
    block_on(rest::rest(&mut rig, Some(&np)))
}

fn torque(c: &mut Client<Bus>) -> u8 {
    block_on(c.read(ID, TORQUE, 1)).expect("read")[0]
}

fn torque_went_on(w: &Writes) -> bool {
    w.borrow()
        .iter()
        .any(|(a, b)| *a == TORQUE && b.first() == Some(&1))
}

fn failed(r: &Rested) -> Vec<String> {
    r.checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.clone())
        .collect()
}

#[test]
fn rest_on_the_simulated_servo_accepts_and_moves_nothing() {
    let (mut c, writes) = servo(BaudRate::B3000000, track(None, 1), Box::new(|_, _| false));
    let stop = AtomicBool::new(false);
    let mut sink = MemSink::default();
    let r = run(&mut c, &stop, &mut sink).expect("starts");
    for ch in &r.checks {
        println!("{}", ch.said());
    }
    assert_eq!(r.ended.way, Way::Done, "{:?} {:?}", r.ended.why, failed(&r));
    assert!(r.ended.exit.torque_off && !r.ended.exit.rebooted);
    assert_eq!(torque(&mut c), 0);
    assert!(torque_went_on(&writes), "the second second runs torque on");
    let s = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
    assert!(!s.unsaved, "CONTROL writes leave the saved servo saved");

    // nothing a set holds was written: only CONTROL, and the mode put back
    let w = writes.borrow();
    assert!(w.iter().all(|(a, _)| *a >= TORQUE), "{w:?}");
    let dm = w.iter().position(|(a, _)| *a == 0x192).expect("dead-man");
    let on = w
        .iter()
        .position(|(a, b)| *a == TORQUE && b == &[1])
        .expect("torque on");
    assert!(dm < on, "the dead-man goes ahead of torque on");
    drop(w);

    let moved = r
        .checks
        .iter()
        .find(|c| c.name == "nothing moved")
        .expect("checked");
    assert_eq!(moved.verdict, Verdict::Pass);
    let ntc = r
        .checks
        .iter()
        .find(|c| c.name == "NTC words")
        .expect("checked");
    assert_eq!(ntc.verdict, Verdict::Pass, "{}", ntc.value);
    assert!(
        ntc.value.contains("designed 2048 2430 474 19934"),
        "{}",
        ntc.value
    );

    let result: serde_json::Value =
        serde_json::from_slice(&sink.files[rest::RESULT]).expect("rest.json");
    assert_eq!(result["verdict"], "accepted");
    assert_eq!(result["image"], "production");
    let zero = result["estimates"]["current_zero_awake_counts"]["value"]
        .as_f64()
        .expect("zero");
    assert!((zero - 1206.0).abs() < 0.5, "{zero}");
    let rail = result["estimates"]["rail_v"]["value"]
        .as_f64()
        .expect("rail");
    assert!((rail - 5.0).abs() < 0.02, "{rail}");
    assert_eq!(
        result["estimates"]["vdd_v"]["value"].as_f64(),
        Some(1.2 * 4096.0 / 1490.0),
        "the chip supply from the one READ of vref_raw"
    );
    let vdd = r
        .checks
        .iter()
        .find(|c| c.name == "chip supply")
        .expect("checked");
    assert_eq!(vdd.verdict, Verdict::NotJudged, "{}", vdd.value);

    for name in ["captures/rest/01-rest", "captures/rest/02-raw"] {
        let csv = gunzip(&sink.files[&format!("{name}.csv.gz")]).expect("gzip");
        assert_eq!(
            csv.iter().filter(|b| **b == b'\n').count(),
            ROWS as usize + 1,
            "{name}"
        );
        let meta: serde_json::Value =
            serde_json::from_slice(&sink.files[&format!("{name}.meta.json")]).expect("meta");
        assert_eq!(
            (meta["rows"].as_u64(), meta["lost"].as_array().map(Vec::len)),
            (Some(ROWS as u64), Some(0))
        );
        assert!(meta["sense"]["gain_milli"].is_u64());
    }
}

/// A servo whose `vref_raw` reads 0 took no boot reading: the step says
/// so, reads the rail through the nominal supply, and still accepts.
#[test]
fn rest_without_a_boot_reading_uses_the_nominal_supply() {
    let (mut c, _) = servo_with_vref(
        BaudRate::B3000000,
        track(None, 1),
        Box::new(|_, _| false),
        0,
    );
    let mut sink = MemSink::default();
    let r = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(r.ended.way, Way::Done, "{:?} {:?}", r.ended.why, failed(&r));
    let vdd = r
        .checks
        .iter()
        .find(|c| c.name == "chip supply")
        .expect("checked");
    assert_eq!(vdd.verdict, Verdict::NotJudged, "{}", vdd.value);
    assert!(
        vdd.value.contains("vref_raw reads 0") && vdd.value.contains("nominal vdd_mv"),
        "{}",
        vdd.value
    );
    assert!(
        r.ended
            .lines
            .iter()
            .any(|l| l.contains("vref_raw reads 0") && l.contains("no chip supply figure")),
        "{:?}",
        r.ended.lines
    );
    let result: serde_json::Value =
        serde_json::from_slice(&sink.files[rest::RESULT]).expect("rest.json");
    assert!(result["estimates"]["vdd_v"].is_null());
    let rail = result["estimates"]["rail_v"]["value"]
        .as_f64()
        .expect("rail");
    assert!((rail - 5.0).abs() < 0.02, "{rail}");
}

#[test]
fn rest_refuses_a_pot_lead_off() {
    let (mut c, _) = servo(
        BaudRate::B3000000,
        track(Some(4095), 1),
        Box::new(|_, _| false),
    );
    let r = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect("starts");
    assert_eq!(r.ended.way, Way::Refused);
    assert!(
        failed(&r)
            .iter()
            .any(|n| n == "pot off the converter rails"),
        "{:?}",
        failed(&r)
    );
    assert_eq!(torque(&mut c), 0);
}

/// A centred horn resting on one code (a wide code reads the same count
/// every row, sd about 0) and a noisy current zero: the step is accepted
/// and reports both.
#[test]
fn rest_on_one_pot_code_is_accepted_and_reports_its_noise() {
    let (mut c, _) = servo(
        BaudRate::B3000000,
        track(Some(2047), 5),
        Box::new(|_, _| false),
    );
    let mut sink = MemSink::default();
    let r = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(r.ended.way, Way::Done, "{:?} {:?}", r.ended.why, failed(&r));
    let said = |name: &str| {
        r.checks
            .iter()
            .find(|c| c.name == name)
            .unwrap_or_else(|| panic!("no {name}"))
            .clone()
    };
    let pot = said("pot noise at rest");
    assert_eq!(pot.verdict, Verdict::NotJudged, "{}", pot.said());
    let zero = said("current zero noise, torque off");
    assert_eq!(zero.verdict, Verdict::NotJudged, "{}", zero.said());
    let doc: serde_json::Value =
        serde_json::from_slice(&sink.files[rest::RESULT]).expect("rest.json");
    assert_eq!(doc["verdict"], "accepted");
    let sd = doc["estimates"]["pot_rest_sd_counts"]["value"]
        .as_f64()
        .expect("sd");
    assert!(sd < 0.05, "{sd}");
    let zero_sd = doc["estimates"]["current_zero_sd_counts"]["value"]
        .as_f64()
        .expect("zero sd");
    assert!(zero_sd > 2.0, "the noisy zero is reported: {zero_sd}");
    assert_eq!(torque(&mut c), 0);
}

#[test]
fn under_3_mbps_is_refused_before_any_write() {
    let (mut c, writes) = servo(BaudRate::B2000000, track(None, 1), Box::new(|_, _| false));
    let mut sink = MemSink::default();
    let e = run(&mut c, &AtomicBool::new(false), &mut sink).expect_err("refused");
    assert!(e.0.contains("3 Mbaud") && e.0.contains("2000 kbaud"), "{e}");
    assert_eq!(*writes.borrow(), [], "nothing written");
    assert!(sink.files.is_empty());
}

/// A burst that lost a row is kept, said so, and taken once more; a second
/// such burst aborts the step, torque off.
#[test]
fn a_burst_with_a_dropped_row_is_refused() {
    // the first arm only loses a row: the repeat is whole and the step goes on
    let hook: Hook = Box::new(|n, p| {
        if n == 1 {
            p.sim_mut().lose_tel_ticks(0, 500, 501);
        } else {
            p.sim_mut().lose_tel_ticks(0, 0, 0);
        }
        false
    });
    let (mut c, _) = servo(BaudRate::B3000000, track(None, 1), hook);
    let mut sink = MemSink::default();
    let r = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(r.ended.way, Way::Done, "{:?}", r.ended.why);
    let first: serde_json::Value =
        serde_json::from_slice(&sink.files["captures/rest/01-rest.meta.json"]).expect("meta");
    assert!(
        !first["lost"].as_array().expect("lost").is_empty(),
        "{first}"
    );
    assert!(sink.files.contains_key("captures/rest/02-rest.csv.gz"));
    assert!(sink.files.contains_key("captures/rest/03-raw.csv.gz"));

    // every arm loses one: the second rest burst aborts the step
    let hook: Hook = Box::new(|_, p| {
        p.sim_mut().lose_tel_ticks(0, 500, 501);
        false
    });
    let (mut c, writes) = servo(BaudRate::B3000000, track(None, 1), hook);
    let mut sink = MemSink::default();
    let r = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(r.ended.way, Way::Error);
    let why = r.ended.why.clone().unwrap_or_default();
    assert!(why.contains("two rest bursts in a row lost rows"), "{why}");
    assert_eq!(c.pipe_mut().arms, 2, "repeated once, no more");
    assert!(!torque_went_on(&writes));
    assert!(r.ended.exit.torque_off);
    assert!(
        !sink.files.contains_key(rest::RESULT),
        "no result from no evidence"
    );
}

/// Done, a refusal, a bus error mid-step and the interrupt flag each end
/// in the one way out: torque off and read back, the dead-man and the
/// telemetry count 0, the mode put back.
#[test]
fn every_exit_path_ends_torque_off() {
    let ends = |hook: Hook, stop: &AtomicBool, track: Vec<TelSample>| {
        let (mut c, writes) = servo(BaudRate::B3000000, track, hook);
        let r = run(&mut c, stop, &mut MemSink::default()).expect("starts");
        let regs = block_on(c.read(ID, 0x184, 1)).expect("mode");
        let dm = block_on(c.read(ID, 0x192, 2)).expect("dead-man");
        let tel = block_on(c.read(ID, 0x1ea, 2)).expect("tel_count");
        assert_eq!(
            (torque(&mut c), regs, dm, tel),
            (0, vec![0], vec![0, 0], vec![0, 0]),
            "{:?}",
            r.ended
        );
        assert!(r.ended.exit.torque_off, "{}", r.ended.exit);
        (r, torque_went_on(&writes))
    };
    let calm = || Box::new(|_: usize, _: &mut FakePipe| false) as Hook;

    let (r, on) = ends(calm(), &AtomicBool::new(false), track(None, 1));
    assert_eq!((r.ended.way, on), (Way::Done, true));

    let (r, on) = ends(calm(), &AtomicBool::new(false), track(Some(4095), 1));
    assert_eq!((r.ended.way, on), (Way::Refused, true));

    // the raw burst's arm never reaches the servo: torque is on, the bus
    // comes back for the way out
    let cut: Hook = Box::new(|n, _| n == 2);
    let (r, on) = ends(cut, &AtomicBool::new(false), track(None, 1));
    assert_eq!((r.ended.way, on), (Way::Error, true), "{:?}", r.ended.why);
    assert!(r.ended.why.unwrap_or_default().contains("bus"));

    // the flag set while the raw burst streams: the burst runs out, then
    // the step stops
    let stop = Rc::new(AtomicBool::new(false));
    let set = stop.clone();
    let flag: Hook = Box::new(move |n, _| {
        if n == 2 {
            set.store(true, Ordering::SeqCst);
        }
        false
    });
    let (r, on) = ends(flag, &stop, track(None, 1));
    assert_eq!((r.ended.way, on), (Way::Interrupted, true));
    assert!(
        r.ended
            .lines
            .iter()
            .any(|l| l.starts_with("raw row: 20000 rows, whole")),
        "{:?}",
        r.ended.lines
    );
}

mod probe {
    use super::*;
    use osc_measure::Board;
    use osc_measure::inputs::ThermalInputs;
    use osc_measure::nameplate::GearClass;
    use osc_measure::state::State;
    use osc_measure::steps::probe::{self, NOT_METAL, ProbeInputs, Probed};

    const DUTY_Q15: u16 = 0x196;
    const ARM: u16 = 0x198;
    const SOFT: u16 = 0x028;
    const CURRENT_LIMIT: u16 = 0x04c;
    const DRIVE_POLARITY: u16 = 0x04f;

    fn board() -> Board {
        common::board()
    }

    fn thermal() -> ThermalInputs {
        common::thermal()
    }

    /// The servo of record with the rest step run and accepted on it, and
    /// its result; the writes log cleared.
    fn rested(rate: BaudRate) -> (Client<Bus>, Writes, serde_json::Value) {
        let (mut c, writes) = servo(BaudRate::B3000000, track(None, 1), Box::new(|_, _| false));
        let mut sink = MemSink::default();
        let r = run(&mut c, &AtomicBool::new(false), &mut sink).expect("rest starts");
        assert_eq!(r.ended.way, Way::Done, "{:?}", r.ended.why);
        let rest: serde_json::Value =
            serde_json::from_slice(&sink.files[rest::RESULT]).expect("rest.json");
        if rate != BaudRate::B3000000 {
            block_on(mgmt::set_baud(&mut c, &[ID], rate)).expect("baud");
        }
        writes.borrow_mut().clear();
        (c, writes, rest)
    }

    fn probe_on(
        c: &mut Client<Bus>,
        np: &Nameplate,
        thermal: Option<&ThermalInputs>,
        rest: &serde_json::Value,
        stop: &AtomicBool,
        sink: &mut MemSink,
    ) -> Result<Probed, install::Refused> {
        probe_with(c, np, thermal, rest, stop, sink, &board())
    }

    fn probe_with(
        c: &mut Client<Bus>,
        np: &Nameplate,
        thermal: Option<&ThermalInputs>,
        rest: &serde_json::Value,
        stop: &AtomicBool,
        sink: &mut MemSink,
        b: &Board,
    ) -> Result<Probed, install::Refused> {
        let d = descriptor();
        let clock = || 1.7e9;
        let mut rig = Rig {
            c,
            id: ID,
            d: &d,
            sink,
            stop,
            clock: &clock,
        };
        block_on(probe::probe(
            &mut rig,
            &ProbeInputs {
                nameplate: np,
                board: b,
                thermal,
                rest,
            },
        ))
    }

    fn table_bytes(c: &mut Client<Bus>) -> Vec<Vec<u8>> {
        [
            (SOFT, 8),
            (CURRENT_LIMIT, 2),
            (DRIVE_POLARITY, 1),
            (0x184, 1),
        ]
        .iter()
        .map(|&(a, n)| block_on(c.read(ID, a, n)).expect("read"))
        .collect()
    }

    fn armed(w: &Writes) -> bool {
        w.borrow().iter().any(|(a, _)| *a == ARM || *a == DUTY_Q15)
    }

    #[test]
    fn a_non_metal_servo_is_refused_before_any_write() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        for gear in [Some(GearClass::Plastic), Some(GearClass::Unknown)] {
            let mut np = nameplate();
            np.gear_class = gear;
            let mut sink = MemSink::default();
            let e = probe_on(
                &mut c,
                &np,
                Some(&thermal()),
                &rest,
                &AtomicBool::new(false),
                &mut sink,
            )
            .expect_err("refused");
            assert!(e.0.contains(NOT_METAL), "{e}");
            assert_eq!(*writes.borrow(), [], "nothing written");
            assert!(sink.files.is_empty());
        }
    }

    /// A nameplate short of a value, no heat model, a rest step not
    /// accepted, a line under 3 Mbps: each refused naming it, nothing
    /// written.
    #[test]
    fn the_probe_refuses_before_any_write() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        let mut np = nameplate();
        np.travel_deg = None;
        let mut not_rested = rest.clone();
        not_rested["verdict"] = "refused".into();
        let full = nameplate();
        let t = thermal();
        let cases: [(&Nameplate, Option<&ThermalInputs>, &serde_json::Value, &str); 3] = [
            (&np, Some(&t), &rest, "travel_deg"),
            (&full, None, &rest, "no heat model"),
            (&full, Some(&t), &not_rested, "an accepted rest step"),
        ];
        for (np, th, r, want) in cases {
            let mut sink = MemSink::default();
            let e = probe_on(&mut c, np, th, r, &AtomicBool::new(false), &mut sink)
                .expect_err("refused");
            assert!(e.0.contains(want), "{want}: {e}");
            assert_eq!(*writes.borrow(), [], "{want}: nothing written");
        }
        let (mut c, writes, rest) = rested(BaudRate::B2000000);
        let e = probe_on(
            &mut c,
            &full,
            Some(&t),
            &rest,
            &AtomicBool::new(false),
            &mut MemSink::default(),
        )
        .expect_err("refused");
        assert!(e.0.contains("3 Mbaud"), "{e}");
        assert_eq!(*writes.borrow(), []);
    }

    /// A board that publishes no stage 1 characterization: the probe, the
    /// stops step's seated read and the window term, which read a burst
    /// through its edges, refuse before any write, naming the word; the
    /// rules and the heat model, which read none of it, give what they give
    /// on the dev board, and the cruise cruises
    /// (`a_board_without_characterization_still_cruises`).
    #[test]
    fn a_board_without_characterization_refuses_only_the_steps_that_need_it() {
        let mut w = common::board_record().registers;
        w.window_scale_q15 = 0;
        let bare = Board::decode(&w).expect("decodes");
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        let mut sink = MemSink::default();
        let e = probe_with(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(false),
            &mut sink,
            &bare,
        )
        .expect_err("refused");
        let word = "CALIB window_scale_q15 reads 0";
        assert!(e.0.contains(word), "{e}");
        let page = descriptor().field("pos_lut_page").expect("page").addr;
        assert!(
            writes.borrow().iter().all(|(a, _)| *a == page),
            "nothing written but the table read's page selects: {:?}",
            writes.borrow()
        );
        assert!(sink.files.is_empty());
        let e = osc_measure::steps::stops::seated_scales(&bare, 1135.0, 248.0)
            .expect_err("the seated read");
        assert!(e.0.contains(word), "{e}");
        let s = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        let sense = rest::Sense::of(&s, &descriptor()).expect("sense");
        let e = osc_measure::steps::window::scales(&sense, &bare, 1200, 93)
            .expect_err("the window term");
        assert!(e.0.contains(word), "{e}");

        let mut i = common::inputs();
        let full = common::rules_set(&i, &common::table()).render();
        i.board = bare.clone();
        assert_eq!(common::rules_set(&i, &common::table()).render(), full);
        let heat = |b: &Board| {
            osc_measure::envelope::HeatModel::new(b, &thermal(), 4.18, 1135.0, 20_000.0)
                .map(|h| format!("{h:?}"))
        };
        assert_eq!(heat(&bare), heat(&board()));
    }

    /// What every way out after a write leaves: torque off and read back,
    /// the dead-man and the telemetry count 0, every volatile field the
    /// step wrote back as it was, every stored field as `start` read it.
    fn ends(c: &mut Client<Bus>, before: &[Vec<u8>], start: &State, p: &Probed) {
        assert_eq!(torque(c), 0, "{:?}", p.ended);
        assert!(p.ended.exit.torque_off, "{}", p.ended.exit);
        assert_eq!(block_on(c.read(ID, 0x192, 2)).expect("dead-man"), [0, 0]);
        assert_eq!(block_on(c.read(ID, 0x1ea, 2)).expect("tel_count"), [0, 0]);
        assert_eq!(table_bytes(c), before, "{}", p.ended.exit);
        // each field went back, but a write to a stored field set the
        // unsaved bit, which only a SAVE or a reboot clears: the command's
        // end reboots onto the stored state where the step began on it,
        // and leaves a servo that began with a volatile value as it is
        let d = descriptor();
        let s = block_on(osc_measure::state::read(c, ID, &d)).expect("state");
        assert!(s.unsaved, "{}", p.ended.exit);
        assert_eq!(s.hash(&d), start.hash(&d), "{}", p.ended.exit);
        let mut steps = Vec::new();
        let after = block_on(install::clear_unsaved(c, ID, &d, start, &mut steps)).expect("reads");
        if start.unsaved {
            assert!(after.is_none() && steps.is_empty(), "{steps:?}");
        } else {
            let after = after.expect("rebooted to clear unsaved");
            assert!(steps.contains(&install::Step::Rebooted), "{steps:?}");
            assert_eq!(after.hash(&d), start.hash(&d), "{}", p.ended.exit);
            assert!(!after.unsaved, "{}", p.ended.exit);
        }
        for name in [
            "current_limit_counts",
            "pos_min_soft_counts",
            "pos_max_soft_counts",
            "mode",
        ] {
            assert!(
                p.ended
                    .exit
                    .lines
                    .iter()
                    .any(|l| l == &format!("{name} put back, read back")),
                "{name}: {}",
                p.ended.exit
            );
        }
        assert!(
            p.ended.lines.iter().any(|l| l.starts_with("end position")),
            "{:?}",
            p.ended.lines
        );
    }

    /// The simulated servo never completes a burst: the poll runs out and
    /// the step aborts through the way out.
    #[test]
    fn a_burst_that_never_completes_ends_through_the_way_out() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        let before = table_bytes(&mut c);
        let start = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        let mut sink = MemSink::default();
        let p = probe_on(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(false),
            &mut sink,
        )
        .expect("starts");
        assert_eq!(p.ended.way, Way::Error, "{:?}", p.ended.why);
        let why = p.ended.why.clone().unwrap_or_default();
        assert!(
            why.contains("shunt burst 0") && why.contains("after 200 polls"),
            "{why}"
        );
        assert!(armed(&writes) && torque_went_on(&writes));
        ends(&mut c, &before, &start, &p);
        let result: serde_json::Value =
            serde_json::from_slice(&sink.files[probe::RESULT]).expect("probe.json");
        assert_eq!(result["verdict"], "aborted");
        assert!(
            result["estimates"]
                .as_object()
                .is_some_and(|m| m.is_empty())
        );
    }

    /// The servo refuses the arm: refused after the step's writes, the rule
    /// named, through the way out.
    #[test]
    fn a_refused_arm_ends_through_the_way_out() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        c.pipe_mut().on_write = Box::new(|addr, _, p| {
            if addr == ARM {
                p.sim_mut()
                    .servo_table_mut(0, |t| t.burst.window.state = burst::STATE_REJECTED);
            }
            false
        });
        let before = table_bytes(&mut c);
        let start = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        let p = probe_on(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(false),
            &mut MemSink::default(),
        )
        .expect("starts");
        assert_eq!(p.ended.way, Way::Refused, "{:?}", p.ended.why);
        let why = p.ended.why.clone().unwrap_or_default();
        assert!(
            why.contains("burst 0: the servo refused the burst"),
            "{why}"
        );
        assert!(torque_went_on(&writes));
        ends(&mut c, &before, &start, &p);
    }

    /// The cable comes out on the first burst's duty write: an error, the
    /// bus back for the way out.
    #[test]
    fn a_bus_error_mid_probe_ends_through_the_way_out() {
        let (mut c, _, rest) = rested(BaudRate::B3000000);
        let mut cut = true;
        c.pipe_mut().on_write = Box::new(move |addr, _, _| {
            let now = addr == DUTY_Q15 && cut;
            cut &= !now;
            now
        });
        let before = table_bytes(&mut c);
        let start = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        let p = probe_on(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(false),
            &mut MemSink::default(),
        )
        .expect("starts");
        assert_eq!(p.ended.way, Way::Error, "{:?}", p.ended.why);
        assert!(
            p.ended.why.clone().unwrap_or_default().contains("bus"),
            "{:?}",
            p.ended.why
        );
        ends(&mut c, &before, &start, &p);
    }

    /// Soft limits 0 / 0, off: the step widens them to the physical range
    /// in one write and the way out puts 0 / 0 back in one write, neither
    /// a side alone the servo would refuse.
    #[test]
    fn soft_limits_off_go_out_and_back_whole() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        block_on(c.write(ID, SOFT, &[0; 8])).expect("soft limits off");
        let phys = block_on(c.read(ID, 0x020, 8)).expect("stops");
        writes.borrow_mut().clear();
        let before = table_bytes(&mut c);
        let start = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        assert_eq!(before[0], [0; 8]);
        let p = probe_on(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(true),
            &mut MemSink::default(),
        )
        .expect("starts");
        assert_eq!(p.ended.way, Way::Interrupted, "{:?}", p.ended.why);
        let soft: Vec<Vec<u8>> = writes
            .borrow()
            .iter()
            .filter(|(a, _)| (SOFT..SOFT + 8).contains(a))
            .map(|(a, b)| {
                assert_eq!(*a, SOFT, "a soft limit written alone");
                b.clone()
            })
            .collect();
        assert_eq!(soft, [phys, vec![0; 8]]);
        ends(&mut c, &before, &start, &p);
    }

    /// The flag set before the first burst: the step stops there, no burst
    /// armed, through the way out.
    #[test]
    fn the_interrupt_flag_stops_the_probe_before_a_burst() {
        let (mut c, writes, rest) = rested(BaudRate::B3000000);
        let before = table_bytes(&mut c);
        let start = block_on(osc_measure::state::read(&mut c, ID, &descriptor())).expect("state");
        let p = probe_on(
            &mut c,
            &nameplate(),
            Some(&thermal()),
            &rest,
            &AtomicBool::new(true),
            &mut MemSink::default(),
        )
        .expect("starts");
        assert_eq!(p.ended.way, Way::Interrupted, "{:?}", p.ended.why);
        assert!(!armed(&writes), "no burst armed");
        ends(&mut c, &before, &start, &p);
    }
}
