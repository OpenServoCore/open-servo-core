//! The rest step on the client's simulated servo at 3 Mbps: what it
//! accepts and refuses, the line it refuses before any write, the burst it
//! will not take as evidence, and the one way out every exit runs.

mod common;

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use common::{ID, descriptor, every_protection, record, table};
use futures_lite::future::block_on;
use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
use osc_client::pipe::{Pipe, PipeError};
use osc_client::{BaudRate, Client, Inst, Opcode, mgmt};
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

/// The fake adapter, every write logged with its bytes.
struct Bus {
    inner: FakePipe,
    writes: Writes,
    arms: usize,
    hook: Hook,
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
                    _scan: [0; 10],
                    pos: pot.unwrap_or_else(|| at(2041, 1)),
                    vref_raw: at(1490, 2),
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
/// resting on `track`.
fn servo(rate: BaudRate, track: Vec<TelSample>, hook: Hook) -> (Client<Bus>, Writes) {
    let mut inner = FakePipe::new(rate, &[1]);
    inner.seed_calibrated(0);
    let writes = Writes::default();
    let mut c = block_on(Client::connect(Bus {
        inner,
        writes: writes.clone(),
        arms: 0,
        hook: Box::new(|_, _| false),
    }))
    .expect("connect");
    let rec = record();
    block_on(install::install(
        &mut c,
        ID,
        &descriptor(),
        &rec,
        Some(&table().points),
        &every_protection(&rec),
        &mut Vec::new(),
    ))
    .expect("the set of record installs");
    block_on(mgmt::save(&mut c, ID)).expect("save");
    c.pipe_mut().inner.set_track(0, track);
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

#[test]
fn rest_refuses_a_noisy_current_zero() {
    let (mut c, _) = servo(BaudRate::B3000000, track(None, 5), Box::new(|_, _| false));
    let r = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect("starts");
    assert_eq!(r.ended.way, Way::Refused);
    assert_eq!(
        failed(&r),
        [
            "current zero noise, torque off",
            "current zero noise, torque on"
        ],
        "{:?}",
        r.checks
    );
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
