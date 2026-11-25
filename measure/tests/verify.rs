//! The closed-loop check: its scores against the notebooks' scorecard on
//! recorded moves (`testdata/verify`, and with OSC_NOTEBOOKS naming a
//! notebooks checkout the whole rung against `full.json`), then the step
//! on the client's simulated servo, which runs no loop: the first
//! enable's order of writes, the refusals before any write, and the way
//! out every exit takes.

mod common;

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
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
use osc_measure::scorecard::{self as sc, Lin, Move, Row, Units};
use osc_measure::sink::MemSink;
use osc_measure::state;
use osc_measure::steps::Rig;
use osc_measure::steps::verify::{self, Verified, VerifyInputs};
use serde_json::Value;

fn json(rel: &str) -> Value {
    common::load(&format!("testdata/verify/{rel}"))
}

/// The rows of a capture by run and segment, as the scores read them: `t`
/// from the segment's first row, the raw pot through `lin`. A capture
/// with no `run` column is run 0.
fn moves_of(csv: &str, lin: &Lin, tick_hz: f64) -> BTreeMap<(i64, i64), Vec<Row>> {
    let mut lines = csv.lines();
    let head: Vec<&str> = lines.next().expect("header").split(',').collect();
    let col = |n: &str| head.iter().position(|h| *h == n).expect(n);
    let (seg, tick, valid, pos, cur, duty) = (
        col("seg"),
        col("tick"),
        col("window_valid"),
        col("pos"),
        col("current_raw"),
        col("duty_counts"),
    );
    let run = head.iter().position(|h| *h == "run");
    let mut out: BTreeMap<(i64, i64), Vec<Row>> = BTreeMap::new();
    let mut first: BTreeMap<(i64, i64), f64> = BTreeMap::new();
    for l in lines {
        let c: Vec<&str> = l.split(',').collect();
        let k = (
            run.map_or(0, |r| c[r].parse().expect("run")),
            c[seg].parse().expect("seg"),
        );
        let t: f64 = c[tick].parse::<f64>().expect("tick");
        let t0 = *first.entry(k).or_insert(t);
        out.entry(k).or_default().push(Row {
            t: (t - t0) / tick_hz,
            pos: lin.counts(c[pos].parse().expect("pos")),
            duty: c[duty].parse().unwrap_or(0),
            current_raw: c[cur].parse().ok(),
            window_valid: c[valid] == "1",
        });
    }
    out
}

fn lin_of(e: &Value) -> Lin {
    let points: Vec<i16> = e["points"]
        .as_array()
        .expect("points")
        .iter()
        .map(|v| v.as_i64().expect("point") as i16)
        .collect();
    let points: [i16; 256] = points.try_into().expect("256 points");
    let slots: Option<Vec<u8>> = e["slots"]
        .as_array()
        .map(|a| a.iter().map(|v| v.as_u64().expect("slot") as u8).collect());
    Lin::new(Some(&points), slots.as_deref())
}

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got} against the scorecard's {want}"
    );
}

/// One move's scores against the scorecard's, and the worst difference
/// of each.
fn compare(rows: Vec<Row>, m: &Value, u: &Units, worst: &mut [f64; 5]) {
    let goal = m["goal_counts"].as_f64().expect("goal");
    let dir = m["direction"].as_f64().expect("direction");
    let mv = Move {
        name: format!("seg {}", m["seg"]),
        from: goal - dir,
        goal,
        rows,
    };
    assert_eq!(mv.rows.len() as u64, m["rows"].as_u64().expect("rows"));
    let mean = mv.rows.iter().map(|r| r.pos).sum::<f64>() / mv.rows.len() as f64;
    let s = sc::score(&mv, u, None);
    let name = &mv.name;
    let pairs = [
        (s.overshoot_deg, "overshoot_deg", 1e-9),
        (s.settle_s, "settle_s", 1e-9),
        (s.rest_error_deg, "rest_error_deg", 1e-4),
        (s.tail_driven_share, "tail_driven_share", 1e-3),
        (1e3 * s.current_peak_a, "i_peak_ma", 1e-6),
    ];
    for (k, (got, key, tol)) in pairs.into_iter().enumerate() {
        let want = m[key].as_f64().expect(key);
        close(&format!("{name} {key}"), got, want, tol);
        worst[k] = worst[k].max((got - want).abs());
    }
    assert_eq!(s.quiet, m["quiet"].as_bool().expect("quiet"), "{name}");
    close(
        &format!("{name} pos mean"),
        mean,
        m["pos_mean_counts"].as_f64().expect("pos"),
        1e-6,
    );
}

/// Four recorded moves of the scorecard's steps captures, 2, 20 and 60
/// deg, both ways, one of them a hold that hunts: the scores are the
/// notebooks' own.
#[test]
fn the_scores_match_the_scorecard_on_recorded_moves() {
    let e = json("expected.json");
    let lin = lin_of(&e);
    let hz = e["tick_hz"].as_f64().expect("hz");
    let text = gunzip(
        &std::fs::read(common::path("testdata/verify/steps-moves.csv.gz")).expect("fixture"),
    )
    .expect("gzip");
    let segs = moves_of(std::str::from_utf8(&text).expect("utf8"), &lin, hz);
    let mut worst = [0.0; 5];
    let mut hunts = 0;
    for m in e["moves"].as_array().expect("moves") {
        let u = Units {
            counts_per_deg: e["counts_per_deg"].as_f64().expect("cpd"),
            counts_per_a: e["counts_per_a"].as_f64().expect("cpa"),
            tick_hz: hz,
            zero_counts: m["zero_counts"].as_f64(),
        };
        let k = (
            m["run"].as_i64().expect("run"),
            m["seg"].as_i64().expect("seg"),
        );
        compare(segs[&k].clone(), m, &u, &mut worst);
        hunts += (m["quiet"] == false) as u32;
    }
    assert_eq!(hunts, 1, "the fixture holds one hold that hunts");
    println!("worst |difference| overshoot, settle, rest, tail share, peak mA: {worst:?}");
}

/// Opt-in: every step move of the rung's three runs, read from the
/// notebooks checkout OSC_NOTEBOOKS, against `full.json`.
#[test]
fn the_whole_rung_matches_the_scorecard() {
    let Some(nb) = std::env::var_os("OSC_NOTEBOOKS") else {
        eprintln!("OSC_NOTEBOOKS unset: the full parity run is skipped");
        return;
    };
    let nb = PathBuf::from(nb);
    let e = json("expected.json");
    let full = json("full.json");
    let lin = lin_of(&e);
    let hz = e["tick_hz"].as_f64().expect("hz");
    let mut worst = [0.0; 5];
    let mut n = 0;
    for run in full["runs"].as_array().expect("runs") {
        let folder = nb.join(run["folder"].as_str().expect("folder"));
        let text =
            std::fs::read_to_string(Path::new(&folder).join("track.csv")).expect("track.csv");
        let segs = moves_of(&text, &lin, hz);
        let all: Vec<Row> = segs.values().flatten().copied().collect();
        let zero = sc::current_zero(all.iter());
        close(
            "the capture's zero",
            zero.expect("zero"),
            run["zero_counts"].as_f64().expect("zero"),
            1e-9,
        );
        let u = Units {
            counts_per_deg: e["counts_per_deg"].as_f64().expect("cpd"),
            counts_per_a: e["counts_per_a"].as_f64().expect("cpa"),
            tick_hz: hz,
            zero_counts: zero,
        };
        for m in run["moves"].as_array().expect("moves") {
            compare(
                segs[&(0, m["seg"].as_i64().expect("seg"))].clone(),
                m,
                &u,
                &mut worst,
            );
            n += 1;
        }
    }
    assert_eq!(n, 36);
    println!(
        "{n} moves; worst |difference| overshoot, settle, rest, tail share, peak mA: {worst:?}"
    );
}

// --- the step on the simulated servo ---

const OSC_EXCHANGE: u8 = 0x20;
const TORQUE: u16 = 0x180;
const MODE: u16 = 0x184;
const GOAL_POSITION: u16 = 0x188;
const DEADMAN: u16 = 0x192;
const CHUNK0: u16 = 0x1e2;
const LUT_PAGE: u16 = 0x19c;

/// What the bus saw: a write (address, bytes, held), or a COMMIT.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Ev {
    Write(u16, Vec<u8>, bool),
    Commit,
}

type Log = Rc<RefCell<Vec<Ev>>>;

/// A write as the log holds it: address, bytes, held.
type Wrote = (u16, Vec<u8>, bool);

/// A hook on every TEL arm, its number from 1, held or not: it may reach
/// into the rig, and fail the arm's own send.
type Hook = Box<dyn FnMut(usize, &mut FakePipe) -> bool>;

struct Bus {
    inner: FakePipe,
    log: Log,
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
        {
            let i = Inst(*inst);
            match i.opcode() {
                Some(Opcode::Write) if payload.len() >= 2 => {
                    let addr = u16::from_le_bytes([payload[0], payload[1]]);
                    self.log
                        .borrow_mut()
                        .push(Ev::Write(addr, payload[2..].to_vec(), i.hold()));
                    if addr == CHUNK0 && payload.len() >= 12 {
                        self.arms += 1;
                        if (self.hook)(self.arms, &mut self.inner) {
                            return Err(PipeError::Io("the cable came out".into()));
                        }
                    }
                }
                Some(Opcode::Commit) => self.log.borrow_mut().push(Ev::Commit),
                _ => {}
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

/// A shaft at rest at raw `pot`, a count of noise on nothing.
fn resting(pot: u16) -> Vec<TelSample> {
    (0..4096u32)
        .map(|i| TelSample {
            tick: TelemetryTick {
                _scan: [0; 9],
                pos: pot,
                vbus_raw: 1241,
                ntc_raw: 2010,
                current_raw: 1206,
                vmotor_a: 773,
                vmotor_b: 772,
                duty_ticks: 0,
                current_trough: 1206,
                i_tick_counts: 0,
                vdiff_counts: 0,
                vmotor_a_trough: 773,
                vmotor_b_trough: 772,
                tick_flags: 0,
                sample_tick: i,
            },
            vbus_counts: 2000,
        })
        .collect()
}

/// The servo of record at `rate`, the set of record installed and saved,
/// resting at raw `pot`, its position estimate the reading of it.
fn servo(rate: BaudRate, pot: u16, hook: Hook) -> (Client<Bus>, Log) {
    let mut inner = FakePipe::new(rate, &[1]);
    inner.seed_calibrated(0);
    let log = Log::default();
    let mut c = block_on(Client::connect(Bus {
        inner,
        log: log.clone(),
        arms: 0,
        hook: Box::new(|_, _| false),
    }))
    .expect("connect");
    let rec = record();
    let d = descriptor();
    block_on(install::install(
        &mut c,
        ID,
        &d,
        &rec,
        Some(&table().points),
        &every_protection(&rec),
        &mut Vec::new(),
    ))
    .expect("the set of record installs");
    block_on(mgmt::save(&mut c, ID)).expect("save");
    c.pipe_mut().inner.set_track(0, resting(pot));
    estimate_at(&mut c, pot);
    c.pipe_mut().hook = hook;
    log.borrow_mut().clear();
    (c, log)
}

/// The servo's estimate set to its own reading of raw `pot`.
fn estimate_at(c: &mut Client<Bus>, pot: u16) {
    let d = descriptor();
    let s = block_on(state::read(c, ID, &d)).expect("state");
    let lin = Lin::new(s.lut.effective(), s.slots(&d).as_deref());
    let q = (lin.counts(pot) * 65536.0).round() as i32;
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.estimates.theta_hat_q16 = q);
}

fn nameplate() -> Nameplate {
    common::load("testdata/mg90-d/nameplate.json")
}

fn run(
    c: &mut Client<Bus>,
    stop: &AtomicBool,
    sink: &mut MemSink,
) -> Result<Verified, install::Refused> {
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
    block_on(verify::verify(&mut rig, &VerifyInputs { nameplate: &np }))
}

fn torque(c: &mut Client<Bus>) -> u8 {
    block_on(c.read(ID, TORQUE, 1)).expect("read")[0]
}

fn writes(log: &Log) -> Vec<Wrote> {
    log.borrow()
        .iter()
        .filter_map(|e| match e {
            Ev::Write(a, b, h) => Some((*a, b.clone(), *h)),
            Ev::Commit => None,
        })
        .collect()
}

/// A raw pot near the middle of the set of record's soft range, 1937
/// counts on its table.
const MID_POT: u16 = 1964;

/// The first enable: the goal on the servo's own estimate, then Position
/// mode and the dead-man, all before torque on; then one plain arm, and
/// the first small step a goal staged with its arm under one COMMIT. The
/// interrupt flag, set at that arm, ends the run after its burst, torque
/// off through the way out.
#[test]
fn the_first_enable_writes_the_goal_and_the_dead_man_before_torque_on() {
    let stop = Rc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let (mut c, log) = servo(
        BaudRate::B3000000,
        MID_POT,
        Box::new(move |n, _| {
            if n == 2 {
                flag.store(true, Ordering::SeqCst);
            }
            false
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &stop, &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Interrupted, "{:?}", v.ended.why);
    assert!(v.ended.exit.torque_off);
    assert_eq!(torque(&mut c), 0);

    let w = writes(&log);
    let at = |pred: &dyn Fn(&Wrote) -> bool| w.iter().position(pred);
    let on = at(&|(a, b, _)| *a == TORQUE && b == &[1]).expect("torque on");
    let goal = at(&|(a, _, h)| *a == GOAL_POSITION && !h).expect("goal");
    let mode = at(&|(a, _, _)| *a == MODE).expect("mode");
    let dm = at(&|(a, b, _)| *a == DEADMAN && b != &[0, 0]).expect("dead-man");
    assert!(goal < on && mode < on && dm < on, "{w:?}");
    assert!(goal < mode, "the goal lands ahead of the mode: {w:?}");
    let enable = v.result.as_ref().expect("result")["enable_goal_counts"]
        .as_f64()
        .expect("goal");
    assert_eq!(
        w[goal].1,
        (enable as i32).to_le_bytes(),
        "the goal is the estimate"
    );
    let first = v.checks.first().expect("the enable check");
    assert_eq!(
        (first.name.as_str(), first.verdict),
        (sc::FIRST_ENABLE, osc_measure::steps::Verdict::Pass),
        "{}",
        first.value
    );
    // the first small step's goal goes out held with its arm, one COMMIT
    let l = log.borrow();
    let held: Vec<usize> = l
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e, Ev::Write(a, _, true) if *a == GOAL_POSITION))
        .map(|(k, _)| k)
        .collect();
    assert_eq!(held.len(), 1, "{l:?}");
    assert!(matches!(&l[held[0] + 1], Ev::Write(a, _, true) if *a == CHUNK0));
    assert_eq!(l[held[0] + 2], Ev::Commit);
    drop(l);
    // the way out puts the mode back and the dead-man to 0
    let tail = &w[on..];
    assert!(tail.iter().any(|(a, b, _)| *a == DEADMAN && b == &[0, 0]));
    let kept = sink
        .files
        .keys()
        .filter(|k| k.starts_with("captures/verify/"))
        .count();
    assert_eq!(
        kept,
        4,
        "two bursts and their meta: {:?}",
        sink.files.keys()
    );
    let result: Value = serde_json::from_slice(&sink.files[verify::RESULT]).expect("verify.json");
    assert_eq!(result["accepted"], false);
    assert_eq!(result["verdict"], "interrupted");
}

/// Refused with nothing written, and nothing kept: torque on, a latched
/// fault, a stamp that is not the live set's, a bus under 3 Mbps, a shaft
/// too near a soft limit.
#[test]
fn the_step_refuses_before_any_write() {
    let refused = |c: &mut Client<Bus>, log: &Log, word: &str| {
        log.borrow_mut().clear();
        let mut sink = MemSink::default();
        let e = run(c, &AtomicBool::new(false), &mut sink).expect_err("refused");
        assert!(e.0.contains(word), "{e}");
        // the table is read through its window: a page and FETCH a write
        let w = writes(log);
        assert!(
            w.iter()
                .all(|(a, b, _)| *a == LUT_PAGE && b.get(1) == Some(&2)),
            "nothing written: {e}: {w:?}"
        );
        assert!(sink.files.is_empty());
    };

    let (mut c, log) = servo(BaudRate::B3000000, MID_POT, Box::new(|_, _| false));
    block_on(c.write(ID, TORQUE, &[1])).expect("torque on");
    refused(&mut c, &log, "torque_enable is on");
    block_on(c.write(ID, TORQUE, &[0])).expect("torque off");

    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x01);
    refused(&mut c, &log, "a fault is latched");
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0);

    let d = descriptor();
    let kp = d.field("p_kp_q88").expect("p_kp_q88");
    let was = block_on(c.read(ID, kp.addr, 2)).expect("read");
    let other = (u16::from_le_bytes([was[0], was[1]]) ^ 1).to_le_bytes();
    block_on(c.write(ID, kp.addr, &other)).expect("a covered field moved");
    refused(
        &mut c,
        &log,
        "the live set is not the one installed and stamped",
    );
    block_on(c.write(ID, kp.addr, &was)).expect("put back");

    c.pipe_mut().inner.set_track(0, resting(330));
    estimate_at(&mut c, 330);
    refused(&mut c, &log, "turn the horn toward the middle");

    let (mut c, log) = servo(BaudRate::B1000000, MID_POT, Box::new(|_, _| false));
    refused(&mut c, &log, "3 Mbaud");
}

/// Soft limits 0 / 0, off: the step's travel is the physical stops, so
/// its edge guard stands in from them, never from 0.
#[test]
fn with_the_soft_limits_off_the_travel_is_the_stops() {
    let (mut c, log) = servo(BaudRate::B3000000, MID_POT, Box::new(|_, _| false));
    let mut i = common::inputs();
    i.policy.soft_margin_share = None;
    let off = common::rules_set(&i, &table());
    block_on(install::install(
        &mut c,
        ID,
        &descriptor(),
        &off,
        Some(&table().points),
        &[],
        &mut Vec::new(),
    ))
    .expect("the soft limits go off");
    c.pipe_mut().inner.set_track(0, resting(200));
    estimate_at(&mut c, 200);
    log.borrow_mut().clear();
    let e = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect_err("refused");
    assert!(
        e.0.contains("of a soft limit (82..3792)") && e.0.contains("turn the horn"),
        "{e}"
    );
}

/// A fault latched under the first enable: refused, the loop not trusted,
/// and torque off through the way out.
#[test]
fn a_fault_at_the_first_enable_is_refused_and_ends_torque_off() {
    let (mut c, _) = servo(
        BaudRate::B3000000,
        MID_POT,
        Box::new(|n, p| {
            if n == 1 {
                p.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x10);
            }
            false
        }),
    );
    let v = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect("starts");
    assert_eq!(v.ended.way, Way::Refused, "{:?}", v.ended.why);
    let why = v.ended.why.clone().unwrap_or_default();
    assert!(why.contains("the loop drove the shaft on its own"), "{why}");
    assert_eq!(v.checks[0].verdict, osc_measure::steps::Verdict::Fail);
    assert!(v.ended.exit.torque_off);
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0);
    assert_eq!(torque(&mut c), 0);
}

/// A shaft that reads off its estimate at torque on: the loop would drive
/// it, refused before the first step.
#[test]
fn a_first_enable_off_the_estimate_is_refused() {
    let (mut c, log) = servo(BaudRate::B3000000, MID_POT, Box::new(|_, _| false));
    // the rows read 60 counts above the goal torque comes on at
    c.pipe_mut().inner.set_track(0, resting(MID_POT + 60));
    let v = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect("starts");
    assert_eq!(v.ended.way, Way::Refused, "{:?}", v.ended.why);
    assert!(
        v.ended
            .why
            .as_deref()
            .is_some_and(|w| w.contains("the loop drove the shaft on its own")),
        "{:?}",
        v.ended.why
    );
    assert!(!log.borrow().contains(&Ev::Commit), "no step was committed");
    assert!(v.ended.exit.torque_off);
    assert_eq!(torque(&mut c), 0);
}

/// The cable out at a small step's arm: aborted, and the way out still
/// runs; the dead-man drops torque on hardware, the simulated servo reads
/// it off.
#[test]
fn a_bus_error_mid_step_ends_through_the_way_out() {
    let (mut c, _) = servo(BaudRate::B3000000, MID_POT, Box::new(|n, _| n == 2));
    let v = run(&mut c, &AtomicBool::new(false), &mut MemSink::default()).expect("starts");
    assert_eq!(v.ended.way, Way::Error, "{:?}", v.ended.why);
    assert!(v.ended.exit.torque_off, "{}", v.ended.exit);
    assert_eq!(torque(&mut c), 0);
}

/// A fault latched in the first self-test move: the set is not accepted
/// on "no fault" alone, the failing move named, and the step ends torque
/// off. The simulated servo runs no loop and the shaft never moves, so the
/// first enable and the small steps pass: an error that stays is not one
/// that grows.
#[test]
fn a_fault_in_the_self_test_fails_no_fault_and_ends_torque_off() {
    // arms: the enable, four small steps, the move to the middle, then
    // the first move
    let (mut c, _) = servo(
        BaudRate::B3000000,
        MID_POT,
        Box::new(|n, p| {
            if n == 7 {
                p.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x10);
            }
            false
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Refused, "{:?}", v.ended.why);
    let result: Value = serde_json::from_slice(&sink.files[verify::RESULT]).expect("verify.json");
    assert_eq!(result["accepted"], false);
    assert_eq!(
        result["failed_on"],
        serde_json::json!([sc::NO_FAULT]),
        "{result}"
    );
    assert_eq!(result["fault_on"], "1% + out");
    assert_eq!(
        result["moves"].as_array().map(Vec::len),
        Some(0),
        "the faulted move is not scored"
    );
    let small = result["small_steps"].as_array().expect("small steps");
    assert_eq!(small.len(), 4);
    assert!(small.iter().all(|s| s["failed"] == false), "{result}");
    assert!(v.ended.exit.torque_off);
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0);
    assert_eq!(torque(&mut c), 0);
}
