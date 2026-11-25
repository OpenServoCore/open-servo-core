//! The cruise step on the client's simulated servo, which runs no loop. A
//! pipe that logs the bus stands in for the plant: a Position goal moves
//! the shaft there at once, and a Velocity goal armed with a burst plays
//! rows of the shaft following the profile's ramp at the set's
//! acceleration, its winding current a friction line of its speed, so a
//! whole step reads valid legs and fits that line. Knobs on it make the
//! shaft run fast or stay put, the current pass the cap, the host slow,
//! a row or a frame go missing. The order of the writes, the refusals
//! before any write, and the way out every exit takes, centered only while
//! the loop is trusted.

mod common;

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use common::{ID, descriptor, every_protection, record, table};
use futures_lite::future::block_on;
use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
use osc_client::pipe::{Pipe, PipeError};
use osc_client::{BaudRate, Client, Inst, Opcode, mgmt};
use osc_measure::fit::cruise as cruise_fit;
use osc_measure::guard::Way;
use osc_measure::install;
use osc_measure::nameplate::Nameplate;
use osc_measure::scorecard::Lin;
use osc_measure::sink::MemSink;
use osc_measure::state;
use osc_measure::steps::cruise::{self, CruiseInputs, Cruised, LEGS_EACH};
use osc_measure::steps::{Rig, rest};
use osc_measure::{Board, install::Refused};
use serde_json::{Value, json};

const OSC_EXCHANGE: u8 = 0x20;
const TICK_HZ: f64 = 20000.0;
/// The set of record's acceleration, counts/s a MEDIUM tick.
const ACCEL: f64 = 10305.0 / 256.0;
/// The plant's friction line, mA and mA per count/s.
const FC_MA: f64 = 40.0;
const FV_MA_PER_CPS: f64 = 0.011;
/// The duty the plant's legs run at: wide, past the settle table.
const DUTY: i16 = 600;
/// The rail every row and poll reads, tap counts.
const VBUS_RAW: u16 = 1241;
/// Rows a leg's motion runs past its burst, then the shaft holds.
const SLACK: usize = 200;
const HOLD: usize = 20000;

fn addr(name: &str) -> u16 {
    descriptor().field(name).expect(name).addr
}

/// What the bus saw: a write (address, bytes, held), or a COMMIT.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Ev {
    Write(u16, Vec<u8>, bool),
    Commit,
}

type Log = Rc<RefCell<Vec<Ev>>>;
/// A write as logged: address, bytes, held.
type Wrote = (u16, Vec<u8>, bool);

/// The plant's knobs, a hook on every TEL arm may turn.
struct Knobs {
    /// Legs play moving rows; else the shaft never leaves its place.
    moving: bool,
    /// The shaft's speed over the goal's.
    speed_gain: f64,
    /// The legs' current over the cap.
    over_cap: bool,
    /// Position goals are not followed.
    frozen: bool,
    /// The host's clock moves this far on every frame sent, s.
    send_s: f64,
    /// The next frame sent fails.
    fail_next: bool,
}

impl Default for Knobs {
    fn default() -> Self {
        Knobs {
            moving: true,
            speed_gain: 1.0,
            over_cap: false,
            frozen: false,
            send_s: 0.0,
            fail_next: false,
        }
    }
}

/// A hook on every TEL arm, its number from 1.
type Hook = Box<dyn FnMut(usize, &mut FakePipe, &mut Knobs)>;

/// The fake adapter, every write logged, the plant played on it.
struct Bus {
    inner: FakePipe,
    log: Log,
    arms: usize,
    hook: Hook,
    knobs: Knobs,
    clock: Rc<Cell<f64>>,
    lin: Option<Lin>,
    /// The table's points sorted, to put a linearized position on a pot.
    inverse: Vec<(f64, u16)>,
    bias: f64,
    counts_per_a: f64,
    /// The window term the crest sample carries at [`DUTY`], counts.
    term: u16,
    at: f64,
    speed: f64,
    goal_speed: Option<f64>,
}

fn sample(pot: u16, current_raw: u16, duty: i16, k: usize) -> TelSample {
    TelSample {
        tick: TelemetryTick {
            _scan: [0; 10],
            pos: pot,
            vref_raw: 1490,
            vbus_raw: VBUS_RAW,
            ntc_raw: 2010,
            current_raw,
            vmotor_a: 773,
            vmotor_b: 773,
            duty_ticks: duty,
            current_trough: current_raw,
            i_tick_counts: 0,
            vdiff_counts: 0,
            vmotor_a_trough: 773,
            vmotor_b_trough: 773,
            tick_flags: 0,
            sample_tick: k as u32,
        },
        vbus_counts: 2000,
    }
}

impl Bus {
    fn pot(&self, at: f64) -> u16 {
        let k = self.inverse.partition_point(|(x, _)| *x < at);
        let pick = |k: usize| self.inverse.get(k).map(|(x, r)| ((x - at).abs(), *r));
        match (k.checked_sub(1).and_then(pick), pick(k)) {
            (Some(a), Some(b)) => {
                if a.0 <= b.0 {
                    a.1
                } else {
                    b.1
                }
            }
            (Some(a), None) | (None, Some(a)) => a.1,
            (None, None) => 0,
        }
    }

    fn estimate(&mut self, at: f64) {
        let q = (at * 65536.0).round() as i32;
        self.inner
            .sim_mut()
            .servo_table_mut(0, |t| t.telemetry.estimates.theta_hat_q16 = q);
    }

    /// A leg's rows: the shaft follows the goal at the profile's ramp for
    /// the burst and [`SLACK`] rows past it, then holds; the next leg takes
    /// up its speed where this one left it, as a shaft between bursts.
    fn cruise(&mut self, goal: f64, rows: u16) {
        if !self.knobs.moving {
            return;
        }
        let target = goal * self.knobs.speed_gain;
        let mut track = Vec::with_capacity(rows as usize + SLACK + HOLD);
        let bias = self.bias.round() as u16;
        for k in 0..rows as usize + SLACK {
            if k % 10 == 0 {
                self.speed += (target - self.speed).clamp(-ACCEL, ACCEL);
            }
            self.at += self.speed / TICK_HZ;
            let (raw, duty) = if goal == 0.0 {
                (bias, 0)
            } else if self.knobs.over_cap {
                (bias + 1400, DUTY * goal.signum() as i16)
            } else {
                let ma = FC_MA + FV_MA_PER_CPS * self.speed.abs();
                let counts = (self.counts_per_a * ma / 1e3).round() as u16;
                (bias + counts + self.term, DUTY * goal.signum() as i16)
            };
            track.push(sample(self.pot(self.at), raw, duty, k));
        }
        let pot = self.pot(self.at);
        let n = track.len();
        track.extend((0..HOLD).map(|k| sample(pot, bias, 0, n + k)));
        self.inner.set_track(0, track);
        let at = self.at;
        self.estimate(at);
    }

    fn move_to(&mut self, goal: f64) {
        if self.knobs.frozen {
            return;
        }
        self.at = goal;
        let pot = self.pot(goal);
        let bias = self.bias.round() as u16;
        self.inner
            .set_track(0, (0..4096).map(|k| sample(pot, bias, 0, k)).collect());
        self.estimate(goal);
    }
}

impl Pipe for Bus {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        self.clock.set(self.clock.get() + self.knobs.send_s);
        if std::mem::take(&mut self.knobs.fail_next) {
            return Err(PipeError::Io("the cable came out".into()));
        }
        let mut moved = None;
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
                    let a = u16::from_le_bytes([payload[0], payload[1]]);
                    self.log
                        .borrow_mut()
                        .push(Ev::Write(a, payload[2..].to_vec(), i.hold()));
                    let word =
                        || i32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
                    if a == addr("goal_position") && payload.len() == 6 {
                        moved = Some(word() as f64);
                    }
                    if a == addr("goal_velocity") && payload.len() == 6 && i.hold() {
                        self.goal_speed = Some(word() as f64);
                    }
                    if a == addr("chunk0_addr") && payload.len() >= 12 {
                        self.arms += 1;
                        (self.hook)(self.arms, &mut self.inner, &mut self.knobs);
                        let rows = u16::from_le_bytes([payload[10], payload[11]]);
                        self.clock.set(self.clock.get() + rows as f64 / TICK_HZ);
                        if let Some(g) = self.goal_speed.take() {
                            self.cruise(g, rows);
                        }
                    }
                }
                Some(Opcode::Commit) => self.log.borrow_mut().push(Ev::Commit),
                _ => {}
            }
        }
        let r = self.inner.send(bytes).await;
        if let Some(g) = moved {
            self.speed = 0.0;
            self.move_to(g);
        }
        r
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        self.inner.recv().await
    }

    async fn pause(&mut self, d: Duration) {
        self.inner.pause(d).await
    }
}

/// A raw pot near the middle of the set of record's soft range.
const MID_POT: u16 = 1964;

/// The servo of record, the set of record installed and saved, resting at
/// raw `pot`, its estimate the reading of it.
fn servo(pot: u16, hook: Hook) -> (Client<Bus>, Log) {
    let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
    inner.seed_calibrated(0);
    let log = Log::default();
    let mut c = block_on(Client::connect(Bus {
        inner,
        log: log.clone(),
        arms: 0,
        hook: Box::new(|_, _, _| {}),
        knobs: Knobs::default(),
        clock: Rc::new(Cell::new(1.7e9)),
        lin: None,
        inverse: Vec::new(),
        bias: 0.0,
        counts_per_a: 0.0,
        term: 0,
        at: 0.0,
        speed: 0.0,
        goal_speed: None,
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
    let s = block_on(state::read(&mut c, ID, &d)).expect("state");
    let lin = Lin::new(s.lut.effective(), s.slots(&d).as_deref());
    let mut inverse: Vec<(f64, u16)> = (0..4096u16).map(|r| (lin.counts(r), r)).collect();
    inverse.sort_by(|a, b| a.0.total_cmp(&b.0));
    let word = |c: &mut Client<Bus>, name: &str| {
        let b = block_on(c.read(ID, addr(name), 2)).expect(name);
        u16::from_le_bytes([b[0], b[1]])
    };
    let bias = word(&mut c, "current_bias_counts");
    let g = word(&mut c, "g_window_q016");
    let arr = word(&mut c, "pwm_period_ticks");
    let p = c.pipe_mut();
    // the resistance a kernel would publish, the record's, and the rail
    // the samples carry
    p.inner.sim_mut().servo_table_mut(0, |t| {
        t.telemetry.estimates.r_hat_q12 = 3676;
        t.telemetry.tick.vbus_raw = VBUS_RAW;
    });
    p.term = cruise_fit::term_counts(g, DUTY as i64, VBUS_RAW as i64, arr as i64) as u16;
    p.bias = bias as f64;
    p.counts_per_a = rest::Sense::of(&s, &d).expect("sense").counts_per_a();
    p.inverse = inverse;
    p.lin = Some(lin.clone());
    p.move_to(lin.counts(pot));
    p.hook = hook;
    log.borrow_mut().clear();
    (c, log)
}

fn nameplate() -> Nameplate {
    common::load("testdata/mg90-d/nameplate.json")
}

fn board() -> Board {
    common::load("boards/osc-dev-v006.json")
}

fn run_with(
    c: &mut Client<Bus>,
    stop: &AtomicBool,
    sink: &mut MemSink,
    stops: Option<&Value>,
) -> Result<Cruised, Refused> {
    let d = descriptor();
    let time = c.pipe_mut().clock.clone();
    let clock = move || time.get();
    let (np, b) = (nameplate(), board());
    let mut rig = Rig {
        c,
        id: ID,
        d: &d,
        sink,
        stop,
        clock: &clock,
    };
    block_on(cruise::cruise(
        &mut rig,
        &CruiseInputs {
            nameplate: &np,
            board: &b,
            stops,
        },
    ))
}

fn run(c: &mut Client<Bus>, stop: &AtomicBool, sink: &mut MemSink) -> Result<Cruised, Refused> {
    run_with(c, stop, sink, None)
}

fn torque(c: &mut Client<Bus>) -> u8 {
    block_on(c.read(ID, addr("torque_enable"), 1)).expect("read")[0]
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

fn result(sink: &MemSink) -> Value {
    serde_json::from_slice(&sink.files[cruise::RESULT]).expect("cruise.json")
}

fn said(v: &Cruised) -> String {
    let mut text = v.ended.lines.join("\n");
    for c in &v.checks {
        text.push_str(&format!("\n  {}", c.said()));
    }
    text
}

fn mode_of(name: &str) -> u8 {
    descriptor()
        .field("mode")
        .expect("mode")
        .variants
        .iter()
        .find(|v| v.name == name)
        .expect(name)
        .value
}

fn ticks(s: f64) -> Vec<u8> {
    ((s * TICK_HZ).round() as u16).to_le_bytes().to_vec()
}

/// The way out ran: torque off read back, the dead-man 0 and the mode put
/// back.
fn ended_torque_off(c: &mut Client<Bus>, v: &Cruised, log: &Log) {
    assert!(v.ended.exit.torque_off, "{}", v.ended.exit);
    assert_eq!(torque(c), 0);
    let w = writes(log);
    let off = w
        .iter()
        .rposition(|(a, b, _)| *a == addr("torque_enable") && b == &[0])
        .expect("torque off");
    assert!(
        w[off..]
            .iter()
            .any(|(a, b, _)| *a == addr("bus_deadman_ticks") && b == &[0, 0]),
        "{w:?}"
    );
    assert!(
        v.ended
            .exit
            .lines
            .iter()
            .any(|l| l == "mode put back, read back"),
        "{}",
        v.ended.exit
    );
}

/// The whole step on a shaft that follows its goals: every speed runs its
/// legs, each Velocity goal held with its burst's arm under one COMMIT,
/// under the short dead-man, the longer one back for the Position phases;
/// every leg reads valid and the line is the plant's, accepted, its
/// estimates kept; the shaft ends at the middle, torque off.
#[test]
fn every_speed_cruises_and_the_step_is_accepted() {
    let (mut c, log) = servo(MID_POT, Box::new(|_, _, _| {}));
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    let text = said(&v);
    println!("{text}");
    assert_eq!(v.ended.way, Way::Done, "{:?}\n{text}", v.ended.why);
    assert!(v.accepted());
    ended_torque_off(&mut c, &v, &log);
    let r = result(&sink);
    assert_eq!(r["verdict"], "accepted");
    assert_eq!(r["legs"].as_array().map(Vec::len), Some(4 * 2 * LEGS_EACH));
    assert!(
        r["legs"]
            .as_array()
            .expect("legs")
            .iter()
            .all(|l| l["valid"] == true),
        "{}",
        r["legs"]
    );
    let cpmr = r["scales"]["counts_per_motor_rad"].as_f64().expect("cpmr");
    let e = &r["estimates"];
    let fc = e["friction_coulomb_ma"]["value"].as_f64().expect("fc");
    let fv = e["friction_viscous_ma_s_per_rad"]["value"]
        .as_f64()
        .expect("fv");
    println!("the line: {fc:.3} mA + {fv:.5} mA per rad/s");
    assert!((fc - FC_MA).abs() < 0.5, "{fc}");
    assert!((fv / (FV_MA_PER_CPS * cpmr) - 1.0).abs() < 0.02, "{fv}");
    for k in ["friction_coulomb_ma", "friction_viscous_ma_s_per_rad"] {
        let x = &e[k];
        assert!(x["lo"].as_f64() < x["value"].as_f64() && x["value"].as_f64() < x["hi"].as_f64());
        assert!(
            x["source"]
                .as_str()
                .is_some_and(|s| s.contains("standard error"))
        );
    }
    assert!(r["line"]["coulomb_se_ma"].is_number(), "{}", r["line"]);
    let legs = r["legs"].as_array().expect("legs");
    assert!(
        legs.iter().all(
            |l| l["before"]["vbus_raw"].as_f64() == Some(VBUS_RAW as f64)
                && l["after"]["t_winding_cc"].is_number()
                && l["read_share"] == 1.0
        ),
        "{}",
        r["legs"]
    );
    assert_eq!(r["t_read_cc"], legs[0]["before"]["t_winding_cc"]);
    assert_eq!(r["scales"]["r_hat_q12"], 3676);
    let mid = r["travel"]["middle_counts"].as_f64().expect("middle");
    let centered = r["position_centered_counts"].as_f64().expect("centered");
    assert!((centered - mid).abs() <= 24.0, "{centered} {mid}");

    // the velocity goals: each held with its arm, one COMMIT carrying both
    let gv = addr("goal_velocity");
    let l = log.borrow();
    let held: Vec<usize> = l
        .iter()
        .enumerate()
        .filter(|(_, e)| matches!(e, Ev::Write(a, _, true) if *a == gv))
        .map(|(k, _)| k)
        .collect();
    assert_eq!(
        held.len(),
        4 * (2 * LEGS_EACH + 1),
        "the legs and the stops"
    );
    for k in &held {
        assert!(matches!(&l[k + 1], Ev::Write(a, _, true) if *a == addr("chunk0_addr")));
        assert_eq!(l[k + 2], Ev::Commit);
    }
    let goals: Vec<i32> = held
        .iter()
        .map(|k| match &l[*k] {
            Ev::Write(_, b, _) => i32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            Ev::Commit => unreachable!(),
        })
        .collect();
    assert_eq!(&goals[..5], &[322, -322, 322, -322, 0], "{goals:?}");
    assert_eq!(&goals[15..], &[4025, -4025, 4025, -4025, 0], "{goals:?}");
    drop(l);

    // goal_velocity 0, the goal, Position, the dead-man before torque on;
    // each speed: the short dead-man, then Velocity; after its stop,
    // Position at the estimate, then the long dead-man
    let w = writes(&log);
    let at = |pred: &dyn Fn(&Wrote) -> bool| w.iter().position(pred);
    let on = at(&|(a, b, _)| *a == addr("torque_enable") && b == &[1]).expect("torque on");
    let zero = at(&|(a, b, h)| *a == gv && b == &[0, 0, 0, 0] && !h).expect("goal_velocity 0");
    let goal = at(&|(a, _, h)| *a == addr("goal_position") && !h).expect("goal");
    let mode = at(&|(a, _, _)| *a == addr("mode")).expect("mode");
    let dm = at(&|(a, b, _)| *a == addr("bus_deadman_ticks") && b != &[0, 0]).expect("dead-man");
    assert!(zero < on && goal < on && mode < on && dm < on, "{w:?}");
    assert_eq!(w[dm].1, ticks(rest::DEADMAN_S));
    let (p, vel) = (mode_of("Position"), mode_of("Velocity"));
    let seq: Vec<(char, Vec<u8>)> = w[on..]
        .iter()
        .filter(|(a, _, _)| *a == addr("mode") || *a == addr("bus_deadman_ticks"))
        .map(|(a, b, _)| (if *a == addr("mode") { 'm' } else { 'd' }, b.clone()))
        .collect();
    let one = [
        ('d', ticks(cruise::VELOCITY_DEADMAN_S)),
        ('m', vec![vel]),
        ('m', vec![p]),
        ('d', ticks(rest::DEADMAN_S)),
    ];
    for k in 0..4 {
        assert_eq!(seq[4 * k..4 * k + 4], one, "{seq:?}");
    }
    let kept = sink
        .files
        .keys()
        .filter(|k| k.starts_with("captures/cruise/"))
        .count();
    assert_eq!(
        kept,
        2 * (1 + 4 * (2 * LEGS_EACH + 1)),
        "{:?}",
        sink.files.keys()
    );
}

/// A shaft that never cruises: every leg reads a shaft at rest, set aside,
/// and the line is refused; the shaft still ends at the middle.
#[test]
fn a_shaft_at_rest_sets_every_leg_aside() {
    let (mut c, log) = servo(MID_POT, Box::new(|_, _, k| k.moving = false));
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Refused, "{:?}", v.ended.why);
    ended_torque_off(&mut c, &v, &log);
    let r = result(&sink);
    assert!(
        r["legs"]
            .as_array()
            .expect("legs")
            .iter()
            .all(|l| l["valid"] == false)
    );
    assert!(r["estimates"].as_object().is_some_and(|m| m.is_empty()));
    assert!(r["position_centered_counts"].is_number());
}

/// Refused with nothing written: torque on, a fault, a stamp that is not
/// the live set's, a data state that refuses the closed loops, soft limits
/// not inside the stops, a shaft by a soft limit.
#[test]
fn the_step_refuses_before_any_write() {
    let refused = |c: &mut Client<Bus>, log: &Log, word: &str| {
        log.borrow_mut().clear();
        let mut sink = MemSink::default();
        let e = run(c, &AtomicBool::new(false), &mut sink).expect_err("refused");
        assert!(e.0.contains(word), "{e}");
        let page = addr("pos_lut_page");
        let w = writes(log);
        assert!(
            w.iter().all(|(a, b, _)| *a == page && b.get(1) == Some(&2)),
            "nothing written: {e}: {w:?}"
        );
        assert!(sink.files.is_empty());
    };
    let restamp = |c: &mut Client<Bus>| {
        let d = descriptor();
        let s = block_on(state::read(c, ID, &d)).expect("state");
        let f = d.field("plant_stamp").expect("plant_stamp");
        block_on(c.write(ID, f.addr, &s.stamp.computed.to_le_bytes())).expect("restamped");
    };
    let (mut c, log) = servo(MID_POT, Box::new(|_, _, _| {}));
    let t = addr("torque_enable");
    block_on(c.write(ID, t, &[1])).expect("torque on");
    refused(&mut c, &log, "torque_enable is on");
    block_on(c.write(ID, t, &[0])).expect("torque off");

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

    // the plant unset: the stamp still matches, the data state refuses
    let ke = d.field("recip_ke_q").expect("recip_ke_q");
    let ke_was = block_on(c.read(ID, ke.addr, 2)).expect("read");
    block_on(c.write(ID, ke.addr, &[0, 0])).expect("Ke unset");
    restamp(&mut c);
    refused(&mut c, &log, "refuses the closed loops");
    block_on(c.write(ID, ke.addr, &ke_was)).expect("put back");
    restamp(&mut c);

    // the low soft limit out to the stop: nothing bounds a leg in flight
    let lo = d.field("pos_min_soft_counts").expect("soft");
    let soft_was = block_on(c.read(ID, lo.addr, 2)).expect("read");
    block_on(c.write(ID, lo.addr, &100u16.to_le_bytes())).expect("soft limit moved");
    restamp(&mut c);
    refused(&mut c, &log, "counts inside the stops");
    block_on(c.write(ID, lo.addr, &soft_was)).expect("put back");
    restamp(&mut c);

    let lin = c.pipe_mut().lin.clone().expect("lin");
    c.pipe_mut().move_to(lin.counts(330));
    refused(&mut c, &log, "turn the horn toward the middle");
}

/// The travel from a run folder's stops result: an accepted one that
/// agrees with the set is the travel, said; one not accepted, or another
/// servo's, is refused with nothing written.
#[test]
fn a_stops_result_is_the_travel_where_it_agrees_with_the_set() {
    let stops = |lo: f64, hi: f64, verdict: &str| {
        json!({"step": "stops", "verdict": verdict,
               "stops": {"low": {"median_counts": lo}, "high": {"median_counts": hi}}})
    };
    let (mut c, log) = servo(MID_POT, Box::new(|_, _, _| {}));
    for (doc, word) in [
        (stops(107.0, 3729.75, "refused"), "is refused, not accepted"),
        (
            stops(600.0, 3000.0, "accepted"),
            "another servo's or an older seat's",
        ),
    ] {
        log.borrow_mut().clear();
        let mut sink = MemSink::default();
        let e =
            run_with(&mut c, &AtomicBool::new(false), &mut sink, Some(&doc)).expect_err("refused");
        assert!(e.0.contains(word), "{e}");
        assert!(sink.files.is_empty());
        let page = addr("pos_lut_page");
        assert!(writes(&log).iter().all(|(a, _, _)| *a == page));
    }
    let mut sink = MemSink::default();
    let doc = stops(107.0, 3729.75, "accepted");
    let v = run_with(&mut c, &AtomicBool::new(false), &mut sink, Some(&doc)).expect("starts");
    assert_eq!(v.ended.way, Way::Done, "{}", said(&v));
    let r = result(&sink);
    assert_eq!(r["travel"]["from"], "the stops step's stops");
}

/// A fault latched under the first leg: the loop is not trusted, so the
/// shaft is not driven to the middle; torque off through the way out.
#[test]
fn a_fault_in_a_leg_ends_torque_off_where_it_is() {
    let (mut c, log) = servo(
        MID_POT,
        Box::new(|n, p, _| {
            if n == 2 {
                p.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x10);
            }
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Error, "{:?}", v.ended.why);
    let why = v.ended.why.clone().unwrap_or_default();
    assert!(
        why.contains("a fault latched on the 322 counts/s leg 1 up"),
        "{why}"
    );
    let text = said(&v);
    assert!(
        text.contains("not brought to the middle: the loop is not trusted"),
        "{text}"
    );
    let r = result(&sink);
    assert!(
        r["fault"].as_str().is_some_and(|f| f.contains("leg 1 up")),
        "{r}"
    );
    assert!(r["position_centered_counts"].is_null());
    let gp = addr("goal_position");
    let l = log.borrow();
    let leg = l
        .iter()
        .position(|e| matches!(e, Ev::Write(a, _, true) if *a == addr("goal_velocity")))
        .expect("the leg");
    assert!(
        !l[leg..]
            .iter()
            .any(|e| matches!(e, Ev::Write(a, _, _) if *a == gp))
    );
    drop(l);
    c.pipe_mut()
        .inner
        .sim_mut()
        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0);
    ended_torque_off(&mut c, &v, &log);
}

/// The watchdog on a leg's rows, the current over the cap: the loop is not
/// trusted, torque off where it is.
#[test]
fn a_watchdog_trip_mid_run_ends_torque_off_where_it_is() {
    let (mut c, log) = servo(MID_POT, Box::new(|n, _, k| k.over_cap = n == 7));
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Error, "{:?}", v.ended.why);
    let why = v.ended.why.clone().unwrap_or_default();
    assert!(
        why.contains("the watchdog on the rows") && why.contains("cap"),
        "{why}"
    );
    assert!(said(&v).contains("not brought to the middle: the loop is not trusted"));
    assert!(result(&sink)["position_centered_counts"].is_null());
    ended_torque_off(&mut c, &v, &log);
}

/// A leg that lost rows, and a frame lost mid-leg: errors with the loop
/// trusted, so the shaft is brought to the middle, the Position phases'
/// dead-man back before the wait, then torque off and the ledger put back.
#[test]
fn lost_rows_or_a_lost_frame_mid_leg_center_then_end_torque_off() {
    let cases: [(&str, Hook); 2] = [
        (
            "lost rows",
            Box::new(|n, p, _| {
                if n == 3 {
                    p.sim_mut().lose_tel_ticks(0, 500, 501);
                } else {
                    p.sim_mut().lose_tel_ticks(0, 0, 0);
                }
            }),
        ),
        (
            "the cable came out",
            Box::new(|n, _, k| {
                if n == 4 {
                    k.fail_next = true;
                }
            }),
        ),
    ];
    for (name, hook) in cases {
        let (mut c, log) = servo(MID_POT, hook);
        let mut sink = MemSink::default();
        let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
        assert_eq!(v.ended.way, Way::Error, "{name}: {:?}", v.ended.why);
        let why = v.ended.why.clone().unwrap_or_default();
        assert!(why.contains(name), "{name}: {why}");
        let text = said(&v);
        assert!(
            text.contains("after the error: brought to the middle"),
            "{name}: {text}"
        );
        let w = writes(&log);
        let gv = addr("goal_velocity");
        let last_leg = w
            .iter()
            .rposition(|(a, _, h)| *a == gv && *h)
            .expect("a leg");
        let tail = &w[last_leg + 1..];
        let zero = tail
            .iter()
            .position(|(a, b, h)| *a == gv && b == &[0, 0, 0, 0] && !h)
            .expect("goal speed 0");
        let long = tail
            .iter()
            .position(|(a, b, _)| *a == addr("bus_deadman_ticks") && *b == ticks(rest::DEADMAN_S))
            .expect("the long dead-man back");
        assert!(zero < long, "{name}: {tail:?}");
        ended_torque_off(&mut c, &v, &log);
    }
}

/// The host slow between two legs: the gap over the plan's ends that
/// speed after its first leg, said, and the next speeds run whole.
#[test]
fn a_gap_over_the_plan_ends_the_speed() {
    let (mut c, log) = servo(
        MID_POT,
        Box::new(|n, _, k| k.send_s = if n == 2 { 0.02 } else { 0.0 }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    let text = said(&v);
    assert!(
        text.contains("the 322 counts/s speed ends after 1 legs: the host took"),
        "{text}"
    );
    let r = result(&sink);
    let cut = &r["plan"]["cut_short"];
    assert_eq!(cut.as_array().map(Vec::len), Some(1), "{cut}");
    assert_eq!(cut[0]["speed_cps"], 322.0);
    let legs = r["legs"].as_array().expect("legs");
    assert_eq!(legs.len(), 1 + 3 * 2 * LEGS_EACH);
    ended_torque_off(&mut c, &v, &log);
}

/// The shaft running past the top speed's goal: the leg ends past where
/// the plan put it, the next leg's prediction leaves the span, and the
/// speed ends there, its stop run.
#[test]
fn a_shaft_past_its_plan_ends_the_speed_inside_the_step() {
    let (mut c, log) = servo(
        MID_POT,
        Box::new(|n, _, k| k.speed_gain = if n == 17 { 1.08 } else { 1.0 }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    let text = said(&v);
    assert!(
        text.contains("the 4025 counts/s speed ends after 1 legs: from"),
        "{text}"
    );
    assert!(text.contains("past the span"), "{text}");
    let gv = addr("goal_velocity");
    let goals: Vec<i32> = writes(&log)
        .iter()
        .filter(|(a, _, h)| *a == gv && *h)
        .map(|(_, b, _)| i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .collect();
    assert_eq!(&goals[goals.len() - 2..], &[4025, 0], "{goals:?}");
    ended_torque_off(&mut c, &v, &log);
}

/// The interrupt between legs: the goal speed zeroed, the long dead-man,
/// Position mode at the estimate, the move to the middle, then torque off.
#[test]
fn an_interrupt_between_legs_centers_then_ends_torque_off() {
    let stop = Rc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let (mut c, log) = servo(
        MID_POT,
        Box::new(move |n, _, _| {
            if n == 3 {
                flag.store(true, Ordering::SeqCst);
            }
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &stop, &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Interrupted, "{:?}", v.ended.why);
    let text = said(&v);
    assert!(
        text.contains("after the error: brought to the middle"),
        "{text}"
    );
    let r = result(&sink);
    let mid = r["travel"]["middle_counts"].as_f64().expect("middle");
    let centered = r["position_centered_counts"].as_f64().expect("centered");
    assert!((centered - mid).abs() <= 24.0, "{centered} {mid}");
    assert_eq!(r["legs"].as_array().map(Vec::len), Some(2));
    let w = writes(&log);
    let gv = addr("goal_velocity");
    let last_leg = w
        .iter()
        .rposition(|(a, _, h)| *a == gv && *h)
        .expect("a leg");
    let tail = &w[last_leg + 1..];
    let zero = tail
        .iter()
        .position(|(a, b, h)| *a == gv && b == &[0, 0, 0, 0] && !h)
        .expect("goal speed 0");
    let to_mid = tail
        .iter()
        .rposition(|(a, b, _)| {
            *a == addr("goal_position")
                && i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64 == mid
        })
        .expect("the move to the middle");
    let off = tail
        .iter()
        .position(|(a, b, _)| *a == addr("torque_enable") && b == &[0])
        .expect("torque off");
    assert!(zero < to_mid && to_mid < off, "{tail:?}");
    ended_torque_off(&mut c, &v, &log);
}

/// The interrupt with a shaft that no longer follows a Position goal: the
/// recovery's move never arrives, said, and the way out still ends torque
/// off.
#[test]
fn a_recovery_that_never_arrives_still_ends_torque_off() {
    let stop = Rc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let (mut c, log) = servo(
        MID_POT,
        Box::new(move |n, _, k| {
            if n == 18 {
                flag.store(true, Ordering::SeqCst);
                k.frozen = true;
            }
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &stop, &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Interrupted, "{:?}", v.ended.why);
    let text = said(&v);
    assert!(
        text.contains("after the error: not brought to the middle: the shaft read"),
        "{text}"
    );
    assert!(result(&sink)["position_centered_counts"].is_null());
    ended_torque_off(&mut c, &v, &log);
}

/// Errors before the first enable's check has passed: lost rows on the
/// enable's burst, and the shaft walking off under it. Neither is followed
/// by a move: torque off where the shaft is.
#[test]
fn before_the_enable_check_passes_no_recovery_is_tried() {
    let (mut c, log) = servo(
        MID_POT,
        Box::new(|n, p, _| {
            if n == 1 {
                p.sim_mut().lose_tel_ticks(0, 500, 501);
            }
        }),
    );
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Error, "{:?}", v.ended.why);
    let text = said(&v);
    assert!(
        text.contains("not brought to the middle: the first enable's check had not passed"),
        "{text}"
    );
    let gp = addr("goal_position");
    let w = writes(&log);
    let on = w
        .iter()
        .position(|(a, b, _)| *a == addr("torque_enable") && b == &[1])
        .expect("torque on");
    assert!(!w[on..].iter().any(|(a, _, _)| *a == gp), "{w:?}");
    ended_torque_off(&mut c, &v, &log);

    // the shaft walks off under the first enable
    let (mut c, log) = servo(MID_POT, Box::new(|_, _, _| {}));
    let lin = c.pipe_mut().lin.clone().expect("lin");
    let pot = c.pipe_mut().pot(lin.counts(MID_POT) + 300.0);
    c.pipe_mut().hook = Box::new(move |n, p, k| {
        if n == 1 {
            k.frozen = true;
            let walk: Vec<TelSample> = (0..8000)
                .map(|j| sample(if j < 400 { MID_POT } else { pot }, 1206, 0, j))
                .collect();
            p.set_track(0, walk);
        }
    });
    let mut sink = MemSink::default();
    let v = run(&mut c, &AtomicBool::new(false), &mut sink).expect("starts");
    assert_eq!(v.ended.way, Way::Refused, "{:?}", v.ended.why);
    let why = v.ended.why.clone().unwrap_or_default();
    assert!(why.contains("the loop drove the shaft on its own"), "{why}");
    assert!(said(&v).contains("not brought to the middle"));
    ended_torque_off(&mut c, &v, &log);
}
