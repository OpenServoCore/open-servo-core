//! Step 5, free spins and their coasts between the stops: the back-EMF
//! constant Ke from each coast, the inertia J and the coast's drag from the
//! run-ups of the highest level that ran both ways, and the open-loop friction line of steady current
//! against steady speed (plan sec 2.7 and 2.9, merged).
//!
//! On the stops step's result, the same run's: its stops, its seated R
//! and L; and the probe's proven sign and the approach duty the stops step
//! drives at. The window term g comes from `results/window.json` when the
//! run holds one, else it is 0 and a warning says so.
//!
//! Every spin starts from rest at a guard, reached by a polled go-to at
//! the approach duty ([`runway::GoTo`]), drives open loop at a share of
//! the stated voltage for the window [`runway::Planner`] sizes, then
//! coasts, one committed drive of [`drive::drive`] with a coast end: the
//! short drive dead-man written, the duty and the TEL arm applied by one
//! COMMIT, the drive's rows read to LAST, then zero duty and the coast's
//! arm by a second COMMIT, [`runway::COAST_S`] of coast rows, then the
//! step's dead-man back. A streaming burst holds the
//! dead-man (kernel sec 2.7), so a host that stalls after
//! the drive's rows holds the duty [`DRIVE_DEADMAN_S`] at most, and every
//! spin is priced through the envelope at its time plus that. A pilot at
//! the lowest level sizes the rest; after every spin the shaft must rest
//! inside both guards, no further than its priced travel, or nothing more
//! drives.
//!
//! The fits read the raw row, the recorded ladder's: Ke per coast through
//! [`coast::coast_ke`] and its gate, the speed from the ripple tach and
//! never the pot, J through [`runup::read_run`] and [`runup::inertia`].
//! Ke pools the first two spins each way a level over one tap zero, the
//! mean of both directions' coast tails ([`tap_zero`]); the top level is
//! pooled only when the line through every level's mean holds
//! [`KE_TREND_MAX`] (plan sec 2.7's trap), else it is left out and said.
//! The pot sizes the runway only. The way out leaves the shaft at the
//! middle of the stops.

use std::time::Duration;

use osc_client::pipe::Pipe;
use serde_json::{Map, Value, json};

use super::drive::{self, Deadmen, Drive, End};
use super::polarity::HANDS;
use super::probe::FromRest;
use super::runway::{
    self, COAST_S, GUARD_COUNTS, Go, GoTo, LEVELS, PILOT_LEVEL, PILOT_MIN_S, Planner, SHORT_COUNTS,
    Settle, SpinRead, Travel, Window,
};
use super::seek::{self, POLL_S, WINDOW_S};
use super::stops::{self, FromProbe, POLL_DEADMAN_S};
use super::{
    Check, Ended, Halt, Rig, Scatter, Taken, Verdict, estimate, interrupted, keep_all, read_u16,
    rest, write,
};
use crate::Board;
use crate::capture::meta;
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{self, Line, StreamCfg};
use crate::envelope::{DRIVE_DEADMAN_S, Envelope, Facts, HeatModel, Kind, Resistance, Rung};
use crate::fit::coast::{self, CoastKe, KePooled};
use crate::fit::runup::{self, Inertia, RowScales, RunUp};
use crate::fit::{mean, median, polyfit};
use crate::guard::{self, Ledger, Way};
use crate::inputs::ThermalInputs;
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::ADC_COUNTS;
use crate::ripple;
use crate::state::{self, State, field};

pub const STEP: &str = "spins";
pub const RESULT: &str = "results/spins.json";
/// The window step's result, read when the run holds one.
pub const WINDOW_RESULT: &str = "results/window.json";
const METHOD: &str = "free-spins-and-coasts";
const REVISION: u32 = 3;
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "the probe's pricing R (planning grade)";

/// The top level is pooled into Ke when the line through every level's
/// mean moves under this share (plan sec 2.7).
pub const KE_TREND_MAX: f64 = 0.02;
/// The steady stretch: the held duty's last 100 ms, half its rows at
/// most, 20 ms at least (the ladder notebook's `spin_window`).
pub const STEADY_S: f64 = 0.100;
pub const STEADY_MIN_S: f64 = 0.020;
/// The levels the open-loop friction line needs.
pub const LINE_LEVELS: usize = 2;
/// The levels Ke is pooled over at least.
pub const KE_LEVELS: usize = 2;
/// A coast's tail, its last stretch with the rotor stopped, where the
/// coasts' tap zero is read.
pub const TAIL_S: f64 = 0.100;
/// A level and direction whose pot counts a ripple revolution lie further
/// than this share from the run's median is left out of the line: its
/// tach or its pot read another speed than the rest.
pub const COUNTS_PER_REV_SHARE: f64 = 0.15;

/// What the step takes from the stops step's result.
#[derive(Clone, Debug, PartialEq)]
pub struct FromStops {
    pub lo: f64,
    pub hi: f64,
    pub r_ohm: f64,
    pub l_h: f64,
}

impl FromStops {
    pub fn of(v: &Value) -> Result<Self, Refused> {
        if v["verdict"] != "accepted" {
            return Err(Refused(format!(
                "the spin step runs on an accepted stops step; the stops step was {}",
                v["verdict"].as_str().unwrap_or("not run")
            )));
        }
        let n = |x: &Value, what: &str| {
            x.as_f64()
                .filter(|v| v.is_finite())
                .ok_or_else(|| Refused(format!("the stops result holds no {what}")))
        };
        let out = FromStops {
            lo: n(&v["stops"]["low"]["median_counts"], "low stop")?,
            hi: n(&v["stops"]["high"]["median_counts"], "high stop")?,
            r_ohm: n(&v["estimates"]["r_seated_ohm"]["value"], "seated R")?,
            l_h: n(&v["estimates"]["l_seated_h"]["value"], "seated L")?,
        };
        if out.hi - out.lo <= 2.0 * (GUARD_COUNTS + SHORT_COUNTS) {
            return Err(Refused(format!(
                "the stops {:.0}..{:.0} leave no runway inside guards {GUARD_COUNTS} counts in and {SHORT_COUNTS} counts short: run the stops step again",
                out.lo, out.hi
            )));
        }
        Ok(out)
    }
}

/// The window term from the window step's result, S, and its words; None
/// when the run holds no window step.
pub fn window_term(v: Option<&Value>) -> Result<Option<(f64, String)>, Refused> {
    let Some(v) = v else { return Ok(None) };
    if v["verdict"] != "accepted" {
        return Err(Refused(format!(
            "the run's window step was {}: run it again before the spins",
            v["verdict"].as_str().unwrap_or("not run")
        )));
    }
    let g = v["estimates"]["g_window_ms"]["value"]
        .as_f64()
        .filter(|g| g.is_finite() && *g >= 0.0)
        .ok_or_else(|| Refused("the window result holds no g_window_ms".into()))?;
    Ok(Some((g * 1e-3, format!("the window step's g, {g:.3} mS"))))
}

/// What the step reads besides the servo.
pub struct SpinsInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The same run's `results/rest.json`, `probe.json`, `stops.json`, and
    /// `window.json` when it holds one.
    pub rest: &'a Value,
    pub probe: &'a Value,
    pub stops: &'a Value,
    pub window: Option<&'a Value>,
}

#[derive(Clone, Debug)]
pub struct Spun {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Spun {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// The Q15 of `level` of the stated voltage on `rail_v`, rounded down.
pub fn level_q15(level: f64, stated_v: f64, rail_v: f64) -> i16 {
    (level * stated_v / rail_v * Q15)
        .floor()
        .clamp(0.0, i16::MAX as f64) as i16
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    stream: StreamCfg,
    pos: f64,
    rail_v: f64,
    tick_hz: f64,
    deadman_ticks: u16,
    poll_deadman_ticks: u16,
    drive_deadman_ticks: u16,
    stored: bool,
    probe: FromProbe,
    rest: FromRest,
    stops: FromStops,
    travel: Travel,
    approach_q15: i16,
    top_q15: i16,
    still_counts: f64,
    env: Envelope,
    stated_v: f64,
    rated_cps: f64,
    per_rev: u32,
    window: Option<(f64, String)>,
    soft: [Vec<u8>; 2],
    soft_counts: (f64, f64),
    pwm_arr: f64,
    counts_per_a: f64,
    counts_per_v: f64,
}

fn num(state: &State, d: &osc_client::descriptor::Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

/// A go-to's window as the envelope prices it: the approach duty for a
/// window and the poll dead-man, the stops step's own pricing, the light
/// press its blocked current allows at a stop.
fn goto_rung(b: &Before, q15: i16, pos: f64) -> Rung {
    Rung {
        name: "a go-to window".into(),
        duty: q15 as f64 / Q15,
        time_s: WINDOW_S + POLL_DEADMAN_S,
        permit: false,
        gap_s: 0.0,
        start: pos,
        speed_cps: (q15.unsigned_abs() as f64 / Q15 * b.rail_v) / b.stated_v * b.rated_cps,
        stop_counts: 0.0,
        kind: Kind::Drive,
    }
}

/// The envelope's price of a go-to window, the stops left out as the stops
/// step prices its approaches, so a drive at the approach duty may arrive
/// at a stop.
fn price_goto(b: &mut Before, q15: i16, pos: f64, admit: bool) -> Result<(), Refused> {
    let r = goto_rung(b, q15, pos);
    let stops = b.env.facts.stops.take();
    let out = if admit {
        b.env.admit(&r)
    } else {
        b.env.price(&r)
    };
    b.env.facts.stops = stops;
    out.map(|_| ())
}

/// A spin as the envelope prices it: free at its duty for its drive and
/// the dead-man, at the speed its bound is the free margin over, then its
/// coast's bound; the coast's rows after it.
fn spin_rung(name: &str, q15: i16, w: &Window) -> Rung {
    Rung {
        name: name.into(),
        duty: q15 as f64 / Q15,
        time_s: w.t_s + DRIVE_DEADMAN_S,
        permit: false,
        gap_s: COAST_S,
        start: w.start,
        speed_cps: w.bound_speed / runway::SPEED_GAIN,
        stop_counts: w.bound_coast,
        kind: Kind::Drive,
    }
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &SpinsInputs<'_>) -> Result<Before, Refused> {
    let np = i.nameplate;
    np.check_user().map_err(Refused)?;
    let thermal = i.thermal.ok_or_else(|| {
        Refused("no thermal values entered (--thermal): the heat of a spin cannot be priced, so nothing drives".into())
    })?;
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Raw)?;
    let rest = FromRest::of(i.rest)?;
    let probe = FromProbe::of(i.probe)?;
    let stops = FromStops::of(i.stops)?;
    let window = window_term(i.window)?;
    let state = state::read(rig.c, rig.id, rig.d).await?;
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this step starts only with torque off, the shaft at rest".into(),
        ));
    }
    let faults = {
        let f = field(rig.d, "fault_flags")?;
        rig.c
            .read(rig.id, f.addr, 1)
            .await?
            .first()
            .copied()
            .unwrap_or(0)
    };
    if faults != 0 {
        return Err(Refused(format!(
            "a fault is latched (fault_flags {faults:#04x}): `osc status` names it"
        )));
    }
    let stored = state
        .bytes(field(rig.d, "drive_polarity")?)
        .and_then(|b| b.first().copied())
        .ok_or_else(|| Refused("the state read holds no drive_polarity".into()))?
        != 0;
    if stored != probe.stored {
        return Err(Refused(format!(
            "the servo's stored drive_polarity is {stored}, and the probe ran on {}: something changed it since; run the probe again",
            probe.stored
        )));
    }
    let speed = np
        .speed_row()
        .ok_or_else(|| Refused("the nameplate has no usable no_load".into()))?;
    let travel_deg = np
        .travel_deg
        .ok_or_else(|| Refused("the nameplate has no usable travel_deg".into()))?;
    let per_rev = np.events_per_rev().map_err(Refused)?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos = rest::pos_now(rig.c, rig.id, rig.d).await?;
    let rail_v = rest::rail_now(rig.c, rig.id, rig.d, &sense).await?;
    let pwm_arr = num(&state, rig.d, "pwm_period_ticks")?;
    let ntc_raw = {
        let f = field(rig.d, "ntc_raw")?;
        let b = rig.c.read(rig.id, f.addr, 2).await?;
        u16::from_le_bytes([b[0], b[1]]) as f64
    };
    let stated_v = speed.at_v;
    let top = LEVELS[LEVELS.len() - 1];
    if top * stated_v >= rail_v {
        return Err(Refused(format!(
            "the {:.2} V rail cannot apply {:.0}% of the stated {stated_v} V ({:.2} V): raise the rail",
            rail_v,
            100.0 * top,
            top * stated_v
        )));
    }
    let travel = Travel {
        lo: stops.lo,
        hi: stops.hi,
    };
    if !(stops.lo < pos && pos < stops.hi) {
        return Err(Refused(format!(
            "the shaft reads {pos:.0}, not between the stops {:.0}..{:.0}: {HANDS}",
            stops.lo, stops.hi
        )));
    }
    let still_counts = seek::still_band(rest.pot_sd);
    let cfg = stops::duties(&probe, rail_v, stated_v, still_counts);
    let top_q15 = level_q15(top, stated_v, rail_v);
    let mut facts = Facts::new(np, rail_v);
    facts.ntc_c = Some(sense.ntc.c_of_raw(ntc_raw));
    facts.polarity_proven = true;
    facts.stops = Some((stops.lo, stops.hi));
    facts.r = Some(Resistance {
        ohm: probe.r_price_ohm,
        source: PRICE_WORDS.into(),
    });
    let heat = HeatModel::new(
        i.board,
        thermal,
        probe.r_price_ohm,
        sense.counts_per_a(),
        sense.tick_hz,
    );
    let ticks = |s: f64| {
        let t = (s * sense.tick_hz).round();
        if (1.0..=u16::MAX as f64).contains(&t) {
            Ok(t as u16)
        } else {
            Err(Refused(format!(
                "a {s} s dead-man is no u16 of ticks at {} Hz",
                sense.tick_hz
            )))
        }
    };
    let soft = [
        state
            .bytes(field(rig.d, "pos_min_phys_counts")?)
            .map(<[u8]>::to_vec),
        state
            .bytes(field(rig.d, "pos_max_phys_counts")?)
            .map(<[u8]>::to_vec),
    ];
    let [Some(lo), Some(hi)] = soft else {
        return Err(Refused("the state read holds no physical range".into()));
    };
    let word = |v: &[u8]| i32::from_le_bytes(v[..4].try_into().unwrap_or([0; 4])) as f64;
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    let mut b = Before {
        line,
        servo,
        stream,
        pos,
        rail_v,
        tick_hz: sense.tick_hz,
        deadman_ticks: ticks(rest::DEADMAN_S)?,
        poll_deadman_ticks: ticks(POLL_DEADMAN_S)?,
        drive_deadman_ticks: ticks(DRIVE_DEADMAN_S)?,
        stored,
        probe,
        rest,
        stops,
        travel,
        approach_q15: cfg.approach_q15,
        top_q15,
        still_counts,
        env: Envelope::new(facts, heat),
        stated_v,
        rated_cps: speed.per_deg / speed.seconds * ADC_COUNTS / travel_deg,
        per_rev,
        window,
        soft_counts: (word(&lo), word(&hi)),
        soft: [lo, hi],
        pwm_arr,
        counts_per_a: sense.counts_per_a(),
        counts_per_v: sense.terminal_counts_per_v(),
    };
    if b.approach_q15 > b.top_q15 {
        return Err(Refused(format!(
            "the approach duty {:.3} V is over the spins' top {:.3} V: run the probe again",
            b.approach_q15 as f64 / Q15 * rail_v,
            b.top_q15 as f64 / Q15 * rail_v
        )));
    }
    // the first go-to window and the pilot priced before any write
    let q = b.approach_q15;
    price_goto(&mut b, q, pos, false)?;
    let plan = Planner::new(b.travel, b.rated_cps);
    let w = plan.pilot(b.travel.start(1));
    if w.t_s < PILOT_MIN_S {
        return Err(Refused(format!(
            "the runway leaves the pilot {:.0} ms, under {:.0} ms ({}): the stops are too close for the nameplate's speed",
            1e3 * w.t_s,
            1e3 * PILOT_MIN_S,
            w.said()
        )));
    }
    let q = level_q15(PILOT_LEVEL, stated_v, rail_v);
    b.env.price(&spin_rung("the pilot", q, &w))?;
    Ok(b)
}

/// One spin taken: its plan, its read and where its bursts are in the
/// step's captures.
#[derive(Clone, Debug)]
pub struct Taken1 {
    pub n: usize,
    pub pilot: bool,
    pub window: Window,
    pub q15: i16,
    pub read: SpinRead,
    /// From the duty's COMMIT to the coast's cut by the servo's stamps, s.
    pub on_s: Option<f64>,
    pub attempt: usize,
    pub drive_k: usize,
    pub coast_k: usize,
}

/// What the body found, as far as it got.
#[derive(Default)]
struct Found {
    corrected: bool,
    spins: Vec<Taken1>,
    /// (level, logical sign, why).
    left_out: Vec<(f64, i8, String)>,
    path: Vec<String>,
    t90_assumed: bool,
}

/// The duty and the dead-man as the host last wrote them.
#[derive(Default)]
struct OnServo {
    duty: Option<i16>,
    driving: bool,
}

/// A duty written, never over the spins' top level.
async fn duty<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, q15: i16) -> Result<(), Halt> {
    if q15.unsigned_abs() > b.top_q15 as u16 {
        return Err(Halt::Error(format!(
            "a duty of {q15} was asked for, over the top level's {}: nothing more drives",
            b.top_q15
        )));
    }
    write(rig, "goal_duty", &q15.to_le_bytes()).await
}

async fn zero<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, on: &mut OnServo) -> Result<(), Halt> {
    if on.driving || on.duty.is_some_and(|q| q != 0) {
        write(rig, "goal_duty", &[0, 0]).await?;
        write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    }
    *on = OnServo::default();
    Ok(())
}

/// The go-to on the servo: each act of [`GoTo`] written and polled under
/// the poll dead-man, every window priced before it starts; the rest
/// position it arrived at.
async fn go_to<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    target: f64,
    name: &str,
    lines: &mut Vec<String>,
) -> Result<f64, Halt> {
    let mut g = GoTo::new(target, b.approach_q15, b.still_counts);
    let mut on = OnServo::default();
    let t0 = (rig.clock)();
    let from = read_u16(rig, "pos").await? as f64;
    let mut pos = from;
    let mut act = g.start(t0, from);
    loop {
        match act {
            Go::Drive { q15, price } => {
                interrupted(rig.stop)?;
                if price.is_some() {
                    price_goto(b, q15, pos, true)?;
                }
                if !on.driving {
                    write(
                        rig,
                        "bus_deadman_ticks",
                        &b.poll_deadman_ticks.to_le_bytes(),
                    )
                    .await?;
                    on.driving = true;
                }
                if on.duty != Some(q15) {
                    duty(rig, b, q15).await?;
                    on.duty = Some(q15);
                }
            }
            Go::Zero => zero(rig, b, &mut on).await?,
            Go::Arrived(at) => {
                zero(rig, b, &mut on).await?;
                lines.push(format!(
                    "to {name} {target:.0}: from {from:.0}, at rest at {at:.0} in {:.1} s",
                    (rig.clock)() - t0
                ));
                return Ok(at);
            }
            Go::Stopped(why) => {
                zero(rig, b, &mut on).await?;
                return Err(Halt::Refused(why));
            }
        }
        rig.c.pause(Duration::from_secs_f64(POLL_S)).await;
        pos = stops::poll(rig).await?;
        act = g.feed((rig.clock)(), pos);
    }
}

/// The shaft at rest after a coast, polled at zero duty.
async fn at_rest<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before) -> Result<f64, Halt> {
    let pos = stops::poll(rig).await?;
    let mut s = Settle::new((rig.clock)(), pos, b.still_counts);
    loop {
        rig.c.pause(Duration::from_secs_f64(POLL_S)).await;
        let pos = stops::poll(rig).await?;
        if let Some(r) = s.feed((rig.clock)(), pos) {
            return r.map_err(Halt::Refused);
        }
    }
}

fn rows_of(s: f64, tick_hz: f64) -> u16 {
    (s * tick_hz).ceil().clamp(1.0, u16::MAX as f64) as u16
}

/// One spin from rest at `w.start`: priced, driven, coasted and read, the
/// checks after it applied. None when a burst lost rows: no evidence, and
/// the caller takes it again.
#[allow(clippy::too_many_arguments)]
async fn spin<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    w: Window,
    n: usize,
    attempt: usize,
    lines: &mut Vec<String>,
) -> Result<Option<Taken1>, Halt> {
    let what = if n == 0 {
        "the pilot".to_string()
    } else {
        format!("spin {n}")
    };
    interrupted(rig.stop)?;
    let q15 = level_q15(w.level, b.stated_v, b.rail_v) * w.dir as i16;
    if q15.unsigned_abs() > b.top_q15 as u16 {
        return Err(Halt::Error(format!(
            "{what} asks a duty of {q15}, over the top level's {}: nothing more drives",
            b.top_q15
        )));
    }
    b.env.admit(&spin_rung(&what, q15, &w))?;
    let drive_rows = rows_of(w.t_s, b.tick_hz);
    let coast_rows = rows_of(COAST_S, b.tick_hz);
    let plan = json!({"spin": what, "parts": ["drive", "coast"], "level_share": w.level, "dir": w.dir,
                      "duty_q15": q15, "rows": drive_rows, "coast_rows": coast_rows, "t_s": w.t_s,
                      "start_counts": w.start, "bound_speed_cps": w.bound_speed,
                      "bound_coast_counts": w.bound_coast, "attempt": attempt});
    let drive_k = taken.len();
    let got = drive::drive(
        rig,
        &b.stream,
        &Deadmen {
            step_ticks: b.deadman_ticks,
            drive_ticks: b.drive_deadman_ticks,
        },
        &Drive {
            layout: Layout::Raw,
            duty_q15: q15,
            rows: drive_rows,
            end: End::Coast(coast_rows),
            plan: &plan,
        },
        taken,
        &b.servo,
    )
    .await?;
    let lost = got.lost();
    let coast_rows_got = got.coast.as_ref().map_or(&[][..], |c| &c.rows[..]);
    let cut = coast::cut_row(coast_rows_got).unwrap_or(0);
    let stamps = got
        .drive
        .first_stamp
        .zip(got.coast.as_ref().and_then(|c| c.first_stamp));
    let on_s = stamps.map(|(d, c)| (c.wrapping_sub(d) as f64 + cut as f64) / b.tick_hz);
    if let Some(s) = on_s.filter(|s| *s > w.t_s + DRIVE_DEADMAN_S + 1.0 / b.tick_hz) {
        return Err(Halt::Error(format!(
            "{what} drove {s:.3} s by the servo's stamps, over the {:.3} s it was priced at: the host is too slow between the bursts",
            w.t_s + DRIVE_DEADMAN_S
        )));
    }
    let end = at_rest(rig, b).await?;
    if !lost.is_empty() {
        if !b.travel.inside_guards(end) {
            let (lo, hi) = b.travel.guards();
            return Err(Halt::Refused(format!(
                "{what} ended at {end:.0}, outside the guards {lo:.0}..{hi:.0}, against a bound of {:.0} counts from {:.0}; its rows lost ({}). The nameplate's speed is too low for this servo; nothing more drives",
                w.travel_bound(),
                w.start,
                lost.join(", ")
            )));
        }
        lines.push(format!(
            "{what}, attempt {attempt}: lost rows ({}), no evidence (captures {:02} and {:02})",
            lost.join(", "),
            drive_k + 1,
            drive_k + 2
        ));
        return Ok(None);
    }
    let (d, c) = (&taken[drive_k].burst.rows, &taken[drive_k + 1].burst.rows);
    let read = runway::read_spin(w.dir, w.start, d, c, end, b.still_counts, b.tick_hz)
        .ok_or_else(|| Halt::Error(format!("{what}: its bursts carry too few rows to read")))?;
    lines.push(format!(
        "{what}: {:.0}% {} from {:.0} for {:.0} ms: {:.0} of {:.0} counts/s, coast {:.0} of {:.0}, at rest at {:.0} ({:.0} of {:.0} counts), bounds from {}",
        100.0 * w.level,
        way(w.dir),
        w.start,
        1e3 * w.t_s,
        read.speed_cps,
        w.bound_speed,
        read.coast_counts,
        w.bound_coast,
        end,
        read.travel,
        w.travel_bound(),
        w.from.said()
    ));
    runway::after(&w, &read, &b.travel, &what).map_err(Halt::Refused)?;
    Ok(Some(Taken1 {
        n,
        pilot: n == 0,
        window: w,
        q15,
        read,
        on_s,
        attempt,
        drive_k,
        coast_k: drive_k + 1,
    }))
}

/// A spin from its guard, taken again once when a burst lost rows.
#[allow(clippy::too_many_arguments)]
async fn spin_from<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    plan: &Planner,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    level: f64,
    dir: i8,
    n: usize,
    lines: &mut Vec<String>,
) -> Result<Option<Taken1>, Halt> {
    let what = if n == 0 {
        "the pilot".to_string()
    } else {
        format!("spin {n}")
    };
    for attempt in 1..=2 {
        let guard = b.travel.start(dir);
        let name = if dir > 0 {
            "the low guard"
        } else {
            "the high guard"
        };
        let start = go_to(rig, b, guard, name, lines).await?;
        let w = if n == 0 {
            plan.pilot(start)
        } else {
            plan.window(level, dir, start)
        };
        let fits = if n == 0 {
            w.t_s >= PILOT_MIN_S
        } else {
            w.fits()
        };
        if !fits {
            let why = format!("{what} from {start:.0}: {}", w.said());
            found.path.push(format!("left out: {why}"));
            if n == 0 {
                return Err(Halt::Refused(format!(
                    "the runway leaves the pilot under {:.0} ms ({why}): the stops are too close for the nameplate's speed",
                    1e3 * PILOT_MIN_S
                )));
            }
            found.left_out.push((level, dir, why));
            return Ok(None);
        }
        found
            .path
            .push(format!("{what} at {:.0}%: {}", 100.0 * level, w.said()));
        if let Some(t) = spin(rig, b, taken, w, n, attempt, lines).await? {
            return Ok(Some(t));
        }
        interrupted(rig.stop)?;
    }
    Err(Halt::Error(format!(
        "{what}: two attempts lost rows: the line is not clean enough; check the cable and its contacts"
    )))
}

fn way(dir: i8) -> &'static str {
    if dir > 0 { "up" } else { "down" }
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    found.corrected =
        stops::open_loop(rig, ledger, &b.probe, &b.soft, b.deadman_ticks, lines).await?;
    let (glo, ghi) = b.travel.guards();
    let v = |q: i16| q as f64 / Q15 * b.rail_v;
    lines.push(format!(
        "volatile: the peak cap unset, the soft limits out to the physical range {:.0}..{:.0}, OpenLoop, dead-man {} ticks ({} s), torque on at zero duty; guards {glo:.0}..{ghi:.0}, {GUARD_COUNTS} in from the stops {:.0}..{:.0}; go-to at {:.3} V, spins at {} of {} V, no duty over {:.3} V; each drive under a {DRIVE_DEADMAN_S} s dead-man, {COAST_S} s of coast",
        b.soft_counts.0,
        b.soft_counts.1,
        b.deadman_ticks,
        rest::DEADMAN_S,
        b.stops.lo,
        b.stops.hi,
        v(b.approach_q15),
        LEVELS
            .iter()
            .map(|l| format!("{:.0}%", 100.0 * l))
            .collect::<Vec<_>>()
            .join(", "),
        b.stated_v,
        v(b.top_q15)
    ));
    let mut plan = Planner::new(b.travel, b.rated_cps);
    let pilot = spin_from(rig, b, &plan, taken, found, PILOT_LEVEL, 1, 0, lines)
        .await?
        .ok_or_else(|| Halt::Refused("the pilot was left out".into()))?;
    let t90 = match pilot.read.t90_s {
        Some(t) => t,
        None => {
            found.t90_assumed = true;
            found.path.push(format!(
                "the pilot's pot never read 90 % of its steady speed in a 5 ms window: its rise taken as its whole drive, {:.0} ms",
                1e3 * pilot.window.t_s
            ));
            pilot.window.t_s
        }
    };
    plan.feed_pilot(pilot.read.seen(PILOT_LEVEL, 1), t90);
    found.spins.push(pilot);
    for slot in runway::order() {
        if found
            .left_out
            .iter()
            .any(|(l, d, _)| *l == slot.level && *d == slot.dir)
        {
            continue;
        }
        let w = plan.window(slot.level, slot.dir, b.travel.start(slot.dir));
        if !w.fits() {
            let why = format!("spin {} from its guard: {}", slot.n, w.said());
            found.path.push(format!("left out: {why}"));
            lines.push(format!(
                "the {:.0}% level is left out {}: {why}",
                100.0 * slot.level,
                way(slot.dir)
            ));
            found.left_out.push((slot.level, slot.dir, why));
            continue;
        }
        let got = spin_from(
            rig, b, &plan, taken, found, slot.level, slot.dir, slot.n, lines,
        )
        .await?;
        match got {
            Some(t) => {
                plan.feed(t.read.seen(slot.level, slot.dir));
                found.spins.push(t);
            }
            None => lines.push(format!(
                "the {:.0}% level is left out {}: {}",
                100.0 * slot.level,
                way(slot.dir),
                found.left_out.last().map_or("", |(_, _, w)| w.as_str())
            )),
        }
    }
    go_to(rig, b, b.travel.middle(), "the middle", lines).await?;
    Ok(())
}

/// The spins the judge reads: those of [`runway::order`], not the pilot.
#[derive(Copy, Clone, Debug)]
pub struct Spin<'a> {
    pub n: usize,
    pub level: f64,
    /// The logical sign.
    pub dir: i8,
    pub drive: &'a [TelFrame],
    pub coast: &'a [TelFrame],
}

/// What the judge reads the spins through.
#[derive(Clone, Debug, PartialEq)]
pub struct JudgeCfg {
    pub fs: f64,
    /// The sign the bridge drives under: a logical + puts + on the taps.
    pub polarity: bool,
    pub zero_i: f64,
    pub zero_e: f64,
    pub row: RowScales,
    /// Where g comes from; None: not measured, taken 0.
    pub window: Option<String>,
    pub r_ohm: f64,
    pub l_h: f64,
    /// The ripple's events a motor revolution, the nameplate's brushes
    /// times rotor segments.
    pub per_rev: u32,
    /// The drag at rest the order count starts from, A; None: the
    /// open-loop line's intercept.
    pub i_drag0_a: Option<f64>,
}

impl JudgeCfg {
    fn bridge(&self, dir: i8) -> i8 {
        if self.polarity { dir } else { -dir }
    }

    fn logical(&self, bridge: i8) -> i8 {
        self.bridge(bridge)
    }
}

/// A spin's steady stretch.
#[derive(Clone, Debug, PartialEq)]
pub struct Steady {
    pub n: usize,
    pub level: f64,
    pub dir: i8,
    pub rows: usize,
    /// The ripple's line over it, Hz, and the speed through the events a
    /// revolution, rev/s.
    pub line_hz: Option<f64>,
    pub rev_s: Option<f64>,
    /// The pot's slope over it, counts/s.
    pub pot_cps: Option<f64>,
    /// The mean crest over the rest zero, and the winding current, the
    /// window term taken off, mA.
    pub crest_ma: Option<f64>,
    pub i_ma: Option<f64>,
    /// Hz, where the line was looked for: the pot's speed over the lower
    /// levels' counts a ripple cycle; None for a free search.
    pub seed_hz: Option<f64>,
}

impl Steady {
    /// Pot counts a ripple cycle.
    pub fn counts_per_cycle(&self) -> Option<f64> {
        self.pot_cps
            .zip(self.line_hz)
            .map(|(c, h)| c / h)
            .filter(|v| v.is_finite() && *v > 0.0)
    }

    pub fn rad_s(&self) -> Option<f64> {
        self.rev_s.map(|r| 2.0 * core::f64::consts::PI * r)
    }
}

/// The ripple's events a revolution.
#[derive(Clone, Debug, PartialEq)]
pub struct PerRev {
    pub value: usize,
    pub source: String,
}

/// The open-loop friction line: steady current against steady speed over
/// each level and direction's mean.
#[derive(Clone, Debug, PartialEq)]
pub struct OpenLine {
    pub coulomb_ma: f64,
    pub viscous_ma_s_per_rad: f64,
    pub points: Vec<LinePoint>,
}

/// A level and direction's steady mean.
#[derive(Clone, Debug, PartialEq)]
pub struct LinePoint {
    pub level: f64,
    pub dir: i8,
    pub rad_s: f64,
    /// mA, corrected and raw.
    pub i_ma: f64,
    pub raw_ma: f64,
    pub spins: usize,
    /// Pot counts a motor revolution: its pot speed over its ripple speed.
    pub counts_per_rev: f64,
}

/// The tap zero the coasts' EMF is read over, counts of raw tap
/// difference.
#[derive(Clone, Debug, PartialEq)]
pub struct TapZero {
    pub counts: f64,
    /// The median of the up and of the down coasts' tails, and how many
    /// coasts held a tail.
    pub up: Option<(f64, usize)>,
    pub down: Option<(f64, usize)>,
    /// From both directions' tails; false: the rest step's zero.
    pub from_tails: bool,
}

impl TapZero {
    /// The zero [`coast::coast_ke`] takes, which it subtracts after signing
    /// the tap difference by `bridge`. The tails' zero is a raw offset, the
    /// same in both directions, so it is signed with the taps; the rest
    /// step's keeps the record's rule, subtracted after the sign.
    pub fn signed(&self, bridge: i8) -> f64 {
        if self.from_tails {
            f64::from(bridge) * self.counts
        } else {
            self.counts
        }
    }

    pub fn said(&self) -> String {
        let tail = |t: Option<(f64, usize)>| {
            t.map_or("no tail".into(), |(m, n)| format!("{m:+.2} over {n}"))
        };
        if self.from_tails {
            format!(
                "{:+.2} counts, the mean of the up coasts' tail median ({}) and the down coasts' ({})",
                self.counts,
                tail(self.up),
                tail(self.down)
            )
        } else {
            format!(
                "the rest step's {:+.2} counts: a direction has no coast at rest over its last {:.0} ms (up {}, down {})",
                self.counts,
                1e3 * TAIL_S,
                tail(self.up),
                tail(self.down)
            )
        }
    }
}

/// The top level's Ke compared with the others before it is pooled.
#[derive(Clone, Debug, PartialEq)]
pub struct TopCompare {
    pub level: f64,
    /// Its mean over the mean of the lower levels' means, less 1.
    pub vs_lower: f64,
    /// The line through every level's mean over their span, a share of
    /// the mean over every level.
    pub trend: f64,
    pub levels: usize,
    pub pooled: bool,
}

#[derive(Clone, Debug)]
pub struct Judged {
    pub per_rev: PerRev,
    pub steady: Vec<Steady>,
    /// Every spin's coast, Ke's or not.
    pub coasts: Vec<(usize, Result<CoastKe, String>)>,
    pub zero: TapZero,
    /// The spins whose coasts are Ke's.
    pub ke_spins: Vec<usize>,
    pub top: Option<TopCompare>,
    /// The levels pooled, each with its mean Ke, and the line through
    /// them over their span as a share of the pooled Ke.
    pub ke_levels: Vec<(f64, f64)>,
    pub trend: Option<f64>,
    pub ke: Option<KePooled>,
    pub line: Option<OpenLine>,
    pub i_drag0_a: Option<f64>,
    /// The level J is read at.
    pub j_level: Option<f64>,
    pub runups: Vec<(usize, i8, Result<RunUp, String>)>,
    pub inertia: Result<Inertia, String>,
    pub checks: Vec<Check>,
}

fn check(name: &str, value: String, bound: String, verdict: Verdict) -> Check {
    Check {
        name: name.into(),
        value,
        bound,
        verdict,
    }
}

/// The held duty's rows of a spin's steady stretch.
pub fn steady_rows(drive: &[TelFrame], fs: f64) -> Option<Vec<&TelFrame>> {
    let ticks = |f: &TelFrame| f.duty_counts.map(|d| f64::from(d.unsigned_abs()));
    let half: Option<Vec<f64>> = drive.get(drive.len() / 2..)?.iter().map(ticks).collect();
    let half = median(&half?)?;
    let held: Vec<&TelFrame> = drive.iter().filter(|f| ticks(f) == Some(half)).collect();
    let n = ((STEADY_S * fs) as usize)
        .min(held.len() / 2)
        .max((STEADY_MIN_S * fs) as usize)
        .min(held.len());
    (n > 0).then(|| held[held.len() - n..].to_vec())
}

/// One spin's steady stretch read.
/// One spin's steady stretch read. With `seed`, pot counts a ripple cycle
/// the lower levels read, the line is looked for within
/// [`ripple::BAND_FRAC`] of the pot's speed over it; else the tallest line
/// in [`ripple::BAND_HZ`], which at speed can be the half line (three
/// events a revolution of six) and read the motor at half its speed.
fn steady(s: &Spin, c: &JudgeCfg, seed: Option<f64>) -> Steady {
    let mut out = Steady {
        n: s.n,
        level: s.level,
        dir: s.dir,
        rows: 0,
        line_hz: None,
        rev_s: None,
        pot_cps: None,
        crest_ma: None,
        i_ma: None,
        seed_hz: None,
    };
    let Some(rows) = steady_rows(s.drive, c.fs) else {
        return out;
    };
    out.rows = rows.len();
    let bridge = c.bridge(s.dir);
    let read: Option<Vec<runup::Driven>> = rows
        .iter()
        .map(|f| runup::driven(f, bridge, c.zero_i, &c.row))
        .collect();
    if let Some(r) = read {
        out.crest_ma = mean(&r.iter().map(|x| x.crest_a).collect::<Vec<_>>()).map(|a| 1e3 * a);
        out.i_ma = mean(&r.iter().map(|x| x.i_a).collect::<Vec<_>>()).map(|a| 1e3 * a);
    }
    let pts: Option<Vec<(f64, f64)>> = rows
        .iter()
        .map(|f| f.pos.map(|p| (f.tick as f64 / c.fs, f64::from(p))))
        .collect();
    if let Some(p) = pts {
        let t0 = p.first().map_or(0.0, |x| x.0);
        let (t, y): (Vec<f64>, Vec<f64>) = p.iter().map(|(t, y)| (t - t0, *y)).unzip();
        out.pot_cps = polyfit::<2>(&t, &y).map(|[_, b]| b * s.dir as f64);
    }
    out.seed_hz = seed
        .zip(out.pot_cps)
        .map(|(cpc, cps)| cps / cpc)
        .filter(|f| f.is_finite() && *f > 0.0);
    let raw: Option<Vec<f64>> = rows.iter().map(|f| f.current_raw.map(f64::from)).collect();
    if let Some(raw) = raw {
        let m = mean(&raw).unwrap_or(0.0);
        let x: Vec<f64> = raw.iter().map(|v| v - m).collect();
        let (lo, hi) = match out.seed_hz {
            Some(f) => (f * (1.0 - ripple::BAND_FRAC), f * (1.0 + ripple::BAND_FRAC)),
            None => ripple::BAND_HZ,
        };
        out.line_hz = ripple::line_freq(&x, lo, hi, c.fs)
            .and_then(|f0| ripple::count_cycles(&x, f0, ripple::BAND_FRAC, c.fs))
            .map(|k| k.hz);
    }
    out
}

/// Every spin's steady stretch, the lowest level by a free search and each
/// level above seeded by the levels under it: their median pot counts a
/// ripple cycle, of its own direction where they ran.
fn steadies(spins: &[Spin], c: &JudgeCfg) -> Vec<Steady> {
    let mut levels: Vec<f64> = spins.iter().map(|s| s.level).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    let mut done: Vec<Steady> = Vec::new();
    for l in levels {
        let mut now = Vec::new();
        for s in spins.iter().filter(|s| s.level == l) {
            let under = |same: bool| -> Vec<f64> {
                done.iter()
                    .filter(|x| x.level < l && (!same || x.dir == s.dir))
                    .filter_map(Steady::counts_per_cycle)
                    .collect()
            };
            let mine = under(true);
            let seed = if mine.is_empty() {
                median(&under(false))
            } else {
                median(&mine)
            };
            now.push(steady(s, c, seed));
        }
        done.extend(now);
    }
    let order = |x: &Steady| spins.iter().position(|s| s.n == x.n && s.level == x.level);
    done.sort_by_key(|x| order(x));
    done
}

/// The events a revolution the spins are read in: the nameplate's.
pub fn per_rev(c: &JudgeCfg) -> PerRev {
    PerRev {
        value: c.per_rev as usize,
        source: format!(
            "the nameplate's brushes times rotor_segments, {}",
            c.per_rev
        ),
    }
}

/// The open-loop line through each level and direction's steady mean,
/// over [`LINE_LEVELS`] levels at least: one level's two directions are
/// two regions of the pot, not two speeds.
fn open_line(steady: &[Steady]) -> (Option<OpenLine>, Vec<(LinePoint, f64)>) {
    let mut keys: Vec<(f64, i8)> = steady.iter().map(|s| (s.level, s.dir)).collect();
    keys.sort_by(|a, b| a.0.total_cmp(&b.0).then(b.1.cmp(&a.1)));
    keys.dedup();
    let mut points = Vec::new();
    for (level, dir) in keys {
        let g: Vec<&Steady> = steady
            .iter()
            .filter(|s| s.level == level && s.dir == dir)
            .filter(|s| {
                s.rad_s().is_some()
                    && s.i_ma.is_some()
                    && s.crest_ma.is_some()
                    && s.pot_cps.is_some()
            })
            .collect();
        let m = |f: &dyn Fn(&Steady) -> Option<f64>| {
            mean(&g.iter().filter_map(|s| f(s)).collect::<Vec<_>>())
        };
        if let (Some(w), Some(i), Some(raw), Some(cps), Some(rev)) = (
            m(&Steady::rad_s),
            m(&|s| s.i_ma),
            m(&|s| s.crest_ma),
            m(&|s| s.pot_cps),
            m(&|s| s.rev_s),
        ) {
            points.push(LinePoint {
                level,
                dir,
                rad_s: w,
                i_ma: i,
                raw_ma: raw,
                spins: g.len(),
                counts_per_rev: cps / rev,
            });
        }
    }
    let med = median(&points.iter().map(|p| p.counts_per_rev).collect::<Vec<_>>());
    let (kept, aside): (Vec<LinePoint>, Vec<LinePoint>) = points.into_iter().partition(|p| {
        med.is_some_and(|m| (p.counts_per_rev / m - 1.0).abs() <= COUNTS_PER_REV_SHARE)
    });
    let aside: Vec<(LinePoint, f64)> = aside
        .into_iter()
        .map(|p| (p, med.unwrap_or(f64::NAN)))
        .collect();
    let mut levels: Vec<f64> = kept.iter().map(|p| p.level).collect();
    levels.dedup();
    if levels.len() < LINE_LEVELS {
        return (None, aside);
    }
    let (w, i): (Vec<f64>, Vec<f64>) = kept.iter().map(|p| (p.rad_s, p.i_ma)).unzip();
    let line = polyfit::<2>(&w, &i).map(|[a, b]| OpenLine {
        coulomb_ma: a,
        viscous_ma_s_per_rad: b,
        points: kept,
    });
    (line, aside)
}

fn split(a: f64, b: f64) -> f64 {
    (a - b).abs() / ((a + b) / 2.0)
}

/// The level J is read at: the highest whose spins ran both ways, from
/// `spins` as (level, logical sign).
pub fn j_level(spins: impl IntoIterator<Item = (f64, i8)>) -> Option<f64> {
    let s: Vec<(f64, i8)> = spins.into_iter().collect();
    s.iter()
        .map(|x| x.0)
        .filter(|l| [1i8, -1].iter().all(|d| s.contains(&(*l, *d))))
        .fold(None, |m: Option<f64>, l| Some(m.map_or(l, |m| m.max(l))))
}

/// Which level J came from: the top level passes; a lower one, the top
/// left out by the runway rule, warns; none refuses.
pub fn j_level_check(level: Option<f64>) -> Check {
    let top = LEVELS[LEVELS.len() - 1];
    let bound = format!("the {:.0}% level, the top", 100.0 * top);
    match level {
        Some(l) if l == top => check(
            "the level J is read at",
            format!("{:.0}%", 100.0 * l),
            bound,
            Verdict::Pass,
        ),
        Some(l) => check(
            "the level J is read at",
            format!(
                "{:.0}%, the highest that ran both ways: the {:.0}% level was left out by the runway rule",
                100.0 * l,
                100.0 * top
            ),
            bound,
            Verdict::Warn,
        ),
        None => check(
            "the level J is read at",
            "no level ran its spins both ways".into(),
            bound,
            Verdict::Fail,
        ),
    }
}

/// A coast's raw tap difference at rest: the median of vA - vB over its
/// last [`TAIL_S`]; None unless the rotor stopped before them.
pub fn coast_tail(coast: &[TelFrame], bridge: i8, c: &JudgeCfg) -> Option<f64> {
    let from = coast.len().checked_sub((TAIL_S * c.fs) as usize)?;
    let cut = coast::cut_row(coast)?;
    let emf = coast::emf_from_cut(coast, bridge, cut, 0.0, c.row.counts_per_v)?;
    if cut + coast::stop_row(&emf)? > from {
        return None;
    }
    let raw: Option<Vec<f64>> = coast[from..]
        .iter()
        .map(|f| Some(f64::from(f.vmotor_a?) - f64::from(f.vmotor_b?)))
        .collect();
    median(&raw?)
}

/// One tap zero for every coast of the run: the mean of the up coasts'
/// and the down coasts' tail medians, else the rest step's.
pub fn tap_zero(spins: &[Spin], c: &JudgeCfg) -> TapZero {
    let way = |d: i8| {
        let t: Vec<f64> = spins
            .iter()
            .filter(|s| s.dir == d)
            .filter_map(|s| coast_tail(s.coast, c.bridge(s.dir), c))
            .collect();
        median(&t).map(|m| (m, t.len()))
    };
    let (up, down) = (way(1), way(-1));
    match (up, down) {
        (Some((u, _)), Some((d, _))) => TapZero {
            counts: 0.5 * (u + d),
            up,
            down,
            from_tails: true,
        },
        _ => TapZero {
            counts: c.zero_e,
            up,
            down,
            from_tails: false,
        },
    }
}

/// Whether each spin's coast is Ke's: the first [`runway::SPINS_EACH`] of
/// its level and direction, plan sec 2.7's 2 cycles a level. The top
/// level's third each way is J's run-up only.
pub fn ke_spins(spins: &[Spin]) -> Vec<bool> {
    spins
        .iter()
        .map(|s| {
            spins
                .iter()
                .filter(|t| t.level == s.level && t.dir == s.dir && t.n < s.n)
                .count()
                < runway::SPINS_EACH
        })
        .collect()
}

/// Each level's mean Ke over its gated coasts, from (level, read), the
/// lowest level first.
fn level_means(coasts: &[(f64, CoastKe)]) -> Vec<(f64, f64)> {
    let mut levels: Vec<f64> = coasts
        .iter()
        .filter(|(_, k)| k.ke().is_some())
        .map(|(l, _)| *l)
        .collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    levels
        .into_iter()
        .filter_map(|l| {
            let ke: Vec<f64> = coasts
                .iter()
                .filter(|(x, _)| *x == l)
                .filter_map(|(_, k)| k.ke())
                .collect();
            Some((l, mean(&ke)?))
        })
        .collect()
}

/// The line through the level means over their span, a share of `mean`;
/// None under [`KE_LEVELS`] levels.
fn ke_trend(levels: &[(f64, f64)], mean: f64) -> Option<f64> {
    let (x, y): (Vec<f64>, Vec<f64>) = levels.iter().copied().unzip();
    let [_, slope] = polyfit::<2>(&x, &y).filter(|_| x.len() >= KE_LEVELS)?;
    let span = x.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - x.iter().copied().fold(f64::INFINITY, f64::min);
    Some((slope * span).abs() / mean)
}

/// Ke as pooled: the coasts, the levels pooled with their means and the
/// line through them, and the top level's compare.
struct KePool {
    ke: Option<KePooled>,
    levels: Vec<(f64, f64)>,
    trend: Option<f64>,
    top: Option<TopCompare>,
}

/// Ke pooled over `coasts` as (level, read): the top level compared first
/// and left out when the line through every level's mean is over
/// [`KE_TREND_MAX`].
fn pool_ke(coasts: &[(f64, CoastKe)]) -> KePool {
    let all = coast::pooled(&coasts.iter().map(|(_, k)| k.clone()).collect::<Vec<_>>());
    let levels = level_means(coasts);
    let top = (|| {
        let p = all.as_ref()?;
        let (&(top, at), lower) = levels.split_last()?;
        let trend = ke_trend(&levels, p.all.mean)?;
        Some(TopCompare {
            level: top,
            vs_lower: at / mean(&lower.iter().map(|x| x.1).collect::<Vec<_>>())? - 1.0,
            trend,
            levels: levels.len(),
            pooled: trend <= KE_TREND_MAX,
        })
    })();
    let (ke, levels) = match &top {
        Some(t) if !t.pooled => {
            let kept: Vec<(f64, CoastKe)> = coasts
                .iter()
                .filter(|(l, _)| *l != t.level)
                .cloned()
                .collect();
            let p = coast::pooled(&kept.iter().map(|(_, k)| k.clone()).collect::<Vec<_>>());
            (p, level_means(&kept))
        }
        _ => (all, levels),
    };
    let trend = ke.as_ref().and_then(|p| ke_trend(&levels, p.all.mean));
    KePool {
        ke,
        levels,
        trend,
        top,
    }
}

/// The spins read: the events a revolution, each steady stretch and
/// coast, Ke pooled, the open-loop line, J from the run-ups of the
/// highest level that ran both ways ([`j_level`]),
/// and the checks (plan sec 2.7 and 2.9): no Ke, no J or no line refuses,
/// the spreads and the counts are reported.
pub fn judge(spins: &[Spin], c: &JudgeCfg) -> Judged {
    let mut checks = Vec::new();
    let st = steadies(spins, c);
    let mut st = st;
    let pr = per_rev(c);
    checks.push(match &c.window {
        Some(w) => check(
            "the window term",
            w.clone(),
            "measured".into(),
            Verdict::Pass,
        ),
        None => check(
            "the window term",
            "not measured: g taken 0, so the steady currents and J's carry the term".into(),
            "the window step's g".into(),
            Verdict::Warn,
        ),
    });
    let per = pr.value;
    for s in st.iter_mut() {
        s.rev_s = s.line_hz.map(|h| h / per as f64);
    }
    let cs = coast::Scales {
        counts_per_v: c.row.counts_per_v,
        per_rev: per,
        fs: c.fs,
    };
    let zero = tap_zero(spins, c);
    checks.push(check(
        "the coasts' tap zero",
        zero.said(),
        format!(
            "both directions' coasts at rest over their last {:.0} ms",
            1e3 * TAIL_S
        ),
        if zero.from_tails {
            Verdict::Pass
        } else {
            Verdict::Warn
        },
    ));
    let coasts: Vec<(usize, Result<CoastKe, String>)> = spins
        .iter()
        .map(|s| {
            let b = c.bridge(s.dir);
            (
                s.n,
                coast::coast_ke(s.drive, s.coast, b, zero.signed(b), &cs),
            )
        })
        .collect();
    let mine = ke_spins(spins);
    let read: Vec<(f64, CoastKe)> = spins
        .iter()
        .zip(&coasts)
        .zip(&mine)
        .filter(|(_, m)| **m)
        .filter_map(|((s, (_, r)), _)| r.as_ref().ok().map(|k| (s.level, k.clone())))
        .collect();
    let ke_n: Vec<usize> = spins
        .iter()
        .zip(&mine)
        .filter(|(_, m)| **m)
        .map(|(s, _)| s.n)
        .collect();
    let KePool {
        ke,
        levels,
        trend,
        top,
    } = pool_ke(&read);
    let gated = read.iter().filter(|(_, k)| k.ke().is_some()).count();
    checks.push(Check::reported(
        "coasts pass the Ke gate",
        format!(
            "{gated} of {}, the first {} spins each way a level",
            ke_n.len(),
            runway::SPINS_EACH
        ),
    ));
    checks.push(match &top {
        Some(t) => {
            let said = format!(
                "{:+.2}% against the lower levels' mean; the line through all {} levels' means {:.2}%",
                100.0 * t.vs_lower,
                t.levels,
                100.0 * t.trend
            );
            check(
                "the top level's Ke",
                if t.pooled {
                    format!("the {:.0}% level pooled: {said}", 100.0 * t.level)
                } else {
                    format!("the {:.0}% level left out of Ke: {said}", 100.0 * t.level)
                },
                format!(
                    "pooled when the line through every level's mean is within {:.0}%",
                    100.0 * KE_TREND_MAX
                ),
                if t.pooled {
                    Verdict::Pass
                } else {
                    Verdict::Warn
                },
            )
        }
        None => check(
            "the top level's Ke",
            "under two levels with a gated coast: nothing to compare".into(),
            "two levels at least".into(),
            Verdict::NotJudged,
        ),
    });
    match &ke {
        Some(p) => {
            let sd = p.all.sd.map(|s| s / p.all.mean);
            checks.push(Check::reported(
                "Ke spread",
                sd.map_or(
                    format!(
                        "{:.3} mV per rev/s from one coast",
                        coast::mv_per_rev_s(p.all.mean)
                    ),
                    |s| {
                        format!(
                            "{:.3} mV per rev/s, sd {:.2}% over {}",
                            coast::mv_per_rev_s(p.all.mean),
                            100.0 * s,
                            p.all.n
                        )
                    },
                ),
            ));
            let dir = |d: i8| {
                p.by_dir
                    .iter()
                    .find(|(b, _)| c.logical(*b) == d)
                    .map(|(_, x)| x.mean)
            };
            checks.push(Check::reported(
                "Ke by direction",
                match (dir(1), dir(-1)) {
                    (Some(u), Some(d)) => format!(
                        "up {:.3}, down {:.3} mV per rev/s: {:.2}% apart",
                        coast::mv_per_rev_s(u),
                        coast::mv_per_rev_s(d),
                        100.0 * split(u, d)
                    ),
                    _ => "a direction has no gated coast".into(),
                },
            ));
            let n = levels.len();
            checks.push(Check::reported(
                "Ke trend with level",
                match trend {
                    Some(trend) => format!(
                        "{n} levels pooled: {:.2}% over the levels' span by the line through their means",
                        100.0 * trend
                    ),
                    None => format!("{n} level pooled: no trend to read"),
                },
            ));
        }
        None => checks.push(check(
            "Ke spread",
            "no coast passed the gate".into(),
            "a pooled Ke".into(),
            Verdict::Fail,
        )),
    }
    let (line, aside) = open_line(&st);
    checks.push(check(
        "the friction line's points agree",
        if aside.is_empty() {
            "every level and direction within the share".into()
        } else {
            format!(
                "set aside: {}",
                aside
                    .iter()
                    .map(|(p, m)| format!(
                        "{:.0}% {} at {:.1} pot counts a revolution against the median {m:.1}",
                        100.0 * p.level,
                        way(p.dir),
                        p.counts_per_rev
                    ))
                    .collect::<Vec<_>>()
                    .join("; ")
            )
        },
        format!(
            "pot counts a ripple revolution within {:.0}% of the run's median",
            100.0 * COUNTS_PER_REV_SHARE
        ),
        if aside.is_empty() {
            Verdict::Pass
        } else {
            Verdict::Warn
        },
    ));
    checks.push(match &line {
        Some(l) => check(
            "the open-loop friction line",
            format!(
                "{:.1} mA + {:.4} mA per rad/s over {} level and direction means",
                l.coulomb_ma,
                l.viscous_ma_s_per_rad,
                l.points.len()
            ),
            format!("{LINE_LEVELS} levels at least"),
            Verdict::Pass,
        ),
        None => check(
            "the open-loop friction line",
            {
                let mut l: Vec<f64> = st
                    .iter()
                    .filter(|s| s.rad_s().is_some() && s.i_ma.is_some())
                    .map(|s| s.level)
                    .collect();
                l.dedup();
                format!(
                    "steady spins at {} level{}: the higher levels did not run, so no line is fitted",
                    l.len(),
                    if l.len() == 1 { "" } else { "s" }
                )
            },
            format!("{LINE_LEVELS} levels at least"),
            Verdict::Fail,
        ),
    });
    let i_drag0_a = c
        .i_drag0_a
        .or_else(|| line.as_ref().map(|l| (l.coulomb_ma / 1e3).max(0.0)));
    let jl = j_level(spins.iter().map(|s| (s.level, s.dir)));
    checks.push(j_level_check(jl));
    let at = jl.unwrap_or(f64::NAN);
    let mut runups = Vec::new();
    let inertia = match (&ke, i_drag0_a, jl) {
        (_, _, None) => Err("no level ran its spins both ways".into()),
        (Some(p), Some(drag), Some(_)) => {
            let s = runup::Scales {
                counts_per_a: c.row.counts_per_a,
                drive_offset_counts: c.row.drive_offset_counts,
                counts_per_v: c.row.counts_per_v,
                terminal_offset_counts: c.row.terminal_offset_counts,
                pwm_arr: c.row.pwm_arr,
                window_s: c.row.window_s,
                r_ohm: c.r_ohm,
                l_h: c.l_h,
                ke: p.all.mean,
                i_drag0_a: drag,
                per_rev: per,
                fs: c.fs,
            };
            for sp in spins.iter().filter(|s| s.level == at) {
                let run = runup::Run {
                    dir: c.bridge(sp.dir),
                    spin: sp.drive,
                    coast: sp.coast,
                    zero_i: c.zero_i,
                    zero_e: c.zero_e,
                };
                runups.push((sp.n, sp.dir, runup::read_run(&run, &s)));
            }
            let ok: Vec<RunUp> = runups
                .iter()
                .filter_map(|(_, _, r)| r.as_ref().ok().cloned())
                .collect();
            runup::inertia(&ok, &s)
        }
        _ => Err("no Ke or no drag at rest to read the run-ups through".into()),
    };
    match &inertia {
        Ok(j) => {
            checks.push(check(
                "J from the run-ups",
                format!(
                    "{:.3e} kg m^2 over {} of {} run-ups at {:.0}%",
                    j.j.mean,
                    j.j.n,
                    runups.len(),
                    100.0 * at
                ),
                "the run-ups of J's level read a J".into(),
                Verdict::Pass,
            ));
            let dir = |d: i8| {
                j.by_dir
                    .iter()
                    .find(|(b, _)| c.logical(*b) == d)
                    .map(|(_, x)| x.mean)
            };
            checks.push(Check::reported(
                "J by direction",
                match (dir(1), dir(-1)) {
                    (Some(u), Some(d)) => format!(
                        "up {u:.3e}, down {d:.3e} kg m^2: {:.1}% apart",
                        100.0 * split(u, d)
                    ),
                    _ => "a direction has no J".into(),
                },
            ));
        }
        Err(e) => checks.push(check(
            "J from the run-ups",
            format!("{e} ({} run-ups at {:.0}%)", runups.len(), 100.0 * at),
            "the run-ups of J's level read a J".into(),
            Verdict::Fail,
        )),
    }
    Judged {
        per_rev: pr,
        steady: st,
        coasts,
        zero,
        ke_spins: ke_n,
        top,
        ke_levels: levels,
        trend,
        ke,
        line,
        i_drag0_a,
        j_level: jl,
        runups,
        inertia,
        checks,
    }
}

/// What to do when a check refuses the step.
pub fn advice(name: &str) -> &'static str {
    match name {
        "Ke spread" => {
            "no coast read a Ke: check the cable and its contacts and that the horn turns freely, then run the step again"
        }
        "J from the run-ups" => {
            "no run-up read a J: the runway may have left the levels out or their rows read too few crossings; check the stops and the nameplate's speed, then run the step again"
        }
        "the level J is read at" => {
            "no level ran its spins both ways: check the stops and the nameplate's speed, then run the step again"
        }
        "the open-loop friction line" => {
            "the friction line needs two levels and the higher levels did not run: check the stops and that the horn turns freely, then run the step again"
        }
        _ => "check the servo, then run the step again",
    }
}

fn pooled(p: &coast::Pooled, unit: &str, source: &str) -> Value {
    estimate(
        p.mean,
        p.sd.map_or(Scatter::Once(p.n), |sd| Scatter::of(sd, p.n)),
        unit,
        source,
    )
}

/// Step 5 on the servo: the pilot and the spins, then the way out.
/// Refused with nothing written when it cannot start; once it has
/// written, it ends through the guard whatever happens, and keeps its
/// captures and result.
pub async fn spins<P: Pipe>(rig: &mut Rig<'_, P>, i: &SpinsInputs<'_>) -> Result<Spun, Refused> {
    let mut b = before(rig, i).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.1} counts, rail {:.2} V, stored drive_polarity {}, the probe's {}, before anything is written",
            b.pos, b.rail_v, b.stored, b.probe.found
        ),
    ];
    let mut ledger = Ledger::default();
    let mut taken = Vec::new();
    let mut found = Found::default();
    let ran = body(rig, &mut b, &mut ledger, &mut taken, &mut found, &mut lines).await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = rest::pos_now(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines).map_err(Halt::from);
    let cfg = JudgeCfg {
        fs: b.tick_hz,
        polarity: b.probe.found,
        zero_i: b.rest.zero_counts,
        zero_e: b.rest.tap_offset,
        row: RowScales {
            counts_per_a: b.counts_per_a,
            drive_offset_counts: 0.0,
            counts_per_v: b.counts_per_v,
            terminal_offset_counts: b.rest.tap_offset,
            pwm_arr: b.pwm_arr,
            window_s: b.window.as_ref().map_or(0.0, |w| w.0),
        },
        window: b.window.as_ref().map(|w| w.1.clone()),
        r_ohm: b.stops.r_ohm,
        l_h: b.stops.l_h,
        per_rev: b.per_rev,
        i_drag0_a: None,
    };
    let finished = ran.is_ok();
    let read: Vec<Spin> = found
        .spins
        .iter()
        .filter(|s| !s.pilot)
        .map(|s| Spin {
            n: s.n,
            level: s.window.level,
            dir: s.window.dir,
            drive: &taken[s.drive_k].burst.rows,
            coast: &taken[s.coast_k].burst.rows,
        })
        .collect();
    let judged = finished.then(|| judge(&read, &cfg));
    if let Some(j) = &judged {
        lines.push(format!("Ke's tap zero: {}", j.zero.said()));
    }
    let mut checks = judged.as_ref().map_or(Vec::new(), |j| j.checks.clone());
    if finished {
        checks.push(check(
            "levels the runway left out",
            if found.left_out.is_empty() {
                "none".into()
            } else {
                found
                    .left_out
                    .iter()
                    .map(|(l, d, _)| format!("{:.0}% {}", 100.0 * l, way(*d)))
                    .collect::<Vec<_>>()
                    .join(", ")
            },
            format!(
                "every level's T at least {:.0} ms both ways, a higher level's bounds from the level below it",
                1e3 * runway::T_MIN_S
            ),
            if found.left_out.is_empty() {
                Verdict::Pass
            } else {
                Verdict::Warn
            },
        ));
    }
    lines.extend(found.path.iter().map(|p| format!("decision: {p}")));
    lines.push(format!(
        "end position {} counts ({:.1} before)",
        after.map_or("not read".into(), |p| format!("{p:.1}")),
        b.pos
    ));
    if !exit.torque_off {
        checks.push(check(
            "torque off at the end",
            "torque_enable did not read back 0".into(),
            "reads 0".into(),
            Verdict::Fail,
        ));
    }
    let failed: Vec<&Check> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .collect();
    let (mut way, mut why) = match (&ran, &kept) {
        (Err(h), _) => (h.way(), Some(h.why())),
        (Ok(()), Err(h)) => (Way::Error, Some(h.why())),
        (Ok(()), Ok(())) if !failed.is_empty() => {
            let mut what: Vec<&str> = failed.iter().map(|c| advice(&c.name)).collect();
            what.dedup();
            (
                Way::Refused,
                Some(format!(
                    "refused by {}: {}",
                    failed
                        .iter()
                        .map(|c| c.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", "),
                    what.join("; ")
                )),
            )
        }
        _ => (Way::Done, None),
    };
    if let Some(w) = why.as_mut() {
        w.push_str(&format!(
            "; the shaft ends at {} counts",
            after.map_or("a position not read".into(), |p| format!("{p:.0}"))
        ));
    }
    let result = result_json(
        &b,
        &found,
        judged.as_ref(),
        &cfg,
        &checks,
        way,
        &exit,
        after,
    );
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Spun {
        ended: Ended {
            way,
            why,
            exit,
            lines,
        },
        checks,
        result: Some(result),
    })
}

fn opt(v: Option<f64>) -> Value {
    v.filter(|x| x.is_finite()).map_or(Value::Null, Value::from)
}

fn pooled_json(p: &coast::Pooled, scale: f64) -> Value {
    json!({"mean": p.mean * scale, "sd": p.sd.map(|s| s * scale), "n": p.n})
}

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    judged: Option<&Judged>,
    cfg: &JudgeCfg,
    checks: &[Check],
    way: Way,
    exit: &guard::Exit,
    after: Option<f64>,
) -> Value {
    let (glo, ghi) = b.travel.guards();
    let spins: Vec<Value> = found
        .spins
        .iter()
        .map(|s| {
            let w = &s.window;
            let r = &s.read;
            json!({
                "n": s.n, "pilot": s.pilot, "level_share": w.level, "dir": w.dir, "duty_q15": s.q15,
                "volts": w.level * b.stated_v, "start_counts": w.start, "attempt": s.attempt,
                "planned": {"bounds_from": w.from.said(), "t_s": w.t_s, "t_rise_s": w.t_rise_s, "t_runway_s": w.t_runway_s, "runway_counts": w.runway,
                            "bound_speed_cps": w.bound_speed, "bound_coast_counts": w.bound_coast, "travel_bound_counts": w.travel_bound()},
                "on_s": s.on_s,
                "end_counts": r.end, "travel_counts": r.travel, "drive_travel_counts": r.drive_travel,
                "pot_speed_cps": r.speed_cps, "t90_s": r.t90_s, "coast_counts": r.coast_counts, "coast_s": r.coast_s,
                "captures": [s.drive_k + 1, s.coast_k + 1],
            })
        })
        .collect();
    let mut estimates = Map::new();
    let mut fits = Value::Null;
    if let Some(j) = judged {
        let g_words = match &cfg.window {
            Some(w) => format!("the window term taken off by {w}"),
            None => "the window term NOT taken off (the window step not run; g taken 0)".into(),
        };
        estimates.insert(
            "events_per_rev".into(),
            estimate(
                j.per_rev.value as f64,
                Scatter::Nominal,
                "events/rev",
                &j.per_rev.source,
            ),
        );
        if let Some(p) = &j.ke {
            estimates.insert(
                "ke_v_s_per_rad".into(),
                pooled(
                    &p.all,
                    "V s/rad",
                    &format!(
                        "the mean over {} coasts, the first {} spins each way a level{}, through the ladder notebook's gate: each coast's EMF over its ripple speed in its whole cycles nearest 10 ms, the tap zero {}",
                        p.all.n,
                        runway::SPINS_EACH,
                        j.top.as_ref().filter(|t| !t.pooled).map_or(String::new(), |t| format!(
                            ", the {:.0}% level left out",
                            100.0 * t.level
                        )),
                        j.zero.said()
                    ),
                ),
            );
        }
        if let Ok(i) = &j.inertia {
            estimates.insert(
                "j_kg_m2".into(),
                pooled(
                    &i.j,
                    "kg m^2",
                    &format!(
                        "the mean over {} run-ups from rest at {:.0}% of the stated voltage, each against its own coast, through the order count, {g_words}",
                        i.j.n,
                        100.0 * j.j_level.unwrap_or(f64::NAN)
                    ),
                ),
            );
        }
        if let Some(l) = &j.line {
            let src = format!(
                "the line of the steady winding current on the steady ripple speed over {} level and direction means, both directions pooled, {g_words}; no in-drive current offset taken off",
                l.points.len()
            );
            let n = Scatter::Once(l.points.len());
            estimates.insert(
                "friction_coulomb_ma".into(),
                estimate(l.coulomb_ma, n, "mA", &src),
            );
            estimates.insert(
                "friction_viscous_ma_s_per_rad".into(),
                estimate(l.viscous_ma_s_per_rad, n, "mA per motor rad/s", &src),
            );
        }
        let wiring = |bridge: i8| cfg.logical(bridge);
        let mut levels: Vec<f64> = j.steady.iter().map(|s| s.level).collect();
        levels.dedup();
        let per_level: Vec<Value> = levels
            .iter()
            .flat_map(|l| [1i8, -1].map(|d| (*l, d)))
            .filter_map(|(l, d)| {
                let g: Vec<&Steady> = j.steady.iter().filter(|s| s.level == l && s.dir == d).collect();
                if g.is_empty() {
                    return None;
                }
                let m = |f: &dyn Fn(&Steady) -> Option<f64>| {
                    mean(&g.iter().filter_map(|s| f(s)).collect::<Vec<_>>())
                };
                Some(json!({"level_share": l, "dir": d, "spins": g.len(),
                            "rev_s": opt(m(&|s| s.rev_s)), "rad_s": opt(m(&Steady::rad_s)),
                            "current_raw_ma": opt(m(&|s| s.crest_ma)), "current_ma": opt(m(&|s| s.i_ma)),
                            "pot_cps": opt(m(&|s| s.pot_cps))}))
            })
            .collect();
        let coasts: Vec<Value> = j
            .coasts
            .iter()
            .map(|(n, r)| match r {
                Ok(k) => json!({"n": n, "dir": wiring(k.dir), "in_ke": j.ke_spins.contains(n), "held_ticks": k.level, "cut": k.cut, "e_first_v": k.e_first,
                                "turned": k.turned, "open_rows": k.open_rows, "f_seed_hz": k.f_seed,
                                "ke_v_s_per_rad": k.window.map(|w| w.ke), "rev_s": k.window.map(|w| w.rev_s),
                                "decel_frac": k.window.map(|w| w.decel_frac), "cycles": k.window.map(|w| w.cycles),
                                "spin_rev_s": k.spin_rev_s, "ke_cut_v_s_per_rad": k.ke_cut(), "gated": k.gated}),
                Err(e) => json!({"n": n, "error": e}),
            })
            .collect();
        let ke = j.ke.as_ref().map_or(Value::Null, |p| {
            json!({
                "v_s_per_rad": pooled_json(&p.all, 1.0),
                "mv_per_rev_s": pooled_json(&p.all, coast::mv_per_rev_s(1.0)),
                "by_dir": p.by_dir.iter().map(|(d, x)| json!({"dir": wiring(*d), "v_s_per_rad": pooled_json(x, 1.0)})).collect::<Vec<_>>(),
                "by_held_ticks": p.by_level.iter().map(|(l, x)| json!({"held_ticks": l, "v_s_per_rad": pooled_json(x, 1.0)})).collect::<Vec<_>>(),
                "coasts": p.coasts,
                "spins": j.ke_spins,
                "levels_pooled": j.ke_levels.iter().map(|(l, m)| json!({"level_share": l, "v_s_per_rad": m})).collect::<Vec<_>>(),
                "trend": opt(j.trend),
                "top": j.top.as_ref().map(|t| json!({"level_share": t.level, "vs_lower": t.vs_lower, "trend_all_levels": t.trend,
                                                     "levels": t.levels, "pooled": t.pooled})),
            })
        });
        let tail = |t: Option<(f64, usize)>| {
            t.map_or(
                Value::Null,
                |(m, n)| json!({"median_counts": m, "coasts": n}),
            )
        };
        let ke_zero = json!({"counts": j.zero.counts, "from_tails": j.zero.from_tails, "tail_s": TAIL_S,
                             "up_tail": tail(j.zero.up), "down_tail": tail(j.zero.down), "source": j.zero.said()});
        let inertia = match &j.inertia {
            Ok(i) => json!({
                "j_kg_m2": pooled_json(&i.j, 1.0),
                "by_dir": i.by_dir.iter().map(|(d, x)| json!({"dir": wiring(*d), "j_kg_m2": pooled_json(x, 1.0)})).collect::<Vec<_>>(),
                "runs": i.runs.iter().map(|r| json!({"dir": wiring(r.dir), "j_kg_m2": r.j, "fits_kept": r.fits_kept, "windows_flagged": r.windows_flagged})).collect::<Vec<_>>(),
                "coast_friction": i.friction.iter().map(|f| json!({"dir": wiring(f.dir), "a0_rad_s2": f.a0, "a1_per_s": f.a1, "tc_n_m": f.tc, "b_n_m_s_per_rad": f.b, "rms_n_m": f.rms, "bins": f.bins})).collect::<Vec<_>>(),
            }),
            Err(e) => json!({"error": e}),
        };
        fits = json!({
            "events_per_rev": {"value": j.per_rev.value, "source": j.per_rev.source},
            "ke": ke,
            "ke_zero": ke_zero,
            "coasts": coasts,
            "steady": per_level,
            "steady_spins": j.steady.iter().map(|s| json!({"n": s.n, "level_share": s.level, "dir": s.dir, "rows": s.rows,
                "line_hz": s.line_hz, "rev_s": s.rev_s, "pot_cps": s.pot_cps, "current_raw_ma": s.crest_ma,
                "current_ma": s.i_ma, "seed_hz": s.seed_hz})).collect::<Vec<_>>(),
            "open_loop_line": j.line.as_ref().map(|l| json!({"coulomb_ma": l.coulomb_ma, "viscous_ma_s_per_rad": l.viscous_ma_s_per_rad,
                "points": l.points.iter().map(|p| json!({"level_share": p.level, "dir": p.dir, "rad_s": p.rad_s, "current_ma": p.i_ma, "current_raw_ma": p.raw_ma, "spins": p.spins, "pot_counts_per_rev": p.counts_per_rev})).collect::<Vec<_>>()})),
            "i_drag0_a": j.i_drag0_a,
            "inertia": inertia,
            "runups": j.runups.iter().map(|(n, d, r)| match r {
                Ok(x) => json!({"n": n, "dir": d, "rows": x.rows, "crossings": x.tc.len(), "fits": x.fits.len(), "dropouts": x.dropouts}),
                Err(e) => json!({"n": n, "dir": d, "error": e}),
            }).collect::<Vec<_>>(),
            "scales": {"counts_per_a": cfg.row.counts_per_a, "drive_offset_counts": cfg.row.drive_offset_counts,
                       "counts_per_v": cfg.row.counts_per_v, "terminal_offset_counts": cfg.row.terminal_offset_counts,
                       "pwm_arr": cfg.row.pwm_arr, "window_s": cfg.row.window_s, "r_ohm": cfg.r_ohm, "l_h": cfg.l_h,
                       "zero_i_counts": cfg.zero_i, "zero_e_counts": cfg.zero_e,
                       "source": "the nominal sense block read off the servo; the rest step's zeros, its tap difference the terminal offset; the stops step's seated R and L; no in-drive current offset"},
        });
    }
    json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "estimates": estimates,
        "fits": fits,
        "spins": spins,
        "left_out": found.left_out.iter().map(|(l, d, w)| json!({"level_share": l, "dir": d, "why": w})).collect::<Vec<_>>(),
        "path": found.path,
        "drives": {
            "stops_counts": [b.stops.lo, b.stops.hi], "guards_counts": [glo, ghi], "guard_counts": GUARD_COUNTS,
            "short_counts": SHORT_COUNTS, "levels_share": LEVELS, "stated_v": b.stated_v,
            "nameplate_cps": b.rated_cps, "approach_q15": b.approach_q15, "top_q15": b.top_q15,
            "drive_deadman_s": DRIVE_DEADMAN_S, "poll_deadman_s": POLL_DEADMAN_S, "coast_s": COAST_S,
            "still_counts": b.still_counts, "rise_assumed": found.t90_assumed,
        },
        "drive_polarity": {"stored": b.stored, "driven": b.probe.found, "corrected_volatile": found.corrected},
        "checks": checks.iter().map(Check::json).collect::<Vec<_>>(),
        "rail_v": b.rail_v,
        "position_before_counts": b.pos,
        "position_after_counts": after,
        "exit": exit.lines,
        "torque_off": exit.torque_off,
    })
}

#[cfg(test)]
mod tests {
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    use std::sync::atomic::AtomicBool;
    use std::time::Duration;

    use futures_lite::future::block_on;
    use osc_client::descriptor::Descriptor;
    use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
    use osc_client::pipe::{Pipe, PipeError};
    use osc_client::{BaudRate, Client, Id, Inst, Opcode};

    use osc_servo_core::kernel::phases::loops::faults::BIT_DEADMAN;

    use super::*;
    use crate::sink::MemSink;
    use crate::steps::polarity::STALLED;
    use crate::steps::runway::{SpinRead, after};

    const OSC_EXCHANGE: u8 = 0x20;
    const ID: Id = Id::new(1);

    /// A write as the servo saw it, held for a COMMIT or not, or a COMMIT.
    #[derive(Clone, Debug, PartialEq)]
    enum Ev {
        Write(u16, Vec<u8>, bool),
        Commit,
    }

    type Log = Rc<RefCell<Vec<Ev>>>;
    type OnWrite = Box<dyn FnMut(u16, &[u8], bool, &mut FakePipe)>;

    struct Spy {
        inner: FakePipe,
        log: Log,
        on_write: OnWrite,
    }

    impl Pipe for Spy {
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
                    Some(Opcode::Write) if payload.len() >= 2 && !i.is_status() => {
                        let addr = u16::from_le_bytes([payload[0], payload[1]]);
                        self.log.borrow_mut().push(Ev::Write(
                            addr,
                            payload[2..].to_vec(),
                            i.hold(),
                        ));
                        (self.on_write)(addr, &payload[2..], i.hold(), &mut self.inner);
                    }
                    Some(Opcode::Commit) if !i.is_status() => {
                        self.log.borrow_mut().push(Ev::Commit)
                    }
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

    fn descriptor() -> Descriptor {
        Descriptor::parse(
            &std::fs::read_to_string(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../descriptors/osc-servo/0.1.json"
            ))
            .expect("descriptor"),
        )
        .expect("parses")
    }

    fn addr(d: &Descriptor, name: &str) -> u16 {
        field(d, name).expect(name).addr
    }

    fn still_at(pos: u16) -> Vec<TelSample> {
        (0..4096u32)
            .map(|i| TelSample {
                tick: TelemetryTick {
                    _scan: [0; 9],
                    pos,
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

    fn rest_json() -> Value {
        let est = |v: f64| json!({"value": v});
        json!({
            "verdict": "accepted",
            "estimates": {
                "current_zero_awake_counts": est(1206.0),
                "pot_rest_sd_counts": est(0.8),
                "tap_a_counts": est(773.0),
                "tap_b_counts": est(772.0),
            },
            "channels": {"raw": {"pos": {"pattern_gain": 0.0}}},
            "frame_ripple": {"pattern_counts": null},
        })
    }

    fn probe_json(stored: bool) -> Value {
        json!({
            "verdict": "accepted",
            "drive_polarity": {"stored": stored, "found": stored, "stored_right": true},
            "part_b": {
                "ceiling_v": 0.945,
                "r_price_ohm": 3.78,
                "pulses": [
                    {"repeat": false, "unwedge": false, "volts": 0.48},
                    {"repeat": true, "unwedge": false, "volts": 0.48},
                ],
            },
        })
    }

    /// The bench servo's stops as the stops step reads them.
    fn stops_json() -> Value {
        json!({
            "verdict": "accepted",
            "stops": {"low": {"median_counts": 432.0}, "high": {"median_counts": 3626.0}},
            "estimates": {"r_seated_ohm": {"value": 4.2}, "l_seated_h": {"value": 0.00082}},
        })
    }

    fn nameplate() -> Nameplate {
        serde_json::from_str(include_str!("../../testdata/mg90-d/nameplate.json"))
            .expect("nameplate")
    }

    struct Ran {
        spun: Result<Spun, Refused>,
        log: Vec<Ev>,
        c: Client<Spy>,
    }

    fn run_with(
        on_write: impl Fn(&Descriptor) -> OnWrite,
        stops: Value,
        window: Option<Value>,
        thermal: bool,
        stop: &AtomicBool,
    ) -> Ran {
        run_on(nameplate(), on_write, stops, window, thermal, stop)
    }

    fn run_on(
        np: Nameplate,
        on_write: impl Fn(&Descriptor) -> OnWrite,
        stops: Value,
        window: Option<Value>,
        thermal: bool,
        stop: &AtomicBool,
    ) -> Ran {
        let d = descriptor();
        let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
        inner.seed_calibrated(0);
        inner.set_track(0, still_at(2041));
        let log = Log::default();
        let mut c = block_on(Client::connect(Spy {
            inner,
            log: log.clone(),
            on_write: on_write(&d),
        }))
        .expect("connect");
        let stored = block_on(c.read(ID, addr(&d, "drive_polarity"), 1)).expect("read")[0] != 0;
        log.borrow_mut().clear();
        let board: Board =
            serde_json::from_str(include_str!("../../boards/osc-dev-v006.json")).expect("board");
        let th: ThermalInputs =
            serde_json::from_str(include_str!("../../testdata/mg90-d/thermal.json"))
                .expect("thermal");
        let (rest, probe) = (rest_json(), probe_json(stored));
        let t = Cell::new(0.0);
        let clock = || {
            t.set(t.get() + 0.011);
            t.get()
        };
        let mut sink = MemSink::default();
        let spun = {
            let mut rig = Rig {
                c: &mut c,
                id: ID,
                d: &d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            block_on(spins(
                &mut rig,
                &SpinsInputs {
                    nameplate: &np,
                    board: &board,
                    thermal: thermal.then_some(&th),
                    rest: &rest,
                    probe: &probe,
                    stops: &stops,
                    window: window.as_ref(),
                },
            ))
        };
        let log = log.borrow().clone();
        Ran { spun, log, c }
    }

    /// The fake has no motor: a go-to's first duty puts the shaft at its
    /// target, and a spin's duty, held for its COMMIT, leaves it there, so
    /// the pilot does not turn.
    fn arrive(d: &Descriptor) -> OnWrite {
        let duty = addr(d, "goal_duty");
        Box::new(move |a, b, hold, pipe| {
            if a == duty && !hold && b != [0, 0] {
                let q = i16::from_le_bytes([b[0], b[1]]);
                let to = if q < 0 { 432 + 250 } else { 3626 - 250 };
                pipe.set_track(0, still_at(to));
            }
        })
    }

    fn duties(d: &Descriptor, log: &[Ev]) -> Vec<i16> {
        let gd = addr(d, "goal_duty");
        log.iter()
            .filter_map(|e| match e {
                Ev::Write(a, b, _) if *a == gd => Some(i16::from_le_bytes([b[0], b[1]])),
                _ => None,
            })
            .collect()
    }

    /// One spin's writes: the short drive dead-man, the duty and the TEL
    /// arm held and applied by one COMMIT, zero duty and the coast's arm by
    /// a second, then the step's dead-man; no duty over 0.75 of the stated
    /// voltage; the way out ends torque off.
    #[test]
    fn a_spin_writes_its_dead_man_two_commits_and_the_long_dead_man() {
        let r = run_with(arrive, stops_json(), None, true, &AtomicBool::new(false));
        let d = descriptor();
        let s = r.spun.expect("starts");
        let e = &s.ended;
        assert_eq!(e.way, Way::Refused, "{:?}\n{}", e.why, e.lines.join("\n"));
        assert!(
            e.why
                .as_deref()
                .unwrap_or("")
                .contains("the pilot did not turn"),
            "{:?}",
            e.why
        );
        let (gd, dm, chunk0) = (
            addr(&d, "goal_duty"),
            addr(&d, "bus_deadman_ticks"),
            addr(&d, "chunk0_addr"),
        );
        let short = (DRIVE_DEADMAN_S * 20_000.0) as u16;
        let long = (rest::DEADMAN_S * 20_000.0) as u16;
        let at = r
            .log
            .iter()
            .position(|w| *w == Ev::Write(dm, short.to_le_bytes().to_vec(), false))
            .expect("the drive's dead-man");
        let rail_v = s.result.as_ref().expect("result")["rail_v"]
            .as_f64()
            .expect("rail");
        let q = level_q15(PILOT_LEVEL, 4.8, rail_v);
        let w = &r.log[at..at + 8];
        assert_eq!(w[1], Ev::Write(gd, q.to_le_bytes().to_vec(), true), "{w:?}");
        assert!(
            matches!(&w[2], Ev::Write(a, _, true) if *a == chunk0),
            "{w:?}"
        );
        assert_eq!(w[3], Ev::Commit, "{w:?}");
        assert_eq!(w[4], Ev::Write(gd, vec![0, 0], true), "{w:?}");
        assert!(
            matches!(&w[5], Ev::Write(a, _, true) if *a == chunk0),
            "{w:?}"
        );
        assert_eq!(w[6], Ev::Commit, "{w:?}");
        assert_eq!(
            w[7],
            Ev::Write(dm, long.to_le_bytes().to_vec(), false),
            "{w:?}"
        );
        let top = level_q15(0.75, 4.8, rail_v);
        for q in duties(&d, &r.log) {
            assert!(q.unsigned_abs() <= top as u16, "{q}");
        }
        assert!(e.exit.torque_off, "{}", e.exit);
        let mut c = r.c;
        assert_eq!(
            block_on(c.read(ID, addr(&d, "torque_enable"), 1)).expect("read"),
            [0]
        );
        let lines = e.lines.join("\n");
        assert!(lines.contains("to the low guard 682"), "{lines}");
        assert!(lines.contains("kept "), "the captures are kept: {lines}");
        let res = s.result.expect("result");
        assert_eq!(res["spins"].as_array().map(Vec::len), Some(0), "{res}");
        assert_eq!(res["verdict"], "refused");
    }

    /// A dead-man latched in the spin refuses with the stall sentence; an
    /// interrupt stops the step; both end torque off.
    #[test]
    fn a_latched_dead_man_or_an_interrupt_ends_torque_off() {
        let latch = |d: &Descriptor| -> OnWrite {
            let mut go = arrive(d);
            let duty = addr(d, "goal_duty");
            Box::new(move |a, b, hold, pipe| {
                go(a, b, hold, pipe);
                if a == duty && hold && b != [0, 0] {
                    pipe.sim_mut()
                        .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
                }
            })
        };
        let r = run_with(latch, stops_json(), None, true, &AtomicBool::new(false));
        let e = r.spun.expect("starts").ended;
        assert_eq!(e.way, Way::Refused, "{:?}", e.why);
        assert!(
            e.why.as_deref().unwrap_or("").starts_with(STALLED),
            "{:?}",
            e.why
        );
        assert!(e.exit.torque_off, "{}", e.exit);

        // the interrupt lands with the go-to's first duty
        let stop = Rc::new(AtomicBool::new(false));
        let set = stop.clone();
        let hook = move |d: &Descriptor| -> OnWrite {
            let mut go = arrive(d);
            let (gd, set) = (addr(d, "goal_duty"), set.clone());
            Box::new(move |a, b, hold, pipe| {
                go(a, b, hold, pipe);
                if a == gd && !hold && b != [0, 0] {
                    set.store(true, std::sync::atomic::Ordering::SeqCst);
                }
            })
        };
        let r = run_with(hook, stops_json(), None, true, &stop);
        let e = r.spun.expect("starts").ended;
        assert_eq!(e.way, Way::Interrupted, "{:?}", e.why);
        assert!(e.exit.torque_off, "{}", e.exit);
        assert!(!r.log.contains(&Ev::Commit), "no spin after the interrupt");
    }

    /// Refused before any write: a stops step not accepted, no thermal
    /// values, a window step refused, the shaft outside the stops, stops
    /// too close for a runway.
    #[test]
    fn the_step_refuses_before_any_write() {
        let none = |_: &Descriptor| -> OnWrite { Box::new(|_, _, _, _| {}) };
        let flag = AtomicBool::new(false);
        let mut refused = stops_json();
        refused["verdict"] = json!("refused");
        let mut outside = stops_json();
        outside["stops"]["high"]["median_counts"] = json!(2000.0);
        let mut narrow = stops_json();
        narrow["stops"]["high"]["median_counts"] = json!(1400.0);
        for (stops, window, thermal, says) in [
            (refused, None, true, "accepted stops step"),
            (stops_json(), None, false, "--thermal"),
            (
                stops_json(),
                Some(json!({"verdict": "refused"})),
                true,
                "window step was refused",
            ),
            (outside, None, true, "not between the stops"),
            (narrow, None, true, "leave no runway"),
        ] {
            let r = run_with(none, stops, window, thermal, &flag);
            let e = r.spun.expect_err("refused");
            assert!(e.0.contains(says), "{says}: {}", e.0);
            // the state read pages the position table: no control write
            let page = addr(&descriptor(), "pos_lut_page");
            assert!(
                !r.log
                    .iter()
                    .any(|w| matches!(w, Ev::Write(a, ..) if *a != page) || *w == Ev::Commit),
                "{says}: {:?}",
                r.log
            );
        }
    }

    /// The window step's g in S, and its absence.
    #[test]
    fn the_window_term_comes_from_the_window_step_or_is_zero() {
        assert_eq!(window_term(None), Ok(None));
        let w = json!({"verdict": "accepted", "estimates": {"g_window_ms": {"value": 3.93}}});
        let (g, words) = window_term(Some(&w)).expect("reads").expect("g");
        assert!((g - 3.93e-3).abs() < 1e-12 && words.contains("3.930 mS"));
        assert!(window_term(Some(&json!({"verdict": "accepted"}))).is_err());
    }

    fn steady_of() -> Steady {
        Steady {
            n: 1,
            level: 0.5,
            dir: 1,
            rows: 2000,
            line_hz: Some(1200.0),
            rev_s: None,
            pot_cps: Some(4700.0),
            crest_ma: Some(60.0),
            i_ma: Some(55.0),
            seed_hz: None,
        }
    }

    /// The spins are read in the nameplate's events a revolution, its
    /// brushes times rotor segments, and the report names them.
    #[test]
    fn the_events_a_revolution_are_the_nameplates() {
        let np = nameplate();
        let c = JudgeCfg {
            fs: 20_000.0,
            polarity: true,
            zero_i: 1206.0,
            zero_e: 1.0,
            row: RowScales {
                counts_per_a: 1108.45,
                drive_offset_counts: 0.0,
                counts_per_v: 248.2,
                terminal_offset_counts: 1.0,
                pwm_arr: 1200.0,
                window_s: 0.0,
            },
            window: None,
            r_ohm: 4.2,
            l_h: 0.82e-3,
            per_rev: np.events_per_rev().expect("counts"),
            i_drag0_a: None,
        };
        let p = per_rev(&c);
        assert_eq!(p.value, 6);
        assert!(
            p.source.contains("brushes times rotor_segments"),
            "{}",
            p.source
        );
    }

    /// A nameplate missing a commutator count is refused before any
    /// write, naming both counts.
    #[test]
    fn a_nameplate_without_the_commutator_counts_is_refused() {
        let none = |_: &Descriptor| -> OnWrite { Box::new(|_, _, _, _| {}) };
        let mut np = nameplate();
        np.rotor_segments = None;
        let r = run_on(np, none, stops_json(), None, true, &AtomicBool::new(false));
        let e = r.spun.expect_err("refused");
        assert_eq!(e.0, crate::nameplate::COUNTS_MISSING);
        assert!(
            r.log
                .iter()
                .all(|w| !matches!(w, Ev::Write(..) | Ev::Commit)),
            "{:?}",
            r.log
        );
    }

    const KE: f64 = 1.16e-3;
    const CPV: f64 = 250.0;

    fn ke_cfg() -> JudgeCfg {
        JudgeCfg {
            fs: 20_000.0,
            polarity: true,
            zero_i: 1210.0,
            zero_e: 2.5,
            row: RowScales {
                counts_per_a: 1108.0,
                drive_offset_counts: 0.0,
                counts_per_v: CPV,
                terminal_offset_counts: 0.0,
                pwm_arr: 1200.0,
                window_s: 0.0,
            },
            window: None,
            r_ohm: 4.2,
            l_h: 8e-4,
            per_rev: 6,
            i_drag0_a: Some(0.04),
        }
    }

    /// A spin held at `ticks` and its coast of `rows` from 300 rev/s at
    /// 1.5e4 rad/s^2, the same speeds at every level so the read's own
    /// bias is common to all: the EMF `ke w` with a 5 % ripple six a turn
    /// on the taps signed by `bridge`, `offset` counts on the raw tap
    /// difference throughout, and once the rotor stops the signed tail
    /// reads 1 count low, as the bench's tails do.
    fn ke_spin(
        ticks: i16,
        bridge: i8,
        ke: f64,
        offset: i32,
        rows: u64,
    ) -> (Vec<TelFrame>, Vec<TelFrame>) {
        let fs = 20_000.0;
        let w0 = 2.0 * core::f64::consts::PI * 300.0;
        let row = |tick: u64, duty: i16, diff: i32, raw: u16| TelFrame {
            tick,
            duty_counts: Some(duty),
            pos: Some(2000),
            vmotor_a: Some((1000 + diff) as u16),
            vmotor_b: Some(1000),
            current_raw: Some(raw),
            ..TelFrame::default()
        };
        let drive = (0..1000u64)
            .map(|k| {
                let th = w0 * k as f64 / fs;
                let raw = (1300.0 + 40.0 * (6.0 * th).cos()) as u16;
                row(k, ticks * bridge as i16, 1000 * bridge as i32, raw)
            })
            .collect();
        let (mut w, mut th) = (w0, 0.0f64);
        let coast = (0..rows)
            .map(|k| {
                let emf = ke * w * (1.0 + 0.05 * (6.0 * th).cos());
                let rest = if w == 0.0 { -1 } else { 0 };
                let diff = bridge as i32 * ((emf * CPV).round() as i32 + rest) + offset;
                th += w / fs;
                w = (w - 1.5e4 / fs).max(0.0);
                let duty = if k < 2 { ticks * bridge as i16 } else { 0 };
                row(1000 + k, duty, diff, 1210)
            })
            .collect();
        (drive, coast)
    }

    type Rows = Vec<(usize, f64, i8, Vec<TelFrame>, Vec<TelFrame>)>;

    /// The step's order at `levels`, 2 spins each way a level and 3 at the
    /// top, the top's Ke `top` times the others', its third spins each way
    /// 10 % high.
    fn ke_run(levels: &[f64], top: f64, offset: i32, rows: u64) -> Rows {
        let mut out = Vec::new();
        for (i, l) in levels.iter().enumerate() {
            let last = i + 1 == levels.len() && levels.len() > 1;
            let each = if last {
                runway::TOP_SPINS_EACH
            } else {
                runway::SPINS_EACH
            };
            for k in 0..2 * each {
                let dir = if k % 2 == 0 { 1 } else { -1 };
                let ke = match (last, k / 2) {
                    (true, 2) => 1.10 * KE,
                    (true, _) => top * KE,
                    _ => KE,
                };
                let ticks = (l * 1156.0) as i16;
                let (d, c) = ke_spin(ticks, dir, ke, offset, rows);
                out.push((out.len() + 1, *l, dir, d, c));
            }
        }
        out
    }

    fn ke_judge(run: &Rows, c: &JudgeCfg) -> Judged {
        let spins: Vec<Spin> = run
            .iter()
            .map(|(n, l, d, dr, co)| Spin {
                n: *n,
                level: *l,
                dir: *d,
                drive: dr,
                coast: co,
            })
            .collect();
        judge(&spins, c)
    }

    fn named(j: &Judged, name: &str) -> Check {
        j.checks
            .iter()
            .find(|c| c.name == name)
            .cloned()
            .unwrap_or_else(|| panic!("no check {name}"))
    }

    /// Ke's coasts' reads at `level`, by the judge's own per-coast read.
    fn reads_at(j: &Judged, run: &Rows, keep: &dyn Fn(f64) -> bool) -> Vec<f64> {
        j.coasts
            .iter()
            .zip(run)
            .filter(|((n, _), (_, l, ..))| j.ke_spins.contains(n) && keep(*l))
            .filter_map(|((_, r), _)| r.as_ref().ok().and_then(CoastKe::ke))
            .collect()
    }

    /// Plan sec 2.7: Ke pools the first two spins each way a level, the
    /// top level's third each way left to J. A top 2.4 % low puts the line
    /// through the three level means over 2 %: it is left out with a
    /// warning that gives its size, and Ke, its spread, direction split
    /// and trend come from the lower two levels.
    #[test]
    fn a_top_level_off_the_line_is_left_out_of_ke_with_a_warning() {
        let run = ke_run(&LEVELS, 0.976, 0, 8000);
        let j = ke_judge(&run, &ke_cfg());
        assert_eq!(j.ke_spins, (1..=12).collect::<Vec<_>>());
        let t = j.top.clone().expect("compared");
        assert!(!t.pooled && t.level == 0.75 && t.levels == 3, "{t:?}");
        assert!((t.vs_lower + 0.024).abs() < 5e-4, "{t:?}");
        assert!(t.trend > KE_TREND_MAX, "{t:?}");
        let ck = named(&j, "the top level's Ke");
        assert_eq!(ck.verdict, Verdict::Warn, "{}", ck.said());
        assert!(
            ck.value.contains("75% level left out")
                && ck
                    .value
                    .contains(&format!("{:+.2}% against", 100.0 * t.vs_lower)),
            "{}",
            ck.said()
        );
        let p = j.ke.clone().expect("pooled");
        let lower = reads_at(&j, &run, &|l| l < 0.75);
        assert_eq!((p.all.n, p.by_level.len(), lower.len()), (8, 2, 8));
        let m = lower.iter().sum::<f64>() / 8.0;
        assert!((p.all.mean / m - 1.0).abs() < 1e-12);
        assert!((p.all.mean / KE - 1.0).abs() < 0.005, "{}", p.all.mean);
        assert_eq!(
            named(&j, "coasts pass the Ke gate").value,
            "12 of 12, the first 2 spins each way a level"
        );
        for name in ["Ke spread", "Ke by direction", "Ke trend with level"] {
            let ck = named(&j, name);
            assert_eq!(ck.verdict, Verdict::NotJudged, "{}", ck.said());
        }
        assert!(
            named(&j, "Ke trend with level")
                .value
                .starts_with("2 levels pooled")
        );
    }

    /// A top 1.5 % low keeps the line within 2 %: the top is pooled, its
    /// two spins each way with the others', its third each way not.
    #[test]
    fn a_top_level_on_the_line_is_pooled() {
        let run = ke_run(&LEVELS, 0.985, 0, 8000);
        let j = ke_judge(&run, &ke_cfg());
        let t = j.top.clone().expect("compared");
        assert!(t.pooled && t.trend <= KE_TREND_MAX, "{t:?}");
        assert_eq!(named(&j, "the top level's Ke").verdict, Verdict::Pass);
        let p = j.ke.clone().expect("pooled");
        let mine = reads_at(&j, &run, &|_| true);
        assert_eq!((p.all.n, p.by_level.len(), mine.len()), (12, 3, 12));
        let m = mine.iter().sum::<f64>() / 12.0;
        assert!((p.all.mean / m - 1.0).abs() < 1e-12);
        assert!(p.all.mean < KE, "the third spins at 1.10 are J's only");
        let trend = named(&j, "Ke trend with level");
        assert_eq!(trend.verdict, Verdict::NotJudged, "{}", trend.said());
        assert!(trend.value.starts_with("3 levels pooled"));
    }

    /// The zero is the mean of the up and the down tails' medians, so the
    /// signed 1 count low the tails read cancels, and an offset common to
    /// both directions on the raw taps is taken off exactly: each coast
    /// reads what it reads with no offset. The rest step's zero is not
    /// used.
    #[test]
    fn the_tap_zero_is_the_two_directions_tail_mean() {
        let c = ke_cfg();
        let (clean, off) = (
            ke_run(&LEVELS, 0.976, 0, 8000),
            ke_run(&LEVELS, 0.976, 7, 8000),
        );
        let (a, b) = (ke_judge(&clean, &c), ke_judge(&off, &c));
        assert_eq!(
            a.zero,
            TapZero {
                counts: 0.0,
                up: Some((-1.0, 7)),
                down: Some((1.0, 7)),
                from_tails: true
            }
        );
        assert_eq!(
            b.zero,
            TapZero {
                counts: 7.0,
                up: Some((6.0, 7)),
                down: Some((8.0, 7)),
                from_tails: true
            }
        );
        assert_eq!(a.coasts, b.coasts);
        assert_eq!(a.ke, b.ke);
        let ck = named(&b, "the coasts' tap zero");
        assert_eq!(ck.verdict, Verdict::Pass, "{}", ck.said());
        assert!(ck.value.starts_with("+7.00 counts"), "{}", ck.said());
    }

    /// Coasts too short to come to rest have no tail: the rest step's
    /// zero is taken, as the record's rule takes it, and the check warns.
    #[test]
    fn without_a_tail_the_rest_zero_is_taken_and_said() {
        let c = ke_cfg();
        let j = ke_judge(&ke_run(&LEVELS, 0.976, 0, 2400), &c);
        assert_eq!((j.zero.counts, j.zero.from_tails), (c.zero_e, false));
        assert_eq!((j.zero.up, j.zero.down), (None, None));
        let ck = named(&j, "the coasts' tap zero");
        assert_eq!(ck.verdict, Verdict::Warn, "{}", ck.said());
        assert!(
            ck.value.contains("the rest step's +2.50 counts"),
            "{}",
            ck.said()
        );
    }

    /// Ke from one level has no trend to read: it is reported, Ke stands,
    /// and nothing about Ke refuses.
    #[test]
    fn one_level_of_ke_is_reported() {
        let j = ke_judge(&ke_run(&[0.25], 1.0, 0, 8000), &ke_cfg());
        assert_eq!(j.top, None);
        assert!(j.ke.is_some());
        assert_eq!(named(&j, "the top level's Ke").verdict, Verdict::NotJudged);
        let ck = named(&j, "Ke trend with level");
        assert_eq!(ck.verdict, Verdict::NotJudged, "{}", ck.said());
        assert_eq!(ck.value, "1 level pooled: no trend to read");
        for name in ["coasts pass the Ke gate", "Ke spread", "Ke by direction"] {
            assert_ne!(named(&j, name).verdict, Verdict::Fail, "{name}");
        }
    }

    /// The open-loop line needs two levels: one level's two directions
    /// fit nothing, and its check refuses saying the higher levels did not
    /// run. A point whose pot counts a ripple revolution lie more than
    /// 15 % from the run's median is set aside, and the line still needs
    /// two levels without it.
    #[test]
    fn the_friction_line_needs_two_levels_that_agree() {
        let at = |level: f64, dir: i8, w: f64, i: f64, cpr: f64| Steady {
            level,
            dir,
            rev_s: Some(w / (2.0 * core::f64::consts::PI)),
            pot_cps: Some(cpr * w / (2.0 * core::f64::consts::PI)),
            i_ma: Some(i),
            crest_ma: Some(i + 3.0),
            ..steady_of()
        };
        let one = [
            at(0.25, 1, 560.0, 45.0, 25.0),
            at(0.25, -1, 600.0, 44.0, 22.5),
        ];
        assert_eq!(open_line(&one), (None, vec![]));
        let mut run2 = vec![
            at(0.25, 1, 545.0, 89.2, 25.7),
            at(0.25, -1, 602.0, 74.6, 22.2),
            at(0.5, 1, 1330.0, 118.8, 23.0),
            at(0.5, -1, 1379.0, 105.5, 24.6),
        ];
        let (l, aside) = open_line(&run2);
        let l = l.expect("a line");
        assert!(aside.is_empty() && l.points.len() == 4, "{l:?}");
        // the 75 % up point read at half its speed: 48 counts a revolution
        run2.push(at(0.75, 1, 1029.0, 143.8, 48.0));
        let (l, aside) = open_line(&run2);
        assert_eq!(l.expect("still two levels").points.len(), 4);
        assert_eq!(aside.len(), 1);
        assert_eq!((aside[0].0.level, aside[0].0.dir), (0.75, 1));
        // without the 50 % level what is left is one level: no line
        let (l, aside) = open_line(&[run2[0].clone(), run2[1].clone(), run2[4].clone()]);
        assert!(l.is_none() && aside.len() == 1, "{aside:?}");
        assert!(advice("the open-loop friction line").contains("higher levels did not run"));
    }

    /// J's level is the highest that ran both ways: the top passes, a
    /// lower one warns, a level that ran one way only is passed over, none
    /// refuses.
    #[test]
    fn j_is_read_at_the_highest_level_that_ran_both_ways() {
        let all: Vec<(f64, i8)> = runway::order().iter().map(|s| (s.level, s.dir)).collect();
        assert_eq!(j_level(all.iter().copied()), Some(0.75));
        assert_eq!(j_level_check(Some(0.75)).verdict, Verdict::Pass);
        let one_way: Vec<(f64, i8)> = all.iter().copied().filter(|s| *s != (0.75, -1)).collect();
        assert_eq!(j_level(one_way), Some(0.5));
        assert_eq!(j_level([(0.25, 1), (0.25, -1)]), Some(0.25));
        assert_eq!(j_level_check(Some(0.25)).verdict, Verdict::Warn);
        assert_eq!(j_level([(0.5, 1)]), None);
        assert_eq!(j_level_check(None).verdict, Verdict::Fail);
    }

    /// The bench servo's class, a 5 V rail, the probe's R, the stops found.
    fn envelope() -> Envelope {
        let board: Board =
            serde_json::from_str(include_str!("../../boards/osc-dev-v006.json")).expect("board");
        let thermal: ThermalInputs =
            serde_json::from_str(include_str!("../../testdata/mg90-d/thermal.json"))
                .expect("thermal");
        let mut f = Facts::new(&nameplate(), 5.0);
        f.polarity_proven = true;
        f.ntc_c = Some(25.0);
        f.stops = Some((82.0, 3792.0));
        f.r = Some(Resistance {
            ohm: 3.78,
            source: PRICE_WORDS.into(),
        });
        Envelope::new(f, HeatModel::new(&board, &thermal, 3.78, 1108.45, 20_000.0))
    }

    /// A motor of `k` times the nameplate's speed at the lowest level,
    /// its speed `step` times the volts' ratio more at each level above
    /// (1.0: speed in proportion to volts): a first-order rise of 20 ms
    /// to it, a constant-deceleration coast whose travel is 1e-5 counts
    /// per (count/s)^2, the ladder's own (about 1e-5 over 15 to 60 % on
    /// 3.7 to 7.4 V). Each spin is driven its T and its dead-man, a host
    /// that stalls after the drive's rows; the planner sees what the step
    /// would read; the go-to to each start runs at the approach duty, its
    /// windows priced. The spins' ends and their margins to the far guard,
    /// those of the order that ran, those left out, where it refused, and
    /// the envelope's peak rise and priced seconds.
    struct Sim {
        ends: Vec<(String, f64)>,
        ran: Vec<(f64, i8)>,
        left_out: Vec<(f64, i8)>,
        refused: Option<String>,
        rise_c: f64,
        priced_s: f64,
        /// The spins whose pot speed read over their bound speed.
        fast: Vec<String>,
    }

    fn simulate(k: f64, step: f64) -> Sim {
        simulate_on(k, step, |_, _| 1.0)
    }

    /// [`simulate`] behind a pot that reads `pot(level, dir)` times the
    /// shaft's counts over the stretch a spin crosses.
    fn simulate_on(k: f64, step: f64, pot: fn(f64, i8) -> f64) -> Sim {
        let travel = Travel {
            lo: 82.0,
            hi: 3792.0,
        };
        let rated = 60.0 / 0.1 * ADC_COUNTS / 184.3328808989794;
        let (tau, coast_k) = (0.020, 1e-5);
        let mut env = envelope();
        let mut plan = Planner::new(travel, rated);
        let mut out = Sim {
            ends: Vec::new(),
            ran: Vec::new(),
            left_out: Vec::new(),
            refused: None,
            rise_c: 0.0,
            priced_s: 0.0,
            fast: Vec::new(),
        };
        let (rise, priced) = (Cell::new(0.0f64), Cell::new(0.0f64));
        let admit = |env: &mut Envelope, r: &Rung| -> Result<(), String> {
            let p = env.admit(r).map_err(|e| e.0)?;
            rise.set(rise.get().max(p.rise_c));
            priced.set(priced.get() + r.time_s + r.gap_s);
            Ok(())
        };
        let speed = |level: f64| {
            let i = LEVELS.iter().position(|l| *l == level).unwrap_or(0);
            k * rated * level * step.powi(i as i32)
        };
        // the go-to at the stops step's approach duty from the probe's
        // 0.48 V and one step, its windows priced as the step prices them
        let app = 0.48 + 0.04 * 4.8;
        let app_q = level_q15(app / 4.8, 4.8, 5.0);
        let mut pos = 2041.0f64;
        let mut drive = |w: &Window, name: &str, env: &mut Envelope| -> Result<SpinRead, String> {
            let v_app = k * rated * app / 4.8;
            let windows = ((w.start - pos).abs() / (v_app * WINDOW_S)).ceil().max(1.0) as usize;
            for _ in 0..windows {
                let r = Rung {
                    name: "a go-to window".into(),
                    duty: (w.start - pos).signum() * app_q as f64 / Q15,
                    time_s: WINDOW_S + POLL_DEADMAN_S,
                    permit: false,
                    gap_s: 0.0,
                    start: pos,
                    speed_cps: rated * app / 4.8,
                    stop_counts: 0.0,
                    kind: Kind::Drive,
                };
                let stops = env.facts.stops.take();
                let got = admit(env, &r);
                env.facts.stops = stops;
                got?;
            }
            let q = level_q15(w.level, 4.8, 5.0) * w.dir as i16;
            admit(env, &spin_rung(name, q, w))?;
            let v = speed(w.level);
            let t = w.t_s + DRIVE_DEADMAN_S;
            let g = pot(w.level, w.dir);
            let x = g * v * (t - tau * (1.0 - (-t / tau).exp()));
            let v_cut = v * (1.0 - (-t / tau).exp());
            let coast = g * coast_k * v_cut * v_cut;
            let end = w.start + w.dir as f64 * (x + coast);
            pos = end;
            let (glo, ghi) = travel.guards();
            let margin = if w.dir > 0 { ghi - end } else { end - glo };
            out.ends.push((name.into(), margin));
            if g * v_cut > w.bound_speed {
                out.fast.push(name.into());
            }
            Ok(SpinRead {
                start: w.start,
                end,
                drive_travel: x,
                travel: x + coast,
                speed_cps: g * v_cut,
                t90_s: Some(tau * 10f64.ln()),
                cut_pos: end - w.dir as f64 * coast,
                coast_counts: coast,
                coast_s: Some(0.1),
            })
        };
        let w = plan.pilot(travel.start(1));
        let r = match drive(&w, "the pilot", &mut env) {
            Ok(r) => r,
            Err(e) => {
                out.refused = Some(e);
                return out;
            }
        };
        if let Err(e) = after(&w, &r, &travel, "the pilot") {
            out.refused = Some(e);
            return out;
        }
        plan.feed_pilot(r.seen(PILOT_LEVEL, 1), r.t90_s.expect("rise"));
        for s in runway::order() {
            if out.left_out.contains(&(s.level, s.dir)) {
                continue;
            }
            let w = plan.window(s.level, s.dir, travel.start(s.dir));
            if !w.fits() {
                out.left_out.push((s.level, s.dir));
                continue;
            }
            let name = format!("spin {} ({:.0}% {})", s.n, 100.0 * s.level, way(s.dir));
            let r = match drive(&w, &name, &mut env) {
                Ok(r) => r,
                Err(e) => {
                    out.refused = Some(e);
                    return out;
                }
            };
            if let Err(e) = after(&w, &r, &travel, &name) {
                out.refused = Some(e);
                return out;
            }
            plan.feed(r.seen(s.level, s.dir));
            out.ran.push((s.level, s.dir));
        }
        out.rise_c = rise.get();
        out.priced_s = priced.get();
        out
    }

    fn said(k: f64, step: f64, s: &Sim) -> f64 {
        let worst = s.ends.iter().map(|(_, m)| *m).fold(f64::INFINITY, f64::min);
        println!(
            "{k} x the nameplate, {step} x the volts' ratio a level: {} spins, the least margin to the far guard {worst:.0} counts, ran {:?}, left out {:?}, refused: {}; peak priced rise {:.2} C, {:.2} s priced",
            s.ends.len(),
            s.ran,
            s.left_out,
            s.refused.as_deref().unwrap_or("no"),
            s.rise_c,
            s.priced_s
        );
        for (name, m) in &s.ends {
            println!("  {name}: {m:.0} counts short of the far guard");
        }
        worst
    }

    /// At 0.6 times the nameplate's speed, the bench servo's case, every
    /// level runs both ways and J is read at the top; at 1.0 times the top
    /// level's coast bound leaves it no runway, J comes from the 50 %
    /// level with a warning; at 1.5 times the pilot runs past its priced
    /// travel and the step refuses before any other spin. No spin's travel, at
    /// its T and its dead-man, passes the far guard; every one that ran
    /// ends its SHORT_COUNTS short or more.
    #[test]
    fn a_simulated_run_never_passes_the_far_guard() {
        for k in [0.6, 1.0, 1.5] {
            let s = simulate(k, 1.0);
            let worst = said(k, 1.0, &s);
            assert!(worst > 0.0, "{k}: {worst}");
            let jl = j_level(s.ran.iter().copied());
            match k {
                0.6 => {
                    assert!(s.refused.is_none(), "{:?}", s.refused);
                    assert!(
                        s.left_out.is_empty() && s.ran.len() == 14,
                        "{:?}",
                        s.left_out
                    );
                    let top = s.ran.iter().filter(|r| r.0 == 0.75);
                    assert_eq!(top.count(), 2 * runway::TOP_SPINS_EACH);
                    assert_eq!(jl, Some(0.75));
                    assert!(worst >= SHORT_COUNTS, "{worst}");
                }
                1.0 => {
                    assert!(s.refused.is_none(), "{:?}", s.refused);
                    assert!(worst >= SHORT_COUNTS, "{worst}");
                    assert_eq!(s.left_out, [(0.75, 1), (0.75, -1)]);
                    assert_eq!(jl, Some(0.5));
                    let ck = j_level_check(jl);
                    assert_eq!(ck.verdict, Verdict::Warn, "{}", ck.said());
                    assert!(
                        ck.value
                            .contains("75% level was left out by the runway rule"),
                        "{}",
                        ck.said()
                    );
                }
                _ => {
                    let why = s.refused.expect("refused");
                    assert!(
                        why.contains("the pilot ran past its priced travel"),
                        "{why}"
                    );
                    assert_eq!(s.ends.len(), 1, "after the pilot");
                }
            }
        }
    }

    /// A servo whose speed rises 60 % faster than the volts' ratio from a
    /// level to the next runs its 50 % spins over the speed the 25 % level
    /// predicts at the 1.5 margin, but inside the travel they were priced
    /// at: they are accepted, the 75 % level is bounded by the line
    /// through the speeds the run read, and no spin comes within
    /// SHORT_COUNTS of the far guard.
    #[test]
    fn a_servo_faster_than_the_volts_ratio_inside_its_price_runs_on() {
        let s = simulate(0.6, 1.6);
        let worst = said(0.6, 1.6, &s);
        assert!(s.refused.is_none(), "{:?}", s.refused);
        assert!(
            s.fast.iter().any(|f| f.starts_with("spin 5")),
            "{:?}",
            s.fast
        );
        assert!(
            s.ran.contains(&(0.5, 1)) && s.ran.contains(&(0.5, -1)),
            "{:?}",
            s.ran
        );
        assert!(worst >= SHORT_COUNTS, "{worst}");
    }

    /// A servo whose speed rises 2.2 times the volts' ratio from a level to
    /// the next runs its first 50 % spin past the travel the 25 % level's
    /// spins priced: the check after it ends the step there, that spin's
    /// travel still short of the far guard.
    #[test]
    fn a_servo_far_faster_than_the_volts_ratio_is_stopped_after_its_first_spin() {
        let s = simulate(0.6, 2.2);
        let worst = said(0.6, 2.2, &s);
        let why = s.refused.as_deref().expect("refused");
        assert!(
            why.contains("spin 5 (50% up) ran past its priced travel"),
            "{why}"
        );
        assert!(
            why.contains("the 25% level's 3 spins up predict, past the 1.5 margin"),
            "{why}"
        );
        assert_eq!(s.ran.len(), 4, "the 25 % level ran: {:?}", s.ran);
        let (last, m) = s.ends.last().expect("a spin");
        assert!(last.starts_with("spin 5") && *m > 0.0, "{last}: {m}");
        assert!(worst > 0.0);
    }

    /// The pot reads 1.3x steep over the stretch the 75 % spins down cross
    /// (the bench servo's reads 1.24x over its upper stretch), the shaft's
    /// speed in proportion to the volts: their pot speed reads over the
    /// 1.25 margin on the line through the 25 % and 50 % levels, their
    /// travel inside its price. Every level runs both ways, none within
    /// SHORT_COUNTS of the far guard.
    #[test]
    fn a_spin_read_fast_by_a_steep_pot_stretch_inside_its_price_runs_on() {
        let pot = |level: f64, dir: i8| if level == 0.75 && dir < 0 { 1.3 } else { 1.0 };
        let s = simulate_on(0.6, 1.0, pot);
        let worst = said(0.6, 1.0, &s);
        assert!(s.refused.is_none(), "{:?}", s.refused);
        assert!(
            s.fast.iter().any(|f| f.contains("75% down")),
            "{:?}",
            s.fast
        );
        assert!(
            s.left_out.is_empty() && s.ran.len() == 14,
            "{:?}",
            s.left_out
        );
        assert!(worst >= SHORT_COUNTS, "{worst}");
    }
}
