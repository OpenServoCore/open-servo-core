//! The cruise step: the running friction line from closed-loop Velocity
//! cruises at mid travel, on the installed, stamped set, the line the
//! velocity loop's friction feed-forward has to match ([`crate::fit::cruise`]).
//! It changes no constant; it reads, cruises and fits.
//!
//! 1. First enable as the closed-loop check's: Position mode, the goal the
//!    servo's own estimate, the dead-man before torque on; [`ENABLE_S`]
//!    captured and judged.
//! 2. Each speed of [`SPEEDS`], the lowest first: a Position move to the
//!    speed's start, then Velocity mode, a poll, and [`LEGS_EACH`] legs
//!    each way, up first, each goal committed with its leg's burst and a
//!    poll after it (the rail, the winding estimate and the current bias
//!    the replay reads); then a zero goal with a burst of the stop, and
//!    Position mode back at the estimate.
//! 3. The goal back to the middle of the step's bound, held until it
//!    arrives, then the way out.
//!
//! Every leg is sized and centered from the measured travel: the speed's
//! legs, back to back, at the host's worst gap between bursts, stay inside
//! the span [`GUARD_COUNTS`] in from each stop and clear of the soft
//! limits' wall ramp, centered on that span, else on the travel's middle;
//! a leg that does not fit is shortened down to [`LEG_MIN_SHARE`] of its
//! length, else the speed is left out, and said. Before each leg after the
//! first, the leg and its stop are predicted again from the last row of
//! the leg before, and the host's time since it is read off its clock; a
//! leg that would leave the span, or a gap over [`GAP_S`], ends the speed
//! early. The step's own bound is the soft range where an application
//! narrowed the travel, else [`STOP_CLEAR_COUNTS`] inside the stops: soft
//! limits on the stops, or 0 / 0, off, are a servo's normal state.
//!
//! The legs run under a [`VELOCITY_DEADMAN_S`] dead-man, the Position
//! phases under the rest step's longer one. A host lost mid-leg leaves the
//! burst to run out, then the countdown: every dispatched instruction
//! reloads it, a streaming burst holds it, and at zero the firmware latches
//! the `deadman` fault (kernel sec 2.7) and from the next tick the drive
//! coasts, bridge off, until the torque enable ack. Until it lapses the
//! Velocity goal holds, the reference capped by the wall ramp of the soft
//! limit it points at. Each speed is planned so that a host lost as any
//! leg ends leaves the shaft [`STOP_CLEAR_COUNTS`] inside the stops after
//! the gap, the dead-man and the coast ([`Geometry::runaway`]), or it is
//! shortened or left out.
//!
//! The firmware's protections stay in force; no stall permit is written.
//! The host's watchdog judges every burst's rows and every poll: a fault,
//! the shaft outside the span, the current over a quarter past the cap.
//! On an error once the first enable's check has passed, with the loop
//! still trusted (no fault, no watchdog trip), the shaft is brought to the
//! middle before the way out; before it, torque goes off where it is.

use std::time::Duration;

use osc_client::data_state::{self, fault};
use osc_client::pipe::Pipe;
use serde_json::{Map, Value, json};

use super::runway::GUARD_COUNTS;
use super::verify::{
    self, CAP_GAIN, CENTER_POLLS, CENTER_STILL, CENTER_TOL, EDGE_SHARE, ENABLE_S, POLL_S, Scale,
    Watch,
};
use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_alive, keep_all, mode_value,
    read_u8, rest, write,
};
use crate::Board;
use crate::capture::meta;
use crate::capture::rows::Layout;
use crate::capture::tel::{self, Line, StreamCfg};
use crate::envelope::DRIVE_DEADMAN_S;
use crate::fit::cruise::{self as fit, Leg, LegRead, Point, Reading};
use crate::fit::window::Settle;
use crate::guard::{self, Ledger, Way};
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::scorecard::{Lin, enable_check};
use crate::state::{State, field};

pub const STEP: &str = "cruise";
pub const RESULT: &str = "results/cruise.json";
const METHOD: &str = "closed-loop-velocity-cruises";
const REVISION: u32 = 2;

/// The cruise speeds, shares of the speed ceiling (`speed_ceiling_cps`),
/// each capped at the move speed the set allows, and each one's leg, s:
/// 322, 805, 1530 and 4025 counts/s on the mg90-d's ceiling of 8050 (the
/// record's 300, 800, 1500 and 4025 to two figures), its legs the
/// record's.
pub const SPEEDS: [(f64, f64); 4] = [(0.04, 1.2), (0.1, 1.2), (0.19, 0.9), (0.5, 0.6)];
pub const LEGS_EACH: usize = 2;
/// The host's longest time between one leg's burst and the next goal's
/// commit, s: recorded cruises started each leg 7 to 20 ms past the leg
/// before ran out. The plan keeps a speed's drift at this gap inside its
/// span; a longer gap ends the speed.
pub const GAP_S: f64 = 0.03;
/// A leg shortened under this share of its length is no leg: the speed
/// is left out.
pub const LEG_MIN_SHARE: f64 = 0.5;
/// The stop's burst holds this past the deceleration, s.
pub const STOP_HOLD_S: f64 = 0.1;
/// The firmware's soft-limit wall ramp: a Velocity goal toward a soft
/// limit is capped at 16 counts/s per count of the estimate's distance to
/// it (core trajectory `WALL_RAMP_SHIFT`). A leg stays clear of it.
pub const WALL_CPS_PER_COUNT: f64 = 16.0;
/// The dead-man the Velocity legs run under, s. Every instruction the bus
/// dispatches reloads it and a streaming burst holds it, so it lapses only
/// after this long with neither: past [`GAP_S`], the longest the host is
/// silent between legs, by a margin of the same size. The Position
/// phases poll every [`POLL_S`] and run under the rest step's longer one.
pub const VELOCITY_DEADMAN_S: f64 = DRIVE_DEADMAN_S;
/// The step's bound lies at least this far inside the stops, and the
/// worst case of a host lost after any leg ends at least this far inside
/// them: the wall ramp stops the reference at a soft limit only, so where
/// the soft limits sit on the stops the plan alone keeps the room.
pub const STOP_CLEAR_COUNTS: f64 = 100.0;
/// A stops result for the travel agrees with the installed set's raw_min
/// and raw_max to this share of the set's span, or it is another servo's
/// or an older seat's.
pub const STOPS_AGREE_SHARE: f64 = 0.03;

/// What the step reads besides the servo.
pub struct CruiseInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    /// The stops step's result whose stops are the travel; None takes the
    /// stops the installed set holds.
    pub stops: Option<&'a Value>,
}

#[derive(Clone, Debug)]
pub struct Cruised {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Cruised {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// What the plan is sized by, linearized counts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Geometry {
    /// The measured stops.
    pub stops: (f64, f64),
    pub soft: (f64, f64),
    /// The speed the shares are of, the set's speed ceiling, and the move
    /// speed the set allows, the most a cruise asks.
    pub ceiling_cps: f64,
    pub move_cps: f64,
    /// The profile's acceleration, counts/s a MEDIUM tick, at the MEDIUM
    /// rate.
    pub accel_per_tick: f64,
    pub medium_hz: f64,
    /// The coast's deceleration once the dead-man has cut the drive,
    /// counts/s^2: the set's Coulomb friction over its inertia, the
    /// viscous part left out so it errs long.
    pub coast_cps2: f64,
    /// The Velocity legs' dead-man, s.
    pub deadman_s: f64,
}

impl Geometry {
    pub fn middle(&self) -> f64 {
        (self.stops.0 + self.stops.1) / 2.0
    }

    /// Where a leg at `cps` may plan: [`GUARD_COUNTS`] in from each stop,
    /// clear of the soft limits' wall ramp.
    pub fn span(&self, cps: f64) -> (f64, f64) {
        let wall = cps / WALL_CPS_PER_COUNT;
        (
            (self.stops.0 + GUARD_COUNTS).max(self.soft.0 + wall),
            (self.stops.1 - GUARD_COUNTS).min(self.soft.1 - wall),
        )
    }

    /// Where a lost host may leave the shaft: [`STOP_CLEAR_COUNTS`] in
    /// from each stop.
    pub fn clear(&self) -> (f64, f64) {
        (
            self.stops.0 + STOP_CLEAR_COUNTS,
            self.stops.1 - STOP_CLEAR_COUNTS,
        )
    }

    /// The stop's burst after a speed's last leg, s.
    pub fn stop_s(&self, cps: f64) -> f64 {
        cps / (self.accel_per_tick * self.medium_hz) + STOP_HOLD_S
    }

    /// Where the shaft ends when the host is lost as a leg ends at `p`
    /// with the reference at `v`: the goal holds through the host's last
    /// gap and the dead-man and one MEDIUM period past it, the reference
    /// slewed toward it and capped by the wall ramp of the soft limit it
    /// points at, as the firmware's profile does; then the dead-man
    /// latches, the drive coasts from the reference's speed at
    /// [`Geometry::coast_cps2`].
    pub fn runaway(&self, p: f64, v: f64) -> f64 {
        let dt = 1.0 / self.medium_hz;
        let n = ((GAP_S + self.deadman_s) * self.medium_hz).ceil() as usize + 1;
        let (mut p, mut r, goal) = (p, v, v);
        for _ in 0..n {
            let g = if goal >= 0.0 {
                goal.min(WALL_CPS_PER_COUNT * (self.soft.1 - p).max(0.0))
            } else {
                goal.max(-WALL_CPS_PER_COUNT * (p - self.soft.0).max(0.0))
            };
            r += (g - r).clamp(-self.accel_per_tick, self.accel_per_tick);
            p += r * dt;
        }
        p + r.signum() * r * r / (2.0 * self.coast_cps2)
    }
}

/// The reference's path through `legs` (goal counts/s, s), slewed at the
/// profile's acceleration each MEDIUM tick, from `from` at `v0`: its
/// lowest and highest position, and where each leg ends and the
/// reference's speed there.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub lo: f64,
    pub hi: f64,
    pub ends: Vec<(f64, f64)>,
}

pub fn path(g: &Geometry, from: f64, v0: f64, legs: &[(f64, f64)]) -> Path {
    let dt = 1.0 / g.medium_hz;
    let (mut p, mut v) = (from, v0);
    let (mut lo, mut hi) = (p, p);
    let mut ends = Vec::new();
    for &(goal, s) in legs {
        let n = (s * g.medium_hz).round() as usize;
        for _ in 0..n {
            v += (goal - v).clamp(-g.accel_per_tick, g.accel_per_tick);
            p += v * dt;
            lo = lo.min(p);
            hi = hi.max(p);
        }
        ends.push((p, v));
    }
    Path { lo, hi, ends }
}

/// A speed's legs, up first, the up ones `t_up` and the down ones
/// `t_down`, then the stop.
fn legs_of(g: &Geometry, cps: f64, t_up: f64, t_down: f64) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = (0..2 * LEGS_EACH)
        .map(|k| {
            if k % 2 == 0 {
                (cps, t_up)
            } else {
                (-cps, t_down)
            }
        })
        .collect();
    v.push((0.0, g.stop_s(cps)));
    v
}

/// A speed as planned.
#[derive(Clone, Debug, PartialEq)]
pub struct Planned {
    pub share: f64,
    pub cps: f64,
    /// The leg, s, and the length [`SPEEDS`] gives it.
    pub t_s: f64,
    pub t_full_s: f64,
    /// Where the speed starts from rest.
    pub start: f64,
    /// The positions its legs reach at the host's worst gaps, and the span
    /// they stay inside.
    pub extent: (f64, f64),
    pub span: (f64, f64),
    /// The furthest a host lost after one of its legs leaves the shaft,
    /// each way, inside [`Geometry::clear`].
    pub runaway: (f64, f64),
}

impl Planned {
    pub fn shortened(&self) -> bool {
        self.t_s < self.t_full_s
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LeftOut {
    pub share: f64,
    pub cps: f64,
    pub why: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub speeds: Vec<Planned>,
    pub left_out: Vec<LeftOut>,
}

/// Where a path's legs but the last (the stop) leave the shaft when the
/// host is lost as each ends, lowest and highest.
fn runaways(g: &Geometry, at: &Path) -> (f64, f64) {
    let n = at.ends.len().saturating_sub(1);
    at.ends[..n]
        .iter()
        .map(|&(p, v)| g.runaway(p, v))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| {
            (a.min(x), b.max(x))
        })
}

/// The speed at `cps` with legs of `t`, placed: its extent at the host's
/// worst gaps, every up leg long or every down leg, centered on the span
/// the speed may use, else on the travel's middle; the first center whose
/// extent stays inside the span and whose lost-host runaways stay clear of
/// the stops. Why not, the span-centered try's reason.
fn place(g: &Geometry, cps: f64, t: f64) -> Result<Planned, String> {
    let high = path(g, 0.0, 0.0, &legs_of(g, cps, t + GAP_S, t));
    let low = path(g, 0.0, 0.0, &legs_of(g, cps, t, t + GAP_S));
    let (lo, hi) = (low.lo.min(high.lo), high.hi.max(low.hi));
    let span = g.span(cps);
    let clear = g.clear();
    let mut first: Option<String> = None;
    for centre in [(span.0 + span.1) / 2.0, g.middle()] {
        let start = centre - (lo + hi) / 2.0;
        let extent = (start + lo, start + hi);
        let why = if extent.0 < span.0 || extent.1 > span.1 {
            Err(format!(
                "its legs reach {:.0}..{:.0} at the host's worst gaps, past the span {:.0}..{:.0}",
                extent.0, extent.1, span.0, span.1
            ))
        } else {
            let worst = [
                path(g, start, 0.0, &legs_of(g, cps, t + GAP_S, t)),
                path(g, start, 0.0, &legs_of(g, cps, t, t + GAP_S)),
            ]
            .iter()
            .map(|at| runaways(g, at))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), (x, y)| {
                (a.min(x), b.max(y))
            });
            if worst.0 < clear.0 || worst.1 > clear.1 {
                Err(format!(
                    "a host lost as a leg ends leaves the shaft at {:.0}..{:.0} after the dead-man and the coast, past {:.0}..{:.0}",
                    worst.0, worst.1, clear.0, clear.1
                ))
            } else {
                Ok(Planned {
                    share: cps / g.ceiling_cps,
                    cps,
                    t_s: t,
                    t_full_s: t,
                    start,
                    extent,
                    span,
                    runaway: worst,
                })
            }
        };
        match why {
            Ok(p) => return Ok(p),
            Err(e) => {
                first.get_or_insert(e);
            }
        }
    }
    Err(first.unwrap_or_default())
}

/// The plan on `g`: each speed of [`SPEEDS`] at its leg, shortened to fit
/// its span and the stops' clearance, or left out, said why.
pub fn plan(g: &Geometry) -> Plan {
    let mut out = Plan {
        speeds: Vec::new(),
        left_out: Vec::new(),
    };
    for (share, t_full) in SPEEDS {
        let cps = (share * g.ceiling_cps).round().min(g.move_cps).max(1.0);
        let placed = match place(g, cps, t_full) {
            Ok(p) => Ok(p),
            Err(_) => match place(g, cps, LEG_MIN_SHARE * t_full) {
                Err(why) => Err(format!(
                    "no {:.0} ms legs fit at {cps:.0} counts/s, {:.0}% of the {:.0} ms leg: {why}",
                    1e3 * LEG_MIN_SHARE * t_full,
                    100.0 * LEG_MIN_SHARE,
                    1e3 * t_full
                )),
                Ok(_) => {
                    let (mut a, mut b) = (LEG_MIN_SHARE * t_full, t_full);
                    for _ in 0..30 {
                        let m = (a + b) / 2.0;
                        if place(g, cps, m).is_ok() {
                            a = m;
                        } else {
                            b = m;
                        }
                    }
                    place(g, cps, (a * 1e3).floor() / 1e3)
                }
            },
        };
        match placed {
            Ok(p) => out.speeds.push(Planned {
                share,
                t_full_s: t_full,
                ..p
            }),
            Err(why) => out.left_out.push(LeftOut { share, cps, why }),
        }
    }
    out
}

/// Whether the next leg, its goal `goal` from the last row of the leg
/// before at `from` moving at `v_prev`, keeps inside the speed's span at
/// the worst gap and its stop after it, whether the next leg turns or
/// stops; and whether a host lost as it ends leaves the shaft clear of
/// the stops.
pub fn next_fits(
    g: &Geometry,
    p: &Planned,
    from: f64,
    v_prev: f64,
    goal: f64,
) -> Result<(), String> {
    let legs = [
        (v_prev, GAP_S),
        (goal, p.t_s + GAP_S),
        (0.0, g.stop_s(p.cps)),
    ];
    let at = path(g, from, v_prev, &legs);
    if at.lo < p.span.0 || at.hi > p.span.1 {
        return Err(format!(
            "from {from:.0} counts it would reach {:.0}..{:.0}, past the span {:.0}..{:.0}",
            at.lo, at.hi, p.span.0, p.span.1
        ));
    }
    let (end, v) = at.ends[1];
    let lost = g.runaway(end, v);
    let clear = g.clear();
    if lost < clear.0 || lost > clear.1 {
        return Err(format!(
            "from {from:.0} counts a host lost as it ends would leave the shaft at {lost:.0}, past {:.0}..{:.0}",
            clear.0, clear.1
        ));
    }
    Ok(())
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    stream: StreamCfg,
    lin: Lin,
    geometry: Geometry,
    plan: Plan,
    travel_from: String,
    reading: Reading,
    scale: Scale,
    gear_ratio: f64,
    pos: f64,
    tick_hz: f64,
    cap_counts: f64,
    deadband: f64,
    /// The Position phases' dead-man and the Velocity legs', ticks.
    deadman_ticks: u16,
    velocity_deadman_ticks: u16,
    position_mode: u8,
    velocity_mode: u8,
    /// The first enable's burst, and each planned speed's leg and stop
    /// bursts, rows.
    enable_rows: u16,
    bursts: Vec<(u16, u16)>,
    bias: f64,
    /// The middle of the step's bound, where the step ends.
    mid: f64,
    /// The rows and polls stay inside the widest span a speed plans in.
    watch_span: (f64, f64),
}

fn num(state: &State, d: &osc_client::descriptor::Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

fn rows_for(s: f64, tick_hz: f64) -> Result<u16, Refused> {
    let n = (s * tick_hz).ceil();
    if !(1.0..=u16::MAX as f64).contains(&n) {
        return Err(Refused(format!(
            "a {s:.3} s burst is no u16 of rows at {tick_hz} Hz: the set's acceleration or the plan asks for a burst the servo cannot stream"
        )));
    }
    Ok(n as u16)
}

fn ticks_for(s: f64, tick_hz: f64) -> Result<u16, Refused> {
    let n = (s * tick_hz).round();
    if !(1.0..=u16::MAX as f64).contains(&n) {
        return Err(Refused(format!(
            "a {s} s dead-man is no u16 of ticks at {tick_hz} Hz"
        )));
    }
    Ok(n as u16)
}

/// The travel the plan is sized by, raw pot counts: the stops step's
/// stops, or the set's, and its words. A stops result is refused unless
/// accepted and within [`STOPS_AGREE_SHARE`] of the set's.
fn travel_raw(i: &CruiseInputs, set: (f64, f64)) -> Result<((f64, f64), String), Refused> {
    let Some(v) = i.stops else {
        return Ok((
            set,
            "the stops the installed set holds (raw_min, raw_max)".into(),
        ));
    };
    if v["verdict"] != "accepted" {
        return Err(Refused(format!(
            "the stops step's result is {}, not accepted: run the stops step again",
            v["verdict"].as_str().unwrap_or("of no verdict")
        )));
    }
    let n = |x: &Value, what: &str| {
        x.as_f64()
            .filter(|v| v.is_finite())
            .ok_or_else(|| Refused(format!("the stops result holds no {what}")))
    };
    let stops = (
        n(&v["stops"]["low"]["median_counts"], "low stop")?,
        n(&v["stops"]["high"]["median_counts"], "high stop")?,
    );
    let tol = STOPS_AGREE_SHARE * (set.1 - set.0);
    if (stops.0 - set.0).abs() > tol || (stops.1 - set.1).abs() > tol {
        return Err(Refused(format!(
            "the stops result's stops {:.0}..{:.0} are not the installed set's raw_min..raw_max {:.0}..{:.0} (within {:.0} counts, {:.0}% of its span): the run folder is another servo's or an older seat's; run the stops step on this servo, or leave --run out",
            stops.0,
            stops.1,
            set.0,
            set.1,
            tol,
            100.0 * STOPS_AGREE_SHARE
        )));
    }
    Ok((stops, "the stops step's stops".into()))
}

async fn read_word<P: Pipe>(rig: &mut Rig<'_, P>, name: &str) -> Result<u16, Refused> {
    let f = field(rig.d, name)?;
    match rig.c.read(rig.id, f.addr, 2).await?[..] {
        [lo, hi, ..] => Ok(u16::from_le_bytes([lo, hi])),
        _ => Err(Refused(format!("the read of {name} came back short"))),
    }
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &CruiseInputs<'_>) -> Result<Before, Refused> {
    let (line, state) = verify::admit(rig.c, rig.id, rig.d).await?;
    if state.stamp.stored == osc_client::stamp::UNSTAMPED || !state.stamp.matches() {
        return Err(Refused(format!(
            "the stamp says {}: the live set is not the one installed and stamped, and the closed loop would run on constants that do not belong together; install the set (`osc-ident install`) first",
            state.stamp_said()
        )));
    }
    if !data_state::allows(state.data.flags, true) {
        return Err(Refused(format!(
            "the servo's data state is {}: it refuses the closed loops, so no cruise runs",
            state.data_said()
        )));
    }
    let mode = |name: &str| mode_value(rig.d, name).map_err(|h| Refused(h.why()));
    let (position_mode, velocity_mode) = (mode("Position")?, mode("Velocity")?);
    let n = |name: &str| num(&state, rig.d, name);
    let held = verify::travel(&state, rig.d)?;
    let soft = (
        held.pos_min_soft_counts as f64,
        held.pos_max_soft_counts as f64,
    );
    if soft.1 <= soft.0 {
        return Err(Refused(format!(
            "the soft range {}..{} is empty",
            soft.0, soft.1
        )));
    }
    let cap = n("current_limit_counts")?;
    let ceiling = n("speed_ceiling_cps")?;
    if cap <= 0.0 || ceiling <= 0.0 {
        return Err(Refused(
            "the set holds no current cap or no speed ceiling: the servo refuses every closed mode"
                .into(),
        ));
    }
    let v = n("velocity_limit_cps")?;
    let move_cps = if v == 0.0 { ceiling } else { v.min(ceiling) };
    let accel = n("accel_limit_q88")? / 256.0;
    if accel <= 0.0 {
        return Err(Refused(
            "the set holds no acceleration limit (accel_limit_q88 0): a Velocity goal would never move the shaft".into(),
        ));
    }
    let j_ff = n("j_ff_q88")?;
    if j_ff <= 0.0 {
        return Err(Refused(
            "the set holds no inertia word (j_ff_q88 0): the coast after a lapsed dead-man cannot be bounded, so no leg can be sized; run the spin step and gather".into(),
        ));
    }
    let fc = n("fric_fc_counts")?;
    if fc <= 0.0 {
        return Err(Refused(
            "the set holds no friction word (fric_fc_counts 0): the coast after a lapsed dead-man cannot be bounded, so no leg can be sized; run the spin step and gather".into(),
        ));
    }
    let gear_ratio = i
        .nameplate
        .gear_ratio
        .filter(|g| g.is_finite() && *g > 0.0)
        .ok_or_else(|| {
            Refused(
                "the nameplate has no usable gear_ratio: the line's speed is per motor rad/s"
                    .into(),
            )
        })?;
    let w = i.board.window_read.as_ref().ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) holds no crest read (window_read): the cruises' current cannot be read as the firmware reads it",
            i.board.name
        ))
    })?;
    let bridge_min = i.board.bridge_min_ticks.ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) holds no bridge_min_ticks: the replay needs the bridge's shortest pulse",
            i.board.name
        ))
    })?;
    let lin = Lin::new(state.lut.effective(), state.slots(rig.d).as_deref());
    let set_raw = (n("raw_min")?, n("raw_max")?);
    let (raw, travel_from) = travel_raw(i, set_raw)?;
    let at = |r: f64| lin.counts(r.round().clamp(0.0, u16::MAX as f64) as u16);
    let stops = (at(raw.0), at(raw.1));
    if stops.1 <= stops.0 {
        return Err(Refused(format!(
            "the stops {:.0}..{:.0} ({travel_from}) leave no travel",
            stops.0, stops.1
        )));
    }
    let bound = (
        soft.0.max(stops.0 + STOP_CLEAR_COUNTS),
        soft.1.min(stops.1 - STOP_CLEAR_COUNTS),
    );
    if bound.1 <= bound.0 {
        return Err(Refused(format!(
            "the soft range {:.0}..{:.0} leaves nothing {STOP_CLEAR_COUNTS:.0} counts inside the stops {:.0}..{:.0} ({travel_from})",
            soft.0, soft.1, stops.0, stops.1
        )));
    }
    let scale = match verify::scale(
        set_raw,
        (n("angle_min_cdeg")?, n("angle_max_cdeg")?),
        0.0,
        None,
    ) {
        Some(s) => s,
        None => {
            let deg = i.nameplate.travel_deg.filter(|d| *d > 0.0).ok_or_else(|| {
                Refused("the servo stores no angle map and the nameplate states no travel_deg: no speed to read in motor rad/s".into())
            })?;
            Scale {
                counts_per_deg: (stops.1 - stops.0) / deg,
                source: format!(
                    "the servo stores no angle map: the nameplate's {deg} deg travel over the {:.0} count stops ({travel_from})",
                    stops.1 - stops.0
                ),
            }
        }
    };
    let sense = rest::Sense::of(&state, rig.d)?;
    let tick_hz = sense.tick_hz;
    let medium_hz = tick_hz / 10.0;
    let geometry = Geometry {
        stops,
        soft,
        ceiling_cps: ceiling,
        move_cps,
        accel_per_tick: accel,
        medium_hz,
        coast_cps2: fc / (j_ff / 256.0) * medium_hz,
        deadman_s: VELOCITY_DEADMAN_S,
    };
    let plan = plan(&geometry);
    if plan.speeds.is_empty() {
        return Err(Refused(format!(
            "the travel {:.0}..{:.0} ({travel_from}) fits no cruise: {}",
            stops.0,
            stops.1,
            plan.left_out
                .iter()
                .map(|l| format!("{:.0} counts/s: {}", l.cps, l.why))
                .collect::<Vec<_>>()
                .join("; ")
        )));
    }
    let enable_rows = rows_for(ENABLE_S, tick_hz)?;
    let bursts = plan
        .speeds
        .iter()
        .map(|p| {
            Ok((
                rows_for(p.t_s, tick_hz)?,
                rows_for(geometry.stop_s(p.cps), tick_hz)?,
            ))
        })
        .collect::<Result<Vec<_>, Refused>>()?;
    let deadman_ticks = ticks_for(rest::DEADMAN_S, tick_hz)?;
    let velocity_deadman_ticks = ticks_for(VELOCITY_DEADMAN_S, tick_hz)?;
    let pos = verify::theta_of(rig.c, rig.id, rig.d).await? as f64;
    let edge = EDGE_SHARE * (bound.1 - bound.0);
    if pos < bound.0 + edge || pos > bound.1 - edge {
        return Err(Refused(format!(
            "the shaft reads {pos:.0} counts, within {:.0}% of the travel ({edge:.0} counts) of its bound ({:.0}..{:.0}): turn the horn toward the middle with torque off",
            100.0 * EDGE_SHARE,
            bound.0,
            bound.1
        )));
    }
    let bias = read_word(rig, "current_bias_counts").await? as f64;
    let r_hat = read_word(rig, "r_hat_q12").await?;
    if r_hat == 0 {
        return Err(Refused(
            "the servo reads no winding resistance (r_hat_q12 0): the cruises' current reference cannot be replayed".into(),
        ));
    }
    let word_of = |name: &str| -> Result<u16, Refused> {
        let x = n(name)?;
        if (0.0..=u16::MAX as f64).contains(&x) {
            Ok(x as u16)
        } else {
            Err(Refused(format!("the set's {name} {x} is no u16")))
        }
    };
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    let watch_span = plan
        .speeds
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), p| {
            (a.min(p.span.0), b.max(p.span.1))
        });
    let watch_span = (
        watch_span.0.min(bound.0 + edge),
        watch_span.1.max(bound.1 - edge),
    );
    Ok(Before {
        servo: meta::servo(&state, rig.d, &line),
        line,
        stream,
        reading: Reading {
            counts_per_a: sense.counts_per_a(),
            g_window_q016: word_of("g_window_q016")?,
            i_kp_q88: word_of("i_kp_q88")?,
            i_ki_q412: word_of("i_ki_q412")?,
            ke_vpc_q: word_of("ke_vpc_q")?,
            r_hat_q12: r_hat,
            j_ff_q88: j_ff,
            fric_fc_counts: fc,
            fric_fv_q016: n("fric_fv_q016")?,
            pwm_arr: word_of("pwm_period_ticks")?,
            tick_hz,
            bridge_min_ticks: bridge_min,
            settle: Settle {
                start_ticks: w.settle_start_ticks,
                q15: w.settle_q15.clone(),
            },
            counts_per_motor_rad: scale.counts_per_deg * 180.0 / core::f64::consts::PI / gear_ratio,
            ceiling_cps: move_cps,
        },
        lin,
        geometry,
        plan,
        travel_from,
        scale,
        gear_ratio,
        pos,
        tick_hz,
        cap_counts: cap,
        deadband: n("pos_deadband_counts")?,
        deadman_ticks,
        velocity_deadman_ticks,
        position_mode,
        velocity_mode,
        enable_rows,
        bursts,
        bias,
        mid: (bound.0 + (bound.1 - bound.0) / 2.0).floor(),
        watch_span,
    })
}

/// A leg as taken: its speed in the plan, goal, the capture it is, its
/// planned rows and the polls before and after it.
#[derive(Clone, Debug)]
struct TakenLeg {
    speed: usize,
    goal: f64,
    k: usize,
    planned_rows: usize,
    before: fit::Poll,
    after: fit::Poll,
}

/// What the body found, as far as it got.
#[derive(Default)]
struct Found {
    goal0: Option<f64>,
    enable: Option<Check>,
    legs: Vec<TakenLeg>,
    /// The winding estimate of the first poll beside a leg, centi-C: the
    /// temperature the start's resistance is followed from.
    t_read_cc: Option<f64>,
    /// A speed ended early: its cps, the leg, why.
    cut_short: Vec<(f64, usize, String)>,
    torque_on: bool,
    /// The first enable's check passed: the loop holds the shaft and may be
    /// trusted to bring it back after an error.
    trusted: bool,
    /// Set when the loop is no longer trusted to bring the shaft back: a
    /// fault, or the watchdog.
    no_recover: Option<String>,
    fault: Option<String>,
    centered: Option<f64>,
    recovered: Option<String>,
}

fn fault_said(flags: u8, code: u8) -> String {
    format!(
        "fault_flags {flags:#04x}, fault_code {code} ({})",
        fault::name(code).unwrap_or("unknown")
    )
}

/// One burst of `rows`, the write `held` committed with it, kept in
/// `taken`: its place there, a fault it latched, said, and the host's
/// clock as its last row came in. A burst that lost a row is no evidence
/// and its goal cannot be taken again: the step aborts.
async fn shot<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    taken: &mut Vec<Taken>,
    rows: u16,
    held: Option<(&str, &[u8])>,
    plan: Value,
) -> Result<(usize, Option<String>, f64), Halt> {
    let held = match held {
        Some((name, bytes)) => Some((field(rig.d, name)?.addr, bytes)),
        None => None,
    };
    let t0 = (rig.clock)();
    let burst = tel::stream_with(rig.c, rig.id, rig.d, &b.stream, Layout::Raw, rows, held).await?;
    let t1 = (rig.clock)();
    let flags = read_u8(rig, "fault_flags").await?;
    let k = taken.len();
    let nn = k as u32 + 1;
    let lost = burst.lost();
    let alert = burst.alert;
    taken.push(Taken {
        nn,
        meta: meta::tel(&b.servo, &burst, plan, (t0, t1)),
        burst,
        cmd_duty_q15: 0,
    });
    if flags != 0 || alert {
        let code = read_u8(rig, "fault_code").await.unwrap_or(0);
        return Ok((k, Some(fault_said(flags, code)), t1));
    }
    if !lost.is_empty() {
        return Err(Halt::Error(format!(
            "capture {nn:02} lost rows ({}): no evidence, and a leg cannot be taken again; check the cable and its contacts",
            lost.join(", ")
        )));
    }
    Ok((k, None, t1))
}

enum Polled {
    Ok(f64),
    Fault(String),
}

/// The poll between bursts: a fault, said; or the watchdog's verdict on
/// the shaft and the held current.
async fn poll<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, w: &mut Watch) -> Result<Polled, Halt> {
    let base = field(rig.d, "fault_flags")?.addr;
    let at = |name: &str| field(rig.d, name).map(|f| (f.addr - base) as usize);
    let (code_at, pos_at, i_at) = (at("fault_code")?, at("pos")?, at("i_tick_counts")?);
    let want = i_at + 2;
    let raw = rig.c.read(rig.id, base, want as u16).await?;
    if raw.len() < want {
        return Err(Halt::Error(format!(
            "the poll came back with {} of its {want} bytes",
            raw.len()
        )));
    }
    let word = |k: usize| u16::from_le_bytes([raw[k], raw[k + 1]]);
    let flags = raw[0];
    if flags != 0 {
        let code = raw[code_at];
        return Ok(Polled::Fault(fault_said(flags, code)));
    }
    let pos = b.lin.counts(word(pos_at));
    w.poll(pos, word(i_at) as i16)
        .map_err(|e| Halt::Error(format!("the watchdog: {e}; torque goes off")))?;
    Ok(Polled::Ok(pos))
}

/// What the replay reads beside a leg: the rail, the winding estimate and
/// the current bias.
async fn leg_poll<P: Pipe>(rig: &mut Rig<'_, P>) -> Result<fit::Poll, Halt> {
    Ok(fit::Poll {
        vbus_raw: super::read_u16(rig, "vbus_raw").await? as f64,
        t_winding_cc: super::read_u16(rig, "t_winding_cc").await? as i16 as f64,
        bias_counts: super::read_u16(rig, "current_bias_counts").await? as f64,
    })
}

/// A fault: the loop is not trusted to bring the shaft back.
fn faulted(found: &mut Found, on: String, f: String) -> Halt {
    found.no_recover = Some(f.clone());
    found.fault = Some(format!("{f} on {on}"));
    Halt::Error(format!("a fault latched on {on}: {f}"))
}

/// The watchdog on a burst's rows.
fn watch_rows(w: &Watch, b: &Before, burst: &Taken, found: &mut Found) -> Result<(), Halt> {
    let rows = verify::chain([&burst.burst], &b.lin, b.tick_hz);
    w.rows(&rows, b.bias).map_err(|e| {
        found.no_recover = Some(e.clone());
        Halt::Error(format!("the watchdog on the rows: {e}; torque goes off"))
    })
}

/// The poll, its fault and the watchdog's trip both ending the loop's
/// trust.
async fn poll_on<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    w: &mut Watch,
    found: &mut Found,
    on: &str,
) -> Result<f64, Halt> {
    match poll(rig, b, w).await {
        Ok(Polled::Ok(p)) => Ok(p),
        Ok(Polled::Fault(f)) => Err(faulted(found, on.into(), f)),
        Err(h) => {
            if let Halt::Error(e) = &h {
                found.no_recover = Some(e.clone());
            }
            Err(h)
        }
    }
}

/// Position mode at the servo's own estimate: the goal written first, so
/// the switch commands no motion.
async fn to_position<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before) -> Result<(), Halt> {
    let theta = verify::theta_of(rig.c, rig.id, rig.d).await?;
    write(rig, "goal_position", &theta.to_le_bytes()).await?;
    write(rig, "mode", &[b.position_mode]).await
}

/// The Position move to `goal`, in Position mode, held until the shaft
/// reads within the deadband and [`CENTER_TOL`] of it for [`CENTER_STILL`]
/// polls; the position it arrived at. With `stoppable` the interrupt
/// flag ends it.
async fn go_to<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    w: &mut Watch,
    found: &mut Found,
    goal: f64,
    stoppable: bool,
) -> Result<f64, Halt> {
    let goal = goal.round();
    write(rig, "goal_position", &(goal as i32).to_le_bytes()).await?;
    let tol = b.deadband + CENTER_TOL;
    let (mut still, mut at) = (0, f64::NAN);
    for _ in 0..CENTER_POLLS {
        if stoppable {
            interrupted(rig.stop)?;
        }
        rig.c.pause(Duration::from_secs_f64(POLL_S)).await;
        at = poll_on(rig, b, w, found, &format!("the move to {goal:.0}")).await?;
        still = if (at - goal).abs() <= tol {
            still + 1
        } else {
            0
        };
        if still >= CENTER_STILL {
            return Ok(at);
        }
    }
    Err(Halt::Error(format!(
        "the shaft read {at:.0} counts, not within {tol:.0} of {goal:.0}, after {:.0} s",
        CENTER_POLLS as f64 * POLL_S
    )))
}

fn way(d: f64) -> &'static str {
    if d > 0.0 { "up" } else { "down" }
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let mut w = Watch::new(b.watch_span, CAP_GAIN * b.cap_counts, b.tick_hz);
    ledger.touched = true;
    ledger.keep(rig.c, rig.id, rig.d, "mode").await?;
    ledger.keep(rig.c, rig.id, rig.d, "goal_velocity").await?;
    write(rig, "goal_velocity", &0i32.to_le_bytes()).await?;
    let goal0 = verify::theta_of(rig.c, rig.id, rig.d).await?;
    found.goal0 = Some(goal0 as f64);
    write(rig, "goal_position", &goal0.to_le_bytes()).await?;
    write(rig, "mode", &[b.position_mode]).await?;
    write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    found.torque_on = true;
    lines.push(format!(
        "first enable: goal {goal0} counts, the servo's own estimate; Position mode; dead-man {} ticks ({} s); then torque on, {ENABLE_S} s captured",
        b.deadman_ticks,
        rest::DEADMAN_S
    ));
    let (bu, fault, _) = shot(
        rig,
        b,
        taken,
        b.enable_rows,
        None,
        json!({"phase": "enable", "goal_counts": goal0}),
    )
    .await?;
    let fault = match fault {
        Some(f) => Some(f),
        None => match poll(rig, b, &mut w).await? {
            Polled::Fault(f) => Some(f),
            Polled::Ok(_) => None,
        },
    };
    let rows = verify::chain([&taken[bu].burst], &b.lin, b.tick_hz);
    let c = enable_check(
        &rows,
        goal0 as f64,
        b.geometry.soft.1 - b.geometry.soft.0,
        fault.as_deref(),
    );
    lines.push(format!("  {}", c.said()));
    let failed = c.verdict == Verdict::Fail;
    found.enable = Some(c.clone());
    if failed {
        found.no_recover = Some(c.value.clone());
        return Err(Halt::Refused(format!(
            "the loop drove the shaft on its own: {}",
            c.value
        )));
    }
    watch_rows(&w, b, &taken[bu], found)?;
    found.trusted = true;
    for (k, p) in b.plan.speeds.iter().enumerate() {
        interrupted(rig.stop)?;
        let (leg_rows, stop_rows) = b.bursts[k];
        let at = go_to(rig, b, &mut w, found, p.start, true).await?;
        lines.push(format!(
            "{:.0} counts/s ({:.3} of the ceiling): from rest at {at:.0}, {} legs each way of {:.0} ms{}, under a {} ms dead-man",
            p.cps,
            p.share,
            LEGS_EACH,
            1e3 * p.t_s,
            if p.shortened() {
                format!(", shortened from {:.0} ms to fit", 1e3 * p.t_full_s)
            } else {
                String::new()
            },
            1e3 * VELOCITY_DEADMAN_S
        ));
        write(rig, "goal_velocity", &0i32.to_le_bytes()).await?;
        write(
            rig,
            "bus_deadman_ticks",
            &b.velocity_deadman_ticks.to_le_bytes(),
        )
        .await?;
        write(rig, "mode", &[b.velocity_mode]).await?;
        let mut polled = leg_poll(rig).await?;
        found.t_read_cc.get_or_insert(polled.t_winding_cc);
        let (mut from, mut v_prev, mut ended) = (at, 0.0, None::<f64>);
        for leg in 0..2 * LEGS_EACH {
            interrupted(rig.stop)?;
            let goal = if leg % 2 == 0 { p.cps } else { -p.cps };
            if let Some(t1) = ended {
                let gap = (rig.clock)() - t1;
                let why = if gap > GAP_S {
                    Err(format!(
                        "the host took {:.0} ms past the leg before, over the {:.0} ms the plan allows",
                        1e3 * gap,
                        1e3 * GAP_S
                    ))
                } else {
                    next_fits(&b.geometry, p, from, v_prev, goal)
                };
                if let Err(why) = why {
                    lines.push(format!(
                        "  the {:.0} counts/s speed ends after {leg} legs: {why}",
                        p.cps
                    ));
                    found.cut_short.push((p.cps, leg, why));
                    break;
                }
            }
            let on = format!("the {:.0} counts/s leg {} {}", p.cps, leg + 1, way(goal));
            let (bu, fault, t1) = shot(
                rig,
                b,
                taken,
                leg_rows,
                Some(("goal_velocity", &(goal as i32).to_le_bytes())),
                json!({"phase": "cruise", "speed_cps": p.cps, "leg": leg + 1, "goal_cps": goal, "t_s": p.t_s}),
            )
            .await?;
            if let Some(f) = fault {
                return Err(faulted(found, on, f));
            }
            watch_rows(&w, b, &taken[bu], found)?;
            let at = poll_on(rig, b, &mut w, found, &on).await?;
            let after = leg_poll(rig).await?;
            from = taken[bu]
                .burst
                .rows
                .last()
                .and_then(|f| f.pos)
                .map_or(at, |p| b.lin.counts(p));
            found.legs.push(TakenLeg {
                speed: k,
                goal,
                k: bu,
                planned_rows: leg_rows as usize,
                before: polled,
                after,
            });
            polled = after;
            (v_prev, ended) = (goal, Some(t1));
        }
        let on = format!("the {:.0} counts/s stop", p.cps);
        let (bu, fault, _) = shot(
            rig,
            b,
            taken,
            stop_rows,
            Some(("goal_velocity", &0i32.to_le_bytes())),
            json!({"phase": "stop", "speed_cps": p.cps}),
        )
        .await?;
        if let Some(f) = fault {
            return Err(faulted(found, on, f));
        }
        watch_rows(&w, b, &taken[bu], found)?;
        poll_on(rig, b, &mut w, found, &on).await?;
        to_position(rig, b).await?;
        write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    }
    interrupted(rig.stop)?;
    let at = go_to(rig, b, &mut w, found, b.mid, true).await?;
    found.centered = Some(at);
    lines.push(format!(
        "end: the shaft at {at:.0} counts, the middle {:.0}",
        b.mid
    ));
    Ok(())
}

/// After an error with the loop trusted: the goal speed zero, the Position
/// phases' dead-man back, Position mode, the move to the middle; best
/// effort, whatever the interrupt flag.
async fn recover<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, found: &mut Found) -> String {
    let mut w = Watch::new(b.watch_span, CAP_GAIN * b.cap_counts, b.tick_hz);
    let top = b.plan.speeds.iter().map(|p| p.cps).fold(0.0, f64::max);
    let stop_s = b.geometry.stop_s(top);
    let r: Result<f64, Halt> = async {
        write(rig, "goal_velocity", &0i32.to_le_bytes()).await?;
        write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
        keep_alive(rig, stop_s).await?;
        to_position(rig, b).await?;
        go_to(rig, b, &mut w, found, b.mid, false).await
    }
    .await;
    match r {
        Ok(at) => {
            found.centered = Some(at);
            format!("brought to the middle {:.0}: at {at:.0} counts", b.mid)
        }
        Err(h) => format!("not brought to the middle: {}", h.why()),
    }
}

/// A leg as read: its read or why not, and what sets it aside.
type ReadLeg = (TakenLeg, Result<LegRead, String>, Vec<String>);

/// The legs read and the line.
struct Judged {
    reads: Vec<ReadLeg>,
    points: Vec<Point>,
    line: Option<fit::Line>,
}

fn judge(b: &Before, found: &Found, taken: &[Taken]) -> Judged {
    let reads: Vec<ReadLeg> = found
        .legs
        .iter()
        .map(|l| {
            let got = match found.t_read_cc {
                Some(t_read_cc) => fit::read(
                    &Leg {
                        goal_cps: l.goal,
                        rows: &taken[l.k].burst.rows,
                        before: l.before,
                        after: l.after,
                        t_read_cc,
                    },
                    &b.lin,
                    &b.reading,
                ),
                None => Err("no poll before the first leg".into()),
            };
            let top = b.plan.speeds[l.speed].share == SPEEDS[SPEEDS.len() - 1].0;
            let fails = match &got {
                Ok(r) => r.fails(top),
                Err(e) => vec![e.clone()],
            };
            (l.clone(), got, fails)
        })
        .collect();
    let points: Vec<Point> = reads
        .iter()
        .filter(|(_, _, f)| f.is_empty())
        .filter_map(|(l, r, _)| {
            let r = r.as_ref().ok()?;
            Some(Point {
                goal_cps: l.goal.abs(),
                dir: r.dir,
                i_ma: r.held_ma,
            })
        })
        .collect();
    let line = fit::line(&points, b.reading.counts_per_motor_rad);
    Judged {
        reads,
        points,
        line,
    }
}

/// What to do when a check refuses the step.
pub fn advice(name: &str) -> &'static str {
    match name {
        fit::RESIDUALS => {
            "under two speeds held a steady cruise, so no line: check that the horn turns freely, then run the step again"
        }
        _ => "check the servo, then run the step again",
    }
}

/// The step on the servo. Refused with nothing written when it cannot
/// start; once it has written it ends through the guard whatever happens,
/// centered first while the loop is trusted, and keeps its captures and
/// its result.
pub async fn cruise<P: Pipe>(
    rig: &mut Rig<'_, P>,
    i: &CruiseInputs<'_>,
) -> Result<Cruised, Refused> {
    let b = before(rig, i).await?;
    let g = &b.geometry;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.0} counts; travel {:.0}..{:.0} from {}; soft range {:.0}..{:.0}; speed ceiling {:.0} counts/s, move speed {:.0} counts/s, acceleration {:.0} counts/s^2; {:.3} counts/deg, {}",
            b.pos,
            g.stops.0,
            g.stops.1,
            b.travel_from,
            g.soft.0,
            g.soft.1,
            g.ceiling_cps,
            g.move_cps,
            g.accel_per_tick * g.medium_hz,
            b.scale.counts_per_deg,
            b.scale.source
        ),
    ];
    for p in &b.plan.speeds {
        lines.push(format!(
            "plan: {:.0} counts/s from {:.0}, legs of {:.0} ms{}, reaching {:.0}..{:.0} inside {:.0}..{:.0}, a host lost as a leg ends leaving it in {:.0}..{:.0}",
            p.cps,
            p.start,
            1e3 * p.t_s,
            if p.shortened() {
                format!(" (shortened from {:.0} ms)", 1e3 * p.t_full_s)
            } else {
                String::new()
            },
            p.extent.0,
            p.extent.1,
            p.span.0,
            p.span.1,
            p.runaway.0,
            p.runaway.1
        ));
    }
    for l in &b.plan.left_out {
        lines.push(format!("plan: {:.0} counts/s left out: {}", l.cps, l.why));
    }
    let mut ledger = Ledger::default();
    let mut taken = Vec::new();
    let mut found = Found::default();
    let ran = body(rig, &b, &mut ledger, &mut taken, &mut found, &mut lines).await;
    if ran.is_err() && found.torque_on {
        let said = match (&found.no_recover, found.trusted) {
            (Some(why), _) => {
                format!("not brought to the middle: the loop is not trusted ({why})")
            }
            (None, false) => {
                "not brought to the middle: the first enable's check had not passed, so the loop is not trusted to drive the shaft".into()
            }
            (None, true) => recover(rig, &b, &mut found).await,
        };
        lines.push(format!("after the error: {said}"));
        found.recovered = Some(said);
    }
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = verify::theta_of(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines).map_err(Halt::from);
    let j = judge(&b, &found, &taken);
    for (l, r, fails) in &j.reads {
        lines.push(format!("  {}", leg_said(l, r, fails, b.tick_hz)));
    }
    let mut checks: Vec<Check> = found.enable.iter().cloned().collect();
    if ran.is_ok() {
        checks.extend(fit::checks(
            &j.points,
            j.line.as_ref(),
            b.reading.ceiling_cps,
        ));
        let aside: Vec<String> = j
            .reads
            .iter()
            .filter(|(_, _, f)| !f.is_empty())
            .map(|(l, _, f)| {
                format!(
                    "{:.0} {} (capture {:02}): {}",
                    l.goal.abs(),
                    way(l.goal),
                    l.k + 1,
                    f.join("; ")
                )
            })
            .collect();
        checks.push(Check {
            name: "cruise legs set aside".into(),
            value: if aside.is_empty() {
                format!("none of {}", j.reads.len())
            } else {
                format!(
                    "{} of {}: {}",
                    aside.len(),
                    j.reads.len(),
                    aside.join(" | ")
                )
            },
            bound: "every leg steady by the rule".into(),
            verdict: if aside.is_empty() {
                Verdict::Pass
            } else {
                Verdict::Warn
            },
        });
        let mut short: Vec<String> = b
            .plan
            .speeds
            .iter()
            .filter(|p| p.shortened())
            .map(|p| format!("{:.0} shortened to {:.0} ms", p.cps, 1e3 * p.t_s))
            .collect();
        short.extend(
            b.plan
                .left_out
                .iter()
                .map(|l| format!("{:.0} left out", l.cps)),
        );
        short.extend(
            found
                .cut_short
                .iter()
                .map(|(c, k, _)| format!("{c:.0} ended after {k} legs")),
        );
        checks.push(Check {
            name: "cruise speeds the travel cut".into(),
            value: if short.is_empty() {
                "none".into()
            } else {
                short.join(", ")
            },
            bound: format!("every speed's {} legs each way at full length", LEGS_EACH),
            verdict: if short.is_empty() {
                Verdict::Pass
            } else {
                Verdict::Warn
            },
        });
    }
    if !exit.torque_off {
        checks.push(Check {
            name: "torque off at the end".into(),
            value: "torque_enable did not read back 0".into(),
            bound: "reads 0".into(),
            verdict: Verdict::Fail,
        });
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
    lines.push(format!(
        "end position {} counts; torque {}",
        after.map_or("not read".into(), |p| p.to_string()),
        if exit.torque_off {
            "off"
        } else {
            "NOT read back off"
        }
    ));
    let result = result_json(&b, &found, &j, &checks, way, &why, &exit, after);
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Cruised {
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

fn leg_said(l: &TakenLeg, r: &Result<LegRead, String>, fails: &[String], tick_hz: f64) -> String {
    let head = format!(
        "{:.0} counts/s {} (capture {:02})",
        l.goal.abs(),
        way(l.goal),
        l.k + 1
    );
    match r {
        Err(e) => format!("{head}: not read, {e}"),
        Ok(r) => format!(
            "{head}: {:.0} ms steady, {:.0} counts/s, halves {:+.2}%, phase {} (sd {:.2} mA, worst {:.2}), read {:.0}%, held {:.2} mA (read rows {:.2}), integrator {:+.2} mA, inertia {:+.2} mA (reported): {}",
            1e3 * r.rows as f64 / tick_hz,
            r.speed_cps,
            100.0 * r.halves,
            r.phase,
            r.sd_in_ma,
            r.sd_worst_ma,
            100.0 * r.read_share,
            r.held_ma,
            r.read_ma,
            r.integ_ma,
            r.inertia_ma,
            if fails.is_empty() {
                "valid".into()
            } else {
                format!("set aside, {}", fails.join("; "))
            }
        ),
    }
}

fn finite(v: f64) -> Value {
    if v.is_finite() { v.into() } else { Value::Null }
}

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    j: &Judged,
    checks: &[Check],
    way: Way,
    why: &Option<String>,
    exit: &guard::Exit,
    after: Option<i32>,
) -> Value {
    let r = &b.reading;
    let g = &b.geometry;
    let mut estimates = Map::new();
    let source = |l: &fit::Line| {
        format!(
            "the least-squares line of each closed-loop cruise leg's held current on its goal speed over {} valid legs at {} speeds, {:.0} to {:.0} counts/s, both directions pooled: the stage 5 cruise-friction method, each leg's current-loop reference replayed from its rows (the board's integer reading over the polled bias, the installed gains, Ke, window term and resistance), averaged over its MEDIUM period at the phase found, the reading on read rows and that mean on the rest, over the leg's middle half; every leg below the top speed and the top speed's legs whose speed held across their steady rows within {}%; per motor rad/s at {:.4} counts a motor radian; on the nominal sense block's {:.3} counts/A; the band is the standard error from the legs' residuals",
            l.legs,
            l.speeds.len(),
            l.goal_cps.0,
            l.goal_cps.1,
            fit::STEADY_TOP_PCT,
            r.counts_per_motor_rad,
            r.counts_per_a,
        )
    };
    if let Some(l) = &j.line {
        let s = source(l);
        estimates.insert(
            "friction_coulomb_ma".into(),
            json!({"value": l.coulomb_ma, "lo": l.coulomb_ma - l.coulomb_se_ma, "hi": l.coulomb_ma + l.coulomb_se_ma, "unit": "mA", "source": s}),
        );
        estimates.insert(
            "friction_viscous_ma_s_per_rad".into(),
            json!({"value": l.viscous_ma_s_per_rad, "lo": l.viscous_ma_s_per_rad - l.viscous_se, "hi": l.viscous_ma_s_per_rad + l.viscous_se, "unit": "mA per motor rad/s", "source": s}),
        );
    }
    let line = j.line.as_ref().map_or(Value::Null, |l| {
        json!({
            "coulomb_ma": l.coulomb_ma, "coulomb_se_ma": l.coulomb_se_ma,
            "viscous_ma_per_kcps": l.viscous_ma_per_kcps, "viscous_se_ma_per_kcps": l.viscous_se_ma_per_kcps,
            "viscous_ma_s_per_rad": l.viscous_ma_s_per_rad, "viscous_se": l.viscous_se,
            "legs": l.legs, "goal_cps": [l.goal_cps.0, l.goal_cps.1], "rms_ma": l.rms_ma,
            "speeds": l.speeds.iter().map(|s| json!({
                "goal_cps": s.goal_cps, "legs": s.legs, "current_ma": s.i_ma,
                "residual_ma": s.residual_ma, "up_ma": s.up_ma, "down_ma": s.down_ma,
            })).collect::<Vec<_>>(),
        })
    });
    let poll = |p: &fit::Poll| json!({"vbus_raw": p.vbus_raw, "t_winding_cc": p.t_winding_cc, "current_bias_counts": p.bias_counts});
    let legs: Vec<Value> = j
        .reads
        .iter()
        .map(|(l, r, fails)| {
            let mut v = json!({"goal_cps": l.goal, "capture": l.k + 1, "planned_rows": l.planned_rows,
                               "before": poll(&l.before), "after": poll(&l.after),
                               "valid": fails.is_empty(), "set_aside": fails});
            if let (Ok(r), Some(m)) = (r, v.as_object_mut()) {
                m.extend([
                    ("leg_rows".into(), r.leg_rows.into()),
                    ("rows".into(), r.rows.into()),
                    ("phase".into(), r.phase.into()),
                    ("sd_in_ma".into(), finite(r.sd_in_ma)),
                    ("sd_worst_ma".into(), finite(r.sd_worst_ma)),
                    ("held_in_ms".into(), r.held_in_ms.map_or(Value::Null, finite)),
                    ("read_share".into(), r.read_share.into()),
                    ("current_read_ma".into(), finite(r.read_ma)),
                    ("current_held_ma".into(), finite(r.held_ma)),
                    ("line_installed_ma".into(), finite(r.line_ma)),
                    ("integrator_ma".into(), finite(r.integ_ma)),
                    ("speed_cps".into(), finite(r.speed_cps)),
                    ("halves".into(), finite(r.halves)),
                    ("current_inertia_ma".into(), finite(r.inertia_ma)),
                ]);
            }
            v
        })
        .collect();
    let accepted = way == Way::Done;
    let failed: Vec<&str> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.as_str())
        .collect();
    json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "why": why,
        "accepted": accepted,
        "failed_on": if accepted { Value::Null } else { json!(failed) },
        "estimates": estimates,
        "line": line,
        "legs": legs,
        "t_read_cc": found.t_read_cc,
        "plan": {
            "speeds": b.plan.speeds.iter().map(|p| json!({
                "share": p.share, "speed_cps": p.cps, "t_s": p.t_s, "t_full_s": p.t_full_s, "shortened": p.shortened(),
                "start_counts": p.start, "extent_counts": [p.extent.0, p.extent.1], "span_counts": [p.span.0, p.span.1],
                "runaway_counts": [p.runaway.0, p.runaway.1],
            })).collect::<Vec<_>>(),
            "left_out": b.plan.left_out.iter().map(|l| json!({"share": l.share, "speed_cps": l.cps, "why": l.why})).collect::<Vec<_>>(),
            "cut_short": found.cut_short.iter().map(|(c, k, w)| json!({"speed_cps": c, "legs_run": k, "why": w})).collect::<Vec<_>>(),
            "legs_each": LEGS_EACH, "gap_s": GAP_S, "leg_min_share": LEG_MIN_SHARE, "guard_counts": GUARD_COUNTS,
            "wall_cps_per_count": WALL_CPS_PER_COUNT, "stop_hold_s": STOP_HOLD_S,
            "velocity_deadman_s": VELOCITY_DEADMAN_S, "position_deadman_s": rest::DEADMAN_S,
            "coast_cps2": g.coast_cps2, "stop_clear_counts": STOP_CLEAR_COUNTS,
            "clear_counts": [g.clear().0, g.clear().1],
        },
        "travel": {
            "stops_counts": [g.stops.0, g.stops.1], "from": b.travel_from, "soft_counts": [g.soft.0, g.soft.1],
            "middle_counts": b.mid, "speed_ceiling_cps": g.ceiling_cps, "move_cps": g.move_cps,
            "accel_cps2": g.accel_per_tick * g.medium_hz,
        },
        "scales": {
            "counts_per_a": r.counts_per_a, "counts_per_a_source": "the servo's nominal sense block, the gather's",
            "g_window_q016": r.g_window_q016, "i_kp_q88": r.i_kp_q88, "i_ki_q412": r.i_ki_q412,
            "ke_vpc_q": r.ke_vpc_q, "r_hat_q12": r.r_hat_q12, "j_ff_q88": r.j_ff_q88,
            "fric_fc_counts": r.fric_fc_counts, "fric_fv_q016": r.fric_fv_q016,
            "pwm_arr": r.pwm_arr, "tick_hz": r.tick_hz, "bridge_min_ticks": r.bridge_min_ticks,
            "settle_start_ticks": r.settle.start_ticks, "settle_q15": r.settle.q15,
            "counts_per_deg": b.scale.counts_per_deg, "gear_ratio": b.gear_ratio,
            "counts_per_motor_rad": r.counts_per_motor_rad,
        },
        "bounds": {
            "steady_cut": fit::STEADY_CUT, "travel_counts": fit::TRAVEL_COUNTS,
            "speed_window_min_s": fit::SPEED_WINDOW_MIN_S, "speed_tol": fit::SPEED_TOL,
            "copper_per_c": fit::COPPER_PER_C, "kept_min_s": fit::KEPT_MIN_S, "steady_top_pct": fit::STEADY_TOP_PCT,
        },
        "enable_goal_counts": found.goal0,
        "fault": found.fault,
        "recovered": found.recovered,
        "checks": checks.iter().map(Check::json).collect::<Vec<_>>(),
        "position_before_counts": b.pos,
        "position_centered_counts": found.centered,
        "position_after_counts": after,
        "exit": exit.lines,
        "torque_off": exit.torque_off,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mg90-d's travel, soft range, speed ceiling, move speed,
    /// acceleration and the record's coast (fric_fc 43 over j_ff 862).
    pub(crate) fn mg90d() -> Geometry {
        Geometry {
            stops: (110.0, 3730.0),
            soft: (314.0, 3560.0),
            ceiling_cps: 8050.0,
            move_cps: 4025.0,
            accel_per_tick: 10305.0 / 256.0,
            medium_hz: 2000.0,
            coast_cps2: 43.0 / (862.0 / 256.0) * 2000.0,
            deadman_s: VELOCITY_DEADMAN_S,
        }
    }

    /// The reference slews at the profile's acceleration: a leg from rest
    /// travels its speed times its time less the ramp's half.
    #[test]
    fn the_path_slews_at_the_acceleration() {
        let g = mg90d();
        let a = g.accel_per_tick * g.medium_hz;
        let p = path(&g, 0.0, 0.0, &[(4025.0, 0.6)]);
        let want = 4025.0 * 0.6 - 4025.0 * 4025.0 / (2.0 * a);
        assert!((p.hi - want).abs() < 5.0, "{} {want}", p.hi);
        assert_eq!(p.lo, 0.0);
        assert_eq!(p.ends, [(p.hi, 4025.0)]);
    }

    /// A host lost as a leg ends: the goal holds through the gap and the
    /// dead-man, the wall ramp slows the reference toward the soft limit it
    /// points at, then the coast. Away from the wall it runs on.
    #[test]
    fn a_lost_host_runs_the_gap_and_the_dead_man_then_coasts() {
        let g = mg90d();
        let t = GAP_S + g.deadman_s;
        // in the middle at 302 c/s: the goal's distance plus a short coast
        let lost = g.runaway(1900.0, 302.0);
        let coast = 302.0 * 302.0 / (2.0 * g.coast_cps2);
        assert!((lost - (1900.0 + 302.0 * t + coast)).abs() < 1.0, "{lost}");
        // at the top speed toward the high soft limit: the wall holds the
        // reference short of the soft limit, the coast stays inside the stop
        let lost = g.runaway(3200.0, 4025.0);
        assert!(lost < 3200.0 + 4025.0 * t, "{lost}");
        assert!(lost < g.clear().1, "{lost}");
        let down = g.runaway(620.0, -4025.0);
        assert!(down > g.clear().0 && down < 620.0, "{down}");
    }

    /// On the mg90-d every speed runs at its full leg, the plan's shares of
    /// the ceiling, centered on its span, inside it at the worst gaps, a
    /// lost host clear of the stops.
    #[test]
    fn the_mg90d_runs_every_speed_at_full_length() {
        let g = mg90d();
        let p = plan(&g);
        assert!(p.left_out.is_empty(), "{:?}", p.left_out);
        let cps: Vec<f64> = p.speeds.iter().map(|s| s.cps).collect();
        assert_eq!(cps, [322.0, 805.0, 1530.0, 4025.0]);
        for s in &p.speeds {
            assert!(!s.shortened(), "{s:?}");
            assert!(s.extent.0 >= s.span.0 && s.extent.1 <= s.span.1, "{s:?}");
            assert!(s.span.0 >= g.stops.0 + GUARD_COUNTS && s.span.1 <= g.stops.1 - GUARD_COUNTS);
            assert!(s.span.0 >= g.soft.0 + s.cps / 16.0 && s.span.1 <= g.soft.1 - s.cps / 16.0);
            assert!(
                s.runaway.0 >= g.clear().0 && s.runaway.1 <= g.clear().1,
                "{s:?}"
            );
            let centre = (s.extent.0 + s.extent.1) / 2.0;
            assert!((centre - (s.span.0 + s.span.1) / 2.0).abs() < 1e-6, "{s:?}");
        }
        let top = &p.speeds[3];
        assert!(top.extent.1 - top.extent.0 > 2400.0, "{top:?}");
        // a ceiling over twice the move speed: the top cruise is capped at it
        let fast = plan(&Geometry {
            ceiling_cps: 9000.0,
            ..g
        });
        assert_eq!(fast.speeds.last().map(|s| s.cps), Some(4025.0));
    }

    /// A servo with a third of the mg90-d's travel: the top speed's legs
    /// do not fit at half length and it is left out, said why; a middle
    /// one is shortened to the longest that fits; the slowest runs whole.
    #[test]
    fn a_short_travel_servo_shortens_or_drops_speeds() {
        let g = Geometry {
            stops: (1000.0, 2400.0),
            soft: (1100.0, 2300.0),
            ..mg90d()
        };
        let p = plan(&g);
        let top = p.left_out.last().expect("the top speed left out");
        assert_eq!(top.cps, 4025.0, "{p:?}");
        assert!(top.why.contains("no 300 ms legs fit"), "{}", top.why);
        let short: Vec<&Planned> = p.speeds.iter().filter(|s| s.shortened()).collect();
        assert!(!short.is_empty(), "{p:?}");
        for s in short {
            assert!(s.t_s >= LEG_MIN_SHARE * s.t_full_s, "{s:?}");
            assert!(s.extent.0 >= s.span.0 && s.extent.1 <= s.span.1, "{s:?}");
            assert!(place(&g, s.cps, s.t_s + 0.002).is_err(), "{s:?}");
        }
        let slow = p.speeds.first().expect("the slowest speed planned");
        assert!(slow.cps == 322.0 && !slow.shortened(), "{slow:?}");
        let g = Geometry {
            stops: (1800.0, 2200.0),
            soft: (1900.0, 2100.0),
            ..mg90d()
        };
        let p = plan(&g);
        assert!(p.speeds.is_empty(), "{p:?}");
        assert_eq!(p.left_out.len(), 4);
    }

    /// Soft limits set well in from one stop: the legs center on the span
    /// they may use, not the travel's middle, and so run longer than
    /// centered on the middle.
    #[test]
    fn asymmetric_soft_limits_center_the_legs_on_the_span() {
        let g = Geometry {
            soft: (314.0, 2900.0),
            ..mg90d()
        };
        let p = plan(&g);
        for s in &p.speeds {
            let centre = (s.extent.0 + s.extent.1) / 2.0;
            let mid = (s.span.0 + s.span.1) / 2.0;
            assert!(
                (centre - mid).abs() < 1e-6 || (centre - g.middle()).abs() < 1e-6,
                "{s:?}"
            );
        }
        let top = p.speeds.iter().find(|s| s.cps == 4025.0);
        let centred_on_middle = {
            let (lo, hi) = {
                let t = top.map_or(0.6, |t| t.t_s);
                let a = path(&g, 0.0, 0.0, &legs_of(&g, 4025.0, t + GAP_S, t));
                let b = path(&g, 0.0, 0.0, &legs_of(&g, 4025.0, t, t + GAP_S));
                (a.lo.min(b.lo), a.hi.max(b.hi))
            };
            let start = g.middle() - (lo + hi) / 2.0;
            let span = g.span(4025.0);
            start + lo >= span.0 && start + hi <= span.1
        };
        let top = top.expect("the top speed planned");
        assert!(
            !centred_on_middle,
            "the travel's middle would not hold the top speed's {} ms legs: {top:?}",
            1e3 * top.t_s
        );
    }

    /// Soft limits out past the stops (0 / 0, off, over stops measured
    /// inside the physical ones): the plan alone keeps a lost host's shaft
    /// clear of the stops, the top speed shortened or left out for it.
    #[test]
    fn soft_limits_outside_the_stops_leave_the_runaway_bound() {
        let g = Geometry {
            soft: (50.0, 3800.0),
            ..mg90d()
        };
        let p = plan(&g);
        for s in &p.speeds {
            assert!(
                s.runaway.0 >= g.clear().0 && s.runaway.1 <= g.clear().1,
                "{s:?}"
            );
        }
        let top = p.speeds.iter().find(|s| s.cps == 4025.0);
        let cut = top.is_none_or(|t| t.shortened());
        assert!(cut, "{p:?}");
        if let Some(l) = p.left_out.iter().find(|l| l.cps == 4025.0) {
            assert!(l.why.contains("host lost"), "{}", l.why);
        }
    }

    /// Soft limits on the stops, as a share of 0 used to leave them, and
    /// 0 / 0, off, over the set's own stops: the plan runs, every leg's
    /// span [`GUARD_COUNTS`] and every lost host [`STOP_CLEAR_COUNTS`]
    /// inside the stops.
    #[test]
    fn soft_limits_on_the_stops_keep_the_guard() {
        for soft in [(110.0, 3730.0), (82.0, 3792.0)] {
            let g = Geometry { soft, ..mg90d() };
            let p = plan(&g);
            assert!(!p.speeds.is_empty(), "{soft:?}: {p:?}");
            for s in &p.speeds {
                assert!(
                    s.span.0 >= g.stops.0 + GUARD_COUNTS && s.span.1 <= g.stops.1 - GUARD_COUNTS,
                    "{s:?}"
                );
                assert!(s.extent.0 >= s.span.0 && s.extent.1 <= s.span.1, "{s:?}");
                assert!(
                    s.runaway.0 >= g.clear().0 && s.runaway.1 <= g.clear().1,
                    "{s:?}"
                );
            }
        }
    }

    /// A burst the servo cannot stream is refused before any write.
    #[test]
    fn a_burst_over_the_row_limit_is_refused() {
        assert_eq!(rows_for(0.6, 20000.0).expect("a leg"), 12000);
        let e = rows_for(4.0, 20000.0).expect_err("over u16");
        assert!(e.0.contains("no u16 of rows"), "{e}");
    }

    /// Before each leg the next is predicted from the leg before's last
    /// row, its turn or stop always counted: a shaft that drifted toward a
    /// guard ends the speed rather than leave the span.
    #[test]
    fn a_drifted_shaft_ends_the_speed() {
        let g = mg90d();
        let p = plan(&g);
        let top = &p.speeds[3];
        let up = path(&g, top.start, 0.0, &[(top.cps, top.t_s)]);
        assert!(next_fits(&g, top, up.hi, top.cps, -top.cps).is_ok());
        let e = next_fits(&g, top, up.hi + 300.0, top.cps, -top.cps).expect_err("past");
        assert!(e.contains("past the span"), "{e}");
        // a down leg whose own end fits the span but whose turn after it
        // does not: refused though it is not the last
        let (v, t) = (top.cps, top.t_s);
        let legs = [(v, GAP_S), (-v, t + GAP_S)];
        let lo = path(&g, 0.0, v, &legs).lo;
        let from = top.span.0 + 10.0 - lo;
        let e = next_fits(&g, top, from, v, -v).expect_err("its turn");
        assert!(e.contains("past the span"), "{e}");
    }
}
