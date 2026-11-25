//! The table step, after the stops: the ADC's wide-code slots and the
//! position table from one open-loop capture, read by
//! [`crate::fit::widecodes`] and the pooled position fit.
//!
//! Slow passes at the stops step's approach duty, the seat volts:
//! a half pass from where the stops step left the shaft to the low stop,
//! then [`Schedule::slow_round_trips`] round trips stop to stop, each
//! one-way pass ending at rest on its stop and the next starting there. A
//! slow pass is chained bursts: free rungs as [`plan_rung`] sizes them,
//! the first leaving the stop, then a stop rung under the stall permit
//! ([`plan_stop_rung`]) that ends pressed on the far stop. While a carry
//! inside the swept range reads thin to the wide-code finder, another slow
//! round trip is added, at most [`Schedule::extra_slow_max`]; a carry thin
//! after them stays 0 and is named.
//!
//! Fast passes at the volts that give one fifth of the rated speed by the
//! nameplate's speed per volt: [`Schedule::fast_round_trips`] round trips,
//! each pass one burst from the near guard, its window cut so the free
//! margin over the predicted travel plus the stopping distance ends inside
//! the far guard, the first at half length to measure the real speed. The
//! guards are the envelope's, [`GUARD_SHARE`] of the stops' span in from
//! each. A rung's length ([`plan_rung`]):
//!
//! ```text
//! T           = min(BURST_MAX_S - DRIVE_DEADMAN_S,
//!                   (runway - bound_speed x COAST_TAU_S) / bound_speed
//!                   - DRIVE_DEADMAN_S)
//! bound_speed = FREE_MARGIN x the fastest speed seen that way at the duty
//! ```
//!
//! Every rung is priced through the envelope before it is sent. After a
//! free rung the shaft rests inside both guards, no faster than its bound;
//! after a stop rung it rests on the stop. A burst that lost a row is
//! taken again once, then the step ends. The way out leaves the shaft at
//! the middle of the stops.
//!
//! The reading ([`judge`]): the wide-code slots from the slow passes'
//! rows more than [`NEAR_STOP_COUNTS`] inside both stops, then one pooled
//! fit `pos = T(phase) + c_rung` over the fast passes' constant-duty
//! stretches and the slow passes' rungs, each its own segment with its own
//! offset, cut where the pot comes within [`NEAR_STOP_COUNTS`] of a stop:
//! the slow rungs fill the ends past the fast passes' brake points.

use std::time::Duration;

use osc_client::pipe::Pipe;
use osc_servo_core::estimator::window::drive_ticks;
use serde_json::{Map, Value, json};

use super::drive::{self, Deadmen, Drive, End};
use super::polarity::{HANDS, MOVE_MIN_COUNTS};
use super::probe::FromRest;
use super::runway::{COAST_TAU_S, Go, GoTo, Settle};
use super::seek::{self, POLL_S, WALK_MAX_COUNTS, WINDOW_S};
use super::stops::{self, FromProbe, POLL_DEADMAN_S};
use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, read_u16, rest, write,
};
use crate::Board;
use crate::capture::meta;
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{self, Line, StreamCfg};
use crate::envelope::{
    BURST_MAX_S, DRIVE_DEADMAN_S, Envelope, FREE_MARGIN, Facts, GUARD_SHARE, HeatModel, Kind,
    PERMIT_MAX_S, Price, Resistance, Rung,
};
use crate::fit::median;
use crate::fit::polyfit;
use crate::fit::postable::{
    self, Agreement, PLAY_COUNTS, PooledFit, SMOOTH_WEIGHT, SPIN_UP_S, Segment, SegmentError,
    TableError, Term,
};
use crate::fit::widecodes::{self, EDGE_MARGIN_COUNTS, Finding, HALF_THIN_ROWS, State, THIN_ROWS};
use crate::guard::{self, Ledger, Way};
use crate::inputs::ThermalInputs;
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::{ADC_COUNTS, Parts};
use crate::state::{self, State as ServoState, field};
use osc_servo_core::adc::ADC_WIDE_SLOTS;
use osc_servo_core::pos_lut::{self, INTERVALS, PAGE_POINTS, POINTS};

pub const STEP: &str = "table";
pub const RESULT: &str = "results/table.json";
const METHOD: &str = "open-loop-slow-and-fast-passes";
const REVISION: u32 = 1;
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "the probe's pricing R (planning grade)";

pub const RUNG_MIN_S: f64 = 0.15;
/// The free rungs a slow pass chains before its stop rung, and the stop
/// rungs it may take to reach the stop.
pub const FREE_RUNGS_MAX: usize = 4;
pub const STOP_RUNGS_MAX: usize = 2;
/// The share of a rung's driven rows, from its end, its speed is read
/// over: past the spin-up.
pub const SPEED_TAIL_SHARE: f64 = 0.6;
/// The coast burst after a rung, s: no estimator reads its rows, so it
/// is as long as the spin-down bound, enough to stamp the zero.
pub const COAST_S: f64 = COAST_TAU_S;
/// Attempts a rung gets when a burst lost rows.
pub const ATTEMPTS: usize = 2;
/// The fast passes' volts, a share of the voltage the nameplate states
/// its speed at: one fifth of the rated speed by its speed per volt.
pub const FAST_SHARE: f64 = 0.2;
/// The first fast pass's share of its window: it measures the real speed.
pub const FIRST_FAST_SHARE: f64 = 0.5;
/// Rows within this many counts of a stop are neither ruler nor
/// histogram rows: the pot slows into a seat over up to 23 counts and
/// reads within 5 of it pressed (the stops step's poll logs).
pub const NEAR_STOP_COUNTS: f64 = 32.0;
/// The median rows a code the slow passes give the wide-code finder at
/// least (stage 1 `widecodes.find`'s floor).
pub const ROWS_A_CODE_MIN: f64 = 100.0;
/// Counts rms the pooled fit's residual may reach.
pub const RESIDUAL_MAX_COUNTS: f64 = 4.0;
/// Counts rms the table may move, line out, with one fast pass left out.
pub const PASS_OUT_MAX_COUNTS: f64 = 4.0;
/// Counts rms of the directions' difference over which it is a warning.
pub const DIRECTION_WARN_COUNTS: f64 = 3.0;
/// A rung's speed spread over which it is a warning, never a refusal.
pub const SPEED_SPREAD_WARN: f64 = 0.2;

/// How many of each pass the step runs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Schedule {
    pub slow_round_trips: usize,
    pub fast_round_trips: usize,
    /// Slow round trips added at most while a carry reads thin.
    pub extra_slow_max: usize,
}

impl Schedule {
    /// Two slow and two fast round trips, two slow ones more at most.
    pub const PLAN: Schedule = Schedule {
        slow_round_trips: 2,
        fast_round_trips: 2,
        extra_slow_max: 2,
    };
}

/// The way a pass or a rung runs: + raises the counts.
fn way(dir: i8) -> &'static str {
    if dir > 0 { "up" } else { "down" }
}

/// One of the stops step's approach legs: its way, its counts and its
/// seconds.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ApproachLeg {
    pub n: usize,
    pub dir: i8,
    pub counts: f64,
    pub secs: f64,
}

impl ApproachLeg {
    /// Its mean speed, counts/s: a lower bound on the free speed, since a
    /// leg ends once its still window at the stop is read.
    pub fn speed_cps(&self) -> f64 {
        self.counts / self.secs
    }
}

/// What the step takes from the stops step's result.
#[derive(Clone, Debug, PartialEq)]
pub struct FromStops {
    pub lo: f64,
    pub hi: f64,
    pub approach_q15: i16,
    pub legs: Vec<ApproachLeg>,
}

impl FromStops {
    pub fn of(v: &Value) -> Result<Self, Refused> {
        if v["verdict"] != "accepted" {
            return Err(Refused(format!(
                "the table step runs on an accepted stops step; the stops step was {}",
                v["verdict"].as_str().unwrap_or("not run")
            )));
        }
        let n = |x: &Value, what: &str| {
            x.as_f64()
                .filter(|v| v.is_finite())
                .ok_or_else(|| Refused(format!("the stops result holds no {what}")))
        };
        let lo = n(&v["stops"]["low"]["median_counts"], "low stop")?;
        let hi = n(&v["stops"]["high"]["median_counts"], "high stop")?;
        let approach_q15 = v["drives"]["approach_q15"]
            .as_i64()
            .and_then(|q| i16::try_from(q).ok())
            .filter(|q| *q > 0)
            .ok_or_else(|| Refused("the stops result holds no drives.approach_q15".into()))?;
        let legs = v["stops"]["legs"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter(|l| l["kind"] == "approach")
                    .filter_map(|l| {
                        let dir = match l["toward"].as_str()? {
                            "high" => 1,
                            "low" => -1,
                            _ => return None,
                        };
                        let counts = (l["to_counts"].as_f64()? - l["from_counts"].as_f64()?).abs();
                        let secs = l["secs"].as_f64().filter(|s| *s > 0.0)?;
                        Some(ApproachLeg {
                            n: l["n"].as_u64().unwrap_or(0) as usize,
                            dir,
                            counts,
                            secs,
                        })
                    })
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        for dir in [1, -1] {
            if !legs.iter().any(|l| l.dir == dir && l.speed_cps() > 0.0) {
                return Err(Refused(format!(
                    "the stops result holds no approach leg {}: no speed bounds the first rung that way; run the stops step again",
                    way(dir)
                )));
            }
        }
        if hi <= lo {
            return Err(Refused(format!(
                "the stops read {lo:.0} and {hi:.0}: no travel; run the stops step again"
            )));
        }
        Ok(FromStops {
            lo,
            hi,
            approach_q15,
            legs,
        })
    }

    /// The stops rounded inward, as the gather writes them.
    pub fn raw(&self) -> (u16, u16) {
        let c = |v: f64| v.clamp(0.0, ADC_COUNTS - 1.0) as u16;
        (c(self.lo.ceil()), c(self.hi.floor()))
    }
}
/// The counts a free rung stays inside: the envelope's band.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Band {
    pub lo: f64,
    pub hi: f64,
}

impl Band {
    /// [`GUARD_SHARE`] of the stops' span in from each, as
    /// [`Facts::band`] has it.
    pub fn of_stops(lo: f64, hi: f64) -> Self {
        let g = GUARD_SHARE * (hi - lo);
        Band {
            lo: lo + g,
            hi: hi - g,
        }
    }

    /// The guard a pass of sign `dir` starts from and the one it ends at.
    pub fn near(&self, dir: i8) -> f64 {
        if dir > 0 { self.lo } else { self.hi }
    }

    pub fn far(&self, dir: i8) -> f64 {
        if dir > 0 { self.hi } else { self.lo }
    }

    /// From `start` to the far guard along `dir`.
    pub fn runway(&self, dir: i8, start: f64) -> f64 {
        dir as f64 * (self.far(dir) - start)
    }

    pub fn holds(&self, pos: f64) -> bool {
        (self.lo..=self.hi).contains(&pos)
    }

    pub fn middle(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }
}

/// The speeds seen this run each way at one duty, counts/s: the fastest,
/// which bounds a free rung, and the slowest, which sizes a stop rung,
/// each with where it was seen.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Speeds {
    pub up: (f64, String),
    pub down: (f64, String),
    pub slow_up: Option<(f64, String)>,
    pub slow_down: Option<(f64, String)>,
}

impl Speeds {
    /// The approach legs' speeds.
    pub fn of_legs(legs: &[ApproachLeg]) -> Self {
        let mut f = Speeds::default();
        for l in legs {
            f.feed(l.dir, l.speed_cps(), format!("approach leg {}", l.n));
        }
        f
    }

    pub fn get(&self, dir: i8) -> f64 {
        if dir > 0 { self.up.0 } else { self.down.0 }
    }

    pub fn from(&self, dir: i8) -> &str {
        if dir > 0 { &self.up.1 } else { &self.down.1 }
    }

    pub fn slowest(&self, dir: i8) -> Option<&(f64, String)> {
        if dir > 0 {
            self.slow_up.as_ref()
        } else {
            self.slow_down.as_ref()
        }
    }

    pub fn feed(&mut self, dir: i8, cps: f64, from: String) {
        if !(cps.is_finite() && cps > 0.0) {
            return;
        }
        let (s, low) = if dir > 0 {
            (&mut self.up, &mut self.slow_up)
        } else {
            (&mut self.down, &mut self.slow_down)
        };
        if cps > s.0 {
            *s = (cps, from.clone());
        }
        if low.as_ref().is_none_or(|l| cps < l.0) {
            *low = Some((cps, from));
        }
    }
}

/// A rung as sized.
#[derive(Clone, Debug, PartialEq)]
pub struct RungPlan {
    pub dir: i8,
    pub start: f64,
    pub far_guard: f64,
    pub runway: f64,
    pub fastest_cps: f64,
    pub bound_speed: f64,
    /// The coast's bound: the bound speed for [`COAST_TAU_S`].
    pub bound_coast: f64,
    /// The longest T the runway allows, and the burst's.
    pub t_runway_s: f64,
    pub t_burst_s: f64,
    /// The stagger's share of T.
    pub share: f64,
    /// The rows' time, rounded down to a row.
    pub t_s: f64,
    pub rows: u16,
}

impl RungPlan {
    /// The predicted travel at the bounds: the rows, the dead-man after
    /// them and the coast.
    pub fn travel_bound(&self) -> f64 {
        self.bound_speed * (self.t_s + DRIVE_DEADMAN_S) + self.bound_coast
    }

    pub fn fits(&self) -> bool {
        self.t_s >= RUNG_MIN_S
    }

    pub fn said(&self) -> String {
        format!(
            "T {:.0} ms ({:.2} of the lesser of the runway's {:.0} ms over {:.0} counts and the burst's {:.0} ms), bound {:.0} counts/s ({FREE_MARGIN} x {:.0}) and coast {:.0} counts: at most {:.0} counts to the guard at {:.0}",
            1e3 * self.t_s,
            self.share,
            1e3 * self.t_runway_s,
            self.runway,
            1e3 * self.t_burst_s,
            self.bound_speed,
            self.fastest_cps,
            self.bound_coast,
            self.travel_bound(),
            self.far_guard
        )
    }
}

/// A rung of sign `dir` from rest at `start` inside `band`, the fastest
/// speed seen that way `fastest_cps`, cut to `share` of its T, its rows at
/// `tick_hz`. It is left out unless [`RungPlan::fits`].
pub fn plan_rung(
    band: &Band,
    dir: i8,
    start: f64,
    fastest_cps: f64,
    share: f64,
    tick_hz: f64,
) -> RungPlan {
    let runway = band.runway(dir, start);
    let bound_speed = FREE_MARGIN * fastest_cps;
    let bound_coast = bound_speed * COAST_TAU_S;
    let t_runway_s = (runway - bound_coast) / bound_speed - DRIVE_DEADMAN_S;
    let t_burst_s = BURST_MAX_S - DRIVE_DEADMAN_S;
    let t = share * t_runway_s.min(t_burst_s);
    let rows = if t.is_finite() && t > 0.0 && bound_speed > 0.0 {
        (t * tick_hz).floor().min(u16::MAX as f64) as u16
    } else {
        0
    };
    RungPlan {
        dir,
        start,
        far_guard: band.far(dir),
        runway,
        fastest_cps,
        bound_speed,
        bound_coast,
        t_runway_s,
        t_burst_s,
        share,
        t_s: rows as f64 / tick_hz,
        rows,
    }
}

/// A rung read by its pot, for the runway's bookkeeping.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RungRead {
    pub start: f64,
    pub end: f64,
    /// Along its sign, counts.
    pub drive_travel: f64,
    pub travel: f64,
    /// The pot's slope over the last [`SPEED_TAIL_SHARE`] of its driven
    /// rows, along its sign.
    pub speed_cps: f64,
}

fn driven(rows: &[TelFrame]) -> &[TelFrame] {
    let on = rows
        .iter()
        .position(|f| f.duty_counts.is_some_and(|d| d != 0))
        .unwrap_or(rows.len());
    &rows[on..]
}

/// The rung of sign `dir` from `start` read by its drive rows and its rest
/// `end`; None when the rows carry too little pot.
pub fn read_rung(dir: i8, start: f64, drive: &[TelFrame], end: f64, fs: f64) -> Option<RungRead> {
    let d = dir as f64;
    let pts: Vec<(f64, f64)> = driven(drive)
        .iter()
        .enumerate()
        .filter_map(|(k, f)| f.pos.map(|p| (k as f64 / fs, p as f64)))
        .collect();
    let tail = (pts.len() as f64 * SPEED_TAIL_SHARE).round() as usize;
    if tail < 3 {
        return None;
    }
    let (t, p): (Vec<f64>, Vec<f64>) = pts[pts.len() - tail..].iter().copied().unzip();
    let [_, slope] = polyfit::<2>(&t, &p)?;
    Some(RungRead {
        start,
        end,
        drive_travel: d * (pts.last()?.1 - pts.first()?.1),
        travel: d * (end - start),
        speed_cps: d * slope,
    })
}

/// The checks after a rung, at rest: it turned, it stayed inside both
/// guards, and it ran no faster than its bound.
pub fn after(p: &RungPlan, r: &RungRead, band: &Band, what: &str) -> Result<(), String> {
    if r.drive_travel < MOVE_MIN_COUNTS {
        return Err(format!(
            "{what} did not turn the shaft: {:.0} counts in its drive, under {MOVE_MIN_COUNTS}. Nothing more drives; {HANDS}, and check that nothing holds the horn",
            r.drive_travel
        ));
    }
    let too_fast = |why: String| {
        format!(
            "{what} {why}: it ran {:.0} counts from {:.0} to {:.0} against a bound of {:.0}, at {:.0} counts/s against a bound of {:.0} from {:.0} seen. It ran faster than any speed the run had seen that way allows; nothing more drives: check that nothing changed on the horn, then start again",
            r.travel,
            r.start,
            r.end,
            p.travel_bound(),
            r.speed_cps,
            p.bound_speed,
            p.fastest_cps
        )
    };
    if !band.holds(r.end) {
        return Err(too_fast(format!(
            "ended at {:.0}, outside the guards {:.0}..{:.0}",
            r.end, band.lo, band.hi
        )));
    }
    if r.speed_cps > p.bound_speed {
        return Err(too_fast("ran faster than its bound".into()));
    }
    Ok(())
}

/// A stop rung of sign `dir` from rest at `start` toward the stop at
/// `stop`, the slowest speed seen that way `slowest_cps`: long enough to
/// cover the counts to the stop at that speed over the free margin, spin
/// up and be read still there, at most the permit's time less the
/// dead-man. The bounds stay the free rung's, for the envelope's price.
pub fn plan_stop_rung(
    dir: i8,
    start: f64,
    stop: f64,
    fastest_cps: f64,
    slowest_cps: f64,
    tick_hz: f64,
) -> RungPlan {
    let d = (dir as f64 * (stop - start)).max(0.0);
    let bound_speed = FREE_MARGIN * fastest_cps;
    let t_runway_s = d / (slowest_cps / FREE_MARGIN) + SPIN_UP_S + WINDOW_S;
    let t_burst_s = PERMIT_MAX_S - DRIVE_DEADMAN_S;
    let t = t_runway_s.min(t_burst_s);
    let rows = if t.is_finite() && t > 0.0 && slowest_cps > 0.0 {
        (t * tick_hz).floor().min(u16::MAX as f64) as u16
    } else {
        0
    };
    RungPlan {
        dir,
        start,
        far_guard: stop,
        runway: d,
        fastest_cps,
        bound_speed,
        bound_coast: bound_speed * COAST_TAU_S,
        t_runway_s,
        t_burst_s,
        share: 1.0,
        t_s: rows as f64 / tick_hz,
        rows,
    }
}

/// Where a stop rung's last [`WINDOW_S`] of rows left the shaft.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Reach {
    /// Still within [`WALK_MAX_COUNTS`] of the stop, at this median.
    Reached(f64),
    /// Still, short of the stop by more than that: a blocked shaft.
    Short(f64),
    /// Still, past the stop's reading by more than that: the stop moved.
    Past(f64),
    /// Not still: the stop is not reached yet.
    Moving,
}

/// A stop rung's drive rows of sign `dir` against the stop at `stop`, a
/// window still when every reading lies within `still_counts` of its
/// median.
pub fn reach(rows: &[TelFrame], dir: i8, stop: f64, still_counts: f64, fs: f64) -> Reach {
    let n = (WINDOW_S * fs).round() as usize;
    let pos: Vec<f64> = rows
        .iter()
        .rev()
        .take(n)
        .filter_map(|f| f.pos.map(f64::from))
        .collect();
    let Some(m) = median(&pos).filter(|_| pos.len() >= n) else {
        return Reach::Moving;
    };
    if pos.iter().any(|p| (p - m).abs() > still_counts) {
        return Reach::Moving;
    }
    let along = dir as f64 * (m - stop);
    if along < -WALK_MAX_COUNTS {
        Reach::Short(m)
    } else if along > WALK_MAX_COUNTS {
        Reach::Past(m)
    } else {
        Reach::Reached(m)
    }
}

/// The histogram's codes: rows of `codes` more than [`NEAR_STOP_COUNTS`]
/// inside both stops, and the range they are counted over.
pub fn histogram_rows(codes: &[u16], raw_min: u16, raw_max: u16) -> (Vec<u16>, u16, u16) {
    let lo = (raw_min as f64 + NEAR_STOP_COUNTS).ceil() as u16;
    let hi = (raw_max as f64 - NEAR_STOP_COUNTS).floor() as u16;
    let inside = codes
        .iter()
        .copied()
        .filter(|c| (lo..=hi).contains(c))
        .collect();
    (inside, lo, hi)
}

/// The median rows a code over the histogram's range less
/// [`EDGE_MARGIN_COUNTS`] at each end, as stage 1's finder reads its
/// floor; 0 when no code is left.
pub fn rows_a_code(codes: &[u16], raw_min: u16, raw_max: u16) -> f64 {
    let (inside, lo, hi) = histogram_rows(codes, raw_min, raw_max);
    let (a, b) = (
        lo as usize + EDGE_MARGIN_COUNTS,
        (hi as usize).saturating_sub(EDGE_MARGIN_COUNTS),
    );
    if b < a {
        return 0.0;
    }
    let h = widecodes::histogram(&inside);
    let rows: Vec<f64> = h[a..=b].iter().map(|&n| n as f64).collect();
    median(&rows).unwrap_or(0.0)
}

/// Whether the slow passes take another round trip after `trips`: the
/// plan's, then while a carry reads thin, at most
/// [`Schedule::extra_slow_max`] more.
pub fn another_trip(sched: &Schedule, trips: usize, thin: &[u16]) -> bool {
    trips < sched.slow_round_trips
        || (!thin.is_empty() && trips < sched.slow_round_trips + sched.extra_slow_max)
}

/// The carries the wide-code finder reads thin on `codes` over the
/// histogram's range; none when it refuses.
pub fn thin_carries(codes: &[u16], raw_min: u16, raw_max: u16) -> Vec<u16> {
    let (inside, lo, hi) = histogram_rows(codes, raw_min, raw_max);
    widecodes::find(&inside, lo, hi).map_or(Vec::new(), |f| {
        f.carries
            .iter()
            .filter(|c| c.state == State::Thin)
            .map(|c| c.code)
            .collect()
    })
}

/// The fast passes' duty, Q15, on `rail_v`: [`FAST_SHARE`] of the
/// voltage the nameplate states its speed at, rounded down.
pub fn fast_q15(stated_v: f64, rail_v: f64) -> i16 {
    (FAST_SHARE * stated_v / rail_v * Q15)
        .floor()
        .clamp(0.0, i16::MAX as f64) as i16
}

/// The counts/s a fast pass is predicted at before one has run: the
/// nameplate's rated speed at [`FAST_SHARE`] of its voltage.
pub fn fast_predicted_cps(rated_cps: f64) -> f64 {
    FAST_SHARE * rated_cps
}

/// The speed that bounds a fast pass of sign `dir`: the fast passes' own
/// that way once one has run, else the larger of the prediction and any
/// fast pass the other way.
pub fn fast_bound_from(measured: [Option<f64>; 2], dir: i8, predicted: f64) -> f64 {
    let (mine, other) = if dir > 0 {
        (measured[0], measured[1])
    } else {
        (measured[1], measured[0])
    };
    mine.unwrap_or_else(|| other.map_or(predicted, |o| o.max(predicted)))
}

pub struct TableInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The same run's `results/rest.json`, `probe.json` and `stops.json`.
    pub rest: &'a Value,
    pub probe: &'a Value,
    pub stops: &'a Value,
    pub schedule: Schedule,
}

#[derive(Clone, Debug)]
pub struct Tabled {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Tabled {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
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
    stops: FromStops,
    band: Band,
    raw: (u16, u16),
    approach_q15: i16,
    fast_q15: i16,
    fast_predicted_cps: f64,
    window_ticks: u32,
    pwm_arr: u16,
    still_counts: f64,
    env: Envelope,
    stated_v: f64,
    rated_cps: f64,
    ripple_per_rev: Option<u32>,
    gear_ratio: Option<f64>,
    travel_deg: f64,
    ntc: Parts,
    soft: [Vec<u8>; 2],
    soft_counts: (f64, f64),
}

fn num(
    state: &ServoState,
    d: &osc_client::descriptor::Descriptor,
    name: &str,
) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

/// A go-to window as the envelope prices it: the approach duty for a
/// window and the poll dead-man, the stops left out as the stops step
/// prices its approaches.
fn price_goto(b: &mut Before, q15: i16, pos: f64, admit: bool) -> Result<(), Refused> {
    let r = Rung {
        name: "a go-to window".into(),
        duty: q15 as f64 / Q15,
        time_s: WINDOW_S + POLL_DEADMAN_S,
        permit: false,
        gap_s: 0.0,
        start: pos,
        speed_cps: (q15.unsigned_abs() as f64 / Q15 * b.rail_v) / b.stated_v * b.rated_cps,
        stop_counts: 0.0,
        kind: Kind::Drive,
    };
    let stops = b.env.facts.stops.take();
    let out = if admit {
        b.env.admit(&r)
    } else {
        b.env.price(&r)
    };
    b.env.facts.stops = stops;
    out.map(|_| ())
}

/// A rung as the envelope prices it: at its duty for its rows and the
/// dead-man, at the speed its bound is the free margin over, then its
/// coast's bound; the coast's rows after it. A stop rung holds the
/// permit.
fn priced(name: &str, q15: i16, p: &RungPlan, permit: bool) -> Rung {
    Rung {
        name: name.into(),
        duty: q15 as f64 / Q15,
        time_s: p.t_s + DRIVE_DEADMAN_S,
        permit,
        gap_s: COAST_S,
        start: p.start,
        speed_cps: p.bound_speed / FREE_MARGIN,
        stop_counts: p.bound_coast,
        kind: Kind::Drive,
    }
}
async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &TableInputs<'_>) -> Result<Before, Refused> {
    let np = i.nameplate;
    np.check_user().map_err(Refused)?;
    let thermal = i.thermal.ok_or_else(|| {
        Refused("no thermal values entered (--thermal): the heat of a rung cannot be priced, so nothing drives".into())
    })?;
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Raw)?;
    let rest = FromRest::of(i.rest)?;
    let probe = FromProbe::of(i.probe)?;
    let stops = FromStops::of(i.stops)?;
    let ripple_per_rev = np.ripple_per_rev;
    if ripple_per_rev == Some(0) {
        return Err(Refused(
            "the nameplate's ripple_per_rev is 0: enter rotor slots times brushes, or leave it out"
                .into(),
        ));
    }
    let travel_deg = np.travel_deg.filter(|t| t.is_finite() && *t > 0.0);
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
    let np_travel = travel_deg.ok_or_else(|| {
        Refused("the nameplate has no usable travel_deg: the go-to's speed and the table's fallback scale need it".into())
    })?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let servo = meta::servo(&state, rig.d, &line);
    let pos = rest::pos_now(rig.c, rig.id, rig.d).await?;
    let rail_v = rest::rail_now(rig.c, rig.id, rig.d, &sense).await?;
    let pwm_arr = num(&state, rig.d, "pwm_period_ticks")?;
    let pwm_arr = u16::try_from(pwm_arr as i64)
        .map_err(|_| Refused(format!("pwm_period_ticks {pwm_arr} is no u16")))?;
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    let window_ticks = drive_ticks(stops.approach_q15, pwm_arr);
    if window_ticks < stream.i_window_min_ticks as u32 {
        return Err(Refused(format!(
            "the approach duty {} Q15 drives a window of {window_ticks} ticks on a PWM period of {pwm_arr}, under i_window_min_ticks {}: no row of a rung would carry a current read; nothing drives",
            stops.approach_q15, stream.i_window_min_ticks
        )));
    }
    let ntc_raw = {
        let f = field(rig.d, "ntc_raw")?;
        let b = rig.c.read(rig.id, f.addr, 2).await?;
        u16::from_le_bytes([b[0], b[1]]) as f64
    };
    let band = Band::of_stops(stops.lo, stops.hi);
    if !(stops.lo < pos && pos < stops.hi) {
        return Err(Refused(format!(
            "the shaft reads {pos:.0}, not between the stops {:.0}..{:.0}: {HANDS}",
            stops.lo, stops.hi
        )));
    }
    let still_counts = seek::still_band(rest.pot_sd);
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
        raw: stops.raw(),
        approach_q15: stops.approach_q15,
        fast_q15: fast_q15(speed.at_v, rail_v),
        fast_predicted_cps: fast_predicted_cps(
            speed.per_deg / speed.seconds * ADC_COUNTS / np_travel,
        ),
        stops,
        band,
        window_ticks,
        pwm_arr,
        still_counts,
        env: Envelope::new(facts, heat),
        stated_v: speed.at_v,
        rated_cps: speed.per_deg / speed.seconds * ADC_COUNTS / np_travel,
        ripple_per_rev,
        gear_ratio: np.gear_ratio.filter(|g| g.is_finite() && *g > 0.0),
        travel_deg: np_travel,
        ntc: sense.ntc,
        soft_counts: (word(&lo), word(&hi)),
        soft: [lo, hi],
    };
    // the first rung of the half pass, the first go-to window and the
    // first fast pass priced before any write
    let q = b.approach_q15;
    price_goto(&mut b, q, pos, false)?;
    let seen = Speeds::of_legs(&b.stops.legs);
    let p = plan_rung(&b.band, -1, pos, seen.get(-1), 1.0, b.tick_hz);
    if p.fits() {
        b.env
            .price(&priced("the half pass's first rung", -q, &p, false))?;
    }
    if b.fast_q15 <= b.approach_q15 {
        return Err(Refused(format!(
            "the fast passes' {:.3} V is no faster than the approach duty's {:.3} V: check the nameplate's speed row",
            b.fast_q15 as f64 / Q15 * rail_v,
            b.approach_q15 as f64 / Q15 * rail_v
        )));
    }
    let start = b.band.near(1);
    let p = plan_rung(
        &b.band,
        1,
        start,
        b.fast_predicted_cps,
        FIRST_FAST_SHARE,
        b.tick_hz,
    );
    if !p.fits() {
        return Err(Refused(format!(
            "the first fast pass gets {:.0} ms, under {:.0} ms ({}): the stops are too close for the nameplate's speed at {:.2} V",
            1e3 * p.t_s,
            1e3 * RUNG_MIN_S,
            p.said(),
            b.fast_q15 as f64 / Q15 * rail_v
        )));
    }
    b.env
        .price(&priced("the first fast pass", b.fast_q15, &p, false))?;
    Ok(b)
}
#[derive(Default)]
struct OnServo {
    duty: Option<i16>,
    driving: bool,
}

async fn duty<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, q15: i16) -> Result<(), Halt> {
    if q15.unsigned_abs() > b.approach_q15 as u16 {
        return Err(Halt::Error(format!(
            "a duty of {q15} was asked for, over the approach duty {}: nothing more drives",
            b.approach_q15
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

/// The go-to on the servo, as the spin step's: each act of [`GoTo`]
/// written and polled under the poll dead-man, every window priced before
/// it starts; the rest position it arrived at.
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

/// The NTC read again after a rung, for the envelope's next price.
async fn ntc_now<P: Pipe>(rig: &mut Rig<'_, P>, b: &mut Before) -> Result<(), Halt> {
    let raw = read_u16(rig, "ntc_raw").await? as f64;
    b.env.facts.ntc_c = Some(b.ntc.c_of_raw(raw));
    Ok(())
}

/// The ticks from a drive's first stamp to its coast's by the servo's
/// u16 stamps, taken as the count nearest `near`, the host's own reading
/// of it: a long rung and the host's time to LAST pass the stamps' wrap at
/// 65536 ticks, where the plain difference would read a drive held too
/// long as a short one.
pub fn stamp_ticks(drive: u16, coast: u16, near: f64) -> f64 {
    let d = coast.wrapping_sub(drive) as f64;
    d + 65536.0 * ((near - d) / 65536.0).round()
}

/// What a rung is for.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RungKind {
    /// The first free rung of a slow pass that starts on a stop.
    Leave,
    Free,
    /// Under the permit, onto the stop.
    Stop,
    /// A fast pass: one burst guard to guard.
    Fast,
}

impl RungKind {
    pub fn word(self) -> &'static str {
        match self {
            RungKind::Leave => "leave",
            RungKind::Free => "free",
            RungKind::Stop => "stop",
            RungKind::Fast => "fast",
        }
    }
}

/// What a pass is.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PassKind {
    /// From where the stops step left the shaft to the low stop.
    Half,
    Slow,
    Fast,
}

impl PassKind {
    pub fn word(self) -> &'static str {
        match self {
            PassKind::Half => "half",
            PassKind::Slow => "slow",
            PassKind::Fast => "fast",
        }
    }
}

/// One rung taken: its place, its plan, its read and where its bursts are.
#[derive(Clone, Debug)]
pub struct Taken1 {
    /// From 1, over the run.
    pub n: usize,
    /// The pass, from 1 over the run, and what it is.
    pub pass: usize,
    pub pass_kind: PassKind,
    pub kind: RungKind,
    pub attempt: usize,
    pub q15: i16,
    pub plan: RungPlan,
    pub read: RungRead,
    pub reach: Option<Reach>,
    pub on_s: Option<f64>,
    pub drive_k: usize,
    pub price: Price,
}

/// A rung's bursts that are no evidence.
#[derive(Clone, Debug)]
struct Unkept {
    n: usize,
    pass: usize,
    kind: RungKind,
    attempt: usize,
    why: String,
    captures: (usize, usize),
}

/// One pass as run.
#[derive(Clone, Debug)]
struct PassRun {
    n: usize,
    kind: PassKind,
    dir: i8,
    start: f64,
    end: f64,
}

/// The slow rows after some slow round trips: the median rows a code and
/// the carries thin.
#[derive(Clone, Debug)]
struct RowsRead {
    trips: usize,
    median: f64,
    thin: Vec<u16>,
}

#[derive(Default)]
struct Found {
    corrected: bool,
    rungs: Vec<Taken1>,
    unkept: Vec<Unkept>,
    passes: Vec<PassRun>,
    /// The rows read after each count of slow round trips from the
    /// plan's on.
    rows_a_code: Vec<RowsRead>,
    path: Vec<String>,
}

impl Found {
    fn next_n(&self) -> usize {
        self.rungs.len() + self.unkept.len() + 1
    }
}

/// A rung on the servo: its bursts and where the shaft came to rest.
struct Sent1 {
    lost: Vec<String>,
    end: f64,
    drive_k: usize,
    on_s: Option<f64>,
    price: Price,
}

/// `p` driven at `q15`: admitted to the envelope, the permit written
/// around it when it is a stop rung, driven and coasted, the shaft read at
/// rest after it, the NTC read for the next price.
#[allow(clippy::too_many_arguments)]
async fn send<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    what: &str,
    q15: i16,
    p: &RungPlan,
    permit: bool,
    plan: Value,
) -> Result<Sent1, Halt> {
    interrupted(rig.stop)?;
    let price = b.env.admit(&priced(what, q15, p, permit))?;
    let coast_rows = (COAST_S * b.tick_hz).ceil().clamp(1.0, u16::MAX as f64) as u16;
    let drive_k = taken.len();
    if permit {
        write(rig, "stall_permit", &[1]).await?;
    }
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
            rows: p.rows,
            end: End::Coast(coast_rows),
            plan: &plan,
        },
        taken,
        &b.servo,
    )
    .await;
    if permit {
        write(rig, "stall_permit", &[0]).await?;
    }
    let got = got?;
    let cut = got
        .coast
        .as_ref()
        .and_then(|c| crate::fit::coast::cut_row(&c.rows))
        .unwrap_or(0);
    let near = got.on_s * b.tick_hz;
    let on_s = got
        .drive
        .first_stamp
        .zip(got.coast.as_ref().and_then(|c| c.first_stamp))
        .map(|(d, c)| (stamp_ticks(d, c, near) + cut as f64) / b.tick_hz);
    if let Some(s) = on_s.filter(|s| *s > p.t_s + DRIVE_DEADMAN_S + 1.0 / b.tick_hz) {
        return Err(Halt::Error(format!(
            "{what} drove {s:.3} s by the servo's stamps, over the {:.3} s it was priced at: the host is too slow between the bursts",
            p.t_s + DRIVE_DEADMAN_S
        )));
    }
    let end = at_rest(rig, b).await?;
    ntc_now(rig, b).await?;
    Ok(Sent1 {
        lost: got.lost(),
        end,
        drive_k,
        on_s,
        price,
    })
}

fn plan_json(
    what: &str,
    pass: usize,
    kind: RungKind,
    q15: i16,
    p: &RungPlan,
    attempt: usize,
) -> Value {
    json!({"rung": what, "parts": ["drive", "coast"], "pass": pass, "kind": kind.word(), "dir": p.dir,
           "duty_q15": q15, "rows": p.rows, "t_s": p.t_s, "share": p.share, "start_counts": p.start,
           "bound_speed_cps": p.bound_speed, "bound_coast_counts": p.bound_coast, "attempt": attempt})
}

/// The pass a rung belongs to.
#[derive(Copy, Clone)]
struct Place {
    pass: usize,
    kind: PassKind,
    dir: i8,
}

/// Lost rows on `s`: said, kept as no evidence, an error on the last
/// attempt; refused at once when the shaft ended outside the guards.
#[allow(clippy::too_many_arguments)]
fn lost(
    found: &mut Found,
    lines: &mut Vec<String>,
    at: &Place,
    kind: RungKind,
    what: &str,
    attempt: usize,
    s: &Sent1,
    band: Option<&Band>,
) -> Result<(), Halt> {
    let why = format!("lost rows ({})", s.lost.join(", "));
    if let Some(b) = band.filter(|b| !b.holds(s.end)) {
        return Err(Halt::Refused(format!(
            "{what} ended at {:.0}, outside the guards {:.0}..{:.0}; its rows are no evidence ({why}). Nothing more drives",
            s.end, b.lo, b.hi
        )));
    }
    lines.push(format!(
        "{what}, attempt {attempt}: {why}, no evidence (captures {:02} and {:02})",
        s.drive_k + 1,
        s.drive_k + 2
    ));
    found.unkept.push(Unkept {
        n: found.next_n(),
        pass: at.pass,
        kind,
        attempt,
        why: why.clone(),
        captures: (s.drive_k + 1, s.drive_k + 2),
    });
    if attempt == ATTEMPTS {
        return Err(Halt::Error(format!(
            "{what}: {ATTEMPTS} attempts in a row were no evidence, the last as {why}: the line is not clean enough; check the cable and its contacts"
        )));
    }
    Ok(())
}

/// One free rung of a slow pass from rest at `start`, taken again once
/// when a burst lost rows. None when it does not fit, else where it
/// rests.
#[allow(clippy::too_many_arguments)]
async fn free_rung<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    seen: &mut Speeds,
    at: &Place,
    leave: bool,
    start: f64,
    lines: &mut Vec<String>,
) -> Result<Option<f64>, Halt> {
    let mut start = start;
    let mut kind = if leave {
        RungKind::Leave
    } else {
        RungKind::Free
    };
    for attempt in 1..=ATTEMPTS {
        let p = plan_rung(&b.band, at.dir, start, seen.get(at.dir), 1.0, b.tick_hz);
        if !p.fits() {
            found.path.push(format!(
                "pass {} {}: no further free rung from {start:.0}: {}",
                at.pass,
                way(at.dir),
                p.said()
            ));
            return Ok(None);
        }
        let n = found.next_n();
        let what = format!(
            "rung {n} (pass {} {}, {})",
            at.pass,
            way(at.dir),
            kind.word()
        );
        found.path.push(format!(
            "{what}, attempt {attempt}: {}; bound from {}",
            p.said(),
            seen.from(at.dir)
        ));
        let q15 = b.approach_q15 * at.dir as i16;
        let plan = plan_json(&what, at.pass, kind, q15, &p, attempt);
        let s = send(rig, b, taken, &what, q15, &p, false, plan).await?;
        if !s.lost.is_empty() {
            lost(found, lines, at, kind, &what, attempt, &s, Some(&b.band))?;
            start = s.end;
            kind = RungKind::Free;
            continue;
        }
        let rows = &taken[s.drive_k].burst.rows;
        let read = read_rung(at.dir, start, rows, s.end, b.tick_hz)
            .ok_or_else(|| Halt::Error(format!("{what}: its bursts carry too few rows to read")))?;
        lines.push(format!(
            "{what}: from {start:.0} for {:.0} ms: {:.0} of {:.0} counts/s, at rest at {:.0} ({:.0} of {:.0} counts)",
            1e3 * p.t_s,
            read.speed_cps,
            p.bound_speed,
            s.end,
            read.travel,
            p.travel_bound()
        ));
        if kind == RungKind::Leave && read.drive_travel < PLAY_COUNTS {
            return Err(Halt::Refused(format!(
                "{what}: the shaft did not leave the stop at the approach duty: its pot moved {:.0} counts, under {PLAY_COUNTS}. Nothing more drives; {HANDS}",
                read.drive_travel
            )));
        }
        after(&p, &read, &b.band, &what).map_err(Halt::Refused)?;
        seen.feed(at.dir, read.speed_cps, what.clone());
        found.rungs.push(Taken1 {
            n,
            pass: at.pass,
            pass_kind: at.kind,
            kind,
            attempt,
            q15,
            plan: p,
            read,
            reach: None,
            on_s: s.on_s,
            drive_k: s.drive_k,
            price: s.price,
        });
        return Ok(Some(s.end));
    }
    Ok(None)
}

/// The stop rungs that end a slow pass on the stop it drives toward, from
/// rest at `start`: at most [`STOP_RUNGS_MAX`] until one reads the stop
/// reached, each burst that lost rows taken again once. Where it rests.
#[allow(clippy::too_many_arguments)]
async fn stop_rungs<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    seen: &Speeds,
    at: &Place,
    start: f64,
    lines: &mut Vec<String>,
) -> Result<f64, Halt> {
    let stop = if at.dir > 0 { b.stops.hi } else { b.stops.lo };
    let word = if at.dir > 0 { "high" } else { "low" };
    let mut start = start;
    let mut rungs = 0;
    let mut attempt = 0;
    while rungs < STOP_RUNGS_MAX {
        let slowest = seen.slowest(at.dir).map_or(seen.get(at.dir), |s| s.0);
        let p = plan_stop_rung(at.dir, start, stop, seen.get(at.dir), slowest, b.tick_hz);
        let n = found.next_n();
        let what = format!(
            "rung {n} (pass {} {}, onto the {word} stop)",
            at.pass,
            way(at.dir)
        );
        attempt += 1;
        found.path.push(format!(
            "{what}, attempt {attempt}: T {:.0} ms ({:.0} counts to the stop at {:.0} at {:.0} counts/s over the free margin, the spin-up and a still window, at most {:.0} ms), the permit held",
            1e3 * p.t_s,
            p.runway,
            stop,
            slowest,
            1e3 * p.t_burst_s
        ));
        let q15 = b.approach_q15 * at.dir as i16;
        let plan = plan_json(&what, at.pass, RungKind::Stop, q15, &p, attempt);
        let s = send(rig, b, taken, &what, q15, &p, true, plan).await?;
        if !s.lost.is_empty() {
            lost(found, lines, at, RungKind::Stop, &what, attempt, &s, None)?;
            start = s.end;
            continue;
        }
        attempt = 0;
        rungs += 1;
        let rows = &taken[s.drive_k].burst.rows;
        let r = reach(rows, at.dir, stop, b.still_counts, b.tick_hz);
        let read = read_rung(at.dir, start, rows, s.end, b.tick_hz).unwrap_or(RungRead {
            start,
            end: s.end,
            drive_travel: 0.0,
            travel: at.dir as f64 * (s.end - start),
            speed_cps: 0.0,
        });
        lines.push(format!(
            "{what}: from {start:.0} for {:.0} ms: {}, at rest at {:.0}",
            1e3 * p.t_s,
            match r {
                Reach::Reached(m) => format!("on the stop at {m:.0}"),
                Reach::Short(m) => format!("still at {m:.0}, short of the stop"),
                Reach::Past(m) => format!("still at {m:.0}, past the stop"),
                Reach::Moving => "still moving at its end".into(),
            },
            s.end
        ));
        found.rungs.push(Taken1 {
            n,
            pass: at.pass,
            pass_kind: at.kind,
            kind: RungKind::Stop,
            attempt: 1,
            q15,
            plan: p,
            read,
            reach: Some(r),
            on_s: s.on_s,
            drive_k: s.drive_k,
            price: s.price,
        });
        match r {
            Reach::Reached(_) => {
                if (s.end - stop).abs() > WALK_MAX_COUNTS {
                    return Err(Halt::Refused(format!(
                        "{what} read the {word} stop reached, then came to rest at {:.0}, {:.0} counts off its reading {stop:.0}, over {WALK_MAX_COUNTS}: the stopper gives or the shaft springs back. Nothing more drives",
                        s.end,
                        (s.end - stop).abs()
                    )));
                }
                return Ok(s.end);
            }
            Reach::Short(m) => {
                return Err(Halt::Refused(format!(
                    "{what} went still at {m:.0}, {:.0} counts short of the {word} stop's {stop:.0}: the shaft is blocked between the stops. Nothing more drives; {HANDS}, and check that nothing holds the horn",
                    (m - stop).abs()
                )));
            }
            Reach::Past(m) => {
                return Err(Halt::Refused(format!(
                    "{what} went still at {m:.0}, {:.0} counts past the {word} stop's {stop:.0}, over {WALK_MAX_COUNTS}: the stop moved. Nothing more drives; run the stops step again",
                    (m - stop).abs()
                )));
            }
            Reach::Moving => start = s.end,
        }
    }
    Err(Halt::Refused(format!(
        "pass {} {}: {STOP_RUNGS_MAX} stop rungs did not reach the {word} stop at {stop:.0}; the shaft rests at {start:.0}. Nothing more drives",
        at.pass,
        way(at.dir)
    )))
}

/// A slow pass from rest at `start` to the stop it drives toward.
#[allow(clippy::too_many_arguments)]
async fn slow_pass<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    seen: &mut Speeds,
    at: Place,
    start: f64,
    lines: &mut Vec<String>,
) -> Result<f64, Halt> {
    let mut pos = start;
    for k in 0..FREE_RUNGS_MAX {
        let leave = k == 0 && at.kind == PassKind::Slow;
        match free_rung(rig, b, taken, found, seen, &at, leave, pos, lines).await? {
            Some(end) => pos = end,
            None => break,
        }
    }
    let end = stop_rungs(rig, b, taken, found, seen, &at, pos, lines).await?;
    found.passes.push(PassRun {
        n: at.pass,
        kind: at.kind,
        dir: at.dir,
        start,
        end,
    });
    Ok(end)
}

/// A fast pass: a go-to to its near guard, then one burst toward the far
/// guard, at half length while the first, taken again once when a burst
/// lost rows.
#[allow(clippy::too_many_arguments)]
async fn fast_pass<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    fast: &mut [Option<f64>; 2],
    at: Place,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let name = if at.dir > 0 {
        "the low guard"
    } else {
        "the high guard"
    };
    for attempt in 1..=ATTEMPTS {
        let start = go_to(rig, b, b.band.near(at.dir), name, lines).await?;
        let first = fast.iter().all(Option::is_none);
        let bound = fast_bound_from(*fast, at.dir, b.fast_predicted_cps);
        let share = if first { FIRST_FAST_SHARE } else { 1.0 };
        let p = plan_rung(&b.band, at.dir, start, bound, share, b.tick_hz);
        let n = found.next_n();
        let what = format!("rung {n} (pass {} {}, fast)", at.pass, way(at.dir));
        if !p.fits() {
            return Err(Halt::Refused(format!(
                "{what} from {start:.0} gets {:.0} ms, under {:.0} ms ({}): the stops are too close for the fast passes' speed",
                1e3 * p.t_s,
                1e3 * RUNG_MIN_S,
                p.said()
            )));
        }
        found.path.push(format!(
            "{what}, attempt {attempt}: {}; bound from {}",
            p.said(),
            if fast.iter().all(Option::is_none) {
                "the nameplate's speed per volt".to_string()
            } else {
                "the fast passes run".into()
            }
        ));
        let q15 = b.fast_q15 * at.dir as i16;
        let plan = plan_json(&what, at.pass, RungKind::Fast, q15, &p, attempt);
        let s = send(rig, b, taken, &what, q15, &p, false, plan).await?;
        if !s.lost.is_empty() {
            lost(
                found,
                lines,
                &at,
                RungKind::Fast,
                &what,
                attempt,
                &s,
                Some(&b.band),
            )?;
            continue;
        }
        let rows = &taken[s.drive_k].burst.rows;
        let read = read_rung(at.dir, start, rows, s.end, b.tick_hz)
            .ok_or_else(|| Halt::Error(format!("{what}: its bursts carry too few rows to read")))?;
        lines.push(format!(
            "{what}: from {start:.0} for {:.0} ms at {q15} Q15: {:.0} of {:.0} counts/s, at rest at {:.0} ({:.0} of {:.0} counts)",
            1e3 * p.t_s,
            read.speed_cps,
            p.bound_speed,
            s.end,
            read.travel,
            p.travel_bound()
        ));
        after(&p, &read, &b.band, &what).map_err(Halt::Refused)?;
        let slot = &mut fast[if at.dir > 0 { 0 } else { 1 }];
        *slot = Some(slot.map_or(read.speed_cps, |v| v.max(read.speed_cps)));
        found.rungs.push(Taken1 {
            n,
            pass: at.pass,
            pass_kind: at.kind,
            kind: RungKind::Fast,
            attempt,
            q15,
            plan: p,
            read,
            reach: None,
            on_s: s.on_s,
            drive_k: s.drive_k,
            price: s.price,
        });
        found.passes.push(PassRun {
            n: at.pass,
            kind: at.kind,
            dir: at.dir,
            start,
            end: s.end,
        });
        return Ok(());
    }
    Ok(())
}

/// The slow passes' driven pot codes so far.
fn slow_codes(found: &Found, taken: &[Taken]) -> Vec<u16> {
    found
        .rungs
        .iter()
        .filter(|r| r.pass_kind != PassKind::Fast)
        .flat_map(|r| driven_codes(&taken[r.drive_k].burst.rows))
        .collect()
}

fn driven_codes(rows: &[TelFrame]) -> impl Iterator<Item = u16> + '_ {
    rows.iter()
        .filter(|f| f.duty_counts.is_some_and(|d| d != 0))
        .filter_map(|f| f.pos)
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    sched: Schedule,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    found.corrected =
        stops::open_loop(rig, ledger, &b.probe, &b.soft, b.deadman_ticks, lines).await?;
    let v = |q: i16| q as f64 / Q15 * b.rail_v;
    lines.push(format!(
        "volatile: the peak cap unset, the soft limits out to the physical range {:.0}..{:.0}, OpenLoop, dead-man {} ticks ({} s), torque on at zero duty; guards {:.0}..{:.0}, {:.0} % of the stops {:.0}..{:.0} in from each; slow passes at the approach duty {:.3} V ({} ticks of drive window, i_window_min_ticks {}), fast passes at {:.3} V; each rung under a {DRIVE_DEADMAN_S} s dead-man, {COAST_S} s of coast",
        b.soft_counts.0,
        b.soft_counts.1,
        b.deadman_ticks,
        rest::DEADMAN_S,
        b.band.lo,
        b.band.hi,
        100.0 * GUARD_SHARE,
        b.stops.lo,
        b.stops.hi,
        v(b.approach_q15),
        b.window_ticks,
        b.stream.i_window_min_ticks,
        v(b.fast_q15)
    ));
    let mut seen = Speeds::of_legs(&b.stops.legs);
    let mut pass = 1;
    let place = |pass, kind, dir| Place { pass, kind, dir };
    let mut at = slow_pass(
        rig,
        b,
        taken,
        found,
        &mut seen,
        place(pass, PassKind::Half, -1),
        b.pos,
        lines,
    )
    .await?;
    let (raw_min, raw_max) = b.raw;
    let mut trips = 0;
    loop {
        if trips >= sched.slow_round_trips {
            let codes = slow_codes(found, taken);
            let thin = thin_carries(&codes, raw_min, raw_max);
            let median = rows_a_code(&codes, raw_min, raw_max);
            found.rows_a_code.push(RowsRead {
                trips,
                median,
                thin: thin.clone(),
            });
            let more = another_trip(&sched, trips, &thin);
            if !thin.is_empty() {
                lines.push(if more {
                    format!(
                        "after {trips} slow round trips the carries {thin:?} read under {THIN_ROWS} reference rows a code (median {median:.0} a code): one more slow round trip"
                    )
                } else {
                    format!(
                        "after {trips} slow round trips the carries {thin:?} still read under {THIN_ROWS} reference rows a code: their slots stay 0"
                    )
                });
            }
            if !more {
                break;
            }
        }
        trips += 1;
        for dir in [1i8, -1] {
            pass += 1;
            at = slow_pass(
                rig,
                b,
                taken,
                found,
                &mut seen,
                place(pass, PassKind::Slow, dir),
                at,
                lines,
            )
            .await?;
        }
    }
    let mut fast = [None; 2];
    for _ in 0..sched.fast_round_trips {
        for dir in [1i8, -1] {
            pass += 1;
            fast_pass(
                rig,
                b,
                taken,
                found,
                &mut fast,
                place(pass, PassKind::Fast, dir),
                lines,
            )
            .await?;
        }
    }
    go_to(rig, b, b.band.middle(), "the middle", lines).await?;
    Ok(())
}

fn check(name: &str, value: String, bound: String, verdict: Verdict) -> Check {
    Check {
        name: name.into(),
        value,
        bound,
        verdict,
    }
}

/// The check names, for [`advice`].
pub const WIDE_CODES: &str = "wide codes";
pub const ROWS: &str = "rows a code";
pub const FIT: &str = "table fit";
pub const RESIDUAL: &str = "table residual";
pub const PASS_OUT: &str = "one fast pass out";
pub const CYCLES: &str = "ripple cycles";
pub const GAINS: &str = "table gains";
pub const DIRECTIONS: &str = "directions";
pub const SPEED: &str = "speed along a rung";

/// What to do about a refused check.
pub fn advice(name: &str) -> &'static str {
    match name {
        WIDE_CODES => {
            "the pot's codes read a pattern the slot map cannot hold, or the sweep did not repeat; check the pot's wiring, then run the step again"
        }
        ROWS => "the shaft runs too fast at the approach duty for the codes to be counted",
        FIT | RESIDUAL | PASS_OUT | CYCLES | GAINS => {
            "the ripple does not count the pot's travel cleanly; check that nothing loads the horn and that the motor's current reads, then run the step again"
        }
        _ => "check the servo, then run the step again",
    }
}

/// One rung handed to the judge: its pass and its drive rows.
#[derive(Copy, Clone, Debug)]
pub struct RungRows<'a> {
    pub n: usize,
    pub pass: usize,
    pub pass_kind: PassKind,
    pub dir: i8,
    pub rows: &'a [TelFrame],
}

/// What the judge needs besides the rows.
#[derive(Clone, Debug)]
pub struct JudgeCfg {
    pub fs: f64,
    pub raw_min: u16,
    pub raw_max: u16,
    pub ripple_per_rev: Option<u32>,
    pub gear_ratio: Option<f64>,
    pub travel_deg: f64,
}

/// One rung read against its ripple.
#[derive(Clone, Debug)]
pub struct RungSegment {
    pub n: usize,
    pub pass: usize,
    pub pass_kind: PassKind,
    pub dir: i8,
    /// The segment, or why the rung is left out.
    pub read: Result<Segment, String>,
}

/// The table with one fast pass left out, against the full one.
#[derive(Clone, Debug)]
pub struct PassOut {
    pub n: usize,
    pub pass: usize,
    /// Counts rms, line out.
    pub moved: Result<f64, TableError>,
}

/// Two same-direction rungs' ripple cycles over the counts they share.
#[derive(Copy, Clone, Debug)]
pub struct Cycles {
    pub a: usize,
    pub b: usize,
    pub agree: Agreement,
}

/// A stretch of the table only one direction's samples reach, counts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct OneWay {
    pub from: f64,
    pub to: f64,
    pub dir: i8,
}

/// The table's reading.
#[derive(Clone, Debug)]
pub struct TableRead {
    pub rungs: Vec<RungSegment>,
    pub fit: Result<PooledFit, TableError>,
    /// Counts rms at the table's mean slope.
    pub residual_counts: Option<f64>,
    pub points: Option<[i16; INTERVALS]>,
    pub valid: Option<Result<(), pos_lut::Reject>>,
    pub gains: Option<(i32, i32)>,
    pub pass_out: Vec<PassOut>,
    pub cycles: Vec<Cycles>,
    /// Up less down on the common span, counts.
    pub term: Result<Term, TableError>,
    pub one_way: Vec<OneWay>,
    /// Counts a degree and its source.
    pub scale: Option<(f64, &'static str)>,
}

/// The capture's reading: the slots, the table and every check.
pub struct Judged {
    pub finding: Option<Finding>,
    pub halves: Option<(Finding, Finding)>,
    /// The histogram's range, the stops less [`NEAR_STOP_COUNTS`].
    pub swept: (u16, u16),
    pub rows_a_code: f64,
    pub table: TableRead,
    pub checks: Vec<Check>,
}

pub fn segment_said(e: &SegmentError) -> String {
    match e {
        SegmentError::NoDrive => "no driven row".into(),
        SegmentError::Still => format!("the pot did not move {PLAY_COUNTS} counts from the edge"),
        SegmentError::Short => "too few rows past the spin-up".into(),
        SegmentError::NoLine => format!(
            "no ripple line inside {:.0}..{:.0} Hz",
            crate::ripple::BAND_HZ.0,
            crate::ripple::BAND_HZ.1
        ),
        SegmentError::FewCrossings(n) => format!("{n} ripple crossings"),
    }
}

pub fn table_said(e: &TableError) -> String {
    let way = |d: i8| match d {
        1 => "the up rungs'",
        -1 => "the down rungs'",
        _ => "the pooled",
    };
    match e {
        TableError::OneWay => "no rung read in one of the directions".into(),
        TableError::NoSpan => "the rungs share under three grid points".into(),
        TableError::NoSamples(d) => format!("{} samples miss the span", way(*d)),
        TableError::Singular => "the fit is singular".into(),
        TableError::Hole {
            dir,
            from,
            intervals,
        } => format!(
            "{} samples leave {intervals} grid intervals in a row empty from {from:.0} counts, over {}",
            way(*dir),
            postable::HOLE_MAX
        ),
        TableError::SpanMismatch => "the tables' spans differ".into(),
    }
}

/// A rung's rows as the table reads them: a slow rung's cut where the pot
/// first comes within [`NEAR_STOP_COUNTS`] of the stop it drives toward,
/// and any rung's samples within that band of either stop dropped.
fn rung_segment(r: &RungRows, slots: &[u8; ADC_WIDE_SLOTS], c: &JudgeCfg) -> RungSegment {
    let (lo, hi) = (
        c.raw_min as f64 + NEAR_STOP_COUNTS,
        c.raw_max as f64 - NEAR_STOP_COUNTS,
    );
    let entered = |f: &TelFrame| {
        f.pos.is_some_and(|p| {
            let p = p as f64;
            if r.dir > 0 { p > hi } else { p < lo }
        })
    };
    let on = r.rows.len() - driven(r.rows).len();
    let rows = match r.pass_kind {
        PassKind::Fast => r.rows,
        _ => match r.rows[on..].iter().position(entered) {
            Some(0) => {
                return RungSegment {
                    n: r.n,
                    pass: r.pass,
                    pass_kind: r.pass_kind,
                    dir: r.dir,
                    read: Err(format!(
                        "it starts within {NEAR_STOP_COUNTS} counts of the stop it drives toward"
                    )),
                };
            }
            Some(k) => &r.rows[..on + k],
            None => r.rows,
        },
    };
    let (mut pos, mut cur, mut duty) = (Vec::new(), Vec::new(), Vec::new());
    for f in rows {
        if let (Some(p), Some(i), Some(d)) = (f.pos, f.current_raw, f.duty_counts) {
            pos.push(p);
            cur.push(i);
            duty.push(d);
        }
    }
    let read = postable::segment(&pos, &cur, &duty, slots, c.fs)
        .and_then(|mut s| {
            let keep: Vec<bool> = s.x.iter().map(|&x| (lo..=hi).contains(&x)).collect();
            let pick = |v: &[f64]| -> Vec<f64> {
                v.iter()
                    .zip(&keep)
                    .filter(|(_, k)| **k)
                    .map(|(v, _)| *v)
                    .collect()
            };
            (s.x, s.y) = (pick(&s.x), pick(&s.y));
            if s.x.len() < 2 {
                return Err(SegmentError::Short);
            }
            Ok(s)
        })
        .map_err(|e| segment_said(&e));
    RungSegment {
        n: r.n,
        pass: r.pass,
        pass_kind: r.pass_kind,
        dir: r.dir,
        read,
    }
}

/// Where only one direction's samples reach inside `fit`'s span.
fn one_way(fit: &PooledFit) -> Vec<OneWay> {
    let (Some(up), Some(down)) = (fit.up, fit.down) else {
        return Vec::new();
    };
    let (lo, hi) = (fit.table.span.lo(), fit.table.span.hi());
    let mut out = Vec::new();
    let (first, dir) = if up.0 < down.0 {
        (down.0, 1)
    } else {
        (up.0, -1)
    };
    if first > lo {
        out.push(OneWay {
            from: lo,
            to: first.min(hi),
            dir,
        });
    }
    let (last, dir) = if up.1 > down.1 {
        (down.1, 1)
    } else {
        (up.1, -1)
    };
    if last < hi {
        out.push(OneWay {
            from: last.max(lo),
            to: hi,
            dir,
        });
    }
    out
}

/// The rungs to the table: each read against its ripple with `slots`,
/// the pooled fit over every rung read, and what is judged on it.
pub fn read_table(rungs: &[RungRows], slots: &[u8; ADC_WIDE_SLOTS], c: &JudgeCfg) -> TableRead {
    let read: Vec<RungSegment> = rungs.iter().map(|r| rung_segment(r, slots, c)).collect();
    let used: Vec<(&RungSegment, &Segment)> = read
        .iter()
        .filter_map(|r| r.read.as_ref().ok().map(|s| (r, s)))
        .collect();
    let segs: Vec<Segment> = used.iter().map(|(_, s)| (*s).clone()).collect();
    let span = postable::pooled_span(&segs);
    let fit = span.and_then(|sp| postable::fit_pooled(&segs, &sp, SMOOTH_WEIGHT));
    let mut t = TableRead {
        rungs: Vec::new(),
        fit: fit.clone(),
        residual_counts: None,
        points: None,
        valid: None,
        gains: None,
        pass_out: Vec::new(),
        cycles: Vec::new(),
        term: Err(TableError::NoSpan),
        one_way: Vec::new(),
        scale: None,
    };
    for (i, (a, sa)) in used.iter().enumerate() {
        for (b, sb) in &used[i + 1..] {
            if a.pass != b.pass
                && let Some(agree) = postable::agreement(sa, sb)
            {
                t.cycles.push(Cycles {
                    a: a.n,
                    b: b.n,
                    agree,
                });
            }
        }
    }
    if let Ok(f) = &fit {
        let span = f.table.span;
        t.residual_counts = Some(f.residual_rms * f.table.counts_per_cycle());
        let points = f.table.points(c.raw_min, c.raw_max);
        let mut full = [0i16; POINTS];
        full[..INTERVALS].copy_from_slice(&points);
        t.valid = Some(pos_lut::validate(&full, c.raw_min, c.raw_max));
        t.gains = Some(postable::gains(&points));
        t.points = Some(points);
        for (k, (r, _)) in used.iter().enumerate() {
            if r.pass_kind != PassKind::Fast {
                continue;
            }
            let rest: Vec<Segment> = segs
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != k)
                .map(|(_, s)| s.clone())
                .collect();
            let moved = postable::fit_pooled(&rest, &span, SMOOTH_WEIGHT)
                .and_then(|o| postable::repeat(&o.table, &f.table))
                .map(|r| r.line_out);
            t.pass_out.push(PassOut {
                n: r.n,
                pass: r.pass,
                moved,
            });
        }
        t.term = postable::common_span(&segs).and_then(|common| {
            let up = postable::fit_direction(&segs, 1, &common, SMOOTH_WEIGHT)?;
            let down = postable::fit_direction(&segs, -1, &common, SMOOTH_WEIGHT)?;
            Ok(postable::direction_term(&up, &down, &f.table))
        });
        t.one_way = one_way(f);
        t.scale = match f.table.scale(c.ripple_per_rev, c.gear_ratio) {
            Some(v) => Some((v, SCALE_RIPPLE)),
            None => Some((
                (c.raw_max as f64 - c.raw_min as f64) / c.travel_deg,
                SCALE_TRAVEL,
            )),
        };
    }
    t.rungs = read;
    t
}

pub const SCALE_RIPPLE: &str = "the ripple: the nameplate's ripple_per_rev x gear_ratio over the table's mean cycles a count, x 360";
pub const SCALE_TRAVEL: &str = "the stops' span over the nameplate's travel_deg";

/// The slow passes' driven rows to the wide-code slots, then the table on
/// them: the finder on the rows more than [`NEAR_STOP_COUNTS`] inside both
/// stops, its halves the odd and the even slow passes; the pooled fit on
/// every rung given the slots found.
pub fn judge(rungs: &[RungRows], c: &JudgeCfg) -> Judged {
    let slow = |odd: Option<bool>| -> Vec<u16> {
        let codes: Vec<u16> = rungs
            .iter()
            .filter(|r| r.pass_kind != PassKind::Fast)
            .filter(|r| odd.is_none_or(|o| (r.pass % 2 == 1) == o))
            .flat_map(|r| driven_codes(r.rows))
            .collect();
        histogram_rows(&codes, c.raw_min, c.raw_max).0
    };
    let (_, lo, hi) = histogram_rows(&[], c.raw_min, c.raw_max);
    let all = slow(None);
    let rows_a_code = rows_a_code(&all, c.raw_min, c.raw_max);
    let mut checks = vec![check(
        ROWS,
        format!(
            "median {rows_a_code:.1} over codes {}..{} from the slow passes",
            lo as usize + EDGE_MARGIN_COUNTS,
            (hi as usize).saturating_sub(EDGE_MARGIN_COUNTS)
        ),
        format!("at least {ROWS_A_CODE_MIN}"),
        if rows_a_code >= ROWS_A_CODE_MIN {
            Verdict::Pass
        } else {
            Verdict::Fail
        },
    )];
    let mut finding = None;
    let mut halves = None;
    let wide = (|| {
        let f = widecodes::find(&all, lo, hi).map_err(|e| e.said())?;
        let half = |v: &[u16]| {
            widecodes::find_in(&widecodes::histogram(v), lo, hi, HALF_THIN_ROWS)
                .map_err(|e| e.said())
        };
        let (even, odd) = (half(&slow(Some(false)))?, half(&slow(Some(true)))?);
        let ok = widecodes::check(&f, &even, &odd, c.raw_min, c.raw_max).map_err(|e| e.said());
        finding = Some(f);
        halves = Some((even, odd));
        ok
    })();
    let reached = finding.as_ref().map_or(0, |f: &Finding| {
        f.carries
            .iter()
            .filter(|k| k.state != State::Unreached)
            .count()
    });
    checks.push(match &wide {
        Ok(()) => check(
            WIDE_CODES,
            format!(
                "slots {:?}, {reached} of 16 carries reached",
                finding.as_ref().map(|f| f.slots).unwrap_or_default()
            ),
            format!(
                "no stray run, odd and even slow passes within {} Q4 at every carry both read, at least half the carries between the stops reached",
                widecodes::Q4_PER_SKIPPED
            ),
            Verdict::Pass,
        ),
        Err(why) => check(
            WIDE_CODES,
            why.clone(),
            "no stray run, the halves within one unit, half the carries reached".into(),
            Verdict::Fail,
        ),
    });
    let slots = finding.as_ref().map_or([0; ADC_WIDE_SLOTS], |f| f.slots);
    let table = read_table(rungs, &slots, c);
    checks.extend(table_checks(&table, c));
    Judged {
        finding,
        halves,
        swept: (lo, hi),
        rows_a_code,
        table,
        checks,
    }
}

fn pass_fail(ok: bool) -> Verdict {
    if ok { Verdict::Pass } else { Verdict::Fail }
}

/// The table's checks: the fit, its residual, one fast pass out, the
/// ripple cycles and the gains refuse; the directions and the speed along
/// a rung are said.
fn table_checks(t: &TableRead, c: &JudgeCfg) -> Vec<Check> {
    let unread: Vec<String> = t
        .rungs
        .iter()
        .filter_map(|r| {
            r.read
                .as_ref()
                .err()
                .map(|e| format!("rung {} ({}): {e}", r.n, r.pass_kind.word()))
        })
        .collect();
    let unread = if unread.is_empty() {
        "none".to_string()
    } else {
        unread.join(", ")
    };
    let f = match &t.fit {
        Ok(f) => f,
        Err(e) => {
            return vec![check(
                FIT,
                format!("{}; rungs not read: {unread}", table_said(e)),
                format!(
                    "a rung read each way, no hole over {} grid intervals",
                    postable::HOLE_MAX
                ),
                Verdict::Fail,
            )];
        }
    };
    let mut out = vec![check(
        FIT,
        format!(
            "{} rungs, {} samples over {:.0}..{:.0} counts ({:.0} % of the stops' span); not read: {}",
            f.offsets.len(),
            f.samples,
            f.table.span.lo(),
            f.table.span.hi(),
            100.0 * f.table.span.coverage(c.raw_min, c.raw_max),
            unread
        ),
        format!(
            "a rung read each way, no hole over {} grid intervals",
            postable::HOLE_MAX
        ),
        Verdict::Pass,
    )];
    let res = t.residual_counts.unwrap_or(f64::INFINITY);
    out.push(check(
        RESIDUAL,
        format!("{res:.2} counts rms"),
        format!("at most {RESIDUAL_MAX_COUNTS}"),
        pass_fail(res <= RESIDUAL_MAX_COUNTS),
    ));
    out.push(if t.pass_out.is_empty() {
        check(
            PASS_OUT,
            "no fast pass read".into(),
            format!("at most {PASS_OUT_MAX_COUNTS} counts rms, line out"),
            Verdict::NotJudged,
        )
    } else {
        let said: Vec<String> = t
            .pass_out
            .iter()
            .map(|p| match &p.moved {
                Ok(m) => format!("rung {} out {m:.2}", p.n),
                Err(e) => format!("rung {} out: no table ({})", p.n, table_said(e)),
            })
            .collect();
        check(
            PASS_OUT,
            format!("counts rms, line out: {}", said.join("; ")),
            format!("at most {PASS_OUT_MAX_COUNTS} counts rms, line out, each"),
            pass_fail(
                t.pass_out
                    .iter()
                    .all(|p| p.moved.as_ref().is_ok_and(|m| *m <= PASS_OUT_MAX_COUNTS)),
            ),
        )
    });
    let off: Vec<String> = t
        .cycles
        .iter()
        .filter(|k| !k.agree.holds())
        .map(|k| {
            format!(
                "rungs {} and {}: {:.2} and {:.2} cycles over {:.0}..{:.0}",
                k.a, k.b, k.agree.a, k.agree.b, k.agree.from, k.agree.to
            )
        })
        .collect();
    out.push(check(
        CYCLES,
        if t.cycles.is_empty() {
            "no two rungs one way share a grid interval".into()
        } else if off.is_empty() {
            format!("{} pairs agree", t.cycles.len())
        } else {
            off.join("; ")
        },
        format!(
            "within {:.1} % or {} cycle over the counts each pair shares",
            100.0 * postable::AGREE_SHARE,
            postable::AGREE_MIN_CYCLES
        ),
        if t.cycles.is_empty() {
            Verdict::NotJudged
        } else {
            pass_fail(off.is_empty())
        },
    ));
    let (glo, ghi) = t.gains.unwrap_or((0, 0));
    out.push(check(
        GAINS,
        match t.valid {
            Some(Ok(())) => format!("{glo}..{ghi} Q4"),
            Some(Err(r)) => format!("{glo}..{ghi} Q4, refused as {r:?}"),
            None => "not read".into(),
        },
        "the firmware's pos_lut::validate: zero at and past the stops, every gain in 1..256 Q4"
            .into(),
        pass_fail(matches!(t.valid, Some(Ok(())))),
    ));
    out.push(match &t.term {
        Ok(term) => check(
            DIRECTIONS,
            format!(
                "up less down {:.2} counts rms, {:.2} max, line out",
                term.rms, term.max
            ),
            format!("said; a warning over {DIRECTION_WARN_COUNTS} counts rms"),
            if term.rms > DIRECTION_WARN_COUNTS {
                Verdict::Warn
            } else {
                Verdict::Pass
            },
        ),
        Err(e) => check(
            DIRECTIONS,
            format!("not read: {}", table_said(e)),
            format!("said; a warning over {DIRECTION_WARN_COUNTS} counts rms"),
            Verdict::NotJudged,
        ),
    });
    let spread: Vec<String> = t
        .rungs
        .iter()
        .filter_map(|r| {
            let s = r.read.as_ref().ok()?;
            (s.speed_spread > SPEED_SPREAD_WARN)
                .then(|| format!("rung {} {:.0} %", r.n, 100.0 * s.speed_spread))
        })
        .collect();
    out.push(check(
        SPEED,
        if spread.is_empty() {
            format!("every rung under {:.0} %", 100.0 * SPEED_SPREAD_WARN)
        } else {
            spread.join(", ")
        },
        format!(
            "said; a warning over {:.0} % about the mean",
            100.0 * SPEED_SPREAD_WARN
        ),
        if spread.is_empty() {
            Verdict::Pass
        } else {
            Verdict::Warn
        },
    ));
    out
}

/// The table step on the servo: the passes, then the way out, then the
/// reading. Refused with nothing written to the servo when it cannot
/// start, its result saying so; once it has written, it ends through the
/// guard whatever happens, and keeps its captures and result.
pub async fn table<P: Pipe>(rig: &mut Rig<'_, P>, i: &TableInputs<'_>) -> Result<Tabled, Refused> {
    let mut b = match before(rig, i).await {
        Ok(b) => b,
        Err(e) => {
            let doc = json!({
                "step": STEP, "method": METHOD, "revision": REVISION,
                "verdict": Way::Refused.name(), "why": e.0, "before_any_write": true,
                "slots": null, "points_by_page": null,
            });
            let mut text = serde_json::to_vec_pretty(&doc).unwrap_or_default();
            text.push(b'\n');
            rig.sink
                .write(RESULT, &text)
                .map_err(|w| Refused(format!("{}; the result was not kept: {w}", e.0)))?;
            return Err(e);
        }
    };
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
    let ran = body(
        rig,
        &mut b,
        i.schedule,
        &mut ledger,
        &mut taken,
        &mut found,
        &mut lines,
    )
    .await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = rest::pos_now(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines).map_err(Halt::from);
    let cfg = JudgeCfg {
        fs: b.tick_hz,
        raw_min: b.raw.0,
        raw_max: b.raw.1,
        ripple_per_rev: b.ripple_per_rev,
        gear_ratio: b.gear_ratio,
        travel_deg: b.travel_deg,
    };
    let finished = ran.is_ok();
    let rows: Vec<RungRows> = found
        .rungs
        .iter()
        .map(|r| RungRows {
            n: r.n,
            pass: r.pass,
            pass_kind: r.pass_kind,
            dir: r.plan.dir,
            rows: &taken[r.drive_k].burst.rows,
        })
        .collect();
    let judged = finished.then(|| judge(&rows, &cfg));
    let mut checks = judged.as_ref().map_or(Vec::new(), |j| j.checks.clone());
    if !exit.torque_off {
        checks.push(check(
            "torque off at the end",
            "torque_enable did not read back 0".into(),
            "reads 0".into(),
            Verdict::Fail,
        ));
    }
    lines.extend(found.path.iter().map(|p| format!("decision: {p}")));
    lines.push(format!(
        "end position {} counts ({:.1} before)",
        after.map_or("not read".into(), |p| format!("{p:.1}")),
        b.pos
    ));
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
                        .map(|c| format!("{} ({})", c.name, c.value))
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
        why.as_deref(),
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
    Ok(Tabled {
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

fn state_word(s: State) -> &'static str {
    match s {
        State::Wide => "wide",
        State::NotWide => "not wide",
        State::Thin => "thin",
        State::Unreached => "unreached",
    }
}

/// Why a slot reads 0 when it is not wide.
fn carry_reason(s: State) -> Option<String> {
    match s {
        State::Thin => Some(format!(
            "under {THIN_ROWS} rows a code around it: written 0"
        )),
        State::Unreached => Some(format!(
            "the rows do not reach {} codes either side of it inside the swept range: written 0",
            widecodes::REF_SIDE_CODES
        )),
        _ => None,
    }
}

fn finding_json(f: &Finding) -> Value {
    json!({
        "swept_counts": [f.lo, f.hi],
        "slots": f.slots,
        "carries": f.carries.iter().map(|c| json!({"slot": c.slot, "code": c.code, "state": state_word(c.state),
            "skipped": c.skipped, "ratio": c.ratio, "rows_ref": c.rows_ref, "slot_q4": f.slots[c.slot],
            "why_zero": carry_reason(c.state)})).collect::<Vec<_>>(),
        "strays": f.strays.iter().map(|s| json!({"code": s.code, "skipped": s.skipped, "ratio": s.ratio})).collect::<Vec<_>>(),
    })
}

fn reach_json(r: &Reach) -> Value {
    match r {
        Reach::Reached(m) => json!({"reached": true, "still_counts": m}),
        Reach::Short(m) => json!({"reached": false, "short": true, "still_counts": m}),
        Reach::Past(m) => json!({"reached": false, "past": true, "still_counts": m}),
        Reach::Moving => json!({"reached": false, "moving": true}),
    }
}

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    judged: Option<&Judged>,
    cfg: &JudgeCfg,
    checks: &[Check],
    way: Way,
    why: Option<&str>,
    exit: &guard::Exit,
    after: Option<f64>,
) -> Value {
    let rungs: Vec<Value> = found
        .rungs
        .iter()
        .map(|r| {
            let p = &r.plan;
            json!({
                "n": r.n, "pass": r.pass, "pass_kind": r.pass_kind.word(), "kind": r.kind.word(),
                "dir": p.dir, "attempt": r.attempt, "duty_q15": r.q15, "start_counts": p.start,
                "planned": {"t_s": p.t_s, "rows": p.rows, "share": p.share, "t_runway_s": p.t_runway_s,
                            "t_burst_s": p.t_burst_s, "runway_counts": p.runway, "far_counts": p.far_guard,
                            "fastest_cps": p.fastest_cps, "bound_speed_cps": p.bound_speed,
                            "bound_coast_counts": p.bound_coast, "travel_bound_counts": p.travel_bound()},
                "priced": {"time_s": p.t_s + DRIVE_DEADMAN_S, "blocked_a": r.price.current_a,
                           "rise_c": r.price.rise_c, "travel_counts": r.price.travel},
                "on_s": r.on_s, "end_counts": r.read.end, "travel_counts": r.read.travel,
                "drive_travel_counts": r.read.drive_travel, "pot_speed_cps": r.read.speed_cps,
                "reach": r.reach.as_ref().map(reach_json),
                "captures": [r.drive_k + 1, r.drive_k + 2],
            })
        })
        .collect();
    let priced_s: f64 = found
        .rungs
        .iter()
        .map(|r| r.plan.t_s + DRIVE_DEADMAN_S)
        .sum();
    let blocked_s: f64 = found
        .rungs
        .iter()
        .filter(|r| r.kind == RungKind::Stop)
        .map(|r| r.plan.t_s + DRIVE_DEADMAN_S)
        .sum();
    let rise = found
        .rungs
        .iter()
        .map(|r| r.price.rise_c)
        .fold(0.0f64, f64::max);
    let v = |q: i16| q as f64 / Q15 * b.rail_v;
    let drives = json!({
        "stops_counts": [b.stops.lo, b.stops.hi], "guards_counts": [b.band.lo, b.band.hi],
        "guard_share": GUARD_SHARE, "near_stop_counts": NEAR_STOP_COUNTS,
        "approach_q15": b.approach_q15, "approach_v": v(b.approach_q15),
        "fast_q15": b.fast_q15, "fast_v": v(b.fast_q15), "fast_predicted_cps": b.fast_predicted_cps,
        "window_ticks": b.window_ticks, "pwm_period_ticks": b.pwm_arr,
        "i_window_min_ticks": b.stream.i_window_min_ticks,
        "rung_min_s": RUNG_MIN_S, "free_margin": FREE_MARGIN, "coast_tau_s": COAST_TAU_S,
        "burst_max_s": BURST_MAX_S, "permit_max_s": PERMIT_MAX_S, "drive_deadman_s": DRIVE_DEADMAN_S,
        "poll_deadman_s": POLL_DEADMAN_S, "coast_s": COAST_S, "still_counts": b.still_counts,
        "approach_legs": b.stops.legs.iter().map(|l| json!({"n": l.n, "dir": l.dir, "counts": l.counts,
            "secs": l.secs, "speed_cps": l.speed_cps()})).collect::<Vec<_>>(),
    });
    let mut out = json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "why": why,
        "slots": null,
        "points_by_page": null,
        "scale": null,
        "stops_used": {"raw_min": cfg.raw_min, "raw_max": cfg.raw_max,
                       "source": "the stops step's stops.low.median_counts up and stops.high.median_counts down: rounded inward, as the gather writes them"},
        "passes": found.passes.iter().map(|p| json!({"n": p.n, "kind": p.kind.word(), "dir": p.dir,
            "start_counts": p.start, "end_counts": p.end})).collect::<Vec<_>>(),
        "rows_by_round_trips": found.rows_a_code.iter().map(|r| json!({"slow_round_trips": r.trips,
            "median_rows_a_code": r.median, "thin_carries": r.thin})).collect::<Vec<_>>(),
        "priced": {"rungs_s": priced_s, "blocked_s": blocked_s, "peak_rise_c": rise,
                   "words": "every rung priced blocked for its rows and dead-man; the stop rungs are the ones that end blocked"},
        "rungs": rungs,
        "unkept": found.unkept.iter().map(|u| json!({"n": u.n, "pass": u.pass, "kind": u.kind.word(),
            "attempt": u.attempt, "why": u.why, "captures": [u.captures.0, u.captures.1]})).collect::<Vec<_>>(),
        "path": found.path,
        "drives": drives,
        "drive_polarity": {"stored": b.stored, "driven": b.probe.found, "corrected_volatile": found.corrected},
        "checks": checks.iter().map(Check::json).collect::<Vec<_>>(),
        "rail_v": b.rail_v,
        "position_before_counts": b.pos,
        "position_after_counts": after,
        "exit": exit.lines,
        "torque_off": exit.torque_off,
    });
    if let (Some(j), Value::Object(o)) = (judged, &mut out) {
        o.insert(
            "wide_codes".into(),
            json!({
                "histogram_counts": [j.swept.0, j.swept.1], "rows_a_code": j.rows_a_code,
                "all": j.finding.as_ref().map(finding_json),
                "even_passes": j.halves.as_ref().map(|h| finding_json(&h.0)),
                "odd_passes": j.halves.as_ref().map(|h| finding_json(&h.1)),
                "thin_rows": THIN_ROWS, "half_thin_rows": HALF_THIN_ROWS,
            }),
        );
        o.insert("table".into(), table_json(&j.table));
        if way == Way::Done {
            accepted(o, j);
        }
    }
    out
}

fn segment_json(r: &RungSegment) -> Value {
    let head = json!({"n": r.n, "pass": r.pass, "pass_kind": r.pass_kind.word(), "dir": r.dir});
    let mut o = head.as_object().cloned().unwrap_or_default();
    match &r.read {
        Ok(s) => {
            let (lo, hi) =
                s.x.iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &v| {
                        (l.min(v), h.max(v))
                    });
            o.extend(
                json!({"read": true, "samples": s.x.len(), "reach_counts": [lo, hi],
                       "cycles": s.y.last().map(|y| y.abs()), "line_hz": s.f_seed,
                       "crossings": s.crossings, "odd_share": s.odd, "speed_spread": s.speed_spread})
                .as_object()
                .cloned()
                .unwrap_or_default(),
            );
        }
        Err(e) => {
            o.insert("read".into(), json!(false));
            o.insert("why".into(), json!(e));
        }
    }
    Value::Object(o)
}

fn table_json(t: &TableRead) -> Value {
    let fit = match &t.fit {
        Ok(f) => json!({
            "span_counts": [f.table.span.lo(), f.table.span.hi()], "samples": f.samples,
            "residual_cycles_rms": f.residual_rms, "residual_counts_rms": t.residual_counts,
            "counts_per_cycle": f.table.counts_per_cycle(),
            "up_reach_counts": f.up.map(|r| [r.0, r.1]), "down_reach_counts": f.down.map(|r| [r.0, r.1]),
            "offsets_cycles": f.offsets.iter().map(|(k, c)| json!({"rung": t.rungs.iter().filter(|r| r.read.is_ok()).nth(*k).map(|r| r.n), "cycles": c})).collect::<Vec<_>>(),
            "smooth_weight": SMOOTH_WEIGHT,
        }),
        Err(e) => json!({"refused": table_said(e)}),
    };
    json!({
        "method": "one pooled fit, pos = T(phase) + one offset a rung (stage 4 position.fit_pwl form), the motor's ripple the ruler",
        "near_stop_counts": NEAR_STOP_COUNTS,
        "rungs": t.rungs.iter().map(segment_json).collect::<Vec<_>>(),
        "fit": fit,
        "gain_q4": t.gains.map(|g| [g.0, g.1]),
        "validate": t.valid.map(|v| v.map_or_else(|r| format!("{r:?}"), |()| "ok".to_string())),
        "one_fast_pass_out": t.pass_out.iter().map(|p| json!({"rung": p.n, "pass": p.pass,
            "line_out_counts_rms": p.moved.as_ref().ok(), "why": p.moved.as_ref().err().map(table_said)})).collect::<Vec<_>>(),
        "cycles": t.cycles.iter().map(|k| json!({"rungs": [k.a, k.b], "from_counts": k.agree.from,
            "to_counts": k.agree.to, "cycles": [k.agree.a, k.agree.b], "limit": k.agree.limit(),
            "holds": k.agree.holds()})).collect::<Vec<_>>(),
        "report": report_json(t),
    })
}

/// What the table step says and never refuses on.
fn report_json(t: &TableRead) -> Value {
    json!({
        "directions": match &t.term {
            Ok(term) => json!({"up_less_down_counts_rms": term.rms, "max_counts": term.max,
                "bands": term.bands.iter().map(|(b, r)| json!({"from_counts": b, "counts_rms": r})).collect::<Vec<_>>(),
                "warn_over_counts_rms": DIRECTION_WARN_COUNTS, "warning": term.rms > DIRECTION_WARN_COUNTS}),
            Err(e) => json!({"not_read": table_said(e)}),
        },
        "gear_play": {"counts": null, "why": GEAR_PLAY_WORDS},
        "speed_spread": t.rungs.iter().filter_map(|r| r.read.as_ref().ok().map(|s| json!({"rung": r.n,
            "spread": s.speed_spread, "warning": s.speed_spread > SPEED_SPREAD_WARN}))).collect::<Vec<_>>(),
        "one_way": t.one_way.iter().map(|w| json!({"from_counts": w.from, "to_counts": w.to,
            "only": way(w.dir)})).collect::<Vec<_>>(),
    })
}

const GEAR_PLAY_WORDS: &str = "not read: each rung's cycle count starts at its own first crossing, so a rung's offset holds where it started, not the gear's play; the play needs one cycle count across the rungs";

/// The accepted reading: the slots, the slots left 0 and why, the points
/// by page and the scale; the report items stay under `table.report`.
fn accepted(o: &mut Map<String, Value>, j: &Judged) {
    if let Some(f) = &j.finding {
        o.insert("slots".into(), json!(f.slots));
        let zero: Vec<Value> = f
            .carries
            .iter()
            .filter(|c| f.slots[c.slot] == 0)
            .map(|c| {
                json!({"slot": c.slot, "code": c.code, "state": state_word(c.state),
                       "why": carry_reason(c.state).unwrap_or_else(|| "read, not wide: written 0".into())})
            })
            .collect();
        o.insert("slots_zero".into(), json!(zero));
    }
    if let Some(p) = &j.table.points {
        let pages: Vec<Vec<i16>> = p.chunks(PAGE_POINTS).map(<[i16]>::to_vec).collect();
        o.insert("points_by_page".into(), json!(pages));
    }
    if let Some((v, source)) = j.table.scale {
        o.insert(
            "scale".into(),
            json!({"counts_per_deg": v, "source": source}),
        );
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;

    use futures_lite::future::block_on;
    use osc_client::Client;
    use serde_json::json;

    use super::*;
    use crate::sink::{MemSink, Sink};
    use crate::steps::runway::ARRIVE_COUNTS;
    use crate::steps::testkit::{self, ID, Motor, OnSent, Sent, Spy, addr, descriptor};

    const APPROACH_Q15: i16 = 4000;
    /// Where the shaft rests before the step, straight counts.
    const X_START: f64 = 2050.0;
    /// The reduced schedule the fake can run: its runaway guard stops a
    /// sim at 10 M events, about a 400 k-row capture.
    const REDUCED: Schedule = Schedule {
        slow_round_trips: 1,
        fast_round_trips: 1,
        extra_slow_max: 0,
    };

    /// A servo on 2300 counts of travel: 380 counts/s at the approach
    /// duty, 2200 at the fast passes' (the nameplate predicts 2667), the
    /// shaft still under about 10.8 % duty, 2 counts a commutation cycle
    /// (the slow passes' line at 190 Hz, inside the ripple band), a pot
    /// reading 1.5x fast at 1900, wide codes at carries 4 and 9.
    fn motor() -> Motor {
        let (slow, fast) = (APPROACH_Q15 as f64 / Q15, fast_q15(4.8, 5.0) as f64 / Q15);
        let k = (2200.0 - 380.0) / (fast - slow);
        Motor {
            cps_full: k,
            dead_duty: slow - 380.0 / k,
            cycles_per_count: 0.5,
            rise_tau_s: 0.02,
            coast_tau_s: 0.03,
            stops: (900.0, 3200.0),
            bend: (1900.0, 30.0, 30.0),
            up_wave: (0.0, 1150.0),
            wide: vec![(1279, 11), (2559, 6)],
            zero: 1206.0,
            level: 300.0,
            ripple: 20.0,
            pwm_arr: 1200.0,
        }
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
                "pulses": [{"repeat": true, "unwedge": false, "volts": 0.48}],
            },
        })
    }

    /// The stops step's result on `m`: its stops as the pot reads them,
    /// the approach duty `q15`, and approach legs read at `leg_share` of
    /// the motor's free speed there.
    fn stops_json(m: &Motor, q15: i16, leg_share: f64) -> Value {
        let (lo, hi) = (m.pot(m.stops.0), m.pot(m.stops.1));
        let v = m.cps_full * (APPROACH_Q15 as f64 / Q15 - m.dead_duty) * leg_share;
        let leg = |n: usize, kind: &str, toward: &str, from: f64, to: f64| {
            json!({"n": n, "kind": kind, "toward": toward, "from_counts": from, "to_counts": to,
                   "secs": (to - from).abs() / v})
        };
        json!({
            "verdict": "accepted",
            "stops": {
                "low": {"median_counts": lo}, "high": {"median_counts": hi},
                "legs": [
                    leg(1, "approach", "low", 2080.0, lo),
                    leg(2, "leave", "high", lo, lo + 150.0),
                    leg(3, "approach", "high", lo + 150.0, hi),
                    leg(4, "leave", "low", hi, hi - 150.0),
                    leg(5, "approach", "low", hi - 150.0, lo),
                ],
            },
            "drives": {"approach_q15": q15},
        })
    }

    struct Ran {
        out: Result<Tabled, Refused>,
        c: Client<Spy>,
        sink: MemSink,
        log: Vec<Sent>,
    }

    fn run(m: Motor, stops: Value, schedule: Schedule, also: OnSent) -> Ran {
        let d = descriptor();
        let (mut c, log, _) = testkit::motor_servo(m, X_START, also);
        let stored = block_on(c.read(ID, addr(&d, "drive_polarity"), 1)).expect("read")[0] != 0;
        log.borrow_mut().clear();
        let (np, board, thermal) = (testkit::nameplate(), testkit::board(), testkit::thermal());
        let (rest, probe) = (rest_json(), probe_json(stored));
        let clock = testkit::clock();
        let stop = AtomicBool::new(false);
        let mut sink = MemSink::default();
        let out = {
            let mut rig = Rig {
                c: &mut c,
                id: ID,
                d: &d,
                sink: &mut sink,
                stop: &stop,
                clock: &clock,
            };
            block_on(table(
                &mut rig,
                &TableInputs {
                    nameplate: &np,
                    board: &board,
                    thermal: Some(&thermal),
                    rest: &rest,
                    probe: &probe,
                    stops: &stops,
                    schedule,
                },
            ))
        };
        let log = log.borrow().clone();
        Ran { out, c, sink, log }
    }

    fn torque_off(c: &mut Client<Spy>) -> bool {
        let d = descriptor();
        block_on(c.read(ID, addr(&d, "torque_enable"), 1)).expect("read") == [0]
    }

    fn result(sink: &mut MemSink) -> Value {
        serde_json::from_slice(&sink.read(RESULT).expect("the result is kept")).expect("json")
    }

    /// A refused or failed run's result: which line, and no slots and no
    /// points.
    fn nothing_written(sink: &mut MemSink) -> Value {
        let r = result(sink);
        assert_ne!(r["verdict"], "accepted", "{r}");
        assert!(r["why"].as_str().is_some_and(|w| !w.is_empty()), "{r}");
        assert!(r["slots"].is_null() && r["points_by_page"].is_null(), "{r}");
        assert!(r["scale"].is_null(), "{r}");
        r
    }

    fn rungs(res: &Value) -> Vec<Value> {
        res["rungs"].as_array().cloned().unwrap_or_default()
    }

    fn n(v: &Value) -> f64 {
        v.as_f64().unwrap_or_else(|| panic!("{v} is no number"))
    }

    /// The plan's schedule cut to one slow and one fast round trip on the
    /// simulated motor: the half pass and each slow pass end at rest on
    /// their stop through a stop rung under the permit, each free and fast
    /// rung rests inside its guards under its bound, the first fast pass
    /// runs at half length, and the shaft ends at the middle with torque
    /// off. The reading is accepted: the converter's slots, the points
    /// that straighten the pot's bend, the scale from the stops over the
    /// travel (the nameplate states no ripple_per_rev).
    #[test]
    fn a_reduced_schedule_runs_every_pass_and_is_accepted() {
        let m = motor();
        let mut r = run(
            m.clone(),
            stops_json(&m, APPROACH_Q15, 0.95),
            REDUCED,
            testkit::quiet(),
        );
        let t = r.out.expect("starts");
        let e = &t.ended;
        assert!(torque_off(&mut r.c) && e.exit.torque_off, "{}", e.exit);
        assert_eq!(e.way, Way::Done, "{:?}\n{}", e.why, e.lines.join("\n"));
        let res = result(&mut r.sink);
        assert_eq!(res["verdict"], "accepted");
        assert!(
            t.checks
                .iter()
                .all(|c| matches!(c.verdict, Verdict::Pass | Verdict::NotJudged)),
            "{:?}",
            t.checks.iter().map(Check::said).collect::<Vec<_>>()
        );
        let passes = res["passes"].as_array().expect("passes");
        let kinds: Vec<(&str, i64)> = passes
            .iter()
            .map(|p| {
                (
                    p["kind"].as_str().unwrap_or(""),
                    p["dir"].as_i64().unwrap_or(0),
                )
            })
            .collect();
        assert_eq!(
            kinds,
            [
                ("half", -1),
                ("slow", 1),
                ("slow", -1),
                ("fast", 1),
                ("fast", -1)
            ]
        );
        let (lo, hi) = (m.pot(m.stops.0), m.pot(m.stops.1));
        for p in &passes[..3] {
            let stop = if p["dir"] == 1 { hi } else { lo };
            assert!((n(&p["end_counts"]) - stop).abs() <= WALK_MAX_COUNTS, "{p}");
        }
        let (glo, ghi) = (
            n(&res["drives"]["guards_counts"][0]),
            n(&res["drives"]["guards_counts"][1]),
        );
        let all = rungs(&res);
        let stops: Vec<&Value> = all.iter().filter(|x| x["kind"] == "stop").collect();
        assert!(stops.len() >= 3, "{}", stops.len());
        for p in 1..=3 {
            let mine: Vec<&&Value> = stops.iter().filter(|x| x["pass"] == p).collect();
            assert!((1..=STOP_RUNGS_MAX).contains(&mine.len()), "pass {p}");
            for (k, x) in mine.iter().enumerate() {
                let last = k + 1 == mine.len();
                assert_eq!(x["reach"]["reached"], last, "{x}");
                assert!(n(&x["priced"]["time_s"]) <= PERMIT_MAX_S + 1e-9, "{x}");
            }
        }
        assert_eq!(
            all.iter().filter(|x| x["kind"] == "leave").count(),
            2,
            "both slow passes leave a stop"
        );
        for x in all.iter().filter(|x| x["kind"] != "stop") {
            let end = n(&x["end_counts"]);
            assert!(glo <= end && end <= ghi, "{x}");
            assert!(
                n(&x["travel_counts"]) <= n(&x["planned"]["travel_bound_counts"]),
                "{x}"
            );
            assert!(
                n(&x["pot_speed_cps"]) <= n(&x["planned"]["bound_speed_cps"]),
                "{x}"
            );
        }
        let fast: Vec<&Value> = all.iter().filter(|x| x["kind"] == "fast").collect();
        assert_eq!(fast.len(), 2);
        assert_eq!(n(&fast[0]["planned"]["share"]), FIRST_FAST_SHARE);
        assert_eq!(n(&fast[1]["planned"]["share"]), 1.0);
        assert_eq!(fast[0]["duty_q15"], fast_q15(4.8, n(&res["rail_v"])));
        let mut want = [0u8; 16];
        (want[4], want[9]) = (88, 48);
        assert_eq!(res["slots"], json!(want), "{}", res["wide_codes"]);
        let zero = res["slots_zero"].as_array().expect("slots_zero");
        assert_eq!(zero.len(), 14);
        assert!(zero.iter().all(|z| z["why"].as_str().is_some()), "{zero:?}");
        assert!(n(&res["wide_codes"]["rows_a_code"]) >= ROWS_A_CODE_MIN);
        straightens_the_bend(&m, &res);
        let raw = &res["stops_used"];
        assert_eq!(res["scale"]["source"], SCALE_TRAVEL);
        assert_eq!(
            n(&res["scale"]["counts_per_deg"]),
            (n(&raw["raw_max"]) - n(&raw["raw_min"]))
                / testkit::nameplate().travel_deg.expect("travel")
        );
        assert!(n(&res["table"]["fit"]["residual_counts_rms"]) <= RESIDUAL_MAX_COUNTS);
        assert_eq!(res["table"]["report"]["directions"]["warning"], false);
        assert!(res["table"]["report"]["gear_play"]["counts"].is_null());
        assert!(n(&res["priced"]["blocked_s"]) > 0.0 && n(&res["priced"]["peak_rise_c"]) > 0.0);
        let middle = (glo + ghi) / 2.0;
        let after = n(&res["position_after_counts"]);
        assert!(
            (after - middle).abs() <= 2.0 * ARRIVE_COUNTS,
            "{after} against {middle}"
        );
        assert!(
            r.sink
                .files
                .keys()
                .any(|k| k.starts_with("captures/table/"))
        );
        let permit = addr(&descriptor(), "stall_permit");
        let on = r
            .log
            .iter()
            .filter(|w| matches!(w, Sent::Write { addr, data, .. } if *addr == permit && data[..] == [1]))
            .count();
        assert_eq!(
            on,
            stops.len(),
            "the permit around each stop rung and no other"
        );
    }

    /// The accepted points against the motor's own bend: inside the
    /// fitted span each point is the straight count the pot's reading
    /// stands for, mapped so both stops map to themselves, within 3
    /// counts.
    fn straightens_the_bend(m: &Motor, res: &Value) {
        let pages = res["points_by_page"].as_array().expect("points_by_page");
        assert_eq!(pages.len(), INTERVALS / PAGE_POINTS);
        let points: Vec<f64> = pages
            .iter()
            .flat_map(|p| p.as_array().expect("page").iter().map(n))
            .collect();
        assert_eq!(points.len(), INTERVALS);
        let (raw_min, raw_max) = (
            n(&res["stops_used"]["raw_min"]),
            n(&res["stops_used"]["raw_max"]),
        );
        let straight = |pot: f64| {
            let (mut lo, mut hi) = m.stops;
            for _ in 0..60 {
                let mid = (lo + hi) / 2.0;
                if m.pot(mid) < pot { lo = mid } else { hi = mid }
            }
            (lo + hi) / 2.0
        };
        let (x0, x1) = (straight(raw_min), straight(raw_max));
        let span = &res["table"]["fit"]["span_counts"];
        let (lo, hi) = (n(&span[0]), n(&span[1]));
        let mut worst = 0.0f64;
        for (k, c) in points.iter().enumerate() {
            let g = (k * 16) as f64;
            if g < lo || g > hi {
                continue;
            }
            let want = raw_min + (straight(g) - x0) * (raw_max - raw_min) / (x1 - x0) - g;
            worst = worst.max((c - want).abs());
        }
        assert!(worst <= 3.0, "a point {worst:.1} counts off the bend");
        assert!(points.iter().any(|c| c.abs() >= 10.0), "{points:?}");
    }

    /// An approach duty whose drive window is under the board's floor
    /// refuses before any write, and says so in its result.
    #[test]
    fn a_duty_under_the_window_floor_refuses_before_any_write() {
        let m = motor();
        let mut r = run(
            m.clone(),
            stops_json(&m, 1000, 0.95),
            REDUCED,
            testkit::quiet(),
        );
        let why = r.out.expect_err("refused").0;
        assert!(why.contains("i_window_min_ticks"), "{why}");
        let page = addr(&descriptor(), "pos_lut_page");
        assert!(
            !r.log
                .iter()
                .any(|w| matches!(w, Sent::Write { addr, .. } if *addr != page)
                    || *w == Sent::Commit),
            "{:?}",
            r.log
        );
        let res = nothing_written(&mut r.sink);
        assert_eq!(res["before_any_write"], true);
    }

    /// A shaft faster than the approach legs allow runs the half pass's
    /// first rung past the low guard: the step ends there, torque off,
    /// nothing written.
    #[test]
    fn a_free_rung_past_its_guard_ends_the_step() {
        let mut m = motor();
        m.cps_full *= 3.0;
        let mut r = run(
            m.clone(),
            stops_json(&m, APPROACH_Q15, 0.3),
            REDUCED,
            testkit::quiet(),
        );
        let e = r.out.expect("starts").ended;
        assert_eq!(e.way, Way::Refused, "{:?}", e.why);
        let why = e.why.as_deref().unwrap_or("");
        assert!(
            why.contains("rung 1") && why.contains("outside the guards"),
            "{why}"
        );
        assert!(torque_off(&mut r.c) && e.exit.torque_off, "{}", e.exit);
        nothing_written(&mut r.sink);
    }

    /// Rows lost on both attempts at a rung end the step: no evidence,
    /// the line named.
    #[test]
    fn a_rung_that_loses_rows_twice_ends_the_step() {
        let lose: OnSent = Box::new(|s, pipe| {
            if *s == Sent::Commit {
                pipe.sim_mut().lose_tel_ticks(0, 400, 410);
            }
        });
        let m = motor();
        let mut r = run(m.clone(), stops_json(&m, APPROACH_Q15, 0.95), REDUCED, lose);
        let e = r.out.expect("starts").ended;
        assert_eq!(e.way, Way::Error, "{:?}", e.why);
        let why = e.why.as_deref().unwrap_or("");
        assert!(
            why.contains("2 attempts") && why.contains("lost rows"),
            "{why}"
        );
        assert!(torque_off(&mut r.c) && e.exit.torque_off, "{}", e.exit);
        let res = nothing_written(&mut r.sink);
        assert_eq!(res["unkept"].as_array().map(Vec::len), Some(2), "{res}");
    }

    /// The stops step read the low stop 30 counts off where the train
    /// stops: the half pass's stop rung goes still past it or short of it,
    /// and the step names which.
    #[test]
    fn a_stop_rung_that_finds_the_stop_moved_or_blocked_ends_the_step() {
        for (off, says) in [(30.0, "the stop moved"), (-30.0, "blocked")] {
            let m = motor();
            let mut stops = stops_json(&m, APPROACH_Q15, 0.95);
            let lo = n(&stops["stops"]["low"]["median_counts"]);
            stops["stops"]["low"]["median_counts"] = json!(lo + off);
            let mut r = run(m.clone(), stops, REDUCED, testkit::quiet());
            let e = r.out.expect("starts").ended;
            assert_eq!(e.way, Way::Refused, "{:?}", e.why);
            let why = e.why.as_deref().unwrap_or("");
            assert!(
                why.contains("onto the low stop") && why.contains(says),
                "{says}: {why}"
            );
            assert!(torque_off(&mut r.c) && e.exit.torque_off, "{}", e.exit);
            nothing_written(&mut r.sink);
        }
    }

    /// The pot read 6 counts high in a wave on the up drives alone: the
    /// directions differ by over 3 counts rms, which is said as a warning,
    /// and the table is still accepted.
    #[test]
    fn a_direction_difference_over_three_counts_warns_and_still_accepts() {
        let mut m = motor();
        m.up_wave = (6.0, 1150.0);
        let mut r = run(
            m.clone(),
            stops_json(&m, APPROACH_Q15, 0.95),
            REDUCED,
            testkit::quiet(),
        );
        let t = r.out.expect("starts");
        let e = &t.ended;
        assert_eq!(e.way, Way::Done, "{:?}\n{}", e.why, e.lines.join("\n"));
        let dirs = t
            .checks
            .iter()
            .find(|c| c.name == DIRECTIONS)
            .expect("directions");
        assert_eq!(dirs.verdict, Verdict::Warn, "{}", dirs.said());
        let res = result(&mut r.sink);
        let d = &res["table"]["report"]["directions"];
        assert!(
            n(&d["up_less_down_counts_rms"]) > DIRECTION_WARN_COUNTS,
            "{d}"
        );
        assert_eq!(d["warning"], true);
        assert!(res["points_by_page"].is_array() && res["slots"].is_array());
    }

    /// A drive's rows as the raw layout carries them.
    fn frames(rows: &[osc_client::fake::TelSample]) -> Vec<TelFrame> {
        rows.iter()
            .enumerate()
            .map(|(k, s)| TelFrame {
                tick: k as u64,
                pos: Some(s.tick.pos),
                current_raw: Some(s.tick.current_raw),
                duty_counts: Some(s.tick.duty_ticks),
                ..TelFrame::default()
            })
            .collect()
    }

    /// Slow passes on `m` with no bus: the half pass, then `trips` round
    /// trips stop to stop, each one drive to its stop.
    fn slow_rows(m: &Motor, trips: usize) -> Vec<(usize, i8, Vec<TelFrame>)> {
        let mut out = Vec::new();
        let mut x = X_START;
        let mut dirs = vec![-1i8];
        for _ in 0..trips {
            dirs.extend([1, -1]);
        }
        for (k, dir) in dirs.into_iter().enumerate() {
            let (track, states) = m.run(x, 0.0, 0, APPROACH_Q15 * dir as i16, 0, None);
            let at_stop = states
                .iter()
                .position(|s| s.1 == 0.0 && s.0 == if dir > 0 { m.stops.1 } else { m.stops.0 })
                .unwrap_or(track.len());
            out.push((k + 1, dir, frames(&track[..at_stop])));
            x = states[at_stop.min(states.len() - 1)].0;
        }
        out
    }

    fn judged(m: &Motor) -> Judged {
        judged_after(m, 2)
    }

    fn judged_after(m: &Motor, trips: usize) -> Judged {
        let ran = slow_rows(m, trips);
        let rows: Vec<RungRows> = ran
            .iter()
            .enumerate()
            .map(|(n, (pass, dir, rows))| RungRows {
                n: n + 1,
                pass: *pass,
                pass_kind: if *pass == 1 {
                    PassKind::Half
                } else {
                    PassKind::Slow
                },
                dir: *dir,
                rows,
            })
            .collect();
        judge(&rows, &cfg(m))
    }

    fn cfg(m: &Motor) -> JudgeCfg {
        JudgeCfg {
            fs: 20_000.0,
            raw_min: m.pot(m.stops.0).ceil() as u16,
            raw_max: m.pot(m.stops.1).floor() as u16,
            ripple_per_rev: None,
            gear_ratio: None,
            travel_deg: 184.3,
        }
    }

    /// The plan's slow passes on the simulated motor: the finder reads the
    /// converter's carries from the rows inside the stops less 32 counts,
    /// the unreached carries 0, and the table from the slow rungs alone
    /// passes; with no fast pass the pass-out line is not judged.
    #[test]
    fn the_slow_passes_give_the_converters_slots() {
        let m = motor();
        let j = judged(&m);
        let f = j.finding.as_ref().expect("found");
        let mut want = [0u8; 16];
        (want[4], want[9]) = (88, 48);
        assert_eq!(f.slots, want);
        assert!(j.rows_a_code >= ROWS_A_CODE_MIN, "{}", j.rows_a_code);
        assert_eq!(f.carries[13].state, State::Unreached);
        assert_eq!(
            j.swept,
            (
                (m.pot(m.stops.0).ceil() + 32.0) as u16,
                (m.pot(m.stops.1).floor() - 32.0) as u16
            )
        );
        for c in &j.checks {
            assert_ne!(c.verdict, Verdict::Fail, "{}", c.said());
        }
        let out = j
            .checks
            .iter()
            .find(|c| c.name == PASS_OUT)
            .expect("pass out");
        assert_eq!(out.verdict, Verdict::NotJudged);
    }

    /// The half pass alone gives the finder about 53 rows a code on the
    /// codes it crosses: the median line refuses and says the count.
    #[test]
    fn too_few_rows_a_code_refuses_with_the_median() {
        let j = judged_after(&motor(), 0);
        let rows = j.checks.iter().find(|c| c.name == ROWS).expect("rows");
        assert_eq!(rows.verdict, Verdict::Fail, "{}", rows.said());
        assert!(rows.value.starts_with("median "), "{}", rows.value);
        assert!(j.rows_a_code < ROWS_A_CODE_MIN, "{}", j.rows_a_code);
    }

    /// One fast pass's rows on `m` from `x` for `secs`, no bus.
    fn fast_rows(m: &Motor, x: f64, dir: i8, secs: f64) -> Vec<TelFrame> {
        let rows = (secs * 20_000.0) as usize;
        let q = fast_q15(4.8, 5.0) * dir as i16;
        let (track, _) = m.run(x, 0.0, 0, q, 0, Some(rows));
        frames(&track[..rows])
    }

    /// Two up fast passes over the low half and one down pass over most
    /// of the travel: past the up passes' reach the table rests on the
    /// down pass alone, so with it left out there is no table there and
    /// the pass-out line refuses; the fit, its residual, the cycles and
    /// the gains pass.
    #[test]
    fn a_table_resting_on_one_fast_pass_refuses_with_it_left_out() {
        let m = motor();
        let (up_a, up_b) = (fast_rows(&m, 970.0, 1, 0.5), fast_rows(&m, 970.0, 1, 0.5));
        let down = fast_rows(&m, 3130.0, -1, 0.8);
        let rungs: Vec<RungRows> = [(1, 1, &up_a), (2, -1, &down), (3, 1, &up_b)]
            .iter()
            .map(|&(k, dir, rows)| RungRows {
                n: k,
                pass: k,
                pass_kind: PassKind::Fast,
                dir,
                rows,
            })
            .collect();
        let mut slots = [0u8; 16];
        (slots[4], slots[9]) = (88, 48);
        let c = cfg(&m);
        let t = read_table(&rungs, &slots, &c);
        let checks = table_checks(&t, &c);
        let failed: Vec<&Check> = checks
            .iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .collect();
        assert!(
            failed.len() == 1 && failed[0].name == PASS_OUT,
            "{:?}",
            checks.iter().map(Check::said).collect::<Vec<_>>()
        );
        assert!(
            failed[0].value.contains("rung 2 out"),
            "{}",
            failed[0].value
        );
        assert!(
            t.pass_out.iter().filter(|p| p.moved.is_ok()).count() == 2,
            "{:?}",
            t.pass_out
        );
    }

    /// A never-read run under a code that is no carry: the wide-code
    /// finder's refusal comes through with its line.
    #[test]
    fn an_estimator_refusal_comes_through_with_its_line() {
        let mut m = motor();
        m.wide.push((1500, 5));
        let j = judged(&m);
        let wide = j
            .checks
            .iter()
            .find(|c| c.name == WIDE_CODES)
            .expect("the wide-code check");
        assert_eq!(wide.verdict, Verdict::Fail);
        assert!(
            wide.value.contains("over code 1500") && wide.value.contains("no carry"),
            "{}",
            wide.value
        );
    }
}
