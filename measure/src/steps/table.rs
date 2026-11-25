//! The table step, after the stops: the ADC's wide-code slots and the
//! position table from one open-loop capture, read by
//! [`crate::fit::widecodes`] and [`crate::fit::postable`].
//!
//! [`ROUND_TRIPS`] round trips at the stops step's approach duty, both
//! ways. A pass starts at rest [`START_INSIDE_COUNTS`] inside its near
//! guard, reached by the polled go-to ([`GoTo`]), and runs up to
//! [`RUNGS_A_PASS`] rungs toward the far guard, each one committed drive of
//! [`drive::drive`] with a coast end on the raw row, no permit. The guards
//! are the envelope's, [`GUARD_SHARE`] of the stops' span in from each.
//! A rung's length ([`plan_rung`]):
//!
//! ```text
//! T           = min(BURST_MAX_S - DRIVE_DEADMAN_S,
//!                   (runway - bound_speed x COAST_TAU_S) / bound_speed
//!                   - DRIVE_DEADMAN_S)
//! bound_speed = FREE_MARGIN x the fastest speed seen this run that way
//! ```
//!
//! runway from the rung's start to the far guard; the speeds seen are the
//! stops step's approach legs, then each rung's last [`SPEED_TAIL_SHARE`]
//! by the pot. The burst cap leaves room for the dead-man the envelope
//! prices on top of a rung's rows. The first rung of round trips 1, 2, 3
//! is cut to [`STAGGER`] of its T, so rung starts fall at different counts
//! and every rung overlaps another of its direction; a rung under
//! [`RUNG_MIN_S`] is left out and ends its pass. Every rung is priced
//! through the envelope before it is sent; after it, at rest, the shaft
//! must lie inside both guards, have run no faster than its bound, and its
//! bursts be whole (a rung taken again once, then the step ends) and its
//! ripple clean ([`postable::ODD_MAX`], the same). The way out leaves the
//! shaft at the middle of the stops.
//!
//! One table: the mean of the two directions' shapes over the counts both
//! cover, straight to the stops. Slots no row reaches are 0 and named. A
//! refused or failed step writes no slots and no points; its result says
//! which line refused.

use std::time::Duration;

use osc_client::pipe::Pipe;
use osc_client::pos_lut::{PAGE_POINTS, PAGES};
use osc_servo_core::adc::ADC_WIDE_SLOTS;
use osc_servo_core::estimator::window::drive_ticks;
use osc_servo_core::pos_lut::{self, INTERVALS, POINTS};
use serde_json::{Map, Value, json};

use super::drive::{self, Deadmen, Drive, End};
use super::polarity::{HANDS, MOVE_MIN_COUNTS};
use super::probe::FromRest;
use super::runway::{COAST_TAU_S, Go, GoTo, Settle};
use super::seek::{self, POLL_S, WINDOW_S};
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
    Resistance, Rung,
};
use crate::fit::polyfit;
use crate::fit::postable::{
    self, DirectionFit, SMOOTH_WEIGHT, Segment, SegmentError, Table, TableError, Term,
};
use crate::fit::widecodes::{self, Finding, HALF_THIN_ROWS, State, THIN_ROWS};
use crate::guard::{self, Ledger, Way};
use crate::inputs::ThermalInputs;
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::{ADC_COUNTS, Parts};
use crate::state::{self, State as ServoState, field};

pub const STEP: &str = "table";
pub const RESULT: &str = "results/table.json";
const METHOD: &str = "open-loop-ripple-table";
const REVISION: u32 = 1;
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "the probe's pricing R (planning grade)";

pub const ROUND_TRIPS: usize = 3;
pub const RUNGS_A_PASS: usize = 3;
/// The first rung of each round trip, a share of its T.
pub const STAGGER: [f64; ROUND_TRIPS] = [1.0, 0.85, 0.7];
/// A pass starts at rest this far inside its near guard.
pub const START_INSIDE_COUNTS: f64 = 40.0;
pub const RUNG_MIN_S: f64 = 0.15;
/// The share of a rung's driven rows, from its end, its speed is read
/// over: past the spin-up.
pub const SPEED_TAIL_SHARE: f64 = 0.6;
/// The coast burst after a rung, s: no estimator reads its rows, so it
/// is as long as the spin-down bound, enough to stamp the zero.
pub const COAST_S: f64 = COAST_TAU_S;
/// Attempts a rung gets when its rows are no evidence.
pub const ATTEMPTS: usize = 2;

/// The acceptance bounds.
pub const RESIDUAL_MAX_COUNTS: f64 = 4.0;
pub const TERM_WARN_COUNTS: f64 = 8.0;
pub const TERM_MAX_COUNTS: f64 = 16.0;
pub const TRIP_LINE_OUT_MAX_COUNTS: f64 = 2.0;
pub const TRIP_SLOPE_MAX: f64 = 0.05;
pub const GAIN_Q4: (i32, i32) = (8, 32);
pub const COVERAGE_MIN: f64 = 0.80;
/// A rung whose ripple line moves more than this share over its quarters
/// is said, never refused.
pub const SPREAD_WARN: f64 = 0.20;

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

/// The counts a rung stays inside: the envelope's band.
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

    /// Where a pass of sign `dir` starts: inside the guard it leaves.
    pub fn start(&self, dir: i8) -> f64 {
        if dir > 0 {
            self.lo + START_INSIDE_COUNTS
        } else {
            self.hi - START_INSIDE_COUNTS
        }
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

/// The fastest speed seen this run each way, counts/s, and where it was
/// seen.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Fastest {
    pub up: (f64, String),
    pub down: (f64, String),
}

impl Fastest {
    /// The approach legs' speeds.
    pub fn of_legs(legs: &[ApproachLeg]) -> Self {
        let mut f = Fastest::default();
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

    pub fn feed(&mut self, dir: i8, cps: f64, from: String) {
        let s = if dir > 0 {
            &mut self.up
        } else {
            &mut self.down
        };
        if cps.is_finite() && cps > s.0 {
            *s = (cps, from);
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

/// The share of T the `k`-th rung (from 0) of round trip `trip` (from 0)
/// drives.
pub fn share(trip: usize, k: usize) -> f64 {
    if k == 0 {
        STAGGER.get(trip).copied().unwrap_or(1.0)
    } else {
        1.0
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

/// What the step reads besides the servo.
pub struct TableInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The same run's `results/rest.json`, `probe.json` and `stops.json`.
    pub rest: &'a Value,
    pub probe: &'a Value,
    pub stops: &'a Value,
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
    window_ticks: u32,
    pwm_arr: u16,
    still_counts: f64,
    env: Envelope,
    stated_v: f64,
    rated_cps: f64,
    ripple_per_rev: Option<u32>,
    gear_ratio: Option<f64>,
    travel_deg: Option<f64>,
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

/// A rung as the envelope prices it: free at its duty for its rows and the
/// dead-man, at the speed its bound is the free margin over, then its
/// coast's bound; the coast's rows after it.
fn rung_rung(name: &str, q15: i16, p: &RungPlan) -> Rung {
    Rung {
        name: name.into(),
        duty: q15 as f64 / Q15,
        time_s: p.t_s + DRIVE_DEADMAN_S,
        permit: false,
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
    let gear_ratio = np.gear_ratio.filter(|g| g.is_finite() && *g > 0.0);
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
        stops,
        band,
        window_ticks,
        pwm_arr,
        still_counts,
        env: Envelope::new(facts, heat),
        stated_v: speed.at_v,
        rated_cps: speed.per_deg / speed.seconds * ADC_COUNTS / np_travel,
        ripple_per_rev,
        gear_ratio,
        travel_deg,
        ntc: sense.ntc,
        soft_counts: (word(&lo), word(&hi)),
        soft: [lo, hi],
    };
    // the first go-to window and the first rung priced before any write
    let q = b.approach_q15;
    price_goto(&mut b, q, pos, false)?;
    let fastest = Fastest::of_legs(&b.stops.legs);
    let p = plan_rung(
        &b.band,
        1,
        b.band.start(1),
        fastest.get(1),
        share(0, 0),
        b.tick_hz,
    );
    if !p.fits() {
        return Err(Refused(format!(
            "the runway leaves the first rung {:.0} ms, under {:.0} ms ({}): the stops are too close for the approach legs' speed",
            1e3 * p.t_s,
            1e3 * RUNG_MIN_S,
            p.said()
        )));
    }
    b.env.price(&rung_rung("the first rung", q, &p))?;
    Ok(b)
}

/// One rung taken: its place, its plan, its read and where its bursts are.
#[derive(Clone, Debug)]
pub struct Taken1 {
    /// From 1, over the run.
    pub n: usize,
    /// The round trip and the pass, from 1; the rung in its pass from 1.
    pub trip: usize,
    pub pass: usize,
    pub k: usize,
    pub attempt: usize,
    pub q15: i16,
    pub plan: RungPlan,
    pub read: RungRead,
    pub on_s: Option<f64>,
    pub drive_k: usize,
    /// The ripple's odd interval share as the run read it.
    pub odd: Option<f64>,
}

/// A rung's rows that are no evidence.
#[derive(Clone, Debug)]
struct Unkept {
    n: usize,
    trip: usize,
    pass: usize,
    k: usize,
    attempt: usize,
    why: String,
    captures: (usize, usize),
}

#[derive(Default)]
struct Found {
    corrected: bool,
    rungs: Vec<Taken1>,
    unkept: Vec<Unkept>,
    /// (pass, rung in it, why).
    left_out: Vec<(usize, usize, String)>,
    path: Vec<String>,
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

/// A rung's raw columns: pot codes, current and duty.
struct Columns {
    pos: Vec<u16>,
    current_raw: Vec<u16>,
    duty: Vec<i16>,
}

fn columns(rows: &[TelFrame]) -> Columns {
    let mut c = Columns {
        pos: Vec::with_capacity(rows.len()),
        current_raw: Vec::with_capacity(rows.len()),
        duty: Vec::with_capacity(rows.len()),
    };
    for f in rows {
        c.pos.push(f.pos.unwrap_or(0));
        c.current_raw.push(f.current_raw.unwrap_or(0));
        c.duty.push(f.duty_counts.unwrap_or(0));
    }
    c
}

fn segment_of(rows: &[TelFrame], slots: &[u8; ADC_WIDE_SLOTS], fs: f64) -> Result<Segment, String> {
    let c = columns(rows);
    postable::segment(&c.pos, &c.current_raw, &c.duty, slots, fs).map_err(|e| segment_words(&e))
}

fn segment_words(e: &SegmentError) -> String {
    match e {
        SegmentError::NoDrive => "no row carries a duty".into(),
        SegmentError::Still => format!(
            "the pot never moved {} counts from the drive's edge",
            postable::PLAY_COUNTS
        ),
        SegmentError::Short => "too few rows past the spin-up".into(),
        SegmentError::NoLine => format!(
            "no ripple line in the current inside {:.0}..{:.0} Hz",
            crate::ripple::BAND_HZ.0,
            crate::ripple::BAND_HZ.1
        ),
        SegmentError::FewCrossings(n) => format!("{n} ripple crossings, too few to count"),
    }
}

/// Why a rung's rows are no evidence, if they are not: lost rows, or a
/// ripple that cannot be counted or counts over [`postable::ODD_MAX`] odd
/// intervals.
fn no_evidence(rows: &[TelFrame], lost: &[String], fs: f64) -> (Option<String>, Option<f64>) {
    if !lost.is_empty() {
        return (Some(format!("lost rows ({})", lost.join(", "))), None);
    }
    match segment_of(rows, &[0; ADC_WIDE_SLOTS], fs) {
        Err(w) => (Some(format!("its ripple is not counted: {w}")), None),
        Ok(s) if s.odd > postable::ODD_MAX => (
            Some(format!(
                "{:.1} % of its ripple's crossing intervals are odd, over {:.0} %",
                100.0 * s.odd,
                100.0 * postable::ODD_MAX
            )),
            Some(s.odd),
        ),
        Ok(s) => (None, Some(s.odd)),
    }
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

/// The pass a rung belongs to, its round trip and its way.
struct Place {
    trip: usize,
    pass: usize,
    dir: i8,
}

/// One rung from rest at `start`: sized, priced, driven, coasted and read,
/// the checks after it applied. None when it is left out, else the rest
/// position it ended at.
#[allow(clippy::too_many_arguments)]
async fn rung<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    fastest: &mut Fastest,
    at: &Place,
    k: usize,
    start: f64,
    lines: &mut Vec<String>,
) -> Result<Option<f64>, Halt> {
    let mut start = start;
    for attempt in 1..=ATTEMPTS {
        interrupted(rig.stop)?;
        let n = found.rungs.len() + found.unkept.len() + 1;
        let what = format!(
            "rung {n} (pass {} {}, rung {})",
            at.pass,
            way(at.dir),
            k + 1
        );
        let p = plan_rung(
            &b.band,
            at.dir,
            start,
            fastest.get(at.dir),
            share(at.trip - 1, k),
            b.tick_hz,
        );
        if !p.fits() {
            let why = format!("{what} from {start:.0}: {}", p.said());
            found.path.push(format!("left out: {why}"));
            found.left_out.push((at.pass, k + 1, why));
            return Ok(None);
        }
        found.path.push(format!(
            "{what}, attempt {attempt}: {}; bound from {}",
            p.said(),
            fastest.from(at.dir)
        ));
        let q15 = b.approach_q15 * at.dir as i16;
        b.env.admit(&rung_rung(&what, q15, &p))?;
        let coast_rows = (COAST_S * b.tick_hz).ceil().clamp(1.0, u16::MAX as f64) as u16;
        let plan = json!({"rung": what, "parts": ["drive", "coast"], "trip": at.trip, "pass": at.pass,
                          "k": k + 1, "dir": at.dir, "duty_q15": q15, "rows": p.rows, "coast_rows": coast_rows,
                          "t_s": p.t_s, "share": p.share, "start_counts": p.start,
                          "bound_speed_cps": p.bound_speed, "bound_coast_counts": p.bound_coast,
                          "attempt": attempt});
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
                rows: p.rows,
                end: End::Coast(coast_rows),
                plan: &plan,
            },
            taken,
            &b.servo,
        )
        .await?;
        let lost = got.lost();
        let cut = got
            .coast
            .as_ref()
            .and_then(|c| crate::fit::coast::cut_row(&c.rows))
            .unwrap_or(0);
        let stamps = got
            .drive
            .first_stamp
            .zip(got.coast.as_ref().and_then(|c| c.first_stamp));
        let near = got.on_s * b.tick_hz;
        let on_s = stamps.map(|(d, c)| (stamp_ticks(d, c, near) + cut as f64) / b.tick_hz);
        if let Some(s) = on_s.filter(|s| *s > p.t_s + DRIVE_DEADMAN_S + 1.0 / b.tick_hz) {
            return Err(Halt::Error(format!(
                "{what} drove {s:.3} s by the servo's stamps, over the {:.3} s it was priced at: the host is too slow between the bursts",
                p.t_s + DRIVE_DEADMAN_S
            )));
        }
        let end = at_rest(rig, b).await?;
        ntc_now(rig, b).await?;
        let rows = &taken[drive_k].burst.rows;
        let read = read_rung(at.dir, start, rows, end, b.tick_hz);
        let what_now = |r: &Option<RungRead>| {
            r.map_or(format!("at rest at {end:.0}"), |r| {
                format!(
                    "{:.0} of {:.0} counts/s, at rest at {end:.0} ({:.0} of {:.0} counts)",
                    r.speed_cps,
                    p.bound_speed,
                    r.travel,
                    p.travel_bound()
                )
            })
        };
        let (bad, odd) = no_evidence(rows, &lost, b.tick_hz);
        if let Some(why) = bad {
            if !b.band.holds(end) {
                return Err(Halt::Refused(format!(
                    "{what} ended at {end:.0}, outside the guards {:.0}..{:.0}, against a bound of {:.0} counts from {start:.0}; its rows are no evidence ({why}). Nothing more drives",
                    b.band.lo,
                    b.band.hi,
                    p.travel_bound()
                )));
            }
            lines.push(format!(
                "{what}, attempt {attempt}: {}; {why}, no evidence (captures {:02} and {:02})",
                what_now(&read),
                drive_k + 1,
                drive_k + 2
            ));
            found.unkept.push(Unkept {
                n,
                trip: at.trip,
                pass: at.pass,
                k: k + 1,
                attempt,
                why: why.clone(),
                captures: (drive_k + 1, drive_k + 2),
            });
            if attempt == ATTEMPTS {
                let lost_twice = !lost.is_empty();
                let words = format!(
                    "{what}: {ATTEMPTS} attempts in a row were no evidence, the last as {why}"
                );
                return Err(if lost_twice {
                    Halt::Error(format!(
                        "{words}: the line is not clean enough; check the cable and its contacts"
                    ))
                } else {
                    Halt::Refused(format!(
                        "{words}: the current carries no ripple the table can count at this duty"
                    ))
                });
            }
            start = end;
            continue;
        }
        let read = read
            .ok_or_else(|| Halt::Error(format!("{what}: its bursts carry too few rows to read")))?;
        lines.push(format!(
            "{what}: {} at {q15} Q15 from {start:.0} for {:.0} ms: {}, bounds from {}",
            way(at.dir),
            1e3 * p.t_s,
            what_now(&Some(read)),
            fastest.from(at.dir)
        ));
        after(&p, &read, &b.band, &what).map_err(Halt::Refused)?;
        fastest.feed(at.dir, read.speed_cps, what.clone());
        found.rungs.push(Taken1 {
            n,
            trip: at.trip,
            pass: at.pass,
            k: k + 1,
            attempt,
            q15,
            plan: p,
            read,
            on_s,
            drive_k,
            odd,
        });
        return Ok(Some(end));
    }
    Ok(None)
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
    let v = |q: i16| q as f64 / Q15 * b.rail_v;
    lines.push(format!(
        "volatile: the peak cap unset, the soft limits out to the physical range {:.0}..{:.0}, OpenLoop, dead-man {} ticks ({} s), torque on at zero duty; guards {:.0}..{:.0}, {:.0} % of the stops {:.0}..{:.0} in from each; every drive at the approach duty {:.3} V ({} ticks of drive window, i_window_min_ticks {}), each rung under a {DRIVE_DEADMAN_S} s dead-man, {COAST_S} s of coast",
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
        b.stream.i_window_min_ticks
    ));
    let mut fastest = Fastest::of_legs(&b.stops.legs);
    let mut pass = 0;
    for trip in 1..=ROUND_TRIPS {
        for dir in [1i8, -1] {
            pass += 1;
            let name = if dir > 0 {
                "the low guard's start"
            } else {
                "the high guard's start"
            };
            let mut at = go_to(rig, b, b.band.start(dir), name, lines).await?;
            let place = Place { trip, pass, dir };
            for k in 0..RUNGS_A_PASS {
                match rung(rig, b, taken, found, &mut fastest, &place, k, at, lines).await? {
                    Some(end) => at = end,
                    None => break,
                }
            }
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

fn pass(ok: bool) -> Verdict {
    if ok { Verdict::Pass } else { Verdict::Fail }
}

/// The check names, for [`advice`].
pub const WIDE_CODES: &str = "wide codes";
pub const ODD: &str = "ripple intervals odd";
pub const FIT: &str = "table fit";
pub const RESIDUAL: &str = "fit residual each way";
pub const TERM: &str = "up less down";
pub const TRIPS: &str = "each round trip against the pooled table";
pub const CYCLES: &str = "same-way rungs' cycles over shared counts";
pub const OVERLAP: &str = "every rung overlaps another of its way";
pub const GAINS: &str = "interval gains";
pub const COVERAGE: &str = "common span";
pub const VALID: &str = "the firmware's table check";
pub const SPREAD: &str = "speed along a rung";

/// What to do about a refused check.
pub fn advice(name: &str) -> &'static str {
    match name {
        WIDE_CODES => {
            "the pot's codes read a pattern the slot map cannot hold, or the sweep did not repeat; check the pot's wiring, then run the step again"
        }
        ODD | CYCLES | TRIPS | RESIDUAL | OVERLAP | FIT => {
            "the ripple did not count the motor's turn the same way each time: check that the horn turns freely and nothing touches it, then run the step again"
        }
        TERM => {
            "the two directions disagree past a miscount: run the step again; if it repeats, check the gear train for play"
        }
        GAINS | VALID => {
            "the pot's shape is steeper or flatter than a table may correct: check the pot"
        }
        COVERAGE => {
            "the rungs did not cover enough of the travel both ways: check the stops step's result, then run the step again"
        }
        _ => "check the servo, then run the step again",
    }
}

/// The pooled rows' reading: the slots, the table and every check.
pub struct Judged {
    pub finding: Option<Finding>,
    pub halves: Option<(Finding, Finding)>,
    pub swept: Option<(u16, u16)>,
    pub segments: Vec<(usize, Segment)>,
    pub fits: Option<(DirectionFit, DirectionFit)>,
    pub table: Option<Table>,
    pub term: Option<Term>,
    /// (trip, line out counts, slope share) or why it was not read.
    pub trips: Vec<(usize, Result<postable::Repeat, String>)>,
    pub points: Option<[i16; INTERVALS]>,
    pub scale: Option<(f64, String)>,
    pub checks: Vec<Check>,
}

/// One rung handed to the judge: its pass, its way and its drive rows.
#[derive(Copy, Clone, Debug)]
pub struct RungRows<'a> {
    pub n: usize,
    pub trip: usize,
    pub pass: usize,
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
    pub travel_deg: Option<f64>,
}

fn codes<'a, 'b: 'a>(rungs: impl Iterator<Item = &'a RungRows<'b>>) -> Vec<u16> {
    rungs
        .flat_map(|r| {
            r.rows
                .iter()
                .filter(|f| f.duty_counts.is_some_and(|d| d != 0))
                .filter_map(|f| f.pos)
        })
        .collect()
}

fn range(c: &[u16]) -> Option<(u16, u16)> {
    Some((*c.iter().min()?, *c.iter().max()?))
}

fn table_words(e: &TableError) -> String {
    match e {
        TableError::OneWay => "the rungs ran one way only".into(),
        TableError::NoSpan => "the two ways share under three grid points".into(),
        TableError::NoSamples(d) => format!("no {} rung has samples on the span", way(*d)),
        TableError::Singular => "the fit's equations are singular".into(),
        TableError::Hole {
            dir,
            from,
            intervals,
        } => format!(
            "the {} rungs leave {intervals} intervals without samples from {from:.0} counts, over {}",
            way(*dir),
            postable::HOLE_MAX
        ),
        TableError::SpanMismatch => "the two ways' spans differ".into(),
    }
}

/// The pooled rows to the slots and the table, every check with its
/// number and bound. A check an earlier refusal leaves nothing to read is
/// not judged.
pub fn judge(rungs: &[RungRows], c: &JudgeCfg) -> Judged {
    let mut j = Judged {
        finding: None,
        halves: None,
        swept: None,
        segments: Vec::new(),
        fits: None,
        table: None,
        term: None,
        trips: Vec::new(),
        points: None,
        scale: None,
        checks: Vec::new(),
    };
    let all = codes(rungs.iter());
    let odd_codes = codes(rungs.iter().filter(|r| r.pass % 2 == 1));
    let even_codes = codes(rungs.iter().filter(|r| r.pass % 2 == 0));
    let wide = (|| {
        let (lo, hi) = range(&all).ok_or_else(|| "no driven row".to_string())?;
        j.swept = Some((lo, hi));
        let f = widecodes::find(&all, lo, hi).map_err(|e| e.said())?;
        let half = |v: &[u16]| -> Result<Finding, String> {
            let (lo, hi) = range(v).ok_or_else(|| "a half holds no driven row".to_string())?;
            widecodes::find_in(&widecodes::histogram(v), lo, hi, HALF_THIN_ROWS)
                .map_err(|e| e.said())
        };
        let (even, odd) = (half(&even_codes)?, half(&odd_codes)?);
        let ok = widecodes::check(&f, &even, &odd, c.raw_min, c.raw_max).map_err(|e| e.said());
        j.finding = Some(f);
        j.halves = Some((even, odd));
        ok
    })();
    let not_judged = |j: &mut Judged, names: &[&str], why: &str| {
        for n in names {
            j.checks.push(check(
                n,
                format!("not read: {why}"),
                String::new(),
                Verdict::NotJudged,
            ));
        }
    };
    let later = [
        ODD, FIT, RESIDUAL, TERM, TRIPS, CYCLES, OVERLAP, GAINS, COVERAGE, VALID,
    ];
    let reached = |f: &Finding| {
        f.carries
            .iter()
            .filter(|k| k.state != State::Unreached)
            .count()
    };
    match &wide {
        Ok(()) => {
            let f = j.finding.as_ref().map_or(0, reached);
            j.checks.push(check(
                WIDE_CODES,
                format!(
                    "slots {:?}, {f} of 16 carries reached, no stray run, the halves within one unit",
                    j.finding.as_ref().map(|f| f.slots).unwrap_or_default()
                ),
                format!(
                    "no stray run, odd and even passes within {} Q4 at every carry both read, at least half the carries between the stops reached",
                    widecodes::Q4_PER_SKIPPED
                ),
                Verdict::Pass,
            ));
        }
        Err(why) => {
            j.checks.push(check(
                WIDE_CODES,
                why.clone(),
                "no stray run, the halves within one unit, half the carries reached".into(),
                Verdict::Fail,
            ));
            not_judged(&mut j, &later, "the wide codes refused");
            return j;
        }
    }
    let slots = j.finding.as_ref().map(|f| f.slots).unwrap_or_default();

    let mut worst_odd = 0.0f64;
    let mut unread = Vec::new();
    for r in rungs {
        match segment_of(r.rows, &slots, c.fs) {
            Ok(s) => {
                worst_odd = worst_odd.max(s.odd);
                j.segments.push((r.n, s));
            }
            Err(w) => unread.push(format!("rung {}: {w}", r.n)),
        }
    }
    let odd_over: Vec<usize> = j
        .segments
        .iter()
        .filter(|(_, s)| s.odd > postable::ODD_MAX)
        .map(|(n, _)| *n)
        .collect();
    j.checks.push(check(
        ODD,
        if unread.is_empty() {
            format!(
                "at most {:.2} % over {} rungs{}",
                100.0 * worst_odd,
                j.segments.len(),
                if odd_over.is_empty() {
                    String::new()
                } else {
                    format!(", over it on rungs {odd_over:?}")
                }
            )
        } else {
            unread.join("; ")
        },
        format!(
            "every rung at most {:.0} % (outside {}..{} x the running median of {})",
            100.0 * postable::ODD_MAX,
            crate::ripple::REPAIR_SHORT,
            crate::ripple::REPAIR_LONG,
            crate::ripple::REPAIR_WINDOW
        ),
        pass(unread.is_empty() && odd_over.is_empty()),
    ));
    let segs: Vec<Segment> = j.segments.iter().map(|(_, s)| s.clone()).collect();
    let fitted = (|| {
        let span = postable::common_span(&segs)?;
        let up = postable::fit_direction(&segs, 1, &span, SMOOTH_WEIGHT)?;
        let down = postable::fit_direction(&segs, -1, &span, SMOOTH_WEIGHT)?;
        let table = postable::mean_table(&up, &down)?;
        Ok::<_, TableError>((up, down, table))
    })();
    let (up, down, table) = match fitted {
        Ok(f) => f,
        Err(e) => {
            j.checks.push(check(
                FIT,
                table_words(&e),
                format!(
                    "both ways on a common span, no hole over {} intervals",
                    postable::HOLE_MAX
                ),
                Verdict::Fail,
            ));
            not_judged(
                &mut j,
                &[
                    RESIDUAL, TERM, TRIPS, CYCLES, OVERLAP, GAINS, COVERAGE, VALID,
                ],
                "the table was not fitted",
            );
            return j;
        }
    };
    let span = table.span;
    j.checks.push(check(
        FIT,
        format!(
            "{} up and {} down samples on {:.0}..{:.0} counts, {} grid points",
            up.samples,
            down.samples,
            span.lo(),
            span.hi(),
            span.count()
        ),
        format!(
            "both ways on a common span, no hole over {} intervals",
            postable::HOLE_MAX
        ),
        Verdict::Pass,
    ));
    let (ru, rd) = (
        postable::residual_counts(&up, &table),
        postable::residual_counts(&down, &table),
    );
    j.checks.push(check(
        RESIDUAL,
        format!("up {ru:.2}, down {rd:.2} counts rms"),
        format!("each at most {RESIDUAL_MAX_COUNTS} counts rms"),
        pass(ru <= RESIDUAL_MAX_COUNTS && rd <= RESIDUAL_MAX_COUNTS),
    ));
    let term = postable::direction_term(&up, &down, &table);
    j.checks.push(check(
        TERM,
        format!(
            "{:.2} counts rms line out, {:.2} max",
            term.rms, term.max
        ),
        format!(
            "warned over {TERM_WARN_COUNTS}, refused over {TERM_MAX_COUNTS} counts rms (a miscount); one table cannot fix it"
        ),
        if term.rms > TERM_MAX_COUNTS {
            Verdict::Fail
        } else if term.rms > TERM_WARN_COUNTS {
            Verdict::Warn
        } else {
            Verdict::Pass
        },
    ));
    j.term = Some(term);

    // a round trip's own table on the pooled span: its rungs within a pass
    // do not overlap, so their offsets rest on the smoothing there, and
    // the hole rule, which is the written table's, is not applied
    let trips: Vec<usize> = {
        let mut t: Vec<usize> = rungs.iter().map(|r| r.trip).collect();
        t.dedup();
        t
    };
    for &t in &trips {
        let mine: Vec<Segment> = j
            .segments
            .iter()
            .filter(|(n, _)| rungs.iter().any(|r| r.n == *n && r.trip == t))
            .map(|(_, s)| s.clone())
            .collect();
        let got = (|| {
            let u = postable::fit_direction(&mine, 1, &span, SMOOTH_WEIGHT)?;
            let d = postable::fit_direction(&mine, -1, &span, SMOOTH_WEIGHT)?;
            let own = Table {
                span,
                angle: u
                    .angle
                    .iter()
                    .zip(&d.angle)
                    .map(|(a, b)| (a + b) / 2.0)
                    .collect(),
            };
            postable::repeat(&own, &table)
        })()
        .map_err(|e| table_words(&e));
        j.trips.push((t, got));
    }
    let trips_ok = !j.trips.is_empty()
        && j.trips.iter().all(|(_, r)| {
            r.as_ref()
                .is_ok_and(|r| r.line_out <= TRIP_LINE_OUT_MAX_COUNTS && r.slope <= TRIP_SLOPE_MAX)
        });
    j.checks.push(check(
        TRIPS,
        j.trips
            .iter()
            .map(|(t, r)| match r {
                Ok(r) => format!(
                    "trip {t}: {:.2} counts line out, {:.1} % slope",
                    r.line_out,
                    100.0 * r.slope
                ),
                Err(w) => format!("trip {t}: {w}"),
            })
            .collect::<Vec<_>>()
            .join("; "),
        format!(
            "each at most {TRIP_LINE_OUT_MAX_COUNTS} counts rms line out and {:.0} % rms in 48-count slope",
            100.0 * TRIP_SLOPE_MAX
        ),
        pass(trips_ok),
    ));

    let inside = |s: &Segment| s.x.iter().any(|&v| v >= span.lo() && v <= span.hi());
    let mut pairs = 0;
    let mut worst: Option<(usize, usize, postable::Agreement)> = None;
    let mut alone = Vec::new();
    for (a, (na, sa)) in j.segments.iter().enumerate() {
        let mut overlaps = false;
        for (nb, sb) in j.segments.iter().skip(a + 1) {
            if let Some(g) = postable::agreement(sa, sb) {
                pairs += 1;
                let over = |g: &postable::Agreement| (g.a - g.b).abs() / g.limit();
                if worst.as_ref().is_none_or(|(_, _, w)| over(&g) > over(w)) {
                    worst = Some((*na, *nb, g));
                }
            }
        }
        for (nb, sb) in j.segments.iter() {
            if nb != na && postable::agreement(sa, sb).is_some() {
                overlaps = true;
            }
        }
        if inside(sa) && !overlaps {
            alone.push(*na);
        }
    }
    let cycles_ok = j.segments.iter().enumerate().all(|(a, (_, sa))| {
        j.segments
            .iter()
            .skip(a + 1)
            .filter_map(|(_, sb)| postable::agreement(sa, sb))
            .all(|g| g.holds())
    });
    j.checks.push(check(
        CYCLES,
        match &worst {
            Some((a, b, g)) => format!(
                "{pairs} pairs; the farthest, rungs {a} and {b} over {:.0}..{:.0}: {:.2} and {:.2} cycles",
                g.from, g.to, g.a, g.b
            ),
            None => "no two rungs of one way share a grid interval".into(),
        },
        format!(
            "every pair within the larger of {} cycle and {:.1} %",
            postable::AGREE_MIN_CYCLES,
            100.0 * postable::AGREE_SHARE
        ),
        pass(pairs > 0 && cycles_ok),
    ));
    j.checks.push(check(
        OVERLAP,
        if alone.is_empty() {
            "every fitted rung shares a grid interval with another of its way".into()
        } else {
            format!("rungs {alone:?} share none")
        },
        "every rung on the span overlaps another of its way: else its offset rests on the smoothing alone".into(),
        pass(alone.is_empty()),
    ));

    let points = table.points(c.raw_min, c.raw_max);
    let (gmin, gmax) = postable::gains(&points);
    j.checks.push(check(
        GAINS,
        format!("{gmin}..{gmax} Q4"),
        format!("every interval inside {}..{} Q4", GAIN_Q4.0, GAIN_Q4.1),
        pass(gmin >= GAIN_Q4.0 && gmax <= GAIN_Q4.1),
    ));
    let cov = span.coverage(c.raw_min, c.raw_max);
    j.checks.push(check(
        COVERAGE,
        format!(
            "{:.0}..{:.0} counts, {:.1} % of the stops' {}..{}",
            span.lo(),
            span.hi(),
            100.0 * cov,
            c.raw_min,
            c.raw_max
        ),
        format!("at least {:.0} % of the stops' span", 100.0 * COVERAGE_MIN),
        pass(cov >= COVERAGE_MIN),
    ));
    let mut full = [0i16; POINTS];
    full[..INTERVALS].copy_from_slice(&points);
    let valid = pos_lut::validate(&full, c.raw_min, c.raw_max);
    j.checks.push(check(
        VALID,
        match valid {
            Ok(()) => "accepted".into(),
            Err(e) => format!("rejected: {e:?}"),
        },
        "zero at and past each stop, every gain 1..255 Q4 (pos_lut::validate)".into(),
        pass(valid.is_ok()),
    ));
    let spread = j
        .segments
        .iter()
        .map(|(n, s)| (*n, s.speed_spread))
        .fold((0, 0.0f64), |m, x| if x.1 > m.1 { x } else { m });
    j.checks.push(check(
        SPREAD,
        format!("at most {:.1} % (rung {})", 100.0 * spread.1, spread.0),
        format!(
            "advisory: the ripple line over a rung's quarters within {:.0} %",
            100.0 * SPREAD_WARN
        ),
        if spread.1 > SPREAD_WARN {
            Verdict::Warn
        } else {
            Verdict::Pass
        },
    ));
    j.scale = match (table.scale(c.ripple_per_rev, c.gear_ratio), c.travel_deg) {
        (Some(s), _) => Some((
            s,
            format!(
                "the ripple over the common span: {} commutations a revolution x a gear ratio of {} over the table's mean slope",
                c.ripple_per_rev.unwrap_or(0),
                c.gear_ratio.unwrap_or(0.0)
            ),
        )),
        (None, Some(t)) => Some((
            (c.raw_max as f64 - c.raw_min as f64) / t,
            format!(
                "the stops' span {}..{} over the nameplate's travel_deg {t}: the nameplate states no ripple_per_rev and gear_ratio both",
                c.raw_min, c.raw_max
            ),
        )),
        (None, None) => None,
    };
    j.points = Some(points);
    j.fits = Some((up, down));
    j.table = Some(table);
    j
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
    let ran = body(rig, &mut b, &mut ledger, &mut taken, &mut found, &mut lines).await;
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
            trip: r.trip,
            pass: r.pass,
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
    let accepted = way == Way::Done;
    let drives = json!({
            "stops_counts": [b.stops.lo, b.stops.hi], "guards_counts": [b.band.lo, b.band.hi],
            "guard_share": GUARD_SHARE, "start_inside_counts": START_INSIDE_COUNTS,
            "approach_q15": b.approach_q15, "approach_v": b.approach_q15 as f64 / Q15 * b.rail_v,
            "window_ticks": b.window_ticks, "pwm_period_ticks": b.pwm_arr,
            "i_window_min_ticks": b.stream.i_window_min_ticks,
            "round_trips": ROUND_TRIPS, "rungs_a_pass": RUNGS_A_PASS, "stagger": STAGGER,
            "rung_min_s": RUNG_MIN_S, "free_margin": FREE_MARGIN, "coast_tau_s": COAST_TAU_S,
            "burst_max_s": BURST_MAX_S, "drive_deadman_s": DRIVE_DEADMAN_S,
            "poll_deadman_s": POLL_DEADMAN_S, "coast_s": COAST_S, "still_counts": b.still_counts,
            "approach_legs": b.stops.legs.iter().map(|l| json!({"n": l.n, "dir": l.dir, "counts": l.counts,
                "secs": l.secs, "speed_cps": l.speed_cps()})).collect::<Vec<_>>(),
    });
    let rungs: Vec<Value> = found
        .rungs
        .iter()
        .map(|r| {
            let p = &r.plan;
            let seg = judged.and_then(|j| j.segments.iter().find(|(n, _)| *n == r.n).map(|(_, s)| s));
            json!({
                "n": r.n, "trip": r.trip, "pass": r.pass, "k": r.k, "dir": r.plan.dir, "attempt": r.attempt,
                "duty_q15": r.q15, "start_counts": p.start,
                "planned": {"t_s": p.t_s, "rows": p.rows, "share": p.share, "t_runway_s": p.t_runway_s, "t_burst_s": p.t_burst_s,
                            "runway_counts": p.runway, "far_guard_counts": p.far_guard, "fastest_cps": p.fastest_cps,
                            "bound_speed_cps": p.bound_speed, "bound_coast_counts": p.bound_coast, "travel_bound_counts": p.travel_bound()},
                "on_s": r.on_s, "end_counts": r.read.end, "travel_counts": r.read.travel,
                "drive_travel_counts": r.read.drive_travel, "pot_speed_cps": r.read.speed_cps,
                "odd_share_run": r.odd,
                "segment": seg.map(|s| json!({"f_seed_hz": s.f_seed, "crossings": s.crossings, "samples": s.x.len(),
                    "odd_share": s.odd, "speed_spread": s.speed_spread})),
                "captures": [r.drive_k + 1, r.drive_k + 2],
            })
        })
        .collect();
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
        "stops_used": {"raw_min": cfg.raw_min, "raw_max": cfg.raw_max,
                       "source": "the stops step's stops.low.median_counts up and stops.high.median_counts down: rounded inward, as the gather writes them"},
        "scale": null,
        "rungs": rungs,
        "unkept": found.unkept.iter().map(|u| json!({"n": u.n, "trip": u.trip, "pass": u.pass, "k": u.k,
            "attempt": u.attempt, "why": u.why, "captures": [u.captures.0, u.captures.1]})).collect::<Vec<_>>(),
        "left_out": found.left_out.iter().map(|(p, k, w)| json!({"pass": p, "k": k, "why": w})).collect::<Vec<_>>(),
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
    let (Some(j), Value::Object(o)) = (judged, &mut out) else {
        return out;
    };
    o.insert("swept_counts".into(), json!(j.swept));
    o.insert(
        "wide_codes".into(),
        json!({
            "all": j.finding.as_ref().map(finding_json),
            "even_passes": j.halves.as_ref().map(|h| finding_json(&h.0)),
            "odd_passes": j.halves.as_ref().map(|h| finding_json(&h.1)),
            "thin_rows": THIN_ROWS, "half_thin_rows": HALF_THIN_ROWS,
        }),
    );
    if let (Some(t), Some((up, down))) = (&j.table, &j.fits) {
        let span = t.span;
        let reach = |dir: i8| {
            j.segments
                .iter()
                .filter(|(_, s)| s.dir == dir)
                .flat_map(|(_, s)| s.x.iter().copied())
                .fold(None, |acc: Option<(f64, f64)>, v| {
                    Some(acc.map_or((v, v), |(l, h)| (l.min(v), h.max(v))))
                })
        };
        o.insert(
            "coverage".into(),
            json!({"common_span_counts": [span.lo(), span.hi()], "grid_points": span.count(),
                   "share_of_stops": span.coverage(cfg.raw_min, cfg.raw_max),
                   "up_counts": reach(1), "down_counts": reach(-1)}),
        );
        o.insert(
            "fit".into(),
            json!({"knots_counts": [span.lo(), span.hi()], "angle_cycles": t.angle,
                   "up": {"angle_cycles": up.angle, "offsets_cycles": up.offsets, "samples": up.samples,
                          "residual_counts": postable::residual_counts(up, t)},
                   "down": {"angle_cycles": down.angle, "offsets_cycles": down.offsets, "samples": down.samples,
                            "residual_counts": postable::residual_counts(down, t)},
                   "smooth_weight": SMOOTH_WEIGHT, "counts_per_cycle": t.counts_per_cycle(),
                   "trips": j.trips.iter().map(|(n, r)| match r {
                       Ok(r) => json!({"trip": n, "line_out_counts": r.line_out, "slope_rms": r.slope}),
                       Err(w) => json!({"trip": n, "error": w}),
                   }).collect::<Vec<_>>()}),
        );
    }
    if let Some(term) = &j.term {
        o.insert(
            "direction_term".into(),
            json!({"rms_counts": term.rms, "max_counts": term.max,
                   "bands": term.bands.iter().map(|(b0, r)| json!({"from_counts": b0, "rms_counts": r})).collect::<Vec<_>>(),
                   "rule": "the table is the mean of the two ways' shapes: half of this term stays in each way"}),
        );
    }
    if accepted {
        if let Some(f) = &j.finding {
            o.insert("slots".into(), json!(f.slots));
            o.insert(
                "slots_zero".into(),
                json!(
                    f.carries
                        .iter()
                        .filter_map(|c| carry_reason(c.state)
                            .map(|w| json!({"slot": c.slot, "code": c.code, "why": w})))
                        .collect::<Vec<_>>()
                ),
            );
        }
        if let Some(p) = &j.points {
            let pages: Vec<Vec<i16>> = (0..PAGES)
                .map(|k| p[k * PAGE_POINTS..(k + 1) * PAGE_POINTS].to_vec())
                .collect();
            o.insert("points_by_page".into(), json!(pages));
        }
        if let Some((s, src)) = &j.scale {
            o.insert("scale".into(), json!({"counts_per_deg": s, "source": src}));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::sync::atomic::AtomicBool;

    use futures_lite::future::block_on;
    use osc_client::Client;
    use serde_json::json;

    use super::*;
    use crate::sink::{MemSink, Sink};
    use crate::steps::runway;
    use crate::steps::testkit::{self, ID, Motor, OnSent, Plant, Sent, Spy, addr, descriptor};

    const APPROACH_Q15: i16 = 4000;
    /// Where the shaft rests before the step, straight counts.
    const X_START: f64 = 2000.0;

    /// The bench servo's class at its approach duty: 750 counts/s, 3.3
    /// counts a commutation cycle, a pot reading 1.5x fast at 1900, wide
    /// codes at carries 4 and 9.
    fn motor() -> Motor {
        Motor {
            cps_full: 750.0 * 32768.0 / APPROACH_Q15 as f64,
            cycles_per_count: 0.3,
            rise_tau_s: 0.02,
            coast_tau_s: 0.03,
            stops: (300.0, 3700.0),
            bend: (1900.0, 30.0, 30.0),
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
        let v = q15 as f64 / Q15 * m.cps_full * leg_share;
        let leg = |n: usize, kind: &str, toward: &str, from: f64, to: f64| {
            json!({"n": n, "kind": kind, "toward": toward, "from_counts": from, "to_counts": to,
                   "secs": (to - from).abs() / v})
        };
        json!({
            "verdict": "accepted",
            "stops": {
                "low": {"median_counts": lo}, "high": {"median_counts": hi},
                "legs": [
                    leg(1, "approach", "low", 2030.0, lo),
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
        plant: Rc<RefCell<Plant>>,
    }

    fn run(m: Motor, stops: Value, also: OnSent) -> Ran {
        let d = descriptor();
        let (mut c, log, plant) = testkit::motor_servo(m, X_START, also);
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
                },
            ))
        };
        let log = log.borrow().clone();
        Ran {
            out,
            c,
            sink,
            log,
            plant,
        }
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

    /// The whole step on a simulated motor: three round trips, every rung
    /// inside its guards and its bound, the slots of the converter's
    /// carries, a table that makes the pot read straight against the
    /// motor's turn, and the shaft at the middle with torque off.
    #[test]
    #[ignore = "the fake adapter's runaway guard (10 M events) trips in the fifth pass: its streams run their whole window, 1.25 x the rows and 250 ms"]
    fn a_simulated_run_writes_the_table_and_ends_at_the_middle() {
        let m = motor();
        let mut r = run(
            m.clone(),
            stops_json(&m, APPROACH_Q15, 0.95),
            testkit::quiet(),
        );
        let t = r.out.expect("starts");
        let e = &t.ended;
        assert_eq!(e.way, Way::Done, "{:?}\n{}", e.why, e.lines.join("\n"));
        assert!(torque_off(&mut r.c) && e.exit.torque_off, "{}", e.exit);
        let res = result(&mut r.sink);
        assert_eq!(&res, t.result.as_ref().expect("result"));
        let mut want = [0u8; 16];
        (want[4], want[9]) = (88, 48);
        assert_eq!(res["slots"], json!(want), "{}", res["wide_codes"]);
        assert_eq!(
            res["wide_codes"]["all"]["carries"][13]["state"],
            "unreached"
        );
        assert!(res["slots_zero"].as_array().is_some_and(|z| !z.is_empty()));

        // every rung inside its guards, none past its bound
        let rungs = res["rungs"].as_array().expect("rungs");
        let n = |v: &Value| v.as_f64().expect("number");
        let (glo, ghi) = (
            n(&res["drives"]["guards_counts"][0]),
            n(&res["drives"]["guards_counts"][1]),
        );
        assert!(rungs.len() >= 2 * ROUND_TRIPS * 2, "{}", rungs.len());
        for x in rungs {
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
            assert!(n(&x["planned"]["t_s"]) >= RUNG_MIN_S, "{x}");
        }
        let first: Vec<f64> = (1..=ROUND_TRIPS)
            .map(|trip| {
                rungs
                    .iter()
                    .find(|x| x["trip"] == trip && x["k"] == 1 && x["dir"] == 1)
                    .map(|x| n(&x["planned"]["share"]))
                    .expect("a first rung up")
            })
            .collect();
        assert_eq!(first, STAGGER);

        let pages = res["points_by_page"].as_array().expect("pages");
        let points: Vec<i16> = pages
            .iter()
            .flat_map(|p| {
                p.as_array()
                    .expect("page")
                    .iter()
                    .map(|v| v.as_i64().expect("point") as i16)
            })
            .collect();
        straightens(&m, &points);
        let cpd = n(&res["scale"]["counts_per_deg"]);
        assert!(
            cpd > 0.0
                && res["scale"]["source"]
                    .as_str()
                    .is_some_and(|s| s.contains("ripple")),
            "{}",
            res["scale"]
        );
        assert!(n(&res["coverage"]["share_of_stops"]) >= COVERAGE_MIN);
        for c in res["checks"].as_array().expect("checks") {
            assert!(c["verdict"] == "pass" || c["verdict"] == "warning", "{c}");
        }
        let middle = (glo + ghi) / 2.0;
        let after = n(&res["position_after_counts"]);
        assert!(
            (after - middle).abs() <= 2.0 * runway::ARRIVE_COUNTS,
            "{after} against {middle}"
        );
        let (x, _) = {
            let mut c = r.c;
            let p = r.plant.borrow();
            p.now(c.pipe_mut().inner_mut())
        };
        assert!((m.pot(x) - after).abs() <= 2.0, "{x}");
        assert!(
            r.sink
                .files
                .keys()
                .any(|k| k.starts_with("captures/table/"))
        );
        assert!(!r.log.is_empty());
    }

    /// An approach duty whose drive window is under the board's floor
    /// refuses before any write, and says so in its result.
    #[test]
    fn a_duty_under_the_window_floor_refuses_before_any_write() {
        let m = motor();
        let mut r = run(m.clone(), stops_json(&m, 1000, 0.95), testkit::quiet());
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

    /// A shaft faster than the approach legs allow runs its first rung
    /// past the far guard: the step ends there, torque off, nothing
    /// written.
    #[test]
    fn a_rung_past_its_guard_ends_the_step() {
        let mut m = motor();
        m.cps_full *= 2.4;
        let mut r = run(
            m.clone(),
            stops_json(&m, APPROACH_Q15, 0.3),
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
        let res = nothing_written(&mut r.sink);
        assert_eq!(res["rungs"].as_array().map(Vec::len), Some(0), "{res}");
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
        let mut r = run(m.clone(), stops_json(&m, APPROACH_Q15, 0.95), lose);
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

    /// Straight count `x` where the pot reads `u`.
    fn x_of(m: &Motor, u: f64) -> f64 {
        let (mut lo, mut hi) = m.stops;
        for _ in 0..60 {
            let mid = (lo + hi) / 2.0;
            if m.pot(mid) < u { lo = mid } else { hi = mid }
        }
        lo
    }

    /// The table makes the pot read straight against the motor's turn:
    /// each grid point's corrected count within 3 of the straight count
    /// there, both stops pinned to themselves, the bend taken off.
    fn straightens(m: &Motor, points: &[i16]) {
        assert_eq!(points.len(), INTERVALS);
        let mut full = [0i16; POINTS];
        full[..INTERVALS].copy_from_slice(points);
        let (raw_min, raw_max) = (
            m.pot(m.stops.0).ceil() as u16,
            m.pot(m.stops.1).floor() as u16,
        );
        assert_eq!(pos_lut::validate(&full, raw_min, raw_max), Ok(()));
        let (x0, x1) = (x_of(m, raw_min as f64), x_of(m, raw_max as f64));
        let truth =
            |u: f64| raw_min as f64 + (x_of(m, u) - x0) * (raw_max - raw_min) as f64 / (x1 - x0);
        let mut worst = 0.0f64;
        let (k0, k1) = (raw_min as usize / 16 + 2, raw_max as usize / 16 - 1);
        for (k, c) in points.iter().enumerate().take(k1).skip(k0) {
            let u = (k * 16) as f64;
            worst = worst.max((u + *c as f64 - truth(u)).abs());
        }
        assert!(worst <= 3.0, "the table is off the motor by {worst} counts");
        let bend = points[1920 / 16] - points[1872 / 16];
        assert!(bend < -15, "the bend's 30 counts come off: {bend}");
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

    /// The step's passes on `m` with no bus: each pass from its start,
    /// each rung sized by [`plan_rung`] from where the last came to rest,
    /// its rows the motor's, the speeds seen fed as the step feeds them.
    fn passes(m: &Motor, legs_cps: f64) -> Vec<(usize, usize, i8, Vec<TelFrame>)> {
        let band = Band::of_stops(m.pot(m.stops.0), m.pot(m.stops.1));
        let mut fastest = Fastest::default();
        for dir in [1, -1] {
            fastest.feed(dir, legs_cps, "the approach legs".into());
        }
        let mut out = Vec::new();
        let mut pass = 0;
        for trip in 1..=ROUND_TRIPS {
            for dir in [1i8, -1] {
                pass += 1;
                let mut x = x_of(m, band.start(dir));
                for k in 0..RUNGS_A_PASS {
                    let p = plan_rung(
                        &band,
                        dir,
                        m.pot(x),
                        fastest.get(dir),
                        share(trip - 1, k),
                        20_000.0,
                    );
                    if !p.fits() {
                        break;
                    }
                    let q = APPROACH_Q15 * dir as i16;
                    let (track, states) =
                        m.run(x, 0.0, 0, q, testkit::EDGE_ROWS, Some(p.rows as usize));
                    let rows = frames(&track[..testkit::EDGE_ROWS + p.rows as usize]);
                    let end = states.last().expect("a track").0;
                    let r = read_rung(dir, m.pot(x), &rows, m.pot(end), 20_000.0).expect("reads");
                    assert_eq!(after(&p, &r, &band, "a rung"), Ok(()));
                    fastest.feed(dir, r.speed_cps, format!("rung {}", out.len() + 1));
                    out.push((trip, pass, dir, rows));
                    x = end;
                }
            }
        }
        out
    }

    fn judged(m: &Motor) -> Judged {
        let ran = passes(m, 0.95 * APPROACH_Q15 as f64 / Q15 * m.cps_full);
        let rungs: Vec<RungRows> = ran
            .iter()
            .enumerate()
            .map(|(n, (trip, pass, dir, rows))| RungRows {
                n: n + 1,
                trip: *trip,
                pass: *pass,
                dir: *dir,
                rows,
            })
            .collect();
        let np = testkit::nameplate();
        judge(
            &rungs,
            &JudgeCfg {
                fs: 20_000.0,
                raw_min: m.pot(m.stops.0).ceil() as u16,
                raw_max: m.pot(m.stops.1).floor() as u16,
                ripple_per_rev: np.ripple_per_rev,
                gear_ratio: np.gear_ratio,
                travel_deg: np.travel_deg,
            },
        )
    }

    /// The step's capture on the simulated motor, judged: every check
    /// passes, the slots are the converter's carries and the unreached
    /// ones 0, and the table makes the pot read straight.
    #[test]
    fn the_capture_on_a_simulated_motor_gives_its_slots_and_table() {
        let m = motor();
        let j = judged(&m);
        for c in &j.checks {
            assert!(
                matches!(c.verdict, Verdict::Pass | Verdict::Warn),
                "{}",
                c.said()
            );
        }
        let f = j.finding.as_ref().expect("found");
        let mut want = [0u8; 16];
        (want[4], want[9]) = (88, 48);
        assert_eq!(f.slots, want);
        assert_eq!(f.carries[13].state, State::Unreached);
        assert!(
            f.carries[2..13]
                .iter()
                .all(|c| c.state != State::Unreached && c.state != State::Thin),
            "{:?}",
            f.carries
        );
        straightens(&m, j.points.as_ref().expect("points"));
        let (cpd, src) = j.scale.as_ref().expect("scale");
        assert!(*cpd > 0.0 && src.contains("ripple"), "{src}");
        assert!(j.term.as_ref().is_some_and(|t| t.rms < 1.0));
        assert_eq!(j.trips.len(), ROUND_TRIPS);
    }

    /// A never-read run under a code that is no carry: the wide-code
    /// finder's refusal comes through with its line, and nothing after it
    /// is read.
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
        assert!(
            j.checks
                .iter()
                .filter(|c| c.name != WIDE_CODES)
                .all(|c| c.verdict == Verdict::NotJudged)
        );
        assert!(j.points.is_none() && j.table.is_none());
        assert_eq!(advice(WIDE_CODES), advice(&wide.name));
    }
}
