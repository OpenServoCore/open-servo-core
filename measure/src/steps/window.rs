//! Step 4, the window term, on the stops step's result: open-loop onsets
//! from rest read by [`crate::fit::window`], the stage 3 window-term
//! notebook's fit.
//!
//! The shaft starts within [`BAND_SHARE`] of the travel of the middle of
//! the two stops, where the stops step leaves it. 24 onsets: the three
//! duties nearest the notebook's 25, 27 and 29 % of the rail whose drive
//! window is at least twice `i_window_min_ticks` ([`duties`]), both signs,
//! four each, the sign alternating so the shaft stays where it started
//! ([`plan`]). Each is [`COAST_S`] at zero duty with torque on, the pot
//! polled and found still, then [`ONSET_S`] of drive by one committed
//! capture ([`super::drive`]): the burst opens at the commit and the edge
//! lands at the next medium tick, so the rows ahead of it are the shaft at
//! rest the fit needs. Every onset is priced as a free drive of its rows
//! and its dead-man, and its worst case kept [`INSET_COUNTS`] inside both
//! stops. A shaft that drifts out of the band ends the step.
//!
//! `g_window_q016` is written 0 volatile first, as the notebook ran. The
//! firmware takes the term off its own reading only (`i_tick_counts`,
//! which the peak cap and the limits act on); the raw row's crest sample
//! carries the whole term whatever the field holds, and with the cap
//! unset nothing in OpenLoop acts on the reading.
//!
//! The notebook's direction check is a warning here: when it fails, a
//! second set of 24 is taken once and all 48 fitted. Every other check
//! refuses, and so do fewer than [`GATED_MIN`] of every 24 onsets through
//! the gate.

use std::time::Duration;

use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_servo_core::estimator::window::drive_ticks;
use serde_json::{Map, Value, json};

use super::drive::{self, Deadmen, Drive, End, priced_s};
use super::probe::FromRest;
use super::stops::{self, FromProbe};
use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, mode_value, read_u16, rest,
    seek, write,
};
use crate::Board;
use crate::capture::meta;
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{self, Line, StreamCfg};
use crate::envelope::{Envelope, Facts, HeatModel, Kind, Price, Resistance, Rung};
use crate::fit::median;
use crate::fit::window::{
    self as term, Method, Onset, Scales, Segment, Settle, Window, WindowError, acceptance,
    field_q016,
};
use crate::guard::{self, Ledger, Way};
use crate::inputs::{Inputs, Measured, Quantity, Source, ThermalInputs};
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::ntc::ADC_COUNTS;
use crate::rules::motor::R_AT_C;
use crate::rules::units::FAST_TICKS_PER_MEDIUM;
use crate::rules::{Outcome, build};
use crate::state::{self, State, field};

pub const STEP: &str = "window";
pub const RESULT: &str = "results/window.json";
const METHOD: &str = "window-term-onsets";
const REVISION: u32 = 1;
/// The notebook's duties, its onsets' 25, 27 and 29 % as osc-lab wrote
/// them (`pct_q15`).
pub const NOTEBOOK_Q15: [i16; 3] = [8191, 8847, 9502];
/// A duty's drive window at least this many times `i_window_min_ticks`.
pub const FLOOR_TIMES: u32 = 2;
/// Onsets a duty a sign in a set.
pub const EACH: usize = 4;
pub const SET: usize = 2 * NOTEBOOK_Q15.len() * EACH;
pub const SETS_MAX: usize = 2;
pub const ONSET_S: f64 = 0.050;
pub const COAST_S: f64 = 0.500;
/// The longest wait past [`COAST_S`] for a shaft that is not yet still.
pub const STILL_MAX_S: f64 = 2.0;
/// The band around the middle of the stops the shaft stays in, a share of
/// their travel.
pub const BAND_SHARE: f64 = 0.20;
/// The least an onset's worst case keeps from either stop, counts.
pub const INSET_COUNTS: f64 = 300.0;
/// Onsets through the gate, of every [`SET`].
pub const GATED_MIN: usize = 20;
/// The name `fit::window::acceptance` gives the direction check.
pub const DIRECTIONS: &str = "the two directions within 10 %";
const Q15: f64 = 32768.0;
const PRICE_WORDS: &str = "the probe's pricing R (planning grade)";

/// The three duties, Q15: the notebook's when each drive window is at
/// least [`FLOOR_TIMES`] x `i_window_min_ticks` at `pwm_arr`, else each
/// raised to the least duty that is; refused when that leaves fewer than
/// three.
pub fn duties(pwm_arr: u16, i_window_min_ticks: u16) -> Result<[i16; 3], Refused> {
    let need = FLOOR_TIMES * i_window_min_ticks as u32;
    let passes = |q: i16| drive_ticks(q, pwm_arr) >= need;
    if NOTEBOOK_Q15.iter().all(|&q| passes(q)) {
        return Ok(NOTEBOOK_Q15);
    }
    let least = (1..=i16::MAX).find(|&q| passes(q)).ok_or_else(|| {
        Refused(format!(
            "no duty gives a drive window of {need} ticks, twice i_window_min_ticks {i_window_min_ticks}, on a PWM period of {pwm_arr} ticks"
        ))
    })?;
    let out = NOTEBOOK_Q15.map(|q| q.max(least));
    if out[0] == out[1] || out[1] == out[2] {
        return Err(Refused(format!(
            "a drive window of {need} ticks, twice i_window_min_ticks {i_window_min_ticks}, needs {:.1}% duty on a PWM period of {pwm_arr} ticks: that leaves fewer than three duties near 25, 27 and 29 %",
            100.0 * least as f64 / Q15
        )));
    }
    Ok(out)
}

/// One onset of the plan.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Planned {
    /// From 1, over the run.
    pub n: usize,
    /// From 0.
    pub set: usize,
    pub duty_q15: i16,
}

/// A set's onsets, numbered on from `after`: each duty in turn at `first`
/// sign and then the other, [`EACH`] rounds, so the sign alternates.
pub fn plan(duties: [i16; 3], first: i8, set: usize, after: usize) -> Vec<Planned> {
    let mut out = Vec::with_capacity(SET);
    for _ in 0..EACH {
        for q in duties {
            for s in [first, -first] {
                out.push(Planned {
                    n: after + out.len() + 1,
                    set,
                    duty_q15: q * s as i16,
                });
            }
        }
    }
    out
}

/// The first sign of a set: toward the middle.
pub fn first_sign(pos: f64, middle: f64) -> i8 {
    if pos > middle { -1 } else { 1 }
}

/// The band around the middle of the stops.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Band {
    pub lo: f64,
    pub hi: f64,
}

impl Band {
    pub fn middle(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }

    pub fn half(&self) -> f64 {
        BAND_SHARE * (self.hi - self.lo)
    }

    pub fn holds(&self, pos: f64) -> bool {
        (pos - self.middle()).abs() <= self.half()
    }

    /// The refusal before any write.
    pub fn not_near(&self, pos: f64) -> String {
        format!(
            "the shaft is at {pos:.0} counts, {:+.0} from the middle {:.0} of the stops {:.0}..{:.0}, past the {:.0} % band of {:.0} counts: the stops step leaves the shaft near the middle; run the steps again",
            pos - self.middle(),
            self.middle(),
            self.lo,
            self.hi,
            100.0 * BAND_SHARE,
            self.half()
        )
    }

    /// The refusal of a shaft that drifted out after onset `n`.
    pub fn drifted(&self, pos: f64, n: usize) -> String {
        format!(
            "after onset {n} the shaft drifted to {pos:.0} counts, {:+.0} from the middle {:.0} of the stops, past the {:.0} % band of {:.0} counts: the onsets did not cancel; check that nothing loads the horn, then run the steps again",
            pos - self.middle(),
            self.middle(),
            100.0 * BAND_SHARE,
            self.half()
        )
    }
}

/// The stops step's still rule: every reading within `band` of their
/// median.
pub fn still(readings: &[f64], band: f64) -> bool {
    median(readings).is_some_and(|m| readings.iter().all(|r| (r - m).abs() <= band))
}

/// What to do once a set is judged.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum After {
    Accept,
    /// The direction check alone warned: one more set, all fitted.
    Second,
    Refuse,
}

/// The second-set rule, `sets` taken so far.
pub fn after(checks: &[Check], sets: usize) -> After {
    if checks.iter().any(|c| c.verdict == Verdict::Fail) {
        After::Refuse
    } else if sets < SETS_MAX
        && checks
            .iter()
            .any(|c| c.name == DIRECTIONS && c.verdict == Verdict::Warn)
    {
        After::Second
    } else {
        After::Accept
    }
}

fn bound_of(name: &str) -> &'static str {
    match name {
        "per-period rms <= 1 mA" => "at most 1 mA",
        "L / R inside 2 to 8 periods" => "2 to 8 PWM periods",
        DIRECTIONS => "within 10 %: a warning, and once a second set",
        "g >= 0" => "at least 0",
        "g_window_q016 x r_q12 <= 2^25" => {
            "at most 2^25, the firmware's bound, at this run's seated R"
        }
        "L and R inside the grid" => "off the grid's edge",
        _ => "the notebook's acceptance",
    }
}

/// The fit's acceptance as the step judges it: the direction check warns,
/// every other refuses.
pub fn verdicts(fit: &[term::Check]) -> Vec<Check> {
    fit.iter()
        .map(|c| Check {
            name: c.name.into(),
            value: c
                .value
                .map_or("nothing to judge".into(), |v| format!("{v:.4} {}", c.unit)),
            bound: bound_of(c.name).into(),
            verdict: match (c.pass, c.name == DIRECTIONS) {
                (true, _) => Verdict::Pass,
                (false, true) => Verdict::Warn,
                (false, false) => Verdict::Fail,
            },
        })
        .collect()
}

/// Why the gate set an onset aside, `ramp` the session's.
pub fn gate_said(o: &Onset, ramp: Option<&[u16]>, m: &Method) -> Option<String> {
    if o.ok {
        return None;
    }
    let mut why = Vec::new();
    if !o.whole {
        why.push(format!(
            "not whole: a field missing, or fewer than {} driven rows",
            m.after
        ));
    }
    if o.pre_rows == 0 {
        why.push("no still row ahead of the edge".into());
    }
    if let Some(v) = o.pre_vd_max.filter(|v| *v > m.tap_gate) {
        why.push(format!(
            "taps {v:.0} counts apart ahead of the edge, over {}: the motor was turning",
            m.tap_gate
        ));
    }
    match o.moved {
        Some(v) if v > m.pot_gate => why.push(format!(
            "the pot moved {v:.0} counts over the first millisecond, over {}",
            m.pot_gate
        )),
        None => why.push("no pot over the first millisecond".into()),
        _ => {}
    }
    if !o.same_sign {
        why.push("the duty changed sign inside the onset".into());
    }
    if ramp.is_some_and(|r| r != o.seq.as_slice()) {
        why.push(format!(
            "a ramp of {:?}, not the session's",
            &o.seq[..o.seq.len().min(4)]
        ));
    }
    Some(format!("onset {}: {}", o.seg, why.join("; ")))
}

/// The least of `n` onsets through the gate.
pub fn gated_min(n: usize) -> usize {
    (GATED_MIN * n).div_ceil(SET)
}

pub fn gate_check(onsets: &[Onset], n: usize) -> Check {
    let k = onsets.iter().filter(|o| o.ok).count();
    Check {
        name: "onsets through the gate".into(),
        value: format!("{k} of {n}"),
        bound: format!(
            "at least {GATED_MIN} of every {SET}: {} of {n}",
            gated_min(n)
        ),
        verdict: if k >= gated_min(n) {
            Verdict::Pass
        } else {
            Verdict::Fail
        },
    }
}

/// One onset as captured: its rows from the commit on.
#[derive(Clone, Debug)]
pub struct Shot {
    pub n: usize,
    pub set: usize,
    pub duty_q15: i16,
    /// The pot before it, counts, and the host's time from its commit to
    /// its zero, s.
    pub start: f64,
    pub on_s: f64,
    pub rows: Vec<TelFrame>,
}

/// The shots as the fit's segments, one capture, a segment an onset.
pub fn segments(shots: &[Shot]) -> Vec<Segment<'_>> {
    shots
        .iter()
        .map(|s| Segment {
            seg: s.n as u32,
            cmd_duty_q15: s.duty_q15,
            rows: &s.rows,
        })
        .collect()
}

/// What the judge made of a set of shots.
#[derive(Clone, Debug)]
pub struct Judged {
    pub onsets: Vec<Onset>,
    pub ramp: Option<Vec<u16>>,
    pub window: Result<Window, WindowError>,
    pub checks: Vec<Check>,
    /// The gate's reason for each onset it set aside.
    pub aside: Vec<String>,
}

/// The shots gated as one session, fitted and judged: the gate's count,
/// then the fit's acceptance with `r_q12` the winding's field.
pub fn judge(shots: &[Shot], s: &Scales, m: &Method, r_q12: u16) -> Judged {
    let failed = |e: WindowError, onsets: Vec<Onset>, ramp, mut checks: Vec<Check>, aside| {
        checks.push(Check {
            name: "the fit".into(),
            value: e.to_string(),
            bound: "a fit of the gated onsets".into(),
            verdict: Verdict::Fail,
        });
        Judged {
            onsets,
            ramp,
            window: Err(e),
            checks,
            aside,
        }
    };
    let mut ons = match term::onsets(0, &segments(shots), m) {
        Ok(o) => o,
        Err(e) => return failed(e, Vec::new(), None, Vec::new(), Vec::new()),
    };
    let ramp = term::gate(&mut ons, m);
    let mut aside: Vec<String> = ons
        .iter()
        .filter_map(|o| gate_said(o, ramp.as_deref(), m))
        .collect();
    aside.extend(
        shots
            .iter()
            .filter(|sh| !ons.iter().any(|o| o.seg as usize == sh.n))
            .map(|sh| format!("onset {}: no driven row in its burst", sh.n)),
    );
    let mut checks = vec![gate_check(&ons, shots.len())];
    match term::fit(&ons, s, m) {
        Ok(w) => {
            checks.extend(verdicts(&acceptance(&w, s, r_q12)));
            Judged {
                onsets: ons,
                ramp,
                window: Ok(w),
                checks,
                aside,
            }
        }
        Err(e) => failed(e, ons, ramp, checks, aside),
    }
}

/// The fit's scales, nominal: the sense block read off the servo, the
/// terminal offset 0, the board's edge lags and its crest read.
pub fn scales(
    sense: &rest::Sense,
    board: &Board,
    pwm_arr: u16,
    v_window_min_ticks: u16,
) -> Result<Scales, Refused> {
    let edges = board.burst_edges.ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) holds no bridge edge lags (burst_edges): the onsets cannot be read",
            board.name
        ))
    })?;
    let w = board.window_read.as_ref().ok_or_else(|| {
        Refused(format!(
            "the board's description ({}) holds no crest read (window_read): the onsets cannot be read",
            board.name
        ))
    })?;
    Ok(Scales {
        counts_per_a: sense.counts_per_a(),
        terminal_counts_per_v: sense.terminal_counts_per_v(),
        terminal_offset_counts: 0.0,
        tick_hz: sense.tick_hz,
        pwm_arr,
        sample_ticks: w.sample_ticks,
        lag_on_ticks: edges.tap_on_ticks,
        lag_off_ticks: edges.off_ticks,
        r_brake_ohm: w.r_brake_ohm,
        v_window_min_ticks,
        settle: Settle {
            start_ticks: w.settle_start_ticks,
            q15: w.settle_q15.clone(),
        },
    })
}

/// `r_q12` of a winding of `r_25c_ohm` at 25 C, by the rules.
pub fn r_q12(
    board: &Board,
    r_25c_ohm: f64,
    counts_per_a: f64,
    counts_per_v: f64,
) -> Result<u16, Refused> {
    let q = |value: f64, unit: &str| Quantity {
        value,
        lo: None,
        hi: None,
        unit: unit.into(),
        source: Source::default(),
    };
    let inputs = Inputs {
        board: board.clone(),
        measured: Measured {
            r_ohm: Some(q(r_25c_ohm, "ohm")),
            r_board_c: Some(q(R_AT_C, "C")),
            current_counts_per_a: Some(q(counts_per_a, "counts/A")),
            terminal_counts_per_v: Some(q(counts_per_v, "counts/V")),
            ..Measured::default()
        },
        nameplate: Nameplate::default(),
        policy: Default::default(),
        thermal: ThermalInputs::default(),
        tuning: Default::default(),
    };
    match build(&inputs).get("r_q12") {
        Some(Outcome::Bound(e)) => Ok(e.value as u16),
        Some(Outcome::Refused(r)) => Err(Refused(format!("r_q12 of the seated R: {}", r.why))),
        _ => Err(Refused("the rules bind no r_q12 of the seated R".into())),
    }
}

/// What the step takes from the stops step's result.
#[derive(Clone, Debug, PartialEq)]
pub struct FromStops {
    pub lo: f64,
    pub hi: f64,
    pub r_25c_ohm: f64,
}

impl FromStops {
    pub fn of(stops: &Value) -> Result<Self, Refused> {
        if stops["verdict"] != "accepted" {
            return Err(Refused(format!(
                "the window step runs on an accepted stops step; the stops step was {}",
                stops["verdict"].as_str().unwrap_or("not run")
            )));
        }
        let n = |v: &Value, what: &str| {
            v.as_f64()
                .filter(|x| x.is_finite())
                .ok_or_else(|| Refused(format!("the stops result holds no {what}")))
        };
        let (lo, hi) = (
            n(&stops["stops"]["low"]["median_counts"], "low stop")?,
            n(&stops["stops"]["high"]["median_counts"], "high stop")?,
        );
        if hi <= lo {
            return Err(Refused(format!(
                "the stops result holds a low stop {lo:.0} at or over its high stop {hi:.0}"
            )));
        }
        Ok(FromStops {
            lo,
            hi,
            r_25c_ohm: n(
                &stops["estimates"]["r_seated_25c_ohm"]["value"],
                "seated R at 25 C",
            )?,
        })
    }
}

/// What the step reads besides the servo.
pub struct WindowInputs<'a> {
    pub nameplate: &'a Nameplate,
    pub board: &'a Board,
    pub thermal: Option<&'a ThermalInputs>,
    /// The same run's `results/rest.json`, `probe.json` and `stops.json`.
    pub rest: &'a Value,
    pub probe: &'a Value,
    pub stops: &'a Value,
}

#[derive(Clone, Debug)]
pub struct Windowed {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Windowed {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    pos: f64,
    rail_v: f64,
    stored: bool,
    probe: FromProbe,
    stops: FromStops,
    band: Band,
    duties: [i16; 3],
    rows: u16,
    tick_hz: f64,
    stream: StreamCfg,
    dm: Deadmen,
    still_counts: f64,
    stated_v: f64,
    rated_cps: f64,
    env: Envelope,
    scales: Scales,
    r_q12: u16,
}

fn num(state: &State, d: &Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

/// An onset as the envelope prices it: a free drive at its duty for its
/// rows and the dead-man, at the nameplate's speed for its volts.
fn onset_rung(b: &Before, name: String, q15: i16, start: f64) -> Rung {
    let volts = q15.unsigned_abs() as f64 / Q15 * b.rail_v;
    Rung {
        name,
        duty: q15 as f64 / Q15,
        time_s: priced_s(b.rows, b.tick_hz),
        permit: false,
        gap_s: COAST_S,
        start,
        speed_cps: volts / b.stated_v * b.rated_cps,
        stop_counts: 0.0,
        kind: Kind::Drive,
    }
}

/// The worst case of `p` at least [`INSET_COUNTS`] inside both stops.
pub fn inset(r: &Rung, p: &Price, lo: f64, hi: f64) -> Result<(), String> {
    let (a, z) = (lo + INSET_COUNTS, hi - INSET_COUNTS);
    let ends = std::iter::once(p.free_end).chain(p.wrong_end);
    match ends.into_iter().find(|e| *e < a || *e > z) {
        None => Ok(()),
        Some(e) => Err(format!(
            "{} is refused before any write: from {:.0} the worst case runs {:.0} counts to {e:.0}, less than {INSET_COUNTS} counts inside the stops {lo:.0}..{hi:.0}",
            r.name, r.start, p.travel
        )),
    }
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &WindowInputs<'_>) -> Result<Before, Refused> {
    let np = i.nameplate;
    np.check_user().map_err(Refused)?;
    let thermal = i.thermal.ok_or_else(|| {
        Refused("no thermal values entered (--thermal): the heat of a drive cannot be priced, so nothing drives".into())
    })?;
    let line = tel::line(rig.c, rig.id).await?;
    line.admit(Layout::Raw)?;
    let rest_in = FromRest::of(i.rest)?;
    let probe = FromProbe::of(i.probe)?;
    let stops = FromStops::of(i.stops)?;
    let state = state::read(rig.c, rig.id, rig.d).await?;
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this step starts only with torque off, the shaft at rest".into(),
        ));
    }
    let faults = rig
        .c
        .read(rig.id, field(rig.d, "fault_flags")?.addr, 1)
        .await?
        .first()
        .copied()
        .unwrap_or(0);
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
            "the servo's stored drive_polarity is {stored}, and the probe ran on {}: something changed it since; run the steps again",
            probe.stored
        )));
    }
    let speed = np
        .speed_row()
        .ok_or_else(|| Refused("the nameplate has no usable no_load".into()))?;
    let travel_deg = np
        .travel_deg
        .ok_or_else(|| Refused("the nameplate has no usable travel_deg".into()))?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let pos = rest::pos_now(rig.c, rig.id, rig.d).await?;
    let band = Band {
        lo: stops.lo,
        hi: stops.hi,
    };
    if !band.holds(pos) {
        return Err(Refused(band.not_near(pos)));
    }
    let servo = meta::servo(&state, rig.d, &line);
    let rail_v = rest::rail_now(rig.c, rig.id, rig.d, &sense).await?;
    let pwm_arr = num(&state, rig.d, "pwm_period_ticks")? as u16;
    let v_floor = num(&state, rig.d, "v_window_min_ticks")? as u16;
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    let duties = duties(pwm_arr, stream.i_window_min_ticks)?;
    let scales = scales(&sense, i.board, pwm_arr, v_floor)?;
    let r_q12 = r_q12(
        i.board,
        stops.r_25c_ohm,
        sense.counts_per_a(),
        sense.terminal_counts_per_v(),
    )?;
    let ntc_raw = rig.c.read(rig.id, field(rig.d, "ntc_raw")?.addr, 2).await?;
    let mut facts = Facts::new(np, rail_v);
    facts.ntc_c = Some(
        sense
            .ntc
            .c_of_raw(u16::from_le_bytes([ntc_raw[0], ntc_raw[1]]) as f64),
    );
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
    let tick_hz = sense.tick_hz;
    let b = Before {
        line,
        servo,
        pos,
        rail_v,
        stored,
        probe,
        stops,
        band,
        duties,
        rows: ((ONSET_S * tick_hz).round() + FAST_TICKS_PER_MEDIUM) as u16,
        tick_hz,
        stream,
        dm: Deadmen {
            step_ticks: ticks(rest::DEADMAN_S)?,
            drive_ticks: ticks(crate::envelope::DRIVE_DEADMAN_S)?,
        },
        still_counts: seek::still_band(rest_in.pot_sd),
        stated_v: speed.at_v,
        rated_cps: speed.per_deg / speed.seconds * ADC_COUNTS / travel_deg,
        env: Envelope::new(facts, heat),
        scales,
        r_q12,
    };
    for q in b.duties {
        for s in [1i16, -1] {
            let r = onset_rung(
                &b,
                format!("an onset at {:+.1}%", 100.0 * (s * q) as f64 / Q15),
                s * q,
                pos,
            );
            let p = b.env.price(&r)?;
            inset(&r, &p, b.stops.lo, b.stops.hi).map_err(Refused)?;
        }
    }
    Ok(b)
}

/// [`COAST_S`] at zero duty with torque on, the pot and the faults polled
/// every [`seek::POLL_S`]; the shaft still over the last
/// [`seek::WINDOW_S`] by the stops step's rule, waited for up to
/// [`STILL_MAX_S`] more, else refused with the readings. The median of
/// the still readings.
async fn coast<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, next: usize) -> Result<f64, Halt> {
    let n = (COAST_S / seek::POLL_S).round() as usize;
    let more = (STILL_MAX_S / seek::POLL_S).round() as usize;
    let k = ((seek::WINDOW_S / seek::POLL_S).ceil() as usize).min(n);
    let mut seen = Vec::with_capacity(n + more);
    for i in 0..n + more {
        rig.c.pause(Duration::from_secs_f64(seek::POLL_S)).await;
        seen.push(stops::poll(rig).await?);
        if i + 1 >= n && still(&seen[seen.len() - k..], b.still_counts) {
            return Ok(median(&seen[seen.len() - k..]).unwrap_or(f64::NAN));
        }
    }
    let tail = &seen[seen.len() - k..];
    let hi = tail.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let lo = tail.iter().copied().fold(f64::INFINITY, f64::min);
    Err(Halt::Refused(format!(
        "before onset {next} the shaft was not still after {:.1} s at zero duty: the pot spans {:.0} counts over the last {} s, past the still band of {:.1} (readings {}); check that nothing touches the horn, then run the steps again",
        COAST_S + STILL_MAX_S,
        hi - lo,
        seek::WINDOW_S,
        b.still_counts,
        tail.iter()
            .map(|v| format!("{v:.0}"))
            .collect::<Vec<_>>()
            .join(" ")
    )))
}

/// One set's onsets on the servo, each after its coast, found still and
/// in the band, priced and inset, driven by one commit; one whose burst
/// lost a row is driven once more, twice aborts.
async fn take_set<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    set: usize,
    shots: &mut Vec<Shot>,
    taken: &mut Vec<Taken>,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let start = read_u16(rig, "pos").await? as f64;
    let first = first_sign(start, b.band.middle());
    for p in plan(b.duties, first, set, shots.len()) {
        let mut attempt = 0;
        loop {
            attempt += 1;
            interrupted(rig.stop)?;
            let pos = coast(rig, b, p.n).await?;
            if !b.band.holds(pos) {
                return Err(Halt::Refused(b.band.drifted(pos, p.n - 1)));
            }
            let r = onset_rung(b, format!("onset {}", p.n), p.duty_q15, pos);
            let price = b.env.admit(&r)?;
            inset(&r, &price, b.stops.lo, b.stops.hi).map_err(Halt::Refused)?;
            let plan = json!({"rung": format!("onset {}", p.n), "set": set + 1, "row": Layout::Raw.name(),
                              "rows": b.rows, "duty_q15": p.duty_q15, "attempt": attempt,
                              "time_s": r.time_s, "start_counts": pos, "priced_counts": price.travel});
            let d = drive::drive(
                rig,
                &b.stream,
                &b.dm,
                &Drive {
                    layout: Layout::Raw,
                    duty_q15: p.duty_q15,
                    rows: b.rows,
                    end: End::Zero,
                    plan: &plan,
                },
                taken,
                &b.servo,
            )
            .await?;
            if d.on_s > r.time_s {
                return Err(Halt::Error(format!(
                    "onset {} ran {:.3} s from its commit to the zero write, over the {:.3} s it was priced at",
                    p.n, d.on_s, r.time_s
                )));
            }
            let lost = d.lost();
            if lost.is_empty() {
                let pre = d
                    .drive
                    .rows
                    .iter()
                    .position(|r| r.duty_counts.is_some_and(|x| x != 0));
                lines.push(format!(
                    "onset {}: {:+.1}% from {pos:.0}, {} rest rows, on {:.3} s",
                    p.n,
                    100.0 * p.duty_q15 as f64 / Q15,
                    pre.map_or("no edge, all".into(), |k| k.to_string()),
                    d.on_s
                ));
                shots.push(Shot {
                    n: p.n,
                    set,
                    duty_q15: p.duty_q15,
                    start: pos,
                    on_s: d.on_s,
                    rows: d.drive.rows,
                });
                break;
            }
            lines.push(format!(
                "onset {}, attempt {attempt}: lost rows ({}), no evidence",
                p.n,
                lost.join(", ")
            ));
            if attempt == 2 {
                return Err(Halt::Error(format!(
                    "onset {}: lost rows twice ({}): the line is not clean enough; check the cable and its contacts",
                    p.n,
                    lost.join(", ")
                )));
            }
        }
    }
    Ok(())
}

/// What the body did, as far as it got.
#[derive(Default)]
struct Found {
    shots: Vec<Shot>,
    sets: usize,
    corrected: bool,
    stored_g: Option<u16>,
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &mut Before,
    ledger: &mut Ledger,
    found: &mut Found,
    taken: &mut Vec<Taken>,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    ledger.touched = true;
    write(rig, "torque_enable", &[0]).await?;
    let open_loop = mode_value(rig.d, "OpenLoop")?;
    for name in ["mode", "current_limit_counts", "g_window_q016"] {
        ledger.keep(rig.c, rig.id, rig.d, name).await?;
    }
    found.stored_g = ledger
        .fields
        .iter()
        .find(|(n, _, _)| n == "g_window_q016")
        .and_then(|(_, _, v)| Some(u16::from_le_bytes([*v.first()?, *v.get(1)?])));
    if b.probe.found != b.stored {
        stops::correct(rig, ledger, &b.probe, lines).await?;
        found.corrected = true;
    }
    write(rig, "g_window_q016", &[0, 0]).await?;
    if read_u16(rig, "g_window_q016").await? != 0 {
        return Err(Halt::Error(
            "g_window_q016 0 did not read back after the write".into(),
        ));
    }
    write(rig, "current_limit_counts", &[0, 0]).await?;
    write(rig, "mode", &[open_loop]).await?;
    write(rig, "goal_duty", &[0, 0]).await?;
    write(rig, "bus_deadman_ticks", &b.dm.step_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    lines.push(format!(
        "volatile: g_window_q016 0 (stored {}), the peak cap unset, OpenLoop; dead-man {} ticks ({} s), then torque on at zero duty",
        found
            .stored_g
            .map_or("not read".into(), |g| g.to_string()),
        b.dm.step_ticks,
        rest::DEADMAN_S
    ));
    let v = |q: i16| q as f64 / Q15 * b.rail_v;
    lines.push(format!(
        "onsets: {:.1}, {:.1} and {:.1}% ({:.3}, {:.3}, {:.3} V), both signs, {EACH} each; {} rows each ({ONSET_S} s and a medium period) under a dead-man of {} ticks, after {COAST_S} s at zero duty",
        100.0 * b.duties[0] as f64 / Q15,
        100.0 * b.duties[1] as f64 / Q15,
        100.0 * b.duties[2] as f64 / Q15,
        v(b.duties[0]),
        v(b.duties[1]),
        v(b.duties[2]),
        b.rows,
        b.dm.drive_ticks
    ));
    let quick = Method {
        draws: 0,
        ..Method::NOTEBOOK
    };
    for set in 0..SETS_MAX {
        if set > 0 {
            write(rig, "torque_enable", &[1]).await?;
        }
        take_set(rig, b, set, &mut found.shots, taken, lines).await?;
        found.sets = set + 1;
        // the fit is host time with the bus silent: torque off for it, so
        // the dead-man has nothing to cut
        write(rig, "torque_enable", &[0]).await?;
        let j = judge(&found.shots, &b.scales, &quick, b.r_q12);
        match after(&j.checks, found.sets) {
            After::Second => lines.push(format!(
                "set {}: the two directions differ by more than 10 %: a second set of {SET}, all fitted",
                set + 1
            )),
            _ => break,
        }
    }
    Ok(())
}

/// What to do when a check refuses the step.
pub fn advice(name: &str) -> &'static str {
    match name {
        "onsets through the gate" => {
            "the onsets did not start from rest: check that nothing touches the horn and that the cable is seated, then run the steps again"
        }
        "g >= 0" | "g_window_q016 x r_q12 <= 2^25" => {
            "a term this board cannot hold: check the motor leads and the board"
        }
        "torque off at the end" => "power the servo off and on",
        _ => {
            "the onsets do not read as an empty winding's: run the steps again; if it is refused again, check the motor leads"
        }
    }
}

/// Step 4 on the servo: the onsets, then the way out, then the fit.
/// Refused with nothing written when it cannot start; once it has
/// written, it ends through the guard whatever happens, and keeps its
/// captures and result.
pub async fn window<P: Pipe>(
    rig: &mut Rig<'_, P>,
    i: &WindowInputs<'_>,
) -> Result<Windowed, Refused> {
    let mut b = before(rig, i).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.0} counts, {:+.0} from the middle of the stops (band {:.0}), rail {:.2} V, before anything is written",
            b.pos,
            b.pos - b.band.middle(),
            b.band.half(),
            b.rail_v
        ),
    ];
    let mut ledger = Ledger::default();
    let mut found = Found::default();
    let mut taken = Vec::new();
    let ran = body(rig, &mut b, &mut ledger, &mut found, &mut taken, &mut lines).await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after_pos = rest::pos_now(rig.c, rig.id, rig.d).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines);
    let judged = ran
        .is_ok()
        .then(|| judge(&found.shots, &b.scales, &Method::NOTEBOOK, b.r_q12));
    let mut checks = judged.as_ref().map_or(Vec::new(), |j| j.checks.clone());
    if let Some(j) = &judged {
        lines.extend(j.aside.iter().map(|a| format!("gate: {a}")));
        if let Ok(w) = &j.window {
            lines.extend(said(w, &b.scales, j));
        }
    }
    lines.push(format!(
        "end position {} counts ({:.0} before)",
        after_pos.map_or("not read".into(), |p| format!("{p:.0}")),
        b.pos
    ));
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
        (Ok(()), Err(h)) => (Way::Error, Some(format!("files: {h}"))),
        (Ok(()), Ok(())) if !failed.is_empty() => {
            let mut what: Vec<&str> = failed.iter().map(|c| advice(&c.name)).collect();
            what.dedup();
            let gate = judged
                .as_ref()
                .filter(|_| failed.iter().any(|c| c.name == "onsets through the gate"))
                .map_or(String::new(), |j| {
                    format!(" (the gate: {})", j.aside.join("; "))
                });
            (
                Way::Refused,
                Some(format!(
                    "refused by {}{gate}: {}",
                    failed
                        .iter()
                        .map(|c| format!("{} {}", c.name, c.value))
                        .collect::<Vec<_>>()
                        .join(", "),
                    what.join("; ")
                )),
            )
        }
        _ => (Way::Done, None),
    };
    let result = result_json(&b, &found, judged.as_ref(), &checks, way, &exit, after_pos);
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Windowed {
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

/// The fit in a few short lines.
fn said(w: &Window, s: &Scales, j: &Judged) -> Vec<String> {
    let ms = |g: f64| g * 1e3;
    let mut out = vec![
        format!(
            "g {:.3} mS (sd {}), over the mean {:.3} mS: g_window_q016 {}",
            ms(w.g_s),
            w.g_sd_s.map_or("none".into(), |x| format!("{:.3}", ms(x))),
            ms(w.g_over_mean_s),
            field_q016(w.g_over_mean_s, s)
        ),
        format!(
            "nuisance values of the fit: L {:.3} mH, R {:.2} ohm, L / R {:.2} periods; rms {:.3} mA",
            w.l_h * 1e3,
            w.r_ohm,
            w.tau_periods,
            w.rms_a * 1e3
        ),
    ];
    for (name, d) in [("forward", w.forward), ("reverse", w.reverse)] {
        if let Some(d) = d {
            out.push(format!(
                "{name}: {} onsets, g {:.3} mS free, {:.3} mS at the pooled L and R",
                d.n,
                ms(d.g_free_s),
                ms(d.g_pinned_s)
            ));
        }
    }
    out.push(format!(
        "gated {} of {}",
        j.onsets.iter().filter(|o| o.ok).count(),
        j.onsets.len()
    ));
    out
}

fn dir_json(d: Option<term::Direction>) -> Value {
    d.map_or(
        Value::Null,
        |d| json!({"n": d.n, "g_free_ms": d.g_free_s * 1e3, "g_pinned_ms": d.g_pinned_s * 1e3}),
    )
}

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    judged: Option<&Judged>,
    checks: &[Check],
    way: Way,
    exit: &guard::Exit,
    after: Option<f64>,
) -> Value {
    let s = &b.scales;
    let w = judged.and_then(|j| j.window.as_ref().ok());
    let mut estimates = Map::new();
    if let Some(w) = w {
        estimates.insert(
            "g_window_ms".into(),
            json!({"value": w.g_over_mean_s * 1e3, "lo": null, "hi": null, "unit": "mS",
                   "source": "open-loop onsets from rest, the window-term notebook's fit on the nominal scales and the board's edge lags: g plus the crest sample's lateness past the window's middle, the term over the period's mean"}),
        );
    }
    let fit = w.map_or(Value::Null, |w| {
        json!({
            "g_ms": w.g_s * 1e3,
            "g_sd_ms": w.g_sd_s.map(|x| x * 1e3),
            "lateness_ms": w.lateness_s * 1e3,
            "g_over_mean_ms": w.g_over_mean_s * 1e3,
            "g_window_q016": field_q016(w.g_over_mean_s, s),
            "nuisance": {"l_mh": w.l_h * 1e3, "r_ohm": w.r_ohm, "tau_periods": w.tau_periods,
                         "note": "free on a grid inside the fit, never taken from it"},
            "rms_ma": w.rms_a * 1e3,
            "on_grid_edge": w.on_grid_edge,
            "v_on_v": w.v_on_v,
            "h_ticks": w.h,
            "resid_ma": w.resid_a.iter().map(|x| x * 1e3).collect::<Vec<_>>(),
            "split": {"forward": dir_json(w.forward), "reverse": dir_json(w.reverse)},
            "draws": w.draws,
            "profiles": w.profiles.iter().map(|p| json!({"onset": p.seg, "sgn": p.sgn, "zero_counts": p.zero_counts,
                "v_on_v": p.v_on_v, "g_pinned_ms": p.g_pinned_s * 1e3})).collect::<Vec<_>>(),
        })
    });
    let onsets: Vec<Value> = found
        .shots
        .iter()
        .map(|sh| {
            let o = judged.and_then(|j| j.onsets.iter().find(|o| o.seg as usize == sh.n));
            json!({"n": sh.n, "set": sh.set + 1, "duty_q15": sh.duty_q15, "start_counts": sh.start, "on_s": sh.on_s,
                   "rows": sh.rows.len(), "pre_rows": o.map(|o| o.pre_rows), "pre_vd_max": o.and_then(|o| o.pre_vd_max),
                   "moved": o.and_then(|o| o.moved), "same_sign": o.map(|o| o.same_sign), "gated": o.map(|o| o.ok)})
        })
        .collect();
    let m = Method::NOTEBOOK;
    json!({
        "step": STEP,
        "method": METHOD,
        "revision": REVISION,
        "image": b.line.image().name(),
        "capability_flags": b.line.caps,
        "verdict": way.name(),
        "estimates": estimates,
        "fit": fit,
        "gate": judged.map_or(Value::Null, |j| json!({
            "gated": j.onsets.iter().filter(|o| o.ok).count(), "of": found.shots.len(),
            "least": gated_min(found.shots.len()), "ramp_ticks": j.ramp, "aside": j.aside,
        })),
        "sets": found.sets,
        "onsets": onsets,
        "method_constants": {"after_rows": m.after, "tap_gate_counts": m.tap_gate, "pot_gate_counts": m.pot_gate,
                             "read_periods": m.read_periods, "v_row_max": m.v_row_max, "ramp_rows": m.ramp_rows,
                             "draws": m.draws, "seed": m.seed},
        "scales": {"counts_per_a": s.counts_per_a, "terminal_counts_per_v": s.terminal_counts_per_v,
                   "terminal_offset_counts": s.terminal_offset_counts, "tick_hz": s.tick_hz, "pwm_arr": s.pwm_arr,
                   "sample_ticks": s.sample_ticks, "lag_on_ticks": s.lag_on_ticks, "lag_off_ticks": s.lag_off_ticks,
                   "r_brake_ohm": s.r_brake_ohm, "v_window_min_ticks": s.v_window_min_ticks,
                   "settle_start_ticks": s.settle.start_ticks, "settle_q15": s.settle.q15,
                   "source": "the nominal sense block read off the servo, the board's burst_edges tap-on and off lags and its window_read; no terminal offset"},
        "fence": {"r_q12": b.r_q12, "r_25c_ohm": b.stops.r_25c_ohm,
                  "source": "this run's seated R at 25 C (results/stops.json) through the rules' r_q12 on the nominal scales"},
        "drives": {"duties_q15": b.duties, "duties_v": b.duties.map(|q| q as f64 / Q15 * b.rail_v),
                   "rows": b.rows, "onset_s": ONSET_S, "coast_s": COAST_S, "drive_deadman_ticks": b.dm.drive_ticks,
                   "step_deadman_ticks": b.dm.step_ticks, "still_counts": b.still_counts,
                   "r_price_ohm": b.probe.r_price_ohm, "inset_counts": INSET_COUNTS},
        "band": {"middle_counts": b.band.middle(), "half_counts": b.band.half(), "share": BAND_SHARE,
                 "stops_counts": [b.stops.lo, b.stops.hi]},
        "g_window_q016": {"stored": found.stored_g, "driven": 0},
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
    use std::rc::Rc;
    use std::sync::atomic::{AtomicBool, Ordering};

    use futures_lite::future::block_on;
    use osc_client::Client;
    use osc_servo_core::kernel::phases::loops::faults::BIT_DEADMAN;

    use super::*;
    use crate::sink::{MemSink, Sink};
    use crate::steps::polarity::STALLED;
    use crate::steps::testkit::{self, ID, Log, OnSent, Sent, Spy, addr, descriptor};

    #[test]
    fn the_duties_are_the_notebooks_unless_the_floor_raises_them() {
        assert_eq!(duties(1200, 64).expect("duties"), NOTEBOOK_Q15);
        // 300 ticks at 25 %: twice a floor of 150 still passes
        assert_eq!(duties(1200, 150).expect("duties"), NOTEBOOK_Q15);
        let raised = duties(1200, 152).expect("raised");
        assert!(drive_ticks(raised[0], 1200) >= 304 && drive_ticks(raised[0] - 1, 1200) < 304);
        assert_eq!(raised[1..], NOTEBOOK_Q15[1..]);
        let why = duties(1200, 170).expect_err("collide").0;
        assert!(why.contains("fewer than three duties"), "{why}");
        assert!(duties(1200, 1000).is_err(), "no duty reaches it");
    }

    #[test]
    fn a_set_alternates_sign_four_of_each_duty_and_sign() {
        let p = plan(NOTEBOOK_Q15, -1, 1, 24);
        assert_eq!(p.len(), SET);
        assert_eq!(
            p.iter().map(|x| x.n).collect::<Vec<_>>(),
            (25..=48).collect::<Vec<_>>()
        );
        assert!(p.iter().all(|x| x.set == 1));
        assert_eq!(p[0].duty_q15, -8191, "the first sign given");
        for w in p.windows(2) {
            assert_eq!(w[0].duty_q15.signum(), -w[1].duty_q15.signum(), "{w:?}");
        }
        for q in NOTEBOOK_Q15 {
            for s in [1, -1] {
                assert_eq!(p.iter().filter(|x| x.duty_q15 == s * q).count(), EACH);
            }
        }
        assert_eq!(
            (first_sign(2100.0, 2029.0), first_sign(2000.0, 2029.0)),
            (-1, 1)
        );
    }

    #[test]
    fn the_band_refuses_a_start_off_the_middle_and_a_drift() {
        let b = Band {
            lo: 432.0,
            hi: 3626.0,
        };
        assert_eq!((b.middle(), b.half()), (2029.0, 0.2 * 3194.0));
        assert!(b.holds(2029.0 + 638.0) && b.holds(2029.0 - 638.0));
        assert!(!b.holds(2029.0 + 640.0));
        let why = b.not_near(2800.0);
        assert!(
            why.contains("+771 from the middle 2029") && why.contains("run the steps again"),
            "{why}"
        );
        let why = b.drifted(1300.0, 7);
        assert!(
            why.starts_with("after onset 7") && why.contains("-729 from the middle"),
            "{why}"
        );
    }

    #[test]
    fn still_is_every_reading_inside_the_band_of_their_median() {
        assert!(still(&[2000.0, 2001.0, 1999.0, 2002.0], 3.0));
        assert!(!still(&[2000.0, 2001.0, 2009.0], 3.0));
        assert!(!still(&[], 3.0));
    }

    fn fit_check(name: &'static str, pass: bool, value: Option<f64>) -> term::Check {
        term::Check {
            name,
            value,
            unit: "u",
            pass,
        }
    }

    #[test]
    fn the_direction_check_warns_and_every_other_refuses() {
        let v = verdicts(&[
            fit_check("per-period rms <= 1 mA", true, Some(0.3)),
            fit_check(DIRECTIONS, false, Some(14.0)),
            fit_check("g >= 0", false, Some(-1.0)),
            fit_check("L and R inside the grid", false, None),
        ]);
        let got: Vec<Verdict> = v.iter().map(|c| c.verdict).collect();
        assert_eq!(
            got,
            [Verdict::Pass, Verdict::Warn, Verdict::Fail, Verdict::Fail]
        );
        assert_eq!(v[3].value, "nothing to judge");
        assert_eq!(
            verdicts(&[fit_check(DIRECTIONS, false, None)])[0].verdict,
            Verdict::Warn
        );
    }

    fn check(name: &str, verdict: Verdict) -> Check {
        Check {
            name: name.into(),
            value: String::new(),
            bound: String::new(),
            verdict,
        }
    }

    #[test]
    fn a_second_set_only_on_the_direction_warning_and_once() {
        let pass = check("g >= 0", Verdict::Pass);
        let warn = check(DIRECTIONS, Verdict::Warn);
        let fail = check("per-period rms <= 1 mA", Verdict::Fail);
        assert_eq!(
            after(&[pass.clone(), check(DIRECTIONS, Verdict::Pass)], 1),
            After::Accept
        );
        assert_eq!(after(&[pass.clone(), warn.clone()], 1), After::Second);
        assert_eq!(
            after(&[pass.clone(), warn.clone()], 2),
            After::Accept,
            "once"
        );
        assert_eq!(after(&[fail.clone(), warn.clone()], 1), After::Refuse);
        assert_eq!(after(&[fail], 2), After::Refuse);
        assert_eq!((gated_min(24), gated_min(48)), (20, 40));
    }

    #[test]
    fn an_onset_whose_worst_case_comes_within_300_counts_of_a_stop_is_refused() {
        let r = Rung::still("onset 1", 0.1, 3000.0);
        let p = |end: f64| Price {
            volts: 1.4,
            current_a: 0.3,
            travel: (end - 3000.0).abs(),
            free_end: end,
            wrong_end: None,
            rise_c: 0.0,
        };
        assert!(inset(&r, &p(3325.0), 432.0, 3626.0).is_ok());
        let why = inset(&r, &p(3327.0), 432.0, 3626.0).expect_err("inside 300");
        assert!(
            why.contains("onset 1") && why.contains("300 counts inside"),
            "{why}"
        );
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

    fn probe_json(stored: bool, found: bool) -> Value {
        json!({
            "verdict": "accepted",
            "drive_polarity": {"stored": stored, "found": found, "stored_right": stored == found},
            "part_b": {
                "ceiling_v": 0.945,
                "r_price_ohm": 3.78,
                "pulses": [{"repeat": true, "unwedge": false, "volts": 0.48}],
            },
        })
    }

    fn stops_json(verdict: &str) -> Value {
        json!({
            "verdict": verdict,
            "stops": {"low": {"median_counts": 432.0}, "high": {"median_counts": 3626.0}},
            "estimates": {"r_seated_25c_ohm": {"value": 4.2}},
        })
    }

    struct Ran {
        out: Result<Windowed, Refused>,
        log: Log,
        c: Client<Spy>,
        sink: MemSink,
    }

    /// The step on the simulated servo resting at `pos`, which runs no
    /// kernel: the rows never carry a duty, so no onset passes the gate.
    /// `g_window_q016` is 1178 before it.
    fn run(pos: u16, stops: &str, found_flips: bool, on_sent: OnSent, stop: &AtomicBool) -> Ran {
        let d = descriptor();
        let (mut c, log) = testkit::servo(pos, on_sent);
        block_on(c.write(ID, addr(&d, "g_window_q016"), &1178u16.to_le_bytes())).expect("g");
        log.borrow_mut().clear();
        let stored = block_on(c.read(ID, addr(&d, "drive_polarity"), 1)).expect("read")[0] != 0;
        let (np, board, thermal) = (testkit::nameplate(), testkit::board(), testkit::thermal());
        let (rest, probe, stops) = (
            rest_json(),
            probe_json(stored, stored != found_flips),
            stops_json(stops),
        );
        let clock = testkit::clock();
        let mut sink = MemSink::default();
        let out = {
            let mut rig = Rig {
                c: &mut c,
                id: ID,
                d: &d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            block_on(window(
                &mut rig,
                &WindowInputs {
                    nameplate: &np,
                    board: &board,
                    thermal: Some(&thermal),
                    rest: &rest,
                    probe: &probe,
                    stops: &stops,
                },
            ))
        };
        Ran { out, log, c, sink }
    }

    fn plain(d: &Descriptor, name: &str, data: &[u8]) -> Sent {
        Sent::Write {
            addr: addr(d, name),
            data: data.to_vec(),
            hold: false,
        }
    }

    fn torque_off(c: &mut Client<Spy>) -> bool {
        let d = descriptor();
        block_on(c.read(ID, addr(&d, "torque_enable"), 1)).expect("read") == [0]
            && block_on(c.read(ID, addr(&d, "goal_duty"), 2)).expect("read") == [0, 0]
    }

    /// The writes in order: g_window 0 before torque on, and each onset's
    /// drive dead-man, the duty and the arm held under one COMMIT, the zero
    /// and the step's dead-man; no duty outside the plan; the stored term
    /// put back and torque off at the end.
    #[test]
    fn an_onset_writes_in_order_inside_the_plan_and_the_way_out_puts_the_term_back() {
        let mut r = run(
            2041,
            "accepted",
            false,
            testkit::quiet(),
            &AtomicBool::new(false),
        );
        let w = r.out.as_ref().expect("starts");
        let e = &w.ended;
        assert_eq!(e.way, Way::Refused, "{:?}", e.why);
        assert!(
            e.why
                .as_deref()
                .unwrap_or("")
                .contains("onsets through the gate 0 of 24"),
            "{:?}",
            e.why
        );
        let d = descriptor();
        let log = r.log.borrow().clone();
        let at = |s: &Sent| log.iter().position(|x| x == s).expect("sent");
        let g0 = at(&plain(&d, "g_window_q016", &[0, 0]));
        let on = at(&plain(&d, "torque_enable", &[1]));
        assert!(g0 < on);
        let short = plain(&d, "bus_deadman_ticks", &1000u16.to_le_bytes());
        let first = at(&short);
        assert!(on < first);
        // 2041 is over the middle 2029: the first onset drives toward it
        let mut arm = Layout::Raw.chunks().to_le_bytes().to_vec();
        arm.extend(1010u16.to_le_bytes());
        assert_eq!(
            log[first..first + 6],
            [
                short.clone(),
                Sent::Write {
                    addr: addr(&d, "goal_duty"),
                    data: (-8191i16).to_le_bytes().to_vec(),
                    hold: true
                },
                Sent::Write {
                    addr: addr(&d, "chunk0_addr"),
                    data: arm,
                    hold: true
                },
                Sent::Commit,
                plain(&d, "goal_duty", &[0, 0]),
                plain(&d, "bus_deadman_ticks", &10_000u16.to_le_bytes()),
            ]
        );
        assert_eq!(log.iter().filter(|s| **s == Sent::Commit).count(), SET);
        let planned: Vec<i16> = NOTEBOOK_Q15.iter().flat_map(|&q| [q, -q]).collect();
        for s in &log {
            if let Sent::Write { addr: a, data, .. } = s
                && *a == addr(&d, "goal_duty")
            {
                let q = i16::from_le_bytes([data[0], data[1]]);
                assert!(q == 0 || planned.contains(&q), "{q}");
            }
        }
        assert!(
            e.exit
                .lines
                .iter()
                .any(|l| l == "g_window_q016 put back, read back"),
            "{}",
            e.exit
        );
        assert_eq!(
            block_on(r.c.read(ID, addr(&d, "g_window_q016"), 2)).expect("read"),
            1178u16.to_le_bytes()
        );
        assert!(e.exit.torque_off && torque_off(&mut r.c), "{}", e.exit);
        assert!(r.sink.read(RESULT).is_ok(), "the result is kept");
        let res = w.result.as_ref().expect("result");
        assert_eq!(res["g_window_q016"]["stored"], 1178);
        assert_eq!(res["onsets"].as_array().map(Vec::len), Some(SET));
    }

    /// A latched dead-man refuses with the stall sentence, an interrupt
    /// stops between onsets; both end torque off through the guard.
    #[test]
    fn a_latched_dead_man_or_an_interrupt_ends_torque_off() {
        let latch: OnSent = Box::new(|s, pipe| {
            if *s == Sent::Commit {
                pipe.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = BIT_DEADMAN);
            }
        });
        let mut r = run(2041, "accepted", false, latch, &AtomicBool::new(false));
        let e = r.out.as_ref().expect("starts").ended.clone();
        assert_eq!(e.way, Way::Refused);
        assert_eq!(e.why.as_deref(), Some(STALLED));
        assert!(e.exit.torque_off && torque_off(&mut r.c), "{}", e.exit);

        let stop = Rc::new(AtomicBool::new(false));
        let set = stop.clone();
        let hook: OnSent = Box::new(move |s, _| {
            if *s == Sent::Commit {
                set.store(true, Ordering::SeqCst);
            }
        });
        let mut r = run(2041, "accepted", false, hook, &stop);
        let e = r.out.as_ref().expect("starts").ended.clone();
        assert_eq!(e.way, Way::Interrupted, "{:?}", e.why);
        assert_eq!(
            r.log
                .borrow()
                .iter()
                .filter(|s| **s == Sent::Commit)
                .count(),
            1
        );
        assert!(e.exit.torque_off && torque_off(&mut r.c), "{}", e.exit);
    }

    /// A shaft off the middle, or stops not accepted: refused with nothing
    /// written but the position table's page select the state read pages
    /// with.
    #[test]
    fn refusals_before_any_write() {
        for (pos, stops, says) in [
            (2800, "accepted", "+771 from the middle"),
            (2041, "refused", "an accepted stops step"),
        ] {
            let r = run(pos, stops, false, testkit::quiet(), &AtomicBool::new(false));
            let why = r.out.expect_err("refused").0;
            assert!(why.contains(says), "{why}");
            let page = addr(&descriptor(), "pos_lut_page");
            assert!(
                r.log
                    .borrow()
                    .iter()
                    .all(|s| matches!(s, Sent::Write { addr, .. } if *addr == page)),
                "{:?}",
                r.log.borrow()
            );
        }
    }

    /// The probe found the stored sign wrong: driven volatile, put back.
    #[test]
    fn a_sign_the_probe_found_wrong_is_driven_volatile_and_put_back() {
        let r = run(
            2041,
            "accepted",
            true,
            testkit::quiet(),
            &AtomicBool::new(false),
        );
        let e = &r.out.as_ref().expect("starts").ended;
        assert!(
            e.exit
                .lines
                .iter()
                .any(|l| l == "drive_polarity put back, read back"),
            "{}",
            e.exit
        );
        assert!(e.lines.iter().any(|l| l.contains("written volatile")));
    }
}
