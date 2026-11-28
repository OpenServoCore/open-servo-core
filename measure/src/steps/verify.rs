//! The closed-loop check of an installed set: the position loop
//! turned on safely, then a few moves scored, so a bad set is caught
//! before it is saved. It changes no constant; it reads, moves and scores.
//!
//! 1. First enable: Position mode, the goal written to the servo's own
//!    position estimate and the bus dead-man written before torque on, so
//!    torque on commands no motion; [`ENABLE_S`] captured.
//! 2. Small steps of [`SMALL_SHARE`] of the travel, out and back each way,
//!    each committed with a [`SMALL_S`] burst: a wrong sign or an unstable
//!    loop shows inside one burst.
//! 3. The self-test: moves of [`SIZES`] of the travel each way from the
//!    middle of the soft range, each held [`HOLD_S`], captured whole.
//!    Each move is followed by a still hold at its goal, [`STILL_S`] on
//!    the loop row, whose observer speed the stall speed's rule reads
//!    ([`crate::screens::still_speed_cps`]).
//! 4. The wide-code holds, as the scorecard's `wide` capture holds them
//!    (notebooks `stages/5-closed-loop/scorecard/testset.py`): for each
//!    ADC wide code that takes a slot, lowest first, two holds of
//!    [`WIDE_HOLD_MS`] at the middle of the codes it skips, the first after
//!    a goal [`WIDE_APPROACH_COUNTS`] above the span the code stands for,
//!    the second after one as far below it ([`wide`]). The pot flips
//!    across the skipped span there, so these rows hold the worst pot steps
//!    a healthy pot reads, which the pot jump's rule reads with the run's
//!    driven rows ([`crate::screens::step_run`]).
//! 5. The goal back to the middle, held until it arrives, then the way out.
//!
//! The firmware's protections stay in force: the current cap, the speed
//! ceiling, the soft limits, the press detector, the position-error
//! screen; no stall permit is written. Between bursts the host's
//! watchdog reads the rows and a poll: a fault, the shaft outside the
//! soft limits, the current over a quarter past the cap for
//! [`CAP_S`]. "Travel" is the soft range, the physical stops while the
//! soft limits are 0 / 0, off ([`travel`]). Every burst but the still
//! holds carries the raw row, the notebooks' row, so the scores are the
//! scorecard's ([`crate::scorecard`]) and the pot steps read the raw pot.

use std::ops::Range;

use osc_client::pipe::Pipe;
use osc_client::store_state::fault;
use osc_servo_core::regions::config::{ConfigLoopPosition, ConfigPosPhys, ConfigPosSoft};
use serde_json::{Map, Value, json};

use super::{
    Check, Ended, Halt, Rig, Taken, Verdict, interrupted, keep_all, mode_value, read_u8, rest,
    write,
};
use crate::capture::meta;
use crate::capture::rows::{Layout, TelFrame};
use crate::capture::tel::{self, Burst, Image, Line, StreamCfg};
use crate::guard::{self, Ledger, Way};
use crate::install::Refused;
use crate::nameplate::Nameplate;
use crate::scorecard::{
    self as sc, Lin, Move, Profile, Row, Scores, Small, Units, enable_check, small_check,
};
use crate::screens::Holes;
use crate::state::{self, State, field};

pub const STEP: &str = "verify";
pub const RESULT: &str = "results/verify.json";
/// The check's estimate the stall speed's rule reads.
pub const STILL_SPEED: &str = "still_speed_cps";
const METHOD: &str = "closed-loop-self-test";
const REVISION: u32 = 1;
/// The first enable's capture, s.
pub const ENABLE_S: f64 = 0.2;
/// The small steps: this share of the travel, each in one burst of
/// [`SMALL_S`].
pub const SMALL_SHARE: f64 = 0.01;
/// The check's small steps: out and back each way.
pub const SMALLS: usize = 4;

/// The units the check tells, each before a goal write with the loop
/// holding still: the first enable, the small steps, the self-test's
/// `moves`, the `wide` holds of its slotted wide codes, and the return to
/// the middle.
pub fn units(moves: usize, wide: usize) -> u32 {
    (1 + SMALLS + moves + wide + 1) as u32
}
pub const SMALL_S: f64 = 0.1;
/// The self-test's moves, shares of the travel, each way from the middle.
pub const SIZES: [f64; 3] = [0.01, 0.10, 0.33];
/// The hold after each move's profile lands, s.
pub const HOLD_S: f64 = 1.5;
/// The still hold after each move's hold, on the loop row, s.
pub const STILL_S: f64 = 1.0;
/// The shaft starts at least this share of the travel inside each soft
/// limit.
pub const EDGE_SHARE: f64 = 0.05;
/// The watchdog's current: a quarter over the cap, since the firmware
/// holds a stall at it; over it this long, rows or polls in a row, s.
pub const CAP_GAIN: f64 = 1.25;
pub const CAP_S: f64 = 0.1;
pub(crate) const POLL_S: f64 = 0.05;
/// The end move arrives when the shaft reads within the deadband plus
/// this many counts of the middle for [`CENTER_STILL`] polls in a row,
/// within [`CENTER_POLLS`] polls.
pub(crate) const CENTER_TOL: f64 = 20.0;
pub(crate) const CENTER_STILL: u32 = 4;
pub(crate) const CENTER_POLLS: u32 = 200;
/// A move whose profile does not land inside this is refused, s.
const PROFILE_MAX_S: f64 = 10.0;
/// The most rows one burst carries: `tel_count` is a u16.
const BURST_ROWS: usize = u16::MAX as usize;
/// The wide-code holds: each held this long, ms, after an approach goal
/// this many counts past the span the code stands for.
pub const WIDE_HOLD_MS: u32 = 2000;
pub const WIDE_APPROACH_COUNTS: i32 = 30;
/// An approach goal is held as long as a step of its travel from the goal
/// before it: the hold of the smallest step size at or above the travel,
/// deg, the largest size's beyond it.
pub const STEP_SIZES_DEG: [f64; 3] = [2.0, 20.0, 60.0];
pub const STEP_HOLD_MS: [u32; 3] = [1200, 1500, 2000];
/// A wide code is held when both approach goals lie this far clear of the
/// soft range, deg.
pub const FIT_MARGIN_DEG: f64 = 5.0;
/// A slot holds half the codes skipped, Q4: 8 x the codes skipped.
const Q4_PER_SKIPPED: u16 = 8;

/// What the step reads besides the servo.
pub struct VerifyInputs<'a> {
    pub nameplate: &'a Nameplate,
    /// The run's pot holes, which the check's pot steps leave out.
    pub holes: &'a Holes,
}

/// An ADC wide code with a slot: its stuck code and the codes it skips.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WideCode {
    pub stuck: u16,
    pub skipped: u16,
}

/// The wide codes the servo's slots `slots` give, lowest first: slot `k`
/// belongs to the stuck code `(k << 8) | 0xFF` and holds 8 x the codes it
/// skips.
pub fn wide_codes(slots: &[u8]) -> Vec<WideCode> {
    slots
        .iter()
        .enumerate()
        .filter(|(_, v)| **v != 0)
        .map(|(k, v)| WideCode {
            stuck: ((k as u16) << osc_servo_core::adc::ADC_WIDE_SHIFT)
                | osc_servo_core::adc::ADC_WIDE_MASK,
            skipped: *v as u16 / Q4_PER_SKIPPED,
        })
        .collect()
}

/// Where a wide-code segment's goal lies against the span.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Side {
    Above,
    Below,
}

impl Side {
    pub fn name(self) -> &'static str {
        match self {
            Side::Above => "above",
            Side::Below => "below",
        }
    }
}

/// One segment of the wide-code holds: an approach or a hold at a code's
/// middle, its raw goal, its goal in the counts the servo runs in, and how
/// long it is held.
#[derive(Clone, Debug, PartialEq)]
pub struct WideSeg {
    pub code: WideCode,
    pub side: Side,
    pub hold: bool,
    pub raw: i32,
    pub goal: i32,
    pub ms: u32,
}

impl WideSeg {
    pub fn test(&self) -> &'static str {
        if self.hold {
            "wide-hold"
        } else {
            "wide-approach"
        }
    }
}

/// A wide code's raw goals: the approach above the span it stands for,
/// the middle of the codes it skips, the approach below the span.
pub fn wide_goals(c: WideCode) -> (i32, i32, i32) {
    let (stuck, skipped) = (c.stuck as i32, c.skipped as i32);
    (
        stuck + skipped + WIDE_APPROACH_COUNTS,
        stuck + 1 + skipped / 2,
        stuck - WIDE_APPROACH_COUNTS,
    )
}

/// How long a goal `travel_deg` from the one before it is held, ms.
pub fn approach_ms(travel_deg: f64) -> u32 {
    STEP_SIZES_DEG
        .iter()
        .zip(STEP_HOLD_MS)
        .find(|(size, _)| travel_deg <= **size)
        .map_or(STEP_HOLD_MS[STEP_HOLD_MS.len() - 1], |(_, ms)| ms)
}

/// Whether goals `lo` and `hi` both lie [`FIT_MARGIN_DEG`] clear of the
/// soft range `soft`.
fn fits(lo: i32, hi: i32, soft: (f64, f64), cpd: f64) -> bool {
    let margin = FIT_MARGIN_DEG * cpd;
    lo as f64 - margin >= soft.0 && hi as f64 + margin <= soft.1
}

/// The wide-code holds in order: per code of `codes` whose approach goals
/// fit the soft range `soft`, lowest first, its middle held after the
/// approach from above, then after the one from below, each approach held
/// for its travel from the goal before it, the first from `center`. With a
/// live table `table` the goals are its reading of the raw goals, to the
/// count; with none, the raw goals. A code whose raw goals leave the
/// converter's codes is not held.
pub fn wide(
    center: f64,
    cpd: f64,
    soft: (f64, f64),
    codes: &[WideCode],
    table: Option<&Lin>,
) -> Vec<WideSeg> {
    let through = |raw: i32| match table {
        None => raw,
        Some(lin) => lin.counts(raw as u16).round_ties_even() as i32,
    };
    let mut codes = codes.to_vec();
    codes.sort_by_key(|c| c.stuck);
    let mut segs = Vec::new();
    let mut at = center;
    for c in codes {
        let (above, middle, below) = wide_goals(c);
        let adc = 0..=osc_servo_core::adc::ADC_MASK as i32;
        if !adc.contains(&above) || !adc.contains(&below) {
            continue;
        }
        let goal = |raw: i32| (raw, through(raw));
        let (above, middle, below) = (goal(above), goal(middle), goal(below));
        if !fits(below.1, above.1, soft, cpd) {
            continue;
        }
        for (side, approach) in [(Side::Above, above), (Side::Below, below)] {
            segs.push(WideSeg {
                code: c,
                side,
                hold: false,
                raw: approach.0,
                goal: approach.1,
                ms: approach_ms((approach.1 as f64 - at).abs() / cpd),
            });
            segs.push(WideSeg {
                code: c,
                side,
                hold: true,
                raw: middle.0,
                goal: middle.1,
                ms: WIDE_HOLD_MS,
            });
            at = middle.1 as f64;
        }
    }
    segs
}

#[derive(Clone, Debug)]
pub struct Verified {
    pub ended: Ended,
    pub checks: Vec<Check>,
    pub result: Option<Value>,
}

impl Verified {
    pub fn accepted(&self) -> bool {
        self.ended.way == Way::Done
    }
}

/// One self-test move as planned.
#[derive(Clone, Debug, PartialEq)]
pub struct Planned {
    pub name: String,
    pub size: f64,
    pub from: f64,
    pub goal: f64,
    pub rows: usize,
}

/// Where degrees come from.
#[derive(Clone, Debug, PartialEq)]
pub struct Scale {
    pub counts_per_deg: f64,
    pub source: String,
}

/// Counts per degree: the set's angle map when the servo stores one, else
/// the nameplate's travel over the soft range.
pub fn scale(
    raw: (f64, f64),
    angle_cdeg: (f64, f64),
    travel_counts: f64,
    nameplate_deg: Option<f64>,
) -> Option<Scale> {
    let (span, deg) = (raw.1 - raw.0, (angle_cdeg.1 - angle_cdeg.0).abs() / 100.0);
    if span > 0.0 && deg > 0.0 {
        return Some(Scale {
            counts_per_deg: span / deg,
            source: format!(
                "the set's angle map on the servo: {deg:.2} deg over raw {}..{}",
                raw.0, raw.1
            ),
        });
    }
    nameplate_deg.filter(|d| *d > 0.0).map(|d| Scale {
        counts_per_deg: travel_counts / d,
        source: format!(
            "the servo stores no angle map: the nameplate's {d} deg travel over the {travel_counts:.0} count soft range"
        ),
    })
}

/// The self-test's moves from the middle `mid`: for each size, out to
/// +size and back, out to -size and back.
pub fn self_test(mid: f64, travel: f64) -> Vec<(String, f64, f64, f64)> {
    let mut out = Vec::new();
    for size in SIZES {
        let s = (size * travel).round();
        for (name, from, goal) in [
            ("+ out", mid, mid + s),
            ("+ back", mid + s, mid),
            ("- out", mid, mid - s),
            ("- back", mid - s, mid),
        ] {
            out.push((format!("{:.0}% {name}", 100.0 * size), size, from, goal));
        }
    }
    out
}

/// The host's watchdog between bursts.
#[derive(Clone, Debug, PartialEq)]
pub struct Watch {
    /// The soft limits, linearized counts.
    pub soft: (f64, f64),
    /// |current| counts over the shunt's bias.
    pub cap_counts: f64,
    pub cap_rows: usize,
    pub cap_polls: u32,
    over: u32,
}

impl Watch {
    pub fn new(soft: (f64, f64), cap_counts: f64, tick_hz: f64) -> Self {
        Watch {
            soft,
            cap_counts,
            cap_rows: ((CAP_S * tick_hz).round() as usize).max(1),
            cap_polls: ((CAP_S / POLL_S).ceil() as u32).max(1),
            over: 0,
        }
    }

    fn inside(&self, pos: f64) -> Result<(), String> {
        let (lo, hi) = self.soft;
        if pos < lo || pos > hi {
            return Err(format!(
                "the shaft at {pos:.0} counts is outside the soft limits {lo:.0}..{hi:.0}"
            ));
        }
        Ok(())
    }

    /// A burst's rows: every row inside the soft limits, the current of
    /// read windows under the cap but for runs shorter than [`CAP_S`].
    pub fn rows(&self, rows: &[Row], bias: f64) -> Result<(), String> {
        let mut run = 0usize;
        for r in rows {
            self.inside(r.pos)
                .map_err(|e| format!("{e} at {:.4} s", r.t))?;
            match r.current_raw.filter(|_| r.window_valid) {
                Some(c) if (c as f64 - bias).abs() > self.cap_counts => {
                    run += 1;
                    if run >= self.cap_rows {
                        return Err(format!(
                            "|current| over the {:.0} count cap for {run} rows, to {:.4} s",
                            self.cap_counts, r.t
                        ));
                    }
                }
                Some(_) => run = 0,
                None => {}
            }
        }
        Ok(())
    }

    /// One poll: the shaft, and the held current over the cap for
    /// [`CAP_S`] of polls.
    pub fn poll(&mut self, pos: f64, i_tick_counts: i16) -> Result<(), String> {
        self.inside(pos)?;
        if i_tick_counts.unsigned_abs() as f64 > self.cap_counts {
            self.over += 1;
        } else {
            self.over = 0;
        }
        if self.over >= self.cap_polls {
            return Err(format!(
                "|current| {} counts over the {:.0} count cap for {} polls",
                i_tick_counts.unsigned_abs(),
                self.cap_counts,
                self.over
            ));
        }
        Ok(())
    }
}

/// What the step knew before its first write.
struct Before {
    line: Line,
    servo: Map<String, Value>,
    stream: StreamCfg,
    lin: Lin,
    soft: (f64, f64),
    travel: f64,
    mid: f64,
    pos: f64,
    scale: Scale,
    counts_per_a: f64,
    tick_hz: f64,
    screen_counts: f64,
    cap_counts: f64,
    deadband: f64,
    deadman_ticks: u16,
    bias: f64,
    profile: Profile,
    small: f64,
    lead_rows: usize,
    moves: Vec<Planned>,
    codes: Vec<WideCode>,
    wide: Vec<WideSeg>,
}

fn num(state: &State, d: &osc_client::descriptor::Descriptor, name: &str) -> Result<f64, Refused> {
    let f = field(d, name)?;
    state
        .text(f)
        .as_f64()
        .ok_or_else(|| Refused(format!("the state read holds no {name}")))
}

/// The travel the servo holds to, as its take resolves it: the soft
/// limits, or the physical stops while those are 0 / 0, off.
pub(crate) fn travel(
    state: &State,
    d: &osc_client::descriptor::Descriptor,
) -> Result<ConfigPosSoft, Refused> {
    let n = |name: &str| num(state, d, name).map(|v| v as i32);
    let soft = ConfigPosSoft {
        pos_min_soft_counts: n("pos_min_soft_counts")?,
        pos_max_soft_counts: n("pos_max_soft_counts")?,
    };
    Ok(soft.or_stops(&ConfigPosPhys {
        pos_min_phys_counts: n("pos_min_phys_counts")?,
        pos_max_phys_counts: n("pos_max_phys_counts")?,
    }))
}

/// The servo's own position estimate, rounded to the count: a `pos` read
/// over the bus rings the pot channel a few counts off the rest.
pub(crate) async fn theta_of<P: Pipe>(
    c: &mut osc_client::Client<P>,
    id: osc_client::Id,
    d: &osc_client::descriptor::Descriptor,
) -> Result<i32, osc_client::Error> {
    let f = field(d, "theta_hat_q16")?;
    let b = c.read(id, f.addr, 4).await?;
    let [q0, q1, q2, q3, ..] = b[..] else {
        return Err(osc_client::Error::Pipe(osc_client::pipe::PipeError::Io(
            format!(
                "the read of theta_hat_q16 came back with {} of its 4 bytes",
                b.len()
            ),
        )));
    };
    let q = i32::from_le_bytes([q0, q1, q2, q3]);
    Ok(((q as i64 + (1 << 15)) >> 16) as i32)
}

async fn theta<P: Pipe>(rig: &mut Rig<'_, P>) -> Result<i32, Halt> {
    Ok(theta_of(rig.c, rig.id, rig.d).await?)
}

fn rows_for(s: f64, tick_hz: f64) -> usize {
    (s * tick_hz).ceil() as usize
}

/// The refusals before any write that no installed set changes: the bus
/// and the image, torque on, a latched fault. The line and the state read.
pub async fn admit<P: Pipe>(
    c: &mut osc_client::Client<P>,
    id: osc_client::Id,
    d: &osc_client::descriptor::Descriptor,
) -> Result<(Line, State), Refused> {
    let line = tel::line(c, id).await?;
    line.admit(Layout::Raw)?;
    line.admit(Layout::Loop)?;
    if line.image() != Image::Production {
        return Err(Refused(format!(
            "the servo runs the {}, which has no closed loops: flash a production image",
            line.said()
        )));
    }
    let state = state::read(c, id, d).await?;
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this step starts only with torque off".into(),
        ));
    }
    let faults = {
        let f = field(d, "fault_flags")?;
        c.read(id, f.addr, 1).await?.first().copied().unwrap_or(0)
    };
    if faults != 0 {
        return Err(Refused(format!(
            "a fault is latched (fault_flags {faults:#04x}): `osc-bus status` names it"
        )));
    }
    Ok((line, state))
}

/// The constants whose 0 the kernel's belt refuses the closed loops on
/// (protocol sec 5.7): a zero Ke, an unset current cap or speed ceiling.
/// They name the reason; `refused_modes` is the verdict.
const BELT: [&str; 4] = [
    "recip_ke_q",
    "ke_vpc_q",
    "current_limit_counts",
    "speed_ceiling_cps",
];

/// Refused when the servo refuses one of `modes` now (`refused_modes`),
/// with the belt's reason: the [`BELT`] constants that read 0. The stored
/// state is no refusal; it is information.
pub(crate) fn belt(
    state: &State,
    d: &osc_client::descriptor::Descriptor,
    modes: &[&str],
) -> Result<(), Refused> {
    let refused: Vec<&str> = modes
        .iter()
        .copied()
        .filter(|m| state.store.refused_modes.iter().any(|r| r == m))
        .collect();
    if refused.is_empty() {
        return Ok(());
    }
    let mut unset = Vec::new();
    for name in BELT {
        if num(state, d, name)? == 0.0 {
            unset.push(name);
        }
    }
    Err(Refused(format!(
        "the servo refuses {} (refused_modes {}): {} 0, unset (protocol sec 5.7), so nothing moves; an identification sets them (`osc-ident run`)",
        refused.join(" and "),
        state.refused_said(),
        unset.join(", ")
    )))
}

async fn before<P: Pipe>(rig: &mut Rig<'_, P>, i: &VerifyInputs<'_>) -> Result<Before, Refused> {
    let (line, state) = admit(rig.c, rig.id, rig.d).await?;
    belt(&state, rig.d, &["Position"])?;
    let n = |name: &str| num(&state, rig.d, name);
    let held = travel(&state, rig.d)?;
    let soft = (
        held.pos_min_soft_counts as f64,
        held.pos_max_soft_counts as f64,
    );
    let travel = soft.1 - soft.0;
    if travel <= 0.0 || travel.is_nan() {
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
    let screen = n("pos_error_counts")?;
    if screen <= 0.0 {
        return Err(Refused(
            "the set holds no position-error screen (pos_error_counts 0): the following error has nothing to be judged by".into(),
        ));
    }
    let scale = scale(
        (n("raw_min")?, n("raw_max")?),
        (n("angle_min_cdeg")?, n("angle_max_cdeg")?),
        travel,
        i.nameplate.travel_deg,
    )
    .ok_or_else(|| {
        Refused("the servo stores no angle map and the nameplate states no travel_deg: no degree to score in".into())
    })?;
    let sense = rest::Sense::of(&state, rig.d)?;
    let tick_hz = sense.tick_hz;
    let lin = Lin::new(state.lut.effective(), state.slots(rig.d).as_deref());
    let live = state.lut.effective().is_some();
    let pos = theta_of(rig.c, rig.id, rig.d).await? as f64;
    let edge = EDGE_SHARE * travel;
    if pos < soft.0 + edge || pos > soft.1 - edge {
        return Err(Refused(format!(
            "the shaft reads {pos:.0} counts, within {:.0}% of the travel ({edge:.0} counts) of a soft limit ({:.0}..{:.0}): turn the horn toward the middle with torque off",
            100.0 * EDGE_SHARE,
            soft.0,
            soft.1
        )));
    }
    let ceiling_u = ceiling as u16;
    let v = n("velocity_limit_cps")? as u16;
    let profile = Profile {
        lp: ConfigLoopPosition {
            p_kp_q88: n("p_kp_q88")? as u16,
            pos_deadband_counts: n("pos_deadband_counts")? as u16,
            speed_ceiling_cps: ceiling_u,
            velocity_limit_cps: if v == 0 { ceiling_u } else { v.min(ceiling_u) },
            accel_limit_q88: n("accel_limit_q88")? as u16,
            _rsvd_align: 0,
        },
        soft: held,
        medium_hz: tick_hz / 10.0,
    };
    let mid = (soft.0 + (soft.1 - soft.0) / 2.0).floor();
    let landed = |from: f64, goal: f64| {
        profile.time_s(from, goal, PROFILE_MAX_S).ok_or_else(|| {
            Refused(format!(
                "the set's profile takes over {PROFILE_MAX_S} s from {from:.0} to {goal:.0} counts: its acceleration or speed limit is near zero"
            ))
        })
    };
    let lead_rows = rows_for(landed(pos, mid)? + HOLD_S, tick_hz);
    let mut moves = Vec::new();
    for (name, size, from, goal) in self_test(mid, travel) {
        moves.push(Planned {
            rows: rows_for(landed(from, goal)? + HOLD_S, tick_hz),
            name,
            size,
            from,
            goal,
        });
    }
    let deadman = (rest::DEADMAN_S * tick_hz).round();
    if !(1.0..=u16::MAX as f64).contains(&deadman) {
        return Err(Refused(format!(
            "a {} s dead-man is no u16 of ticks at {tick_hz} Hz",
            rest::DEADMAN_S
        )));
    }
    let bias = {
        let f = field(rig.d, "current_bias_counts")?;
        let b = rig.c.read(rig.id, f.addr, 2).await?;
        u16::from_le_bytes([b[0], b[1]]) as f64
    };
    let stream = StreamCfg::read(rig.c, rig.id, rig.d).await?;
    let codes = wide_codes(&lin.slots);
    let wide = wide(
        mid,
        scale.counts_per_deg,
        soft,
        &codes,
        live.then_some(&lin),
    );
    Ok(Before {
        servo: meta::servo(&state, rig.d, &line),
        line,
        stream,
        lin,
        soft,
        travel,
        mid,
        pos,
        scale,
        counts_per_a: sense.counts_per_a(),
        tick_hz,
        screen_counts: screen,
        cap_counts: cap,
        deadband: n("pos_deadband_counts")?,
        deadman_ticks: deadman as u16,
        bias,
        profile,
        small: (SMALL_SHARE * travel).round().max(1.0),
        lead_rows,
        moves,
        codes,
        wide,
    })
}

/// A burst's rows on one time axis with the bursts before it in its
/// capture: each burst starts past the end of the one before by the
/// shortest wrap of its first row's 16-bit stamp, as the scorecard puts
/// segments on its axis.
pub(crate) fn chain<'a>(
    bursts: impl IntoIterator<Item = &'a Burst>,
    lin: &Lin,
    tick_hz: f64,
) -> Vec<Row> {
    let mut out = Vec::new();
    let mut end: Option<u64> = None;
    let mut origin = None;
    for b in bursts {
        let stamp = b.first_stamp.unwrap_or(0) as u64;
        let start = match end {
            None => stamp,
            Some(e) => e + (stamp.wrapping_sub(e) & 0xFFFF),
        };
        let o = *origin.get_or_insert(start);
        for f in &b.rows {
            out.push(row(f, start + f.tick - o, lin, tick_hz));
        }
        end = Some(start + b.rows.last().map_or(0, |f| f.tick + 1));
    }
    out
}

fn row(f: &TelFrame, tick: u64, lin: &Lin, tick_hz: f64) -> Row {
    Row {
        t: tick as f64 / tick_hz,
        pos: f.pos.map_or(f64::NAN, |p| lin.counts(p)),
        duty: f.duty_counts.unwrap_or(0),
        current_raw: f.current_raw,
        window_valid: f.window_valid,
    }
}

/// What the body found, as far as it got.
#[derive(Default)]
struct Found {
    goal0: Option<f64>,
    enable: Option<Check>,
    small: Vec<Small>,
    /// Each self-test move scored, its bursts' places in the step's
    /// captures.
    moves: Vec<(Planned, Range<usize>)>,
    /// Each still hold: its move's name and its place in the step's
    /// captures.
    still: Vec<(String, usize)>,
    /// Each wide-code segment held, its bursts' places in the step's
    /// captures.
    wide: Vec<(WideSeg, Range<usize>)>,
    fault: Option<String>,
    fault_on: Option<String>,
    centered: Option<f64>,
}

fn fault_said(flags: u8, code: u8) -> String {
    format!(
        "fault_flags {flags:#04x}, fault_code {code} ({})",
        fault::name(code).unwrap_or("unknown")
    )
}

/// One burst of `rows` on `layout`, its goal committed with it when `goal` is Some,
/// kept in `taken`: its place there, and a fault it latched, said. A
/// burst that lost a row is no evidence, and a goal's burst cannot be
/// taken again: the step aborts.
async fn shot<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    taken: &mut Vec<Taken>,
    rows: u16,
    layout: Layout,
    goal: Option<i32>,
    plan: Value,
) -> Result<(usize, Option<String>), Halt> {
    let gp = field(rig.d, "goal_position")?.addr;
    let bytes = goal.map(i32::to_le_bytes);
    let t0 = (rig.clock)();
    let burst = tel::stream_with(
        rig.c,
        rig.id,
        rig.d,
        &b.stream,
        layout,
        rows,
        bytes.as_ref().map(|g| (gp, &g[..])),
    )
    .await?;
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
        return Ok((k, Some(fault_said(flags, code))));
    }
    if !lost.is_empty() {
        return Err(Halt::Error(format!(
            "capture {nn:02} lost rows ({}): no evidence, and a move cannot be taken again; check the cable and its contacts",
            lost.join(", ")
        )));
    }
    Ok((k, None))
}

/// The poll between bursts: a fault, said; or the watchdog's verdict on
/// the shaft and the held current.
async fn poll<P: Pipe>(rig: &mut Rig<'_, P>, b: &Before, w: &mut Watch) -> Result<Polled, Halt> {
    let base = field(rig.d, "fault_flags")?.addr;
    let at = |name: &str| field(rig.d, name).map(|f| (f.addr - base) as usize);
    let (code_at, pos_at, i_at) = (at("fault_code")?, at("pos")?, at("i_tick_counts")?);
    let raw = rig.c.read(rig.id, base, i_at as u16 + 2).await?;
    let word = |k: usize| {
        raw.get(k..k + 2)
            .map_or(0, |b| u16::from_le_bytes([b[0], b[1]]))
    };
    let flags = raw.first().copied().unwrap_or(0);
    if flags != 0 {
        let code = raw.get(code_at).copied().unwrap_or(0);
        return Ok(Polled::Fault(fault_said(flags, code)));
    }
    let pos = b.lin.counts(word(pos_at));
    w.poll(pos, word(i_at) as i16)
        .map_err(|e| Halt::Error(format!("the watchdog: {e}; torque goes off")))?;
    Ok(Polled::Ok(pos))
}

enum Polled {
    Ok(f64),
    Fault(String),
}

fn watch_rows(w: &Watch, b: &Before, bursts: &[Taken]) -> Result<(), Halt> {
    let bursts = bursts.iter().map(|t| &t.burst);
    w.rows(&chain(bursts, &b.lin, b.tick_hz), b.bias)
        .map_err(|e| Halt::Error(format!("the watchdog on the rows: {e}; torque goes off")))
}

/// A refusal the first enable or a small step makes: the set is not
/// trusted with a loop.
fn drove(why: String) -> Halt {
    Halt::Refused(format!("the loop drove the shaft on its own: {why}"))
}

/// A capture of `rows` with its goal committed by the first burst,
/// chained in bursts of at most [`BURST_ROWS`], the watchdog between.
async fn capture<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    w: &mut Watch,
    taken: &mut Vec<Taken>,
    rows: usize,
    goal: i32,
    plan: &Value,
) -> Result<(Range<usize>, Option<String>), Halt> {
    let first = taken.len();
    let mut left = rows;
    let mut k = 0;
    while left > 0 {
        let n = left.min(BURST_ROWS);
        let mut p = plan.clone();
        if let Some(m) = p.as_object_mut() {
            m.insert("burst".into(), k.into());
        }
        let (_, fault) = shot(
            rig,
            b,
            taken,
            n as u16,
            Layout::Raw,
            (k == 0).then_some(goal),
            p,
        )
        .await?;
        if fault.is_some() {
            return Ok((first..taken.len(), fault));
        }
        watch_rows(w, b, &taken[first..])?;
        if let Polled::Fault(f) = poll(rig, b, w).await? {
            return Ok((first..taken.len(), Some(f)));
        }
        left -= n;
        k += 1;
    }
    Ok((first..taken.len(), None))
}

async fn body<P: Pipe>(
    rig: &mut Rig<'_, P>,
    b: &Before,
    ledger: &mut Ledger,
    taken: &mut Vec<Taken>,
    found: &mut Found,
    lines: &mut Vec<String>,
) -> Result<(), Halt> {
    let mut w = Watch::new(b.soft, CAP_GAIN * b.cap_counts, b.tick_hz);
    ledger.touched = true;
    let position = mode_value(rig.d, "Position")?;
    ledger.keep(rig.c, rig.id, rig.d, "mode").await?;
    let goal0 = theta(rig).await?;
    found.goal0 = Some(goal0 as f64);
    let of = units(b.moves.len(), b.wide.len());
    let mut at = 0;
    let mut tell = |rig: &Rig<'_, P>| {
        at += 1;
        (rig.tell)(super::Unit {
            at,
            of,
            what: "move",
        })
    };
    tell(rig);
    write(rig, "goal_position", &goal0.to_le_bytes()).await?;
    write(rig, "mode", &[position]).await?;
    write(rig, "bus_deadman_ticks", &b.deadman_ticks.to_le_bytes()).await?;
    write(rig, "torque_enable", &[1]).await?;
    lines.push(format!(
        "first enable: goal {goal0} counts, the servo's own estimate; Position mode; dead-man {} ticks ({} s); then torque on, {ENABLE_S} s captured",
        b.deadman_ticks,
        rest::DEADMAN_S
    ));
    let (bu, fault) = shot(
        rig,
        b,
        taken,
        rows_for(ENABLE_S, b.tick_hz) as u16,
        Layout::Raw,
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
    let rows = chain([&taken[bu].burst], &b.lin, b.tick_hz);
    let c = enable_check(&rows, goal0 as f64, b.travel, fault.as_deref());
    lines.push(format!("  {}", c.said()));
    let failed = c.verdict == Verdict::Fail;
    found.enable = Some(c.clone());
    if failed {
        return Err(drove(c.value));
    }
    watch_rows(&w, b, &taken[bu..=bu])?;
    interrupted(rig.stop)?;

    let s = b.small;
    let g = goal0 as f64;
    lines.push(format!(
        "small steps: {s:.0} counts ({:.0}% of the travel) out and back each way, each committed with a {SMALL_S} s burst",
        100.0 * SMALL_SHARE
    ));
    for (name, goal) in [
        ("+ out", g + s),
        ("+ back", g),
        ("- out", g - s),
        ("- back", g),
    ] {
        tell(rig);
        let (bu, fault) = shot(
            rig,
            b,
            taken,
            rows_for(SMALL_S, b.tick_hz) as u16,
            Layout::Raw,
            Some(goal as i32),
            json!({"phase": "small", "move": name, "goal_counts": goal}),
        )
        .await?;
        let rows = chain([&taken[bu].burst], &b.lin, b.tick_hz);
        let st = Small::of(name, goal, s, &rows, fault);
        lines.push(format!("  {}", st.said()));
        let failed = st.failed();
        found.small.push(st.clone());
        if failed {
            return Err(drove(format!(
                "the small step {}; a wrong sign or an unstable loop",
                st.said()
            )));
        }
        watch_rows(&w, b, &taken[bu..=bu])?;
        if let Polled::Fault(f) = poll(rig, b, &mut w).await? {
            let st = found.small.last_mut().map(|st| {
                st.fault = Some(f.clone());
                st.said()
            });
            return Err(drove(format!(
                "after the small step {}",
                st.unwrap_or_default()
            )));
        }
        interrupted(rig.stop)?;
    }

    lines.push(format!(
        "self-test: to the middle ({:.0} counts), then {} moves of {} of the travel each way, each held {HOLD_S} s past its profile, then {STILL_S} s still on the loop row",
        b.mid,
        b.moves.len(),
        SIZES
            .iter()
            .map(|s| format!("{:.0}%", 100.0 * s))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    let (_, fault) = capture(
        rig,
        b,
        &mut w,
        taken,
        b.lead_rows,
        b.mid as i32,
        &json!({"phase": "lead", "from_counts": g, "goal_counts": b.mid}),
    )
    .await?;
    if let Some(f) = fault {
        found.fault_on = Some("the move to the middle".into());
        found.fault = Some(f.clone());
        return Err(Halt::Refused(format!(
            "the set is not accepted: a fault latched on the move to the middle: {f}"
        )));
    }
    for m in &b.moves {
        interrupted(rig.stop)?;
        tell(rig);
        let plan = json!({"phase": "self-test", "move": m.name, "size_share": m.size,
                          "from_counts": m.from, "goal_counts": m.goal});
        let (bursts, fault) = capture(rig, b, &mut w, taken, m.rows, m.goal as i32, &plan).await?;
        // a faulted move's rows are the coast after the fault: kept, not
        // scored
        if fault.is_none() {
            found.moves.push((m.clone(), bursts));
        }
        if let Some(f) = fault {
            found.fault_on = Some(m.name.clone());
            found.fault = Some(f.clone());
            return Err(Halt::Refused(format!(
                "the set is not accepted: a fault latched on the {} move: {f}",
                m.name
            )));
        }
        let plan = json!({"phase": "still", "move": m.name, "goal_counts": m.goal});
        let rows = rows_for(STILL_S, b.tick_hz) as u16;
        let (k, mut fault) = shot(rig, b, taken, rows, Layout::Loop, None, plan).await?;
        if fault.is_none()
            && let Polled::Fault(f) = poll(rig, b, &mut w).await?
        {
            fault = Some(f);
        }
        if let Some(f) = fault {
            let on = format!("the still hold after the {} move", m.name);
            found.fault_on = Some(on.clone());
            found.fault = Some(f.clone());
            return Err(Halt::Refused(format!(
                "the set is not accepted: a fault latched on {on}: {f}"
            )));
        }
        found.still.push((m.name.clone(), k));
    }

    lines.push(wide_said(b));
    for s in &b.wide {
        interrupted(rig.stop)?;
        tell(rig);
        let plan = json!({"phase": s.test(), "stuck_code": s.code.stuck, "skipped": s.code.skipped,
                          "side": s.side.name(), "raw_counts": s.raw, "goal_counts": s.goal, "ms": s.ms});
        let rows = rows_for(s.ms as f64 / 1e3, b.tick_hz);
        let (bursts, fault) = capture(rig, b, &mut w, taken, rows, s.goal, &plan).await?;
        if let Some(f) = fault {
            let on = format!(
                "the {} at wide code {} from {}",
                s.test(),
                s.code.stuck,
                s.side.name()
            );
            found.fault_on = Some(on.clone());
            found.fault = Some(f.clone());
            return Err(Halt::Refused(format!(
                "the set is not accepted: a fault latched on {on}: {f}"
            )));
        }
        found.wide.push((s.clone(), bursts));
    }

    tell(rig);
    write(rig, "goal_position", &(b.mid as i32).to_le_bytes()).await?;
    let tol = b.deadband + CENTER_TOL;
    let mut still = 0;
    let mut at = f64::NAN;
    for _ in 0..CENTER_POLLS {
        interrupted(rig.stop)?;
        rig.c
            .pause(std::time::Duration::from_secs_f64(POLL_S))
            .await;
        match poll(rig, b, &mut w).await? {
            Polled::Fault(f) => {
                return Err(Halt::Error(format!(
                    "a fault latched on the way back to the middle: {f}"
                )));
            }
            Polled::Ok(p) => at = p,
        }
        still = if (at - b.mid).abs() <= tol {
            still + 1
        } else {
            0
        };
        if still >= CENTER_STILL {
            found.centered = Some(at);
            lines.push(format!(
                "end: the shaft at {at:.0} counts, the middle {:.0}",
                b.mid
            ));
            return Ok(());
        }
    }
    Err(Halt::Error(format!(
        "the shaft read {at:.0} counts, not within {tol:.0} of the middle {:.0}, after {:.0} s: torque goes off where it is",
        b.mid,
        CENTER_POLLS as f64 * POLL_S
    )))
}

/// The step on the servo. Refused with nothing written when it cannot
/// start; once it has written it ends through the guard whatever happens,
/// keeps its captures and its result.
pub async fn verify<P: Pipe>(
    rig: &mut Rig<'_, P>,
    i: &VerifyInputs<'_>,
) -> Result<Verified, Refused> {
    let b = before(rig, i).await?;
    let mut lines = vec![
        format!("{}; bus at {} baud", b.line.said(), b.line.baud.as_hz()),
        format!(
            "position {:.0} counts, soft range {:.0}..{:.0} ({:.0} counts of travel), the middle {:.0}; {:.3} counts/deg, {}",
            b.pos, b.soft.0, b.soft.1, b.travel, b.mid, b.scale.counts_per_deg, b.scale.source
        ),
    ];
    let mut ledger = Ledger::default();
    let mut taken = Vec::new();
    let mut found = Found::default();
    let ran = body(rig, &b, &mut ledger, &mut taken, &mut found, &mut lines).await;
    let exit = guard::exit(rig.c, rig.id, rig.d, &ledger).await;
    let after = theta(rig).await.ok();
    let kept = keep_all(rig.sink, STEP, &taken, &mut lines).map_err(Halt::from);
    let units = Units {
        counts_per_deg: b.scale.counts_per_deg,
        counts_per_a: b.counts_per_a,
        tick_hz: b.tick_hz,
        zero_counts: sc::current_zero(
            chain(
                taken
                    .iter()
                    .map(|t| &t.burst)
                    .filter(|b| b.layout == Layout::Raw),
                &b.lin,
                b.tick_hz,
            )
            .iter(),
        ),
    };
    let scored: Vec<(Planned, Scores)> = found
        .moves
        .iter()
        .map(|(p, at)| {
            let m = Move {
                name: p.name.clone(),
                from: p.from,
                goal: p.goal,
                rows: chain(
                    taken[at.clone()].iter().map(|t| &t.burst),
                    &b.lin,
                    b.tick_hz,
                ),
            };
            (p.clone(), sc::score(&m, &units, Some(&b.profile)))
        })
        .collect();
    for (p, s) in &scored {
        lines.push(format!("  {}", said(p, s, &units, b.cap_counts)));
    }
    let mut checks: Vec<Check> = found.enable.iter().cloned().collect();
    if !found.small.is_empty() {
        checks.push(small_check(&found.small));
    }
    let complete = found.moves.len() == b.moves.len();
    // the moves scored are every planned one, or those ahead of a fault
    if complete || found.fault.is_some() {
        let named: Vec<(String, Scores)> =
            scored.iter().map(|(p, s)| (p.name.clone(), *s)).collect();
        let fault = found
            .fault
            .as_ref()
            .map(|f| format!("{f} on {}", found.fault_on.as_deref().unwrap_or("a move")));
        checks.extend(sc::accept(
            &named,
            &units,
            b.screen_counts,
            fault.as_deref(),
        ));
    }
    if !exit.torque_off {
        checks.push(Check {
            name: "torque off at the end".into(),
            value: "torque_enable did not read back 0".into(),
            bound: "reads 0".into(),
            verdict: Verdict::Fail,
        });
    }
    let failed: Vec<String> = checks
        .iter()
        .filter(|c| c.verdict == Verdict::Fail)
        .map(|c| c.name.clone())
        .collect();
    let (mut way, mut why) = match (&ran, &kept) {
        (Err(h), _) => (h.way(), Some(h.why())),
        (Ok(()), Err(h)) => (Way::Error, Some(h.why())),
        (Ok(()), Ok(())) if !failed.is_empty() => (
            Way::Refused,
            Some(format!(
                "the set is not accepted: it failed {}",
                failed.join(", ")
            )),
        ),
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
    let mut result = result_json(
        &b, &found, &taken, &scored, &units, &checks, &failed, way, &why, &exit, after,
    );
    if ran.is_ok() {
        let driven: Vec<&[TelFrame]> = taken
            .iter()
            .filter(|t| t.burst.layout == Layout::Raw)
            .map(|t| t.burst.rows.as_slice())
            .collect();
        super::with_pot_step4(
            &mut result,
            &driven,
            i.holes,
            "the check, its wide-code holds too",
        );
    }
    let mut text = serde_json::to_vec_pretty(&result).unwrap_or_default();
    text.push(b'\n');
    match rig.sink.write(RESULT, &text) {
        Ok(()) => lines.push(format!("result: {}", rig.sink.shown(RESULT))),
        Err(e) => {
            way = Way::Error;
            why = Some(format!("the result was not kept: {e}"));
        }
    }
    Ok(Verified {
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

fn said(p: &Planned, s: &Scores, u: &Units, cap_counts: f64) -> String {
    format!(
        "{}: {:.2} deg step, overshoot {:.2} deg (bound {:.2}), settled in {:.0} ms, last 500 ms {:.1}% driven ({}), rest error {:.2} deg, peak current {:.0} mA of the {:.0} mA cap, following error {:.1} counts",
        p.name,
        s.step_deg,
        s.overshoot_deg,
        sc::overshoot_bound_deg(s.step_deg),
        1e3 * s.settle_s,
        100.0 * s.tail_driven_share,
        if s.quiet { "quiet" } else { "HUNTS" },
        s.rest_error_deg,
        1e3 * s.current_peak_a,
        1e3 * cap_counts / u.counts_per_a,
        s.follow_peak_counts
    )
}

/// The wide-code holds' line: the codes held, the ones not, and the time.
fn wide_said(b: &Before) -> String {
    if b.codes.is_empty() {
        return "wide-code holds: none, the servo's wide-code map holds no slot".into();
    }
    if b.wide.is_empty() {
        return format!(
            "wide-code holds: none, every code with a slot ({}) has an approach goal within {FIT_MARGIN_DEG} deg of the soft range",
            b.codes
                .iter()
                .map(|c| c.stuck.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let mut held_codes: Vec<u16> = b.wide.iter().map(|s| s.code.stuck).collect();
    held_codes.dedup();
    let not: Vec<String> = b
        .codes
        .iter()
        .filter(|c| !held_codes.contains(&c.stuck))
        .map(|c| c.stuck.to_string())
        .collect();
    format!(
        "wide-code holds: codes {} (of {} with a slot), each held {WIDE_HOLD_MS} ms twice at the middle of the codes it skips, after a goal {WIDE_APPROACH_COUNTS} counts above its span and one as far below; {:.1} s{}",
        held_codes
            .iter()
            .map(u16::to_string)
            .collect::<Vec<_>>()
            .join(", "),
        b.codes.len(),
        wide_s(&b.wide),
        if not.is_empty() {
            String::new()
        } else {
            format!(
                "; not held, an approach goal within {FIT_MARGIN_DEG} deg of the soft range: {}",
                not.join(", ")
            )
        }
    )
}

/// The wide-code holds' planned time, s.
pub fn wide_s(segs: &[WideSeg]) -> f64 {
    segs.iter().map(|s| s.ms as f64).sum::<f64>() / 1e3
}

/// The largest observer speed over the still holds, once every planned
/// move held still: `{"still_speed_cps": ...}`, else empty.
fn still_estimate(found: &Found, b: &Before, taken: &[Taken]) -> Value {
    let holds: Vec<&[TelFrame]> = found
        .still
        .iter()
        .map(|(_, k)| taken[*k].burst.rows.as_slice())
        .collect();
    let mut m = Map::new();
    if holds.len() == b.moves.len()
        && let Some(v) = crate::screens::still_speed_cps(&holds)
    {
        let src = format!(
            "the largest |omega_hat| over the {} still holds, {STILL_S} s each on the loop row at each self-test move's goal after its {HOLD_S} s hold, each read from {} ms in",
            holds.len(),
            crate::screens::STILL_SKIP_MS
        );
        m.insert(
            STILL_SPEED.into(),
            super::estimate(v, super::Scatter::Once(holds.len()), "count/s", &src),
        );
    }
    Value::Object(m)
}

fn finite(v: f64) -> Value {
    if v.is_finite() { v.into() } else { Value::Null }
}

#[allow(clippy::too_many_arguments)]
fn result_json(
    b: &Before,
    found: &Found,
    taken: &[Taken],
    scored: &[(Planned, Scores)],
    u: &Units,
    checks: &[Check],
    failed: &[String],
    way: Way,
    why: &Option<String>,
    exit: &guard::Exit,
    after: Option<i32>,
) -> Value {
    let moves: Vec<Value> = scored
        .iter()
        .map(|(p, s)| {
            json!({
                "name": p.name, "size_share": p.size, "from_counts": p.from, "goal_counts": p.goal, "rows_planned": p.rows,
                "step_deg": finite(s.step_deg), "overshoot_deg": finite(s.overshoot_deg),
                "overshoot_bound_deg": finite(sc::overshoot_bound_deg(s.step_deg)), "settle_s": finite(s.settle_s),
                "tail_driven_share": finite(s.tail_driven_share), "quiet": s.quiet, "rest_error_deg": finite(s.rest_error_deg),
                "current_peak_a": finite(s.current_peak_a), "current_peak_counts": finite(s.current_peak_counts),
                "current_cap_counts": b.cap_counts, "follow_peak_counts": finite(s.follow_peak_counts),
                "follow_peak_deg": finite(s.follow_peak_counts / u.counts_per_deg), "screen_counts": b.screen_counts,
            })
        })
        .collect();
    let small: Vec<Value> = found
        .small
        .iter()
        .map(|s| {
            json!({"move": s.name, "goal_counts": s.goal, "step_counts": s.step_counts, "error_start_counts": s.error_start,
                   "error_end_counts": s.error_end, "travel_counts": s.travel, "fault": s.fault, "failed": s.failed()})
        })
        .collect();
    let accepted = way == Way::Done;
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
        "units": {
            "counts": "linearized pot counts: the raw pot through the servo's wide-code slots and position table",
            "counts_per_deg": {"value": u.counts_per_deg, "source": b.scale.source},
            "counts_per_a": {"value": u.counts_per_a, "source": "the servo's nominal sense block"},
            "current_zero_counts": u.zero_counts,
            "tick_hz": u.tick_hz,
            "travel_counts": b.travel,
            "soft_counts": [b.soft.0, b.soft.1],
            "middle_counts": b.mid,
        },
        "bounds": {
            "enable_share": sc::ENABLE_SHARE, "small_share": SMALL_SHARE, "small_travel_steps": sc::SMALL_TRAVEL,
            "overshoot_deg": sc::OVERSHOOT_DEG, "overshoot_share": sc::OVERSHOOT_SHARE,
            "tail_driven_share": sc::TAIL_DRIVEN_SHARE, "follow_share_of_screen": sc::FOLLOW_SHARE,
            "rest_deg": sc::REST_DEG, "screen_counts": b.screen_counts, "watchdog_cap_counts": CAP_GAIN * b.cap_counts,
        },
        "estimates": still_estimate(found, b, taken),
        "still_holds": found.still.iter().map(|(name, k)| {
            let rows = taken[*k].burst.rows.as_slice();
            json!({"move": name, "capture": taken[*k].nn, "rows": rows.len(),
                   "still_speed_cps": crate::screens::still_speed_cps(&[rows])})
        }).collect::<Vec<_>>(),
        "wide_codes": {
            "codes": b.codes.iter().map(|c| json!({"stuck_code": c.stuck, "skipped": c.skipped,
                "held": b.wide.iter().any(|s| s.code == *c)})).collect::<Vec<_>>(),
            "hold_ms": WIDE_HOLD_MS, "approach_counts": WIDE_APPROACH_COUNTS, "fit_margin_deg": FIT_MARGIN_DEG,
            "planned_s": wide_s(&b.wide),
            "segments": b.wide.iter().enumerate().map(|(k, s)| json!({"test": s.test(), "stuck_code": s.code.stuck,
                "side": s.side.name(), "raw_counts": s.raw, "goal_counts": s.goal,
                "goal_deg": (s.goal as f64 - b.mid) / u.counts_per_deg, "ms": s.ms,
                "captures": found.wide.get(k).map_or(Vec::new(), |(_, at)| taken[at.clone()].iter().map(|t| t.nn).collect())})).collect::<Vec<_>>(),
        },
        "enable_goal_counts": found.goal0,
        "small_steps": small,
        "moves": moves,
        "fault": found.fault,
        "fault_on": found.fault_on,
        "checks": checks.iter().map(Check::json).collect::<Vec<_>>(),
        "position_before_counts": b.pos,
        "position_centered_counts": found.centered,
        "position_after_counts": after,
        "exit": exit.lines,
        "torque_off": exit.torque_off,
    })
}
