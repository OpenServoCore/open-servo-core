//! The fields in counts of position: the stops, the soft limits, the
//! angle map and the gearing records, the move's speed and acceleration,
//! and the hold band. The kernel's estimate runs in the table's count, so
//! an angle the policy states in degrees is that many counts there. Two
//! fields are in raw pot counts whatever the table: the hold band, which
//! the kernel takes through the table's local gain itself, and the sensor
//! screen (`limits`).

use osc_servo_core::kernel::phases::loops::trajectory::GOAL_LIM_COUNTS;

use super::units::{counts_per_deg, deg_to_counts, medium_hz};
use super::{Ctx, Fail, Format, Kind, Outcome, Rule, Set, get, q, table};
use crate::inputs::Inputs;

fn stops(c: &mut Ctx) -> Result<(f64, f64), Fail> {
    let s = get!(c, measured, stops)?;
    Ok((s.raw_min as f64, s.raw_max as f64))
}

fn raw_min(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(stops(c)?.0)
}

fn raw_max(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(stops(c)?.1)
}

/// The least a soft limit sits inside its stop, deg: the stop's drift over
/// days of sittings plus the hold band.
const STOP_CLEARANCE_DEG: f64 = 1.0;

/// The motor cycles over which the rotor slows into a stop: the clearance
/// is no less than 1.25 times their degrees, which on a low-ratio gear are
/// more than [`STOP_CLEARANCE_DEG`].
const CONTACT_CYCLES: f64 = 4.0;

/// The narrowest soft window the run sets, deg: a narrower one is stored
/// this wide.
pub const SOFT_TRAVEL_FLOOR_DEG: f64 = 1.0;

/// The soft window in counts: its middle, its half width, and what it was
/// narrowed or widened from where the stated width does not fit.
struct Window {
    mid: f64,
    half: f64,
    said: Option<String>,
}

/// The soft window: the policy's width, else the profile's rated travel,
/// centered between the stops, at least [`SOFT_TRAVEL_FLOOR_DEG`] and at
/// most the stops' travel less the clearance inside each; refused when the
/// stops leave no room for the floor.
fn window(c: &mut Ctx) -> Result<Window, Fail> {
    let width = match c.inputs.policy.soft_travel_deg {
        Some(_) => get!(c, policy, soft_travel_deg)?,
        None => get!(c, nameplate, rated_travel_deg)?,
    };
    let (lo, hi) = stops(c)?;
    let cpd = counts_per_deg(c)?;
    let events = (get!(c, nameplate, brushes)? * get!(c, nameplate, rotor_segments)?) as f64;
    let gear = get!(c, nameplate, gear_ratio)?;
    let clearance = STOP_CLEARANCE_DEG.max(1.25 * CONTACT_CYCLES * 360.0 / (events * gear));
    let travel = (hi - lo) / cpd;
    let most = travel - 2.0 * clearance;
    if most < SOFT_TRAVEL_FLOOR_DEG {
        return Err(Fail::refused(format!(
            "the stops are {travel:.1} deg apart: no soft window of {SOFT_TRAVEL_FLOOR_DEG} deg fits {clearance:.1} deg inside each"
        )));
    }
    let set = width.clamp(SOFT_TRAVEL_FLOOR_DEG, most);
    let said = if width > most {
        Some(format!(
            "the stops are {travel:.1} deg apart, under the {width} deg soft travel plus {clearance:.1} deg at each end: clamped to {set:.1} deg, centered"
        ))
    } else if width < SOFT_TRAVEL_FLOOR_DEG {
        Some(format!(
            "the {width} deg soft travel is under the {SOFT_TRAVEL_FLOOR_DEG} deg floor: clamped to {set:.1} deg, centered"
        ))
    } else {
        None
    };
    Ok(Window {
        mid: (lo + hi) / 2.0,
        half: set / 2.0 * cpd,
        said,
    })
}

/// A soft limit: an end of the soft window (protocol sec 5.8).
fn soft(c: &mut Ctx, high: bool) -> Result<f64, Fail> {
    let w = window(c)?;
    Ok(if high { w.mid + w.half } else { w.mid - w.half })
}

/// Each soft limit of a window narrowed or widened from its stated width
/// says so.
pub(super) fn say_soft(set: &mut Set, inputs: &Inputs) {
    let Ok(Window {
        said: Some(said), ..
    }) = window(&mut Ctx::new(inputs))
    else {
        return;
    };
    for (name, outcome) in &mut set.fields {
        if let ("pos_min_soft_counts" | "pos_max_soft_counts", Outcome::Bound(e)) = (*name, outcome)
        {
            e.clamp = Some(match e.clamp.take() {
                Some(was) => format!("{was}; {said}"),
                None => said.clone(),
            });
        }
    }
}

fn soft_min(c: &mut Ctx) -> Result<f64, Fail> {
    soft(c, false)
}

fn soft_max(c: &mut Ctx) -> Result<f64, Fail> {
    soft(c, true)
}

fn travel_from_stops_deg(c: &mut Ctx) -> Result<f64, Fail> {
    let (lo, hi) = stops(c)?;
    Ok((hi - lo) / counts_per_deg(c)?)
}

/// The angle map, centi-deg at each stop, rising counterclockwise facing
/// the horn: the clockwise stop at the policy's angle, the other one the
/// travel past it, so a host that divides the map's span by the stops'
/// gets the table's counts per deg.
fn angle(c: &mut Ctx, at_high: bool) -> Result<f64, Fail> {
    let cw = get!(c, policy, angle_cw_stop_deg)?;
    let ccw_high = get!(c, nameplate, count_rises_ccw)?;
    let past = if ccw_high == at_high {
        travel_from_stops_deg(c)?
    } else {
        0.0
    };
    Ok(100.0 * (cw + past))
}

fn angle_min(c: &mut Ctx) -> Result<f64, Fail> {
    angle(c, false)
}

fn angle_max(c: &mut Ctx) -> Result<f64, Fail> {
    angle(c, true)
}

fn gear_ratio(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(100.0 * get!(c, nameplate, gear_ratio)?)
}

fn gear_class(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(get!(c, nameplate, gear_class)?.byte() as f64)
}

fn velocity_limit(c: &mut Ctx) -> Result<f64, Fail> {
    let deg = get!(c, policy, velocity_limit_deg_s)?;
    deg_to_counts(c, deg)
}

/// The speed one MEDIUM period adds.
fn accel_limit(c: &mut Ctx) -> Result<f64, Fail> {
    let deg = get!(c, policy, accel_limit_deg_s2)?;
    Ok(deg_to_counts(c, deg)? / medium_hz(c))
}

/// The hold band: the policy's multiple of the pot's noise at rest, no
/// less than its floor, so a quiet pot cannot make it too tight to hold.
fn deadband(c: &mut Ctx) -> Result<f64, Fail> {
    let k = get!(c, policy, hold_band_noise_multiple)?;
    let sd = q!(c, measured, pot_rest_sd_counts)?;
    Ok((k * sd).max(get!(c, policy, hold_band_floor_counts)?))
}

const SOFT: Format = Format::within(
    Kind::I32,
    Some(-(GOAL_LIM_COUNTS as i64)),
    Some(GOAL_LIM_COUNTS as i64),
);
const COUNTS: &str = "counts";

pub const RULES: &[Rule] = table![
    pos_min_phys_counts: "stops.low", Format::of(Kind::I32), COUNTS, Measured, raw_min;
    pos_max_phys_counts: "stops.high", Format::of(Kind::I32), COUNTS, Measured, raw_max;
    pos_min_soft_counts: "soft_min.window", SOFT, COUNTS, Policy, soft_min;
    pos_max_soft_counts: "soft_max.window", SOFT, COUNTS, Policy, soft_max;
    pos_deadband_counts: "deadband.pot_noise", Format::of(Kind::U16), "raw counts", Policy,
        deadband;
    velocity_limit_cps: "velocity_limit.policy", Format::of(Kind::U16), "count/s", Policy,
        velocity_limit;
    accel_limit_q88: "accel_limit.policy", Format::q(Kind::U16, 8), "(count/s)/tick", Policy,
        accel_limit;
    raw_min: "stops.low", Format::of(Kind::U16), COUNTS, Measured, raw_min;
    raw_max: "stops.high", Format::of(Kind::U16), COUNTS, Measured, raw_max;
    angle_min_cdeg: "angle_min.travel", Format::of(Kind::I16), "cdeg", Measured, angle_min;
    angle_max_cdeg: "angle_max.travel", Format::of(Kind::I16), "cdeg", Measured, angle_max;
    gear_ratio_centi: "gear_ratio.nameplate", Format::of(Kind::U16), "x100", Nameplate, gear_ratio;
    gear_class: "gear_class.nameplate", Format::of(Kind::U8), "class", Nameplate, gear_class;
];
