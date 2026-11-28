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
use crate::fit::stopcontact;
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

/// The narrowest soft window, as a multiple of the hold margin K
/// ([`stopcontact::hold_margin_deg`]): one K either side of the window's
/// center, the room each soft limit keeps from its stop. A width under it,
/// a typo of 0 among them, would put both limits on the center, where a goal
/// has no travel to land in.
pub const SOFT_FLOOR_MARGINS: f64 = 2.0;

/// The soft window in counts: its middle, its half width, and what it was
/// narrowed or widened from where the stated width does not fit.
struct Window {
    mid: f64,
    half: f64,
    said: Option<String>,
}

/// The soft window (oscnb `constants.soft_window`): centered between the
/// two first contacts by the ripple, its width W the policy's
/// soft_travel_deg where it states one, else the profile's rated travel,
/// clamped to at most the physical travel P less the hold margin K at each
/// end and to at least [`SOFT_FLOOR_MARGINS`] x K. The degrees are the
/// table's, whose counts are linear in ripple cycles, so a half width
/// reaches counts only through the table. A clamp is said, never refused;
/// stops with no room for a window at all are refused.
fn window(c: &mut Ctx) -> Result<Window, Fail> {
    let (asked, whose) = match c.inputs.policy.soft_travel_deg {
        Some(_) => (
            get!(c, policy, soft_travel_deg)?,
            "the policy's soft_travel_deg",
        ),
        None => (get!(c, nameplate, rated_travel_deg)?, "the rated travel"),
    };
    let s = get!(c, measured, stops)?;
    let [lo, hi] = s
        .contact_counts
        .ok_or_else(|| Fail::missing("measured.stops.contact_counts"))?;
    let cpd = counts_per_deg(c)?;
    let events = get!(c, nameplate, brushes)? * get!(c, nameplate, rotor_segments)?;
    let gear = get!(c, nameplate, gear_ratio)?;
    let margin = stopcontact::hold_margin_deg(stopcontact::deg_per_cycle(events, gear));
    let physical = (hi - lo) / cpd;
    let (most, least) = (physical - 2.0 * margin, SOFT_FLOOR_MARGINS * margin);
    if most <= 0.0 {
        return Err(Fail::refused(format!(
            "the stops are {physical:.2} deg apart, no wider than {margin:.2} deg at each end: there is no room for a soft window"
        )));
    }
    let width = asked.max(least).min(most);
    let said = (width != asked).then(|| {
        let why = if asked > most {
            format!("over the stops' {physical:.2} deg less {margin:.2} deg at each end")
        } else {
            format!("under the least window, {least:.2} deg")
        };
        format!(
            "soft limits: {whose}, {asked} deg, is {why}: the window is clamped to {width:.2} deg, centered on the stops"
        )
    });
    Ok(Window {
        mid: (lo + hi) / 2.0,
        half: width / 2.0 * cpd,
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
