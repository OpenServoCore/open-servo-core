//! The fields in counts of position: the stops, the soft limits, the
//! angle map, whose 0 deg is the servo's center, and the gearing records, the move's speed and acceleration,
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

/// The soft window's width W and what bounds it, output deg, and what it
/// was narrowed or widened from where the stated width does not fit.
struct Width {
    width: f64,
    margin: f64,
    physical: f64,
    said: Option<String>,
}

/// The soft window's width (oscnb `constants.soft_width`): the policy's
/// soft_travel_deg where it states one, else the profile's rated travel,
/// clamped to at most the physical travel P less the hold margin K at each
/// end and to at least [`SOFT_FLOOR_MARGINS`] x K. P runs between the two
/// first contacts by the ripple. A clamp is said, never refused; stops with
/// no room for a window at all are refused.
fn width(c: &mut Ctx) -> Result<Width, Fail> {
    let (asked, whose) = match c.inputs.policy.soft_travel_deg {
        Some(_) => (
            get!(c, policy, soft_travel_deg)?,
            "the policy's soft_travel_deg",
        ),
        None => (get!(c, nameplate, rated_travel_deg)?, "the rated travel"),
    };
    let [lo, hi] = contacts(c)?;
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
    Ok(Width {
        width,
        margin,
        physical,
        said,
    })
}

fn contacts(c: &mut Ctx) -> Result<[f64; 2], Fail> {
    get!(c, measured, stops)?
        .contact_counts
        .ok_or_else(|| Fail::missing("measured.stops.contact_counts"))
}

/// Where 0 deg is: the table count, the offset from the center of travel
/// and the offset asked, output deg counterclockwise, and what a clamp says.
#[derive(Clone, Debug, PartialEq)]
pub struct Center {
    pub counts: f64,
    pub offset_deg: f64,
    pub asked_offset_deg: Option<f64>,
    pub said: Option<String>,
}

/// 0 deg (oscnb `constants.center`): the midpoint of the two first
/// contacts by the ripple, in the table's counts, moved by the policy's
/// center_offset_deg where it states one. The offset is clamped so the soft
/// window, centered on 0 deg, keeps K inside each contact: 0 deg stays the
/// window's middle. A clamp is said, never refused.
fn center(c: &mut Ctx) -> Result<Center, Fail> {
    let [lo, hi] = contacts(c)?;
    let middle = (lo + hi) / 2.0;
    if c.inputs.policy.center_offset_deg.is_none() {
        return Ok(Center {
            counts: middle,
            offset_deg: 0.0,
            asked_offset_deg: None,
            said: None,
        });
    }
    let asked = get!(c, policy, center_offset_deg)?;
    let w = width(c)?;
    let room = ((w.physical - 2.0 * w.margin - w.width) / 2.0).max(0.0);
    let offset = asked.clamp(-room, room);
    let said = (offset != asked).then(|| {
        let end = if asked > 0.0 {
            "counterclockwise"
        } else {
            "clockwise"
        };
        format!(
            "center: the policy's center_offset_deg, {asked:+} deg, puts the {:.2} deg soft window past the stops less {:.2} deg at the {end} end: 0 deg is clamped to {offset:+.2} deg from the center of travel, the furthest the window fits",
            w.width, w.margin
        )
    });
    let ccw = if get!(c, nameplate, count_rises_ccw)? {
        1.0
    } else {
        -1.0
    };
    Ok(Center {
        counts: middle + ccw * offset * counts_per_deg(c)?,
        offset_deg: offset,
        asked_offset_deg: Some(asked),
        said,
    })
}

/// Where the set puts 0 deg, none where its inputs give no center.
pub fn center_of(inputs: &Inputs) -> Option<Center> {
    center(&mut Ctx::new(inputs)).ok()
}

/// A soft limit: an end of the soft window, centered on 0 deg (protocol
/// sec 5.8). The degrees are the table's, whose counts are linear in
/// ripple cycles, so a half width reaches counts only through the table.
fn soft(c: &mut Ctx, high: bool) -> Result<f64, Fail> {
    let half = width(c)?.width / 2.0 * counts_per_deg(c)?;
    let zero = center(c)?.counts;
    Ok(if high { zero + half } else { zero - half })
}

/// Each soft limit of a window narrowed or widened from its stated width
/// says so, and the soft limits and the angle map of a clamped center say
/// that.
pub(super) fn say_window(set: &mut Set, inputs: &Inputs) {
    let width = width(&mut Ctx::new(inputs)).ok().and_then(|w| w.said);
    let center = center_of(inputs).and_then(|c| c.said);
    for (name, outcome) in &mut set.fields {
        let Outcome::Bound(e) = outcome else {
            continue;
        };
        let soft = matches!(*name, "pos_min_soft_counts" | "pos_max_soft_counts");
        let angle = matches!(*name, "angle_min_cdeg" | "angle_max_cdeg");
        let says = [
            width.as_ref().filter(|_| soft),
            center.as_ref().filter(|_| soft || angle),
        ];
        for said in says.into_iter().flatten() {
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

/// An end of the angle map, centi-deg at its stop, rising counterclockwise
/// facing the horn: the stop's angle from 0 deg ([`center`]), so the map
/// crosses 0 there and a host that divides its span by the stops' gets the
/// table's counts per deg (protocol sec 5.5).
fn angle(c: &mut Ctx, at_high: bool) -> Result<f64, Fail> {
    let zero = center(c)?.counts;
    let stop = if at_high { raw_max(c)? } else { raw_min(c)? };
    let ccw = if get!(c, nameplate, count_rises_ccw)? {
        1.0
    } else {
        -1.0
    };
    Ok(100.0 * ccw * (stop - zero) / counts_per_deg(c)?)
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
    angle_min_cdeg: "angle_min.center", Format::of(Kind::I16), "cdeg", Measured, angle_min;
    angle_max_cdeg: "angle_max.center", Format::of(Kind::I16), "cdeg", Measured, angle_max;
    gear_ratio_centi: "gear_ratio.nameplate", Format::of(Kind::U16), "x100", Nameplate, gear_ratio;
    gear_class: "gear_class.nameplate", Format::of(Kind::U8), "class", Nameplate, gear_class;
];
