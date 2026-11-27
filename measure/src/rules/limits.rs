//! The protections: the two caps read off the nameplate, the press
//! detector's currents and times, the board's overcurrent trip, the
//! thermal cutoff, the screens, the supply terms; and the board's supply
//! word. A field over the field that bounds it is stored at that bound
//! here, or refused where no bound has a source.

use osc_servo_core::kernel::phases::loops::trajectory::VEL_CAP_CPS;
use osc_servo_core::regions::config::{STALL_TAU_TRIP_OFF, StallResponse as Response};

use super::units::{
    counts_per_deg, deg_to_counts, ma_to_counts, ms_to_ticks, share, shunt_counts_per_a,
};
use super::{Ctx, Fail, Format, Kind, Rule, Set, board, get, q, table};
use crate::nameplate::GearClass;
use crate::policy::{
    PotJump, StallRelease, StallResponse, StallSpeed, Switch, Yield, YieldRefusal, yield_torque,
};

/// The row the maker states at the stated voltage nearest the one the
/// servo runs on, the lower of two as near, of the rows inside the
/// operating range; with a single operating voltage, the row at it.
fn stated<T: Copy>(
    c: &mut Ctx,
    name: &'static str,
    rows: &[T],
    volts_of: fn(&T) -> f64,
) -> Result<T, Fail> {
    if rows.is_empty() {
        return Err(Fail::missing(name));
    }
    c.fact(name, ());
    let op = get!(c, nameplate, operating_v)?;
    let run = if op.min == op.max {
        None
    } else {
        Some(get!(c, nameplate, run_v)?)
    };
    crate::nameplate::stated(op, run, rows, volts_of).ok_or_else(|| Fail::missing(name))
}

/// The speed ceiling, deg/s: the stated no-load speed at the nameplate's
/// speed derate. It protects the brushes.
pub fn speed_ceiling_deg_s(c: &mut Ctx) -> Result<f64, Fail> {
    let rows = c.inputs.nameplate.no_load.clone();
    let row = stated(c, "nameplate.no_load", &rows, |r| r.at_v)?;
    let speed = row.per_deg / row.seconds;
    let [num, den] = get!(c, nameplate, derate_speed)?;
    if num == 0 || num > den {
        return Err(Fail::refused(format!(
            "nameplate.derate_speed is {num}/{den}: a derate is a ratio over 0 and at most 1"
        )));
    }
    Ok(speed * num as f64 / den as f64)
}

fn speed_ceiling(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(speed_ceiling_deg_s(c)? * counts_per_deg(c)?)
}

/// The hard peak cap, A: the stated stall voltage over the seated winding
/// as identified, at the nameplate's derate. A stated stall current never
/// binds it: the gather reports it beside V / R.
pub fn current_limit_a(c: &mut Ctx) -> Result<f64, Fail> {
    let rows = c.inputs.nameplate.stall.clone();
    let row = stated(c, "nameplate.stall", &rows, |r| r.at_v)?;
    let derate = share("nameplate.derate_other", get!(c, nameplate, derate_other)?)?;
    Ok(derate * row.at_v / q!(c, measured, r_ohm)?)
}

fn current_limit(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(current_limit_a(c)? * shunt_counts_per_a(c)?)
}

/// The yield current, A, 0 for the cap: the policy's, or a max yield
/// torque taken to current by the torque ratio, the torque over the stall
/// torque the maker states times the voltage the stall is stated at over
/// the seated winding. Two torques at one voltage cancel Kt, the gear
/// ratio and the train's efficiency, which no step measures at the horn.
fn yield_current_a(c: &mut Ctx) -> Result<f64, Fail> {
    let v = c.inputs.policy.yield_;
    let asked = match c.get("policy.yield", v)? {
        Yield::Ma(ma) => return Ok(ma / 1000.0),
        Yield::TorqueKgCm(t) => t,
    };
    let metal = get!(c, nameplate, gear_class)? == GearClass::Metal;
    let p = &c.inputs.policy;
    let (limit, above) = (p.plastic_torque_limit_kg_cm, p.above_plastic_limit);
    if !metal {
        c.fact("policy.plastic_torque_limit_kg_cm", ());
    }
    let torque = match yield_torque(metal, asked, limit, above) {
        Ok(None) => return Ok(0.0),
        Ok(Some(t)) => t,
        Err(YieldRefusal::NotKnown) => {
            return Err(Fail::missing("policy.plastic_torque_limit_kg_cm"));
        }
        Err(e) => return Err(Fail::refused(format!("policy.yield: {}", e.said()))),
    };
    let rows = c.inputs.nameplate.stall.clone();
    let row = stated(c, "nameplate.stall", &rows, |r| r.at_v)?;
    let Some(stall) = row.torque_kg_cm else {
        return Err(Fail::missing("nameplate.stall torque_kg_cm"));
    };
    Ok(torque / stall * row.at_v / q!(c, measured, r_ohm)?)
}

fn yield_current(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(yield_current_a(c)? * shunt_counts_per_a(c)?)
}

/// The stall release: the policy's, or its share of the yield current as
/// the kernel runs it, the current limit where the yield current is 0.
fn stall_release(c: &mut Ctx) -> Result<f64, Fail> {
    match get!(c, policy, stall_release)? {
        StallRelease::Ma(ma) => ma_to_counts(c, ma),
        StallRelease::ShareOfYield(s) => {
            let folds_to = yield_current_a(c)?;
            let held = if folds_to > 0.0 {
                folds_to
            } else {
                current_limit_a(c)?
            };
            Ok(s * held * shunt_counts_per_a(c)?)
        }
    }
}

fn oc_trip(c: &mut Ctx) -> Result<f64, Fail> {
    let ma = board!(c, oc_trip_ma);
    ma_to_counts(c, ma)
}

fn oc_trip_ticks(c: &mut Ctx) -> Result<f64, Fail> {
    get!(c, policy, oc_trip_samples)
}

fn stall_response(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(match get!(c, policy, stall_response)? {
        StallResponse::Fault => Response::Fault as u8 as f64,
        StallResponse::Yield => Response::Yield as u8 as f64,
    })
}

fn stall_time(c: &mut Ctx) -> Result<f64, Fail> {
    let ms = get!(c, policy, stall_time_ms)?;
    Ok(ms_to_ticks(c, ms))
}

/// The stall speed: the policy's, or its multiple of the largest speed
/// the observer reads over the still holds of a closed-loop check, what the
/// detector must clear on every MEDIUM period of a press.
fn stall_omega_max(c: &mut Ctx) -> Result<f64, Fail> {
    match get!(c, policy, stall_speed)? {
        StallSpeed::DegS(deg) => deg_to_counts(c, deg),
        StallSpeed::MultipleOfStillSpeed(m) => Ok(m * q!(c, measured, still_speed_cps)?),
    }
}

/// The collision trip, off by the policy: the field's largest value,
/// which no estimate passes.
fn stall_tau_trip(c: &mut Ctx) -> Result<f64, Fail> {
    match get!(c, policy, collision_trip)? {
        Switch::Off => Ok(STALL_TAU_TRIP_OFF as f64),
        Switch::On => Err(Fail::refused(
            "policy.collision_trip is on: a level of the trip has no rule yet",
        )),
    }
}

fn zero_brake(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(match get!(c, policy, openloop_zero_brake)? {
        Switch::Off => 0.0,
        Switch::On => 1.0,
    })
}

fn cutoff(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(100.0 * get!(c, policy, cutoff_c)?)
}

/// The rail fold's slope: the admittance of the board's source, shunt
/// counts per rail count through the measured chains.
fn rail_fold(c: &mut Ctx) -> Result<f64, Fail> {
    let shunt = shunt_counts_per_a(c)?;
    let rail = q!(c, measured, rail_counts_per_v)?;
    Ok(shunt * 1000.0 / (rail * board!(c, rail_source_mohm)))
}

fn pos_error(c: &mut Ctx) -> Result<f64, Fail> {
    let deg = get!(c, policy, pos_error_deg)?;
    deg_to_counts(c, deg)
}

fn pos_error_time(c: &mut Ctx) -> Result<f64, Fail> {
    let ms = get!(c, policy, pos_error_time_ms)?;
    Ok(ms_to_ticks(c, ms))
}

/// The pot jump: the policy's, or its multiple of the worst run of 4
/// successive RAIL-period pot steps over the run's driven rows, the
/// statistic the sensor screen tests.
fn sensor_delta(c: &mut Ctx) -> Result<f64, Fail> {
    match get!(c, policy, pot_jump)? {
        PotJump::Counts(n) => Ok(n),
        PotJump::MultipleOfStep4(m) => Ok(m * q!(c, measured, pot_step4_counts)?),
    }
}

fn sensor_bad(c: &mut Ctx) -> Result<f64, Fail> {
    get!(c, policy, sensor_bad_samples)
}

fn vdd(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(board!(c, vdd_mv))
}

const STALL: Kind = Kind::Enum(&[
    ("Fault", Response::Fault as i64),
    ("Yield", Response::Yield as i64),
]);
const U16: Kind = Kind::U16;
const CEILING: Format = Format::within(U16, None, Some(VEL_CAP_CPS as i64));

pub const RULES: &[Rule] = table![
    speed_ceiling_cps: "speed_ceiling.nameplate", CEILING, "count/s", Nameplate, speed_ceiling cap;
    current_limit_counts: "current_limit.stall_derate", Format::of(U16), "ccounts", Nameplate,
        current_limit cap;
    stall_response: "stall_response.policy", Format::of(STALL), "response", Policy, stall_response;
    stall_time_ticks: "stall_time.policy", Format::of(Kind::U32), "ticks", Policy, stall_time;
    stall_omega_max_cps: "stall_omega_max.policy", Format::of(U16), "count/s", Policy,
        stall_omega_max;
    yield_current_counts: "yield_current.policy", Format::of(U16), "ccounts", Policy,
        yield_current;
    stall_release_counts: "stall_release.policy", Format::of(U16), "ccounts", Policy, stall_release;
    stall_tau_trip_counts: "stall_tau_trip.policy", Format::of(U16), "ccounts", Policy,
        stall_tau_trip;
    oc_trip_counts: "oc_trip.board", Format::of(U16), "ccounts", Board, oc_trip;
    oc_trip_ticks: "oc_trip_ticks.policy", Format::of(Kind::U8), "samples", Policy, oc_trip_ticks;
    openloop_zero_brake: "openloop_zero_brake.policy", Format::of(Kind::Bool), "flag", Policy,
        zero_brake;
    cutoff_cc: "cutoff.policy", Format::of(Kind::I16), "cC", Policy, cutoff;
    pos_error_time_ticks: "pos_error_time.policy", Format::of(Kind::U32), "ticks", Policy,
        pos_error_time;
    pos_error_counts: "pos_error.policy", Format::of(U16), "counts", Policy, pos_error;
    sensor_delta_max: "sensor_delta.policy", Format::of(U16), "raw counts/tick", Policy,
        sensor_delta;
    sensor_bad_count: "sensor_bad.policy", Format::of(Kind::U8), "samples", Policy, sensor_bad;
    rail_fold_q8: "rail_fold.board", Format::q(U16, 8), "ccounts/vcount", Board, rail_fold;
    vdd_mv: "vdd.board", Format::of(U16), "mV", Board, vdd;
];

/// A field over the one that bounds it, where it cannot act, stored at
/// that bound and said: the move speed over the speed ceiling, and the
/// current limit over the board's overcurrent trip, which fires before the
/// limit folds. The press fold's release at or over the yield current is
/// refused, since no value under it has a source: a held press would never
/// let go, and under it the Yield response never folds. The yield current
/// 0 reads as the cap, the current limit as stored.
pub(super) fn bound_over_bounds(set: &mut Set) {
    if let (Some(v), Some(ceiling)) = (
        set.value("velocity_limit_cps"),
        set.value("speed_ceiling_cps"),
    ) && v > ceiling
    {
        set.clamp_down(
            "velocity_limit_cps",
            ceiling,
            format!(
                "the move speed gives {v} count/s, over speed_ceiling_cps {ceiling}, where it cannot act: stored as {ceiling}"
            ),
        );
    }
    if let (Some(limit), Some(trip)) = (
        set.value("current_limit_counts"),
        set.value("oc_trip_counts"),
    ) && limit > trip
    {
        set.clamp_down(
            "current_limit_counts",
            trip,
            format!(
                "the derate gives {limit} ccounts, over the board's overcurrent trip oc_trip_counts {trip}, which fires before the limit folds: stored as {trip}"
            ),
        );
    }
    if let Some(release) = set.value("stall_release_counts") {
        let held = match set.value("yield_current_counts") {
            Some(0) => set.value("current_limit_counts"),
            other => other,
        };
        match held {
            Some(held) if release < held => {}
            Some(held) => set.refuse(
                "stall_release_counts",
                format!("stall_release_counts {release} is not under the yield current {held}"),
            ),
            None => set.refuse(
                "stall_release_counts",
                "stall_release_counts: the yield current it sits under is not bound".into(),
            ),
        }
    }
}
