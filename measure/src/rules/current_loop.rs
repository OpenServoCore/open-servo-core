//! The current loop (ident gains.rs synthesize): the proportional gain
//! cancels the winding's electrical pole, w x L, the integral gain sets
//! the crossover, w x R a FAST tick, the anti-windup gain is twice the
//! integral one; w is the policy's bandwidth. And the duty cap.

use osc_servo_core::regions::config::DEFAULT_DUTY_MAX_Q15;

use super::units::{r_vcounts, share, shunt_counts_per_a, tap_counts_per_v};
use super::{Ctx, Fail, Format, Kind, Rule, board, get, q, table};

fn loop_rad_s(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(2.0 * core::f64::consts::PI * get!(c, policy, current_loop_hz)?)
}

fn i_kp(c: &mut Ctx) -> Result<f64, Fail> {
    let w = loop_rad_s(c)?;
    let l = q!(c, measured, l_mh)?;
    Ok(w * l / 1000.0 * tap_counts_per_v(c)? / shunt_counts_per_a(c)?)
}

fn i_ki(c: &mut Ctx) -> Result<f64, Fail> {
    let w = loop_rad_s(c)?;
    Ok(w * r_vcounts(c)? / board!(c, tick_hz))
}

fn i_kaw(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(2.0 * i_ki(c)?)
}

/// The policy's share of the rail; 1 is no cap.
fn duty_max(c: &mut Ctx) -> Result<f64, Fail> {
    let s = share("policy.duty_max_share", get!(c, policy, duty_max_share)?)?;
    Ok(s * DEFAULT_DUTY_MAX_Q15 as f64)
}

const UNIT_KI: &str = "vcounts/ccount per tick";
const DUTY: Format = Format::within(Kind::U16, None, Some(DEFAULT_DUTY_MAX_Q15 as i64));

pub const RULES: &[Rule] = table![
    i_kp_q88: "i_kp.bandwidth", Format::q(Kind::U16, 8), "vcounts/ccount", Policy, i_kp;
    i_ki_q412: "i_ki.bandwidth", Format::q(Kind::U16, 12), UNIT_KI, Policy, i_ki;
    i_kaw_q412: "i_kaw.bandwidth", Format::q(Kind::U16, 12), UNIT_KI, Policy, i_kaw;
    duty_max_q15: "duty_max.policy", DUTY, "Q15 of the rail", Policy, duty_max;
];
