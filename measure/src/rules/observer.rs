//! The pot observer's gains: three coincident poles at the policy's
//! bandwidth on the MEDIUM tick (ident gains.rs synthesize); `l3` reads B.
//! And the position reading's two current terms ahead of it, tuning for
//! the load at hand, off unless the user sets them.

use osc_servo_core::regions::config::POT_COUNTS_PER_A_MAX;

use super::units::{b_gain, medium_hz};
use super::{Ctx, Fail, Format, Kind, Rule, get, table};

fn pole(c: &mut Ctx) -> Result<f64, Fail> {
    let hz = get!(c, policy, observer_hz)?;
    Ok((-2.0 * core::f64::consts::PI * hz / medium_hz(c)).exp())
}

fn l1(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(1.0 - pole(c)?.powf(3.0))
}

fn l2(c: &mut Ctx) -> Result<f64, Fail> {
    let p = pole(c)?;
    Ok(1.5 * (1.0 - p).powf(2.0) * (1.0 + p) * medium_hz(c))
}

fn l3(c: &mut Ctx) -> Result<f64, Fail> {
    let p = pole(c)?;
    Ok((1.0 - p).powf(3.0) * medium_hz(c) / (2.0 * b_gain(c)?))
}

fn pot_steady(c: &mut Ctx) -> Result<f64, Fail> {
    let v = c.inputs.tuning.pot_steady_counts_per_a;
    Ok(c.fact("tuning.pot_steady_counts_per_a", v).unwrap_or(0.0))
}

fn pot_fast(c: &mut Ctx) -> Result<f64, Fail> {
    let v = c.inputs.tuning.pot_fast_counts_per_a;
    Ok(c.fact("tuning.pot_fast_counts_per_a", v).unwrap_or(0.0))
}

const POT: Format = Format::within(Kind::U16, None, Some(POT_COUNTS_PER_A_MAX as i64));

pub const RULES: &[Rule] = table![
    l1_q016: "l1.observer", Format::q(Kind::U16, 16), "1", Policy, l1;
    l2_q88: "l2.observer", Format::q(Kind::U16, 8), "(count/s)/count", Policy, l2;
    l3_q88: "l3.observer", Format::q(Kind::U16, 8), "ccounts/count", Policy, l3;
    pot_steady_counts_per_a: "pot_steady.tuning", POT, "counts/A", Tuning, pot_steady;
    pot_fast_counts_per_a: "pot_fast.tuning", POT, "counts/A", Tuning, pot_fast;
];
