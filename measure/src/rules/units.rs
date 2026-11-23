//! From SI to the kernel's counts: tap counts (vcounts), shunt counts
//! (ccounts), position counts, the MEDIUM tick.
//!   R  vcounts/ccount            = R [ohm] x tap counts per V / shunt counts per A
//!   Ke vcounts/(count/s)         = Ke [V s/rad] x tap counts per V / counts per motor rad
//!   B  (count/s)/tick per ccount = Ke [N m/A] / shunt counts per A / J x counts per motor rad
//!                                  / MEDIUM rate
//! Each conversion keeps the notebook rules' order of operations, so a
//! value at a rounding edge rounds the same way.

use osc_servo_core::estimator::thermal::ALPHA_Q26;

use super::{Ctx, Fail, board, get, q};

/// FAST ticks a MEDIUM period: the kernel's `DECIM_MED`, private to its
/// scheduler (core kernel/scheduler/table.rs).
pub const FAST_TICKS_PER_MEDIUM: f64 = 10.0;

/// Copper's resistance slope, per C: the kernel's `ALPHA_Q26` (per centi-C,
/// Q26), the slope `r_q12` is defined on, so the host's conversion to 25 C
/// is the exact inverse of the kernel's reading.
pub const ALPHA_CU: f64 = ALPHA_Q26 as f64 * 100.0 / (1u64 << 26) as f64;

pub fn medium_hz(c: &mut Ctx) -> f64 {
    board!(c, tick_hz) / FAST_TICKS_PER_MEDIUM
}

pub fn tap_counts_per_v(c: &mut Ctx) -> Result<f64, Fail> {
    q!(c, measured, terminal_counts_per_v)
}

pub fn shunt_counts_per_a(c: &mut Ctx) -> Result<f64, Fail> {
    q!(c, measured, current_counts_per_a)
}

pub fn counts_per_deg(c: &mut Ctx) -> Result<f64, Fail> {
    let table = get!(c, measured, table)?;
    Ok(table.counts_per_deg)
}

pub fn counts_per_motor_rad(c: &mut Ctx) -> Result<f64, Fail> {
    let cpd = counts_per_deg(c)?;
    Ok(cpd * 180.0 / core::f64::consts::PI / get!(c, nameplate, gear_ratio)?)
}

pub fn ke_v_s_per_rad(c: &mut Ctx) -> Result<f64, Fail> {
    q!(c, measured, ke_v_s_per_rad)
}

/// The winding's resistance as identified, vcounts per ccount, at the
/// board temperature it was read at.
pub fn r_vcounts(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(q!(c, measured, r_ohm)? * tap_counts_per_v(c)? / shunt_counts_per_a(c)?)
}

/// Ke, tap counts per count/s.
pub fn ke_vcounts(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(ke_v_s_per_rad(c)? * tap_counts_per_v(c)? / counts_per_motor_rad(c)?)
}

/// B, the speed a shunt count of current adds in a MEDIUM tick.
pub fn b_gain(c: &mut Ctx) -> Result<f64, Fail> {
    let ke = ke_v_s_per_rad(c)?;
    let shunt = shunt_counts_per_a(c)?;
    let j = q!(c, measured, j_kg_m2)?;
    let cpr = counts_per_motor_rad(c)?;
    Ok(ke / shunt / j * cpr / medium_hz(c))
}

/// A length, a speed or an acceleration stated in degrees, in counts.
pub fn deg_to_counts(c: &mut Ctx, deg: f64) -> Result<f64, Fail> {
    Ok(deg * counts_per_deg(c)?)
}

/// A time stated in ms, in kernel ticks.
pub fn ms_to_ticks(c: &mut Ctx, ms: f64) -> f64 {
    ms * board!(c, tick_hz) / 1000.0
}

/// A current stated in mA, in shunt counts.
pub fn ma_to_counts(c: &mut Ctx, ma: f64) -> Result<f64, Fail> {
    Ok(ma / 1000.0 * shunt_counts_per_a(c)?)
}

/// A share, over 0 and at most 1; anything else is refused.
pub fn share(name: &str, x: f64) -> Result<f64, Fail> {
    if x > 0.0 && x <= 1.0 {
        Ok(x)
    } else {
        Err(Fail::refused(format!(
            "{name} is {x}: a share, over 0 and at most 1"
        )))
    }
}
