//! The motor block: the winding, the back-EMF constant, the inertia and
//! the friction line, in the kernel's counts; the window term; the brush
//! term, held off; and the drive's sign.

use super::units::{
    ALPHA_CU, b_gain, counts_per_motor_rad, ke_v_s_per_rad, ke_vcounts, ma_to_counts, r_vcounts,
    shunt_counts_per_a, tap_counts_per_v,
};
use osc_servo_core::estimator::thermal::R_REF_CC;

use super::{Ctx, Fail, Format, Kind, Rule, get, q, table};
use crate::policy::Switch;

/// Where the firmware states the resistance (core estimator/thermal.rs
/// `R_REF_CC`).
pub const R_AT_C: f64 = R_REF_CC as f64 / 100.0;

/// `r_q12`: the winding at 25 C, the inverse of the kernel's
/// `r(T) = r_q12 x (1 + ALPHA x (T - 25 C))` at the board temperature it
/// was read at.
fn r_at_25(c: &mut Ctx) -> Result<f64, Fail> {
    let r = r_vcounts(c)?;
    let t = q!(c, measured, r_board_c)?;
    Ok(r / (1.0 + ALPHA_CU * (t - R_AT_C)))
}

fn recip_ke(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(1.0 / ke_vcounts(c)?)
}

fn ke_vpc(c: &mut Ctx) -> Result<f64, Fail> {
    ke_vcounts(c)
}

fn ke_uvs(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(ke_v_s_per_rad(c)? * 1e6)
}

fn b_i(c: &mut Ctx) -> Result<f64, Fail> {
    b_gain(c)
}

fn j_ff(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(1.0 / b_gain(c)?)
}

// The friction line is the current the velocity loop holds in a cruise,
// with the window term off its reading: the line the feedforward has to
// match for the integrator to sit at zero. It is in current, so no Ke
// enters, and its slope is per motor rad/s, so it holds on any table.
fn fric_fc(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(q!(c, measured, cruise_coulomb_ma)? * 1e-3 * shunt_counts_per_a(c)?)
}

fn fric_fv(c: &mut Ctx) -> Result<f64, Fail> {
    let iv = q!(c, measured, cruise_viscous_ma_s_per_rad)?;
    Ok(iv * 1e-3 * shunt_counts_per_a(c)? / counts_per_motor_rad(c)?)
}

fn fric_breakaway(c: &mut Ctx) -> Result<f64, Fail> {
    let ma = q!(c, measured, breakaway_ma)?;
    ma_to_counts(c, ma)
}

fn g_window(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(q!(c, measured, g_window_ms)? * 1e-3 * shunt_counts_per_a(c)? / tap_counts_per_v(c)?)
}

/// The brush term of the back-EMF close, off while the policy holds it
/// off: 0 turns it off (core regions/calib.rs CalibMotor).
fn brush_off(c: &mut Ctx) -> Result<f64, Fail> {
    match get!(c, policy, brush_term)? {
        Switch::Off => Ok(0.0),
        Switch::On => Err(Fail::refused(
            "policy.brush_term is on: a size of the brush term has no rule yet",
        )),
    }
}

fn drive_polarity(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(if get!(c, measured, drive_polarity)?.value {
        1.0
    } else {
        0.0
    })
}

const U16: Kind = Kind::U16;

pub const RULES: &[Rule] = table![
    r_q12: "r.at_25c", Format::q(U16, 12), "vcounts/ccount", Measured, r_at_25;
    recip_ke_q: "recip_ke", Format::q(U16, 10), "(count/s)/vcount", Measured, recip_ke;
    b_i_q313: "b_i", Format::q(U16, 13), "(count/s)/tick per ccount", Measured, b_i;
    fric_fc_counts: "fric_fc.cruise", Format::of(U16), "ccounts", Measured, fric_fc;
    fric_fv_q016: "fric_fv.cruise", Format::q(U16, 16), "ccounts/(count/s)", Measured, fric_fv;
    ke_vpc_q: "ke_vpc", Format::q(U16, 12), "vcounts/(count/s)", Measured, ke_vpc;
    r_brush_q12: "r_brush.off", Format::q(U16, 12), "vcounts/ccount", Policy, brush_off;
    brush_cps: "brush_cps.off", Format::of(U16), "count/s", Policy, brush_off;
    g_window_q016: "g_window.onsets", Format::q(U16, 16), "ccounts/vcount", Measured, g_window;
    ke_uvs_per_rad: "ke_uvs", Format::of(U16), "uV s/rad", Measured, ke_uvs;
    fric_breakaway_counts: "fric_breakaway.ramp", Format::of(U16), "ccounts", Measured,
        fric_breakaway;
    j_ff_q88: "j_ff", Format::q(U16, 8), "ccounts/((count/s)/tick)", Measured, j_ff;
    drive_polarity: "drive_polarity", Format::of(Kind::Bool), "flag", Measured, drive_polarity;
];
