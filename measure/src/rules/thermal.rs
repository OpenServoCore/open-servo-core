//! The winding estimate: the board NTC's curve, from the board's
//! description, plus two heat nodes, each a leak in seconds and a rise in
//! centi-C per count squared of its heat (core estimator/thermal.rs). The
//! nodes are entered by the user: a node's time constant and its gain in
//! C/W. Only the rise derives by rule, through the winding's resistance as
//! identified and the shunt's counts per A. A node is entered whole, and
//! with the NTC on the can the NTC is the body node, so the body node's
//! words bind 0 whatever the profile states, and the set says so.

use super::units::shunt_counts_per_a;
use super::{Ctx, Fail, Format, Kind, Outcome, Rule, Set, board, q, table};
use crate::board::NtcSite;
use crate::inputs::Inputs;

/// The slow node: the user's, none on a board whose NTC sits on the can.
fn body_node(c: &mut Ctx) -> Result<bool, Fail> {
    Ok(board!(c, ntc_site) == NtcSite::Air)
}

/// On a board whose NTC sits on the can, the body node's words, bound 0,
/// say that the slow node the user entered is not used.
pub(super) fn can_body(set: &mut Set, inputs: &Inputs) {
    let t = &inputs.thermal;
    if inputs.board.ntc_site != NtcSite::Can
        || (t.slow_tau_s.is_none() && t.slow_gain_c_per_w.is_none())
    {
        return;
    }
    for (name, o) in &mut set.fields {
        if let ("thermal_tau2_s" | "th_rise2_q016", Outcome::Bound(e)) = (*name, o) {
            e.clamp = Some(
                "0: the board's NTC sits on the can, so it is the body node; the profile's slow node is not used"
                    .into(),
            );
        }
    }
}

/// A node's two inputs come together or not at all.
fn whole(
    c: &Ctx,
    tau: fn(&Inputs) -> bool,
    gain: fn(&Inputs) -> bool,
    node: &str,
) -> Result<(), Fail> {
    if tau(c.inputs) == gain(c.inputs) {
        Ok(())
    } else {
        Err(Fail::refused(format!(
            "thermal: the {node} node is entered whole, its time constant with its gain"
        )))
    }
}

fn fast_whole(c: &Ctx) -> Result<(), Fail> {
    whole(
        c,
        |i| i.thermal.fast_tau_s.is_some(),
        |i| i.thermal.fast_gain_c_per_w.is_some(),
        "fast",
    )
}

fn slow_whole(c: &Ctx) -> Result<(), Fail> {
    whole(
        c,
        |i| i.thermal.slow_tau_s.is_some(),
        |i| i.thermal.slow_gain_c_per_w.is_some(),
        "slow",
    )
}

/// A gain, C/W, as a stored rise: 100 x G x R / (counts per A)^2, the
/// copper loss at the winding's resistance with the current in counts. A
/// gain over 0 that rounds to a rise of 0 is refused: 0 turns the node off.
fn rise(c: &mut Ctx, field: &str, gain: f64) -> Result<f64, Fail> {
    let r = q!(c, measured, r_ohm)?;
    let x = 100.0 * gain * r / shunt_counts_per_a(c)?.powf(2.0);
    if gain > 0.0 && (x * f64::from(1u32 << 16)).round() == 0.0 {
        return Err(Fail::refused(format!(
            "{field}: {gain} C/W over {r:.2} ohm is under the field's least step, so the node would be off; the gain is too small for this board's current scale"
        )));
    }
    Ok(x)
}

fn thermal_tau(c: &mut Ctx) -> Result<f64, Fail> {
    fast_whole(c)?;
    q!(c, thermal, fast_tau_s)
}

fn th_rise(c: &mut Ctx) -> Result<f64, Fail> {
    fast_whole(c)?;
    let g = q!(c, thermal, fast_gain_c_per_w)?;
    rise(c, "th_rise_q016", g)
}

fn thermal_tau2(c: &mut Ctx) -> Result<f64, Fail> {
    if !body_node(c)? {
        return Ok(0.0);
    }
    slow_whole(c)?;
    q!(c, thermal, slow_tau_s)
}

fn th_rise2(c: &mut Ctx) -> Result<f64, Fail> {
    if !body_node(c)? {
        return Ok(0.0);
    }
    slow_whole(c)?;
    let g = q!(c, thermal, slow_gain_c_per_w)?;
    rise(c, "th_rise2_q016", g)
}

fn ntc_raw_ref(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(board!(c, ntc_curve).raw_ref)
}

fn ntc_t_ref(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(board!(c, ntc_curve).t_ref_cc)
}

fn ntc_k1(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(board!(c, ntc_curve).k1_q88 / (1u32 << 8) as f64)
}

fn ntc_k2(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(board!(c, ntc_curve).k2_q24 / (1u32 << 24) as f64)
}

const U16: Kind = Kind::U16;
const RISE: &str = "cC/count^2";

pub const RULES: &[Rule] = table![
    thermal_tau_s: "thermal_tau.user", Format::of(U16), "s", User, thermal_tau;
    thermal_tau2_s: "thermal_tau2.user", Format::of(U16), "s", User, thermal_tau2;
    th_rise_q016: "th_rise.user_gain", Format::q(U16, 16), RISE, User, th_rise;
    th_rise2_q016: "th_rise2.user_gain", Format::q(U16, 16), RISE, User, th_rise2;
    ntc_raw_ref: "ntc_raw_ref.board", Format::of(U16), "counts", Board, ntc_raw_ref;
    ntc_t_ref_cc: "ntc_t_ref.board", Format::of(Kind::I16), "cC", Board, ntc_t_ref;
    ntc_k1_q88: "ntc_k1.board", Format::q(Kind::I16, 8), "cC/count", Board, ntc_k1;
    ntc_k2_q24: "ntc_k2.board", Format::q(U16, 24), RISE, Board, ntc_k2;
];
