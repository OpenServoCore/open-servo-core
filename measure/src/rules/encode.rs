//! The one encoder: a rule's value in the field's unit to the integer the
//! field holds.

use super::{Fail, Rounding, Rule};

/// `x` scaled to the field's fraction bits and rounded, half away from
/// zero, a derated cap down; no range check.
pub fn rounded(rule: &Rule, x: f64) -> i64 {
    let y = x * (1u64 << rule.format.q) as f64;
    let r = match rule.rounding {
        Rounding::Down => y.floor(),
        Rounding::Nearest => (y.abs() + 0.5).floor().copysign(y),
    };
    r as i64
}

/// `x` as the field holds it, refused, never clamped, when it does not
/// fit the field's storage or the firmware's own bounds.
pub fn encode(rule: &Rule, x: f64) -> Result<i64, Fail> {
    let (lo, hi) = rule.format.bounds();
    if !x.is_finite() {
        return Err(Fail::refused(format!(
            "{}: {} gives {x} {}, which no field holds",
            rule.field, rule.name, rule.unit
        )));
    }
    let v = rounded(rule, x);
    if !(lo..=hi).contains(&v) {
        return Err(Fail::refused(format!(
            "{}: {} gives {x} {}, {v} as the field holds it, which does not fit its {lo}..{hi}",
            rule.field, rule.name, rule.unit
        )));
    }
    Ok(v)
}
