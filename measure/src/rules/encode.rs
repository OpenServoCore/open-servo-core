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

/// `v` at the firmware's own bound where it is past one; a value past its
/// storage alone stays as it is.
pub fn held(rule: &Rule, v: i64) -> i64 {
    let (lo, hi) = rule.format.bounds();
    match (rule.format.min, rule.format.max) {
        (Some(_), _) if v < lo => lo,
        (_, Some(_)) if v > hi => hi,
        _ => v,
    }
}

/// `x` as the field holds it, with a line where it is stored at the
/// firmware's own bound, the nearest value the field takes. Refused when
/// it is no number or its storage alone cannot hold it: a gain past its
/// width would be another loop, not the nearest one.
pub fn encode(rule: &Rule, x: f64) -> Result<(i64, Option<String>), Fail> {
    let (lo, hi) = rule.format.bounds();
    if !x.is_finite() {
        return Err(Fail::refused(format!(
            "{}: {} gives {x} {}, which no field holds",
            rule.field, rule.name, rule.unit
        )));
    }
    let v = rounded(rule, x);
    let end = held(rule, v);
    if end != v {
        return Ok((
            end,
            Some(format!(
                "{} gives {x} {}, {v} as the field holds it, past the firmware's {lo}..{hi}: stored as {end}",
                rule.name, rule.unit
            )),
        ));
    }
    if !(lo..=hi).contains(&v) {
        return Err(Fail::refused(format!(
            "{}: {} gives {x} {}, {v} as the field holds it, which does not fit its {lo}..{hi}",
            rule.field, rule.name, rule.unit
        )));
    }
    Ok((v, None))
}
