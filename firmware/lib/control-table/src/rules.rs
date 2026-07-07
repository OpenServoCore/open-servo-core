//! Shared field-rule bodies. The Block derive emits one call per rule instead
//! of expanding the load/widen/compare sequence at every field site — the
//! per-site cost is a handful of argument moves, and exactly one compare body
//! exists no matter how many rules or shapes the table grows (width, sign,
//! abs, and op arrive packed in a const-folded [`spec`] word; the runtime
//! dispatch is a couple of compares on the µs-budget write path). Widening and
//! saturating-abs semantics are pinned here (carried over from the deleted
//! rule interpreter) and unit-tested directly instead of per generated copy.

use crate::map::View;
use crate::{Error, ValidationKind};

pub const OP_LT: u8 = 0;
pub const OP_LE: u8 = 1;
pub const OP_GT: u8 = 2;
pub const OP_GE: u8 = 3;
pub const OP_EQ: u8 = 4;
pub const OP_NE: u8 = 5;

const SPEC_SIGNED: u16 = 1 << 4;
const SPEC_ABS: u16 = 1 << 5;

/// Rhs flavor bit in a spec word: set = `val` names another register.
pub const SPEC_RHS_REG: u16 = 1 << 6; // must not collide: width [3:0], signed 4, abs 5, op [15:8]

/// One compare rule as flash data: `addr` is table-absolute after section
/// concat, `spec` is `spec(..)` (possibly | SPEC_RHS_REG), `val` is the
/// immediate's i32 bits or the RHS register's table-absolute address.
#[derive(Copy, Clone)]
pub struct CmpRule {
    pub addr: u16,
    pub spec: u16,
    pub val: u32,
}

/// One enum/bool rule as flash data (field width is always 1).
#[derive(Copy, Clone)]
pub struct AllowedRule {
    pub addr: u16,
    pub allowed: &'static [u8],
}

/// Pack a compare rule's shape into one word: width in bits [3:0], signed at
/// bit 4, abs at bit 5, op in bits [15:8]. Const-folded at every call site.
pub const fn spec(width: u8, signed: bool, abs: bool, op: u8) -> u16 {
    width as u16
        | if signed { SPEC_SIGNED } else { 0 }
        | if abs { SPEC_ABS } else { 0 }
        | (op as u16) << 8
}

/// Compare right-hand side: an immediate from the rule expression, or another
/// register loaded with the SAME width + signedness as the field (pinned).
#[derive(Copy, Clone)]
pub enum Rhs {
    Imm(i32),
    Reg(u16),
}

/// Enum/bool rule: the candidate byte must be one of `allowed`.
#[inline(never)]
pub fn check_allowed(view: &View, addr: u16, allowed: &[u8]) -> Result<(), Error> {
    let b = view.read_fixed(addr, 1)?;
    if allowed.contains(&b[0]) {
        Ok(())
    } else {
        Err(Error::ValidationError(ValidationKind::Enum))
    }
}

/// Load `[addr, addr+width)` widened to `i32` with per-width sign extension.
fn widen(view: &View, addr: u16, width: u8, signed: bool) -> Result<i32, Error> {
    let b = view.read_fixed(addr, width as usize)?;
    Ok(match (width, signed) {
        (1, false) => b[0] as i32,
        (1, true) => b[0] as i8 as i32,
        (2, false) => u16::from_le_bytes([b[0], b[1]]) as i32,
        (2, true) => i16::from_le_bytes([b[0], b[1]]) as i32,
        _ => i32::from_le_bytes(b),
    })
}

/// Compare rule; `spec` comes from [`spec`]. With abs, both sides take a
/// saturating absolute value clamped to the field width's positive max.
#[inline(never)]
pub fn check_cmp(view: &View, addr: u16, spec: u16, rhs: Rhs) -> Result<(), Error> {
    let width = (spec & 0xF) as u8;
    let signed = spec & SPEC_SIGNED != 0;
    let mut a = widen(view, addr, width, signed)?;
    let mut r = match rhs {
        Rhs::Imm(v) => v,
        Rhs::Reg(reg) => widen(view, reg, width, signed)?,
    };
    if spec & SPEC_ABS != 0 {
        let sat_max = match width {
            1 => i8::MAX as i32,
            2 => i16::MAX as i32,
            _ => i32::MAX,
        };
        a = a.saturating_abs().min(sat_max);
        r = r.saturating_abs().min(sat_max);
    }
    let ok = match (spec >> 8) as u8 {
        OP_LT => a < r,
        OP_LE => a <= r,
        OP_GT => a > r,
        OP_GE => a >= r,
        OP_EQ => a == r,
        _ => a != r,
    };
    if ok {
        Ok(())
    } else {
        Err(Error::ValidationError(ValidationKind::Compare))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn view_over(storage: &[u8]) -> View<'_> {
        View::new(storage.as_ptr(), storage.len(), 0, &[])
    }

    fn cmp(v: &View, width: u8, signed: bool, abs: bool, op: u8, rhs: Rhs) -> Result<(), Error> {
        check_cmp(v, 0, spec(width, signed, abs, op), rhs)
    }

    #[test]
    fn allowed_accepts_and_rejects() {
        let s = [1u8, 7];
        let v = view_over(&s);
        assert!(check_allowed(&v, 0, &[0, 1]).is_ok());
        assert!(matches!(
            check_allowed(&v, 1, &[0, 1]),
            Err(Error::ValidationError(ValidationKind::Enum))
        ));
    }

    #[test]
    fn widen_signs_per_width() {
        let s = [0xFFu8, 0xFF, 0xFF, 0xFF];
        let v = view_over(&s);
        assert_eq!(widen(&v, 0, 1, false), Ok(255));
        assert_eq!(widen(&v, 0, 1, true), Ok(-1));
        assert_eq!(widen(&v, 0, 2, false), Ok(65535));
        assert_eq!(widen(&v, 0, 2, true), Ok(-1));
        assert_eq!(widen(&v, 0, 4, true), Ok(-1));
    }

    #[test]
    fn cmp_ops_fold_correctly() {
        let s = 5i16.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, true, false, OP_LE, Rhs::Imm(5)).is_ok());
        assert!(cmp(&v, 2, true, false, OP_LT, Rhs::Imm(5)).is_err());
        assert!(cmp(&v, 2, true, false, OP_GE, Rhs::Imm(5)).is_ok());
        assert!(cmp(&v, 2, true, false, OP_NE, Rhs::Imm(4)).is_ok());
        assert!(cmp(&v, 2, true, false, OP_EQ, Rhs::Imm(4)).is_err());
    }

    #[test]
    fn cmp_reg_rhs_loads_same_width() {
        let mut s = [0u8; 4];
        s[..2].copy_from_slice(&10i16.to_le_bytes());
        s[2..].copy_from_slice(&20i16.to_le_bytes());
        let v = view_over(&s);
        assert!(cmp(&v, 2, true, false, OP_LE, Rhs::Reg(2)).is_ok());
        assert!(cmp(&v, 2, true, false, OP_GT, Rhs::Reg(2)).is_err());
    }

    #[test]
    fn abs_saturates_at_width_max() {
        // i8::MIN's plain abs would overflow; the pinned semantics saturate,
        // then clamp to the width's positive max.
        let s = [0x80u8]; // -128
        let v = view_over(&s);
        assert!(cmp(&v, 1, true, true, OP_LE, Rhs::Imm(127)).is_ok());
        assert!(cmp(&v, 1, true, true, OP_GT, Rhs::Imm(126)).is_ok());
        // i16::MIN likewise clamps to i16::MAX.
        let s = i16::MIN.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, true, true, OP_EQ, Rhs::Imm(i16::MAX as i32)).is_ok());
    }

    #[test]
    fn overlay_is_visible_to_rules() {
        let s = [0u8, 0];
        let pending = [0xFFu8, 0x7F]; // i16::MAX about to be committed
        let v = View::new(s.as_ptr(), s.len(), 0, &pending);
        assert!(cmp(&v, 2, true, false, OP_GE, Rhs::Imm(1000)).is_ok());
    }
}
