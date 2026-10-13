//! Shared field-rule bodies. The Block derive emits one call per rule instead
//! of expanding the load/widen/compare sequence at every field site -- the
//! per-site cost is a handful of argument moves, and exactly one compare body
//! exists no matter how many rules or shapes the table grows (width, sign,
//! abs, and op arrive packed in a const-folded [`spec`] word; the runtime
//! dispatch is a couple of compares on the us-budget write path). Widening and
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
/// Bitfield ops: `bits` = value is a subset of the rhs mask, `max_ones` =
/// popcount cap. Both compare raw field bits (sign/abs never apply).
pub const OP_BITS: u8 = 6;
pub const OP_MAX_ONES: u8 = 7;
/// Two-sided `lo <= value <= hi` in one rule: one field load where a `ge` +
/// `le` pair pays two. Bounds packed by [`within`].
pub const OP_IN: u8 = 8;

/// Field width in bytes, spec bits [3:0].
pub const SPEC_WIDTH: u16 = 0xF;
const SPEC_SIGNED: u16 = 1 << 4;
const SPEC_ABS: u16 = 1 << 5;

/// Rhs flavor bit in a spec word: set = `val` names another register.
pub const SPEC_RHS_REG: u16 = 1 << 6; // must not collide: width [3:0], signed 4, abs 5, op [15:8]
/// As [`SPEC_RHS_REG`] for an [`OP_IN`] rule's `hi` bound (`lo` takes
/// `SPEC_RHS_REG`).
pub const SPEC_HI_REG: u16 = 1 << 7;

/// One compare rule as flash data: `addr` is table-absolute after section
/// concat, `spec` is `spec(..)` (possibly | SPEC_RHS_REG), `val` is the
/// immediate's i32 bits or the RHS register's table-absolute address
/// ([`within`] for `OP_IN`).
#[derive(Copy, Clone)]
pub struct CmpRule {
    pub addr: u16,
    pub spec: u16,
    pub val: u32,
}

impl CmpRule {
    /// One past the field's last byte.
    pub const fn end(&self) -> usize {
        self.addr as usize + (self.spec & SPEC_WIDTH) as usize
    }
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

/// `OP_IN`'s `val`: `lo` in bits [15:0], `hi` in [31:16], each an immediate
/// or a register address. A bound outside i16 fails the build.
pub const fn within(lo: i32, hi: i32) -> u32 {
    assert!(
        lo as i16 as i32 == lo && hi as i16 as i32 == hi,
        "within bound must fit i16; use a ge + le pair"
    );
    lo as u16 as u32 | (hi as u32) << 16
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

/// Bits above a `width`-byte value in an i32: the shift that sign-extends
/// it, and `i32::MAX >> width_shift` is the width's positive max.
const fn width_shift(width: u8) -> u32 {
    32u32.wrapping_sub(8 * width as u32)
}

/// Load `[addr, addr+width)` widened to `i32` with per-width sign extension.
/// `read_fixed` zero-pads past `width`, so only signed loads shift.
fn widen(view: &View, addr: u16, width: u8, signed: bool) -> Result<i32, Error> {
    let x = u32::from_le_bytes(view.read_fixed(addr, width as usize)?);
    let sh = width_shift(width);
    Ok(if signed {
        (x.wrapping_shl(sh) as i32).wrapping_shr(sh)
    } else {
        x as i32
    })
}

/// A register RHS `v` widened like the field (SAME width + signedness,
/// pinned), else the immediate `v`.
#[inline(always)]
fn rhs(view: &View, v: i32, reg: bool, width: u8, signed: bool) -> Result<i32, Error> {
    if reg {
        widen(view, v as u16, width, signed)
    } else {
        Ok(v)
    }
}

/// Compare rule `field op val`; `spec` comes from [`spec`]. With abs (never
/// on `OP_IN`), both sides take a saturating absolute value clamped to the
/// field width's positive max.
#[inline(never)]
pub fn check_cmp(view: &View, addr: u16, spec: u16, val: u32) -> Result<(), Error> {
    let width = (spec & SPEC_WIDTH) as u8;
    let signed = spec & SPEC_SIGNED != 0;
    let mut a = widen(view, addr, width, signed)?;
    let op = (spec >> 8) as u8;
    let lo_reg = spec & SPEC_RHS_REG != 0;
    let ok = if op == OP_IN {
        let lo = rhs(view, (val << 16) as i32 >> 16, lo_reg, width, signed)?;
        let hi = rhs(
            view,
            val as i32 >> 16,
            spec & SPEC_HI_REG != 0,
            width,
            signed,
        )?;
        a >= lo && a <= hi
    } else {
        let mut r = rhs(view, val as i32, lo_reg, width, signed)?;
        if spec & SPEC_ABS != 0 {
            let sat_max = i32::MAX.wrapping_shr(width_shift(width)) as u32;
            a = a.unsigned_abs().min(sat_max) as i32;
            r = r.unsigned_abs().min(sat_max) as i32;
        }
        match op {
            OP_LT => a < r,
            OP_LE => a <= r,
            OP_GT => a > r,
            OP_GE => a >= r,
            OP_EQ => a == r,
            OP_BITS => a as u32 & !(r as u32) == 0,
            OP_MAX_ONES => (a as u32).count_ones() <= r as u32,
            _ => a != r,
        }
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

    fn cmp(v: &View, width: u8, signed: bool, abs: bool, op: u8, imm: i32) -> Result<(), Error> {
        check_cmp(v, 0, spec(width, signed, abs, op), imm as u32)
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
        assert!(cmp(&v, 2, true, false, OP_LE, 5).is_ok());
        assert!(cmp(&v, 2, true, false, OP_LT, 5).is_err());
        assert!(cmp(&v, 2, true, false, OP_GE, 5).is_ok());
        assert!(cmp(&v, 2, true, false, OP_NE, 4).is_ok());
        assert!(cmp(&v, 2, true, false, OP_EQ, 4).is_err());
    }

    #[test]
    fn cmp_reg_rhs_loads_same_width() {
        let mut s = [0u8; 4];
        s[..2].copy_from_slice(&10i16.to_le_bytes());
        s[2..].copy_from_slice(&20i16.to_le_bytes());
        let v = view_over(&s);
        let reg = |op| check_cmp(&v, 0, spec(2, true, false, op) | SPEC_RHS_REG, 2);
        assert!(reg(OP_LE).is_ok());
        assert!(reg(OP_GT).is_err());
    }

    #[test]
    fn abs_saturates_at_width_max() {
        // i8::MIN's plain abs would overflow; the pinned semantics saturate,
        // then clamp to the width's positive max.
        let s = [0x80u8]; // -128
        let v = view_over(&s);
        assert!(cmp(&v, 1, true, true, OP_LE, 127).is_ok());
        assert!(cmp(&v, 1, true, true, OP_GT, 126).is_ok());
        // i16::MIN likewise clamps to i16::MAX.
        let s = i16::MIN.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, true, true, OP_EQ, i16::MAX as i32).is_ok());
    }

    #[test]
    fn bits_requires_a_subset_of_the_mask() {
        let s = 0x01C5u16.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, false, false, OP_BITS, 0x01FF).is_ok());
        assert!(cmp(&v, 2, false, false, OP_BITS, 0x01C5).is_ok());
        assert!(cmp(&v, 2, false, false, OP_BITS, 0x00FF).is_err());
        let s = 0u16.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, false, false, OP_BITS, 0).is_ok());
    }

    #[test]
    fn max_ones_caps_the_popcount() {
        let s = 0x003Fu16.to_le_bytes(); // 6 bits set
        let v = view_over(&s);
        assert!(cmp(&v, 2, false, false, OP_MAX_ONES, 6).is_ok());
        assert!(cmp(&v, 2, false, false, OP_MAX_ONES, 5).is_err());
        let s = 0u16.to_le_bytes();
        let v = view_over(&s);
        assert!(cmp(&v, 2, false, false, OP_MAX_ONES, 0).is_ok());
    }

    #[test]
    fn overlay_is_visible_to_rules() {
        let s = [0u8, 0];
        let pending = [0xFFu8, 0x7F]; // i16::MAX about to be committed
        let v = View::new(s.as_ptr(), s.len(), 0, &pending);
        assert!(cmp(&v, 2, true, false, OP_GE, 1000).is_ok());
    }

    /// `OP_IN` against the `ge` + `le` pair it replaces: every value of an
    /// i8 and a u8 field, every bound pair (immediate or register, inverted
    /// included).
    #[test]
    fn within_matches_its_ge_le_pair() {
        let bounds = [-128i32, -127, -1, 0, 1, 5, 126, 127];
        for signed in [true, false] {
            for &lo in &bounds {
                for &hi in &bounds {
                    for x in 0..=255u8 {
                        let s = [x, lo as u8, hi as u8];
                        let v = view_over(&s);
                        let sp = |op| spec(1, signed, false, op);
                        let (lo_v, hi_v) = if signed {
                            (lo, hi)
                        } else {
                            (lo as u8 as i32, hi as u8 as i32)
                        };
                        let pair = check_cmp(&v, 0, sp(OP_GE), lo_v as u32)
                            .and_then(|_| check_cmp(&v, 0, sp(OP_LE), hi_v as u32));
                        let fused = check_cmp(&v, 0, sp(OP_IN), within(lo_v, hi_v));
                        assert_eq!(fused, pair, "imm x {x} [{lo_v}, {hi_v}]");
                        let pair = check_cmp(&v, 0, sp(OP_GE) | SPEC_RHS_REG, 1)
                            .and_then(|_| check_cmp(&v, 0, sp(OP_LE) | SPEC_RHS_REG, 2));
                        let fused =
                            check_cmp(&v, 0, sp(OP_IN) | SPEC_RHS_REG | SPEC_HI_REG, within(1, 2));
                        assert_eq!(fused, pair, "reg x {x} [{lo}, {hi}]");
                        let pair = check_cmp(&v, 0, sp(OP_GE) | SPEC_RHS_REG, 1)
                            .and_then(|_| check_cmp(&v, 0, sp(OP_LE), hi_v as u32));
                        let fused = check_cmp(&v, 0, sp(OP_IN) | SPEC_RHS_REG, within(1, hi_v));
                        assert_eq!(fused, pair, "reg-imm x {x} [{lo}, {hi_v}]");
                        let pair = check_cmp(&v, 0, sp(OP_GE), lo_v as u32)
                            .and_then(|_| check_cmp(&v, 0, sp(OP_LE) | SPEC_RHS_REG, 2));
                        let fused = check_cmp(&v, 0, sp(OP_IN) | SPEC_HI_REG, within(lo_v, 2));
                        assert_eq!(fused, pair, "imm-reg x {x} [{lo_v}, {hi}]");
                    }
                }
            }
        }
    }

    #[test]
    fn within_packs_signed_halves() {
        assert_eq!(within(-8192, 8192), 0x2000_E000);
        assert_eq!(within(i16::MIN as i32, i16::MAX as i32), 0x7FFF_8000);
    }

    #[test]
    #[should_panic(expected = "within bound must fit i16")]
    fn within_rejects_a_bound_past_i16() {
        within(0, i16::MAX as i32 + 1);
    }
}
