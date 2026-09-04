//! Fixed-point primitives. Every widening multiply in the kernel funnels
//! through here - a single fix point if codegen regresses. On rv32ec+Zmmul
//! LLVM lowers the extended 32x32 products to inline mul+mulh and the
//! constant 64-bit shifts to short register-pair sequences (no __muldi3);
//! check-soft-arith.sh is the CI backstop.

/// `(a * b) >> k` via an i64 widening product. Arithmetic shift: truncates
/// toward negative infinity, so `q_mul(-3, 1, 1) == -2` while
/// `q_mul(3, 1, 1) == 1`.
#[inline(always)]
pub const fn q_mul(a: i32, b: i32, k: u32) -> i32 {
    ((a as i64 * b as i64) >> k) as i32
}

/// Unsigned twin of `q_mul`. The u64 product cannot overflow for any
/// operands; the caller picks `k` so the result fits u32.
#[inline(always)]
pub const fn q_mul_u(a: u32, b: u32, k: u32) -> u32 {
    ((a as u64 * b as u64) >> k) as u32
}

/// `a*b >= c*d` via u64 widening products compared directly. The trajectory
/// decel test needs only the comparison, never a quotient - rv32ec has no
/// hardware divide, so cross-multiplying keeps it a mul+mulh pair per side
/// plus a 64-bit compare.
#[inline(always)]
pub fn wide_ge(a: u32, b: u32, c: u32, d: u32) -> bool {
    a as u64 * b as u64 >= c as u64 * d as u64
}

// Q1.30 linear reciprocal seed r0 = 48/17 - (32/17)x for the normalized
// x in [0.5, 1): max relative error 1/17, so two Newton steps land at
// (1/17)^4 ~ 1.2e-5. A clz power-of-two seed can start a factor of 2 off,
// where r*(2 - r*d) convergence stalls and needs ~8 steps; the linear seed
// keeps the full-accuracy reciprocal one-shot at ~10 multiplies, cheap
// enough for every caller to share this implementation.
const SEED_C1_Q30: u32 = 3_031_741_621; // round(48/17 << 30)
const SEED_C2_Q30: u32 = 2_021_161_080; // round(32/17 << 30)

/// `num / d` without a divide. Normalize d into [2^31, 2^32) (Q0.32), linear
/// seed + two Newton steps `r' = r*(2 - r*d)` in Q1.30, then one widening
/// multiply denormalizes straight into the quotient. Error <= 1.2e-5
/// relative plus truncation (pinned in tests). d == 0 has no quotient;
/// callers gate the degenerate span, num is the no-panic backstop.
pub fn recip_div(num: u32, d: u32) -> u32 {
    if d <= 1 {
        return num;
    }
    let n = d.leading_zeros(); // 1..=30 for d >= 2
    let dn = d << n;
    let mut r = SEED_C1_Q30 - q_mul_u(SEED_C2_Q30, dn, 32);
    r = q_mul_u(r, (2u32 << 30) - q_mul_u(r, dn, 32), 30);
    r = q_mul_u(r, (2u32 << 30) - q_mul_u(r, dn, 32), 30);
    // num/d = num*r >> (62-n), split as (>> 32) then (>> 30-n): identical
    // bits, but the u64 shift stays constant and the variable shift is u32 -
    // a variable 64-bit shift is soft on rv32ec (check-soft-arith.sh bans it)
    let p = num as u64 * r as u64;
    ((p >> 32) as u32) >> (30 - n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q_mul_identity() {
        for x in [0, 1, -1, 12345, -12345, i32::MAX >> 1, i32::MIN >> 1] {
            assert_eq!(q_mul(x, 1 << 12, 12), x);
        }
    }

    #[test]
    fn q_mul_scales() {
        assert_eq!(q_mul(3 << 15, 5 << 15, 15), 15 << 15);
        assert_eq!(q_mul(1000, 256, 8), 1000);
        assert_eq!(q_mul(i32::MAX, i32::MAX, 61), 1);
    }

    #[test]
    fn q_mul_signs() {
        assert_eq!(q_mul(-1000, 256, 8), -1000);
        assert_eq!(q_mul(-1000, -256, 8), 1000);
        assert_eq!(q_mul(1000, -256, 8), -1000);
    }

    #[test]
    fn q_mul_truncates_toward_negative_infinity() {
        assert_eq!(q_mul(3, 1, 1), 1);
        assert_eq!(q_mul(-3, 1, 1), -2);
        assert_eq!(q_mul(-1, 1, 1), -1);
        assert_eq!(q_mul(-1, 1, 31), -1);
    }

    #[test]
    fn q_mul_u_extremes() {
        // MAX*MAX = 2^64 - 2^33 + 1 fits u64; >> 32 floors to 2^32 - 2.
        assert_eq!(q_mul_u(u32::MAX, u32::MAX, 32), u32::MAX - 1);
        assert_eq!(q_mul_u(u32::MAX, u32::MAX, 63), 1);
        assert_eq!(q_mul_u(u32::MAX, 1, 0), u32::MAX);
        assert_eq!(q_mul_u(0, u32::MAX, 15), 0);
    }

    #[test]
    fn wide_ge_equality() {
        assert!(wide_ge(0, 0, 0, 0));
        assert!(wide_ge(u32::MAX, u32::MAX, u32::MAX, u32::MAX));
        // 6*4 == 8*3
        assert!(wide_ge(6, 4, 8, 3));
        assert!(wide_ge(8, 3, 6, 4));
    }

    #[test]
    fn wide_ge_off_by_one_large() {
        // 65536*65536 = 2^32; 65537*65535 = 2^32 - 1
        assert!(wide_ge(65536, 65536, 65537, 65535));
        assert!(!wide_ge(65537, 65535, 65536, 65536));
        assert!(wide_ge(u32::MAX, u32::MAX, u32::MAX, u32::MAX - 1));
        assert!(!wide_ge(u32::MAX, u32::MAX - 1, u32::MAX, u32::MAX));
    }

    #[test]
    fn recip_div_accuracy_pinned() {
        // worst seeds are just under a power of two (x -> 1 from below maps
        // to r near the interval edge); sweep those plus a dense band and
        // the u32-extreme numerator. Bound: 1.2e-5 relative + 2 LSB.
        let spot = [32767u32, 32768, 65534, 65535];
        for d in (1..=4096u32).chain(spot) {
            for num in [1u32, 999, 1_200 * 2_000, 4_294_836_225] {
                let got = recip_div(num, d);
                let exact = (num / d) as i64;
                let err = (got as i64 - exact).abs();
                assert!(
                    err <= exact / 65536 + 2,
                    "num={num} d={d} got={got} exact={exact}"
                );
            }
        }
    }
}
