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

/// clz-based seed for a Q15 reciprocal Newton iteration. Contract: the
/// converged recip satisfies `(recip * v) >> 15 ~= 32767`, i.e.
/// `duty = q_mul(u, recip, 15)` maps `u == vbus` to ~full-scale 32767. The
/// seed is `2^(clz(v)-1)`, so `seed * v` lands in `[2^30, 2^31)` - within a
/// factor of 2 of the `32767 << 15` target for any `v` in `1..2^31` (vbus
/// counts are 12-bit in practice); the vbus estimator's Newton steps refine
/// it. `v == 0` has no reciprocal and just yields the max seed - no panic.
/// `leading_zeros` is an intrinsic, no libcall.
#[inline]
pub fn recip_seed_q15(v: u32) -> u32 {
    1u32 << v.leading_zeros().saturating_sub(1)
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
    fn recip_seed_within_factor_of_two() {
        let target = (32767u64) << 15;
        for v in 512..=4095u32 {
            let seed = recip_seed_q15(v);
            assert!(seed > 0);
            let p = seed as u64 * v as u64;
            assert!(p >= target / 2, "v={v} seed={seed} p={p}");
            assert!(p <= target * 2, "v={v} seed={seed} p={p}");
        }
    }

    #[test]
    fn recip_seed_edges_no_panic() {
        assert_eq!(recip_seed_q15(0), 1 << 31);
        assert_eq!(recip_seed_q15(1), 1 << 30);
        assert_eq!(recip_seed_q15(u32::MAX), 1);
    }
}
