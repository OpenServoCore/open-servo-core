//! Velocity PI with model feedforward (control-theory "The Velocity Loop"):
//! PI on the fused velocity plus the mechanical model run backwards - J*alpha
//! along the profile and the friction model at the profile velocity - so the
//! PI only cleans up what the model missed. Kt never appears: gains are
//! host-synthesized in current counts per velocity count. The downstream
//! i_lim clip is the limits fold, and its excess back-calculates into the one
//! integrator - "whatever clips here back-propagates". Cogging reads 0 in v1.

use crate::math::q_mul;

/// PI error clamp, csQ16 (16384 c/s). omega spans +-32767<<16 so the raw
/// difference spans 2^32 - saturating_sub first, then clamp. The bound keeps
/// q_mul(ki, e, 16) i32-exact (2^16 * 2^30 >> 16 = 2^30) and the kp product
/// far inside i32 after its shift. SG90 tops ~9000 c/s; past 16384 c/s of
/// error any practical gain already commands far beyond the 4095-count shunt
/// full scale, so the flat region costs nothing.
const E_LIM_CSQ16: i32 = 16384 << 16;

/// Back-calculation excess clamp, ccounts. i_ref - i_raw is unbounded
/// (i_raw saturates i32 under hostile inputs); 8192 is 2x shunt full scale,
/// so clamping only limits the per-tick unwind rate, and kaw * 2^13 <= 2^29
/// stays i32-exact (current.rs AW_LIM_VC discipline).
const AW_LIM_CC: i32 = 8192;

/// Coulomb feedforward zero band, ~1 c/s (fusion's constant): omega_star
/// dithering around zero must not chatter +-fric_fc into i_ref.
const FRIC_OMEGA_EPS_CSQ16: i32 = 1 << 16;

/// CONFIG loop_velocity gains plus the CALIB friction model, loaded fresh
/// each step by the kernel (`CurrentGains` convention).
#[derive(Copy, Clone)]
pub struct VelocityGains {
    pub kp_q88: u16,
    pub ki_q412: u16,
    pub kaw_q412: u16,
    pub j_ff_q88: u16,
    pub fric_fc_counts: u16,
    pub fric_fv_q016: u16,
}

/// sgn(omega_star)*fric_fc outside the zero band plus the viscous slope:
/// Q0.16 * csQ16 >> 32 -> whole ccounts, <= 2^15 for any omega.
fn fric_ff(omega_q16: i32, gains: &VelocityGains) -> i32 {
    let coulomb = if omega_q16 > FRIC_OMEGA_EPS_CSQ16 {
        gains.fric_fc_counts as i32
    } else if omega_q16 < -FRIC_OMEGA_EPS_CSQ16 {
        -(gains.fric_fc_counts as i32)
    } else {
        0
    };
    coulomb.saturating_add(q_mul(gains.fric_fv_q016 as i32, omega_q16, 32))
}

/// State: one current-counts integrator (ccQ16).
#[derive(Default)]
pub struct VelocityLoop {
    integ_ccq16: i32,
}

impl VelocityLoop {
    pub const fn new() -> Self {
        Self { integ_ccq16: 0 }
    }

    /// Zero state; kernel calls on the enable edge so a stale integrator
    /// never kicks a fresh enable.
    pub fn reset(&mut self) {
        self.integ_ccq16 = 0;
    }

    /// One MEDIUM-tick update -> i_ref, whole ccounts clamped into `band`.
    /// `omega_ref_q16` is the position loop's output; alpha_star/omega_star
    /// come from the trajectory generator; `band` from the limits fold
    /// (directional: an endstop zeroes only the inward side). The clip
    /// excess i_ref - i_raw IS that fold's clip, fed back through kaw. The
    /// integrator is also clamped to the band's widest side << 16 - a belt
    /// on top of back-calc, and when the band shrinks mid-run (derate,
    /// stall fold) the clamp drains the stale charge to the new ceiling in
    /// one tick: intended, the fold propagates instantly.
    pub fn step(
        &mut self,
        omega_ref_q16: i32,
        omega_hat_q16: i32,
        alpha_star_q16: i32,
        omega_star_q16: i32,
        band: super::limits::IBand,
        gains: &VelocityGains,
    ) -> i32 {
        let e = omega_ref_q16
            .saturating_sub(omega_hat_q16)
            .clamp(-E_LIM_CSQ16, E_LIM_CSQ16);
        // Q8.8 * csQ16 >> 24 -> whole ccounts; <= 2^16 * 2^30 >> 24 = 2^22
        let i_p = q_mul(gains.kp_q88 as i32, e, 24);
        // j term <= 2^16 * 2^31 >> 24 = 2^23 for any alpha
        let i_ff = q_mul(gains.j_ff_q88 as i32, alpha_star_q16, 24)
            .saturating_add(fric_ff(omega_star_q16, gains));
        let i_raw = i_p
            .saturating_add(self.integ_ccq16 >> 16)
            .saturating_add(i_ff);
        let i_ref = band.clamp(i_raw);
        let aw = i_ref.saturating_sub(i_raw).clamp(-AW_LIM_CC, AW_LIM_CC);
        // Q4.12 * csQ16 -> ccQ16 needs >> 12, split as >> 16 then << 4 so
        // the q_mul result stays i32-exact (<= 2^30 per E_LIM) and only the
        // shift saturates; kaw term mirrors current.rs (<= 2^29, << 4 can
        // reach 2^33). Integrator clamp: i_lim <= 65535 so the << 16 can
        // saturate too (12-bit shunt scale keeps real limits far below).
        let ki_term = q_mul(gains.ki_q412 as i32, e, 16).saturating_mul(1 << 4);
        let aw_term = q_mul(gains.kaw_q412 as i32, aw, 0).saturating_mul(1 << 4);
        let lim = band.hi.abs().max(band.lo.abs()).saturating_mul(1 << 16);
        self.integ_ccq16 = self
            .integ_ccq16
            .saturating_add(ki_term)
            .saturating_add(aw_term)
            .clamp(-lim, lim);
        i_ref
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::limits::IBand;

    fn band(l: i32) -> IBand {
        IBand { lo: -l, hi: l }
    }

    const I_LIM: u16 = 4095;

    fn g(kp: u16, ki: u16, kaw: u16, j_ff: u16, fc: u16, fv: u16) -> VelocityGains {
        VelocityGains {
            kp_q88: kp,
            ki_q412: ki,
            kaw_q412: kaw,
            j_ff_q88: j_ff,
            fric_fc_counts: fc,
            fric_fv_q016: fv,
        }
    }

    /// First-order toy plant: drive b = 2^10 csQ16 per ccount per tick,
    /// viscous drain omega/16 per tick -> steady omega = i_ref << 14
    /// (0.25 c/s per ccount, i.e. 2000 ccounts hold 500 c/s).
    fn plant_omega(omega_q16: i32, i_ref: i32) -> i32 {
        omega_q16 - (omega_q16 >> 4) + (i_ref << 10)
    }

    #[test]
    fn p_term_sign_and_scale() {
        // kp 1.0 ccount per c/s: e = +-500 c/s -> +-500 ccounts exactly
        let gains = g(1 << 8, 0, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        assert_eq!(
            vel.step(500 << 16, 0, 0, 0, band(I_LIM as i32), &gains),
            500
        );
        assert_eq!(
            vel.step(-(500 << 16), 0, 0, 0, band(I_LIM as i32), &gains),
            -500
        );
        // kp 1/256: 500/256 floors to 1, arithmetic shift floors -500/256
        // to -2
        let gains = g(1, 0, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        assert_eq!(vel.step(500 << 16, 0, 0, 0, band(I_LIM as i32), &gains), 1);
        assert_eq!(
            vel.step(-(500 << 16), 0, 0, 0, band(I_LIM as i32), &gains),
            -2
        );
    }

    #[test]
    fn integrator_eliminates_steady_state_error() {
        // Plant wants i = 2000 for 500 c/s. P-only settles where
        // kp*e/4 (ccounts) holds omega: omega = ref/17 - huge standing
        // error; PI closes it.
        let p_only = g(64, 0, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        let mut omega = 0i32;
        for _ in 0..500 {
            let i_ref = vel.step(500 << 16, omega, 0, 0, band(I_LIM as i32), &p_only);
            omega = plant_omega(omega, i_ref);
        }
        assert!(omega >> 16 < 100, "p-only omega={}", omega >> 16);

        let pi = g(64, 40, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        let mut omega = 0i32;
        let mut i_ref = 0i32;
        for _ in 0..5000 {
            i_ref = vel.step(500 << 16, omega, 0, 0, band(I_LIM as i32), &pi);
            omega = plant_omega(omega, i_ref);
        }
        assert!(((omega >> 16) - 500).abs() <= 1, "pi omega={}", omega >> 16);
        assert!((i_ref - 2000).abs() <= 8, "pi i_ref={i_ref}");
    }

    #[test]
    fn feedforward_only_path() {
        // kp=ki=0: output is pure model. j_ff 1.0 * alpha 100 -> 100;
        // Coulomb 50 signed by omega_star; viscous 1/256 per c/s.
        let gains = g(0, 0, 0, 1 << 8, 50, 1 << 8);
        let mut vel = VelocityLoop::new();
        // 100 + 50 + floor(1000/256) = 100 + 50 + 3
        assert_eq!(
            vel.step(0, 0, 100 << 16, 1000 << 16, band(I_LIM as i32), &gains),
            153
        );
        // reverse: -50 + floor(-1000/256) = -50 - 4 (arithmetic shift)
        assert_eq!(
            vel.step(0, 0, 0, -(1000 << 16), band(I_LIM as i32), &gains),
            -54
        );
        // zero band: |omega_star| <= 1 c/s suppresses Coulomb, viscous
        // 1/256 of 1 c/s floors to 0
        assert_eq!(vel.step(0, 0, 0, 1 << 16, band(I_LIM as i32), &gains), 0);
        assert_eq!(
            vel.step(0, 0, 0, -(1 << 16), band(I_LIM as i32), &gains),
            -1
        );
        // one past the band: Coulomb kicks in
        assert_eq!(
            vel.step(0, 0, 0, (1 << 16) + 1, band(I_LIM as i32), &gains),
            50
        );
    }

    #[test]
    fn back_calc_prevents_windup() {
        // Phase 1: 500 c/s unreachable at i_lim 100 (plant caps at 25 c/s);
        // phase 2: drop to reachable 10 c/s and count ticks pinned high.
        // Back-calc holds the integrator at the saturation equilibrium
        // i_raw = i_lim + (ki/kaw)*e, so the release unsaturates on the
        // first tick; without it the integrator rails at the i_lim<<16
        // clamp and must grind back down through ki alone.
        let run = |kaw: u16| -> u32 {
            let gains = g(64, 40, kaw, 0, 0, 0);
            let mut vel = VelocityLoop::new();
            let mut omega = 0i32;
            let mut i_ref = 0i32;
            for _ in 0..300 {
                i_ref = vel.step(500 << 16, omega, 0, 0, band(100), &gains);
                omega = plant_omega(omega, i_ref);
            }
            assert_eq!(i_ref, 100, "saturated, kaw={kaw}");
            let mut ticks = 0u32;
            while ticks < 500 {
                i_ref = vel.step(10 << 16, omega, 0, 0, band(100), &gains);
                omega = plant_omega(omega, i_ref);
                if i_ref <= 60 {
                    break;
                }
                ticks += 1;
            }
            ticks
        };
        let with_aw = run(2048);
        let without_aw = run(0);
        assert!(with_aw <= 3, "with_aw={with_aw}");
        assert!(without_aw > 3 * (with_aw + 1), "without_aw={without_aw}");
    }

    #[test]
    fn integrator_drains_when_lim_folds() {
        // Pure I (ki 1.0) charges 1000 cc/tick against a held error and
        // rails at 4000<<16. Folding i_lim to 500 clamps the integrator to
        // 500<<16 in that same step - restoring the limit afterwards shows
        // the charge is gone, not merely masked by the output clamp.
        let gains = g(0, 1 << 12, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        for _ in 0..200 {
            vel.step(1000 << 16, 0, 0, 0, band(4000), &gains);
        }
        assert_eq!(vel.step(0, 0, 0, 0, band(4000), &gains), 4000);
        assert_eq!(vel.step(0, 0, 0, 0, band(500), &gains), 500);
        assert_eq!(vel.step(0, 0, 0, 0, band(4000), &gains), 500);
    }

    #[test]
    fn i_ref_capped_for_any_input() {
        // All-max gains, i32-extreme inputs, 20 ticks of accumulated state:
        // output never escapes +-i_lim, nothing panics.
        let gains = g(u16::MAX, u16::MAX, u16::MAX, u16::MAX, u16::MAX, u16::MAX);
        let ext = [i32::MIN, -(9000 << 16), 0, 9000 << 16, i32::MAX];
        for lim in [0u16, 100, 4095, u16::MAX] {
            for o_ref in ext {
                for o_hat in ext {
                    for star in [i32::MIN, 0, i32::MAX] {
                        let mut vel = VelocityLoop::new();
                        for _ in 0..20 {
                            let i = vel.step(o_ref, o_hat, star, star, band(lim as i32), &gains);
                            assert!(
                                i.unsigned_abs() <= lim as u32,
                                "i={i} lim={lim} o_ref={o_ref} o_hat={o_hat} star={star}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn reset_zeroes_state() {
        let gains = g(64, 200, 0, 0, 0, 0);
        let mut vel = VelocityLoop::new();
        for _ in 0..50 {
            vel.step(500 << 16, 0, 0, 0, band(I_LIM as i32), &gains);
        }
        assert_ne!(vel.step(0, 0, 0, 0, band(I_LIM as i32), &gains), 0);
        vel.reset();
        assert_eq!(vel.step(0, 0, 0, 0, band(I_LIM as i32), &gains), 0);
    }
}
