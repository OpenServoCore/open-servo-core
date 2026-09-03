//! Back-EMF velocity observer, telemetry-only in v1 (nothing consumes it
//! yet). Slow decay shorts the terminals off-window (v_diff ~ 0), so the
//! period-average applied differential voltage is just the duty fraction
//! times the drive-window differential - bridge drops included, which is the
//! point. Subtract the resistive drop and scale by 1/Ke and the motor is its
//! own tachometer: omega = (v_mean - R*i) / Ke, divide-free via caller-side
//! reciprocals (control-theory "Back-EMF, a Velocity Sensor for Free").

use crate::math::q_mul;

/// `recip_arr` Q: chip const-evals `(1 << RECIP_ARR_SHIFT) / pwm_arr`. Q24
/// keeps reciprocal quantization under arr/2^24 relative (arr=1200 -> 0.007%
/// vs 1.2% at Q16). `drive_ticks * vdiff` <= 65535 * 4095 fits i32 and
/// `q_mul` widens to i64, so the extra shift is free.
pub const RECIP_ARR_SHIFT: u32 = 24;

/// `recip_ke_q` (CALIB motor block) Q: c/s per vcount at Q6.10. SG90-scale
/// motors land ~3.6 c/s per vcount (~3700 stored, 0.03% quantization); the
/// u16 caps at 64 c/s per vcount, 16x headroom over the rig motor.
pub const RECIP_KE_SHIFT: u32 = 10;

const GUARD: u32 = 2;
const ALPHA_SHIFT: u32 = 2;

/// The EWMA lives here, not downstream: per-period vdiff/shunt sampling
/// noise is this observer's own artifact and telemetry reads the output
/// directly. Alpha 1/4 with 2 guard bits; first valid sample seeds the state.
#[derive(Default)]
pub struct BemfObs {
    state_qg: i32,
    initialized: bool,
}

impl BemfObs {
    pub const fn new() -> Self {
        Self {
            state_qg: 0,
            initialized: false,
        }
    }

    /// One MEDIUM-tick update. `vdiff_drive` from `vdrive_from_frame`,
    /// `i_meas` from `i_from_frame`; either `None` (window too narrow) holds
    /// the last output. `recip_arr` and `recip_ke_q` per the shift consts
    /// above, `r_q12` vcounts/ccount Q4.12. Returns the telemetry value.
    pub fn step(
        &mut self,
        vdiff_drive: Option<i32>,
        i_meas: Option<i32>,
        drive_ticks: u32,
        recip_arr: u32,
        r_q12: u16,
        recip_ke_q: u16,
    ) -> i16 {
        if let (Some(vdiff), Some(i)) = (vdiff_drive, i_meas) {
            let v_mean = q_mul(
                drive_ticks as i32 * vdiff,
                recip_arr as i32,
                RECIP_ARR_SHIFT,
            );
            let r_drop = q_mul(r_q12 as i32, i, 12);
            let omega = q_mul(v_mean - r_drop, recip_ke_q as i32, RECIP_KE_SHIFT);
            let x = omega << GUARD;
            if self.initialized {
                self.state_qg += (x - self.state_qg) >> ALPHA_SHIFT;
            } else {
                self.state_qg = x;
                self.initialized = true;
            }
        }
        self.omega_cps_i16()
    }

    /// Filtered estimate, whole c/s, full width.
    pub fn omega_cps(&self) -> i32 {
        self.state_qg >> GUARD
    }

    /// Saturating cast matching telemetry `est.omega_bemf_cps`.
    pub fn omega_cps_i16(&self) -> i16 {
        self.omega_cps().clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ARR: u32 = 1200;
    const RECIP_ARR: u32 = (1 << RECIP_ARR_SHIFT) / ARR;
    // unity Ke: 1.0 c/s per vcount
    const KE_UNITY: u16 = 1 << RECIP_KE_SHIFT;

    #[test]
    fn zero_current_scaling() {
        // 50% duty of vdiff=3000 -> v_mean ideal 1500, floor 1499
        let mut obs = BemfObs::new();
        let out = obs.step(Some(3000), Some(0), 600, RECIP_ARR, 4096, KE_UNITY);
        assert_eq!(out, 1499);
        // rig-scale recip_ke ~3.61 c/s per vcount: 1499 * 3700 >> 10
        let mut obs = BemfObs::new();
        let out = obs.step(Some(3000), Some(0), 600, RECIP_ARR, 4096, 3700);
        assert_eq!(out, 5416);
    }

    #[test]
    fn r_drop_reduces_omega_toward_zero() {
        // v_mean 1499; r=0.5 Q4.12, i=2048 -> r_drop 1024 -> omega 475
        let mut obs = BemfObs::new();
        let out = obs.step(Some(3000), Some(2048), 600, RECIP_ARR, 2048, KE_UNITY);
        assert_eq!(out, 475);
        assert!(out < 1499);
        assert!(out > 0);
    }

    #[test]
    fn sign_symmetry() {
        // exact-power-of-two drive: arr=1024, ticks=512, vdiff +-2048 -> +-1024
        let recip = (1u32 << RECIP_ARR_SHIFT) / 1024;
        let mut fwd = BemfObs::new();
        let mut rev = BemfObs::new();
        let f = fwd.step(Some(2048), Some(0), 512, recip, 4096, KE_UNITY);
        let r = rev.step(Some(-2048), Some(0), 512, recip, 4096, KE_UNITY);
        assert_eq!(f, 1024);
        assert_eq!(r, -1024);
    }

    #[test]
    fn invalid_window_holds_last_output() {
        let recip = (1u32 << RECIP_ARR_SHIFT) / 1024;
        let mut obs = BemfObs::new();
        assert_eq!(
            obs.step(Some(2048), Some(0), 512, recip, 4096, KE_UNITY),
            1024
        );
        assert_eq!(obs.step(None, Some(0), 512, recip, 4096, KE_UNITY), 1024);
        assert_eq!(obs.step(Some(2048), None, 512, recip, 4096, KE_UNITY), 1024);
        assert_eq!(obs.step(None, None, 0, recip, 4096, KE_UNITY), 1024);
        // valid sample moves it again
        assert!(obs.step(Some(0), Some(0), 512, recip, 4096, KE_UNITY) < 1024);
    }

    #[test]
    fn saturates_at_i16_bounds() {
        // full duty, full vdiff, max recip_ke: omega 262012 >> i16::MAX
        let mut obs = BemfObs::new();
        let out = obs.step(Some(4095), Some(0), ARR, RECIP_ARR, 0, u16::MAX);
        assert_eq!(out, i16::MAX);
        assert_eq!(obs.omega_cps(), 262_012);
        let mut obs = BemfObs::new();
        let out = obs.step(Some(-4095), Some(0), ARR, RECIP_ARR, 0, u16::MAX);
        assert_eq!(out, i16::MIN);
        assert_eq!(obs.omega_cps(), -262_077);
    }

    #[test]
    fn ewma_seeds_then_converges() {
        let recip = (1u32 << RECIP_ARR_SHIFT) / 1024;
        let mut obs = BemfObs::new();
        // seed at 0, then step toward 1024: alpha 1/4 from the seed
        assert_eq!(obs.step(Some(0), Some(0), 512, recip, 4096, KE_UNITY), 0);
        assert_eq!(
            obs.step(Some(2048), Some(0), 512, recip, 4096, KE_UNITY),
            256
        );
        assert_eq!(
            obs.step(Some(2048), Some(0), 512, recip, 4096, KE_UNITY),
            448
        );
        let mut last = 0i16;
        for _ in 0..40 {
            last = obs.step(Some(2048), Some(0), 512, recip, 4096, KE_UNITY);
        }
        assert!((last as i32 - 1024).abs() <= 1, "last={last}");
    }
}
