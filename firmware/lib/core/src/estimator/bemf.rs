//! Model back-EMF velocity on a 1 ms boxcar. Slow decay shorts the
//! terminals off-window (v_diff ~ 0), so the period-average applied
//! differential voltage is just the duty fraction times the drive-window
//! differential - bridge drops included, which is the point. Subtract the
//! resistive drop and scale by 1/Ke and the motor is its own tachometer:
//! omega = (v_mean - R*i) / Ke, divide-free via caller-side reciprocals
//! (control-theory "Back-EMF, a Velocity Sensor for Free").
//!
//! Every FAST tick adds one sample; the kernel closes a half at each MEDIUM
//! boundary (DECIM_MED samples) and the output is the mean over the two
//! most recent halves - 2 x DECIM_MED FAST ticks, 1 ms at 20 kHz. A half is
//! valid only if every sample in it had both drive windows above the
//! sampling floors; one sub-floor tick voids both boxcars it sits in.

use crate::kernel::DECIM_MED;
use crate::math::q_mul;

/// `recip_arr` Q: chip const-evals `(1 << RECIP_ARR_SHIFT) / pwm_arr`. Q24
/// keeps reciprocal quantization under arr/2^24 relative (arr=1200 -> 0.007%
/// vs 1.2% at Q16). `drive_ticks * vdiff` <= 65535 * 4095 fits i32 and
/// `q_mul` widens to i64, so the extra shift is free. The kernel computes
/// `v_mean = q_mul(drive_ticks * vdiff, recip_arr, RECIP_ARR_SHIFT)` once
/// per medium tick for the winding thermometer; the boxcar applies the same
/// reciprocal to a half's summed `drive_ticks * vdiff`.
pub const RECIP_ARR_SHIFT: u32 = 24;

/// `recip_ke_q` (CALIB motor block) Q: c/s per vcount at Q6.10. SG90-scale
/// motors land ~3.6 c/s per vcount (~3700 stored, 0.03% quantization); the
/// u16 caps at 64 c/s per vcount, 16x headroom over the rig motor.
pub const RECIP_KE_SHIFT: u32 = 10;

/// Boxcar length in FAST ticks: two MEDIUM halves.
pub const BOXCAR_TICKS: i32 = 2 * DECIM_MED as i32;

/// 1/BOXCAR_TICKS, Q16, rounded (const-eval; the chip has no divide).
const RECIP_BOXCAR_SHIFT: u32 = 16;
const RECIP_BOXCAR: i32 = ((1 << RECIP_BOXCAR_SHIFT) + BOXCAR_TICKS / 2) / BOXCAR_TICKS;

/// Half-boxcar accumulators plus the previous closed half.
#[derive(Default)]
pub struct BemfObs {
    /// Sum of `drive_ticks * vdiff` over the open half.
    sum_tv: i32,
    /// Sum of the signed shunt current over the open half.
    sum_i: i32,
    half_valid: bool,
    /// Previous half's `sum(v_mean) - R * sum(i)`, None if any sample in it
    /// was sub-floor.
    prev_half: Option<i32>,
}

impl BemfObs {
    pub const fn new() -> Self {
        Self {
            sum_tv: 0,
            sum_i: 0,
            half_valid: true,
            prev_half: None,
        }
    }

    /// One FAST-tick sample. `ticks` is the drive width the window select
    /// used, `vdiff` from `vdiff_from_frame`, `i` from `i_from_frame`;
    /// either `None` voids the open half. Saturating: 10 x 65535 x 4095
    /// exceeds i32 only above a 52k ARR, a hostile timing nothing else
    /// survives either.
    pub fn sample(&mut self, ticks: u32, vdiff: Option<i32>, i: Option<i32>) {
        match (vdiff, i) {
            (Some(v), Some(i)) => {
                self.sum_tv = self.sum_tv.saturating_add((ticks as i32).saturating_mul(v));
                self.sum_i = self.sum_i.saturating_add(i);
            }
            _ => self.half_valid = false,
        }
    }

    /// MEDIUM-boundary close: folds the open half and returns the boxcar
    /// velocity in whole c/s when both halves are valid. `recip_arr_q24`
    /// per `RECIP_ARR_SHIFT`, `recip_ke_q` per `RECIP_KE_SHIFT`, `r_q12`
    /// vcounts/ccount Q4.12.
    pub fn close_half(&mut self, r_q12: u16, recip_ke_q: u16, recip_arr_q24: u32) -> Option<i32> {
        let half = if self.half_valid {
            // sum(v_mean) - R * sum(i): |sum_i| <= 10 * 4095 keeps the R
            // product i64-exact for any gain encoding
            let v = q_mul(self.sum_tv, recip_arr_q24 as i32, RECIP_ARR_SHIFT);
            Some(v.saturating_sub(q_mul(r_q12 as i32, self.sum_i, 12)))
        } else {
            None
        };
        self.sum_tv = 0;
        self.sum_i = 0;
        self.half_valid = true;
        let out = match (self.prev_half, half) {
            // (a + b) / BOXCAR_TICKS * recip_ke in one truncation: the
            // reciprocal product <= 65535 * 3277 stays i32, the i64 widen
            // holds |a + b| * that for any half sum
            (Some(a), Some(b)) => Some(q_mul(
                a.saturating_add(b),
                recip_ke_q as i32 * RECIP_BOXCAR,
                RECIP_KE_SHIFT + RECIP_BOXCAR_SHIFT,
            )),
            _ => None,
        };
        self.prev_half = half;
        out
    }
}

/// Saturating cast matching telemetry `est.omega_bemf_cps`.
pub fn omega_cps_i16(omega: i32) -> i16 {
    omega.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    const ARR: u32 = 1200;
    const RECIP_ARR: u32 = (1u32 << RECIP_ARR_SHIFT) / ARR;
    // unity Ke: 1.0 c/s per vcount
    const KE_UNITY: u16 = 1 << RECIP_KE_SHIFT;
    const HALF: usize = DECIM_MED as usize;

    /// Feed one whole half of identical samples and close it.
    fn half(obs: &mut BemfObs, ticks: u32, vdiff: Option<i32>, i: Option<i32>) -> Option<i32> {
        for _ in 0..HALF {
            obs.sample(ticks, vdiff, i);
        }
        obs.close_half(4096, KE_UNITY, RECIP_ARR)
    }

    /// Two clean halves of a constant pair with explicit gains/reciprocal.
    fn boxcar(ticks: u32, vdiff: i32, i: i32, r_q12: u16, recip_ke: u16, recip_arr: u32) -> i32 {
        let mut obs = BemfObs::new();
        for n in 0..2 * HALF {
            obs.sample(ticks, Some(vdiff), Some(i));
            if n == HALF - 1 {
                obs.close_half(r_q12, recip_ke, recip_arr);
            }
        }
        obs.close_half(r_q12, recip_ke, recip_arr).unwrap()
    }

    /// The closed form a constant pair must land on: the boxcar mean of
    /// v_mean - R*i, times 1/Ke, with real division.
    fn closed_form(ticks: u32, vdiff: i32, i: i32, r_q12: u16, recip_ke: u16) -> i64 {
        let v = (BOXCAR_TICKS as i64 * ticks as i64 * vdiff as i64 * RECIP_ARR as i64) >> 24;
        let r = (r_q12 as i64 * BOXCAR_TICKS as i64 * i as i64) >> 12;
        (((v - r) * recip_ke as i64) / (BOXCAR_TICKS as i64)) >> RECIP_KE_SHIFT
    }

    #[test]
    fn boxcar_ticks_is_one_ms_at_20khz() {
        assert_eq!(BOXCAR_TICKS, 20);
        assert_eq!(RECIP_BOXCAR, 3277);
    }

    #[test]
    fn first_close_has_no_previous_half() {
        let mut obs = BemfObs::new();
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), None);
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), Some(1499));
    }

    #[test]
    fn zero_current_scaling() {
        // 50% duty of vdiff=3000 -> v_mean ideal 1500 per tick; the summed
        // form floors once (29999 over 20 ticks) and the 1/20 reciprocal
        // (3277 vs 3276.8) rounds up by under a count
        let out = boxcar(600, 3000, 0, 4096, KE_UNITY, RECIP_ARR);
        assert!((out as i64 - closed_form(600, 3000, 0, 4096, KE_UNITY)).abs() <= 1);
        assert_eq!(out, 1499, "pin");
        // rig-scale recip_ke ~3.61 c/s per vcount
        let out = boxcar(600, 3000, 0, 4096, 3700, RECIP_ARR);
        assert!((out as i64 - closed_form(600, 3000, 0, 4096, 3700)).abs() <= 2);
        assert_eq!(out, 5419, "pin");
    }

    #[test]
    fn r_drop_reduces_omega_toward_zero() {
        // r=0.5 Q4.12, i=2048 -> r_drop 1024 per tick -> ~475
        let out = boxcar(600, 3000, 2048, 2048, KE_UNITY, RECIP_ARR);
        assert!((out as i64 - closed_form(600, 3000, 2048, 2048, KE_UNITY)).abs() <= 1);
        assert_eq!(out, 475, "pin");
    }

    #[test]
    fn sign_symmetry() {
        // exact-power-of-two drive: arr=1024, ticks=512, vdiff +-2048 ->
        // +-1024 per tick; the arithmetic shift floors the negative side
        let recip = (1u32 << RECIP_ARR_SHIFT) / 1024;
        assert_eq!(boxcar(512, 2048, 0, 4096, KE_UNITY, recip), 1024);
        assert_eq!(boxcar(512, -2048, 0, 4096, KE_UNITY, recip), -1025);
    }

    #[test]
    fn one_sub_floor_tick_voids_both_boxcars() {
        let mut obs = BemfObs::new();
        half(&mut obs, 600, Some(3000), Some(0));
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), Some(1499));
        // one voided sample in the third half
        obs.sample(0, None, None);
        for _ in 1..HALF {
            obs.sample(600, Some(3000), Some(0));
        }
        assert_eq!(obs.close_half(4096, KE_UNITY, RECIP_ARR), None);
        // the fourth half is clean but pairs with the voided third
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), None);
        // the fifth pairs with the clean fourth
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), Some(1499));
    }

    #[test]
    fn either_window_invalid_voids_the_half() {
        for (vdiff, i) in [(None, Some(0)), (Some(3000), None)] {
            let mut obs = BemfObs::new();
            half(&mut obs, 600, Some(3000), Some(0));
            obs.sample(600, vdiff, i);
            for _ in 1..HALF {
                obs.sample(600, Some(3000), Some(0));
            }
            assert_eq!(obs.close_half(4096, KE_UNITY, RECIP_ARR), None);
        }
    }

    #[test]
    fn saturates_at_i16_bounds() {
        // full duty, full vdiff, max recip_ke: omega ~262k >> i16::MAX
        let out = boxcar(ARR, 4095, 0, 0, u16::MAX, RECIP_ARR);
        assert_eq!(omega_cps_i16(out), i16::MAX);
        assert_eq!(out, 262_085, "pin");
        let out = boxcar(ARR, -4095, 0, 0, u16::MAX, RECIP_ARR);
        assert_eq!(omega_cps_i16(out), i16::MIN);
        assert_eq!(out, -262_092, "pin");
    }

    #[test]
    fn hostile_inputs_never_wrap() {
        // debug overflow checks are the wrap detector
        let mut obs = BemfObs::new();
        for n in 0..200usize {
            let v = if n & 1 == 0 { i32::MAX } else { i32::MIN };
            let i = if n & 2 == 0 { i32::MAX } else { i32::MIN };
            obs.sample(u16::MAX as u32, Some(v), Some(i));
            if n % HALF == HALF - 1 {
                obs.close_half(u16::MAX, u16::MAX, u32::MAX);
            }
        }
    }
}
