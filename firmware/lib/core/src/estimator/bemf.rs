//! Model back-EMF velocity on a 1 ms boxcar. Slow decay shorts the
//! terminals off-window (v_diff ~ 0), so the period-average applied
//! differential voltage is just the duty fraction times the drive-window
//! differential - bridge drops included, which is the point. Subtract the
//! resistive and the inductive drop and scale by 1/Ke and the motor is its
//! own tachometer: omega = (v_mean - R*i - L*di/dt) / Ke, divide-free via
//! caller-side reciprocals (control-theory "Back-EMF, a Velocity Sensor for
//! Free"). Summed over a window the per-tick L*di/dt telescopes to
//! L*(i_end - i_start): the edge samples carry the whole term, nothing for
//! a current that ends where it started, the whole surge for a window the
//! velocity loop steps across the sampling floor - which would otherwise
//! read as speed.
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
    /// Latest valid current sample: the open half's end edge.
    i_last: i32,
    /// `i_last` at the previous close: the open half's start edge, held
    /// through sub-floor ticks the way the kernel holds its current; 0
    /// from boot, where the winding is at rest.
    i_ref: i32,
    half_valid: bool,
    /// Previous half's `sum(v_mean) - R * sum(i) - L * di`, None if any
    /// sample in it was sub-floor.
    prev_half: Option<i32>,
}

impl BemfObs {
    pub const fn new() -> Self {
        Self {
            sum_tv: 0,
            sum_i: 0,
            i_last: 0,
            i_ref: 0,
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
                self.i_last = i;
            }
            _ => self.half_valid = false,
        }
    }

    /// MEDIUM-boundary close: folds the open half and returns the boxcar
    /// velocity in whole c/s when both halves are valid. `recip_arr_q24`
    /// per `RECIP_ARR_SHIFT`, `recip_ke_q` per `RECIP_KE_SHIFT`, `r_q12`
    /// vcounts/ccount Q4.12, `l_tick_q412` vcounts per ccount per FAST tick
    /// Q4.12 (the inductance over the tick). An unset Ke (0) yields no
    /// estimate: a zero would read as "at rest" to the velocity loop, the
    /// runaway seed.
    pub fn close_half(
        &mut self,
        r_q12: u16,
        l_tick_q412: u16,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        let half = if self.half_valid {
            // sum(v_mean) - R * sum(i) - L * (i_end - i_start): |sum_i| <=
            // 10 * 4095 keeps the R product i64-exact for any gain encoding
            let v = q_mul(self.sum_tv, recip_arr_q24 as i32, RECIP_ARR_SHIFT);
            let di = self.i_last.saturating_sub(self.i_ref);
            Some(
                v.saturating_sub(q_mul(r_q12 as i32, self.sum_i, 12))
                    .saturating_sub(q_mul(l_tick_q412 as i32, di, 12)),
            )
        } else {
            None
        };
        self.sum_tv = 0;
        self.sum_i = 0;
        self.i_ref = self.i_last;
        self.half_valid = true;
        let out = match (self.prev_half, half) {
            // (a + b) / BOXCAR_TICKS * recip_ke in one truncation: the
            // reciprocal product <= 65535 * 3277 stays i32, the i64 widen
            // holds |a + b| * that for any half sum
            (Some(a), Some(b)) if recip_ke_q != 0 => Some(q_mul(
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
    // unity L: 1.0 vcount per ccount per tick
    const L_UNITY: u16 = 1 << 12;
    const HALF: usize = DECIM_MED as usize;

    /// Feed one whole half of identical samples and close it, no inductive
    /// term.
    fn half(obs: &mut BemfObs, ticks: u32, vdiff: Option<i32>, i: Option<i32>) -> Option<i32> {
        for _ in 0..HALF {
            obs.sample(ticks, vdiff, i);
        }
        obs.close_half(4096, 0, KE_UNITY, RECIP_ARR)
    }

    /// Two clean halves of a constant pair with explicit gains/reciprocal,
    /// no inductive term.
    fn boxcar(ticks: u32, vdiff: i32, i: i32, r_q12: u16, recip_ke: u16, recip_arr: u32) -> i32 {
        let mut obs = BemfObs::new();
        window(&mut obs, ticks, vdiff, |_| i, r_q12, 0, recip_ke, recip_arr).unwrap()
    }

    /// One whole window (two halves) of a per-tick current sequence through
    /// `obs`, returning the second close.
    #[allow(clippy::too_many_arguments)]
    fn window(
        obs: &mut BemfObs,
        ticks: u32,
        vdiff: i32,
        i: impl Fn(usize) -> i32,
        r_q12: u16,
        l_q412: u16,
        recip_ke: u16,
        recip_arr: u32,
    ) -> Option<i32> {
        for n in 0..2 * HALF {
            obs.sample(ticks, Some(vdiff), Some(i(n)));
            if n == HALF - 1 {
                obs.close_half(r_q12, l_q412, recip_ke, recip_arr);
            }
        }
        obs.close_half(r_q12, l_q412, recip_ke, recip_arr)
    }

    /// The closed form a constant pair must land on: the boxcar mean of
    /// v_mean - R*i, times 1/Ke, with real division.
    fn closed_form(ticks: u32, vdiff: i32, i: i32, r_q12: u16, recip_ke: u16) -> i64 {
        closed_form_di(ticks, vdiff, BOXCAR_TICKS * i, 0, r_q12, 0, recip_ke)
    }

    /// The closed form for a window whose current sums to `i_sum` and
    /// rises `di` from the sample before it to its last sample.
    fn closed_form_di(
        ticks: u32,
        vdiff: i32,
        i_sum: i32,
        di: i32,
        r_q12: u16,
        l_q412: u16,
        recip_ke: u16,
    ) -> i64 {
        let v = (BOXCAR_TICKS as i64 * ticks as i64 * vdiff as i64 * RECIP_ARR as i64) >> 24;
        let r = (r_q12 as i64 * i_sum as i64) >> 12;
        let l = (l_q412 as i64 * di as i64) >> 12;
        (((v - r - l) * recip_ke as i64) / (BOXCAR_TICKS as i64)) >> RECIP_KE_SHIFT
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
    fn inductive_term_books_the_current_rise_over_the_window() {
        // current ramps 10 per tick from rest to 200 across the window: at
        // unity L the window books 200 vcounts, 10 per tick, 10 c/s at
        // unity Ke off the zero-current pin
        let ramp = |n: usize| 10 * (n as i32 + 1);
        let mut obs = BemfObs::new();
        let out = window(&mut obs, 600, 3000, ramp, 0, L_UNITY, KE_UNITY, RECIP_ARR).unwrap();
        assert!(
            (out as i64 - closed_form_di(600, 3000, 2100, 200, 0, L_UNITY, KE_UNITY)).abs() <= 1
        );
        assert_eq!(out, 1489, "pin");
        // the same window with no inductance reads the rise as speed
        let mut obs = BemfObs::new();
        let out = window(&mut obs, 600, 3000, ramp, 0, 0, KE_UNITY, RECIP_ARR).unwrap();
        assert_eq!(out, 1499);
    }

    #[test]
    fn inductive_term_at_the_rig_scale() {
        // rev 2A MG90 at a 15% drive window on a 7.1 V rail: r_q12 4317,
        // L 0.757 mH through the sense chain = 3.39 vcounts per ccount
        // per tick (13889), recip_ke 11.04 c/s per vcount (11305); the
        // window's current climbs from a settled 100 to 200 counts, the
        // shape of a velocity-loop duty step across the sampling floor
        let ramp = |n: usize| 100 + 5 * (n as i32 + 1);
        let mut obs = BemfObs::new();
        window(&mut obs, 180, 1762, |_| 100, 4317, 13889, 11305, RECIP_ARR);
        let out = window(&mut obs, 180, 1762, ramp, 4317, 13889, 11305, RECIP_ARR).unwrap();
        let want = closed_form_di(180, 1762, 3050, 100, 4317, 13889, 11305);
        assert!((out as i64 - want).abs() <= 2, "got {out} want {want}");
        assert_eq!(out, 956, "pin");
        // without the term the same window over-reads by L * 100 / 20
        // vcounts per tick = 17 vcounts = 187 c/s of speed
        let mut obs = BemfObs::new();
        window(&mut obs, 180, 1762, |_| 100, 4317, 0, 11305, RECIP_ARR);
        let out = window(&mut obs, 180, 1762, ramp, 4317, 0, 11305, RECIP_ARR).unwrap();
        assert_eq!(out, 1143, "pin");
    }

    #[test]
    fn inductive_term_telescopes_to_the_window_edges() {
        // a 0 -> 200 step books the same 200 vcounts wherever it lands in
        // the window, and the settled window after it books nothing
        for at in [0usize, 3, 10, 19] {
            let step = |n: usize| if n >= at { 200 } else { 0 };
            let mut obs = BemfObs::new();
            let out = window(&mut obs, 600, 3000, step, 0, L_UNITY, KE_UNITY, RECIP_ARR).unwrap();
            assert_eq!(out, 1489, "step at {at}");
            let out = window(
                &mut obs,
                600,
                3000,
                |_| 200,
                0,
                L_UNITY,
                KE_UNITY,
                RECIP_ARR,
            )
            .unwrap();
            assert_eq!(out, 1499, "settled after the step at {at}");
        }
    }

    #[test]
    fn boot_edge_is_rest() {
        // the first window after boot references a winding at rest, so a
        // constant current books its rise once; the next window is clean
        let mut obs = BemfObs::new();
        let out = window(
            &mut obs,
            600,
            3000,
            |_| 200,
            0,
            L_UNITY,
            KE_UNITY,
            RECIP_ARR,
        )
        .unwrap();
        assert_eq!(out, 1489);
        for _ in 0..HALF {
            obs.sample(600, Some(3000), Some(200));
        }
        assert_eq!(obs.close_half(0, L_UNITY, KE_UNITY, RECIP_ARR), Some(1499));
    }

    #[test]
    fn inductive_start_edge_holds_through_a_sub_floor_gap() {
        // a voided half leaves the start edge at the last valid sample, so
        // the first clean window after the gap at the same current books
        // no surge; a window that resumes higher books the difference
        let mut obs = BemfObs::new();
        window(
            &mut obs,
            600,
            3000,
            |_| 200,
            0,
            L_UNITY,
            KE_UNITY,
            RECIP_ARR,
        );
        for _ in 0..HALF {
            obs.sample(0, None, None);
        }
        assert_eq!(obs.close_half(0, L_UNITY, KE_UNITY, RECIP_ARR), None);
        assert_eq!(
            window(
                &mut obs,
                600,
                3000,
                |_| 200,
                0,
                L_UNITY,
                KE_UNITY,
                RECIP_ARR
            ),
            Some(1499)
        );
        for _ in 0..HALF {
            obs.sample(0, None, None);
        }
        obs.close_half(0, L_UNITY, KE_UNITY, RECIP_ARR);
        assert_eq!(
            window(
                &mut obs,
                600,
                3000,
                |_| 400,
                0,
                L_UNITY,
                KE_UNITY,
                RECIP_ARR
            ),
            Some(1489)
        );
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
        assert_eq!(obs.close_half(4096, 0, KE_UNITY, RECIP_ARR), None);
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
            assert_eq!(obs.close_half(4096, 0, KE_UNITY, RECIP_ARR), None);
        }
    }

    #[test]
    fn zero_recip_ke_yields_no_estimate() {
        let mut obs = BemfObs::new();
        half(&mut obs, 600, Some(3000), Some(0));
        for _ in 0..HALF {
            obs.sample(600, Some(3000), Some(0));
        }
        assert_eq!(obs.close_half(4096, 0, 0, RECIP_ARR), None);
        // the halves kept folding: the next clean close pairs as usual
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), Some(1499));
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
                obs.close_half(u16::MAX, u16::MAX, u16::MAX, u32::MAX);
            }
        }
    }
}
