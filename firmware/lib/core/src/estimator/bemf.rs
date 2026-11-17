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
//!
//! The kernel's close (`close`) holds two more rules over that pairing. A
//! boxcar needs `STARTUP_HALVES` valid halves ahead of its own two: the
//! first rows of a drive read fast while the current rises. And a half in
//! which the drive's sign changed is void, so the halves behind it start
//! up again. Both live at the close: the per-tick sample carries neither.
//!
//! At speed the spinning brushes add volts that follow the current beyond
//! the winding's resistance, so the boxcar would hand the speed loop its
//! own current as speed. The close takes that off a valid half too, the
//! brush term: the identified brush resistance (CALIB `r_brush_q12`) times
//! the half's current less its slow average, the deviation bounded at the
//! board's `KernelTiming::brush_dev_max`, weighted by the speed from 0
//! `BRUSH_SPAN_CPS` under CALIB `brush_cps` to full at it. The average
//! restarts on the first valid half after a void one; a void half leaves
//! it alone. A size of 0 is the close without the term, bit for bit.

use crate::kernel::DECIM_MED;
use crate::math::q_mul;
use crate::regions::config::current_counts;

/// `recip_arr` Q: chip const-evals `(1 << RECIP_ARR_SHIFT) / pwm_arr`. Q24
/// keeps reciprocal quantization under arr/2^24 relative (arr=1200 -> 0.007%
/// vs 1.2% at Q16). `drive_ticks * vdiff` <= 65535 * 4095 fits i32 and
/// `q_mul` widens to i64, so the extra shift is free. The boxcar applies
/// the reciprocal to a half's summed `drive_ticks * vdiff`.
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

/// Full-scale 12-bit terminal differential, vcounts.
const TAP_FULL_SCALE: u32 = 4095;

/// The widest PWM ARR the half sums hold for: DECIM_MED widths times a
/// full-scale differential stay under i32.
pub const ARR_MAX: u32 = i32::MAX as u32 / (DECIM_MED as u32 * TAP_FULL_SCALE);

/// Valid halves a boxcar needs ahead of its own two (`BemfObs::close`).
pub const STARTUP_HALVES: u8 = 1;

/// How far under CALIB `brush_cps` the brush term's weight falls to 0, c/s.
pub const BRUSH_SPAN_CPS: i32 = 1024;
/// 256 weight steps of 4 c/s span the fade: a shift, no divide.
const BRUSH_W_SHIFT: u32 = 2;
const _: () = assert!(BRUSH_SPAN_CPS == 256 << BRUSH_W_SHIFT);
/// The brush term's current average, 2^BRUSH_AVG_SHIFT valid halves: an
/// octave over the blend's standing corner (`omega_blend::D_SHIFT - 1`,
/// which the blend asserts; the experiment image has no blend), so the
/// term follows only what the blend passes as fast.
pub const BRUSH_AVG_SHIFT: u32 = 6;
/// The bound on the current deviation the brush term follows, mA.
pub const BRUSH_DEV_MA: u16 = 40;

/// The brush term's bound, `sum_i` units: [`BRUSH_DEV_MA`] through the
/// board's sense chain (`current_counts`) over a half's DECIM_MED ticks.
/// Const-eval only (`KernelTiming::brush_dev_max`).
pub const fn brush_dev_max(shunt_r_mohm: u16, gain_milli: u16, vdd_mv: u16) -> i32 {
    current_counts(BRUSH_DEV_MA, shunt_r_mohm, gain_milli, vdd_mv) as i32 * DECIM_MED as i32
}

/// Whether a bound keeps the half's bounded product one i32 multiply: the
/// size is at most `i16::MAX` (`brush_rc_q12`).
pub const fn brush_dev_max_fits(dev_max: i32) -> bool {
    dev_max >= 0 && i16::MAX as i64 * dev_max as i64 <= i32::MAX as i64
}

/// The take's brush resistance, `r_q12` units: `r_brush_q12` capped at half
/// the stored winding's `r_q12`, and 0 when `brush_cps` is under the fade's
/// span.
pub fn brush_rc_q12(r_brush_q12: u16, brush_cps: u16, r_q12: u16) -> i32 {
    if (brush_cps as i32) < BRUSH_SPAN_CPS {
        return 0;
    }
    r_brush_q12.min(r_q12 >> 1) as i32
}

/// The brush term's size this period, `r_q12` units: `rc` weighted by
/// |`omega_hat_q16`|, 0 at `lo` c/s and under, full from `lo +
/// BRUSH_SPAN_CPS` up, linear between.
#[inline(always)]
pub fn brush_xw_q12(omega_hat_q16: i32, rc: i32, lo: i32) -> i32 {
    let w = ((((omega_hat_q16 >> 16).unsigned_abs() as i32) - lo).max(0) >> BRUSH_W_SHIFT).min(256);
    (rc * w) >> 8
}

/// `BemfObs::hist` of a valid half ahead of the open one: the plain
/// pairing's.
const PAIR: u8 = 1;
/// `BemfObs::hist` of that half and the `STARTUP_HALVES` ahead of it.
const STARTUP: u8 = (1 << (1 + STARTUP_HALVES)) - 1;

/// Half-boxcar accumulators plus the previous closed half.
#[derive(Default)]
pub struct BemfObs {
    /// Sum of `drive_ticks * vdiff` over the open half.
    sum_tv: i32,
    /// Sum of the signed shunt current over the open half.
    sum_i: i32,
    half_valid: bool,
    /// Previous half's `sum(v_mean) - R * sum(i)`, stale unless `hist`
    /// names that half valid.
    prev_half: i32,
    /// The closed halves' validity, bit n the half n + 1 closes back: no
    /// sub-floor sample in it and, under `close`, no drive flip.
    hist: u8,
    /// The sign of the last nonzero duty a `close` saw, 0 until one has.
    drive: i8,
    /// The last `close` saw the drive's sign change.
    flip: bool,
    /// Slow average of the valid halves' `sum_i`, restarted at the first
    /// valid half after a void one.
    i_avg: i32,
    /// The bound on the brush term's current deviation, `sum_i` units
    /// (`brush_dev_max`).
    dev_max: i32,
}

impl BemfObs {
    /// `dev_max` per `brush_dev_max_fits`.
    pub const fn new(dev_max: i32) -> Self {
        Self {
            sum_tv: 0,
            sum_i: 0,
            half_valid: true,
            prev_half: 0,
            hist: 0,
            drive: 0,
            flip: false,
            i_avg: 0,
            dev_max,
        }
    }

    /// One FAST-tick sample. `ticks` is the drive width the window select
    /// used, `vdiff` from `vdiff_from_frame`, `i` the reading
    /// `Fast::measure` takes; either `None` voids the open half. A half is
    /// DECIM_MED samples of a width <= ARR times a 12-bit differential, so
    /// `sum_tv` holds for any ARR up to `ARR_MAX`; `sum_i` stays under 2^21.
    pub fn sample(&mut self, ticks: u32, vdiff: Option<i32>, i: Option<i32>) {
        match (vdiff, i) {
            (Some(v), Some(i)) => {
                self.sum_tv += ticks as i32 * v;
                self.sum_i += i;
            }
            _ => self.half_valid = false,
        }
    }

    /// MEDIUM-boundary close: folds the open half and returns the boxcar
    /// velocity in whole c/s when both halves are valid. `recip_arr_q24`
    /// per `RECIP_ARR_SHIFT`, `recip_ke_q` per `RECIP_KE_SHIFT`, `r_q12`
    /// vcounts/ccount Q4.12. An unset Ke (0) yields no estimate: a zero
    /// would read as "at rest" to the velocity loop, the runaway seed.
    pub fn close_half(&mut self, r_q12: u16, recip_ke_q: u16, recip_arr_q24: u32) -> Option<i32> {
        self.fold(self.half_valid, PAIR, r_q12, 0, recip_ke_q, recip_arr_q24)
    }

    /// The kernel's close: `close_half` behind the start-up half and the
    /// drive flip. `duty_q15` is the duty that drove the half's last
    /// sample. Its sign against the last close's names a flip: the half
    /// holds rows of both signs, so it is void, and the halves behind it
    /// start up again. A zero duty drove nothing and has no sign: it is
    /// no flip, the sign a later drive is compared with stays the last one
    /// that drove, and the half is void, as its last sample had no window
    /// (`window::select`). A sign that changes and changes back between
    /// two closes is not seen. `xw_q12` is the brush term's size
    /// (`brush_xw_q12`).
    pub fn close(
        &mut self,
        duty_q15: i16,
        r_q12: u16,
        xw_q12: i32,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        let duty = duty_q15 as i32;
        if duty == 0 {
            self.flip = false;
            return self.fold(false, STARTUP, r_q12, xw_q12, recip_ke_q, recip_arr_q24);
        }
        let sign = (duty >> 31) | 1;
        let flip = sign + self.drive as i32 == 0;
        self.drive = sign as i8;
        self.flip = flip;
        self.fold(
            self.half_valid & !flip,
            STARTUP,
            r_q12,
            xw_q12,
            recip_ke_q,
            recip_arr_q24,
        )
    }

    /// Whether the last `close` saw the drive's sign change: set for the
    /// one period behind that close.
    pub fn flipped(&self) -> bool {
        self.flip
    }

    /// Folds the open half, `valid` or not, and pairs it with the one
    /// before when the halves `need` names were valid. `xw` is the brush
    /// term's size, `r_q12` units, 0 to `i16::MAX`.
    #[inline(always)]
    fn fold(
        &mut self,
        valid: bool,
        need: u8,
        r_q12: u16,
        xw: i32,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        let hist = self.hist;
        self.hist = (hist << 1) | valid as u8;
        let out = if valid {
            // sum(v_mean) - R * sum(i): |sum_i| <= 10 * 4095 keeps the R
            // product i64-exact for any gain encoding
            let v = q_mul(self.sum_tv, recip_arr_q24 as i32, RECIP_ARR_SHIFT);
            // i_avg sits between past half sums, so |dev| is under twice
            // the largest |sum_i|
            let dev = if hist & 1 != 0 {
                let dev = self.sum_i - self.i_avg;
                self.i_avg += dev >> BRUSH_AVG_SHIFT;
                dev
            } else {
                self.i_avg = self.sum_i;
                0
            };
            // both drops are bounded far under i32 (|R x sum_i| < 2^26, the
            // brush product per `brush_dev_max_fits`), so one saturation
            // holds the pair
            let dev_max = self.dev_max;
            let half = v.saturating_sub(
                q_mul(r_q12 as i32, self.sum_i, 12) + ((xw * dev.max(-dev_max).min(dev_max)) >> 12),
            );
            let prev = self.prev_half;
            self.prev_half = half;
            if hist & need == need && recip_ke_q != 0 {
                // (a + b) / BOXCAR_TICKS * recip_ke in one truncation: the
                // reciprocal product <= 65535 * 3277 stays i32, the i64
                // widen holds |a + b| * that for any half sum
                Some(q_mul(
                    prev.saturating_add(half),
                    recip_ke_q as i32 * RECIP_BOXCAR,
                    RECIP_KE_SHIFT + RECIP_BOXCAR_SHIFT,
                ))
            } else {
                None
            }
        } else {
            None
        };
        self.sum_tv = 0;
        self.sum_i = 0;
        self.half_valid = true;
        out
    }
}

/// Saturating cast matching telemetry `est.omega_bemf_cps`.
pub fn omega_cps_i16(omega: i32) -> i16 {
    omega.max(i16::MIN as i32).min(i16::MAX as i32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::window::I_READING_MAX;

    const ARR: u32 = 1200;
    const RECIP_ARR: u32 = (1u32 << RECIP_ARR_SHIFT) / ARR;
    // unity Ke: 1.0 c/s per vcount
    const KE_UNITY: u16 = 1 << RECIP_KE_SHIFT;
    const HALF: usize = DECIM_MED as usize;
    /// The development board's bound (`the_bound_is_40_ma_a_tick_through_the_dev_boards_chain`).
    const DEV_MAX: i32 = 440;

    /// Feed one whole half of identical samples and close it.
    fn half(obs: &mut BemfObs, ticks: u32, vdiff: Option<i32>, i: Option<i32>) -> Option<i32> {
        for _ in 0..HALF {
            obs.sample(ticks, vdiff, i);
        }
        obs.close_half(4096, KE_UNITY, RECIP_ARR)
    }

    /// Two clean halves of a constant pair with explicit gains/reciprocal.
    fn boxcar(ticks: u32, vdiff: i32, i: i32, r_q12: u16, recip_ke: u16, recip_arr: u32) -> i32 {
        let mut obs = BemfObs::new(DEV_MAX);
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
        let mut obs = BemfObs::new(DEV_MAX);
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
        let mut obs = BemfObs::new(DEV_MAX);
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
            let mut obs = BemfObs::new(DEV_MAX);
            half(&mut obs, 600, Some(3000), Some(0));
            obs.sample(600, vdiff, i);
            for _ in 1..HALF {
                obs.sample(600, Some(3000), Some(0));
            }
            assert_eq!(obs.close_half(4096, KE_UNITY, RECIP_ARR), None);
        }
    }

    #[test]
    fn zero_recip_ke_yields_no_estimate() {
        let mut obs = BemfObs::new(DEV_MAX);
        half(&mut obs, 600, Some(3000), Some(0));
        for _ in 0..HALF {
            obs.sample(600, Some(3000), Some(0));
        }
        assert_eq!(obs.close_half(4096, 0, RECIP_ARR), None);
        // the halves kept folding: the next clean close pairs as usual
        assert_eq!(half(&mut obs, 600, Some(3000), Some(0)), Some(1499));
    }

    /// `close` on any run of halves is the plain pairing behind the two
    /// rules, each stated here on its own: a half is void on a sub-floor
    /// sample, which the last sample under a zero duty is, or when the
    /// duty at its close has the other sign from the last nonzero duty at
    /// a close, and a boxcar needs the half ahead of its two valid as
    /// well. The duty takes both extremes, zero and small values of either
    /// sign; the flag is that close's flip alone.
    #[test]
    fn close_is_the_plain_pairing_behind_the_startup_half_and_the_flip() {
        const DUTIES: [i16; 8] = [i16::MAX, 1, 0, -1, i16::MIN, 8000, 0, -8000];
        let (mut obs, mut plain) = (BemfObs::new(DEV_MAX), BemfObs::new(DEV_MAX));
        let mut lcg = 0x2545_f491u32;
        let mut next = move || {
            lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            lcg >> 16
        };
        // the last sign that drove, and the valid halves in a row
        let (mut drove, mut run) = (0i32, 0u32);
        let (mut boxcars, mut flips, mut held, mut stops) = (0u32, 0u32, 0u32, 0u32);
        for _ in 0..20_000 {
            // runs of one duty, so most closes see no flip
            let duty = if next() % 4 == 0 {
                DUTIES[next() as usize % DUTIES.len()]
            } else {
                drove as i16 * 8000
            };
            let sub_floor = next() % 8 == 0;
            let v = if duty < 0 { -3000 } else { 3000 };
            for n in 0..HALF {
                let row = if (sub_floor && n == 3) || (duty == 0 && n == HALF - 1) {
                    (0, None, None)
                } else {
                    (600, Some(v), Some(v / 30))
                };
                obs.sample(row.0, row.1, row.2);
                plain.sample(row.0, row.1, row.2);
            }
            let sign = (duty as i32).signum();
            let flip = sign != 0 && sign == -drove;
            if sign != 0 {
                drove = sign;
            }
            if flip {
                plain.sample(0, None, None);
            }
            let pair = plain.close_half(4096, KE_UNITY, RECIP_ARR);
            let want = pair.filter(|_| run >= 2);
            run = if sub_floor || flip || sign == 0 {
                0
            } else {
                run + 1
            };
            assert_eq!(obs.close(duty, 4096, 0, KE_UNITY, RECIP_ARR), want);
            assert_eq!(obs.flipped(), flip);
            boxcars += want.is_some() as u32;
            flips += flip as u32;
            held += (pair.is_some() && want.is_none()) as u32;
            stops += (sign == 0) as u32;
        }
        assert!(
            boxcars > 2000 && flips > 1000 && held > 1000 && stops > 1000,
            "{boxcars} boxcars, {flips} flips, {held} held off, {stops} stops"
        );
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
    fn legal_extremes_never_wrap() {
        // debug overflow checks are the wrap detector: the widest window the
        // sum holds for times a full-scale 12-bit differential, and the
        // largest reading `Fast::measure` takes
        const V: i32 = TAP_FULL_SCALE as i32;
        let mut obs = BemfObs::new(DEV_MAX);
        // each half one sign, so the sums reach their extremes
        for n in 0..4 * HALF {
            let neg = (n / HALF) & 1 == 1;
            let v = if neg { -V } else { V };
            let i = if neg { -I_READING_MAX } else { I_READING_MAX };
            obs.sample(ARR_MAX, Some(v), Some(i));
            if n % HALF == HALF - 1 {
                obs.close_half(u16::MAX, u16::MAX, u32::MAX);
            }
        }
    }

    /// The largest bound `brush_dev_max_fits` passes.
    const DEV_MAX_FIT: i32 = i32::MAX / i16::MAX as i32;

    #[test]
    fn legal_extremes_never_wrap_at_the_largest_size_and_bound() {
        const V: i32 = TAP_FULL_SCALE as i32;
        let xw = brush_xw_q12(i32::MAX, brush_rc_q12(u16::MAX, u16::MAX, u16::MAX), 0);
        assert_eq!(xw, i16::MAX as i32);
        assert!(brush_dev_max_fits(DEV_MAX_FIT));
        let mut obs = BemfObs::new(DEV_MAX_FIT);
        // one drive sign throughout, so every half after the first is
        // valid and the deviation swings the whole current range
        for n in 0..8 * HALF {
            let neg = (n / HALF) & 1 == 1;
            let v = if neg { -V } else { V };
            let i = if neg { -I_READING_MAX } else { I_READING_MAX };
            obs.sample(ARR_MAX, Some(v), Some(i));
            if n % HALF == HALF - 1 {
                obs.close(i16::MAX, u16::MAX, xw, u16::MAX, u32::MAX);
            }
        }
    }

    #[test]
    fn the_bound_keeps_the_product_one_i32_multiply() {
        assert!(brush_dev_max_fits(0) && brush_dev_max_fits(DEV_MAX));
        assert!(!brush_dev_max_fits(DEV_MAX_FIT + 1) && !brush_dev_max_fits(-1));
        // the widest sense chain saturates its counts and does not fit
        assert!(!brush_dev_max_fits(brush_dev_max(u16::MAX, u16::MAX, 1)));
    }

    #[test]
    fn the_bound_is_40_ma_a_tick_through_the_dev_boards_chain() {
        // 60 mOhm, G 14.884, 3300 mV: 44 counts a tick, ten ticks a half
        assert_eq!(current_counts(BRUSH_DEV_MA, 60, 14_884, 3300), 44);
        assert_eq!(brush_dev_max(60, 14_884, 3300), DEV_MAX);
    }

    const R_RIG: u16 = 3744;
    const KE_RIG: u16 = 3700;
    /// The set of record's brush resistance and its full-weight speed, and
    /// the fade's low end under it.
    const RC: i32 = 1093;
    const CPS: i32 = 2540;
    const LO: i32 = CPS - BRUSH_SPAN_CPS;

    /// An observer under a brush term and its size-0 twin, fed the same
    /// halves under `close`.
    struct Twins {
        w: BemfObs,
        z: BemfObs,
        xw: i32,
    }

    impl Twins {
        fn new(xw: i32) -> Self {
            Self {
                w: BemfObs::new(DEV_MAX),
                z: BemfObs::new(DEV_MAX),
                xw,
            }
        }

        /// One half at `i` a tick, `step` more on its first sample, its
        /// first sample sub-floor when `void`; what the weighted close
        /// took off the half beyond the twin's.
        fn half(&mut self, i: i32, step: i32, void: bool) -> i32 {
            for n in 0..HALF {
                let row = match (n, void) {
                    (0, true) => (0, None, None),
                    (0, false) => (600, Some(3000), Some(i + step)),
                    _ => (600, Some(3000), Some(i)),
                };
                self.w.sample(row.0, row.1, row.2);
                self.z.sample(row.0, row.1, row.2);
            }
            self.w.close(8000, R_RIG, self.xw, KE_RIG, RECIP_ARR);
            self.z.close(8000, R_RIG, 0, KE_RIG, RECIP_ARR);
            self.z.prev_half - self.w.prev_half
        }
    }

    /// A weight of 0, at any speed under the fade or with no size, is the
    /// close without the term over varied halves: the fold written out.
    #[test]
    fn weight_zero_is_the_close_without_the_term() {
        let (mut plain, mut obs) = (BemfObs::new(DEV_MAX), BemfObs::new(DEV_MAX));
        let mut lcg = 0x1234_5678u32;
        let mut next = move || {
            lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (lcg >> 8) as i32
        };
        // the half's volts less R x its current, paired with the half
        // before when both were valid
        let (mut prev, mut prev_valid, mut run) = (0i32, false, 0u32);
        let (mut boxcars, mut moved) = (0u32, 0u32);
        for n in 0..5_000 {
            let (mut sum_tv, mut sum_i, mut valid) = (0i32, 0i32, true);
            let level = next() % 1500;
            for _ in 0..HALF {
                let void = next() % 64 == 0;
                let ticks = (next() % 1200) as u32;
                let v = next() % 4096;
                let i = level + next() % 600 - 300;
                let row = if void {
                    (0, None, None)
                } else {
                    (ticks, Some(v), Some(i))
                };
                if void {
                    valid = false;
                } else {
                    sum_tv += ticks as i32 * v;
                    sum_i += i;
                }
                plain.sample(row.0, row.1, row.2);
                obs.sample(row.0, row.1, row.2);
            }
            let omega_q16 = (next() % (LO + 1)) << 16;
            let xw = if n % 2 == 0 {
                brush_xw_q12(if n % 4 == 0 { omega_q16 } else { -omega_q16 }, RC, LO)
            } else {
                brush_xw_q12(i32::MAX, brush_rc_q12(0, CPS as u16, R_RIG), LO)
            };
            assert_eq!(xw, 0);
            let want = if valid {
                let v = q_mul(sum_tv, RECIP_ARR as i32, RECIP_ARR_SHIFT);
                let half = v.saturating_sub(q_mul(R_RIG as i32, sum_i, 12));
                let out = prev_valid.then(|| {
                    q_mul(
                        prev.saturating_add(half),
                        KE_RIG as i32 * RECIP_BOXCAR,
                        RECIP_KE_SHIFT + RECIP_BOXCAR_SHIFT,
                    )
                });
                prev = half;
                out
            } else {
                None
            };
            assert_eq!(plain.close_half(R_RIG, KE_RIG, RECIP_ARR), want);
            assert_eq!(
                obs.close(8000, R_RIG, xw, KE_RIG, RECIP_ARR),
                want.filter(|_| run >= 2)
            );
            assert_eq!(obs.prev_half, plain.prev_half);
            moved += (obs.i_avg != 0) as u32;
            boxcars += want.is_some() as u32;
            prev_valid = valid;
            run = if valid { run + 1 } else { 0 };
        }
        assert!(boxcars > 2000 && moved > 2000, "{boxcars} boxcars, {moved}");
    }

    #[test]
    fn a_small_deviation_takes_the_size_times_the_deviation() {
        let full = brush_xw_q12(CPS << 16, RC, LO);
        let half_w = brush_xw_q12((LO + 512) << 16, RC, LO);
        assert_eq!((full, half_w), (RC, (RC * 128) >> 8));
        for d in [1, 37, 400, DEV_MAX, -1, -37, -400, -DEV_MAX] {
            let mut t = Twins::new(full);
            assert_eq!(t.half(300, 0, false), 0);
            assert_eq!(t.half(300, 0, false), 0);
            let took = t.half(300, d, false);
            assert_eq!(took, (RC * d) >> 12, "dev {d}");
            assert!(d.abs() < 4 || took != 0, "dev {d}");
            let mut t = Twins::new(half_w);
            t.half(300, 0, false);
            assert_eq!(t.half(300, d, false), (half_w * d) >> 12, "dev {d}");
        }
    }

    #[test]
    fn a_large_current_step_takes_no_more_than_the_bound() {
        // the product floors, so the bound's negative side is one further
        let (lo, hi) = ((RC * -DEV_MAX) >> 12, (RC * DEV_MAX) >> 12);
        for d in [DEV_MAX + 1, 1000, 20_000, -DEV_MAX - 1, -1000, -20_000] {
            let mut t = Twins::new(RC);
            t.half(300, 0, false);
            assert_eq!(t.half(300, d, false), (RC * d.signum() * DEV_MAX) >> 12);
            // a held step: the average closes in, the term never passes
            // the bound
            let mut t = Twins::new(RC);
            t.half(300, 0, false);
            for _ in 0..200 {
                let took = t.half(300 + d, 0, false);
                assert!((lo..=hi).contains(&took), "dev {d}: {took}");
            }
        }
    }

    #[test]
    fn a_steady_current_takes_nothing_once_the_average_settles() {
        let mut t = Twins::new(RC);
        for _ in 0..100 {
            assert_eq!(t.half(500, 0, false), 0);
        }
        // the level falls and holds: the average closes on it
        let mut t = Twins::new(RC);
        t.half(800, 0, false);
        let mut took = 0;
        for _ in 0..8 << BRUSH_AVG_SHIFT {
            took = t.half(500, 0, false);
        }
        assert_eq!(took, 0);
    }

    #[test]
    fn the_first_valid_half_after_a_void_one_restarts_the_average() {
        let mut t = Twins::new(RC);
        for _ in 0..3 {
            t.half(300, 0, false);
        }
        t.half(300, 0, true);
        // a level 6000 counts a half away from the old average
        assert_eq!(t.half(900, 0, false), 0);
        assert_eq!(t.half(900, 0, false), 0);
        assert_eq!(t.half(900, 30, false), (RC * 30) >> 12);
    }

    #[test]
    fn a_void_half_leaves_the_average_alone() {
        let mut t = Twins::new(RC);
        t.half(300, 0, false);
        let avg = t.w.i_avg;
        t.half(5000, 0, true);
        assert_eq!(t.w.i_avg, avg);
    }

    #[test]
    fn the_weight_ramps_over_the_fade_in_either_sign() {
        // a size of 256 reads the weight itself
        for (cps, w) in [
            (0, 0),
            (LO - 1, 0),
            (LO, 0),
            (LO + 4, 1),
            (LO + 512, 128),
            (CPS - 1, 255),
            (CPS, 256),
            (CPS + 1, 256),
            (32_767, 256),
        ] {
            assert_eq!(brush_xw_q12(cps << 16, 256, LO), w, "{cps}");
            assert_eq!(brush_xw_q12((cps << 16) | 0xffff, 256, LO), w, "{cps}");
            assert_eq!(brush_xw_q12(-(cps << 16), 256, LO), w, "{cps}");
        }
        assert_eq!(brush_xw_q12(i32::MAX, 256, LO), 256);
        assert_eq!(brush_xw_q12(i32::MIN, 256, LO), 256);
        assert_eq!((LO, LO + 512), (1516, 2028));
    }

    #[test]
    fn the_take_caps_the_size_at_half_the_winding() {
        for (x, r_q12, rc) in [
            (RC as u16, R_RIG, RC),
            (5000, R_RIG, (R_RIG >> 1) as i32),
            (RC as u16, 2 * RC as u16, RC),
            (RC as u16, 2 * RC as u16 - 1, RC - 1),
            (RC as u16, 0, 0),
            (u16::MAX, u16::MAX, i16::MAX as i32),
            (0, R_RIG, 0),
        ] {
            assert_eq!(brush_rc_q12(x, CPS as u16, r_q12), rc, "{x} {r_q12}");
        }
    }

    #[test]
    fn a_full_weight_speed_under_the_span_turns_the_term_off() {
        for cps in [0, 1, BRUSH_SPAN_CPS as u16 - 1] {
            let rc = brush_rc_q12(RC as u16, cps, R_RIG);
            assert_eq!(rc, 0, "{cps}");
            assert_eq!(brush_xw_q12(i32::MAX, rc, cps as i32 - BRUSH_SPAN_CPS), 0);
        }
        assert_eq!(brush_rc_q12(RC as u16, BRUSH_SPAN_CPS as u16, R_RIG), RC);
    }

    #[test]
    fn both_fields_zero_turn_the_term_off() {
        let rc = brush_rc_q12(0, 0, R_RIG);
        assert_eq!(rc, 0);
        assert_eq!(brush_xw_q12(i32::MAX, rc, -BRUSH_SPAN_CPS), 0);
    }
}
