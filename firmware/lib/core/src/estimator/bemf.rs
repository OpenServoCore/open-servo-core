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

use crate::kernel::DECIM_MED;
use crate::math::q_mul;

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
}

impl BemfObs {
    pub const fn new() -> Self {
        Self {
            sum_tv: 0,
            sum_i: 0,
            half_valid: true,
            prev_half: 0,
            hist: 0,
            drive: 0,
            flip: false,
        }
    }

    /// One FAST-tick sample. `ticks` is the drive width the window select
    /// used, `vdiff` from `vdiff_from_frame`, `i` from `i_from_frame`;
    /// either `None` voids the open half. A half is DECIM_MED samples of a
    /// width <= ARR times a 12-bit differential, so `sum_tv` holds for any
    /// ARR up to `ARR_MAX`; `sum_i` stays under 2^21.
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
        self.fold(self.half_valid, PAIR, r_q12, recip_ke_q, recip_arr_q24)
    }

    /// The kernel's close: `close_half` behind the start-up half and the
    /// drive flip. `duty_q15` is the duty that drove the half's last
    /// sample. Its sign against the last close's names a flip: the half
    /// holds rows of both signs, so it is void, and the halves behind it
    /// start up again. A zero duty drove nothing and has no sign: it is
    /// no flip, the sign a later drive is compared with stays the last one
    /// that drove, and the half is void, as its last sample had no window
    /// (`window::select`). A sign that changes and changes back between
    /// two closes is not seen.
    pub fn close(
        &mut self,
        duty_q15: i16,
        r_q12: u16,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        let duty = duty_q15 as i32;
        if duty == 0 {
            self.flip = false;
            return self.fold(false, STARTUP, r_q12, recip_ke_q, recip_arr_q24);
        }
        let sign = (duty >> 31) | 1;
        let flip = sign + self.drive as i32 == 0;
        self.drive = sign as i8;
        self.flip = flip;
        self.fold(
            self.half_valid & !flip,
            STARTUP,
            r_q12,
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
    /// before when the halves `need` names were valid.
    #[inline(always)]
    fn fold(
        &mut self,
        valid: bool,
        need: u8,
        r_q12: u16,
        recip_ke_q: u16,
        recip_arr_q24: u32,
    ) -> Option<i32> {
        let hist = self.hist;
        self.hist = (hist << 1) | valid as u8;
        let out = if valid {
            // sum(v_mean) - R * sum(i): |sum_i| <= 10 * 4095 keeps the R
            // product i64-exact for any gain encoding
            let v = q_mul(self.sum_tv, recip_arr_q24 as i32, RECIP_ARR_SHIFT);
            let half = v.saturating_sub(q_mul(r_q12 as i32, self.sum_i, 12));
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
    omega.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::window::I_FROM_FRAME_MAX;

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
    fn zero_recip_ke_yields_no_estimate() {
        let mut obs = BemfObs::new();
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
        let (mut obs, mut plain) = (BemfObs::new(), BemfObs::new());
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
            assert_eq!(obs.close(duty, 4096, KE_UNITY, RECIP_ARR), want);
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
        // largest current `i_from_frame` returns
        const V: i32 = TAP_FULL_SCALE as i32;
        let mut obs = BemfObs::new();
        // each half one sign, so the sums reach their extremes
        for n in 0..4 * HALF {
            let neg = (n / HALF) & 1 == 1;
            let v = if neg { -V } else { V };
            let i = if neg {
                -I_FROM_FRAME_MAX
            } else {
                I_FROM_FRAME_MAX
            };
            obs.sample(ARR_MAX, Some(v), Some(i));
            if n % HALF == HALF - 1 {
                obs.close_half(u16::MAX, u16::MAX, u32::MAX);
            }
        }
    }
}
