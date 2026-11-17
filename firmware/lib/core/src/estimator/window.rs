//! Drive-window selection for the twin per-period ADC scans. Center-aligned
//! PWMMODE1 with the chip-side CCR mapping (STATIC_HIGH + arr-ticks) puts
//! the drive window at the counter PEAK for either drive direction, and the
//! trough in the brake half. The shunt is direction-blind, so the signed
//! current takes its sign from the commanded duty.

use crate::estimator::bemf::RECIP_ARR_SHIFT;
use crate::math::{q_mul, q_mul_u};
use crate::{Frame, Shared};

/// Whether the drive window is wide enough for each sense path to have
/// settled.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WindowSel {
    pub i_valid: bool,
    pub v_valid: bool,
}

/// `drive_ticks` is the TIM1 compare width the motor write used; the caller
/// passes 0 for Coast/Brake/Disabled or zero duty, which invalidates both
/// paths even against a zero floor. Floors are board-data minimum widths
/// (`i_window_min_ticks` / `v_window_min_ticks`).
pub fn select(drive_ticks: u32, i_floor: u16, v_floor: u16) -> WindowSel {
    WindowSel {
        i_valid: drive_ticks != 0 && drive_ticks >= i_floor as u32,
        v_valid: drive_ticks != 0 && drive_ticks >= v_floor as u32,
    }
}

/// Band width of `SettleGain`, 8 ticks.
const SETTLE_BAND_SHIFT: u32 = 3;

/// Board data: the inverse of the shunt amplifier's step response at the
/// sample instant, by drive width. A window shorter than the amplifier
/// takes to settle samples the shunt short of its plateau by a fraction
/// the width alone fixes; the gain restores it.
#[derive(Copy, Clone)]
pub struct SettleGain {
    /// Drive width (TIM1 ticks) the first band starts at; narrower windows
    /// take the first band.
    pub start_ticks: u16,
    /// Q15 gain (1.0 = 32768) per 8-tick band from `start_ticks`; windows
    /// past the last band read at unity.
    pub q15: &'static [u16],
}

impl SettleGain {
    pub const UNITY: Self = Self {
        start_ticks: 0,
        q15: &[],
    };

    pub fn q15_at(&self, drive_ticks: u32) -> u32 {
        let band = drive_ticks.saturating_sub(self.start_ticks as u32) >> SETTLE_BAND_SHIFT;
        self.q15.get(band as usize).map_or(1 << 15, |&g| g as u32)
    }
}

/// Bias-subtracted shunt current from the drive-window scan along the
/// drive, scaled by the window's settle gain (`SettleGain::q15_at`), or
/// `None` when the window is too narrow for a settled sample: the terminal
/// current, ahead of the window term and the drive's sign. The sample
/// reads a wide code as the middle of its band (`Shared::adc_mid`), ahead
/// of the bias.
pub fn i_from_frame(
    shared: &Shared,
    frame: Frame<'_>,
    sel: WindowSel,
    bias: u16,
    gain_q15: u32,
) -> Option<i32> {
    if !sel.i_valid {
        return None;
    }
    let sample = shared.adc_mid(frame.current());
    // round half up: floor((floor(2x) + 1) / 2) == floor(x + 1/2)
    Some((q_mul(sample as i32 - bias as i32, gain_q15 as i32, 14) + 1) >> 1)
}

/// The largest |i| `i_from_frame` returns: any u16 sample, a wide one
/// read up by the largest slot, and any bias under a u16 settle gain.
#[cfg(test)]
pub(crate) const I_FROM_FRAME_MAX: i32 =
    ((((u16::MAX as i64 + crate::adc::ADC_WIDE_MID_MAX as i64) * u16::MAX as i64) >> 14) as i32
        + 1)
        >> 1;

/// The largest |i| the kernel's reading takes: `i_from_frame`'s, less the
/// largest window term on the drive's side.
#[cfg(test)]
pub(crate) const I_READING_MAX: i32 = I_FROM_FRAME_MAX + WINDOW_V_MAX as i32;

/// The rail the window term reads is held at most this, vcounts: a 12-bit
/// tap under a rail scale below 2.0 reads under it. A conductance under one
/// count a vcount keeps the term at most it, and a 12-bit reading less the
/// term inside i16.
pub const WINDOW_V_MAX: u16 = (1 << 13) - 1;

/// The narrowest PWM period whose conductance per tick
/// (`WindowTerm::conductance`) a u32 holds: under it `recip_arr_q24`
/// passes 2^16.
pub const WINDOW_ARR_MIN: u16 = 256;

/// The current the shunt's drive-window sample reads beyond the winding's
/// (kernel sec 2.2): the motor's loss conductance times the volts the
/// window leaves across the inductance, `g x (1 - D) x V_bus` once the
/// current settles, along the drive. `k` is Q16 counts per tick of the
/// headroom `pwm_arr - ticks`, `k0` the whole period's; zero is no term.
/// Each take returns whole counts and keeps the dropped fraction in
/// `carry` for the next, so the takes sum to the Q16 term within half a
/// count: a whole-count term alone loses the part the duty's dither moves
/// by under a count.
#[derive(Copy, Clone)]
pub struct WindowTerm {
    k: u32,
    k0: u32,
    carry: u16,
}

impl Default for WindowTerm {
    /// No term, the carry at half a count: the first take rounds.
    fn default() -> Self {
        Self {
            k: 0,
            k0: 0,
            carry: 1 << 15,
        }
    }
}

impl WindowTerm {
    /// The take's conductance per tick of the period, Q40 counts per
    /// vcount: CALIB `g_window_q016` while the winding bounds it,
    /// `0 < g x r_q12 <= 2^25` (g R at most 1/8), else 0. Needs
    /// `pwm_arr >= WINDOW_ARR_MIN`.
    pub fn conductance(g_q016: u16, r_q12: u16, recip_arr_q24: u32) -> u32 {
        let fits = r_q12 != 0 && g_q016 as u32 * r_q12 as u32 <= 1 << 25;
        g_q016 as u32 * fits as u32 * recip_arr_q24
    }

    /// The term on the rail clamp's `vbus_eff`, held at most
    /// [`WINDOW_V_MAX`]: `k x pwm_arr <= g x V < 2^29`. The carry stays:
    /// it is under a count whatever term it was owed on.
    pub fn set_rail(&mut self, g_arr: u32, vbus_eff: u16, pwm_arr: u16) {
        let k = q_mul_u(g_arr, vbus_eff.min(WINDOW_V_MAX) as u32, RECIP_ARR_SHIFT);
        self.k = k;
        self.k0 = k * pwm_arr as u32;
    }

    /// Counts at a drive width of `ticks <= pwm_arr`, >= 0, and 0 at full
    /// duty; the fraction left carries into the next take.
    #[inline(always)]
    pub fn take(&mut self, ticks: u32) -> i32 {
        let acc = self.k0 - self.k * ticks + self.carry as u32;
        self.carry = acc as u16;
        (acc >> 16) as i32
    }

    /// The exact term at `ticks`, Q16 counts.
    #[cfg(test)]
    pub(crate) fn q16(&self, ticks: u32) -> u32 {
        self.k0 - self.k * ticks
    }
}

/// Drive-window terminal differential `va - vb`. Positive duty drives IN1
/// (TIM1 CH3, PC6) -> DRV8212P OUT1 -> MOT_A -> VSNA -> `vmotor_a`, so
/// `va - vb` is positive under forward drive, negative under reverse - the
/// sign convention downstream bemf math relies on. Both taps share the
/// board's terminal bias, so it cancels here. Each tap reads a wide code as
/// the middle of its band (`Shared::adc_mid`).
pub fn vdiff_from_frame(shared: &Shared, frame: Frame<'_>, sel: WindowSel) -> Option<i32> {
    if !sel.v_valid {
        return None;
    }
    let (va, vb) = (
        shared.adc_mid(frame.vmotor_a()),
        shared.adc_mid(frame.vmotor_b()),
    );
    Some(va as i32 - vb as i32)
}

/// A settled brake trough carries no drive current: both low-side FETs on,
/// the winding current recirculating inside the bridge and none of it
/// crossing the return-path shunt, so the reading is pure amplifier offset.
/// Zero ticks is Coast/Brake/Disabled (no PWM edge), so it is out.
///
/// The amplifier leaves the drive pulse saturated and its tail outlasts the
/// drive window's floor by far, so only a brake half of at least
/// `brake_min_ticks` (board data, `KernelTiming::bias_brake_min_ticks`)
/// reads the rest offset. A narrower one would teach the tracker a bias
/// tens of counts high and poison every current reading.
pub fn trough_is_brake(drive_ticks: u32, pwm_arr: u16, brake_min_ticks: u16) -> bool {
    let brake_ticks = (pwm_arr as u32).saturating_sub(drive_ticks);
    drive_ticks != 0 && brake_ticks != 0 && brake_ticks >= brake_min_ticks as u32
}

/// Mirrors chip-side `effort_to_ticks` (`mag * arr / 32767` approximated as
/// round(`mag * arr >> 15`)) so validity floors compare against the same
/// width the motor write programs.
pub fn drive_ticks(duty: i16, pwm_arr: u16) -> u32 {
    let mag = duty.unsigned_abs() as u32;
    (mag * pwm_arr as u32 + (1 << 14)) >> 15
}

/// The TEL `duty` a logical Q15 duty drives at `pwm_arr`: the
/// [`drive_ticks`] width, negative in reverse. Hosts quantize a goal with
/// it to compare against the rows.
pub fn duty_counts(duty_q15: i16, pwm_arr: u16) -> i16 {
    let ticks = drive_ticks(duty_q15, pwm_arr) as i16;
    if duty_q15 < 0 { -ticks } else { ticks }
}

/// Inverse of `drive_ticks`: the smallest duty magnitude whose window
/// `select` accepts against the `min_ticks` floor; full duty when none does.
/// `recip_arr_q24` per `bemf::RECIP_ARR_SHIFT`. The floored reciprocal reads
/// the quotient low by under two while `min_ticks <= 512`; the remainder
/// closes the gap, so no divide. Boot only: out of line, one copy.
#[inline(never)]
pub fn floor_duty(min_ticks: u16, pwm_arr: u16, recip_arr_q24: u32) -> u16 {
    // drive_ticks(d) >= t  <=>  d * arr >= (t << 15) - (1 << 14)
    let need = ((min_ticks.max(1) as u32) << 15) - (1 << 14);
    let est = q_mul_u(need, recip_arr_q24, RECIP_ARR_SHIFT);
    let short = need.saturating_sub(est * pwm_arr as u32);
    let d = est + (short != 0) as u32 + (short > pwm_arr as u32) as u32;
    d.min(i16::MAX as u32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adc::{ADC_WIDE_SLOTS, wide_q4};
    use crate::regions::TelemetryTick;
    use crate::{RegionStorage, SensorFrame};

    /// `f` landed in a tick block of its own.
    fn block(f: &SensorFrame) -> TelemetryTick {
        let mut t = TelemetryTick::default();
        // SAFETY: a local, written and read here alone.
        unsafe { f.land(&mut t) };
        t
    }

    fn i_from_frame(f: &SensorFrame, sel: WindowSel, bias: u16, gain: u32) -> Option<i32> {
        super::i_from_frame(&Shared::new(), Frame::of(&block(f)), sel, bias, gain)
    }

    fn vdiff_from_frame(f: &SensorFrame, sel: WindowSel) -> Option<i32> {
        super::vdiff_from_frame(&Shared::new(), Frame::of(&block(f)), sel)
    }

    fn frame() -> SensorFrame {
        SensorFrame {
            current: 2500,
            vmotor_a: 3000,
            vmotor_b: 100,
            ..Default::default()
        }
    }

    const UNITY: u32 = 1 << 15;

    const VALID: WindowSel = WindowSel {
        i_valid: true,
        v_valid: true,
    };

    #[test]
    fn floor_boundaries() {
        let s = select(239, 240, 220);
        assert!(!s.i_valid);
        assert!(s.v_valid);
        let s = select(240, 240, 220);
        assert!(s.i_valid);
        assert!(s.v_valid);
        let s = select(219, 240, 220);
        assert!(!s.i_valid);
        assert!(!s.v_valid);
    }

    #[test]
    fn zero_ticks_invalid_even_with_zero_floors() {
        let s = select(0, 0, 0);
        assert!(!s.i_valid);
        assert!(!s.v_valid);
    }

    #[test]
    fn current_bias_along_the_drive() {
        let f = frame();
        assert_eq!(i_from_frame(&f, VALID, 2048, UNITY), Some(452));
        // a sample below the bias reads against the drive
        let under = SensorFrame {
            current: 1500,
            ..Default::default()
        };
        assert_eq!(i_from_frame(&under, VALID, 2048, UNITY), Some(-548));
    }

    const GAIN: SettleGain = SettleGain {
        start_ticks: 40,
        q15: &[35143, 34194, 33734],
    };

    #[test]
    fn settle_gain_bands_start_at_their_edge() {
        assert_eq!(GAIN.q15_at(40), 35143);
        assert_eq!(GAIN.q15_at(47), 35143);
        assert_eq!(GAIN.q15_at(48), 34194);
        assert_eq!(GAIN.q15_at(55), 34194);
        assert_eq!(GAIN.q15_at(56), 33734);
        assert_eq!(GAIN.q15_at(63), 33734);
    }

    #[test]
    fn settle_gain_clamps_below_and_is_unity_above() {
        // narrower than the first band: the first band, never an index
        // under the table
        assert_eq!(GAIN.q15_at(39), 35143);
        assert_eq!(GAIN.q15_at(0), 35143);
        assert_eq!(GAIN.q15_at(64), UNITY);
        assert_eq!(GAIN.q15_at(u32::MAX), UNITY);
        for t in [0, 1, 40, 1200, u32::MAX] {
            assert_eq!(SettleGain::UNITY.q15_at(t), UNITY);
        }
    }

    #[test]
    fn current_gain_rounds_and_keeps_the_sign_symmetric() {
        let f = SensorFrame {
            current: 2348,
            ..Default::default()
        };
        // 300 x 33259 / 32768 = 304.49
        assert_eq!(i_from_frame(&f, VALID, 2048, 33259), Some(304));
        // 300 x 33300 / 32768 = 304.87
        assert_eq!(i_from_frame(&f, VALID, 2048, 33300), Some(305));
        // under the bias: -300 x 33259 / 32768 = -304.49
        assert_eq!(i_from_frame(&f, VALID, 2648, 33259), Some(-304));
    }

    #[test]
    fn current_unity_gain_is_exact() {
        for sample in [0, 1, 2047, 2048, 2049, 4095, u16::MAX] {
            let f = SensorFrame {
                current: sample,
                ..Default::default()
            };
            let want = sample as i32 - 2048;
            assert_eq!(i_from_frame(&f, VALID, 2048, UNITY), Some(want));
        }
    }

    #[test]
    fn current_invalid_window_is_none() {
        let f = frame();
        let sel = WindowSel {
            i_valid: false,
            v_valid: true,
        };
        assert_eq!(i_from_frame(&f, sel, 0, UNITY), None);
    }

    #[test]
    fn vdiff_sign_tracks_drive_direction() {
        assert!(vdiff_from_frame(&frame(), VALID).unwrap() > 0);
        let rev = SensorFrame {
            vmotor_a: 100,
            vmotor_b: 3000,
            ..Default::default()
        };
        assert_eq!(vdiff_from_frame(&rev, VALID), Some(-2900));
    }

    #[test]
    fn vdiff_invalid_window_is_none() {
        let f = frame();
        let sel = WindowSel {
            i_valid: true,
            v_valid: false,
        };
        assert_eq!(vdiff_from_frame(&f, sel), None);
    }

    #[test]
    fn trough_is_brake_only_inside_a_settled_slow_brake_phase() {
        assert!(trough_is_brake(1, 1200, 960));
        // 960 brake ticks, the last duty whose trough is settled; one tick
        // more of drive and the feed shuts off.
        assert!(trough_is_brake(240, 1200, 960));
        assert!(!trough_is_brake(241, 1200, 960));
        assert!(!trough_is_brake(600, 1200, 960));
        assert!(!trough_is_brake(0, 1200, 960));
        assert!(!trough_is_brake(1200, 1200, 0));
        assert!(!trough_is_brake(drive_ticks(i16::MAX, 1200), 1200, 0));
    }

    const ADC_MAX: u16 = 4095;

    #[test]
    fn i_from_frame_max_bounds_every_u16_input() {
        // without a map the two extremes read what they read before the
        // correction
        const RAW_MAX: i32 = (((u16::MAX as i64 * u16::MAX as i64) >> 14) as i32 + 1) >> 1;
        // the widest sample under the largest slot reaches the bound
        let widest = Shared::new();
        widest
            .table
            .with_mut(|t| t.calib.adc.adc_wide_mid_q4 = [u8::MAX; ADC_WIDE_SLOTS]);
        for (sample, bias) in [(u16::MAX, 0), (0, u16::MAX)] {
            let f = SensorFrame {
                current: sample,
                ..Default::default()
            };
            let i = i_from_frame(&f, VALID, bias, u16::MAX as u32).unwrap();
            assert_eq!(i.abs(), RAW_MAX, "{sample} {bias}");
            let i =
                super::i_from_frame(&widest, Frame::of(&block(&f)), VALID, bias, u16::MAX as u32)
                    .unwrap();
            assert!(i.abs() <= I_FROM_FRAME_MAX, "{sample} {bias}");
            assert_eq!(i.abs() == I_FROM_FRAME_MAX, sample == u16::MAX);
        }
        const { assert!(RAW_MAX < I_FROM_FRAME_MAX) };
    }

    #[test]
    fn i_from_frame_fits_i16_at_adc_extremes() {
        for sample in [0, ADC_MAX] {
            for bias in [0, ADC_MAX] {
                for gain in [0, UNITY, u16::MAX as u32] {
                    let f = SensorFrame {
                        current: sample,
                        ..Default::default()
                    };
                    let i = i_from_frame(&f, VALID, bias, gain).unwrap();
                    assert!(i16::try_from(i).is_ok(), "{sample} {bias} {gain}: {i}");
                }
            }
        }
    }

    #[test]
    fn vdiff_from_frame_fits_i16_at_adc_extremes() {
        for (va, vb) in [(0, ADC_MAX), (ADC_MAX, 0)] {
            let f = SensorFrame {
                vmotor_a: va,
                vmotor_b: vb,
                ..Default::default()
            };
            let v = vdiff_from_frame(&f, VALID).unwrap();
            assert!(i16::try_from(v).is_ok(), "{va} - {vb}: {v}");
        }
    }

    #[test]
    fn drive_ticks_pins_chip_mapping() {
        assert_eq!(drive_ticks(0, 1200), 0);
        assert_eq!(drive_ticks(i16::MAX, 1200), 1200);
        assert_eq!(drive_ticks(-i16::MAX, 1200), 1200);
        assert_eq!(drive_ticks(i16::MIN, 1200), 1200);
    }

    #[test]
    fn drive_ticks_monotonic() {
        let mut last = 0;
        for duty in (0..=i16::MAX).step_by(317) {
            let t = drive_ticks(duty, 1200);
            assert!(t >= last, "non-monotonic at duty={duty}: {last} -> {t}");
            last = t;
        }
    }

    fn recip(arr: u16) -> u32 {
        (1 << RECIP_ARR_SHIFT) / arr as u32
    }

    fn is_floor(d: u16, floor: u16, arr: u16) -> bool {
        let valid = |d: u16| select(drive_ticks(d as i16, arr), floor, 0).i_valid;
        valid(d) && (d == 0 || !valid(d - 1))
    }

    #[test]
    fn floor_duty_is_window_valid() {
        for floor in [100, 160, 240] {
            let d = floor_duty(floor, 1200, recip(1200));
            assert!(drive_ticks(d as i16, 1200) >= floor as u32);
            assert!(is_floor(d, floor, 1200), "floor {floor}: duty {d}");
        }
        assert_eq!(floor_duty(160, 1200, recip(1200)), 4356);
    }

    #[test]
    fn floor_duty_is_the_smallest_valid_duty() {
        for floor in 0..=1200 {
            let d = floor_duty(floor, 1200, recip(1200));
            assert!(is_floor(d, floor, 1200), "floor {floor}: duty {d}");
        }
        for arr in [600, 1000, 1199, 2400, 4800] {
            for floor in 0..=512.min(arr) {
                let d = floor_duty(floor, arr, recip(arr));
                assert!(is_floor(d, floor, arr), "arr {arr} floor {floor}: duty {d}");
            }
        }
    }

    #[test]
    fn floor_duty_saturates_past_full_duty() {
        assert_eq!(floor_duty(1201, 1200, recip(1200)), i16::MAX as u16);
        assert_eq!(floor_duty(u16::MAX, 1200, recip(1200)), i16::MAX as u16);
    }

    /// The shunt sample and each tap, with every other field of the frame
    /// on a code of its own off the wide ones.
    fn on_channel(code: u16) -> [(SensorFrame, &'static str); 3] {
        let rest = SensorFrame {
            current: 0x812,
            current_trough: 0x834,
            vmotor_a: 0x856,
            vmotor_a_trough: 0x878,
            vmotor_b: 0x09a,
            vmotor_b_trough: 0x0bc,
            ..Default::default()
        };
        let mut out = [(rest, "shunt"), (rest, "tap a"), (rest, "tap b")];
        out[0].0.current = code;
        out[1].0.vmotor_a = code;
        out[2].0.vmotor_b = code;
        out
    }

    /// The current and the differential a frame reads with no correction
    /// at all.
    fn raw_reads(f: &SensorFrame, bias: u16) -> (i32, i32) {
        (
            f.current as i32 - bias as i32,
            f.vmotor_a as i32 - f.vmotor_b as i32,
        )
    }

    fn reads(sh: &Shared, f: &SensorFrame, bias: u16) -> (i32, i32) {
        let t = block(f);
        (
            super::i_from_frame(sh, Frame::of(&t), VALID, bias, UNITY).unwrap(),
            super::vdiff_from_frame(sh, Frame::of(&t), VALID).unwrap(),
        )
    }

    /// An all-zero map changes no reading: for every 12-bit code on the
    /// shunt sample and on each tap, the current and the differential are
    /// the raw subtractions.
    #[test]
    fn a_zero_map_reads_every_window_code_raw() {
        let sh = Shared::new();
        for code in 0..=ADC_MAX {
            for (f, name) in on_channel(code) {
                assert_eq!(reads(&sh, &f, 2048), raw_reads(&f, 2048), "{name} {code}");
            }
        }
    }

    /// A slot adds to the window's sample at exactly its stuck code, in
    /// whole counts rounded half up, on the channel that sits on the code
    /// and on no other: up on the shunt and on tap a, down on tap b, and
    /// nothing at any other code.
    #[test]
    fn a_slot_adds_at_its_stuck_code_on_its_channel_alone() {
        let map = [
            0, 1, 7, 8, 9, 15, 16, 23, 24, 25, 100, 160, 200, 247, 248, 255,
        ];
        let sh = Shared::new();
        sh.table.with_mut(|t| t.calib.adc.adc_wide_mid_q4 = map);
        let mut added = 0;
        for code in 0..=ADC_MAX {
            let mid = ((wide_q4(code, &map) as i32) + 8) >> 4;
            added += (mid != 0) as u32;
            for (n, (f, name)) in on_channel(code).into_iter().enumerate() {
                let (i, v) = raw_reads(&f, 2048);
                let want = match n {
                    0 => (i + mid, v),
                    1 => (i, v + mid),
                    _ => (i, v - mid),
                };
                assert_eq!(reads(&sh, &f, 2048), want, "{name} {code}");
            }
        }
        // slots 0, 1 and 7 round to nothing; the other thirteen add
        assert_eq!(added, 13);
    }

    /// The set of record's conductance (4.75 mS on the board's scales) and
    /// winding.
    const G: u16 = 1423;
    const R: u16 = 3725;

    /// Rails from none past the term's hold, vcounts.
    const RAILS: [u16; 7] = [0, 1, 922, 1238, 4095, WINDOW_V_MAX, u16::MAX];

    fn at_rail(g_arr: u32, vbus_eff: u16, pwm_arr: u16) -> WindowTerm {
        let mut w = WindowTerm::default();
        w.set_rail(g_arr, vbus_eff, pwm_arr);
        w
    }

    /// The take a fresh term reads first, the carry at its seed.
    fn first(mut w: WindowTerm, ticks: u32) -> i32 {
        w.take(ticks)
    }

    /// The carry's extremes: none and a whole count less one.
    const CARRIES: [u16; 2] = [0, u16::MAX];

    /// A zero conductance, a fenced-off one and the seed take nothing at
    /// any width or rail, take after take: the reading is the sample's, bit
    /// for bit.
    #[test]
    fn a_zero_window_conductance_reads_as_today() {
        assert_eq!(WindowTerm::conductance(0, R, recip(1200)), 0);
        assert_eq!(WindowTerm::conductance(G, 0, recip(1200)), 0);
        assert_eq!(WindowTerm::conductance(u16::MAX, R, recip(1200)), 0);
        let mut seed = WindowTerm::default();
        for arr in [WINDOW_ARR_MIN, 1200, u16::MAX] {
            for v in RAILS {
                let mut w = at_rail(0, v, arr);
                for t in 0..=arr as u32 {
                    assert_eq!(w.take(t), 0, "arr {arr} rail {v} width {t}");
                    assert_eq!(seed.take(t), 0);
                }
            }
        }
    }

    /// The term is `g x V x (arr - ticks) / arr`: in Q16 within the
    /// floors' share, a Q16 count a tick of headroom and the reciprocal's
    /// `arr / 2^24` of the term, at every width; a fresh term's first take
    /// is it rounded, within half a count and the floors; 0 at full duty
    /// whatever the carry; 25 counts at the 64-tick floor on the 5 V rail.
    #[test]
    fn the_term_is_the_conductance_on_the_headroom_and_the_rail() {
        for arr in [WINDOW_ARR_MIN, 1200, 2400] {
            let g_arr = WindowTerm::conductance(G, R, recip(arr));
            for v in RAILS {
                let w = at_rail(g_arr, v, arr);
                let rail = v.min(WINDOW_V_MAX) as f64;
                for t in 0..=arr as u32 {
                    let exact = G as f64 / 65536.0 * rail * (arr as u32 - t) as f64 / arr as f64;
                    let q16 = w.q16(t) as f64 / 65536.0;
                    let got = first(w, t) as f64;
                    let floors = arr as f64 / 65536.0 + exact * arr as f64 / (1u32 << 24) as f64;
                    assert!(
                        (q16 - exact).abs() <= floors,
                        "arr {arr} rail {v} width {t}: {q16:.4} for {exact:.4}"
                    );
                    assert!(
                        (got - exact).abs() <= 0.5 + floors,
                        "arr {arr} rail {v} width {t}: {got} for {exact:.3}"
                    );
                }
                for carry in CARRIES {
                    let mut w = WindowTerm { carry, ..w };
                    assert_eq!(w.take(arr as u32), 0, "arr {arr} rail {v}: full duty");
                }
            }
        }
        let w = at_rail(WindowTerm::conductance(G, R, recip(1200)), 1238, 1200);
        assert_eq!([64, 293, 1200].map(|t| first(w, t)), [25, 20, 0]);
    }

    /// The takes sum to the Q16 term within half a count, the seed's half
    /// less the carry left, over a duty that ramps 0.2 counts a tick across
    /// the period and back on the 5 V rail. A whole-count term alone holds
    /// each count 5 / 0.0224 ticks, 223 (0.0224 counts a tick of width),
    /// and its running sum strays by an eighth of that, 28 counts, at
    /// each count's middle: the pin asks for half.
    #[test]
    fn the_takes_sum_to_the_exact_term_over_a_slow_ramp() {
        let mut w = at_rail(WindowTerm::conductance(G, R, recip(1200)), 1238, 1200);
        let (mut exact, mut carried, mut whole) = (0i64, 0i64, 0i64);
        let (mut worst, mut strays) = (0i64, 0i64);
        let up = (0..5 * 1200).map(|n| n / 5);
        for t in up.clone().chain(up.rev()) {
            exact += w.q16(t) as i64;
            carried += (w.take(t) as i64) << 16;
            whole += ((w.q16(t) + (1 << 15)) >> 16) as i64;
            worst = worst.max((carried - exact).abs());
            strays = strays.max(((whole << 16) - exact).abs());
        }
        assert!(worst <= 1 << 15, "{} counts off", worst as f64 / 65536.0);
        assert!(
            strays > 14 << 16,
            "the whole-count sum strays {} counts",
            strays as f64 / 65536.0
        );
    }

    /// Every 12-bit reading less the largest term any field values give,
    /// the widest conductance on the narrowest period at any rail, stays
    /// inside i16, and every product inside its u32 (debug overflow
    /// checks).
    #[test]
    fn legal_extremes_keep_the_reading_inside_i16() {
        for arr in [WINDOW_ARR_MIN, 1200, u16::MAX] {
            let g_arr = WindowTerm::conductance(u16::MAX, 1, recip(arr));
            assert_ne!(g_arr, 0, "arr {arr}");
            for v in RAILS {
                for (t, carry) in [0, 1, arr as u32 / 2, arr as u32]
                    .into_iter()
                    .flat_map(|t| CARRIES.map(|c| (t, c)))
                {
                    let term = WindowTerm {
                        carry,
                        ..at_rail(g_arr, v, arr)
                    }
                    .take(t);
                    assert!(term <= WINDOW_V_MAX as i32, "arr {arr} rail {v} width {t}");
                    for sample in [0, ADC_MAX] {
                        for bias in [0, ADC_MAX] {
                            for gain in [0, UNITY, u16::MAX as u32] {
                                let f = SensorFrame {
                                    current: sample,
                                    ..Default::default()
                                };
                                let i = i_from_frame(&f, VALID, bias, gain).unwrap() - term;
                                assert!(
                                    i16::try_from(i).is_ok() && i16::try_from(-i).is_ok(),
                                    "{sample} {bias} {gain} less {term}: {i}"
                                );
                                assert!(i.abs() <= I_READING_MAX);
                            }
                        }
                    }
                }
            }
        }
    }
}
