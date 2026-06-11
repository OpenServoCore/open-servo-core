//! Chip-side sensor pipeline. The 20 kHz control-loop ISR drains
//! `ADC_DMA_BUF` at scan-slot indices defined here, converts raw codes into
//! engineering units, and hands the result to the kernel via
//! `osc_core::Sensors::sample`. Math is V006-and-board-shaped: 12-bit ADC,
//! shunt-resistor current, NTC-thermistor temp, voltage-divider rails,
//! peak/trough sampling under center-aligned PWM.

use osc_core::{ConversionVariables, RawSamples, Sample, Sensors as SensorsTrait};
use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::cfg::{Calibration, Divider, NtcCal};
use crate::statics::{ADC_DMA_BUF, ADC_SCAN_LEN, read_sample_tick};

const ADC_MAX_RAW: u32 = 4095;

/// Q32 reciprocals so the 20 kHz ISR runs a single `mulhu` per conversion
/// instead of pulling in `__udivdi3` / `__divdi3`.
#[derive(Copy, Clone)]
pub struct Scales {
    pub vbus_q32: u32,
    pub vmotor_q32: u32,
    pub shunt_q32: u32,
}

impl Scales {
    pub const fn new(cal: &Calibration, gain_factor: u16) -> Self {
        Self {
            vbus_q32: divider_q32(&cal.vbus_divider),
            vmotor_q32: divider_q32(&cal.vmotor_divider),
            shunt_q32: shunt_q32(gain_factor, cal.shunt_r_mohm),
        }
    }
}

const fn divider_q32(d: &Divider) -> u32 {
    let bot = if d.bot_ohm == 0 { 1 } else { d.bot_ohm } as u64;
    let sum = d.top_ohm.saturating_add(d.bot_ohm) as u64;
    let num = sum << 32;
    let den = ADC_MAX_RAW as u64 * bot;
    let q = num / den;
    if q > u32::MAX as u64 {
        u32::MAX
    } else {
        q as u32
    }
}

const fn shunt_q32(gain_factor: u16, r_mohm: u16) -> u32 {
    let g = if gain_factor == 0 { 1 } else { gain_factor } as u64;
    let r = if r_mohm == 0 { 1 } else { r_mohm } as u64;
    let num = 1000u64 << 32;
    let den = ADC_MAX_RAW as u64 * g * r;
    let q = num / den;
    if q > u32::MAX as u64 {
        u32::MAX
    } else {
        q as u32
    }
}

// UG fires TRGO at CNT=0 before CEN=1, so the trough scan lands first.
const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
const SCAN_TROUGH_OFFSET: usize = 0;

const SCAN_IDX_SHUNT_POST: usize = 0;
const SCAN_IDX_POS: usize = 1;
const SCAN_IDX_NTC: usize = 2;
const SCAN_IDX_VMOTOR_A: usize = 3;
const SCAN_IDX_VMOTOR_B: usize = 4;
const SCAN_IDX_VCAL: usize = 5;

/// Read trough slots before peak; DMA overwrites trough first after TC.
#[inline(always)]
fn scan_slot(offset: usize, idx: usize) -> u16 {
    let i = offset + idx;
    debug_assert!(i < 2 * ADC_SCAN_LEN);
    // SAFETY: index bounded above; `ADC_DMA_BUF` is a fixed-length static.
    unsafe { (ADC_DMA_BUF.get() as *const u16).add(i).read_volatile() }
}

/// EWMA low-pass, α = 1/128. `state_q6` keeps 6 sub-LSB bits.
struct VcalLpf {
    state_q6: i32,
    initialized: bool,
}

impl VcalLpf {
    const fn new() -> Self {
        Self {
            state_q6: 0,
            initialized: false,
        }
    }

    fn update(&mut self, raw: u16) -> u16 {
        let x = (raw as i32) << 6;
        if !self.initialized {
            self.state_q6 = x;
            self.initialized = true;
        } else {
            self.state_q6 += (x - self.state_q6) >> 7;
        }
        (self.state_q6 >> 6) as u16
    }
}

fn divider_to_mv(raw: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let prod = raw as u32 * vdd_mv;
    let v_in = ((prod as u64) * scale_q32 as u64) >> 32;
    Millivolts(v_in.min(i16::MAX as u64) as i16)
}

fn vmotor_diff_mv(raw_a: u16, raw_b: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let a = divider_to_mv(raw_a, vdd_mv, scale_q32).0 as i32;
    let b = divider_to_mv(raw_b, vdd_mv, scale_q32).0 as i32;
    Millivolts((a - b).unsigned_abs().min(i16::MAX as u32) as i16)
}

fn shunt_to_milliamps(raw: u16, bias_raw: u16, vdd_mv: u32, scale_q32: u32) -> Milliamps {
    let signed = raw as i32 - bias_raw as i32;
    let mag = signed.unsigned_abs() as u64 * vdd_mv as u64;
    let i_ma_mag = ((mag * scale_q32 as u64) >> 32) as i32;
    let i_ma = if signed >= 0 { i_ma_mag } else { -i_ma_mag };
    Milliamps(i_ma.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
}

fn ntc_to_centi_celsius(_raw: u16, cal: &NtcCal) -> CentiCelsius {
    // TODO: full β-model needs fixed-point ln; pass-through returns T₀.
    let _ = (cal.beta, cal.r0_ohm, cal.bias_r_ohm);
    CentiCelsius(cal.t0_cc)
}

fn pos_to_microrads(raw: u16, min_urad: i32, max_urad: i32) -> Microrads {
    // `>>12` approximates `/4095` (1 part in 4096) to dodge __divdi3 in the ISR.
    let span = (max_urad as i64).wrapping_sub(min_urad as i64);
    let interp = (span * raw as i64) >> 12;
    Microrads(min_urad.saturating_add(interp as i32))
}

pub struct Ch32Sensors {
    calibration: Calibration,
    shunt_bias_raw: u16,
    scales: Scales,
    vcal_lpf: VcalLpf,
}

impl Ch32Sensors {
    pub const fn new(calibration: Calibration, shunt_bias_raw: u16, scales: Scales) -> Self {
        Self {
            calibration,
            shunt_bias_raw,
            scales,
            vcal_lpf: VcalLpf::new(),
        }
    }
}

impl SensorsTrait for Ch32Sensors {
    /// Called from DMA1 TC ISR. Peak drives current; trough is diagnostic.
    fn sample(&mut self, vars: &ConversionVariables) -> Sample {
        let raw_shunt_post_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_vmotor_a_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_B);

        let raw_shunt_post_peak = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_pos = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_POS);
        let raw_ntc = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_NTC);
        let raw_vmotor_a = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_B);
        let raw_vcal = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VCAL);
        let raw_vbus = 0u16;

        let filtered_vcal = self.vcal_lpf.update(raw_vcal);
        let vdd_mv = vars.vdd_mv as u32;

        let post_peak = shunt_to_milliamps(
            raw_shunt_post_peak,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );
        let post_trough = shunt_to_milliamps(
            raw_shunt_post_trough,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );

        Sample {
            tick: read_sample_tick(),
            pos: pos_to_microrads(raw_pos, vars.pos_min_phys_urad, vars.pos_max_phys_urad),
            current: post_peak,
            current_post_trough: post_trough,
            temp: ntc_to_centi_celsius(raw_ntc, &self.calibration.ntc),
            vbus: divider_to_mv(raw_vbus, vdd_mv, self.scales.vbus_q32),
            vmotor: vmotor_diff_mv(raw_vmotor_a, raw_vmotor_b, vdd_mv, self.scales.vmotor_q32),
            raw: RawSamples {
                pos: raw_pos,
                current: raw_shunt_post_peak,
                shunt_post_trough: raw_shunt_post_trough,
                temp: raw_ntc,
                vbus: raw_vbus,
                vmotor_a: raw_vmotor_a,
                vmotor_a_trough: raw_vmotor_a_trough,
                vmotor_b: raw_vmotor_b,
                vmotor_b_trough: raw_vmotor_b_trough,
                vcal: raw_vcal,
                vcal_lpf: filtered_vcal,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VDD_MV: u32 = 3300;
    const HALF_PI_URAD: i32 = 1_570_796;

    fn rev_b_calibration() -> Calibration {
        Calibration {
            shunt_r_mohm: 10,
            vbus_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            vmotor_divider: Divider {
                top_ohm: 20_000,
                bot_ohm: 10_000,
            },
            ntc: NtcCal {
                beta: 3950,
                r0_ohm: 10_000,
                t0_cc: 2500,
                bias_r_ohm: 10_000,
            },
        }
    }

    #[test]
    fn shunt_zero_at_bias() {
        let scale = shunt_q32(32, 10);
        assert_eq!(shunt_to_milliamps(2048, 2048, VDD_MV, scale), Milliamps(0));
    }

    #[test]
    fn shunt_sign_follows_bias_offset() {
        let scale = shunt_q32(32, 10);
        let above = shunt_to_milliamps(2148, 2048, VDD_MV, scale);
        let below = shunt_to_milliamps(1948, 2048, VDD_MV, scale);
        assert!(above.0 > 0, "above bias must be positive, got {}", above.0);
        assert!(below.0 < 0, "below bias must be negative, got {}", below.0);
        assert_eq!(above.0, -below.0);
    }

    #[test]
    fn shunt_saturates_to_i16_range() {
        let scale = shunt_q32(32, 10);
        let max_pos = shunt_to_milliamps(u16::MAX, 0, VDD_MV, scale);
        let max_neg = shunt_to_milliamps(0, u16::MAX, VDD_MV, scale);
        assert_eq!(max_pos, Milliamps(i16::MAX));
        assert_eq!(max_neg, Milliamps(i16::MIN));
    }

    #[test]
    fn divider_known_two_to_one() {
        let scale = divider_q32(&Divider {
            top_ohm: 20_000,
            bot_ohm: 10_000,
        });
        let v = divider_to_mv(2048, VDD_MV, scale);
        assert!((4900..=5000).contains(&v.0), "got {}", v.0);
    }

    #[test]
    fn divider_full_scale_caps_at_i16_max() {
        let scale = divider_q32(&Divider {
            top_ohm: 90_000,
            bot_ohm: 10_000,
        });
        let v = divider_to_mv(4095, VDD_MV, scale);
        assert_eq!(v, Millivolts(i16::MAX));
    }

    #[test]
    fn vmotor_diff_is_symmetric() {
        let scale = divider_q32(&Divider {
            top_ohm: 20_000,
            bot_ohm: 10_000,
        });
        let ab = vmotor_diff_mv(3000, 1000, VDD_MV, scale);
        let ba = vmotor_diff_mv(1000, 3000, VDD_MV, scale);
        assert_eq!(ab, ba);
    }

    #[test]
    fn vmotor_diff_zero_when_equal() {
        let scale = divider_q32(&Divider {
            top_ohm: 20_000,
            bot_ohm: 10_000,
        });
        assert_eq!(vmotor_diff_mv(2000, 2000, VDD_MV, scale), Millivolts(0));
    }

    #[test]
    fn pos_endpoints_match_range() {
        let lo = pos_to_microrads(0, -HALF_PI_URAD, HALF_PI_URAD);
        let hi = pos_to_microrads(4095, -HALF_PI_URAD, HALF_PI_URAD);
        assert_eq!(lo, Microrads(-HALF_PI_URAD));
        let span_per_lsb = (2 * HALF_PI_URAD as i64) / 4096;
        let err = (HALF_PI_URAD - hi.0) as i64;
        assert!(err <= span_per_lsb + 1, "err={err} lsb={span_per_lsb}");
    }

    #[test]
    fn pos_midpoint_near_zero() {
        let mid = pos_to_microrads(2048, -HALF_PI_URAD, HALF_PI_URAD);
        let span_per_lsb = (2 * HALF_PI_URAD as i64) / 4096;
        assert!(
            (mid.0 as i64).abs() <= span_per_lsb,
            "midpoint not near zero: {}",
            mid.0
        );
    }

    #[test]
    fn vcal_lpf_initialises_on_first_sample() {
        let mut lpf = VcalLpf::new();
        assert_eq!(lpf.update(1234), 1234);
    }

    #[test]
    fn vcal_lpf_constant_input_steady_state() {
        let mut lpf = VcalLpf::new();
        lpf.update(500);
        for _ in 0..256 {
            assert_eq!(lpf.update(500), 500);
        }
    }

    #[test]
    fn vcal_lpf_step_converges() {
        let mut lpf = VcalLpf::new();
        lpf.update(0);
        let mut last = 0;
        for _ in 0..2000 {
            last = lpf.update(1000) as i32;
        }
        assert!((last - 1000).abs() <= 2, "last={last}");
    }

    #[test]
    fn scales_new_returns_nonzero_q32() {
        let scales = Scales::new(&rev_b_calibration(), 32);
        assert!(scales.vbus_q32 > 0);
        assert!(scales.vmotor_q32 > 0);
        assert!(scales.shunt_q32 > 0);
    }
}
