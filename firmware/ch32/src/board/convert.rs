use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::statics::{ADC_DMA_BUF, ADC_SCAN_LEN};

use super::config::{Calibration, Divider, NtcCal};

const ADC_MAX_RAW: u32 = 4095;

/// Q32 reciprocals so the 20 kHz ISR runs a single `mulhu` per conversion
/// instead of pulling in `__udivdi3` / `__divdi3`.
#[derive(Copy, Clone)]
pub(super) struct Scales {
    pub(super) vbus_q32: u32,
    pub(super) vmotor_q32: u32,
    pub(super) shunt_q32: u32,
}

impl Scales {
    pub(super) fn new(cal: &Calibration, gain_factor: u16) -> Self {
        Self {
            vbus_q32: divider_q32(&cal.vbus_divider),
            vmotor_q32: divider_q32(&cal.vmotor_divider),
            shunt_q32: shunt_q32(gain_factor, cal.shunt_r_mohm),
        }
    }
}

fn divider_q32(d: &Divider) -> u32 {
    let bot = d.bot_ohm.max(1) as u64;
    let sum = d.top_ohm.saturating_add(d.bot_ohm) as u64;
    let num = sum << 32;
    let den = ADC_MAX_RAW as u64 * bot;
    (num / den).min(u32::MAX as u64) as u32
}

fn shunt_q32(gain_factor: u16, r_mohm: u16) -> u32 {
    let g = gain_factor.max(1) as u64;
    let r = r_mohm.max(1) as u64;
    let num = 1000u64 << 32;
    let den = ADC_MAX_RAW as u64 * g * r;
    (num / den).min(u32::MAX as u64) as u32
}

// UG fires TRGO at CNT=0 before CEN=1, so the trough scan lands first.
pub(super) const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
pub(super) const SCAN_TROUGH_OFFSET: usize = 0;

pub(super) const SCAN_IDX_SHUNT_POST: usize = 0;
pub(super) const SCAN_IDX_POS: usize = 1;
pub(super) const SCAN_IDX_NTC: usize = 2;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 3;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 4;
pub(super) const SCAN_IDX_VCAL: usize = 5;

/// Read trough slots before peak; DMA overwrites trough first after TC.
#[inline(always)]
pub(super) fn scan_slot(offset: usize, idx: usize) -> u16 {
    let i = offset + idx;
    debug_assert!(i < 2 * ADC_SCAN_LEN);
    // SAFETY: index bounded above; `ADC_DMA_BUF` is a fixed-length static.
    unsafe { (ADC_DMA_BUF.get() as *const u16).add(i).read_volatile() }
}

/// EWMA low-pass, α = 1/128. `state_q6` keeps 6 sub-LSB bits.
pub(super) struct VcalLpf {
    state_q6: i32,
    initialized: bool,
}

impl VcalLpf {
    pub(super) const fn new() -> Self {
        Self {
            state_q6: 0,
            initialized: false,
        }
    }

    pub(super) fn update(&mut self, raw: u16) -> u16 {
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

pub(super) fn divider_to_mv(raw: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let prod = raw as u32 * vdd_mv;
    let v_in = ((prod as u64) * scale_q32 as u64) >> 32;
    Millivolts(v_in.min(i16::MAX as u64) as i16)
}

pub(super) fn vmotor_diff_mv(raw_a: u16, raw_b: u16, vdd_mv: u32, scale_q32: u32) -> Millivolts {
    let a = divider_to_mv(raw_a, vdd_mv, scale_q32).0 as i32;
    let b = divider_to_mv(raw_b, vdd_mv, scale_q32).0 as i32;
    Millivolts((a - b).unsigned_abs().min(i16::MAX as u32) as i16)
}

pub(super) fn shunt_to_milliamps(
    raw: u16,
    bias_raw: u16,
    vdd_mv: u32,
    scale_q32: u32,
) -> Milliamps {
    let signed = raw as i32 - bias_raw as i32;
    let mag = signed.unsigned_abs() as u64 * vdd_mv as u64;
    let i_ma_mag = ((mag * scale_q32 as u64) >> 32) as i32;
    let i_ma = if signed >= 0 { i_ma_mag } else { -i_ma_mag };
    Milliamps(i_ma.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
}

pub(super) fn ntc_to_centi_celsius(_raw: u16, cal: &NtcCal) -> CentiCelsius {
    // TODO: full β-model needs fixed-point ln; pass-through returns T₀.
    let _ = (cal.beta, cal.r0_ohm, cal.bias_r_ohm);
    CentiCelsius(cal.t0_cc)
}

pub(super) fn pos_to_microrads(raw: u16, min_urad: i32, max_urad: i32) -> Microrads {
    // `>>12` approximates `/4095` (1 part in 4096) to dodge __divdi3 in the ISR.
    let span = (max_urad as i64).wrapping_sub(min_urad as i64);
    let interp = (span * raw as i64) >> 12;
    Microrads(min_urad.saturating_add(interp as i32))
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
