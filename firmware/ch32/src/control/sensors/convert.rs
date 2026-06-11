//! Stateless raw-ADC → engineering-unit math. `Scales` precomputes Q32
//! reciprocals at compile time (called from `cfg::Precomputed`) so the
//! 20 kHz ISR runs one `mulhu` per conversion and the linker drops
//! `__udivdi3` / `__divdi3` entirely.

use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::cfg::{Calibration, Divider, NtcCal};

const ADC_MAX_RAW: u32 = 4095;

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
    fn scales_new_returns_nonzero_q32() {
        let scales = Scales::new(&rev_b_calibration(), 32);
        assert!(scales.vbus_q32 > 0);
        assert!(scales.vmotor_q32 > 0);
        assert!(scales.shunt_q32 > 0);
    }
}
