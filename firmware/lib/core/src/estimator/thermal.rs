//! Winding estimate, SLOW rate (control-theory "The I2t Winding
//! Estimate"). The board NTC is the base; the winding's excess over it is
//! two heat nodes on one input, each an EWMA of the held current squared
//! on its own leak (`kernel::phases::loops::limits`), scaled by a
//! host-written gain each: `t = t_ntc + rise x heat + rise2 x heat2`. The
//! rotor node (`thermal_tau_s`, `th_rise_q016`) is the copper and iron
//! over the can, tens of seconds; the body node (`thermal_tau2_s`,
//! `th_rise2_q016`) the can and case into wherever the NTC sits, hundreds.
//! A gain is a thermal resistance times the winding's resistance in the
//! board's current counts; the host fits both from a hold and a step-down
//! on the servo at hand. With the NTC on the can the body gain goes to 0
//! and the rotor node alone rides on the NTC. The estimate is a protection
//! number, never a thermometer.
//!
//! The heat input follows the warm copper: the square is raised by the
//! resistance's fractional rise over cold, alpha x the last published
//! excess ([`warm_q16`]), so a hold at one current heats as the copper
//! does. The feedback has a fixed point while the open-loop rise sits
//! under 1 / alpha = 254 C, which the derate takes the current down long
//! before.
//!
//! No NTC curve is UNSET: `t` reads [`UNSET_CC`], which sits under every
//! derate threshold, and the flag says so. A curve with no gain reads the
//! NTC alone. The NTC itself is checked every time: a raw count within
//! [`NTC_RAIL_COUNTS`] of a rail (an open or shorted divider, curve or
//! not) or a converted temperature outside the band (a wrong curve) is
//! NTC_FAULT, and `t` reads [`NTC_FAULT_CC`], over every cutoff, so the
//! derate goes to zero and the overtemperature fault latches on the same
//! finish, at boot before anything drives.

use crate::math::q_mul;

/// What the estimate reads from CALIB (kernel config take).
#[derive(Copy, Clone, Default)]
pub struct ThermCfg {
    /// Rotor rise over the NTC per count squared of its heat, centi-C,
    /// Q0.16.
    pub rise_q016: u16,
    /// Body rise over the NTC per count squared of its heat, centi-C,
    /// Q0.16; 0 = no body node.
    pub rise2_q016: u16,
    pub ntc: NtcCfg,
}

/// The board NTC's counts-to-centi-C curve, host-derived from the beta
/// model (`CalibSenseExt` pull-up / R25 / beta) as a quadratic about the
/// 25 C point: `t = t_ref + k1 d + k2 d^2`, `d = raw_ref - raw`. Within
/// 0.7 C of the beta curve from 15 to 50 C on 10K/10K/3950 (the tangent
/// alone reads 2.3 C low at 45 C); `k1 == 0` = no curve.
#[derive(Copy, Clone, Default)]
pub struct NtcCfg {
    pub raw_ref: u16,
    pub t_ref_cc: i16,
    /// centi-C per count at the reference, Q8.8.
    pub k1_q88: i16,
    /// centi-C per count squared, Q0.24.
    pub k2_q24: u16,
}

/// `therm_flags` bits.
pub mod flag {
    /// No NTC curve: `t_winding_cc` reads the sentinel.
    pub const UNSET: u8 = 1 << 0;
    /// The NTC reads a rail or its curve a temperature outside the band:
    /// `t_winding_cc` reads [`super::NTC_FAULT_CC`].
    pub const NTC_FAULT: u8 = 1 << 1;
}

/// `t_winding_cc` while UNSET.
pub const UNSET_CC: i16 = i16::MIN;
/// `t_winding_cc` while NTC_FAULT: over every cutoff.
pub const NTC_FAULT_CC: i16 = i16::MAX;

const ADC_FULL: u32 = 4096;
/// A raw count this close to a rail is an open or shorted divider, never
/// a temperature: the sense chain's noise is about 2 counts, and on a
/// 10K/10K/3950 divider 8 counts from a rail is already past 180 C and
/// under -90 C.
const NTC_RAIL_COUNTS: u16 = 8;
/// The temperatures a converted NTC may read, centi-C: outside them the
/// curve is wrong.
const NTC_BAND_LO_CC: i16 = -4000;
const NTC_BAND_HI_CC: i16 = 15_000;

/// Copper's resistance rise per centi-C, 0.00393 per C, Q26.
const ALPHA_Q26: u32 = 2637;
/// The rise fed back into the heat input stops here, 150 C over the NTC:
/// the fault tripped long before, and the bound keeps the warm factor
/// under 2^16 so its product with the square fits u32 (`limits::heat`).
const RISE_FEEDBACK_MAX_CC: u16 = 15_000;
const _: () = assert!((RISE_FEEDBACK_MAX_CC as u32 * ALPHA_Q26) >> 10 < 1 << 16);

/// The heat input's warm factor for a rise over the NTC: the copper's
/// fractional resistance rise over cold, Q0.16, at most 38627.
pub fn warm_q16(rise_cc: u16) -> u32 {
    (rise_cc.min(RISE_FEEDBACK_MAX_CC) as u32 * ALPHA_Q26) >> 10
}

/// Board NTC counts to centi-C through the host's quadratic; None at the
/// rails or without a curve.
pub fn ntc_cc(raw: u16, ntc: &NtcCfg) -> Option<i16> {
    if ntc.k1_q88 == 0 || raw == 0 || raw as u32 >= ADC_FULL - 1 {
        return None;
    }
    // a host-written reference outside the ADC range cannot overflow the quadratic
    let span = (ADC_FULL - 1) as i32;
    let d = (ntc.raw_ref as i32 - raw as i32).clamp(-span, span);
    let t =
        ntc.t_ref_cc as i32 + q_mul(d, ntc.k1_q88 as i32, 8) + q_mul(d * d, ntc.k2_q24 as i32, 24);
    (i16::MIN as i32 + 1..=i16::MAX as i32)
        .contains(&t)
        .then_some(t as i16)
}

/// The rise over the NTC for `heat` counts squared, centi-C, saturating.
/// One 32 x 32 -> 64 multiply: the heat runs to 2^27, the rise to 2^16.
pub fn rise_cc(heat: u32, rise_q016: u16) -> i32 {
    ((heat as u64 * rise_q016 as u64) >> 16).min(i16::MAX as u64) as i32
}

pub struct WindingTherm {
    t_cc: i16,
    t_ntc_cc: i16,
    /// The last published excess over the NTC, the heat input's feedback.
    rise_cc: u16,
    flags: u8,
}

impl Default for WindingTherm {
    fn default() -> Self {
        Self::new()
    }
}

impl WindingTherm {
    pub const fn new() -> Self {
        Self {
            t_cc: UNSET_CC,
            t_ntc_cc: 0,
            rise_cc: 0,
            flags: flag::UNSET,
        }
    }

    /// SLOW-rate: the NTC check and conversion and the estimate from the
    /// two heats (`LimitState::heats`); the temperature, [`UNSET_CC`]
    /// without a curve, [`NTC_FAULT_CC`] on a railed NTC or a reading
    /// outside the band.
    pub fn update(&mut self, ntc_raw: u16, heat: [u32; 2], cfg: &ThermCfg) -> i16 {
        let ntc = ntc_cc(ntc_raw, &cfg.ntc);
        let railed = ntc_raw <= NTC_RAIL_COUNTS || ntc_raw >= ADC_FULL as u16 - 1 - NTC_RAIL_COUNTS;
        if let Some(t) = ntc.filter(|_| !railed) {
            self.t_ntc_cc = t;
        }
        (self.t_cc, self.rise_cc, self.flags) = match ntc {
            _ if railed => (NTC_FAULT_CC, 0, flag::NTC_FAULT),
            None => (UNSET_CC, 0, flag::UNSET),
            Some(t) if !(NTC_BAND_LO_CC..=NTC_BAND_HI_CC).contains(&t) => {
                (NTC_FAULT_CC, 0, flag::NTC_FAULT)
            }
            Some(t) => {
                let rise = (rise_cc(heat[0], cfg.rise_q016) + rise_cc(heat[1], cfg.rise2_q016))
                    .min(i16::MAX as i32);
                (
                    (t as i32 + rise).min(i16::MAX as i32) as i16,
                    rise as u16,
                    0,
                )
            }
        };
        self.t_cc
    }

    /// The heat input's warm factor on the last published excess
    /// ([`warm_q16`]); 0 while UNSET.
    pub fn warm_q16(&self) -> u32 {
        warm_q16(self.rise_cc)
    }

    pub fn t_cc(&self) -> i16 {
        self.t_cc
    }

    /// The board NTC as last converted, centi-C (0 before the first).
    pub fn t_ntc_cc(&self) -> i16 {
        self.t_ntc_cc
    }

    pub fn flags(&self) -> u8 {
        self.flags
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The beta model of the dev-v006 board's NTC (10K pull-up, 10K at
    /// 25 C, beta 3950), as the host evaluates it.
    fn beta_c(raw: f64) -> f64 {
        let r_over_r25 = raw / (4096.0 - raw);
        1.0 / (1.0 / 298.15 + r_over_r25.ln() / 3950.0) - 273.15
    }

    /// The host's quadratic about 25 C: the central-difference slope at
    /// the reference and the curvature that lands the +18 C point.
    fn ntc_from_beta() -> NtcCfg {
        let raw_ref = 2048.0;
        let k1 = (beta_c(raw_ref - 100.0) - beta_c(raw_ref + 100.0)) / 200.0 * 100.0;
        let d = 800.0;
        let k2 = ((beta_c(raw_ref - d) - 25.0) * 100.0 - k1 * d) / (d * d);
        NtcCfg {
            raw_ref: raw_ref as u16,
            t_ref_cc: 2500,
            k1_q88: (k1 * 256.0).round() as i16,
            k2_q24: (k2 * 16_777_216.0).round() as u16,
        }
    }

    /// The dev-v006 board's NTC.
    const NTC: NtcCfg = NtcCfg {
        raw_ref: 2048,
        t_ref_cc: 2500,
        k1_q88: 563,
        k2_q24: 5800,
    };
    /// The bench MG90 with the NTC on the board in room air: 0.01592
    /// centi-C per count squared (notebook 21).
    pub(crate) const CFG: ThermCfg = ThermCfg {
        rise_q016: 1044,
        rise2_q016: 0,
        ntc: NTC,
    };
    /// Counts at 25.00 C for a 10K/10K divider.
    const RAW_25: u16 = 2048;

    #[test]
    fn ntc_quadratic_tracks_the_beta_curve_within_a_degree_from_15_to_50_c() {
        let ntc = ntc_from_beta();
        assert!(
            (ntc.k1_q88 as i32 - NTC.k1_q88 as i32).abs() <= 8,
            "{}",
            ntc.k1_q88
        );
        assert!(
            (ntc.k2_q24 as i32 - NTC.k2_q24 as i32).abs() <= 400,
            "{}",
            ntc.k2_q24
        );
        for t_c in [15.0f64, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0] {
            let r = 10_000.0 * (3950.0 * (1.0 / (t_c + 273.15) - 1.0 / 298.15)).exp();
            let raw = (4096.0 * r / (10_000.0 + r)).round();
            let t_fw = ntc_cc(raw as u16, &ntc).expect("in range") as f64 / 100.0;
            assert!((t_fw - beta_c(raw)).abs() < 0.7, "{t_c} C: {t_fw}");
        }
        assert_eq!(ntc_cc(0, &NTC), None);
        assert_eq!(ntc_cc(4095, &NTC), None);
        assert_eq!(ntc_cc(2048, &NtcCfg::default()), None);
    }

    /// A host-written reference past the ADC span reads the curve at the
    /// span's end, never an overflowed or wrapped temperature.
    #[test]
    fn ntc_reference_outside_the_adc_span_clamps_the_deviation() {
        let at = |raw_ref: u16| NtcCfg { raw_ref, ..NTC };
        assert_eq!(ntc_cc(1, &at(u16::MAX)), ntc_cc(1, &at(4096)));
        assert!(ntc_cc(1, &at(4096)).is_some());
        assert_eq!(ntc_cc(0, &at(u16::MAX)), None, "a rail is no reading");
        let low = ntc_cc(ADC_FULL as u16 - 2, &at(0)).expect("in range");
        assert!(low < NTC.t_ref_cc, "{low}");
    }

    /// The estimate is the NTC plus the rise on the heat, in the kernel's
    /// integers: the 277-count rating squared at the bench constant reads
    /// 8.31 C over a 25.00 C board (notebook 21 section 6).
    #[test]
    fn the_estimate_is_the_ntc_plus_the_rise_on_the_heat() {
        let mut th = WindingTherm::new();
        assert_eq!(th.update(RAW_25, [0; 2], &CFG), 2500);
        assert_eq!(th.update(RAW_25, [277 * 277, 0], &CFG), 2500 + 1222);
        assert_eq!((th.t_ntc_cc(), th.flags()), (2500, 0));
        assert_eq!(rise_cc(76729, 1044), 1222);
        assert_eq!(rise_cc(1 << 16, 1), 1);
    }

    /// Both nodes add their gain on their own heat, and the body node
    /// alone is a model: the step-down's candidate pair on board D (706
    /// rotor, 825 body) at the 554-count cap settled reads 33.1 + 38.6 C
    /// over the board.
    #[test]
    fn the_body_node_adds_its_own_gain_on_its_own_heat() {
        let two = ThermCfg {
            rise_q016: 706,
            rise2_q016: 825,
            ..CFG
        };
        let mut th = WindingTherm::new();
        let sq = 554 * 554;
        assert_eq!(th.update(RAW_25, [sq, sq], &two), 2500 + 3306 + 3863);
        assert_eq!(th.update(RAW_25, [sq, 0], &two), 2500 + 3306);
        assert_eq!(th.update(RAW_25, [0, sq], &two), 2500 + 3863);
        let body_only = ThermCfg {
            rise_q016: 0,
            ..two
        };
        assert_eq!(th.update(RAW_25, [sq, sq], &body_only), 2500 + 3863);
        assert_eq!(th.flags(), 0);
    }

    /// The warm factor is the copper's fractional resistance rise on the
    /// last published excess: 0.393 % per C, 0 while UNSET, bounded at
    /// 150 C over so its product with any square fits u32.
    #[test]
    fn the_warm_factor_follows_the_published_excess_and_is_bounded() {
        assert_eq!(warm_q16(0), 0);
        assert_eq!(warm_q16(1000), 2575, "10 C: 3.93 % is 2575 in Q0.16");
        assert_eq!(warm_q16(2548), 6561);
        assert_eq!(warm_q16(RISE_FEEDBACK_MAX_CC), 38627);
        assert_eq!(warm_q16(i16::MAX as u16), 38627);
        let mut th = WindingTherm::new();
        assert_eq!(th.warm_q16(), 0);
        th.update(RAW_25, [277 * 277, 0], &CFG);
        assert_eq!(th.warm_q16(), warm_q16(1222));
        th.update(0, [277 * 277, 0], &CFG);
        assert_eq!(th.warm_q16(), 0, "UNSET feeds nothing back");
    }

    /// The rise saturates at the sentinel's far end instead of wrapping,
    /// and so do the two nodes' sum and the sum with the NTC.
    #[test]
    fn the_estimate_saturates() {
        let mut th = WindingTherm::new();
        assert_eq!(rise_cc(u32::MAX, u16::MAX), i16::MAX as i32);
        assert_eq!(th.update(RAW_25, [u32::MAX, 0], &CFG), i16::MAX);
        let two = ThermCfg {
            rise2_q016: u16::MAX,
            ..CFG
        };
        assert_eq!(th.update(RAW_25, [u32::MAX, u32::MAX], &two), i16::MAX);
        assert_eq!(th.warm_q16(), 38627);
    }

    /// No NTC curve: the sentinel, the flag; a curve with no gain reads
    /// the NTC alone; the last good NTC reading stays published.
    #[test]
    fn unset_reads_the_sentinel_and_a_curve_without_a_gain_reads_the_ntc() {
        let mut th = WindingTherm::new();
        assert_eq!(th.t_cc(), UNSET_CC);
        assert_eq!(th.flags(), flag::UNSET);
        let no_curve = ThermCfg {
            ntc: NtcCfg::default(),
            ..CFG
        };
        assert_eq!(th.update(RAW_25, [76729; 2], &no_curve), UNSET_CC);
        assert_eq!((th.t_ntc_cc(), th.flags()), (0, flag::UNSET));
        let no_rise = ThermCfg {
            rise_q016: 0,
            ..CFG
        };
        assert_eq!(th.update(RAW_25, [76729; 2], &no_rise), 2500);
        assert_eq!((th.t_ntc_cc(), th.flags(), th.warm_q16()), (2500, 0, 0));
        assert_eq!(th.update(RAW_25, [76729, 0], &CFG), 3722);
        assert_eq!(th.update(RAW_25, [76729; 2], &no_curve), UNSET_CC);
        assert_eq!(th.t_ntc_cc(), 2500, "the last conversion stands");
        assert_eq!(th.warm_q16(), 0, "UNSET feeds nothing back");
    }

    /// An NTC within 8 counts of either rail is an open or shorted
    /// divider, curve or not: the fault value over every cutoff, the flag,
    /// nothing fed back, the last good reading kept; 9 counts in the curve
    /// reads.
    #[test]
    fn a_railed_ntc_reads_the_fault_value_with_or_without_a_curve() {
        let mut th = WindingTherm::new();
        th.update(RAW_25, [76729, 0], &CFG);
        for raw in [0, 1, 8, 4087, 4094, 4095] {
            assert_eq!(th.update(raw, [76729, 0], &CFG), NTC_FAULT_CC, "{raw}");
            assert_eq!((th.flags(), th.warm_q16()), (flag::NTC_FAULT, 0), "{raw}");
        }
        assert_eq!(th.t_ntc_cc(), 2500, "the last conversion stands");
        let no_curve = ThermCfg {
            ntc: NtcCfg::default(),
            ..CFG
        };
        assert_eq!(th.update(0, [0; 2], &no_curve), NTC_FAULT_CC);
        assert_eq!(th.flags(), flag::NTC_FAULT);
        assert_ne!(th.update(9, [0; 2], &CFG), NTC_FAULT_CC);
        assert_ne!(th.update(4086, [0; 2], &CFG), NTC_FAULT_CC);
        assert_eq!(th.flags(), 0);
    }

    /// A curve that reads outside -40..150 C is a wrong curve: the fault
    /// value, not a temperature.
    #[test]
    fn a_reading_outside_the_band_is_a_wrong_curve() {
        let steep = ThermCfg {
            ntc: NtcCfg {
                k1_q88: NTC.k1_q88 * 8,
                ..NTC
            },
            ..CFG
        };
        let mut th = WindingTherm::new();
        assert_eq!(th.update(RAW_25, [0; 2], &steep), 2500);
        assert_eq!(th.update(1000, [0; 2], &steep), NTC_FAULT_CC, "hot side");
        assert_eq!(th.flags(), flag::NTC_FAULT);
        assert_eq!(th.update(3500, [0; 2], &steep), NTC_FAULT_CC, "cold side");
        assert_ne!(
            th.update(3500, [0; 2], &CFG),
            NTC_FAULT_CC,
            "the right curve reads it"
        );
        assert_eq!(th.flags(), 0);
    }
}
