//! Winding estimate, SLOW rate (control-theory "The I2t Winding
//! Estimate"). The board NTC is the base; the winding's excess over it is
//! the I2t budget's heat, an EWMA of the held current squared on
//! `thermal_tau_s` (`kernel::phases::loops::limits`), scaled by one
//! host-written constant: `t = t_ntc + rise x heat`. The rise is the
//! thermal resistance from the winding to wherever the NTC sits, times
//! the winding's resistance, in the board's current counts; the host
//! derives it from a hold on the servo at hand or a nameplate K/W. With
//! the NTC on the can the heat carries the fast node alone; on a board in
//! room air it carries the whole motor with one leak, and the estimate is
//! a protection number, never a thermometer.
//!
//! A zero rise or no NTC curve is UNSET: `t` reads [`UNSET_CC`], which
//! sits under every derate threshold, and the flag says so.

use crate::math::q_mul;

/// What the estimate reads from CALIB (kernel config take).
#[derive(Copy, Clone, Default)]
pub struct ThermCfg {
    /// Winding rise over the NTC per count squared of heat, centi-C,
    /// Q0.16; 0 = no estimate.
    pub rise_q016: u16,
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
    /// No rise constant or no NTC curve: `t_winding_cc` reads the sentinel.
    pub const UNSET: u8 = 1 << 0;
}

/// `t_winding_cc` while UNSET.
pub const UNSET_CC: i16 = i16::MIN;

const ADC_FULL: u32 = 4096;

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
            flags: flag::UNSET,
        }
    }

    /// SLOW-rate: the NTC conversion and the estimate from the I2t heat
    /// (`LimitState::heat_counts2`); the temperature, [`UNSET_CC`] without
    /// a model.
    pub fn update(&mut self, ntc_raw: u16, heat: u32, cfg: &ThermCfg) -> i16 {
        let ntc = ntc_cc(ntc_raw, &cfg.ntc);
        if let Some(t) = ntc {
            self.t_ntc_cc = t;
        }
        self.t_cc = match ntc {
            Some(t) if cfg.rise_q016 != 0 => {
                self.flags = 0;
                (t as i32 + rise_cc(heat, cfg.rise_q016)).min(i16::MAX as i32) as i16
            }
            _ => {
                self.flags = flag::UNSET;
                UNSET_CC
            }
        };
        self.t_cc
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
        assert_eq!(th.update(RAW_25, 0, &CFG), 2500);
        assert_eq!(th.update(RAW_25, 277 * 277, &CFG), 2500 + 1222);
        assert_eq!((th.t_ntc_cc(), th.flags()), (2500, 0));
        assert_eq!(rise_cc(76729, 1044), 1222);
        assert_eq!(rise_cc(1 << 16, 1), 1);
    }

    /// The rise saturates at the sentinel's far end instead of wrapping,
    /// and so does the sum.
    #[test]
    fn the_estimate_saturates() {
        let mut th = WindingTherm::new();
        assert_eq!(rise_cc(u32::MAX, u16::MAX), i16::MAX as i32);
        assert_eq!(th.update(RAW_25, u32::MAX, &CFG), i16::MAX);
    }

    /// No rise constant, or no NTC curve, or an NTC at a rail: the sentinel,
    /// the flag, and the last good NTC reading stays published.
    #[test]
    fn unset_reads_the_sentinel_and_never_a_temperature() {
        let mut th = WindingTherm::new();
        assert_eq!(th.t_cc(), UNSET_CC);
        assert_eq!(th.flags(), flag::UNSET);
        let no_rise = ThermCfg {
            rise_q016: 0,
            ..CFG
        };
        assert_eq!(th.update(RAW_25, 76729, &no_rise), UNSET_CC);
        assert_eq!((th.t_ntc_cc(), th.flags()), (2500, flag::UNSET));
        let no_curve = ThermCfg {
            ntc: NtcCfg::default(),
            ..CFG
        };
        assert_eq!(th.update(RAW_25, 76729, &no_curve), UNSET_CC);
        assert_eq!(th.update(RAW_25, 76729, &CFG), 3722);
        assert_eq!(th.update(0, 76729, &CFG), UNSET_CC, "an NTC at a rail");
        assert_eq!(th.t_ntc_cc(), 2500, "the last conversion stands");
        assert_eq!(th.flags(), flag::UNSET);
    }
}
