//! Winding estimate, SLOW rate (control-theory "The Winding
//! Estimate"). The board NTC is the base; the winding's excess over it is
//! two heat nodes on one input, each an EWMA of the SLOW cycle's mean of
//! the held current squared on its own leak
//! (`kernel::phases::loops::limits`), scaled by a host-written gain each:
//! `t = t_ntc + rise x heat + rise2 x heat2`. The rotor node
//! (`thermal_tau_s`, `th_rise_q016`) is the copper and iron over the can,
//! tens of seconds; the body node (`thermal_tau2_s`, `th_rise2_q016`) the
//! can and case into wherever the NTC sits, hundreds.
//! A gain is a thermal resistance times the winding's resistance in the
//! board's current counts; the host fits both from one hold at one duty
//! on the servo at hand. With the NTC on the can the body gain goes to 0
//! and the rotor node alone rides on the NTC. The estimate is a protection
//! number, never a thermometer.
//!
//! The heat input follows the warm copper: the square is raised by the
//! resistance's fractional rise over cold, alpha x the last published
//! excess ([`warm_q16`]), so a hold at one current heats as the copper
//! does. The feedback has a fixed point while the open-loop rise sits
//! under 1 / alpha = 254 C. Nothing folds the current on the way up: a
//! hold whose fixed point sits at the cutoff or over it, or that has none,
//! climbs until the estimate reads the cutoff, where the overtemperature
//! fault takes the drive off and the heats leak. The rise fed back stops
//! at [`RISE_FEEDBACK_MAX_CC`] and the estimate saturates at the top of
//! its range, which reads over every cutoff, so a climb that does not
//! settle under the cutoff ends in the latch, whatever the fields hold
//! (unit `the_warm_feedback_ends_under_the_cutoff_or_in_the_latch`).
//!
//! The same slope gives the resistance the control path reads
//! ([`WindingTherm::r_hat_q12`]): `r_q12` is the winding at 25.00 C, read
//! at the estimate's temperature, the NTC plus the excess. It is the
//! model's number, not a measurement.
//!
//! No NTC curve is UNSET: `t` reads [`UNSET_CC`], which sits under the
//! cutoff, and the flag says so. A curve with no gain reads the NTC
//! alone. The NTC itself is checked every time: a raw count within
//! [`NTC_RAIL_COUNTS`] of a rail (an open or shorted divider, curve or
//! not) or a converted temperature outside the band (a wrong curve) is
//! NTC_FAULT, and `t` reads [`NTC_FAULT_CC`], over every cutoff, so the
//! overtemperature fault latches on the same finish, at boot before
//! anything drives.

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

/// The board NTC's counts-to-centi-C curve, a host-written quadratic
/// about a reference point: `t = t_ref + k1 d + k2 d^2`,
/// `d = raw_ref - raw`; `k1 == 0` = no curve.
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

/// Copper's resistance rise per centi-C, 0.00393 per C, Q26: the slope
/// `r_q12` is defined on, `r(T) = r_q12 x (1 + ALPHA x (T - 25.00 C))`, so
/// a host converts a measurement to 25.00 C with this constant.
pub const ALPHA_Q26: u32 = 2637;
/// The temperature `r_q12` is stored at, centi-C.
pub const R_REF_CC: i32 = 2500;
/// The rise fed back into the heat input stops here, 150 C over the NTC:
/// the bound keeps the warm factor under 2^16 so its product with the
/// square fits u32 (`limits::heat`), whatever cutoff the host wrote.
const RISE_FEEDBACK_MAX_CC: u16 = 15_000;
const _: () = assert!((RISE_FEEDBACK_MAX_CC as u32 * ALPHA_Q26) >> 10 < 1 << 16);
// `r_hat_q12`'s factor over the band and the feedback bound: above -1 so
// the resistance stays positive, and its product with a u16 fits i32.
const _: () = {
    let lo = ((NTC_BAND_LO_CC as i32 - R_REF_CC) * ALPHA_Q26 as i32) >> 12;
    let hi =
        ((NTC_BAND_HI_CC as i32 - R_REF_CC + RISE_FEEDBACK_MAX_CC as i32) * ALPHA_Q26 as i32) >> 12;
    assert!(lo > -(1 << 14) && hi as i64 * u16::MAX as i64 <= i32::MAX as i64);
};

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
    let d = (ntc.raw_ref as i32 - raw as i32).max(-span).min(span);
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

    /// The winding resistance at the estimate's temperature, Q4.12
    /// vcounts per ccount: `r_q12`, the winding at 25.00 C, along copper's
    /// slope to the NTC plus the last published excess (bounded as the
    /// feedback is), saturating; `r_q12` itself while UNSET or NTC_FAULT,
    /// 0 for an unidentified winding. The correction truncates toward
    /// zero, so an identified winding never reads 0 on a cold board.
    pub fn r_hat_q12(&self, r_q12: u16) -> u16 {
        let d = if self.flags == 0 {
            let rise = if self.rise_cc > RISE_FEEDBACK_MAX_CC {
                core::hint::cold_path();
                RISE_FEEDBACK_MAX_CC
            } else {
                self.rise_cc
            };
            self.t_ntc_cc as i32 - R_REF_CC + rise as i32
        } else {
            core::hint::cold_path();
            0
        };
        // the fraction of `r_q12` the temperature adds, Q14
        let f = (d * ALPHA_Q26 as i32) >> 12;
        let r = r_q12 as i32;
        let p = r * f;
        (r + ((p + ((p >> 31) & 0x3fff)) >> 14)).min(u16::MAX as i32) as u16
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
    use crate::kernel::{DECIM_SLOW, LimitState};

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

    /// A quadratic about 25 C on the beta model of a 10K/10K/3950 divider.
    const NTC: NtcCfg = NtcCfg {
        raw_ref: 2048,
        t_ref_cc: 2500,
        k1_q88: 563,
        k2_q24: 5800,
    };
    /// The bench MG90 with the NTC on the board in room air: 0.01592
    /// centi-C per count squared.
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
    /// 8.31 C over a 25.00 C board.
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

    /// On a 25.00 C board the modeled resistance is the stored one raised
    /// by copper's slope on the last published excess: 2.0 vcounts per
    /// count reads 2.096 at 12.22 C over the NTC (4.80 %) and 2.222 at the
    /// 28.31 C a 400-count hold settles at on the bench constant
    /// (11.12 %). It is the stored one with no heat, while UNSET and on a
    /// failed NTC, 0 for an unidentified winding at any heat, and it
    /// saturates in its u16 at the 150 C the feedback stops at (58.94 %).
    #[test]
    fn the_modeled_resistance_is_the_stored_one_raised_by_the_warm_factor() {
        let mut th = WindingTherm::new();
        assert_eq!(th.r_hat_q12(8192), 8192, "UNSET");
        th.update(RAW_25, [0; 2], &CFG);
        assert_eq!(th.r_hat_q12(8192), 8192, "no heat");
        th.update(RAW_25, [277 * 277, 0], &CFG);
        assert_eq!((th.warm_q16(), th.r_hat_q12(8192)), (3146, 8585));
        assert_eq!(th.r_hat_q12(0), 0);
        th.update(RAW_25, [177_769, 0], &CFG);
        assert_eq!(th.t_cc(), 5331);
        assert_eq!((th.warm_q16(), th.r_hat_q12(8192)), (7290, 9103));
        assert_eq!(th.r_hat_q12(1), 1);
        th.update(0, [177_769, 0], &CFG);
        assert_eq!(th.r_hat_q12(8192), 8192, "NTC_FAULT");
        let no_curve = ThermCfg {
            ntc: NtcCfg::default(),
            ..CFG
        };
        th.update(RAW_25, [177_769, 0], &no_curve);
        assert_eq!(th.r_hat_q12(8192), 8192, "UNSET");
        th.update(RAW_25, [u32::MAX, 0], &CFG);
        assert_eq!(th.warm_q16(), 38627);
        assert_eq!(th.r_hat_q12(8192), 13020);
        assert_eq!(th.r_hat_q12(41233), 65533, "the last one under the top");
        assert_eq!(th.r_hat_q12(41235), u16::MAX, "65537 saturates");
        assert_eq!(th.r_hat_q12(u16::MAX), u16::MAX);
        assert_eq!(th.r_hat_q12(0), 0);
    }

    /// Every resistance the field holds at every temperature the estimate
    /// can carry into it, the board across the NTC band and the excess to
    /// past the feedback bound: within the Q14 factor's quantization of
    /// copper's law on 25.00 C, never 0 for an identified winding, never
    /// wrapped, saturating at the top, and never falling as it warms.
    #[test]
    fn r_hat_follows_copper_on_25_c_for_every_resistance_and_temperature() {
        let alpha = ALPHA_Q26 as f64 / (1u64 << 26) as f64;
        let boards = [
            NTC_BAND_LO_CC,
            -2000,
            -191,
            0,
            1500,
            2430,
            2500,
            2501,
            6700,
            NTC_BAND_HI_CC,
        ];
        let rises = [
            0u16,
            1,
            1222,
            2831,
            9999,
            RISE_FEEDBACK_MAX_CC,
            i16::MAX as u16,
        ];
        for r_q12 in 0..=u16::MAX {
            let mut read = [(0i32, 0u16); 70];
            let mut n = 0;
            for t_ntc_cc in boards {
                for rise_cc in rises {
                    let th = WindingTherm {
                        t_cc: 0,
                        t_ntc_cc,
                        rise_cc,
                        flags: 0,
                    };
                    let got = th.r_hat_q12(r_q12);
                    let d = t_ntc_cc as i32 - R_REF_CC + rise_cc.min(RISE_FEEDBACK_MAX_CC) as i32;
                    let law = r_q12 as f64 * (1.0 + alpha * d as f64);
                    let want = law.min(u16::MAX as f64);
                    let r = r_q12 as f64;
                    assert!(
                        (got as f64 - want).abs() <= r / 16384.0 + 1.0,
                        "r_q12 {r_q12} at {d} cc: {got}, the law {law:.2}"
                    );
                    assert_eq!(got == 0, r_q12 == 0, "r_q12 {r_q12} at {d} cc");
                    read[n] = (d, got);
                    n += 1;
                }
            }
            read.sort_unstable();
            assert!(
                read.windows(2).all(|w| w[0].1 <= w[1].1),
                "r_q12 {r_q12}: {read:?}"
            );
        }
        let flagged = |flags| WindingTherm {
            t_cc: 0,
            t_ntc_cc: NTC_BAND_LO_CC,
            rise_cc: RISE_FEEDBACK_MAX_CC,
            flags,
        };
        for flags in [flag::UNSET, flag::NTC_FAULT] {
            assert_eq!(flagged(flags).r_hat_q12(3725), 3725, "{flags}");
        }
    }

    /// A held current through the SLOW block's two stages as the kernel
    /// runs them, the heats on the cycle's mean square and the last
    /// estimate's warm factor and then the estimate, up to the finish that
    /// reads `cutoff_cc`: that finish, the estimate before it and the
    /// estimate on it. None when `finishes` pass under the cutoff, with the
    /// last estimate. Every finish reads at least the one before.
    fn hold_to_cutoff(
        i_held: i16,
        cfg: &ThermCfg,
        shifts: [u8; 2],
        cutoff_cc: i16,
        finishes: u32,
    ) -> (Option<u32>, i16, i16) {
        let mut heat = LimitState::new();
        let mut th = WindingTherm::new();
        let mut under = th.update(RAW_25, heat.heats(), cfg);
        assert!(under < cutoff_cc);
        for n in 1..=finishes {
            for _ in 0..DECIM_SLOW {
                heat.heat_sample(i_held.unsigned_abs());
            }
            let mean_sq = heat.take_heat_mean();
            heat.heat(mean_sq, th.warm_q16(), shifts);
            let t = th.update(RAW_25, heat.heats(), cfg);
            assert!(t >= under, "finish {n}: {t} after {under}");
            assert!(th.warm_q16() <= warm_q16(RISE_FEEDBACK_MAX_CC));
            if t >= cutoff_cc {
                return (Some(n), under, t);
            }
            under = t;
        }
        (None, under, under)
    }

    /// Nothing folds the current while the estimate climbs, so a held
    /// current ends one of two ways: at a fixed point under the cutoff, or
    /// on the finish that reads the cutoff, where the fault takes the drive
    /// off. The warm feedback opens no third: the rise it feeds back stops
    /// at 150 C over the NTC and the estimate saturates at the top of its
    /// range, which reads over every cutoff a host can write.
    #[test]
    fn the_warm_feedback_ends_under_the_cutoff_or_in_the_latch() {
        // the bench constant at 400 counts: a fixed point under the cutoff
        assert_eq!(
            hold_to_cutoff(400, &CFG, [8, 0], 9000, 5000),
            (None, 5331, 5331)
        );
        // at 1200 counts the fixed point sits over it: 74 finishes, 1.18 s
        assert_eq!(
            hold_to_cutoff(1200, &CFG, [8, 0], 9000, 5000),
            (Some(74), 8951, 9038)
        );
        // the cutoff at the top of its range: 1000 counts still settle,
        // 159 C of open-loop rise raised by the 150 C the feedback stops at
        assert_eq!(rise_cc(1000 * 1000, CFG.rise_q016), 15930);
        assert_eq!(
            hold_to_cutoff(1000, &CFG, [8, 0], i16::MAX, 5000),
            (None, 27813, 27813)
        );
        // 1300 counts have no fixed point, 269 C of open-loop rise: the
        // estimate saturates, and the top of the range is the latch
        assert_eq!(rise_cc(1300 * 1300, CFG.rise_q016), 26921);
        assert_eq!(
            hold_to_cutoff(1300, &CFG, [8, 0], i16::MAX, 5000),
            (Some(345), 32761, i16::MAX)
        );
        // the largest the fields hold, both gains and the heat input at
        // the top of their ranges: the first finish on the shortest leak,
        // the second or the eighth on the longest
        let max = ThermCfg {
            rise_q016: u16::MAX,
            rise2_q016: u16::MAX,
            ..CFG
        };
        for cutoff in [9000, i16::MAX] {
            assert_eq!(
                hold_to_cutoff(i16::MIN, &max, [0, 0], cutoff, 10),
                (Some(1), 2500, i16::MAX)
            );
        }
        assert_eq!(
            hold_to_cutoff(i16::MIN, &max, [15, 15], 9000, 100),
            (Some(2), 6592, 10688)
        );
        assert_eq!(
            hold_to_cutoff(i16::MIN, &max, [15, 15], i16::MAX, 100),
            (Some(8), 31166, i16::MAX)
        );
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
