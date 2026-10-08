//! The winding thermometer's CALIB set: what `osc ident` writes so the
//! kernel's `t_winding_cc` reads a temperature (core `estimator::thermal`).
//!
//! The absolute base is the board NTC; the same-seat resistance RATIO reads
//! the rise within a hold; a first-order model carries the winding's excess
//! over the NTC across moves and rests. Nothing here is a resistance
//! anchor for the absolute: the brush contact moves R by percents per seat
//! and shifts the whole cloud for hours after running (1% is 2.6 C).
//!
//! The model constants are a MOTOR FAMILY's, not this unit's: a seated
//! hold's own rise carries 1-3 contact steps of 0.7-1.6% (2-4 C each),
//! which spread a one-node fit of R_th over a factor of two (bringup
//! kb/ident-replay-note.md). Notebook 18's two-node model of the MG90 on
//! the dev board in open air gives the family numbers; a later ident with a
//! can thermocouple may replace them per installation.

use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW};

use crate::gains::{Encoded, enc};

/// Where copper's resistance extrapolates to zero, degrees C: R(T) is
/// proportional to (234.5 + T) for annealed copper (IEC 60287-1-1; the
/// handbook 0.00393 per C is 1 / (234.5 + 20)). Handbook, never a fitted
/// slope: the DC pair on the bench MG90, 0.396 +/- 0.010 %/C (notebook 17),
/// says the loop is copper, not what the slope is.
pub const COPPER_ZERO_R_C: f64 = 234.5;
/// The temperature the stored cold R (`r0_q12`) is reduced to.
pub const R_COLD_REF_C: f64 = 25.0;

/// The MG90 family on the dev board in open air (notebook 18 replayed with
/// the slow mode fixed): the winding's excess over the board NTC settles at
/// 63 C/W and carries on the two-node slow mode, 128 s.
pub const MG90_TAU_S: f64 = 128.0;
pub const MG90_R_TH_C_PER_W: f64 = 63.0;

/// The LMS settling time at the hold current, seconds: ~125 SLOW samples
/// averaged against the kernel's per-sample noise, and fast against the
/// winding's warm-up over the can, tens of seconds (notebook 17).
pub const SETTLE_S: f64 = 2.0;

/// The thermometer's current floor, eighths of the hold current the cold R
/// was read at: the kernel's v / i is current-dependent (4.12 / 4.43 / 4.55
/// ohm at 141 / 225 / 273 counts, seated holds on the rev 2A bench MG90),
/// and the ratio re-bases on an eighth of current change anyway.
pub const FLOOR_EIGHTHS_OF_HOLD: u32 = 7;

/// `rtherm_i_min_counts` from the hold's median current, counts.
pub fn floor_counts(i_hold_counts: f64) -> Encoded {
    let hold = i_hold_counts.round().clamp(0.0, u16::MAX as f64);
    let raw = (hold as u32 * FLOOR_EIGHTHS_OF_HOLD / 8) as u16;
    let physical = i_hold_counts * FLOOR_EIGHTHS_OF_HOLD as f64 / 8.0;
    Encoded {
        physical,
        raw,
        quantization_pct: if physical != 0.0 {
            (raw as f64 - physical).abs() / physical.abs() * 100.0
        } else {
            0.0
        },
        saturated: hold != i_hold_counts.round(),
    }
}

/// The floor's report line.
pub fn floor_line(i_hold_counts: f64) -> String {
    format!(
        "rtherm_i_min_counts {:>3}  ({FLOOR_EIGHTHS_OF_HOLD}/8 of the hold's {i_hold_counts:.0} counts)",
        floor_counts(i_hold_counts).raw
    )
}

/// The kernel's SLOW rate, Hz, where the thermometer samples.
pub fn slow_hz(tick_hz: f64) -> f64 {
    tick_hz / (DECIM_MED as f64 * DECIM_SLOW as f64)
}

/// The board NTC's beta model: degrees C at an ADC reading, for a
/// thermistor to GND under a pull-up to VDD. None at the rails.
pub fn ntc_beta_c(raw: f64, pullup_ohm: f64, r25_ohm: f64, beta: f64) -> Option<f64> {
    if !(raw > 0.0 && raw < 4096.0) || pullup_ohm <= 0.0 || r25_ohm <= 0.0 || beta <= 0.0 {
        return None;
    }
    let r = pullup_ohm * raw / (4096.0 - raw);
    Some(1.0 / (1.0 / 298.15 + (r / r25_ohm).ln() / beta) - 273.15)
}

/// The CALIB thermometer set, encoded. `fields` is the write set in table
/// order; `th_alpha_q24`, `th_g_q016` and the NTC reference and slope are
/// stamp-covered.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Thermal {
    pub th_alpha_q24: Encoded,
    pub th_g_q016: Encoded,
    pub th_mu_q016: Encoded,
    pub ntc_raw_ref: Encoded,
    pub ntc_t_ref_cc: Encoded,
    pub ntc_k1_q88: Encoded,
    pub ntc_k2_q24: Encoded,
}

/// The board's NTC divider constants (`CalibSenseExt`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NtcBoard {
    pub pullup_ohm: f64,
    pub r25_ohm: f64,
    pub beta: f64,
}

impl Thermal {
    /// `tau_s`, `r_th_c_per_w` the family model; `v_per_vcount`,
    /// `a_per_ccount` the board's sense scale (the kernel's power is
    /// `i (v - Ke omega)` in vcount-ccount); `i_hold_counts` the current the
    /// LMS settles at; `tick_hz` the fast tick. None on a degenerate input:
    /// nothing is written from a zero.
    pub fn new(
        tau_s: f64,
        r_th_c_per_w: f64,
        v_per_vcount: f64,
        a_per_ccount: f64,
        i_hold_counts: u16,
        ntc: NtcBoard,
        tick_hz: f64,
    ) -> Option<Self> {
        let finite_pos = |x: f64| x.is_finite() && x > 0.0;
        if ![tau_s, r_th_c_per_w, v_per_vcount, a_per_ccount, tick_hz]
            .iter()
            .all(|&x| finite_pos(x))
            || i_hold_counts == 0
        {
            return None;
        }
        let alpha = 1.0 / (tau_s * slow_hz(tick_hz));
        let g = r_th_c_per_w * v_per_vcount * a_per_ccount * 100.0;
        let mu = 4096.0 / (i_hold_counts as f64 * SETTLE_S * slow_hz(tick_hz));
        // the quadratic about the 25 C reading: the central-difference slope
        // there and the curvature that lands the ~+18 C point (within 0.7 C
        // of beta from 15 to 50 C on 10K/10K/3950)
        let t = |raw: f64| ntc_beta_c(raw, ntc.pullup_ohm, ntc.r25_ohm, ntc.beta);
        let raw_ref = (4096.0 * ntc.r25_ohm / (ntc.pullup_ohm + ntc.r25_ohm)).round();
        let (lo, hi, far) = (
            t(raw_ref + 100.0)?,
            t(raw_ref - 100.0)?,
            t(raw_ref - 800.0)?,
        );
        let k1 = (hi - lo) / 200.0 * 100.0;
        let k2 = ((far - R_COLD_REF_C) * 100.0 - k1 * 800.0) / (800.0 * 800.0);
        if !(k1.is_finite() && k1 > 0.0 && k2.is_finite()) {
            return None;
        }
        Some(Self {
            th_alpha_q24: enc(alpha, 16_777_216.0),
            th_g_q016: enc(g, 65536.0),
            th_mu_q016: enc(mu, 65536.0),
            ntc_raw_ref: enc(raw_ref, 1.0),
            ntc_t_ref_cc: enc(R_COLD_REF_C * 100.0, 1.0),
            ntc_k1_q88: enc(k1, 256.0),
            ntc_k2_q24: enc(k2, 16_777_216.0),
        })
    }

    /// (name, field) pairs in table order.
    pub fn fields(&self) -> [(&'static str, Encoded); 7] {
        [
            ("th_alpha_q24", self.th_alpha_q24),
            ("th_g_q016", self.th_g_q016),
            ("th_mu_q016", self.th_mu_q016),
            ("ntc_raw_ref", self.ntc_raw_ref),
            ("ntc_t_ref_cc", self.ntc_t_ref_cc),
            ("ntc_k1_q88", self.ntc_k1_q88),
            ("ntc_k2_q24", self.ntc_k2_q24),
        ]
    }

    /// The temperature the kernel reads for an NTC count through this set,
    /// degrees C, in the kernel's own arithmetic.
    pub fn ntc_reads_c(&self, raw: u16) -> f64 {
        let d = self.ntc_raw_ref.raw as i64 - raw as i64;
        let t = self.ntc_t_ref_cc.raw as i16 as i64
            + ((d * self.ntc_k1_q88.raw as i16 as i64) >> 8)
            + ((d * d * self.ntc_k2_q24.raw as i64) >> 24);
        t as f64 / 100.0
    }
}

/// The stored cold R: the kernel's own R at a seated hold after a long
/// idle, reduced to [`R_COLD_REF_C`] through copper from the NTC's
/// temperature then. None when the reading is not cold (the servo was not
/// idle long enough) or degenerate.
pub fn cold_r_q12(r_vpc: f64, t_ntc_c: f64, idle_ok: bool) -> Option<Encoded> {
    if !idle_ok || !(r_vpc.is_finite() && r_vpc > 0.0) || !t_ntc_c.is_finite() {
        return None;
    }
    let r25 = r_vpc * (COPPER_ZERO_R_C + R_COLD_REF_C) / (COPPER_ZERO_R_C + t_ntc_c);
    let e = enc(r25, 4096.0);
    (e.raw != 0).then_some(e)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NTC: NtcBoard = NtcBoard {
        pullup_ohm: 10_000.0,
        r25_ohm: 10_000.0,
        beta: 3950.0,
    };

    /// The dev-v006 rev 2A board: 4.0283 mV per terminal count, 0.90216 mA
    /// per current count, 20 kHz; the MG90 family, a 280-count hold.
    fn bench() -> Thermal {
        Thermal::new(
            MG90_TAU_S,
            MG90_R_TH_C_PER_W,
            4.0283e-3,
            0.90216e-3,
            280,
            NTC,
            20_000.0,
        )
        .expect("a set")
    }

    #[test]
    fn encodes_the_family_model_and_the_ntc_curve() {
        let t = bench();
        // 1 / (128 s x 62.5 Hz) = 1.25e-4 -> 2097 in Q0.24
        assert_eq!(t.th_alpha_q24.raw, 2097);
        // 63 C/W x 3.634e-6 W per vcount-ccount x 100 cc/C = 0.0229 -> 1500
        assert!(
            (t.th_g_q016.raw as i32 - 1500).abs() <= 2,
            "{}",
            t.th_g_q016.raw
        );
        assert_eq!(t.th_mu_q016.raw, 7670);
        assert_eq!(t.ntc_raw_ref.raw, 2048);
        assert_eq!(t.ntc_t_ref_cc.raw, 2500);
        assert!(
            (t.ntc_k1_q88.raw as i32 - 563).abs() <= 4,
            "{}",
            t.ntc_k1_q88.raw
        );
        assert!(
            (t.ntc_k2_q24.raw as i32 - 5800).abs() <= 300,
            "{}",
            t.ntc_k2_q24.raw
        );
        assert!(t.fields().iter().all(|(_, f)| !f.saturated));
    }

    /// Through the kernel's arithmetic the quadratic tracks the beta curve
    /// within 0.7 C from 15 to 50 C.
    #[test]
    fn the_kernel_reads_the_beta_curve_through_the_quadratic() {
        let t = bench();
        for c in [15.0f64, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0] {
            let r = 10_000.0 * (3950.0 * (1.0 / (c + 273.15) - 1.0 / 298.15)).exp();
            let raw = (4096.0 * r / (10_000.0 + r)).round();
            let truth = ntc_beta_c(raw, NTC.pullup_ohm, NTC.r25_ohm, NTC.beta).unwrap();
            let read = t.ntc_reads_c(raw as u16);
            assert!(
                (read - truth).abs() < 0.7,
                "{c} C: reads {read} for {truth}"
            );
        }
    }

    #[test]
    fn cold_r_reduces_to_25_c_and_refuses_a_warm_reading() {
        let e = cold_r_q12(1.0078, 28.4, true).unwrap();
        assert_eq!(e.raw, (1.0078f64 * 259.5 / 262.9 * 4096.0).round() as u16);
        assert!(cold_r_q12(1.0078, 28.4, false).is_none());
        assert!(cold_r_q12(0.0, 25.0, true).is_none());
    }

    #[test]
    fn degenerate_inputs_write_nothing() {
        let f = |tau, rth, i, beta| {
            Thermal::new(
                tau,
                rth,
                4.0283e-3,
                0.90216e-3,
                i,
                NtcBoard { beta, ..NTC },
                20_000.0,
            )
        };
        assert!(f(0.0, 63.0, 280, 3950.0).is_none());
        assert!(f(128.0, -1.0, 280, 3950.0).is_none());
        assert!(f(128.0, 63.0, 0, 3950.0).is_none());
        assert!(f(128.0, 63.0, 280, 0.0).is_none());
    }
}
