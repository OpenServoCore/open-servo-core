//! The winding thermometer's anchor: what `osc ident` writes into CALIB
//! `CalibWinding` so the kernel's `t_winding_cc` reads a temperature.
//!
//! The kernel tracks R as the fixed point of its sign-LMS, mean(duty x
//! vdiff) / mean(i) over SLOW samples with current flowing into a shaft
//! at rest, and reads `t = t0 + k (R - r0)` (core `estimator::thermal`).
//! A thermometer built on a ratio has one instrument: `r0_q12` is that
//! same estimator's reading, the ident aggregate's duty x vdiff / i at a
//! seated hold under the current limit ([`crate::exp::anchor`]), never
//! the burst waveform's R, which reads the winding by another route on
//! another scale (3 to 10% apart on the bench MG90, 8 to 25 C of
//! thermometer). `t0_cc` is the rest temperature the host supplies, the
//! one number the servo cannot measure. `k_r2t_q88` is copper's handbook
//! line through that anchor, so the kernel reads T = T0 + (R / R0 - 1)
//! (234.5 + T0) with its one multiply, and `mu_q016` is the LMS step for a
//! settling time at the current limit.

use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW};

use crate::gains::{Encoded, enc};

/// Where copper's resistance extrapolates to zero, degrees C: R(T) is
/// proportional to (234.5 + T) for annealed copper (IEC 60287-1-1; the
/// handbook 0.00393 per C is 1 / (234.5 + 20)). Handbook, never a fitted
/// slope: the DC pair on the bench MG90, 0.396 +/- 0.010 %/C (notebook 17),
/// says the loop is copper, not what the slope is.
pub const COPPER_ZERO_R_C: f64 = 234.5;

/// The LMS settling time at the current limit, seconds: ~125 SLOW samples
/// averaged against the kernel's per-sample noise, and fast against the
/// winding's warm-up over the can, tens of seconds (notebook 17). At the
/// thermometer's current floor the step is proportionally slower.
pub const SETTLE_S: f64 = 2.0;

/// The thermometer's current floor, eighths of the anchor's hold current.
/// The kernel's v / i is current-dependent (4.12 / 4.43 / 4.55 ohm at
/// 141 / 225 / 273 counts, seated holds on the rev 2A bench MG90), so an
/// anchor reads true only near its own current; the floor sits just under
/// the hold.
pub const FLOOR_EIGHTHS_OF_HOLD: u32 = 7;

/// `rtherm_i_min_counts` from the anchor hold's median current, counts:
/// the thermometer samples only where the anchor holds.
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

/// The floor's report line, shared by `osc ident anchor` and the run's
/// report.
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

/// The four CALIB fields, encoded. `fields` is the write set in table
/// order; none of them is stamp-covered.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Anchor {
    pub r0_q12: Encoded,
    pub t0_cc: Encoded,
    pub k_r2t_q88: Encoded,
    pub mu_q016: Encoded,
}

impl Anchor {
    /// `r_vpc` the kernel's R at rest, vcounts per ccount; `ambient_c` the
    /// winding's temperature then; `i_lim_counts` the current limit the
    /// step settles at; `tick_hz` the fast tick rate. None on a degenerate
    /// input: nothing is written from a zero.
    pub fn new(r_vpc: f64, ambient_c: f64, i_lim_counts: u16, tick_hz: f64) -> Option<Self> {
        if !r_vpc.is_finite()
            || r_vpc <= 0.0
            || !ambient_c.is_finite()
            || i_lim_counts == 0
            || !tick_hz.is_finite()
            || tick_hz <= 0.0
        {
            return None;
        }
        let r0_q12 = enc(r_vpc, 4096.0);
        if r0_q12.raw == 0 {
            return None;
        }
        // centi-C per Q4.12 LSB: copper's line through the anchor
        let k = 100.0 * (COPPER_ZERO_R_C + ambient_c) / r0_q12.raw as f64;
        // the LMS closes on its fixed point by mu x i / 4096 of the
        // remaining error per SLOW sample
        let mu = 4096.0 / (i_lim_counts as f64 * SETTLE_S * slow_hz(tick_hz));
        Some(Self {
            r0_q12,
            t0_cc: enc_cc(ambient_c),
            k_r2t_q88: enc(k, 256.0),
            mu_q016: enc(mu, 65536.0),
        })
    }

    /// (name, field) pairs in table order.
    pub fn fields(&self) -> [(&'static str, Encoded); 4] {
        [
            ("r0_q12", self.r0_q12),
            ("t0_cc", self.t0_cc),
            ("k_r2t_q88", self.k_r2t_q88),
            ("mu_q016", self.mu_q016),
        ]
    }

    /// The temperature the kernel reads at `r_vpc` through this anchor,
    /// degrees C, in the kernel's own arithmetic (`t0 + ((r - r0) x k) >> 8`).
    pub fn reads_c(&self, r_vpc: f64) -> f64 {
        let r = (r_vpc * 4096.0).round() as i64;
        let dt = ((r - self.r0_q12.raw as i64) * self.k_r2t_q88.raw as i64) >> 8;
        (self.t0_cc.raw as i16 as i64 + dt) as f64 / 100.0
    }
}

/// Degrees C to centi-C in the i16 the table stores, carried as its u16
/// bit pattern (the write path takes raw u16s); clamped at the i16 rails.
fn enc_cc(c: f64) -> Encoded {
    let ideal = c * 100.0;
    let cc = ideal.round().clamp(i16::MIN as f64, i16::MAX as f64);
    let saturated = cc != ideal.round();
    let quantization_pct = if c != 0.0 {
        (cc / 100.0 - c).abs() / c.abs() * 100.0
    } else {
        0.0
    };
    Encoded {
        physical: c,
        raw: cc as i16 as u16,
        quantization_pct,
        saturated,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bench MG90 on the rev 2A board: 4.725 ohm at rest is ~1.05
    /// vcounts per ccount, 26.23 C, limit 280, 20 kHz.
    const R_VPC: f64 = 4310.0 / 4096.0;
    const AMBIENT_C: f64 = 26.23;

    fn bench() -> Anchor {
        Anchor::new(R_VPC, AMBIENT_C, 280, 20_000.0).expect("an anchor")
    }

    #[test]
    fn encodes_the_handbook_line_through_the_anchor() {
        let a = bench();
        assert_eq!(a.r0_q12.raw, 4310);
        assert_eq!(a.t0_cc.raw, 2623);
        // 100 x 260.73 / 4310 = 6.0494 cc per LSB -> 1548.6 in Q8.8
        assert_eq!(a.k_r2t_q88.raw, 1549);
        // 4096 / (280 x 2 s x 62.5 Hz) = 0.1170 -> 7670 in Q0.16
        assert_eq!(slow_hz(20_000.0), 62.5);
        assert_eq!(a.mu_q016.raw, 7670);
        assert!(a.fields().iter().all(|(_, f)| !f.saturated));
        assert_eq!(
            a.fields().map(|(n, _)| n),
            ["r0_q12", "t0_cc", "k_r2t_q88", "mu_q016"]
        );
    }

    /// Through the kernel's arithmetic the anchor reads its own R as the
    /// ambient, and R up copper's 0.393 %/C as the rise that produced it:
    /// 40 C of winding is 15.3% of R on this anchor.
    #[test]
    fn the_kernel_reads_copper_through_it() {
        let a = bench();
        assert!((a.reads_c(R_VPC) - AMBIENT_C).abs() < 0.01);
        for rise in [10.0, 40.0, 80.0] {
            let r = R_VPC * (1.0 + rise / (COPPER_ZERO_R_C + AMBIENT_C));
            let read = a.reads_c(r);
            assert!(
                (read - (AMBIENT_C + rise)).abs() < 0.05,
                "rise {rise}: reads {read}"
            );
        }
        // 1% of R is ~2.6 C, the thermometer's resolution argument
        assert!((a.reads_c(R_VPC * 1.01) - AMBIENT_C - 2.61).abs() < 0.05);
    }

    #[test]
    fn a_cold_room_is_a_negative_cc() {
        let a = Anchor::new(R_VPC, -5.5, 280, 20_000.0).unwrap();
        assert_eq!(a.t0_cc.raw as i16, -550);
        assert!(!a.t0_cc.saturated);
        assert!((a.reads_c(R_VPC) + 5.5).abs() < 0.01);
    }

    #[test]
    fn degenerate_inputs_write_nothing() {
        assert!(Anchor::new(0.0, 25.0, 280, 20_000.0).is_none());
        assert!(Anchor::new(-1.0, 25.0, 280, 20_000.0).is_none());
        assert!(Anchor::new(R_VPC, f64::NAN, 280, 20_000.0).is_none());
        assert!(Anchor::new(R_VPC, 25.0, 0, 20_000.0).is_none());
        assert!(Anchor::new(R_VPC, 25.0, 280, 0.0).is_none());
        assert!(Anchor::new(1e-5, 25.0, 280, 20_000.0).is_none());
    }

    /// A winding under 0.025 vcounts per ccount puts the slope over Q8.8's
    /// 256 cc per LSB; a limit under 33 counts asks for a step over 1.
    /// Both clamp flagged, never wrap.
    #[test]
    fn saturation_is_flagged() {
        let a = Anchor::new(0.02, 25.0, 280, 20_000.0).unwrap();
        assert!(a.k_r2t_q88.saturated);
        assert_eq!(a.k_r2t_q88.raw, u16::MAX);
        let a = Anchor::new(R_VPC, 25.0, 20, 20_000.0).unwrap();
        assert!(a.mu_q016.saturated);
        assert_eq!(a.mu_q016.raw, u16::MAX);
        let a = Anchor::new(R_VPC, 400.0, 280, 20_000.0).unwrap();
        assert!(a.t0_cc.saturated);
        assert_eq!(a.t0_cc.raw as i16, i16::MAX);
    }
}
