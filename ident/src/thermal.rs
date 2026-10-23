//! The winding estimate's constants (core `estimator::thermal`): the
//! kernel reads `t = t_ntc + rise x heat`, the heat the I2t EWMA of the held
//! current squared on `thermal_tau_s`. The board NTC's counts-to-centi-C
//! curve is a board fact, encoded from its beta constants ([`NtcCurve`]).
//! Two host routes to the rise, `th_rise_q016`: a nameplate thermal
//! resistance ([`rise_from_rth`]), or a hold's own copper against the
//! replayed heat ([`fit_rise`]).

use osc_servo_core::kernel::phases::loops::limits::{LimitState, SLOW_PERIOD_MS, tau_shift};

use crate::exp::WindowSample;
use crate::gains::{Encoded, enc};

/// Where copper's resistance extrapolates to zero, degrees C: R(T) is
/// proportional to (234.5 + T) for annealed copper (IEC 60287-1-1).
pub const COPPER_ZERO_R_C: f64 = 234.5;

/// The NTC curve's reference temperature, C.
pub const NTC_REF_C: f64 = 25.0;

/// The board's NTC divider constants (`CalibSenseExt`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NtcBoard {
    pub pullup_ohm: f64,
    pub r25_ohm: f64,
    pub beta: f64,
}

impl NtcBoard {
    /// The beta model: degrees C at an ADC reading, for a thermistor to
    /// GND under a pull-up to VDD. None at the rails.
    pub fn beta_c(&self, raw: f64) -> Option<f64> {
        if !(raw > 0.0 && raw < 4096.0)
            || self.pullup_ohm <= 0.0
            || self.r25_ohm <= 0.0
            || self.beta <= 0.0
        {
            return None;
        }
        let r = self.pullup_ohm * raw / (4096.0 - raw);
        Some(1.0 / (1.0 / 298.15 + (r / self.r25_ohm).ln() / self.beta) - 273.15)
    }
}

/// The board NTC's curve as the kernel runs it (`estimator::thermal::
/// NtcCfg`): `t = t_ref + k1 d + k2 d^2`, `d = raw_ref - raw`, about the
/// 25 C reading.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NtcCurve {
    pub ntc_raw_ref: Encoded,
    pub ntc_t_ref_cc: Encoded,
    pub ntc_k1_q88: Encoded,
    pub ntc_k2_q24: Encoded,
}

impl NtcCurve {
    /// The central-difference slope at the 25 C reading and the curvature
    /// that lands the ~+18 C point. None on a degenerate divider.
    pub fn new(ntc: NtcBoard) -> Option<Self> {
        let t = |raw: f64| ntc.beta_c(raw);
        let raw_ref = (4096.0 * ntc.r25_ohm / (ntc.pullup_ohm + ntc.r25_ohm)).round();
        let (lo, hi, far) = (
            t(raw_ref + 100.0)?,
            t(raw_ref - 100.0)?,
            t(raw_ref - 800.0)?,
        );
        let k1 = (hi - lo) / 200.0 * 100.0;
        let k2 = ((far - NTC_REF_C) * 100.0 - k1 * 800.0) / (800.0 * 800.0);
        if !(k1.is_finite() && k1 > 0.0 && k2.is_finite()) {
            return None;
        }
        Some(Self {
            ntc_raw_ref: enc(raw_ref, 1.0),
            ntc_t_ref_cc: enc(NTC_REF_C * 100.0, 1.0),
            ntc_k1_q88: enc(k1, 256.0),
            ntc_k2_q24: enc(k2, 16_777_216.0),
        })
    }

    /// (name, field) pairs in table order.
    pub fn fields(&self) -> [(&'static str, Encoded); 4] {
        [
            ("ntc_raw_ref", self.ntc_raw_ref),
            ("ntc_t_ref_cc", self.ntc_t_ref_cc),
            ("ntc_k1_q88", self.ntc_k1_q88),
            ("ntc_k2_q24", self.ntc_k2_q24),
        ]
    }

    /// The temperature the kernel reads for an NTC count, degrees C, in its
    /// own arithmetic.
    pub fn reads_c(&self, raw: u16) -> f64 {
        let d = self.ntc_raw_ref.raw as i64 - raw as i64;
        let t = self.ntc_t_ref_cc.raw as i16 as i64
            + ((d * self.ntc_k1_q88.raw as i16 as i64) >> 8)
            + ((d * d * self.ntc_k2_q24.raw as i64) >> 24);
        t as f64 / 100.0
    }
}

/// The rise in centi-C per count squared of heat: a held current i
/// dissipates R i^2 (A/count)^2 watts, which stands R_th C/W over the NTC.
pub fn rise_per_count2(rth_c_per_w: f64, r_ohm: f64, a_per_count: f64) -> f64 {
    100.0 * rth_c_per_w * r_ohm * a_per_count * a_per_count
}

/// `th_rise_q016` from a winding-to-NTC thermal resistance, saturating at
/// the u16 rail.
pub fn rise_from_rth(rth_c_per_w: f64, r_ohm: f64, a_per_count: f64) -> Encoded {
    enc(rise_per_count2(rth_c_per_w, r_ohm, a_per_count), 65536.0)
}

/// The thermal resistance a rise implies, C/W.
pub fn rth_of(k_cc_per_count2: f64, r_ohm: f64, a_per_count: f64) -> f64 {
    k_cc_per_count2 / (100.0 * r_ohm * a_per_count * a_per_count)
}

/// One poll of the hold: host time, the tick block's signed held current,
/// the board NTC's reading and the kernel's own winding estimate.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HoldPoll {
    pub t_ms: f64,
    pub i_counts: f64,
    pub t_ntc_cc: f64,
    pub t_winding_cc: f64,
}

/// The windows from 1 to 3 s into the hold set the reference R: past the
/// limiter's slew in, before the copper has risen a degree.
pub const R_REF_MS: (f64, f64) = (1000.0, 3000.0);
/// The NTC base is its mean over this long before the hold.
pub const BASE_MS: f64 = 5000.0;
/// The fit's grid.
pub const BIN_MS: f64 = 1000.0;
/// The hold's end is the mean over this many closing bins.
pub const SETTLED_BINS: usize = 10;
/// Bins a hold must cover before its rise stands.
pub const MIN_BINS: usize = 30;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RiseFit {
    /// centi-C per count squared of heat.
    pub k: f64,
    pub rise_q016: Encoded,
    pub r_ref_ohm: f64,
    /// The C/W the rise implies on this winding and sense scale.
    pub rth_c_per_w: f64,
    /// The NTC base, C.
    pub t0_c: f64,
    /// Per-bin scatter of the copper's excess over the NTC around k x h, C.
    pub residual_sd_c: f64,
    /// The copper's excess over the NTC, the mean over the last
    /// [`SETTLED_BINS`] bins, C.
    pub settled_rise_c: f64,
    /// The kernel's winding estimate less the copper's temperature over the
    /// same closing bins, C; None while the estimate read UNSET.
    pub estimate_err_c: Option<f64>,
    pub bins: usize,
}

fn mean(xs: impl Iterator<Item = f64>) -> Option<f64> {
    let (s, n) = xs.fold((0.0, 0usize), |(s, n), x| (s + x, n + 1));
    (n > 0).then(|| s / n as f64)
}

/// A window's R, vcounts per ccount: duty and vdiff carry the drive's
/// sign, i is folded by it, so every term is positive into the stop.
fn window_r_vpc(w: &WindowSample) -> Option<f64> {
    (w.i != 0.0 && w.duty_q15 != 0.0)
        .then(|| (w.duty_q15 * w.vdiff / 32767.0) / (w.i * w.duty_q15.signum()))
}

/// The kernel's heat over the polls: its own integer EWMA of the held
/// current squared stepped on the SLOW grid, each step holding the last
/// poll at or before it. Sampled at `at_ms` (ascending).
pub fn replay_heat(polls: &[HoldPoll], thermal_tau_s: u16, at_ms: &[f64]) -> Vec<f64> {
    let shift = tau_shift(thermal_tau_s);
    let Some(first) = polls.first() else {
        return vec![0.0; at_ms.len()];
    };
    let mut st = LimitState::new();
    let mut out = Vec::with_capacity(at_ms.len());
    let (mut p, mut g) = (0usize, first.t_ms);
    for &at in at_ms {
        while g <= at {
            while p + 1 < polls.len() && polls[p + 1].t_ms <= g {
                p += 1;
            }
            let i = polls[p].i_counts.abs().round().min(i16::MAX as f64) as i16;
            st.heat(i, shift);
            g += SLOW_PERIOD_MS as f64;
        }
        out.push(st.heat_counts2() as f64);
    }
    out
}

struct Bin {
    centre_ms: f64,
    x_w: f64,
    x_n: f64,
    t_winding_c: Option<f64>,
}

/// The rise from a hold's own copper: per 1 s bin from the hold start
/// `t0_ms`, the copper's rise `x_w = (R / R_ref - 1)(234.5 + T_0)` less the
/// NTC's `x_n = T_ntc - T_0`, regressed through the origin on the replayed
/// heat. R_ref is the mean window R [`R_REF_MS`] into the hold, T_0 the
/// NTC's mean over the [`BASE_MS`] before it. `v_per_count` /
/// `a_per_count` the board's terminal and current scales.
pub fn fit_rise(
    polls: &[HoldPoll],
    windows: &[WindowSample],
    t0_ms: f64,
    thermal_tau_s: u16,
    v_per_count: f64,
    a_per_count: f64,
) -> Result<RiseFit, String> {
    if !(v_per_count > 0.0 && a_per_count > 0.0) {
        return Err("the sense scales are degenerate".into());
    }
    let r_ohm = |w: &WindowSample| window_r_vpc(w).map(|r| r * v_per_count / a_per_count);
    let t0_c = mean(
        polls
            .iter()
            .filter(|p| p.t_ms >= t0_ms - BASE_MS && p.t_ms < t0_ms)
            .map(|p| p.t_ntc_cc / 100.0),
    )
    .ok_or("no NTC reading in the 5 s before the hold")?;
    let r_ref = mean(
        windows
            .iter()
            .filter(|w| w.t_ms >= t0_ms + R_REF_MS.0 && w.t_ms < t0_ms + R_REF_MS.1)
            .filter_map(r_ohm),
    )
    .filter(|r| r.is_finite() && *r > 0.0)
    .ok_or("no window with current 1 to 3 s into the hold")?;
    let end = windows.iter().map(|w| w.t_ms).fold(t0_ms, f64::max);
    let n = ((end - t0_ms) / BIN_MS).ceil() as usize;
    let span = COPPER_ZERO_R_C + t0_c;
    let mut bins = Vec::new();
    for j in 0..n {
        let (lo, hi) = (t0_ms + j as f64 * BIN_MS, t0_ms + (j + 1) as f64 * BIN_MS);
        let inside: Vec<&HoldPoll> = polls
            .iter()
            .filter(|p| p.t_ms >= lo && p.t_ms < hi)
            .collect();
        let r = mean(
            windows
                .iter()
                .filter(|w| w.t_ms >= lo && w.t_ms < hi)
                .filter_map(r_ohm),
        );
        let t = mean(inside.iter().map(|p| p.t_ntc_cc / 100.0));
        let unset = inside.iter().any(|p| p.t_winding_cc <= i16::MIN as f64);
        if let (Some(r), Some(t)) = (r, t) {
            bins.push(Bin {
                centre_ms: (lo + hi) / 2.0,
                x_w: (r / r_ref - 1.0) * span,
                x_n: t - t0_c,
                t_winding_c: mean(inside.iter().map(|p| p.t_winding_cc / 100.0)).filter(|_| !unset),
            });
        }
    }
    if bins.len() < MIN_BINS {
        return Err(format!(
            "the hold covers {} one-second bins with a window and an NTC reading, under the \
             {MIN_BINS} a rise needs",
            bins.len()
        ));
    }
    let at: Vec<f64> = bins.iter().map(|b| b.centre_ms).collect();
    let h = replay_heat(polls, thermal_tau_s, &at);
    let shh: f64 = h.iter().map(|h| h * h).sum();
    if shh <= 0.0 {
        return Err("the hold carried no heat".into());
    }
    let k_c = h
        .iter()
        .zip(&bins)
        .map(|(h, b)| h * (b.x_w - b.x_n))
        .sum::<f64>()
        / shh;
    let res: Vec<f64> = h
        .iter()
        .zip(&bins)
        .map(|(h, b)| b.x_w - b.x_n - k_c * h)
        .collect();
    let residual_sd_c = (res.iter().map(|e| e * e).sum::<f64>() / (res.len() - 1) as f64).sqrt();
    let settled = &bins[bins.len().saturating_sub(SETTLED_BINS)..];
    let m = settled.len() as f64;
    let settled_rise_c = settled.iter().map(|b| b.x_w - b.x_n).sum::<f64>() / m;
    let copper_c = t0_c + settled.iter().map(|b| b.x_w).sum::<f64>() / m;
    let estimate_err_c = settled
        .iter()
        .map(|b| b.t_winding_c)
        .sum::<Option<f64>>()
        .map(|s| s / m - copper_c);
    let k = k_c * 100.0;
    Ok(RiseFit {
        k,
        rise_q016: enc(k, 65536.0),
        r_ref_ohm: r_ref,
        rth_c_per_w: rth_of(k, r_ref, a_per_count),
        t0_c,
        residual_sd_c,
        settled_rise_c,
        estimate_err_c,
        bins: bins.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{self, SenseParams};

    /// The dev-v006 rev 2A front end (units' own pin).
    const DEV: SenseParams = SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 14_884,
        vmotor_div_top: 6_400,
        vmotor_div_bot: 1_600,
        vdd_mv: 3300,
        tick_hz: 20_000,
    };

    #[test]
    fn tau_shift_rounds_to_the_nearest_octave() {
        assert_eq!(SLOW_PERIOD_MS, 16);
        assert_eq!(tau_shift(0), 0);
        assert_eq!(tau_shift(1), 6);
        assert_eq!(tau_shift(4), 8);
        assert_eq!(tau_shift(66), 12);
        assert_eq!(tau_shift(115), 13);
        assert_eq!(tau_shift(262), 14);
        assert_eq!(tau_shift(u16::MAX), 14);
    }

    /// The bench MG90 on the dev board, 4.51 ohm at 21.0 C/W over the NTC.
    #[test]
    fn a_nameplate_rth_encodes_the_rise() {
        let a = units::amps_per_count(&DEV);
        let e = rise_from_rth(21.0, 4.51, a);
        assert_eq!(e.raw, 505, "{a} A/count -> {e:?}");
        assert!(!e.saturated);
        assert!((rth_of(e.physical, 4.51, a) - 21.0).abs() < 1e-9);
        assert!(rise_from_rth(1e9, 4.51, a).saturated);
        assert_eq!(rise_from_rth(1e9, 4.51, a).raw, u16::MAX);
    }

    /// A hold whose copper rises over the NTC by exactly k x h at each bin
    /// centre, the NTC climbing on its own: the fit reads k back.
    #[test]
    fn recovers_the_rise_of_a_synthetic_hold() {
        const K_C: f64 = 0.0105;
        const TAU_S: u16 = 4;
        const T0: f64 = 0.0;
        const T_NTC0: f64 = 27.5;
        const I: f64 = -270.0;
        const DUTY: f64 = -5000.0;
        let v = units::volts_per_count(&DEV);
        let a = units::amps_per_count(&DEV);
        let r_ref_vpc = 1.0;
        let polls: Vec<HoldPoll> = (-500..30_000)
            .map(|n| {
                let t_ms = n as f64 * SLOW_PERIOD_MS as f64;
                HoldPoll {
                    t_ms,
                    i_counts: if t_ms >= T0 { I } else { 0.0 },
                    t_ntc_cc: 0.0,
                    t_winding_cc: 0.0,
                }
            })
            .collect();
        let bins = 470;
        let centres: Vec<f64> = (0..bins).map(|j| T0 + (j as f64 + 0.5) * BIN_MS).collect();
        let h = replay_heat(&polls, TAU_S, &centres);
        // the NTC climbs on its own; for the first three bins it falls
        // instead by the copper's excess, so R reads R_ref where R_ref is
        // read
        let x_n: Vec<f64> = (0..bins)
            .map(|j| if j < 3 { -K_C * h[j] } else { 0.004 * j as f64 })
            .collect();
        let polls: Vec<HoldPoll> = polls
            .into_iter()
            .map(|p| {
                let j = ((p.t_ms - T0) / BIN_MS).floor();
                let x = if j >= 0.0 {
                    x_n.get(j as usize).copied().unwrap_or(0.0)
                } else {
                    0.0
                };
                // the kernel's estimate reads 0.5 C over the copper
                let x_w = x + K_C * h.get(j.max(0.0) as usize).copied().unwrap_or(0.0);
                HoldPoll {
                    t_ntc_cc: (T_NTC0 + x) * 100.0,
                    t_winding_cc: (T_NTC0 + x_w + 0.5) * 100.0,
                    ..p
                }
            })
            .collect();
        let windows: Vec<WindowSample> = (0..bins)
            .map(|j| {
                let x_w = x_n[j] + K_C * h[j];
                let r = r_ref_vpc * (1.0 + x_w / (COPPER_ZERO_R_C + T_NTC0));
                WindowSample {
                    t_ms: centres[j],
                    i: I,
                    vdiff: r * I * DUTY.signum() * 32767.0 / DUTY,
                    duty_q15: DUTY,
                }
            })
            .collect();
        let f = fit_rise(&polls, &windows, T0, TAU_S, v, a).unwrap();
        assert!((f.k - K_C * 100.0).abs() < 1e-9, "{f:?}");
        assert_eq!(f.rise_q016.raw, (K_C * 100.0 * 65536.0).round() as u16);
        assert!(f.residual_sd_c < 1e-9, "{f:?}");
        assert!((f.t0_c - T_NTC0).abs() < 1e-12);
        assert!((f.r_ref_ohm - r_ref_vpc * v / a).abs() < 1e-9);
        assert!((f.rth_c_per_w - rth_of(f.k, f.r_ref_ohm, a)).abs() < 1e-12);
        assert_eq!(f.bins, bins);
        let steady = K_C * h[bins - 1];
        assert!((f.settled_rise_c - steady).abs() < 0.01 * steady, "{f:?}");
        assert!((f.estimate_err_c.unwrap() - 0.5).abs() < 1e-9, "{f:?}");
    }

    #[test]
    fn a_short_or_unbased_hold_is_refused() {
        let v = units::volts_per_count(&DEV);
        let a = units::amps_per_count(&DEV);
        let w = |t_ms| WindowSample {
            t_ms,
            i: -270.0,
            vdiff: -800.0,
            duty_q15: -5000.0,
        };
        let p = |t_ms| HoldPoll {
            t_ms,
            i_counts: -270.0,
            t_ntc_cc: 2750.0,
            t_winding_cc: 2750.0,
        };
        let short: Vec<WindowSample> = (0..10).map(|k| w(k as f64 * 500.0)).collect();
        let polls: Vec<HoldPoll> = (-10..10).map(|k| p(k as f64 * 500.0)).collect();
        assert!(
            fit_rise(&polls, &short, 0.0, 4, v, a)
                .unwrap_err()
                .contains("bins")
        );
        let late: Vec<HoldPoll> = (0..10).map(|k| p(k as f64 * 500.0)).collect();
        assert!(
            fit_rise(&late, &short, 0.0, 4, v, a)
                .unwrap_err()
                .contains("before the hold")
        );
    }

    const NTC: NtcBoard = NtcBoard {
        pullup_ohm: 10_000.0,
        r25_ohm: 10_000.0,
        beta: 3950.0,
    };

    #[test]
    fn encodes_the_10k_3950_ntc_curve() {
        let c = NtcCurve::new(NTC).expect("a curve");
        assert_eq!(c.ntc_raw_ref.raw, 2048);
        assert_eq!(c.ntc_t_ref_cc.raw, 2500);
        assert!(
            (c.ntc_k1_q88.raw as i32 - 563).abs() <= 4,
            "{}",
            c.ntc_k1_q88.raw
        );
        assert!(
            (c.ntc_k2_q24.raw as i32 - 5800).abs() <= 300,
            "{}",
            c.ntc_k2_q24.raw
        );
        assert!(c.fields().iter().all(|(_, f)| !f.saturated));
        assert!(NtcCurve::new(NtcBoard { beta: 0.0, ..NTC }).is_none());
        assert!(
            NtcCurve::new(NtcBoard {
                r25_ohm: 0.0,
                ..NTC
            })
            .is_none()
        );
    }

    /// Through the kernel's arithmetic the quadratic reads the beta model
    /// exactly at 25 C and within 0.7 C from 15 to 50 C.
    #[test]
    fn the_kernel_reads_the_beta_curve_through_the_quadratic() {
        let c = NtcCurve::new(NTC).unwrap();
        let at_ref = c.ntc_raw_ref.raw;
        assert!((NTC.beta_c(at_ref as f64).unwrap() - 25.0).abs() < 1e-9);
        assert_eq!(c.reads_c(at_ref), 25.0);
        for t in [15.0f64, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0] {
            let r = 10_000.0 * (3950.0 * (1.0 / (t + 273.15) - 1.0 / 298.15)).exp();
            let raw = (4096.0 * r / (10_000.0 + r)).round();
            let truth = NTC.beta_c(raw).unwrap();
            let read = c.reads_c(raw as u16);
            assert!(
                (read - truth).abs() < 0.7,
                "{t} C: reads {read} for {truth}"
            );
        }
    }
}
