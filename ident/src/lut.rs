//! Ripple-referenced pot linearization LUT: builder + host-side apply.
//!
//! The pot is nonlinear, so a 2-point linear map (raw_min at one rail,
//! raw_max at the other) mis-reports angle mid-travel. This module builds a
//! 55-knot correction table from a calibration sweep and applies it on the
//! host - the firmware only STORES the block, it never converts.
//!
//! Angle clock: over a constant-duty full-travel sweep the motor commutation
//! ripple on the winding current is a motor-shaft angle clock. Its cumulative
//! phase is a drift-resistant TRUE-angle axis (no constant-velocity
//! assumption, unlike sampling against wall time); raw pot counts sampled
//! against that axis reveal the pot's nonlinearity. One monotonic pot
//! channel, so the result is a plain raw->fraction curve (no 2D search).
//!
//! LUT semantics (host-defined; firmware does not apply the block):
//! - 55 knots evenly spaced in raw counts, raw_i = raw_min + i*span/54.
//! - corrected(raw_i) = raw_i + corr[i]; linear interp between knots; raw
//!   outside [raw_min,raw_max] clamps to the rail.
//! - linearize(raw) = (corrected(raw) - raw_min)/span clamped to [0,1].
//! - all-zero corr == identity == the 2-point-linear baseline: corr shifts
//!   each knot's corrected value off the straight line, so zeros leave
//!   linearize equal to the plain linear fraction.

use crate::ripple;

/// Knot count = the firmware PotLutBlock's lut_corr length.
const N_KNOTS: usize = 55;
/// Intervals between knots.
const N_INTERVALS: usize = N_KNOTS - 1;
/// Below this many aligned samples the sweep can't define a curve.
const MIN_SAMPLES: usize = 4;
/// cumulative_phase needs a majority of windows to find ripple, else the
/// sweep SNR is too low to trust as an angle clock.
const MIN_GOOD_FRAC: f64 = 0.5;

/// Host mirror of the firmware PotLutBlock (raw_min/raw_max/lut_corr).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PotLut {
    pub raw_min: u16,
    pub raw_max: u16,
    pub corr: [i16; N_KNOTS],
}

impl PotLut {
    /// All-zero corr: linearize reduces to the plain linear fraction.
    pub fn identity(raw_min: u16, raw_max: u16) -> PotLut {
        PotLut {
            raw_min,
            raw_max,
            corr: [0; N_KNOTS],
        }
    }

    /// Corrected raw value at knot i: on-line position plus its correction.
    fn corrected_knot(&self, i: usize) -> f64 {
        let span = self.raw_max as f64 - self.raw_min as f64;
        self.raw_min as f64 + i as f64 * span / N_INTERVALS as f64 + self.corr[i] as f64
    }

    /// Normalized linearized fraction in [0,1]. Degenerate span -> 0.
    pub fn linearize(&self, raw: u16) -> f64 {
        let lo = self.raw_min as f64;
        let hi = self.raw_max as f64;
        let span = hi - lo;
        if span <= 0.0 {
            return 0.0;
        }
        let r = (raw as f64).clamp(lo, hi);
        let pos = ((r - lo) / span * N_INTERVALS as f64).clamp(0.0, N_INTERVALS as f64);
        let i0 = pos.floor() as usize;
        let corrected = if i0 >= N_INTERVALS {
            self.corrected_knot(N_INTERVALS)
        } else {
            let frac = pos - i0 as f64;
            let c0 = self.corrected_knot(i0);
            let c1 = self.corrected_knot(i0 + 1);
            c0 + frac * (c1 - c0)
        };
        ((corrected - lo) / span).clamp(0.0, 1.0)
    }

    /// Angle in centi-degrees, composing the linearized fraction with the
    /// kinematics endpoints (angle_min_cdeg at raw_min, angle_max at raw_max).
    pub fn angle_cdeg(&self, raw: u16, angle_min_cdeg: i16, angle_max_cdeg: i16) -> f64 {
        let f = self.linearize(raw);
        angle_min_cdeg as f64 + f * (angle_max_cdeg as f64 - angle_min_cdeg as f64)
    }
}

/// Cumulative motor phase (revs) per sample from a constant-duty sweep's
/// current series, paired with the fraction of windows that found ripple (a
/// coverage/confidence metric in `MIN_GOOD_FRAC..=1.0`). Slides ripple_speed
/// over the series, integrates the per-window rev/s (drift-resistant: an
/// occasional dropped window is interpolated across, not extrapolated from one
/// global rate), and returns a monotonic phase. None when input is too short
/// or a majority of windows fail the ripple confidence floor.
pub fn cumulative_phase(current: &[f64], fs: f64, ripple_per_rev: f64) -> Option<(Vec<f64>, f64)> {
    let n = current.len();
    if fs <= 0.0 || ripple_per_rev <= 0.0 {
        return None;
    }
    let win = ripple::min_window(fs);
    if win == 0 || n < win {
        return None;
    }
    let hop = (win / 4).max(1);
    // (window center sample, motor rev/s) for windows that clear the floor
    let mut centers: Vec<(f64, f64)> = Vec::new();
    let mut total = 0usize;
    let mut start = 0usize;
    while start + win <= n {
        total += 1;
        if let Some(e) = ripple::ripple_speed(&current[start..start + win], fs, ripple_per_rev) {
            centers.push((start as f64 + win as f64 / 2.0, e.motor_rev_s));
        }
        start += hop;
    }
    if centers.len() < 2 || (centers.len() as f64) < MIN_GOOD_FRAC * total as f64 {
        return None;
    }
    let good_frac = centers.len() as f64 / total as f64;
    // per-sample rev/s: linear interp across window centers, clamped at the
    // ends; cumulative-trapezoid to revs. rev/s >= 0 keeps phase monotone.
    let mut phase = Vec::with_capacity(n);
    let mut acc = 0.0;
    let mut prev = rate_at(&centers, 0.0);
    phase.push(0.0);
    for k in 1..n {
        let rs = rate_at(&centers, k as f64);
        acc += 0.5 * (prev + rs) / fs;
        phase.push(acc);
        prev = rs;
    }
    Some((phase, good_frac))
}

/// Build the LUT from time-aligned raw pot samples and the cumulative motor
/// phase (monotonic, from cumulative_phase). Normalizes phase to a true
/// fraction, forms a monotone raw->fraction curve, and sets each knot's corr
/// to the count offset that lands its corrected value on the curve. Degenerate
/// input (too few samples, length mismatch, zero raw or phase span) -> identity.
pub fn build(raw_pot: &[u16], motor_phase: &[f64], raw_min: u16, raw_max: u16) -> PotLut {
    let n = raw_pot.len();
    let span = raw_max as i32 - raw_min as i32;
    if n < MIN_SAMPLES || n != motor_phase.len() || span <= 0 {
        return PotLut::identity(raw_min, raw_max);
    }
    let p0 = motor_phase[0];
    let pspan = motor_phase[n - 1] - p0;
    if !pspan.is_finite() || pspan == 0.0 {
        return PotLut::identity(raw_min, raw_max);
    }
    // (raw, true_frac) pairs. Orient frac to increase with raw: raw is
    // monotonic in true angle and so is phase, making frac monotone in raw
    // regardless of sweep direction.
    let mut pts: Vec<(f64, f64)> = (0..n)
        .map(|k| (raw_pot[k] as f64, (motor_phase[k] - p0) / pspan))
        .collect();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    if pts[pts.len() - 1].1 < pts[0].1 {
        for p in pts.iter_mut() {
            p.1 = 1.0 - p.1;
        }
    }
    // average duplicate raws (pot flat spots, rounding collisions)
    let mut curve: Vec<(f64, f64)> = Vec::with_capacity(pts.len());
    let mut i = 0;
    while i < pts.len() {
        let r = pts[i].0;
        let mut s = 0.0;
        let mut c = 0.0;
        while i < pts.len() && pts[i].0 == r {
            s += pts[i].1;
            c += 1.0;
            i += 1;
        }
        curve.push((r, s / c));
    }
    if curve.len() < 2 {
        return PotLut::identity(raw_min, raw_max);
    }
    // enforce non-decreasing frac (noise guard)
    for j in 1..curve.len() {
        if curve[j].1 < curve[j - 1].1 {
            curve[j].1 = curve[j - 1].1;
        }
    }
    let span_f = span as f64;
    let mut corr = [0i16; N_KNOTS];
    for (k, c) in corr.iter_mut().enumerate() {
        let raw_i = raw_min as f64 + k as f64 * span_f / N_INTERVALS as f64;
        // knots the sweep never reached (SNR gap near a rail) fall on the
        // clamped end of the curve, so their corr stays small rather than
        // extrapolating a wild slope past the covered range.
        let frac = interp(&curve, raw_i);
        let corrected = raw_min as f64 + frac * span_f;
        *c = sat_i16((corrected - raw_i).round());
    }
    PotLut {
        raw_min,
        raw_max,
        corr,
    }
}

/// Rate (rev/s) at sample index x by linear interp over window centers,
/// clamped to the first/last center outside the covered span.
fn rate_at(centers: &[(f64, f64)], x: f64) -> f64 {
    interp(centers, x)
}

/// Linear interpolation over an x-sorted table, clamping outside the range.
fn interp(pts: &[(f64, f64)], x: f64) -> f64 {
    let last = pts.len() - 1;
    if x <= pts[0].0 {
        return pts[0].1;
    }
    if x >= pts[last].0 {
        return pts[last].1;
    }
    let (mut lo, mut hi) = (0usize, last);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if pts[mid].0 <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (x0, y0) = pts[lo];
    let (x1, y1) = pts[hi];
    if x1 == x0 {
        return y0;
    }
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

fn sat_i16(v: f64) -> i16 {
    if !v.is_finite() {
        return 0;
    }
    v.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    const RAW_MIN: u16 = 200;
    const RAW_MAX: u16 = 3900;
    const SPAN: f64 = (RAW_MAX - RAW_MIN) as f64;

    // Monotonic nonlinear pot: raw = raw_min + span*(f + a*sin(pi*f)).
    // a < 1/pi keeps it monotone; endpoints fixed (sin(0)=sin(pi)=0).
    fn pot_raw(f: f64, a: f64) -> u16 {
        (RAW_MIN as f64 + SPAN * (f + a * (PI * f).sin())).round() as u16
    }

    fn sweep(a: f64, m: usize) -> (Vec<u16>, Vec<f64>) {
        let mut raw = Vec::with_capacity(m);
        let mut phase = Vec::with_capacity(m);
        for k in 0..m {
            let f = k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, a));
            // rigid gear: motor phase proportional to true angle
            phase.push(3.0 * f);
        }
        (raw, phase)
    }

    #[test]
    fn identity_linearize_is_linear_fraction() {
        let lut = PotLut::identity(RAW_MIN, RAW_MAX);
        for raw in [RAW_MIN, 1000, 2048, 3000, RAW_MAX] {
            let expect = (raw as f64 - RAW_MIN as f64) / SPAN;
            assert!((lut.linearize(raw) - expect).abs() < 1e-12, "raw {raw}");
        }
        assert_eq!(lut.angle_cdeg(RAW_MIN, 0, 19000), 0.0);
        assert!((lut.angle_cdeg(RAW_MAX, 0, 19000) - 19000.0).abs() < 1e-9);
    }

    #[test]
    fn out_of_range_raw_clamps() {
        let lut = PotLut::identity(RAW_MIN, RAW_MAX);
        assert_eq!(lut.linearize(0), 0.0);
        assert_eq!(lut.linearize(u16::MAX), 1.0);
    }

    #[test]
    fn build_linear_pot_yields_near_zero_corr() {
        let (raw, phase) = sweep(0.0, 300);
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        // only rounding of the u16 raw samples separates corr from zero
        assert!(
            lut.corr.iter().all(|&c| c.abs() <= 2),
            "corr {:?}",
            lut.corr
        );
    }

    #[test]
    fn build_recovers_nonlinear_pot_and_beats_identity() {
        let a = 0.15;
        let (raw, phase) = sweep(a, 400);
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        let ident = PotLut::identity(RAW_MIN, RAW_MAX);
        let mut lut_max = 0.0f64;
        let mut ident_max = 0.0f64;
        for (k, &r) in raw.iter().enumerate() {
            let f = k as f64 / 399.0;
            lut_max = lut_max.max((lut.linearize(r) - f).abs());
            ident_max = ident_max.max((ident.linearize(r) - f).abs());
        }
        // fraction error is a fraction of span; < 1% of span required
        assert!(lut_max < 0.01, "lut err {lut_max}");
        // identity is off by ~a*sin(pi*f) mid-travel; LUT must trounce it
        assert!(ident_max > 0.1, "ident err {ident_max}");
        assert!(
            lut_max < ident_max / 10.0,
            "lut {lut_max} ident {ident_max}"
        );
    }

    #[test]
    fn build_degenerate_is_identity() {
        assert_eq!(
            build(&[], &[], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
        assert_eq!(
            build(&[1, 2], &[0.0, 1.0], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
        // zero raw span
        let (raw, phase) = sweep(0.1, 100);
        assert_eq!(build(&raw, &phase, 500, 500), PotLut::identity(500, 500));
        // length mismatch
        assert_eq!(
            build(&raw, &phase[..50], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
    }

    #[test]
    fn linearize_zero_span_is_zero() {
        let lut = PotLut::identity(500, 500);
        assert_eq!(lut.linearize(500), 0.0);
    }

    // --- cumulative_phase ---

    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, half: f64) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) * half
        }
    }

    const FS: f64 = 20_100.0;

    fn ripple_series(freq: f64, amp: f64, noise: f64, n: usize) -> Vec<f64> {
        let mut lcg = Lcg(41);
        (0..n)
            .map(|k| {
                let t = k as f64 / FS;
                60.0 + amp * (2.0 * PI * freq * t).sin() + lcg.next(noise)
            })
            .collect()
    }

    #[test]
    fn cumulative_phase_recovers_total_revs() {
        let n = 4000;
        let i = ripple_series(1800.0, 4.0, 3.0, n);
        let (phase, good) = cumulative_phase(&i, FS, 6.0).expect("phase found");
        assert_eq!(phase.len(), n);
        assert!((MIN_GOOD_FRAC..=1.0).contains(&good), "coverage {good}");
        // monotone non-decreasing
        assert!(phase.windows(2).all(|w| w[1] >= w[0]));
        // 1800 Hz / 6 = 300 rev/s over (n-1)/fs s
        let expect = 300.0 * (n - 1) as f64 / FS;
        let got = phase[n - 1];
        assert!(
            (got - expect).abs() / expect < 0.03,
            "revs {got} vs {expect}"
        );
    }

    #[test]
    fn cumulative_phase_rejects_noise_and_short() {
        let mut lcg = Lcg(9);
        let noise: Vec<f64> = (0..4000).map(|_| 512.0 + lcg.next(3.0)).collect();
        assert!(cumulative_phase(&noise, FS, 6.0).is_none(), "pure noise");
        assert!(cumulative_phase(&[1.0; 10], FS, 6.0).is_none(), "too short");
    }
}
