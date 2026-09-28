//! Commutation ripple as a motor-side clock. A brushed 3-slot motor
//! commutates 6 times per rotor rev, stamping a ~kHz ripple on the winding
//! current - well above the pot's bandwidth but comfortably under the
//! 20 kHz tick rate, so a constant-duty rung's per-tick current carries a
//! direct, gear-slip-immune motor speed and, integrated, a motor angle.
//!
//! Two readers of it live here:
//!
//! - [`ripple_speed`], a tachometer: detrend with a sliding mean, then the
//!   dominant period by normalized autocorrelation with parabolic
//!   interpolation around the peak lag. Feeds [`crate::lut::cumulative_phase`].
//! - [`motor_angle`], the angle tracker of `oscnb.ripple`: the ripple line's
//!   integrated phase in cycles at every sample of a rung. A ridge track walks
//!   backward from the settled end of the rung in 200-sample windows, then a
//!   demodulation against the ridge phase counts cycles exactly wherever the
//!   line is present. Its traps, each measured in nb06: a spectral peak over
//!   an accelerating rung sits at the final frequency, not a mean, so cycles
//!   are counted, never divided; the line has a family at f/2, f/3 and f/6
//!   that overtakes it near 60% duty, so the seed comes from the pot speed
//!   times a coupling prior instead of the tallest peak; and the track needs
//!   the line above 250 Hz.

use core::f64::consts::PI;

use crate::fitmath;

/// One ripple-frequency estimate over a current window.
#[derive(Copy, Clone, Debug)]
pub struct RippleEstimate {
    /// Dominant ripple frequency, Hz.
    pub freq_hz: f64,
    /// Motor shaft speed, rev/s (= freq / ripple_per_rev).
    pub motor_rev_s: f64,
    /// Normalized autocorrelation at the peak (0..1); below ~0.15 the
    /// caller should not trust the estimate (ripple_speed returns None
    /// there already).
    pub strength: f64,
}

/// Lowest ripple frequency the autocorr band tracks; also fixes the max lag
/// (fs/TACH_F_LO samples) and thus ripple_speed's minimum input length.
const TACH_F_LO: f64 = 500.0;

/// Minimum sample count ripple_speed needs at `fs`: 4 periods of the slowest
/// tracked ripple. cumulative_phase in [`crate::lut`] sizes its window off it.
pub fn min_window(fs: f64) -> usize {
    4 * (fs / TACH_F_LO).round() as usize
}

/// Estimate ripple frequency in `i` sampled at `fs` Hz. `ripple_per_rev`
/// = commutation events per rotor rev (6 for a 3-slot brushed motor).
/// Searches 500 Hz..fs/5; None when the series is too short, flat, or no
/// autocorrelation peak clears the confidence floor.
pub fn ripple_speed(i: &[f64], fs: f64, ripple_per_rev: f64) -> Option<RippleEstimate> {
    const MIN_STRENGTH: f64 = 0.15;
    // band = TACH_F_LO .. fs/5: below 5 samples per period the parabolic
    // interpolation has nothing to stand on
    let lag_min = 5usize;
    let lag_max = (fs / TACH_F_LO).round() as usize;
    if i.len() < 4 * lag_max || fs <= 0.0 || ripple_per_rev <= 0.0 {
        return None;
    }
    let d = detrend(i, lag_max);
    let e: f64 = d.iter().map(|x| x * x).sum();
    if e <= 0.0 || !e.is_finite() {
        return None;
    }
    // normalized autocorrelation over the lag band
    let ac: Vec<f64> = (lag_min..=lag_max)
        .map(|lag| {
            let s: f64 = d[..d.len() - lag]
                .iter()
                .zip(&d[lag..])
                .map(|(a, b)| a * b)
                .sum();
            s / e
        })
        .collect();
    let k = ac
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1))
        .map(|(k, _)| k)?;
    // Reject a peak at either band edge: at lag_min it is short-range sample
    // correlation (the acceleration/reversal transient locks here), at lag_max
    // it is the detrend tail - neither is a real ripple period, and only an
    // interior peak has the two neighbors parabolic interpolation needs.
    if k == 0 || k + 1 == ac.len() {
        return None;
    }
    let strength = ac[k];
    if strength < MIN_STRENGTH {
        return None;
    }
    // parabolic interpolation around the peak lag (guaranteed interior above)
    let lag = (lag_min + k) as f64 + {
        let (a, b, c) = (ac[k - 1], ac[k], ac[k + 1]);
        let den = a - 2.0 * b + c;
        if den.abs() > 1e-12 {
            (0.5 * (a - c) / den).clamp(-0.5, 0.5)
        } else {
            0.0
        }
    };
    let freq_hz = fs / lag;
    Some(RippleEstimate {
        freq_hz,
        motor_rev_s: freq_hz / ripple_per_rev,
        strength,
    })
}

/// Gear-ratio cross-check: motor rev/s over output rev/s, with the output
/// speed taken as pot slope / `counts_per_output_rev`.
///
/// CAVEAT: the pot's 4096 counts span its electrical travel (~300 deg),
/// not a full output revolution, so with the default 4096.0 this is a
/// RELATIVE ratio - constant across rungs on a healthy train (its rung-to-
/// rung stability is the Ke-slip check), but not the physical gear ratio
/// until the pot's degrees-per-count is calibrated.
pub fn gear_ratio_check(motor_rev_s: f64, pot_omega_cps: f64, counts_per_output_rev: f64) -> f64 {
    motor_rev_s / (pot_omega_cps / counts_per_output_rev)
}

/// Subtract a centered sliding mean of width ~2*half+1 (clamped at the
/// edges); wide enough to pass the ripple band untouched.
fn detrend(v: &[f64], half: usize) -> Vec<f64> {
    let n = v.len();
    let mut out = Vec::with_capacity(n);
    // prefix sums make each window mean O(1)
    let mut pre = Vec::with_capacity(n + 1);
    pre.push(0.0);
    for &x in v {
        pre.push(pre.last().unwrap() + x);
    }
    for (k, &x) in v.iter().enumerate() {
        let lo = k.saturating_sub(half);
        let hi = (k + half + 1).min(n);
        let m = (pre[hi] - pre[lo]) / (hi - lo) as f64;
        out.push(x - m);
    }
    out
}

/// Search band of the ripple line, Hz: holds the 2S lines at 30 to 45% duty.
pub const F_LO: f64 = 200.0;
pub const F_HI: f64 = 6000.0;
/// The last fraction of a rung taken as settled.
pub const SETTLED: f64 = 0.6;
/// Zero padding of the settled window's spectrum.
const NFFT_MULT: usize = 8;

/// Ridge window and hop, samples: 10 ms and 1 ms at 20 kHz.
const RIDGE_WIN: usize = 200;
const RIDGE_HOP: usize = 20;
const RIDGE_NFFT: usize = 8192;
/// Per-hop search around the previous ridge point: excludes the nearest
/// family member (5/6 of the line) while the line itself moves at most 6%
/// per hop at the steepest acceleration seen.
const RIDGE_TOL: f64 = 0.15;
const RIDGE_F_MIN: f64 = 150.0;
/// Demodulation low-pass, as a fraction of the lowest ridge frequency, with
/// a floor in Hz.
const LP_FRAC: f64 = 0.25;
const LP_MIN_HZ: f64 = 30.0;
/// Ridge acceptance: amplitude over the band's median, and the frequency
/// under which the track is not trusted.
const SNR_MIN: f64 = 2.0;
const F_FLOOR: f64 = 250.0;
/// Seed band either side of the pot speed times the coupling prior.
const PRIOR_TOL: f64 = 0.3;
/// Samples dropped at the head of the accepted track: 5 ms at 20 kHz, fixed
/// in samples.
const TRACK_LEAD: usize = 100;

/// Amplitude spectrum in the units of the input, bins `k * bin_hz`.
#[derive(Clone, Debug)]
pub struct Spectrum {
    pub bin_hz: f64,
    pub amp: Vec<f64>,
}

impl Spectrum {
    pub fn freq(&self, k: usize) -> f64 {
        k as f64 * self.bin_hz
    }

    /// Bins with `lo <= freq <= hi`, inclusive; None when the band holds none.
    fn band(&self, lo: f64, hi: f64) -> Option<(usize, usize)> {
        let last = self.amp.len().checked_sub(1)?;
        let mut a = ((lo / self.bin_hz).floor().max(0.0) as usize).min(last);
        while a > 0 && self.freq(a - 1) >= lo {
            a -= 1;
        }
        while a < last && self.freq(a) < lo {
            a += 1;
        }
        let mut b = ((hi / self.bin_hz).ceil().max(0.0) as usize).min(last);
        while b < last && self.freq(b + 1) <= hi {
            b += 1;
        }
        while b > 0 && self.freq(b) > hi {
            b -= 1;
        }
        (a <= b && self.freq(a) >= lo && self.freq(b) <= hi).then_some((a, b))
    }
}

/// A spectral peak refined between bins.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Peak {
    pub hz: f64,
    pub amp: f64,
}

/// Amplitude spectrum of `x`, Hann windowed and zero padded 8x, the
/// window's coherent gain removed.
pub fn line_spectrum(x: &[f64], tick_hz: f64) -> Spectrum {
    windowed_spectrum(x, NFFT_MULT * x.len(), tick_hz)
}

fn windowed_spectrum(x: &[f64], nfft: usize, tick_hz: f64) -> Spectrum {
    let w = hann(x.len());
    let wsum: f64 = w.iter().sum();
    let d = detrend_linear(x);
    let xw: Vec<f64> = d.iter().zip(&w).map(|(a, b)| a * b).collect();
    let (re, im) = rfft(&xw, nfft);
    let amp = re
        .iter()
        .zip(&im)
        .map(|(r, i)| r.hypot(*i) * 2.0 / wsum)
        .collect();
    Spectrum {
        bin_hz: 1.0 / (nfft as f64 * (1.0 / tick_hz)),
        amp,
    }
}

/// Tallest bin in `lo..=hi` Hz, refined by a log-parabola through its
/// neighbors when it has two inside the band.
pub fn interp_peak(s: &Spectrum, lo: f64, hi: f64) -> Option<Peak> {
    let (a, b) = s.band(lo, hi)?;
    let mut i = a;
    for k in a + 1..=b {
        if s.amp[k] > s.amp[i] {
            i = k;
        }
    }
    let mut d = 0.0;
    if a < i && i < b {
        let (p, q, r) = (s.amp[i - 1].ln(), s.amp[i].ln(), s.amp[i + 1].ln());
        d = 0.5 * (p - r) / (p - 2.0 * q + r);
        if !d.is_finite() {
            d = 0.0;
        }
    }
    Some(Peak {
        hz: s.freq(i) + d * s.bin_hz,
        amp: s.amp[i],
    })
}

/// The ripple line of a settled window: the tallest peak in the search
/// band, promoted to its 6x, 3x or 2x multiple when that is at least half
/// as tall (the peak was a family member).
pub fn settled_line(s: &Spectrum) -> Option<Peak> {
    let p0 = interp_peak(s, F_LO, F_HI)?;
    for k in [6.0, 3.0, 2.0] {
        if k * p0.hz * 1.05 <= F_HI
            && let Some(pk) = interp_peak(s, k * p0.hz * 0.95, k * p0.hz * 1.05)
            && pk.amp >= 0.5 * p0.amp
        {
            return Some(pk);
        }
    }
    Some(p0)
}

/// First sample of the settled part of an `n`-sample rung.
pub fn settled(n: usize) -> usize {
    (n as f64 * (1.0 - SETTLED)) as usize
}

/// Least-squares slope of the wiper over the window, counts per second.
pub fn pot_speed(pos: &[u16], tick_hz: f64) -> Option<f64> {
    let xy: Vec<(f64, f64)> = pos
        .iter()
        .enumerate()
        .map(|(k, &p)| (k as f64 * (1.0 / tick_hz), p as f64))
        .collect();
    fitmath::linear_ls(&xy).map(|f| f.b)
}

/// One window of the ridge track, centered on sample `n`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RidgePoint {
    pub n: usize,
    pub hz: f64,
    pub amp: f64,
    /// Median amplitude over the ridge band, the floor `amp` is judged against.
    pub noise: f64,
}

/// Sliding-window peak track, walking backward from the end of `x` seeded
/// with the settled line; returned in sample order.
pub fn ridge_track(x: &[f64], seed_hz: f64, tick_hz: f64) -> Vec<RidgePoint> {
    let half = RIDGE_WIN / 2;
    let mut out = Vec::new();
    if x.len() < RIDGE_WIN {
        return out;
    }
    let centers: Vec<usize> = (half..x.len() - half).step_by(RIDGE_HOP).collect();
    let mut fprev = seed_hz;
    for &c in centers.iter().rev() {
        let s = windowed_spectrum(&x[c - half..c + half], RIDGE_NFFT, tick_hz);
        let Some((a, b)) = s.band(RIDGE_F_MIN, F_HI) else {
            break;
        };
        let Some(noise) = fitmath::median(&s.amp[a..=b]) else {
            break;
        };
        let lo = RIDGE_F_MIN.max(fprev * (1.0 - RIDGE_TOL));
        let hi = F_HI.min(fprev * (1.0 + RIDGE_TOL));
        let Some(pk) = interp_peak(&s, lo, hi) else {
            break;
        };
        out.push(RidgePoint {
            n: c,
            hz: pk.hz,
            amp: pk.amp,
            noise,
        });
        fprev = pk.hz;
    }
    out.reverse();
    out
}

/// Ripple phase in cycles at every sample of `x`: mix down with the ridge
/// phase, low-pass, unwrap the residual, add the ridge phase back. None
/// when the ridge is empty or its low-pass cannot be built.
pub fn demod_cycles(x: &[f64], ridge: &[RidgePoint], tick_hz: f64) -> Option<Vec<f64>> {
    let n = x.len();
    if ridge.is_empty() || n < 2 {
        return None;
    }
    let mut fmin = f64::INFINITY;
    let mut phref = Vec::with_capacity(n);
    let mut acc = 0.0;
    for k in 0..n {
        let f = interp_ridge(ridge, k);
        fmin = fmin.min(f);
        acc += f;
        phref.push(2.0 * PI * acc * (1.0 / tick_hz));
    }
    let d = detrend_linear(x);
    let zr: Vec<f64> = d.iter().zip(&phref).map(|(v, p)| v * p.cos()).collect();
    let zi: Vec<f64> = d.iter().zip(&phref).map(|(v, p)| v * -p.sin()).collect();
    let sos = butter3_lowpass(LP_MIN_HZ.max(LP_FRAC * fmin), tick_hz)?;
    let padlen = 1000.min(n - 1);
    let zr = sosfiltfilt(&sos, &zr, padlen);
    let zi = sosfiltfilt(&sos, &zi, padlen);
    let mut ang: Vec<f64> = zr.iter().zip(&zi).map(|(r, i)| i.atan2(*r)).collect();
    unwrap(&mut ang);
    Some(
        phref
            .iter()
            .zip(&ang)
            .map(|(p, a)| (p + a) / (2.0 * PI))
            .collect(),
    )
}

/// Linear interpolation of the ridge frequency at sample `k`, held flat
/// beyond the first and last ridge points.
fn interp_ridge(ridge: &[RidgePoint], k: usize) -> f64 {
    let Some(last) = ridge.last() else {
        return 0.0;
    };
    if k <= ridge[0].n {
        return ridge[0].hz;
    }
    if k >= last.n {
        return last.hz;
    }
    let j = ridge.partition_point(|r| r.n <= k) - 1;
    let (a, b) = (ridge[j], ridge[j + 1]);
    if k == a.n {
        return a.hz;
    }
    let slope = (b.hz - a.hz) / (b.n as f64 - a.n as f64);
    slope * (k as f64 - a.n as f64) + a.hz
}

/// The tracked motor angle of one constant-duty rung.
#[derive(Clone, Debug)]
pub struct MotorAngle {
    /// Ripple cycles at every sample, rising with the motor's rotation.
    pub cycles: Vec<f64>,
    /// First sample the track is trusted from.
    pub start: usize,
    /// The ridge the demodulation followed.
    pub ridge: Vec<RidgePoint>,
}

/// Motor angle over one drive rung from its bias-removed current and raw pot
/// samples. `c_prior` is a coupling estimate in ripple cycles per pot count,
/// used only to seed the track off the sub-line family. None when nothing in
/// the rung tracked.
pub fn motor_angle(current: &[f64], pos: &[u16], c_prior: f64, tick_hz: f64) -> Option<MotorAngle> {
    let n = current.len();
    if n != pos.len() || n == 0 || tick_hz <= 0.0 || tick_hz.is_nan() {
        return None;
    }
    let s0 = settled(n);
    let v = pot_speed(&pos[s0..], tick_hz)?.abs();
    let spec = line_spectrum(&current[s0..], tick_hz);
    let seed = interp_peak(
        &spec,
        F_LO.max(c_prior * v * (1.0 - PRIOR_TOL)),
        F_HI.min(c_prior * v * (1.0 + PRIOR_TOL)),
    )?;
    let mut ridge = ridge_track(current, seed.hz, tick_hz);
    let ok = |r: &RidgePoint| r.amp / r.noise >= SNR_MIN && r.hz >= F_FLOOR;
    if let Some(last_bad) = ridge.iter().rposition(|r| !ok(r)) {
        ridge.drain(..=last_bad);
    }
    if ridge.len() < 2 {
        return None;
    }
    let cycles = demod_cycles(current, &ridge, tick_hz)?;
    Some(MotorAngle {
        cycles,
        start: ridge[0].n + TRACK_LEAD,
        ridge,
    })
}

/// Periodic Hann window, scipy's `get_window("hann", n)`.
fn hann(n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![1.0; n];
    }
    let step = 2.0 * PI / n as f64;
    (0..n)
        .map(|k| 0.5 + 0.5 * (k as f64 * step - PI).cos())
        .collect()
}

/// Least-squares line removed.
fn detrend_linear(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 2 {
        return vec![0.0; n];
    }
    let mt = (n as f64 - 1.0) / 2.0;
    let my = x.iter().sum::<f64>() / n as f64;
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for (k, &y) in x.iter().enumerate() {
        let t = k as f64 - mt;
        sxx += t * t;
        sxy += t * (y - my);
    }
    let b = sxy / sxx;
    x.iter()
        .enumerate()
        .map(|(k, &y)| y - (my + b * (k as f64 - mt)))
        .collect()
}

/// Phase unwrap over 2 pi, numpy's `unwrap`.
fn unwrap(p: &mut [f64]) {
    let Some(mut prev) = p.first().copied() else {
        return;
    };
    let mut cum = 0.0;
    for v in p.iter_mut().skip(1) {
        let dd = *v - prev;
        prev = *v;
        let mut ddmod = (dd + PI) % (2.0 * PI);
        if ddmod < 0.0 {
            ddmod += 2.0 * PI;
        }
        ddmod -= PI;
        if ddmod == -PI && dd > 0.0 {
            ddmod = PI;
        }
        if dd.abs() >= PI {
            cum += ddmod - dd;
        }
        *v += cum;
    }
}

/// Bins `0..=nfft/2` of the DFT of `x` zero padded to `nfft`.
fn rfft(x: &[f64], nfft: usize) -> (Vec<f64>, Vec<f64>) {
    let mut re = vec![0.0; nfft];
    re[..x.len()].copy_from_slice(x);
    let mut im = vec![0.0; nfft];
    if nfft.is_power_of_two() {
        fft_pow2(&mut re, &mut im, false);
    } else {
        bluestein(&mut re, &mut im);
    }
    re.truncate(nfft / 2 + 1);
    im.truncate(nfft / 2 + 1);
    (re, im)
}

/// In-place radix-2 FFT; `inverse` conjugates the twiddles and scales by 1/n.
fn fft_pow2(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    if n < 2 {
        return;
    }
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let sign = if inverse { 1.0 } else { -1.0 };
    let tw: Vec<(f64, f64)> = (0..n / 2)
        .map(|k| {
            let a = sign * 2.0 * PI * k as f64 / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let stride = n / len;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let (wr, wi) = tw[k * stride];
                let (a, b) = (start + k, start + k + half);
                let xr = re[b] * wr - im[b] * wi;
                let xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
    if inverse {
        let s = 1.0 / n as f64;
        for k in 0..n {
            re[k] *= s;
            im[k] *= s;
        }
    }
}

/// In-place DFT of any length as a convolution with a chirp, three power-of-2
/// FFTs of at least twice the length.
fn bluestein(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let m = (2 * n - 1).next_power_of_two();
    let chirp: Vec<(f64, f64)> = (0..n)
        .map(|k| {
            let q = ((k as u64 * k as u64) % (2 * n as u64)) as f64;
            let a = -PI * q / n as f64;
            (a.cos(), a.sin())
        })
        .collect();
    let (mut ar, mut ai) = (vec![0.0; m], vec![0.0; m]);
    let (mut br, mut bi) = (vec![0.0; m], vec![0.0; m]);
    for k in 0..n {
        let (cr, ci) = chirp[k];
        ar[k] = re[k] * cr - im[k] * ci;
        ai[k] = re[k] * ci + im[k] * cr;
        br[k] = cr;
        bi[k] = -ci;
        if k > 0 {
            br[m - k] = cr;
            bi[m - k] = -ci;
        }
    }
    fft_pow2(&mut ar, &mut ai, false);
    fft_pow2(&mut br, &mut bi, false);
    for k in 0..m {
        let r = ar[k] * br[k] - ai[k] * bi[k];
        let i = ar[k] * bi[k] + ai[k] * br[k];
        ar[k] = r;
        ai[k] = i;
    }
    fft_pow2(&mut ar, &mut ai, true);
    for k in 0..n {
        let (cr, ci) = chirp[k];
        re[k] = ar[k] * cr - ai[k] * ci;
        im[k] = ar[k] * ci + ai[k] * cr;
    }
}

/// One biquad, `[b0, b1, b2, 1, a1, a2]`.
type Sos = [f64; 6];

/// scipy's `butter(3, fc, "lowpass", fs, output="sos")`: the analog prototype
/// warped and bilinear transformed, laid out as scipy pairs it (the real
/// pole with two zeros at -1 and the gain, the complex pair with one). None
/// unless `0 < fc < fs / 2`.
fn butter3_lowpass(fc: f64, fs: f64) -> Option<[Sos; 2]> {
    let wn = 2.0 * fc / fs;
    if !(wn > 0.0 && wn < 1.0) {
        return None;
    }
    let warped = 2.0 * 2.0 * (PI * wn / 2.0).tan();
    let fs2 = 4.0;
    let analog = |m: f64| {
        let th = (PI * m) * (1.0 / 6.0);
        (-(th.cos()) * warped, -(th.sin()) * warped)
    };
    let digital = |p: (f64, f64)| cdiv((fs2 + p.0, p.1), (fs2 - p.0, -p.1));
    let mut prod = (1.0, 0.0);
    for p in [analog(-2.0), analog(0.0), analog(2.0)] {
        let q = (fs2 - p.0, -p.1);
        prod = (prod.0 * q.0 - prod.1 * q.1, prod.0 * q.1 + prod.1 * q.0);
    }
    let k = warped.powf(3.0) * cdiv((1.0, 0.0), prod).0;
    let (pc, pr) = (digital(analog(-2.0)), digital(analog(0.0)));
    let c1 = -pc.0 - pc.0;
    let c2 = pc.0 * pc.0 + pc.1 * pc.1;
    Some([
        [k, k * 2.0, k, 1.0, -pr.0, 0.0],
        [1.0, 1.0, 0.0, 1.0, c1, c2],
    ])
}

/// Complex division as numpy does it, the scaled form.
fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    if b.0.abs() >= b.1.abs() {
        let rat = b.1 / b.0;
        let scl = 1.0 / (b.0 + b.1 * rat);
        ((a.0 + a.1 * rat) * scl, (a.1 - a.0 * rat) * scl)
    } else {
        let rat = b.0 / b.1;
        let scl = 1.0 / (b.1 + b.0 * rat);
        ((a.0 * rat + a.1) * scl, (a.1 * rat - a.0) * scl)
    }
}

/// Steady-state section states for a unit step, scipy's `sosfilt_zi`.
fn sos_zi(sos: &[Sos]) -> Vec<[f64; 2]> {
    let mut scale = 1.0;
    let mut out = Vec::with_capacity(sos.len());
    for s in sos {
        let [b0, b1, b2, _, a1, a2] = *s;
        // (I - A^T) z = B for the transposed direct form II, 2 x 2 with
        // partial pivoting
        let (m00, m01, m10, m11) = (1.0 + a1, -1.0, a2, 1.0);
        let (r0, r1) = (b1 - a1 * b0, b2 - a2 * b0);
        let (z0, z1) = if m00.abs() >= m10.abs() {
            let l = m10 / m00;
            let u11 = m11 - l * m01;
            let y1 = r1 - l * r0;
            let z1 = y1 / u11;
            ((r0 - m01 * z1) / m00, z1)
        } else {
            let l = m00 / m10;
            let u11 = m01 - l * m11;
            let y1 = r0 - l * r1;
            let z1 = y1 / u11;
            ((r1 - m11 * z1) / m10, z1)
        };
        out.push([scale * z0, scale * z1]);
        scale *= (b0 + b1 + b2) / (1.0 + a1 + a2);
    }
    out
}

/// Cascade of biquads over `x`, transposed direct form II from `zi`.
fn sosfilt(sos: &[Sos], x: &[f64], zi: &[[f64; 2]]) -> Vec<f64> {
    let mut z: Vec<[f64; 2]> = zi.to_vec();
    x.iter()
        .map(|&v| {
            let mut cur = v;
            for (s, st) in sos.iter().zip(z.iter_mut()) {
                let [b0, b1, b2, _, a1, a2] = *s;
                let y = b0 * cur + st[0];
                st[0] = b1 * cur - a1 * y + st[1];
                st[1] = b2 * cur - a2 * y;
                cur = y;
            }
            cur
        })
        .collect()
}

/// Zero-phase filtering, scipy's `sosfiltfilt` with odd extension by
/// `padlen` samples at each end and step-matched initial states.
fn sosfiltfilt(sos: &[Sos], x: &[f64], padlen: usize) -> Vec<f64> {
    let n = x.len();
    if n == 0 || padlen >= n {
        return Vec::new();
    }
    let mut ext = Vec::with_capacity(n + 2 * padlen);
    ext.extend((0..padlen).map(|j| 2.0 * x[0] - x[padlen - j]));
    ext.extend_from_slice(x);
    ext.extend((0..padlen).map(|j| 2.0 * x[n - 1] - x[n - 2 - j]));
    let zi = sos_zi(sos);
    let scaled = |v: f64| -> Vec<[f64; 2]> { zi.iter().map(|z| [z[0] * v, z[1] * v]).collect() };
    let y = sosfilt(sos, &ext, &scaled(ext[0]));
    let mut rev: Vec<f64> = y.iter().rev().copied().collect();
    rev = sosfilt(sos, &rev, &scaled(rev[0]));
    rev.reverse();
    rev[padlen..rev.len() - padlen].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

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

    fn ripple_series(freq: f64, amp: f64, noise: f64, drift: f64, n: usize) -> Vec<f64> {
        let mut lcg = Lcg(41);
        (0..n)
            .map(|k| {
                let t = k as f64 / FS;
                60.0 + drift * t
                    + amp * (2.0 * std::f64::consts::PI * freq * t).sin()
                    + lcg.next(noise)
            })
            .collect()
    }

    #[test]
    fn recovers_synthetic_ripple_frequency() {
        // 1.8 kHz ripple, amp 4 counts, noise +-3 counts, bemf-collapse
        // drift: 2% recovery despite the 11-sample raw lag grid
        let i = ripple_series(1800.0, 4.0, 3.0, -40.0, 4000);
        let e = ripple_speed(&i, FS, 6.0).expect("ripple found");
        assert!(
            (e.freq_hz - 1800.0).abs() / 1800.0 < 0.02,
            "freq {}",
            e.freq_hz
        );
        assert!((e.motor_rev_s - 300.0).abs() / 300.0 < 0.02);
        assert!(e.strength > 0.3, "strength {}", e.strength);
    }

    #[test]
    fn flat_or_pure_noise_returns_none() {
        assert!(ripple_speed(&vec![512.0; 4000], FS, 6.0).is_none());
        let mut lcg = Lcg(9);
        let noise: Vec<f64> = (0..4000).map(|_| 512.0 + lcg.next(3.0)).collect();
        assert!(ripple_speed(&noise, FS, 6.0).is_none(), "no periodicity");
        assert!(ripple_speed(&[1.0; 10], FS, 6.0).is_none(), "too short");
    }

    #[test]
    fn gear_ratio_check_is_rung_consistent() {
        // same physical train sampled at two speeds: the relative ratio
        // must agree even though 4096.0 is not a physical output rev
        let ratio_a = gear_ratio_check(300.0, 3679.0, 4096.0);
        let ratio_b = gear_ratio_check(150.0, 3679.0 / 2.0, 4096.0);
        assert!((ratio_a - ratio_b).abs() / ratio_a < 1e-12);
        assert!((ratio_a - 300.0 / (3679.0 / 4096.0)).abs() < 1e-9);
    }

    /// Parity against `oscnb.ripple.motor_angle` on committed mg90-a__2s
    /// rungs: `<name>.csv` is the rung's pos and current_raw, `<name>.json`
    /// the Python inputs, seed, ridge and cycles at a stride.
    struct Fixture {
        pos: Vec<u16>,
        current: Vec<f64>,
        r: serde_json::Value,
    }

    fn fixture(csv: &str, json: &str) -> Fixture {
        let r: serde_json::Value = serde_json::from_str(json).expect("reference json");
        let bias = r["bias"].as_f64().unwrap();
        let (mut pos, mut current) = (Vec::new(), Vec::new());
        for line in csv.lines().skip(1).filter(|l| !l.trim().is_empty()) {
            let (p, c) = line.split_once(',').expect("pos,current_raw");
            pos.push(p.trim().parse::<u16>().expect("pos"));
            current.push(c.trim().parse::<u16>().expect("current_raw") as f64 - bias);
        }
        assert_eq!(pos.len(), r["n"].as_u64().unwrap() as usize);
        Fixture { pos, current, r }
    }

    fn floats(v: &serde_json::Value) -> Vec<f64> {
        v.as_array()
            .unwrap()
            .iter()
            .map(|x| x.as_f64().unwrap())
            .collect()
    }

    /// Parity bar: cycles within 1e-6 of Python at every sampled index,
    /// ridge frequencies within 1e-6 relative; the seed and settled line
    /// within 1e-6 Hz.
    const CYCLES_TOL: f64 = 1e-6;
    const REL_TOL: f64 = 1e-6;

    fn check_parity(name: &str, fx: &Fixture) -> (f64, f64) {
        let tick_hz = fx.r["tick_hz"].as_f64().unwrap();
        let c_prior = fx.r["c_prior"].as_f64().unwrap();
        let n = fx.pos.len();
        let s0 = settled(n);
        let v = pot_speed(&fx.pos[s0..], tick_hz).expect("pot speed");
        let v_ref = fx.r["pot_speed_cps"].as_f64().unwrap();
        assert!(
            ((v - v_ref) / v_ref).abs() < REL_TOL,
            "{name}: pot speed {v} vs {v_ref}"
        );
        let spec = line_spectrum(&fx.current[s0..], tick_hz);
        let seed = interp_peak(
            &spec,
            F_LO.max(c_prior * v.abs() * (1.0 - PRIOR_TOL)),
            F_HI.min(c_prior * v.abs() * (1.0 + PRIOR_TOL)),
        )
        .expect("seed");
        let seed_ref = fx.r["seed_hz"].as_f64().unwrap();
        assert!(
            (seed.hz - seed_ref).abs() < REL_TOL,
            "{name}: seed {} vs {seed_ref}",
            seed.hz
        );
        let amp_ref = fx.r["seed_amp"].as_f64().unwrap();
        assert!(
            ((seed.amp - amp_ref) / amp_ref).abs() < REL_TOL,
            "{name}: seed amp {} vs {amp_ref}",
            seed.amp
        );
        let line = settled_line(&spec).expect("settled line");
        let line_ref = fx.r["settled_line_hz"].as_f64().unwrap();
        assert!(
            (line.hz - line_ref).abs() < REL_TOL,
            "{name}: settled line {} vs {line_ref}",
            line.hz
        );

        let m = motor_angle(&fx.current, &fx.pos, c_prior, tick_hz).expect("tracked");
        assert_eq!(
            m.start,
            fx.r["start"].as_u64().unwrap() as usize,
            "{name}: start"
        );
        assert_eq!(m.cycles.len(), n);
        let ridge_n = floats(&fx.r["ridge_n"]);
        let ridge_hz = floats(&fx.r["ridge_hz"]);
        let ridge_amp = floats(&fx.r["ridge_amp"]);
        let ridge_noise = floats(&fx.r["ridge_noise"]);
        assert_eq!(m.ridge.len(), ridge_n.len(), "{name}: ridge rows");
        let mut worst_hz = 0.0f64;
        for (i, r) in m.ridge.iter().enumerate() {
            assert_eq!(r.n, ridge_n[i] as usize, "{name}: ridge n at {i}");
            let e = ((r.hz - ridge_hz[i]) / ridge_hz[i]).abs();
            worst_hz = worst_hz.max(e);
            assert!(
                e < REL_TOL,
                "{name}: ridge hz at n {}: {} vs {}",
                r.n,
                r.hz,
                ridge_hz[i]
            );
            assert!(
                ((r.amp - ridge_amp[i]) / ridge_amp[i]).abs() < REL_TOL,
                "{name}: ridge amp at n {}: {} vs {}",
                r.n,
                r.amp,
                ridge_amp[i]
            );
            assert!(
                ((r.noise - ridge_noise[i]) / ridge_noise[i]).abs() < REL_TOL,
                "{name}: ridge noise at n {}: {} vs {}",
                r.n,
                r.noise,
                ridge_noise[i]
            );
        }
        let mut worst_cyc = 0.0f64;
        for pair in fx.r["cycles"].as_array().unwrap() {
            let i = pair[0].as_u64().unwrap() as usize;
            let c = pair[1].as_f64().unwrap();
            let e = (m.cycles[i] - c).abs();
            worst_cyc = worst_cyc.max(e);
            assert!(
                e < CYCLES_TOL,
                "{name}: cycles at {i}: {} vs {c}",
                m.cycles[i]
            );
        }
        (worst_hz, worst_cyc)
    }

    #[test]
    fn motor_angle_matches_oscnb_on_a_session_rung() {
        let fx = fixture(
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/ripple/session-1-grid-fwd40.csv"
            )),
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/ripple/session-1-grid-fwd40.json"
            )),
        );
        let (hz, cyc) = check_parity("session fwd 40%", &fx);
        eprintln!("session fwd 40%: worst ridge {hz:.2e} rel, worst cycles {cyc:.2e}");
    }

    #[test]
    fn motor_angle_matches_oscnb_on_an_ends_rung() {
        let fx = fixture(
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/ripple/ends-1-rev20.csv"
            )),
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/ripple/ends-1-rev20.json"
            )),
        );
        let (hz, cyc) = check_parity("ends rev 20%", &fx);
        eprintln!("ends rev 20%: worst ridge {hz:.2e} rel, worst cycles {cyc:.2e}");
    }

    #[test]
    fn interp_peak_resolves_a_tone_between_bins() {
        let x: Vec<f64> = (0..2000)
            .map(|k| 3.0 * (2.0 * PI * 1234.5 * k as f64 / FS).sin())
            .collect();
        let s = line_spectrum(&x, FS);
        let p = interp_peak(&s, F_LO, F_HI).expect("peak");
        assert!((p.hz - 1234.5).abs() < 0.05, "hz {}", p.hz);
        assert!((p.amp - 3.0).abs() < 0.01, "amp {}", p.amp);
        assert!(interp_peak(&s, 7000.0, 6000.0).is_none(), "empty band");
    }

    #[test]
    fn settled_line_promotes_off_a_family_member() {
        // the once-per-rev modulation at f/6 taller than the line itself
        let x: Vec<f64> = (0..4000)
            .map(|k| {
                let t = k as f64 / FS;
                4.0 * (2.0 * PI * 300.0 * t).sin() + 3.0 * (2.0 * PI * 1800.0 * t).sin()
            })
            .collect();
        let s = line_spectrum(&x, FS);
        let tallest = interp_peak(&s, F_LO, F_HI).expect("peak");
        assert!((tallest.hz - 300.0).abs() < 1.0, "tallest {}", tallest.hz);
        let line = settled_line(&s).expect("line");
        assert!((line.hz - 1800.0).abs() < 1.0, "line {}", line.hz);
    }

    #[test]
    fn motor_angle_counts_a_synthetic_spin_up() {
        // ripple sweeping 900 to 2000 Hz over 75 ms then holding, the pot
        // following at the prior's coupling, noise +-3 counts: the count
        // over the accepted track is the integrated frequency
        const C_PRIOR: f64 = 0.27;
        let n = 6000;
        let mut lcg = Lcg(7);
        let (mut ph, mut p) = (0.0f64, 600.0f64);
        let (mut current, mut pos, mut true_cyc) = (Vec::new(), Vec::new(), Vec::new());
        for k in 0..n {
            let f = 900.0 + 1100.0 * (k as f64 / 1500.0).min(1.0);
            true_cyc.push(ph);
            current.push(5.0 * (2.0 * PI * ph).sin() + lcg.next(3.0));
            pos.push(p.round() as u16);
            ph += f / FS;
            p += f / C_PRIOR / FS;
        }
        let m = motor_angle(&current, &pos, C_PRIOR, FS).expect("tracked");
        assert!(m.start < 400, "start {}", m.start);
        // the last few samples carry the zero-phase filter's edge, as they
        // do in Python; the count is read inside the track
        let end = n - 200;
        let got = m.cycles[end] - m.cycles[m.start];
        let want = true_cyc[end] - true_cyc[m.start];
        assert!((got - want).abs() < 0.1, "cycles {got} vs {want}");
        assert!(motor_angle(&current[..100], &pos[..100], C_PRIOR, FS).is_none());
        assert!(motor_angle(&current, &pos[..n - 1], C_PRIOR, FS).is_none());
    }

    #[test]
    fn zero_phase_lowpass_matches_scipy_shape() {
        // butter(3, 400, fs=20000, output="sos") as scipy lays it out
        let sos = butter3_lowpass(400.0, 20_000.0).expect("sos");
        let want = [
            [
                2.1960621122536214e-4,
                4.392124224507243e-4,
                2.1960621122536214e-4,
                1.0,
                -0.881618592363189,
                0.0,
            ],
            [1.0, 1.0, 0.0, 1.0, -1.8672172168514867, 0.8820578047856396],
        ];
        for (s, w) in sos.iter().zip(&want) {
            for (a, b) in s.iter().zip(w) {
                assert!((a - b).abs() <= 1e-12 * b.abs().max(1.0), "{sos:?}");
            }
        }
        assert!(butter3_lowpass(FS / 2.0, FS).is_none());
        // sosfiltfilt(sos, step at 1500 of 3000, padlen=1000), scipy's values
        let x: Vec<f64> = (0..3000)
            .map(|k| if k < 1500 { 0.0 } else { 1.0 })
            .collect();
        let y = sosfiltfilt(&sos, &x, 1000);
        assert_eq!(y.len(), x.len());
        for (i, want) in [
            (0, 0.0),
            (1490, 0.14435099669639767),
            (1500, 0.5209303492321176),
            (1520, 1.043891825718056),
            (2999, 1.0),
        ] {
            assert!((y[i] - want).abs() < 1e-12, "y[{i}] {} vs {want}", y[i]);
        }
    }
}
