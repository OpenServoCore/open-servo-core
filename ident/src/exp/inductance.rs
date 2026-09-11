//! Winding L from the firmware high-rate shunt burst. One capture is a
//! duty step sampled every ~1.08 us for 1.04 ms - fast enough that the
//! rotor cannot follow it, which is the bias [`super::rl`] could not shed.
//!
//! NOTHING HERE FEEDS GAIN SYNTHESIS YET, same standing as E7: the run is
//! recorded, fitted, gated and reported, and [`super::resistance`] remains
//! the R the table is built from.
//!
//! What the trace looks like. Under slow decay the shunt carries current
//! only during the ON half of each PWM period, so the capture is amplifier
//! bias with an ON window every ~46 samples. Inside a window the current
//! ramps; the first samples carry the shunt amplifier's edge settling
//! (2 to 3 us on this board, so 2 to 5 samples); the last full sample is
//! the ripple peak and the sample straddling the OFF edge is a partial
//! aperture.
//!
//! TWO INDUCTANCES come out, and they are different quantities rather than
//! two estimates of one:
//!
//!   - `L_ripple`, the incremental inductance over one ~25 us ON window.
//!     During ON the winding sees the rail whatever the duty is, so
//!     L = (V - V0 - i R) / (di/dt) and the duty never enters - which is
//!     the run's own consistency check. R comes from the pairs route and
//!     V0 from the from-a-hold control, because neither is separable
//!     inside a single from-rest capture. [`l_off`] measures the same
//!     quantity through the brake decay with no voltage term at all.
//!   - `L_env`, R x tau from the envelope of the per-window levels, over
//!     hundreds of microseconds.
//!
//! An iron-core motor reads LOWER at the PWM timescale - eddy currents in
//! the laminations oppose fast flux change - so L_ripple under L_env is
//! the expected sign and not a fault. The gates are a consistency check on
//! R (the pairs route against the asymptote route) and a spread check on
//! each L across the repeats of one duty.
//!
//! R itself comes from PAIRS of from-rest captures at two step duties:
//! delta(asymptote) / delta(D x V), where the fixed bridge and brush drop
//! cancels in the difference.
//!
//! Two bench facts shape the fit. The rail is read BEFORE the arm with no
//! load, so on a soft supply the winding sees less than that during the ON
//! window and the slope route reads L high - the same run's pairs R reads
//! high by the same mechanism, which is the tell. And the shunt amplifier
//! SLEWS on a big edge: on a 7.3 V rail a 40% ON edge moves 500 counts and
//! takes four or five samples, not one. That is why the ON window is fitted
//! with the lag as a column of the basis rather than by skipping leading
//! samples - at 20% duty the whole window is nine samples - and why a
//! window that is mostly transient declines its slope instead of reporting
//! the amplifier's catch-up as di/dt.
//!
//! The envelope's asymptote is an EXTRAPOLATION: the post-step half of a
//! capture is ~520 us, under four tau, so tau and every R route that leans
//! on the asymptote carry that error while the slope route does not.

use core::fmt::Write as _;

use super::rl::{Gate, Scales};
use super::{Cmd, Experiment, RigParams};
use crate::burst::{Capture, SAMPLE_US, nominal_cadence};
use crate::fitmath::{lag_ls, median, quantile, stddev};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

const Q15: f64 = 32767.0;

/// Mid travel with no soft guard configured: the pot's own midpoint.
const POT_MID: u16 = 2048;

/// Autocorrelation search band for the PWM period, in samples. 20 is well
/// under any PWM period this ADC clock can produce, 120 well over.
const LAG_MIN: usize = 20;
const LAG_MAX: usize = 120;

/// Windows whose last sample lands this close to the end of the buffer are
/// truncated by the capture, not by the PWM edge.
const CLIP_MARGIN: usize = 2;

/// The drive edge falls INSIDE one conversion aperture, so that sample is
/// a time average of before and after and no model describes it. Exactly
/// one: the lag basis covers everything after it.
const APERTURE_SKIP: usize = 1;

/// Fixed series drop - bridge FETs plus brushes - assumed when no
/// from-a-hold control measured it. Volts; the value E7's plant model
/// carries, stated wherever it is used rather than folded in silently.
pub const V0_DEFAULT_VOLTS: f64 = 0.4;

/// A control whose solved drop exceeds this is reporting mostly back-EMF,
/// not the fixed series drop, and is refused as a V0 source. Volts.
const V0_MAX_VOLTS: f64 = 1.5;

// --- fit configuration ------------------------------------------------------

/// Knobs shared by the live fit and the offline refit.
#[derive(Clone, Debug)]
pub struct FitCfg {
    pub sample_us: f64,
    /// Measured cadence must land inside nominal +/- this, samples.
    pub cadence_tol: f64,
    /// step_index the firmware publishes must land in this band.
    pub step_index_band: (u16, u16),
    pub min_post_windows: usize,
    /// Pre-step half at zero duty must read the bias this quietly, counts.
    pub pre_sd_max: f64,
    /// ON-window threshold as a fraction of the local peak deviation.
    pub on_frac: f64,
    /// Floor under `on_frac` so a small window is not found in noise.
    pub min_dev_counts: f64,
    /// Deviation that still counts as inside the ON window when walking
    /// its start back to the drive edge, counts.
    pub start_dev_counts: f64,
    pub min_fit_points: usize,
    /// Search band for the amplifier edge time constant, microseconds. The
    /// ceiling is load bearing: at one conversion per 1.08 us an
    /// exponential slower than a couple of microseconds is collinear with
    /// the ramp column, and the profile will happily trade slope for lag.
    pub settle_band_us: (f64, f64),
    /// Settling time constants an ON window must hold, on top of the
    /// aperture sample and the fit itself, before its slope is trusted.
    /// The amplifier SLEWS on a big step - on a 7.3 V rail a 40% ON edge
    /// moves 500 counts - so a window that is mostly transient reports the
    /// amplifier's catch-up as di/dt and L reads low.
    pub settle_taus: f64,
    /// Amplifier edge time constant to use instead of profiling this
    /// capture alone, microseconds. [`fit_captures`] profiles it once over
    /// the whole run and fills it in - it is a property of the board, not
    /// of one capture, and one window is a thin thing to fit it on.
    pub settle_us: Option<f64>,
    /// Post-step windows the slope route averages L over.
    pub l_windows: usize,
    /// Largest relative spread of one L across the repeats of a duty, and
    /// between the per-duty L medians.
    pub l_agree_tol: f64,
    /// Largest relative gap between the two measured R routes.
    pub r_agree_tol: f64,
    /// Smallest duty span a from-rest pair may carry for R, fraction of
    /// full scale. Under it the asymptote extrapolation error swamps
    /// delta(asymptote): the bench 20-vs-26% pair reads R 4x high.
    pub pair_min_duty_span: f64,
}

impl Default for FitCfg {
    fn default() -> Self {
        Self {
            sample_us: SAMPLE_US,
            cadence_tol: 1.0,
            step_index_band: (470, 500),
            min_post_windows: 6,
            pre_sd_max: 3.0,
            on_frac: 0.40,
            min_dev_counts: 5.0,
            start_dev_counts: 3.0,
            min_fit_points: 4,
            settle_band_us: (0.3, 3.0),
            settle_taus: 2.0,
            settle_us: None,
            l_windows: 3,
            l_agree_tol: 0.35,
            r_agree_tol: 0.25,
            pair_min_duty_span: 0.10,
        }
    }
}

// --- segmentation -----------------------------------------------------------

/// One ON window: `start` is the first sample the drive edge moved,
/// `end` the last full sample before the OFF edge.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Window {
    pub start: usize,
    pub end: usize,
}

impl Window {
    /// Full samples the window holds; never zero by construction.
    pub fn samples(&self) -> usize {
        self.end + 1 - self.start
    }
}

#[derive(Clone, Debug)]
pub struct Segmentation {
    /// PWM period from the autocorrelation of the trace's first difference.
    pub period_samples: f64,
    /// Median spacing of successive ON starts - the gated cadence.
    pub cadence_samples: f64,
    /// Median absolute first difference, counts.
    pub noise_counts: f64,
    pub windows: Vec<Window>,
}

/// PWM period from the trace alone. Autocorrelating the FIRST DIFFERENCE,
/// not the trace: the difference is blind to the rising envelope that a
/// raw-level autocorrelation would lock onto.
fn period_samples(v: &[f64]) -> Option<f64> {
    if v.len() < LAG_MAX * 3 {
        return None;
    }
    let d: Vec<f64> = v.windows(2).map(|w| w[1] - w[0]).collect();
    let m = d.iter().sum::<f64>() / d.len() as f64;
    let x: Vec<f64> = d.iter().map(|a| a - m).collect();
    let c0: f64 = x.iter().map(|a| a * a).sum();
    if c0 <= 0.0 {
        return None;
    }
    let corr =
        |lag: usize| -> f64 { x.iter().zip(&x[lag..]).map(|(a, b)| a * b).sum::<f64>() / c0 };
    let mut best = (f64::NEG_INFINITY, LAG_MIN);
    for lag in LAG_MIN..=LAG_MAX {
        let c = corr(lag);
        if c > best.0 {
            best = (c, lag);
        }
    }
    let l = best.1;
    if l == LAG_MIN || l == LAG_MAX {
        return Some(l as f64);
    }
    // parabolic refinement on the three correlations around the peak
    let (y0, y1, y2) = (corr(l - 1), best.0, corr(l + 1));
    let den = y0 - 2.0 * y1 + y2;
    Some(if den == 0.0 {
        l as f64
    } else {
        l as f64 - 0.5 * (y2 - y0) / den
    })
}

/// Median of `v` over a centred window of `w` samples, clipped at the ends.
fn rolling_median(v: &[f64], w: usize) -> Vec<f64> {
    let h = w / 2;
    (0..v.len())
        .map(|i| {
            let lo = i.saturating_sub(h);
            let hi = (i + h + 1).min(v.len());
            median(&v[lo..hi]).unwrap_or(v[i])
        })
        .collect()
}

fn rolling_peak(v: &[f64], w: usize) -> Vec<f64> {
    let h = w / 2;
    (0..v.len())
        .map(|i| {
            let lo = i.saturating_sub(h);
            let hi = (i + h + 1).min(v.len());
            v[lo..hi].iter().fold(0.0f64, |a, b| a.max(b.abs()))
        })
        .collect()
}

/// Find the ON windows. The OFF phase holds most of each period, so a
/// rolling median tracks the OFF level and an ON window is a run of samples
/// that departs from it - in EITHER direction, because a step taken from a
/// spinning rotor can drive the winding current negative (the ON window
/// then reads BELOW the bias; a from-a-hold capture on the bench does).
pub fn segment(samples: &[u16], cfg: &FitCfg) -> Option<Segmentation> {
    let v: Vec<f64> = samples.iter().map(|&x| x as f64).collect();
    let period = period_samples(&v)?;
    let w = (period.round() as usize) | 1;
    let noise = median(
        &v.windows(2)
            .map(|p| (p[1] - p[0]).abs())
            .collect::<Vec<_>>(),
    )?
    .max(1.0);
    let base = rolling_median(&v, w);
    let dev: Vec<f64> = v.iter().zip(&base).map(|(a, b)| a - b).collect();
    let peak = rolling_peak(&dev, w);
    let start_dev = cfg.start_dev_counts.max(2.0 * noise);

    let mut windows = Vec::new();
    let mut i = 0;
    while i < v.len() {
        let thr = |k: usize| cfg.min_dev_counts.max(cfg.on_frac * peak[k]);
        if dev[i].abs() < thr(i) {
            i += 1;
            continue;
        }
        let up = dev[i] > 0.0;
        let mut j = i;
        while j + 1 < v.len() && dev[j + 1].abs() >= thr(j + 1) && (dev[j + 1] > 0.0) == up {
            j += 1;
        }
        if j >= i + 2 {
            // The ramp direction is the ON window's own sign, not the edge's:
            // a regenerating winding steps DOWN into the window and ramps up
            // through it.
            let diffs: Vec<f64> = (i..j).map(|k| v[k + 1] - v[k]).collect();
            let dir = if median(&diffs).unwrap_or(0.0) >= 0.0 {
                1.0
            } else {
                -1.0
            };
            // trailing samples that turn back are the partial aperture at
            // the OFF edge
            let mut end = j;
            while end > i + 1 && (v[end] - v[end - 1]) * dir < 0.0 {
                end -= 1;
            }
            // walk the start back to the drive edge: the run threshold sits
            // at a fraction of the peak and cuts into the ramp
            let sgn = if up { 1.0 } else { -1.0 };
            let mut start = i;
            while start > 0 && dev[start - 1] * sgn > start_dev {
                start -= 1;
            }
            if end >= start + 2 {
                windows.push(Window { start, end });
            }
        }
        i = j + 1;
    }
    let gaps: Vec<f64> = windows
        .windows(2)
        .map(|p| (p[1].start - p[0].start) as f64)
        .collect();
    let cadence = median(&gaps).unwrap_or(period);
    Some(Segmentation {
        period_samples: period,
        cadence_samples: cadence,
        noise_counts: noise,
        windows,
    })
}

// --- per-capture fit --------------------------------------------------------

/// One ON window reduced to a slope and a level. Every current here is the
/// LAG-FREE part of the fit: the amplifier's edge transient is a column of
/// the basis, not an error to be skipped past.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindowFit {
    pub start: usize,
    pub end: usize,
    pub didt_a_per_s: f64,
    /// Period-mean current, amps, bias subtracted. The ripple is
    /// piecewise linear, so the ramp fit at the ON window's centre is the
    /// mean over the whole period - no OFF-phase samples needed.
    pub level_a: f64,
    /// Current at the first and last fitted sample: the ripple trough and
    /// peak. Their difference is the ripple this window carries.
    pub i_start_a: f64,
    pub i_end_a: f64,
    /// Fit residual, counts. A window the lag model does not describe
    /// (a dropped conversion, a commutation event) shows up here.
    pub rms_counts: f64,
}

impl WindowFit {
    pub fn ripple_a(&self) -> f64 {
        self.i_end_a - self.i_start_a
    }
}

#[derive(Clone, Debug)]
pub struct CaptureFit {
    pub cadence_samples: f64,
    pub period_samples: f64,
    pub noise_counts: f64,
    /// Current-sense zero used, counts.
    pub bias_counts: f64,
    /// True when the bias came from the capture's own zero-duty pre-step
    /// half instead of the servo's published tracker value.
    pub bias_from_trace: bool,
    pub pre_sd_counts: f64,
    /// Amplifier edge time constant profiled out of this capture's own
    /// windows, microseconds.
    pub settle_us: f64,
    /// Median ON-window length, samples.
    pub window_samples: f64,
    /// The ON windows are long enough for an unbiased slope. False means
    /// the envelope numbers stand but L_ripple declines.
    pub slope_ok: bool,
    pub windows: Vec<WindowFit>,
    /// Step duty as a fraction of full scale, magnitude.
    pub duty: f64,
    /// Pre-step duty as a fraction of full scale, magnitude.
    pub duty_pre: f64,
    /// Rail as the host read it BEFORE the arm - no load. Under a soft
    /// supply the winding sees less than this during the ON window.
    pub v_rail: f64,
    /// The capture started from a braked shaft at zero duty (E = 0).
    pub from_rest: bool,
    pub tau_us: f64,
    /// Decay time constant of the OFF (brake) phase, from the peak of each
    /// window to the trough of the next. Measured at the PWM timescale and
    /// with no voltage term at all, so R x tau_off is an inductance the
    /// rail reading cannot bias.
    pub tau_off_us: f64,
    /// Envelope asymptote, amps. An extrapolation - see the module note.
    pub asymptote_a: f64,
    /// delta(D x V) / delta(settled current) across the capture's own two
    /// halves. From rest the pre-step half is zero current at zero duty, so
    /// this is D x V / asymptote with the fixed bridge drop folded in.
    pub r_capture_ohm: f64,
    /// Settled current of the pre-step half, amps. Zero from rest.
    pub pre_level_a: f64,
    /// E plus the fixed bridge and brush drop the two halves imply, volts.
    /// Exactly zero from rest, where it is not separable from R.
    pub drop_volts: f64,
    pub step_index: u16,
    pub gates: Vec<Gate>,
    pub ok: bool,
    pub notes: Vec<String>,
}

fn gate(name: &'static str, pass: bool, detail: String) -> Gate {
    Gate { name, pass, detail }
}

/// Minimise `cost` over a time constant in [lo, hi]: log grid, then golden
/// section around the best cell. Both time constants this module fits - the
/// amplifier edge and the current envelope - are one-dimensional profiles
/// with everything else closed form at each trial value.
fn profile_min(lo: f64, hi: f64, cost: impl Fn(f64) -> f64) -> f64 {
    const GRID: usize = 60;
    const GOLDEN_STEPS: usize = 40;
    let mut best = (f64::INFINITY, lo);
    for k in 0..GRID {
        let x = lo * (hi / lo).powf(k as f64 / (GRID - 1) as f64);
        let c = cost(x);
        if c < best.0 {
            best = (c, x);
        }
    }
    let step = (hi / lo).powf(1.0 / (GRID - 1) as f64);
    let (mut a, mut b) = ((best.1 / step).max(lo), (best.1 * step).min(hi));
    let phi = (5f64.sqrt() - 1.0) / 2.0;
    for _ in 0..GOLDEN_STEPS {
        let x1 = b - phi * (b - a);
        let x2 = a + phi * (b - a);
        if cost(x1) < cost(x2) { b = x2 } else { a = x1 }
    }
    (a + b) / 2.0
}

/// Fit A - B * exp(-t / tau) to the level envelope by profiling tau: A and
/// B are closed form at each tau. None when the basis is degenerate.
fn fit_exponential(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    const TAU_LO: f64 = 10e-6;
    const TAU_HI: f64 = 3000e-6;
    if pts.len() < 3 || !pts.iter().all(|(t, y)| t.is_finite() && y.is_finite()) {
        return None;
    }
    let solve = |tau: f64| -> Option<(f64, f64)> {
        let n = pts.len() as f64;
        let (mut se, mut see, mut sy, mut sey) = (0.0, 0.0, 0.0, 0.0);
        for (t, y) in pts {
            let e = (-t / tau).exp();
            se += e;
            see += e * e;
            sy += y;
            sey += e * y;
        }
        let det = n * see - se * se;
        if det.abs() < 1e-15 {
            return None;
        }
        let a = (sy * see - se * sey) / det;
        let c = (n * sey - se * sy) / det;
        let rss = pts
            .iter()
            .map(|(t, y)| (y - (a + c * (-t / tau).exp())).powi(2))
            .sum::<f64>();
        Some((rss, a))
    };
    let tau = profile_min(TAU_LO, TAU_HI, |x| solve(x).map_or(f64::INFINITY, |r| r.0));
    let (_, a) = solve(tau)?;
    (tau.is_finite() && a.is_finite()).then_some((tau, a))
}

/// Captures the settle profile samples. The amplifier edge is one number
/// for the board, so a handful of captures spread across the run pins it;
/// profiling on all forty buys nothing and costs a segmentation each.
const SETTLE_PROFILE_CAPTURES: usize = 8;

/// The board's amplifier edge time constant, microseconds, profiled over a
/// spread of the run's captures. Segmentation is hoisted out of the search:
/// a profile re-fits every window a few hundred times and re-segmenting
/// each trial would dominate the run.
pub fn settle_profile(caps: &[Capture], sc: &Scales, cfg: &FitCfg) -> f64 {
    let stride = caps.len().div_ceil(SETTLE_PROFILE_CAPTURES).max(1);
    let prepared: Vec<(&Capture, Segmentation)> = caps
        .iter()
        .step_by(stride)
        .filter_map(|c| segment(&c.samples, cfg).map(|s| (c, s)))
        .collect();
    let cost = |us: f64| -> f64 {
        let probe = FitCfg {
            settle_us: Some(us),
            ..cfg.clone()
        };
        let rs: Vec<f64> = prepared
            .iter()
            .filter_map(|(c, seg)| fit_segmented(c, sc, &probe, seg))
            .flat_map(|f| f.windows.into_iter().map(|w| w.rms_counts))
            .collect();
        if rs.is_empty() {
            f64::INFINITY
        } else {
            rs.iter().sum::<f64>() / rs.len() as f64
        }
    };
    profile_min(cfg.settle_band_us.0, cfg.settle_band_us.1, cost)
}

/// Everything one capture gives, from the samples alone.
pub fn fit_capture(cap: &Capture, sc: &Scales, cfg: &FitCfg) -> Option<CaptureFit> {
    let seg = segment(&cap.samples, cfg)?;
    fit_segmented(cap, sc, cfg, &seg)
}

fn fit_segmented(
    cap: &Capture,
    sc: &Scales,
    cfg: &FitCfg,
    seg: &Segmentation,
) -> Option<CaptureFit> {
    let n = cap.samples.len();
    let v: Vec<f64> = cap.samples.iter().map(|&x| x as f64).collect();
    let step_index = cap.meta.step_index as usize;
    if step_index >= n {
        return None;
    }
    let mut notes = Vec::new();

    // Bias. The pre-step half at zero duty IS the amplifier zero, measured
    // in the same capture and the same aperture; prefer it to the servo's
    // published tracker value, which is sampled under drive.
    let pre = &v[..step_index];
    let pre_has_drive = seg.windows.iter().any(|w| w.end < step_index);
    let from_rest = cap.meta.pre_q15 == 0 && !pre_has_drive;
    let pre_sd = stddev(pre).unwrap_or(f64::INFINITY);
    let (bias, bias_from_trace) = match from_rest.then(|| median(pre)).flatten() {
        Some(b) => (b, true),
        None => (cap.meta.bias as f64, false),
    };
    if bias_from_trace {
        let delta = bias - cap.meta.bias as f64;
        if delta.abs() > cfg.pre_sd_max {
            notes.push(format!(
                "trace bias {bias:.1} counts is {delta:+.1} off the servo's published \
                 {} - the tracker samples under drive",
                cap.meta.bias
            ));
        }
    }

    // Post-step windows: the ones the step actually drove, whole, and not
    // cut short by the end of the buffer.
    let post: Vec<Window> = seg
        .windows
        .iter()
        .copied()
        .filter(|w| w.start >= step_index && w.end + CLIP_MARGIN < n)
        .collect();
    let full = median(&post.iter().map(|w| w.samples() as f64).collect::<Vec<_>>()).unwrap_or(0.0);
    // The first post-step window is a HALF window: the compare register
    // latches at the crest, so only the falling half of that period carries
    // the new duty. Its slope is honest, its centre is not the period's, so
    // it is dropped rather than fitted.
    let keep: Vec<Window> = post
        .iter()
        .copied()
        .filter(|w| {
            let l = w.samples() as f64;
            l >= 0.6 * full && l <= 1.4 * full
        })
        .collect();

    // The low-side shunt sees drive current the same way whichever way the
    // bridge is pointed, so a reverse step reads above the bias exactly
    // like a forward one; no direction sign enters here. A reading BELOW
    // the bias is real regeneration, not a reversed drive.
    let amps = |k: usize| (v[k] - bias) * sc.amps_per_count;
    let t = |k: usize| k as f64 * cfg.sample_us * 1e-6;
    let counts = |a: f64| a / (sc.amps_per_count.max(f64::MIN_POSITIVE));

    // One ON window as a ramp read through a first-order lag. Dropping
    // leading samples instead cannot work at low duty - a 20% window is
    // nine samples and the edge is two or three of them - so the lag is a
    // column of the fit and every sample is used. APERTURE_SKIP drops the
    // one sample the drive edge falls inside, which no model describes.
    let fit_window = |w: &Window, tau_s: f64| {
        let pts: Vec<(f64, f64)> = (w.start + APERTURE_SKIP..=w.end)
            .map(|k| (t(k) - t(w.start + APERTURE_SKIP), amps(k)))
            .collect();
        (pts.len() >= cfg.min_fit_points)
            .then(|| lag_ls(&pts, tau_s))
            .flatten()
    };
    let pooled_rms = |tau_s: f64| -> f64 {
        let rs: Vec<f64> = keep
            .iter()
            .filter_map(|w| fit_window(w, tau_s).map(|f| f.rms))
            .collect();
        if rs.is_empty() {
            f64::INFINITY
        } else {
            rs.iter().sum::<f64>() / rs.len() as f64
        }
    };
    let settle_s = match cfg.settle_us {
        Some(us) => us * 1e-6,
        None => profile_min(
            cfg.settle_band_us.0 * 1e-6,
            cfg.settle_band_us.1 * 1e-6,
            pooled_rms,
        ),
    };

    let mut windows = Vec::new();
    for w in &keep {
        let Some(f) = fit_window(w, settle_s) else {
            continue;
        };
        let span = t(w.end) - t(w.start + APERTURE_SKIP);
        windows.push(WindowFit {
            start: w.start,
            end: w.end,
            didt_a_per_s: f.b,
            // The window's own centre, not the fitted span's: the fitted
            // span starts one aperture in.
            level_a: f.a
                + f.b * (t(w.start) + (t(w.end) - t(w.start)) / 2.0 - t(w.start + APERTURE_SKIP)),
            i_start_a: f.a,
            i_end_a: f.a + f.b * span,
            rms_counts: counts(f.rms),
        });
    }

    // Settled level of the pre-step half, from its last windows. Zero when
    // the shaft was braked at zero duty.
    let pre_level = {
        let mut pw: Vec<&Window> = seg.windows.iter().filter(|w| w.end < step_index).collect();
        pw.reverse();
        let lv: Vec<f64> = pw
            .iter()
            .take(3)
            .filter_map(|w| {
                fit_window(w, settle_s).map(|f| {
                    f.a + f.b
                        * (t(w.start) + (t(w.end) - t(w.start)) / 2.0 - t(w.start + APERTURE_SKIP))
                })
            })
            .collect();
        median(&lv).unwrap_or(0.0)
    };

    // OFF-phase decay: the peak of one window to the trough of the next,
    // over the OFF interval between them. No voltage term enters, so this
    // tau is immune to whatever the rail actually did under load.
    let tau_off_us = {
        let ratios: Vec<f64> = windows
            .windows(2)
            .filter_map(|p| {
                let t_off = t(p[1].start + APERTURE_SKIP) - t(p[0].end);
                let (peak, trough) = (p[0].i_end_a, p[1].i_start_a);
                (t_off > 0.0 && peak > 0.0 && trough > 0.0 && trough < peak)
                    .then(|| -t_off / (trough / peak).ln() * 1e6)
            })
            .collect();
        median(&ratios).unwrap_or(0.0)
    };

    let duty = (cap.meta.step_q15 as f64 / Q15).abs();
    let duty_pre = (cap.meta.pre_q15 as f64 / Q15).abs();
    let v_rail = cap.meta.vbus_raw as f64 * sc.v_rail_per_count;
    let env: Vec<(f64, f64)> = windows
        .iter()
        .map(|w| (t(w.start) - t(step_index), w.level_a))
        .collect();
    let (tau_s, asym) = fit_exponential(&env).unwrap_or((0.0, 0.0));
    // Both halves of one capture see the same rotor speed (500 us of
    // mechanical time), so D V = I R + drop with drop = E + the bridge and
    // brush offset solves for R and drop from the pair of settled levels.
    // From rest it degenerates to D V / I with drop = 0, which folds the
    // fixed offset into R - that is why the R the run reports comes from
    // pairs of CAPTURES, where the offset cancels.
    // A step DOWN carries both differences negative; only their ratio has
    // to be positive for the solve to mean anything.
    let di = asym - pre_level;
    let dd = duty - duty_pre;
    let r_capture = if di != 0.0 && dd / di > 0.0 {
        dd * v_rail / di
    } else {
        0.0
    };
    let drop = duty_pre * v_rail - pre_level * r_capture;
    // A window has to outlive the amplifier before its slope means
    // anything: the aperture sample, then `settle_taus` of edge transient,
    // then something to fit.
    let settle_samples = (cfg.settle_taus * settle_s * 1e6 / cfg.sample_us).ceil();
    let slope_min = APERTURE_SKIP as f64 + settle_samples + cfg.min_fit_points as f64;
    let slope_ok = full >= slope_min;
    if !slope_ok {
        notes.push(format!(
            "ON windows are {full:.0} samples at {:.0}% duty, under the {slope_min:.0} the \
             aperture sample plus {:.1} us of settling plus a fit needs: no slope route, \
             the envelope numbers stand",
            duty * 100.0,
            cfg.settle_taus * settle_s * 1e6
        ));
    }

    let nominal = nominal_cadence(cap.meta.pwm_arr);
    let gates = vec![
        gate(
            "cadence",
            (seg.cadence_samples - nominal).abs() <= cfg.cadence_tol,
            format!(
                "{:.2} samples/period (nominal {nominal:.2})",
                seg.cadence_samples
            ),
        ),
        gate(
            "step-index",
            (cfg.step_index_band.0..=cfg.step_index_band.1).contains(&cap.meta.step_index),
            format!(
                "{} (band {}..{})",
                cap.meta.step_index, cfg.step_index_band.0, cfg.step_index_band.1
            ),
        ),
        match from_rest {
            true => gate(
                "pre-bias",
                pre_sd <= cfg.pre_sd_max,
                format!("sd {pre_sd:.2} counts at zero duty"),
            ),
            false => gate(
                "pre-bias",
                true,
                format!("pre-step drive {:+} q15: not applicable", cap.meta.pre_q15),
            ),
        },
        gate(
            "windows",
            windows.len() >= cfg.min_post_windows,
            format!(
                "{} post-step, {} usable, {full:.0} samples each (slope needs {slope_min:.0})",
                post.len(),
                windows.len()
            ),
        ),
    ];
    let ok = gates.iter().all(|g| g.pass);
    if !from_rest {
        notes.push(
            "pre-step drive: the rotor is spinning, so the two-half solve reports E plus the \
             bridge drop together in drop_volts - which is what makes it the V0 source"
                .into(),
        );
    }
    Some(CaptureFit {
        cadence_samples: seg.cadence_samples,
        period_samples: seg.period_samples,
        noise_counts: seg.noise_counts,
        bias_counts: bias,
        bias_from_trace,
        pre_sd_counts: pre_sd,
        settle_us: settle_s * 1e6,
        window_samples: full,
        slope_ok,
        windows,
        duty,
        duty_pre,
        v_rail,
        from_rest,
        tau_us: tau_s * 1e6,
        tau_off_us,
        asymptote_a: asym,
        r_capture_ohm: r_capture,
        pre_level_a: pre_level,
        drop_volts: drop,
        step_index: cap.meta.step_index,
        gates,
        ok,
        notes,
    })
}

/// L at the PWM timescale from a capture's ON-window slopes: during ON the
/// winding sees the rail less the fixed series drop and its own i R, so
/// L = (V - V0 - i R) / (di/dt) and the duty never enters. `r_ohm` and
/// `v0` are the RUN's numbers - the pairs route and the from-a-hold
/// control - because neither is separable inside one from-rest capture.
///
/// `V` is the rail as read before the arm. On a soft supply the winding
/// sees less than that while the drive is on and this route reads high;
/// [`l_off`] is the version with no voltage term at all.
pub fn l_ripple(f: &CaptureFit, r_ohm: f64, v0: f64, cfg: &FitCfg) -> Option<f64> {
    if !f.slope_ok {
        return None;
    }
    let ls: Vec<f64> = f
        .windows
        .iter()
        .take(cfg.l_windows)
        .filter(|w| w.didt_a_per_s > 0.0)
        .map(|w| (f.v_rail - v0 - w.level_a * r_ohm) / w.didt_a_per_s)
        .filter(|l| *l > 0.0 && l.is_finite())
        .collect();
    median(&ls)
}

/// L at the PWM timescale with no voltage term: during the brake phase the
/// winding is shorted, so the decay is pure L/R. The same incremental
/// inductance as [`l_ripple`], measured through a different loop - the
/// brake path is both low-side FETs where the drive path adds the high
/// side and the supply wiring - so a few percent apart is expected.
pub fn l_off(f: &CaptureFit, r_ohm: f64) -> Option<f64> {
    (f.slope_ok && f.tau_off_us > 0.0 && r_ohm > 0.0).then_some(r_ohm * f.tau_off_us * 1e-6)
}

/// R from one pair of from-rest captures at two step duties. V0 - the
/// bridge and brush drop, fixed at both duties - cancels in the difference,
/// which the per-capture route cannot do.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PairR {
    pub r_ohm: f64,
    pub duty_lo: f64,
    pub duty_hi: f64,
    pub di_a: f64,
    pub ddv_volts: f64,
}

pub fn pair_resistance(a: &CaptureFit, b: &CaptureFit, cfg: &FitCfg) -> Option<PairR> {
    if !a.from_rest || !b.from_rest {
        return None;
    }
    let (lo, hi) = if a.duty <= b.duty { (a, b) } else { (b, a) };
    if hi.duty - lo.duty < cfg.pair_min_duty_span {
        return None;
    }
    let di = hi.asymptote_a - lo.asymptote_a;
    let ddv = hi.duty * hi.v_rail - lo.duty * lo.v_rail;
    (di > 0.0 && ddv > 0.0).then(|| PairR {
        r_ohm: ddv / di,
        duty_lo: lo.duty,
        duty_hi: hi.duty,
        di_a: di,
        ddv_volts: ddv,
    })
}

// --- run result -------------------------------------------------------------

/// The run's two inductances. They are DIFFERENT QUANTITIES, not two
/// estimates of one: `ripple` is the incremental inductance the winding
/// shows over one ~25 us ON window, `env` the effective inductance the
/// current envelope integrates over hundreds of microseconds. An iron-core
/// motor reads lower at the PWM timescale - eddy currents in the laminations
/// oppose fast flux change - so ripple below env is the expected sign, and
/// only their spread ACROSS REPEATS is a gate.
#[derive(Clone, Debug)]
pub struct InductanceResult {
    /// Incremental L from the ON-window slopes, henries.
    pub l_ripple_h: f64,
    pub l_ripple_bracket: (f64, f64),
    /// The same incremental L measured through the brake path, with no
    /// voltage term: R x tau_off. None when no capture resolved the OFF
    /// decay.
    pub l_off_h: Option<f64>,
    /// Envelope L: R x tau. Henries.
    pub l_env_h: f64,
    pub l_env_bracket: (f64, f64),
    pub tau_us: f64,
    pub tau_bracket: (f64, f64),
    pub tau_off_us: f64,
    /// R from from-rest pairs, ohms. None when no pair cleared the duty
    /// span the asymptote extrapolation needs. This is the run's R.
    pub r_pair_ohm: Option<f64>,
    pub r_pair_bracket: Option<(f64, f64)>,
    pub pairs: Vec<PairR>,
    /// (D x V - V0) / asymptote, median over the from-rest captures - the
    /// second measured R, independent of the pair differencing.
    pub r_asym_ohm: f64,
    /// Fixed series drop used by the slope route, volts.
    pub v0_volts: f64,
    /// True when V0 came from a from-a-hold control rather than the
    /// pre-registered default.
    pub v0_measured: bool,
    /// L_ripple per step duty - the duty-independence check, (duty, L).
    pub l_by_duty: Vec<(f64, f64)>,
    /// Worst relative spread of L_ripple / L_env across the repeats of one
    /// step duty.
    pub ripple_spread: f64,
    pub env_spread: f64,
    pub bias_counts: f64,
    pub settle_us: f64,
    pub window_samples: f64,
    pub cadence_samples: f64,
    pub rest_captures: usize,
    pub hold_captures: usize,
    pub gates: Vec<Gate>,
    /// Every gate passed. Advisory: nothing consumes this yet.
    pub ok: bool,
    pub warnings: Vec<String>,
}

fn bracket(v: &[f64], fallback: f64) -> (f64, f64) {
    match (quantile(v, 0.16), quantile(v, 0.84)) {
        (Some(lo), Some(hi)) => (lo, hi),
        _ => (fallback, fallback),
    }
}

/// Worst (max - min) / median across the groups, each group being one step
/// duty's repeats. Grouping first keeps a real duty dependence out of the
/// repeat-spread number.
fn worst_spread(groups: &[Vec<f64>]) -> f64 {
    groups
        .iter()
        .filter_map(|g| {
            let m = median(g)?;
            let lo = g.iter().copied().fold(f64::MAX, f64::min);
            let hi = g.iter().copied().fold(f64::MIN, f64::max);
            (m > 0.0 && g.len() > 1).then(|| (hi - lo) / m)
        })
        .fold(0.0f64, f64::max)
}

/// Every number the run reports, from recorded captures alone - so the
/// offline refit and the live fit cannot diverge.
///
/// Three passes, because the slope route needs numbers only the whole run
/// has: fit every capture, solve R from the pairs and V0 from the control,
/// then read L out of the per-capture geometry with both in hand.
pub fn fit_captures(caps: &[Capture], sc: &Scales, cfg: &FitCfg) -> Option<InductanceResult> {
    let cfg = &FitCfg {
        settle_us: cfg
            .settle_us
            .or_else(|| Some(settle_profile(caps, sc, cfg))),
        ..cfg.clone()
    };
    let fits: Vec<CaptureFit> = caps
        .iter()
        .filter_map(|c| fit_capture(c, sc, cfg))
        .collect();
    if fits.is_empty() {
        return None;
    }
    let rest: Vec<&CaptureFit> = fits.iter().filter(|f| f.from_rest).collect();
    let holds: Vec<&CaptureFit> = fits.iter().filter(|f| !f.from_rest).collect();
    let hold = holds.len();
    let mut warnings = Vec::new();

    let taus: Vec<f64> = rest.iter().map(|f| f.tau_us).filter(|t| *t > 0.0).collect();
    let tau = median(&taus).unwrap_or(0.0);

    // Group the from-rest captures by step duty: the pair route differences
    // GROUP medians, not single noisy asymptotes.
    let mut duties: Vec<f64> = rest.iter().map(|f| f.duty).collect();
    duties.sort_by(f64::total_cmp);
    duties.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let by_duty: Vec<Vec<&CaptureFit>> = duties
        .iter()
        .map(|d| {
            rest.iter()
                .copied()
                .filter(|f| (f.duty - d).abs() < 1e-6)
                .collect()
        })
        .collect();
    let mut groups: Vec<CaptureFit> = Vec::new();
    for same in &by_duty {
        let mut rep = same[0].clone();
        rep.asymptote_a =
            median(&same.iter().map(|f| f.asymptote_a).collect::<Vec<_>>()).unwrap_or(0.0);
        rep.v_rail = median(&same.iter().map(|f| f.v_rail).collect::<Vec<_>>()).unwrap_or(0.0);
        groups.push(rep);
    }
    let mut pairs = Vec::new();
    for (i, a) in groups.iter().enumerate() {
        for b in &groups[i + 1..] {
            if let Some(p) = pair_resistance(a, b, cfg) {
                pairs.push(p);
            }
        }
    }
    let rs: Vec<f64> = pairs.iter().map(|p| p.r_ohm).collect();
    let r_pair = median(&rs);
    let r_pair_bracket = r_pair.map(|r| {
        (
            rs.iter().copied().fold(r, f64::min),
            rs.iter().copied().fold(r, f64::max),
        )
    });

    // V0 from the control: its two halves see one rotor speed, so the
    // solve returns E + V0 together. E is not separable, which is why the
    // control is the floor on V0 and not a measurement of it - a control
    // taken with the shaft barely turning is the useful one.
    let v0_measured = median(
        &holds
            .iter()
            .map(|f| f.drop_volts)
            .filter(|v| (-V0_MAX_VOLTS..V0_MAX_VOLTS).contains(v))
            .collect::<Vec<_>>(),
    );
    // A control taken while the shaft was already turning solves to a
    // slightly negative drop when the two halves disagree; clamp rather
    // than hand the slope route a voltage that adds energy.
    let v0 = v0_measured.unwrap_or(V0_DEFAULT_VOLTS).max(0.0);
    if v0_measured.is_none() {
        warnings.push(format!(
            "no from-a-hold control produced a usable drop; the slope route uses the \
             pre-registered V0 = {V0_DEFAULT_VOLTS} V"
        ));
    }

    // Pass three: L with the run's R and V0 in hand.
    let r = r_pair.unwrap_or(0.0);
    let ls: Vec<f64> = rest
        .iter()
        .filter_map(|f| l_ripple(f, r, v0, cfg))
        .collect();
    let l_ripple_h = median(&ls).unwrap_or(0.0);
    let l_offs: Vec<f64> = rest.iter().filter_map(|f| l_off(f, r)).collect();
    let envs: Vec<f64> = taus.iter().map(|t| r * t * 1e-6).collect();
    let l_env_h = r * tau * 1e-6;
    // A duty whose ON windows are too short for a slope contributes no L
    // at all rather than a zero - it must not drag the duty spread.
    let l_by_duty: Vec<(f64, f64)> = duties
        .iter()
        .zip(&by_duty)
        .filter_map(|(d, g)| {
            let v: Vec<f64> = g.iter().filter_map(|f| l_ripple(f, r, v0, cfg)).collect();
            median(&v).map(|l| (*d, l))
        })
        .collect();
    for (d, g) in duties.iter().zip(&by_duty) {
        if g.iter().all(|f| !f.slope_ok) {
            warnings.push(format!(
                "the {:.0}% rung's {:.0}-sample ON windows are mostly amplifier edge, so it \
                 feeds the envelope and the pairs R but no slope",
                d * 100.0,
                g[0].window_samples
            ));
        }
    }
    let ripple_spread = worst_spread(
        &by_duty
            .iter()
            .map(|g| g.iter().filter_map(|f| l_ripple(f, r, v0, cfg)).collect())
            .collect::<Vec<Vec<f64>>>(),
    );
    let env_spread = worst_spread(
        &by_duty
            .iter()
            .map(|g| g.iter().map(|f| f.tau_us).filter(|t| *t > 0.0).collect())
            .collect::<Vec<Vec<f64>>>(),
    );

    // The second R: the same asymptote with V0 taken off explicitly. It
    // shares the envelope extrapolation with the pair route but not the
    // differencing, so agreement says the extrapolation is stable.
    let r_asym = median(
        &rest
            .iter()
            .filter(|f| f.asymptote_a > 0.0)
            .map(|f| (f.duty * f.v_rail - v0) / f.asymptote_a)
            .filter(|r| *r > 0.0)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);

    // Per-capture gates fold into the run's: one bad capture must not hide
    // inside a median. `over` scopes the fold - rig health (cadence, step
    // index, bias) is every capture's business, while the fit gates cover
    // only the from-rest captures the pooled numbers come from. A control
    // that failed is a warning below, not a verdict on L.
    let fold = |name: &'static str, over: &[&CaptureFit]| -> Gate {
        let bad: Vec<&&CaptureFit> = over
            .iter()
            .filter(|f| f.gates.iter().any(|g| g.name == name && !g.pass))
            .collect();
        match bad.first() {
            None => gate(name, true, format!("{} captures", over.len())),
            Some(f) => gate(
                name,
                false,
                format!(
                    "{} of {} failed, first: {}",
                    bad.len(),
                    over.len(),
                    f.gates
                        .iter()
                        .find(|g| g.name == name)
                        .map(|g| g.detail.clone())
                        .unwrap_or_default()
                ),
            ),
        }
    };
    let every: Vec<&CaptureFit> = fits.iter().collect();
    for f in holds.iter().filter(|f| !f.ok) {
        warnings.push(format!(
            "from-a-hold control at {:.0}% failed its own gates: {}",
            f.duty * 100.0,
            f.gates
                .iter()
                .filter(|g| !g.pass)
                .map(|g| format!("{} ({})", g.name, g.detail))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    let duty_spread = match (
        l_by_duty.iter().map(|(_, l)| *l).fold(f64::MAX, f64::min),
        l_by_duty.iter().map(|(_, l)| *l).fold(0.0f64, f64::max),
    ) {
        (lo, hi) if l_ripple_h > 0.0 && lo.is_finite() && hi > 0.0 => (hi - lo) / l_ripple_h,
        _ => f64::INFINITY,
    };
    let r_gap = match (r_pair, r_asym > 0.0) {
        (Some(r), true) => (r - r_asym).abs() / r_asym,
        _ => f64::INFINITY,
    };
    let mut gates = vec![
        gate(
            "captures",
            rest.len() >= 2,
            format!("{} from rest, {hold} from a hold", rest.len()),
        ),
        fold("cadence", &every),
        fold("step-index", &every),
        fold("pre-bias", &every),
        fold("windows", &rest),
        gate(
            "r-consistency",
            r_gap <= cfg.r_agree_tol,
            format!(
                "pairs {} vs asymptote {r_asym:.2} ohm ({:.0}% apart)",
                r_pair.map_or("-".into(), |r| format!("{r:.2}")),
                r_gap * 100.0
            ),
        ),
        gate(
            "l-ripple-spread",
            ripple_spread <= cfg.l_agree_tol,
            format!(
                "{:.0}% across the repeats of one duty",
                ripple_spread * 100.0
            ),
        ),
        gate(
            "l-env-spread",
            env_spread <= cfg.l_agree_tol,
            format!("{:.0}% across the repeats of one duty", env_spread * 100.0),
        ),
        gate(
            "l-duty",
            duty_spread <= cfg.l_agree_tol,
            format!(
                "{:.0}% across the {} step duties that carry a slope",
                duty_spread * 100.0,
                l_by_duty.len()
            ),
        ),
        match r_pair {
            Some(r) => gate(
                "r-pairs",
                true,
                format!("{} pairs, median {r:.2} ohm", rs.len()),
            ),
            None => gate(
                "r-pairs",
                false,
                format!(
                    "no from-rest pair spans {:.0}% of full scale",
                    cfg.pair_min_duty_span * 100.0
                ),
            ),
        },
    ];
    if let Some(lo) = median(&l_offs)
        && l_ripple_h > 0.0
        && (lo - l_ripple_h).abs() / l_ripple_h > cfg.l_agree_tol
    {
        warnings.push(format!(
            "the two incremental-L routes disagree: ON slope {:.3} mH against brake decay \
             {:.3} mH. The slope route is the one that uses the rail reading, so a soft \
             supply moves it and not the other",
            l_ripple_h * 1e3,
            lo * 1e3
        ));
    }
    if hold > 0 {
        warnings.push(format!(
            "{hold} from-a-hold captures recorded as the E-nonzero control; they are excluded \
             from the pooled L, tau and R and supply V0 only"
        ));
    }
    gates.retain(|g| !(g.name == "pre-bias" && rest.is_empty()));
    let ok = gates.iter().all(|g| g.pass);
    Some(InductanceResult {
        l_ripple_h,
        l_ripple_bracket: bracket(&ls, l_ripple_h),
        l_off_h: median(&l_offs),
        l_env_h,
        l_env_bracket: bracket(&envs, l_env_h),
        tau_us: tau,
        tau_bracket: bracket(&taus, tau),
        tau_off_us: median(
            &rest
                .iter()
                .map(|f| f.tau_off_us)
                .filter(|t| *t > 0.0)
                .collect::<Vec<_>>(),
        )
        .unwrap_or(0.0),
        r_pair_ohm: r_pair,
        r_pair_bracket,
        pairs,
        r_asym_ohm: r_asym,
        v0_volts: v0,
        v0_measured: v0_measured.is_some(),
        l_by_duty,
        ripple_spread,
        env_spread,
        bias_counts: median(&fits.iter().map(|f| f.bias_counts).collect::<Vec<_>>()).unwrap_or(0.0),
        settle_us: median(&fits.iter().map(|f| f.settle_us).collect::<Vec<_>>()).unwrap_or(0.0),
        window_samples: median(&rest.iter().map(|f| f.window_samples).collect::<Vec<_>>())
            .unwrap_or(0.0),
        cadence_samples: median(&fits.iter().map(|f| f.cadence_samples).collect::<Vec<_>>())
            .unwrap_or(0.0),
        rest_captures: rest.len(),
        hold_captures: hold,
        gates,
        ok,
        warnings,
    })
}

// --- experiment -------------------------------------------------------------

/// One planned burst.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Arm {
    /// Duty held over the pre-step half; 0 is the braked shaft at rest.
    pub pre_q15: i16,
    pub step_q15: i16,
}

#[derive(Clone, Debug)]
pub struct Cfg {
    /// Step duties, percent of full scale, low first so the settled
    /// current is known before the bigger rungs are armed.
    pub step_pct: Vec<u8>,
    pub repeats: u32,
    pub both_signs: bool,
    /// The E-nonzero control: (pre, step) percent of full scale.
    pub hold_pct: Option<(u8, u8)>,
    /// Settled winding current the plan stays under, amps. A rung whose
    /// settled current would exceed it is dropped, measured from the rungs
    /// already captured.
    pub i_max_a: f64,
    pub centre_margin: u16,
    pub seek_duty_q15: i16,
    pub seek_poll_ms: u32,
    pub seek_cap_polls: u32,
    /// Pause between reaching the launch state and arming: the pre-step
    /// drive has to reach steady speed, and a braked shaft has to stop.
    pub settle_ms: u32,
    pub rest_ms: u32,
    pub fit: FitCfg,
}

impl Default for Cfg {
    fn default() -> Self {
        Self {
            step_pct: vec![20, 30, 40],
            repeats: 5,
            both_signs: true,
            hold_pct: Some((10, 26)),
            i_max_a: 0.4,
            centre_margin: 300,
            seek_duty_q15: 8520,
            seek_poll_ms: 25,
            seek_cap_polls: 400,
            settle_ms: 200,
            rest_ms: 150,
            fit: FitCfg::default(),
        }
    }
}

fn pct_q15(pct: u8) -> i16 {
    (pct as i32 * Q15 as i32 / 100) as i16
}

/// The full arm plan: every step duty x sign x repeat from rest, low duty
/// first, then the from-a-hold control.
pub fn plan(cfg: &Cfg) -> Vec<Arm> {
    let signs: &[i16] = if cfg.both_signs { &[1, -1] } else { &[1] };
    let mut out = Vec::new();
    for pct in &cfg.step_pct {
        for s in signs {
            for _ in 0..cfg.repeats {
                out.push(Arm {
                    pre_q15: 0,
                    step_q15: s * pct_q15(*pct),
                });
            }
        }
    }
    if let Some((pre, step)) = cfg.hold_pct {
        for s in signs {
            for _ in 0..cfg.repeats {
                out.push(Arm {
                    pre_q15: s * pct_q15(pre),
                    step_q15: s * pct_q15(step),
                });
            }
        }
    }
    out
}

enum Phase {
    ModeWrite,
    TelOff,
    SeekTorqueOn,
    SeekRead,
    SeekEval,
    SeekWait,
    ArmPre,
    ArmSettle,
    ArmBurst,
    ArmRelax,
    ArmPause,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Inductance {
    cfg: Cfg,
    sc: Scales,
    band: (u16, u16),
    phase: Phase,
    plan: Vec<Arm>,
    at: usize,
    polls: u32,
    caps: Vec<Capture>,
    warnings: Vec<String>,
}

impl Inductance {
    pub fn new(cfg: Cfg, params: &RigParams, sc: Scales) -> Self {
        let mid = params.pos_guard.map_or(POT_MID, |(lo, hi)| (lo + hi) / 2);
        let band = (
            mid.saturating_sub(cfg.centre_margin),
            mid.saturating_add(cfg.centre_margin),
        );
        let plan = plan(&cfg);
        Self {
            cfg,
            sc,
            band,
            phase: Phase::ModeWrite,
            plan,
            at: 0,
            polls: 0,
            caps: Vec::new(),
            warnings: Vec::new(),
        }
    }

    pub fn captures(&self) -> &[Capture] {
        &self.caps
    }

    pub fn warnings(&self) -> &[String] {
        &self.warnings
    }

    pub fn fit(&self) -> Option<InductanceResult> {
        let mut r = fit_captures(&self.caps, &self.sc, &self.cfg.fit)?;
        r.warnings.splice(0..0, self.warnings.iter().cloned());
        Some(r)
    }

    fn arm(&self) -> Option<Arm> {
        self.plan.get(self.at).copied()
    }

    /// Drop every remaining rung whose settled current the last capture
    /// says would clear the envelope. The settled current is proportional
    /// to duty from rest, so one measured rung sizes all of them.
    fn prune(&mut self, from: &CaptureFit) {
        if from.duty <= 0.0 || from.asymptote_a <= 0.0 {
            return;
        }
        let per_duty = from.asymptote_a / from.duty;
        let cap_q15 = (self.cfg.i_max_a / per_duty * Q15) as i32;
        let before = self.plan.len();
        let at = self.at;
        let mut k = 0;
        self.plan.retain(|a| {
            let keep = k <= at || (a.step_q15 as i32).abs() <= cap_q15;
            k += 1;
            keep
        });
        if self.plan.len() < before {
            self.warnings.push(format!(
                "{} rungs dropped: at {:.0}% the settled current was {:.3} A, so anything over \
                 {:.0}% would clear the {:.2} A envelope",
                before - self.plan.len(),
                from.duty * 100.0,
                from.asymptote_a,
                cap_q15 as f64 / Q15 * 100.0,
                self.cfg.i_max_a
            ));
        }
    }
}

impl Experiment for Inductance {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TelOff;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            // The servo refuses an arm while TEL is still streaming; a
            // previous experiment can have left the producer armed.
            Phase::TelOff => {
                self.phase = Phase::SeekTorqueOn;
                Cmd::Write {
                    reg: control::TEL_COUNT,
                    value: 0,
                }
            }
            Phase::SeekTorqueOn => {
                self.phase = Phase::SeekRead;
                self.polls = 0;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                let pos = obs.map(|o| o.pos).unwrap_or(POT_MID);
                self.polls += 1;
                if (self.band.0..=self.band.1).contains(&pos) {
                    self.phase = Phase::ArmPre;
                    return Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: 0,
                    };
                }
                if self.polls >= self.cfg.seek_cap_polls {
                    self.warnings
                        .push(format!("seek stuck at pos {pos}; run cut short"));
                    self.phase = Phase::FinishDuty;
                    return Cmd::Pause { ms: 0 };
                }
                self.phase = Phase::SeekWait;
                let duty = if pos < self.band.0 {
                    self.cfg.seek_duty_q15
                } else {
                    -self.cfg.seek_duty_q15
                };
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: duty as i32,
                }
            }
            Phase::SeekWait => {
                self.phase = Phase::SeekRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
            Phase::ArmPre => match self.arm() {
                Some(a) => {
                    self.phase = Phase::ArmSettle;
                    Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: a.pre_q15 as i32,
                    }
                }
                None => {
                    self.phase = Phase::FinishDuty;
                    Cmd::Pause { ms: 0 }
                }
            },
            Phase::ArmSettle => {
                self.phase = Phase::ArmBurst;
                Cmd::Pause {
                    ms: self.cfg.settle_ms,
                }
            }
            Phase::ArmBurst => {
                let a = self.arm().unwrap_or(Arm {
                    pre_q15: 0,
                    step_q15: 0,
                });
                self.phase = Phase::ArmRelax;
                Cmd::Burst {
                    duty_q15: a.step_q15,
                    pre_q15: a.pre_q15,
                }
            }
            Phase::ArmRelax => {
                self.phase = Phase::ArmPause;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::ArmPause => {
                self.at += 1;
                self.phase = if self.at < self.plan.len() {
                    Phase::SeekTorqueOn
                } else {
                    Phase::FinishDuty
                };
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::FinishTorque => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }

    fn push_burst(&mut self, cap: &Capture) {
        if let Some(f) = fit_capture(cap, &self.sc, &self.cfg.fit)
            && f.from_rest
        {
            self.prune(&f);
        }
        self.caps.push(cap.clone());
    }
}

/// One-line geometry summary per capture for the run log. No L: the slope
/// route needs the run's R and V0, which one capture does not have.
pub fn render_capture(f: &CaptureFit) -> String {
    let first = f.windows.first();
    let mid = f.windows.get(f.windows.len() / 2);
    let mut s = String::new();
    let _ = write!(
        s,
        "duty {:>3.0}%{} {:>2} windows of {:>4.1} | didt {:>6.0} -> {:>6.0} A/s  \
         ripple {:.4} A  asym {:.4} A  tau {:>5.1} us  tau_off {:>5.1} us  \
         settle {:.2} us  cad {:.2}  drop {:.3} V",
        f.duty * 100.0,
        if f.from_rest { " rest" } else { " hold" },
        f.windows.len(),
        f.window_samples,
        first.map_or(0.0, |w| w.didt_a_per_s),
        mid.map_or(0.0, |w| w.didt_a_per_s),
        mid.map_or(0.0, |w| w.ripple_a()),
        f.asymptote_a,
        f.tau_us,
        f.tau_off_us,
        f.settle_us,
        f.cadence_samples,
        f.drop_volts
    );
    s
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, SynthBurst, pump};
    use super::super::{Guarded, RigParams};
    use super::*;
    use crate::burst::from_csv;
    use crate::units::SenseParams;

    /// Bodged board D as fitted: 60 mohm shunt, G 15, 15k/10k rail tap.
    const BOARD_D: SenseParams = SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 15_000,
        vmotor_div_top: 6_800,
        vmotor_div_bot: 3_300,
        vdd_mv: 3_300,
        tick_hz: 20_100,
    };
    const VBUS_DIV: (u16, u16) = (15_000, 10_000);

    fn scales() -> Scales {
        Scales::from_sense(&BOARD_D, VBUS_DIV.0, VBUS_DIV.1).unwrap()
    }

    /// What the three from-rest fixtures' pair route and their from-a-hold
    /// control give, so a single-capture assertion reads against the same
    /// numbers the run would hand it.
    const FIXTURE_R_OHM: f64 = 7.0;
    const FIXTURE_V0_VOLTS: f64 = 0.44;

    const FIXTURES: [(&str, &str); 5] = [
        (
            "rest-to-20",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/rest-to-20.csv"
            )),
        ),
        (
            "rest-to-26",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/rest-to-26.csv"
            )),
        ),
        (
            "rest-to-40",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/rest-to-40.csv"
            )),
        ),
        (
            "10-to-26",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/10-to-26.csv"
            )),
        ),
        (
            "26-to-10",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/26-to-10.csv"
            )),
        ),
    ];

    fn fixtures() -> Vec<(&'static str, Capture)> {
        FIXTURES
            .iter()
            .map(|(n, t)| (*n, from_csv(t).expect("fixture parses")))
            .collect()
    }

    #[test]
    fn bench_fixtures_segment_and_fit() {
        let sc = scales();
        let cfg = FitCfg::default();
        for (name, cap) in fixtures() {
            let f = fit_capture(&cap, &sc, &cfg).unwrap_or_else(|| panic!("{name} fit"));
            println!(
                "{name:12} {} bias {:.1}{}",
                render_capture(&f),
                f.bias_counts,
                if f.bias_from_trace { " (trace)" } else { "" }
            );
            assert!(
                (f.cadence_samples - 45.8).abs() <= 1.0,
                "{name} cadence {}",
                f.cadence_samples
            );
            assert_eq!(f.step_index, 485, "{name} step_index");
            // 26-to-10 steps DOWN to a 4.6-sample ON window. One aperture
            // sample in, three are left and the lag fit needs four: this
            // capture carries no windows at all, let alone an envelope.
            if name == "26-to-10" {
                assert!(f.windows.is_empty(), "{} windows", f.windows.len());
                assert!(!f.slope_ok);
                assert!(!f.gates.iter().find(|g| g.name == "windows").unwrap().pass);
                continue;
            }
            assert!(
                (80.0..250.0).contains(&f.tau_us),
                "{name} tau {} us",
                f.tau_us
            );
            // A 20% ON window is nine samples and the amplifier edge owns
            // four of them: the slope route declines and says so, while
            // the envelope numbers above stand.
            if !f.slope_ok {
                assert!(f.notes.iter().any(|n| n.contains("no slope route")));
                assert!(
                    f.window_samples < 10.0,
                    "{name} declined a {}-sample window",
                    f.window_samples
                );
                continue;
            }
            // smoke bounds, not a number pin: the fixtures are one motor on
            // one soft rail. R and V0 are the run's, so a single capture is
            // read against the pairs R the three from-rest fixtures give.
            let l = l_ripple(&f, FIXTURE_R_OHM, FIXTURE_V0_VOLTS, &cfg).expect("L");
            println!("{name:12} L_ripple {:.3} mH", l * 1e3);
            assert!((0.3e-3..1.2e-3).contains(&l), "{name} L {} mH", l * 1e3);
        }
    }

    #[test]
    fn from_rest_fixtures_pair_into_an_r() {
        let sc = scales();
        let cfg = FitCfg::default();
        let caps: Vec<Capture> = fixtures()
            .into_iter()
            .filter(|(n, _)| n.starts_with("rest"))
            .map(|(_, c)| c)
            .collect();
        let r = fit_captures(&caps, &sc, &cfg).expect("run fit");
        for p in &r.pairs {
            println!(
                "pair {:.0}%/{:.0}% R {:.2} ohm (di {:.4} A, ddv {:.3} V)",
                p.duty_lo * 100.0,
                p.duty_hi * 100.0,
                p.r_ohm,
                p.di_a,
                p.ddv_volts
            );
        }
        println!(
            "pooled L_ripple {:.3} mH {:?} L_env {:.3} mH tau {:.0} us R pair {:?} asym {:.2}",
            r.l_ripple_h * 1e3,
            (r.l_ripple_bracket.0 * 1e3, r.l_ripple_bracket.1 * 1e3),
            r.l_env_h * 1e3,
            r.tau_us,
            r.r_pair_ohm,
            r.r_asym_ohm
        );
        // the 20-vs-26% span is under pair_min_duty_span, so it never
        // reaches the median that made it read 30 ohm
        assert!(
            r.pairs
                .iter()
                .all(|p| p.duty_hi - p.duty_lo >= cfg.pair_min_duty_span),
            "a short-span pair leaked into the fit"
        );
        assert!(r.r_pair_ohm.is_some(), "no usable pair");
        assert_eq!(r.rest_captures, 3);
    }

    #[test]
    fn a_from_a_hold_fixture_is_flagged_not_pooled() {
        let sc = scales();
        let cfg = FitCfg::default();
        let caps: Vec<Capture> = fixtures().into_iter().map(|(_, c)| c).collect();
        let r = fit_captures(&caps, &sc, &cfg).expect("run fit");
        assert_eq!(r.hold_captures, 2, "10-to-26 and 26-to-10 are the control");
        assert!(r.warnings.iter().any(|w| w.contains("E-nonzero")));
        let f = fit_capture(&caps[3], &sc, &cfg).unwrap();
        assert!(!f.from_rest);
        assert!(f.notes.iter().any(|n| n.contains("drop_volts")));
    }

    /// The regenerating capture: after a step DOWN from a 26% hold the
    /// rotor's back-EMF exceeds the new drive and the ON window reads BELOW
    /// the bias. Segmentation has to find those windows anyway.
    #[test]
    fn a_regenerating_window_is_still_segmented() {
        let (_, cap) = fixtures()
            .into_iter()
            .find(|(n, _)| *n == "26-to-10")
            .unwrap();
        let seg = segment(&cap.samples, &FitCfg::default()).expect("segment");
        let v = &cap.samples;
        let below = seg
            .windows
            .iter()
            .filter(|w| w.start > cap.meta.step_index as usize)
            .filter(|w| (v[w.start] as f64) < cap.meta.bias as f64)
            .count();
        assert!(below >= 3, "no sub-bias windows found: {:?}", seg.windows);
        assert!((seg.cadence_samples - 46.0).abs() <= 1.0);
    }

    #[test]
    fn recovers_a_synthetic_r_and_l() {
        let sc = scales();
        let plant = SynthBurst {
            r: 4.0,
            l: 0.6e-3,
            settle_us: 0.9,
            noise_counts: 2.0,
            ..SynthBurst::board_d()
        };
        let cap = plant.capture(pct_q15(26), 0);
        let cfg = FitCfg::default();
        let f = fit_capture(&cap, &sc, &cfg).expect("fit");
        println!("{}", render_capture(&f));
        let tau = plant.l / plant.r * 1e6;
        // V0 is zero in the synthetic plant - the bridge drop is folded
        // into its R - so the slope route is read with V0 = 0 here.
        let l = l_ripple(&f, plant.r, 0.0, &cfg).expect("L");
        assert!(
            (l - plant.l).abs() / plant.l < 0.05,
            "L {} mH vs {} mH",
            l * 1e3,
            plant.l * 1e3
        );
        assert!(
            (f.tau_us - tau).abs() / tau < 0.05,
            "tau {} us vs {tau} us",
            f.tau_us
        );
        // the profiled edge time constant has to land near the planted one
        assert!(
            (0.5..2.0).contains(&f.settle_us),
            "settle {} us",
            f.settle_us
        );
        assert!(f.slope_ok, "windows {} samples", f.window_samples);
        assert!(f.ok, "gates {:?}", f.gates);
    }

    #[test]
    fn synthetic_pairs_recover_the_planted_r() {
        let sc = scales();
        let plant = SynthBurst {
            r: 4.0,
            l: 0.6e-3,
            settle_us: 0.9,
            noise_counts: 2.0,
            ..SynthBurst::board_d()
        };
        let mut caps: Vec<Capture> = [20u8, 30, 40]
            .iter()
            .map(|p| plant.capture(pct_q15(*p), 0))
            .collect();
        // the control the run plans: without it V0 falls back to the
        // pre-registered 0.4 V, which this plant does not have
        caps.push(plant.capture(pct_q15(26), pct_q15(10)));
        let r = fit_captures(&caps, &sc, &FitCfg::default()).expect("fit");
        let pair = r.r_pair_ohm.expect("pair R");
        println!(
            "pair R {pair:.3} asymptote {:.3} L_ripple {:.4} mH L_env {:.4} mH",
            r.r_asym_ohm,
            r.l_ripple_h * 1e3,
            r.l_env_h * 1e3
        );
        assert!((pair - plant.r).abs() / plant.r < 0.10, "R {pair}");
        assert!(r.ok, "gates {:?}", r.gates);
        // L is duty independent by construction: the ON-phase voltage is
        // the rail whatever the duty
        for (d, l) in &r.l_by_duty {
            assert!(
                (l - plant.l).abs() / plant.l < 0.08,
                "duty {d}: L {} mH",
                l * 1e3
            );
        }
    }

    #[test]
    fn a_cadence_that_drifts_fails_its_gate() {
        let sc = scales();
        let plant = SynthBurst {
            // 1000 HCLK per half period instead of 1200: 38.5 samples
            arr: 1000,
            ..SynthBurst::board_d()
        };
        let mut cap = plant.capture(pct_q15(26), 0);
        // publish the real ARR the servo runs, so the nominal cadence the
        // gate compares against is the 1200 one
        cap.meta.pwm_arr = 1200;
        let f = fit_capture(&cap, &sc, &FitCfg::default()).expect("fit");
        assert!(
            !f.gates.iter().find(|g| g.name == "cadence").unwrap().pass,
            "cadence {}",
            f.cadence_samples
        );
        assert!(!f.ok);
    }

    #[test]
    fn plan_orders_low_duty_first_and_ends_with_the_control() {
        let cfg = Cfg {
            repeats: 2,
            ..Cfg::default()
        };
        let p = plan(&cfg);
        assert_eq!(p.len(), 3 * 2 * 2 + 2 * 2);
        assert!(p[..12].iter().all(|a| a.pre_q15 == 0));
        assert!(p[12..].iter().all(|a| a.pre_q15 != 0));
        assert!(p[0].step_q15.unsigned_abs() < p[11].step_q15.unsigned_abs());
        assert!(p.iter().any(|a| a.step_q15 < 0) && p.iter().any(|a| a.step_q15 > 0));
    }

    #[test]
    fn drives_the_rig_safely_end_to_end() {
        let params = RigParams::default();
        let cfg = Cfg {
            repeats: 1,
            step_pct: vec![20, 40],
            ..Cfg::default()
        };
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        let mut exp = Guarded::new(Inductance::new(cfg, &params, scales()), params);
        let log = pump(&mut exp, &mut servo, 200_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        assert!(exp.abort().is_none(), "abort {:?}", exp.abort());
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .unwrap();
        let first_drive = log
            .iter()
            .position(|l| {
                l.starts_with("burst") || (l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            })
            .unwrap();
        assert!(torque_on < first_drive, "torque on before any drive");
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
        let exp = exp.into_inner();
        // 2 duties x 2 signs plus the from-a-hold control x 2 signs
        assert_eq!(exp.captures().len(), 6);
        assert!(exp.fit().is_some(), "the run fits");
    }

    #[test]
    fn a_rung_over_the_current_envelope_is_dropped() {
        let params = RigParams::default();
        let cfg = Cfg {
            repeats: 1,
            step_pct: vec![20, 40],
            both_signs: false,
            hold_pct: None,
            // 20% already draws ~0.13 A on the fake plant, so 40% cannot fit
            i_max_a: 0.15,
            ..Cfg::default()
        };
        let mut servo = FakeServo::new(3.37);
        servo.dynamic = true;
        let mut exp = Guarded::new(Inductance::new(cfg, &params, scales()), params);
        pump(&mut exp, &mut servo, 200_000);
        let exp = exp.into_inner();
        assert_eq!(exp.captures().len(), 1, "the 40% rung must be dropped");
        assert!(
            exp.warnings().iter().any(|w| w.contains("envelope")),
            "{:?}",
            exp.warnings()
        );
    }
}
