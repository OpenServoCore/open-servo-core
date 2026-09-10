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
//! (~1 us, so 2 to 4 samples); the last full sample is the ripple peak and
//! the sample straddling the OFF edge is a partial aperture.
//!
//! Three numbers come out, in falling order of trust:
//!
//!   - L from the ON-window slope. During ON the winding sees the rail
//!     whatever the duty is, so L = (V - E - i R) / (di/dt) is duty
//!     independent - the run's own consistency check. From rest E = 0 and
//!     i R is a ~10% correction taken from the same capture's settled
//!     level, so L rides on the slope and the rail alone.
//!   - tau from the envelope of the per-window levels, one exponential.
//!   - R from PAIRS of from-rest captures at two step duties:
//!     delta(asymptote) / delta(D x V), where the fixed bridge and brush
//!     drop cancels in the difference. R x tau is the cross-check on L.
//!
//! The envelope's asymptote is an EXTRAPOLATION: the post-step half of a
//! capture is ~520 us, under four tau, so tau and every R route that leans
//! on the asymptote carry that error while the slope route does not. The
//! gates say so rather than hiding it.

use core::fmt::Write as _;

use super::rl::{Gate, Scales};
use super::{Cmd, Experiment, RigParams};
use crate::burst::{Capture, SAMPLE_US, nominal_cadence};
use crate::fitmath::{median, quantile, stddev, theil_sen};
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
    /// Leading samples skipped by the probe fit the settling is measured
    /// against; also the largest skip the estimator may choose.
    pub probe_skip: usize,
    /// A leading sample is settled once its residual is under this, counts.
    pub settle_resid_counts: f64,
    /// Post-step windows the slope route averages L over.
    pub l_windows: usize,
    /// Largest relative gap between the slope and R x tau routes for L,
    /// and between the per-duty L medians.
    pub l_agree_tol: f64,
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
            probe_skip: 4,
            settle_resid_counts: 1.5,
            l_windows: 3,
            l_agree_tol: 0.35,
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

/// One ON window reduced to a slope and a level.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindowFit {
    pub start: usize,
    pub end: usize,
    /// Signed so a reverse step reads positive: di/dt inside the window.
    pub didt_a_per_s: f64,
    /// Period-mean current, amps, bias subtracted. The ripple is
    /// piecewise linear, so the ramp fit at the ON window's centre is the
    /// mean over the whole period - no OFF-phase samples needed.
    pub level_a: f64,
    /// L this window alone gives, henries.
    pub l_henries: f64,
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
    /// Leading samples per window the slope fit dropped.
    pub skip: usize,
    /// Amplifier edge settling the leading residuals imply, microseconds.
    pub settle_us: f64,
    pub windows: Vec<WindowFit>,
    /// Step duty as a fraction of full scale, magnitude.
    pub duty: f64,
    pub v_rail: f64,
    /// The capture started from a braked shaft at zero duty (E = 0).
    pub from_rest: bool,
    pub tau_us: f64,
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
    /// L from the first `l_windows` ON-window slopes, henries.
    pub l_slope_h: f64,
    pub l_bracket: (f64, f64),
    /// r_capture x tau: the cross-check route.
    pub l_tau_h: f64,
    pub step_index: u16,
    pub gates: Vec<Gate>,
    pub ok: bool,
    pub notes: Vec<String>,
}

fn gate(name: &'static str, pass: bool, detail: String) -> Gate {
    Gate { name, pass, detail }
}

/// Fit A - B * exp(-t / tau) by profiling tau: A and B are closed form at
/// each tau, so the search is one dimensional. Log grid then golden
/// section. None when the basis is degenerate.
fn fit_exponential(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    const TAU_LO: f64 = 10e-6;
    const TAU_HI: f64 = 3000e-6;
    const GRID: usize = 300;
    if pts.len() < 3 || !pts.iter().all(|(t, y)| t.is_finite() && y.is_finite()) {
        return None;
    }
    let solve = |tau: f64| -> Option<(f64, f64, f64)> {
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
        Some((rss, a, -c))
    };
    let mut best = (f64::INFINITY, TAU_LO);
    for k in 0..GRID {
        let tau = TAU_LO * (TAU_HI / TAU_LO).powf(k as f64 / (GRID - 1) as f64);
        if let Some((rss, _, _)) = solve(tau)
            && rss < best.0
        {
            best = (rss, tau);
        }
    }
    if !best.0.is_finite() {
        return None;
    }
    let phi = (5f64.sqrt() - 1.0) / 2.0;
    let (mut lo, mut hi) = (best.1 / 1.3, best.1 * 1.3);
    for _ in 0..60 {
        let x1 = hi - phi * (hi - lo);
        let x2 = lo + phi * (hi - lo);
        let r1 = solve(x1).map_or(f64::INFINITY, |r| r.0);
        let r2 = solve(x2).map_or(f64::INFINITY, |r| r.0);
        if r1 < r2 { hi = x2 } else { lo = x1 }
    }
    let tau = (lo + hi) / 2.0;
    let (_, a, _) = solve(tau)?;
    (tau.is_finite() && a.is_finite()).then_some((tau, a))
}

/// Everything one capture gives, from the samples alone.
pub fn fit_capture(cap: &Capture, sc: &Scales, cfg: &FitCfg) -> Option<CaptureFit> {
    let seg = segment(&cap.samples, cfg)?;
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

    // Settling: fit each window past `probe_skip`, then read the leading
    // samples' residuals. They decay geometrically at the amplifier's edge
    // time constant; the skip is the first sample back under the noise.
    // The low-side shunt sees drive current the same way whichever way the
    // bridge is pointed, so a reverse step reads above the bias exactly
    // like a forward one; no direction sign enters here. A reading BELOW
    // the bias is real regeneration, not a reversed drive.
    let amps = |k: usize| (v[k] - bias) * sc.amps_per_count;
    let t = |k: usize| k as f64 * cfg.sample_us * 1e-6;
    let line = |w: &Window, skip: usize| {
        let pts: Vec<(f64, f64)> = (w.start + skip..=w.end).map(|k| (t(k), amps(k))).collect();
        (pts.len() >= cfg.min_fit_points)
            .then(|| theil_sen(&pts))
            .flatten()
    };
    let mut resid = vec![Vec::new(); cfg.probe_skip + 1];
    for w in &keep {
        if let Some(f) = line(w, cfg.probe_skip) {
            for (j, r) in resid.iter_mut().enumerate() {
                if w.start + j <= w.end {
                    r.push(amps(w.start + j) - (f.a + f.b * t(w.start + j)));
                }
            }
        }
    }
    let counts = |a: f64| a / (sc.amps_per_count.max(f64::MIN_POSITIVE));
    let med_resid: Vec<f64> = resid.iter().map(|r| median(r).unwrap_or(0.0)).collect();
    let limit = cfg.settle_resid_counts.max(2.0 * seg.noise_counts);
    let skip = med_resid
        .iter()
        .position(|r| counts(*r).abs() <= limit)
        .unwrap_or(cfg.probe_skip);
    // r_j = r_0 * a^j while the amplifier is still catching up
    let settle_us = match (counts(med_resid[0]).abs(), counts(med_resid[1]).abs()) {
        (r0, r1) if r1 > 0.0 && r0 > r1 => cfg.sample_us / (r0 / r1).ln(),
        _ => 0.0,
    };

    let mut windows = Vec::new();
    for w in &keep {
        let Some(f) = line(w, skip) else { continue };
        let centre = t(w.start) + (t(w.end) - t(w.start)) / 2.0;
        windows.push(WindowFit {
            start: w.start,
            end: w.end,
            didt_a_per_s: f.b,
            level_a: f.a + f.b * centre,
            l_henries: 0.0,
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
                line(w, skip).map(|f| {
                    let c = t(w.start) + (t(w.end) - t(w.start)) / 2.0;
                    f.a + f.b * c
                })
            })
            .collect();
        median(&lv).unwrap_or(0.0)
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
    // The slope route needs a window that outlives the amplifier: the skip
    // plus a fittable run. At 10% duty an ON window is under five samples
    // and the edge eats three of them, so there is no honest slope and the
    // route declines rather than fitting the settling.
    let slope_min = (cfg.probe_skip + cfg.min_fit_points) as f64;
    let long_enough = full >= slope_min;
    if !long_enough {
        notes.push(format!(
            "ON windows are {full:.0} samples at {:.0}% duty, under the {slope_min:.0} the \
             settling skip plus a fit needs: no slope route",
            duty * 100.0
        ));
    }
    for w in windows.iter_mut() {
        if w.didt_a_per_s > 0.0 && long_enough {
            w.l_henries = (v_rail - drop - w.level_a * r_capture) / w.didt_a_per_s;
        }
    }
    let firsts: Vec<f64> = windows
        .iter()
        .take(cfg.l_windows)
        .map(|w| w.l_henries)
        .filter(|l| *l > 0.0)
        .collect();
    let l_slope = median(&firsts).unwrap_or(0.0);
    let l_bracket = (
        firsts.iter().copied().fold(l_slope, f64::min),
        firsts.iter().copied().fold(l_slope, f64::max),
    );
    let l_tau = r_capture * tau_s;

    let nominal = nominal_cadence(cap.meta.pwm_arr);
    let l_gap = if l_slope > 0.0 {
        (l_slope - l_tau).abs() / l_slope
    } else {
        f64::INFINITY
    };
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
            windows.len() >= cfg.min_post_windows && long_enough,
            format!(
                "{} post-step, {} usable, {full:.0} samples each (need {slope_min:.0})",
                post.len(),
                windows.len()
            ),
        ),
        gate(
            "l-routes",
            l_gap <= cfg.l_agree_tol,
            format!(
                "slope {:.3} mH vs r x tau {:.3} mH ({:.0}% apart)",
                l_slope * 1e3,
                l_tau * 1e3,
                l_gap * 100.0
            ),
        ),
    ];
    let ok = gates.iter().all(|g| g.pass);
    if !from_rest {
        notes.push(
            "pre-step drive: the rotor is spinning, so E is not zero and both L routes read \
             the winding plus an unknown back-EMF term - control only"
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
        skip,
        settle_us,
        windows,
        duty,
        v_rail,
        from_rest,
        tau_us: tau_s * 1e6,
        asymptote_a: asym,
        r_capture_ohm: r_capture,
        pre_level_a: pre_level,
        drop_volts: drop,
        l_slope_h: l_slope,
        l_bracket,
        l_tau_h: l_tau,
        step_index: cap.meta.step_index,
        gates,
        ok,
        notes,
    })
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

#[derive(Clone, Debug)]
pub struct InductanceResult {
    /// Pooled L from the from-rest captures' slope route, henries.
    pub l_henries: f64,
    pub l_bracket: (f64, f64),
    pub tau_us: f64,
    pub tau_bracket: (f64, f64),
    /// R from from-rest pairs, ohms. None when no pair cleared the duty
    /// span the asymptote extrapolation needs.
    pub r_pair_ohm: Option<f64>,
    pub r_pair_bracket: Option<(f64, f64)>,
    pub pairs: Vec<PairR>,
    /// Median of the per-capture two-half route. From rest it carries the
    /// fixed bridge drop folded into R and reads high.
    pub r_capture_ohm: f64,
    /// L per step duty - the duty-independence check, (duty, L).
    pub l_by_duty: Vec<(f64, f64)>,
    pub bias_counts: f64,
    pub skip: usize,
    pub settle_us: f64,
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

/// Every number the run reports, from recorded captures alone - so the
/// offline refit and the live fit cannot diverge.
pub fn fit_captures(caps: &[Capture], sc: &Scales, cfg: &FitCfg) -> Option<InductanceResult> {
    let fits: Vec<CaptureFit> = caps
        .iter()
        .filter_map(|c| fit_capture(c, sc, cfg))
        .collect();
    if fits.is_empty() {
        return None;
    }
    let rest: Vec<&CaptureFit> = fits.iter().filter(|f| f.from_rest).collect();
    let hold = fits.len() - rest.len();
    let mut warnings = Vec::new();

    let ls: Vec<f64> = rest
        .iter()
        .map(|f| f.l_slope_h)
        .filter(|l| *l > 0.0)
        .collect();
    let taus: Vec<f64> = rest.iter().map(|f| f.tau_us).filter(|t| *t > 0.0).collect();
    let l = median(&ls).unwrap_or(0.0);
    let tau = median(&taus).unwrap_or(0.0);
    let r_capture = median(
        &rest
            .iter()
            .map(|f| f.r_capture_ohm)
            .filter(|r| *r > 0.0)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);

    // group by step duty so the pair route differences GROUP medians, not
    // single noisy asymptotes
    let mut duties: Vec<f64> = rest.iter().map(|f| f.duty).collect();
    duties.sort_by(f64::total_cmp);
    duties.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let mut groups: Vec<CaptureFit> = Vec::new();
    let mut l_by_duty = Vec::new();
    for d in &duties {
        let same: Vec<&&CaptureFit> = rest.iter().filter(|f| (f.duty - d).abs() < 1e-6).collect();
        let mut rep = (*same[0]).clone();
        rep.asymptote_a =
            median(&same.iter().map(|f| f.asymptote_a).collect::<Vec<_>>()).unwrap_or(0.0);
        rep.v_rail = median(&same.iter().map(|f| f.v_rail).collect::<Vec<_>>()).unwrap_or(0.0);
        let ld = median(
            &same
                .iter()
                .map(|f| f.l_slope_h)
                .filter(|l| *l > 0.0)
                .collect::<Vec<_>>(),
        )
        .unwrap_or(0.0);
        l_by_duty.push((*d, ld));
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
    for f in fits.iter().filter(|f| !f.from_rest) {
        if !f.ok {
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
    }
    let duty_spread = match (
        l_by_duty.iter().map(|(_, l)| *l).fold(f64::MAX, f64::min),
        l_by_duty.iter().map(|(_, l)| *l).fold(0.0f64, f64::max),
    ) {
        (lo, hi) if l > 0.0 && lo.is_finite() && hi > 0.0 => (hi - lo) / l,
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
        fold("l-routes", &rest),
        gate(
            "l-duty",
            duty_spread <= cfg.l_agree_tol,
            format!(
                "{:.0}% across {} step duties",
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
    if let Some(r) = r_pair
        && r_capture > 0.0
        && (r - r_capture).abs() / r_capture > cfg.l_agree_tol
    {
        warnings.push(format!(
            "pair R {r:.2} ohm and per-capture R {r_capture:.2} ohm disagree: the per-capture \
             route folds the fixed bridge drop into R, and both lean on an asymptote \
             extrapolated from under four tau of post-step capture"
        ));
    }
    if hold > 0 {
        warnings.push(format!(
            "{hold} from-a-hold captures recorded as the E-nonzero control; they are excluded \
             from the pooled L, tau and R"
        ));
    }
    gates.retain(|g| !(g.name == "pre-bias" && rest.is_empty()));
    let ok = gates.iter().all(|g| g.pass);
    Some(InductanceResult {
        l_henries: l,
        l_bracket: bracket(&ls, l),
        tau_us: tau,
        tau_bracket: bracket(&taus, tau),
        r_pair_ohm: r_pair,
        r_pair_bracket,
        pairs,
        r_capture_ohm: r_capture,
        l_by_duty,
        bias_counts: median(&fits.iter().map(|f| f.bias_counts).collect::<Vec<_>>()).unwrap_or(0.0),
        skip: median(&fits.iter().map(|f| f.skip as f64).collect::<Vec<_>>()).unwrap_or(0.0)
            as usize,
        settle_us: median(&fits.iter().map(|f| f.settle_us).collect::<Vec<_>>()).unwrap_or(0.0),
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

/// One-line summary per capture for the run log.
pub fn render_capture(f: &CaptureFit) -> String {
    let mut s = String::new();
    let _ = write!(
        s,
        "duty {:.0}%{} L {:.3} mH tau {:.0} us r1 {:.2} ohm  cad {:.2} skip {} \
         ({:.1} us settling) drop {:.2} V windows {}",
        f.duty * 100.0,
        if f.from_rest { " rest" } else { " hold" },
        f.l_slope_h * 1e3,
        f.tau_us,
        f.r_capture_ohm,
        f.cadence_samples,
        f.skip,
        f.settle_us,
        f.drop_volts,
        f.windows.len()
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
            assert!(
                (80.0..250.0).contains(&f.tau_us),
                "{name} tau {} us",
                f.tau_us
            );
            // 26-to-10 steps DOWN to a 4.6-sample ON window: the slope
            // route declines rather than fitting the amplifier edge, and
            // says so on the windows gate
            if name == "26-to-10" {
                assert_eq!(f.l_slope_h, 0.0, "a 10% window has no honest slope");
                assert!(!f.gates.iter().find(|g| g.name == "windows").unwrap().pass);
                continue;
            }
            // smoke bounds, not a number pin: the fixtures are one motor on
            // one soft rail
            assert!(
                (0.3e-3..1.2e-3).contains(&f.l_slope_h),
                "{name} L {} mH",
                f.l_slope_h * 1e3
            );
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
            "pooled L {:.3} mH {:?} tau {:.0} us R pair {:?} single {:.2}",
            r.l_henries * 1e3,
            (r.l_bracket.0 * 1e3, r.l_bracket.1 * 1e3),
            r.tau_us,
            r.r_pair_ohm,
            r.r_capture_ohm
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
        assert!(f.notes.iter().any(|n| n.contains("back-EMF")));
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
        let f = fit_capture(&cap, &sc, &FitCfg::default()).expect("fit");
        println!("{}", render_capture(&f));
        let tau = plant.l / plant.r * 1e6;
        assert!(
            (f.l_slope_h - plant.l).abs() / plant.l < 0.05,
            "L {} mH vs {} mH",
            f.l_slope_h * 1e3,
            plant.l * 1e3
        );
        assert!(
            (f.tau_us - tau).abs() / tau < 0.05,
            "tau {} us vs {tau} us",
            f.tau_us
        );
        // the settling estimate has to land near the planted 0.9 us, and
        // the skip has to clear it
        assert!(
            (0.5..2.0).contains(&f.settle_us),
            "settle {} us",
            f.settle_us
        );
        assert!((2..=4).contains(&f.skip), "skip {}", f.skip);
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
        let caps: Vec<Capture> = [20u8, 30, 40]
            .iter()
            .map(|p| plant.capture(pct_q15(*p), 0))
            .collect();
        let r = fit_captures(&caps, &sc, &FitCfg::default()).expect("fit");
        let pair = r.r_pair_ohm.expect("pair R");
        println!("pair R {pair:.3} per-capture {:.3}", r.r_capture_ohm);
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
