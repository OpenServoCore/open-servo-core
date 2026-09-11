//! Winding voltage from the burst's voltage channels: the volt-seconds the
//! pre-arm rail reading can only assume. On a soft supply the rail sags
//! while the drive is on, so D x V_prearm over-states what the winding saw
//! and every R built on it reads high; a channel sampled inside the burst
//! sees the sag.
//!
//! A channel gets one sample per frame, 2 to 4 conversions apart, so a 9 to
//! 18 conversion ON window holds only a handful. The unit of work is
//! therefore the PERIOD: per ON window the mean of the settled samples, per
//! OFF gap likewise, and the crest-to-crest mean V = D x V_on + (1 - D) x
//! V_off with D the commanded duty. Integrating the samples straight
//! across the period would carry a quantisation of one sample per edge -
//! +/-20% of the ON volt-seconds per period at frame_len 3 - and the burst
//! launches tick-synchronous, so that error does not dither away across
//! repeats; in a regression it lands on the regressor and biases L high. A
//! constant ON-time loss (dead time, propagation) is a fixed offset in V
//! and cancels in the pairs difference and in the regression's intercept,
//! which is what D costs; the rectangle integral is kept as the check on
//! it ([`VoltDiag::duty_ratio`]).
//!
//! Three routes, strongest first: both taps give the bias-free terminal
//! difference; one driven tap gives that terminal's absolute volts through
//! the divider bias, the idle side estimated; the rail alone gives the
//! winding only after an estimated drop on both sides of the bridge.

use super::inductance::{CaptureFit, PairR};
use super::rl::Scales;
use crate::burst::{CHAN_VBUS, CHAN_VMOTOR_A, CHAN_VMOTOR_B, Capture};
use crate::fitmath::{linear_ls, lstsq, median};

/// DRV8212P RDS(on) per FET, typical at 27 C (SLVSFZ0A sec 7.5 gives no
/// min or max). Only ever the estimate for a drop no channel sees.
pub const RDS_ON_OHM: f64 = 0.140;

/// A voltage sample this long after a drive edge is settled: 1.5 us is 4.5
/// tau of the 2.2k x 150 pF terminal tap, the rest covers the error in
/// where the edge is.
pub const TAP_CLEAR_US: f64 = 2.0;

/// A voltage sample this close before a drive edge may already be past it:
/// the island's RC and the amplifier's lag hold the shunt's first visible
/// move up to ~1 us behind the edge, and the grid inherits that lateness.
pub const EDGE_GUARD_US: f64 = 2.0;

/// A chopping terminal whose OFF phase sits below this is conducting
/// through a body diode, not a low-side channel, volts.
pub const BODY_DIODE_VOLTS: f64 = -0.3;

/// An ON-window voltage that moves more than this fraction of its level
/// across the window is not flat.
pub const ON_SAG_FRAC: f64 = 0.01;

/// The regression's time term enters only when it cuts the residual this
/// hard: the F statistic of the nested fit.
pub const EMF_TERM_F_MIN: f64 = 10.0;

/// A tap at full scale is a rail beyond its range, not a reading: the
/// unbiased 15k/10k rail tap clips above 8.25 V.
const ADC_FULL_SCALE: u16 = 4095;

/// Where a capture's winding voltage comes from, weakest first.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Route {
    /// The rail less an estimated drop on both sides of the bridge.
    Rail,
    /// The rail less an estimated high-side drop, the idle terminal
    /// measured.
    RailIdleTap,
    /// The driven terminal measured, the idle side estimated.
    DrivenTap,
    /// Both terminals: the bias-free difference.
    Terminals,
}

impl Route {
    pub fn as_str(self) -> &'static str {
        match self {
            Route::Rail => "rail less an estimated bridge drop (weakest)",
            Route::RailIdleTap => "rail plus the idle terminal",
            Route::DrivenTap => "driven terminal",
            Route::Terminals => "both terminals",
        }
    }
}

/// One interval between successive ON windows of the post-step envelope.
/// The currents are charge-balance levels, the winding's mean over each
/// ON window, so the interval runs ON-centre to ON-centre.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Period {
    /// Opening window after the step, ms.
    pub t_ms: f64,
    /// Winding current over the two ON windows, amps.
    pub i0: f64,
    pub i1: f64,
    /// Mean winding voltage over the interval, volts.
    pub v: f64,
}

#[derive(Clone, Debug)]
pub struct CaptureVolts {
    /// None: the pre-arm rail stands in for a measurement.
    pub route: Option<Route>,
    pub duty: f64,
    /// The charge-balance envelope's asymptote, amps.
    pub asymptote_a: f64,
    pub periods: Vec<Period>,
    /// Mean winding voltage at the envelope asymptote, volts.
    pub v_settled: f64,
    /// (window current, the high side's ON level as sampled): the rail, or
    /// the driven terminal one high-side drop under it.
    pub hi_on: Vec<(f64, f64)>,
    pub hi_is_rail: bool,
    /// The chopping terminal's settled OFF samples, volts.
    pub off: Vec<f64>,
    /// (us since the edge, V - that window's mean) for the high side's ON
    /// samples.
    pub on_dev: Vec<(f64, f64)>,
    /// Settled ON span a window holds, us.
    pub on_span_us: f64,
    /// Rectangle-rule mean of the chopping terminal over whole periods,
    /// as a fraction of the ON-to-OFF swing: the duty it actually saw.
    pub duty_eff: Option<f64>,
    pub notes: Vec<String>,
}

/// The chopping leg follows the step's sign: positive drives OUT1, which is
/// MOT_A and the vmotor_a tap.
fn legs(step_q15: i16) -> (u8, u8) {
    if step_q15 >= 0 {
        (CHAN_VMOTOR_A, CHAN_VMOTOR_B)
    } else {
        (CHAN_VMOTOR_B, CHAN_VMOTOR_A)
    }
}

/// One slot's (raw index, code) sampled inside [t0, t1], microseconds.
fn slot_samples(
    cap: &Capture,
    f: &CaptureFit,
    slot: usize,
    (t0, t1): (f64, f64),
) -> Vec<(usize, f64)> {
    let fl = f.frame_len;
    let k0 = (t0 / f.raw_us / fl as f64).floor().max(0.0) as usize;
    let k1 = (t1 / f.raw_us / fl as f64).ceil().max(0.0) as usize;
    (k0..=k1)
        .map(|k| k * fl + slot)
        .filter(|&r| r < cap.samples.len())
        .filter(|&r| (t0..=t1).contains(&(r as f64 * f.raw_us)))
        .map(|r| (r, cap.samples[r] as f64))
        .collect()
}

fn slot_mean(
    cap: &Capture,
    f: &CaptureFit,
    slot: usize,
    span: (f64, f64),
    conv: impl Fn(f64) -> f64,
) -> Option<f64> {
    let v = slot_samples(cap, f, slot, span);
    (!v.is_empty()).then(|| v.iter().map(|(_, c)| conv(*c)).sum::<f64>() / v.len() as f64)
}

/// Each window's drive edges, microseconds: the rising edge from the
/// capture's PWM grid, the falling edge the commanded ON width after it.
/// The falling side of the shunt is no ruler - the island's repayment and
/// the amplifier's lag hold it up past the OFF edge.
fn edges(f: &CaptureFit) -> Option<Vec<(f64, f64)>> {
    let (a, b) = f.rise_line?;
    Some(
        f.cb.iter()
            .map(|w| {
                let rise = a + ((w.rise as f64 - 0.5 - a) / b).round() * b;
                (rise * f.sample_us, (rise + f.duty * b) * f.sample_us)
            })
            .collect(),
    )
}

/// Everything one capture's voltage channels say, or the pre-arm stand-in
/// when `measured` is false or no channel gives the winding. None when the
/// capture holds fewer than two windows.
pub fn capture_volts(
    cap: &Capture,
    f: &CaptureFit,
    sc: &Scales,
    measured: bool,
) -> Option<CaptureVolts> {
    if f.cb.len() < 2 {
        return None;
    }
    let fl = f.frame_len;
    let mut notes = Vec::new();
    let (hi_bit, lo_bit) = legs(cap.meta.step_q15);
    let mut usable = |bit: u8| -> Option<usize> {
        let s = cap.slot(bit)?;
        if cap.stream(s).contains(&ADC_FULL_SCALE) {
            notes.push(format!("slot {s} clips at full scale; not used"));
            return None;
        }
        Some(s)
    };
    let hi_tap = usable(hi_bit);
    let lo_tap = usable(lo_bit);
    let rail = usable(CHAN_VBUS);

    // Before a from-rest step the bridge coasts, so a tap sits exactly on
    // the divider bias: that reading, in the same capture and aperture,
    // beats the boot-time one.
    let taps: Vec<usize> = [hi_tap, lo_tap].into_iter().flatten().collect();
    let pre: Vec<f64> = taps
        .iter()
        .flat_map(|&s| (0..f.step).map(move |k| cap.samples[k * fl + s] as f64))
        .collect();
    let vb = match (f.from_rest, median(&pre)) {
        (true, Some(v)) => Some(v),
        _ => (cap.meta.vmotor_bias > 0).then_some(cap.meta.vmotor_bias as f64),
    };
    let route = match (measured, hi_tap, lo_tap, rail, vb) {
        (false, ..) => None,
        (_, Some(_), Some(_), ..) => Some(Route::Terminals),
        (_, Some(_), None, _, Some(_)) => Some(Route::DrivenTap),
        (_, None, Some(_), Some(_), Some(_)) => Some(Route::RailIdleTap),
        (_, _, _, Some(_), _) => Some(Route::Rail),
        _ => None,
    };
    if measured && route.is_none() && cap.meta.chans != 0 {
        notes.push(format!(
            "chans {} gives no winding voltage on a {} step; the pre-arm rail stands in",
            cap.meta.chans,
            if cap.meta.step_q15 >= 0 {
                "forward"
            } else {
                "reverse"
            }
        ));
    }
    let vbv = vb.unwrap_or(0.0);
    let tap_v = |c: f64| sc.terminal_volts(c, vbv);
    let rail_v = |c: f64| c * sc.v_rail_per_count;
    let hi_is_tap = matches!(route, Some(Route::Terminals | Route::DrivenTap));
    let lo_is_tap = matches!(route, Some(Route::Terminals | Route::RailIdleTap));
    let edge = edges(f)?;
    // Settled ON span of window k and settled OFF span after it.
    let on_span = |k: usize| (edge[k].0 + TAP_CLEAR_US, edge[k].1 - EDGE_GUARD_US);
    let off_span = |k: usize| (edge[k].1 + TAP_CLEAR_US, edge[k + 1].0 - EDGE_GUARD_US);
    // The high side as sampled, before any drop correction.
    let hi_raw_on = |k: usize| -> Option<f64> {
        match route {
            Some(Route::Terminals | Route::DrivenTap) => {
                slot_mean(cap, f, hi_tap?, on_span(k), tap_v)
            }
            Some(Route::Rail | Route::RailIdleTap) => slot_mean(cap, f, rail?, on_span(k), rail_v),
            None => Some(f.v_rail),
        }
    };
    let v_on = |k: usize| -> Option<f64> {
        let i = f.cb[k].level_a;
        let hi = hi_raw_on(k)? - if hi_is_tap { 0.0 } else { i * RDS_ON_OHM };
        let lo = match lo_is_tap {
            true => slot_mean(cap, f, lo_tap?, on_span(k), tap_v)?,
            false => i * (RDS_ON_OHM + sc.shunt_ohm),
        };
        Some(hi - lo)
    };
    let v_off = |k: usize| -> Option<f64> {
        let i = (f.cb[k].level_a + f.cb[k + 1].level_a) / 2.0;
        let hi = match hi_is_tap {
            true => slot_mean(cap, f, hi_tap?, off_span(k), tap_v)?,
            false => -i * RDS_ON_OHM,
        };
        let lo = match lo_is_tap {
            true => slot_mean(cap, f, lo_tap?, off_span(k), tap_v)?,
            false => i * RDS_ON_OHM,
        };
        Some(hi - lo)
    };

    let d = f.duty;
    let mut periods = Vec::new();
    for k in 0..f.cb.len() - 1 {
        let (w0, w1) = (&f.cb[k], &f.cb[k + 1]);
        let gap = (w1.rise - w0.rise) as f64;
        if (gap - f.period_samples).abs() > f.period_samples / 2.0 {
            continue;
        }
        let (Some(a), Some(b), Some(off)) = (v_on(k), v_on(k + 1), v_off(k)) else {
            continue;
        };
        periods.push(Period {
            t_ms: (w0.rise as f64 - f.step as f64) * f.sample_us * 1e-3,
            i0: w0.level_a,
            i1: w1.level_a,
            v: d * (a + b) / 2.0 + (1.0 - d) * off,
        });
    }
    let v_settled = {
        let pts: Vec<(f64, f64)> = periods.iter().map(|p| ((p.i0 + p.i1) / 2.0, p.v)).collect();
        match linear_ls(&pts) {
            Some(l) if pts.len() >= 3 => l.a + l.b * f.asymptote_cb_a,
            _ => median(&periods.iter().map(|p| p.v).collect::<Vec<_>>()).unwrap_or(0.0),
        }
    };

    // Diagnostics read the channels as sampled; the pre-arm stand-in has
    // none to read.
    let mut hi_on = Vec::new();
    let mut off = Vec::new();
    let mut on_dev = Vec::new();
    let mut spans = Vec::new();
    let abs_ok = !hi_is_tap || vb.is_some();
    let hi_slot = if hi_is_tap { hi_tap } else { rail };
    let hi_conv = |c: f64| if hi_is_tap { tap_v(c) } else { rail_v(c) };
    if let (Some(_), Some(slot), true) = (route, hi_slot, abs_ok) {
        for (k, w) in f.cb.iter().enumerate() {
            let i = w.level_a;
            let span = on_span(k);
            let s = slot_samples(cap, f, slot, span);
            if s.is_empty() {
                continue;
            }
            let vs: Vec<f64> = s.iter().map(|(_, c)| hi_conv(*c)).collect();
            let m = vs.iter().sum::<f64>() / vs.len() as f64;
            hi_on.push((i, m));
            spans.push(span.1 - span.0);
            for ((r, _), v) in s.iter().zip(&vs) {
                on_dev.push((*r as f64 * f.raw_us - edge[k].0, v - m));
            }
            if hi_is_tap && k + 1 < f.cb.len() {
                off.extend(
                    slot_samples(cap, f, slot, off_span(k))
                        .iter()
                        .map(|(_, c)| tap_v(*c)),
                );
            }
        }
    }
    // The rectangle rule over whole periods, trough to trough.
    let duty_eff = match (hi_is_tap && abs_ok, hi_tap) {
        (true, Some(slot)) => {
            let period_us = f.period_samples * f.sample_us;
            let end_us = cap.samples.len() as f64 * f.raw_us;
            let all: Vec<f64> = edge
                .iter()
                .filter(|(r, fall)| (r + fall + period_us) / 2.0 < end_us)
                .flat_map(|(r, fall)| {
                    let c = (r + fall) / 2.0;
                    slot_samples(
                        cap,
                        f,
                        slot,
                        (c - period_us / 2.0, c + period_us / 2.0 - f.raw_us),
                    )
                })
                .map(|(_, c)| tap_v(c))
                .collect();
            let on = median(&hi_on.iter().map(|p| p.1).collect::<Vec<_>>());
            match (all.is_empty(), on, median(&off)) {
                (false, Some(on), Some(lo)) if on > lo => {
                    Some((all.iter().sum::<f64>() / all.len() as f64 - lo) / (on - lo))
                }
                _ => None,
            }
        }
        _ => None,
    };
    Some(CaptureVolts {
        route,
        duty: d,
        asymptote_a: f.asymptote_cb_a,
        periods,
        v_settled,
        hi_on,
        hi_is_rail: !hi_is_tap,
        off,
        on_dev,
        on_span_us: median(&spans).unwrap_or(0.0),
        duty_eff,
        notes,
    })
}

// --- run --------------------------------------------------------------------

/// R, L and tau from the per-period regression
/// `i1 = a i0 + b (V - c - e t)`, the exact discretisation of
/// `L di/dt = V - R i - c(t)` over one crest-to-crest interval:
/// a = exp(-T / tau), b = (1 - a) / R.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PeriodFit {
    pub r_ohm: f64,
    pub l_h: f64,
    pub tau_us: f64,
    /// Fixed series drop plus back-EMF at the step, volts.
    pub c_volts: f64,
    /// Back-EMF growth, volts per ms - present only when the residuals
    /// demanded the time term.
    pub emf_v_per_ms: Option<f64>,

    pub rms_a: f64,
    pub n: usize,
}

/// Pool the intervals of every capture into one regression. The step
/// duties differ between captures, which is what separates the voltage
/// coefficient from the intercept: inside one from-rest capture V is
/// nearly constant and only (V - c) / R and tau are identifiable.
pub fn period_fit(rows: &[Period], period_s: f64) -> Option<PeriodFit> {
    let y: Vec<f64> = rows.iter().map(|p| p.i1).collect();
    let x3: Vec<Vec<f64>> = rows.iter().map(|p| vec![p.i0, p.v, 1.0]).collect();
    let x4: Vec<Vec<f64>> = rows.iter().map(|p| vec![p.i0, p.v, 1.0, p.t_ms]).collect();
    let (b3, rss3) = lstsq(&x3, &y)?;
    let four = lstsq(&x4, &y).filter(|(_, rss4)| {
        let dof = rows.len().saturating_sub(4) as f64;
        *rss4 > 0.0 && dof > 0.0 && (rss3 - rss4) / (rss4 / dof) > EMF_TERM_F_MIN
    });
    let (coef, e, rss) = match four {
        Some((b4, rss4)) => {
            let e = Some(b4[3]);
            (b4, e, rss4)
        }
        None => (b3, None, rss3),
    };
    let (a, b, g) = (coef[0], coef[1], coef[2]);
    if !(a > 0.0 && a < 1.0 && b > 0.0) {
        return None;
    }
    let tau = -period_s / a.ln();
    let r = (1.0 - a) / b;
    Some(PeriodFit {
        r_ohm: r,
        l_h: r * tau,
        tau_us: tau * 1e6,
        c_volts: -g / b,
        emf_v_per_ms: e.map(|e| -e / b),
        rms_a: (rss / rows.len() as f64).sqrt(),
        n: rows.len(),
    })
}

/// What the voltage channels showed across the run.
#[derive(Clone, Debug, Default)]
pub struct VoltDiag {
    /// In-burst source impedance at the pulse timescale: the high side's
    /// ON level against the winding current, less R_hs when the high side
    /// is a terminal. Ohms.
    pub z_src_ohm: Option<f64>,
    pub z_from_rail: bool,
    /// Where that line meets zero current, volts.
    pub rail_open_v: Option<f64>,
    /// The chopping terminal through the OFF phase, volts.
    pub off_median_v: Option<f64>,
    pub off_min_v: Option<f64>,
    /// The OFF phase sits below [`BODY_DIODE_VOLTS`].
    pub body_diode: bool,
    /// Change of the high side across one settled ON span, volts, and the
    /// span it is measured over.
    pub on_sag_v: Option<f64>,
    pub on_span_us: f64,
    pub on_flat: Option<bool>,
    /// Median effective duty over commanded: 1 is volt-seconds exactly as
    /// commanded.
    pub duty_ratio: Option<f64>,
}

/// One voltage source's two R routes and what they imply.
#[derive(Clone, Debug)]
pub struct VoltRun {
    /// Weakest route among the pooled captures; None when the pre-arm rail
    /// stood in.
    pub route: Option<Route>,
    pub captures: usize,
    /// Settled asymptotes against the settled mean voltage, from-rest pairs.
    pub r_pair_ohm: Option<f64>,
    pub r_pair_bracket: Option<(f64, f64)>,
    pub pairs: Vec<PairR>,
    pub reg: Option<PeriodFit>,
    pub diag: Option<VoltDiag>,
    pub notes: Vec<String>,
}

/// Pool one voltage source over the from-rest captures. `span_min` is the
/// smallest duty span a pair may carry, as for the pre-arm pairs.
pub fn volt_run(caps: &[(&CaptureFit, CaptureVolts)], span_min: f64) -> VoltRun {
    let route = caps.iter().filter_map(|(_, v)| v.route).min();
    let mut duties: Vec<f64> = caps.iter().map(|(f, _)| f.duty).collect();
    duties.sort_by(f64::total_cmp);
    duties.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let groups: Vec<(f64, f64, f64)> = duties
        .iter()
        .filter_map(|d| {
            let g: Vec<&CaptureVolts> = caps
                .iter()
                .filter(|(f, _)| (f.duty - d).abs() < 1e-6)
                .map(|(_, v)| v)
                .collect();
            let asym = median(&g.iter().map(|v| v.asymptote_a).collect::<Vec<_>>())?;
            let vs = median(&g.iter().map(|v| v.v_settled).collect::<Vec<_>>())?;
            Some((*d, asym, vs))
        })
        .collect();
    let mut pairs = Vec::new();
    for (k, lo) in groups.iter().enumerate() {
        for hi in &groups[k + 1..] {
            let (di, dv) = (hi.1 - lo.1, hi.2 - lo.2);
            if hi.0 - lo.0 >= span_min && di > 0.0 && dv > 0.0 {
                pairs.push(PairR {
                    r_ohm: dv / di,
                    duty_lo: lo.0,
                    duty_hi: hi.0,
                    di_a: di,
                    ddv_volts: dv,
                });
            }
        }
    }
    let rs: Vec<f64> = pairs.iter().map(|p| p.r_ohm).collect();
    let r_pair = median(&rs);
    let rows: Vec<Period> = caps
        .iter()
        .flat_map(|(_, v)| v.periods.iter().copied())
        .collect();
    let period_s = median(
        &caps
            .iter()
            .map(|(f, _)| f.period_samples * f.sample_us * 1e-6)
            .collect::<Vec<_>>(),
    )
    .unwrap_or(0.0);
    let mut notes: Vec<String> = caps.iter().flat_map(|(_, v)| v.notes.clone()).collect();
    notes.sort();
    notes.dedup();
    VoltRun {
        route,
        captures: caps.len(),
        r_pair_ohm: r_pair,
        r_pair_bracket: r_pair.map(|r| {
            (
                rs.iter().copied().fold(r, f64::min),
                rs.iter().copied().fold(r, f64::max),
            )
        }),
        pairs,
        reg: period_fit(&rows, period_s),
        diag: route.map(|_| diagnose(caps)),
        notes,
    }
}

fn diagnose(caps: &[(&CaptureFit, CaptureVolts)]) -> VoltDiag {
    let hi_on: Vec<(f64, f64)> = caps.iter().flat_map(|(_, v)| v.hi_on.clone()).collect();
    let from_rail = caps.iter().all(|(_, v)| v.hi_is_rail);
    let line = linear_ls(&hi_on);
    let off: Vec<f64> = caps.iter().flat_map(|(_, v)| v.off.clone()).collect();
    let off_median = median(&off);
    let dev: Vec<(f64, f64)> = caps.iter().flat_map(|(_, v)| v.on_dev.clone()).collect();
    let span = median(&caps.iter().map(|(_, v)| v.on_span_us).collect::<Vec<_>>()).unwrap_or(0.0);
    let sag = linear_ls(&dev).map(|l| l.b * span);
    let level = median(&hi_on.iter().map(|p| p.1).collect::<Vec<_>>());
    VoltDiag {
        z_src_ohm: line.map(|l| -l.b - if from_rail { 0.0 } else { RDS_ON_OHM }),
        z_from_rail: from_rail,
        rail_open_v: line.map(|l| l.a),
        off_median_v: off_median,
        off_min_v: off.iter().copied().reduce(f64::min),
        body_diode: off_median.is_some_and(|v| v < BODY_DIODE_VOLTS),
        on_sag_v: sag,
        on_span_us: span,
        on_flat: sag
            .zip(level)
            .map(|(s, l)| s.abs() <= ON_SAG_FRAC * l.abs()),
        duty_ratio: median(
            &caps
                .iter()
                .filter_map(|(_, v)| v.duty_eff.map(|d| d / v.duty))
                .collect::<Vec<_>>(),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::super::inductance::{FitCfg, InductanceResult, fit_captures};
    use super::super::testkit::SynthBurst;
    use super::*;
    use crate::burst::Chans;
    use crate::units::SenseParams;

    fn scales() -> Scales {
        let s = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        Scales::from_sense(&s, 15_000, 10_000).unwrap()
    }

    /// A 4 ohm, 0.6 mH winding with a 0.2 V brush drop behind board D's
    /// bridge, read through the bench's 2 us amplifier lag.
    fn winding() -> SynthBurst {
        SynthBurst {
            r: 4.0,
            l: 0.6e-3,
            v0: 0.2,
            settle_us: 2.0,
            ..SynthBurst::board_d().with_bridge()
        }
    }

    /// 2S behind a stiff source, with a 10 uF island inside the shunt's
    /// ground so the ON window under-reads and charge balance has to hold.
    fn stiff() -> SynthBurst {
        SynthBurst {
            v_rail: 7.3,
            c_island: 10e-6,
            r_feed: 0.1,
            ..winding()
        }
    }

    /// USB: 1.3 ohm behind a 100 uF bulk that sags inside every ON window.
    fn soft() -> SynthBurst {
        SynthBurst {
            v_rail: 4.37,
            r_src: 1.3,
            ..winding()
        }
    }

    /// From rest at 20/30/40% both signs, plus the from-a-hold control.
    fn run(plant: &SynthBurst, chans: Chans) -> InductanceResult {
        let mut caps = Vec::new();
        for pct in [20i32, 30, 40] {
            for sgn in [1i32, -1] {
                let q = (sgn * pct * 32767 / 100) as i16;
                let p = SynthBurst {
                    chans: chans.for_step(q),
                    ..plant.clone()
                };
                caps.push(p.capture(q, 0));
            }
        }
        for sgn in [1i16, -1] {
            let p = SynthBurst {
                chans: chans.for_step(sgn),
                ..plant.clone()
            };
            caps.push(p.capture(sgn * 8520, sgn * 3276));
        }
        fit_captures(&caps, &scales(), &FitCfg::default()).expect("fit")
    }

    fn rel(got: f64, want: f64) -> f64 {
        (got - want) / want
    }

    #[test]
    fn measured_volt_seconds_recover_r_and_l_on_both_rails() {
        for (rail, plant) in [("stiff", stiff()), ("soft", soft())] {
            for chans in [
                Chans::Driven,
                Chans::Fixed(CHAN_VMOTOR_A | CHAN_VMOTOR_B),
                Chans::Fixed(CHAN_VBUS),
                Chans::Fixed(CHAN_VMOTOR_A | CHAN_VBUS),
            ] {
                let r = run(&plant, chans);
                let v = &r.volts;
                let pair = v.r_pair_ohm.expect("pairs");
                let g = v.reg.expect("regression");
                println!(
                    "{rail:5} {chans:?}: {:?} R pairs {pair:.3} regression {:.3} ohm, \
                     L_env {:.4} mH, share {:.3}",
                    v.route,
                    g.r_ohm,
                    g.l_h * 1e3,
                    r.shunt_on_share.unwrap_or(0.0)
                );
                assert!(v.route.is_some(), "{rail} {chans:?} not measured");
                assert!(
                    rel(pair, plant.r).abs() < 0.03,
                    "{rail} {chans:?} R pairs {pair}"
                );
                assert!(
                    rel(g.r_ohm, plant.r).abs() < 0.03,
                    "{rail} {chans:?} R {}",
                    g.r_ohm
                );
                assert!(
                    rel(g.l_h, plant.l).abs() < 0.05,
                    "{rail} {chans:?} L {}",
                    g.l_h
                );
                assert!(
                    r.promotable(),
                    "{rail} {chans:?} blocked by {:?}",
                    r.blocking()
                );
            }
        }
    }

    /// The same winding with no voltage channel: the pre-arm rail stands in
    /// for the volt-seconds, which the soft supply does not deliver.
    #[test]
    fn the_pre_arm_rail_over_reads_r_on_the_soft_supply_only() {
        let over = |plant: &SynthBurst| {
            let r = run(plant, Chans::Fixed(0));
            assert!(r.volts.route.is_none());
            println!(
                "pre-arm R pairs {:?} (ON-window, bridge included {:?}), supply {:?} ohm",
                r.volts.r_pair_ohm, r.r_pair_ohm, r.src_prearm_ohm
            );
            (rel(r.volts.r_pair_ohm.expect("pairs"), plant.r), r)
        };
        let (stiff_over, stiff_run) = over(&stiff());
        let (soft_over, soft_run) = over(&soft());
        assert!(stiff_over < 0.05, "stiff over-read {stiff_over}");
        assert!(soft_over > 0.05, "soft over-read {soft_over}");
        // unmeasured, the supply gate decides: stiff promotes, soft declines
        assert!(stiff_run.promotable(), "{:?}", stiff_run.blocking());
        assert_eq!(soft_run.blocking(), vec!["supply"]);
    }

    /// Charge balance holds while the island's voltage repeats period to
    /// period. On USB behind the 100 uF bulk the rail sags on the envelope's
    /// own timescale, the island follows it down, and its slow discharge
    /// reaches the winding without crossing the shunt: tau and L_env read
    /// long, R holds.
    #[test]
    fn a_sagging_island_stretches_the_envelope_but_not_r() {
        let plant = SynthBurst {
            c_island: 10e-6,
            r_feed: 0.1,
            ..soft()
        };
        let r = run(&plant, Chans::Driven);
        let g = r.volts.reg.expect("regression");
        let pair = r.volts.r_pair_ohm.expect("pairs");
        println!(
            "soft + island: R pairs {pair:.3} regression {:.3} L_env {:.4} mH",
            g.r_ohm,
            g.l_h * 1e3
        );
        assert!(rel(pair, plant.r).abs() < 0.03, "R pairs {pair}");
        assert!(rel(g.r_ohm, plant.r).abs() < 0.03, "R {}", g.r_ohm);
        assert!((0.03..0.12).contains(&rel(g.l_h, plant.l)), "L {}", g.l_h);
    }

    #[test]
    fn a_body_diode_off_phase_is_flagged_and_a_brake_is_not() {
        for chans in [Chans::Driven, Chans::Fixed(CHAN_VMOTOR_A | CHAN_VMOTOR_B)] {
            for diode in [false, true] {
                let plant = SynthBurst {
                    body_diode: diode,
                    ..stiff()
                };
                let r = run(&plant, chans);
                let d = r.volts.diag.expect("measured");
                println!(
                    "{chans:?} diode {diode}: OFF median {:?} min {:?}",
                    d.off_median_v, d.off_min_v
                );
                assert_eq!(d.body_diode, diode, "{chans:?} diode {diode}");
            }
        }
    }

    #[test]
    fn in_burst_diagnostics_read_the_planted_supply() {
        let r = run(&stiff(), Chans::Driven);
        let d = r.volts.diag.expect("measured");
        // the island's feed is the only resistance between rail and bridge
        let z = d.z_src_ohm.expect("source");
        assert!((0.05..0.2).contains(&z), "stiff source {z}");
        assert!((d.rail_open_v.unwrap() - 7.3).abs() < 0.05);
        assert_eq!(d.on_flat, Some(true));
        assert!((d.duty_ratio.unwrap() - 1.0).abs() < 0.03);
        let r = run(&soft(), Chans::Fixed(CHAN_VBUS));
        let d = r.volts.diag.expect("measured");
        assert!(d.z_from_rail);
        assert!(d.z_src_ohm.unwrap() > 0.3, "soft source {:?}", d.z_src_ohm);
    }
}
