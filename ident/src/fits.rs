//! Experiment samples -> physical motor parameters, count domain
//! throughout (vcounts, ccounts, c/s). Encoding to table Q formats is gain
//! synthesis' job. Started by the ladder (Ke + friction line); the inertia
//! step fits (direct-alpha and exponential-rise B estimators) live here too.

use crate::fitmath::{linear_ls, origin_ls, sliding_quadratic_deriv};

/// One steady-state ladder rung reduced to its operating point. Signs are
/// as driven: rev rungs carry negative omega, i and v.
#[derive(Copy, Clone, Debug)]
pub struct RungPoint {
    /// Steady speed, c/s (pos-slope LS over the steady segment).
    pub omega: f64,
    /// Steady window-current mean, ccounts.
    pub i: f64,
    /// Winding volts, vcounts: duty * vdiff / 32767 per window, averaged.
    pub v: f64,
}

#[derive(Copy, Clone, Debug)]
pub struct KeFit {
    /// Back-emf constant, vcounts per c/s.
    pub ke_vpc: f64,
    pub r2: f64,
    pub rms: f64,
    pub n: usize,
}

/// Origin LS of (v - R*i) vs omega across all rungs, both directions: the
/// bemf residual after the ohmic drop is Ke * omega through zero.
pub fn ke_fit(rungs: &[RungPoint], r_vpc: f64) -> Option<KeFit> {
    let xy: Vec<(f64, f64)> = rungs.iter().map(|p| (p.omega, p.v - r_vpc * p.i)).collect();
    let f = origin_ls(&xy)?;
    Some(KeFit {
        ke_vpc: f.slope,
        r2: f.r2,
        rms: f.rms,
        n: f.n,
    })
}

/// Kinetic friction line i = fc + fv * omega for one drive direction, rev
/// samples folded positive so fc/fv come out positive for a healthy train.
#[derive(Copy, Clone, Debug)]
pub struct FrictionFit {
    /// Coulomb intercept, ccounts.
    pub fc: f64,
    /// Viscous slope, ccounts per c/s.
    pub fv: f64,
    pub r2: f64,
    pub n: usize,
}

pub fn friction_line(rungs: &[RungPoint], dir: i8) -> Option<FrictionFit> {
    let fold = dir as f64;
    let xy: Vec<(f64, f64)> = rungs
        .iter()
        .filter(|p| p.omega * fold > 0.0)
        .map(|p| (p.omega * fold, p.i * fold))
        .collect();
    let f = linear_ls(&xy)?;
    Some(FrictionFit {
        fc: f.a,
        fv: f.b,
        r2: f.r2,
        n: f.n,
    })
}

/// One inertia step's captured series, uniform arrays over the step's own
/// timebase (TEL: tick index x tick_s; aggregate: window t_ms / 1000).
/// Signs are as driven; `mask` = true where the sample may enter a fit
/// (slip zone and invalid windows excluded by the experiment).
#[derive(Clone, Debug, Default)]
pub struct StepSeries {
    pub t: Vec<f64>,
    pub pos: Vec<f64>,
    /// Signed window current, ccounts.
    pub i: Vec<f64>,
    pub mask: Vec<bool>,
    pub duty_q15: f64,
}

/// Priors the inertia estimators consume - everything the earlier
/// experiments already fitted, plus the tick rate.
#[derive(Copy, Clone, Debug)]
pub struct InertiaPriors {
    pub r_vpc: f64,
    pub ke_vpc: f64,
    /// Coulomb friction, ccounts (direction-folded mean).
    pub fc: f64,
    /// Viscous friction, ccounts per c/s.
    pub fv: f64,
    pub tick_hz: f64,
}

impl InertiaPriors {
    /// b_i couples per MEDIUM tick (fusion.rs: one step per DECIM_MED = 10
    /// fast ticks), so B's rate anchor is tick_hz / 10.
    pub fn f_med(&self) -> f64 {
        self.tick_hz / 10.0
    }
}

#[derive(Copy, Clone, Debug)]
pub struct BDirect {
    /// B, (c/s per medium tick) per ccount.
    pub b: f64,
    pub r2: f64,
    pub n: usize,
}

/// Direct-alpha estimator: alpha = B * f_med * (i - fc*sgn - fv*omega),
/// so B is the origin-LS slope of d2y(pos) against
/// f_med * (i - fc*sgn(omega) - fv*omega) pooled over all steps. Samples
/// need |torque above friction| > `torque_floor` ccounts so the near-zero
/// cluster does not swamp the slope. Loose by construction: the double
/// derivative needs smoothing windows comparable to the transient itself
/// (fitmath window-pin test), so the exp-rise estimator usually wins.
pub fn b_direct_fit(
    steps: &[StepSeries],
    p: &InertiaPriors,
    half_window: usize,
    torque_floor: f64,
) -> Option<BDirect> {
    let mut xy = Vec::new();
    for s in steps {
        let d = sliding_quadratic_deriv(&s.t, &s.pos, half_window);
        for k in 0..s.t.len() {
            let (Some(omega), Some(alpha)) = (d.dy[k], d.d2y[k]) else {
                continue;
            };
            if !s.mask[k] {
                continue;
            }
            let torque = s.i[k] - p.fc * omega.signum() - p.fv * omega;
            if torque.abs() < torque_floor {
                continue;
            }
            xy.push((p.f_med() * torque, alpha));
        }
    }
    let f = origin_ls(&xy)?;
    (f.slope > 0.0).then_some(BDirect {
        b: f.slope,
        r2: f.r2,
        n: f.n,
    })
}

#[derive(Clone, Debug)]
pub struct BExpStep {
    pub duty_q15: f64,
    pub tau_s: f64,
    pub omega_ss: f64,
    pub r2: f64,
    pub b: f64,
}

#[derive(Clone, Debug)]
pub struct BExp {
    /// Quality-weighted mean of the per-step B estimates.
    pub b: f64,
    /// Relative spread (stddev / mean) across steps; the consistency metric.
    pub spread: f64,
    pub steps: Vec<BExpStep>,
}

/// Exponential-rise estimator. At constant duty the current collapses as
/// bemf rises, i = (v - Ke*omega)/R; substituting into
/// alpha = B*f_med*(i - fc - fv*omega) gives
///
///   alpha = B*f_med*((v/R - fc) - (fv + Ke/R)*omega)
///
/// a first-order rise with tau = 1 / (B * f_med * (fv + Ke/R)) and
/// omega_ss = (v/R - fc) / (fv + Ke/R). So tau plus the fitted fv/Ke/R
/// recovers B without ever double-differentiating: omega comes from the
/// robust FIRST derivative, and tau from a separable exponential fit -
/// for fixed tau the model omega = a + c*exp(-t/tau) is linear_ls on
/// x = exp(-t/tau), so tau is a 1-D search (log grid + golden section)
/// on the residual RMS. No omega_ss prior, no saturation requirement;
/// a capture cut mid-rise just fits with wider error bars (lower r2).
pub fn b_exp_fit(steps: &[StepSeries], p: &InertiaPriors, half_window: usize) -> Option<BExp> {
    let fv_eff = p.fv + p.ke_vpc / p.r_vpc;
    if fv_eff <= 0.0 {
        return None;
    }
    let mut out = Vec::new();
    for s in steps {
        let d = sliding_quadratic_deriv(&s.t, &s.pos, half_window);
        let sgn = if s.duty_q15 >= 0.0 { 1.0 } else { -1.0 };
        let tw: Vec<(f64, f64)> = (0..s.t.len())
            .filter_map(|k| d.dy[k].map(|w| (s.t[k], w * sgn)).filter(|_| s.mask[k]))
            .collect();
        if tw.len() < 16 {
            continue;
        }
        let t0 = tw[0].0;
        let rms_at = |tau: f64| -> Option<(f64, crate::fitmath::LinearFit)> {
            let xy: Vec<(f64, f64)> = tw
                .iter()
                .map(|&(t, w)| ((-(t - t0) / tau).exp(), w))
                .collect();
            linear_ls(&xy).map(|f| (f.rms, f))
        };
        // log grid seed over 1 ms .. 2 s, then golden-section refine
        const LO: f64 = 1e-3;
        const HI: f64 = 2.0;
        let mut best = (f64::INFINITY, LO);
        for k in 0..=40 {
            let tau = LO * (HI / LO).powf(k as f64 / 40.0);
            if let Some((rms, _)) = rms_at(tau)
                && rms < best.0
            {
                best = (rms, tau);
            }
        }
        if !best.0.is_finite() || best.1 <= LO * 1.01 || best.1 >= HI * 0.99 {
            continue; // no interior optimum: not an exponential rise
        }
        let phi = (5f64.sqrt() - 1.0) / 2.0;
        let (mut a, mut b) = (
            best.1 / (HI / LO).powf(0.025),
            best.1 * (HI / LO).powf(0.025),
        );
        for _ in 0..40 {
            let x1 = b - phi * (b - a);
            let x2 = a + phi * (b - a);
            let r1 = rms_at(x1).map(|r| r.0).unwrap_or(f64::INFINITY);
            let r2 = rms_at(x2).map(|r| r.0).unwrap_or(f64::INFINITY);
            if r1 < r2 {
                b = x2;
            } else {
                a = x1;
            }
        }
        let tau = (a + b) / 2.0;
        let Some((_, f)) = rms_at(tau) else { continue };
        // omega(inf) = intercept; c must be negative (a rise, not a decay)
        if f.b >= 0.0 || f.a <= 0.0 {
            continue;
        }
        out.push(BExpStep {
            duty_q15: s.duty_q15,
            tau_s: tau,
            omega_ss: f.a,
            r2: f.r2,
            b: 1.0 / (tau * p.f_med() * fv_eff),
        });
    }
    if out.is_empty() {
        return None;
    }
    let wsum: f64 = out.iter().map(|s| s.r2.max(0.0)).sum();
    if wsum <= 0.0 {
        return None;
    }
    let b = out.iter().map(|s| s.b * s.r2.max(0.0)).sum::<f64>() / wsum;
    let var = out.iter().map(|s| (s.b - b).powi(2)).sum::<f64>() / out.len() as f64;
    Some(BExp {
        b,
        spread: var.sqrt() / b,
        steps: out,
    })
}
