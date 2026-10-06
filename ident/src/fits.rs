//! Experiment samples -> physical motor parameters, count domain
//! throughout (vcounts, ccounts, c/s). Encoding to table Q formats is gain
//! synthesis' job. Started by the ladder (Ke + friction line); the inertia
//! fits live here too: B from the current decay after a step, the one the
//! gains take, the ladder's governed climbs as its cross-check, and the
//! pot-speed estimators (direct-alpha and exponential-rise), diagnostics.

use crate::fitmath::{
    LinearFit, linear_ls, lstsq, mean, origin_ls, sliding_quadratic_deriv, stddev,
};

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
    /// Rungs the fit took, the slowest ones up to the knee.
    pub n: usize,
    /// Rungs above the knee, left out.
    pub left_out: usize,
    /// The largest share of volts per speed a left-out rung needs over the
    /// fit, 0 when every rung was taken.
    pub excess_top: f64,
}

/// A rung's own Ke, (v - R i) / omega, within this of the fit over the
/// slower rungs joins the fit. The MG90 ladder is flat within 2% from 26 to
/// 40% duty, then needs 5% more volts per speed at 47% and 15% at 64%; an
/// origin fit over every rung weights by omega squared and hands that to
/// Ke, 6% high against the coast EMF, and the observer reads low by it at
/// every duty.
pub const KE_FLAT_TOL: f64 = 0.03;

/// Origin LS of (v - R*i) vs omega, both directions, walked up from the two
/// slowest rungs: each faster rung joins while its own Ke sits within
/// [`KE_FLAT_TOL`] of the fit so far, and the first one that does not ends
/// the band. Volts per speed the plant needs beyond the drive model above
/// the knee are reported, never fitted.
pub fn ke_fit(rungs: &[RungPoint], r_vpc: f64) -> Option<KeFit> {
    let mut xy: Vec<(f64, f64)> = rungs
        .iter()
        .map(|p| (p.omega.abs(), (p.v - r_vpc * p.i) * p.omega.signum()))
        .collect();
    xy.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut n = 2.min(xy.len());
    while n < xy.len() {
        let f = origin_ls(&xy[..n])?;
        let (w, e) = xy[n];
        if f.slope <= 0.0 || w <= 0.0 || ((e / w - f.slope) / f.slope).abs() > KE_FLAT_TOL {
            break;
        }
        n += 1;
    }
    let f = origin_ls(&xy[..n])?;
    let excess_top = xy[n..]
        .iter()
        .map(|(w, e)| e / w / f.slope - 1.0)
        .fold(0.0, f64::max);
    Some(KeFit {
        ke_vpc: f.slope,
        r2: f.r2,
        rms: f.rms,
        n: f.n,
        left_out: xy.len() - n,
        excess_top,
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

    /// The damping a constant duty adds to friction: fv + Ke/R, ccounts
    /// per c/s. Its time constant is 1 / (B f_med this).
    fn damping(&self) -> f64 {
        self.fv + self.ke_vpc / self.r_vpc
    }
}

/// y = a + c exp(-(t - t0) / tau) over `tw`, t0 the first t: for a fixed
/// tau it is linear in exp(-(t - t0)/tau), so tau is a 1-D search (log grid
/// over 1 ms .. 2 s, then golden section) on the residual rms. None without
/// an interior optimum: not an exponential.
fn exp_fit(tw: &[(f64, f64)]) -> Option<(f64, LinearFit)> {
    const LO: f64 = 1e-3;
    const HI: f64 = 2.0;
    let t0 = tw.first()?.0;
    let rms_at = |tau: f64| -> Option<(f64, LinearFit)> {
        let xy: Vec<(f64, f64)> = tw
            .iter()
            .map(|&(t, w)| ((-(t - t0) / tau).exp(), w))
            .collect();
        linear_ls(&xy).map(|f| (f.rms, f))
    };
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
        return None;
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
    rms_at(tau).map(|(_, f)| (tau, f))
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
        let Some((tau, f)) = exp_fit(&tw) else {
            continue;
        };
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

/// Skipped after a step's goal is applied before its current decay is
/// fitted, s: the winding's electrical rise and the first windows.
pub const DECAY_SKIP_S: f64 = 0.002;

/// A decay's amplitude over the fit's residual rms, the least that
/// stands out of the current's commutation ripple: the bench servo's
/// steps read 3.7 to 7.8.
pub const DECAY_SNR_MIN: f64 = 2.0;

#[derive(Clone, Debug)]
pub struct BDecayStep {
    pub duty_q15: f64,
    /// Mechanical time constant, s.
    pub tau_s: f64,
    /// Settled current and the decay's amplitude at the fit's start,
    /// ccounts, folded by the drive's sign.
    pub i_ss: f64,
    pub amp: f64,
    pub r2: f64,
    pub b: f64,
}

#[derive(Clone, Debug)]
pub struct BDecay {
    /// Mean of the per-step B.
    pub b: f64,
    /// Sample sd over the mean across steps.
    pub spread: f64,
    pub steps: Vec<BDecayStep>,
}

/// Current-decay estimator, one step. After a duty step the current jumps
/// and falls as the back-EMF rises with the speed: at constant duty
/// i = i_ss + A exp(-t/tau), with the tau the exp-rise estimator reads off
/// the speed, 1 / (B f_med (fv + Ke/R)). The current carries it without
/// the pot's noise and without a derivative. Err says, in plain words, why
/// the step's current is not such a decay.
pub fn b_decay_step(s: &StepSeries, p: &InertiaPriors) -> Result<BDecayStep, &'static str> {
    let sgn = if s.duty_q15 >= 0.0 { 1.0 } else { -1.0 };
    let t0 = (0..s.t.len())
        .find(|&k| s.mask[k])
        .map(|k| s.t[k])
        .ok_or("no sample applied the step")?;
    let tw: Vec<(f64, f64)> = (0..s.t.len())
        .filter(|&k| s.mask[k] && s.t[k] >= t0 + DECAY_SKIP_S)
        .map(|k| (s.t[k], s.i[k] * sgn))
        .collect();
    if tw.len() < 16 {
        return Err("too few samples after the step to fit its current");
    }
    let (tau, f) = exp_fit(&tw).ok_or("its current does not decay exponentially")?;
    if f.b <= 0.0 || f.a <= 0.0 {
        return Err("its current does not fall after the step");
    }
    if f.b < DECAY_SNR_MIN * f.rms {
        return Err("its current shows no decay above its ripple");
    }
    Ok(BDecayStep {
        duty_q15: s.duty_q15,
        tau_s: tau,
        i_ss: f.a,
        amp: f.b,
        r2: f.r2,
        b: 1.0 / (tau * p.f_med() * p.damping()),
    })
}

/// The steps' B pooled: their mean and spread. None without a step.
pub fn b_decay_pool(steps: Vec<BDecayStep>) -> Option<BDecay> {
    let bs: Vec<f64> = steps.iter().map(|s| s.b).collect();
    let b = mean(&bs)?;
    Some(BDecay {
        b,
        spread: stddev(&bs).unwrap_or(0.0) / b,
        steps,
    })
}

/// One governed climb off the ladder: the limiter holds the current while
/// the shaft accelerates. Host time, s; the kernel's counts; window
/// current, ccounts; both as driven, `dir` their sign.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Climb {
    pub dir: i8,
    pub t: Vec<f64>,
    pub pos: Vec<f64>,
    pub i: Vec<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BClimb {
    pub b: f64,
    pub sd: f64,
    pub n: usize,
}

/// Climb estimator: each climb's acceleration, pos = c0 + c1 t + alpha t^2
/// / 2, against the torque its held current leaves over friction at its
/// mean speed, alpha = B f_med (i - fc - fv w). None without a climb that
/// accelerates on a positive torque.
pub fn b_climb_fit(climbs: &[Climb], p: &InertiaPriors) -> Option<BClimb> {
    let bs: Vec<f64> = climbs
        .iter()
        .filter_map(|c| {
            let s = c.dir as f64;
            let t0 = *c.t.first()?;
            let x: Vec<Vec<f64>> =
                c.t.iter()
                    .map(|t| vec![1.0, t - t0, (t - t0) * (t - t0)])
                    .collect();
            let y: Vec<f64> = c.pos.iter().map(|q| q * s).collect();
            let (coef, _) = lstsq(&x, &y)?;
            let alpha = 2.0 * coef[2];
            let w = coef[1] + alpha * (mean(&c.t)? - t0);
            let torque = mean(&c.i)? * s - p.fc - p.fv * w;
            (alpha > 0.0 && torque > 0.0).then(|| alpha / (p.f_med() * torque))
        })
        .collect();
    let b = mean(&bs)?;
    Some(BClimb {
        b,
        sd: stddev(&bs).unwrap_or(0.0),
        n: bs.len(),
    })
}

#[cfg(test)]
mod ke_tests {
    use super::*;

    fn pair(w: f64, i: f64, v: f64) -> [RungPoint; 2] {
        [1.0, -1.0].map(|s| RungPoint {
            omega: s * w,
            i: s * i,
            v: s * v,
        })
    }

    #[test]
    fn flat_ladder_takes_every_rung() {
        let (ke, r) = (0.084, 1.04);
        let pts: Vec<RungPoint> = [4683.0, 6157.0, 7552.0, 8841.0, 10082.0, 11320.0]
            .iter()
            .flat_map(|&w| pair(w, 120.0, ke * w + r * 120.0))
            .collect();
        let f = ke_fit(&pts, r).unwrap();
        assert!((f.ke_vpc - ke).abs() < 1e-9, "{}", f.ke_vpc);
        assert_eq!((f.n, f.left_out), (12, 0));
        assert_eq!(f.excess_top, 0.0);
    }

    #[test]
    fn ke_fit_stops_at_the_knee() {
        // volts per speed beyond the model growing as the square of the speed
        // over the three top rungs, the MG90 ladder's shape
        let (ke, r) = (0.084, 1.04);
        let pts: Vec<RungPoint> = [4683.0, 6157.0, 7552.0, 8841.0, 10082.0, 11320.0]
            .iter()
            .flat_map(|&w| {
                let x = f64::max((w - 7552.0) / (11320.0 - 7552.0), 0.0);
                pair(w, 120.0, ke * w + r * 120.0 + 400.0 * x * x)
            })
            .collect();
        let f = ke_fit(&pts, r).unwrap();
        assert!((f.ke_vpc - ke).abs() < 1e-9, "{}", f.ke_vpc);
        assert_eq!((f.n, f.left_out), (6, 6));
        assert!(
            (f.excess_top - 400.0 / (ke * 11320.0)).abs() < 1e-9,
            "{}",
            f.excess_top
        );
    }

    /// The MG90 ladder on rev 2A board #1 (ident run 1791265396, rungs.csv),
    /// R the burst winding 4.64 ohm (1.0386 vcounts/ccount). Over every rung
    /// the origin fit lands at 0.0900; the coast EMF says 0.0813 per ripple
    /// count and 0.0838 per linearized pot count. The flat band is 26-40%.
    #[test]
    fn mg90_ladder_ke_comes_from_the_flat_band() {
        let pts = [
            (4682.851513, 108.344595, 504.812068),
            (-4580.333915, -109.271084, -504.630136),
            (6156.841809, 115.107784, 640.349594),
            (-6058.686622, -118.248322, -640.146978),
            (7552.125382, 122.363636, 775.442136),
            (-7534.046148, -124.925926, -775.236521),
            (8840.676041, 122.966292, 910.651474),
            (-8881.587330, -131.056338, -910.135971),
            (10082.450708, 126.803571, 1065.093686),
            (-10456.812864, -138.396552, -1064.277824),
            (11320.000897, 134.0, 1238.633754),
            (-11727.213305, -138.05, -1237.703082),
        ]
        .map(|(omega, i, v)| RungPoint { omega, i, v });
        let f = ke_fit(&pts, 1.0386).unwrap();
        assert!((f.ke_vpc - 0.0853).abs() < 0.0005, "{}", f.ke_vpc);
        assert_eq!((f.n, f.left_out), (6, 6));
        assert!(
            f.excess_top > 0.12 && f.excess_top < 0.15,
            "{}",
            f.excess_top
        );
        let all = origin_ls(&pts.map(|p| (p.omega, p.v - 1.0386 * p.i)))
            .unwrap()
            .slope;
        assert!((all - 0.0900).abs() < 0.0005, "{all}");
    }
}
