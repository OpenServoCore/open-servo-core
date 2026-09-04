//! Experiment samples -> physical motor parameters, count domain
//! throughout (vcounts, ccounts, c/s). Encoding to table Q formats is gain
//! synthesis' job. Started by the ladder (Ke + friction line); later
//! experiments add their fits here.

use crate::fitmath::{linear_ls, origin_ls};

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
