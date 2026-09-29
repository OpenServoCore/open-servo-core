//! Ripple-referenced position linearization on the firmware's grid.
//!
//! The pot's local gain varies along its travel, so a straight line between
//! the stops mis-reports position and, worse, speed. The commutation ripple
//! is a motor-side angle clock with no pot in it ([`crate::ripple`]), and
//! constant-duty rungs driven through the travel turn it into a table:
//!
//! 1. [`build`] tracks every rung in the core duty band with
//!    [`ripple::motor_angle`], seeded by a coupling prior (ripple cycles per
//!    pot count) measured on the settled windows of the 20 to 40% rungs, and
//!    keeps the rungs the tracker held over [`MIN_ACCEPT`] of and whose
//!    whole-rung coupling sits within [`CPC_TOL`] of the median.
//! 2. [`stitch`] deposits each time step's angle increment into the counts
//!    the pot occupied during that step, so a rung's deposits sum to its
//!    angle span and the pot's jitter never reorders the clock. Rungs share
//!    the count axis and average per count; interior gaps are bridged from
//!    the median density of up to [`GAP_ANCHOR`] covered counts either side.
//!    Backlash is a constant offset between the two directions and has no
//!    density, so it cannot leak into the table.
//! 3. [`fit_grid`] anchors on rung coverage, never on the extreme sample (one
//!    stray rung would move every point): points inside the stretch at least
//!    [`MIN_RUNG_COVER`] rungs cross carry the chord through the band ends,
//!    every other point is zero, so the insets and the stops map to
//!    themselves.
//!
//! The table is the one the firmware applies (core pos_lut.rs): [`INTERVALS`]
//! intervals of [`GRID`] raw counts over the 12-bit ADC domain, [`POINTS`]
//! i16 corrections against the identity ramp, point k at raw k * GRID, the
//! last fixed 0. Per sample the firmware indexes with raw >> GRID_SHIFT and
//! interpolates in integer math to a Q4 word, linearized counts times 16.
//! [`interp_q4`] and [`validate`] mirror it bit for bit; [`Image`] is the
//! JSON `osc lut write` consumes.
//!
//! The tail of this module is the older 55-point [`PosLut`], clocked by the
//! autocorrelation tachometer ([`cumulative_phase`]). The servo stores only
//! the grid; `osc cal-replay` still builds the 55-point table to report on a
//! recorded sweep.

use serde::{Deserialize, Serialize};

use crate::{fitmath, ripple};

pub const ADC_BITS: u32 = 12;
pub const GRID_SHIFT: u32 = 4;
/// Raw counts per interval.
pub const GRID: u16 = 1 << GRID_SHIFT;
pub const INTERVALS: usize = 1 << (ADC_BITS - GRID_SHIFT);
/// Point INTERVALS is fixed 0.
pub const POINTS: usize = INTERVALS + 1;
const ADC_MASK: u16 = (1 << ADC_BITS) - 1;
const FRAC_MASK: u16 = GRID - 1;
/// Local gain at or above this is corruption, not a pot.
pub const GAIN_MAX: i32 = 16;

/// The interval a raw sample falls in.
pub fn index(raw: u16) -> usize {
    ((raw & ADC_MASK) >> GRID_SHIFT) as usize
}

/// Linearized counts in Q4 as the u16 the firmware computes from a sample
/// and the two points around it: one multiply, no divide, the same wrap as
/// the u16 cast (a table that passes [`validate`] never reaches it).
pub fn interp_q4(raw: u16, c0: i16, c1: i16) -> u16 {
    let raw = raw & ADC_MASK;
    let f = (raw & FRAC_MASK) as i32;
    ((((raw as i32) + c0 as i32) << GRID_SHIFT) + (c1 as i32 - c0 as i32) * f) as u16
}

/// Why the firmware refuses a table.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reject {
    /// A nonzero point at or beyond a stop.
    Ends,
    /// An interval's gain outside 1/GRID .. GAIN_MAX.
    Shape,
}

/// The firmware's check: identity at and beyond the stops, so raw_min and
/// raw_max map to themselves whether or not they sit on a point and points 0,
/// INTERVALS - 1 and INTERVALS are zero; then every interval's Q4 gain
/// `d = GRID + c[k + 1] - c[k]` in `1 <= d < GAIN_MAX * GRID`, so the
/// output is monotone and stays a u16.
pub fn validate(points: &[i16; POINTS], raw_min: u16, raw_max: u16) -> Result<(), Reject> {
    let lo = (raw_min as usize + GRID as usize - 1) >> GRID_SHIFT;
    let hi = raw_max as usize >> GRID_SHIFT;
    if points
        .iter()
        .enumerate()
        .any(|(k, &c)| (k <= lo || k >= hi) && c != 0)
    {
        return Err(Reject::Ends);
    }
    let gain = |w: &[i16]| GRID as i32 + w[1] as i32 - w[0] as i32;
    if points
        .windows(2)
        .any(|w| !(1..GAIN_MAX * GRID as i32).contains(&gain(w)))
    {
        return Err(Reject::Shape);
    }
    Ok(())
}

/// The firmware's table: POINTS corrections against the identity ramp, point
/// k at raw k * GRID, the last fixed 0. All-zero is the identity, raw << 4
/// everywhere.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct GridLut {
    pub points: [i16; POINTS],
}

impl GridLut {
    pub const IDENTITY: GridLut = GridLut {
        points: [0; POINTS],
    };

    /// The Q4 word the firmware computes for a raw sample.
    pub fn q4(&self, raw: u16) -> u16 {
        let i = index(raw);
        interp_q4(raw, self.points[i], self.points[i + 1])
    }

    /// Linearized counts as the firmware sees them, the Q4 word over GRID.
    pub fn counts(&self, raw: u16) -> f64 {
        self.q4(raw) as f64 / GRID as f64
    }

    pub fn validate(&self, raw_min: u16, raw_max: u16) -> Result<(), Reject> {
        validate(&self.points, raw_min, raw_max)
    }

    /// The image body with the stops the table was built against.
    pub fn image(
        &self,
        raw_min: u16,
        raw_max: u16,
        dataset: &str,
        rungs: usize,
        covered: (u16, u16),
        source: &str,
    ) -> Image {
        Image {
            raw_min,
            raw_max,
            grid_shift: GRID_SHIFT,
            points: self.points[..INTERVALS].to_vec(),
            dataset: dataset.to_owned(),
            rungs,
            covered: [covered.0, covered.1],
            source: source.to_owned(),
        }
    }
}

/// The image `osc lut write` takes: points 0..INTERVALS - 1 (the fixed last
/// point left out), the stops the table was built against, and where it came
/// from. `covered` is the anchor band, `rungs` how many rungs stitched it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Image {
    pub raw_min: u16,
    pub raw_max: u16,
    pub grid_shift: u32,
    pub points: Vec<i16>,
    pub dataset: String,
    pub rungs: usize,
    pub covered: [u16; 2],
    pub source: String,
}

impl Image {
    /// The table, or None unless the image is on the firmware grid.
    pub fn lut(&self) -> Option<GridLut> {
        if self.grid_shift != GRID_SHIFT || self.points.len() != INTERVALS {
            return None;
        }
        let mut points = [0i16; POINTS];
        points[..INTERVALS].copy_from_slice(&self.points);
        Some(GridLut { points })
    }
}

/// Core duty band, percent: what every route and every set tracks cleanly.
pub const CORE_PCT: (i32, i32) = (15, 50);
/// Duty band the coupling prior is measured on.
pub const PRIOR_PCT: (i32, i32) = (20, 40);
/// A rung counts only if the tracker holds the line over this much of it.
pub const MIN_ACCEPT: f64 = 0.90;
/// And its whole-rung coupling sits within this of the median.
pub const CPC_TOL: f64 = 0.03;
/// Points anchor on the stretch at least this many rungs cross.
pub const MIN_RUNG_COVER: u32 = 20;

/// One constant-duty drive rung: the raw pot and the bias-removed current at
/// every tick, the commanded duty in percent (negative = reverse).
#[derive(Copy, Clone, Debug)]
pub struct Rung<'a> {
    pub pos: &'a [u16],
    pub current: &'a [f64],
    pub duty_pct: i32,
    pub tick_hz: f64,
}

/// What became of one rung.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Verdict {
    /// Stitched into the table.
    Used,
    /// Duty outside CORE_PCT.
    OffCore,
    /// The tracker found no ripple line, or the pot never moved under it.
    Untracked,
    /// The track holds this fraction of the rung, under MIN_ACCEPT.
    Short(f64),
    /// Whole-rung coupling over the median, off by CPC_TOL or more.
    Coupling(f64),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BuildError {
    /// No rung in PRIOR_PCT gave a coupling prior.
    NoPrior,
    /// Fewer than two counts crossed by MIN_RUNG_COVER accepted rungs.
    Uncovered,
    /// The table would fail the firmware's check.
    Rejected(Reject),
}

/// The table and how it was built.
#[derive(Clone, Debug)]
pub struct Build {
    pub lut: GridLut,
    /// The anchor band, the stretch MIN_RUNG_COVER rungs cross.
    pub covered: (u16, u16),
    /// Ripple cycles per raw count, the tracker's seed.
    pub c_prior: f64,
    /// One per input rung.
    pub verdicts: Vec<Verdict>,
}

impl Build {
    /// Rungs stitched into the table.
    pub fn rungs(&self) -> usize {
        self.verdicts
            .iter()
            .filter(|v| **v == Verdict::Used)
            .count()
    }
}

fn in_band(duty_pct: i32, band: (i32, i32)) -> bool {
    (band.0..=band.1).contains(&duty_pct.abs())
}

/// Ripple cycles per raw count over a rung's settled window: the line
/// frequency over the pot speed.
fn coupling(r: &Rung) -> Option<f64> {
    let s0 = ripple::settled(r.pos.len());
    let v = ripple::pot_speed(r.pos.get(s0..)?, r.tick_hz)?.abs();
    let line = ripple::settled_line(&ripple::line_spectrum(r.current.get(s0..)?, r.tick_hz))?;
    Some(line.hz / v).filter(|c| c.is_finite())
}

struct Track {
    cycles: Vec<f64>,
    start: usize,
    accept: f64,
    cpc: Option<f64>,
}

fn track(r: &Rung, c_prior: f64) -> Option<Track> {
    let m = ripple::motor_angle(r.current, r.pos, c_prior, r.tick_hz)?;
    let n = r.pos.len();
    let (first, last) = (*r.pos.get(m.start)?, *r.pos.last()?);
    let travel = (last as i32 - first as i32).abs();
    let cpc = (travel != 0).then(|| (m.cycles[n - 1] - m.cycles[m.start]) / travel as f64);
    Some(Track {
        cycles: m.cycles,
        start: m.start,
        accept: (n - m.start) as f64 / n as f64,
        cpc,
    })
}

/// The grid table from a set of rungs, stops `raw_min..raw_max`. The prior
/// is the median coupling of the PRIOR_PCT rungs; every CORE_PCT rung is
/// tracked from it and accepted on MIN_ACCEPT and CPC_TOL against the
/// median of the tracked; the accepted rungs stitch from their first
/// trusted sample and the table anchors on the MIN_RUNG_COVER band. Float
/// sums follow rung order, so the same rungs in the same order give the
/// same points.
pub fn build(rungs: &[Rung], raw_min: u16, raw_max: u16) -> Result<Build, BuildError> {
    let priors: Vec<f64> = rungs
        .iter()
        .filter(|r| in_band(r.duty_pct, PRIOR_PCT))
        .filter_map(coupling)
        .collect();
    let c_prior = fitmath::median(&priors).ok_or(BuildError::NoPrior)?;
    let tracks: Vec<Option<Track>> = rungs
        .iter()
        .map(|r| {
            in_band(r.duty_pct, CORE_PCT)
                .then(|| track(r, c_prior))
                .flatten()
        })
        .collect();
    let cpcs: Vec<f64> = tracks.iter().flatten().filter_map(|t| t.cpc).collect();
    let med = fitmath::median(&cpcs).unwrap_or(f64::NAN);
    let mut verdicts = Vec::with_capacity(rungs.len());
    let mut chunks: Vec<(&[u16], &[f64])> = Vec::new();
    for (r, t) in rungs.iter().zip(&tracks) {
        let v = match t {
            _ if !in_band(r.duty_pct, CORE_PCT) => Verdict::OffCore,
            None => Verdict::Untracked,
            Some(t) => match t.cpc {
                None => Verdict::Untracked,
                _ if t.accept < MIN_ACCEPT => Verdict::Short(t.accept),
                Some(cpc) if (cpc / med - 1.0).abs() < CPC_TOL => {
                    chunks.push((&r.pos[t.start..], &t.cycles[t.start..]));
                    Verdict::Used
                }
                Some(cpc) => Verdict::Coupling(cpc / med),
            },
        };
        verdicts.push(v);
    }
    let st = stitch(&chunks, raw_min, raw_max).ok_or(BuildError::Uncovered)?;
    let covered = st
        .well_covered(MIN_RUNG_COVER)
        .ok_or(BuildError::Uncovered)?;
    let lut = fit_grid(&st, covered);
    lut.validate(raw_min, raw_max)
        .map_err(BuildError::Rejected)?;
    Ok(Build {
        lut,
        covered,
        c_prior,
        verdicts,
    })
}

/// Below this many samples a chunk cannot define a curve.
const MIN_SAMPLES: usize = 4;
/// Covered counts scanned inward from a gap edge to form the anchor median.
pub const GAP_ANCHOR: usize = 8;

/// Per-count angle over the shared pot axis from several chunks. Bin b is
/// count raw_min + b; `cum` is the angle at the lower edge of each bin from
/// the first covered bin, `chunks` how many chunks crossed it (0 = gap or
/// outside), `fc..=lc` the covered bins.
#[derive(Clone, Debug)]
pub struct Stitch {
    raw_min: u16,
    cum: Vec<f64>,
    chunks: Vec<u32>,
    fc: usize,
    lc: usize,
}

impl Stitch {
    /// First covered count.
    pub fn lo(&self) -> u16 {
        self.raw_min + self.fc as u16
    }

    /// Last covered count.
    pub fn hi(&self) -> u16 {
        self.raw_min + self.lc as u16
    }

    /// Covered fraction of the stops' span.
    pub fn coverage(&self) -> f64 {
        self.chunks.iter().filter(|&&c| c > 0).count() as f64 / self.chunks.len() as f64
    }

    /// Chunks crossing a count, 0 outside the stops.
    pub fn chunks_at(&self, raw: u16) -> u32 {
        raw.checked_sub(self.raw_min)
            .and_then(|b| self.chunks.get(b as usize))
            .copied()
            .unwrap_or(0)
    }

    /// Stitched angle at raw counts, relative to the first covered count,
    /// linear between counts and held flat beyond the stops.
    pub fn angle(&self, raw: f64) -> f64 {
        let x = raw - self.raw_min as f64;
        let last = self.cum.len() - 1;
        if x.is_nan() || x <= 0.0 {
            return self.cum[0];
        }
        if x >= last as f64 {
            return self.cum[last];
        }
        let j = x as usize;
        let xj = j as f64;
        if x == xj {
            return self.cum[j];
        }
        (self.cum[j + 1] - self.cum[j]) * (x - xj) + self.cum[j]
    }

    /// `(lo, hi)`: the stretch at least `min_cover` chunks cross, or None.
    pub fn well_covered(&self, min_cover: u32) -> Option<(u16, u16)> {
        let well = |&(_, &c): &(usize, &u32)| c >= min_cover;
        let a = self.chunks.iter().enumerate().find(well)?.0;
        let b = self.chunks.iter().enumerate().rfind(well)?.0;
        (a < b).then(|| (self.raw_min + a as u16, self.raw_min + b as u16))
    }

    /// Linearized counts straight from the stitch, no points: the curve a
    /// finer table converges to, the chord through `lo` and `hi` mapping to
    /// themselves.
    pub fn true_counts(&self, raw: f64, lo: u16, hi: u16) -> f64 {
        let (a0, a1) = (self.angle(lo as f64), self.angle(hi as f64));
        let rel = (self.angle(raw) - a0) / (a1 - a0);
        lo as f64 + rel * (hi as f64 - lo as f64)
    }
}

/// One chunk's angle per count bin and the bins it touched, from its
/// time-ordered steps. A step that sat on one count lands there whole (a
/// dwell at raw_max folds into the top interval); a step that crossed
/// counts spreads evenly over the counts traversed, clipped to the stops.
/// The crossing deposits accumulate as edge differences and a running sum,
/// the order the notebook sums them in.
fn deposit(
    pos: &[u16],
    cyc: &[f64],
    raw_min: u16,
    raw_max: u16,
    local: &mut [f64],
    touched: &mut [bool],
) {
    let bins = local.len();
    local.fill(0.0);
    let mut touch = vec![0i64; bins];
    let mut d = vec![0.0; bins + 1];
    let mut t = vec![0i64; bins + 1];
    let steps = || {
        pos.windows(2).zip(cyc.windows(2)).map(|(p, c)| {
            let (a, b) = (p[0].min(p[1]), p[0].max(p[1]));
            (a, b, (c[1] - c[0]).max(0.0))
        })
    };
    for (a, b, dphi) in steps() {
        if a == b && a >= raw_min && a <= raw_max {
            let idx = (a.min(raw_max - 1) - raw_min) as usize;
            local[idx] += dphi;
            touch[idx] += 1;
        }
    }
    let edges = || {
        steps().filter(|(a, b, _)| a != b).map(|(a, b, dphi)| {
            let s = dphi / (b - a) as f64;
            let lo = (a.clamp(raw_min, raw_max) - raw_min) as usize;
            let hi = (b.clamp(raw_min, raw_max) - raw_min) as usize;
            (lo, hi, s)
        })
    };
    for (lo, _, s) in edges() {
        d[lo] += s;
        t[lo] += 1;
    }
    for (_, hi, s) in edges() {
        d[hi] -= s;
        t[hi] -= 1;
    }
    let (mut run, mut cover) = (0.0, 0i64);
    for b in 0..bins {
        run += d[b];
        cover += t[b];
        local[b] += run;
        touched[b] = touch[b] + cover > 0;
    }
}

/// Median density over up to GAP_ANCHOR covered bins scanning from `start`
/// in direction `dir`, staying within `lo..=hi`; skips gap fills so anchors
/// are real measurements. 0 if none found.
fn anchor_density(
    avg: &[f64],
    chunks: &[u32],
    start: usize,
    dir: i64,
    lo: usize,
    hi: usize,
) -> f64 {
    let mut vals: Vec<f64> = Vec::with_capacity(GAP_ANCHOR);
    let mut b = start as i64;
    while vals.len() < GAP_ANCHOR && b >= lo as i64 && b <= hi as i64 {
        if chunks[b as usize] > 0 {
            vals.push(avg[b as usize]);
        }
        b += dir;
    }
    vals.sort_by(f64::total_cmp);
    vals.get(vals.len() / 2).copied().unwrap_or(0.0)
}

/// Per-count angle over the shared pot axis from several `(pos, angle)`
/// chunks, each one time-ordered run of raw pot samples with the motor
/// angle at every sample. A chunk shorter than MIN_SAMPLES or with
/// mismatched lengths is skipped. None when the stops are degenerate, fewer
/// than two counts are covered, or the integrated angle is not positive.
pub fn stitch(chunks: &[(&[u16], &[f64])], raw_min: u16, raw_max: u16) -> Option<Stitch> {
    if raw_max <= raw_min {
        return None;
    }
    let bins = (raw_max - raw_min) as usize + 1;
    let mut acc = vec![0.0; bins];
    let mut cov = vec![0u32; bins];
    let mut local = vec![0.0; bins];
    let mut touched = vec![false; bins];
    for &(pos, cyc) in chunks {
        if pos.len() != cyc.len() || pos.len() < MIN_SAMPLES {
            continue;
        }
        deposit(pos, cyc, raw_min, raw_max, &mut local, &mut touched);
        for b in 0..bins {
            if touched[b] {
                acc[b] += local[b];
                cov[b] += 1;
            }
        }
    }
    let fc = cov.iter().position(|&c| c > 0)?;
    let lc = cov.iter().rposition(|&c| c > 0)?;
    if fc == lc {
        return None;
    }
    let mut avg: Vec<f64> = acc
        .iter()
        .zip(&cov)
        .map(|(&a, &c)| if c > 0 { a / c as f64 } else { 0.0 })
        .collect();
    let mut prev = fc;
    for b in (fc + 1)..=lc {
        if cov[b] > 0 {
            if b > prev + 1 {
                let y0 = anchor_density(&avg, &cov, prev, -1, fc, lc);
                let y1 = anchor_density(&avg, &cov, b, 1, fc, lc);
                let dx = (b - prev) as f64;
                for (j, g) in avg[(prev + 1)..b].iter_mut().enumerate() {
                    *g = y0 + (y1 - y0) * (j + 1) as f64 / dx;
                }
            }
            prev = b;
        }
    }
    let mut cum = vec![0.0; bins];
    for b in (fc + 1)..=lc {
        cum[b] = cum[b - 1] + avg[b];
    }
    for b in (lc + 1)..bins {
        cum[b] = cum[lc];
    }
    if cum[lc] - cum[fc] <= 0.0 {
        return None;
    }
    Some(Stitch {
        raw_min,
        cum,
        chunks: cov,
        fc,
        lc,
    })
}

/// The grid table from a stitch: points inside `band` carry the chord
/// through the band ends, rounded half away from zero; every other point is
/// zero.
pub fn fit_grid(st: &Stitch, band: (u16, u16)) -> GridLut {
    let mut points = [0i16; POINTS];
    for (k, c) in points.iter_mut().enumerate() {
        let r = (k * GRID as usize) as u16;
        if band.0 <= r && r <= band.1 {
            *c = sat_i16((st.true_counts(r as f64, band.0, band.1) - r as f64).round());
        }
    }
    GridLut { points }
}

fn sat_i16(v: f64) -> i16 {
    if !v.is_finite() {
        return 0;
    }
    v.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

// --- the 55-point table cal-replay reports ---

const N_POINTS: usize = 55;
const N_INTERVALS: usize = N_POINTS - 1;
/// cumulative_phase needs a majority of windows to find ripple, else the
/// sweep SNR is too low to trust as an angle clock.
const MIN_GOOD_FRAC: f64 = 0.5;

/// A sweep covering less than this fraction of the rail span leaves too much
/// of travel for `fit_points`' affine anchoring to extrapolate through: the
/// anchoring assumes local linearity over the uncovered insets, which fails
/// once whole rail regions are uncovered. Below this coverage `build_multi`
/// returns identity.
pub const MIN_SPAN_COVER: f64 = 0.7;

/// The 55-point table (raw_min/raw_max/corr), host side only: 55 points
/// evenly spaced in raw counts, `raw_i = raw_min + i * span / 54`,
/// `corrected(raw_i) = raw_i + corr[i]`, linear between points, raw outside
/// the rails clamped; all-zero corr is the 2-point-linear baseline.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PosLut {
    pub raw_min: u16,
    pub raw_max: u16,
    pub corr: [i16; N_POINTS],
}

impl PosLut {
    /// All-zero corr: linearize reduces to the plain linear fraction.
    pub fn identity(raw_min: u16, raw_max: u16) -> PosLut {
        PosLut {
            raw_min,
            raw_max,
            corr: [0; N_POINTS],
        }
    }

    /// Corrected raw value at point i: on-line position plus its correction.
    fn corrected_point(&self, i: usize) -> f64 {
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
            self.corrected_point(N_INTERVALS)
        } else {
            let frac = pos - i0 as f64;
            let c0 = self.corrected_point(i0);
            let c1 = self.corrected_point(i0 + 1);
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
    let mut prev = interp(&centers, 0.0);
    phase.push(0.0);
    for k in 1..n {
        let rs = interp(&centers, k as f64);
        acc += 0.5 * (prev + rs) / fs;
        phase.push(acc);
        prev = rs;
    }
    Some((phase, good_frac))
}

/// Fraction of the rail span [raw_min,raw_max] the sweep's raw pot samples
/// actually covered, from robust 2/98 percentiles of raw_pot rather than
/// absolute min/max: (p98 - p2) / (raw_max - raw_min), clamped to [0,1]. The
/// percentiles keep a couple of outlier samples near a rail from inflating a
/// partial sweep to full coverage. 0.0 on empty input or a zero/degenerate
/// denominator.
pub fn span_coverage(raw_pot: &[u16], raw_min: u16, raw_max: u16) -> f64 {
    if raw_pot.is_empty() {
        return 0.0;
    }
    let denom = raw_max as f64 - raw_min as f64;
    if denom <= 0.0 {
        return 0.0;
    }
    let mut sorted = raw_pot.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();
    let idx = |p: f64| ((n - 1) as f64 * p).round() as usize;
    let lo = sorted[idx(0.02)] as f64;
    let hi = sorted[idx(0.98)] as f64;
    ((hi - lo) / denom).clamp(0.0, 1.0)
}

/// Fit the 55-point correction table from `pts` = (raw, rel), where rel is in
/// [0,1] and ALREADY oriented to increase with raw. Anchors the covered pot
/// endpoints affinely into full-rail true-fraction space, pins the rails to
/// frac 0/1, forms a monotone raw->fraction curve, and sets each point's corr
/// to the count offset landing its corrected value on the curve. Degenerate
/// input (single raw value, <2 curve points) -> identity.
fn fit_points(mut pts: Vec<(f64, f64)>, raw_min: u16, raw_max: u16) -> PosLut {
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let span_f = raw_max as f64 - raw_min as f64;
    // Anchor rel into full-rail true-fraction space. rel spans 0..1 over only
    // the covered pot range [rp_lo,rp_hi]; map it affinely onto that range's
    // fractions of the FULL rail span, assuming local linearity over the small
    // uncovered insets. A full-rail sweep has rp_lo=raw_min, rp_hi=raw_max ->
    // tf_lo=0, tf_hi=1 -> the mapping is the identity (backward compatible).
    let rp_lo = pts[0].0;
    let rp_hi = pts[pts.len() - 1].0;
    let tf_lo = (rp_lo - raw_min as f64) / span_f;
    let tf_hi = (rp_hi - raw_min as f64) / span_f;
    if tf_hi == tf_lo {
        return PosLut::identity(raw_min, raw_max);
    }
    for p in pts.iter_mut() {
        p.1 = tf_lo + p.1 * (tf_hi - tf_lo);
    }
    // pin the mechanical rails to frac 0/1: raw_min/raw_max are the kinematics
    // endpoints, so linearize must return exactly 0/1 there. interp then links
    // each rail linearly to the nearest covered endpoint, tapering inset corr to
    // 0 at the rail. A full-rail sweep already carries tf_lo=0/tf_hi=1 there, so
    // the dedup pass averages the duplicate rail cleanly (behavior unchanged).
    pts.push((raw_min as f64, 0.0));
    pts.push((raw_max as f64, 1.0));
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
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
        return PosLut::identity(raw_min, raw_max);
    }
    // enforce non-decreasing frac (noise guard)
    for j in 1..curve.len() {
        if curve[j].1 < curve[j - 1].1 {
            curve[j].1 = curve[j - 1].1;
        }
    }
    let mut corr = [0i16; N_POINTS];
    for (k, c) in corr.iter_mut().enumerate() {
        let raw_i = raw_min as f64 + k as f64 * span_f / N_INTERVALS as f64;
        // uncovered-inset points lie on the interp segment from the rail anchor
        // (frac 0/1) to the nearest covered endpoint, so their corr tapers to 0
        // at the rail; the span_coverage gate above rejects sweeps whose
        // uncovered regions would grow that taper large.
        let frac = interp(&curve, raw_i);
        let corrected = raw_min as f64 + frac * span_f;
        *c = sat_i16((corrected - raw_i).round());
    }
    PosLut {
        raw_min,
        raw_max,
        corr,
    }
}

/// The `(pos, current)` chunks clocked by cumulative_phase and stitched; a
/// chunk whose ripple phase can't be recovered is skipped, not fatal.
fn stitch_tach(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> Option<Stitch> {
    let clocked: Vec<(&[u16], Vec<f64>)> = chunks
        .iter()
        .filter(|(pos, current)| pos.len() == current.len() && pos.len() >= MIN_SAMPLES)
        .filter_map(|(pos, current)| {
            Some((
                pos.as_slice(),
                cumulative_phase(current, fs, ripple_per_rev)?.0,
            ))
        })
        .collect();
    let refs: Vec<(&[u16], &[f64])> = clocked
        .iter()
        .map(|(p, phi)| (*p, phi.as_slice()))
        .collect();
    stitch(&refs, raw_min, raw_max)
}

/// Build the position table from MULTIPLE clean sweep chunks (each (pos,
/// current)), stitched over the shared pos axis. Chunks may cover overlapping
/// or disjoint pos ranges with gaps between them; the per-count slope
/// integration bridges the gaps. Identity when coverage < MIN_SPAN_COVER or
/// the stitch is degenerate.
pub fn build_multi(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> PosLut {
    let Some(st) = stitch_tach(chunks, fs, ripple_per_rev, raw_min, raw_max) else {
        return PosLut::identity(raw_min, raw_max);
    };
    if st.coverage() < MIN_SPAN_COVER {
        return PosLut::identity(raw_min, raw_max);
    }
    let total = st.cum[st.lc] - st.cum[st.fc];
    // (pos, rel), rel in [0,1] ascending with pos (no flip needed)
    let pts: Vec<(f64, f64)> = (st.fc..=st.lc)
        .map(|b| {
            (
                raw_min as f64 + b as f64,
                (st.cum[b] - st.cum[st.fc]) / total,
            )
        })
        .collect();
    fit_points(pts, raw_min, raw_max)
}

/// Full-travel motor revs + coverage from the stitched chunks, for the gear/
/// travel anchor. Extrapolates the covered-span revs to the full rail span.
pub fn stitched_motor_revs(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> Option<(f64, f64)> {
    let st = stitch_tach(chunks, fs, ripple_per_rev, raw_min, raw_max)?;
    let revs = st.cum[st.lc] - st.cum[st.fc];
    let pot_disp = (st.lc - st.fc) as f64;
    let count_span = raw_max as f64 - raw_min as f64;
    let full = crate::kinematics::full_span_motor_revs(revs, pot_disp, count_span);
    (full != 0.0).then_some((full, st.coverage()))
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

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    const RAW_MIN: u16 = 200;
    const RAW_MAX: u16 = 3900;
    const SPAN: f64 = (RAW_MAX - RAW_MIN) as f64;
    const FS: f64 = 20_000.0;

    // Monotonic nonlinear pot: raw = raw_min + span*(f + a*sin(pi*f)).
    // a < 1/pi keeps it monotone; endpoints fixed (sin(0)=sin(pi)=0).
    fn pot(f: f64, a: f64) -> f64 {
        RAW_MIN as f64 + SPAN * (f + a * (PI * f).sin())
    }

    fn pot_raw(f: f64, a: f64) -> u16 {
        pot(f, a).round() as u16
    }

    /// True angle fraction at a raw count, the inverse of `pot`.
    fn frac_at(raw: f64, a: f64) -> f64 {
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if pot(mid, a) < raw {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// Linearized counts of `raw` in the chord gauge through `lo` and `hi`.
    fn true_lin(raw: u16, a: f64, lo: u16, hi: u16) -> f64 {
        let (f0, f1) = (frac_at(lo as f64, a), frac_at(hi as f64, a));
        lo as f64 + (frac_at(raw as f64, a) - f0) / (f1 - f0) * (hi - lo) as f64
    }

    /// Largest `(error, raw)` of the table against the pot `a` over the points
    /// inside `band`, the stretch the table models.
    fn worst_error(lut: &GridLut, a: f64, band: (u16, u16)) -> (f64, u16) {
        let first = band.0.div_ceil(GRID) * GRID;
        let last = band.1 / GRID * GRID;
        (first..=last)
            .map(|raw| {
                (
                    (lut.counts(raw) - true_lin(raw, a, band.0, band.1)).abs(),
                    raw,
                )
            })
            .max_by(|x, y| x.0.total_cmp(&y.0))
            .expect("points in band")
    }

    #[test]
    fn identity_q4_is_raw_shifted_on_every_code() {
        for raw in 0..=ADC_MASK {
            assert_eq!(GridLut::IDENTITY.q4(raw), raw << GRID_SHIFT, "raw {raw}");
            assert_eq!(GridLut::IDENTITY.q4(raw | 0xF000), raw << GRID_SHIFT);
        }
        assert_eq!(GridLut::IDENTITY.counts(3849), 3849.0);
    }

    #[test]
    fn points_land_on_raw_plus_correction() {
        let mut lut = GridLut::IDENTITY;
        for (k, c) in lut.points.iter_mut().enumerate().take(200).skip(40) {
            *c = ((k as f64 * 0.37).sin() * 20.0) as i16;
        }
        assert_eq!(lut.validate(300, 3800), Ok(()));
        for k in 0..INTERVALS {
            let raw = (k * GRID as usize) as u16;
            assert_eq!(
                lut.q4(raw),
                ((raw as i32 + lut.points[k] as i32) << GRID_SHIFT) as u16
            );
        }
        // between points the Q4 word walks the chord, one interval gain per count
        let (c0, c1) = (lut.points[100] as i32, lut.points[101] as i32);
        for f in 0..=GRID {
            let want = lut.q4(1600) as i32 + (GRID as i32 + c1 - c0) * f as i32;
            assert_eq!(lut.q4(1600 + f) as i32, want, "raw {}", 1600 + f);
        }
    }

    #[test]
    fn validate_rejects_ends_and_shape() {
        assert_eq!(GridLut::IDENTITY.validate(209, 3849), Ok(()));
        assert_eq!(GridLut::IDENTITY.validate(0, 0), Ok(()));
        let mut lut = GridLut::IDENTITY;
        lut.points[14] = 1;
        // point 14 at raw 224 is inside the low inset of stop 209 (14 <= (209 + 15) >> 4)
        assert_eq!(lut.validate(209, 3849), Err(Reject::Ends));
        assert_eq!(lut.validate(200, 3849), Ok(()));
        assert_eq!(lut.validate(0, 0), Err(Reject::Ends));
        let mut lut = GridLut::IDENTITY;
        lut.points[100] = -16;
        assert_eq!(lut.validate(209, 3849), Err(Reject::Shape), "gain 0");
        lut.points[100] = -15;
        assert_eq!(lut.validate(209, 3849), Ok(()), "gain 1/16");
        // a step up to `top` then down 15 per point: only the step's gain moves
        let ramp = |top: i16| {
            let mut lut = GridLut::IDENTITY;
            for j in 0..=16 {
                lut.points[100 + j] = (top - 15 * j as i16).max(0);
            }
            lut
        };
        assert_eq!(ramp(239).validate(209, 3849), Ok(()), "gain 255/16");
        assert_eq!(ramp(240).validate(209, 3849), Err(Reject::Shape), "gain 16");
        // the last interval before the top stop
        let mut lut = GridLut::IDENTITY;
        lut.points[240] = 3;
        assert_eq!(
            lut.validate(209, 3849),
            Err(Reject::Ends),
            "240 >= 3849 >> 4"
        );
        lut.points[240] = 0;
        lut.points[239] = 3;
        assert_eq!(lut.validate(209, 3849), Ok(()));
    }

    #[test]
    fn image_round_trips_and_needs_the_grid() {
        let mut lut = GridLut::IDENTITY;
        lut.points[80] = -7;
        let img = lut.image(209, 3849, "mg90-a__2s", 99, (542, 3520), "a test");
        assert_eq!(img.points.len(), INTERVALS);
        let text = serde_json::to_string(&img).expect("json");
        let back: Image = serde_json::from_str(&text).expect("image");
        assert_eq!(back, img);
        assert_eq!(back.lut(), Some(lut));
        let mut off = img.clone();
        off.grid_shift = 5;
        assert_eq!(off.lut(), None);
        let mut short = img.clone();
        short.points.pop();
        assert_eq!(short.lut(), None);
        // the field order osc lut write reads
        assert!(text.starts_with(r#"{"raw_min":209,"raw_max":3849,"grid_shift":4,"points":["#));
        assert!(text.ends_with(
            r#"],"dataset":"mg90-a__2s","rungs":99,"covered":[542,3520],"source":"a test"}"#
        ));
    }

    /// `m` (pos, angle) chunks of a rigid train over the pot `a`, each
    /// covering `f0..f1` of travel with `n` samples, alternating direction;
    /// 3 revs per travel, a different angle origin per chunk.
    fn rigid_chunks(a: f64, m: usize, n: usize, f0: f64, f1: f64) -> Vec<(Vec<u16>, Vec<f64>)> {
        (0..m)
            .map(|i| {
                let rev = i % 2 == 1;
                let (mut pos, mut ang) = (Vec::with_capacity(n), Vec::with_capacity(n));
                for k in 0..n {
                    let s = k as f64 / (n - 1) as f64;
                    let f = if rev {
                        f1 - (f1 - f0) * s
                    } else {
                        f0 + (f1 - f0) * s
                    };
                    pos.push(pot_raw(f, a));
                    ang.push(3.0 * (if rev { 1.0 - f } else { f }) + 0.1 * i as f64);
                }
                (pos, ang)
            })
            .collect()
    }

    fn refs(chunks: &[(Vec<u16>, Vec<f64>)]) -> Vec<(&[u16], &[f64])> {
        chunks
            .iter()
            .map(|(p, c)| (p.as_slice(), c.as_slice()))
            .collect()
    }

    #[test]
    fn grid_follows_the_stitched_curve_inside_the_band() {
        let a = 0.15;
        let chunks = rigid_chunks(a, 24, 6000, 0.05, 0.95);
        let st = stitch(&refs(&chunks), RAW_MIN, RAW_MAX).expect("stitch");
        assert!(st.well_covered(25).is_none());
        let band = st.well_covered(MIN_RUNG_COVER).expect("band");
        assert_eq!(
            band,
            (st.lo(), st.hi()),
            "every chunk spans the same stretch"
        );
        assert!(
            (st.coverage() - 0.9).abs() < 0.02,
            "coverage {}",
            st.coverage()
        );
        let lut = fit_grid(&st, band);
        assert_eq!(lut.validate(RAW_MIN, RAW_MAX), Ok(()));
        // the pot is off by up to a*sin*span ~ 500 counts; over the points the
        // grid leaves the point rounding plus the half-count bin convention
        // times the 0.5..1.5x gain swing
        let worst = worst_error(&lut, a, band);
        assert!(worst.0 < 2.5, "worst {} counts at {}", worst.0, worst.1);
        assert!(
            lut.points.iter().any(|&c| c.abs() > 400),
            "{:?}",
            &lut.points[..]
        );
        // the chord gauge: the insets are identity up to the band
        assert!(lut.points[..index(band.0)].iter().all(|&c| c == 0));
        assert!(lut.points[index(band.1) + 1..].iter().all(|&c| c == 0));
    }

    #[test]
    fn stitch_bridges_a_gap_and_rejects_the_degenerate() {
        let a = 0.15;
        let mut chunks = rigid_chunks(a, 2, 3000, 0.1, 0.4);
        chunks.extend(rigid_chunks(a, 2, 3000, 0.6, 0.9));
        let st = stitch(&refs(&chunks), RAW_MIN, RAW_MAX).expect("stitch");
        assert_eq!(st.chunks_at(pot_raw(0.5, a)), 0);
        assert_eq!(st.chunks_at(pot_raw(0.25, a)), 2);
        assert_eq!(st.chunks_at(RAW_MIN), 0);
        let mut prev = st.angle(st.lo() as f64);
        for raw in st.lo()..=st.hi() {
            let now = st.angle(raw as f64);
            assert!(now >= prev, "angle falls at {raw}");
            prev = now;
        }
        assert!(
            (st.angle(st.hi() as f64) - 3.0 * 0.8).abs() < 0.05,
            "gap bridged"
        );
        assert_eq!(st.angle(0.0), 0.0);
        assert_eq!(st.angle(4095.0), st.angle(st.hi() as f64));
        assert!(stitch(&refs(&chunks), 500, 500).is_none());
        assert!(stitch(&[], RAW_MIN, RAW_MAX).is_none());
        let flat = (vec![1000u16; 10], vec![1.0; 10]);
        assert!(
            stitch(&refs(&[flat]), RAW_MIN, RAW_MAX).is_none(),
            "one count"
        );
        let short = (vec![1000u16, 1001, 1002], vec![0.0, 0.5, 1.0]);
        assert!(
            stitch(&refs(&[short]), RAW_MIN, RAW_MAX).is_none(),
            "under MIN_SAMPLES"
        );
    }

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

    /// A constant-duty rung over the pot `a`: the motor at a constant speed
    /// through `f0..f1` of travel, the ripple at `c` cycles per count of the
    /// straight-line pot, amp 5 over +-3 noise.
    fn ripple_rung(a: f64, n: usize, f0: f64, f1: f64, c: f64, seed: u64) -> (Vec<u16>, Vec<f64>) {
        let mut lcg = Lcg(seed);
        let cycles_per_f = c * SPAN;
        (0..n)
            .map(|k| {
                let s = k as f64 / (n - 1) as f64;
                let f = f0 + (f1 - f0) * s;
                let ph = cycles_per_f * (f - f0).abs();
                (pot_raw(f, a), 5.0 * (2.0 * PI * ph).sin() + lcg.next(3.0))
            })
            .unzip()
    }

    #[test]
    fn build_stitches_tracked_rungs_onto_the_grid() {
        // a pot off by up to 74 counts, the mg90-a class: a stronger bump
        // splits the two directions' whole-rung coupling past CPC_TOL
        let a = 0.02;
        let c = 0.1;
        let duties = [15, 20, 25, 30, 35, 40, 45, 50, 60, 20, 30];
        let data: Vec<(Vec<u16>, Vec<f64>, i32)> = (0..22)
            .map(|i| {
                let duty = duties[i % duties.len()] * if i % 2 == 1 { -1 } else { 1 };
                let (f0, f1) = if duty < 0 { (0.95, 0.05) } else { (0.05, 0.95) };
                let (pos, cur) = ripple_rung(a, 3000, f0, f1, c, 11 + i as u64);
                (pos, cur, duty)
            })
            .collect();
        let rungs: Vec<Rung> = data
            .iter()
            .map(|(pos, cur, duty)| Rung {
                pos,
                current: cur,
                duty_pct: *duty,
                tick_hz: FS,
            })
            .collect();
        let b = build(&rungs, RAW_MIN, RAW_MAX).expect("built");
        assert!((b.c_prior - c).abs() / c < 0.05, "prior {}", b.c_prior);
        let used = b.rungs();
        assert_eq!(b.verdicts.len(), rungs.len());
        for (r, v) in rungs.iter().zip(&b.verdicts) {
            if r.duty_pct.abs() == 60 {
                assert_eq!(*v, Verdict::OffCore);
            } else {
                assert_eq!(*v, Verdict::Used, "duty {}", r.duty_pct);
            }
        }
        assert_eq!(used, 20);
        assert_eq!(b.lut.validate(RAW_MIN, RAW_MAX), Ok(()));
        // every rung stitches from its trusted start, 200 samples in: the band
        // is where both directions' cut rungs still overlap
        let cut = 0.9 * 200.0 / 2999.0;
        let (lo, hi) = b.covered;
        assert!(
            (lo as f64 - pot(0.05 + cut, a)).abs() < 3.0
                && (hi as f64 - pot(0.95 - cut, a)).abs() < 3.0,
            "band {lo}..{hi}"
        );
        let worst = worst_error(&b.lut, a, b.covered);
        assert!(worst.0 < 2.0, "worst {} counts at {}", worst.0, worst.1);
        assert!(
            b.lut.points.iter().any(|&c| c.abs() > 40),
            "{:?}",
            &b.lut.points[..]
        );
        let img = b
            .lut
            .image(RAW_MIN, RAW_MAX, "synthetic", used, b.covered, "test");
        assert_eq!(img.covered, [lo, hi]);
        assert_eq!(img.lut(), Some(b.lut));

        // fewer than MIN_RUNG_COVER accepted rungs anchor nothing
        assert_eq!(
            build(&rungs[..4], RAW_MIN, RAW_MAX).err(),
            Some(BuildError::Uncovered)
        );
        // no 20 to 40% rung, no prior
        let hot: Vec<Rung> = rungs
            .iter()
            .map(|r| Rung {
                duty_pct: 60 * r.duty_pct.signum(),
                ..*r
            })
            .collect();
        assert_eq!(
            build(&hot, RAW_MIN, RAW_MAX).err(),
            Some(BuildError::NoPrior)
        );
        // pure noise never tracks
        let mut lcg = Lcg(3);
        let noise: Vec<f64> = (0..3000).map(|_| lcg.next(3.0)).collect();
        let deaf: Vec<Rung> = rungs
            .iter()
            .map(|r| Rung {
                current: &noise,
                ..*r
            })
            .collect();
        assert!(matches!(
            build(&deaf, RAW_MIN, RAW_MAX),
            Err(BuildError::NoPrior | BuildError::Uncovered)
        ));
    }

    /// The mg90-a__2s captures the notebook built its table from: the grid
    /// block of every session capture and the ends capture, each rung as
    /// (pos, current minus the recording's torque-off bias, duty).
    mod mg90 {
        use std::io::{BufRead, BufReader};

        const DATASET: &str = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../notebooks/telemetry/mg90-a__2s"
        );

        pub struct Recorded {
            pub cap: u32,
            pub seg: u32,
            pub duty_pct: i32,
            pub pos: Vec<u16>,
            pub current: Vec<f64>,
        }

        fn duty_pct(q15: i32) -> i32 {
            (q15 as f64 * 100.0 / 32767.0).round() as i32
        }

        fn rows(path: &str) -> Vec<(u32, i32, u16, u16)> {
            let file = std::fs::File::open(path).unwrap_or_else(|e| panic!("{path}: {e}"));
            let mut lines = BufReader::new(flate2::read::GzDecoder::new(file)).lines();
            let header = lines.next().expect("header").expect("header");
            let col = |name: &str| header.split(',').position(|c| c == name).expect(name);
            let (seg, q15, pos, cur) = (
                col("seg"),
                col("cmd_duty_q15"),
                col("pos"),
                col("current_raw"),
            );
            lines
                .map(|l| {
                    let l = l.expect("line");
                    let f: Vec<&str> = l.split(',').collect();
                    (
                        f[seg].parse().expect("seg"),
                        f[q15].parse().expect("cmd_duty_q15"),
                        f[pos].parse().expect("pos"),
                        f[cur].parse().expect("current_raw"),
                    )
                })
                .collect()
        }

        /// Every rung of one recording whose seg the filter keeps, in seg order.
        fn rungs(path: &str, cap: u32, keep: impl Fn(u32) -> bool) -> Vec<Recorded> {
            let rows = rows(path);
            let base: Vec<f64> = rows
                .iter()
                .filter(|r| r.0 == 0)
                .map(|r| r.3 as f64)
                .collect();
            let bias = base.iter().sum::<f64>() / base.len() as f64;
            let mut out: Vec<Recorded> = Vec::new();
            for &(seg, q15, pos, cur) in &rows {
                if seg == 0 || !keep(seg) {
                    continue;
                }
                match out.last_mut() {
                    Some(r) if r.seg == seg => {
                        assert_eq!(duty_pct(q15), r.duty_pct, "chained flip");
                        r.pos.push(pos);
                        r.current.push(cur as f64 - bias);
                    }
                    _ => out.push(Recorded {
                        cap,
                        seg,
                        duty_pct: duty_pct(q15),
                        pos: vec![pos],
                        current: vec![cur as f64 - bias],
                    }),
                }
            }
            assert!(
                out.windows(2).all(|w| w[0].seg < w[1].seg),
                "{path}: segs out of order"
            );
            out
        }

        pub fn session_grid(cap: u32) -> Vec<Recorded> {
            let dir = format!("{DATASET}/session/capture-{cap}");
            let meta: serde_json::Value = serde_json::from_str(
                &std::fs::read_to_string(format!("{dir}/slow.meta.json")).expect("meta"),
            )
            .expect("meta");
            let n = meta["schedule"].as_array().expect("schedule").len() as u32;
            let dirs = meta["dirs"].as_array().expect("dirs").len() as u32;
            let grid = meta["session"]["blocks"]
                .as_array()
                .expect("blocks")
                .iter()
                .find(|b| b["name"] == "grid")
                .expect("grid block");
            let (first, count) = (
                grid["first"].as_u64().unwrap() as u32,
                grid["count"].as_u64().unwrap() as u32,
            );
            let step = |seg: u32| {
                (0..dirs).find_map(|d| {
                    let k = seg.checked_sub(1 + d * n + first)?;
                    (k < count).then_some(1 + d * count + k)
                })
            };
            let mut out = rungs(&format!("{dir}/slow.csv.gz"), cap, |seg| {
                step(seg).is_some()
            });
            // numbered as the notebook's grid view numbers them: a sweep of the block alone
            for r in &mut out {
                r.seg = step(r.seg).expect("grid seg");
            }
            out
        }

        pub fn ends(cap: u32) -> Vec<Recorded> {
            rungs(
                &format!("{DATASET}/ends/capture-{cap}/sweep.csv.gz"),
                cap,
                |_| true,
            )
        }
    }

    /// Parity with the notebook build: the same rungs in the same order give
    /// the committed prior, verdicts, band and points.
    #[test]
    fn build_matches_the_notebook_on_mg90_a() {
        let t0 = std::time::Instant::now();
        let mut recorded: Vec<(&str, mg90::Recorded)> = Vec::new();
        for cap in 1..=5 {
            recorded.extend(mg90::session_grid(cap).into_iter().map(|r| ("session", r)));
        }
        for cap in 1..=5 {
            recorded.extend(mg90::ends(cap).into_iter().map(|r| ("ends", r)));
        }
        let loaded = t0.elapsed();
        let r: serde_json::Value = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/lut/mg90-a-rungs.json"
        )))
        .expect("reference");
        let tick_hz = r["tick_hz"].as_f64().unwrap();
        let (raw_min, raw_max) = (
            r["raw_min"].as_u64().unwrap() as u16,
            r["raw_max"].as_u64().unwrap() as u16,
        );
        let rungs: Vec<Rung> = recorded
            .iter()
            .map(|(_, r)| Rung {
                pos: &r.pos,
                current: &r.current,
                duty_pct: r.duty_pct,
                tick_hz,
            })
            .collect();
        let b = build(&rungs, raw_min, raw_max).expect("built");
        let built = t0.elapsed() - loaded;

        let c_ref = r["c_prior"].as_f64().unwrap();
        assert!(
            ((b.c_prior - c_ref) / c_ref).abs() < 1e-12,
            "prior {} vs {c_ref}",
            b.c_prior
        );
        let expect = r["rungs"].as_array().unwrap();
        let mut seen = 0;
        for ((set, rec), v) in recorded.iter().zip(&b.verdicts) {
            let Some(e) = expect
                .iter()
                .find(|e| e["set"] == *set && e["cap"] == rec.cap && e["seg"] == rec.seg)
            else {
                assert_eq!(
                    *v,
                    Verdict::OffCore,
                    "{set} {} seg {} duty {}",
                    rec.cap,
                    rec.seg,
                    rec.duty_pct
                );
                continue;
            };
            seen += 1;
            assert_eq!(e["duty"], rec.duty_pct);
            assert_eq!(e["n"].as_u64().unwrap() as usize, rec.pos.len());
            assert_eq!(
                *v == Verdict::Used,
                e["used"].as_bool().unwrap(),
                "{set} {} seg {} duty {}: {v:?}, notebook accept {} cpc {}",
                rec.cap,
                rec.seg,
                rec.duty_pct,
                e["accept"],
                e["cpc"]
            );
            if let Verdict::Short(accept) = v {
                assert!((accept - e["accept"].as_f64().unwrap()).abs() < 1e-12);
            }
        }
        assert_eq!(seen, expect.len());
        assert_eq!(b.rungs(), 99);
        let covered = r["covered"].as_array().unwrap();
        assert_eq!(
            b.covered,
            (
                covered[0].as_u64().unwrap() as u16,
                covered[1].as_u64().unwrap() as u16
            )
        );

        let committed = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/testdata/lut/pos-lut-mg90-a-grid.json"
        ));
        let want: Image = serde_json::from_str(committed).expect("committed image");
        let img = b.lut.image(
            raw_min,
            raw_max,
            &want.dataset,
            b.rungs(),
            b.covered,
            &want.source,
        );
        let off: Vec<(usize, i16, i16)> = img
            .points
            .iter()
            .zip(&want.points)
            .enumerate()
            .filter(|(_, (a, b))| a != b)
            .map(|(k, (a, b))| (k, *a, *b))
            .collect();
        assert!(
            off.is_empty(),
            "{} points differ (point, built, notebook): {off:?}",
            off.len()
        );
        assert_eq!(img, want);
        assert_eq!(
            serde_json::to_string_pretty(&img).expect("json") + "\n",
            committed
        );
        assert_eq!(b.lut.validate(209, 3849), Ok(()));
        assert_eq!(b.lut.validate(232, 3849), Ok(()));
        eprintln!(
            "mg90-a: {} rungs loaded in {loaded:.1?}, built in {built:.1?}, prior {:.6}, band {:?}",
            rungs.len(),
            b.c_prior,
            b.covered
        );
    }

    // --- the 55-point block ---

    fn sweep(a: f64, m: usize) -> (Vec<u16>, Vec<f64>) {
        let mut raw = Vec::with_capacity(m);
        let mut phase = Vec::with_capacity(m);
        for k in 0..m {
            let f = k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, a));
            phase.push(3.0 * f);
        }
        (raw, phase)
    }

    #[test]
    fn identity_linearize_is_linear_fraction() {
        let lut = PosLut::identity(RAW_MIN, RAW_MAX);
        for raw in [RAW_MIN, 1000, 2048, 3000, RAW_MAX] {
            let expect = (raw as f64 - RAW_MIN as f64) / SPAN;
            assert!((lut.linearize(raw) - expect).abs() < 1e-12, "raw {raw}");
        }
        assert_eq!(lut.angle_cdeg(RAW_MIN, 0, 19000), 0.0);
        assert!((lut.angle_cdeg(RAW_MAX, 0, 19000) - 19000.0).abs() < 1e-9);
        assert_eq!(lut.linearize(0), 0.0);
        assert_eq!(lut.linearize(u16::MAX), 1.0);
        assert_eq!(PosLut::identity(500, 500).linearize(500), 0.0);
    }

    #[test]
    fn fit_points_recovers_a_nonlinear_pot() {
        let a = 0.15;
        let (raw, phase) = sweep(a, 400);
        let pts: Vec<(f64, f64)> = raw
            .iter()
            .zip(&phase)
            .map(|(&r, &p)| (r as f64, p / 3.0))
            .collect();
        let lut = fit_points(pts, RAW_MIN, RAW_MAX);
        let ident = PosLut::identity(RAW_MIN, RAW_MAX);
        let mut lut_max = 0.0f64;
        let mut ident_max = 0.0f64;
        for (k, &r) in raw.iter().enumerate() {
            let f = k as f64 / 399.0;
            lut_max = lut_max.max((lut.linearize(r) - f).abs());
            ident_max = ident_max.max((ident.linearize(r) - f).abs());
        }
        assert!(lut_max < 0.01, "lut err {lut_max}");
        assert!(
            ident_max > 0.1 && lut_max < ident_max / 10.0,
            "lut {lut_max} ident {ident_max}"
        );
        assert_eq!(
            fit_points(vec![(1000.0, 0.0), (1000.0, 1.0)], RAW_MIN, RAW_MAX),
            ident
        );
    }

    #[test]
    fn span_coverage_fraction_and_degenerate() {
        assert!((span_coverage(&[RAW_MIN, RAW_MAX], RAW_MIN, RAW_MAX) - 1.0).abs() < 1e-12);
        let q1 = RAW_MIN + (SPAN * 0.25) as u16;
        let q3 = RAW_MIN + (SPAN * 0.75) as u16;
        assert!((span_coverage(&[q1, q3], RAW_MIN, RAW_MAX) - 0.5).abs() < 0.01);
        assert_eq!(span_coverage(&[], RAW_MIN, RAW_MAX), 0.0);
        assert_eq!(span_coverage(&[500, 600], 500, 500), 0.0);
        // outliers at the rails do not inflate a middle-of-travel sweep
        let mut raw: Vec<u16> = (0..300)
            .map(|k| pot_raw(0.3 + 0.4 * k as f64 / 299.0, 0.0))
            .collect();
        raw.push(RAW_MIN + 1);
        raw.push(RAW_MAX - 1);
        let cov = span_coverage(&raw, RAW_MIN, RAW_MAX);
        assert!(cov > 0.35 && cov < 0.45, "coverage {cov}");
    }

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
        assert!(phase.windows(2).all(|w| w[1] >= w[0]));
        // 1800 Hz / 6 = 300 rev/s over (n-1)/fs s
        let expect = 300.0 * (n - 1) as f64 / FS;
        let got = phase[n - 1];
        assert!(
            (got - expect).abs() / expect < 0.03,
            "revs {got} vs {expect}"
        );
        let mut lcg = Lcg(9);
        let noise: Vec<f64> = (0..4000).map(|_| 512.0 + lcg.next(3.0)).collect();
        assert!(cumulative_phase(&noise, FS, 6.0).is_none(), "pure noise");
        assert!(cumulative_phase(&[1.0; 10], FS, 6.0).is_none(), "too short");
    }

    // Full nonlinear sweep with a CONSTANT-frequency ripple current: constant
    // duty => constant motor speed => true fraction f linear in sample index.
    fn ripple_sweep(a: f64, freq: f64, n: usize) -> (Vec<u16>, Vec<f64>) {
        let pos: Vec<u16> = (0..n)
            .map(|k| pot_raw(k as f64 / (n - 1) as f64, a))
            .collect();
        (pos, ripple_series(freq, 4.0, 3.0, n))
    }

    fn chunkify(pos: &[u16], cur: &[f64], ranges: &[(usize, usize)]) -> Vec<(Vec<u16>, Vec<f64>)> {
        ranges
            .iter()
            .map(|&(lo, hi)| (pos[lo..hi].to_vec(), cur[lo..hi].to_vec()))
            .collect()
    }

    const RANGES: [(usize, usize); 5] = [
        (0, 850),
        (950, 1750),
        (1850, 2700),
        (2800, 3600),
        (3700, 4500),
    ];

    #[test]
    fn build_multi_recovers_full_sweep_from_chunks() {
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        let chunks = chunkify(&pos, &cur, &RANGES);
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        let ident = PosLut::identity(RAW_MIN, RAW_MAX);
        assert!(
            lut.corr.iter().any(|&c| c != 0),
            "corr all zero {:?}",
            lut.corr
        );
        let mut lut_max = 0.0f64;
        let mut ident_max = 0.0f64;
        for &(lo, hi) in RANGES.iter() {
            for (j, &r) in pos[lo..hi].iter().enumerate() {
                let f = (lo + j) as f64 / (n - 1) as f64;
                lut_max = lut_max.max((lut.linearize(r) - f).abs());
                ident_max = ident_max.max((ident.linearize(r) - f).abs());
            }
        }
        assert!(lut_max < 0.025, "lut err {lut_max}");
        assert!(lut_max < ident_max / 5.0, "lut {lut_max} ident {ident_max}");
        // chunks covering only the middle ~40% of travel -> below the gate
        let lo = (0.30 * (n - 1) as f64) as usize;
        let mid = (0.50 * (n - 1) as f64) as usize;
        let hi = (0.70 * (n - 1) as f64) as usize;
        let chunks = chunkify(&pos, &cur, &[(lo, mid), (mid, hi)]);
        assert_eq!(build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX), ident);
        assert_eq!(build_multi(&[], FS, 6.0, RAW_MIN, RAW_MAX), ident);
        assert!(stitched_motor_revs(&[], FS, 6.0, RAW_MIN, RAW_MAX).is_none());
    }

    #[test]
    fn stitched_motor_revs_recovers_total() {
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        let chunks = chunkify(&pos, &cur, &RANGES);
        let (full, cov) = stitched_motor_revs(&chunks, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        // 1800 Hz / 6 = 300 rev/s over N/FS s
        let total = 300.0 * n as f64 / FS;
        assert!(
            (full - total).abs() / total < 0.05,
            "revs {full} vs {total}"
        );
        assert!(cov > MIN_SPAN_COVER && cov <= 1.0, "coverage {cov}");
    }

    // Oversampled + jittered sweep: pos advances well under 1 count/sample (the
    // pot ADC lands many samples on each count) and dithers +/-1 count. Mirrors
    // the real capture that undercounted revs ~oversampling-fold before the
    // per-chunk pos dedup in the stitch.
    fn jitter_ripple_sweep(a: f64, freq: f64, n: usize) -> (Vec<u16>, Vec<f64>) {
        let mut lcg = Lcg(7);
        let pos: Vec<u16> = (0..n)
            .map(|k| {
                let f = k as f64 / (n - 1) as f64;
                let j = lcg.next(1.5).round() as i32;
                (pot_raw(f, a) as i32 + j).clamp(RAW_MIN as i32, RAW_MAX as i32) as u16
            })
            .collect();
        (pos, ripple_series(freq, 4.0, 3.0, n))
    }

    #[test]
    fn stitched_motor_revs_survives_oversampling_and_jitter() {
        let a = 0.15;
        // ~8 samples per count over the 3700-count span
        let n = 30_000;
        let (pos, cur) = jitter_ripple_sweep(a, 1800.0, n);
        let total = 300.0 * n as f64 / FS;
        let single = vec![(pos.clone(), cur.clone())];
        let (full1, cov1) = stitched_motor_revs(&single, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        assert!(
            (full1 - total).abs() / total < 0.05,
            "single {full1} vs {total}"
        );
        assert!(cov1 > MIN_SPAN_COVER && cov1 <= 1.0, "coverage {cov1}");
        let ranges = [
            (0, 6000),
            (6500, 12000),
            (12500, 18000),
            (18500, 24000),
            (24500, 30000),
        ];
        let chunks = chunkify(&pos, &cur, &ranges);
        let (fullc, _) = stitched_motor_revs(&chunks, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        assert!(
            (fullc - total).abs() / total < 0.05,
            "chunked {fullc} vs {total}"
        );
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        let mut emax = 0.0f64;
        for &(lo, hi) in ranges.iter() {
            for (j, &r) in pos[lo..hi].iter().enumerate() {
                let f = (lo + j) as f64 / (n - 1) as f64;
                emax = emax.max((lut.linearize(r) - f).abs());
            }
        }
        assert!(emax < 0.03, "linearize err {emax}");
    }
}
