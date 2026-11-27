//! The position table from the motor's commutation ripple: on open-loop
//! one-way rungs the ripple counts the motor's turn, so the pot read
//! against it gives the pot's shape with no other sensor. Each rung's
//! crossings are the ruler. The table is one fit over every rung of both
//! directions on the firmware's 16-count grid (one offset a rung, a
//! second-difference penalty), continued straight to the stops and pinned
//! so both map to themselves. Each direction fitted alone, and the mean of
//! the two shapes, are the report's measure of how far the directions
//! differ. Angles are in commutation cycles, rising with the pot.

use osc_servo_core::adc::ADC_WIDE_SLOTS;
use osc_servo_core::pos_lut::{GRID, INTERVALS};

use super::widecodes::compensated;
use super::{interp, polyfit, polyval, solve_spd};
use crate::ripple::{
    BAND_HZ, REPAIR_LONG, REPAIR_SHORT, REPAIR_WINDOW, TRACK_CYCLES, line_freq, rolling_median,
    tracked_crossings,
};

/// s after the drive's edge a rung's rows start: the spin-up.
pub const SPIN_UP_S: f64 = 0.1;
/// Counts the pot moves from the edge before a rung's rows start: the
/// gear play taken up.
pub const PLAY_COUNTS: f64 = 24.0;
/// s at a rung's end left out.
pub const TAIL_S: f64 = 0.002;
/// Rows a sample, and the pot's centered mean at it.
pub const SAMPLE_ROWS: usize = 16;
/// Rows in the middle of a rung its ripple line is read from.
pub const SEED_ROWS: usize = 4000;
/// A rung with more of its crossing intervals off the repair bounds than
/// this share is repeated, then refused.
pub const ODD_MAX: f64 = 0.02;
/// The smoothing weight, in samples an interval.
pub const SMOOTH_WEIGHT: f64 = 1.0;
/// Intervals in a row a direction may leave without samples inside the
/// common span.
pub const HOLE_MAX: usize = 4;
/// Counts either side of a slope reading: 48-count slopes.
pub const SLOPE_HALF: f64 = 24.0;
/// Counts between slope readings.
pub const SLOPE_STEP: f64 = 4.0;
/// Counts a band of the direction term covers.
pub const BAND_COUNTS: f64 = 256.0;
/// Two same-direction rungs' cycles over the counts they share agree
/// within this share of the travel's cycles, the table's from stop to
/// stop (plan sec 2.4).
pub const AGREE_SHARE: f64 = 0.003;

const GRID_F: f64 = GRID as f64;

/// One rung read against its ripple: time-ordered samples of the pot and
/// the motor's angle.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    /// +1 when the pot rose.
    pub dir: i8,
    /// The compensated pot's centered mean at each sample, counts.
    pub x: Vec<f64>,
    /// `dir` x the cycles since the first kept crossing.
    pub y: Vec<f64>,
    /// The line the count was seeded on, Hz.
    pub f_seed: f64,
    pub crossings: usize,
    /// The share of crossing intervals outside the repair bounds of the
    /// running median.
    pub odd: f64,
    /// The line's spread over the rung's four quarters, (max - min) / mean.
    pub speed_spread: f64,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SegmentError {
    NoDrive,
    /// The pot never moved [`PLAY_COUNTS`] from the edge.
    Still,
    Short,
    NoLine,
    FewCrossings(usize),
}

/// Where a rung's ripple count is seeded.
#[derive(Copy, Clone)]
pub enum Seed<'a> {
    /// The strongest line inside [`BAND_HZ`] over [`SEED_ROWS`] in the
    /// middle of the rung.
    Middle,
    /// The tallest line within [`SEED_WINDOW_SHARE`] of the pot's speed
    /// over the first [`SEED_ROWS`] tracked, over the counts a cycle this
    /// gives at the pot there: a slow rung's, its counts a cycle from
    /// rungs already counted.
    Near(&'a dyn Fn(f64) -> f64),
    /// This line, Hz: a rung read again on the line an earlier read of it
    /// was seeded on.
    Line(f64),
}

/// The share either side of the line the pot's speed implies a seed is
/// looked for in.
pub const SEED_WINDOW_SHARE: f64 = 0.3;

/// A rung's rows (raw pot codes, raw current, signed duty) read against
/// the ripple in its current, the pot given `slots`, rows at `fs`, seeded
/// at the middle.
pub fn segment(
    pos: &[u16],
    current_raw: &[u16],
    duty: &[i16],
    slots: &[u8; ADC_WIDE_SLOTS],
    fs: f64,
) -> Result<Segment, SegmentError> {
    segment_seeded(pos, current_raw, duty, slots, fs, Seed::Middle)
}

/// [`segment`] with the seed chosen by `seed`.
pub fn segment_seeded(
    pos: &[u16],
    current_raw: &[u16],
    duty: &[i16],
    slots: &[u8; ADC_WIDE_SLOTS],
    fs: f64,
    seed: Seed,
) -> Result<Segment, SegmentError> {
    let n = pos.len().min(current_raw.len()).min(duty.len());
    let edge = duty[..n]
        .iter()
        .position(|&d| d != 0)
        .ok_or(SegmentError::NoDrive)?;
    let p: Vec<f64> = pos[..n].iter().map(|&c| compensated(c, slots)).collect();
    let moved = (edge..n)
        .find(|&i| (p[i] - p[edge]).abs() >= PLAY_COUNTS)
        .ok_or(SegmentError::Still)?;
    let start = moved.max(edge + (SPIN_UP_S * fs).round() as usize);
    let stop = n.saturating_sub((TAIL_S * fs).round() as usize);
    if start + 2 * SAMPLE_ROWS >= stop {
        return Err(SegmentError::Short);
    }
    let x: Vec<f64> = current_raw[..n].iter().map(|&v| v as f64).collect();
    let f_seed = match seed {
        Seed::Middle => {
            let half = (SEED_ROWS / 2).min(n / 2);
            line_freq(&x[n / 2 - half..n / 2 + half], BAND_HZ.0, BAND_HZ.1, fs)
        }
        Seed::Near(counts_per_cycle) => {
            let b = (start + SEED_ROWS).min(stop);
            let cps = (p[b - 1] - p[start]).abs() / ((b - 1 - start) as f64 / fs);
            let f = cps / counts_per_cycle((p[b - 1] + p[start]) / 2.0);
            (f.is_finite() && f > 0.0)
                .then(|| {
                    line_freq(
                        &x[start..b],
                        (1.0 - SEED_WINDOW_SHARE) * f,
                        (1.0 + SEED_WINDOW_SHARE) * f,
                        fs,
                    )
                })
                .flatten()
        }
        Seed::Line(f) => Some(f),
    }
    .ok_or(SegmentError::NoLine)?;
    let tc = tracked_crossings(&x, start, stop as f64, f_seed, TRACK_CYCLES, fs);
    if tc.len() < 3 {
        return Err(SegmentError::FewCrossings(tc.len()));
    }
    let boxed = centered_mean(&p, SAMPLE_ROWS);
    let index: Vec<f64> = (0..tc.len()).map(|k| k as f64).collect();
    let last = tc[tc.len() - 1];
    let rows: Vec<usize> = (tc[0].ceil() as usize..)
        .step_by(SAMPLE_ROWS)
        .take_while(|&r| (r as f64) < last)
        .collect();
    if rows.len() < 2 {
        return Err(SegmentError::Short);
    }
    let rise = boxed[rows[rows.len() - 1]] - boxed[rows[0]];
    if rise == 0.0 {
        return Err(SegmentError::Still);
    }
    let dir: i8 = if rise > 0.0 { 1 } else { -1 };
    let xs = rows.iter().map(|&r| boxed[r]).collect();
    let ys = rows
        .iter()
        .map(|&r| dir as f64 * interp(r as f64, &tc, &index, 0.0, 0.0))
        .collect();
    Ok(Segment {
        dir,
        x: xs,
        y: ys,
        f_seed,
        crossings: tc.len(),
        odd: odd_share(&tc),
        speed_spread: speed_spread(&tc),
    })
}

/// One whole ripple cycle of a segment: the pot at cycles `k` and `k + 1`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cycle {
    pub k: usize,
    pub from: f64,
    pub to: f64,
}

impl Cycle {
    /// Pot counts over the cycle.
    pub fn counts(&self) -> f64 {
        (self.to - self.from).abs()
    }

    pub fn lo(&self) -> f64 {
        self.from.min(self.to)
    }

    pub fn hi(&self) -> f64 {
        self.from.max(self.to)
    }
}

/// Each whole cycle the segment spans, the pot read at whole cycle
/// indices on its samples.
pub fn cycles(s: &Segment) -> Vec<Cycle> {
    let y: Vec<f64> = s.y.iter().map(|v| v.abs()).collect();
    let (Some(&first), Some(&last)) = (y.first(), y.last()) else {
        return Vec::new();
    };
    let (k0, k1) = (first.ceil() as usize, last.floor() as usize);
    if k1 < k0 + 1 {
        return Vec::new();
    }
    let x: Vec<f64> = (k0..=k1)
        .map(|k| interp(k as f64, &y, &s.x, s.x[0], s.x[s.x.len() - 1]))
        .collect();
    x.windows(2)
        .enumerate()
        .map(|(j, w)| Cycle {
            k: k0 + j,
            from: w[0],
            to: w[1],
        })
        .collect()
}

/// The mean of `w` rows centered on each, `w / 2` before and `w / 2 - 1`
/// after on an even `w`, as pandas' centered rolling mean, fewer at the
/// ends.
fn centered_mean(v: &[f64], w: usize) -> Vec<f64> {
    let n = v.len();
    let mut sum = vec![0.0; n + 1];
    for i in 0..n {
        sum[i + 1] = sum[i] + v[i];
    }
    (0..n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(w / 2), (i + w.div_ceil(2)).min(n));
            (sum[b] - sum[a]) / (b - a) as f64
        })
        .collect()
}

fn odd_share(tc: &[f64]) -> f64 {
    let iv: Vec<f64> = tc.windows(2).map(|w| w[1] - w[0]).collect();
    let med = rolling_median(&iv, REPAIR_WINDOW);
    let odd = iv
        .iter()
        .zip(&med)
        .filter(|&(d, m)| *d < REPAIR_SHORT * m || *d > REPAIR_LONG * m)
        .count();
    odd as f64 / iv.len() as f64
}

fn speed_spread(tc: &[f64]) -> f64 {
    let q = (tc.len() - 1) / 4;
    if q == 0 {
        return 0.0;
    }
    let f: Vec<f64> = (0..4)
        .map(|j| q as f64 / (tc[(j + 1) * q] - tc[j * q]))
        .collect();
    let (lo, hi) = f
        .iter()
        .fold((f64::INFINITY, 0.0f64), |(l, h), &v| (l.min(v), h.max(v)));
    (hi - lo) / (f.iter().sum::<f64>() / 4.0)
}

/// Grid points `16 k0 ..= 16 k1`, counts.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Span {
    pub k0: usize,
    pub k1: usize,
}

impl Span {
    pub fn count(&self) -> usize {
        self.k1 - self.k0 + 1
    }

    pub fn lo(&self) -> f64 {
        (self.k0 * GRID) as f64
    }

    pub fn hi(&self) -> f64 {
        (self.k1 * GRID) as f64
    }

    pub fn knots(&self) -> Vec<f64> {
        (self.k0..=self.k1).map(|k| (k * GRID) as f64).collect()
    }

    /// Where the 48-count slopes are read: every [`SLOPE_STEP`] counts
    /// [`SLOPE_HALF`] inside each end.
    pub fn slope_grid(&self) -> Vec<f64> {
        let (a, b) = (self.lo() + SLOPE_HALF, self.hi() - SLOPE_HALF);
        (0..)
            .map(|j| a + j as f64 * SLOPE_STEP)
            .take_while(|&x| x <= b)
            .collect()
    }

    /// The 48-count slopes of `angle` at its knots, cycles a count, at
    /// [`Span::slope_grid`].
    pub fn slopes(&self, angle: &[f64]) -> Vec<f64> {
        let k = self.knots();
        let at = |x: f64| interp(x, &k, angle, angle[0], angle[angle.len() - 1]);
        self.slope_grid()
            .iter()
            .map(|&x| (at(x + SLOPE_HALF) - at(x - SLOPE_HALF)) / (2.0 * SLOPE_HALF))
            .collect()
    }

    /// The share of the stops' span it covers.
    pub fn coverage(&self, raw_min: u16, raw_max: u16) -> f64 {
        (self.hi() - self.lo()) / (raw_max as f64 - raw_min as f64)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TableError {
    /// No rung in one direction.
    OneWay,
    /// The directions share under three grid points.
    NoSpan,
    /// No rung of direction `dir` (0 any direction) on the span.
    NoSamples(i8),
    Singular,
    /// `intervals` in a row without samples in direction `dir` (0 the
    /// pooled samples) from `from` counts.
    Hole {
        dir: i8,
        from: f64,
        intervals: usize,
    },
    SpanMismatch,
}

/// The least and the most counts the `dir` rungs' samples reach.
pub fn reach(segments: &[Segment], dir: i8) -> Option<(f64, f64)> {
    segments
        .iter()
        .filter(|s| s.dir == dir)
        .flat_map(|s| s.x.iter().copied())
        .fold(None, |acc: Option<(f64, f64)>, v| {
            Some(acc.map_or((v, v), |(l, h)| (l.min(v), h.max(v))))
        })
}

/// The grid points strictly inside `lo..hi` counts.
fn inside_span(lo: f64, hi: f64) -> Result<Span, TableError> {
    let k0 = (lo / GRID_F).ceil() as i64 + 1;
    let k1 = (hi / GRID_F).floor() as i64 - 1;
    if k0 < 0 || k1 < k0 + 2 {
        return Err(TableError::NoSpan);
    }
    Ok(Span {
        k0: k0 as usize,
        k1: k1 as usize,
    })
}

/// The grid points strictly inside the counts both directions cover, each
/// direction's rungs pooled.
pub fn common_span(segments: &[Segment]) -> Result<Span, TableError> {
    let (Some(up), Some(down)) = (reach(segments, 1), reach(segments, -1)) else {
        return Err(TableError::OneWay);
    };
    inside_span(up.0.max(down.0), up.1.min(down.1))
}

/// The grid points strictly inside the counts any rung covers, a rung in
/// each direction required.
pub fn pooled_span(segments: &[Segment]) -> Result<Span, TableError> {
    let (Some(up), Some(down)) = (reach(segments, 1), reach(segments, -1)) else {
        return Err(TableError::OneWay);
    };
    inside_span(up.0.min(down.0), up.1.max(down.1))
}

/// One direction's shape on a span.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectionFit {
    pub dir: i8,
    pub span: Span,
    /// Cycles at the knots, the first rung's level.
    pub angle: Vec<f64>,
    /// (the rung's place in the slice fitted, its offset in cycles); the
    /// first 0, rungs with no sample on the span left out.
    pub offsets: Vec<(usize, f64)>,
    /// Cycles rms.
    pub residual_rms: f64,
    pub samples: usize,
    /// Samples in each interval.
    pub per_interval: Vec<usize>,
}

/// The `dir` rungs of `segments` fitted alone: the angle at the span's
/// knots (hat functions) and one offset a rung, the first held at 0, under
/// a second-difference penalty of `weight` x the samples an interval.
pub fn fit_direction(
    segments: &[Segment],
    dir: i8,
    span: &Span,
    weight: f64,
) -> Result<DirectionFit, TableError> {
    let s = solve(segments, Some(dir), span, weight)?;
    Ok(DirectionFit {
        dir,
        span: *span,
        angle: s.angle,
        offsets: s.offsets,
        residual_rms: s.residual_rms,
        samples: s.samples,
        per_interval: s.per_interval,
    })
}

struct Solved {
    angle: Vec<f64>,
    offsets: Vec<(usize, f64)>,
    residual_rms: f64,
    samples: usize,
    per_interval: Vec<usize>,
}

/// The `dir` rungs, or every rung, with a sample on the span fitted as one.
fn solve(
    segments: &[Segment],
    dir: Option<i8>,
    span: &Span,
    weight: f64,
) -> Result<Solved, TableError> {
    let k = span.count();
    let inside = |v: f64| v >= span.lo() && v <= span.hi();
    let used: Vec<usize> = segments
        .iter()
        .enumerate()
        .filter(|(_, s)| dir.is_none_or(|d| s.dir == d) && s.x.iter().any(|&v| inside(v)))
        .map(|(j, _)| j)
        .collect();
    if used.is_empty() {
        return Err(TableError::NoSamples(dir.unwrap_or(0)));
    }
    let m = k + used.len() - 1;
    let mut a = vec![0.0; m * m];
    let mut b = vec![0.0; m];
    let mut per_interval = vec![0usize; k - 1];
    let mut samples = 0;
    let basis = |v: f64| {
        let i = (((v - span.lo()) / GRID_F).floor() as usize).min(k - 2);
        (i, (v - span.lo() - i as f64 * GRID_F) / GRID_F)
    };
    for (p, &j) in used.iter().enumerate() {
        let s = &segments[j];
        for (&v, &y) in s.x.iter().zip(&s.y).filter(|(v, _)| inside(**v)) {
            let (i, fr) = basis(v);
            let mut terms = [(i, 1.0 - fr), (i + 1, fr), (0, 0.0)];
            let n = if p > 0 {
                terms[2] = (k + p - 1, 1.0);
                3
            } else {
                2
            };
            for &(r, vr) in &terms[..n] {
                b[r] += vr * y;
                for &(c, vc) in &terms[..n] {
                    a[r * m + c] += vr * vc;
                }
            }
            per_interval[i] += 1;
            samples += 1;
        }
    }
    let lam = weight * samples as f64 / (k - 1) as f64;
    for j in 0..k - 2 {
        let d = [(j, 1.0), (j + 1, -2.0), (j + 2, 1.0)];
        for &(r, vr) in &d {
            for &(c, vc) in &d {
                a[r * m + c] += lam * vr * vc;
            }
        }
    }
    let sol = solve_spd(&a, &b).ok_or(TableError::Singular)?;
    let angle = sol[..k].to_vec();
    let offsets: Vec<(usize, f64)> = used
        .iter()
        .enumerate()
        .map(|(p, &j)| (j, if p > 0 { sol[k + p - 1] } else { 0.0 }))
        .collect();
    let mut ss = 0.0;
    for &(j, off) in &offsets {
        let s = &segments[j];
        for (&v, &y) in s.x.iter().zip(&s.y).filter(|(v, _)| inside(**v)) {
            let (i, fr) = basis(v);
            let r = y - (angle[i] * (1.0 - fr) + angle[i + 1] * fr + off);
            ss += r * r;
        }
    }
    Ok(Solved {
        angle,
        offsets,
        residual_rms: (ss / samples as f64).sqrt(),
        samples,
        per_interval,
    })
}

/// The table: one angle at the span's knots.
#[derive(Clone, Debug, PartialEq)]
pub struct Table {
    pub span: Span,
    /// Cycles at the knots.
    pub angle: Vec<f64>,
}

/// The table of every rung, both directions, fitted as one.
#[derive(Clone, Debug, PartialEq)]
pub struct PooledFit {
    /// The first rung's level.
    pub table: Table,
    /// As [`DirectionFit::offsets`], over both directions.
    pub offsets: Vec<(usize, f64)>,
    /// Cycles rms.
    pub residual_rms: f64,
    pub samples: usize,
    /// Samples in each interval.
    pub per_interval: Vec<usize>,
    /// [`reach`] of each direction: past one's ends the table rests on the
    /// other alone.
    pub up: Option<(f64, f64)>,
    pub down: Option<(f64, f64)>,
}

/// Every rung of `segments` fitted as [`fit_direction`] fits one
/// direction's. Refused when the pooled samples leave a hole of more than
/// [`HOLE_MAX`] intervals inside the span.
pub fn fit_pooled(segments: &[Segment], span: &Span, weight: f64) -> Result<PooledFit, TableError> {
    let s = solve(segments, None, span, weight)?;
    no_hole(0, span, &s.per_interval)?;
    Ok(PooledFit {
        table: Table {
            span: *span,
            angle: s.angle,
        },
        offsets: s.offsets,
        residual_rms: s.residual_rms,
        samples: s.samples,
        per_interval: s.per_interval,
        up: reach(segments, 1),
        down: reach(segments, -1),
    })
}

fn no_hole(dir: i8, span: &Span, per_interval: &[usize]) -> Result<(), TableError> {
    let mut run = 0;
    for (i, &n) in per_interval.iter().enumerate() {
        run = if n == 0 { run + 1 } else { 0 };
        if run > HOLE_MAX {
            return Err(TableError::Hole {
                dir,
                from: span.lo() + ((i + 1 - run) * GRID) as f64,
                intervals: run,
            });
        }
    }
    Ok(())
}

/// The mean of the two directions' shapes. Refused when either leaves a
/// hole of more than [`HOLE_MAX`] intervals inside the span.
pub fn mean_table(up: &DirectionFit, down: &DirectionFit) -> Result<Table, TableError> {
    if up.span != down.span || up.angle.len() != down.angle.len() {
        return Err(TableError::SpanMismatch);
    }
    for f in [up, down] {
        no_hole(f.dir, &f.span, &f.per_interval)?;
    }
    Ok(Table {
        span: up.span,
        angle: up
            .angle
            .iter()
            .zip(&down.angle)
            .map(|(u, d)| (u + d) / 2.0)
            .collect(),
    })
}

impl Table {
    /// Cycles a count, end to end over the span.
    pub fn mean_slope(&self) -> f64 {
        (self.angle[self.angle.len() - 1] - self.angle[0]) / (self.span.hi() - self.span.lo())
    }

    /// The angle at `x` counts, continued at [`Table::mean_slope`] past
    /// the span.
    pub fn angle_at(&self, x: f64) -> f64 {
        let (a0, a1) = (self.angle[0], self.angle[self.angle.len() - 1]);
        let sl = self.mean_slope();
        if x < self.span.lo() {
            a0 + (x - self.span.lo()) * sl
        } else if x > self.span.hi() {
            a1 + (x - self.span.hi()) * sl
        } else {
            interp(x, &self.span.knots(), &self.angle, a0, a1)
        }
    }

    /// The firmware's 256 points: the angle mapped linearly onto the stops
    /// so both map to themselves, point k the correction at raw 16 k,
    /// zero at and past each stop (`pos_lut::validate`'s ends rule).
    pub fn points(&self, raw_min: u16, raw_max: u16) -> [i16; INTERVALS] {
        let (lo, hi) = (raw_min as f64, raw_max as f64);
        let (a_lo, a_hi) = (self.angle_at(lo), self.angle_at(hi));
        let k_lo = (raw_min as usize).div_ceil(GRID);
        let k_hi = raw_max as usize / GRID;
        let mut out = [0i16; INTERVALS];
        for (k, c) in out.iter_mut().enumerate() {
            if k <= k_lo || k >= k_hi {
                continue;
            }
            let g = (k * GRID) as f64;
            let t = lo + (self.angle_at(g) - a_lo) * (hi - lo) / (a_hi - a_lo);
            *c = (t - g)
                .round_ties_even()
                .clamp(i16::MIN as f64, i16::MAX as f64) as i16;
        }
        out
    }

    /// Counts a degree of the output from the ripple, in the frame the
    /// points write: the stops' span over the output's degrees between
    /// them, the table's cycles from stop to stop ([`Table::angle_at`])
    /// times 360 over the motor's events a revolution (the nameplate's
    /// brushes times rotor segments) times the counted gear ratio (plan
    /// sec 2.6).
    pub fn scale(
        &self,
        events_per_rev: u32,
        gear_ratio: f64,
        raw_min: u16,
        raw_max: u16,
    ) -> Option<f64> {
        let (n, g) = (events_per_rev as f64, gear_ratio);
        let deg = self.travel_cycles(raw_min, raw_max) * 360.0 / (n * g);
        let v = (raw_max as f64 - raw_min as f64) / deg;
        (n > 0.0 && g > 0.0 && v.is_finite() && v > 0.0).then_some(v)
    }

    /// The cycles from stop to stop ([`Table::angle_at`]).
    pub fn travel_cycles(&self, raw_min: u16, raw_max: u16) -> f64 {
        self.angle_at(raw_max as f64) - self.angle_at(raw_min as f64)
    }

    /// Counts a cycle, end to end over the span.
    pub fn counts_per_cycle(&self) -> f64 {
        1.0 / self.mean_slope()
    }
}

/// A fit's residual in pot counts at the table's mean slope.
pub fn residual_counts(fit: &DirectionFit, table: &Table) -> f64 {
    fit.residual_rms * table.counts_per_cycle()
}

/// The best straight line in the knots taken out of `v`.
pub fn line_out(span: &Span, v: &[f64]) -> Vec<f64> {
    let k = span.knots();
    match polyfit::<2>(&k, v) {
        Some(c) => v.iter().zip(&k).map(|(y, x)| y - polyval(&c, *x)).collect(),
        None => v.to_vec(),
    }
}

fn rms(v: &[f64]) -> f64 {
    (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt()
}

/// What one table cannot fix, up less down, line out, counts.
#[derive(Clone, Debug, PartialEq)]
pub struct Term {
    pub rms: f64,
    pub max: f64,
    /// (band start, rms) for each [`BAND_COUNTS`] band with a knot.
    pub bands: Vec<(f64, f64)>,
}

/// The up shape less the down, line out, in counts at the table's slope.
pub fn direction_term(up: &DirectionFit, down: &DirectionFit, table: &Table) -> Term {
    let diff: Vec<f64> = up
        .angle
        .iter()
        .zip(&down.angle)
        .map(|(u, d)| u - d)
        .collect();
    let d: Vec<f64> = line_out(&up.span, &diff)
        .iter()
        .map(|v| v * table.counts_per_cycle())
        .collect();
    let k = up.span.knots();
    let mut bands = Vec::new();
    let mut b0 = (k[0] / BAND_COUNTS).floor() * BAND_COUNTS;
    while b0 <= k[k.len() - 1] {
        let v: Vec<f64> = k
            .iter()
            .zip(&d)
            .filter(|(x, _)| **x >= b0 && **x < b0 + BAND_COUNTS)
            .map(|(_, v)| *v)
            .collect();
        if !v.is_empty() {
            bands.push((b0, rms(&v)));
        }
        b0 += BAND_COUNTS;
    }
    Term {
        rms: rms(&d),
        max: d.iter().fold(0.0f64, |m, v| m.max(v.abs())),
        bands,
    }
}

/// Two tables on one span against each other.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Repeat {
    /// Counts rms of `a` less `b`, line out, at `a`'s slope.
    pub line_out: f64,
    /// The rms of `a`'s 48-count slope over `b`'s, less 1.
    pub slope: f64,
}

pub fn repeat(a: &Table, b: &Table) -> Result<Repeat, TableError> {
    if a.span != b.span {
        return Err(TableError::SpanMismatch);
    }
    let diff: Vec<f64> = a.angle.iter().zip(&b.angle).map(|(x, y)| x - y).collect();
    let (sa, sb) = (a.span.slopes(&a.angle), b.span.slopes(&b.angle));
    let ratio: Vec<f64> = sa.iter().zip(&sb).map(|(x, y)| x / y - 1.0).collect();
    Ok(Repeat {
        line_out: rms(&line_out(&a.span, &diff)) * a.counts_per_cycle(),
        slope: rms(&ratio),
    })
}

/// The least and the most interval gain `16 + c[k+1] - c[k]` (Q4) of
/// `points`, the fixed last point 0 included.
pub fn gains(points: &[i16; INTERVALS]) -> (i32, i32) {
    let c = |k: usize| points.get(k).copied().unwrap_or(0) as i32;
    (0..INTERVALS)
        .map(|k| GRID as i32 + c(k + 1) - c(k))
        .fold((i32::MAX, i32::MIN), |(l, h), g| (l.min(g), h.max(g)))
}

/// Two same-direction rungs' cycles over the counts they share.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Agreement {
    pub from: f64,
    pub to: f64,
    pub a: f64,
    pub b: f64,
}

/// Cycles two rungs may differ by on a travel of `travel_cycles`.
pub fn agree_limit(travel_cycles: f64) -> f64 {
    AGREE_SHARE * travel_cycles
}

impl Agreement {
    pub fn holds(&self, limit: f64) -> bool {
        (self.a - self.b).abs() <= limit
    }
}

/// The cycles each of two same-direction rungs counts over the counts
/// both cover, two counts in from each end; None when they differ in
/// direction or share under a grid interval.
pub fn agreement(a: &Segment, b: &Segment) -> Option<Agreement> {
    if a.dir != b.dir {
        return None;
    }
    let reach = |s: &Segment| {
        s.x.iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &v| {
                (l.min(v), h.max(v))
            })
    };
    let ((la, ha), (lb, hb)) = (reach(a), reach(b));
    let (from, to) = (la.max(lb) + 2.0, ha.min(hb) - 2.0);
    if to - from < GRID_F {
        return None;
    }
    let cycles = |s: &Segment| Some((at(s, to)? - at(s, from)?).abs());
    Some(Agreement {
        from,
        to,
        a: cycles(a)?,
        b: cycles(b)?,
    })
}

/// The angle where the rung's pot first reaches `e` in its direction of
/// travel, between the two samples around it.
fn at(s: &Segment, e: f64) -> Option<f64> {
    let d = s.dir as f64;
    let j = s.x.iter().position(|&v| d * (v - e) >= 0.0)?;
    if j == 0 {
        return Some(s.y[0]);
    }
    let (x0, x1) = (s.x[j - 1], s.x[j]);
    let fr = if x1 != x0 { (e - x0) / (x1 - x0) } else { 1.0 };
    Some(s.y[j - 1] + fr * (s.y[j] - s.y[j - 1]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    const FS: f64 = 20_000.0;

    /// The pot's counts at motor angle `t` cycles: 4 counts a cycle with a
    /// bend that reads fast over 1600..1700.
    fn pot_of(t: f64) -> f64 {
        let u = 500.0 + 4.0 * t;
        u + 15.0 * (1.0 + ((u - 1650.0) / 30.0).tanh())
    }

    /// A rung at a steady `hz` ripple line from cycle `t0`, `dir` x the
    /// motor's turn, `rows` long, the current a ripple on a level.
    fn rung(t0: f64, dir: f64, hz: f64, rows: usize) -> (Vec<u16>, Vec<u16>, Vec<i16>) {
        let mut pos = Vec::with_capacity(rows);
        let mut cur = Vec::with_capacity(rows);
        for i in 0..rows {
            let t = t0 + dir * hz * i as f64 / FS;
            pos.push(pot_of(t).round() as u16);
            cur.push((1200.0 + 20.0 * (2.0 * PI * hz * i as f64 / FS).sin()).round() as u16);
        }
        (pos, cur, vec![(dir * 1000.0) as i16; rows])
    }

    fn segs() -> Vec<Segment> {
        let slots = [0u8; ADC_WIDE_SLOTS];
        [
            (10.0, 1.0, 470.0),
            (30.0, 1.0, 520.0),
            (560.0, -1.0, 480.0),
            (545.0, -1.0, 500.0),
        ]
        .iter()
        .map(|&(t0, d, hz)| {
            let (p, c, u) = rung(t0, d, hz, 20_000);
            segment(&p, &c, &u, &slots, FS).expect("segment")
        })
        .collect()
    }

    #[test]
    fn a_rung_is_read_against_its_ripple() {
        let s = &segs()[0];
        assert_eq!(s.dir, 1);
        assert!((s.f_seed - 470.0).abs() < 2.0, "{}", s.f_seed);
        assert_eq!(s.odd, 0.0);
        assert!(s.speed_spread < 0.01, "{}", s.speed_spread);
        // 0.1 s of spin-up and 2 ms of tail out
        assert!(
            (s.crossings as f64 - 470.0 * 0.898).abs() < 3.0,
            "{}",
            s.crossings
        );
        assert_eq!(s.x.len(), s.y.len());
        assert!(s.y.windows(2).all(|w| w[1] > w[0]));
        let d = segs()[2].clone();
        assert_eq!(d.dir, -1);
        assert!(d.y.windows(2).all(|w| w[1] < w[0]));
        let (p, c, _) = rung(10.0, 1.0, 470.0, 4000);
        assert_eq!(
            segment(&p, &c, &[0; 4000], &[0; ADC_WIDE_SLOTS], FS),
            Err(SegmentError::NoDrive)
        );
    }

    #[test]
    fn the_fit_recovers_a_known_bend() {
        let segs = segs();
        let span = common_span(&segs).expect("span");
        let up = fit_direction(&segs, 1, &span, SMOOTH_WEIGHT).expect("up");
        let down = fit_direction(&segs, -1, &span, SMOOTH_WEIGHT).expect("down");
        let t = mean_table(&up, &down).expect("table");
        assert_eq!(up.offsets.len(), 2);
        assert!(up.residual_rms < 0.05 && down.residual_rms < 0.05);
        // the motor's angle at each knot against the truth, up to a level
        let truth: Vec<f64> = span
            .knots()
            .iter()
            .map(|&x| {
                let (mut lo, mut hi) = (-200.0, 800.0);
                for _ in 0..60 {
                    let m = (lo + hi) / 2.0;
                    if pot_of(m) < x { lo = m } else { hi = m }
                }
                lo
            })
            .collect();
        let off = t.angle[0] - truth[0];
        let err: Vec<f64> = t
            .angle
            .iter()
            .zip(&truth)
            .map(|(a, b)| a - b - off)
            .collect();
        assert!(rms(&err) < 0.05, "{}", rms(&err));
        // the bend reads 6 counts a cycle at its middle, 4 off it
        let (sl, want) = (span.slopes(&t.angle), span.slopes(&truth));
        let fast = want.iter().fold(1.0f64, |m, v| m.min(*v));
        assert!(fast < 0.2, "{fast}");
        let worst = sl
            .iter()
            .zip(&want)
            .fold(0.0f64, |m, (a, b)| m.max((a / b - 1.0).abs()));
        assert!(worst < 0.035, "{worst}");
        let term = direction_term(&up, &down, &t);
        assert!(term.rms < 0.5, "{term:?}");
        let limit = agree_limit(t.angle_at(span.hi()) - t.angle_at(span.lo()));
        assert!(agreement(&segs[0], &segs[1]).expect("overlap").holds(limit));
        assert!(agreement(&segs[0], &segs[2]).is_none());
    }

    #[test]
    fn the_pooled_fit_recovers_the_bend_over_both_directions() {
        let segs = segs();
        let span = pooled_span(&segs).expect("span");
        let common = common_span(&segs).expect("span");
        assert!(span.k0 < common.k0 && span.k1 > common.k1);
        let p = fit_pooled(&segs, &span, SMOOTH_WEIGHT).expect("pooled");
        assert_eq!(p.offsets.len(), 4);
        assert_eq!(p.offsets[0], (0, 0.0));
        assert!(p.residual_rms < 0.05, "{}", p.residual_rms);
        assert_eq!((p.up, p.down), (reach(&segs, 1), reach(&segs, -1)));
        let up = fit_direction(&segs, 1, &common, SMOOTH_WEIGHT).expect("up");
        let down = fit_direction(&segs, -1, &common, SMOOTH_WEIGHT).expect("down");
        let mean = mean_table(&up, &down).expect("table");
        let rel = |t: &Table, x: f64| t.angle_at(x) - t.angle_at(common.lo());
        let worst = common.knots().iter().fold(0.0f64, |m, &x| {
            m.max((rel(&p.table, x) - rel(&mean, x)).abs())
        });
        assert!(worst < 0.05, "{worst}");
        let holed: Vec<Segment> = segs
            .iter()
            .map(|s| {
                let (x, y) =
                    s.x.iter()
                        .zip(&s.y)
                        .filter(|(v, _)| !(1300.0..1400.0).contains(*v))
                        .unzip();
                Segment { x, y, ..s.clone() }
            })
            .collect();
        assert_eq!(
            fit_pooled(&holed, &span, SMOOTH_WEIGHT).map(|_| ()),
            Err(TableError::Hole {
                dir: 0,
                from: 1312.0,
                intervals: 5
            })
        );
    }

    #[test]
    fn a_hole_in_one_direction_refuses() {
        let segs = segs();
        let span = common_span(&segs).expect("span");
        let up = fit_direction(&segs, 1, &span, SMOOTH_WEIGHT).expect("up");
        let mut down = fit_direction(&segs, -1, &span, SMOOTH_WEIGHT).expect("down");
        for n in &mut down.per_interval[10..14] {
            *n = 0;
        }
        assert!(mean_table(&up, &down).is_ok());
        down.per_interval[14] = 0;
        assert_eq!(
            mean_table(&up, &down),
            Err(TableError::Hole {
                dir: -1,
                from: span.lo() + 160.0,
                intervals: 5
            })
        );
        assert_eq!(common_span(&segs[..2]), Err(TableError::OneWay));
    }

    #[test]
    fn the_points_pin_both_stops_and_keep_the_ends_straight() {
        // a table that reads 0.25 cycles a count with a bend, on 1008..2000
        let span = Span { k0: 63, k1: 125 };
        let angle: Vec<f64> = span
            .knots()
            .iter()
            .map(|&x| 0.25 * x + 3.0 * ((x - 1500.0) / 100.0).tanh())
            .collect();
        let t = Table { span, angle };
        let (lo, hi) = (110u16, 3730u16);
        let p = t.points(lo, hi);
        let mut full = [0i16; INTERVALS + 1];
        full[..INTERVALS].copy_from_slice(&p);
        assert_eq!(osc_servo_core::pos_lut::validate(&full, lo, hi), Ok(()));
        assert!(p[..=7].iter().all(|&c| c == 0));
        assert!(p[233..].iter().all(|&c| c == 0));
        // past the span the corrections run straight: equal steps
        let step = p[20] - p[19];
        assert!(p[9..62].windows(2).all(|w| (w[1] - w[0] - step).abs() <= 1));
        // each stop maps to itself
        let lut = |raw: u16| osc_servo_core::pos_lut::interp_q4(raw, &full) as f64 / 16.0;
        assert_eq!(lut(lo), lo as f64);
        assert_eq!(lut(hi), hi as f64);
        let (gmin, gmax) = gains(&p);
        assert!(gmin >= 8 && gmax <= 32, "{gmin} {gmax}");
        // both stops past the span, where the table runs on its mean slope
        let s = t.scale(6, 300.0, lo, hi).expect("scale");
        assert!((s - 6.0 * 300.0 / (360.0 * t.mean_slope())).abs() < 1e-9);
        assert_eq!(t.scale(0, 300.0, lo, hi), None);
        assert!((span.coverage(lo, hi) - 992.0 / 3620.0).abs() < 1e-12);
    }

    /// A table reading 0.2 cycles a count below 1500 and 0.3 above, on
    /// 512..3072: its scale is the stops' span over the cycles between
    /// them, not the middle's slope nor the span's mean; stops past the
    /// span read the cycles on its mean slope.
    #[test]
    fn the_scale_is_the_stop_to_stop_frame() {
        let span = Span { k0: 32, k1: 192 };
        let a = |x: f64| 0.2 * x.min(1500.0) + 0.3 * (x - 1500.0).max(0.0);
        let t = Table {
            span,
            angle: span.knots().iter().map(|&x| a(x)).collect(),
        };
        let (n, g) = (6, 300.0);
        let (lo, hi) = (1000u16, 1900u16);
        let s = t.scale(n, g, lo, hi).expect("scale");
        let cycles = a(1900.0) - a(1000.0);
        let want = 900.0 / (cycles * 360.0 / (6.0 * 300.0));
        assert!((s - want).abs() < 1e-9, "{s} against {want}");
        let middle = 6.0 * 300.0 / (360.0 * 0.2);
        let mean = 6.0 * 300.0 / (360.0 * t.mean_slope());
        assert!((s / middle - 1.0).abs() > 0.05 && (s / mean - 1.0).abs() > 0.05);
        let (lo, hi) = (200u16, 1900u16);
        let s = t.scale(n, g, lo, hi).expect("scale");
        let cycles = a(1900.0) - (a(512.0) - 312.0 * t.mean_slope());
        let want = 1700.0 / (cycles * 360.0 / (6.0 * 300.0));
        assert!((s - want).abs() < 1e-9, "{s} against {want}");
        let s = t.scale(n, g, 200, 3500).expect("scale");
        assert!((s - mean).abs() < 1e-9, "{s} against {mean}");
    }

    /// Two fast rungs over a third of a 940-cycle travel: 1.10 cycles
    /// apart holds, as the travel's share allows 2.82; 3.0 apart refuses.
    #[test]
    fn a_pair_agrees_within_a_share_of_the_travel() {
        let limit = agree_limit(940.0);
        assert!((limit - 2.82).abs() < 1e-9, "{limit}");
        let mut k = Agreement {
            from: 2263.0,
            to: 3476.0,
            a: 316.45,
            b: 315.35,
        };
        assert!(k.holds(limit));
        k.b = k.a - 3.0;
        assert!(!k.holds(limit));
    }
}
