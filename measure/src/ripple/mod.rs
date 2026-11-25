//! The ripple tach: a brushed motor's speed counted in its commutation
//! ripple, oscnb `ripple` ported. Each commutation puts a dip in the
//! winding current and, with the bridge open, in the terminal EMF, slots x
//! brushes of them a revolution, so the ripple's line sits at that many
//! times the shaft's turn rate. The line is counted in time: a band-pass
//! around it run forward and back, so it shifts no crossing, and its
//! upward zero crossings interpolated to a fraction of a row.
//!
//! A line that moves faster than a window holds is counted two more ways.
//! [`tracked_crossings`] re-centers the band every few cycles on the line
//! the last chunk read. The order count ([`order`]), for a run-up from rest
//! whose line sweeps up from zero, resamples the current onto a rough angle
//! where the ripple sits at one frequency whatever the speed.
//!
//! Every function takes the row rate `fs` and works in rows from the start
//! of the slice it is given, except the order count's, which work in
//! seconds. A series too short to filter, a seed that is not a frequency
//! or a non-finite row reads as no crossings, never a panic.

pub mod filter;
pub mod order;
pub mod spectrum;
pub mod table;

pub use spectrum::line_freq;

/// Hz, where a ripple line is looked for: 25 to 1000 rev/s at six events a
/// revolution.
pub const BAND_HZ: (f64, f64) = (150.0, 6000.0);
/// The band-pass half-width as a share of the line: the lines at half and
/// at one and a half times it stay outside.
pub const BAND_FRAC: f64 = 0.35;
/// The band's top under this share of Nyquist.
pub const NYQUIST_SHARE: f64 = 0.95;
/// ms, a window spans the whole cycles closest to this.
pub const WINDOW_MS: f64 = 10.0;
/// Cycles a window holds at least, which the slowest coasts need.
pub const MIN_CYCLES: usize = 3;
/// The band-pass runs over the window plus this many cycles, stretched by
/// [`SPAN_MARGIN`], so the window's crossings are never the filter's edge.
pub const SPAN_CYCLES: usize = 3;
pub const SPAN_MARGIN: f64 = 1.3;
/// Counts at most, the band centered on the last.
pub const PASSES: usize = 6;
/// A count this close to the last ends the passes.
pub const CONVERGED: f64 = 1e-4;
/// Cycles a chunk of the tracked count.
pub const TRACK_CYCLES: usize = 4;

/// Upward crossings of the line near `f0` Hz, in rows, the two ends
/// dropped: `x` less its mean through the band `f0 x (1 +/- frac)`, its
/// top held under [`NYQUIST_SHARE`] of Nyquist.
pub fn crossings(x: &[f64], f0: f64, frac: f64, fs: f64) -> Vec<f64> {
    let n = x.len();
    if n == 0 || !x.iter().all(|v| v.is_finite()) {
        return Vec::new();
    }
    let hi = (f0 * (1.0 + frac)).min(NYQUIST_SHARE * fs / 2.0);
    let Some(sos) = filter::band(f0 * (1.0 - frac), hi, fs) else {
        return Vec::new();
    };
    let mean = x.iter().sum::<f64>() / n as f64;
    let centered: Vec<f64> = x.iter().map(|v| v - mean).collect();
    let Some(y) = filter::filtfilt(&sos, &centered) else {
        return Vec::new();
    };
    let up: Vec<f64> = (0..n - 1)
        .filter(|&k| y[k] < 0.0 && y[k + 1] >= 0.0)
        .map(|k| k as f64 + (-y[k]) / (y[k + 1] - y[k]))
        .collect();
    if up.len() < 2 {
        return Vec::new();
    }
    up[1..up.len() - 1].to_vec()
}

/// A count over whole cycles of the line: its mean frequency, Hz.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Cycles {
    pub hz: f64,
    pub cycles: usize,
}

/// The mean frequency over every whole cycle of the line in `x`; None
/// under three crossings.
pub fn count_cycles(x: &[f64], f0: f64, frac: f64, fs: f64) -> Option<Cycles> {
    let tc = crossings(x, f0, frac, fs);
    if tc.len() < 3 {
        return None;
    }
    let cycles = tc.len() - 1;
    Some(Cycles {
        hz: cycles as f64 / ((tc[cycles] - tc[0]) / fs),
        cycles,
    })
}

/// A window of whole cycles: its frequency, its first and last row, and
/// its cycles.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Window {
    pub hz: f64,
    pub i0: f64,
    pub i1: f64,
    pub cycles: usize,
}

/// The window from `start` of the whole cycles nearest `target_ms`,
/// [`MIN_CYCLES`] at least, counted up to [`PASSES`] times with the band
/// centered on the last count. `snap` rounds its ends inward to whole
/// rows. None when the series ends first or the count comes short.
pub fn read_window(
    x: &[f64],
    start: usize,
    f_seed: f64,
    target_ms: f64,
    snap: bool,
    fs: f64,
) -> Option<Window> {
    let mut f = f_seed;
    let mut tc = Vec::new();
    let mut n = 0;
    for _ in 0..PASSES {
        if !(f.is_finite() && f > 0.0) {
            return None;
        }
        n = MIN_CYCLES.max((target_ms / 1e3 * f).round_ties_even() as usize);
        let stop = start + ((n + SPAN_CYCLES) as f64 * fs / f * SPAN_MARGIN) as usize;
        if stop > x.len() {
            return None;
        }
        tc = crossings(&x[start..stop], f, BAND_FRAC, fs);
        if tc.len() < n + 1 {
            return None;
        }
        tc.truncate(n + 1);
        let f_new = n as f64 / ((tc[n] - tc[0]) / fs);
        let done = (f_new / f - 1.0).abs() < CONVERGED;
        f = f_new;
        if done {
            break;
        }
    }
    let s = start as f64;
    let (i0, i1) = if snap {
        (s + tc[0].ceil(), s + tc[n].floor())
    } else {
        (s + tc[0], s + tc[n])
    };
    Some(Window {
        hz: f,
        i0,
        i1,
        cycles: n,
    })
}

/// Upward crossings from `start` to `stop`, in rows, the band re-centered
/// every `cycles` cycles on the line the last chunk read.
pub fn tracked_crossings(
    x: &[f64],
    start: usize,
    stop: f64,
    f_seed: f64,
    cycles: usize,
    fs: f64,
) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    let mut f = f_seed;
    let mut last = start as f64 - 0.5;
    while last < stop {
        if !(f.is_finite() && f > 0.0) {
            break;
        }
        let lo = ((last - 3.0 * fs / f) as i64).max(0) as usize;
        let hi = ((last + (cycles + 4) as f64 * fs / f) as i64).clamp(0, x.len() as i64) as usize;
        if hi <= lo || ((hi - lo) as f64) < 6.0 * fs / f {
            break;
        }
        let tc: Vec<f64> = crossings(&x[lo..hi], f, BAND_FRAC, fs)
            .into_iter()
            .map(|t| t + lo as f64)
            .filter(|&t| t > last + 0.3 * fs / f && t <= stop)
            .collect();
        if tc.is_empty() {
            break;
        }
        let take = tc.len().min(cycles);
        out.extend_from_slice(&tc[..take]);
        if out.len() >= 2 {
            let recent = &out[out.len().saturating_sub(cycles + 1)..];
            f = (recent.len() - 1) as f64 / ((recent[recent.len() - 1] - recent[0]) / fs);
        }
        last = out[out.len() - 1];
        if hi >= x.len() && tc.len() <= take {
            break;
        }
    }
    out
}

/// One cycle between two crossings: its middle and span, s, and its speed.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CycleSpeed {
    pub t_mid_s: f64,
    pub span_s: f64,
    pub rev_s: f64,
}

/// The speed of each cycle between two crossings in rows.
pub fn cycle_speeds(tc: &[f64], per_rev: usize, fs: f64) -> Vec<CycleSpeed> {
    tc.windows(2)
        .map(|w| {
            let span_s = (w[1] - w[0]) / fs;
            CycleSpeed {
                t_mid_s: (w[0] + w[1]) / 2.0 / fs,
                span_s,
                rev_s: 1.0 / span_s / per_rev as f64,
            }
        })
        .collect()
}

/// rev/s over the whole cycles of a run of crossings in rows; None under
/// three.
pub fn speed_of(tc: &[f64], per_rev: usize, fs: f64) -> Option<f64> {
    if tc.len() < 3 {
        return None;
    }
    let v = (tc.len() - 1) as f64 / ((tc[tc.len() - 1] - tc[0]) / fs) / per_rev as f64;
    v.is_finite().then_some(v)
}

/// Crossings with missed ones put back and doubled ones taken out.
#[derive(Clone, Debug, PartialEq)]
pub struct Repaired {
    pub tc: Vec<f64>,
    pub added: usize,
    pub removed: usize,
}

/// An interval this short against its neighbors' median loses a crossing.
pub const REPAIR_SHORT: f64 = 0.7;
/// An interval this long against the median is split evenly.
pub const REPAIR_LONG: f64 = 1.5;
/// A merge lands within this share of the median or is not made.
pub const REPAIR_MERGE_TOL: f64 = 0.35;
/// Intervals in the rolling median.
pub const REPAIR_WINDOW: usize = 15;
/// Fixes at most.
pub const REPAIR_FIXES: usize = 500;

/// The rolling median of `n` intervals centered on each, three at least,
/// as pandas reads it; NaN where fewer.
pub fn rolling_median(iv: &[f64], n: usize) -> Vec<f64> {
    let h = n / 2;
    (0..iv.len())
        .map(|j| {
            let w = &iv[j.saturating_sub(h)..(j + n - h).min(iv.len())];
            if w.len() < 3 {
                f64::NAN
            } else {
                crate::fit::median(w).unwrap_or(f64::NAN)
            }
        })
        .collect()
}

/// One missed or extra crossing at a time, each interval held against the
/// rolling median of [`REPAIR_WINDOW`] around it: a short one merged into
/// a neighbor when the merge lands near the median, a long one split
/// evenly into the whole intervals it holds.
pub fn repair(tc: &[f64]) -> Repaired {
    let mut tc = tc.to_vec();
    let (mut added, mut removed) = (0, 0);
    for _ in 0..REPAIR_FIXES {
        if tc.len() < 2 {
            break;
        }
        let iv: Vec<f64> = tc.windows(2).map(|w| w[1] - w[0]).collect();
        let med = rolling_median(&iv, REPAIR_WINDOW);
        let ratio: Vec<f64> = iv.iter().zip(&med).map(|(a, m)| a / m).collect();
        if ratio.iter().any(|r| r.is_nan()) {
            break;
        }
        let j = (0..ratio.len())
            .reduce(|a, b| if ratio[b] < ratio[a] { b } else { a })
            .unwrap_or(0);
        if ratio[j] < REPAIR_SHORT {
            let mut best: Option<(f64, usize)> = None;
            let mut cand = |err: f64, k: usize| {
                if best.is_none_or(|(e, i)| err < e || (err == e && k < i)) {
                    best = Some((err, k));
                }
            };
            if j > 0 {
                cand(((iv[j - 1] + iv[j]) / med[j] - 1.0).abs(), j);
            }
            if j + 1 < iv.len() {
                cand(((iv[j] + iv[j + 1]) / med[j] - 1.0).abs(), j + 1);
            }
            if let Some((err, drop)) = best
                && err < REPAIR_MERGE_TOL
            {
                tc.remove(drop);
                removed += 1;
                continue;
            }
        }
        let j = (0..ratio.len())
            .reduce(|a, b| if ratio[b] > ratio[a] { b } else { a })
            .unwrap_or(0);
        if ratio[j] > REPAIR_LONG {
            let m = ratio[j].round_ties_even() as usize;
            let fill: Vec<f64> = (1..m)
                .map(|k| tc[j] + iv[j] * k as f64 / m as f64)
                .collect();
            tc.splice(j + 1..j + 1, fill);
            added += m - 1;
            continue;
        }
        break;
    }
    Repaired { tc, added, removed }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 20_000.0;

    fn tone(hz: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|k| 100.0 + 3.0 * (2.0 * core::f64::consts::PI * hz * k as f64 / FS).sin())
            .collect()
    }

    #[test]
    fn a_steady_tone_counts_at_its_frequency() {
        let x = tone(600.0, 4000);
        let c = count_cycles(&x, 650.0, BAND_FRAC, FS).expect("counts");
        assert!((c.hz - 600.0).abs() < 0.05, "{}", c.hz);
        assert!(c.cycles >= 115, "{}", c.cycles);
        let tc = crossings(&x, 600.0, BAND_FRAC, FS);
        let v = speed_of(&tc, 6, FS).expect("speed");
        assert!((v - 100.0).abs() < 0.01, "{v}");
        let tr = tracked_crossings(&x, 0, x.len() as f64, 650.0, TRACK_CYCLES, FS);
        assert!(tr.len() >= 115, "{}", tr.len());
        assert!(
            cycle_speeds(&tr, 6, FS)
                .iter()
                .all(|c| (c.rev_s - 100.0).abs() < 1.0)
        );
        let w = read_window(&x, 20, 650.0, WINDOW_MS, true, FS).expect("window");
        assert_eq!(w.cycles, 6);
        assert!((w.hz - 600.0).abs() < 0.5 && w.i0.fract() == 0.0 && w.i1.fract() == 0.0);
    }

    #[test]
    fn empty_flat_short_and_nan_series_count_nothing() {
        let nan = {
            let mut x = tone(600.0, 4000);
            x[100] = f64::NAN;
            x
        };
        for x in [vec![], vec![5.0; 4000], tone(600.0, 10), nan] {
            assert!(crossings(&x, 600.0, BAND_FRAC, FS).is_empty());
            assert_eq!(count_cycles(&x, 600.0, BAND_FRAC, FS), None);
            assert_eq!(read_window(&x, 0, 600.0, WINDOW_MS, true, FS), None);
            assert!(tracked_crossings(&x, 0, x.len() as f64, 600.0, 4, FS).is_empty());
        }
        assert_eq!(line_freq(&[], 150.0, 6000.0, FS), None);
        assert_eq!(line_freq(&[1.0, f64::NAN, 2.0], 150.0, 6000.0, FS), None);
        assert!(crossings(&tone(600.0, 4000), f64::NAN, BAND_FRAC, FS).is_empty());
        assert!(crossings(&tone(600.0, 4000), 0.0, BAND_FRAC, FS).is_empty());
        assert_eq!(speed_of(&[1.0, 2.0], 6, FS), None);
        assert_eq!(repair(&[]).tc, Vec::<f64>::new());
    }

    #[test]
    fn a_missed_and_a_doubled_crossing_are_repaired() {
        let mut tc: Vec<f64> = (0..60).map(|k| 33.3 * k as f64).collect();
        tc.remove(20);
        tc.insert(40, 33.3 * 40.0 + 5.0);
        let r = repair(&tc);
        assert_eq!((r.added, r.removed), (1, 1));
        assert_eq!(r.tc.len(), 60);
        for (k, t) in r.tc.iter().enumerate() {
            assert!((t - 33.3 * k as f64).abs() < 1e-9, "{k} {t}");
        }
    }
}
