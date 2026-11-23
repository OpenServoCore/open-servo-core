//! The order count of a run-up from rest, oscnb `ripple.count_speed`
//! ported: the current resampled onto a rough angle, [`GRID_SAMPLES`] a
//! ripple cycle, where the ripple sits at one frequency whatever the
//! speed, band-passed there and its upward crossings counted, each pass on
//! the angle the last one counted. The first pass's angle is the EMF the
//! volts leave through R and L over Ke ([`rough_speed`]); no pot enters.
//! Times are in seconds from the drive's first row, speeds in rad/s.

use core::f64::consts::PI;

use super::{filter, repair};
use crate::fit::{interp, median};

/// The ripple resampled to this many samples a cycle.
pub const GRID_SAMPLES: usize = 16;
/// The band-pass at +/- this share of the line on the angle grid.
pub const ORDER_BAND: f64 = 0.25;
pub const ORDER_PASSES: usize = 5;
/// 1 ms, the current's L/R rise: no crossing read in it.
pub const RISE_ROWS: usize = 20;
/// The crossing before which the speed is the current's integral.
pub const INTEGRAL_UNTIL: usize = 9;
/// About 2 ms, the EMF seed's boxcar.
pub const SEED_BOX_ROWS: usize = 41;
/// Intervals in the local speed's median.
pub const LOCAL_SPEED_INTERVALS: usize = 31;

/// numpy's `convolve(x, ones(m) / m, "same")` at `x`'s own length.
pub fn boxcar(x: &[f64], m: usize) -> Vec<f64> {
    let n = x.len();
    if m == 0 {
        return x.to_vec();
    }
    let w = 1.0 / m as f64;
    let hi = (m - 1) / 2;
    let lo = m - 1 - hi;
    (0..n)
        .map(|i| {
            let a = i.saturating_sub(lo);
            let b = (i + hi + 1).min(n);
            x[a..b].iter().map(|v| v * w).sum()
        })
        .collect()
}

/// The first pass's speed, rad/s a row: the EMF the volts leave through R
/// and L, `u - R i - L di/dt`, over a boxcar of [`SEED_BOX_ROWS`], clipped
/// at zero, over `ke` (V s/rad). Empty under two rows.
pub fn rough_speed(
    current: &[f64],
    volts: &[f64],
    r_ohm: f64,
    l_h: f64,
    ke: f64,
    fs: f64,
) -> Vec<f64> {
    let n = current.len().min(volts.len());
    if n < 2 {
        return Vec::new();
    }
    let grad = |k: usize| -> f64 {
        if k == 0 {
            current[1] - current[0]
        } else if k == n - 1 {
            current[n - 1] - current[n - 2]
        } else {
            (current[k + 1] - current[k - 1]) / 2.0
        }
    };
    let emf: Vec<f64> = (0..n)
        .map(|k| volts[k] - r_ohm * current[k] - l_h * grad(k) * fs)
        .collect();
    boxcar(&emf, SEED_BOX_ROWS)
        .into_iter()
        .map(|e| (if e.is_nan() { e } else { e.max(0.0) }) / ke)
        .collect()
}

/// numpy's `arange(start, stop, step)` for floats.
pub fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    let len = ((stop - start) / step).ceil();
    if !len.is_finite() || len <= 0.0 {
        return Vec::new();
    }
    let len = len as usize;
    let next = start + step;
    let delta = next - start;
    (0..len)
        .map(|i| match i {
            0 => start,
            1 => next,
            _ => start + i as f64 * delta,
        })
        .collect()
}

/// Upward ripple crossings in rad of the angle `theta`, `x` resampled to
/// [`GRID_SAMPLES`] a ripple cycle, the two ends dropped.
pub fn angle_crossings(x: &[f64], theta: &[f64], per_rev: usize) -> Vec<f64> {
    let n = x.len().min(theta.len());
    if n == 0 {
        return Vec::new();
    }
    let per = 2.0 * PI / per_rev as f64;
    let step = per / GRID_SAMPLES as f64;
    let (mut tk, mut xk) = (vec![theta[0]], vec![x[0]]);
    for k in 1..n {
        if theta[k] - theta[k - 1] > 0.0 {
            tk.push(theta[k]);
            xk.push(x[k]);
        }
    }
    let grid = arange(theta[0], theta[n - 1], step);
    let xs: Vec<f64> = grid
        .iter()
        .map(|&g| interp(g, &tk, &xk, xk[0], xk[xk.len() - 1]))
        .collect();
    if xs.is_empty() || !xs.iter().all(|v| v.is_finite()) {
        return Vec::new();
    }
    let ns = GRID_SAMPLES as f64;
    let Some(sos) = filter::band(1.0 - ORDER_BAND, 1.0 + ORDER_BAND, ns) else {
        return Vec::new();
    };
    let mean = xs.iter().sum::<f64>() / xs.len() as f64;
    let c: Vec<f64> = xs.iter().map(|v| v - mean).collect();
    let Some(y) = filter::filtfilt(&sos, &c) else {
        return Vec::new();
    };
    let up: Vec<f64> = (0..y.len() - 1)
        .filter(|&s| y[s] < 0.0 && y[s + 1] >= 0.0)
        .map(|s| grid[s] + (-y[s]) / (y[s + 1] - y[s]) * step)
        .collect();
    if up.len() < 2 {
        return Vec::new();
    }
    up[1..up.len() - 1].to_vec()
}

/// rad/s at each row from the crossing times `tc` (s): one ripple cycle
/// over the median of the [`LOCAL_SPEED_INTERVALS`] intervals around,
/// NaN before the first interval's middle.
pub fn local_speed(tc: &[f64], rows: usize, per_rev: usize, fs: f64) -> Vec<f64> {
    if tc.len() < 2 {
        return vec![f64::NAN; rows];
    }
    let per = 2.0 * PI / per_rev as f64;
    let iv: Vec<f64> = tc.windows(2).map(|w| w[1] - w[0]).collect();
    let h = LOCAL_SPEED_INTERVALS / 2;
    let w: Vec<f64> = (0..iv.len())
        .map(|j| {
            per / median(&iv[j.saturating_sub(h)..(j + h + 1).min(iv.len())]).unwrap_or(f64::NAN)
        })
        .collect();
    let mids: Vec<f64> = tc.windows(2).map(|p| (p[1] + p[0]) / 2.0).collect();
    let last = w[w.len() - 1];
    (0..rows)
        .map(|k| interp(k as f64 / fs, &mids, &w, f64::NAN, last))
        .collect()
}

/// An order count: the crossings (s from the drive's first row), what the
/// repair changed, and the speed at each row (rad/s).
#[derive(Clone, Debug, PartialEq)]
pub struct Count {
    pub tc: Vec<f64>,
    pub added: usize,
    pub removed: usize,
    pub speed: Vec<f64>,
}

/// The order count of a run-up from rest over `current` (A a row), its
/// first angle from `rough` (rad/s a row). Before crossing
/// [`INTEGRAL_UNTIL`] the speed is the current's integral over
/// `i_drag0_a`, the drag at rest, scaled to the count's speed there. None
/// when a pass counts too few crossings to reach it.
pub fn count_speed(
    current: &[f64],
    rough: &[f64],
    i_drag0_a: f64,
    per_rev: usize,
    fs: f64,
) -> Option<Count> {
    let n = current.len();
    if n <= RISE_ROWS || rough.len() != n || !current.iter().all(|v| v.is_finite()) {
        return None;
    }
    let per = 2.0 * PI / per_rev as f64;
    let t: Vec<f64> = (0..n).map(|k| k as f64 / fs).collect();
    let mut speed = rough.to_vec();
    let mut out = None;
    for _ in 0..ORDER_PASSES {
        let mut acc = 0.0;
        let theta: Vec<f64> = speed
            .iter()
            .map(|s| {
                acc += s;
                acc / fs
            })
            .collect();
        if !theta.iter().all(|v| v.is_finite()) {
            return None;
        }
        let floor = theta[RISE_ROWS] + 2.0 * per;
        let c: Vec<f64> = angle_crossings(&current[RISE_ROWS..], &theta[RISE_ROWS..], per_rev)
            .into_iter()
            .filter(|&c| c > floor)
            .collect();
        let at: Vec<f64> = c
            .iter()
            .map(|&c| interp(c, &theta, &t, t[0], t[n - 1]))
            .collect();
        let r = repair(&at);
        if r.tc.len() <= INTEGRAL_UNTIL {
            return None;
        }
        let local = local_speed(&r.tc, n, per_rev, fs);
        let t_int = r.tc[INTEGRAL_UNTIL];
        let mut acc = 0.0;
        let w_int: Vec<f64> = current
            .iter()
            .map(|i| {
                acc += i - i_drag0_a;
                acc / fs
            })
            .collect();
        let last = |v: &[f64]| v[v.len() - 1];
        let k = interp(t_int, &t, &local, local[0], last(&local))
            / interp(t_int, &t, &w_int, w_int[0], last(&w_int));
        speed = (0..n)
            .map(|j| if t[j] < t_int { w_int[j] * k } else { local[j] })
            .collect();
        out = Some(Count {
            tc: r.tc,
            added: r.added,
            removed: r.removed,
            speed: speed.clone(),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arange_is_numpys() {
        assert_eq!(arange(0.1, 1.0, 0.3), vec![0.1, 0.4, 0.7000000000000001]);
        assert_eq!(arange(0.0, 0.055, 0.0005).len(), 110);
        assert!(arange(1.0, 1.0, 0.1).is_empty());
        assert!(arange(0.0, f64::NAN, 0.1).is_empty());
    }

    #[test]
    fn boxcar_centers_odd_and_even_kernels_as_numpy() {
        let mut a = vec![0.0; 20];
        a[10] = 1.0;
        let hit = |m| -> Vec<usize> {
            boxcar(&a, m)
                .iter()
                .enumerate()
                .filter(|(_, v)| **v != 0.0)
                .map(|(k, _)| k)
                .collect()
        };
        assert_eq!(hit(4), vec![9, 10, 11, 12]);
        assert_eq!(hit(5), vec![8, 9, 10, 11, 12]);
    }

    /// A run-up from rest whose ripple sweeps up from zero: the count
    /// follows the planted speed past its first turns.
    #[test]
    fn the_order_count_follows_a_planted_run_up() {
        let fs = 20_000.0;
        let n = 1040;
        let (w_inf, tau) = (2.0 * PI * 500.0, 0.02);
        let w = |t: f64| w_inf * (1.0 - (-t / tau).exp());
        let th = |t: f64| w_inf * (t - tau * (1.0 - (-t / tau).exp()));
        let i: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                0.2 + 0.8 * (-t / tau).exp() + 0.03 * (6.0 * th(t)).cos()
            })
            .collect();
        let rough: Vec<f64> = (0..n).map(|k| 1.1 * w(k as f64 / fs)).collect();
        let c = count_speed(&i, &rough, 0.04, 6, fs).expect("counts");
        assert!(c.tc.len() > 60, "{}", c.tc.len());
        // the local median runs short of its window over the first 15 crossings
        for k in (500..n - 60).step_by(50) {
            let want = w(k as f64 / fs);
            assert!(
                (c.speed[k] / want - 1.0).abs() < 0.02,
                "{k}: {} {want}",
                c.speed[k]
            );
        }
        assert_eq!(count_speed(&[], &[], 0.04, 6, fs), None);
        let mut nan = i.clone();
        nan[500] = f64::NAN;
        assert_eq!(count_speed(&nan, &rough, 0.04, 6, fs), None);
    }
}
