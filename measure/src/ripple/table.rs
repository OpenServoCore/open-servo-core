//! What a run-up's crossings give, oscnb `ripple.rev_table`,
//! `flag_windows` and `fit_table` ported: turn windows, one revolution of
//! crossings each, and acceleration fits, a quadratic of the counted angle
//! on time. Crossings are in seconds.

use core::f64::consts::PI;

use super::order::arange;
use crate::fit::{polyfit, polyval, sd};

/// s, the +/- half width of an acceleration fit.
pub const FIT_HALF_S: f64 = 0.005;
/// s, a fit center every 0.5 ms.
pub const FIT_STEP_S: f64 = 0.0005;
/// s: the first fit's window opens on the first crossing exactly, which
/// rounding would otherwise keep or drop at random.
pub const FIT_EDGE_S: f64 = 1e-12;
/// Windows either side a turn window is held against.
pub const FLAG_WINDOWS: usize = 15;
/// A turn window this far off its neighbors' quadratic is flagged.
pub const FLAG_TOL: f64 = 0.05;

/// One revolution of crossings: its middle, s, and its speed, rad/s.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TurnWindow {
    pub t: f64,
    pub w: f64,
}

/// A turn window from every crossing to the one `per_rev` on.
pub fn turn_windows(tc: &[f64], per_rev: usize) -> Vec<TurnWindow> {
    if per_rev == 0 {
        return Vec::new();
    }
    (0..tc.len().saturating_sub(per_rev))
        .map(|j| TurnWindow {
            t: (tc[j] + tc[j + per_rev]) / 2.0,
            w: 2.0 * PI / (tc[j + per_rev] - tc[j]),
        })
        .collect()
}

/// True where a turn window sits within [`FLAG_TOL`] of the quadratic in
/// time through the [`FLAG_WINDOWS`] windows either side, refitted three
/// times without the windows off it. A window that holds a crossing too
/// many or too few reads 7/6 or 5/6 of its neighbors.
pub fn flag_windows(windows: &[TurnWindow]) -> Vec<bool> {
    let n = FLAG_WINDOWS;
    (0..windows.len())
        .map(|j| {
            let near = &windows[j.saturating_sub(n)..(j + n + 1).min(windows.len())];
            let tt: Vec<f64> = near.iter().map(|x| x.t - windows[j].t).collect();
            let ww: Vec<f64> = near.iter().map(|x| x.w).collect();
            let mut keep = vec![true; near.len()];
            let mut p: Option<[f64; 3]> = None;
            for _ in 0..3 {
                if keep.iter().filter(|k| **k).count() < 5 {
                    break;
                }
                let (x, y): (Vec<f64>, Vec<f64>) = (0..near.len())
                    .filter(|&k| keep[k])
                    .map(|k| (tt[k], ww[k]))
                    .unzip();
                p = polyfit::<3>(&x, &y);
                let Some(c) = p else { break };
                keep = (0..near.len())
                    .map(|k| (ww[k] / polyval(&c, tt[k]) - 1.0).abs() <= FLAG_TOL)
                    .collect();
            }
            p.is_some_and(|c| (windows[j].w / polyval(&c, 0.0) - 1.0).abs() <= FLAG_TOL)
        })
        .collect()
}

/// One acceleration fit: its center, s, the speed and acceleration of the
/// quadratic there, its crossings, the angle residual's sd in degrees, the
/// dropped rows under it, and the mean current over its rows.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AccelFit {
    pub t: f64,
    pub w: f64,
    pub alpha: f64,
    pub n: usize,
    pub res_deg: f64,
    pub open_rows: usize,
    pub i: f64,
}

/// The acceleration fits over `tc`: a quadratic of the counted angle on
/// time over +/- [`FIT_HALF_S`] every [`FIT_STEP_S`], two turns of
/// crossings at least, with `current`'s mean over the same rows. A
/// crossing on a window's edge is inside it.
pub fn fit_table(
    tc: &[f64],
    per_rev: usize,
    current: &[f64],
    dropped: &[bool],
    fs: f64,
) -> Vec<AccelFit> {
    if tc.len() < 2 || per_rev == 0 {
        return Vec::new();
    }
    let step = 2.0 * PI / per_rev as f64;
    let angle: Vec<f64> = (0..tc.len()).map(|k| k as f64 * step).collect();
    let cum = |f: &dyn Fn(usize) -> f64, n: usize| -> Vec<f64> {
        let mut acc = 0.0;
        let mut out = Vec::with_capacity(n + 1);
        out.push(0.0);
        for k in 0..n {
            acc += f(k);
            out.push(acc);
        }
        out
    };
    let cum_i = cum(&|k| current[k], current.len());
    let cum_open = cum(&|k| dropped[k] as u8 as f64, dropped.len());
    let (h, last) = (FIT_HALF_S, tc[tc.len() - 1]);
    let mut out = Vec::new();
    for tm in arange(tc[0] + h, last - h, FIT_STEP_S) {
        let (x, y): (Vec<f64>, Vec<f64>) = tc
            .iter()
            .zip(&angle)
            .filter(|(t, _)| **t >= tm - h - FIT_EDGE_S && **t <= tm + h + FIT_EDGE_S)
            .map(|(t, a)| (t - tm, *a))
            .unzip();
        if x.len() < 2 * per_rev + 1 {
            continue;
        }
        let Some(p) = polyfit::<3>(&x, &y) else {
            continue;
        };
        let res: Vec<f64> = x.iter().zip(&y).map(|(x, y)| y - polyval(&p, *x)).collect();
        let a = ((tm - h) * fs).round_ties_even();
        let b = ((tm + h) * fs).round_ties_even();
        if !(a >= 0.0 && b > a && b < cum_i.len() as f64 && b < cum_open.len() as f64) {
            continue;
        }
        let (a, b) = (a as usize, b as usize);
        out.push(AccelFit {
            t: tm,
            w: p[1],
            alpha: 2.0 * p[2],
            n: x.len(),
            res_deg: sd(&res, 0).unwrap_or(f64::NAN).to_degrees(),
            open_rows: (cum_open[b] - cum_open[a]) as usize,
            i: (cum_i[b] - cum_i[a]) / (b - a) as f64,
        });
    }
    out
}
