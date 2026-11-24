//! The running friction line from closed-loop Velocity cruises at mid
//! travel: the mean winding current a steady speed holds, in the
//! firmware's own reading, against the speed, both directions pooled.
//! Pure: a leg's rows in, its numbers and the line out.
//!
//! A leg's rows are cut three ways:
//! - from [`SETTLE_SHARE`] into the leg: the speed has settled;
//! - inside the middle of the measured travel, [`BAND_SHARE`] in from
//!   each end: the low end drags more;
//! - inside the speed's window, the positions both directions crossed
//!   after settling: a short leg's up and down stretches would otherwise
//!   read different places of the travel.
//!
//! The reading is the firmware's: the crest sample over the polled bias
//! times the board's settle gain, less the installed window term
//! `g (1 - D) V_on` on the tap difference's median, signed by the drive.
//! Under [`READ_SHARE_MIN`] of the rows read, the read rows are the wide
//! pulses and read high, so the leg's mean is the all-tick estimate: the
//! read rows regressed on the duty through a first-order winding lag, the
//! line's mean over every row. Inertia comes off through the installed
//! `j_ff` on the table speed's change between the stretch's halves.
//!
//! A leg counts when its speed is held near the goal and its inertia
//! correction is small ([`INERTIA_MAX_MA`]). The correction takes the
//! stretch's mean acceleration times the inertia off the stretch's mean
//! current, which is exact for that mean whatever the shape of the speed's
//! change; what is left is the inertia's own error times the correction,
//! which the bound keeps small. A leg still easing toward its speed is
//! therefore no worse than a flat one: the speed's halves are reported,
//! not judged.

use std::ops::Range;

use super::polyfit;
use super::window::Settle;
use crate::capture::rows::TelFrame;
use crate::scorecard::Lin;
use crate::steps::{Check, Verdict};

/// The middle of the travel the rows are kept in: this share in from each
/// end, the middle third. Closed-loop cruises on the mg90-d read 7 to
/// 14 mA more going up below about a quarter of the travel; the high end
/// reads as the middle, kept out the same for symmetry.
pub const BAND_SHARE: f64 = 1.0 / 3.0;
/// A leg's first quarter is its reversal and the loop's settling: the
/// record's cut (the cruise-friction notebook's steady rows).
pub const SETTLE_SHARE: f64 = 0.25;
/// The shortest stretch a leg is read over, s: the spin step's steady
/// stretch.
pub const KEPT_MIN_S: f64 = 0.100;
/// Under this share of rows read the read rows' mean is replaced by the
/// all-tick estimate.
pub const READ_SHARE_MIN: f64 = 0.98;
/// The winding lag's grid, us, and the duty's lags, rows: the dev-v006
/// winding's L/R and one row of the duty's own lag, bracketed.
pub const TAU_US: [f64; 7] = [50.0, 70.0, 100.0, 130.0, 160.0, 200.0, 250.0];
pub const LAGS: [usize; 2] = [0, 1];
/// The steadiness rule, at every speed: the stretch's speed within this
/// share of the goal (the record's held test), and its inertia current
/// under [`INERTIA_MAX_MA`], the size of a leg's mean's repeat, so the
/// error the correction can carry, a share of the correction, stays well
/// under the scatter it hides in.
pub const HELD_SHARE: f64 = 0.05;
pub const INERTIA_MAX_MA: f64 = 1.5;
/// The line's checks: this many speeds with a valid leg; the lowest at
/// most this share of the ceiling, so the offset is no long
/// extrapolation; each speed's mean residual within this, mA: twice a
/// leg's mean's repeat of 1.0 to 1.5 mA.
pub const SPEEDS_MIN: usize = 3;
pub const LOW_SHARE_MAX: f64 = 0.4;
pub const RESIDUAL_MAX_MA: f64 = 3.0;
/// The directions' check: the mean over the speeds valid both ways of each
/// speed's up less down, within this, mA, over at least
/// [`DIRS_SPEEDS_MIN`] speeds. What it catches, a current zero off by `b`
/// or a load one way, reads `+b` up and `-b` down at every speed alike, so
/// the mean holds all of it, while a single speed's difference of two
/// legs a way carries their scatter: at the slowest speed, read through
/// the all-tick estimate, legs repeat to 1.5 to 2 mA and its difference
/// swings past the bound either way on a healthy servo. A single speed
/// past the bound is said, not refused.
pub const DIRS_MAX_MA: f64 = 3.0;
pub const DIRS_SPEEDS_MIN: usize = 2;

/// What a leg's rows are read through: the servo's installed constants and
/// the board's.
#[derive(Clone, Debug, PartialEq)]
pub struct Reading {
    pub counts_per_a: f64,
    /// `g_window_q016`, ccounts per vcount, Q0.16.
    pub g_window_q016: f64,
    /// `j_ff_q88`, ccounts per (count/s per MEDIUM tick), Q8.8.
    pub j_ff_q88: f64,
    pub pwm_arr: f64,
    pub tick_hz: f64,
    pub medium_hz: f64,
    pub v_window_min_ticks: u16,
    /// The bridge's shortest pulse, ticks: a narrower one drives nothing.
    pub bridge_min_ticks: f64,
    pub settle: Settle,
    pub counts_per_motor_rad: f64,
    /// The speed the shares are of, counts/s.
    pub ceiling_cps: f64,
}

impl Reading {
    pub fn ma(&self, counts: f64) -> f64 {
        1e3 * counts / self.counts_per_a
    }

    /// Inertia current at `accel` counts/s^2, ccounts.
    pub fn inertia_counts(&self, accel: f64) -> f64 {
        self.j_ff_q88 / 256.0 * accel / self.medium_hz
    }
}

/// One leg as captured: its rows, the row's tick its place from the leg's
/// first row; the goal, counts/s, signed; the rows it was planned for;
/// the current bias polled after it.
#[derive(Copy, Clone, Debug)]
pub struct Leg<'a> {
    pub goal_cps: f64,
    pub rows: &'a [TelFrame],
    pub planned_rows: usize,
    pub bias_counts: f64,
}

impl Leg<'_> {
    pub fn dir(&self) -> i8 {
        if self.goal_cps >= 0.0 { 1 } else { -1 }
    }

    fn settle_tick(&self) -> u64 {
        (SETTLE_SHARE * self.planned_rows as f64) as u64
    }
}

/// The middle of `travel`, linearized counts.
pub fn band(travel: (f64, f64)) -> (f64, f64) {
    let t = travel.1 - travel.0;
    (travel.0 + BAND_SHARE * t, travel.1 - BAND_SHARE * t)
}

/// The positions a leg's settled rows cross inside `within`.
pub fn settled_span(leg: &Leg, lin: &Lin, within: (f64, f64)) -> Option<(f64, f64)> {
    let at = leg.settle_tick();
    leg.rows
        .iter()
        .filter(|f| f.tick >= at)
        .filter_map(|f| f.pos.map(|p| lin.counts(p)))
        .filter(|p| (within.0..=within.1).contains(p))
        .fold(None, |s: Option<(f64, f64)>, p| {
            Some(s.map_or((p, p), |(lo, hi)| (lo.min(p), hi.max(p))))
        })
}

/// A speed's window from its legs' settled spans, each with its
/// direction: the positions each direction's legs together crossed,
/// intersected over the directions. None when they cross no common
/// stretch.
pub fn window(spans: &[(i8, (f64, f64))]) -> Option<(f64, f64)> {
    let by = |d: i8| {
        spans
            .iter()
            .filter(|(s, _)| *s == d)
            .fold(None, |u: Option<(f64, f64)>, (_, (lo, hi))| {
                Some(u.map_or((*lo, *hi), |(a, b)| (a.min(*lo), b.max(*hi))))
            })
    };
    let w = match (by(1), by(-1)) {
        (Some(u), Some(d)) => (u.0.max(d.0), u.1.min(d.1)),
        (Some(x), None) | (None, Some(x)) => x,
        (None, None) => return None,
    };
    (w.0 < w.1).then_some(w)
}

/// A leg read over its stretch.
#[derive(Clone, Debug, PartialEq)]
pub struct LegRead {
    pub dir: i8,
    pub goal_cps: f64,
    /// The stretch: its rows of the leg's, from its first tick.
    pub rows: usize,
    pub first_tick: u64,
    pub pos: (f64, f64),
    /// The tap difference's median over the stretch's wide rows, counts.
    pub v_on_counts: f64,
    pub read_share: f64,
    pub i_read_ma: f64,
    /// The all-tick estimate, its lag and time constant; None when it has
    /// too few read rows.
    pub all_ticks: Option<AllTicks>,
    /// The mean the leg counts by, before inertia, and after.
    pub i_used_ma: f64,
    pub all_ticks_used: bool,
    pub speed_cps: f64,
    pub rad_s: f64,
    /// The second half's table speed over the first's, less 1.
    pub halves: f64,
    pub accel_cps2: f64,
    pub i_inertia_ma: f64,
    pub i_ma: f64,
}

impl LegRead {
    /// The steadiness rule's reasons against the leg; none when it counts.
    pub fn fails(&self) -> Vec<String> {
        let mut why = Vec::new();
        let held = (self.speed_cps / self.goal_cps.abs() - 1.0).abs();
        if held.is_nan() || held > HELD_SHARE {
            why.push(format!(
                "speed {:.0} counts/s, {:.1}% off the goal {:.0} (bound {:.0}%)",
                self.speed_cps,
                100.0 * held,
                self.goal_cps.abs(),
                100.0 * HELD_SHARE
            ));
        }
        if self.i_inertia_ma.is_nan() || self.i_inertia_ma.abs() > INERTIA_MAX_MA {
            why.push(format!(
                "inertia current {:.2} mA (bound {INERTIA_MAX_MA} mA)",
                self.i_inertia_ma
            ));
        }
        why
    }
}

/// The all-tick estimate's fit.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AllTicks {
    /// ccounts.
    pub counts: f64,
    pub lag: usize,
    pub tau_us: f64,
    pub mse: f64,
}

/// The first-order lag `y[k] = (1 - a) u[k] + a y[k - 1]`, from rest.
fn lagged(u: &[f64], a: f64) -> Vec<f64> {
    let mut acc = 0.0;
    u.iter()
        .map(|x| {
            acc = (1.0 - a) * x + a * acc;
            acc
        })
        .collect()
}

/// The all-tick mean over `kept` of a leg's signed reading `i` (ccounts,
/// read where `valid`), from its effective duty `de`: the read rows
/// regressed on `de` through the lag and time constant that fit them
/// best, the line's mean over every kept row. None with under two read
/// rows or a singular fit.
pub fn all_ticks(
    i: &[f64],
    de: &[f64],
    valid: &[bool],
    kept: Range<usize>,
    tick_hz: f64,
) -> Option<AllTicks> {
    let dt_us = 1e6 / tick_hz;
    let mut best: Option<AllTicks> = None;
    for lag in LAGS {
        let u: Vec<f64> = (0..de.len())
            .map(|k| if k >= lag { de[k - lag] } else { 0.0 })
            .collect();
        for tau in TAU_US {
            let y = lagged(&u, (-dt_us / tau).exp());
            let (x, z): (Vec<f64>, Vec<f64>) = kept
                .clone()
                .filter(|&k| valid[k])
                .map(|k| (y[k], i[k]))
                .unzip();
            if x.len() < 2 {
                return None;
            }
            let Some([c0, c1]) = super::lstsq(&x.iter().map(|&v| [1.0, v]).collect::<Vec<_>>(), &z)
            else {
                continue;
            };
            let mse = x
                .iter()
                .zip(&z)
                .map(|(v, w)| (w - c0 - c1 * v).powi(2))
                .sum::<f64>()
                / x.len() as f64;
            if best.is_none_or(|b| mse < b.mse) {
                let n = kept.len() as f64;
                let ym = kept.clone().map(|k| y[k]).sum::<f64>() / n;
                best = Some(AllTicks {
                    counts: c0 + c1 * ym,
                    lag,
                    tau_us: tau,
                    mse,
                });
            }
        }
    }
    best
}

fn slope(t: &[f64], y: &[f64]) -> Option<f64> {
    polyfit::<2>(t, y).map(|[_, b]| b)
}

/// A leg read over the rows of `window` from its settled tick on, the
/// stretch from the first such row to the last. Refused, said why, when
/// the stretch is under [`KEPT_MIN_S`], has no wide row to read the taps
/// on, or no read row.
pub fn read(leg: &Leg, lin: &Lin, window: (f64, f64), r: &Reading) -> Result<LegRead, String> {
    let at = leg.settle_tick();
    let pos: Vec<Option<f64>> = leg
        .rows
        .iter()
        .map(|f| f.pos.map(|p| lin.counts(p)))
        .collect();
    let inside = |k: usize| {
        leg.rows[k].tick >= at && pos[k].is_some_and(|p| (window.0..=window.1).contains(&p))
    };
    let (Some(a), Some(b)) = (
        (0..leg.rows.len()).find(|&k| inside(k)),
        (0..leg.rows.len()).rev().find(|&k| inside(k)),
    ) else {
        return Err(format!(
            "no settled row inside the window {:.0}..{:.0}",
            window.0, window.1
        ));
    };
    let kept = a..b + 1;
    let n = kept.len();
    if (n as f64) < KEPT_MIN_S * r.tick_hz {
        return Err(format!(
            "{:.0} ms inside the window {:.0}..{:.0}, under {:.0} ms",
            1e3 * n as f64 / r.tick_hz,
            window.0,
            window.1,
            1e3 * KEPT_MIN_S
        ));
    }
    let s = leg.dir() as f64;
    let duty: Vec<i16> = leg
        .rows
        .iter()
        .map(|f| f.duty_counts.unwrap_or(0))
        .collect();
    let taps: Vec<f64> = kept
        .clone()
        .filter(|&k| duty[k].unsigned_abs() >= r.v_window_min_ticks)
        .filter_map(|k| {
            let f = &leg.rows[k];
            Some((f.vmotor_a? as f64 - f.vmotor_b? as f64) * duty[k].signum() as f64)
        })
        .collect();
    let v_on = super::median(&taps).ok_or("no wide row in the stretch to read the taps on")?;
    let gq = r.g_window_q016 / 65536.0;
    let mut i = vec![0.0; leg.rows.len()];
    let mut de = vec![0.0; leg.rows.len()];
    let mut valid = vec![false; leg.rows.len()];
    for (k, f) in leg.rows.iter().enumerate() {
        let h = duty[k].unsigned_abs();
        let sd = duty[k].signum() as f64 * s;
        if let Some(raw) = f.current_raw.filter(|_| f.window_valid) {
            let crest = (raw as f64 - leg.bias_counts) * r.settle.gain(h);
            i[k] = (crest - gq * (1.0 - h as f64 / r.pwm_arr) * v_on) * sd;
            valid[k] = true;
        }
        let on = if (h as f64) >= r.bridge_min_ticks {
            h as f64
        } else {
            0.0
        };
        de[k] = on * sd / r.pwm_arr;
    }
    let read_rows: Vec<f64> = kept.clone().filter(|&k| valid[k]).map(|k| i[k]).collect();
    let read_share = read_rows.len() as f64 / n as f64;
    let i_read = super::mean(&read_rows).ok_or("no read row in the stretch")?;
    let all = all_ticks(&i, &de, &valid, kept.clone(), r.tick_hz);
    let all_used = read_share < READ_SHARE_MIN;
    let used = if all_used {
        all.ok_or("too few read rows for the all-tick estimate")?
            .counts
    } else {
        i_read
    };
    let t: Vec<f64> = (0..n).map(|k| k as f64 / r.tick_hz).collect();
    let p: Vec<f64> = kept.clone().map(|k| pos[k].unwrap_or(f64::NAN)).collect();
    if p.iter().any(|x| x.is_nan()) {
        return Err("a row in the stretch carries no pot".into());
    }
    let half = n / 2;
    let fit = |t: &[f64], y: &[f64]| slope(t, y).map(|v| v * s).ok_or("no speed fit");
    let cps = fit(&t, &p)?;
    let v1 = fit(&t[..half], &p[..half])?;
    let v2 = fit(&t[half..], &p[half..])?;
    let accel = (v2 - v1) / (n as f64 / 2.0 / r.tick_hz);
    let i_acc = r.inertia_counts(accel);
    let (lo, hi) = p
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &x| {
            (a.min(x), b.max(x))
        });
    Ok(LegRead {
        dir: leg.dir(),
        goal_cps: leg.goal_cps,
        rows: n,
        first_tick: leg.rows[a].tick,
        pos: (lo, hi),
        v_on_counts: v_on,
        read_share,
        i_read_ma: r.ma(i_read),
        all_ticks: all.map(|x| AllTicks {
            counts: r.ma(x.counts),
            ..x
        }),
        i_used_ma: r.ma(used),
        all_ticks_used: all_used,
        speed_cps: cps,
        rad_s: cps / r.counts_per_motor_rad,
        halves: v2 / v1 - 1.0,
        accel_cps2: accel,
        i_inertia_ma: r.ma(i_acc),
        i_ma: r.ma(used - i_acc),
    })
}

/// A valid leg as the line takes it: its speed's planned goal, counts/s,
/// the group it is weighted in; its direction, speed and current.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point {
    pub goal_cps: f64,
    pub dir: i8,
    pub rad_s: f64,
    pub i_ma: f64,
}

/// One speed's share of the line.
#[derive(Clone, Debug, PartialEq)]
pub struct BySpeed {
    pub goal_cps: f64,
    pub legs: usize,
    pub rad_s: f64,
    pub i_ma: f64,
    pub residual_ma: f64,
    /// Mean up and mean down, where the speed has legs both ways.
    pub up_ma: Option<f64>,
    pub down_ma: Option<f64>,
}

/// The friction line `i = coulomb + viscous x rad/s`, with standard
/// errors: each the larger of the legs' scatter about the line and the
/// speeds' means about it, [`Line::coulomb_se_from`] and
/// [`Line::viscous_se_from`] naming which.
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub coulomb_ma: f64,
    pub coulomb_se_ma: f64,
    pub viscous_ma_s_per_rad: f64,
    pub viscous_se: f64,
    /// The errors from the legs' weighted residuals, legs less two degrees
    /// of freedom.
    pub coulomb_se_legs_ma: f64,
    pub viscous_se_legs: f64,
    /// The errors from the speeds' mean residuals, speeds less two degrees
    /// of freedom: None under three speeds. A misfit between speeds the
    /// legs' count would understate.
    pub coulomb_se_speeds_ma: Option<f64>,
    pub viscous_se_speeds: Option<f64>,
    pub coulomb_se_from: &'static str,
    pub viscous_se_from: &'static str,
    pub legs: usize,
    pub rad_s: (f64, f64),
    pub rms_ma: f64,
    pub speeds: Vec<BySpeed>,
}

fn group(points: &[Point]) -> Vec<(f64, Vec<&Point>)> {
    let mut goals: Vec<f64> = points.iter().map(|p| p.goal_cps.abs()).collect();
    goals.sort_by(f64::total_cmp);
    goals.dedup();
    goals
        .into_iter()
        .map(|g| (g, points.iter().filter(|p| p.goal_cps.abs() == g).collect()))
        .collect()
}

/// The line through `points`, each speed weighted once whatever its count
/// of valid legs, both directions pooled; the standard errors the larger
/// of the legs' and the speeds' ([`Line`]). None under two speeds or three
/// legs.
pub fn line(points: &[Point]) -> Option<Line> {
    let groups = group(points);
    if groups.len() < 2 || points.len() < 3 {
        return None;
    }
    let w: Vec<f64> = points
        .iter()
        .map(|p| {
            1.0 / groups
                .iter()
                .find(|(g, _)| *g == p.goal_cps.abs())
                .map_or(1, |(_, v)| v.len()) as f64
        })
        .collect();
    let (mut a, mut b) = ([[0.0; 2]; 2], [0.0; 2]);
    for (p, wk) in points.iter().zip(&w) {
        let x = [1.0, p.rad_s];
        for r in 0..2 {
            b[r] += wk * x[r] * p.i_ma;
            for c in 0..2 {
                a[r][c] += wk * x[r] * x[c];
            }
        }
    }
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if det.is_nan() || det == 0.0 {
        return None;
    }
    let inv = [
        [a[1][1] / det, -a[0][1] / det],
        [-a[1][0] / det, a[0][0] / det],
    ];
    let c0 = inv[0][0] * b[0] + inv[0][1] * b[1];
    let c1 = inv[1][0] * b[0] + inv[1][1] * b[1];
    let resid: Vec<f64> = points.iter().map(|p| p.i_ma - c0 - c1 * p.rad_s).collect();
    let s2 =
        resid.iter().zip(&w).map(|(r, wk)| wk * r * r).sum::<f64>() / (points.len() - 2) as f64;
    let mean = |v: &[f64]| super::mean(v).unwrap_or(f64::NAN);
    let speeds: Vec<BySpeed> = groups
        .iter()
        .map(|(g, v)| {
            let res: Vec<f64> = points
                .iter()
                .zip(&resid)
                .filter(|(p, _)| p.goal_cps.abs() == *g)
                .map(|(_, r)| *r)
                .collect();
            let by = |d: i8| {
                super::mean(
                    &v.iter()
                        .filter(|p| p.dir == d)
                        .map(|p| p.i_ma)
                        .collect::<Vec<_>>(),
                )
            };
            BySpeed {
                goal_cps: *g,
                legs: v.len(),
                rad_s: mean(&v.iter().map(|p| p.rad_s).collect::<Vec<_>>()),
                i_ma: mean(&v.iter().map(|p| p.i_ma).collect::<Vec<_>>()),
                residual_ma: mean(&res),
                up_ma: by(1),
                down_ma: by(-1),
            }
        })
        .collect();
    let (lo, hi) = points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), p| {
            (a.min(p.rad_s), b.max(p.rad_s))
        });
    let legs_se = ((s2 * inv[0][0]).sqrt(), (s2 * inv[1][1]).sqrt());
    let speeds_se = speeds_se(&speeds);
    let larger = |legs: f64, by: Option<f64>| match by {
        Some(b) if b > legs => (b, SE_SPEEDS),
        _ => (legs, SE_LEGS),
    };
    let (c_se, c_from) = larger(legs_se.0, speeds_se.map(|s| s.0));
    let (v_se, v_from) = larger(legs_se.1, speeds_se.map(|s| s.1));
    Some(Line {
        coulomb_ma: c0,
        coulomb_se_ma: c_se,
        viscous_ma_s_per_rad: c1,
        viscous_se: v_se,
        coulomb_se_legs_ma: legs_se.0,
        viscous_se_legs: legs_se.1,
        coulomb_se_speeds_ma: speeds_se.map(|s| s.0),
        viscous_se_speeds: speeds_se.map(|s| s.1),
        coulomb_se_from: c_from,
        viscous_se_from: v_from,
        legs: points.len(),
        rad_s: (lo, hi),
        rms_ma: (resid.iter().map(|r| r * r).sum::<f64>() / resid.len() as f64).sqrt(),
        speeds,
    })
}

pub const SE_LEGS: &str = "the legs' scatter about the line";
pub const SE_SPEEDS: &str = "the speeds' means about the line";

/// The standard errors of the line's two terms from each speed's mean
/// speed and mean residual: speeds less two degrees of freedom, None
/// under three speeds.
fn speeds_se(speeds: &[BySpeed]) -> Option<(f64, f64)> {
    let g = speeds.len();
    if g < 3 {
        return None;
    }
    let (mut a, mut s) = ([[0.0; 2]; 2], 0.0);
    for b in speeds {
        let x = [1.0, b.rad_s];
        for r in 0..2 {
            for c in 0..2 {
                a[r][c] += x[r] * x[c];
            }
        }
        s += b.residual_ma * b.residual_ma;
    }
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if det.is_nan() || det == 0.0 {
        return None;
    }
    let s2 = s / (g - 2) as f64;
    Some(((s2 * a[1][1] / det).sqrt(), (s2 * a[0][0] / det).sqrt()))
}

pub const SPEEDS: &str = "cruise speeds with a valid leg";
pub const LOWEST: &str = "the lowest valid cruise speed";
pub const RESIDUALS: &str = "the friction line's residuals by speed";
pub const DIRECTIONS: &str = "the cruise directions agree";
pub const DIRECTION_SPEEDS: &str = "each cruise speed's directions";

fn check(name: &str, value: String, bound: String, ok: bool) -> Check {
    Check {
        name: name.into(),
        value,
        bound,
        verdict: if ok { Verdict::Pass } else { Verdict::Fail },
    }
}

/// The line's checks over the valid legs' `points` and the `line` through
/// them; the ceiling the lowest speed is a share of.
pub fn checks(points: &[Point], line: Option<&Line>, ceiling_cps: f64) -> Vec<Check> {
    let groups = group(points);
    let speeds: Vec<String> = groups
        .iter()
        .map(|(g, v)| format!("{g:.0} ({})", v.len()))
        .collect();
    let mut out = vec![check(
        SPEEDS,
        if speeds.is_empty() {
            "none".into()
        } else {
            format!(
                "{} speeds, counts/s (valid legs): {}",
                speeds.len(),
                speeds.join(", ")
            )
        },
        format!("at least {SPEEDS_MIN}"),
        groups.len() >= SPEEDS_MIN,
    )];
    let low = groups.first().map(|(g, _)| *g);
    out.push(check(
        LOWEST,
        low.map_or("no valid leg".into(), |g| {
            format!(
                "{g:.0} counts/s, {:.2} of the ceiling {ceiling_cps:.0}",
                g / ceiling_cps
            )
        }),
        format!("at most {LOW_SHARE_MAX} of the ceiling: the offset read, not extrapolated"),
        low.is_some_and(|g| g <= LOW_SHARE_MAX * ceiling_cps),
    ));
    match line {
        Some(l) => {
            let worst = l
                .speeds
                .iter()
                .map(|s| s.residual_ma.abs())
                .fold(0.0, f64::max);
            out.push(check(
                RESIDUALS,
                l.speeds
                    .iter()
                    .map(|s| format!("{:+.2} mA at {:.0}", s.residual_ma, s.goal_cps))
                    .collect::<Vec<_>>()
                    .join(", "),
                format!("each within {RESIDUAL_MAX_MA} mA"),
                worst <= RESIDUAL_MAX_MA,
            ));
            let both: Vec<(f64, f64)> = l
                .speeds
                .iter()
                .filter_map(|s| Some((s.goal_cps, s.up_ma? - s.down_ma?)))
                .collect();
            let each = both
                .iter()
                .map(|(g, d)| format!("up less down {d:+.2} mA at {g:.0}"))
                .collect::<Vec<_>>()
                .join(", ");
            let mean = super::mean(&both.iter().map(|(_, d)| *d).collect::<Vec<_>>());
            out.push(check(
                DIRECTIONS,
                match mean {
                    None => "no speed has valid legs both ways".into(),
                    Some(m) => format!(
                        "up less down {m:+.2} mA on the mean of {} speeds ({each})",
                        both.len()
                    ),
                },
                format!(
                    "at least {DIRS_SPEEDS_MIN} speeds valid both ways, their mean within {DIRS_MAX_MA} mA"
                ),
                both.len() >= DIRS_SPEEDS_MIN && mean.is_some_and(|m| m.abs() <= DIRS_MAX_MA),
            ));
            let past: Vec<String> = both
                .iter()
                .filter(|(_, d)| d.abs() > DIRS_MAX_MA)
                .map(|(g, d)| format!("{d:+.2} mA at {g:.0}"))
                .collect();
            out.push(Check {
                name: DIRECTION_SPEEDS.into(),
                value: if past.is_empty() {
                    format!("none past the bound of {}", both.len())
                } else {
                    format!("up less down past the bound: {}", past.join(", "))
                },
                bound: format!("each within {DIRS_MAX_MA} mA, said, not judged"),
                verdict: if past.is_empty() {
                    Verdict::Pass
                } else {
                    Verdict::Warn
                },
            });
        }
        None => out.push(check(
            RESIDUALS,
            "no line: under two speeds with a valid leg".into(),
            format!("each within {RESIDUAL_MAX_MA} mA"),
            false,
        )),
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(crate) fn reading() -> Reading {
        Reading {
            counts_per_a: 1135.0,
            g_window_q016: 1178.0,
            j_ff_q88: 862.0,
            pwm_arr: 1200.0,
            tick_hz: 20000.0,
            medium_hz: 2000.0,
            v_window_min_ticks: 93,
            bridge_min_ticks: 27.0,
            settle: Settle {
                start_ticks: 40,
                q15: vec![37138, 35303, 34342, 33918],
            },
            counts_per_motor_rad: 3.7434,
            ceiling_cps: 4025.0,
        }
    }

    /// A leg of `n` rows at `cps` from `p0` with acceleration `acc`, the
    /// duty `h` ticks, a reading of `i_counts` over the bias of 1200, every
    /// row read and its taps 1220 apart; the identity table.
    fn leg_rows(n: usize, p0: f64, cps: f64, acc: f64, h: i16, i_counts: f64) -> Vec<TelFrame> {
        (0..n)
            .map(|k| {
                let t = k as f64 / 20000.0;
                TelFrame {
                    tick: k as u64,
                    window_valid: true,
                    pos: Some((p0 + cps * t + 0.5 * acc * t * t).round() as u16),
                    duty_counts: Some(h * cps.signum() as i16),
                    current_raw: Some((1200.0 + i_counts).round() as u16),
                    vmotor_a: Some(2000 + if cps > 0.0 { 1220 } else { 0 }),
                    vmotor_b: Some(2000 + if cps > 0.0 { 0 } else { 1220 }),
                    ..TelFrame::default()
                }
            })
            .collect()
    }

    fn identity() -> Lin {
        Lin::new(None, None)
    }

    #[test]
    fn the_band_is_the_middle_third_of_the_travel() {
        let (lo, hi) = band((110.0, 3730.0));
        assert!((lo - 1316.67).abs() < 0.01 && (hi - 2523.33).abs() < 0.01);
    }

    /// The rows kept: from a quarter into the leg, inside the window,
    /// first to last; a stretch under 100 ms is refused.
    #[test]
    fn the_rows_are_cut_after_the_settle_and_inside_the_window() {
        let r = reading();
        let rows = leg_rows(12000, 900.0, 4025.0, 0.0, 432, 100.0);
        let leg = Leg {
            goal_cps: 4025.0,
            rows: &rows,
            planned_rows: 12000,
            bias_counts: 1200.0,
        };
        let w = band((110.0, 3730.0));
        let span = settled_span(&leg, &identity(), w).expect("a span");
        assert!(span.0 >= w.0 && span.1 <= w.1, "{span:?}");
        let got = read(&leg, &identity(), w, &r).expect("read");
        // 900 + 4025 t reaches 1317 at 2070 rows, past the settle at 3000
        assert_eq!(got.first_tick, 3000);
        let at_hi = ((w.1.floor() + 0.5 - 900.0) / 4025.0 * 20000.0).ceil() as usize;
        assert!(
            (got.rows as i64 - (at_hi as i64 - 3000)).abs() <= 6,
            "{}",
            got.rows
        );
        assert!(got.pos.0 >= w.0 && got.pos.1 <= w.1);
        assert!((got.speed_cps - 4025.0).abs() < 1.0, "{}", got.speed_cps);
        let short = read(&leg, &identity(), (2400.0, 2500.0), &r).expect_err("short");
        assert!(short.contains("under 100 ms"), "{short}");
        let none = read(&leg, &identity(), (3600.0, 3700.0), &r).expect_err("none");
        assert!(none.contains("no settled row"), "{none}");
    }

    /// The window is what both directions crossed; one direction alone is
    /// its own; none crossed in common is no window.
    #[test]
    fn the_window_is_what_both_directions_crossed() {
        let w = window(&[
            (1, (2034.0, 2290.0)),
            (1, (2030.0, 2296.0)),
            (-1, (1945.0, 2213.0)),
            (-1, (1942.0, 2218.0)),
        ]);
        assert_eq!(w, Some((2030.0, 2218.0)));
        assert_eq!(window(&[(1, (10.0, 20.0))]), Some((10.0, 20.0)));
        assert_eq!(window(&[(1, (10.0, 20.0)), (-1, (30.0, 40.0))]), None);
        assert_eq!(window(&[]), None);
    }

    /// A leg still speeding up carries J a in its current: the inertia
    /// read off the halves through j_ff is taken off, and over the bound
    /// the leg fails.
    #[test]
    fn the_inertia_comes_off_by_the_speed_slope() {
        let r = reading();
        let acc = 20000.0;
        let rows = leg_rows(6000, 1300.0, 2000.0, acc, 300, 60.0);
        let leg = Leg {
            goal_cps: 2000.0,
            rows: &rows,
            planned_rows: 6000,
            bias_counts: 1200.0,
        };
        let got = read(&leg, &identity(), (1000.0, 3000.0), &r).expect("read");
        let want = 862.0 / 256.0 * acc / 2000.0;
        assert!(
            (got.accel_cps2 / acc - 1.0).abs() < 0.01,
            "{}",
            got.accel_cps2
        );
        assert!(
            (got.i_inertia_ma - r.ma(want)).abs() < 0.05,
            "{} against {}",
            got.i_inertia_ma,
            r.ma(want)
        );
        assert!((got.i_ma - (got.i_used_ma - got.i_inertia_ma)).abs() < 1e-12);
        let why = got.fails();
        assert!(why.iter().any(|w| w.contains("inertia")), "{why:?}");
        // steady at the same speed: nothing comes off and the leg counts
        let rows = leg_rows(6000, 1300.0, 2000.0, 0.0, 300, 60.0);
        let leg = Leg { rows: &rows, ..leg };
        let got = read(&leg, &identity(), (1000.0, 3000.0), &r).expect("read");
        assert!(got.i_inertia_ma.abs() < 0.05, "{}", got.i_inertia_ma);
        assert!(got.fails().is_empty(), "{:?}", got.fails());
    }

    /// The reading is the firmware's: the settle gain at the width, the
    /// window term on the taps' median, signed by the drive.
    #[test]
    fn a_leg_reads_the_firmware_reading() {
        let r = reading();
        for (cps, h) in [(2000.0, 300i16), (-2000.0, 300), (2000.0, 60)] {
            let rows = leg_rows(
                8000,
                if cps > 0.0 { 1300.0 } else { 2500.0 },
                cps,
                0.0,
                h,
                80.0,
            );
            let leg = Leg {
                goal_cps: cps,
                rows: &rows,
                planned_rows: 8000,
                bias_counts: 1200.0,
            };
            let got = read(&leg, &identity(), (1000.0, 3000.0), &r);
            if h < 93 {
                assert!(got.expect_err("no wide row").contains("taps"));
                continue;
            }
            let got = got.expect("read");
            let term = 1178.0 / 65536.0 * (1.0 - h as f64 / 1200.0) * 1220.0;
            let want = r.ma(80.0 * r.settle.gain(h as u16) - term);
            assert!(
                (got.i_read_ma - want).abs() < 1e-9,
                "{} {want}",
                got.i_read_ma
            );
            assert_eq!(got.dir, cps.signum() as i8);
            assert!(!got.all_ticks_used);
        }
    }

    /// The all-tick estimate: a winding whose current follows the duty
    /// through a 100 us lag, read only on the wide pulses. The read rows'
    /// mean sits high; the estimate gives the mean over every row.
    #[test]
    fn the_all_tick_estimate_recovers_the_mean_over_every_row() {
        let n = 20000;
        let de: Vec<f64> = (0..n)
            .map(|k| (75.0 + 45.0 * (k as f64 * core::f64::consts::TAU / 40.0).sin()) / 1200.0)
            .collect();
        let y = lagged(
            &std::iter::once(0.0)
                .chain(de[..n - 1].iter().copied())
                .collect::<Vec<_>>(),
            (-50.0f64 / 100.0).exp(),
        );
        let i: Vec<f64> = y.iter().map(|v| -5.0 + 600.0 * v).collect();
        let valid: Vec<bool> = de.iter().map(|d| d * 1200.0 >= 64.0).collect();
        let kept = 1000..n;
        let truth = kept.clone().map(|k| i[k]).sum::<f64>() / kept.len() as f64;
        let read: Vec<f64> = kept.clone().filter(|&k| valid[k]).map(|k| i[k]).collect();
        let read_mean = read.iter().sum::<f64>() / read.len() as f64;
        assert!(
            read_mean - truth > 3.0,
            "the read rows sit high: {read_mean} {truth}"
        );
        let got = all_ticks(&i, &de, &valid, kept, 20000.0).expect("fit");
        assert_eq!((got.lag, got.tau_us), (1, 100.0));
        assert!((got.counts - truth).abs() < 1e-6, "{} {truth}", got.counts);
        assert!(all_ticks(&i, &de, &vec![false; n], 0..n, 20000.0).is_none());
    }

    /// The halves are said, never judged; a leg off its goal or past the
    /// inertia bound fails at any speed.
    #[test]
    fn the_steadiness_rule_by_speed() {
        let leg = |goal: f64, speed: f64, halves: f64| LegRead {
            dir: 1,
            goal_cps: goal,
            rows: 10000,
            first_tick: 0,
            pos: (0.0, 1.0),
            v_on_counts: 1220.0,
            read_share: 1.0,
            i_read_ma: 50.0,
            all_ticks: None,
            i_used_ma: 50.0,
            all_ticks_used: false,
            speed_cps: speed,
            rad_s: speed / 3.7,
            halves,
            accel_cps2: 0.0,
            i_inertia_ma: 0.1,
            i_ma: 49.9,
        };
        // the halves never set a leg aside, at any speed
        assert!(leg(300.0, 300.0, 0.13).fails().is_empty());
        assert!(leg(1500.0, 1500.0, 0.05).fails().is_empty());
        assert!(leg(4025.0, 4025.0, f64::NAN).fails().is_empty());
        assert_eq!(leg(300.0, 330.0, 0.0).fails().len(), 1);
        let mut l = leg(2150.0, 2150.0, 0.0);
        l.i_inertia_ma = -1.6;
        assert_eq!(l.fails().len(), 1, "past the inertia bound");
        let mut l = leg(4025.0, f64::NAN, 0.0);
        l.i_inertia_ma = f64::NAN;
        assert_eq!(l.fails().len(), 2, "a NaN never passes");
    }

    fn points(c: f64, v: f64, wiggle: &[(f64, i8, f64)]) -> Vec<Point> {
        wiggle
            .iter()
            .map(|&(goal, dir, e)| {
                let w = goal / 3.7434;
                Point {
                    goal_cps: goal,
                    dir,
                    rad_s: w,
                    i_ma: c + v * w + e,
                }
            })
            .collect()
    }

    /// The line through legs on a line is that line; each speed counts
    /// once, so a speed with more legs does not pull it; the standard
    /// errors grow with the scatter.
    #[test]
    fn the_line_weights_each_speed_once() {
        let exact = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 0.0),
                (300.0, -1, 0.0),
                (1500.0, 1, 0.0),
                (4025.0, -1, 0.0),
            ],
        );
        let l = line(&exact).expect("line");
        assert!(
            (l.coulomb_ma - 39.0).abs() < 1e-9 && (l.viscous_ma_s_per_rad - 0.04).abs() < 1e-12
        );
        assert!(l.coulomb_se_ma < 1e-6);
        // four legs at 1500 read 1 mA high, one at 300 and one at 4025 on
        // the line: each speed once puts the line through their means
        let mut p = points(39.0, 0.04, &[(300.0, 1, 0.0), (4025.0, 1, 0.0)]);
        p.extend(points(39.0, 0.04, &[(1500.0, 1, 1.0); 4]));
        let l = line(&p).expect("line");
        let w = [300.0, 1500.0, 4025.0].map(|g| g / 3.7434);
        let y = [39.0 + 0.04 * w[0], 40.0 + 0.04 * w[1], 39.0 + 0.04 * w[2]];
        let xm = w.iter().sum::<f64>() / 3.0;
        let ym = y.iter().sum::<f64>() / 3.0;
        let sxy: f64 = w.iter().zip(&y).map(|(x, y)| (x - xm) * (y - ym)).sum();
        let sxx: f64 = w.iter().map(|x| (x - xm).powi(2)).sum();
        assert!((l.viscous_ma_s_per_rad - sxy / sxx).abs() < 1e-12);
        assert!((l.coulomb_ma - (ym - sxy / sxx * xm)).abs() < 1e-9);
        assert!(l.coulomb_se_ma > 0.0 && l.viscous_se > 0.0);
        assert_eq!(l.speeds.len(), 3);
        assert_eq!(l.speeds[1].legs, 4);
        assert!(line(&points(39.0, 0.04, &[(300.0, 1, 0.0), (300.0, -1, 0.0)])).is_none());
    }

    /// The standard errors against the closed forms of the straight line:
    /// the weighted legs' `s^2 / Sxx` and `s^2 (1/W + xbar^2 / Sxx)`, and
    /// the same unweighted on the speeds' means; each term the larger.
    #[test]
    fn the_standard_errors_are_the_larger_of_legs_and_speeds() {
        let closed = |x: &[f64], y: &[f64], w: &[f64], dof: f64| {
            let sw: f64 = w.iter().sum();
            let xm = x.iter().zip(w).map(|(x, w)| w * x).sum::<f64>() / sw;
            let ym = y.iter().zip(w).map(|(y, w)| w * y).sum::<f64>() / sw;
            let sxx: f64 = x.iter().zip(w).map(|(x, w)| w * (x - xm).powi(2)).sum();
            let sxy: f64 = x
                .iter()
                .zip(y)
                .zip(w)
                .map(|((x, y), w)| w * (x - xm) * (y - ym))
                .sum();
            let b = sxy / sxx;
            let a = ym - b * xm;
            let s2 = x
                .iter()
                .zip(y)
                .zip(w)
                .map(|((x, y), w)| w * (y - a - b * x).powi(2))
                .sum::<f64>()
                / dof;
            ((s2 * (1.0 / sw + xm * xm / sxx)).sqrt(), (s2 / sxx).sqrt())
        };
        let near = |a: f64, b: f64| (a - b).abs() <= 1e-9 * b.abs().max(1e-12);
        // the scatter inside each speed, the means on the line: the legs'
        let p = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 1.0),
                (300.0, -1, -1.0),
                (1500.0, 1, 0.7),
                (1500.0, -1, -0.7),
                (1500.0, 1, 0.2),
                (4025.0, 1, 1.3),
                (4025.0, -1, -1.3),
            ],
        );
        let l = line(&p).expect("line");
        let n = |g: f64| p.iter().filter(|q| q.goal_cps == g).count() as f64;
        let x: Vec<f64> = p.iter().map(|q| q.rad_s).collect();
        let y: Vec<f64> = p.iter().map(|q| q.i_ma).collect();
        let w: Vec<f64> = p.iter().map(|q| 1.0 / n(q.goal_cps)).collect();
        let (a_se, b_se) = closed(&x, &y, &w, p.len() as f64 - 2.0);
        assert!(
            near(l.coulomb_se_legs_ma, a_se) && near(l.viscous_se_legs, b_se),
            "{l:?}"
        );
        assert_eq!((l.coulomb_se_from, l.viscous_se_from), (SE_LEGS, SE_LEGS));
        assert!(near(l.coulomb_se_ma, a_se) && near(l.viscous_se, b_se));
        // four like legs at each of three speeds whose means miss the line:
        // the legs' count understates it, the speeds' means do not
        let mut p = Vec::new();
        for (g, off) in [(300.0, 0.8), (2150.0, -1.6), (4025.0, 0.8)] {
            p.extend(points(39.0, 0.04, &[(g, 1, off); 2]));
            p.extend(points(39.0, 0.04, &[(g, -1, off); 2]));
        }
        let l = line(&p).expect("line");
        let xg: Vec<f64> = l.speeds.iter().map(|s| s.rad_s).collect();
        let yg: Vec<f64> = l.speeds.iter().map(|s| s.i_ma).collect();
        let (a_se, b_se) = closed(&xg, &yg, &[1.0; 3], 1.0);
        let (sa, sb) = (
            l.coulomb_se_speeds_ma.expect("three speeds"),
            l.viscous_se_speeds.expect("three speeds"),
        );
        assert!(near(sa, a_se) && near(sb, b_se), "{l:?} {a_se} {b_se}");
        assert!(l.coulomb_se_legs_ma < sa && l.viscous_se_legs < sb);
        assert_eq!(
            (l.coulomb_se_from, l.viscous_se_from),
            (SE_SPEEDS, SE_SPEEDS)
        );
        assert_eq!((l.coulomb_se_ma, l.viscous_se), (sa, sb));
        // two speeds: no speeds' error, the legs' stands
        let two = points(
            39.0,
            0.04,
            &[(300.0, 1, 0.5), (300.0, -1, 0.0), (4025.0, 1, 0.0)],
        );
        let l = line(&two).expect("line");
        assert!(l.coulomb_se_speeds_ma.is_none() && l.coulomb_se_from == SE_LEGS);
    }

    /// The line's checks: three speeds, the lowest at most 0.4 of the
    /// ceiling, residuals within 3 mA, two speeds valid both ways and the
    /// mean of their up less down within 3 mA; a single speed past it is
    /// a warning.
    #[test]
    fn the_line_checks() {
        let ok = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 0.5),
                (300.0, -1, -0.5),
                (1500.0, 1, 0.0),
                (2150.0, -1, 0.0),
                (4025.0, 1, 0.2),
                (4025.0, -1, -0.2),
            ],
        );
        let l = line(&ok);
        let c = checks(&ok, l.as_ref(), 4025.0);
        assert!(c.iter().all(|c| c.verdict == Verdict::Pass), "{c:?}");
        // two speeds, the lowest at 0.53: the count and the low speed fail
        let two = points(
            39.0,
            0.04,
            &[(2150.0, 1, 0.0), (2150.0, -1, 0.0), (4025.0, 1, 0.0)],
        );
        let c = checks(&two, line(&two).as_ref(), 4025.0);
        let failed: Vec<&str> = c
            .iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .map(|c| c.name.as_str())
            .collect();
        assert_eq!(failed, [SPEEDS, LOWEST, DIRECTIONS], "{c:?}");
        // three speeds, only 300 valid both ways: one drag direction would
        // not show
        let one_way = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 0.0),
                (300.0, -1, 0.0),
                (1500.0, 1, 0.0),
                (4025.0, 1, 0.0),
            ],
        );
        let c = checks(&one_way, line(&one_way).as_ref(), 4025.0);
        let d = c.iter().find(|c| c.name == DIRECTIONS).expect("directions");
        assert_eq!(d.verdict, Verdict::Fail, "{d:?}");
        assert!(d.value.contains("+0.00 mA at 300"), "{d:?}");
        // up 4 mA over down at every speed, as a current zero off by 2 mA
        // reads: the mean holds it all, refused
        let offset = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 2.0),
                (300.0, -1, -2.0),
                (1500.0, 1, 2.0),
                (1500.0, -1, -2.0),
                (4025.0, 1, 2.0),
                (4025.0, -1, -2.0),
            ],
        );
        let c = checks(&offset, line(&offset).as_ref(), 4025.0);
        let d = c.iter().find(|c| c.name == DIRECTIONS).expect("directions");
        assert_eq!(d.verdict, Verdict::Fail, "{d:?}");
        assert!(
            d.value.contains("+4.00 mA on the mean of 3 speeds"),
            "{d:?}"
        );
        let w = c.iter().find(|c| c.name == DIRECTION_SPEEDS).expect("each");
        assert_eq!(w.verdict, Verdict::Warn, "{w:?}");
        // one speed 3.5 mA apart, the others near zero: the scatter of two
        // legs a way, said, not refused
        let one = points(
            39.0,
            0.04,
            &[
                (300.0, 1, 1.75),
                (300.0, -1, -1.75),
                (1500.0, 1, 0.1),
                (1500.0, -1, 0.0),
                (4025.0, 1, -0.1),
                (4025.0, -1, 0.0),
            ],
        );
        let c = checks(&one, line(&one).as_ref(), 4025.0);
        let d = c.iter().find(|c| c.name == DIRECTIONS).expect("directions");
        assert_eq!(d.verdict, Verdict::Pass, "{d:?}");
        assert!(
            d.value.contains("+1.17 mA on the mean of 3 speeds"),
            "{d:?}"
        );
        let w = c.iter().find(|c| c.name == DIRECTION_SPEEDS).expect("each");
        assert_eq!(w.verdict, Verdict::Warn, "{w:?}");
        assert!(w.value.contains("+3.50 mA at 300"), "{w:?}");
        assert!(c.iter().all(|c| c.verdict != Verdict::Fail), "{c:?}");
        // no valid leg at all
        let c = checks(&[], None, 4025.0);
        assert!(c.iter().all(|c| c.verdict == Verdict::Fail), "{c:?}");
    }
}
