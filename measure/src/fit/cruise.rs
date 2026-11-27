//! The running friction line from closed-loop Velocity cruises: the
//! current the velocity loop holds at a steady speed, against the goal
//! speed, both directions pooled. The method of record, stage 5
//! cruise-friction (`replay`, `leg`, `leg_numbers` and the line of record,
//! `LINE`), as it stands. Pure: a leg's rows and polls in, its numbers
//! and the line out.
//!
//! The rows carry no current reference, so each leg's is replayed tick by
//! tick from the next row's width, the one unknown of the current loop's
//! known arithmetic: on a tick whose reading the loop took, the PI
//! inverted on the board's integer reading; on a tick without one, the
//! command less the back-EMF feed-forward over the loop's resistance. The
//! reference moves once a MEDIUM period, so it is averaged over the period
//! at the phase where it is flattest. A read row holds its reading, an
//! unread row its period's reference, and the held mean is over the leg's
//! steady rows, its middle half. The line is a least-squares fit of the
//! held means against the goal speed, every leg below the top speed and
//! the top speed's legs whose speed held across their steady rows within
//! [`STEADY_TOP_PCT`]. The current a leg's gain in speed costs through the
//! installed inertia word is reported, not taken off.

use osc_servo_core::estimator::thermal::{NTC_FAULT_CC, UNSET_CC};

use super::window::Settle;
use crate::capture::rows::TelFrame;
use crate::rules::units::FAST_TICKS_PER_MEDIUM;
use crate::scorecard::Lin;
use crate::steps::{Check, Verdict};

/// A leg's steady rows: its middle half, a quarter cut from each end.
pub const STEADY_CUT: usize = 4;
/// Counts of travel a speed window spans at the goal, and the shortest
/// window, s: the pot's own steps.
pub const TRAVEL_COUNTS: f64 = 60.0;
pub const SPEED_WINDOW_MIN_S: f64 = 0.02;
/// Share of the goal a speed window holds within to count as held (when
/// the leg first held, reported). The step's steadiness rule takes the
/// same share for the steady rows' speed.
pub const SPEED_TOL: f64 = 0.05;
/// Copper's slope, per C: the loop's resistance followed from the start's
/// winding estimate to each leg's.
pub const COPPER_PER_C: f64 = 0.0039;
/// The fewest steady rows a leg is read over, s.
pub const KEPT_MIN_S: f64 = 0.100;
/// A top-speed leg counts when its speed held across its steady rows
/// within this, %: its second half's over its first's.
pub const STEADY_TOP_PCT: f64 = 2.0;
/// What a leg's rows are replayed through: the installed set's words, the
/// board's and the scales.
#[derive(Clone, Debug, PartialEq)]
pub struct Reading {
    pub counts_per_a: f64,
    pub g_window_q016: u16,
    pub i_kp_q88: u16,
    pub i_ki_q412: u16,
    /// Tap counts per count/s, Q4.12.
    pub ke_vpc_q: u16,
    /// The winding resistance the loop ran on at the step's start, Q4.12.
    pub r_hat_q12: u16,
    /// `j_ff_q88`, ccounts per (count/s per MEDIUM tick), Q8.8.
    pub j_ff_q88: f64,
    /// The installed friction words, the line the integrator's share is
    /// read against.
    pub fric_fc_counts: f64,
    pub fric_fv_q016: f64,
    pub pwm_arr: u16,
    pub tick_hz: f64,
    /// The bridge's shortest pulse, ticks: a reading after a narrower one
    /// is not taken.
    pub bridge_min_ticks: f64,
    pub settle: Settle,
    pub counts_per_motor_rad: f64,
    /// The move speed the lowest cruise is a share of, counts/s.
    pub ceiling_cps: f64,
}

impl Reading {
    pub fn ma(&self, counts: f64) -> f64 {
        1e3 * counts / self.counts_per_a
    }

    pub fn medium_hz(&self) -> f64 {
        self.tick_hz / FAST_TICKS_PER_MEDIUM
    }

    /// The installed friction line at `cps`, mA.
    pub fn line_ma(&self, cps: f64) -> f64 {
        self.ma(self.fric_fc_counts + self.fric_fv_q016 / 65536.0 * cps.abs())
    }
}

/// A poll beside a leg: the rail, the winding estimate and the current
/// bias.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Poll {
    pub vbus_raw: f64,
    pub t_winding_cc: f64,
    pub bias_counts: f64,
}

/// One leg as captured: its goal, counts/s, signed; its rows, the polls
/// before and after it, and the winding estimate of the step's first poll.
#[derive(Copy, Clone, Debug)]
pub struct Leg<'a> {
    pub goal_cps: f64,
    pub rows: &'a [TelFrame],
    pub before: Poll,
    pub after: Poll,
    pub t_read_cc: f64,
}

impl Leg<'_> {
    pub fn dir(&self) -> i8 {
        if self.goal_cps >= 0.0 { 1 } else { -1 }
    }
}

/// The board's settle gain at half-width `h`, its Q15 word.
fn settle_q15(s: &Settle, h: i64) -> i64 {
    let band = ((h - s.start_ticks as i64).max(0) >> 3) as usize;
    s.q15.get(band).map_or(1 << 15, |&g| g as i64)
}

/// The window term in shunt counts at half-width `h`, the board's
/// integers: `(K0 - k h) >> 16`, `k = (g_arr vbus) >> 24`,
/// `K0 = k arr + 2^15`, `g_arr` the field times `2^24 / arr`.
pub fn term_counts(g_q016: u16, h: i64, vbus: i64, arr: i64) -> i64 {
    let g_arr = g_q016 as i64 * ((1 << 24) / arr);
    let k = (g_arr * vbus) >> 24;
    let k0 = k * arr + (1 << 15);
    (k0 - k * h) >> 16
}

/// The board's reading of a row, whole shunt counts: the settle-gained
/// crest sample over the bias, rounded, less the window term, signed by
/// the drive.
pub fn reading(
    raw: i64,
    bias: i64,
    duty: i64,
    s: &Settle,
    g_q016: u16,
    vbus: i64,
    arr: i64,
) -> i64 {
    let h = duty.abs();
    let gained = ((raw - bias) * settle_q15(s, h)).div_euclid(1 << 14);
    let along = (gained + 1).div_euclid(2);
    let v = along - term_counts(g_q016, h, vbus, arr);
    if duty >= 0 { v } else { -v }
}

/// A leg's replay, signed by its direction: the current reference a row
/// (NaN on the last, which has no next width), the reading on read rows
/// (NaN on the rest), and which rows were read.
#[derive(Clone, Debug, PartialEq)]
pub struct Replay {
    pub i_ref: Vec<f64>,
    pub i_meas: Vec<f64>,
    pub valid: Vec<bool>,
}

fn winding_ok(t: f64) -> bool {
    t.is_finite() && t != UNSET_CC as f64 && t != NTC_FAULT_CC as f64
}

/// The current loop's reference, solved tick by tick from the next row's
/// width: the PI on a reading the loop took, the command less the
/// feed-forward over the resistance on a tick without one. The rail is
/// the two polls' mean, the bias the poll after, the resistance the
/// start's followed by copper's slope to the polls' mean winding estimate.
pub fn replay(leg: &Leg, r: &Reading) -> Result<Replay, String> {
    let (p0, p1) = (leg.before, leg.after);
    if ![p0.t_winding_cc, p1.t_winding_cc, leg.t_read_cc]
        .into_iter()
        .all(winding_ok)
    {
        return Err(
            "the winding estimate reads its sentinel (no NTC curve, or an NTC fault): the loop's resistance cannot be followed to the leg".into(),
        );
    }
    let vbus = (p0.vbus_raw + p1.vbus_raw) / 2.0;
    let t_c = (p0.t_winding_cc + p1.t_winding_cc) / 2.0 / 100.0;
    let r_loop = r.r_hat_q12 as f64 / 4096.0 * (1.0 + COPPER_PER_C * (t_c - leg.t_read_cc / 100.0));
    let bias = p1.bias_counts.round_ties_even() as i64;
    let arr = r.pwm_arr as i64;
    let (kp, ki) = (r.i_kp_q88 as f64 / 256.0, r.i_ki_q412 as f64 / 4096.0);
    let n = leg.rows.len();
    let duty: Vec<i64> = leg
        .rows
        .iter()
        .map(|f| f.duty_counts.map(i64::from).ok_or("a row carries no duty"))
        .collect::<Result<_, _>>()?;
    let valid: Vec<bool> = leg.rows.iter().map(|f| f.window_valid).collect();
    let vb = vbus.round_ties_even() as i64;
    let mut i_meas = vec![f64::NAN; n];
    for (k, f) in leg.rows.iter().enumerate() {
        if valid[k] {
            let raw = f
                .current_raw
                .ok_or("a read row carries no current sample")?;
            i_meas[k] = reading(
                raw as i64,
                bias,
                duty[k],
                &r.settle,
                r.g_window_q016,
                vb,
                arr,
            ) as f64;
        }
    }
    let u_ff = r.ke_vpc_q as f64 / 4096.0 * leg.goal_cps;
    let mut i_ref = vec![f64::NAN; n];
    let mut integ = 0.0;
    for k in 0..n.saturating_sub(1) {
        let after = if k == 0 { 0 } else { duty[k - 1].abs() };
        let no_pulse = after >= 1 && (after as f64) < r.bridge_min_ticks;
        let u = duty[k + 1] as f64 / arr as f64 * vbus;
        if valid[k] && !no_pulse {
            let e = (u - u_ff - integ) / kp;
            i_ref[k] = i_meas[k] + e;
            integ += ki * e;
        } else {
            let v = u - u_ff;
            i_ref[k] = v / r_loop;
            integ = v;
        }
    }
    let s = leg.dir() as f64;
    Ok(Replay {
        i_ref: i_ref.iter().map(|x| x * s).collect(),
        i_meas: i_meas.iter().map(|x| x * s).collect(),
        valid,
    })
}

/// The runs of rows that share a MEDIUM period at `phase` (the ticks
/// rise).
fn periods(tick: &[i64], phase: i64) -> Vec<std::ops::Range<usize>> {
    let d = FAST_TICKS_PER_MEDIUM as i64;
    let mut out = Vec::new();
    let mut a = 0;
    for k in 1..=tick.len() {
        if k == tick.len() || (tick[k] - phase).div_euclid(d) != (tick[a] - phase).div_euclid(d) {
            out.push(a..k);
            a = k;
        }
    }
    out
}

fn finite_mean(v: impl Iterator<Item = f64>) -> f64 {
    let (s, n) = v
        .filter(|x| !x.is_nan())
        .fold((0.0, 0usize), |(s, n), x| (s + x, n + 1));
    if n == 0 { f64::NAN } else { s / n as f64 }
}

/// The MEDIUM period's phase in FAST ticks: the one at which the replayed
/// reference is flattest inside each period (it moves once a period).
/// The phase, the median sd inside a period there, the same at the worst.
pub fn medium_phase(i_ref: &[f64], tick: &[i64]) -> (i64, f64, f64) {
    let sd: Vec<f64> = (0..FAST_TICKS_PER_MEDIUM as i64)
        .map(|ph| {
            let each: Vec<f64> = periods(tick, ph)
                .into_iter()
                .filter_map(|g| super::sd(&i_ref[g], 1))
                .filter(|x| !x.is_nan())
                .collect();
            super::median(&each).unwrap_or(f64::NAN)
        })
        .collect();
    let best = (0..sd.len()).fold(0, |b, k| if sd[k] < sd[b] { k } else { b });
    let worst = sd.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    (best as i64, sd[best], worst)
}

/// Each row's MEDIUM period mean of the reference at `phase`.
fn period_means(i_ref: &[f64], tick: &[i64], phase: i64) -> Vec<f64> {
    let mut out = vec![f64::NAN; i_ref.len()];
    for g in periods(tick, phase) {
        let m = finite_mean(i_ref[g.clone()].iter().copied());
        out[g].fill(m);
    }
    out
}

/// The least-squares slope of `y` on `x`.
fn slope(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let (xm, ym) = (x.iter().sum::<f64>() / n, y.iter().sum::<f64>() / n);
    let (sxy, sxx) = x.iter().zip(y).fold((0.0, 0.0), |(a, b), (xi, yi)| {
        (a + (xi - xm) * (yi - ym), b + (xi - xm) * (xi - xm))
    });
    sxy / sxx
}

/// The speed along the travel over consecutive windows of `rows` rows, a
/// line through the pot each, counts/s.
pub fn window_speeds(pos: &[f64], dir: f64, hz: f64, rows: usize) -> Vec<f64> {
    if rows < 2 || pos.len() < rows {
        return Vec::new();
    }
    let t: Vec<f64> = (0..rows).map(|k| k as f64 / hz).collect();
    (0..=pos.len() - rows)
        .step_by(rows)
        .map(|k| dir * slope(&t, &pos[k..k + rows]))
        .collect()
}

/// A leg read: the replay's checks, its speed, the held mean and what
/// comes off it.
#[derive(Clone, Debug, PartialEq)]
pub struct LegRead {
    pub dir: i8,
    pub goal_cps: f64,
    /// The leg's rows, and its steady rows.
    pub leg_rows: usize,
    pub rows: usize,
    /// The MEDIUM phase found, the reference's sd inside a period there and
    /// at the worst phase, mA.
    pub phase: i64,
    pub sd_in_ma: f64,
    pub sd_worst_ma: f64,
    /// When a speed window first held within [`SPEED_TOL`] of the goal, ms
    /// into the leg; None when none did.
    pub held_in_ms: Option<f64>,
    pub read_share: f64,
    /// The read rows' mean and the held mean over the steady rows.
    pub read_ma: f64,
    pub held_ma: f64,
    /// The installed line at the goal, and the integrator's share over it.
    pub line_ma: f64,
    pub integ_ma: f64,
    /// The table speed over the steady rows, and its second half's over
    /// its first's, less 1.
    pub speed_cps: f64,
    pub halves: f64,
    /// The current the gain in speed costs through the inertia word,
    /// reported.
    pub inertia_ma: f64,
}

impl LegRead {
    /// The reasons against the leg; none when it counts. Every leg's speed
    /// holds within [`SPEED_TOL`] of the goal; a leg at the `top` speed
    /// also holds its speed across its steady rows within
    /// [`STEADY_TOP_PCT`].
    pub fn fails(&self, top: bool) -> Vec<String> {
        let mut why = Vec::new();
        let held = (self.speed_cps / self.goal_cps.abs() - 1.0).abs();
        if held.is_nan() || held > SPEED_TOL {
            why.push(format!(
                "speed {:.0} counts/s, {:.1}% off the goal {:.0} (bound {:.0}%)",
                self.speed_cps,
                100.0 * held,
                self.goal_cps.abs(),
                100.0 * SPEED_TOL
            ));
        }
        let pct = 100.0 * self.halves;
        let steady = pct.abs() < STEADY_TOP_PCT;
        if top && !steady {
            why.push(format!(
                "the top speed still changing: {pct:+.2}% across its steady rows (bound {STEADY_TOP_PCT}%)"
            ));
        }
        why
    }
}

/// A leg replayed and read over its steady rows. Refused, said why, when
/// its steady rows are under [`KEPT_MIN_S`], a row lacks what the replay
/// reads, the ticks do not rise, or no steady row was read.
pub fn read(leg: &Leg, lin: &Lin, r: &Reading) -> Result<LegRead, String> {
    let n = leg.rows.len();
    let sl = n / STEADY_CUT..n - n / STEADY_CUT;
    let m = sl.len();
    if (m as f64) < KEPT_MIN_S * r.tick_hz {
        return Err(format!(
            "{:.0} ms of steady rows, under {:.0} ms",
            1e3 * m as f64 / r.tick_hz,
            1e3 * KEPT_MIN_S
        ));
    }
    let tick: Vec<i64> = leg.rows.iter().map(|f| f.tick as i64).collect();
    if tick.windows(2).any(|w| w[1] <= w[0]) {
        return Err("the leg's ticks do not rise".into());
    }
    let pos: Vec<f64> = leg
        .rows
        .iter()
        .map(|f| f.pos.map(|p| lin.counts(p)).ok_or("a row carries no pot"))
        .collect::<Result<_, _>>()?;
    let rep = replay(leg, r)?;
    let s = leg.dir() as f64;
    let goal = leg.goal_cps.abs();
    let hz = r.tick_hz;
    let (phase, sd_in, sd_worst) = medium_phase(&rep.i_ref[..n - 1], &tick[..n - 1]);
    let i_blk = period_means(&rep.i_ref, &tick, phase);
    let w = (SPEED_WINDOW_MIN_S.max(TRAVEL_COUNTS / goal) * hz) as usize;
    let held_in_ms = window_speeds(&pos, s, hz, w)
        .iter()
        .position(|v| (v / goal - 1.0).abs() < SPEED_TOL)
        .map(|k| (k * w) as f64 / hz * 1e3);
    let read_rows = sl.clone().filter(|&k| rep.valid[k]).count();
    if read_rows == 0 {
        return Err("no steady row was read".into());
    }
    let held = finite_mean(sl.clone().map(|k| {
        if rep.valid[k] {
            rep.i_meas[k]
        } else {
            i_blk[k]
        }
    }));
    let read_mean = finite_mean(sl.clone().filter(|&k| rep.valid[k]).map(|k| rep.i_meas[k]));
    let t: Vec<f64> = sl.clone().map(|k| tick[k] as f64 / hz).collect();
    let p = &pos[sl.clone()];
    let half = m / 2;
    let cps = s * slope(&t, p);
    let v1 = s * slope(&t[..half], &p[..half]);
    let v2 = s * slope(&t[half..], &p[half..]);
    let pct = (v2 / v1 - 1.0) * 100.0;
    let dv = cps * (pct / 100.0) / (1.0 + pct / 200.0);
    let inertia = r.ma(r.j_ff_q88 / 256.0 * dv / (m as f64 / 2.0 / hz) / r.medium_hz());
    let line = r.line_ma(goal);
    let held_ma = r.ma(held);
    Ok(LegRead {
        dir: leg.dir(),
        goal_cps: leg.goal_cps,
        leg_rows: n,
        rows: m,
        phase,
        sd_in_ma: r.ma(sd_in),
        sd_worst_ma: r.ma(sd_worst),
        held_in_ms,
        read_share: read_rows as f64 / m as f64,
        read_ma: r.ma(read_mean),
        held_ma,
        line_ma: line,
        integ_ma: r.ma(finite_mean(sl.map(|k| i_blk[k]))) - line,
        speed_cps: cps,
        halves: pct / 100.0,
        inertia_ma: inertia,
    })
}

/// A valid leg as the line takes it: its goal speed, counts/s, unsigned;
/// its direction and held mean.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Point {
    pub goal_cps: f64,
    pub dir: i8,
    pub i_ma: f64,
}

/// One speed's share of the line.
#[derive(Clone, Debug, PartialEq)]
pub struct BySpeed {
    pub goal_cps: f64,
    pub legs: usize,
    pub i_ma: f64,
    pub residual_ma: f64,
    /// Mean up and mean down, where the speed has legs both ways.
    pub up_ma: Option<f64>,
    pub down_ma: Option<f64>,
}

/// The friction line `i = coulomb + viscous x speed`, least squares over
/// the legs against the goal, with the standard errors from the residuals
/// (legs less two degrees of freedom); the slope per 1000 counts/s and per
/// motor rad/s.
#[derive(Clone, Debug, PartialEq)]
pub struct Line {
    pub coulomb_ma: f64,
    pub coulomb_se_ma: f64,
    pub viscous_ma_per_kcps: f64,
    pub viscous_se_ma_per_kcps: f64,
    pub viscous_ma_s_per_rad: f64,
    pub viscous_se: f64,
    pub legs: usize,
    pub goal_cps: (f64, f64),
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

/// The line through `points` against the goal in 1000 counts/s, every leg
/// once; the slope per motor rad/s at `counts_per_motor_rad`. None under
/// two speeds or three legs.
pub fn line(points: &[Point], counts_per_motor_rad: f64) -> Option<Line> {
    let groups = group(points);
    if groups.len() < 2 || points.len() < 3 {
        return None;
    }
    let x: Vec<f64> = points.iter().map(|p| p.goal_cps.abs() / 1e3).collect();
    let y: Vec<f64> = points.iter().map(|p| p.i_ma).collect();
    let n = x.len() as f64;
    let xm = x.iter().sum::<f64>() / n;
    let b = slope(&x, &y);
    let a = y.iter().sum::<f64>() / n - b * xm;
    let resid: Vec<f64> = x.iter().zip(&y).map(|(xi, yi)| yi - a - b * xi).collect();
    let ss: f64 = resid.iter().map(|r| r * r).sum();
    let (a_se, b_se) = super::line_se(&x, &y, a, b)?;
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
                i_ma: mean(&v.iter().map(|p| p.i_ma).collect::<Vec<_>>()),
                residual_ma: mean(&res),
                up_ma: by(1),
                down_ma: by(-1),
            }
        })
        .collect();
    let per_rad = counts_per_motor_rad / 1e3;
    Some(Line {
        coulomb_ma: a,
        coulomb_se_ma: a_se,
        viscous_ma_per_kcps: b,
        viscous_se_ma_per_kcps: b_se,
        viscous_ma_s_per_rad: b * per_rad,
        viscous_se: b_se * per_rad,
        legs: points.len(),
        goal_cps: (groups[0].0, groups[groups.len() - 1].0),
        rms_ma: (ss / n).sqrt(),
        speeds,
    })
}

pub const SPEEDS: &str = "cruise speeds with a valid leg";
pub const LOWEST: &str = "the lowest valid cruise speed";
pub const RESIDUALS: &str = "the friction line's residuals by speed";
pub const DIRECTIONS: &str = "the cruise directions agree";

/// The line's checks over the valid legs' `points` and the `line` through
/// them, the ceiling the lowest speed is a share of: no line refuses, the
/// speeds, the residuals and the directions are reported.
pub fn checks(points: &[Point], line: Option<&Line>, ceiling_cps: f64) -> Vec<Check> {
    let groups = group(points);
    let speeds: Vec<String> = groups
        .iter()
        .map(|(g, v)| format!("{g:.0} ({})", v.len()))
        .collect();
    let mut out = vec![Check::reported(
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
    )];
    let low = groups.first().map(|(g, _)| *g);
    out.push(Check::reported(
        LOWEST,
        low.map_or("no valid leg".into(), |g| {
            format!(
                "{g:.0} counts/s, {:.2} of the ceiling {ceiling_cps:.0}",
                g / ceiling_cps
            )
        }),
    ));
    match line {
        Some(l) => {
            out.push(Check::reported(
                RESIDUALS,
                l.speeds
                    .iter()
                    .map(|s| format!("{:+.2} mA at {:.0}", s.residual_ma, s.goal_cps))
                    .collect::<Vec<_>>()
                    .join(", "),
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
            out.push(Check::reported(
                DIRECTIONS,
                match mean {
                    None => "no speed has valid legs both ways".into(),
                    Some(m) => format!(
                        "up less down {m:+.2} mA on the mean of {} speeds ({each})",
                        both.len()
                    ),
                },
            ));
        }
        None => out.push(Check {
            name: RESIDUALS.into(),
            value: "no line: under two speeds with a valid leg".into(),
            bound: "a line through two speeds at least".into(),
            verdict: Verdict::Fail,
        }),
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reading_of_record() -> Reading {
        Reading {
            counts_per_a: 1135.0,
            g_window_q016: 1178,
            i_kp_q88: 289,
            i_ki_q412: 1176,
            ke_vpc_q: 315,
            r_hat_q12: 3676,
            j_ff_q88: 862.0,
            fric_fc_counts: 43.0,
            fric_fv_q016: 947.0,
            pwm_arr: 1200,
            tick_hz: 20000.0,
            bridge_min_ticks: 27.0,
            settle: Settle {
                start_ticks: 40,
                q15: vec![37138, 35303, 34342, 33918],
            },
            counts_per_motor_rad: 3.7434,
            ceiling_cps: 4025.0,
        }
    }

    const POLL: Poll = Poll {
        vbus_raw: 1240.0,
        t_winding_cc: 2500.0,
        bias_counts: 1200.0,
    };

    /// A leg of `n` rows at `cps` from `p0` with acceleration `acc`, the
    /// duty `h` ticks, a crest sample `i_counts` over the bias of 1200,
    /// every row read; the identity table.
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
                    ..TelFrame::default()
                }
            })
            .collect()
    }

    fn leg(goal: f64, rows: &[TelFrame]) -> Leg<'_> {
        Leg {
            goal_cps: goal,
            rows,
            before: POLL,
            after: POLL,
            t_read_cc: 2500.0,
        }
    }

    fn identity() -> Lin {
        Lin::new(None, None)
    }

    /// The board's integers: the window term at the record's field and
    /// rail, and the reading rounded and signed by the drive.
    #[test]
    fn the_reading_is_the_boards_integers() {
        // g_arr = 1178 x 13981, k = g_arr x 1240 >> 24 = 1217,
        // K0 = 1217 x 1200 + 2^15
        assert_eq!(term_counts(1178, 0, 1240, 1200), 22);
        assert_eq!(term_counts(1178, 600, 1240, 1200), 11);
        assert_eq!(term_counts(1178, 1200, 1240, 1200), 0);
        let s = Settle {
            start_ticks: 40,
            q15: vec![37138],
        };
        // past the table the gain is one: 101 over the bias reads 101,
        // less the term
        assert_eq!(reading(1301, 1200, 600, &s, 1178, 1240, 1200), 101 - 11);
        assert_eq!(reading(1301, 1200, -600, &s, 1178, 1240, 1200), -(101 - 11));
        // inside it: floor(100 x 37138 / 2^14) = 226 half counts, 113
        assert_eq!(reading(1300, 1200, 44, &s, 0, 1240, 1200), 113);
    }

    /// Every row read: the PI inverted gives back the reading plus the
    /// error the next width asks for; a narrow pulse before a read row
    /// leaves it untaken, its reference the command over the resistance.
    #[test]
    fn the_replay_inverts_the_loop() {
        let r = reading_of_record();
        let rows = leg_rows(40, 2000.0, 300.0, 0.0, 300, 80.0);
        let rep = replay(&leg(300.0, &rows), &r).expect("replay");
        let kp = 289.0 / 256.0;
        let u = 300.0 / 1200.0 * 1240.0 - 315.0 / 4096.0 * 300.0;
        let i0 = reading(1280, 1200, 300, &r.settle, 1178, 1240, 1200) as f64;
        assert!(
            (rep.i_ref[0] - (i0 + u / kp)).abs() < 1e-9,
            "{}",
            rep.i_ref[0]
        );
        assert!(rep.i_ref[39].is_nan());
        assert!(rep.valid.iter().all(|v| *v));
        let mut rows = rows;
        rows[4].duty_counts = Some(10);
        let rep = replay(&leg(300.0, &rows), &r).expect("replay");
        let r_loop = 3676.0 / 4096.0;
        assert!((rep.i_ref[5] - u / r_loop).abs() < 1e-9, "{}", rep.i_ref[5]);
        let mut cold = leg(300.0, &rows);
        cold.after.t_winding_cc = UNSET_CC as f64;
        assert!(
            replay(&cold, &r)
                .expect_err("sentinel")
                .contains("sentinel")
        );
    }

    /// The phase is the one the reference is flat inside: a reference that
    /// steps every ten ticks from tick 3 on reads phase 3, flat there.
    #[test]
    fn the_medium_phase_is_where_the_reference_is_flat() {
        let tick: Vec<i64> = (100..400).collect();
        let i: Vec<f64> = tick.iter().map(|t| ((t - 3) / 10) as f64).collect();
        let (ph, sd_in, worst) = medium_phase(&i, &tick);
        assert_eq!(ph, 3);
        assert!(sd_in < 1e-12 && worst > 0.1, "{sd_in} {worst}");
        let m = period_means(&i, &tick, 3);
        assert!(m.iter().zip(&i).all(|(a, b)| (a - b).abs() < 1e-12));
    }

    /// The steady rows are the middle half; a leg still speeding up
    /// reports J a through j_ff, and at the top speed it is left out, named
    /// with its gain across its steady rows; below the top it counts.
    #[test]
    fn a_top_leg_still_gaining_speed_is_left_out() {
        let r = reading_of_record();
        // 1700 counts/s at the leg's start, the goal at its middle
        let acc = 2000.0;
        let rows = leg_rows(6000, 1300.0, 1700.0, acc, 300, 60.0);
        let got = read(&leg(2000.0, &rows), &identity(), &r).expect("read");
        assert_eq!((got.leg_rows, got.rows), (6000, 3000));
        let want = r.ma(862.0 / 256.0 * acc / 2000.0);
        assert!(
            (got.inertia_ma - want).abs() < 0.02 * want,
            "{} against {want}",
            got.inertia_ma
        );
        assert!(got.halves > 0.05, "{}", got.halves);
        assert!(got.fails(false).is_empty(), "{:?}", got.fails(false));
        let top = got.fails(true);
        assert!(
            top.len() == 1 && top[0].contains("top speed still changing"),
            "{top:?}"
        );
        let rows = leg_rows(6000, 1300.0, 2000.0, 0.0, 300, 60.0);
        let got = read(&leg(2000.0, &rows), &identity(), &r).expect("read");
        assert!(got.inertia_ma.abs() < 0.05, "{}", got.inertia_ma);
        assert!(got.fails(true).is_empty(), "{:?}", got.fails(true));
        assert_eq!(got.read_share, 1.0);
        let short = leg_rows(3000, 1300.0, 2000.0, 0.0, 300, 60.0);
        let e = read(&leg(2000.0, &short), &identity(), &r).expect_err("short");
        assert!(e.contains("under 100 ms"), "{e}");
    }

    /// Every row read: the held mean is the reading's mean.
    #[test]
    fn a_read_leg_holds_its_reading() {
        let r = reading_of_record();
        for (cps, p0) in [(2000.0, 1300.0), (-2000.0, 2500.0)] {
            let rows = leg_rows(8000, p0, cps, 0.0, 300, 80.0);
            let got = read(&leg(cps, &rows), &identity(), &r).expect("read");
            let want = r.ma(reading(1280, 1200, 300, &r.settle, 1178, 1240, 1200) as f64);
            assert!((got.held_ma - want).abs() < 1e-9, "{} {want}", got.held_ma);
            assert!((got.read_ma - want).abs() < 1e-9);
            assert_eq!(got.dir, cps.signum() as i8);
            assert!((got.speed_cps - 2000.0).abs() < 1.0, "{}", got.speed_cps);
            assert!(
                got.held_in_ms.is_some_and(|t| t < 50.0),
                "{:?}",
                got.held_in_ms
            );
        }
    }

    fn points(c: f64, v: f64, wiggle: &[(f64, i8, f64)]) -> Vec<Point> {
        wiggle
            .iter()
            .map(|&(goal, dir, e)| Point {
                goal_cps: goal,
                dir,
                i_ma: c + v * goal / 1e3 + e,
            })
            .collect()
    }

    /// The line through legs on a line is that line, against the goal,
    /// every leg once; its errors are the closed forms of the straight
    /// line on the residuals.
    #[test]
    fn the_line_is_least_squares_on_the_goal() {
        let exact = points(
            37.6,
            12.7,
            &[
                (300.0, 1, 0.0),
                (300.0, -1, 0.0),
                (1500.0, 1, 0.0),
                (4025.0, -1, 0.0),
            ],
        );
        let l = line(&exact, 3.7434).expect("line");
        assert!((l.coulomb_ma - 37.6).abs() < 1e-9 && (l.viscous_ma_per_kcps - 12.7).abs() < 1e-9);
        assert!((l.viscous_ma_s_per_rad - 12.7e-3 * 3.7434).abs() < 1e-12);
        assert!(l.coulomb_se_ma < 1e-6);
        let p = points(
            37.6,
            12.7,
            &[
                (300.0, 1, 1.0),
                (300.0, -1, -1.0),
                (800.0, 1, 0.5),
                (1500.0, -1, -0.7),
                (1500.0, 1, 0.2),
                (4025.0, 1, 1.3),
            ],
        );
        let l = line(&p, 3.7434).expect("line");
        let x: Vec<f64> = p.iter().map(|q| q.goal_cps / 1e3).collect();
        let y: Vec<f64> = p.iter().map(|q| q.i_ma).collect();
        let n = x.len() as f64;
        let xm = x.iter().sum::<f64>() / n;
        let ym = y.iter().sum::<f64>() / n;
        let sxx: f64 = x.iter().map(|v| (v - xm).powi(2)).sum();
        let b = x
            .iter()
            .zip(&y)
            .map(|(a, c)| (a - xm) * (c - ym))
            .sum::<f64>()
            / sxx;
        let a = ym - b * xm;
        let s2 = x
            .iter()
            .zip(&y)
            .map(|(xi, yi)| (yi - a - b * xi).powi(2))
            .sum::<f64>()
            / (n - 2.0);
        assert!((l.viscous_ma_per_kcps - b).abs() < 1e-12 && (l.coulomb_ma - a).abs() < 1e-9);
        assert!((l.viscous_se_ma_per_kcps - (s2 / sxx).sqrt()).abs() < 1e-12);
        assert!((l.coulomb_se_ma - (s2 * (1.0 / n + xm * xm / sxx)).sqrt()).abs() < 1e-12);
        assert_eq!(l.speeds.len(), 4);
        assert_eq!(l.goal_cps, (300.0, 4025.0));
        assert!(
            line(
                &points(37.6, 12.7, &[(300.0, 1, 0.0), (300.0, -1, 0.0)]),
                3.7
            )
            .is_none()
        );
    }

    /// The line's checks: the speeds, the lowest's share of the ceiling,
    /// the residuals and the directions are reported, whatever they read;
    /// no line (under two speeds) refuses.
    #[test]
    fn the_line_checks_report_and_only_no_line_refuses() {
        let reported = |c: &[Check]| {
            c.iter()
                .all(|c| c.verdict == Verdict::NotJudged && c.bound == crate::steps::REPORTED)
        };
        let ok = points(
            37.6,
            12.7,
            &[
                (322.0, 1, 0.5),
                (322.0, -1, -0.5),
                (805.0, 1, 0.0),
                (1530.0, -1, 0.0),
                (4025.0, 1, 0.2),
                (4025.0, -1, -0.2),
            ],
        );
        let c = checks(&ok, line(&ok, 3.7434).as_ref(), 4025.0);
        assert!(reported(&c), "{c:?}");
        // two speeds, the lowest at 0.53 of the ceiling: a line, reported
        let two = points(
            37.6,
            12.7,
            &[(2150.0, 1, 0.0), (2150.0, -1, 0.0), (4025.0, 1, 0.0)],
        );
        let c = checks(&two, line(&two, 3.7434).as_ref(), 4025.0);
        assert!(reported(&c), "{c:?}");
        let low = c.iter().find(|c| c.name == LOWEST).expect("lowest");
        assert!(low.value.contains("0.53 of the ceiling"), "{low:?}");
        // up 4 mA over down at every speed, as a current zero off by 2 mA
        // reads: the mean holds it all, reported
        let offset = points(
            37.6,
            12.7,
            &[
                (322.0, 1, 2.0),
                (322.0, -1, -2.0),
                (1530.0, 1, 2.0),
                (1530.0, -1, -2.0),
                (4025.0, 1, 2.0),
                (4025.0, -1, -2.0),
            ],
        );
        let c = checks(&offset, line(&offset, 3.7434).as_ref(), 4025.0);
        assert!(reported(&c), "{c:?}");
        let d = c.iter().find(|c| c.name == DIRECTIONS).expect("directions");
        assert!(
            d.value.contains("+4.00 mA on the mean of 3 speeds")
                && d.value.contains("up less down +4.00 mA at 322"),
            "{d:?}"
        );
        let c = checks(&[], None, 4025.0);
        let failed: Vec<&str> = c
            .iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .map(|c| c.name.as_str())
            .collect();
        assert_eq!(failed, [RESIDUALS], "{c:?}");
    }
}
