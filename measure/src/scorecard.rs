//! The closed-loop scores of a move, pure: the notebooks' stage 5
//! scorecard (`oscnb/scorecard.py`) on the rows of one goal and its hold,
//! and the acceptance of a set on them (the closed-loop steps' self-test).
//!
//! Every score reads the pot as the scorecard does: the raw code through
//! the servo's ADC wide-code slots and position table into linearized
//! counts ([`Lin`]), then a centered rolling median of [`MEDIAN_ROWS`]
//! rows over the move. A row's current is its raw shunt reading over the
//! capture's own zero ([`current_zero`]) where the drive window was read.
//! Degrees are counts over the run's counts per degree.

use osc_servo_core::adc::{ADC_MASK, ADC_WIDE_SLOTS, wide_q4};
use osc_servo_core::kernel::phases::loops::trajectory::TrajGen;
use osc_servo_core::pos_lut::{GRID, INTERVALS};
use osc_servo_core::regions::config::{ConfigLoopPosition, ConfigPosSoft};

use crate::fit::median;
use crate::steps::{Check, Verdict};

/// The rolling median's width, rows: about 2 ms, wider than a pot spike.
pub const MEDIAN_ROWS: usize = 41;
/// The band a move settles into, deg.
pub const SETTLE_BAND_DEG: f64 = 0.5;
/// The tail of a hold the rest error and the quiet test read, s.
pub const REST_S: f64 = 0.5;
/// A tail driven over this share of its rows hunts.
pub const TAIL_DRIVEN_SHARE: f64 = 0.02;

/// Acceptance of a set (the closed-loop steps' self-test): overshoot at
/// most the larger of [`OVERSHOOT_DEG`] and [`OVERSHOOT_SHARE`] of the
/// step, following error under [`FOLLOW_SHARE`] of the position-error
/// screen, rest error at most [`REST_DEG`].
pub const OVERSHOOT_DEG: f64 = 1.0;
pub const OVERSHOOT_SHARE: f64 = 0.02;
pub const FOLLOW_SHARE: f64 = 0.5;
pub const REST_DEG: f64 = 0.5;
/// The first enable moves the shaft at most this share of the travel; a
/// small step travels at most [`SMALL_TRAVEL`] times itself.
pub const ENABLE_SHARE: f64 = 0.01;
pub const SMALL_TRAVEL: f64 = 3.0;

/// The servo's reading of a raw pot code: its wide-code step, then its
/// position table, in linearized counts.
#[derive(Clone, Debug, PartialEq)]
pub struct Lin {
    /// The table's points with the fixed last one; all zero is the
    /// identity.
    pub points: [i16; INTERVALS + 1],
    pub slots: [u8; ADC_WIDE_SLOTS],
}

impl Lin {
    /// `points` are the table the servo applies, None for the identity;
    /// `slots` its wide-code slots, None for none.
    pub fn new(points: Option<&[i16; INTERVALS]>, slots: Option<&[u8]>) -> Self {
        let mut p = [0i16; INTERVALS + 1];
        if let Some(k) = points {
            p[..INTERVALS].copy_from_slice(k);
        }
        let mut s = [0u8; ADC_WIDE_SLOTS];
        if let Some(b) = slots {
            for (d, v) in s.iter_mut().zip(b) {
                *d = *v;
            }
        }
        Lin {
            points: p,
            slots: s,
        }
    }

    /// Linearized counts at `raw`, float: the code read as the middle of
    /// the span it stands for, then the table's line between its points,
    /// as `postable.Table.lin` reads it. The servo floors the same line to
    /// a sixteenth of a count.
    pub fn counts(&self, raw: u16) -> f64 {
        let code = raw & ADC_MASK;
        let r =
            (code as f64 + wide_q4(code, &self.slots) as f64 / GRID as f64).min(ADC_MASK as f64);
        let g = GRID as f64;
        let i = ((r / g).floor() as usize).min(INTERVALS - 1);
        let (c0, c1) = (self.points[i] as f64, self.points[i + 1] as f64);
        r + c0 + (c1 - c0) * (r - g * i as f64) / g
    }
}

/// One row of a move as the scores read it.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Row {
    /// s from the move's first row, on the servo's tick.
    pub t: f64,
    /// Linearized counts ([`Lin::counts`]).
    pub pos: f64,
    pub duty: i16,
    pub current_raw: Option<u16>,
    pub window_valid: bool,
}

/// The centered rolling median of `w` rows, an edge taking the rows it
/// has (pandas' `rolling(w, center=True, min_periods=1)`).
pub fn rolling_median(x: &[f64], w: usize) -> Vec<f64> {
    let half = w / 2;
    let mut win: Vec<f64> = Vec::with_capacity(w + 1);
    let mut out = Vec::with_capacity(x.len());
    let ins = |win: &mut Vec<f64>, v: f64| {
        let at = win.partition_point(|&a| a < v);
        win.insert(at, v);
    };
    for &v in x.iter().take(half) {
        ins(&mut win, v);
    }
    for i in 0..x.len() {
        if let Some(&v) = x.get(i + half) {
            ins(&mut win, v);
        }
        if i > half {
            let old = x[i - half - 1];
            let at = win.partition_point(|&a| a < old);
            win.remove(at);
        }
        let n = win.len();
        out.push(if n % 2 == 1 {
            win[n / 2]
        } else {
            (win[n / 2 - 1] + win[n / 2]) / 2.0
        });
    }
    out
}

/// The shunt's zero for a capture: the median raw current over its rows
/// at zero duty, where the bridge drives nothing; None with no such row.
pub fn current_zero<'a>(rows: impl IntoIterator<Item = &'a Row>) -> Option<f64> {
    let idle: Vec<f64> = rows
        .into_iter()
        .filter(|r| r.duty == 0)
        .filter_map(|r| r.current_raw.map(f64::from))
        .collect();
    median(&idle)
}

/// What a move's scores are read at.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Units {
    pub counts_per_deg: f64,
    pub counts_per_a: f64,
    pub tick_hz: f64,
    /// The capture's current zero, counts.
    pub zero_counts: Option<f64>,
}

/// One move: the goal it commits, where the goal before it was, and its
/// rows from the commit to the end of its hold.
#[derive(Clone, Debug, PartialEq)]
pub struct Move {
    pub name: String,
    pub from: f64,
    pub goal: f64,
    pub rows: Vec<Row>,
}

impl Move {
    /// +1 when the goal is above the goal before, else -1.
    pub fn direction(&self) -> f64 {
        if self.goal >= self.from { 1.0 } else { -1.0 }
    }

    pub fn step_deg(&self, u: &Units) -> f64 {
        (self.goal - self.from).abs() / u.counts_per_deg
    }
}

/// A move's scores. A score with nothing to read by is NaN.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Scores {
    pub step_deg: f64,
    /// deg past the goal along the travel, 0 if none.
    pub overshoot_deg: f64,
    /// s from the commit to the end of the last row out of the settle band.
    pub settle_s: f64,
    /// The share of the hold's last [`REST_S`] driven.
    pub tail_driven_share: f64,
    pub quiet: bool,
    /// |goal - the mean median pot over the last [`REST_S`]|, deg.
    pub rest_error_deg: f64,
    /// The largest |current| a read window carried, A, and counts.
    pub current_peak_a: f64,
    pub current_peak_counts: f64,
    /// The largest |profile - median pot|, counts.
    pub follow_peak_counts: f64,
}

/// The rows of the last `s` seconds.
fn tail(rows: &[Row], s: f64) -> &[Row] {
    let Some(last) = rows.last() else {
        return rows;
    };
    let from = rows.partition_point(|r| r.t < last.t - s);
    &rows[from..]
}

/// The servo's own position profile for a move, sampled at `rows`: the
/// trajectory generator stepped every medium tick from rest at `from`
/// toward `goal`, its first step at the move's first row. The
/// position-error screen judges the shaft against this reference.
pub struct Profile {
    pub lp: ConfigLoopPosition,
    pub soft: ConfigPosSoft,
    pub medium_hz: f64,
}

impl Profile {
    fn reseeded(&self, from: f64) -> TrajGen {
        let mut g = TrajGen::new();
        g.reseed(TrajGen::goal_q16(from.round() as i32, &self.soft));
        g
    }

    fn dt_q32(&self) -> u32 {
        ((1u64 << 32) as f64 / self.medium_hz).round() as u32
    }

    /// The reference at each of `t`, s from the first step, counts.
    pub fn at(&self, from: f64, goal: f64, t: impl IntoIterator<Item = f64>) -> Vec<f64> {
        let mut g = self.reseeded(from);
        let dt = self.dt_q32();
        let mut steps = 0u64;
        let goal = goal.round() as i32;
        t.into_iter()
            .map(|t| {
                let want = (t * self.medium_hz).floor().max(0.0) as u64 + 1;
                while steps < want {
                    g.step_position(goal, dt, &self.lp, &self.soft);
                    steps += 1;
                }
                g.theta_star_q16() as f64 / 65536.0
            })
            .collect()
    }

    /// s until the reference lands at rest on the goal; None past `max_s`.
    pub fn time_s(&self, from: f64, goal: f64, max_s: f64) -> Option<f64> {
        let mut g = self.reseeded(from);
        let dt = self.dt_q32();
        let target = TrajGen::goal_q16(goal.round() as i32, &self.soft);
        let max = (max_s * self.medium_hz).ceil() as u64;
        for k in 1..=max {
            g.step_position(goal.round() as i32, dt, &self.lp, &self.soft);
            if g.theta_star_q16() == target && g.omega_star_q16() == 0 {
                return Some(k as f64 / self.medium_hz);
            }
        }
        None
    }
}

/// A move's scores at `u`; the following error against `profile`, NaN
/// without one.
pub fn score(m: &Move, u: &Units, profile: Option<&Profile>) -> Scores {
    let cpd = u.counts_per_deg;
    let pos: Vec<f64> = m.rows.iter().map(|r| r.pos).collect();
    let f = rolling_median(&pos, MEDIAN_ROWS);
    let dir = m.direction();
    let overshoot_deg = f
        .iter()
        .map(|p| dir * (p - m.goal) / cpd)
        .fold(0.0, f64::max);
    let t0 = m.rows.first().map_or(0.0, |r| r.t);
    let settle_s = f
        .iter()
        .zip(&m.rows)
        .rfind(|(p, _)| (*p - m.goal).abs() / cpd > SETTLE_BAND_DEG)
        .map_or(0.0, |(_, r)| r.t - t0 + 1.0 / u.tick_hz);
    let tl = tail(&m.rows, REST_S);
    let tf = &f[f.len() - tl.len()..];
    let tail_driven_share = if tl.is_empty() {
        f64::NAN
    } else {
        tl.iter().filter(|r| r.duty != 0).count() as f64 / tl.len() as f64
    };
    let rest_error_deg = if tf.is_empty() {
        f64::NAN
    } else {
        (m.goal - tf.iter().sum::<f64>() / tf.len() as f64).abs() / cpd
    };
    let current_peak_counts = match u.zero_counts {
        Some(z) => m
            .rows
            .iter()
            .filter(|r| r.window_valid)
            .filter_map(|r| r.current_raw)
            .map(|c| (c as f64 - z).abs())
            .fold(f64::NAN, f64::max),
        None => f64::NAN,
    };
    let follow_peak_counts = match profile {
        Some(p) => p
            .at(m.from, m.goal, m.rows.iter().map(|r| r.t - t0))
            .iter()
            .zip(&f)
            .map(|(r, p)| (r - p).abs())
            .fold(0.0, f64::max),
        None => f64::NAN,
    };
    Scores {
        step_deg: m.step_deg(u),
        overshoot_deg,
        settle_s,
        tail_driven_share,
        quiet: tail_driven_share <= TAIL_DRIVEN_SHARE,
        rest_error_deg,
        current_peak_a: current_peak_counts / u.counts_per_a,
        current_peak_counts,
        follow_peak_counts,
    }
}

/// The overshoot a move of `step_deg` may make, deg.
pub fn overshoot_bound_deg(step_deg: f64) -> f64 {
    OVERSHOOT_DEG.max(OVERSHOOT_SHARE * step_deg)
}

/// `v` at most `bound`; a NaN, nothing read, is not.
fn within(v: f64, bound: f64) -> bool {
    v <= bound
}

fn under(v: f64, bound: f64) -> bool {
    v < bound
}

fn pass(ok: bool) -> Verdict {
    if ok { Verdict::Pass } else { Verdict::Fail }
}

fn judged(moves: usize, ok: bool) -> Verdict {
    if moves == 0 {
        Verdict::NotJudged
    } else {
        pass(ok)
    }
}

/// The check names, the result's `failed_on` words.
pub const FIRST_ENABLE: &str = "the first enable holds still";
pub const SMALL_STEPS: &str = "the small steps close on their goals";
pub const NO_FAULT: &str = "no fault";
pub const OVERSHOOT: &str = "overshoot";
pub const QUIET: &str = "every hold quiet in its last 500 ms";
pub const FOLLOW: &str = "following error";
pub const REST: &str = "rest error";

/// The first enable: the median pot over its capture against the goal
/// torque came on at; `fault` a fault it latched.
pub fn enable_check(rows: &[Row], goal: f64, travel: f64, fault: Option<&str>) -> Check {
    let pos: Vec<f64> = rows.iter().map(|r| r.pos).collect();
    let moved = rolling_median(&pos, MEDIAN_ROWS)
        .iter()
        .map(|p| (p - goal).abs())
        .fold(0.0, f64::max);
    let bound = ENABLE_SHARE * travel;
    let mut value = format!(
        "the shaft read at most {moved:.1} counts ({:.2}% of the travel) off the goal it came on at, over {} rows",
        100.0 * moved / travel,
        rows.len()
    );
    if let Some(f) = fault {
        value.push_str(&format!("; {f}"));
    }
    Check {
        name: FIRST_ENABLE.into(),
        value,
        bound: format!(
            "at most {:.0}% of the travel ({bound:.1} counts), no fault",
            100.0 * ENABLE_SHARE
        ),
        verdict: pass(moved <= bound && fault.is_none() && !rows.is_empty()),
    }
}

/// One small step as its burst read it.
#[derive(Clone, Debug, PartialEq)]
pub struct Small {
    pub name: String,
    pub goal: f64,
    pub step_counts: f64,
    /// |goal - median pot| at the burst's first row and at its last.
    pub error_start: f64,
    pub error_end: f64,
    /// The largest |median pot - its first row's|.
    pub travel: f64,
    pub fault: Option<String>,
}

impl Small {
    pub fn of(
        name: &str,
        goal: f64,
        step_counts: f64,
        rows: &[Row],
        fault: Option<String>,
    ) -> Self {
        let pos: Vec<f64> = rows.iter().map(|r| r.pos).collect();
        let f = rolling_median(&pos, MEDIAN_ROWS);
        let (first, last) = (
            f.first().copied().unwrap_or(f64::NAN),
            f.last().copied().unwrap_or(f64::NAN),
        );
        Small {
            name: name.into(),
            goal,
            step_counts,
            error_start: (goal - first).abs(),
            error_end: (goal - last).abs(),
            travel: f.iter().map(|p| (p - first).abs()).fold(0.0, f64::max),
            fault,
        }
    }

    /// The error grew, the shaft ran past three steps, or a fault latched.
    pub fn failed(&self) -> bool {
        !within(self.error_end, self.error_start)
            || !within(self.travel, SMALL_TRAVEL * self.step_counts)
            || self.fault.is_some()
    }

    pub fn said(&self) -> String {
        format!(
            "{}: error {:.1} -> {:.1} counts, travel {:.1} counts{}",
            self.name,
            self.error_start,
            self.error_end,
            self.travel,
            self.fault
                .as_ref()
                .map_or(String::new(), |f| format!(", {f}"))
        )
    }
}

pub fn small_check(steps: &[Small]) -> Check {
    let step = steps.first().map_or(0.0, |s| s.step_counts);
    Check {
        name: SMALL_STEPS.into(),
        value: steps.iter().map(Small::said).collect::<Vec<_>>().join("; "),
        bound: format!(
            "each burst ends no farther from its goal than it starts, travels at most {SMALL_TRAVEL} x the {step:.0} count step, no fault"
        ),
        verdict: pass(!steps.is_empty() && steps.iter().all(|s| !s.failed())),
    }
}

/// The acceptance of a set over its self-test moves: `fault` a fault the
/// moves latched, `screen_counts` the position-error screen. With no move
/// scored, the moves' checks have nothing to judge.
pub fn accept(
    moves: &[(String, Scores)],
    u: &Units,
    screen_counts: f64,
    fault: Option<&str>,
) -> Vec<Check> {
    let cpd = u.counts_per_deg;
    let named = |bad: Vec<String>| {
        if bad.is_empty() {
            String::new()
        } else {
            format!("; over on {}", bad.join(", "))
        }
    };
    let worst = |f: &dyn Fn(&Scores) -> f64| {
        moves.iter().map(|(n, s)| (n.as_str(), f(s))).fold(
            None,
            |w: Option<(&str, f64)>, (n, v)| match w {
                Some((_, b)) if v <= b || b.is_nan() => w,
                _ => Some((n, v)),
            },
        )
    };
    let mut out = vec![Check {
        name: NO_FAULT.into(),
        value: fault.map_or("no fault latched".into(), str::to_string),
        bound: "fault_flags 0 through every move".into(),
        verdict: pass(fault.is_none()),
    }];
    let over: Vec<String> = moves
        .iter()
        .filter(|(_, s)| !within(s.overshoot_deg, overshoot_bound_deg(s.step_deg)))
        .map(|(n, s)| {
            format!(
                "{n} ({:.2} deg of {:.2})",
                s.overshoot_deg,
                overshoot_bound_deg(s.step_deg)
            )
        })
        .collect();
    let w = worst(&|s| s.overshoot_deg / overshoot_bound_deg(s.step_deg));
    out.push(Check {
        name: OVERSHOOT.into(),
        value: format!(
            "worst {}{}",
            w.map_or("none".into(), |(n, r)| format!(
                "{n}, {:.0}% of its bound",
                100.0 * r
            )),
            named(over.clone())
        ),
        bound: format!(
            "at most the larger of {OVERSHOOT_DEG} deg and {:.0}% of the step",
            100.0 * OVERSHOOT_SHARE
        ),
        verdict: judged(moves.len(), over.is_empty()),
    });
    let hunting: Vec<String> = moves
        .iter()
        .filter(|(_, s)| !s.quiet)
        .map(|(n, s)| format!("{n} ({:.1}% driven)", 100.0 * s.tail_driven_share))
        .collect();
    let w = worst(&|s| s.tail_driven_share);
    out.push(Check {
        name: QUIET.into(),
        value: format!(
            "most driven {}{}",
            w.map_or("none".into(), |(n, v)| format!(
                "{n}, {:.1}% of its rows",
                100.0 * v
            )),
            named(hunting.clone())
        ),
        bound: format!(
            "at most {:.0}% of the rows driven",
            100.0 * TAIL_DRIVEN_SHARE
        ),
        verdict: judged(moves.len(), hunting.is_empty()),
    });
    let half = FOLLOW_SHARE * screen_counts;
    let lag: Vec<String> = moves
        .iter()
        .filter(|(_, s)| !under(s.follow_peak_counts, half))
        .map(|(n, s)| format!("{n} ({:.0} counts)", s.follow_peak_counts))
        .collect();
    let w = worst(&|s| s.follow_peak_counts);
    out.push(Check {
        name: FOLLOW.into(),
        value: format!(
            "largest {}{}",
            w.map_or("none".into(), |(n, v)| format!(
                "{n}, {v:.1} counts ({:.2} deg)",
                v / cpd
            )),
            named(lag.clone())
        ),
        bound: format!(
            "under half the {screen_counts:.0} count position-error screen ({half:.1} counts, {:.2} deg), against the servo's own profile",
            half / cpd
        ),
        verdict: judged(moves.len(), screen_counts > 0.0 && lag.is_empty()),
    });
    let off: Vec<String> = moves
        .iter()
        .filter(|(_, s)| !within(s.rest_error_deg, REST_DEG))
        .map(|(n, s)| format!("{n} ({:.2} deg)", s.rest_error_deg))
        .collect();
    let w = worst(&|s| s.rest_error_deg);
    out.push(Check {
        name: REST.into(),
        value: format!(
            "largest {}{}",
            w.map_or("none".into(), |(n, v)| format!("{n}, {v:.2} deg")),
            named(off.clone())
        ),
        bound: format!("at most {REST_DEG} deg over the last 500 ms of the hold"),
        verdict: judged(moves.len(), off.is_empty()),
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: f64 = 20_000.0;
    const CPD: f64 = 20.0;
    const ZERO: f64 = 1210.0;
    /// The set of record's screen.
    const SCREEN: f64 = 199.0;

    fn units() -> Units {
        Units {
            counts_per_deg: CPD,
            counts_per_a: 1135.0,
            tick_hz: HZ,
            zero_counts: Some(ZERO),
        }
    }

    /// The set of record's profile on its soft range.
    fn profile() -> Profile {
        Profile {
            lp: ConfigLoopPosition {
                p_kp_q88: 0,
                pos_deadband_counts: 4,
                speed_ceiling_cps: 8050,
                velocity_limit_cps: 4025,
                accel_limit_q88: 10305,
                _rsvd_align: 0,
            },
            soft: ConfigPosSoft {
                pos_min_soft_counts: 314,
                pos_max_soft_counts: 3560,
            },
            medium_hz: HZ / 10.0,
        }
    }

    /// A move of 10 % of the travel that follows the servo's profile to the
    /// count, driven while it travels and quiet once it lands, its hold
    /// 1.5 s; `shape` moves each row's position from there, given its time
    /// past the landing.
    fn a_move(shape: impl Fn(f64, f64) -> f64) -> Move {
        let (from, goal) = (1937.0, 1937.0 + 325.0);
        let p = profile();
        let land = p.time_s(from, goal, 5.0).expect("lands");
        let n = ((land + 1.5) * HZ) as usize;
        let t: Vec<f64> = (0..n).map(|k| k as f64 / HZ).collect();
        let reference = p.at(from, goal, t.iter().copied());
        let rows = t
            .iter()
            .zip(&reference)
            .map(|(&t, &r)| Row {
                t,
                pos: shape(r, t - land),
                duty: if t < land { 300 } else { 0 },
                current_raw: Some(if t < land {
                    ZERO as u16 + 120
                } else {
                    ZERO as u16
                }),
                window_valid: t < land,
            })
            .collect();
        Move {
            name: "10% + out".into(),
            from,
            goal,
            rows,
        }
    }

    fn failed(m: &Move) -> Vec<String> {
        let s = score(m, &units(), Some(&profile()));
        accept(&[(m.name.clone(), s)], &units(), SCREEN, None)
            .into_iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .map(|c| c.name)
            .collect()
    }

    #[test]
    fn the_rolling_median_takes_what_an_edge_has() {
        let x = [5.0, 1.0, 4.0, 2.0, 3.0, 9.0];
        assert_eq!(rolling_median(&x, 3), [3.0, 4.0, 2.0, 3.0, 3.0, 6.0]);
        assert_eq!(rolling_median(&x, 41), [3.5; 6]);
    }

    #[test]
    fn a_move_on_its_profile_passes_every_check() {
        let m = a_move(|r, _| r);
        let s = score(&m, &units(), Some(&profile()));
        assert!(failed(&m).is_empty(), "{s:?}");
        assert!(s.follow_peak_counts < 1.0, "{s:?}");
        assert_eq!((s.overshoot_deg, s.tail_driven_share), (0.0, 0.0));
        assert!(s.quiet && s.rest_error_deg < 1e-9, "{s:?}");
        assert!((s.current_peak_counts - 120.0).abs() < 1e-9, "{s:?}");
        assert!(s.settle_s > 0.0 && s.settle_s < 0.2, "{s:?}");
    }

    /// 2 deg past the goal once landed, back well inside the hold.
    #[test]
    fn an_overshoot_over_the_bound_fails_its_check_alone() {
        let m = a_move(|r, past| {
            r + if (0.0..0.2).contains(&past) {
                40.0 * (std::f64::consts::PI * past / 0.2).sin()
            } else {
                0.0
            }
        });
        assert_eq!(failed(&m), [OVERSHOOT]);
    }

    /// Driven on every fifth row of its last 500 ms: the loop hunts.
    #[test]
    fn a_hold_that_hunts_fails_its_check_alone() {
        let mut m = a_move(|r, _| r);
        let end = m.rows.last().expect("rows").t;
        for (k, r) in m.rows.iter_mut().enumerate() {
            if r.t > end - 0.4 && k % 5 == 0 {
                r.duty = 40;
            }
        }
        assert_eq!(failed(&m), [QUIET]);
    }

    /// 0.6 deg short of the goal through the hold.
    #[test]
    fn a_rest_error_over_the_bound_fails_its_check_alone() {
        let m = a_move(|r, past| {
            if past >= 0.0 {
                r - 12.0
            } else {
                r.min(1937.0 + 313.0)
            }
        });
        assert_eq!(failed(&m), [REST]);
    }

    /// 120 counts behind the profile as it lands, over half the screen,
    /// then on the goal 100 ms later.
    #[test]
    fn a_following_error_over_half_the_screen_fails_its_check_alone() {
        let m = a_move(|r, past| {
            let lag = if past < 0.0 {
                120.0f64.min(r - 1937.0)
            } else {
                120.0 * (1.0 - past / 0.1).max(0.0)
            };
            r - lag
        });
        assert_eq!(failed(&m), [FOLLOW]);
    }

    fn still(drift: f64) -> Vec<Row> {
        (0..4000)
            .map(|k| Row {
                t: k as f64 / HZ,
                pos: 1937.0 + drift * k as f64 / 4000.0,
                ..Row::default()
            })
            .collect()
    }

    #[test]
    fn a_first_enable_that_moves_fails_its_check() {
        assert_eq!(
            enable_check(&still(0.5), 1937.0, 3246.0, None).verdict,
            Verdict::Pass
        );
        let c = enable_check(&still(40.0), 1937.0, 3246.0, None);
        assert_eq!((c.name.as_str(), c.verdict), (FIRST_ENABLE, Verdict::Fail));
        let c = enable_check(&still(0.0), 1937.0, 3246.0, Some("fault_flags 0x01"));
        assert_eq!(c.verdict, Verdict::Fail);
    }

    /// A step of 32 counts whose shaft runs the other way: the error
    /// grows inside its burst.
    #[test]
    fn a_small_step_whose_error_grows_fails_its_check() {
        let rows = |to: f64| -> Vec<Row> {
            (0..2000)
                .map(|k| Row {
                    t: k as f64 / HZ,
                    pos: 1937.0 + to * (k as f64 / 400.0).min(1.0),
                    ..Row::default()
                })
                .collect()
        };
        let right = Small::of("+ out", 1969.0, 32.0, &rows(31.0), None);
        let wrong = Small::of("+ out", 1969.0, 32.0, &rows(-32.0), None);
        let runaway = Small::of("+ out", 1969.0, 32.0, &rows(140.0), None);
        assert!(!right.failed(), "{}", right.said());
        assert!(wrong.failed() && wrong.error_end > wrong.error_start);
        assert!(runaway.failed() && runaway.travel > 3.0 * 32.0);
        assert_eq!(
            small_check(std::slice::from_ref(&right)).verdict,
            Verdict::Pass
        );
        let c = small_check(&[right, wrong]);
        assert_eq!((c.name.as_str(), c.verdict), (SMALL_STEPS, Verdict::Fail));
    }

    #[test]
    fn a_fault_fails_no_fault_alone() {
        let m = a_move(|r, _| r);
        let s = score(&m, &units(), Some(&profile()));
        let bad: Vec<String> = accept(&[(m.name, s)], &units(), SCREEN, Some("fault_flags 0x10"))
            .into_iter()
            .filter(|c| c.verdict == Verdict::Fail)
            .map(|c| c.name)
            .collect();
        assert_eq!(bad, [NO_FAULT]);
    }

    /// The identity table and a wide code read as the middle of its band.
    #[test]
    fn a_wide_code_reads_as_the_middle_of_its_band() {
        let mut slots = [0u8; ADC_WIDE_SLOTS];
        slots[3] = 80;
        let lin = Lin::new(None, Some(&slots));
        assert_eq!(lin.counts(1000), 1000.0);
        assert_eq!(lin.counts(0x3FF), 1023.0 + 5.0);
        assert_eq!(lin.counts(0x3FE), 1022.0);
    }
}
