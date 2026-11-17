//! The bench scorecard's rows over the sim. The servo of record's rig
//! (`mg90_d`) runs the stage 5 scorecard's test set, its steps, follow and
//! hold captures, and each row is computed from the kernel's own per-tick
//! telemetry as the notebooks' scorecard computes it from a capture's
//! rows: the pot, the shunt's raw reading and the applied duty, one row a
//! FAST tick at 20 kHz as the bench's TEL rows are. Three pot-noise seeds
//! stand for the bench's three runs; a row is their median, pinned
//! one-sided in its worse direction at 1.5x the sim's own worst seed when
//! the pin was recorded, or at that seed plus a stated floor where the
//! floor reaches further. The worst seed, not the median: a row of a few
//! events a move spreads wide over the seeds, and the median of three may
//! be any of them after a change that moves nothing else.
//!
//! Where the sim's rows differ from a bench capture's: the position table
//! is the identity and the board's wide codes are absent, so `pos` is the
//! raw pot with no table or wide-code step; each capture starts the plant
//! at rest at the center with torque coming on at its first row; a goal
//! commits on the tick after the item before it ends, the steps and holds
//! leaving no hole between items, while a ramp's goal item keeps the
//! bench's 25.6 ms period with rows over its first 20 ms only; a row's
//! shunt window is read when its duty is past the board's shunt floor,
//! as the host decides it.
//!
//! `cargo test -p osc-integration --test scorecard -- --nocapture` prints
//! the table.

use osc_integration::mg90::{self, Mg90};
use osc_integration::mg90_d::{self, COUNTS_PER_DEG, CURRENT_COUNTS_PER_A};
use osc_integration::plant::FakeIo;
use osc_integration::sim::DEV_V006_SENSE;
use osc_servo_core::{Kernel, RegionStorage};

const TICKS_PER_MS: u32 = 20;
const TICK_HZ: f64 = 20_000.0;
const PWM_PERIOD_TICKS: f64 = 1200.0;

/// The test set's limits of motion, on the set's count.
const VELOCITY_LIMIT_CPS: u16 = 1500;
const ACCEL_LIMIT_Q88: u16 = 3840;
/// The servo's no-load speed at the horn, deg/s: its stage 3 run-up
/// result, 533.72 rev/s at the motor, through the 308.0533 gear ratio.
const NO_LOAD_DEG_S: f64 = 533.7211404557644 / 308.0533 * 360.0;

const STEP_SIZES_DEG: [u32; 3] = [2, 20, 60];
const STEP_HOLD_MS: [u32; 3] = [1200, 1500, 2000];
const ENABLE_HOLD_MS: u32 = 1000;
const FOLLOW_SPEED_PCT: [u32; 3] = [1, 3, 10];
const FOLLOW_SPAN_DEG: f64 = 30.0;
const FOLLOW_SPAN_S: f64 = 3.0;
const GOAL_ITEM_MS: u32 = 20;
const GOAL_PERIOD_TICKS: u32 = 512;
const PAUSE_MS: u32 = 500;
const HOLD_MS: u32 = 3000;
const HOLDS: usize = 2;

const MEDIAN_ROWS: usize = 41;
const SETTLE_BAND_DEG: f64 = 0.5;
const HOLD_HEAD_MS: f64 = 500.0;
const FOLLOW_TAIL: f64 = 2.0 / 3.0;

const SEEDS: [u64; 3] = [1, 2, 3];

/// Per row: the sim's worst seed when the pin was recorded and the floor
/// added to it where that reaches past 1.5x of it. The overshoot's floor
/// is two counts of the pot, the hold's driven share the share a single
/// noise wake leaves, its current the rms of that share driven at 100 mA.
/// The rows a few events make, the reversals and the worst landing, take
/// a floor of 10 reversals a second and 50 ms: one event more or less
/// moves them past 1.5x.
const PINS: &[(&str, f64, f64)] = &[
    ("overshoot_deg_2deg", 0.0, 0.1),
    ("settle_ms_2deg", 57.35, 0.0),
    ("move_current_rms_ma_2deg", 102.58, 0.0),
    ("move_reversals_per_s_2deg", 4.67, 10.0),
    ("move_duty_rms_pct_2deg", 12.81, 0.0),
    ("overshoot_deg_20deg", 0.1491, 0.1),
    ("settle_ms_20deg", 296.55, 0.0),
    ("move_current_rms_ma_20deg", 92.83, 0.0),
    ("move_reversals_per_s_20deg", 2.56, 10.0),
    ("move_duty_rms_pct_20deg", 15.96, 0.0),
    ("overshoot_deg_60deg", 0.1987, 0.1),
    ("settle_ms_60deg", 830.95, 0.0),
    ("move_current_rms_ma_60deg", 90.1, 0.0),
    ("move_reversals_per_s_60deg", 0.6043, 10.0),
    ("move_duty_rms_pct_60deg", 16.18, 0.0),
    ("lag_ms_1pct", 19.86, 0.0),
    ("follow_error_sd_deg_1pct", 0.0719, 0.0),
    ("follow_current_rms_ma_1pct", 83.63, 0.0),
    ("follow_reversals_per_s_1pct", 96.51, 10.0),
    ("lag_ms_3pct", 19.08, 0.0),
    ("follow_error_sd_deg_3pct", 0.1364, 0.0),
    ("follow_current_rms_ma_3pct", 94.31, 0.0),
    ("follow_reversals_per_s_3pct", 49.09, 10.0),
    ("lag_ms_10pct", 33.18, 0.0),
    ("follow_error_sd_deg_10pct", 0.3952, 0.0),
    ("follow_current_rms_ma_10pct", 104.32, 0.0),
    ("follow_reversals_per_s_10pct", 9.36, 10.0),
    ("hold_jitter_deg", 0.0617, 0.0),
    ("hold_buzz_share", 0.0, 0.02),
    ("hold_current_rms_ma", 0.0, 14.1),
    ("landing_drive_ms", 28.45, 0.0),
    ("landing_drive_ms_worst", 84.55, 50.0),
];

#[derive(Clone, Copy, Debug, PartialEq)]
enum Test {
    Enable,
    Lead,
    Step { size: u32, dir: i32 },
    Ramp { pct: u32, dir: i32, ramp: usize },
    Pause,
    Hold,
}

struct Seg {
    test: Test,
    goal: Option<i32>,
    /// The item's ticks, and of those the first `rows` captured.
    ticks: u32,
    rows: u32,
}

impl Seg {
    fn hold(ms: u32, test: Test) -> Self {
        let ticks = ms * TICKS_PER_MS;
        Seg {
            test,
            goal: None,
            ticks,
            rows: ticks,
        }
    }

    fn goal(goal: i32, ms: u32, test: Test) -> Self {
        Seg {
            goal: Some(goal),
            ..Self::hold(ms, test)
        }
    }
}

fn round(x: f64) -> i32 {
    x.round_ties_even() as i32
}

/// The soft range's raw middle, on the identity table.
fn center() -> i32 {
    let sh = mg90_d::rig(|_| {});
    let (lo, hi) = sh.table.with(|t| {
        let s = &t.config.pos_soft;
        (s.pos_min_soft_counts, s.pos_max_soft_counts)
    });
    lo + (hi - lo) / 2
}

fn steps(center: i32) -> Vec<Seg> {
    let mut segs = vec![
        Seg::hold(ENABLE_HOLD_MS, Test::Enable),
        Seg::goal(center, STEP_HOLD_MS[0], Test::Lead),
    ];
    for (size, ms) in STEP_SIZES_DEG.into_iter().zip(STEP_HOLD_MS) {
        let out = round(size as f64 * COUNTS_PER_DEG);
        for (goal, dir) in [
            (center + out, 1),
            (center, -1),
            (center - out, -1),
            (center, 1),
        ] {
            segs.push(Seg::goal(goal, ms, Test::Step { size, dir }));
        }
    }
    segs
}

fn follow(center: i32) -> Vec<Seg> {
    let mut segs = vec![Seg::goal(center, PAUSE_MS, Test::Lead)];
    let period_s = GOAL_PERIOD_TICKS as f64 / TICK_HZ;
    let mut ramp = 0;
    for pct in FOLLOW_SPEED_PCT {
        let deg_s = NO_LOAD_DEG_S * pct as f64 / 100.0;
        let span = round(FOLLOW_SPAN_DEG.min(deg_s * FOLLOW_SPAN_S) * COUNTS_PER_DEG);
        let per = deg_s * COUNTS_PER_DEG * period_s;
        for (start, dir) in [(center, 1), (center + span, -1)] {
            let n = (span as f64 / per).ceil() as i32;
            for k in 1..=n {
                segs.push(Seg {
                    test: Test::Ramp { pct, dir, ramp },
                    goal: Some(start + dir * round((k as f64 * per).min(span as f64))),
                    ticks: GOAL_PERIOD_TICKS,
                    rows: GOAL_ITEM_MS * TICKS_PER_MS,
                });
            }
            segs.push(Seg::hold(PAUSE_MS, Test::Pause));
            ramp += 1;
        }
    }
    segs
}

fn hold() -> Vec<Seg> {
    (0..HOLDS).map(|_| Seg::hold(HOLD_MS, Test::Hold)).collect()
}

/// One captured tick, as a TEL row carries it.
#[derive(Clone, Copy)]
struct Row {
    t: f64,
    pos: f64,
    current_raw: u16,
    duty: i16,
}

/// A segment's rows, with the goal in force and the median pot.
struct Rows {
    test: Test,
    goal: i32,
    rows: Vec<Row>,
    pos_f: Vec<f64>,
}

struct Capture {
    segs: Vec<Rows>,
    /// The shunt's zero, the median raw reading over the rows at zero duty.
    zero: f64,
}

impl Capture {
    fn of(&self, f: impl Fn(Test) -> bool) -> impl Iterator<Item = &Rows> {
        self.segs.iter().filter(move |s| f(s.test))
    }

    /// The shunt's read current, mA, 0 where the drive window was not read.
    fn i_ma(&self, r: &Row) -> f64 {
        let floor = DEV_V006_SENSE.i_window_min_ticks;
        if r.duty != 0 && r.duty.unsigned_abs() >= floor {
            (r.current_raw as f64 - self.zero) / CURRENT_COUNTS_PER_A * 1e3
        } else {
            0.0
        }
    }

    fn current_rms<'a>(&self, rows: impl Iterator<Item = &'a Row>) -> f64 {
        rms(rows.map(|r| self.i_ma(r)))
    }
}

fn rms(x: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for v in x {
        sum += v * v;
        n += 1;
    }
    (sum / n as f64).sqrt()
}

fn duty_rms_pct<'a>(rows: impl Iterator<Item = &'a Row>) -> f64 {
    rms(rows.map(|r| r.duty as f64)) / PWM_PERIOD_TICKS * 100.0
}

fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

fn mean(v: impl Iterator<Item = f64>) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for x in v {
        sum += x;
        n += 1;
    }
    sum / n as f64
}

fn sd(v: &[f64]) -> f64 {
    let m = mean(v.iter().copied());
    (v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (v.len() - 1) as f64).sqrt()
}

/// A centered rolling median of `MEDIAN_ROWS` rows, fewer at the ends.
fn median_pos(rows: &[Row]) -> Vec<f64> {
    let half = MEDIAN_ROWS / 2;
    let mut win: Vec<f64> = Vec::with_capacity(MEDIAN_ROWS);
    let (mut lo, mut hi) = (0, 0);
    let mut out = Vec::with_capacity(rows.len());
    for i in 0..rows.len() {
        let (a, b) = (i.saturating_sub(half), (i + half + 1).min(rows.len()));
        while hi < b {
            let x = rows[hi].pos;
            win.insert(win.partition_point(|&y| y < x), x);
            hi += 1;
        }
        while lo < a {
            let x = rows[lo].pos;
            win.remove(win.partition_point(|&y| y < x));
            lo += 1;
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

/// Sign changes of the applied duty, zero rows skipped.
fn reversals<'a>(rows: impl Iterator<Item = &'a Row>) -> u32 {
    let mut last = 0;
    let mut n = 0;
    for r in rows {
        let s = r.duty.signum();
        if s != 0 {
            n += (last != 0 && s != last) as u32;
            last = s;
        }
    }
    n
}

/// One capture of `segs` on the record rig, pot-noise seed `seed`: the
/// plant at rest at `center`, torque on at the first row.
fn run(segs: Vec<Seg>, center: i32, seed: u64) -> Capture {
    let sh = mg90_d::rig(|t| {
        let lp = &mut t.config.loop_position;
        lp.velocity_limit_cps = VELOCITY_LIMIT_CPS;
        lp.accel_limit_q88 = ACCEL_LIMIT_Q88;
        t.control.lifecycle.goal_position = center;
    });
    let mut k = mg90::kernel();
    let mut p = mg90_d::plant(center as f64, seed);
    let mut tick = 0u64;
    let step = |k: &mut Kernel<FakeIo>, p: &mut Mg90| {
        let f = p.step(k.io.motor.last);
        k.on_scan(f, &sh);
    };
    for _ in 0..400 {
        step(&mut k, &mut p);
        tick += 1;
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    let mut goal = center;
    let mut out = Vec::with_capacity(segs.len());
    for s in segs {
        if let Some(g) = s.goal {
            goal = g;
            sh.table.with_mut(|t| t.control.lifecycle.goal_position = g);
        }
        let mut rows = Vec::with_capacity(s.rows as usize);
        for n in 0..s.ticks {
            step(&mut k, &mut p);
            if n < s.rows {
                let tk = sh.table.with(|t| t.telemetry.tick);
                rows.push(Row {
                    t: tick as f64 / TICK_HZ,
                    pos: tk.pos as f64,
                    current_raw: tk.current_raw,
                    duty: tk.duty_ticks,
                });
            }
            tick += 1;
        }
        let pos_f = median_pos(&rows);
        out.push(Rows {
            test: s.test,
            goal,
            rows,
            pos_f,
        });
    }
    let faults = sh.table.with(|t| t.telemetry.common.fault_flags);
    assert_eq!(faults, 0, "seed {seed}: the capture faulted");
    let mut idle: Vec<f64> = out
        .iter()
        .flat_map(|s| &s.rows)
        .filter(|r| r.duty == 0)
        .map(|r| r.current_raw as f64)
        .collect();
    let zero = median(&mut idle);
    Capture { segs: out, zero }
}

type Scores = Vec<(String, f64)>;

/// One move: overshoot past the goal along the travel, deg, the settle,
/// s from the commit to the end of the last row out of the band, and the
/// rows before the settle.
fn step_move(s: &Rows, dir: i32) -> (f64, f64, &[Row]) {
    let goal = s.goal as f64;
    let overshoot = s
        .pos_f
        .iter()
        .map(|p| dir as f64 * (p - goal) / COUNTS_PER_DEG)
        .fold(0.0, f64::max);
    let t0 = s.rows[0].t;
    let settle = s
        .pos_f
        .iter()
        .rposition(|p| (p - goal).abs() / COUNTS_PER_DEG > SETTLE_BAND_DEG)
        .map_or(0.0, |k| s.rows[k].t - t0 + 1.0 / TICK_HZ);
    let n = s.rows.partition_point(|r| r.t - t0 < settle);
    (overshoot, settle, &s.rows[..n])
}

fn step_scores(c: &Capture, out: &mut Scores) {
    for size in STEP_SIZES_DEG {
        let moves: Vec<_> = c
            .of(|t| matches!(t, Test::Step { size: s, .. } if s == size))
            .map(|s| match s.test {
                Test::Step { dir, .. } => step_move(s, dir),
                _ => unreachable!(),
            })
            .collect();
        let spans = || moves.iter().flat_map(|m| m.2);
        let span_s: f64 = moves.iter().map(|m| m.1).sum();
        let revs: u32 = moves.iter().map(|m| reversals(m.2.iter())).sum();
        let tag = format!("{size}deg");
        out.push((
            format!("overshoot_deg_{tag}"),
            moves.iter().map(|m| m.0).fold(f64::MIN, f64::max),
        ));
        out.push((
            format!("settle_ms_{tag}"),
            moves.iter().map(|m| m.1).fold(f64::MIN, f64::max) * 1000.0,
        ));
        out.push((format!("move_current_rms_ma_{tag}"), c.current_rms(spans())));
        out.push((format!("move_reversals_per_s_{tag}"), revs as f64 / span_s));
        out.push((format!("move_duty_rms_pct_{tag}"), duty_rms_pct(spans())));
    }
}

/// One goal: s from the first row of the median pot inside the band to
/// the end of the last row driven from there, 0 if none is; the whole
/// segment when the pot never enters the band.
fn landing_drive_s(s: &Rows) -> f64 {
    let goal = s.goal as f64;
    let Some(first) = s
        .pos_f
        .iter()
        .position(|p| (p - goal).abs() / COUNTS_PER_DEG <= SETTLE_BAND_DEG)
    else {
        return s.rows[s.rows.len() - 1].t - s.rows[0].t + 1.0 / TICK_HZ;
    };
    let rows = &s.rows[first..];
    rows.iter()
        .rposition(|r| r.duty != 0)
        .map_or(0.0, |k| rows[k].t - rows[0].t + 1.0 / TICK_HZ)
}

fn landing_scores(c: &Capture, out: &mut Scores) {
    let mut ms: Vec<f64> = c
        .of(|t| matches!(t, Test::Lead | Test::Step { .. }))
        .map(|s| landing_drive_s(s) * 1000.0)
        .collect();
    let worst = ms.iter().copied().fold(f64::MIN, f64::max);
    out.push(("landing_drive_ms".into(), median(&mut ms)));
    out.push(("landing_drive_ms_worst".into(), worst));
}

fn hold_scores(c: &Capture, out: &mut Scores) {
    let holds: Vec<&[Row]> = c
        .of(|t| t == Test::Hold)
        .enumerate()
        .map(|(n, s)| {
            let t0 = s.rows[0].t;
            let cut = if n == 0 {
                s.rows.partition_point(|r| r.t < t0 + HOLD_HEAD_MS / 1000.0)
            } else {
                0
            };
            &s.rows[cut..]
        })
        .collect();
    let worst = |f: &dyn Fn(&[Row]) -> f64| holds.iter().map(|g| f(g)).fold(f64::MIN, f64::max);
    out.push((
        "hold_jitter_deg".into(),
        worst(&|g| sd(&g.iter().map(|r| r.pos).collect::<Vec<_>>())) / COUNTS_PER_DEG,
    ));
    out.push((
        "hold_buzz_share".into(),
        worst(&|g| g.iter().filter(|r| r.duty != 0).count() as f64 / g.len() as f64),
    ));
    out.push((
        "hold_current_rms_ma".into(),
        worst(&|g| c.current_rms(g.iter())),
    ));
}

fn follow_scores(c: &Capture, out: &mut Scores) {
    for pct in FOLLOW_SPEED_PCT {
        let mut err = Vec::new();
        let mut tails: Vec<Row> = Vec::new();
        let (mut span_s, mut revs, mut speeds) = (0.0, 0, Vec::new());
        let ramps: Vec<usize> = {
            let mut r: Vec<usize> = c
                .segs
                .iter()
                .filter_map(|s| match s.test {
                    Test::Ramp { pct: p, ramp, .. } if p == pct => Some(ramp),
                    _ => None,
                })
                .collect();
            r.dedup();
            r
        };
        for ramp in ramps {
            let items: Vec<&Rows> = c
                .of(|t| matches!(t, Test::Ramp { ramp: r, .. } if r == ramp))
                .collect();
            let dir = match items[0].test {
                Test::Ramp { dir, .. } => dir as f64,
                _ => unreachable!(),
            };
            let (first, last) = (items[0], items[items.len() - 1]);
            let (t0, t1) = (first.rows[0].t, last.rows[last.rows.len() - 1].t);
            let from = t1 - FOLLOW_TAIL * (t1 - t0);
            let mut tail: Vec<Row> = Vec::new();
            for s in &items {
                for (r, p) in s.rows.iter().zip(&s.pos_f) {
                    if r.t >= from {
                        err.push(dir * (s.goal as f64 - p) / COUNTS_PER_DEG);
                        tail.push(*r);
                    }
                }
            }
            span_s += tail[tail.len() - 1].t - tail[0].t;
            revs += reversals(tail.iter());
            speeds.push(
                (last.goal - first.goal).abs() as f64
                    / (last.rows[0].t - first.rows[0].t)
                    / COUNTS_PER_DEG,
            );
            tails.extend(tail);
        }
        let tag = format!("{pct}pct");
        let lag_deg = mean(err.iter().copied());
        let speed = mean(speeds.into_iter());
        out.push((format!("lag_ms_{tag}"), 1000.0 * lag_deg / speed));
        out.push((format!("follow_error_sd_deg_{tag}"), sd(&err)));
        out.push((
            format!("follow_current_rms_ma_{tag}"),
            c.current_rms(tails.iter()),
        ));
        out.push((
            format!("follow_reversals_per_s_{tag}"),
            revs as f64 / span_s,
        ));
    }
}

fn score(seed: u64) -> Scores {
    let center = center();
    let mut out = Scores::new();
    let steps = run(steps(center), center, seed * 3);
    step_scores(&steps, &mut out);
    let follow = run(follow(center), center, seed * 3 + 1);
    follow_scores(&follow, &mut out);
    let hold = run(hold(), center, seed * 3 + 2);
    hold_scores(&hold, &mut out);
    landing_scores(&steps, &mut out);
    out
}

#[test]
fn the_scorecard_rows_hold_their_pins() {
    let per: Vec<Scores> = std::thread::scope(|s| {
        let runs: Vec<_> = SEEDS.map(|n| s.spawn(move || score(n))).into();
        runs.into_iter().map(|r| r.join().expect("seed")).collect()
    });
    let mut failed = Vec::new();
    println!(
        "{:<30} {:>10} {:>22} {:>10}",
        "row", "median", "[lo, hi]", "bound"
    );
    for (k, (name, _)) in per[0].iter().enumerate() {
        let mut v: Vec<f64> = per.iter().map(|p| p[k].1).collect();
        let m = median(&mut v);
        let (lo, hi) = (v[0], v[v.len() - 1]);
        let bound = PINS
            .iter()
            .find(|p| p.0 == name)
            .map(|&(_, rec, floor)| (1.5 * rec).max(rec + floor));
        println!(
            "{name:<30} {m:>10.4} {:>22} {:>10}",
            format!("[{lo:.4}, {hi:.4}]"),
            bound.map_or("-".into(), |b| format!("{b:.4}"))
        );
        match bound {
            Some(b) if m <= b => {}
            Some(b) => failed.push(format!("{name}: {m:.4} over {b:.4}")),
            None => failed.push(format!("{name}: no pin")),
        }
    }
    assert!(failed.is_empty(), "{failed:#?}");
}
