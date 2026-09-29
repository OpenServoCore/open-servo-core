//! Whether a recording holds every segment its schedule promises, each clean,
//! and each drive judged by what its block measures. Under the current limit
//! a drive from rest climbs at the limit before its duty reaches the goal:
//! a grid or ends rung measures its settled tail, so the climb is trimmed
//! and the tail must hold the goal; a breakaway, a step, a reversal and a
//! drive that feeds a coast or a brake measure the climb itself, so they are
//! never declined for it, only marked governed. A duty that falls under its
//! goal after reaching it means the load changed mid rung. Current over the
//! abort means the servo is not holding its limit, and nothing drives it
//! again.

use osc_ident::exp::{Applied, judge};
use osc_ident::limits::Ma;
use osc_ident::runway::{SETTLE_MS, STEADY_MIN_MS};

use super::envelope::{Dir, Envelope};
use super::plan::Block;
use crate::sweep::{Cfg, Recording, Segment, Step};

/// Segments a sweep commits: the baseline, then one per step per direction.
pub(crate) fn expected_segments(baseline: bool, dirs: usize, steps: usize) -> usize {
    baseline as usize + dirs * steps
}

/// Samples a current window averages, as the firmware's ident window does.
const WINDOW: usize = 16;
/// How long a leg that reverses the one before it draws the base duty's
/// volts plus the back-EMF it reverses against, ms: the limiter cannot
/// apply less than the base.
const REVERSAL_MS: f64 = 30.0;
/// Over what a reversal draws by that sum, the residual's margin.
const REVERSAL_MARGIN: f64 = 1.25;

/// What a recording's drives are held against: the abort a quarter over the
/// current limit, and what sizes a reversal's residual.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Abort {
    /// Counts over the torque-off baseline.
    pub(crate) i_abort: f64,
    pub(crate) floor_q15: u16,
    /// vcounts.
    pub(crate) rail: f64,
    /// Winding R, vcounts per ccount.
    pub(crate) r_vpc: f64,
    /// Fast decay: the driven half reads in the trough.
    pub(crate) fast: bool,
    pub(crate) tick_hz: f64,
    pub(crate) ma: Ma,
}

#[derive(Debug, PartialEq)]
pub(crate) enum Verdict {
    /// Per segment, whether the limit held its duty under the goal.
    Accepted { governed: Vec<bool> },
    /// Retried.
    Rejected(String),
    /// A drive that stalled far short of what the pilot measured there.
    Blocked(String),
    /// The servo is not holding its current limit.
    Overcurrent(String),
}

/// The check on a recording `sweep::record` returned Ok (a chain that gave
/// up is already an Err): every segment present, numbered in order, clean,
/// every direction driven, no window over the abort, and each drive judged
/// by its block in `blocks`.
pub(crate) fn verdict(
    rec: &Recording,
    cfg: &Cfg,
    blocks: &[Block],
    env: &Envelope,
    abort: &Abort,
) -> Verdict {
    if let Err(why) = shape(rec, cfg) {
        return Verdict::Rejected(why);
    }
    let baseline = cfg.baseline_ms > 0;
    if let Err(why) = over_abort(&rec.segments, &cfg.steps, baseline, abort) {
        return Verdict::Overcurrent(why);
    }
    let segs = &rec.segments;
    let mut governed = vec![false; segs.len()];
    for (i, s) in segs.iter().enumerate() {
        if s.cmd_duty_q15 == 0 {
            continue;
        }
        let k = (i - baseline as usize) % cfg.steps.len();
        let step = cfg.steps[k];
        let start = match (step, i.checked_sub(1)) {
            (Step::Drive(..), _) | (_, None) => 0,
            (_, Some(prev)) => segs[prev]
                .frames
                .iter()
                .rev()
                .find_map(|f| f.duty_q15)
                .unwrap_or(0),
        };
        let duty = s.frames.iter().filter_map(|f| Some((f.tick, f.duty_q15?)));
        governed[i] = judge(duty, s.cmd_duty_q15, start).contains(&Applied::Governed);
        let block = block_of(blocks, k);
        if let Some(ms) = fell(s, abort.tick_hz) {
            let why = format!(
                "seg {}: the applied duty fell under its goal {ms:.1} ms into the window, after \
                 reaching it: the load changed mid rung",
                s.seg
            );
            if block == Some("grid")
                && let Step::Drive(pct, _) = step
                && let Some((travel, want)) = short(s, env, pct, abort.tick_hz)
            {
                return Verdict::Blocked(format!(
                    "{why}; it travelled {travel} counts of the {want:.0} the pilot measured \
                     there: the shaft is blocked"
                ));
            }
            return Verdict::Rejected(why);
        }
        if matches!(block, Some("grid" | "ends"))
            && matches!(step, Step::Drive(..))
            && let Err(why) = settled(s, abort.tick_hz)
        {
            return Verdict::Rejected(format!("seg {}: {why}", s.seg));
        }
    }
    Verdict::Accepted { governed }
}

/// Every segment present, numbered in order, clean, and every direction
/// driven.
fn shape(rec: &Recording, cfg: &Cfg) -> Result<(), String> {
    let segs = &rec.segments;
    let baseline = cfg.baseline_ms > 0;
    let want = expected_segments(baseline, cfg.dirs.signs().len(), cfg.steps.len());
    if segs.len() != want {
        return Err(format!("{} of {want} segments", segs.len()));
    }
    let first = u32::from(!baseline);
    for (i, s) in segs.iter().enumerate() {
        if s.seg != first + i as u32 {
            return Err(format!(
                "segment {i} is seg {}, not {}",
                s.seg,
                first + i as u32
            ));
        }
        if s.stats.holes > 0 || s.stats.garble > 0 {
            return Err(format!(
                "seg {}: {} holes, {} garble",
                s.seg, s.stats.holes, s.stats.garble
            ));
        }
    }
    for &d in cfg.dirs.signs() {
        if !segs.iter().any(|s| s.dir == d) {
            return Err(format!("no segment drives dir {d:+}"));
        }
    }
    Ok(())
}

/// The block that holds step `k` of the schedule.
pub(crate) fn block_of(blocks: &[Block], k: usize) -> Option<&str> {
    blocks
        .iter()
        .find(|b| (b.first..b.first + b.count).contains(&k))
        .map(|b| b.name.as_str())
}

/// Ms into the segment where its applied duty first fell under its goal
/// after reaching it; None when it never did.
fn fell(s: &Segment, tick_hz: f64) -> Option<f64> {
    let goal = s.cmd_duty_q15;
    let mut reached = false;
    for f in &s.frames {
        let Some(d) = f.duty_q15 else {
            continue;
        };
        if d == goal {
            reached = true;
        } else if reached && (d.signum() != goal.signum() || d.unsigned_abs() < goal.unsigned_abs())
        {
            return Some(f.tick as f64 * 1000.0 / tick_hz);
        }
    }
    None
}

/// A grid rung's travel, and what the pilot measured its duty to cross in
/// the same window, when the rung covered under half of it.
fn short(s: &Segment, env: &Envelope, pct: u8, tick_hz: f64) -> Option<(u16, f64)> {
    let rung = env.grid.rungs.iter().find(|r| r.pct == pct)?;
    let dir = if s.dir > 0 { Dir::Fwd } else { Dir::Rev };
    let r = rung.get(dir);
    let window = s.frames.last()?.tick as f64 * 1000.0 / tick_hz;
    let want = r.travel as f64 + r.v_ss.max(0.0) * (window - r.t_goal_ms).max(0.0);
    let ahead = |p: u16| s.dir as i32 * p as i32;
    let pos: Vec<i32> = s.frames.iter().filter_map(|f| f.pos.map(ahead)).collect();
    let travel = pos.iter().max()? - pos.first()?;
    let travel = travel.max(0) as u16;
    ((travel as f64) < want / 2.0).then_some((travel, want))
}

/// A grid or ends rung's tail: the applied duty holds its goal over the
/// last SETTLE_MS + STEADY_MIN_MS of the window, unbroken.
pub(crate) fn settled(s: &Segment, tick_hz: f64) -> Result<(), String> {
    let goal = s.cmd_duty_q15;
    let Some(last) = s.frames.last() else {
        return Err("no samples".into());
    };
    let held = s
        .frames
        .iter()
        .rev()
        .take_while(|f| f.duty_q15 == Some(goal))
        .last()
        .map_or(0.0, |f| (last.tick - f.tick + 1) as f64 * 1000.0 / tick_hz);
    let need = SETTLE_MS + STEADY_MIN_MS;
    if held < need {
        return Err(format!(
            "the applied duty held its goal for the last {held:.0} ms of the window, under the \
             {need:.0} ms a settled tail needs"
        ));
    }
    Ok(())
}

/// The driven half's current, raw: the crest under slow decay, the trough
/// under fast.
fn current(f: &osc_ident::frame::TelFrame, fast: bool) -> Option<u16> {
    if fast {
        f.current_trough
    } else {
        f.current_raw
    }
}

/// Err when any WINDOW consecutive samples of a drive, its applied duty of
/// the goal's sign and at or over the window floor, average over the abort,
/// the torque-off baseline removed. The first REVERSAL_MS of a leg that
/// reverses the one before it are held against what the reversal draws
/// instead: REVERSAL_MARGIN x (floor x rail + the back-EMF the leg before
/// ended on) / R. Without a baseline there is nothing to judge by.
pub(crate) fn over_abort(
    segs: &[Segment],
    steps: &[Step],
    baseline: bool,
    a: &Abort,
) -> Result<(), String> {
    let Some(base) = baseline
        .then(|| segs.first())
        .flatten()
        .and_then(|s| mean(s.frames.iter().filter_map(|f| current(f, a.fast))))
    else {
        return Ok(());
    };
    let floor = a.floor_q15 as i32;
    for (i, s) in segs.iter().enumerate() {
        let goal = s.cmd_duty_q15;
        if goal == 0 {
            continue;
        }
        let k = (i - baseline as usize) % steps.len();
        let prev = i.checked_sub(1).map(|p| &segs[p]);
        let residual = match (steps[k], prev) {
            (Step::Then(..), Some(p)) if p.cmd_duty_q15.signum() == -goal.signum() => {
                let d = p.cmd_duty_q15.unsigned_abs() as f64 / 32767.0;
                let driven: Vec<f64> = p
                    .frames
                    .iter()
                    .filter_map(|f| current(f, a.fast))
                    .map(|c| c as f64 - base)
                    .collect();
                let tail = &driven[driven.len().saturating_sub(WINDOW)..];
                let i_end = mean(tail.iter().map(|&c| c.round().max(0.0) as u16)).unwrap_or(0.0);
                let emf = (d * a.rail - a.r_vpc * i_end).max(0.0);
                let f = floor as f64 / 32767.0;
                Some(REVERSAL_MARGIN * (f * a.rail + emf) / a.r_vpc)
            }
            _ => None,
        };
        let eligible = |f: &osc_ident::frame::TelFrame| -> Option<f64> {
            let d = f.duty_q15? as i32;
            let c = current(f, a.fast)?;
            (d.signum() == goal.signum() as i32 && d.abs() >= floor).then_some(c as f64 - base)
        };
        let reversal_ticks = REVERSAL_MS * a.tick_hz / 1000.0;
        let mut sum = 0.0;
        let mut run = 0usize;
        for (j, f) in s.frames.iter().enumerate() {
            match eligible(f) {
                Some(c) => {
                    sum += c;
                    run += 1;
                    if run > WINDOW {
                        sum -= eligible(&s.frames[j - WINDOW]).unwrap_or(0.0);
                        run = WINDOW;
                    }
                }
                None => {
                    sum = 0.0;
                    run = 0;
                }
            }
            if run < WINDOW {
                continue;
            }
            let m = sum / WINDOW as f64;
            let first = s.frames[j + 1 - WINDOW].tick as f64;
            let limit = match residual {
                Some(r) if first < reversal_ticks => r.max(a.i_abort),
                _ => a.i_abort,
            };
            if m > limit {
                return Err(format!(
                    "the servo is not holding its current limit: seg {} drew a {WINDOW}-sample \
                     mean of {} at {:.1} ms, over the abort of {}",
                    s.seg,
                    a.ma.of(m),
                    f.tick as f64 * 1000.0 / a.tick_hz,
                    a.ma.of(limit)
                ));
            }
        }
    }
    Ok(())
}

fn mean(v: impl Iterator<Item = u16>) -> Option<f64> {
    let (sum, n) = v.fold((0.0, 0usize), |(s, n), c| (s + c as f64, n + 1));
    (n > 0).then(|| sum / n as f64)
}

#[cfg(test)]
pub(super) mod bench {
    use super::*;
    use crate::capture::Supply;
    use crate::capture::front;
    use crate::rig::servo::bench::Bench;
    use crate::sweep::{Decay, Dirs};

    /// A recording's schedule both ways from a torque-off baseline, at the
    /// bench servo's seek.
    pub(crate) fn cfg(steps: Vec<Step>, dirs: Dirs) -> Cfg {
        Cfg {
            steps,
            dirs,
            decay: Decay::Slow,
            window_ms: 150,
            rest_ms: 500,
            baseline_ms: 200,
            seek_duty_pct: 15,
            seek_cap_pct: 15,
            settle_ms: 300,
            stall: false,
            static_load: false,
            guard: (532, 3526),
            stops: Some((232, 3849)),
            tel_mask: 0x1cd,
            rung_tries: 3,
        }
    }

    /// One block holding the whole schedule.
    pub(crate) fn one_block(name: &str, cfg: &Cfg) -> Vec<Block> {
        vec![Block {
            name: name.into(),
            first: 0,
            count: cfg.steps.len(),
        }]
    }

    /// `cfg` recorded on the bench servo on 2S, `set` shaping it first.
    pub(crate) fn record(cfg: &Cfg, set: impl FnOnce(&mut Bench)) -> Recording {
        let mut b = Bench::mg90(Supply::TwoS);
        set(&mut b);
        crate::sweep::record(&mut b, cfg, |_| Ok(())).unwrap()
    }

    pub(crate) fn abort() -> Abort {
        front::bench().abort(Decay::Slow)
    }
}

#[cfg(test)]
mod tests {
    use super::bench::{abort, cfg as bench_cfg, one_block, record};
    use super::*;
    use crate::capture::envelope;
    use crate::capture::store::fixture::{plan, sweep_meta, tmp};
    use crate::capture::store::{Capture, CaptureMeta, Store};
    use crate::rig::pump::BurstStats;
    use crate::sweep::{Decay, Dirs, Segment};
    use osc_ident::frame::TelFrame;

    fn cfg(dirs: Dirs, baseline_ms: u32) -> Cfg {
        Cfg {
            steps: vec![Step::Drive(20, Some(80)), Step::Coast(400)],
            dirs,
            decay: Decay::Slow,
            window_ms: 150,
            rest_ms: 1500,
            baseline_ms,
            seek_duty_pct: 15,
            seek_cap_pct: 15,
            settle_ms: 300,
            stall: false,
            static_load: false,
            guard: (532, 3526),
            stops: Some((232, 3849)),
            tel_mask: 0x1cd,
            rung_tries: 3,
        }
    }

    fn seg(seg: u32, dir: i8, holes: u64) -> Segment {
        Segment {
            seg,
            dir,
            cmd_duty_q15: 0,
            frames: Vec::new(),
            stats: BurstStats {
                frames: 1,
                samples: 16,
                holes,
                garble: 0,
            },
        }
    }

    /// What a clean both-ways run commits: seg 0, then fwd, then rev.
    fn clean(cfg: &Cfg) -> Recording {
        let n = cfg.steps.len() as u32;
        let mut segments = Vec::new();
        if cfg.baseline_ms > 0 {
            segments.push(seg(0, 0, 0));
        }
        for (d, &dir) in cfg.dirs.signs().iter().enumerate() {
            for k in 0..n {
                segments.push(seg(1 + d as u32 * n + k, dir, 0));
            }
        }
        Recording { segments }
    }

    fn judged(r: &Recording, c: &Cfg) -> Verdict {
        verdict(r, c, &one_block("coast", c), &envelope::mg90(), &abort())
    }

    fn rejected(r: &Recording, c: &Cfg) -> String {
        match judged(r, c) {
            Verdict::Rejected(why) => why,
            v => panic!("{v:?}"),
        }
    }

    #[test]
    fn clean_recordings_pass() {
        for c in [
            cfg(Dirs::Both, 1000),
            cfg(Dirs::Fwd, 0),
            cfg(Dirs::Rev, 1000),
        ] {
            let n = clean(&c).segments.len();
            assert_eq!(
                judged(&clean(&c), &c),
                Verdict::Accepted {
                    governed: vec![false; n]
                }
            );
        }
    }

    #[test]
    fn a_short_recording_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        r.segments.pop();
        assert_eq!(rejected(&r, &c), "4 of 5 segments");
    }

    #[test]
    fn a_dirty_segment_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        r.segments[3].stats.holes = 2;
        assert!(rejected(&r, &c).starts_with("seg 3: 2 holes"));
        let mut r = clean(&c);
        r.segments[1].stats.garble = 1;
        assert!(rejected(&r, &c).contains("1 garble"));
    }

    #[test]
    fn a_missing_direction_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        for s in &mut r.segments[3..] {
            s.dir = 1;
        }
        assert_eq!(rejected(&r, &c), "no segment drives dir -1");
    }

    #[test]
    fn segments_number_from_the_baseline() {
        let c = cfg(Dirs::Fwd, 1000);
        let mut r = clean(&c);
        r.segments[2].seg = 4;
        assert!(rejected(&r, &c).contains("is seg 4, not 2"));
        let c = cfg(Dirs::Fwd, 0);
        let mut r = clean(&c);
        r.segments[0].seg = 0;
        assert!(rejected(&r, &c).contains("is seg 0, not 1"));
    }

    fn grid(ms: u32) -> (Recording, Cfg) {
        let c = bench_cfg(vec![Step::Drive(40, Some(ms))], Dirs::Both);
        (record(&c, |_| {}), c)
    }

    fn grid_verdict(r: &Recording, c: &Cfg) -> Verdict {
        verdict(r, c, &one_block("grid", c), &envelope::mg90(), &abort())
    }

    /// The bench servo's 40% rung at the pilot's 361 ms window: the limit
    /// holds the duty under the goal for its first 72 ms, then the goal holds
    /// to the end. The climb is trimmed, the tail settled: accepted, the
    /// rung marked governed.
    #[test]
    fn a_governed_climb_is_trimmed_when_the_tail_settles() {
        let (r, c) = grid(361);
        let first = r.segments[1]
            .frames
            .iter()
            .find(|f| f.duty_q15 == Some(r.segments[1].cmd_duty_q15))
            .unwrap();
        assert!(first.tick > 1000, "the goal after {} ticks", first.tick);
        assert_eq!(
            grid_verdict(&r, &c),
            Verdict::Accepted {
                governed: vec![false, true, true]
            }
        );
        assert!(settled(&r.segments[1], 20_000.0).is_ok());
    }

    /// The same rung in 120 ms reaches its goal at 72 ms and holds it for
    /// only 48 before the window ends; in 60 ms it never reaches it.
    /// Neither measured a settled speed: rejected, and retried.
    #[test]
    fn a_grid_rung_that_never_settles_is_rejected() {
        let (r, c) = grid(120);
        let why = rejected_grid(&r, &c);
        assert!(
            why.starts_with("seg 1: the applied duty held its goal for the last 4"),
            "{why}"
        );
        assert!(
            why.ends_with("ms of the window, under the 100 ms a settled tail needs"),
            "{why}"
        );
        let (r, c) = grid(60);
        assert_eq!(
            rejected_grid(&r, &c),
            "seg 1: the applied duty held its goal for the last 0 ms of the window, under the \
             100 ms a settled tail needs"
        );
    }

    fn rejected_grid(r: &Recording, c: &Cfg) -> String {
        match grid_verdict(r, c) {
            Verdict::Rejected(why) => why,
            v => panic!("{v:?}"),
        }
    }

    /// The step block's 30% for 20 ms never reaches its goal: the limit
    /// holds it the whole drive. What it measures is that climb, so it is
    /// accepted, marked governed, and the mark lands in the meta.
    #[test]
    fn a_governed_step_is_accepted_and_marked() {
        let c = bench_cfg(vec![Step::Drive(30, Some(20)), Step::Coast(400)], Dirs::Fwd);
        let r = record(&c, |_| {});
        let drive = &r.segments[1];
        assert!(
            drive
                .frames
                .iter()
                .all(|f| f.duty_q15.unwrap() < drive.cmd_duty_q15)
        );
        let v = verdict(&r, &c, &one_block("step", &c), &envelope::mg90(), &abort());
        let Verdict::Accepted { governed } = v else {
            panic!("{v:?}");
        };
        assert_eq!(governed, [false, true, false]);
        // the same drive as a grid rung measures nothing
        assert!(matches!(grid_verdict(&r, &c), Verdict::Rejected(_)));

        let root = tmp("verdict-governed");
        let cap = Capture::open(&Store::new(root.clone()), "session", 1).unwrap();
        let mut meta = sweep_meta();
        meta["drive"] = serde_json::json!({ "rule": "limit", "current_limit_counts": 280 });
        meta["schedule"] = serde_json::json!(["30@20", "coast:400"]);
        meta["dirs"] = serde_json::json!([1]);
        let mut t = cap.begin("slow", &meta).unwrap();
        for s in &r.segments {
            t.on_seg(s).unwrap();
        }
        let mut p = plan();
        p.schedule = c.steps.clone();
        t.accept(&CaptureMeta {
            supply: crate::capture::Supply::TwoS,
            plan: &p,
            attempt: 1,
            governed: &governed,
        })
        .unwrap();
        let text = std::fs::read_to_string(cap.dir().join("slow.meta.json")).unwrap();
        let landed: serde_json::Value = serde_json::from_str(&text).unwrap();
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(
            landed["drive"]["governed"],
            serde_json::json!([false, true, false])
        );
    }

    /// A rung that reached its goal and then lost it, the load changed under
    /// it: a sticky stretch of the train, the rung carried on past it, is
    /// rejected and retried; a shaft stopped dead on the way, under half
    /// the travel the pilot measured at that duty, is blocked.
    #[test]
    fn a_duty_that_falls_under_its_goal_rejects_the_chain() {
        let mut c = bench_cfg(vec![Step::Drive(40, Some(361))], Dirs::Fwd);
        c.baseline_ms = 0;
        let from_band = |b: &mut crate::rig::servo::bench::Bench| b.servo.pos = 600.0;
        let sticky = record(&c, |b| {
            from_band(b);
            b.servo.sticky = Some((1500.0, 1700.0, 250.0));
        });
        let why = match grid_verdict(&sticky, &c) {
            Verdict::Rejected(why) => why,
            v => panic!("{v:?}"),
        };
        assert!(
            why.starts_with("seg 1: the applied duty fell under its goal"),
            "{why}"
        );
        assert!(why.ends_with("after reaching it: the load changed mid rung"));

        let jammed = record(&c, |b| {
            from_band(b);
            b.servo.jam = Some(1200.0);
        });
        let why = match grid_verdict(&jammed, &c) {
            Verdict::Blocked(why) => why,
            v => panic!("{v:?}"),
        };
        assert!(
            why.contains("the load changed mid rung; it travelled "),
            "{why}"
        );
        assert!(
            why.ends_with("the pilot measured there: the shaft is blocked"),
            "{why}"
        );

        // under the step block the same dip still rejects the chain
        let v = verdict(
            &sticky,
            &c,
            &one_block("step", &c),
            &envelope::mg90(),
            &abort(),
        );
        assert!(matches!(v, Verdict::Rejected(_)), "{v:?}");
    }

    /// The reversal block's 20% legs on the bench servo: each reversed leg
    /// draws the base duty's volts plus the back-EMF it reverses, over the
    /// abort a quarter above the limit, for its first milliseconds. That is
    /// the residual the firmware states, not a limit the servo lost.
    #[test]
    fn the_reversal_residual_is_not_an_abort() {
        let steps = vec![
            Step::Drive(20, Some(60)),
            Step::Then(-20, Some(60)),
            Step::Then(20, Some(60)),
            Step::Brake(200),
        ];
        let c = bench_cfg(steps, Dirs::Both);
        let r = record(&c, |_| {});
        let v = verdict(
            &r,
            &c,
            &one_block("reversal", &c),
            &envelope::mg90(),
            &abort(),
        );
        assert!(matches!(v, Verdict::Accepted { .. }), "{v:?}");

        // judged as legs from rest, the same samples trip the abort
        let from_rest: Vec<Step> = c
            .steps
            .iter()
            .map(|s| match *s {
                Step::Then(pct, ms) => Step::Drive(pct.unsigned_abs(), ms),
                s => s,
            })
            .collect();
        let e = over_abort(&r.segments, &from_rest, true, &abort()).unwrap_err();
        assert!(
            e.starts_with(
                "the servo is not holding its current limit: seg 2 drew a 16-sample \
                           mean of"
            ),
            "{e}"
        );
    }

    /// A 16-sample window of the driven half over the abort, the
    /// baseline removed; fifteen hot samples average under it, and a duty
    /// under the floor, or a recording without a baseline, is not judged.
    #[test]
    fn a_window_over_the_abort_is_found() {
        let a = abort();
        let frame = |t: u64, duty: i16, raw: u16| TelFrame {
            tick: t,
            duty_q15: Some(duty),
            current_raw: Some(raw),
            ..TelFrame::default()
        };
        let segs = |frames: Vec<TelFrame>| {
            let mut base = seg(0, 0, 0);
            base.frames = (0..20).map(|t| frame(t, 0, 512)).collect();
            let mut drive = seg(1, 1, 0);
            drive.cmd_duty_q15 = 9830;
            drive.frames = frames;
            [base, drive]
        };
        let hot = |n: u64| -> Vec<TelFrame> {
            (0..100)
                .map(|t| {
                    let raw = if (40..40 + n).contains(&t) {
                        512 + 360
                    } else {
                        512 + 100
                    };
                    frame(t, 9830, raw)
                })
                .collect()
        };
        let steps = [Step::Drive(30, Some(5))];
        let e = over_abort(&segs(hot(16)), &steps, true, &a).unwrap_err();
        assert_eq!(
            e,
            "the servo is not holding its current limit: seg 1 drew a 16-sample mean of 360 \
             counts (322 mA) at 2.8 ms, over the abort of 350 counts (313 mA)"
        );
        assert!(over_abort(&segs(hot(15)), &steps, true, &a).is_ok());
        // under the window floor the shunt reads nothing to judge
        let under = hot(16)
            .into_iter()
            .map(|f| TelFrame {
                duty_q15: Some(4000),
                ..f
            })
            .collect();
        assert!(over_abort(&segs(under), &steps, true, &a).is_ok());
        // nor with no baseline to take the zero from
        let [_, drive] = segs(hot(16));
        assert!(over_abort(&[drive], &steps, false, &a).is_ok());
    }
}
