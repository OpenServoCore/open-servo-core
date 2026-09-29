//! `osc capture pilot`: measure the servo under its current limit and write
//! the envelope the campaign sizes its windows by. A rung is one unpolled
//! TEL burst, so its window is the only bound on how far the shaft travels.
//! The pilot climbs the grid block's duties from the bottom, both ways, one
//! runway (osc-ident `runway`) per direction, and runs a rung only while its
//! predicted climb, tail and braked stop fit the runway and that stop fits
//! between the soft limit and the stop beyond it: a rung its host abandons
//! is braked at the soft limit by the firmware and must stop short of the
//! stop. Each rung's climb is read off the applied duty, its speed off the
//! samples at the goal past the settle, its stop off where the brake left
//! it, and the next rung is sized by them. The coast ladder and every other
//! chain the session drives then run the same way, each excursion recorded
//! so a plan refuses a chain the pilot never ran. The capture front runs
//! first, and the pack is read at rest before every recording.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::pipe::Pipe;
use osc_ident::frame::TelFrame;
use osc_ident::limits::{POT_MAX, guards, is_board_default};
use osc_ident::runway::{
    self, Climb, Runway, SETTLE_MS, STEADY_MIN_MS, WINDOW_MAX_MS, coast_fit, dead_host_fits, fits,
};

use super::envelope::{
    ChainRefused, ChainRun, Coast, CoastRun, CoastRung, Dir, Envelope, Excursion, Fit, Grid,
    GridRung, LadderRefused, Limits, PerDir, Refused, RungRun, Speed, civil_date, round_to,
};
use super::front::{self, Front, Proved};
use super::procs::{CoastBlock, Procedure};
use super::{REST_MS, RULE, RUNG_TRIES, SETTLE_MS as SEEK_SETTLE_MS, Supply, TEL_MASK, WINDOW_MS};
use crate::rig::park::park;
use crate::rig::pump;
use crate::rig::servo::{Servo, Wire, guard};
use crate::rig::{battery, blocked, centre};
use crate::sweep::{self, BAND_HALF, Cfg, Decay, Dirs, Recording, Segment, Step, feeds, pct_q15};

/// `osc capture pilot` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>__limit`.
    #[arg(long)]
    servo: String,
    /// Supply the servo runs on.
    #[arg(long, value_enum)]
    supply: Supply,
    /// Dataset root; defaults to notebooks/telemetry in this git checkout.
    #[arg(long)]
    root: Option<PathBuf>,
}

/// Shortest guard-to-guard runway worth sizing windows against.
const PILOT_MIN_RUNWAY: u16 = 1500;
/// The window of a rung nothing below it sizes: the shaft has not moved in
/// any rung yet. At the most such a rung may run at, it crosses under a
/// third of the bench runway.
const PILOT_W0: u32 = 150;
/// Highest duty a rung, or the coast ladder, may open on with nothing
/// measured to size it by.
const FIRST_MAX_PCT: u8 = 30;
/// A rung's window runs its predicted climb this many times over, then its
/// tail.
const CLIMB_MARGIN: f64 = 1.25;
/// Share added to a campaign rung's travel before it is held against the
/// room.
const WINDOW_MARGIN: f64 = 0.1;
/// The brake write after a stream's last frame, ms: the shaft runs on at
/// speed until it lands (a write takes 1.7 ms on the bench bus).
const GAP_MS: f64 = 10.0;
/// v_ss is fit over the last 40% of a coast, to judge whether it stopped.
const SETTLED_FRAC: f64 = 0.4;
/// Fewest pos samples (1 ms of ticks) a slope is fit from.
const SETTLED_MIN_SAMPLES: usize = 20;
/// Share of the runway a campaign rung is sized to cross; the bench hand
/// table's bar.
const RUNWAY_FRAC: f64 = 0.8;
/// Below this many counts/ms the shaft is breaking away, not cruising: its
/// rung sizes nothing, and stays out of the speed fit.
const V_SS_MIN: f64 = 0.3;
/// A coast's drive runs on this long past its goal: its entry speed is the
/// pot slope over those samples.
const ENTRY_MS: u32 = 20;
/// Share added to a predicted coast or chain excursion before it is held
/// against the room, and to a measured one before its duty can top the
/// coast block. The worst under-prediction replaying mg90-a's five 2S
/// spindown captures through the coast ladder was 2.8%.
const COAST_MARGIN: f64 = 0.1;
const DIRS: [Dir; 2] = [Dir::Fwd, Dir::Rev];

/// Entry from `osc capture pilot`.
pub(crate) fn run(a: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let root = match &a.root {
        Some(r) => r.clone(),
        None => super::default_root()?,
    };
    let dir = super::dataset_dir(&root, &a.servo, a.supply);
    let (procedure, _) = Procedure::load()?;
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);
    // Nothing has moved yet, so a refusal here leaves the horn where it is.
    let front = front::read(&mut c, id, a.supply)?;
    let fw = c.identity(id)?.fw;
    let mut s = Wire::new(c, id);
    let (env, parked) = pilot(&mut s, &front, a.supply, &procedure, fw);
    let env = match env {
        Err(e) if blocked(&e) => front::exit_blocked(&e),
        r => r?,
    };
    std::fs::create_dir_all(&dir).with_context(|| format!("mkdir {}", dir.display()))?;
    let path = env.save(&dir)?;
    println!("envelope: {}", path.display());
    parked
}

/// The jam check, every measurement and the park at mid travel: the
/// envelope, and how the park went. A blocked shaft is left where it
/// stopped.
pub(super) fn pilot<S: Servo>(
    s: &mut S,
    front: &Front,
    supply: Supply,
    p: &Procedure,
    fw: u16,
) -> (Result<Envelope>, Result<()>) {
    let mut proved = None;
    let env = measure_all(s, front, supply, p, fw, &mut proved);
    let parked = match (&env, proved) {
        (Err(e), _) if blocked(e) => Ok(()),
        (_, Some(p)) => {
            let center = limits(front.lim.soft, front.lim.phys).map(|l| l.center);
            match center {
                Ok(center) => guard(s, |s| park(s, center, p.seek_q15())),
                Err(e) => Err(e),
            }
        }
        (_, None) => Ok(()),
    };
    (env, parked)
}

fn measure_all<S: Servo>(
    s: &mut S,
    front: &Front,
    supply: Supply,
    p: &Procedure,
    fw: u16,
    proved: &mut Option<Proved>,
) -> Result<Envelope> {
    if front.tick_hz == 0 {
        bail!("tick_hz reads 0: TEL sample rate unknown");
    }
    let grid = ladder_duties(&p.block.grid.duties, None).context("the grid block")?;
    let coast_duties =
        ladder_duties(&p.block.coast.duties, Some(FIRST_MAX_PCT)).context("the coast block")?;
    let lim = limits(front.lim.soft, front.lim.phys)?;
    println!(
        "[limits] soft {}..{} phys {}..{} guard {}..{} runway {} centre {}",
        lim.soft[0],
        lim.soft[1],
        lim.phys[0],
        lim.phys[1],
        lim.guard[0],
        lim.guard[1],
        lim.runway,
        lim.center
    );
    for d in DIRS {
        println!(
            "  {d}: {:.0} counts from the soft limit to the stop beyond it, where a rung's braked \
             stop must fit",
            margin(&lim, d)
        );
    }
    let p_ok = front.jam_check(|exp| centre::drive(s, exp))?;
    *proved = Some(p_ok);
    let rig = Rig {
        lim,
        proved: p_ok,
        hz: front.tick_hz as f64 / 1000.0,
    };

    let ladder = grid_ladder(s, &rig, &grid)?;
    let v_ss = speed(&ladder.rungs)?;
    for d in DIRS {
        let f = v_ss.of(d);
        println!(
            "[fit] {d}: v_ss = {} x duty {:+} counts/ms, r2 {}",
            f.slope, f.intercept, f.r2
        );
    }
    let windows_ms: BTreeMap<u8, u32> = ladder.rungs.iter().map(|r| (r.pct, r.window_ms)).collect();
    let row: Vec<String> = windows_ms.iter().map(|(d, w)| format!("{d}:{w}")).collect();
    println!("[windows] {}", row.join(" "));
    let top_pct = ladder.rungs.last().map_or(0, |r| r.pct);
    let grid = Grid {
        top_pct,
        rungs: ladder.rungs,
        refused: ladder.refused,
    };

    let coast = coast_ladder(s, &rig, &p.block.coast, &coast_duties, &grid, &v_ss)?;
    let (chains, refused_chains) = chains(s, &rig, p, &v_ss, &coast, &ladder.runways)?;
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    Ok(Envelope {
        supply,
        fw,
        git_sha: sweep::git_sha(),
        measured: civil_date(secs),
        seek_pct: p_ok.seek_pct(),
        rule: RULE,
        current_limit_counts: Some(front.lim.i_lim),
        rail_mv: Some(front.rail_mv),
        limits: lim,
        v_ss,
        windows_ms,
        grid,
        coast,
        chains,
        refused_chains,
    })
}

/// The servo the pilot drives: its limits, the seek its jam check proved,
/// and its fast ticks per ms.
struct Rig {
    lim: Limits,
    proved: Proved,
    hz: f64,
}

/// Counts between the soft limit a drive of `dir` runs at and the stop
/// beyond it.
fn margin(lim: &Limits, dir: Dir) -> f64 {
    match dir {
        Dir::Fwd => lim.phys[1] as f64 - lim.soft[1] as f64,
        Dir::Rev => lim.soft[0] as f64 - lim.phys[0] as f64,
    }
}

/// What the grid ladder kept, the first duty it did not, and the runway
/// each direction ended with.
struct Ladder {
    rungs: Vec<GridRung>,
    refused: Option<LadderRefused>,
    runways: [Runway; 2],
}

/// Climb `duties` from the bottom, each both ways; the first rung that does
/// not fit, or does not keep a window, ends the ladder.
fn grid_ladder<S: Servo>(s: &mut S, rig: &Rig, duties: &[u8]) -> Result<Ladder> {
    let g = (rig.lim.guard[0], rig.lim.guard[1]);
    let mut ladder = Ladder {
        rungs: Vec::new(),
        refused: None,
        runways: [Runway::new(g), Runway::new(g)],
    };
    println!(
        "[grid] {duties:?}% from the bottom, both ways, while each fits {:.0} counts of runway",
        ladder.runways[0].room()
    );
    for &pct in duties {
        let mut runs = Vec::new();
        for (k, dir) in DIRS.into_iter().enumerate() {
            match grid_rung(s, rig, &mut ladder.runways[k], dir, pct)? {
                Ok(r) => runs.push(r),
                Err(why) => {
                    println!("  {why}: the ladder ends here");
                    ladder.refused = Some(LadderRefused { pct, dir, why });
                    return Ok(ladder);
                }
            }
        }
        let room = ladder.runways[0].room();
        match keep(rig, pct, &runs, room) {
            Ok(window_ms) => {
                println!("  {pct}%: window {window_ms} ms");
                ladder.rungs.push(GridRung {
                    pct,
                    window_ms,
                    fwd: runs[0].record(),
                    rev: runs[1].record(),
                });
            }
            Err((dir, why)) => {
                println!("  {why}: the ladder ends here");
                ladder.refused = Some(LadderRefused { pct, dir, why });
                return Ok(ladder);
            }
        }
    }
    Ok(ladder)
}

/// A rung that ran.
struct Ran {
    ms: u32,
    climb: Climb,
    v_ss: f64,
    stop: f64,
}

impl Ran {
    fn record(&self) -> RungRun {
        RungRun {
            ran_ms: self.ms,
            t_goal_ms: round_to(self.climb.ms, 2),
            travel: to_counts(self.climb.travel),
            v_ss: round_to(self.v_ss, 3) + 0.0,
            stop: to_counts(self.stop),
        }
    }
}

/// One grid rung of `dir`: sized, run, measured, and fed to the runway. A
/// rung that did not reach its goal or hold it past the settle runs once
/// more at a window that would, when that still fits. Err in the Ok is why
/// the rung does not run.
fn grid_rung<S: Servo>(
    s: &mut S,
    rig: &Rig,
    rw: &mut Runway,
    dir: Dir,
    pct: u8,
) -> Result<Result<Ran, String>> {
    let tail = SETTLE_MS + STEADY_MIN_MS;
    let mut window = match first_window(rw, rig, dir, pct, tail) {
        Ok(w) => w,
        Err(why) => return Ok(Err(why)),
    };
    let sign = sign(dir);
    let goal = (sign as i32 * pct_q15(pct) as i32) as i16;
    for retry in [false, true] {
        let (frames, stop) = run_rung(s, rig, dir, pct, window)?;
        let climb = runway::climb(&frames, goal, rig.hz * 1000.0);
        let v = climb.and_then(|_| settled_v(&frames, goal, rig.hz));
        match (climb, v) {
            (Some(climb), Some(v_ss)) => {
                println!(
                    "[rung] {dir} {pct}@{window}: goal after {:.1} ms and {:.0} counts, v_ss \
                     {v_ss:.2} counts/ms, braked in {stop:.0}",
                    climb.ms,
                    climb.travel + 0.0
                );
                if v_ss > V_SS_MIN {
                    rw.climbed(climb.accel);
                    rw.ran(pct as f64 / 100.0, v_ss);
                    rw.stopped(v_ss, stop);
                }
                return Ok(Ok(Ran {
                    ms: window,
                    climb,
                    v_ss,
                    stop,
                }));
            }
            _ if !retry => {
                let longer = match climb {
                    Some(c) => ((c.ms + tail).ceil() as u32 + 1).max(window + 1),
                    None => 2 * window,
                };
                if let Err(why) = sized(rw, rig, dir, pct, longer) {
                    return Ok(Err(why));
                }
                println!("  {dir} {pct}@{window} settled too late: once more at {longer} ms");
                window = longer;
            }
            (None, _) => {
                return Ok(Err(format!(
                    "{dir} {pct}% never reached its goal duty in {window} ms"
                )));
            }
            (Some(_), None) => {
                return Ok(Err(format!(
                    "{dir} {pct}% did not hold its goal duty for {STEADY_MIN_MS:.0} ms past the \
                     settle in {window} ms"
                )));
            }
        }
    }
    unreachable!("the second try always returns")
}

/// The window a rung first runs at: its predicted climb 1.25 times over and
/// `tail`, when the runway can size it; a short one when nothing has moved
/// the shaft yet.
fn first_window(rw: &Runway, rig: &Rig, dir: Dir, pct: u8, tail: f64) -> Result<u32, String> {
    match rw.plan(sign(dir), pct as f64 / 100.0, 0.0) {
        Some(n) => {
            let w = (CLIMB_MARGIN * n.climb_ms + tail).ceil() as u32;
            sized(rw, rig, dir, pct, w).map(|()| w)
        }
        None if pct <= FIRST_MAX_PCT => Ok(PILOT_W0),
        None => Err(format!(
            "{dir} {pct}%: no rung under it moved the shaft, so nothing sizes it"
        )),
    }
}

/// A rung of `window` ms fits: its climb, its run at speed for what the
/// window leaves and its margined braked stop fit the runway, and that stop
/// fits between the soft limit and the stop beyond it.
fn sized(rw: &Runway, rig: &Rig, dir: Dir, pct: u8, window: u32) -> Result<(), String> {
    let d = pct as f64 / 100.0;
    let Some(n0) = rw.plan(sign(dir), d, 0.0) else {
        return if pct <= FIRST_MAX_PCT && window <= 2 * PILOT_W0 {
            Ok(())
        } else {
            Err(format!(
                "{dir} {pct}%: no rung under it moved the shaft, so nothing sizes it"
            ))
        };
    };
    let steady = (window as f64 - n0.climb_ms).max(0.0);
    let need = rw
        .plan(sign(dir), d, steady)
        .ok_or_else(|| format!("{dir} {pct}%: nothing sizes it"))?;
    if !fits(&need, rw.room()) {
        return Err(format!(
            "{dir} {pct}% in {window} ms needs {:.0} counts of runway, over the {:.0} there is",
            need.total(),
            rw.room()
        ));
    }
    let m = margin(&rig.lim, dir);
    if !dead_host_fits(need.stop, m) {
        return Err(format!(
            "{dir} {pct}% would stop in about {:.0} counts, over the {m:.0} between the soft \
             limit and the stop: a rung its host abandoned would hit the stop",
            need.stop
        ));
    }
    Ok(())
}

/// One rung, the pack gated before it: its frames, and how far the brake
/// let the shaft run on past the last of them.
fn run_rung<S: Servo>(
    s: &mut S,
    rig: &Rig,
    dir: Dir,
    pct: u8,
    window: u32,
) -> Result<(Vec<TelFrame>, f64)> {
    let step = Step::Drive(pct, Some(window));
    let rec = record(s, &cfg(vec![step], sweep_dirs(dir), window, rig))?;
    let seg = one(&rec.segments)?;
    let last = seg
        .frames
        .iter()
        .rev()
        .find_map(|f| f.pos)
        .ok_or_else(|| anyhow!("{dir} {step} streamed no position"))?;
    let rest = s.snapshot()?.pos;
    let stop = (f64::from(sign(dir)) * (rest as f64 - last as f64)).max(0.0);
    Ok((seg.frames.clone(), stop))
}

/// Steady speed along the goal's sign, counts/ms, over the samples the
/// applied duty held the goal past the settle (`runway::settled_ticks`):
/// the climb and anything the limit governed stay out. None when those
/// samples span under the least steady time.
fn settled_v(frames: &[TelFrame], goal_q15: i16, ticks_per_ms: f64) -> Option<f64> {
    let range = runway::settled_ticks(frames, goal_q15, ticks_per_ms * 1000.0)?;
    if ((range.end() - range.start()) as f64) < STEADY_MIN_MS * ticks_per_ms {
        return None;
    }
    let pts: Vec<(u64, u16)> = frames
        .iter()
        .filter(|f| range.contains(&f.tick))
        .filter_map(|f| f.pos.map(|p| (f.tick, p)))
        .collect();
    let slope = slope_from(&pts, *range.start() as f64)?;
    let sign = if goal_q15 < 0 { -1.0 } else { 1.0 };
    Some(sign * slope * ticks_per_ms)
}

/// The window a campaign rung drives at `pct`, both ways: each direction's
/// climb and a tail crossing RUNWAY_FRAC of the runway (`runway::window_ms`),
/// cut to where its travel over the window and the brake write after it,
/// WINDOW_MARGIN added, and its braked stop, STOP_MARGIN times, still fit
/// the room. Err names the direction that keeps no window that settles.
fn keep(rig: &Rig, pct: u8, runs: &[Ran], room: f64) -> Result<u32, (Dir, String)> {
    let tail = SETTLE_MS + STEADY_MIN_MS;
    let span = RUNWAY_FRAC * rig.lim.runway as f64;
    let mut window = f64::INFINITY;
    let mut least: f64 = 0.0;
    for (dir, r) in DIRS.into_iter().zip(runs) {
        let m = margin(&rig.lim, dir);
        if !dead_host_fits(r.stop, m) {
            return Err((
                dir,
                format!(
                    "{dir} {pct}% stopped in {:.0} counts, over the {m:.0} between the soft \
                     limit and the stop: a rung its host abandoned would hit the stop",
                    r.stop
                ),
            ));
        }
        let v = r.v_ss.max(0.0);
        let mut w = runway::window_ms(&r.climb, v, span);
        if v > 0.0 {
            let travel = (room - runway::STOP_MARGIN * r.stop) / (1.0 + WINDOW_MARGIN);
            w = w.min(r.climb.ms - GAP_MS + (travel - r.climb.travel) / v);
        }
        let settles = r.climb.ms + tail;
        if w < settles {
            return Err((
                dir,
                format!(
                    "{dir} {pct}% needs {:.0} ms to settle and the runway leaves it {:.0}",
                    settles, w
                ),
            ));
        }
        window = window.min(w);
        least = least.max(settles);
    }
    if window < least {
        return Err((
            Dir::Fwd,
            format!("{pct}%: no one window settles both ways inside the runway"),
        ));
    }
    Ok(window.floor().min(WINDOW_MAX_MS) as u32)
}

/// Steady speed per direction: the affine fit over the kept rungs that
/// moved.
fn speed(rungs: &[GridRung]) -> Result<Speed> {
    let fit = |d: Dir| -> Result<Fit> {
        let pts: Vec<(f64, f64)> = rungs
            .iter()
            .filter(|r| r.get(d).v_ss > V_SS_MIN)
            .map(|r| (r.pct as f64, r.get(d).v_ss))
            .collect();
        let (a, b, r2) =
            affine_fit(&pts).ok_or_else(|| anyhow!("{d}: under 2 rungs moved, no v_ss fit"))?;
        Ok(Fit::rounded(a, b, r2))
    };
    let (fwd, rev) = (fit(Dir::Fwd)?, fit(Dir::Rev)?);
    Ok(Speed {
        duties: rungs
            .iter()
            .filter(|r| DIRS.iter().all(|&d| r.get(d).v_ss > V_SS_MIN))
            .map(|r| r.pct)
            .collect(),
        fwd,
        rev,
    })
}

/// A coast's drive: to the later of its grid rung's two goals, whole ms,
/// then ENTRY_MS on.
fn coast_drive_ms(r: &GridRung) -> u32 {
    r.fwd.t_goal_ms.max(r.rev.t_goal_ms).ceil() as u32 + ENTRY_MS
}

/// Climb the coast block's duties, each driven to its goal and on for
/// ENTRY_MS, then coasting, both ways, only while the travel predicted
/// from the rungs below fits the room; the top is the last rung whose
/// measured travel fit too. A duty the grid ladder did not keep has no
/// climb to drive by and ends the ladder.
fn coast_ladder<S: Servo>(
    s: &mut S,
    rig: &Rig,
    block: &CoastBlock,
    duties: &[u8],
    grid: &Grid,
    v_ss: &Speed,
) -> Result<Coast> {
    let room = coast_room(&rig.lim);
    println!(
        "[coast] ladder {duties:?}%: each driven to its goal and {ENTRY_MS} ms on, then {} ms \
         coast, room fwd {} rev {}, margin {COAST_MARGIN}",
        block.coast_ms, room.fwd, room.rev
    );
    let mut ladder = Vec::new();
    let mut refused = None;
    let mut top = None;
    for &pct in duties {
        let Some(rung) = grid.rungs.iter().find(|r| r.pct == pct) else {
            let why = format!("the grid ladder never kept {pct}%, so it has no climb to drive by");
            println!("  {pct}%: {why}, stopping");
            refused = Some(Refused { pct, why });
            break;
        };
        let drive_ms = coast_drive_ms(rung);
        let predicted = match next_rung(&ladder, pct, v_ss, &room) {
            Next::Run(p) => p,
            Next::Refuse(p) => {
                let why = format!(
                    "predicted travel fwd {:.0} rev {:.0} does not fit the room",
                    p.fwd, p.rev
                );
                println!("  {pct}%: {why}, stopping");
                refused = Some(Refused { pct, why });
                break;
            }
        };
        let chain = vec![
            Step::Drive(pct, Some(drive_ms)),
            Step::Coast(block.coast_ms),
        ];
        let rec = record(s, &cfg(chain, Dirs::Both, drive_ms, rig))?;
        let run = |d| {
            coast_run(
                &rec.segments,
                d,
                rig.lim.soft,
                rig.hz,
                predicted.map(|p| *p.get(d)),
            )
            .with_context(|| format!("{pct}% coast"))
        };
        let rung = CoastRung {
            pct,
            drive_ms,
            fwd: run(Dir::Fwd)?,
            rev: run(Dir::Rev)?,
        };
        for d in DIRS {
            let r = rung.get(d);
            let pred = r
                .predicted
                .map_or(String::new(), |p| format!(" (predicted {p})"));
            println!(
                "  {pct:3}@{drive_ms} {d}: entry {} counts/ms, lead {} + coast {} = {}{pred}, {} \
                 inside soft",
                r.entry, r.lead, r.coast, r.travel, r.peak_inside_soft
            );
        }
        let fits = rung_fits(&rung, &room);
        ladder.push(rung);
        if !fits {
            let why =
                format!("its measured travel does not fit the room with {COAST_MARGIN} to spare");
            println!("  {pct}%: {why}, stopping");
            refused = Some(Refused { pct, why });
            break;
        }
        top = Some(pct);
    }
    let top_pct = top.ok_or_else(|| {
        anyhow!(
            "the {}% coast already travels too far for the room, or never ran: no coast duty fits",
            duties[0]
        )
    })?;
    println!("  top duty {top_pct}%");
    Ok(Coast {
        coast_ms: block.coast_ms,
        margin: COAST_MARGIN,
        top_pct,
        room,
        refused,
        ladder,
    })
}

/// One direction's drive-then-coast chain of a ladder rung, measured. Travel
/// runs from the drive's first sample to the furthest one; the coast from
/// the coast's first sample, so the lead also holds the gap between the two
/// bursts, where the drive duty still applies. A drive that never reached
/// its goal entered the coast from a climb, not a speed.
fn coast_run(
    segs: &[Segment],
    dir: Dir,
    soft: [u16; 2],
    ticks_per_ms: f64,
    predicted: Option<f64>,
) -> Result<CoastRun> {
    let s = sign(dir);
    let mine: Vec<&Segment> = segs.iter().filter(|g| g.dir == s).collect();
    let [drive, coasting] = mine.as_slice() else {
        bail!("{dir} chain committed {} segments, not 2", mine.len());
    };
    if !drive
        .frames
        .iter()
        .any(|f| f.duty_q15 == Some(drive.cmd_duty_q15))
    {
        bail!("{dir} drive never reached its goal duty before the coast");
    }
    let dp = pos_ticks(&drive.frames);
    let cp = pos_ticks(&coasting.frames);
    let (Some(&(_, start)), Some(&(t_end, _)), Some(&(_, coast_start))) =
        (dp.first(), dp.last(), cp.first())
    else {
        bail!("{dir} chain has no pos samples");
    };
    let entry = slope_from(&dp, t_end as f64 - ENTRY_MS as f64 * ticks_per_ms)
        .map(|v| f64::from(s) * v * ticks_per_ms)
        .ok_or_else(|| {
            anyhow!(
                "{dir} drive: under {SETTLED_MIN_SAMPLES} pos samples in its last {ENTRY_MS} ms"
            )
        })?;
    if entry <= V_SS_MIN {
        bail!("{dir} drive entered the coast at {entry:.2} counts/ms: the shaft never got going");
    }
    // A coast still rolling at the end of its capture understates its
    // distance, and every prediction fit to it.
    let tail = settled_slope(&cp, SETTLED_FRAC)
        .map(|v| v * ticks_per_ms)
        .ok_or_else(|| anyhow!("{dir} coast: under {SETTLED_MIN_SAMPLES} settled pos samples"))?;
    if tail.abs() >= V_SS_MIN {
        bail!("{dir} coast still moving at {tail:.2} counts/ms at its end: lengthen coast_ms");
    }
    let ahead = |p: u16| i32::from(s) * i32::from(p);
    let peak = dp
        .iter()
        .chain(&cp)
        .map(|&(_, p)| ahead(p))
        .fold(i32::MIN, i32::max);
    let limit = if dir == Dir::Fwd { soft[1] } else { soft[0] };
    let inside = ahead(limit) - peak;
    if inside < 0 {
        bail!("{dir} coast crossed the soft limit by {} counts", -inside);
    }
    let travel = peak - ahead(start);
    let coast = (peak - ahead(coast_start)).max(0);
    let counts = |v: i32| to_counts(v as f64);
    Ok(CoastRun {
        predicted: predicted.map(to_counts),
        entry: round_to(entry, 3),
        lead: counts(travel - coast),
        coast: counts(coast),
        travel: counts(travel),
        peak_inside_soft: inside,
    })
}

/// The chains of `steps`: each drive with the steps it feeds.
pub(super) fn chains_of(steps: &[Step]) -> Vec<Vec<Step>> {
    let mut out: Vec<Vec<Step>> = Vec::new();
    for (k, &step) in steps.iter().enumerate() {
        if k > 0
            && feeds(steps, k - 1)
            && let Some(c) = out.last_mut()
        {
            c.push(step);
            continue;
        }
        out.push(vec![step]);
    }
    out
}

/// A chain as a schedule names it.
pub(super) fn chain_name(chain: &[Step]) -> String {
    chain
        .iter()
        .map(Step::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

/// Every chain of the session's step, reversal and ends blocks, both ways,
/// each only when its predicted excursion fits: the room to the soft limit
/// with COAST_MARGIN to spare, or for an ends chain, which drives into the
/// soft limit on purpose, a braked stop that fits beyond it.
fn chains<S: Servo>(
    s: &mut S,
    rig: &Rig,
    p: &Procedure,
    v_ss: &Speed,
    coast: &Coast,
    runways: &[Runway; 2],
) -> Result<(Vec<ChainRun>, Vec<ChainRefused>)> {
    let room = coast_room(&rig.lim);
    let blocks = [
        ("step", &p.block.step.steps),
        ("reversal", &p.block.reversal.steps),
        ("ends", &p.block.ends.steps),
    ];
    let coasts = runway::Envelope {
        supply: runway::Supply::TwoS,
        phys: (0, 0),
        fwd: runway::Line {
            slope: 0.0,
            intercept: 0.0,
        },
        rev: runway::Line {
            slope: 0.0,
            intercept: 0.0,
        },
        coast: coast
            .ladder
            .iter()
            .flat_map(|r| [r.fwd, r.rev])
            .map(|c| (c.entry, c.coast as f64))
            .collect(),
    };
    let (mut ran, mut refused) = (Vec::new(), Vec::new());
    for (block, steps) in blocks {
        for chain in chains_of(steps) {
            let name = chain_name(&chain);
            if ran
                .iter()
                .any(|c: &ChainRun| c.block == block && c.chain == name)
            {
                continue;
            }
            let predicted = DIRS
                .into_iter()
                .zip(runways)
                .map(|(d, rw)| excursion(&chain, v_ss.of(d), &coasts, rw))
                .collect::<Option<Vec<f64>>>();
            let why = match &predicted {
                None => Some("nothing the pilot measured predicts it".to_string()),
                Some(e) => judge_chain(block, &chain, e, &room, rig, v_ss, runways),
            };
            let predicted =
                predicted.map_or(0, |e| to_counts(e.iter().copied().fold(0.0, f64::max)));
            if let Some(why) = why {
                println!("[chain] {block} {name}: predicted {predicted}, {why}: refused");
                refused.push(ChainRefused {
                    block: block.into(),
                    chain: name,
                    predicted,
                    why,
                });
                continue;
            }
            let window = chain
                .iter()
                .find_map(|s| match s {
                    Step::Drive(_, Some(ms)) => Some(*ms),
                    _ => None,
                })
                .unwrap_or(WINDOW_MS);
            let rec = record(s, &cfg(chain.clone(), Dirs::Both, window, rig))?;
            let run = |d| chain_excursion(&rec.segments, d, &rig.lim);
            let (fwd, rev) = (run(Dir::Fwd)?, run(Dir::Rev)?);
            println!(
                "[chain] {block} {name}: predicted {predicted}, ran fwd {} ({} inside soft) rev {} \
                 ({} inside soft)",
                fwd.travel, fwd.peak_inside_soft, rev.travel, rev.peak_inside_soft
            );
            ran.push(ChainRun {
                block: block.into(),
                chain: name,
                predicted,
                fwd,
                rev,
            });
        }
    }
    Ok((ran, refused))
}

/// Why a chain predicted to carry the shaft `excursions` (fwd, rev) does
/// not run; None when it does.
fn judge_chain(
    block: &str,
    chain: &[Step],
    excursions: &[f64],
    room: &PerDir<u16>,
    rig: &Rig,
    v_ss: &Speed,
    runways: &[Runway; 2],
) -> Option<String> {
    let inside = DIRS
        .iter()
        .zip(excursions)
        .all(|(&d, &e)| fits_room(e, *room.get(d)));
    if inside {
        return None;
    }
    if block != "ends" {
        return Some("its excursion does not fit the room to the soft limit".into());
    }
    for (k, d) in DIRS.into_iter().enumerate() {
        let v = top_speed(chain, v_ss.of(d));
        let m = margin(&rig.lim, d);
        match runways[k].stop(v) {
            Some(stop) if dead_host_fits(stop, m) => {}
            Some(stop) => {
                return Some(format!(
                    "{d} it would stop in about {stop:.0} counts past the soft limit, over the \
                     {m:.0} to the stop"
                ));
            }
            None => return Some("no braked stop measured to judge it by".into()),
        }
    }
    None
}

/// The fastest steady speed a chain's drives reach, by the fit.
fn top_speed(chain: &[Step], fit: &Fit) -> f64 {
    chain
        .iter()
        .filter_map(|s| match s {
            Step::Drive(pct, _) => Some(*pct),
            Step::Then(pct, _) => Some(pct.unsigned_abs()),
            _ => None,
        })
        .map(|pct| fit.at(pct).max(0.0))
        .fold(0.0, f64::max)
}

/// A chain's predicted excursion, counts: every drive at its steady speed
/// for its window and the brake write after it, whichever way it points,
/// then how it ends - a coast by the coasts measured, a brake by the braked
/// stops - from the fastest speed it reached. None when nothing measured
/// predicts the end.
fn excursion(chain: &[Step], fit: &Fit, coasts: &runway::Envelope, rw: &Runway) -> Option<f64> {
    let mut travel = 0.0;
    for step in chain {
        let (pct, ms) = match *step {
            Step::Drive(pct, ms) => (pct, ms),
            Step::Then(pct, ms) => (pct.unsigned_abs(), ms),
            Step::Coast(_) | Step::Brake(_) => continue,
        };
        let ms = ms.unwrap_or(WINDOW_MS) as f64 + GAP_MS;
        travel += fit.at(pct).max(0.0) * ms;
    }
    let v = top_speed(chain, fit);
    let end = match chain.last() {
        Some(Step::Coast(_)) if !coasts.coast.is_empty() => coasts.coast(v),
        Some(Step::Coast(_)) => return None,
        _ => rw.stop(v)?,
    };
    Some(travel + end)
}

/// How far one direction of a chain carried the shaft from its first
/// sample, and how far inside the soft limit ahead the furthest sample
/// stayed. A chain that reached the stop beyond the soft limit ends the
/// pilot.
fn chain_excursion(segs: &[Segment], dir: Dir, lim: &Limits) -> Result<Excursion> {
    let s = sign(dir);
    let pos: Vec<u16> = segs
        .iter()
        .filter(|g| g.dir == s)
        .flat_map(|g| g.frames.iter().filter_map(|f| f.pos))
        .collect();
    let Some(&start) = pos.first() else {
        bail!("{dir} chain has no pos samples");
    };
    let ahead = |p: u16| i32::from(s) * i32::from(p);
    let peak = pos.iter().map(|&p| ahead(p)).fold(i32::MIN, i32::max);
    let (soft, stop) = match dir {
        Dir::Fwd => (lim.soft[1], lim.phys[1]),
        Dir::Rev => (lim.soft[0], lim.phys[0]),
    };
    if peak >= ahead(stop) {
        bail!("{dir} chain reached the stop at {stop}");
    }
    Ok(Excursion {
        travel: to_counts((peak - ahead(start)) as f64),
        peak_inside_soft: ahead(soft) - peak,
    })
}

/// One pilot recording, the pack read at rest before it.
fn record<S: Servo>(s: &mut S, cfg: &Cfg) -> Result<Recording> {
    let id = s.id();
    gate(s.client(), id)?;
    sweep::record(s, cfg, |_| Ok(()))
}

fn gate<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<()> {
    battery::before_drive(c, id)
}

/// A pilot recording at the session defaults: no baseline, every step with
/// its own window, the seeks at the jam check's duty.
fn cfg(steps: Vec<Step>, dirs: Dirs, window_ms: u32, rig: &Rig) -> Cfg {
    let lim = &rig.lim;
    Cfg {
        steps,
        dirs,
        decay: Decay::Slow,
        window_ms,
        rest_ms: REST_MS,
        baseline_ms: 0,
        seek_duty_pct: rig.proved.seek_pct(),
        seek_cap_pct: rig.proved.cap_pct(),
        settle_ms: SEEK_SETTLE_MS,
        stall: false,
        static_load: false,
        guard: (lim.guard[0], lim.guard[1]),
        stops: Some((lim.phys[0], lim.phys[1])),
        tel_mask: TEL_MASK,
        rung_tries: RUNG_TRIES,
    }
}

fn sign(d: Dir) -> i8 {
    match d {
        Dir::Fwd => 1,
        Dir::Rev => -1,
    }
}

fn sweep_dirs(d: Dir) -> Dirs {
    match d {
        Dir::Fwd => Dirs::Fwd,
        Dir::Rev => Dirs::Rev,
    }
}

fn one(segs: &[Segment]) -> Result<&Segment> {
    match segs {
        [s] => Ok(s),
        _ => bail!("expected one segment, recorded {}", segs.len()),
    }
}

fn pos_ticks(frames: &[TelFrame]) -> Vec<(u64, u16)> {
    frames
        .iter()
        .filter_map(|f| f.pos.map(|p| (f.tick, p)))
        .collect()
}

/// Soft, guard, centre and runway from the servo's soft and phys limits,
/// refusing a servo `osc cal` never set up or one too short to pilot.
fn limits(soft: (i32, i32), phys: (i32, i32)) -> Result<Limits> {
    if is_board_default(soft) {
        bail!(
            "soft limits {}..{} are the board default: run osc cal first",
            soft.0,
            soft.1
        );
    }
    let pot = |v: i32| {
        u16::try_from(v)
            .ok()
            .filter(|&v| v as i32 <= POT_MAX)
            .ok_or_else(|| anyhow!("limit {v} is outside the pot range: run osc cal first"))
    };
    let soft = [pot(soft.0)?, pot(soft.1)?];
    let phys = [pot(phys.0)?, pot(phys.1)?];
    if soft[0] >= soft[1] {
        bail!("soft limits {}..{} are inverted", soft[0], soft[1]);
    }
    let (lo, hi) = guards((soft[0], soft[1]));
    let guard = [lo, hi];
    let runway = guard[1] as i32 - guard[0] as i32;
    if runway < PILOT_MIN_RUNWAY as i32 {
        bail!("guard runway {runway} counts is under {PILOT_MIN_RUNWAY}: too short to pilot");
    }
    Ok(Limits {
        soft,
        phys,
        guard,
        center: soft[0] + (soft[1] - soft[0]) / 2,
        runway: runway as u16,
    })
}

/// Least-squares `y = a x + b` with r2; None under two distinct x.
fn affine_fit(pts: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    if pts.len() < 2 {
        return None;
    }
    let n = pts.len() as f64;
    let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
    let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
    let (sxx, sxy, syy) = pts.iter().fold((0.0, 0.0, 0.0), |(xx, xy, yy), &(x, y)| {
        let (dx, dy) = (x - mx, y - my);
        (xx + dx * dx, xy + dx * dy, yy + dy * dy)
    });
    if sxx == 0.0 {
        return None;
    }
    let a = sxy / sxx;
    let b = my - a * mx;
    let res: f64 = pts.iter().map(|&(x, y)| (y - (a * x + b)).powi(2)).sum();
    let r2 = if syy == 0.0 { 1.0 } else { 1.0 - res / syy };
    Some((a, b, r2))
}

/// Slope of pos over tick across the last `frac` of the tick span, counts
/// per tick; None under SETTLED_MIN_SAMPLES.
fn settled_slope(pts: &[(u64, u16)], frac: f64) -> Option<f64> {
    let (t0, t1) = (pts.first()?.0, pts.last()?.0);
    slope_from(pts, t1 as f64 - frac * (t1 - t0) as f64)
}

/// Slope of pos over tick from tick `from` on, counts per tick; None under
/// SETTLED_MIN_SAMPLES.
fn slope_from(pts: &[(u64, u16)], from: f64) -> Option<f64> {
    let t0 = pts.first()?.0;
    let tail: Vec<(f64, f64)> = pts
        .iter()
        .filter(|&&(t, _)| t as f64 >= from)
        .map(|&(t, p)| ((t - t0) as f64, p as f64))
        .collect();
    if tail.len() < SETTLED_MIN_SAMPLES {
        return None;
    }
    affine_fit(&tail).map(|(a, _, _)| a)
}

fn to_counts(x: f64) -> u16 {
    x.round().clamp(0.0, u16::MAX as f64) as u16
}

/// A block's duties in ladder order, refusing one that would drive past
/// full scale or, with `first_max`, open above it.
fn ladder_duties(duties: &[u8], first_max: Option<u8>) -> Result<Vec<u8>> {
    let mut v = duties.to_vec();
    v.sort_unstable();
    v.dedup();
    match (v.first(), v.last()) {
        (None, _) => bail!("no duties"),
        (Some(&lo), _) if first_max.is_some_and(|m| lo > m) => bail!(
            "opens at {lo}%: the ladder's first rung runs unpredicted, so it must open at or \
             under {}%",
            first_max.unwrap_or(0)
        ),
        (_, Some(&hi)) if hi > 100 => bail!("duty {hi}% is over full scale"),
        _ => Ok(v),
    }
}

/// Coast travel room per direction: from the far edge of the start band to
/// the soft limit ahead.
fn coast_room(lim: &Limits) -> PerDir<u16> {
    PerDir {
        fwd: lim.soft[1].saturating_sub(lim.guard[0].saturating_add(BAND_HALF)),
        rev: lim.guard[1]
            .saturating_sub(BAND_HALF)
            .saturating_sub(lim.soft[0]),
    }
}

fn fits_room(travel: f64, room: u16) -> bool {
    (1.0 + COAST_MARGIN) * travel <= room as f64
}

/// Entry speed of a `pct` drive from the (duty, entry) rungs so far: the
/// least-squares line through them, or with one rung v_ss(pct), which no
/// drive from rest outruns. Held between the last entry and v_ss(pct).
fn entry_at(pts: &[(f64, f64)], pct: u8, v_ss: &Fit) -> f64 {
    let last = pts.last().map_or(0.0, |p| p.1);
    let ceil = v_ss.at(pct).max(last);
    affine_fit(pts)
        .map_or(ceil, |(a, b, _)| a * pct as f64 + b)
        .clamp(last, ceil)
}

/// The (duty, travel) rungs so far carried to `pct`: in proportion to duty
/// from the last rung and along the line through the last two, whichever is
/// longer.
fn travel_trend(pts: &[(f64, f64)], pct: u8) -> f64 {
    let d = pct as f64;
    let Some(&(d1, t1)) = pts.last() else {
        return 0.0;
    };
    let prop = t1 * d / d1;
    match pts {
        [.., (d0, t0), _] => prop.max(t1 + (t1 - t0) / (d1 - d0) * (d - d1)),
        _ => prop,
    }
}

/// Predicted travel of a `pct` coast chain in `dir` after the rungs so far:
/// the last lead scaled by entry speed, plus the coast the fit over every
/// run gives at that speed, never under the measured travel's trend. Until
/// two rungs fit, or when the fit is unphysical, the coast is the last run's
/// distance/v^2 times v^2, which over-predicts: distance/v^2 falls with
/// speed.
fn predict(ladder: &[CoastRung], dir: Dir, pct: u8, v_ss: &Fit) -> f64 {
    let Some(last) = ladder.last().map(|r| r.get(dir)) else {
        return 0.0;
    };
    let by_duty = |f: fn(&CoastRun) -> f64| -> Vec<(f64, f64)> {
        ladder
            .iter()
            .map(|r| (r.pct as f64, f(r.get(dir))))
            .collect()
    };
    let v = entry_at(&by_duty(|r| r.entry), pct, v_ss);
    let scale = v / last.entry;
    let runs: Vec<(f64, f64)> = ladder
        .iter()
        .flat_map(|r| DIRS.map(|d| (r.get(d).entry, r.get(d).coast as f64)))
        .collect();
    let fit = if ladder.len() >= 2 {
        coast_fit(&runs)
    } else {
        None
    };
    let coast = fit.map_or(last.coast as f64 * scale * scale, |(a, b)| {
        a * v + b * v * v
    });
    let trend = travel_trend(&by_duty(|r| r.travel as f64), pct);
    (last.lead as f64 * scale + coast).max(trend)
}

/// The ladder's call on its next duty.
#[derive(Debug, PartialEq)]
enum Next {
    /// Run it, with the per-direction predictions (none on the first rung).
    Run(Option<PerDir<f64>>),
    /// Stop: a direction's prediction does not fit its room.
    Refuse(PerDir<f64>),
}

fn next_rung(ladder: &[CoastRung], pct: u8, v_ss: &Speed, room: &PerDir<u16>) -> Next {
    if ladder.is_empty() {
        return Next::Run(None);
    }
    let p = PerDir {
        fwd: predict(ladder, Dir::Fwd, pct, &v_ss.fwd),
        rev: predict(ladder, Dir::Rev, pct, &v_ss.rev),
    };
    if DIRS.iter().all(|&d| fits_room(*p.get(d), *room.get(d))) {
        Next::Run(Some(p))
    } else {
        Next::Refuse(p)
    }
}

/// Whether a rung's measured travel, margin included, fits the room both
/// ways: the campaign repeats the duty, so a run that only just fit is not
/// a top.
fn rung_fits(r: &CoastRung, room: &PerDir<u16>) -> bool {
    DIRS.iter()
        .all(|&d| fits_room(r.get(d).travel as f64, *room.get(d)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::pump::BurstStats;
    use crate::rig::servo::bench::{Bench, rail, torque};
    use osc_ident::limits::CLASS_R_MIN;
    use osc_ident::regs::control;

    fn bench_rig(front: &Front) -> Rig {
        Rig {
            lim: limits(front.lim.soft, front.lim.phys).unwrap(),
            proved: Proved {
                moved: 0.13,
                plan: front
                    .lim
                    .stall_plan(front.sc.r_vpc(CLASS_R_MIN), Some(0.13)),
            },
            hz: front.tick_hz as f64 / 1000.0,
        }
    }

    fn procedure() -> Procedure {
        Procedure::parse(include_str!("session.toml")).unwrap()
    }

    fn bench(supply: Supply) -> (Bench, Front) {
        let mut b = Bench::mg90(supply);
        let id = b.id();
        let front = front::read(&mut b.c, id, supply).unwrap();
        (b, front)
    }

    /// The whole pilot on the bench servo at the bench bus's timing, on 2S
    /// and on USB: the jam check, the grid ladder climbing both ways until
    /// the dead-host rule refuses the next duty, the coast ladder to the
    /// highest duty the grid kept, every chain of the procedure, the park,
    /// torque off and the permit clear; the envelope it writes reads back.
    #[test]
    fn pilot_completes_on_the_bench_fixture() {
        for (supply, top, coast_top) in [(Supply::TwoS, 55, 40), (Supply::Usb, 85, 80)] {
            let (mut b, front) = bench(supply);
            let (env, parked) = pilot(&mut b, &front, supply, &procedure(), 64);
            let env = env.unwrap();
            parked.unwrap();
            assert!(!b.servo.torque && !b.servo.permit_live(), "{supply:?}");
            assert!(
                (b.servo.pos - 2029.0).abs() <= 60.0,
                "parked at {}",
                b.servo.pos
            );

            let lim = &env.limits;
            assert_eq!(env.grid.top_pct, top, "{supply:?}");
            let kept: Vec<u8> = env.grid.rungs.iter().map(|r| r.pct).collect();
            let want: Vec<u8> = (1..=top / 5).map(|k| k * 5).collect();
            assert_eq!(kept, want);
            assert_eq!(env.windows_ms.keys().copied().collect::<Vec<_>>(), want);
            let refused = env.grid.refused.as_ref().unwrap();
            assert_eq!(refused.pct, top + 5);
            assert!(refused.why.contains("between the soft limit and the stop"));
            for r in &env.grid.rungs {
                for d in DIRS {
                    let run = r.get(d);
                    assert!(
                        (run.stop as f64) <= margin(lim, d),
                        "{}% {d} stop {}",
                        r.pct,
                        run.stop
                    );
                    assert!(r.window_ms as f64 >= run.t_goal_ms + SETTLE_MS + STEADY_MIN_MS);
                    // the window's travel, the brake write after it and
                    // the margined stop fit the room
                    let travel = run.travel as f64
                        + run.v_ss.max(0.0) * (r.window_ms as f64 + GAP_MS - run.t_goal_ms);
                    let room = (lim.runway - runway::START_BAND) as f64;
                    assert!(
                        1.1 * travel + runway::STOP_MARGIN * run.stop as f64 <= room + 1.0,
                        "{}% {d}",
                        r.pct
                    );
                }
            }
            // governed from 20% on 2S: the goal comes later as the duty rises
            let goals: Vec<f64> = env.grid.rungs.iter().map(|r| r.fwd.t_goal_ms).collect();
            assert!(goals.windows(2).all(|w| w[1] >= w[0]), "{goals:?}");

            assert_eq!(env.coast.top_pct, coast_top);
            for r in &env.coast.ladder {
                let grid = env.grid.rungs.iter().find(|g| g.pct == r.pct).unwrap();
                assert_eq!(r.drive_ms, coast_drive_ms(grid));
            }
            assert!(env.refused_chains.is_empty(), "{:?}", env.refused_chains);
            assert_eq!(env.chains.len(), 9);
            for c in &env.chains {
                for d in DIRS {
                    let e = c.get(d);
                    assert!(
                        e.travel as f64 <= c.predicted as f64 + 1.0,
                        "{} {d}",
                        c.chain
                    );
                    assert!(e.peak_inside_soft > 0, "{} {d}", c.chain);
                }
            }
            assert!(env.v_ss.fwd.r2 > 0.99 && env.v_ss.rev.r2 > 0.99);

            let dir = std::env::temp_dir().join(format!(
                "osc-pilot-{}-{}",
                supply.as_str(),
                std::process::id()
            ));
            std::fs::create_dir_all(&dir).unwrap();
            env.save(&dir).unwrap();
            let back = Envelope::load(&dir).unwrap();
            std::fs::remove_dir_all(&dir).unwrap();
            assert_eq!(back, env);
        }
    }

    /// The window a campaign rung keeps: its measured climb, then the time
    /// at v_ss to cross what the climb left of 0.8 of the runway; cut where
    /// its travel, the brake write and the margined stop would not fit the
    /// room; never under the settle and the least steady time.
    #[test]
    fn window_is_the_climb_plus_the_tail() {
        let (_, front) = bench(Supply::TwoS);
        let rig = bench_rig(&front);
        let ran = |t_goal: f64, travel: f64, v_ss: f64, stop: f64| Ran {
            ms: 200,
            climb: Climb {
                ms: t_goal,
                travel,
                accel: 2.0 * travel / (t_goal * t_goal) * 1000.0,
            },
            v_ss,
            stop,
        };
        let room = 2919.0;
        // 40% on the bench servo: 71.8 + (0.8 x 2994 - 236) / 7.46
        let r40 = ran(71.8, 236.0, 7.46, 113.0);
        assert_eq!(
            keep(&rig, 40, &[r40, ran(71.8, 236.0, 7.46, 113.0)], room),
            Ok(361)
        );
        // 55%: the tail to 0.8 of the runway, 290 ms, would not fit the room
        // with its stop; the fit cuts it to 286
        let r55 = || ran(130.8, 707.0, 10.58, 171.0);
        let w0 = runway::window_ms(&r55().climb, 10.58, 0.8 * 2994.0);
        assert_eq!(w0.floor(), 290.0);
        assert_eq!(keep(&rig, 55, &[r55(), r55()], room), Ok(286));
        // the shorter direction sets it
        let slow = ran(130.8, 707.0, 10.0, 171.0);
        assert_eq!(keep(&rig, 55, &[r55(), slow], room), Ok(286));
        // a duty that does not move the shaft takes the longest window
        assert_eq!(
            keep(
                &rig,
                5,
                &[ran(0.0, 0.0, 0.0, 0.0), ran(0.0, 0.0, 0.0, 0.0)],
                room
            ),
            Ok(1500)
        );
        // a climb that runs past what the room allows keeps no window
        let long = ran(400.0, 2200.0, 12.0, 150.0);
        let (dir, why) = keep(&rig, 70, &[long, r55()], room).unwrap_err();
        assert_eq!(dir, Dir::Fwd);
        assert_eq!(
            why,
            "fwd 70% needs 500 ms to settle and the runway leaves it 414"
        );
        // and a measured stop over the margin keeps none either
        let (dir, why) =
            keep(&rig, 60, &[r55(), ran(156.6, 971.0, 11.62, 206.0)], room).unwrap_err();
        assert_eq!(dir, Dir::Rev);
        assert_eq!(
            why,
            "rev 60% stopped in 206 counts, over the 200 between the soft limit and the stop: a \
             rung its host abandoned would hit the stop"
        );
    }

    /// Frames at 20 ticks/ms: the climb under the goal at one speed, the
    /// first 40 ms at the goal still speeding up, then steady at 0.3
    /// counts/tick. The fit reads only the steady samples.
    #[test]
    fn v_ss_is_fit_over_samples_at_the_goal_only() {
        let goal = 9830;
        let frames = |n: u64, dip: Option<u64>| -> Vec<TelFrame> {
            let mut pos = 600.0;
            (0..n)
                .map(|t| {
                    let (duty, v) = match t {
                        _ if dip == Some(t) => (goal - 128, 0.3),
                        0..400 => (4484 + (t as i16 % 40), 0.1),
                        400..1200 => (goal, 0.2 + 0.1 * (t - 400) as f64 / 800.0),
                        _ => (goal, 0.3),
                    };
                    pos += v;
                    TelFrame {
                        tick: t,
                        pos: Some(pos.round() as u16),
                        duty_q15: Some(duty),
                        ..TelFrame::default()
                    }
                })
                .collect()
        };
        let v = settled_v(&frames(4000, None), goal, 20.0).unwrap();
        assert!((v - 6.0).abs() < 0.01, "{v}");
        // down the pot the speed counts along the goal
        let down: Vec<TelFrame> = frames(4000, None)
            .into_iter()
            .map(|f| TelFrame {
                pos: f.pos.map(|p| 4000 - p),
                duty_q15: f.duty_q15.map(|d| -d),
                ..f
            })
            .collect();
        let v = settled_v(&down, -goal, 20.0).unwrap();
        assert!((v - 6.0).abs() < 0.01, "{v}");
        // the whole stream would read the climb and the spin-up in
        let all = pos_ticks(&frames(4000, None));
        let whole = slope_from(&all, 0.0).unwrap() * 20.0;
        assert!((whole - 6.0).abs() > 0.2, "{whole}");
        // a dip under the goal restarts the settle from after it
        let dipped = frames(4000, Some(1500));
        let range = runway::settled_ticks(&dipped, goal, 20_000.0).unwrap();
        assert_eq!(*range.start(), 1501 + 800);
        let v = settled_v(&dipped, goal, 20.0).unwrap();
        assert!((v - 6.0).abs() < 0.02, "{v}");
        // under 60 ms at the goal past the settle measures nothing
        assert_eq!(settled_v(&frames(2400, None), goal, 20.0), None);
        assert!(settled_v(&frames(3200, None), goal, 20.0).is_some());
        // a goal lost at the end is no steady speed
        assert_eq!(settled_v(&frames(4000, Some(3999)), goal, 20.0), None);
    }

    /// The runway after the bench servo's 50 and 55% rungs: 60% fits the
    /// room both ways, but its braked stop from 11.6 counts/ms, 206 by the
    /// 55% stop scaled by speed squared, fits the 223 counts beyond the
    /// high soft limit and not the 200 beyond the low one: the reverse rung
    /// never runs. On the bench fixture the ladder ends there.
    #[test]
    fn ladder_ends_at_the_first_rung_that_does_not_fit() {
        let (mut b, front) = bench(Supply::TwoS);
        let rig = bench_rig(&front);
        let mut rw = Runway::new((532, 3526));
        rw.ran(0.50, 9.54);
        rw.ran(0.55, 10.58);
        rw.climbed(80.0);
        rw.stopped(10.58, 171.0);
        let w = first_window(&rw, &rig, Dir::Fwd, 60, SETTLE_MS + STEADY_MIN_MS).unwrap();
        assert_eq!(w, 282);
        assert_eq!(
            first_window(&rw, &rig, Dir::Rev, 60, SETTLE_MS + STEADY_MIN_MS),
            Err(
                "rev 60% would stop in about 206 counts, over the 200 between the soft limit and \
                 the stop: a rung its host abandoned would hit the stop"
                    .into()
            )
        );
        // a runway too short for the rung
        let mut short = Runway::new((532, 2200));
        short.ran(0.50, 9.54);
        short.ran(0.55, 10.58);
        short.climbed(80.0);
        short.stopped(10.58, 171.0);
        let why = sized(&short, &rig, Dir::Fwd, 60, 282).unwrap_err();
        assert!(why.starts_with("fwd 60% in 282 ms needs "), "{why}");
        assert!(
            why.ends_with("counts of runway, over the 1593 there is"),
            "{why}"
        );
        // nothing measured: a first rung opens short, and only low
        let empty = Runway::new((532, 3526));
        assert_eq!(
            first_window(&empty, &rig, Dir::Fwd, 30, 100.0),
            Ok(PILOT_W0)
        );
        assert_eq!(
            first_window(&empty, &rig, Dir::Fwd, 35, 100.0),
            Err("fwd 35%: no rung under it moved the shaft, so nothing sizes it".into())
        );

        let p = procedure();
        let duties = ladder_duties(&p.block.grid.duties, None).unwrap();
        let ladder = grid_ladder(&mut b, &rig, &duties).unwrap();
        let kept: Vec<u8> = ladder.rungs.iter().map(|r| r.pct).collect();
        assert_eq!(kept, (1..=11u8).map(|k| k * 5).collect::<Vec<_>>());
        let refused = ladder.refused.unwrap();
        assert_eq!((refused.pct, refused.dir), (60, Dir::Rev));
        assert!(!b.servo.torque);
    }

    /// A coast drives to its grid rung's later goal, whole ms, and 20 ms
    /// on; a drive that never reached its goal entered the coast from a
    /// climb and is refused.
    #[test]
    fn a_coast_drive_runs_to_its_goal() {
        let run = |t_goal_ms| RungRun {
            ran_ms: 170,
            t_goal_ms,
            travel: 80,
            v_ss: 5.37,
            stop: 76,
        };
        let rung = GridRung {
            pct: 30,
            window_ms: 471,
            fwd: run(40.5),
            rev: run(40.2),
        };
        assert_eq!(coast_drive_ms(&rung), 61);

        let soft = [432, 3626];
        let [mut d, c] = coast_chain(1, 600);
        d.cmd_duty_q15 = 9830;
        let err = coast_run(&[d, c], Dir::Fwd, soft, 1.0, None)
            .unwrap_err()
            .to_string();
        assert_eq!(
            err,
            "fwd drive never reached its goal duty before the coast"
        );
        let [mut d, c] = coast_chain(1, 600);
        d.cmd_duty_q15 = 9830;
        for f in &mut d.frames[40..] {
            f.duty_q15 = Some(9830);
        }
        assert!(coast_run(&[d, c], Dir::Fwd, soft, 1.0, None).is_ok());
    }

    /// The pilot reads the pack at rest before anything moves, in its
    /// front, and again before every recording: a pack that sags under its
    /// floor between two rungs stops the pilot before the next one drives.
    #[test]
    fn pilot_gates_the_battery() {
        let flat = "the pack reads 6.99 V at rest, under its floor of 7.00 V (3.50 V a cell): \
                    charge it before anything drives";
        let mut b = Bench::mg90(Supply::TwoS);
        let id = b.id();
        rail(&mut b.c, 3351);
        let e = front::read(&mut b.c, id, Supply::TwoS).unwrap_err();
        assert_eq!(e.to_string(), flat);
        assert_eq!(torque(&mut b.c, id), 0);

        let (mut b, front) = bench(Supply::TwoS);
        let rig = bench_rig(&front);
        rail(&mut b.c, 3351);
        let t = b.servo.t_ms;
        let step = Step::Drive(20, Some(150));
        let e = record(&mut b, &cfg(vec![step], Dirs::Fwd, 150, &rig))
            .err()
            .unwrap();
        assert_eq!(e.to_string(), flat);
        assert!(!b.servo.torque);
        assert_eq!((b.servo.t_ms, b.servo.tel_mask), (t, 0), "nothing moved");
        let id = b.id();
        let armed = b.c.read(id, control::TEL_COUNT.addr, 2).unwrap();
        assert_eq!(armed, [0, 0], "no burst armed");
    }

    #[test]
    fn guard_band_clears_the_soft_clamp() {
        const { assert!(osc_ident::limits::GUARD_INSET > BAND_HALF) };
    }

    #[test]
    fn limits_from_a_calibrated_servo() {
        let lim = limits((432, 3626), (232, 3849)).unwrap();
        assert_eq!(lim.soft, [432, 3626]);
        assert_eq!(lim.phys, [232, 3849]);
        assert_eq!(lim.guard, [532, 3526]);
        assert_eq!(lim.center, 2029);
        assert_eq!(lim.runway, 2994);
        assert_eq!(
            (margin(&lim, Dir::Fwd), margin(&lim, Dir::Rev)),
            (223.0, 200.0)
        );
    }

    #[test]
    fn uncalibrated_or_short_servos_are_refused() {
        let err = |soft| limits(soft, (0, 4095)).unwrap_err().to_string();
        assert!(err((0, 4095)).contains("run osc cal first"));
        assert!(err((-4096, 8191)).contains("run osc cal first"));
        assert!(err((-10, 3000)).contains("outside the pot range"));
        assert!(err((3000, 1000)).contains("inverted"));
        // runway = span - 2 x GUARD_INSET
        assert!(limits((1000, 2699), (0, 4095)).is_err());
        assert!(limits((1000, 2700), (0, 4095)).is_ok());
    }

    const MG90: Fit = Fit {
        slope: 0.2275,
        intercept: -0.845,
        r2: 0.998,
    };

    #[test]
    fn affine_fit_recovers_a_planted_line() {
        let pts: Vec<(f64, f64)> = [10u8, 20, 30, 40, 50]
            .iter()
            .map(|&d| (d as f64, MG90.at(d)))
            .collect();
        let (a, b, r2) = affine_fit(&pts).unwrap();
        assert!((a - 0.2275).abs() < 1e-12, "{a}");
        assert!((b + 0.845).abs() < 1e-12, "{b}");
        assert!((r2 - 1.0).abs() < 1e-12, "{r2}");
        assert!(affine_fit(&pts[..1]).is_none());
        assert!(affine_fit(&[(10.0, 1.0), (10.0, 2.0)]).is_none());
    }

    #[test]
    fn settled_slope_skips_a_30_ms_spinup() {
        // 150 ms at 20 ticks/ms: quadratic to 0.1 counts/tick over 600 ticks,
        // then a ramp at 0.1
        let pts: Vec<(u64, u16)> = (0..3000u64)
            .map(|t| {
                let p = if t < 600 {
                    0.1 * (t * t) as f64 / 1200.0
                } else {
                    30.0 + 0.1 * (t - 600) as f64
                };
                (1_000_000 + t, (100.0 + p).round() as u16)
            })
            .collect();
        let v = settled_slope(&pts, SETTLED_FRAC).unwrap();
        assert!((v - 0.1).abs() < 1e-3, "{v}");
        // the whole window would read the spin-up in
        let all = settled_slope(&pts, 1.0).unwrap();
        assert!((all - 0.1).abs() > 1e-3, "{all}");
        assert!(settled_slope(&pts[..40], SETTLED_FRAC).is_none());
        assert!(settled_slope(&[], SETTLED_FRAC).is_none());
    }

    /// mg90-a 2S spindown (bridge/spindown, five captures) per coast duty:
    /// entry counts/ms, lead and coast counts, taken alike both ways.
    const MG90_COAST: [(u8, f64, u16, u16); 5] = [
        (20, 3.16, 235, 130),
        (30, 5.62, 377, 316),
        (40, 7.55, 514, 536),
        (60, 11.67, 760, 979),
        (80, 13.15, 938, 1333),
    ];

    /// The committed mg90-a 2S envelope's v_ss fits.
    fn mg90_2s_v_ss() -> Speed {
        Speed {
            duties: vec![10, 20, 30, 40, 50],
            fwd: Fit::rounded(0.2047, -0.721, 0.9968),
            rev: Fit::rounded(0.2079, -0.831, 0.9983),
        }
    }

    fn run(entry: f64, lead: u16, coast: u16) -> CoastRun {
        CoastRun {
            predicted: None,
            entry,
            lead,
            coast,
            travel: lead + coast,
            peak_inside_soft: 0,
        }
    }

    fn rung(pct: u8, fwd: CoastRun, rev: CoastRun) -> CoastRung {
        CoastRung {
            pct,
            drive_ms: 80,
            fwd,
            rev,
        }
    }

    #[test]
    fn ladder_duties_climb_from_a_low_rung() {
        let first = Some(FIRST_MAX_PCT);
        assert_eq!(
            ladder_duties(&[80, 20, 40, 20], first).unwrap(),
            [20, 40, 80]
        );
        assert_eq!(ladder_duties(&[30], first).unwrap(), [30]);
        assert_eq!(ladder_duties(&[60, 40], None).unwrap(), [40, 60]);
        let err = |d: &[u8]| ladder_duties(d, first).unwrap_err().to_string();
        assert!(err(&[]).contains("no duties"));
        assert!(err(&[40, 60]).contains("opens at 40%"));
        assert!(err(&[20, 101]).contains("over full scale"));
    }

    #[test]
    fn coast_room_runs_from_the_far_band_edge_to_soft() {
        let lim = limits((432, 3626), (232, 3849)).unwrap();
        // 3626 - (532 + 75), (3526 - 75) - 432
        assert_eq!(
            coast_room(&lim),
            PerDir {
                fwd: 3019,
                rev: 3019
            }
        );
        assert!(fits_room(2744.0, 3019));
        assert!(!fits_room(2745.0, 3019));
    }

    #[test]
    fn coast_fit_recovers_the_mg90_law_and_refuses_unphysical_ones() {
        let pts: Vec<(f64, f64)> = MG90_COAST
            .iter()
            .map(|&(_, v, _, d)| (v, d as f64))
            .collect();
        let (a, b) = coast_fit(&pts).unwrap();
        assert!(
            (a - 24.94).abs() < 0.01 && (b - 5.557).abs() < 0.001,
            "{a} {b}"
        );
        // 80% from its measured entry: 1289, 3% short of the 1333 it coasted
        assert_eq!(to_counts(a * 13.15 + b * 13.15 * 13.15), 1289);

        let planted: Vec<(f64, f64)> = [2.0, 5.0, 9.0]
            .iter()
            .map(|&v| (v, 3.0 * v + 7.0 * v * v))
            .collect();
        let (a, b) = coast_fit(&planted).unwrap();
        assert!((a - 3.0).abs() < 1e-9 && (b - 7.0).abs() < 1e-9, "{a} {b}");
        // one speed cannot separate the terms
        assert_eq!(coast_fit(&[(5.0, 300.0), (5.0, 300.0)]), None);
        assert_eq!(coast_fit(&[]), None);
        // coasting shorter than linear in speed: negative v^2 term
        assert_eq!(coast_fit(&[(2.0, 100.0), (8.0, 200.0)]), None);
    }

    #[test]
    fn entry_speed_follows_the_rungs_under_the_v_ss_ceiling() {
        let v_ss = mg90_2s_v_ss().rev;
        // one rung: v_ss(30) = 5.406 bounds it
        assert!((entry_at(&[(20.0, 3.16)], 30, &v_ss) - 5.406).abs() < 1e-9);
        // two or more: the least-squares line
        let v = entry_at(&[(10.0, 1.0), (20.0, 3.0), (30.0, 5.0)], 40, &v_ss);
        assert!((v - 7.0).abs() < 1e-9, "{v}");
        // never over v_ss(pct) = 7.485, the speed a drive from rest spins up
        // toward
        let v = entry_at(&[(20.0, 3.16), (30.0, 5.62)], 40, &v_ss);
        assert!((v - 7.485).abs() < 1e-9, "{v}");
        // never under the last entry, even when the line falls
        assert_eq!(entry_at(&[(20.0, 6.0), (30.0, 5.0)], 40, &v_ss), 5.0);
    }

    #[test]
    fn travel_trend_extends_the_measured_travel() {
        assert_eq!(travel_trend(&[], 40), 0.0);
        assert_eq!(travel_trend(&[(20.0, 365.0)], 40), 730.0);
        // the line through the last two, 1050 + 20 x 35.7, beats proportion
        let t = travel_trend(&[(20.0, 365.0), (30.0, 693.0), (40.0, 1050.0)], 60);
        assert!((t - 1764.0).abs() < 1e-9, "{t}");
        // proportion beats a flattening line
        assert_eq!(
            travel_trend(&[(40.0, 1000.0), (60.0, 1100.0)], 80),
            1100.0 * 4.0 / 3.0
        );
    }

    #[test]
    fn mg90_ladder_reaches_80_on_the_measured_coasts() {
        let lim = limits((432, 3626), (232, 3849)).unwrap();
        let (room, v_ss) = (coast_room(&lim), mg90_2s_v_ss());
        let mut ladder = Vec::new();
        for &(pct, entry, lead, coast) in &MG90_COAST {
            let Next::Run(predicted) = next_rung(&ladder, pct, &v_ss, &room) else {
                panic!("{pct}% refused");
            };
            let measured = lead + coast;
            // Every prediction lands within 3% short and 20% long of the
            // travel it ran; 80% predicts 2649 against 3019 of room.
            for d in predicted.iter().flat_map(|p| DIRS.map(|d| *p.get(d))) {
                let r = d / measured as f64;
                assert!((0.97..1.2).contains(&r), "{pct}%: {d:.0} for {measured}");
            }
            let r = run(entry, lead, coast);
            let next = rung(pct, r, r);
            assert!(rung_fits(&next, &room), "{pct}%");
            ladder.push(next);
        }
        // a 100% rung would predict 3352, over the room
        assert!(matches!(
            next_rung(&ladder, 100, &v_ss, &room),
            Next::Refuse(p) if to_counts(p.fwd) == 3352
        ));
    }

    #[test]
    fn the_worst_direction_stops_the_ladder() {
        let v_ss = mg90_2s_v_ss();
        let (a, b) = (run(3.16, 235, 130), run(5.62, 377, 316));
        let ladder = [rung(20, a, a), rung(30, b, b)];
        let room = PerDir {
            fwd: 3019,
            rev: 3019,
        };
        let Next::Run(Some(p)) = next_rung(&ladder, 40, &v_ss, &room) else {
            panic!("40% refused");
        };
        // just under what rev predicts: rev alone refuses
        let tight = PerDir {
            fwd: 3019,
            rev: (p.rev * (1.0 + COAST_MARGIN)) as u16,
        };
        assert_eq!(next_rung(&ladder, 40, &v_ss, &tight), Next::Refuse(p));
        // a rung whose one direction ran long fails its measured bar
        let long = rung(40, run(7.55, 514, 536), run(7.55, 514, 2300));
        assert!(!rung_fits(&long, &room));
        assert!(rung_fits(
            &long,
            &PerDir {
                fwd: 3019,
                rev: 3500
            }
        ));
        // no rungs yet: the first runs unpredicted
        assert_eq!(next_rung(&[], 20, &v_ss, &room), Next::Run(None));
    }

    /// Chains split where a drive starts: the steps a drive feeds stay with
    /// it; a chain's excursion adds every leg at its steady speed and the
    /// way it ends.
    #[test]
    fn chains_split_at_every_drive_and_predict_their_excursion() {
        let p = procedure();
        let names: Vec<String> = chains_of(&p.block.reversal.steps)
            .iter()
            .map(|c| chain_name(c))
            .collect();
        assert_eq!(
            names,
            [
                "20@60,then:-20@60,then:20@60,brake:200",
                "40@60,then:-40@60,then:40@60,brake:200"
            ]
        );
        assert_eq!(chains_of(&p.block.step.steps).len(), 5);
        assert_eq!(chains_of(&p.block.ends.steps).len(), 2);

        let fit = mg90_2s_v_ss().fwd;
        let mut rw = Runway::new((532, 3526));
        rw.ran(0.2, 3.3);
        rw.stopped(3.3, 40.0);
        let none = runway::Envelope {
            coast: Vec::new(),
            ..envelope_with_coasts()
        };
        let chain = &chains_of(&p.block.reversal.steps)[0];
        // three 60 ms legs at v(20) = 3.373, each with the 10 ms gap, then
        // the braked stop from that speed
        let e = excursion(chain, &fit, &none, &rw).unwrap();
        let v = fit.at(20);
        let stop = rw.stop(v).unwrap();
        assert!((e - (3.0 * v * 70.0 + stop)).abs() < 1e-9, "{e}");
        // a coast ending needs a measured coast
        let step = &chains_of(&p.block.step.steps)[0];
        assert_eq!(excursion(step, &fit, &none, &rw), None);
        let coasts = envelope_with_coasts();
        let e = excursion(step, &fit, &coasts, &rw).unwrap();
        let v = fit.at(30);
        assert!((e - (v * 30.0 + coasts.coast(v))).abs() < 1e-9, "{e}");
    }

    fn envelope_with_coasts() -> runway::Envelope {
        let line = runway::Line {
            slope: 0.0,
            intercept: 0.0,
        };
        runway::Envelope {
            supply: runway::Supply::TwoS,
            phys: (232, 3849),
            fwd: line,
            rev: line,
            coast: MG90_COAST
                .iter()
                .map(|&(_, v, _, d)| (v, d as f64))
                .collect(),
        }
    }

    fn seg(dir: i8, pos: &[u16]) -> Segment {
        let frames = pos
            .iter()
            .enumerate()
            .map(|(i, &p)| TelFrame {
                tick: i as u64,
                window_valid: true,
                pos: Some(p),
                duty_q15: Some(0),
                ..TelFrame::default()
            })
            .collect();
        Segment {
            seg: 1,
            dir,
            cmd_duty_q15: 0,
            frames,
            stats: BurstStats {
                frames: 0,
                samples: 0,
                holes: 0,
                garble: 0,
            },
        }
    }

    /// A drive at 5 counts/tick from `start` for 60 ticks, 12 counts on in
    /// the gap, then a coast slowing by 1 count/tick per tick to rest, held
    /// for 60 ticks.
    fn coast_chain(dir: i8, start: u16) -> [Segment; 2] {
        let at = |x: i32| (start as i32 + dir as i32 * x) as u16;
        let drive: Vec<u16> = (0..60).map(|t| at(5 * t)).collect();
        let mut x = 5 * 59 + 12;
        let mut coast = vec![at(x)];
        for v in (1..=4).rev() {
            x += v;
            coast.push(at(x));
        }
        coast.extend([at(x); 60]);
        [seg(dir, &drive), seg(dir, &coast)]
    }

    #[test]
    fn coast_run_measures_each_direction() {
        let soft = [432, 3626];
        let [fd, fc] = coast_chain(1, 600);
        let [rd, rc] = coast_chain(-1, 3450);
        let segs = [fd, fc, rd, rc];
        // lead 295 of drive + 12 in the gap, coast 10, peak 600 + 317
        let fwd = coast_run(&segs, Dir::Fwd, soft, 1.0, Some(300.4)).unwrap();
        assert_eq!(
            fwd,
            CoastRun {
                predicted: Some(300),
                entry: 5.0,
                lead: 307,
                coast: 10,
                travel: 317,
                peak_inside_soft: 3626 - 917,
            }
        );
        let rev = coast_run(&segs, Dir::Rev, soft, 1.0, None).unwrap();
        assert_eq!((rev.entry, rev.lead, rev.coast), (5.0, 307, 10));
        assert_eq!(rev.peak_inside_soft, 3450 - 317 - 432);
    }

    #[test]
    fn coast_run_refuses_a_crossing_or_a_rolling_coast() {
        let soft = [432, 3626];
        let err = |segs: &[Segment]| {
            coast_run(segs, Dir::Fwd, soft, 1.0, None)
                .unwrap_err()
                .to_string()
        };
        assert!(err(&coast_chain(1, 3400)).contains("crossed the soft limit by 91"));
        let [d, _] = coast_chain(1, 600);
        let rolling: Vec<u16> = (0..60).map(|t| 1000 + 2 * t).collect();
        assert!(err(&[d, seg(1, &rolling)]).contains("still moving"));
        let [d, _] = coast_chain(1, 600);
        assert!(err(&[d]).contains("1 segments"));
        let [_, c] = coast_chain(1, 600);
        let stalled = seg(1, &[600; 60]);
        assert!(err(&[stalled, c]).contains("never got going"));
    }

    /// A chain's excursion from its first sample, the peak's distance
    /// inside the soft limit ahead; reaching the stop ends the pilot.
    #[test]
    fn chain_excursion_is_the_peak_from_the_start() {
        let lim = limits((432, 3626), (232, 3849)).unwrap();
        let [d, c] = coast_chain(1, 600);
        let [rd, rc] = coast_chain(-1, 3450);
        let segs = [d, c, rd, rc];
        let e = chain_excursion(&segs, Dir::Fwd, &lim).unwrap();
        assert_eq!(
            e,
            Excursion {
                travel: 317,
                peak_inside_soft: 3626 - 917,
            }
        );
        let e = chain_excursion(&segs, Dir::Rev, &lim).unwrap();
        assert_eq!(e.travel, 317);
        let [d, c] = coast_chain(1, 3600);
        let err = chain_excursion(&[d, c], Dir::Fwd, &lim).unwrap_err();
        assert_eq!(err.to_string(), "fwd chain reached the stop at 3849");
    }
}
