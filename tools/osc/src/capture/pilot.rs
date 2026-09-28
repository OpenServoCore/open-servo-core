//! `osc capture pilot`: measure the servo's steady-state speed per duty and
//! write the envelope the campaign sizes its windows by. A sweep rung is one
//! unpolled TEL burst, so its window is the only bound on how far the shaft
//! travels. Every bound here comes from the servo's own soft/phys limits
//! (written by `osc cal`); the derived windows and coast duty are run on the
//! servo before the envelope is saved.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::frame::TelFrame;
use osc_ident::regs::{calib, config};

use super::envelope::{
    Coast, CoastRun, CoastRung, Dir, Envelope, Fit, Limits, PerDir, Refused, Speed, Verified,
    civil_date, round_to,
};
use super::procs::{CoastBlock, Procedure};
use super::{REST_MS, RUNG_TRIES, SEEK_CAP_PCT, SEEK_PCT, SETTLE_MS, Supply, TEL_MASK};
use crate::rig::park::park;
use crate::rig::pump::{self, read_i32};
use crate::rig::snapshot::read_u16;
use crate::sweep::{self, BAND_HALF, Cfg, Decay, Dirs, Segment, Step};

/// `osc capture pilot` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>`.
    #[arg(long)]
    servo: String,
    /// Supply the servo runs on.
    #[arg(long, value_enum)]
    supply: Supply,
    /// Dataset root; defaults to notebooks/telemetry in this git checkout.
    #[arg(long)]
    root: Option<PathBuf>,
}

/// Full scale of the 12-bit pot ADC; soft limits spanning it are the board
/// default, not a calibration.
const POT_MAX: i32 = 4095;
/// Soft-to-guard inset. The seek band spans BAND_HALF either side of the
/// guard, so the guard must sit more than BAND_HALF inside soft or the band
/// reaches the firmware soft-limit clamp and the seek fights it; 25 counts
/// clear of it.
const GUARD_INSET: u16 = BAND_HALF + 25;
/// Shortest guard-to-guard runway worth sizing windows against: PILOT_W0 at
/// the fastest 10% seen already crosses 255 counts of it.
const PILOT_MIN_RUNWAY: u16 = 1500;
/// Soft-to-phys distance below which a runaway can still reach the stop: the
/// firmware brakes at the soft limit and stops a few hundred counts past it.
const BRAKE_MARGIN_MIN: i32 = 300;
/// Speed rungs, per direction.
const PILOT_DUTIES: [u8; 5] = [10, 20, 30, 40, 50];
/// First rung's window: 1.7 counts/ms, the fastest 10% seen (2S), crosses
/// 255 counts of a ~3000 runway in it.
const PILOT_W0: u32 = 150;
/// Share of the runway each later speed rung is sized to cross; half of
/// PILOT_ABORT_FRAC, the headroom a one-point prediction runs into.
const PILOT_TRAVEL_FRAC: f64 = 0.4;
/// A speed rung crossing more of the runway than this means the prediction
/// is badly off: stop before a faster rung gets a window that long.
const PILOT_ABORT_FRAC: f64 = 0.8;
/// v_ss is fit over the last 40% of a window, plant.py's settled tail.
const SETTLED_FRAC: f64 = 0.4;
/// Fewest settled pos samples (1 ms of ticks) a v_ss slope is fit from.
const SETTLED_MIN_SAMPLES: usize = 20;
/// Longest rung window; the bench hand table's cap.
const WINDOW_MAX_MS: u32 = 1500;
/// Share of the runway a campaign rung is sized to cross; the bench hand
/// table's bar.
const RUNWAY_FRAC: f64 = 0.8;
/// Duty step to steady speed; the bench hand table's spin-up allowance.
const SPINUP_MS: u32 = 30;
/// Below this many counts/ms the shaft is breaking away, not cruising: the
/// window caps, and a speed rung stays out of the fit.
const V_SS_MIN: f64 = 0.3;
/// A verified rung must cross this share of the runway: the windows aim at
/// RUNWAY_FRAC, so less means v_ss over-predicts and the rungs waste runway.
const VERIFY_MIN_FRAC: f64 = 0.55;
/// And at most this share, 10% of runway short of the guard.
const VERIFY_MAX_FRAC: f64 = 0.9;
/// The 60% rung's travel must land within 10% of the fit's prediction.
const VERIFY_PRED_TOL: f64 = 0.1;
/// Highest duty the coast ladder may open on: its first rung runs with no
/// prediction to hold it back. 30% crosses a quarter of mg90-a's 2S runway.
const COAST_FIRST_MAX_PCT: u8 = 30;
/// A coast's entry speed is the pot slope over the drive's last 20 ms; over
/// 5 ms it reads the pot track's local nonlinearity (9.6 counts/ms at 80% on
/// mg90-a 2S where 20 ms reads 13.7).
const ENTRY_MS: u32 = 20;
/// Share added to a predicted coast travel before it is held against the
/// room, and to a measured one before its duty can top the coast block. The
/// worst under-prediction replaying mg90-a's five 2S spindown captures
/// through the ladder was 2.8%; its 80% travel spread 6% across them, and
/// the campaign repeats every coast duty.
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
    let (procedure, source) = Procedure::load()?;
    let block = &procedure.block.coast;
    let duties = ladder_duties(&block.duties).with_context(|| format!("procedure {source}"))?;
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);

    let fw = c.identity(id)?.fw;
    let tick_hz = read_u16(&mut c, id, calib::TICK_HZ)?;
    if tick_hz == 0 {
        bail!("tick_hz reads 0: TEL sample rate unknown");
    }
    let soft = (
        read_i32(&mut c, id, config::POS_MIN_SOFT_COUNTS)?,
        read_i32(&mut c, id, config::POS_MAX_SOFT_COUNTS)?,
    );
    let phys = (
        read_i32(&mut c, id, config::POS_MIN_PHYS_COUNTS)?,
        read_i32(&mut c, id, config::POS_MAX_PHYS_COUNTS)?,
    );
    // Nothing has moved yet, so a refusal here leaves the horn where it is.
    let lim = limits(soft, phys)?;
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
    for (end, m) in [
        ("low", lim.soft[0] as i32 - lim.phys[0] as i32),
        ("high", lim.phys[1] as i32 - lim.soft[1] as i32),
    ] {
        println!("  soft-to-phys margin {end}: {m} counts");
        if m < BRAKE_MARGIN_MIN {
            eprintln!(
                "warning: {end} soft limit sits {m} counts from the stop, under \
                 {BRAKE_MARGIN_MIN}: the firmware brake stops past soft, so a runaway can \
                 still hit the stop"
            );
        }
    }

    let r = measure(&mut c, id, &lim, tick_hz as f64 / 1000.0, block, &duties);
    // Parks on failure too; a failed run reports its own error, not the park's.
    let parked = park(&mut c, id, lim.center);
    let (v_ss, windows_ms, coast, verified) = r?;
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let env = Envelope {
        supply: a.supply,
        fw,
        git_sha: sweep::git_sha(),
        measured: civil_date(secs),
        seek_pct: SEEK_PCT,
        limits: lim,
        v_ss,
        windows_ms,
        coast,
        verified,
    };
    std::fs::create_dir_all(&dir).with_context(|| format!("mkdir {}", dir.display()))?;
    let path = env.save(&dir)?;
    println!("envelope: {}", path.display());
    parked
}

type Measured = (Speed, BTreeMap<u8, u32>, Coast, Vec<Verified>);

/// Every motion of the pilot; `run` parks after it whatever it returns.
fn measure(
    c: &mut Client<NusbPipe>,
    id: Id,
    lim: &Limits,
    ticks_per_ms: f64,
    block: &CoastBlock,
    duties: &[u8],
) -> Result<Measured> {
    let fwd = speed_fit(c, id, lim, ticks_per_ms, Dir::Fwd)?;
    let rev = speed_fit(c, id, lim, ticks_per_ms, Dir::Rev)?;
    let used = if fwd.at(100) >= rev.at(100) {
        Dir::Fwd
    } else {
        Dir::Rev
    };
    for (d, f) in [(Dir::Fwd, &fwd), (Dir::Rev, &rev)] {
        let mark = if d == used {
            "  <- sizes the windows"
        } else {
            ""
        };
        println!(
            "[fit] {d}: v_ss = {} x duty {:+} counts/ms, r2 {}, v(100) {:.2}{mark}",
            f.slope,
            f.intercept,
            f.r2,
            f.at(100)
        );
    }
    let v_ss = Speed {
        duties: PILOT_DUTIES.to_vec(),
        used,
        fwd,
        rev,
    };
    let fit = *v_ss.used();
    let wins = windows(&fit, lim.runway);
    let row = wins
        .iter()
        .map(|(d, w)| format!("{d}:{w}"))
        .collect::<Vec<_>>()
        .join(" ");
    println!("[windows] {row}");

    // In the direction that sized the windows: it travels furthest.
    let mut verified = Vec::new();
    for (d, check_pred) in [(60, true), (100, false)] {
        let w = window_ms(fit.at(d), lim.runway);
        let step = Step::Drive(d, Some(w));
        let rec = sweep::record(
            c,
            id,
            &cfg(vec![step], sweep_dirs(used), w, lim),
            |_| Ok(()),
        )?;
        let travel = span(&one(&rec.segments)?.frames)?;
        let predicted = predicted_travel(fit.at(d), w);
        println!(
            "[verify] {used} {step}: travel {travel} ({:.0}% of runway), predicted {predicted}",
            100.0 * travel as f64 / lim.runway as f64
        );
        if !runway_frac_ok(travel, lim.runway) {
            bail!(
                "{step} crossed {travel} of {} runway, outside {VERIFY_MIN_FRAC}..{VERIFY_MAX_FRAC}",
                lim.runway
            );
        }
        if check_pred && !matches_prediction(travel, predicted) {
            bail!("{step} crossed {travel}, not within {VERIFY_PRED_TOL} of {predicted}");
        }
        verified.push(Verified {
            step: step.to_string(),
            travel,
            predicted,
        });
    }

    let coast = coast_ladder(c, id, lim, ticks_per_ms, block, duties, &v_ss)?;
    Ok((v_ss, wins, coast, verified))
}

/// One direction's five speed rungs, each window sized off the rungs before
/// it, then the affine fit over the ones that moved.
fn speed_fit(
    c: &mut Client<NusbPipe>,
    id: Id,
    lim: &Limits,
    ticks_per_ms: f64,
    dir: Dir,
) -> Result<Fit> {
    let dirs = sweep_dirs(dir);
    let sign = f64::from(sign(dir));
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for d in PILOT_DUTIES {
        let w = next_window(&pts, d, lim.runway);
        let step = Step::Drive(d, Some(w));
        let rec = sweep::record(c, id, &cfg(vec![step], dirs, w, lim), |_| Ok(()))?;
        let frames = &one(&rec.segments)?.frames;
        let travel = span(frames)?;
        if travel as f64 > PILOT_ABORT_FRAC * lim.runway as f64 {
            bail!(
                "{dir} {step} crossed {travel} of {} runway, over {PILOT_ABORT_FRAC}: \
                 the window prediction is off, stopping",
                lim.runway
            );
        }
        let slope = settled_slope(&pos_ticks(frames), SETTLED_FRAC).ok_or_else(|| {
            anyhow!("{dir} {step}: under {SETTLED_MIN_SAMPLES} settled pos samples")
        })?;
        let v = sign * slope * ticks_per_ms;
        println!("[rung] {dir} {step}: v_ss {v:.3} counts/ms, travel {travel}");
        if v > V_SS_MIN {
            pts.push((d as f64, v));
        } else {
            println!("  under {V_SS_MIN} counts/ms: breaking away, left out of the fit");
        }
    }
    let (a, b, r2) =
        affine_fit(&pts).ok_or_else(|| anyhow!("{dir}: under 2 moving rungs, no v_ss fit"))?;
    Ok(Fit::rounded(a, b, r2))
}

/// Climb the coast block's duties, each run both ways only while the travel
/// predicted from the rungs below fits the room; the top is the last rung
/// whose measured travel fit too. Every duty up to the top has then run on
/// the servo, so there is nothing left to verify.
fn coast_ladder(
    c: &mut Client<NusbPipe>,
    id: Id,
    lim: &Limits,
    ticks_per_ms: f64,
    block: &CoastBlock,
    duties: &[u8],
    v_ss: &Speed,
) -> Result<Coast> {
    let room = coast_room(lim);
    println!(
        "[coast] ladder {duties:?}%: {} ms drive, {} ms coast, room fwd {} rev {}, margin {COAST_MARGIN}",
        block.drive_ms, block.coast_ms, room.fwd, room.rev
    );
    let mut ladder = Vec::new();
    let mut refused = None;
    let mut top = None;
    for &pct in duties {
        let predicted = match next_rung(&ladder, pct, v_ss, &room) {
            Next::Run(p) => p,
            Next::Refuse(p) => {
                println!(
                    "  {pct}%: predicted travel fwd {:.0} rev {:.0} does not fit the room, stopping",
                    p.fwd, p.rev
                );
                refused = Some(Refused {
                    pct,
                    predicted: PerDir {
                        fwd: to_counts(p.fwd),
                        rev: to_counts(p.rev),
                    },
                });
                break;
            }
        };
        let chain = vec![
            Step::Drive(pct, Some(block.drive_ms)),
            Step::Coast(block.coast_ms),
        ];
        let rec = sweep::record(c, id, &cfg(chain, Dirs::Both, block.drive_ms, lim), |_| {
            Ok(())
        })?;
        let run = |d| {
            coast_run(
                &rec.segments,
                d,
                lim.soft,
                ticks_per_ms,
                predicted.map(|p| *p.get(d)),
            )
            .with_context(|| format!("{pct}% coast"))
        };
        let rung = CoastRung {
            pct,
            fwd: run(Dir::Fwd)?,
            rev: run(Dir::Rev)?,
        };
        for d in DIRS {
            let r = rung.get(d);
            let pred = r
                .predicted
                .map_or(String::new(), |p| format!(" (predicted {p})"));
            println!(
                "  {pct:3}% {d}: entry {} counts/ms, lead {} + coast {} = {}{pred}, {} inside soft",
                r.entry, r.lead, r.coast, r.travel, r.peak_inside_soft
            );
        }
        let fits = rung_fits(&rung, &room);
        ladder.push(rung);
        if !fits {
            println!("  {pct}% travel does not fit the room, stopping");
            break;
        }
        top = Some(pct);
    }
    let top_pct = top.ok_or_else(|| {
        anyhow!(
            "the {}% coast already travels too far for the room: no coast duty fits",
            duties[0]
        )
    })?;
    println!("  top duty {top_pct}%");
    Ok(Coast {
        drive_ms: block.drive_ms,
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
/// bursts, where the drive duty still applies.
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

/// A pilot recording at the session defaults: no baseline, every step with
/// its own window.
fn cfg(steps: Vec<Step>, dirs: Dirs, window_ms: u32, lim: &Limits) -> Cfg {
    Cfg {
        steps,
        dirs,
        decay: Decay::Slow,
        window_ms,
        rest_ms: REST_MS,
        baseline_ms: 0,
        seek_duty_pct: SEEK_PCT,
        seek_cap_pct: SEEK_CAP_PCT,
        settle_ms: SETTLE_MS,
        stall: false,
        static_load: false,
        guard: (lim.guard[0], lim.guard[1]),
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

fn span(frames: &[TelFrame]) -> Result<u16> {
    let pos = frames.iter().filter_map(|f| f.pos);
    let (lo, hi) = pos.fold((u16::MAX, 0), |(lo, hi), p| (lo.min(p), hi.max(p)));
    if lo > hi {
        bail!("segment has no pos samples");
    }
    Ok(hi - lo)
}

/// Soft, guard, centre and runway from the servo's soft and phys limits,
/// refusing a servo `osc cal` never set up or one too short to pilot.
fn limits(soft: (i32, i32), phys: (i32, i32)) -> Result<Limits> {
    if soft == (0, POT_MAX) || soft.1 - soft.0 >= POT_MAX {
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
    let guard = guards(soft);
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

fn guards(soft: [u16; 2]) -> [u16; 2] {
    [
        soft[0].saturating_add(GUARD_INSET),
        soft[1].saturating_sub(GUARD_INSET),
    ]
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

/// Window for speed rung `d` from the moving rungs so far (duty, counts/ms).
/// One point cannot see the intercept, so it predicts through the origin,
/// which under-predicts an affine law with a negative intercept: the second
/// rung runs long by v_true / v_pred (1.3x at 10 -> 20% on the mg90 fit,
/// 0.52 of runway), the headroom PILOT_TRAVEL_FRAC leaves under
/// PILOT_ABORT_FRAC. Speed never falls with duty, so no prediction goes
/// below the fastest rung measured.
fn next_window(pts: &[(f64, f64)], d: u8, runway: u16) -> u32 {
    let d = d as f64;
    let pred = match pts {
        [] => return PILOT_W0,
        [(d1, v1)] => v1 * d / d1,
        _ => affine_fit(pts).map_or(0.0, |(a, b, _)| a * d + b),
    };
    let v = pts.iter().fold(pred, |m, p| m.max(p.1));
    ((PILOT_TRAVEL_FRAC * runway as f64 / v).round() as u32).min(WINDOW_MAX_MS)
}

/// Campaign window for a rung cruising at `v` counts/ms.
fn window_ms(v: f64, runway: u16) -> u32 {
    if v <= V_SS_MIN {
        return WINDOW_MAX_MS;
    }
    ((RUNWAY_FRAC * runway as f64 / v).round() as u32 + SPINUP_MS).min(WINDOW_MAX_MS)
}

pub(super) fn windows(fit: &Fit, runway: u16) -> BTreeMap<u8, u32> {
    (1..=20u8)
        .map(|k| k * 5)
        .map(|d| (d, window_ms(fit.at(d), runway)))
        .collect()
}

fn predicted_travel(v: f64, w: u32) -> u16 {
    (v * w.saturating_sub(SPINUP_MS) as f64)
        .round()
        .clamp(0.0, u16::MAX as f64) as u16
}

fn runway_frac_ok(travel: u16, runway: u16) -> bool {
    (VERIFY_MIN_FRAC..=VERIFY_MAX_FRAC).contains(&(travel as f64 / runway as f64))
}

fn matches_prediction(travel: u16, predicted: u16) -> bool {
    let p = predicted as f64;
    ((1.0 - VERIFY_PRED_TOL) * p..=(1.0 + VERIFY_PRED_TOL) * p).contains(&(travel as f64))
}

fn to_counts(x: f64) -> u16 {
    x.round().clamp(0.0, u16::MAX as f64) as u16
}

/// The coast block's duties in ladder order, refusing a ladder that would
/// open above COAST_FIRST_MAX_PCT or drive past full scale.
fn ladder_duties(duties: &[u8]) -> Result<Vec<u8>> {
    let mut v = duties.to_vec();
    v.sort_unstable();
    v.dedup();
    match (v.first(), v.last()) {
        (None, _) => bail!("coast block has no duties"),
        (Some(&lo), _) if lo > COAST_FIRST_MAX_PCT => bail!(
            "coast block opens at {lo}%: the ladder's first rung runs unpredicted, so it must \
             open at or under {COAST_FIRST_MAX_PCT}%"
        ),
        (_, Some(&hi)) if hi > 100 => bail!("coast duty {hi}% is over full scale"),
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

/// Least-squares `coast = a v + b v^2` over (entry, coast) points; None when
/// the points cannot separate the terms or either comes out negative, which
/// no friction law gives.
fn coast_fit(pts: &[(f64, f64)]) -> Option<(f64, f64)> {
    let (s2, s3, s4, t1, t2) = pts.iter().fold(
        (0.0, 0.0, 0.0, 0.0, 0.0),
        |(s2, s3, s4, t1, t2), &(v, d)| {
            let v2 = v * v;
            (s2 + v2, s3 + v2 * v, s4 + v2 * v2, t1 + v * d, t2 + v2 * d)
        },
    );
    let det = s2 * s4 - s3 * s3;
    if det <= 0.0 {
        return None;
    }
    let a = (t1 * s4 - t2 * s3) / det;
    let b = (s2 * t2 - s3 * t1) / det;
    (a >= 0.0 && b >= 0.0).then_some((a, b))
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

    const MG90: Fit = Fit {
        slope: 0.2275,
        intercept: -0.845,
        r2: 0.998,
    };

    #[test]
    fn mg90_windows_match_the_bench_hand_table() {
        let want: BTreeMap<u8, u32> = [
            (5, 1500),
            (10, 1500),
            (15, 971),
            (20, 682),
            (25, 529),
            (30, 434),
            (35, 369),
            (40, 323),
            (45, 287),
            (50, 259),
            (55, 237),
            (60, 219),
            (65, 203),
            (70, 190),
            (75, 179),
            (80, 169),
            (85, 161),
            (90, 153),
            (95, 146),
            (100, 140),
        ]
        .into();
        assert_eq!(windows(&MG90, 3020), want);
    }

    #[test]
    fn slow_or_stalled_duties_cap_the_window() {
        // 5%: v 0.29 is under V_SS_MIN; 10%: v 1.43 needs 1719 ms
        assert!(MG90.at(5) <= V_SS_MIN);
        assert_eq!(window_ms(MG90.at(5), 3020), WINDOW_MAX_MS);
        assert_eq!(window_ms(MG90.at(10), 3020), WINDOW_MAX_MS);
        assert_eq!(window_ms(-1.0, 3020), WINDOW_MAX_MS);
        assert_eq!(window_ms(V_SS_MIN, 3020), WINDOW_MAX_MS);
    }

    #[test]
    fn predicted_travel_excludes_the_spinup() {
        // 60%: 12.805 counts/ms over 219 - 30 ms
        assert_eq!(predicted_travel(MG90.at(60), 219), 2420);
        assert_eq!(predicted_travel(5.0, 10), 0);
    }

    #[test]
    fn guard_band_clears_the_soft_clamp() {
        const { assert!(GUARD_INSET > BAND_HALF) };
        assert_eq!(guards([432, 3626]), [532, 3526]);
    }

    #[test]
    fn limits_from_a_calibrated_servo() {
        let lim = limits((432, 3626), (209, 3849)).unwrap();
        assert_eq!(lim.soft, [432, 3626]);
        assert_eq!(lim.phys, [209, 3849]);
        assert_eq!(lim.guard, [532, 3526]);
        assert_eq!(lim.center, 2029);
        assert_eq!(lim.runway, 2994);
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

    #[test]
    fn affine_fit_recovers_a_planted_line() {
        let pts: Vec<(f64, f64)> = PILOT_DUTIES
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

    #[test]
    fn next_window_opens_short_then_predicts() {
        assert_eq!(next_window(&[], 10, 3000), PILOT_W0);
        // one point: through the origin, 1.43 -> 2.86 at 20%
        assert_eq!(next_window(&[(10.0, 1.43)], 20, 3000), 420);
        // two or more: affine, exact on the mg90 line at 30%
        let pts = [(10.0, MG90.at(10)), (20.0, MG90.at(20))];
        assert_eq!(next_window(&pts, 30, 3000), 201);
        // a falling prediction never beats the fastest rung measured
        assert_eq!(next_window(&[(10.0, 4.0), (20.0, 2.0)], 30, 3000), 300);
        // and a slow servo caps
        assert_eq!(next_window(&[(10.0, 0.35)], 20, 3000), WINDOW_MAX_MS);
    }

    #[test]
    fn verification_bands() {
        assert!(runway_frac_ok(1647, 2994));
        assert!(!runway_frac_ok(1640, 2994));
        assert!(runway_frac_ok(2694, 2994));
        assert!(!runway_frac_ok(2700, 2994));
        assert!(matches_prediction(2179, 2420));
        assert!(matches_prediction(2661, 2420));
        assert!(!matches_prediction(2177, 2420));
        assert!(!matches_prediction(2663, 2420));
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
            duties: PILOT_DUTIES.to_vec(),
            used: Dir::Rev,
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
        CoastRung { pct, fwd, rev }
    }

    #[test]
    fn ladder_duties_climb_from_a_low_rung() {
        assert_eq!(ladder_duties(&[80, 20, 40, 20]).unwrap(), [20, 40, 80]);
        assert_eq!(ladder_duties(&[30]).unwrap(), [30]);
        let err = |d: &[u8]| ladder_duties(d).unwrap_err().to_string();
        assert!(err(&[]).contains("no duties"));
        assert!(err(&[40, 60]).contains("opens at 40%"));
        assert!(err(&[20, 101]).contains("over full scale"));
    }

    #[test]
    fn coast_room_runs_from_the_far_band_edge_to_soft() {
        let lim = limits((432, 3626), (209, 3849)).unwrap();
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
    fn mg90_ladder_reaches_80_where_the_one_probe_model_stopped_at_66() {
        let lim = limits((432, 3626), (209, 3849)).unwrap();
        let (room, v_ss) = (coast_room(&lim), mg90_2s_v_ss());

        // The one-probe model on this servo: the 40% probe's k = 551 / 7.293^2
        // put on v_ss, drive 50 ms at speed, capped 80% of the runway.
        let k = 551.0 / (7.293f64 * 7.293);
        let old = (40..=100u8)
            .rev()
            .find(|&d| {
                let v = v_ss.rev.at(d);
                v * 50.0 + k * v * v <= RUNWAY_FRAC * lim.runway as f64
            })
            .unwrap();
        assert_eq!(old, 66);

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

    fn seg(dir: i8, pos: &[u16]) -> Segment {
        let frames = pos
            .iter()
            .enumerate()
            .map(|(i, &p)| TelFrame {
                tick: i as u64,
                window_valid: true,
                pos: Some(p),
                current: None,
                current_trough: None,
                duty_q15: None,
                vdiff: None,
                vbus: None,
                current_raw: None,
                vmotor_a: None,
                vmotor_b: None,
                vbus_raw: None,
                ntc_raw: None,
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

    #[test]
    fn span_is_pos_max_minus_min() {
        assert_eq!(span(&seg(1, &[900, 532, 1200, 1100]).frames).unwrap(), 668);
        assert!(span(&seg(1, &[]).frames).is_err());
    }
}
