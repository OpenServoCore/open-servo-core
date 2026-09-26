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

use super::envelope::{Coast, Dir, Envelope, Fit, Limits, Speed, Verified, civil_date, round_to};
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
/// table's bar, and the coast derate's.
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
/// Coast probe duty, inside the 10..50% the speed rungs measured.
const COAST_PROBE_PCT: u8 = 40;
/// Drive before the coast: SPINUP_MS plus 50 ms at speed.
const COAST_DRIVE_MS: u32 = 80;
/// Coast capture, long enough to reach rest; a coast still moving at its end
/// is refused.
const COAST_MS: u32 = 400;

/// Entry from `osc capture pilot`.
pub(crate) fn run(a: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let root = match &a.root {
        Some(r) => r.clone(),
        None => super::default_root()?,
    };
    let dir = super::dataset_dir(&root, &a.servo, a.supply);
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

    let r = measure(&mut c, id, &lim, tick_hz as f64 / 1000.0);
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
fn measure(c: &mut Client<NusbPipe>, id: Id, lim: &Limits, ticks_per_ms: f64) -> Result<Measured> {
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

    let coast = coast_derate(c, id, lim, ticks_per_ms, &fit)?;
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
    let sign = if dir == Dir::Fwd { 1.0 } else { -1.0 };
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

/// Measure one coast, derate the coast block's top duty from it, and run
/// that top duty's chain both ways.
fn coast_derate(
    c: &mut Client<NusbPipe>,
    id: Id,
    lim: &Limits,
    ticks_per_ms: f64,
    fit: &Fit,
) -> Result<Coast> {
    let chain = |pct| {
        vec![
            Step::Drive(pct, Some(COAST_DRIVE_MS)),
            Step::Coast(COAST_MS),
        ]
    };
    println!("[coast] probe {COAST_PROBE_PCT}% for {COAST_DRIVE_MS} ms, then coast {COAST_MS} ms");
    let rec = sweep::record(
        c,
        id,
        &cfg(chain(COAST_PROBE_PCT), Dirs::Fwd, COAST_DRIVE_MS, lim),
        |_| Ok(()),
    )?;
    let [drive, coasting] = rec.segments.as_slice() else {
        bail!(
            "coast probe committed {} segments, not 2",
            rec.segments.len()
        );
    };
    let settled = |s: &Segment| {
        settled_slope(&pos_ticks(&s.frames), SETTLED_FRAC)
            .map(|v| v * ticks_per_ms)
            .ok_or_else(|| anyhow!("coast probe: under {SETTLED_MIN_SAMPLES} settled pos samples"))
    };
    let entry = round_to(settled(drive)?, 3);
    if entry <= V_SS_MIN {
        bail!("coast probe entered at {entry} counts/ms: the shaft never got going");
    }
    // A coast still rolling at the end of its capture understates the
    // distance, and k with it.
    let tail = settled(coasting)?;
    if tail.abs() >= V_SS_MIN {
        bail!("coast still moving at {tail:.2} counts/ms after {COAST_MS} ms");
    }
    let travel = span(&coasting.frames)?;
    let k = travel as f64 / (entry * entry);
    let top = derate_top_pct(fit, k, lim.runway);
    println!("  entry {entry} counts/ms, coasted {travel} counts: k {k:.3} ms^2/count");
    let bar = RUNWAY_FRAC * lim.runway as f64;
    for d in (COAST_PROBE_PCT..=100).step_by(10) {
        let v = fit.at(d).max(0.0);
        println!(
            "  {d:3}%: drive {:5.0} + coast {:5.0} = {:5.0} counts (bar {bar:.0})",
            v * (COAST_DRIVE_MS - SPINUP_MS) as f64,
            k * v * v,
            excursion(fit, k, d)
        );
    }
    println!("  top duty {top}%, verifying both ways");

    let rec = sweep::record(
        c,
        id,
        &cfg(chain(top), Dirs::Both, COAST_DRIVE_MS, lim),
        |_| Ok(()),
    )?;
    let peak = closest_to_soft(&rec.segments, lim.soft)
        .ok_or_else(|| anyhow!("coast verify: no pos samples"))?;
    if peak < 0 {
        bail!("{top}% coast crossed a soft limit by {} counts", -peak);
    }
    println!("  closest approach to soft: {peak} counts inside");
    Ok(Coast {
        probe_pct: COAST_PROBE_PCT,
        probe_entry: entry,
        probe_travel: travel,
        top_pct: top,
        peak_inside_soft: peak,
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
    let from = t1 as f64 - frac * (t1 - t0) as f64;
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

fn windows(fit: &Fit, runway: u16) -> BTreeMap<u8, u32> {
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

/// Counts from the start guard a `d`% coast chain reaches: the drive at
/// speed, then a coast of `k v^2` (kinetic energy spent against Coulomb
/// friction). A viscous `k v` model would extrapolate shorter coasts at
/// higher speed; v^2 derates harder, the safe side.
fn excursion(fit: &Fit, k: f64, d: u8) -> f64 {
    let v = fit.at(d).max(0.0);
    v * (COAST_DRIVE_MS - SPINUP_MS) as f64 + k * v * v
}

/// Top duty whose coast chain stays within RUNWAY_FRAC of the runway, never
/// under the probe's own duty (the probe already ran).
fn derate_top_pct(fit: &Fit, k: f64, runway: u16) -> u8 {
    let bar = RUNWAY_FRAC * runway as f64;
    (COAST_PROBE_PCT..=100)
        .rev()
        .find(|&d| excursion(fit, k, d) <= bar)
        .unwrap_or(COAST_PROBE_PCT)
}

/// Closest any sample came to the soft limit it was heading for, counts
/// inside soft; negative when one crossed.
fn closest_to_soft(segs: &[Segment], soft: [u16; 2]) -> Option<i32> {
    segs.iter()
        .flat_map(|s| s.frames.iter().filter_map(move |f| Some((s.dir, f.pos?))))
        .map(|(dir, p)| {
            if dir > 0 {
                soft[1] as i32 - p as i32
            } else {
                p as i32 - soft[0] as i32
            }
        })
        .min()
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

    #[test]
    fn derate_tops_out_where_drive_plus_coast_meets_the_bar() {
        // k = 0: drive alone, 50 ms at v(74) = 15.99 -> 800 of the 800 bar
        assert_eq!(derate_top_pct(&MG90, 0.0, 1000), 74);
        let one = derate_top_pct(&MG90, 1.0, 1000);
        let two = derate_top_pct(&MG90, 2.0, 1000);
        assert_eq!((one, two), (59, 52));
        // a coast too long for even the probe still keeps the probe's duty
        assert_eq!(derate_top_pct(&MG90, 1e6, 1000), COAST_PROBE_PCT);
        assert_eq!(derate_top_pct(&MG90, 0.0, 30000), 100);
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

    #[test]
    fn closest_to_soft_follows_each_segments_direction() {
        let soft = [432, 3626];
        let segs = [seg(1, &[532, 2900, 3000]), seg(-1, &[3526, 900, 700])];
        assert_eq!(closest_to_soft(&segs, soft), Some(268));
        let crossed = [seg(1, &[532, 3630])];
        assert_eq!(closest_to_soft(&crossed, soft), Some(-4));
        assert_eq!(closest_to_soft(&[seg(1, &[])], soft), None);
    }

    #[test]
    fn span_is_pos_max_minus_min() {
        assert_eq!(span(&seg(1, &[900, 532, 1200, 1100]).frames).unwrap(), 668);
        assert!(span(&seg(1, &[]).frames).is_err());
    }
}
