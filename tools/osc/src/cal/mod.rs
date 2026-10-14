//! `osc cal` - the calibration orchestrator: take the gear train's
//! material, find the mechanical end-stops and the drive polarity (applied
//! at once), ask at a stop which count end is the most-counterclockwise,
//! confirm the real-world angle range with the operator, print the
//! count->unit report, then write the limits, pot stops, angle endpoints,
//! gear ratio and class, stamp the set and persist with MGMT SAVE.
//! Degrees rise counterclockwise facing the horn, so the angle map
//! descends on a pot whose counts rise clockwise; the counts stay the
//! pot's.
//!
//! The stops are found gently, in the order osc-ident's `Run::for_cal`
//! names: the jam check and a burst for winding R at mid travel, then each
//! stop approached at one duty whose stall the gear train's stop-hold
//! limit holds and read seated at half that limit. A servo that needs more than its limit to
//! start moving is refused before anything drives at a stop; a burst that
//! measures no R leaves the stops to the duty that moved the shaft. A
//! refusal or an abort writes nothing to config, calib or the position
//! table.
//!
//! Interactive by default; flags make it headless. The rail-to-rail traverse
//! streams a TEL current+pos sweep in bus bursts: its commutation ripple
//! gives a MEASURED gear ratio (the gear prompt's default), gear-2-dependent
//! and degrading gracefully (operator-input gear) when ripple SNR is low.
//! The traverse runs free inside the stops and brakes short of them by the
//! runway rule. The position table is not cal's to write: a new table
//! re-defines the domain the identified constants were fitted in, so it
//! travels with an ident; cal only re-COMMITs a LIVE one against the stops
//! it just moved. The experiments and the kinematics/units math live in
//! osc-ident; this wrapper owns USB, prompts, and files.

pub mod replay;

use std::path::Path;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use dialoguer::{Confirm, Input};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::calib::GearClass;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut;
use osc_ident::exp::bias::{Bias, BiasCfg};
use osc_ident::exp::endstop::{Endstop, EndstopResult};
use osc_ident::exp::inductance::{Cfg as InductanceCfg, FitCfg, Inductance, fit_captures};
use osc_ident::exp::rl::Scales;
use osc_ident::exp::sweep::{self as traverse, Sweep, SweepCfg};
use osc_ident::exp::{Experiment, Guarded, Permitted, RigParams};
use osc_ident::frame::TelFrame;
use osc_ident::gains;
use osc_ident::kinematics::{self, KinematicsResult, angle_endpoints};
use osc_ident::limits::{BurstAllowance, Refusal, ServoLimits};
use osc_ident::lut::{self, stitched_motor_revs};
use osc_ident::regs::{Reg, calib, config};
use osc_ident::run::{self as order, Ended, Over, Run, Stage};
use osc_ident::runway::{Runway, Supply};
use osc_ident::slip;
use osc_ident::sources;
use osc_ident::units::{self, SenseParams};

use crate::capture::envelope;
use crate::rig::Bus;
use crate::rig::centre::{centre_on, drive as drive_on, jam_check};
use crate::rig::csvio::{self, OutDir, SnapshotLog};
use crate::rig::pump::{Pump, write_reg};
use crate::rig::servo::{Servo, Wire, guard};
use crate::rig::snapshot::{self, read_u16};
use crate::rig::{Aborted, check_abort};

/// Commutation events per rotor rev for the brushed 3-slot motor: the ripple
/// tach and the LUT angle clock both count 6 per revolution.
const RIPPLE_PER_REV: f64 = 6.0;

/// Ripple coverage (fraction of sweep windows that found ripple) below this
/// flags the gear estimate as low-confidence in the printout - still used, as
/// expected on a stripped gear where slip corrupts the pot displacement.
const RIPPLE_CONF_MIN: f64 = 0.7;

/// Assumed total rated travel when the operator gives no phys angle: a 0..180
/// deg span. Only a default; every real servo overrides it.
const DEFAULT_TRAVEL_DEG: f64 = 180.0;

/// Share of the stop-to-stop travel each default soft limit sits inside its
/// stop. A rung whose host dies runs on until the firmware brakes it at the
/// soft limit, and the capture pilot runs only rungs whose braked stop fits
/// this room (`runway::dead_host_fits`). A braked stop scales with speed and
/// supply, neither of which cal fixes for the saved limits, so the room is a
/// share of travel: on the rev 2A MG90 (stops 283/3829) 222 counts an end,
/// past the 2S braked stop of a 30% rung (about 95) and near a 55% one's
/// (about 224).
const SOFT_MARGIN_FRAC: f64 = 0.0625;

/// The current a stall against a stop may draw, mA, by gear train: plastic
/// teeth strip under a held stall (an SG90 train holds 0.33 A healthy and
/// slips at 0.18 A once damaged); a metal MG90 train holds 0.52 A at a stop.
const STOP_HOLD_PLASTIC_MA: f64 = 300.0;
const STOP_HOLD_METAL_MA: f64 = 500.0;

/// The gear train cal writes: plastic or metal, never unknown.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum Gears {
    Plastic,
    Metal,
}

impl Gears {
    fn of(class: GearClass) -> Option<Self> {
        match class {
            GearClass::Plastic => Some(Gears::Plastic),
            GearClass::Metal => Some(Gears::Metal),
            GearClass::Unknown => None,
        }
    }

    fn class(self) -> GearClass {
        match self {
            Gears::Plastic => GearClass::Plastic,
            Gears::Metal => GearClass::Metal,
        }
    }

    fn stop_hold_ma(self) -> f64 {
        match self {
            Gears::Plastic => STOP_HOLD_PLASTIC_MA,
            Gears::Metal => STOP_HOLD_METAL_MA,
        }
    }
}

/// The count end of the pot at the horn's most-counterclockwise stop,
/// facing the horn. Degrees rise counterclockwise (ROS REP 103), so the
/// angle map descends when this is the low end; counts follow the pot.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum CcwEnd {
    Low,
    High,
}

impl CcwEnd {
    /// What a calibrated angle map says, None while it is unset.
    fn of(angle_min_cdeg: i16, angle_max_cdeg: i16) -> Option<Self> {
        match angle_max_cdeg.cmp(&angle_min_cdeg) {
            std::cmp::Ordering::Greater => Some(CcwEnd::High),
            std::cmp::Ordering::Less => Some(CcwEnd::Low),
            std::cmp::Ordering::Equal => None,
        }
    }

    fn other(self) -> Self {
        match self {
            CcwEnd::Low => CcwEnd::High,
            CcwEnd::High => CcwEnd::Low,
        }
    }

    /// The count end nearer `pos`, between the stops `lo` and `hi`.
    fn nearer(pos: u16, (lo, hi): (i32, i32)) -> Self {
        let pos = pos as i32;
        if pos - lo < hi - pos {
            CcwEnd::Low
        } else {
            CcwEnd::High
        }
    }

    /// The angles at the low and the high count stop, from the angles at
    /// the most-clockwise and most-counterclockwise stops.
    fn rail_angles(self, cw_deg: f64, ccw_deg: f64) -> (f64, f64) {
        match self {
            CcwEnd::High => (cw_deg, ccw_deg),
            CcwEnd::Low => (ccw_deg, cw_deg),
        }
    }
}

/// How cal settles the counterclockwise end once it rests at a stop: the
/// operator says, or under `--yes` the flag, else the servo's angle map.
#[derive(Copy, Clone, Debug)]
struct Ccw {
    known: Option<CcwEnd>,
    ask: bool,
}

impl Ccw {
    /// Refuses `--yes` with nothing to settle it before anything drives.
    fn new(args: &Args, stored: Option<CcwEnd>) -> Result<Self> {
        let known = args.ccw_end.or(stored);
        if args.yes && known.is_none() {
            bail!(
                "--yes needs --ccw-end low|high: the servo's angle map does not say which count \
                 end is the most-counterclockwise"
            );
        }
        Ok(Self {
            known,
            ask: !args.yes,
        })
    }

    /// The counterclockwise end, the shaft at rest at `pos` beside one of
    /// `stops`.
    fn settle(&self, pos: u16, stops: (i32, i32)) -> Result<CcwEnd> {
        let at = CcwEnd::nearer(pos, stops);
        if !self.ask {
            return self
                .known
                .context("the most-counterclockwise end is not known");
        }
        let mut prompt = Confirm::new()
            .with_prompt("Is the horn at its most-counterclockwise end? (facing the horn)");
        if let Some(k) = self.known {
            prompt = prompt.default(k == at);
        }
        Ok(if prompt.interact()? { at } else { at.other() })
    }
}

/// `osc cal` args: output dir plus the headless overrides. `--baud`/`--id`
/// come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir for the rollback snapshot, the endstop log, the bursts and
    /// the traverse (default ./cal-out).
    #[arg(long)]
    out: Option<std::path::PathBuf>,
    /// Real angle (deg) at the most-clockwise stop, facing the horn.
    #[arg(long)]
    phys_angle_min: Option<f64>,
    /// Real angle (deg) at the most-counterclockwise stop. With
    /// --gear-ratio, a check on the travel the gear derives: cal writes
    /// nothing past 10% apart.
    #[arg(long)]
    phys_angle_max: Option<f64>,
    /// App-facing working-range min (deg); defaults to 6.25% of the travel
    /// in from the most-clockwise stop, room for the firmware to brake there.
    #[arg(long)]
    soft_angle_min: Option<f64>,
    /// App-facing working-range max (deg); defaults to 6.25% of the travel
    /// in from the most-counterclockwise stop.
    #[arg(long)]
    soft_angle_max: Option<f64>,
    /// Known gear ratio (motor rev per output rev); unset leaves it 0.
    #[arg(long)]
    gear_ratio: Option<f64>,
    /// The gear train's material; required with --yes on a servo whose
    /// class is unknown.
    #[arg(long, value_enum)]
    gears: Option<Gears>,
    /// The pot's count end at the horn's most-counterclockwise stop, facing
    /// the horn; required with --yes unless the servo's angle map says.
    #[arg(long, value_enum)]
    ccw_end: Option<CcwEnd>,
    /// Assume yes: never prompt. Required values must come from flags.
    #[arg(long)]
    yes: bool,
    /// Write and stamp the set without MGMT SAVE: it runs until a reboot
    /// brings the saved set back.
    #[arg(long)]
    no_save: bool,
}

/// Entry from the top-level `osc cal` dispatch.
pub fn run(args: &Args, bus: Bus, id: u8) -> Result<()> {
    crate::rig::pump::install_ctrlc();
    let id = Id::new(id);
    let mut c = crate::rig::connect(&bus, &[id])?;
    let d = crate::state::descriptor(&mut c, id)?;
    crate::state::warn(&mut c, id, &d)?;
    crate::rig::pump::admit_watch(&mut c, id)?;
    let lim = crate::rig::limits::read(&mut c, id)?;

    let sense = read_sense(&mut c, id)?;
    let sc = Scales::from_sense(
        &sense,
        read_u16(&mut c, id, calib::VBUS_DIV_TOP_OHM)?,
        read_u16(&mut c, id, calib::VBUS_DIV_BOT_OHM)?,
    )
    .context("CalibSense scales degenerate (shunt/gain/dividers/vdd)")?;
    // TEL frames arrive one per fast tick, so tick_hz is the sweep sample rate.
    let fs = sense.tick_hz as f64;
    let ke_vpc_q = read_u16(&mut c, id, calib::KE_VPC_Q)?;
    if ke_vpc_q == 0 {
        eprintln!("warning: ke_vpc_q is 0 - run `osc ident` first for the torque report");
    }

    let gears = gears(args, c.gear_class(id, &d)?)?;
    let ccw = Ccw::new(
        args,
        CcwEnd::of(
            read_u16(&mut c, id, calib::ANGLE_MIN_CDEG)? as i16,
            read_u16(&mut c, id, calib::ANGLE_MAX_CDEG)? as i16,
        ),
    )?;
    let stop_lim = stop_hold(lim, gears);
    println!(
        "gear train {}: stops held under {}",
        gears.class().name(),
        lim.ma().of(stop_lim.i_lim as f64)
    );

    // Safety gate before any motion: both mechanical ends get driven.
    if !args.yes {
        println!("cal drives the servo into BOTH mechanical end-stops.");
        if !confirm("proceed", false)? {
            println!("declined, no writes");
            return Ok(());
        }
    }

    let out = OutDir::create(args.out.as_deref().unwrap_or(Path::new("./cal-out")))?;
    let Found {
        stops: r,
        tel,
        ccw_end,
    } = find_stops(&mut Wire::new(&mut c, id), &out, lim, stop_lim, sc, fs, ccw)?;
    let EndstopResult {
        pos_min_phys,
        pos_max_phys,
        ..
    } = r;

    // The traverse's bursts can still fragment on dropped frames (16-tick
    // holes), so the anchor stitches ALL chunks (build_sweep_chunks) over
    // the shared pos axis; the single longest run (build_sweep) is kept only
    // for the slip health-check and the moving-run print.
    let sweep = build_sweep(&tel);
    let chunks = build_sweep_chunks(&tel);
    match &sweep {
        Some((pos, _)) => println!(
            "[sweep] {} tel frames, moving run {} (fs {fs:.0} Hz from tick_hz)",
            tel.len(),
            pos.len()
        ),
        None => println!("[sweep] {} tel frames, no usable moving run", tel.len()),
    }

    // Full-traverse motor revs from the ripple sweep: anchor-free geometry,
    // stitched over all chunks and extrapolated from the covered phase span to
    // the whole rail count span. Paired with a gear ratio it yields travel;
    // paired with a travel angle it yields the gear. Both anchors consume this
    // same ripple half.
    let motor_revs_full = stitched_motor_revs(
        &chunks,
        fs,
        RIPPLE_PER_REV,
        pos_min_phys as u16,
        pos_max_phys as u16,
    );
    let mrf = motor_revs_full.map(|(m, _)| m);
    let coverage = motor_revs_full.map(|(_, c)| c);
    println!(
        "[sweep] {} clean chunks, stitched coverage {:.0}%",
        chunks.len(),
        coverage.unwrap_or(0.0) * 100.0
    );

    // Resolve the anchor: gear is preferred, travel is the fallback. --yes stays
    // headless through the pure resolve_anchor; interactively the operator
    // anchors on the gear first (enter 0 to switch to a travel angle instead).
    let (phys_cw, phys_ccw, gear_ratio_centi) = if args.yes {
        let (lo, hi, gear) = resolve_anchor(
            mrf,
            args.gear_ratio,
            args.phys_angle_min,
            args.phys_angle_max,
        );
        report_yes_anchor(args, mrf, coverage, lo, hi, gear)?;
        (lo, hi, gear)
    } else {
        resolve_anchor_interactive(args, mrf, coverage)?
    };

    let (soft_min_deg, soft_max_deg) = soft_angles(args, phys_cw, phys_ccw)?;
    let (at_min, at_max) = ccw_end.rail_angles(phys_cw, phys_ccw);
    let (angle_min_cdeg, angle_max_cdeg) = angle_endpoints(at_min, at_max);
    let dpc = kinematics::deg_per_count(angle_min_cdeg, angle_max_cdeg, pos_min_phys, pos_max_phys);

    // gear-mesh slip check on the SAME sweep: pot must advance in fixed
    // proportion to motor rotation; a slipping/stripped tooth spikes or zeros
    // a segment. Advisory - one sweep can miss a localized slip zone.
    if let Some((pos, current)) = &sweep
        && let Some((phase, _)) = lut::cumulative_phase(current, fs, RIPPLE_PER_REV)
        && let Some(rep) = slip::slip_metrics(pos, &phase, 10, 1)
    {
        println!(
            "gear mesh: slip CoV {:.2} ({} core segments)",
            rep.slip_cov, rep.core_segments
        );
        if rep.flagged {
            eprintln!(
                "warning: possible worn/stripped gear teeth - gear-slip signature (slip CoV {:.2}, {} stuck segment(s)); a single sweep is not conclusive, re-run cal to confirm",
                rep.slip_cov, rep.stuck_count
            );
        }
    }

    // soft angle -> phys count via the linear phys map, clamped to the rails;
    // a descending map puts the larger angle on the lower count
    let soft_counts = [soft_min_deg, soft_max_deg]
        .map(|deg| soft_to_count(deg, at_min, at_max, pos_min_phys, pos_max_phys));
    let (soft_min_count, soft_max_count) = (
        soft_counts[0].min(soft_counts[1]),
        soft_counts[0].max(soft_counts[1]),
    );

    let kin = KinematicsResult {
        angle_min_cdeg,
        angle_max_cdeg,
        gear_ratio_centi,
    };
    let factors = units::derive(&sense, &kin, pos_min_phys, pos_max_phys, ke_vpc_q);
    print!("{}", units::render(&factors));

    println!("deg/count {dpc:.6}");
    println!(
        "phys angle {phys_cw:.2} deg clockwise..{phys_ccw:.2} deg counterclockwise, the \
         counterclockwise end at the {} counts; soft angle {soft_min_deg:.2}..{soft_max_deg:.2} deg",
        match ccw_end {
            CcwEnd::Low => "low",
            CcwEnd::High => "high",
        }
    );
    println!("soft counts {soft_min_count}..{soft_max_count}");

    let prompt = if args.no_save {
        "write these values, no SAVE"
    } else {
        "write these values and MGMT SAVE"
    };
    if !args.yes && !confirm(prompt, false)? {
        println!("declined before write");
        return Ok(());
    }

    // snapshot the calib + gain block before any write (rollback safety)
    snapshot::take_snapshot(&mut c, id, &out.0.join("snapshot.json"))?;
    let set = Calibration {
        phys: (pos_min_phys, pos_max_phys),
        soft: (soft_min_count, soft_max_count),
        angle_cdeg: (angle_min_cdeg as i32, angle_max_cdeg as i32),
        gear_ratio_centi,
        gears,
    };
    write_calibration(&mut c, id, &d, &set, !args.no_save)
}

/// The gear train: `--gears`, else the class the servo holds. Under
/// `--yes` an unknown class with no flag refuses; interactively the
/// operator is asked, the known class the default.
fn gears(args: &Args, stored: GearClass) -> Result<Gears> {
    let known = args.gears.or(Gears::of(stored));
    if args.yes {
        return known
            .context("--yes needs --gears plastic|metal: the servo's gear class is unknown");
    }
    let mut prompt = Input::<String>::new()
        .with_prompt("Gear train: plastic or metal?")
        .validate_with(|s: &String| Gears::from_str(s, true).map(|_| ()));
    if let Some(g) = known {
        prompt = prompt.default(g.class().name().into());
    }
    Gears::from_str(&prompt.interact_text()?, true).map_err(anyhow::Error::msg)
}

/// The limits cal's run plans under: the servo's, the current limit
/// lowered to the gear train's stop hold through the sense scale, never
/// raised past the servo's own.
fn stop_hold(lim: ServoLimits, gears: Gears) -> ServoLimits {
    let counts = gears.stop_hold_ma() / 1000.0 / lim.amps_per_count;
    ServoLimits {
        i_lim: counts.floor().min(lim.i_lim as f64) as u16,
        ..lim
    }
}

/// What cal writes once the operator accepts it.
struct Calibration {
    /// The stops, counts: CONFIG's phys limits and CALIB's pot stops.
    phys: (i32, i32),
    soft: (i32, i32),
    angle_cdeg: (i32, i32),
    gear_ratio_centi: u16,
    gears: Gears,
}

/// Write the calibration, re-COMMIT a LIVE position table against the stops
/// it moved, stamp the set and, with `save`, SAVE.
fn write_calibration<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &Calibration,
    save: bool,
) -> Result<()> {
    let writes: [(Reg, i32); 10] = [
        (config::POS_MIN_PHYS_COUNTS, set.phys.0),
        (config::POS_MAX_PHYS_COUNTS, set.phys.1),
        (config::POS_MIN_SOFT_COUNTS, set.soft.0),
        (config::POS_MAX_SOFT_COUNTS, set.soft.1),
        (calib::RAW_MIN, set.phys.0),
        (calib::RAW_MAX, set.phys.1),
        (calib::ANGLE_MIN_CDEG, set.angle_cdeg.0),
        (calib::ANGLE_MAX_CDEG, set.angle_cdeg.1),
        (calib::GEAR_RATIO_CENTI, set.gear_ratio_centi as i32),
        (calib::GEAR_CLASS, set.gears.class().byte() as i32),
    ];
    for (reg, value) in writes {
        write_reg(c, id, reg, value)?;
    }

    // The servo validates a table only at COMMIT and at boot: without this
    // a LIVE table keeps running against the old stops, the stamp below
    // blesses it, and the reboot that fails it lands STAMP_MISMATCH.
    if c.pos_lut_state(id, d)? == pos_lut::state::LIVE {
        match c.recommit_pos_lut(id, d)? {
            pos_lut::state::LIVE => println!("position table: still fits the new stops, LIVE"),
            s => println!(
                "position table: no longer fits the new stops ({}); the kernel runs the identity - `osc lut build`, `osc lut write` and `osc ident` put a table back",
                pos_lut::state::name(s).map_or(format!("state {s}"), String::from)
            ),
        }
    }
    crate::state::commit(c, id, d, save)?;
    Ok(())
}

/// A plausible pot reading: the ADC is 12-bit, so a valid pos is 0..=4095.
/// Anything larger is bit-corrupted.
fn pos_plausible(pos: u16) -> bool {
    pos <= 4095
}

/// Time-aligned (pos, current) sweep from the captured frames: the longest
/// tick-contiguous run. Splitting on tick holes is the ONLY selection - it
/// keeps the sample spacing uniform, which the ripple autocorr assumes. Pos
/// is NOT required to be monotonic: the commutation ripple lives on the
/// winding current, so a slipping stripped gear (pos jittering while the
/// motor spins) still carries a clean signal, and lut::build sorts by pos
/// regardless. A flat stall segment self-rejects downstream (no ripple ->
/// None). None when nothing was captured.
fn build_sweep(tel: &[TelFrame]) -> Option<(Vec<u16>, Vec<f64>)> {
    // Drop value-domain corruption: pos > 4095 is impossible from a 12-bit
    // ADC, so a set bit above b11 is a bit error (the stream is CRC-clean,
    // but recorded CSVs from older captures replay through here too).
    // Dropping the frame removes its tick, which splits the contiguous run
    // at that point and isolates the corruption.
    let s: Vec<(u64, u16, f64)> = tel
        .iter()
        .filter_map(|f| {
            let pos = f.pos?;
            pos_plausible(pos).then_some((f.tick, pos, f.current? as f64))
        })
        .collect();
    let (lo, hi) = longest_contiguous_run(&s)?;
    // trim fused rail dwell here too: the slip check reads this run, and a
    // pinned-pot buzz segment reads as a stuck (slipping) gear segment.
    let run = trim_edge_dwell(&s[lo..hi]);
    Some((
        run.iter().map(|x| x.1).collect(),
        run.iter().map(|x| x.2).collect(),
    ))
}

/// A chunk shorter than this is too short to carry usable ripple and is almost
/// certainly a corruption fragment, so it is dropped from the stitch.
const MIN_CHUNK_SAMPLES: usize = 200;

/// A chunk whose pos span is below this is a STALL, not a sweep segment: the
/// open-loop constant-duty traverse can overshoot into a rail and buzz there
/// with pos frozen while commutation ripple keeps accruing revs. Such a chunk
/// has many samples but ~0 pos travel, so its revs/count is nonsense (spikes
/// the stitched anchor and dumps a bogus rail correction); drop it by span.
const MIN_CHUNK_SPAN: u16 = 50;

/// A moving run can also carry a rail dwell FUSED to its ends (too much travel
/// for the span gate above to catch): the traverse overshoots into an end-stop
/// and buzzes there seq-contiguously with the sweep, pot pinned while the motor
/// keeps turning. Those samples accrue ripple revs with ~zero pot travel and
/// poison the per-count density (bench: 270 ms of max-rail buzz fused to one
/// chunk deposited 71 garbage revs into the top bins - travel read 340 deg on a
/// ~225 deg servo, and the pinned-pot segment faked a gear-slip warning). Trim
/// each end at the sweep's first crossing out of / into the band this many
/// counts from that end's extreme pos.
const EDGE_DWELL_COUNTS: u16 = 8;

/// Trim rail dwell fused to a run's ends (see EDGE_DWELL_COUNTS): keep from the
/// first sample past the head-extreme band to the first sample reaching the
/// tail-extreme band. First-crossing on both ends is robust to buzz wiggle and
/// mid-run jitter; on a clean rail-to-rail sweep it costs only the outermost
/// EDGE_DWELL_COUNTS counts of each end. Runs narrower than the two bands are
/// returned unchanged.
fn trim_edge_dwell(run: &[(u64, u16, f64)]) -> &[(u64, u16, f64)] {
    let (mut lo, mut hi) = (run[0].1, run[0].1);
    for x in run {
        lo = lo.min(x.1);
        hi = hi.max(x.1);
    }
    if hi - lo <= 2 * EDGE_DWELL_COUNTS {
        return run;
    }
    let up = run[run.len() - 1].1 >= run[0].1;
    let past_head = |p: u16| {
        if up {
            p > lo + EDGE_DWELL_COUNTS
        } else {
            p < hi - EDGE_DWELL_COUNTS
        }
    };
    let at_tail = |p: u16| {
        if up {
            p >= hi - EDGE_DWELL_COUNTS
        } else {
            p <= lo + EDGE_DWELL_COUNTS
        }
    };
    let start = run.iter().position(|x| past_head(x.1)).unwrap_or(0);
    let end = run
        .iter()
        .position(|x| at_tail(x.1))
        .map(|i| i + 1)
        .unwrap_or(run.len());
    &run[start..end.max(start + 1)]
}

/// ALL clean sweep chunks (not just the longest): each maximal run of
/// tick-contiguous, in-range (pos<=4095) frames, as (pos, current). Chunks
/// shorter than MIN_CHUNK_SAMPLES are dropped as corruption fragments. The
/// stitch (lut::build_multi) reassembles them over the shared pos axis.
pub(super) fn build_sweep_chunks(tel: &[TelFrame]) -> Vec<(Vec<u16>, Vec<f64>)> {
    // Same filter as build_sweep: a dropped (corrupt/absent) frame removes
    // its tick, so the run splits there just as a 16-tick hole would.
    let s: Vec<(u64, u16, f64)> = tel
        .iter()
        .filter_map(|f| {
            let pos = f.pos?;
            pos_plausible(pos).then_some((f.tick, pos, f.current? as f64))
        })
        .collect();
    let mut chunks = Vec::new();
    if s.is_empty() {
        return chunks;
    }
    let mut lo = 0usize;
    for i in 1..s.len() {
        if s[i].0 != s[i - 1].0 + 1 {
            emit_chunk(&s[lo..i], &mut chunks);
            lo = i;
        }
    }
    emit_chunk(&s[lo..], &mut chunks);
    chunks
}

/// Push one maximal seq-contiguous run as a (pos, current) chunk, dropping it
/// when shorter than MIN_CHUNK_SAMPLES or when its pos span is below
/// MIN_CHUNK_SPAN (a rail stall, not a sweep segment). Rail dwell fused to the
/// ends is trimmed first; the length gate re-applies to the trimmed run.
fn emit_chunk(run: &[(u64, u16, f64)], chunks: &mut Vec<(Vec<u16>, Vec<f64>)>) {
    if run.len() < MIN_CHUNK_SAMPLES {
        return;
    }
    let (mut lo, mut hi) = (run[0].1, run[0].1);
    for x in run {
        lo = lo.min(x.1);
        hi = hi.max(x.1);
    }
    if hi - lo < MIN_CHUNK_SPAN {
        return;
    }
    let run = trim_edge_dwell(run);
    if run.len() >= MIN_CHUNK_SAMPLES {
        chunks.push((
            run.iter().map(|x| x.1).collect(),
            run.iter().map(|x| x.2).collect(),
        ));
    }
}

/// Longest half-open index range `[lo, hi)` of samples whose tick increments
/// by exactly one each step (no dropped frames). None when nothing has >= 2
/// contiguous samples.
fn longest_contiguous_run(s: &[(u64, u16, f64)]) -> Option<(usize, usize)> {
    if s.len() < 2 {
        return None;
    }
    let (mut best_lo, mut best_hi) = (0usize, 0usize);
    let mut lo = 0usize;
    for i in 1..s.len() {
        if s[i].0 != s[i - 1].0 + 1 {
            if i - lo > best_hi - best_lo {
                (best_lo, best_hi) = (lo, i);
            }
            lo = i;
        }
    }
    if s.len() - lo > best_hi - best_lo {
        (best_lo, best_hi) = (lo, s.len());
    }
    (best_hi - best_lo >= 2).then_some((best_lo, best_hi))
}

fn read_sense<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<SenseParams> {
    Ok(SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: read_u16(c, id, calib::TICK_HZ)?,
    })
}

fn pct(duty: f64) -> String {
    format!("{:.1}%", duty * 100.0)
}

/// What cal's run found: the stops, and the traverse's frames.
struct Found {
    stops: EndstopResult,
    tel: Vec<TelFrame>,
    ccw_end: CcwEnd,
}

/// Cal's run, in the order osc-ident's [`Run::for_cal`] names: bias, the
/// jam check and the burst for R at mid travel, the stops, the ripple
/// traverse, and back to mid travel. The first abort ends it, torque off;
/// a refusal ends it before anything drives at a stop. Of config, calib
/// and the position table only the drive polarity is written here: when
/// the jam check sees the motor turn against it, and once both stops are
/// found.
fn find_stops<S: Servo>(
    s: &mut S,
    out: &OutDir,
    lim: ServoLimits,
    stop_lim: ServoLimits,
    sc: Scales,
    tick_hz: f64,
    ccw: Ccw,
) -> Result<Found> {
    let mut cal = Cal {
        lim,
        stop_lim: stop_lim.i_lim,
        sc,
        tick_hz,
        out,
        known: (lim.calibrated() && lim.raw.0 < lim.raw.1).then_some(lim.raw),
        polarity: lim.drive_polarity,
        stops: None,
        refused: None,
        tel: Vec::new(),
        ccw,
        ccw_end: None,
    };
    let mut run = Run::for_cal(stop_lim, &sc);
    while let Some(stage) = run.next_stage() {
        match cal.stage(&stage, &mut run, s) {
            Ok(how) => run.ended(how),
            Err(e) => {
                if let Some(a) = e.downcast_ref::<Aborted>() {
                    run.ended(Ended::Aborted(a.reason));
                }
                return Err(e);
            }
        }
    }
    match run.over() {
        None => {}
        Some(Over::BreakawayOverLimit { need }) => bail!(
            "{}",
            Refusal::BreakawayOverLimit {
                need,
                i_lim: stop_lim.i_lim,
                ma: lim.ma(),
            }
        ),
        Some(Over::Declined(_)) => bail!(
            "{}; no writes",
            cal.refused.as_deref().unwrap_or("the stops were not found")
        ),
        Some(Over::Unproven) => {
            bail!("the jam check never saw the shaft move, so cal cannot look for the stops")
        }
        Some(other) => bail!("cal ended early ({other:?}); no writes"),
    }
    let stops = cal.stops.context("cal found no stops; no writes")?;
    let ccw_end = cal
        .ccw_end
        .context("the counterclockwise end was not settled; no writes")?;
    Ok(Found {
        stops,
        tel: cal.tel,
        ccw_end,
    })
}

struct Cal<'a> {
    lim: ServoLimits,
    /// The current limit the stops are planned under, counts.
    stop_lim: u16,
    sc: Scales,
    tick_hz: f64,
    out: &'a OutDir,
    /// The stops an earlier cal found; None on a servo never calibrated.
    known: Option<(u16, u16)>,
    /// The drive polarity in force.
    polarity: bool,
    stops: Option<EndstopResult>,
    /// Why the stops found are no stops.
    refused: Option<String>,
    tel: Vec<TelFrame>,
    ccw: Ccw,
    /// The count end at the most-counterclockwise stop, once settled at a
    /// stop.
    ccw_end: Option<CcwEnd>,
}

impl Cal<'_> {
    /// The rig before this run found the stops: the soft limits' guard and
    /// the stops of an earlier cal, when there was one.
    fn params(&self) -> RigParams {
        let p = RigParams::new(self.lim.guard().ok(), self.lim.abort_default())
            .with_floor(self.lim.window_floor_q15)
            .with_polarity(self.polarity);
        match self.known {
            Some(s) => p.with_stops(s),
            None => p,
        }
    }

    fn found(&self) -> Option<(u16, u16)> {
        self.stops
            .map(|r| (r.pos_min_phys as u16, r.pos_max_phys as u16))
    }

    fn stage<S: Servo>(&mut self, stage: &Stage, run: &mut Run, s: &mut S) -> Result<Ended> {
        match stage {
            Stage::Bias => {
                println!("[bias] torque off: the current zero and the position noise");
                let p = self.params().without_pos_guard();
                let mut exp = Guarded::new(Bias::new(BiasCfg::default(), &p), p);
                drive(s, self.out, &mut exp, None)?;
                check_abort("bias", exp.abort())?;
            }
            Stage::JamCheck => {
                let exp = jam_check(s, &self.lim, &self.sc, self.params())?;
                self.polarity = exp.polarity();
                run.nudged(exp.moved_at());
            }
            Stage::Centre { duty, cap } => {
                println!("[centring] back to mid travel at {}", pct(*duty));
                let p = match self.found() {
                    Some(s) => RigParams::new(Some(s), self.lim.abort_default())
                        .with_floor(self.lim.window_floor_q15)
                        .with_stops(s),
                    None => self.params(),
                };
                let cfg = order::centre_cfg(*duty, *cap);
                centre_on(|e| drive_on(s, e), cfg, p, "centring")?;
            }
            Stage::Burst { rungs, pre, seek } => {
                let base = InductanceCfg {
                    repeats: 5,
                    i_max_a: BurstAllowance::i_max_a(),
                    ..InductanceCfg::default()
                };
                let cfg = order::burst_cfg(rungs, *pre, *seek, base);
                println!(
                    "[burst] winding R from short bursts at mid travel at {}",
                    cfg.step_pct
                        .iter()
                        .map(|p| format!("{p}%"))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                let p = self.params();
                let mut exp = Guarded::new(Inductance::new(cfg, &p, self.sc), p);
                drive(s, self.out, &mut exp, None)?;
                check_abort("the burst", exp.abort())?;
                let exp = exp.into_inner();
                for w in exp.warnings() {
                    println!("  warn: {w}");
                }
                let caps = exp.captures().to_vec();
                csvio::write_bursts(self.out, &caps)?;
                let at =
                    FitCfg::default().with_limit(self.lim.i_lim as f64 * self.sc.amps_per_count);
                let fit = fit_captures(&caps, &self.sc, &at);
                let w =
                    sources::winding(fit.as_ref(), None, Some(&self.sc), gains::DEFAULT_L_HENRIES);
                let Some(w) = w else {
                    println!(
                        "  the burst measured no winding R ({}): the stops are taken without it",
                        fit.as_ref()
                            .map_or("no burst could be fitted".into(), |r| r.reason())
                    );
                    return Ok(Ended::Declined);
                };
                match w.r_ohm {
                    Some(r) => println!("  winding V/I at the current limit {r:.2} ohm"),
                    None => println!("  winding R {:.4} vcounts/ccount", w.r_vpc),
                }
                run.measured(w.r_vpc);
            }
            Stage::Stops {
                approach,
                seat,
                cap,
            } => return self.stops(run, s, (*approach, *seat, *cap)),
            Stage::Traverse { seek } => return self.traverse(run, s, *seek),
            Stage::Resistance { .. }
            | Stage::Breakaway { .. }
            | Stage::Ladder { .. }
            | Stage::Inertia { .. }
            | Stage::Anchor { .. } => bail!("osc cal has no {} stage", stage.name()),
        }
        Ok(Ended::Done)
    }

    /// Both stops, approached at one duty and read seated at half the
    /// limit, stall permit held; then the polarity they show.
    fn stops<S: Servo>(
        &mut self,
        run: &Run,
        s: &mut S,
        (approach, seat, cap): (f64, f64, f64),
    ) -> Result<Ended> {
        let ma = self.lim.ma();
        match run.plan() {
            Some(_) => println!(
                "[stops] each stop approached at {}, then held at {} while it is read: half the \
                 stop-hold limit of {}; stall permit held",
                pct(approach),
                pct(seat),
                ma.of(self.stop_lim as f64)
            ),
            None => println!(
                "[stops] taken without winding R: each stop approached at {}, the duty that \
                 moved the shaft, then held at {} while it is read: half the class-safe duty; \
                 stall permit held",
                pct(approach),
                pct(seat)
            ),
        }
        let p = self.params().without_pos_guard();
        let exp = Endstop::new(order::endstop_cfg(approach, seat, cap), &p);
        let mut exp = Guarded::new(Permitted::new(exp), p);
        drive(s, self.out, &mut exp, Some("endstop_snapshots.csv"))?;
        check_abort("the stop approach", exp.abort())?;
        let exp = exp.into_inner().into_inner();
        if exp.approach() > approach + 1e-4 {
            println!(
                "  a sticky spot on the way raised the approach to {}",
                pct(exp.approach())
            );
        }
        let Some(r) = exp.result() else {
            self.refused = Some("the stop approach did not read both stops".into());
            return Ok(Ended::Declined);
        };
        if let Some(why) = r.refusal() {
            self.refused = Some(why);
            return Ok(Ended::Declined);
        }
        // The run drove under the polarity in force: a positive duty that
        // raised the counts confirms it.
        let polarity = r.drive_polarity == self.polarity;
        println!(
            "rails: min {} max {} span {} counts, polarity {}, stall {}",
            r.pos_min_phys,
            r.pos_max_phys,
            r.pos_max_phys - r.pos_min_phys,
            if polarity { "normal" } else { "reversed" },
            ma.of(r.i_stall_counts as f64)
        );
        // Written before the operator confirms so the traverse and the
        // centring drive in the logical convention; RAM only until SAVE,
        // and the value is the physical truth whether or not the rest is
        // accepted.
        if polarity != self.polarity {
            s.write(config::DRIVE_POLARITY, polarity as i32)?;
            self.polarity = polarity;
        }
        let rest = s.snapshot()?.pos;
        self.ccw_end = Some(self.ccw.settle(rest, (r.pos_min_phys, r.pos_max_phys))?);
        self.stops = Some(r);
        Ok(Ended::Done)
    }

    /// The ripple traverse inside the stops just found, on the runway rule.
    /// The decoded frames land in sweep_tel.csv - cal-replay's input.
    fn traverse<S: Servo>(&mut self, run: &Run, s: &mut S, seek: f64) -> Result<Ended> {
        let Some(stops) = self.found() else {
            return Ok(Ended::Declined);
        };
        let soft = self
            .lim
            .calibrated()
            .then_some((self.lim.soft.0 as u16, self.lim.soft.1 as u16));
        let mut runway = Runway::new(traverse::guard(stops, soft));
        let cfg = order::sweep_cfg(
            seek,
            SweepCfg {
                tick_hz: self.tick_hz,
                ..SweepCfg::default()
            },
        );
        println!(
            "[traverse] the ripple capture at {} from the low end to the high end, braking \
             inside the stops; first a pass to the low end at {}",
            pct(cfg.duty_q15 as f64 / 32767.0),
            pct(seek)
        );
        let phys = (stops.0 as i32, stops.1 as i32);
        let (left, sized) =
            envelope::size_runway(&mut runway, Supply::of_rail(run.rail_mv()), phys);
        for (path, why) in left {
            println!("  left out {}: {why}", path.display());
        }
        match sized {
            Some(path) => println!("  sized by the pilot envelope {}", path.display()),
            None => println!(
                "  no pilot envelope describes this servo on this supply: sized from the first \
                 pass"
            ),
        }
        let p = RigParams::new(None, self.lim.abort_default())
            .with_floor(self.lim.window_floor_q15)
            .with_stops(stops);
        let mut exp = Guarded::new(Sweep::new(cfg, &p, runway), p);
        let tel = drive(s, self.out, &mut exp, None)?;
        check_abort("the traverse", exp.abort())?;
        let exp = exp.into_inner();
        csvio::write_tel_frames(self.out, "sweep_tel.csv", &tel)?;
        println!(
            "[sweep] decoded frames -> {}",
            self.out.0.join("sweep_tel.csv").display()
        );
        self.tel = tel;
        if let Some(why) = exp.declined() {
            println!("  no ripple captured: {why}");
            return Ok(Ended::Declined);
        }
        if let (Some(cap), Some(rest)) = (exp.captured(), exp.rest()) {
            println!(
                "  captured {} samples in {} bursts over counts {}..{}, braked to rest at {rest}",
                cap.samples,
                cap.bursts,
                cap.from,
                cap.to.unwrap_or(cap.from)
            );
        }
        Ok(Ended::Done)
    }
}

/// Pump `exp` to its end and park the servo safe however it ends: the TEL
/// frames it streamed. `log` names a snapshot CSV in `out`.
fn drive<S: Servo>(
    s: &mut S,
    out: &OutDir,
    exp: &mut dyn Experiment,
    log: Option<&str>,
) -> Result<Vec<TelFrame>> {
    let mut log = log.map(|n| SnapshotLog::create(out, n)).transpose()?;
    guard(s, |s| {
        let mut pump = Pump::on(s, log.as_mut());
        pump.run(exp)?;
        Ok(std::mem::take(&mut pump.tel))
    })
}

/// Phys angle at each rail: from flags under `--yes` (both required), else
/// prompted with the flags or 0..DEFAULT_TRAVEL_DEG as defaults.
fn phys_angles(args: &Args) -> Result<(f64, f64)> {
    if args.yes {
        let min = args
            .phys_angle_min
            .context("--yes needs --phys-angle-min")?;
        let max = args
            .phys_angle_max
            .context("--yes needs --phys-angle-max")?;
        return Ok((min, max));
    }
    let min = input_f64(
        "angle at the most-clockwise stop (deg)",
        args.phys_angle_min.unwrap_or(0.0),
    )?;
    let max = input_f64(
        "angle at the most-counterclockwise stop (deg)",
        args.phys_angle_max.unwrap_or(DEFAULT_TRAVEL_DEG),
    )?;
    Ok((min, max))
}

/// Soft (working-range) angle: flags under `--yes`, else prompted with the
/// flags as defaults; each unset end defaults to [`SOFT_MARGIN_FRAC`] of the
/// travel inside its stop.
fn soft_angles(args: &Args, phys_min: f64, phys_max: f64) -> Result<(f64, f64)> {
    let inset = (phys_max - phys_min) * SOFT_MARGIN_FRAC;
    let (dmin, dmax) = (phys_min + inset, phys_max - inset);
    if args.yes {
        return Ok((
            args.soft_angle_min.unwrap_or(dmin),
            args.soft_angle_max.unwrap_or(dmax),
        ));
    }
    let min = input_f64("soft angle min (deg)", args.soft_angle_min.unwrap_or(dmin))?;
    let max = input_f64("soft angle max (deg)", args.soft_angle_max.unwrap_or(dmax))?;
    Ok((min, max))
}

/// Resolve the headless (`--yes`) anchor to (angle_at_min, angle_at_max,
/// gear_centi). GEAR is the preferred anchor: given a `--gear-ratio` flag and a
/// ripple-derived `motor_revs_full`, the physical travel is derived from the
/// gear (pure geometry, protocol-free) and the counted gear stored as-is;
/// angle_at_min comes from `--phys-angle-min` (0 default), angle_at_max is that
/// plus the derived travel. Falling back: explicit `--phys-angle-min/max` flags
/// anchor on travel and the gear is derived from ripple (the inverse) - though a
/// counted `--gear-ratio` flag, when present, still wins for storage (matching
/// the pre-inverse precedence). Neither anchor -> the 0/0/0 unset sentinels.
fn resolve_anchor(
    motor_revs_full: Option<f64>,
    gear_flag: Option<f64>,
    phys_min_flag: Option<f64>,
    phys_max_flag: Option<f64>,
) -> (f64, f64, u16) {
    if let (Some(g), Some(m)) = (gear_flag, motor_revs_full)
        && g > 0.0
    {
        let travel = kinematics::travel_deg_from_gear(m, g);
        let angle_min = phys_min_flag.unwrap_or(0.0);
        return (angle_min, angle_min + travel, ratio_to_centi(Some(g)));
    }
    if let (Some(lo), Some(hi)) = (phys_min_flag, phys_max_flag) {
        let ripple_centi = motor_revs_full
            .map(|m| ratio_to_centi(travel_to_gear(m, hi - lo)))
            .unwrap_or(0);
        let gear_centi = match gear_flag {
            Some(g) if g > 0.0 => ratio_to_centi(Some(g)),
            _ => ripple_centi,
        };
        return (lo, hi, gear_centi);
    }
    (0.0, 0.0, 0)
}

/// Print the headless anchor outcome. Gear anchor -> the derived travel,
/// checked against the travel flags when they are given. Travel anchor with a
/// ripple-derived gear -> the ripple info line.
fn report_yes_anchor(
    args: &Args,
    mrf: Option<f64>,
    coverage: Option<f64>,
    angle_min: f64,
    angle_max: f64,
    gear_centi: u16,
) -> Result<()> {
    let gear_flag = args.gear_ratio.filter(|&g| g > 0.0);
    if gear_flag.is_some() && mrf.is_some() {
        let travel = angle_max - angle_min;
        println!(
            "derived travel {travel:.1} deg from gear {:.2}",
            gear_centi as f64 / 100.0,
        );
        check_travel(args, travel)?;
    } else if gear_flag.is_none() && gear_centi != 0 {
        print_ripple_gear(gear_centi, coverage.unwrap_or(0.0));
    }
    Ok(())
}

/// Relative disagreement between the derived travel and the operator's over
/// which cal writes nothing.
const TRAVEL_TOL: f64 = 0.1;

/// `--phys-angle-min/max` given beside a gear anchor cross-check the travel
/// it derives: when they disagree, the gear, the flags or the ripple count is
/// wrong (a mistyped flag, a slipping gear, a miscounted ripple line), and
/// none of them can be written.
fn check_travel(args: &Args, derived: f64) -> Result<()> {
    let (Some(lo), Some(hi)) = (args.phys_angle_min, args.phys_angle_max) else {
        return Ok(());
    };
    let given = hi - lo;
    match (derived / given - 1.0).abs() {
        off if off <= TRAVEL_TOL => Ok(()),
        _ => bail!(
            "the derived travel {derived:.1} deg disagrees with --phys-angle-min/max ({given:.1} \
             deg) by more than {:.0}%: a wrong flag, a slipping gear or a miscounted ripple; no \
             writes",
            TRAVEL_TOL * 100.0
        ),
    }
}

/// Interactive anchor resolution, nudged toward the gear. Prompts the gear ratio
/// first: a positive gear with a ripple-derived `motor_revs_full` derives travel
/// (gear anchor); entering 0, or having no ripple, falls back to the travel-angle
/// prompts and derives the gear from ripple (the inverse). With no ripple at all,
/// an entered gear still stands so a known ratio can be stored without a sweep.
fn resolve_anchor_interactive(
    args: &Args,
    mrf: Option<f64>,
    coverage: Option<f64>,
) -> Result<(f64, f64, u16)> {
    let gear = input_f64(
        "gear ratio (motor rev per output rev) - most accurate if you counted the teeth; 0 to enter travel angle instead",
        args.gear_ratio.unwrap_or(1.0),
    )?;
    if gear > 0.0
        && let Some(m) = mrf
    {
        let travel = kinematics::travel_deg_from_gear(m, gear);
        let angle_min = input_f64(
            "angle at the most-clockwise stop (deg)",
            args.phys_angle_min.unwrap_or(0.0),
        )?;
        println!("derived travel {travel:.1} deg from gear {gear:.2}");
        check_travel(args, travel)?;
        return Ok((angle_min, angle_min + travel, ratio_to_centi(Some(gear))));
    }
    let (lo, hi) = phys_angles(args)?;
    let gear_centi = match mrf {
        Some(m) => {
            let centi = ratio_to_centi(travel_to_gear(m, hi - lo));
            if centi != 0 {
                print_ripple_gear(centi, coverage.unwrap_or(0.0));
            }
            centi
        }
        None => ratio_to_centi((gear > 0.0).then_some(gear)),
    };
    Ok((lo, hi, gear_centi))
}

/// Ripple gear ratio (motor rev per output rev) implied by the full-traverse
/// motor revs and an operator travel angle - the inverse of
/// kinematics::travel_deg_from_gear. None on a zero/non-finite travel, so
/// ratio_to_centi then stores the 0 sentinel.
fn travel_to_gear(motor_revs_full: f64, travel_deg: f64) -> Option<f64> {
    (travel_deg.is_finite() && travel_deg > 0.0).then_some(motor_revs_full * 360.0 / travel_deg)
}

/// The ripple-derived gear info line (the historical "measured gear ratio ..."
/// print), shown when the travel anchor is used.
fn print_ripple_gear(gear_centi: u16, coverage: f64) {
    let conf = if coverage < RIPPLE_CONF_MIN {
        " (low confidence)"
    } else {
        ""
    };
    println!(
        "measured gear ratio {:.2}, ripple-derived ({:.0}% coverage){conf}",
        gear_centi as f64 / 100.0,
        coverage * 100.0,
    );
}

fn ratio_to_centi(ratio: Option<f64>) -> u16 {
    match ratio {
        Some(r) if r.is_finite() && r > 0.0 => {
            (r * 100.0).round().clamp(0.0, u16::MAX as f64) as u16
        }
        _ => 0,
    }
}

fn soft_to_count(angle: f64, phys_min: f64, phys_max: f64, pos_min: i32, pos_max: i32) -> i32 {
    let denom = phys_max - phys_min;
    let frac = if denom == 0.0 {
        0.0
    } else {
        (angle - phys_min) / denom
    };
    let count = pos_min as f64 + frac * (pos_max - pos_min) as f64;
    let (lo, hi) = (pos_min.min(pos_max), pos_min.max(pos_max));
    (count.round() as i32).clamp(lo, hi)
}

fn confirm(prompt: &str, default: bool) -> Result<bool> {
    Ok(Confirm::new()
        .with_prompt(prompt)
        .default(default)
        .interact()?)
}

fn input_f64(prompt: &str, default: f64) -> Result<f64> {
    Ok(Input::<f64>::new()
        .with_prompt(prompt)
        .default(default)
        .interact_text()?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_ident::runway::dead_host_fits;

    #[test]
    fn soft_maps_and_clamps_within_rails() {
        // 0..180 deg over counts 100..4000
        assert_eq!(soft_to_count(0.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(180.0, 0.0, 180.0, 100, 4000), 4000);
        assert_eq!(soft_to_count(90.0, 0.0, 180.0, 100, 4000), 2050);
        // tighter-than-phys stays in band; out-of-range clamps to a rail
        assert_eq!(soft_to_count(-10.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(200.0, 0.0, 180.0, 100, 4000), 4000);
    }

    #[test]
    fn soft_degenerate_phys_span_is_min_rail() {
        assert_eq!(soft_to_count(50.0, 90.0, 90.0, 100, 4000), 100);
    }

    fn soft_flags(lo: Option<f64>, hi: Option<f64>) -> Args {
        Args {
            soft_angle_min: lo,
            soft_angle_max: hi,
            ..travel_flags(Some(0.0), Some(183.5))
        }
    }

    /// The rev 2A MG90's stops, 283/3829 over 183.5 deg: the default soft
    /// limits leave each end room for the 2S braked stop of a 30% rung,
    /// about 95 counts, where a centered 180 deg window left 34.
    #[test]
    fn default_soft_limits_leave_room_to_brake() {
        let (lo, hi) = soft_angles(&soft_flags(None, None), 0.0, 183.5).unwrap();
        let soft = (
            soft_to_count(lo, 0.0, 183.5, 283, 3829),
            soft_to_count(hi, 0.0, 183.5, 283, 3829),
        );
        assert_eq!(soft, (505, 3607));
        let margins = ((soft.0 - 283) as f64, (3829 - soft.1) as f64);
        assert!(margins.0 >= 200.0 && margins.1 >= 200.0, "{margins:?}");
        assert!(dead_host_fits(95.0, margins.0) && dead_host_fits(95.0, margins.1));
    }

    #[test]
    fn soft_angle_flags_override_the_default() {
        let both = soft_angles(&soft_flags(Some(5.0), Some(178.0)), 0.0, 183.5).unwrap();
        assert_eq!(both, (5.0, 178.0));
        let high = soft_angles(&soft_flags(None, Some(178.0)), 0.0, 183.5).unwrap();
        assert_eq!(high, (183.5 * SOFT_MARGIN_FRAC, 178.0));
    }

    /// Contiguous-tick samples from a pos list (current = pos for simplicity).
    fn ticked(pos: &[u16]) -> Vec<(u64, u16, f64)> {
        pos.iter()
            .enumerate()
            .map(|(i, &p)| (i as u64, p, p as f64))
            .collect()
    }

    #[test]
    fn contiguous_run_splits_on_tick_hole_and_picks_longest() {
        // 30 samples, tick hole at index 20: the longer half [0,20) wins, and
        // pos need not be monotonic (slip is fine - current carries ripple).
        let mut s = ticked(&(0..30).map(|k| 1000 + (k % 3) * 10).collect::<Vec<_>>());
        for e in s.iter_mut().skip(20) {
            e.0 += 16; // a dropped frame's 16-tick hole at 20
        }
        assert_eq!(longest_contiguous_run(&s), Some((0, 20)));
    }

    #[test]
    fn contiguous_run_none_when_too_short() {
        assert!(longest_contiguous_run(&[]).is_none());
        assert!(longest_contiguous_run(&ticked(&[7])).is_none());
    }

    #[test]
    fn build_sweep_none_without_frames() {
        assert!(build_sweep(&[]).is_none());
    }

    #[test]
    fn pos_plausible_rejects_above_12bit() {
        assert!(pos_plausible(0));
        assert!(pos_plausible(4095));
        assert!(!pos_plausible(4096));
        assert!(!pos_plausible(9999));
    }

    #[test]
    fn build_sweep_drops_out_of_range_pos() {
        // ticks 0..10, all pos in-range except tick 5 which is bit-corrupt
        // (>4095, an old-capture replay). Dropping it removes its tick, so the
        // contiguous run splits there: the longer half is ticks 0..=4 (5
        // samples) and the corrupt pos is excluded.
        let frames: Vec<TelFrame> = (0..10u64)
            .map(|k| TelFrame {
                tick: k,
                pos: Some(if k == 5 { 9999 } else { 1000 + k as u16 }),
                current: Some(50),
                ..Default::default()
            })
            .collect();
        let (pos, current) = build_sweep(&frames).expect("a run survives");
        assert!(
            pos.iter().all(|&p| p <= 4095),
            "corrupt pos leaked: {pos:?}"
        );
        assert!(!pos.contains(&9999));
        assert_eq!(pos, vec![1000, 1001, 1002, 1003, 1004]);
        assert_eq!(current.len(), pos.len());
    }

    #[test]
    fn build_sweep_chunks_keeps_big_runs_drops_fragments_and_stalls() {
        // tick = index. Moving runs (pos sweeps a real span) split by
        // out-of-range pos, plus a trailing short fragment, a long STALL run
        // (pos frozen at a rail), and a moving run with a rail buzz FUSED to
        // its tail. Kept: the moving runs, edge-trimmed by EDGE_DWELL_COUNTS;
        // the fused buzz tail is trimmed off entirely.
        let mut frames: Vec<TelFrame> = Vec::new();
        let mut push = |base: u16, count: u32, moving: bool| {
            for j in 0..count {
                let tick = frames.len() as u64;
                let pos = if moving { base + j as u16 } else { base };
                frames.push(TelFrame {
                    tick,
                    pos: Some(pos),
                    current: Some(50),
                    ..Default::default()
                });
            }
        };
        push(1000, 250, true); // run A: span 249, kept (trimmed)
        push(9999, 1, true); // split (out-of-range)
        push(2000, 300, true); // run B: span 299, kept (trimmed)
        push(9999, 1, true); // split
        push(3000, 50, true); // fragment: < MIN_CHUNK_SAMPLES, dropped
        push(9999, 1, true); // split
        push(4095, 250, false); // stall: >= samples but span 0, dropped
        push(9999, 1, true); // split
        push(3600, 300, true); // run C sweeps 3600..3899 ...
        push(3899, 400, false); // ... then buzzes at the rail seq-contiguously

        let chunks = build_sweep_chunks(&frames);
        assert_eq!(chunks.len(), 3, "the three moving runs survive");
        // each end trimmed at the first crossing out of the extreme band:
        // head loses EDGE_DWELL_COUNTS+1 samples, tail keeps through the first
        // sample inside the band (1 count/sample here)
        let trimmed = |n: usize| n - (2 * EDGE_DWELL_COUNTS as usize + 1);
        assert_eq!(chunks[0].0.len(), trimmed(250));
        assert_eq!(chunks[1].0.len(), trimmed(300));
        assert_eq!(chunks[0].1.len(), trimmed(250));
        // the stall run (span 0) is gone despite having >= MIN_CHUNK_SAMPLES
        assert!(chunks.iter().all(|(p, _)| p.iter().max() != Some(&4095)));
        // run C: the 400-sample fused buzz is trimmed off with the tail band
        assert_eq!(chunks[2].0.len(), trimmed(300));
        assert!(chunks[2].0.iter().max() < Some(&3899));
    }

    #[test]
    fn build_sweep_chunks_empty_without_frames() {
        assert!(build_sweep_chunks(&[]).is_empty());
    }

    fn travel_flags(lo: Option<f64>, hi: Option<f64>) -> Args {
        Args {
            out: None,
            phys_angle_min: lo,
            phys_angle_max: hi,
            soft_angle_min: None,
            soft_angle_max: None,
            gear_ratio: Some(308.05),
            gears: None,
            ccw_end: None,
            yes: true,
            no_save: false,
        }
    }

    /// The rev-2A cal: --gear-ratio 308.05 with --phys-angle 0..184.22, and
    /// the ripple read at half its line, deriving 91.1 deg. The flags refuse
    /// it; the travel the line itself derives passes.
    #[test]
    fn travel_flags_refuse_a_derived_travel_they_disagree_with() {
        let args = travel_flags(Some(0.0), Some(184.22));
        let err = check_travel(&args, 91.1).unwrap_err().to_string();
        assert!(
            err.contains("91.1 deg") && err.contains("184.2 deg"),
            "{err}"
        );
        assert!(check_travel(&args, 182.2).is_ok());
        assert!(check_travel(&args, 184.22 * 1.11).is_err());
        assert!(check_travel(&args, 184.22 * 0.91).is_ok());
        // a degenerate travel flag never passes; without both flags there is
        // nothing to check
        assert!(check_travel(&travel_flags(Some(90.0), Some(90.0)), 182.2).is_err());
        assert!(check_travel(&travel_flags(Some(0.0), None), 91.1).is_ok());
        assert!(check_travel(&travel_flags(None, None), 91.1).is_ok());
    }

    #[test]
    fn resolve_anchor_gear_flag_derives_travel() {
        // gear 150 + motor_revs_full 79.1667 -> travel ~190 deg; endpoints 0..190
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), None, None);
        assert_eq!(lo, 0.0);
        assert!((hi - 190.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_gear_flag_honors_min_offset() {
        // angle_at_min from --phys-angle-min, angle_at_max = min + derived travel
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), Some(-95.0), None);
        assert_eq!(lo, -95.0);
        assert!((hi - 95.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_travel_flags_derive_gear() {
        // travel 0..190 deg + motor_revs_full 79.1667 -> ripple gear ~150
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), None, Some(0.0), Some(190.0));
        assert_eq!((lo, hi), (0.0, 190.0));
        assert!((gear as i32 - 15000).abs() <= 2, "gear {gear}");
    }

    #[test]
    fn resolve_anchor_gear_flag_wins_when_both_given() {
        // both a gear flag and travel flags: gear anchor wins, travel derived
        // from the gear (max ignores --phys-angle-max as an endpoint)
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), Some(0.0), Some(999.0));
        assert_eq!(lo, 0.0);
        assert!((hi - 190.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_flag_stored_without_ripple() {
        // travel flags but no ripple: endpoints kept, counted gear flag stored
        assert_eq!(
            resolve_anchor(None, Some(234.73), Some(0.0), Some(190.0)),
            (0.0, 190.0, 23473)
        );
    }

    #[test]
    fn resolve_anchor_unset_sentinels() {
        // neither anchor -> all-zero sentinel
        assert_eq!(resolve_anchor(Some(79.0), None, None, None), (0.0, 0.0, 0));
        // travel flags, no ripple, no gear flag -> gear unset
        assert_eq!(
            resolve_anchor(None, None, Some(0.0), Some(190.0)),
            (0.0, 190.0, 0)
        );
        // gear flag but no ripple and no travel -> nothing to anchor on
        assert_eq!(resolve_anchor(None, Some(150.0), None, None), (0.0, 0.0, 0));
    }

    #[test]
    fn ratio_centi_encoding_and_sentinel() {
        assert_eq!(ratio_to_centi(Some(150.0)), 15000);
        assert_eq!(ratio_to_centi(Some(1.5)), 150);
        assert_eq!(ratio_to_centi(None), 0);
        assert_eq!(ratio_to_centi(Some(0.0)), 0);
        assert_eq!(ratio_to_centi(Some(-3.0)), 0);
        assert_eq!(ratio_to_centi(Some(f64::NAN)), 0);
        assert_eq!(ratio_to_centi(Some(1e9)), u16::MAX);
    }

    const DESCRIPTOR: &str = include_str!("../../../../descriptors/osc-servo/0.1.json");
    const MG90_A: &str = include_str!("../../../../ident/testdata/lut/pos-lut-mg90-a-grid.json");

    fn mg90_a() -> [i16; pos_lut::INTERVALS] {
        let img: serde_json::Value = serde_json::from_str(MG90_A).unwrap();
        let points: Vec<i16> = img["points"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_i64().unwrap() as i16)
            .collect();
        points.try_into().unwrap()
    }

    fn read<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg) -> i32 {
        let raw = c.read(id, reg.addr, reg.width as u16).unwrap();
        match raw.as_slice() {
            [a] => *a as i32,
            [a, b] => i16::from_le_bytes([*a, *b]) as i32,
            [a, b, c, d] => i32::from_le_bytes([*a, *b, *c, *d]),
            r => panic!("{r:?}"),
        }
    }

    /// Re-cal moving the stops under a LIVE position table, on the
    /// in-process servo: the write re-COMMITs the table against the new
    /// stops, then stamps and SAVEs, so the servo reboots clean with the
    /// set as written. Stops inside the table's insets keep it LIVE; a stop
    /// into its covered span rejects it, the saved servo runs the identity,
    /// and the stamp hashes what the kernel applies.
    #[test]
    fn recal_recommits_a_live_position_table_and_restamps() {
        use osc_client::BaudRate;
        use osc_client::fake::FakePipe;

        let d = Descriptor::parse(DESCRIPTOR).unwrap();
        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = Client::connect(pipe).unwrap();
        let id = Id::new(1);
        write_reg(&mut c, id, calib::RAW_MIN, 209).unwrap();
        write_reg(&mut c, id, calib::RAW_MAX, 3849).unwrap();
        c.write_pos_lut(id, &d, &mg90_a()).unwrap();
        c.restamp(id, &d).unwrap();
        assert_eq!(c.data_state(id, &d).unwrap().flags, 0);

        for (lo, state) in [(232, pos_lut::state::LIVE), (600, pos_lut::state::IDENTITY)] {
            let set = Calibration {
                phys: (lo, 3849),
                soft: (lo + 223, 3626),
                angle_cdeg: (0, 18420),
                gear_ratio_centi: 30805,
                gears: Gears::Metal,
            };
            write_calibration(&mut c, id, &d, &set, true).unwrap();
            assert_eq!(c.pos_lut_state(id, &d).unwrap(), state, "{lo}");
            assert_eq!(c.data_state(id, &d).unwrap().flags, 0, "{lo}");
            c.reboot(id).unwrap();
            assert_eq!(c.data_state(id, &d).unwrap().flags, 0, "{lo}: saved");
            assert_eq!(c.pos_lut_state(id, &d).unwrap(), state, "{lo}: saved");
            for (reg, want) in [
                (config::POS_MIN_PHYS_COUNTS, lo),
                (config::POS_MAX_PHYS_COUNTS, 3849),
                (config::POS_MIN_SOFT_COUNTS, lo + 223),
                (config::POS_MAX_SOFT_COUNTS, 3626),
                (calib::RAW_MIN, lo),
                (calib::RAW_MAX, 3849),
                (calib::ANGLE_MIN_CDEG, 0),
                (calib::ANGLE_MAX_CDEG, 18420),
                (calib::GEAR_RATIO_CENTI, 30805),
                (calib::GEAR_CLASS, 2),
            ] {
                assert_eq!(read(&mut c, id, reg), want, "{lo}: {reg:?}");
            }
        }
    }

    /// Without SAVE the set runs stamped until a reboot, which brings the
    /// saved set back.
    #[test]
    fn a_cal_without_save_lasts_until_a_reboot() {
        use osc_client::BaudRate;
        use osc_client::fake::FakePipe;

        let d = Descriptor::parse(DESCRIPTOR).unwrap();
        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = Client::connect(pipe).unwrap();
        let id = Id::new(1);
        let regs = [
            config::POS_MIN_PHYS_COUNTS,
            config::POS_MAX_SOFT_COUNTS,
            calib::RAW_MIN,
            calib::GEAR_RATIO_CENTI,
        ];
        let saved: Vec<i32> = regs.iter().map(|&r| read(&mut c, id, r)).collect();
        let set = Calibration {
            phys: (251, 3811),
            soft: (474, 3588),
            angle_cdeg: (0, 18774),
            gear_ratio_centi: 30805,
            gears: Gears::Metal,
        };
        assert_ne!(saved, [251, 3588, 251, 30805]);
        write_calibration(&mut c, id, &d, &set, false).unwrap();
        let live: Vec<i32> = regs.iter().map(|&r| read(&mut c, id, r)).collect();
        assert_eq!(live, [251, 3588, 251, 30805]);
        c.reboot(id).unwrap();
        let back: Vec<i32> = regs.iter().map(|&r| read(&mut c, id, r)).collect();
        assert_eq!(back, saved);
    }

    /// A servo never calibrated holds no gear class: `--yes` refuses
    /// without `--gears`, before anything drives. The class cal writes is
    /// stamped and saved with the set, so the next `--yes` cal reads it
    /// back and needs no flag.
    #[test]
    fn an_unknown_gear_class_needs_the_flag_and_cal_saves_the_class() {
        use osc_client::BaudRate;
        use osc_client::data_state::STAMP_MISMATCH;
        use osc_client::fake::FakePipe;

        let d = Descriptor::parse(DESCRIPTOR).unwrap();
        let pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        let mut c = Client::connect(pipe).unwrap();
        let id = Id::new(1);
        let stored = c.gear_class(id, &d).unwrap();
        assert_eq!(stored, GearClass::Unknown);
        let headless = travel_flags(Some(0.0), Some(183.5));
        let err = gears(&headless, stored).unwrap_err().to_string();
        assert!(err.contains("--gears"), "{err}");
        let flagged = Args {
            gears: Some(Gears::Plastic),
            ..travel_flags(Some(0.0), Some(183.5))
        };
        assert_eq!(gears(&flagged, stored).unwrap(), Gears::Plastic);

        let set = Calibration {
            phys: (251, 3811),
            soft: (474, 3588),
            angle_cdeg: (0, 18774),
            gear_ratio_centi: 23473,
            gears: Gears::Plastic,
        };
        write_calibration(&mut c, id, &d, &set, true).unwrap();
        c.reboot(id).unwrap();
        let flags = c.data_state(id, &d).unwrap().flags;
        assert_eq!(flags & STAMP_MISMATCH, 0, "stamped and saved");
        let stored = c.gear_class(id, &d).unwrap();
        assert_eq!(stored, GearClass::Plastic);
        assert_eq!(gears(&headless, stored).unwrap(), Gears::Plastic);
        let metal = Args {
            gears: Some(Gears::Metal),
            ..travel_flags(Some(0.0), Some(183.5))
        };
        assert_eq!(
            gears(&metal, stored).unwrap(),
            Gears::Metal,
            "the flag wins"
        );
    }

    /// The stops plan under the gear train's stop hold, converted through
    /// the dev board's sense scale (60 mOhm, x14.884, 3.3 V: 1108 counts
    /// per A): on the 500 mA class default, plastic seats under 300 mA and
    /// metal under 500 mA, the approach cap stalling at that limit and the
    /// seat at half of it. A servo limit under the class's stays the
    /// servo's.
    #[test]
    fn the_stops_plan_under_the_gear_trains_stop_hold() {
        use osc_ident::limits::DutyPlan;

        let (mut c, id) = crate::rig::servo::bench::table(1961);
        let servo = crate::rig::limits::read(&mut c, id).unwrap();
        let apc = servo.amps_per_count;
        assert_eq!((1.0 / apc) as u32, 1108);
        let lim = ServoLimits {
            i_lim: (0.5 / apc) as u16,
            ..servo
        };
        let r = 2.0;
        for (gears, want) in [(Gears::Plastic, 332), (Gears::Metal, 554)] {
            let stop = stop_hold(lim, gears);
            assert_eq!(stop.i_lim, want, "{gears:?}");
            assert!(stop.i_lim as f64 * apc * 1000.0 <= gears.stop_hold_ma());
            let plan = DutyPlan::new(&stop, r, None);
            assert!((plan.stall(plan.stop_cap) - want as f64).abs() < 1e-6);
            assert!((plan.stall(plan.hold) - want as f64 / 2.0).abs() < 1e-6);
        }
        assert_eq!(stop_hold(servo, Gears::Metal).i_lim, 280, "never raised");
        assert_eq!(stop_hold(servo, Gears::Plastic).i_lim, 280);
    }

    /// `--yes` takes the counterclockwise end from the flag, else from the
    /// servo's angle map, and refuses before anything drives with neither;
    /// at a stop the headless run settles on it whichever stop it rests at.
    #[test]
    fn the_ccw_end_comes_from_the_flag_or_the_angle_map() {
        let flags = travel_flags(None, None);
        let err = Ccw::new(&flags, None).unwrap_err().to_string();
        assert!(err.contains("--ccw-end"), "{err}");
        let stored = Ccw::new(&flags, CcwEnd::of(18000, 0)).unwrap();
        assert_eq!(stored.settle(250, (232, 3849)).unwrap(), CcwEnd::Low);
        assert_eq!(stored.settle(3800, (232, 3849)).unwrap(), CcwEnd::Low);
        let flagged = Args {
            ccw_end: Some(CcwEnd::High),
            ..travel_flags(None, None)
        };
        let ccw = Ccw::new(&flagged, CcwEnd::of(18000, 0)).unwrap();
        assert_eq!(
            ccw.settle(250, (232, 3849)).unwrap(),
            CcwEnd::High,
            "the flag wins"
        );
        assert_eq!(CcwEnd::of(0, 18000), Some(CcwEnd::High));
        assert_eq!(CcwEnd::of(0, 0), None, "an unset map says nothing");
        assert_eq!(CcwEnd::nearer(300, (232, 3849)), CcwEnd::Low);
        assert_eq!(CcwEnd::nearer(3700, (232, 3849)), CcwEnd::High);
    }

    /// Cal on the bench servo for every way its pot and motor can be wired:
    /// the counts stay the pot's, rising whichever way the pot runs, the
    /// drive polarity is learned against them, and the angle map makes a
    /// positive degree step counterclockwise. A swapped pot runs its counts
    /// clockwise, so its counterclockwise end is the low one. The run stays
    /// inside every guard with the kernel's drive check on (an abort fails
    /// it), and the soft limits land inside the stops.
    #[test]
    fn every_wiring_calibrates_degrees_counterclockwise_positive() {
        use crate::rig::servo::bench::Bench;

        for (pot_swapped, motor_swapped) in
            [(false, false), (true, false), (false, true), (true, true)]
        {
            let case = format!("pot swapped {pot_swapped}, motor swapped {motor_swapped}");
            let mut b = Bench::mg90(crate::capture::Supply::TwoS);
            // a positive duty raises the counts unless one of the two is swapped
            b.servo.drive_polarity = pot_swapped == motor_swapped;
            b.servo.drive_check = true;
            let ccw_end = if pot_swapped {
                CcwEnd::Low
            } else {
                CcwEnd::High
            };
            let id = b.id();
            let lim = crate::rig::limits::read(&mut b.c, id).unwrap();
            let sense = read_sense(&mut b.c, id).unwrap();
            let sc = Scales::from_sense(
                &sense,
                read_u16(&mut b.c, id, calib::VBUS_DIV_TOP_OHM).unwrap(),
                read_u16(&mut b.c, id, calib::VBUS_DIV_BOT_OHM).unwrap(),
            )
            .unwrap();
            let args = Args {
                ccw_end: Some(ccw_end),
                ..travel_flags(Some(0.0), Some(180.0))
            };
            let dir = std::env::temp_dir().join(format!(
                "osc-cal-wiring-{}-{pot_swapped}-{motor_swapped}",
                std::process::id()
            ));
            let out = OutDir::create(&dir).unwrap();
            let found = find_stops(
                &mut b,
                &out,
                lim,
                stop_hold(lim, Gears::Metal),
                sc,
                sense.tick_hz as f64,
                Ccw::new(&args, None).unwrap(),
            )
            .unwrap_or_else(|e| panic!("{case}: {e:#}"));
            let _ = std::fs::remove_dir_all(&dir);
            assert_eq!(found.ccw_end, ccw_end, "{case}");
            assert!(
                b.servo.drive_polarity,
                "{case}: a positive duty raises the counts"
            );
            let (lo, hi) = (found.stops.pos_min_phys, found.stops.pos_max_phys);
            assert!(lo < hi, "{case}: counts rise with the pot");

            let (at_min, at_max) = ccw_end.rail_angles(0.0, 180.0);
            let (amin, amax) = angle_endpoints(at_min, at_max);
            let dpc = kinematics::deg_per_count(amin, amax, lo, hi);
            // counts per degree, then which way the shaft turns: the pot's
            // counts rise counterclockwise unless its ends are swapped
            let counts = 1.0 / dpc;
            let ccw_turn = if pot_swapped { -counts } else { counts };
            assert!(ccw_turn > 0.0, "{case}: +1 deg moves {counts:.2} counts");
            let soft = [10.0, 170.0].map(|deg| soft_to_count(deg, at_min, at_max, lo, hi));
            assert!(
                soft.iter().all(|&c| lo < c && c < hi),
                "{case}: soft {soft:?}"
            );
        }
    }

    /// The traverse on the bench servo the way cal drives it: a servo that
    /// drops rows from its bursts ends it in capture's words, torque off,
    /// with no ripple counted from them.
    #[test]
    fn a_traverse_with_dropped_rows_is_refused() {
        use crate::rig::servo::bench::{Bench, torque};
        use crate::rig::servo::{Servo, guard};

        let mut b = Bench::mg90(crate::capture::Supply::TwoS);
        b.servo.rows_dropped = 3;
        let stops = (232, 3849);
        let runway = Runway::new(traverse::guard(stops, Some((432, 3626))));
        let p = RigParams::new(None, 350).with_floor(4356).with_stops(stops);
        let cfg = order::sweep_cfg(0.1551, SweepCfg::default());
        let mut exp = Guarded::new(Sweep::new(cfg, &p, runway), p);
        guard(&mut b, |s| Pump::on(s, None).run(&mut exp)).unwrap();
        assert_eq!(
            check_abort("the traverse", exp.abort())
                .unwrap_err()
                .to_string(),
            "the traverse aborted: the servo dropped 3 TEL rows: both stream buffers were \
             waiting for the wire"
        );
        assert_eq!(exp.into_inner().captured().map(|c| c.to), Some(None));
        let id = b.id();
        assert_eq!(torque(&mut b.c, id), 0);
    }
}
