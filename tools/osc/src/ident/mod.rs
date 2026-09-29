//! `osc ident` - the identification subcommand: drives the osc-ident
//! experiments over the osc-adapter, records raw + derived CSVs, fits the
//! plant, synthesizes and encodes gains, and writes them back with
//! snapshot/rollback safety. The sans-io engine lives in osc-ident; this
//! wrapper owns USB, wall time, and files. TEL captures ride the main bus
//! as bursts (the pump's Stream arm) - no side channel. The run's order and
//! every duty it drives at come from osc-ident's `run`.

pub(crate) mod params;

use std::path::{Path, PathBuf};

use crate::rig::plant::Lut;
use crate::rig::pump::{self, Pump, with_guard, write_reg};
use crate::rig::{Aborted, check_abort, csvio, snapshot};
use anyhow::{Context, Result, bail};
use clap::{Subcommand, ValueEnum};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::{self, DataState};
use osc_client::descriptor::Descriptor;
use osc_client::nusb::NusbPipe;
use osc_ident::burst::{Capture, Chans};
use osc_ident::exp::bias::{Bias, BiasCfg, BiasResult};
use osc_ident::exp::breakaway::{Breakaway, BreakawayCfg, BreakawayResult};
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::held::{Held, HeldCfg, Stops};
use osc_ident::exp::inductance::{
    Cfg as InductanceCfg, FitCfg, Inductance, InductanceResult, fit_captures,
};
use osc_ident::exp::inertia::{Inertia, InertiaCfg, InertiaResult};
use osc_ident::exp::ladder::{Ladder, LadderCfg, LadderResult};
use osc_ident::exp::resistance::{Resistance, ResistanceCfg, ResistanceResult};
use osc_ident::exp::rl::{Rl, RlCfg, RlFitCfg, RlResult, Scales};
use osc_ident::exp::verify::{
    VerifyCurrent, VerifyCurrentCfg, VerifyResult, VerifyVelocity, VerifyVelocityCfg,
};
use osc_ident::exp::{Guarded, Permitted, RigParams};
use osc_ident::fits::{self, InertiaPriors};
use osc_ident::gains::{self, BwTargets, PlantParams};
use osc_ident::limits::{
    BurstAllowance, CLASS_R_MIN, DutyPlan, Envelope, Refusal, ServoLimits, pct_floor,
};
use osc_ident::pot::Pot;
use osc_ident::regs::{calib, control};
use osc_ident::report::{self, PlantInputs, ReportInputs};
use osc_ident::run::{self as order, Ended, Over, Run, Stage};
use osc_ident::sources::{self, Source, Winding};
use params::{
    BiasJson, BreakawayJson, GainJson, InductanceJson, InertiaJson, LadderJson, ParamsFile,
    PlantJson, PotJson, ResistanceJson, RlJson, SenseJson,
};

/// Where recorded runs land when `--out` is absent.
const DEFAULT_OUT: &str = "./ident-out";

const Q15: f64 = 32767.0;

/// The `osc ident` arg group: output, rig envelope, and bandwidth targets,
/// all scoped to the ident subtree. `--baud`/`--id` come from the top-level
/// osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output directory root; runs land in <out>/<timestamp>/ [default:
    /// ./ident-out]. `synth` takes it as the params.json path instead.
    #[arg(long, global = true)]
    out: Option<PathBuf>,
    /// Travel guard, low end, counts [default: the servo's low soft limit,
    /// 100 counts in].
    #[arg(long, global = true)]
    guard_lo: Option<u16>,
    /// Travel guard, high end, counts [default: the servo's high soft
    /// limit, 100 counts in].
    #[arg(long, global = true)]
    guard_hi: Option<u16>,
    #[arg(long, global = true, default_value_t = 1250)]
    slip_lo: u16,
    #[arg(long, global = true, default_value_t = 1650)]
    slip_hi: u16,
    /// Current that aborts a run, counts [default: a quarter over the
    /// servo's current_limit_counts, where the firmware holds a stall];
    /// above that is refused.
    #[arg(long, global = true)]
    i_abort: Option<i16>,
    /// Motor inductance, henries (not identifiable from this telemetry).
    #[arg(long, global = true, default_value_t = gains::DEFAULT_L_HENRIES)]
    l_henries: f64,
    /// Toggle step (`ident rl`), PWM periods. The default 20 (1 ms) is long
    /// enough for the rotor to follow, which biases R and L - see
    /// osc-ident's `exp::rl`.
    #[arg(long, global = true, default_value_t = 20)]
    step_periods: u16,
    /// Burst step duties, percent of full scale [default: 25 and 40, or
    /// what 3.2 V allows on this rail]; a rung over 3.2 V is refused.
    #[arg(long, global = true, value_delimiter = ',')]
    burst_pct: Option<Vec<u8>>,
    /// Burst captures per step duty and sign.
    #[arg(long, global = true, default_value_t = 5)]
    burst_repeats: u32,
    /// Settled winding current the burst duty ladder stays under, amps
    /// [default: 3.2 V over the class's lowest winding R, 3.0 ohm: 1.07 A].
    #[arg(long, global = true)]
    burst_i_max: Option<f64>,
    /// Burst voltage channels: a mask (bit 0 vmotor_a, bit 1 vmotor_b, bit 2
    /// vbus) or `driven`, the tap of the terminal each step drives. One
    /// channel keeps frame_len at 2 and sees the chopping leg through ON and
    /// OFF on both signs; the unbuffered rail tap reads ~1% low in-burst.
    #[arg(long, global = true, default_value = "driven", value_parser = parse_chans)]
    burst_chans: Chans,
    /// Burst held route: the duty the seek arrives at and the bursts step
    /// from, percent of full scale [default: the duty whose stall draws
    /// half the current limit].
    #[arg(long, global = true)]
    burst_hold_pct: Option<u8>,
    /// `ident burst` only: after the free shaft, seat against these
    /// mechanical stops and burst there too (the held route). Without it
    /// nothing stalls a stop.
    #[arg(long, global = true, value_enum)]
    burst_stops: Option<BurstStops>,
    /// Burst held route: captures per step duty at each stop.
    #[arg(long, global = true, default_value_t = 4)]
    burst_hold_repeats: u32,
    /// Inertia step length, ms. Sized to the runway: after the base's 100 ms
    /// the 150 ms default travels under 1000 of the ~2900 counts from the
    /// seek band to the far soft wall on the bench servo, either rail.
    #[arg(long, global = true, default_value_t = 150)]
    inertia_ms: u32,
    /// Nominal gear ratio, informational only (printed in the report dir).
    #[arg(long, global = true)]
    gear_ratio: Option<f64>,
    // bandwidth targets, Hz
    #[arg(long, global = true, default_value_t = 1000.0)]
    f_ci: f64,
    #[arg(long, global = true, default_value_t = 200.0)]
    f_cv: f64,
    #[arg(long, global = true, default_value_t = 25.0)]
    f_cp: f64,
    #[arg(long, global = true, default_value_t = 15.0)]
    f_o: f64,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum BurstStops {
    Low,
    High,
    Both,
}

impl From<BurstStops> for Stops {
    fn from(s: BurstStops) -> Self {
        match s {
            BurstStops::Low => Stops::Low,
            BurstStops::High => Stops::High,
            BurstStops::Both => Stops::Both,
        }
    }
}

/// Runner context: the ident args plus the resolved bus baud, threaded
/// through every experiment fn (the runners predate the arg-group split and
/// still read `cli.field`).
struct Ctx {
    baud: String,
    out: PathBuf,
    guard: (Option<u16>, Option<u16>),
    slip_lo: u16,
    slip_hi: u16,
    i_abort: Option<i16>,
    l_henries: f64,
    step_periods: u16,
    burst_pct: Option<Vec<u8>>,
    burst_repeats: u32,
    burst_i_max: Option<f64>,
    burst_chans: Chans,
    burst_hold_pct: Option<u8>,
    burst_stops: Option<BurstStops>,
    burst_hold_repeats: u32,
    inertia_ms: u32,
    gear_ratio: Option<f64>,
    f_ci: f64,
    f_cv: f64,
    f_cp: f64,
    f_o: f64,
    /// The servo's position table, read once the bus is up: the experiments
    /// fit in the counts the kernel controls on.
    lut: Option<Lut>,
    /// The servo's limits and the envelope resolved from them, read before
    /// any drive.
    drive: Option<Drive>,
}

struct Drive {
    lim: ServoLimits,
    env: Envelope,
    sense: SenseJson,
    sc: Scales,
}

impl Ctx {
    fn pot(&self) -> Pot {
        self.lut.as_ref().map_or(Pot::RAW, Lut::pot)
    }
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// The full pipeline: bias -> centring and the jam check (out and back
    /// at mid travel, raising the duty up to 2 V while the shaft does not
    /// move) -> burst at 25% and 40% (at most 3.2 V) -> breakaway -> ladder
    /// -> inertia, each from mid travel -> centring -> fit -> report +
    /// params.json. R and L come from the burst; every drive that could
    /// stall after it is planned from R and the rail so its stall stays
    /// under the current limit, while ladder and inertia rungs run free on
    /// the firmware limiter. A burst that declines ends the run: nothing
    /// after it can be planned, unless --stall-ladder asks for R from the
    /// stops instead. The first stage that aborts ends the run. Nothing
    /// stalls a stop unless asked. Write-back stays explicit.
    Run {
        /// When the burst declines, measure winding R from the resistance
        /// stop ladder instead of stopping: each stop stalled at up to four
        /// duties between the lowest the current sensor reads and the
        /// current limit, stall permit held.
        #[arg(long)]
        stall_ladder: bool,
    },
    /// Torque-off noise and bias floor.
    Bias,
    /// The resistance stop ladder -> winding R: each stop stalled at up to
    /// four duties between the lowest the current sensor reads and the
    /// current limit, stall permit held, then back to mid travel.
    Resistance,
    /// The toggle experiment: free-shaft duty toggles -> winding R and L
    /// (advisory; the 1 ms step is rotor-followed and biased).
    Rl,
    /// High-rate shunt bursts -> winding R, L and tau: the front of `run`,
    /// then the held route at a stop with --burst-stops.
    Burst,
    /// Breakaway duty ramp: the front of `run`, then breakaway.
    Breakaway,
    /// Steady-state duty ladder -> Ke + friction line: the front of `run`
    /// through breakaway, then the ladder.
    Ladder,
    /// Duty-step transients -> B (TEL when wired): the front of `run`
    /// through the ladder, then inertia.
    Inertia,
    /// Closed-loop verification on the written gains: verify current, then
    /// verify velocity. Needs a clean data state: a set written but not
    /// SAVEd on a fresh servo is refused (`ident write --save` first).
    Verify,
    /// Refit offline from a recorded run directory.
    Fit { dir: PathBuf },
    /// Synthesize gains from a hand-written plant - no run directory.
    ///
    /// <FILE> is JSON carrying a `plant` object in the params.json schema;
    /// every other params.json section may be absent. Fields, counts
    /// domain unless noted:
    ///   r_vpc        vcounts per ccount
    ///   ke_vpc       vcounts per (count/s)
    ///   fc           ccounts
    ///   fv           ccounts per (count/s)
    ///   b            count/s per medium tick per ccount
    ///   sigma_theta  counts
    ///   l_cd         vcount*s per ccount; omit or zero to derive it
    ///   l_henries    H, the SI inductance l_cd derives from
    ///   tick_hz      fast tick rate, Hz
    ///   f_med        medium tick rate, Hz
    ///   f_ci f_cv f_cp f_o   bandwidth targets, Hz; absent = the flags
    ///   r_ohm r_source l_source sigma_source   provenance, reported only
    ///
    /// Deriving l_cd needs the sense block (shunt_r_mohm, gain_milli,
    /// vmotor_div_top, vmotor_div_bot): put a `sense` object in the file
    /// or leave it out and the servo is read for it.
    ///
    /// The params.json lands at --out, else next to <FILE>.
    #[command(verbatim_doc_comment)]
    Synth { file: PathBuf },
    /// Write a params.json gain set to the table and stamp it: torque off,
    /// snapshot, read-back verified writes, the plant stamp over the set
    /// as written (the servo's checkpoint verifies it), SAVE with --save.
    ///
    /// A servo that boots CALIB_STALE or CALIB_VIRGIN (factory-fresh, or
    /// flashed across a CALIB layout change) reaches a clean data state
    /// without re-running the experiments:
    ///   osc status                              the reasons and the stamp
    ///   osc cal --yes --gear-ratio <g>          stops, polarity, angles; stamps + SAVEs
    ///   osc ident write <params.json> --save    the identified set + stamp, SAVE
    ///   osc ident verify                        closed loop, only now
    /// VIRGIN and STALE clear only on SAVE, so verify comes after --save.
    /// Without motion, a saved table does the same: osc recover --from.
    #[command(verbatim_doc_comment)]
    Write {
        params: PathBuf,
        /// Persist with MGMT SAVE after the stamp.
        #[arg(long)]
        save: bool,
    },
    /// Restore a snapshot.json written by `write`, then restamp.
    Rollback { snapshot: PathBuf },
    /// Print the current table values of every ident-owned field.
    Show,
}

/// Entry from the top-level `osc ident` dispatch. `baud`/`id` are the osc
/// globals; `args` carries the ident-scoped flags and subcommand.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    let mut cli = Ctx {
        baud,
        out: args.out.clone().unwrap_or_else(|| DEFAULT_OUT.into()),
        guard: (args.guard_lo, args.guard_hi),
        slip_lo: args.slip_lo,
        slip_hi: args.slip_hi,
        i_abort: args.i_abort,
        l_henries: args.l_henries,
        step_periods: args.step_periods,
        burst_pct: args.burst_pct.clone(),
        burst_repeats: args.burst_repeats,
        burst_i_max: args.burst_i_max,
        burst_chans: args.burst_chans,
        burst_hold_pct: args.burst_hold_pct,
        burst_stops: args.burst_stops,
        burst_hold_repeats: args.burst_hold_repeats,
        inertia_ms: args.inertia_ms,
        gear_ratio: args.gear_ratio,
        f_ci: args.f_ci,
        f_cv: args.f_cv,
        f_cp: args.f_cp,
        f_o: args.f_o,
        lut: None,
        drive: None,
    };
    pump::install_ctrlc();
    if let Cmd::Fit { dir } = &args.cmd {
        return fit_dir(&cli, dir.clone());
    }
    if let Cmd::Synth { file } = &args.cmd {
        return synth_file(&cli, id, file, args.out.as_deref());
    }
    let mut c = crate::rig::connect(&cli.baud)?;
    let id = Id::new(id);
    let d = crate::state::descriptor(&mut c, id)?;
    crate::state::warn(&mut c, id, &d)?;
    if drives(&args.cmd) {
        let lim = crate::rig::limits::read(&mut c, id)?;
        let env = lim.envelope(cli.guard, cli.i_abort)?;
        let sense = read_sense(&mut c, id)?;
        let sc = sense
            .scales()
            .context("CalibSense scales degenerate (shunt/gain/dividers/vdd)")?;
        let ma = lim.ma();
        println!(
            "limits: current limit {}, travel guard {}..{}, a run aborts over {}",
            ma.of(lim.i_lim as f64),
            env.guard.0,
            env.guard.1,
            ma.of(env.i_abort as f64)
        );
        cli.drive = Some(Drive {
            lim,
            env,
            sense,
            sc,
        });
    }
    let lut = Lut::read(&mut c, id, &d)?;
    println!("pot: {} counts, lut {}", lut.pot().label(), lut.describe());
    cli.lut = Some(lut);
    let cli = &cli;
    match &args.cmd {
        Cmd::Run { stall_ladder } => run_all(cli, &mut c, id, *stall_ladder),
        Cmd::Bias => {
            let out = csvio::OutDir::create(&cli.out)?;
            let (b, _) = run_bias(cli, &mut c, id, &out)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    bias: Some(&b),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Resistance => {
            let out = csvio::OutDir::create(&cli.out)?;
            let d = drive(cli)?;
            let plan = d.lim.stall_plan(d.sc.r_vpc(CLASS_R_MIN), None);
            let rungs = plan.stall_ladder(d.lim.window_floor())?;
            let cfg = order::resistance_cfg(plan.seek, &rungs, ResistanceCfg::default());
            let r =
                run_resistance(cli, &mut c, id, &out, cfg)?.context("resistance fit degenerate")?;
            println!(
                "R = {:.4} vcounts/ccount (r2 {:.4}, n {}, drift {:+.5}/s)",
                r.r_vpc, r.r2, r.n, r.drift_vpc_per_s
            );
            Ok(())
        }
        Cmd::Rl => {
            let out = csvio::OutDir::create(&cli.out)?;
            let sense = read_sense(&mut c, id)?;
            let r = run_rl(cli, &mut c, id, &out, &sense)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    rl: Some(&r),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Burst => {
            let rec = drive_stages(cli, &mut c, id, Until::Burst, false)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    inductance: rec.e8.as_ref(),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Breakaway => {
            let rec = drive_stages(cli, &mut c, id, Until::Breakaway, false)?;
            if let Some(bk) = rec.breakaway {
                println!("{bk:#?}");
            }
            Ok(())
        }
        Cmd::Ladder => {
            let rec = drive_stages(cli, &mut c, id, Until::Ladder, false)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    ladder: rec.ladder.as_ref(),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Inertia => {
            let rec = drive_stages(cli, &mut c, id, Until::Inertia, false)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    inertia: rec.inertia.as_ref(),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Verify => run_verify(cli, &mut c, id),
        Cmd::Write { params, save } => {
            let p = ParamsFile::load(params)?;
            if p.gains.is_empty() {
                bail!(
                    "{}: no gains section (run `ident fit` first)",
                    params.display()
                );
            }
            write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
            let snap = params
                .parent()
                .unwrap_or(std::path::Path::new("."))
                .join("snapshot.json");
            snapshot::take_snapshot(&mut c, id, &snap)?;
            snapshot::write_gains(&mut c, id, &p.gains)?;
            let s = crate::state::commit(&mut c, id, &d, *save)?;
            if data_state::allows(s.flags, true) {
                println!("next: ident verify");
            } else if !*save {
                println!(
                    "next: ident write {} --save, then ident verify",
                    params.display()
                );
            }
            Ok(())
        }
        Cmd::Rollback { snapshot } => {
            write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
            snapshot::rollback(&mut c, id, snapshot)?;
            crate::state::commit(&mut c, id, &d, false)?;
            Ok(())
        }
        Cmd::Show => snapshot::show(&mut c, id),
        Cmd::Fit { .. } | Cmd::Synth { .. } => unreachable!("handled above"),
    }
}

fn parse_chans(s: &str) -> Result<Chans, String> {
    Chans::parse(s).ok_or_else(|| format!("`{s}` is neither `driven` nor a mask 0..=7"))
}

/// The subcommands that move the shaft.
fn drives(cmd: &Cmd) -> bool {
    matches!(
        cmd,
        Cmd::Run { .. }
            | Cmd::Bias
            | Cmd::Resistance
            | Cmd::Rl
            | Cmd::Burst
            | Cmd::Breakaway
            | Cmd::Ladder
            | Cmd::Inertia
            | Cmd::Verify
    )
}

fn drive(cli: &Ctx) -> Result<&Drive> {
    cli.drive
        .as_ref()
        .context("the servo's limits were not read before a drive")
}

fn rig(cli: &Ctx) -> Result<RigParams> {
    let d = drive(cli)?;
    Ok(RigParams {
        slip: (cli.slip_lo, cli.slip_hi),
        pot: cli.pot(),
        ..RigParams::new(Some(d.env.guard), d.env.i_abort).with_stops(d.lim.raw)
    })
}

fn targets(cli: &Ctx) -> BwTargets {
    BwTargets {
        f_ci: cli.f_ci,
        f_cv: cli.f_cv,
        f_cp: cli.f_cp,
        f_o: cli.f_o,
    }
}

fn pct(duty: f64) -> String {
    format!("{:.1}%", duty * 100.0)
}

/// Into the band at mid travel ([`Centre`]); a blocked shaft ends the run.
/// The duty the shaft first travelled at, when it travelled.
fn centre(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id, cfg: CentreCfg) -> Result<Option<f64>> {
    let what = if cfg.nudge {
        "the jam check"
    } else {
        "centring"
    };
    let params = rig(cli)?;
    let mut exp = Guarded::new(Centre::new(cfg, &params), params.without_pos_guard());
    with_guard(c, id, |c| Pump::new(c, id, None).run(&mut exp))?;
    check_abort(what, exp.abort())?;
    let exp = exp.into_inner();
    if !exp.arrived() {
        bail!("centring did not reach mid travel in 5 s (gear slipping?)");
    }
    Ok(exp.moved_at())
}

/// Centring outside the run: at the duty whose stall the limit holds by
/// the table's R when the servo carries one, else at the class-safe duty.
fn centre_outside_the_run(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let d = drive(cli)?;
    let duty = match d.lim.r_vpc() {
        Some(r) => DutyPlan::new(&d.lim, r, None).seek,
        None => d.lim.bootstrap_duty(d.sc.r_vpc(CLASS_R_MIN)),
    };
    centre(cli, c, id, order::centre_cfg(duty, duty, false)).map(|_| ())
}

fn read_sense(c: &mut Client<NusbPipe>, id: Id) -> Result<SenseJson> {
    Ok(SenseJson {
        shunt_r_mohm: snapshot::read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: snapshot::read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: snapshot::read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: snapshot::read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        tick_hz: snapshot::read_u16(c, id, calib::TICK_HZ)?,
        vdd_mv: snapshot::read_u16(c, id, calib::VDD_MV)?,
        vbus_div_top_ohm: snapshot::read_u16(c, id, calib::VBUS_DIV_TOP_OHM)?,
        vbus_div_bot_ohm: snapshot::read_u16(c, id, calib::VBUS_DIV_BOT_OHM)?,
    })
}

/// Fit knobs anchored to the servo's own tick rate.
fn rl_fit_cfg(sense: &SenseJson) -> RlFitCfg {
    RlFitCfg {
        tick_hz: sense.tick_hz as f64,
        ..RlFitCfg::default()
    }
}

// --- experiment runners -----------------------------------------------------

fn run_bias(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
) -> Result<(BiasResult, f64)> {
    println!("[bias]");
    // torque off throughout: the travel guard has nothing to guard
    let params = rig(cli)?.without_pos_guard();
    let mut log = csvio::SnapshotLog::create(out, "bias_snapshots.csv")?;
    let mut exp = Guarded::new(Bias::new(BiasCfg::default(), &params), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("bias", exp.abort())?;
    let b = exp
        .into_inner()
        .result()
        .context("bias collected no samples")?;
    let vbus = b.vbus_mean;
    Ok((b, vbus))
}

/// The resistance stop ladder at `cfg`'s stall-safe duties, then back to
/// mid travel; None when no dwell fitted.
fn run_resistance(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: ResistanceCfg,
) -> Result<Option<ResistanceResult>> {
    let q = |d: i16| pct(d as f64 / Q15);
    println!(
        "[resistance] each stop stalled at {}, seeks at {}; pos guard off, stall permit held",
        cfg.ladder_q15
            .iter()
            .map(|d| q(*d))
            .collect::<Vec<_>>()
            .join(", "),
        q(cfg.seek_duty_q15)
    );
    let params = rig(cli)?.without_pos_guard();
    let mut log = csvio::SnapshotLog::create(out, "resistance_snapshots.csv")?;
    // stalling at the mechanical rails IS the method
    let mut exp = Guarded::new(Permitted::new(Resistance::new(cfg, &params)), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("resistance", exp.abort())?;
    centre_outside_the_run(cli, c, id)?;
    let exp = exp.into_inner().into_inner();
    csvio::write_dwell_samples(out, exp.samples())?;
    for w in exp.warnings() {
        println!("  warn: {w}");
    }
    Ok(exp.fit())
}

fn run_rl(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    sense: &SenseJson,
) -> Result<RlResult> {
    println!("[toggle] winding R/L (free shaft at mid travel, chained duty toggles)");
    centre_outside_the_run(cli, c, id)?;
    let params = rig(cli)?;
    let sc = sense
        .scales()
        .context("CalibSense scales degenerate (shunt/gain/dividers/vdd)")?;
    let cfg = RlCfg {
        step_periods: cli.step_periods,
        fit: rl_fit_cfg(sense),
        ..RlCfg::default()
    };
    let mut log = csvio::SnapshotLog::create(out, "rl_snapshots.csv")?;
    let mut exp = Guarded::new(Rl::new(cfg, &params, sc), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("toggle", exp.abort())?;
    let exp = exp.into_inner();
    csvio::write_rl_segments(out, exp.segments())?;
    // The planner's notes are the only account of a run that captured
    // nothing to fit; on the failure path they ARE the error.
    exp.fit().with_context(|| {
        format!(
            "rl fit degenerate ({} bursts captured; notes: {})",
            exp.segments().len(),
            match exp.warnings() {
                [] => "none".to_string(),
                w => w.join("; "),
            }
        )
    })
}

/// Free-shaft bursts from rest at mid travel ([`Inductance`]): their
/// captures and the planner's notes.
fn run_bursts(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: InductanceCfg,
    log_name: &str,
) -> Result<(Vec<Capture>, Vec<String>)> {
    let params = rig(cli)?;
    let sc = drive(cli)?.sc;
    let mut log = csvio::SnapshotLog::create(out, log_name)?;
    let mut exp = Guarded::new(Inductance::new(cfg, &params, sc), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("burst", exp.abort())?;
    let exp = exp.into_inner();
    Ok((exp.captures().to_vec(), exp.warnings().to_vec()))
}

/// The burst knobs the flags set; the stage sets the duties.
fn burst_base(cli: &Ctx) -> InductanceCfg {
    InductanceCfg {
        repeats: cli.burst_repeats,
        i_max_a: cli.burst_i_max.unwrap_or(BurstAllowance::i_max_a()),
        chans: cli.burst_chans,
        ..InductanceCfg::default()
    }
}

/// The held route at the stops --burst-stops names, at the plan's hold and
/// under its stop cap. Stalling a stop is the method, so it never runs
/// unless asked.
fn run_held(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    stops: BurstStops,
    plan: &DutyPlan,
    r_vpc: f64,
) -> Result<(Vec<Capture>, Vec<String>)> {
    let d = drive(cli)?;
    let hold = cli.burst_hold_pct.unwrap_or(pct_floor(plan.hold));
    let steps = cli
        .burst_pct
        .clone()
        .unwrap_or_else(|| vec![pct_floor(plan.stop_cap)]);
    let q = |p: u8| (p as i32 * Q15 as i32 / 100) as i16;
    d.lim
        .check_stall_at("the burst's hold against the stop", q(hold), r_vpc)?;
    for s in &steps {
        d.lim
            .check_stall_at("a burst against the stop", q(*s), r_vpc)?;
    }
    println!(
        "[burst, held] (seat at the {stops:?} stop at {hold}%, burst toward it; stall_permit for \
         the run)"
    );
    let cap = pct_floor(plan.stop_cap);
    let cfg = HeldCfg {
        stops: stops.into(),
        hold_pct: hold,
        step_pct: steps,
        repeats: cli.burst_hold_repeats,
        i_max_a: burst_base(cli).i_max_a,
        chans: cli.burst_chans,
        seek_cap_pct: cap,
        centre_cap_pct: cap,
        ..HeldCfg::default()
    };
    let params = rig(cli)?;
    let mut log = csvio::SnapshotLog::create(out, "held_snapshots.csv")?;
    let mut held = Guarded::new(Held::new(cfg, &params, d.sc), params.without_pos_guard());
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut held))?;
    check_abort("burst", held.abort())?;
    let held = held.into_inner();
    for s in held.seats() {
        println!(
            "  seated at pos {} driving {:+}: applied {} q15 at the hold, arrived at {} q15",
            s.pos, s.dir, s.duty_applied_q15, s.arrived_q15
        );
    }
    for w in held.warnings() {
        println!("  warn: {w}");
    }
    Ok((held.captures().to_vec(), held.warnings().to_vec()))
}

fn run_breakaway(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: BreakawayCfg,
    r_vpc: f64,
    vbus_mean: f64,
) -> Result<BreakawayResult> {
    let mut log = csvio::SnapshotLog::create(out, "breakaway_snapshots.csv")?;
    let mut exp = Guarded::new(Breakaway::new(cfg), rig(cli)?);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("breakaway", exp.abort())?;
    Ok(exp.into_inner().fit(r_vpc, vbus_mean))
}

fn run_ladder(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: LadderCfg,
    r_vpc: f64,
) -> Result<LadderResult> {
    let params = rig(cli)?;
    let mut log = csvio::SnapshotLog::create(out, "ladder_snapshots.csv")?;
    let mut exp = Guarded::new(Ladder::new(cfg, &params), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("ladder", exp.abort())?;
    let l = exp
        .into_inner()
        .fit(r_vpc)
        .context("ladder fit degenerate")?;
    csvio::write_rungs(out, &l.rungs)?;
    Ok(l)
}

fn run_inertia(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: InertiaCfg,
    plan: DutyPlan,
    priors: &InertiaPriors,
) -> Result<InertiaResult> {
    let params = rig(cli)?;
    let mut log = csvio::SnapshotLog::create(out, "inertia_snapshots.csv")?;
    let mut exp = Guarded::new(Inertia::new(cfg, plan, &params), params);
    let all_tel = with_guard(c, id, |c| {
        let mut pump = Pump::new(c, id, Some(&mut log));
        pump.run(&mut exp)?;
        Ok(std::mem::take(&mut pump.tel))
    })?;
    check_abort("inertia", exp.abort())?;
    let exp = exp.into_inner();
    csvio::write_tel_frames(out, "inertia_tel.csv", &all_tel)?;
    csvio::write_step_series(out, &exp.step_series())?;
    exp.fit(priors).with_context(|| {
        format!(
            "no inertia step fitted (notes: {})",
            match exp.notes().as_slice() {
                [] => "none".to_string(),
                n => n.join("; "),
            }
        )
    })
}

fn run_verify(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let params = rig(cli)?;
    let tick_hz = snapshot::read_u16(c, id, calib::TICK_HZ)? as f64;
    let (d, before) = servo_state(c, id)?;
    refuse_closed_loop(&before)?;
    centre_outside_the_run(cli, c, id)?;
    println!("[verify current] (current steps; end-stop stalls; stall permit held)");
    // deliberate rail stall in Current mode: the directional endstop band
    // would zero i_ref at the soft wall, and the permit opens it, as for
    // resistance
    let mut e5 = Guarded::new(
        Permitted::new(VerifyCurrent::new(VerifyCurrentCfg::default(), &params)),
        params.without_pos_guard(),
    );
    with_guard(c, id, |c| Pump::new(c, id, None).run(&mut e5))?;
    check_abort("verify current", e5.abort())?;
    let cur = e5.into_inner().into_inner().result();
    refuse_closed_loop(&c.data_state(id, &d)?)?;
    // E5 ends stalled against an end-stop; E6 runs with the pos guard on
    // and its first read would abort right there
    centre_outside_the_run(cli, c, id)?;
    println!("[verify velocity] (velocity legs)");
    let mut e6 = Guarded::new(
        VerifyVelocity::new(VerifyVelocityCfg::default(), &params, tick_hz),
        params,
    );
    with_guard(c, id, |c| Pump::new(c, id, None).run(&mut e6))?;
    check_abort("verify velocity", e6.abort())?;
    let vel = e6.into_inner().result();
    for s in &cur.steps {
        println!(
            "  goal {:+5} -> {:+8.1} ({:.1}% err, settle {})",
            s.goal,
            s.mean_i,
            s.err_pct,
            s.settle_ms
                .map(|t| format!("{t:.0} ms"))
                .unwrap_or_else(|| "never".into()),
        );
    }
    for l in &vel.legs {
        println!(
            "  goal {:+6} c/s -> {:+8.1} ({:.1}% err, r2 {:.4}, n {})",
            l.goal_cps, l.meas_cps, l.err_pct, l.r2, l.n
        );
    }
    let v = VerifyResult::assemble(Some(cur), Some(vel));
    println!("verify: {}", if v.pass { "PASS" } else { "FAIL" });
    if !v.pass {
        std::process::exit(1);
    }
    Ok(())
}

fn servo_state(c: &mut Client<NusbPipe>, id: Id) -> Result<(Descriptor, DataState)> {
    let d = crate::state::descriptor(c, id)?;
    let s = c.data_state(id, &d)?;
    Ok((d, s))
}

/// The servo refuses closed loop under any reason; verify says so before
/// an enable latches CODE_DATA.
fn refuse_closed_loop(s: &DataState) -> Result<()> {
    if data_state::allows(s.flags, true) {
        return Ok(());
    }
    bail!(
        "closed loop refused: {} - {}",
        s.names(),
        s.message().unwrap_or_default()
    )
}

// --- fitting ----------------------------------------------------------------

fn priors_of(r_vpc: f64, l: &LadderResult, sense: &SenseJson) -> InertiaPriors {
    let mean_opt = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a + b) / 2.0,
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.0,
    };
    InertiaPriors {
        r_vpc,
        ke_vpc: l.ke.ke_vpc,
        fc: mean_opt(l.fric_fwd.map(|f| f.fc), l.fric_rev.map(|f| f.fc)),
        fv: mean_opt(l.fric_fwd.map(|f| f.fv), l.fric_rev.map(|f| f.fv)),
        tick_hz: sense.tick_hz as f64,
    }
}

fn run_all(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id, stall_ladder: bool) -> Result<()> {
    let rec = drive_stages(cli, c, id, Until::End, stall_ladder)?;
    let (Some((bias, _)), Some(breakaway)) = (&rec.bias, &rec.breakaway) else {
        bail!("the run ended without its bias or breakaway");
    };
    let p = ParamsFile {
        bias: Some(BiasJson::from(bias)),
        inductance: rec.e8.as_ref().map(InductanceJson::from),
        breakaway: Some(BreakawayJson::from(breakaway)),
        sense: Some(drive(cli)?.sense),
        pot: cli.lut.as_ref().map(PotJson::from),
        ..Default::default()
    };
    let dir = rec.out.0;
    p.save(&dir.join("params.json"))?;
    if let Some(g) = cli.gear_ratio {
        std::fs::write(dir.join("gear_ratio.txt"), format!("{g}\n"))?;
    }
    // the offline path is THE fit path - run records, fit computes
    fit_dir(cli, dir)
}

/// How far a subcommand takes the run before its closing centring.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Until {
    Burst,
    Breakaway,
    Ladder,
    Inertia,
    End,
}

impl Until {
    fn reached(self, stage: &Stage) -> bool {
        matches!(
            (self, stage),
            (Until::Burst, Stage::Burst { .. })
                | (Until::Breakaway, Stage::Breakaway { .. })
                | (Until::Ladder, Stage::Ladder { .. })
                | (Until::Inertia, Stage::Inertia { .. })
        )
    }
}

/// The run's stages in the order osc-ident's [`Run`] names them, through
/// `until` and the closing centring. The first abort ends it; so does a
/// burst that declines, since nothing after it can be planned, unless
/// `stall_ladder` hands over to the resistance stop ladder.
fn drive_stages(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    until: Until,
    stall_ladder: bool,
) -> Result<Recorded> {
    let out = csvio::OutDir::create(&cli.out)?;
    println!("recording to {}", out.0.display());
    let d = drive(cli)?;
    let mut run = Run::new(d.lim, &d.sc);
    if stall_ladder {
        run = run.with_stall_ladder();
    }
    let mut rec = Recorded {
        out,
        caps: Vec::new(),
        notes: Vec::new(),
        bias: None,
        e8: None,
        resistance: None,
        w: None,
        breakaway: None,
        ladder: None,
        inertia: None,
    };
    while let Some(stage) = run.next_stage() {
        match rec.stage(&stage, &mut run, cli, c, id, until) {
            Ok(how) => run.ended(how),
            Err(e) => {
                if let Some(a) = e.downcast_ref::<Aborted>() {
                    run.ended(Ended::Aborted(a.reason));
                }
                return Err(e);
            }
        }
        if until.reached(&stage) {
            run.finish();
        }
    }
    match run.over() {
        Some(Over::Declined) if until != Until::Burst => bail!(
            "no winding R: the burst declined ({}), and nothing after it can be planned, so the \
             run stops here; `osc ident run --stall-ladder` measures R from stop stalls held \
             under the current limit instead",
            rec.e8
                .as_ref()
                .map_or("no fit".into(), |r| r.blocking().join(", "))
        ),
        Some(Over::NoLadderRoom { floor, cap }) => bail!(
            "no winding R: the burst measured none, and {}",
            Refusal::NoLadderRoom { floor, cap }
        ),
        Some(Over::ResistanceDeclined) => bail!(
            "no winding R: neither the burst nor the resistance stop ladder fitted one, so the \
             run stops here"
        ),
        Some(Over::Unproven) => {
            bail!("the jam check never saw the shaft move, so no burst may run")
        }
        Some(Over::RailTooHigh { rail_mv }) => bail!(
            "no winding R: on a {:.1} V rail the 3.2 V burst allowance leaves its two rungs \
             under the 10% apart the fit needs, so the run stops here; `osc ident run \
             --stall-ladder` measures R from stop stalls held under the current limit instead",
            rail_mv / 1000.0
        ),
        _ => Ok(rec),
    }
}

/// What the stages record and hand each other.
struct Recorded {
    out: csvio::OutDir,
    /// Every burst capture so far, free and held routes alike: one
    /// family, one fit.
    caps: Vec<Capture>,
    notes: Vec<String>,
    bias: Option<(BiasResult, f64)>,
    e8: Option<InductanceResult>,
    resistance: Option<ResistanceResult>,
    w: Option<Winding>,
    breakaway: Option<BreakawayResult>,
    ladder: Option<LadderResult>,
    inertia: Option<InertiaResult>,
}

impl Recorded {
    /// The winding the run plans from, from here on.
    fn planned(&mut self, w: Winding, run: &mut Run, lim: &ServoLimits) -> Result<()> {
        run.measured(w.r_vpc);
        let plan = run.plan().context("no plan")?;
        println!(
            "[winding] R {:.4} vcounts/ccount from {}, L {:.4} mH from {}",
            w.r_vpc,
            w.r_from.as_str(),
            w.l_h * 1e3,
            w.l_from.as_str()
        );
        let ma = lim.ma();
        println!(
            "[plan] every drive that could stall from here stays under the current limit of {}: \
             seeks at {}, stop drives up to {}",
            ma.of(lim.i_lim as f64),
            pct(plan.seek),
            pct(plan.stop_cap)
        );
        self.w = Some(w);
        Ok(())
    }

    fn stage(
        &mut self,
        stage: &Stage,
        run: &mut Run,
        cli: &Ctx,
        c: &mut Client<NusbPipe>,
        id: Id,
        until: Until,
    ) -> Result<Ended> {
        let d = drive(cli)?;
        let out = &self.out;
        match stage {
            Stage::Bias => self.bias = Some(run_bias(cli, c, id, out)?),
            Stage::Centre { duty, cap, nudge } if *nudge => {
                println!(
                    "[centring] the jam check: out and back at mid travel from {}, raised up to \
                     {} while the shaft does not move",
                    pct(*duty),
                    pct(*cap)
                );
                let moved = centre(cli, c, id, order::centre_cfg(*duty, *cap, *nudge))?;
                if let Some(m) = moved {
                    println!("  the shaft moves at {}", pct(m));
                }
                run.nudged(moved);
            }
            Stage::Centre { duty, cap, nudge } => {
                println!("[centring] to mid travel at {}", pct(*duty));
                centre(cli, c, id, order::centre_cfg(*duty, *cap, *nudge))?;
            }
            Stage::Burst { rungs, pre, seek } => {
                let most = BurstAllowance::MAX_MV / run.rail_mv();
                let explicit: Option<Vec<f64>> = cli
                    .burst_pct
                    .as_ref()
                    .map(|p| p.iter().map(|p| *p as f64 / 100.0).collect());
                if let Some(hi) = explicit.iter().flatten().copied().reduce(f64::max)
                    && hi > most
                {
                    bail!(
                        "a burst rung of {:.0}% applies more than the 3.2 V a burst may on this \
                         {:.1} V rail ({} at most): leave --burst-pct out",
                        hi * 100.0,
                        run.rail_mv() / 1000.0,
                        pct(most)
                    );
                }
                let cfg = order::burst_cfg(
                    explicit.as_deref().unwrap_or(rungs),
                    *pre,
                    *seek,
                    burst_base(cli),
                );
                println!(
                    "[burst, free] from rest at mid travel at {}",
                    cfg.step_pct
                        .iter()
                        .map(|p| format!("{p}%"))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                let (caps, notes) = run_bursts(cli, c, id, out, cfg, "inductance_snapshots.csv")?;
                self.caps.extend(caps);
                self.notes.extend(notes);
                let fitted = fit_captures(&self.caps, &d.sc, &FitCfg::default());
                let w = sources::winding(fitted.as_ref(), None, Some(&d.sc), cli.l_henries);
                if let (Some(w), Some(stops), Until::Burst) = (w, cli.burst_stops, until) {
                    let plan = DutyPlan::new(&d.lim, w.r_vpc, None);
                    let (caps, notes) = run_held(cli, c, id, out, stops, &plan, w.r_vpc)?;
                    self.caps.extend(caps);
                    self.notes.extend(notes);
                }
                csvio::write_bursts(out, &self.caps)?;
                let mut fitted = fit_captures(&self.caps, &d.sc, &FitCfg::default());
                if let Some(r) = fitted.as_mut() {
                    r.warnings.splice(0..0, self.notes.iter().cloned());
                }
                self.e8 = fitted;
                let Some(w) = sources::winding(self.e8.as_ref(), None, Some(&d.sc), cli.l_henries)
                else {
                    return Ok(Ended::Declined);
                };
                self.planned(w, run, &d.lim)?;
            }
            Stage::Resistance { seek, rungs } => {
                println!("[resistance] the burst measured no R: the stop ladder, as asked");
                let cfg = order::resistance_cfg(*seek, rungs, ResistanceCfg::default());
                let Some(r) = run_resistance(cli, c, id, out, cfg)? else {
                    return Ok(Ended::Declined);
                };
                let w = sources::winding(self.e8.as_ref(), Some(&r), Some(&d.sc), cli.l_henries)
                    .context("no winding R")?;
                self.resistance = Some(r);
                self.planned(w, run, &d.lim)?;
            }
            Stage::Breakaway { cap } => {
                println!("[breakaway] ramp up to {}", pct(*cap));
                let r_vpc = self.w.as_ref().context("no winding R")?.r_vpc;
                let vbus = self.bias.as_ref().map_or(d.lim.vbus as f64, |b| b.1);
                let bk = run_breakaway(cli, c, id, out, order::breakaway_cfg(*cap), r_vpc, vbus)?;
                let top = bk.duty_bk_fwd.into_iter().chain(bk.duty_bk_rev).max();
                run.broke_away(top.map(|q| q as f64 / Q15));
                self.breakaway = Some(bk);
            }
            Stage::Ladder { seek, rungs } => {
                println!(
                    "[ladder] rungs {}, seeks at {}",
                    rungs.iter().map(|r| pct(*r)).collect::<Vec<_>>().join(", "),
                    pct(*seek)
                );
                let r_vpc = self.w.as_ref().context("no winding R")?.r_vpc;
                let cfg = order::ladder_cfg(*seek, rungs);
                self.ladder = Some(run_ladder(cli, c, id, out, cfg, r_vpc)?);
            }
            Stage::Inertia { seek, base } => {
                println!(
                    "[inertia] steps from a moving base at {}, each sized to what the current \
                     limit leaves over the base's running current; seeks at {}",
                    pct(*base),
                    pct(*seek)
                );
                let r_vpc = self.w.as_ref().context("no winding R")?.r_vpc;
                let ladder = self.ladder.as_ref().context("no ladder")?;
                let priors = priors_of(r_vpc, ladder, &d.sense);
                let cfg = InertiaCfg {
                    tick_hz: priors.tick_hz,
                    capture_ms: cli.inertia_ms,
                    ..InertiaCfg::default()
                };
                let cfg = order::inertia_cfg(*seek, *base, cfg);
                let plan = run.plan().context("no plan")?;
                self.inertia = Some(run_inertia(cli, c, id, out, cfg, plan, &priors)?);
            }
        }
        Ok(Ended::Done)
    }
}

/// Refit from a recorded directory: reads params.json (bias, breakaway,
/// sense) plus the derived CSVs, recomputes every fit, synthesizes, and
/// rewrites params.json with the plant and encoded gains.
fn fit_dir(cli: &Ctx, dir: PathBuf) -> Result<()> {
    let path = dir.join("params.json");
    let mut p = ParamsFile::load(&path)?;
    let sense = p.sense.context("params.json has no sense block")?;
    let tick_hz = sense.tick_hz as f64;
    println!(
        "{}",
        p.pot.as_ref().map_or_else(
            || "pot counts: raw (the run recorded no lut)".into(),
            PotJson::describe
        )
    );

    // Refitted and reported when the run captured one, consumed by nothing:
    // the 1 ms toggle step is rotor-followed (osc-ident `exp::rl`).
    let rl = match dir.join("rl.csv").exists() {
        true => {
            let segs = csvio::read_rl_segments(&dir)?;
            let sc = sense.scales().context(
                "params.json sense block predates the R/L band (no vdd_mv / vbus divider)",
            )?;
            Some(
                osc_ident::exp::rl::fit_segments(&segs, &sc, &rl_fit_cfg(&sense))
                    .context("rl refit degenerate")?,
            )
        }
        false => None,
    };
    let sc = sense.scales();
    let inductance = match (csvio::read_bursts(&dir)?.as_slice(), sc) {
        ([], _) | (_, None) => None,
        (caps, Some(sc)) => osc_ident::exp::inductance::fit_captures(caps, &sc, &FitCfg::default()),
    };
    // E2 is refitted whenever the run recorded it, but the winding takes it
    // only behind a declined E8.
    let resistance = match dir.join("resistance.csv").exists() {
        true => Some(
            Resistance::fit_samples(&csvio::read_dwell_samples(&dir)?)
                .context("resistance refit degenerate")?,
        ),
        false => None,
    };
    let w = sources::winding(
        inductance.as_ref(),
        resistance.as_ref(),
        sc.as_ref(),
        cli.l_henries,
    )
    .context("no winding R: burst is missing or declined and no resistance recording exists")?;
    let rungs = csvio::read_rungs(&dir)?;
    let pts: Vec<fits::RungPoint> = csvio::read_rung_points(&dir)?;
    let ke = fits::ke_fit(&pts, w.r_vpc).context("ke refit degenerate")?;
    let fric_fwd = fits::friction_line(&pts, 1);
    let fric_rev = fits::friction_line(&pts, -1);
    let ladder = LadderResult {
        ke,
        fric_fwd,
        fric_rev,
        rungs,
        warnings: Vec::new(),
    };
    let priors = priors_of(w.r_vpc, &ladder, &sense);
    let series = csvio::read_step_series(&dir)?;
    let tel_steps = series.iter().filter(|(_, tel)| *tel).count();
    // same smoothing-window rule as Inertia::fit
    let hw = if tel_steps > 0 {
        (0.010 * tick_hz) as usize
    } else {
        12
    };
    let b_direct = fits::b_direct_fit(&series_only(&series), &priors, hw, 5.0);
    let b_exp = fits::b_exp_fit(&series_only(&series), &priors, hw);
    let b_best = match (&b_exp, &b_direct) {
        (Some(e), Some(d)) => {
            if d.r2 > 0.98 && d.r2 > 1.0 - e.spread {
                d.b
            } else {
                e.b
            }
        }
        (Some(e), None) => e.b,
        (None, Some(d)) => d.b,
        (None, None) => bail!("no inertia estimate from either estimator"),
    };
    let inertia = osc_ident::exp::inertia::InertiaResult {
        b_direct,
        b_exp,
        b_best,
        j_ff: 1.0 / b_best,
        tel_steps,
        warnings: Vec::new(),
    };

    let bias = p.bias;
    let sigma_theta = bias.map(|b| b.sigma_theta).unwrap_or(1.0);
    let sigma_from = match bias {
        Some(_) => Source::Bias,
        None => Source::Default,
    };
    let l_cd = gains::l_cd_from_si(
        w.l_h,
        sense.shunt_r_mohm,
        sense.gain_milli,
        sense.vmotor_div_top,
        sense.vmotor_div_bot,
    )
    .context("sense scales degenerate")?;
    let mean_opt = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a + b) / 2.0,
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.0,
    };
    let plant = PlantParams {
        r_vpc: w.r_vpc,
        ke_vpc: ladder.ke.ke_vpc,
        fc: mean_opt(ladder.fric_fwd.map(|f| f.fc), ladder.fric_rev.map(|f| f.fc)),
        fv: mean_opt(ladder.fric_fwd.map(|f| f.fv), ladder.fric_rev.map(|f| f.fv)),
        b: inertia.b_best,
        sigma_theta,
        l_cd,
        tick_hz,
        f_med: tick_hz / 10.0,
    };
    let t = targets(cli);
    let gains_set = gains::synthesize(&plant, &t);
    let encoded = gains::encode(&gains_set);

    let bias_res = bias.map(|b| osc_ident::exp::bias::BiasResult {
        sigma_theta: b.sigma_theta,
        pos_mean: b.pos_mean,
        i_noise: b.i_noise,
        i_bias_delta: b.i_bias_delta,
        vbus_mean: b.vbus_mean,
        vbus_sd: b.vbus_sd,
        n: b.n,
    });
    let bk_res = p
        .breakaway
        .map(|b| osc_ident::exp::breakaway::BreakawayResult {
            duty_bk_fwd: b.duty_bk_fwd,
            duty_bk_rev: b.duty_bk_rev,
            fric_fwd_counts: b.fric_fwd_counts,
            fric_rev_counts: b.fric_rev_counts,
            model_derived: b.model_derived,
            asymmetry: b.asymmetry,
        });
    let text = report::render(&ReportInputs {
        bias: bias_res.as_ref(),
        resistance: resistance.as_ref(),
        rl: rl.as_ref(),
        inductance: inductance.as_ref(),
        breakaway: bk_res.as_ref(),
        ladder: Some(&ladder),
        inertia: Some(&inertia),
        gains: Some((&gains_set, &encoded)),
        plant: Some(PlantInputs {
            plant: &plant,
            winding: &w,
            sigma_from,
        }),
    });
    println!("{text}");
    std::fs::write(dir.join("report.txt"), &text)?;

    p.resistance = resistance.as_ref().map(ResistanceJson::from);
    p.rl = rl.as_ref().map(RlJson::from);
    p.inductance = inductance.as_ref().map(InductanceJson::from);
    p.ladder = Some(LadderJson {
        ke_vpc: ladder.ke.ke_vpc,
        ke_r2: ladder.ke.r2,
        fc_fwd: ladder.fric_fwd.map(|f| f.fc),
        fv_fwd: ladder.fric_fwd.map(|f| f.fv),
        fc_rev: ladder.fric_rev.map(|f| f.fc),
        fv_rev: ladder.fric_rev.map(|f| f.fv),
        rungs_used: ladder.rungs.iter().filter(|r| r.used).count(),
    });
    p.inertia = Some(InertiaJson {
        b_best: inertia.b_best,
        b_direct: inertia.b_direct.as_ref().map(|d| d.b),
        b_exp: inertia.b_exp.as_ref().map(|e| e.b),
        j_ff: inertia.j_ff,
        tel_steps: inertia.tel_steps,
    });
    p.plant = Some(PlantJson::new(&plant, &t, &w, sigma_from.as_str()));
    p.gains = GainJson::set(&encoded);
    p.save(&path)?;
    println!("params: {}", path.display());
    println!(
        "next: ident write {} [--save], then ident verify",
        path.display()
    );
    Ok(())
}

// --- hand-written plant -----------------------------------------------------

/// The plant as `gains::synthesize` takes it, plus the targets it runs
/// against: `l_cd` either as written or derived from `l_henries` through the
/// sense block, and each absent bandwidth target filled from `cli`.
fn synth_plant(
    p: &PlantJson,
    sense: Option<&SenseJson>,
    cli: &BwTargets,
) -> Result<(PlantParams, BwTargets)> {
    let l_cd = if p.l_cd > 0.0 {
        p.l_cd
    } else {
        if p.l_henries <= 0.0 {
            bail!("plant has neither l_cd nor l_henries");
        }
        let s = sense.context("no sense block: l_cd cannot be derived from l_henries")?;
        gains::l_cd_from_si(
            p.l_henries,
            s.shunt_r_mohm,
            s.gain_milli,
            s.vmotor_div_top,
            s.vmotor_div_bot,
        )
        .context("sense scales degenerate")?
    };
    let pick = |v: f64, d: f64| if v > 0.0 { v } else { d };
    let t = BwTargets {
        f_ci: pick(p.f_ci, cli.f_ci),
        f_cv: pick(p.f_cv, cli.f_cv),
        f_cp: pick(p.f_cp, cli.f_cp),
        f_o: pick(p.f_o, cli.f_o),
    };
    let plant = PlantParams {
        r_vpc: p.r_vpc,
        ke_vpc: p.ke_vpc,
        fc: p.fc,
        fv: p.fv,
        b: p.b,
        sigma_theta: p.sigma_theta,
        l_cd,
        tick_hz: p.tick_hz,
        f_med: p.f_med,
    };
    Ok((plant, t))
}

/// Synthesize from a hand-written plant: same synthesis, encoding and
/// report as the fit path, none of the experiments. The servo is touched
/// only for a sense block the file omits.
fn synth_file(cli: &Ctx, id: u8, file: &Path, out: Option<&Path>) -> Result<()> {
    let f = ParamsFile::load(file)?;
    let pj = f
        .plant
        .clone()
        .with_context(|| format!("{}: no plant section", file.display()))?;
    let path = match out {
        Some(p) => p.to_path_buf(),
        None => file.parent().unwrap_or(Path::new(".")).join("params.json"),
    };
    if path == file {
        bail!("{} would overwrite the input: pass --out", path.display());
    }
    let mut sense = f.sense;
    if pj.l_cd <= 0.0 && sense.is_none() {
        let mut c = crate::rig::connect(&cli.baud)?;
        sense = Some(read_sense(&mut c, Id::new(id))?);
    }
    let (plant, t) = synth_plant(&pj, sense.as_ref(), &targets(cli))?;
    let g = gains::synthesize(&plant, &t);
    let encoded = gains::encode(&g);
    // the file's own source strings stay in params.json; the report has
    // only the enum, and a hand-written plant is measured by none of E0-E8
    let w = Winding {
        r_ohm: pj.r_ohm,
        r_vpc: plant.r_vpc,
        r_from: Source::Default,
        l_h: pj.l_henries,
        l_from: Source::Default,
    };
    println!(
        "{}",
        render_partial(ReportInputs {
            gains: Some((&g, &encoded)),
            plant: Some(PlantInputs {
                plant: &plant,
                winding: &w,
                sigma_from: Source::Default,
            }),
            ..Default::default()
        })
    );
    let p = ParamsFile {
        sense,
        plant: Some(PlantJson {
            l_cd: plant.l_cd,
            f_ci: t.f_ci,
            f_cv: t.f_cv,
            f_cp: t.f_cp,
            f_o: t.f_o,
            ..pj
        }),
        gains: GainJson::set(&encoded),
        ..Default::default()
    };
    p.save(&path)?;
    println!("params: {}", path.display());
    println!("next: ident write {} [--save]", path.display());
    Ok(())
}

fn series_only(s: &[(fits::StepSeries, bool)]) -> Vec<fits::StepSeries> {
    s.iter().map(|(s, _)| s.clone()).collect()
}

fn render_partial(inputs: ReportInputs<'_>) -> String {
    report::render(&inputs)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The `ident synth` path end to end without a servo: a hand-written
    /// plant with no l_cd takes it from l_henries through the file's own
    /// sense block, and the absent targets fall back to the CLI's.
    #[test]
    fn synth_encodes_a_hand_written_plant() {
        let json = r#"{
          "plant": {
            "r_vpc": 3.37, "ke_vpc": 0.2, "fc": 20.0, "fv": 0.001,
            "b": 0.1, "sigma_theta": 6.9, "tick_hz": 20100.0, "f_med": 2010.0,
            "l_henries": 0.0005
          },
          "sense": {
            "shunt_r_mohm": 33, "gain_milli": 15000,
            "vmotor_div_top": 18200, "vmotor_div_bot": 10000, "tick_hz": 20100
          }
        }"#;
        let dir = std::env::temp_dir().join(format!("ident-synth-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("plant.json");
        std::fs::write(&path, json).unwrap();

        let f = ParamsFile::load(&path).unwrap();
        let pj = f.plant.clone().unwrap();
        let (plant, t) = synth_plant(&pj, f.sense.as_ref(), &BwTargets::default()).unwrap();
        let encoded = gains::encode(&gains::synthesize(&plant, &t));
        let set = GainJson::set(&encoded);

        assert!(!set.is_empty());
        let fc = set.iter().find(|g| g.name == "fric_fc_counts").unwrap();
        assert_eq!(fc.raw, 20);
        let expect = gains::l_cd_from_si(0.5e-3, 33, 15_000, 18_200, 10_000).unwrap();
        assert!((plant.l_cd - expect).abs() < 1e-12);
        assert_eq!(t.f_ci, BwTargets::default().f_ci);
    }
}
