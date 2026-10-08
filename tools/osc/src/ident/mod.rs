//! `osc ident` - the identification subcommand: drives the osc-ident
//! experiments over the osc-adapter, records raw + derived CSVs, fits the
//! plant, synthesizes and encodes gains, and writes them back with
//! snapshot/rollback safety. The sans-io engine lives in osc-ident; this
//! wrapper owns USB, wall time, and files. TEL captures ride the main bus
//! as bursts (the pump's Stream arm) - no side channel. The run's order and
//! every duty it drives at come from osc-ident's `run`.

pub(crate) mod params;
mod verify;

use std::cell::Cell;
use std::path::{Path, PathBuf};

use crate::capture::envelope;
use crate::rig::centre::adopt_polarity;
use crate::rig::plant::Lut;
use crate::rig::pump::{self, Pump, read_i32, with_guard, write_reg};
use crate::rig::servo::{Servo, Wire};
use crate::rig::{Aborted, check_abort, csvio, snapshot};
use anyhow::{Context, Result, bail};
use clap::{Subcommand, ValueEnum};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::{self, DataState};
use osc_client::descriptor::Descriptor;
use osc_client::nusb::NusbPipe;
use osc_ident::burst::{Capture, Chans};
use osc_ident::exp::anchor::{Anchor, AnchorCfg, AnchorResult};
use osc_ident::exp::bias::{Bias, BiasCfg, BiasResult};
use osc_ident::exp::breakaway::{Breakaway, BreakawayCfg, BreakawayResult};
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::held::{HELD_STEP_MIN_PCT, Held, HeldCfg, NoHeldRoom, Stops, held_steps};
use osc_ident::exp::inductance::{
    Cfg as InductanceCfg, FitCfg, Inductance, InductanceResult, fit_captures,
};
use osc_ident::exp::inertia::{
    Inertia, InertiaCfg, InertiaResult, PriorsFrom, StoredMotion, fit_steps,
};
use osc_ident::exp::ladder::{Ladder, LadderCfg, LadderResult};
use osc_ident::exp::resistance::{Resistance, ResistanceCfg, ResistanceResult};
use osc_ident::exp::rl::{Rl, RlCfg, RlFitCfg, RlResult, Scales};
use osc_ident::exp::verify::{VERIFY_CURRENT, VerifyCurrentCfg, VerifyVelocityCfg};
use osc_ident::exp::wavefit::WaveCfg;
use osc_ident::exp::{Guarded, Permitted, RigParams};
use osc_ident::fits::{self, Climb, InertiaPriors};
use osc_ident::gains::{self, BwTargets, PlantParams};
use osc_ident::limits::{
    BurstAllowance, CLASS_R_MIN, DutyPlan, Envelope, POT_MAX, Refusal, STOP_LADDER, ServoLimits,
    guards, pct_floor, q15_floor,
};
use osc_ident::pot::Pot;
use osc_ident::regs::{calib, config, control, telemetry};
use osc_ident::report::{self, PlantInputs, ReportInputs};
use osc_ident::run::{self as order, Ended, Over, Run, Stage};
use osc_ident::runway::{Runway, Supply};
use osc_ident::sources::{self, HeldR, Source, Winding};
use osc_ident::thermometer;
use params::{
    BiasJson, BreakawayJson, GainJson, InductanceJson, InertiaJson, LadderJson, ParamsFile,
    PlantJson, PotJson, ResistanceJson, RlJson, SenseJson, StoredMotionJson, StoredWindingJson,
};

/// Where recorded runs land when `--out` is absent.
const DEFAULT_OUT: &str = "./ident-out";

const Q15: f64 = 32767.0;

/// Why the ladder declined, in the run's directory: the fit reads it back.
const LADDER_DECLINED: &str = "ladder_declined.txt";

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
    /// A stretch of travel to leave out of the fit, low end, counts, for a
    /// servo with a damaged spot; none by default.
    #[arg(long, global = true, requires = "slip_hi")]
    slip_lo: Option<u16>,
    /// The same stretch, high end, counts.
    #[arg(long, global = true, requires = "slip_lo")]
    slip_hi: Option<u16>,
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
    /// what 3.2 V less a 1% margin allows on this rail]; a rung over that
    /// is refused.
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
    /// vbus, bit 3 a shunt ahead of every extra), `driven`, the tap of the
    /// terminal each step drives, or `diff`, both terminals interleaved
    /// (mask 11). One channel keeps frame_len at 2 and sees the chopping leg
    /// through ON and OFF on both signs; `diff` keeps the shunt at the same
    /// rate and fits the winding on the terminals' difference, the low side
    /// measured rather than assumed. The unbuffered rail tap reads ~1% low
    /// in-burst.
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
    slip: Option<(u16, u16)>,
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
    /// The `drive_polarity` in force: the stored one until the jam check
    /// sees the motor turn against it.
    polarity: Cell<bool>,
    env: Envelope,
    sense: SenseJson,
    sc: Scales,
    /// The winding an earlier identification left on the servo.
    stored: Option<Winding>,
    /// The Ke and friction line an earlier identification left on the
    /// servo: its back-EMF speed checks the ladder's, and inertia reads B
    /// against them when the ladder declines.
    stored_motion: Option<StoredMotion>,
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
    /// the firmware limiter. When the burst declines, R comes from the
    /// stops if --stall-ladder asks for it and the supply leaves it room,
    /// else R and L are the ones the servo carries from an earlier
    /// identification; with neither the run stops, back at mid travel.
    /// The first stage that aborts ends the run. Nothing stalls a stop
    /// unless asked. Write-back stays explicit.
    Run {
        /// When the burst declines, measure winding R from the resistance
        /// stop ladder before falling back to the winding the servo
        /// carries: each stop stalled at up to four duties between the
        /// lowest duty the servo reads current and terminal voltage from
        /// and the current limit, stall
        /// permit held.
        #[arg(long)]
        stall_ladder: bool,
    },
    /// Torque-off noise and bias floor.
    Bias,
    /// The resistance stop ladder -> winding R: each stop stalled at up to
    /// four duties between the lowest duty the servo reads current and
    /// terminal voltage from and the current limit, stall permit held, then back to mid travel.
    Resistance,
    /// The toggle experiment: free-shaft duty toggles -> winding R and L
    /// (advisory; the 1 ms step is rotor-followed and biased). The jam
    /// check first; the toggles run free under the current limit, and the
    /// seeks back to mid travel drive at the duty whose stall it holds.
    Rl,
    /// High-rate shunt bursts -> winding R, L and tau: the front of `run`,
    /// then the held route at a stop with --burst-stops.
    Burst {
        /// The load is a resistor, not a motor: no jam check, no centring
        /// and no seek, the rungs burst in place under the stall permit.
        /// V0 is pinned at zero, the fit's tau, V0 and residual bounds are
        /// reported, not gated.
        #[arg(long)]
        static_load: bool,
    },
    /// Breakaway duty ramp: the front of `run`, then breakaway.
    Breakaway,
    /// Steady-state duty ladder -> Ke + friction line: the front of `run`
    /// through breakaway, then the ladder.
    Ladder,
    /// Duty-step transients -> B (TEL when wired): the front of `run`
    /// through the ladder, then inertia.
    Inertia,
    /// Write the winding thermometer on an identified servo: the jam check,
    /// a seated hold at the low stop under the current limit that reads the
    /// kernel's own winding R, back to mid travel; then the motor family's
    /// model (tau, R_th), the LMS step, the board NTC's curve and
    /// rtherm_i_min_counts (7/8 of the hold current) are written read-back
    /// verified, SAVE with --save. With --rested the hold's R also becomes
    /// the stored cold R (reduced to 25 C through the NTC): say it only
    /// after 11 min or more without current; the cold-R health check and
    /// the hot-reboot floor read it.
    Anchor {
        /// Persist with MGMT SAVE after the write.
        #[arg(long)]
        save: bool,
        /// The servo sat idle 11 min or more: the hold reads the cold R.
        #[arg(long)]
        rested: bool,
    },
    /// Closed-loop verification on the written gains: verify current, then
    /// verify velocity. Needs a clean data state: a set written but not
    /// SAVEd on a fresh servo is refused (`ident write --save` first).
    /// Every seek drives at the duty whose stall the current limit holds;
    /// verify current holds steps between the lowest current the sensor
    /// reads and the limit at each stop, stall permit held, and is refused
    /// on a supply that leaves no room between the two. Recorded under
    /// <out>/<timestamp>/: meta.json, verify_current.csv,
    /// verify_velocity.csv.
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
    ///   hold and step by hand at mid travel     closed loop, only now
    /// VIRGIN and STALE clear only on SAVE, so closed loop comes after --save.
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
        slip: args.slip_lo.zip(args.slip_hi),
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
        let stored = sources::stored(
            lim.r_q12,
            snapshot::read_u16(&mut c, id, config::I_KP_Q88)?,
            snapshot::read_u16(&mut c, id, config::I_KI_Q412)?,
            sense.tick_hz,
            cli.f_ci,
            &sc,
        );
        let stored_motion = StoredMotion::read(
            snapshot::read_u16(&mut c, id, calib::KE_VPC_Q)?,
            snapshot::read_u16(&mut c, id, calib::FRIC_FC_COUNTS)?,
            snapshot::read_u16(&mut c, id, calib::FRIC_FV_Q016)?,
        );
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
            polarity: Cell::new(lim.drive_polarity),
            env,
            sense,
            sc,
            stored,
            stored_motion,
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
            let d = drive(cli)?;
            let jog = Run::new(d.lim, &d.sc).with_stored_winding(d.stored).jog();
            let (b, _) = run_bias(cli, &mut c, id, &out, jog)?;
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
            let rungs = plan.stall_ladder(STOP_LADDER, d.lim.vdiff_floor())?;
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
        Cmd::Burst { static_load } => {
            let e8 = if *static_load {
                Some(run_static_bursts(cli, &mut c, id)?)
            } else {
                drive_stages(cli, &mut c, id, Until::Burst, false)?.e8
            };
            println!(
                "{}",
                render_partial(ReportInputs {
                    inductance: e8.as_ref(),
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
        Cmd::Anchor { save, rested } => run_anchor_alone(cli, &mut c, id, *save, *rested),
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
                println!("next: {}", hold_and_step(&mut c, id)?);
            } else if !*save {
                println!(
                    "next: ident write {} --save, then {}",
                    params.display(),
                    hold_and_step(&mut c, id)?
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

/// The first check new gains get: a hold at mid travel and a few steps
/// inside the travel guard, by hand, nowhere near a stop.
fn hold_and_step(c: &mut Client<NusbPipe>, id: Id) -> Result<String> {
    let soft = (
        read_i32(c, id, config::POS_MIN_SOFT_COUNTS)?,
        read_i32(c, id, config::POS_MAX_SOFT_COUNTS)?,
    );
    Ok(hold_and_step_hint(soft))
}

fn hold_and_step_hint(soft: (i32, i32)) -> String {
    let pot = |v: i32| v.clamp(0, POT_MAX) as u16;
    let (lo, hi) = guards((pot(soft.0), pot(soft.1)));
    format!(
        "hold and step by hand at mid travel: `osc set mode Position`, `osc set goal_position \
         {}`, `osc set torque_enable on`, a few goal_position steps inside {lo}..{hi}, then \
         `osc set torque_enable off`",
        (pot(soft.0) as u32 + pot(soft.1) as u32) / 2
    )
}

fn parse_chans(s: &str) -> Result<Chans, String> {
    Chans::parse(s).ok_or_else(|| format!("`{s}` is not `driven`, `diff` or a mask 0..=15"))
}

/// The subcommands that move the shaft.
fn drives(cmd: &Cmd) -> bool {
    matches!(
        cmd,
        Cmd::Run { .. }
            | Cmd::Bias
            | Cmd::Resistance
            | Cmd::Rl
            | Cmd::Burst { .. }
            | Cmd::Breakaway
            | Cmd::Ladder
            | Cmd::Inertia
            | Cmd::Anchor { .. }
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
        slip: cli.slip,
        pot: cli.pot(),
        ..RigParams::new(Some(d.env.guard), d.env.i_abort)
            .with_floor(d.lim.window_floor_q15)
            .with_stops(d.lim.raw)
            .with_polarity(d.polarity.get())
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
    let nudge = cfg.nudge;
    let params = rig(cli)?;
    let mut exp = Guarded::new(Centre::new(cfg, &params), params.without_pos_guard());
    with_guard(c, id, |c| Pump::new(c, id, None).run(&mut exp))?;
    check_abort(what, exp.abort())?;
    let exp = exp.into_inner();
    if !exp.arrived() {
        bail!("centring did not reach mid travel in 5 s (gear slipping?)");
    }
    if nudge {
        let d = drive(cli)?;
        d.polarity
            .set(adopt_polarity(c, id, &exp, d.polarity.get())?);
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

/// Torque off at rest, the rest noise from a TEL stream; a rest on the
/// position sensor's carry is jogged off at `jog` first.
fn run_bias(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    jog: f64,
) -> Result<(BiasResult, f64)> {
    println!(
        "[bias] torque off: the current zero, the rail and the position noise; a rest on the \
         position sensor's carry is jogged off at {}",
        pct(jog)
    );
    // at rest, or a jog of a few counts: the travel guard has nothing to guard
    let params = rig(cli)?.without_pos_guard();
    let mut log = csvio::SnapshotLog::create(out, "bias_snapshots.csv")?;
    let cfg = BiasCfg {
        jog_q15: q15_floor(jog),
        ..BiasCfg::default()
    };
    let mut exp = Guarded::new(Bias::new(cfg, &params), params);
    let tel = with_guard(c, id, |c| {
        let mut pump = Pump::new(c, id, Some(&mut log));
        pump.run(&mut exp)?;
        Ok(std::mem::take(&mut pump.tel))
    })?;
    check_abort("bias", exp.abort())?;
    csvio::write_tel_frames(out, "bias_tel.csv", &tel)?;
    let b = exp
        .into_inner()
        .result()
        .context("bias collected no samples")?;
    for w in &b.warnings {
        println!("  warn: {w}");
    }
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

/// The anchor hold: seated at the low stop with the permit held, the
/// kernel's own winding R from its aggregate windows ([`Anchor`]). The
/// windows land in anchor.csv for the offline fit.
fn run_anchor(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: AnchorCfg,
) -> Result<Result<AnchorResult, String>> {
    let params = rig(cli)?.without_pos_guard();
    let mut log = csvio::SnapshotLog::create(out, "anchor_snapshots.csv")?;
    let mut exp = Guarded::new(Permitted::new(Anchor::new(cfg, &params)), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("anchor", exp.abort())?;
    let exp = exp.into_inner().into_inner();
    csvio::write_anchor_samples(out, exp.samples())?;
    Ok(exp.result())
}

/// `osc ident anchor`: the thermometer set on an identified servo, written
/// and verified at once, SAVEd when asked. The model constants are the
/// motor family's ([`thermometer::MG90_TAU_S`], [`thermometer::MG90_R_TH_C_PER_W`]):
/// a seated hold's own rise carries contact steps that spread a per-unit
/// fit over a factor of two. The stamp covers the model and the NTC curve:
/// `osc stamp` after it.
fn run_anchor_alone(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    save: bool,
    rested: bool,
) -> Result<()> {
    let d = drive(cli)?;
    if d.stored.is_none() {
        bail!(
            "this servo carries no winding R from an earlier identification, so nothing plans \
             the hold; `osc ident run` identifies it first"
        );
    }
    let ntc = thermometer::NtcBoard {
        pullup_ohm: snapshot::read_u16(c, id, calib::NTC_PULLUP_OHM)? as f64,
        r25_ohm: snapshot::read_u16(c, id, calib::NTC_R25_OHM)? as f64,
        beta: snapshot::read_u16(c, id, calib::NTC_BETA)? as f64,
    };
    let ntc_raw = snapshot::read_u16(c, id, telemetry::NTC_RAW)?;
    let t_ntc_c = thermometer::ntc_beta_c(ntc_raw as f64, ntc.pullup_ohm, ntc.r25_ohm, ntc.beta)
        .context("the board NTC reads at a rail or the board has no NTC curve")?;
    let rec = drive_stages(cli, c, id, Until::Anchor, false)?;
    let Some(a) = rec.anchor else {
        bail!(
            "the hold declined: {}; nothing written",
            rec.declined.as_deref().unwrap_or("no hold was read")
        );
    };
    let enc = thermometer::Thermal::new(
        thermometer::MG90_TAU_S,
        thermometer::MG90_R_TH_C_PER_W,
        d.sc.v_term_per_count,
        d.sc.amps_per_count,
        a.i_counts.round().clamp(1.0, u16::MAX as f64) as u16,
        ntc,
        d.sense.tick_hz as f64,
    )
    .context("the thermometer set encodes to nothing (zero scale, hold or NTC)")?;
    let mut fields = GainJson::thermal(&enc);
    fields.push(GainJson::floor(thermometer::floor_counts(a.i_counts)));
    let cold = thermometer::cold_r_q12(a.r_vpc, t_ntc_c, rested);
    if let Some(r) = cold {
        fields.push(GainJson::cold_r(r));
    }
    write_reg(c, id, control::TORQUE_ENABLE, 0)?;
    snapshot::take_snapshot(c, id, &rec.out.0.join("snapshot.json"))?;
    snapshot::write_gains(c, id, &fields)?;
    for (name, f) in enc.fields() {
        println!("  {name:<12} {:>7}  ({:.5})", f.raw, f.physical);
    }
    println!("  {}", thermometer::floor_line(a.i_counts));
    match cold {
        Some(r) => println!(
            "  r0_q12       {:>7}  (cold R {:.4} vcounts/ccount at 25 C from {:.4} at the NTC's {t_ntc_c:.1} C)",
            r.raw, r.physical, a.r_vpc
        ),
        None => println!(
            "  r0_q12       not written: the hold read {:.4} vcounts/ccount at the NTC's {t_ntc_c:.1} C; \
             --rested after 11 min without current stores it as the cold R",
            a.r_vpc
        ),
    }
    println!(
        "the thermometer bases on the board NTC ({t_ntc_c:.1} C now), reads each seat's rise as a \
         ratio and carries across moves on the MG90 family model (tau {} s, {} C/W); the model \
         and the NTC curve are stamp-covered: osc stamp next",
        thermometer::MG90_TAU_S,
        thermometer::MG90_R_TH_C_PER_W
    );
    if save {
        crate::save(c, id)?;
    } else {
        println!("not saved: osc save persists it");
    }
    Ok(())
}

fn run_rl(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    sense: &SenseJson,
) -> Result<RlResult> {
    let d = drive(cli)?;
    let front = Run::new(d.lim, &d.sc);
    println!(
        "[centring] the jam check: out and back at mid travel from {}, raised up to {} while \
         the shaft does not move",
        pct(front.bootstrap()),
        pct(front.nudge_cap())
    );
    let cfg = order::centre_cfg(front.bootstrap(), front.nudge_cap(), true);
    let moved = centre(cli, c, id, cfg)?
        .context("the jam check never saw the shaft move, so no toggle may run")?;
    let plan = d.lim.stall_plan(d.sc.r_vpc(CLASS_R_MIN), Some(moved));
    println!(
        "[toggle] winding R/L: chained duty toggles on the free shaft at mid travel, the \
         current limit governing them; seeks back to mid travel at {}",
        pct(plan.seek)
    );
    let params = rig(cli)?;
    let sc = sense
        .scales()
        .context("CalibSense scales degenerate (shunt/gain/dividers/vdd)")?;
    let cfg = RlCfg {
        step_periods: cli.step_periods,
        fit: rl_fit_cfg(sense),
        ..RlCfg::planned(&plan)
    };
    let mut log = csvio::SnapshotLog::create(out, "rl_snapshots.csv")?;
    let mut exp = Guarded::new(Rl::new(cfg, &params, sc), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("toggle", exp.abort())?;
    println!("[centring] to mid travel at {}", pct(plan.seek));
    centre(cli, c, id, order::centre_cfg(plan.seek, plan.seek, false))?;
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

/// Bursts on a resistor in the motor's place: every rung in place, the
/// stall permit held, since its current flows with no motion.
fn run_static_bursts(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<InductanceResult> {
    let out = csvio::OutDir::create(&cli.out)?;
    println!("recording to {}", out.0.display());
    let d = drive(cli)?;
    let rail_mv = Run::new(d.lim, &d.sc).rail_mv();
    let rungs = match asked_rungs(cli, rail_mv)? {
        Some(r) => r,
        None => BurstAllowance::rungs(rail_mv)
            .context("the 3.2 V burst allowance leaves no two rungs on this rail")?
            .to_vec(),
    };
    let cfg = InductanceCfg {
        fit: FitCfg {
            wave: WaveCfg::default().static_load(),
            ..FitCfg::default()
        }
        .with_limit(d.lim.i_lim as f64 * d.sc.amps_per_count),
        static_load: true,
        ..order::burst_cfg(&rungs, 0.0, 0.0, burst_base(cli))
    };
    println!(
        "[burst, static load] in place at {}; stall permit held",
        cfg.step_pct
            .iter()
            .map(|p| format!("{p}%"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let params = rig(cli)?.without_pos_guard();
    let mut log = csvio::SnapshotLog::create(&out, "inductance_snapshots.csv")?;
    let mut exp = Guarded::new(Permitted::new(Inductance::new(cfg, &params, d.sc)), params);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("burst", exp.abort())?;
    let exp = exp.into_inner().into_inner();
    csvio::write_bursts(&out, exp.captures())?;
    exp.fit().context("the bursts gave nothing to fit")
}

/// The rungs --burst-pct asks for, refused over what a burst may drive on a
/// rail of `rail_mv`.
fn asked_rungs(cli: &Ctx, rail_mv: f64) -> Result<Option<Vec<f64>>> {
    let most = BurstAllowance::top(rail_mv);
    let explicit: Option<Vec<f64>> = cli
        .burst_pct
        .as_ref()
        .map(|p| p.iter().map(|p| *p as f64 / 100.0).collect());
    if let Some(hi) = explicit.iter().flatten().copied().reduce(f64::max)
        && hi > most
    {
        bail!(
            "a burst rung of {:.0}% is over the {} a burst may drive on this {:.1} V rail: 3.2 V \
             applied, less 1% in case the rail rises before the arm (leave --burst-pct out)",
            hi * 100.0,
            pct(most),
            rail_mv / 1000.0
        );
    }
    Ok(explicit)
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
/// two step duties up to its stop cap ([`held_steps`]); skipped before
/// seating when the limit leaves no room for two. Stalling a stop is the
/// method, so it never runs unless asked.
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
    let steps = match (&cli.burst_pct, held_steps(plan, hold)) {
        (Some(asked), _) => asked.clone(),
        (None, Ok(planned)) => planned.to_vec(),
        (None, Err(NoHeldRoom { hold_pct, cap_pct })) => {
            println!(
                "[burst, held] skipped before seating: the held route cannot resolve R under \
                 this limit of {}; it needs two step duties from {HELD_STEP_MIN_PCT}%, the \
                 shortest ON window a burst reads, up to the stop cap of {cap_pct}% over the \
                 {hold_pct}% hold",
                d.lim.ma().of(d.lim.i_lim as f64)
            );
            return Ok((Vec::new(), Vec::new()));
        }
    };
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

/// The ladder's rungs inside `runway`, and the runway as the rungs left it;
/// Err with the reason, in plain words, when the ladder gives the fit
/// nothing.
fn run_ladder(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    cfg: LadderCfg,
    runway: Runway,
    r_taps_vpc: f64,
) -> Result<(Result<LadderResult, String>, Runway, Vec<Climb>)> {
    let params = rig(cli)?;
    let mut log = csvio::SnapshotLog::create(out, "ladder_snapshots.csv")?;
    let aborts = params.abort_at_soft(drive(cli)?.lim.soft);
    let mut exp = Guarded::new(Ladder::new(cfg, &params, runway), aborts);
    with_guard(c, id, |c| Pump::new(c, id, Some(&mut log)).run(&mut exp))?;
    check_abort("ladder", exp.abort())?;
    let exp = exp.into_inner();
    let room = exp.runway().room();
    for (duty, n) in exp.sized() {
        println!(
            "  {:+.0}%: needs {:.0} of {room:.0} counts of travel (climb {:.0}, run {:.0}, stop \
             {:.0} at {:.1} counts/ms)",
            *duty as f64 / Q15 * 100.0,
            n.total(),
            n.climb,
            n.run,
            n.stop,
            n.v
        );
    }
    for w in exp.warnings() {
        println!("  {w}");
    }
    let runway = exp.runway().clone();
    let climbs = exp.climbs().to_vec();
    csvio::write_climbs(out, &climbs)?;
    if let Some(why) = exp.declined() {
        std::fs::write(out.0.join(LADDER_DECLINED), format!("{why}\n"))?;
        return Ok((Err(why.to_string()), runway, climbs));
    }
    let l = exp.fit(r_taps_vpc).context("ladder fit degenerate")?;
    csvio::write_rungs(out, &l.rungs)?;
    Ok((Ok(l), runway, climbs))
}

/// The ladder's runway: sized by the pilot envelope of the dataset that
/// describes this servo on this supply, else by the run's own rungs.
fn ladder_runway(d: &Drive, rail_mv: f64) -> Runway {
    let mut runway = Runway::new(d.env.guard);
    let (left, sized) = envelope::size_runway(&mut runway, Supply::of_rail(rail_mv), d.lim.phys);
    for (path, why) in left {
        println!("  left out {}: {why}", path.display());
    }
    if let Some(path) = sized {
        println!("  rungs sized by the pilot envelope {}", path.display());
        return runway;
    }
    println!(
        "  no pilot envelope describes this servo on this supply: each rung is sized from the \
         seek and the rungs before it"
    );
    runway
}

fn run_inertia(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    exp: Inertia,
    priors: &InertiaPriors,
    climbs: &[Climb],
) -> Result<Result<InertiaResult, String>> {
    let aborts = rig(cli)?.abort_at_soft(drive(cli)?.lim.soft);
    let mut log = csvio::SnapshotLog::create(out, "inertia_snapshots.csv")?;
    let mut exp = Guarded::new(exp, aborts);
    let all_tel = with_guard(c, id, |c| {
        let mut pump = Pump::new(c, id, Some(&mut log));
        pump.run(&mut exp)?;
        Ok(std::mem::take(&mut pump.tel))
    })?;
    check_abort("inertia", exp.abort())?;
    let exp = exp.into_inner();
    csvio::write_tel_frames(out, "inertia_tel.csv", &all_tel)?;
    csvio::write_step_series(out, &exp.step_series())?;
    Ok(exp.fit(priors, climbs))
}

fn run_verify(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let params = rig(cli)?;
    let (d, before) = servo_state(c, id)?;
    refuse_closed_loop(&before)?;
    let drv = drive(cli)?;
    let plan = drv.lim.stall_plan(drv.sc.r_vpc(CLASS_R_MIN), None);
    let current = VerifyCurrentCfg::planned(&plan, drv.lim.window_floor())?;
    centre_outside_the_run(cli, c, id)?;
    let ma = drv.lim.ma();
    println!(
        "[verify current] current steps of {} held at each stop, seeks at {}; stall permit held",
        current
            .steps_counts
            .iter()
            .map(|s| ma.of(*s as f64))
            .collect::<Vec<_>>()
            .join(" and "),
        pct(plan.seek)
    );
    let out = csvio::OutDir::create(&cli.out)?;
    let velocity = VerifyVelocityCfg::planned(&plan);
    let mut s = Wire::new(&mut *c, id);
    let v = verify::run(
        &mut s,
        &out,
        &drv.lim,
        params,
        current,
        velocity,
        |s, ended| {
            let c = s.client();
            if ended == VERIFY_CURRENT {
                refuse_closed_loop(&c.data_state(id, &d)?)?;
            }
            // E5 ends stalled against an end-stop; E6 runs with the pos
            // guard on and its first read would abort right there
            centre_outside_the_run(cli, c, id)?;
            if ended == VERIFY_CURRENT {
                println!(
                    "[verify velocity] velocity legs across the travel, parked at {}",
                    pct(plan.seek)
                );
            }
            Ok(())
        },
    )?;
    println!("verify: recorded in {}", out.0.display());
    let (cur, vel) = (v.current.as_ref(), v.velocity.as_ref());
    for s in cur.iter().flat_map(|c| &c.steps) {
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
    for l in vel.iter().flat_map(|v| &v.legs) {
        println!(
            "  goal {:+6} c/s -> {:+8.1} ({:.1}% err, r2 {:.4}, n {})",
            l.goal_cps, l.meas_cps, l.err_pct, l.r2, l.n
        );
    }
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

fn run_all(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id, stall_ladder: bool) -> Result<()> {
    let rec = drive_stages(cli, c, id, Until::End, stall_ladder)?;
    let (Some((bias, _)), Some(breakaway)) = (&rec.bias, &rec.breakaway) else {
        bail!("the run ended without its bias or breakaway");
    };
    let p = ParamsFile {
        bias: Some(BiasJson::from(bias)),
        inductance: rec.e8.as_ref().map(InductanceJson::from),
        breakaway: Some(BreakawayJson::from(breakaway)),
        stored_winding: rec
            .w
            .filter(|w| w.r_from == Source::Stored)
            .as_ref()
            .map(StoredWindingJson::from),
        stored_motion: drive(cli)?
            .stored_motion
            .as_ref()
            .map(StoredMotionJson::from),
        sense: Some(drive(cli)?.sense),
        pot: cli.lut.as_ref().map(PotJson::from),
        thermometer: None,
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
    /// The anchor alone ([`Run::for_anchor`]).
    Anchor,
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
                | (Until::Anchor, Stage::Anchor { .. })
        )
    }
}

/// The run's stages in the order osc-ident's [`Run`] names them, through
/// `until` and the closing centring. The first abort ends it. A burst that
/// declines hands over to the resistance stop ladder when `stall_ladder`
/// asks and it has room, else to the winding the servo carries; with
/// neither the run ends after its closing centring.
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
    let mut run = match until {
        Until::Anchor => Run::for_anchor(d.lim, &d.sc),
        _ => Run::new(d.lim, &d.sc),
    }
    .with_stored_winding(d.stored)
    .with_stored_motion(d.stored_motion.is_some());
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
        anchor: None,
        declined: None,
        runway: None,
        climbs: Vec::new(),
    };
    while let Some(stage) = run.next_stage() {
        if rec.w.is_none()
            && let (Some((w, why)), Some(plan)) = (run.reused(), run.plan())
        {
            if until != Until::Burst {
                let e8 = rec.e8.as_ref();
                for line in reuse_note(&w, why, &gates(e8), sources::stale(e8, &w)) {
                    println!("{line}");
                }
                say_plan(&plan, &d.lim);
            }
            rec.w = Some(w);
        }
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
        // the fit keeps what came before it, and says what is missing
        Some(Over::Declined("inertia" | "ladder")) if until == Until::End => Ok(rec),
        Some(Over::Declined("anchor")) => bail!(
            "no winding R to plan the hold with: this servo carries none from an earlier \
             identification; `osc ident run --ambient-c <C>` identifies it and anchors the \
             thermometer in the same run"
        ),
        Some(Over::Declined(stage)) if stage != "burst" => bail!(
            "the {stage} declined: {}; the run ended at mid travel with nothing to fit",
            rec.declined.as_deref().unwrap_or("nothing fitted")
        ),
        Some(Over::Declined("burst")) if until != Until::Burst => bail!(
            "{}",
            sources::unmeasured(
                &rec.e8
                    .as_ref()
                    .map_or("the bursts gave nothing to fit".into(), |r| r.reason())
            )
        ),
        Some(Over::NoLadderRoom { floor, cap }) => bail!(
            "no winding R: the burst measured none{}, and {}",
            match d.stored {
                Some(_) => "",
                None => ", this servo carries no winding from an earlier identification",
            },
            Refusal::NoLadderRoom {
                what: STOP_LADDER,
                floor,
                cap
            }
        ),
        Some(Over::ResistanceDeclined) => bail!(
            "no winding R: neither the burst nor the resistance stop ladder fitted one and this \
             servo carries no winding from an earlier identification, so the run stops here, \
             back at mid travel"
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

/// The burst's failed gates, as the messages name them.
fn gates(e8: Option<&InductanceResult>) -> String {
    e8.map_or("no fit".into(), |r| r.blocking().join(", "))
}

/// Why the run plans from the winding the servo carries, in plain words,
/// and a warning when the burst's own rough R, `stale`, disagrees with it.
fn reuse_note(w: &Winding, why: Over, gates: &str, stale: Option<f64>) -> Vec<String> {
    let ladder = match why {
        Over::NoLadderRoom { floor, cap } => format!(
            " and the resistance stop ladder has no room on this supply (the servo reads \
             current and terminal voltage from {:.1}% duty, the current limit allows {:.1}% at a stop)",
            floor * 100.0,
            cap * 100.0
        ),
        Over::ResistanceDeclined => " and the resistance stop ladder fitted none".into(),
        _ => String::new(),
    };
    let r = w.r_ohm.unwrap_or_default();
    let mut lines = vec![
        format!(
            "[winding] the burst measured no winding R ({gates}){ladder}, so the run uses the R \
             and L this servo already carries from an earlier identification: R {r:.2} ohm, L \
             {:.3} mH",
            w.l_h * 1e3
        ),
        "  they still hold: the winding belongs to the motor and does not change with the \
         position table or the gear train"
            .into(),
    ];
    if let Some(rough) = stale {
        lines.push(format!(
            "warning: the burst's rough R, {rough:.2} ohm, is {:.0}% away from the stored \
             {r:.2} ohm, so the stored winding may be stale; `osc ident run --stall-ladder` \
             measures it again from stop stalls, on a lower supply (USB) when this one leaves \
             the stop ladder no room",
            (rough / r - 1.0).abs() * 100.0
        ));
    }
    lines
}

/// The R the held route plans from and the line that names its source;
/// Err is the line that says why the held route is skipped.
fn held_r(
    fitted: Option<&InductanceResult>,
    stored: Option<f64>,
    sc: &Scales,
) -> Result<(f64, String), String> {
    let ohm = |r_vpc: f64| r_vpc / sc.r_vpc(1.0);
    let declined = match fitted {
        Some(f) => format!(
            "the free fit declined on {} ({})",
            gates(fitted),
            f.reason()
        ),
        None => "the bursts gave nothing to fit".into(),
    };
    match sources::held_r(fitted, stored, sc) {
        Some((r, HeldR::Promoted)) => Ok((
            r,
            format!(
                "[burst, held] hold and stop duties from the free fit's R, {:.2} ohm",
                ohm(r)
            ),
        )),
        Some((r, HeldR::Stored)) => Ok((
            r,
            format!(
                "[burst, held] {declined}: hold and stop duties from the R stored on the servo, \
                 {:.2} ohm",
                ohm(r)
            ),
        )),
        Some((r, HeldR::Declined)) => Ok((
            r,
            format!(
                "[burst, held] {declined} and nothing is stored on the servo: hold and stop \
                 duties from the fit's own waveform R, {:.2} ohm",
                ohm(r)
            ),
        )),
        None => Err(format!(
            "[burst, held] skipped: no winding R to size the hold and stop duties ({declined}, \
             nothing is stored on the servo, and the fit has no waveform R)"
        )),
    }
}

/// The stall-safe duties every drive from here plans with.
fn say_plan(plan: &DutyPlan, lim: &ServoLimits) {
    println!(
        "[plan] every drive that could stall from here stays under the current limit of {}: \
         seeks at {}, stop drives up to {}",
        lim.ma().of(lim.i_lim as f64),
        pct(plan.seek),
        pct(plan.stop_cap)
    );
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
    /// The thermometer's anchor hold.
    anchor: Option<AnchorResult>,
    /// Why a stage gave the fit nothing, in plain words.
    declined: Option<String>,
    /// What the ladder measured: inertia runs inside the same runway.
    runway: Option<Runway>,
    /// The ladder's governed climbs: inertia's cross-check.
    climbs: Vec<Climb>,
}

impl Recorded {
    /// The winding the run plans from, from here on.
    fn planned(&mut self, w: Winding, run: &mut Run, lim: &ServoLimits) -> Result<()> {
        run.measured(w.r_vpc);
        let plan = run.plan().context("no plan")?;
        println!(
            "[winding] R {:.4} vcounts/ccount from {} (the plan), {:.4} between the taps \
             (r_q12), current loop R {:.4}, L {:.4} mH from {}",
            w.r_vpc,
            w.r_from.as_str(),
            w.r_taps_vpc,
            w.r_loop_vpc,
            w.l_h * 1e3,
            w.l_from.as_str()
        );
        say_plan(&plan, lim);
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
            Stage::Bias => self.bias = Some(run_bias(cli, c, id, out, run.jog())?),
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
                let explicit = asked_rungs(cli, run.rail_mv())?;
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
                let at = FitCfg::default().with_limit(d.lim.i_lim as f64 * d.sc.amps_per_count);
                let fitted = fit_captures(&self.caps, &d.sc, &at);
                if let (Some(stops), Until::Burst) = (cli.burst_stops, until) {
                    match held_r(fitted.as_ref(), d.lim.r_vpc(), &d.sc) {
                        Ok((r_vpc, line)) => {
                            println!("{line}");
                            let plan = DutyPlan::new(&d.lim, r_vpc, None);
                            let (caps, notes) = run_held(cli, c, id, out, stops, &plan, r_vpc)?;
                            self.caps.extend(caps);
                            self.notes.extend(notes);
                        }
                        Err(line) => println!("{line}"),
                    }
                }
                csvio::write_bursts(out, &self.caps)?;
                let mut fitted = fit_captures(&self.caps, &d.sc, &at);
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
                let r_taps = self.w.as_ref().context("no winding R")?.r_taps_vpc;
                let base = LadderCfg {
                    servo_ke: d.stored_motion.is_some(),
                    ..LadderCfg::new(d.sense.tick_hz as f64)
                };
                let cfg = order::ladder_cfg(*seek, rungs, base);
                let runway = ladder_runway(d, run.rail_mv());
                let (ladder, runway, climbs) = run_ladder(cli, c, id, out, cfg, runway, r_taps)?;
                self.runway = Some(runway);
                self.climbs = climbs;
                match ladder {
                    Ok(l) => self.ladder = Some(l),
                    Err(why) => {
                        self.declined = Some(why);
                        return Ok(Ended::Declined);
                    }
                }
            }
            Stage::Inertia { seek, base } => {
                println!(
                    "[inertia] steps from a moving base at {}, each sized to what the current \
                     limit leaves over the base's running current; seeks at {}",
                    pct(*base),
                    pct(*seek)
                );
                let r_loop = self.w.as_ref().context("no winding R")?.r_loop_vpc;
                let tick_hz = d.sense.tick_hz as f64;
                let (priors, from) = match osc_ident::exp::inertia::priors(
                    self.ladder.as_ref(),
                    d.stored_motion,
                    r_loop,
                    tick_hz,
                ) {
                    Ok(p) => p,
                    Err(why) => {
                        println!("  {why}");
                        self.declined.get_or_insert(why);
                        return Ok(Ended::Declined);
                    }
                };
                if from == PriorsFrom::Stored {
                    println!("  B is read against {}: the ladder declined", from.as_str());
                }
                let cfg = InertiaCfg {
                    capture_ms: cli.inertia_ms,
                    ..InertiaCfg::new(tick_hz)
                };
                let cfg = order::inertia_cfg(*seek, *base, cfg);
                let plan = run.plan().context("no plan")?;
                let runway = self.runway.clone().context("no ladder runway")?;
                let exp = Inertia::new(cfg, plan, runway, &rig(cli)?);
                match run_inertia(cli, c, id, out, exp, &priors, &self.climbs)? {
                    Ok(r) => self.inertia = Some(r),
                    Err(why) => {
                        println!("  {why}");
                        self.declined.get_or_insert(why);
                        return Ok(Ended::Declined);
                    }
                }
            }
            Stage::Anchor { seek, hold } => {
                println!(
                    "[anchor] seek the low stop at {}, hold at {} (the current limit governs) \
                     for a second: the kernel's own winding R at rest, for r0_q12; pos guard \
                     off, stall permit held",
                    pct(*seek),
                    pct(*hold)
                );
                let cfg = order::anchor_cfg(*seek, *hold, AnchorCfg::default());
                match run_anchor(cli, c, id, out, cfg)? {
                    Ok(a) => {
                        println!(
                            "  R0 {:.4} vcounts/ccount, median of {} windows (scatter {:.1}%), \
                             held at {} drawing {}",
                            a.r_vpc,
                            a.n,
                            a.spread * 100.0,
                            pct(a.duty),
                            d.lim.ma().of(a.i_counts)
                        );
                        self.anchor = Some(a);
                    }
                    Err(why) => {
                        println!("  the anchor declined: {why}");
                        self.declined.get_or_insert(why);
                        return Ok(Ended::Declined);
                    }
                }
            }
            Stage::Stops { .. } | Stage::Traverse { .. } => {
                bail!("osc ident has no {} stage: it is osc cal's", stage.name())
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
        (caps, Some(sc)) => {
            let log = dir.join("inductance_snapshots.csv");
            let i_lim = csvio::read_current_limit(&log)?.with_context(|| {
                format!(
                    "{} holds no current limit: the burst's line is read at the servo's limit",
                    log.display()
                )
            })?;
            let at = FitCfg::default().with_limit(i_lim as f64 * sc.amps_per_count);
            osc_ident::exp::inductance::fit_captures(caps, &sc, &at)
        }
    };
    // E2 is refitted whenever the run recorded it, but the winding takes it
    // only behind a declined E8. A ladder that fitted nothing is why a run
    // took the stored winding.
    let resistance = match dir.join("resistance.csv").exists() {
        true => match Resistance::fit_samples(&csvio::read_dwell_samples(&dir)?) {
            None if p.stored_winding.is_some() => None,
            r => Some(r.context("resistance refit degenerate")?),
        },
        false => None,
    };
    let w = sources::winding(
        inductance.as_ref(),
        resistance.as_ref(),
        sc.as_ref(),
        cli.l_henries,
    )
    .or_else(|| p.stored_winding.map(|s| s.winding()))
    .context(
        "no winding R: burst is missing or declined, no resistance recording exists and the \
         run took no stored winding",
    )?;
    let bias_res = p.bias.as_ref().map(BiasJson::result);
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
    p.resistance = resistance.as_ref().map(ResistanceJson::from);
    p.rl = rl.as_ref().map(RlJson::from);
    p.inductance = inductance.as_ref().map(InductanceJson::from);
    let climbs = csvio::read_climbs(&dir)?;
    let series = match dir.join("inertia_steps.csv").exists() {
        true => csvio::read_step_series(&dir)?,
        false => Vec::new(),
    };
    if !dir.join("rungs.csv").exists() {
        let why = match std::fs::read_to_string(dir.join(LADDER_DECLINED)) {
            Ok(why) => format!("the ladder declined ({})", why.trim()),
            Err(_) => "the run recorded no ladder".into(),
        };
        // a declined ladder hands inertia nothing: B, for the report, reads
        // against the Ke and friction the servo carried, when it did
        let stored = p.stored_motion.map(|m| m.motion());
        let inertia = match (series.is_empty(), stored) {
            (false, Some(_)) => {
                let (priors, from) =
                    osc_ident::exp::inertia::priors(None, stored, w.r_loop_vpc, tick_hz)
                        .map_err(anyhow::Error::msg)?;
                fit_steps(&series, &climbs, &priors).ok().map(|mut r| {
                    r.warnings
                        .insert(0, format!("B read against {}", from.as_str()));
                    p.inertia = Some(InertiaJson::new(&r, from.as_str()));
                    r
                })
            }
            _ => None,
        };
        let text = report::render(&ReportInputs {
            bias: bias_res.as_ref(),
            resistance: resistance.as_ref(),
            rl: rl.as_ref(),
            inductance: inductance.as_ref(),
            breakaway: bk_res.as_ref(),
            inertia: inertia.as_ref(),
            ..Default::default()
        });
        println!("{text}");
        std::fs::write(dir.join("report.txt"), &text)?;
        p.save(&path)?;
        println!("params: {}", path.display());
        bail!(
            "no gains: {why}, and the gains are built on its Ke and friction line; report.txt \
             and params.json in {} keep what did fit - bias, winding and breakaway - but there \
             is no gain set to write: run `osc ident run` again",
            dir.display()
        );
    }
    let rungs = csvio::read_rungs(&dir)?;
    let pts: Vec<fits::RungPoint> = csvio::read_rung_points(&dir)?;
    let ke = fits::ke_fit(&pts, w.r_taps_vpc).context("ke refit degenerate")?;
    let fric_fwd = fits::friction_line(&pts, 1);
    let fric_rev = fits::friction_line(&pts, -1);
    let ladder = LadderResult {
        ke,
        fric_fwd,
        fric_rev,
        servo: osc_ident::exp::ladder::servo_check(&rungs),
        rungs,
        warnings: Vec::new(),
    };
    let (priors, from) =
        osc_ident::exp::inertia::priors(Some(&ladder), None, w.r_loop_vpc, tick_hz)
            .map_err(anyhow::Error::msg)?;

    p.ladder = Some(LadderJson {
        ke_vpc: ladder.ke.ke_vpc,
        ke_r2: ladder.ke.r2,
        fc_fwd: ladder.fric_fwd.map(|f| f.fc),
        fv_fwd: ladder.fric_fwd.map(|f| f.fv),
        fc_rev: ladder.fric_rev.map(|f| f.fc),
        fv_rev: ladder.fric_rev.map(|f| f.fv),
        rungs_used: ladder.rungs.iter().filter(|r| r.used).count(),
    });

    let inertia = match fit_steps(&series, &climbs, &priors) {
        Ok(r) => r,
        Err(why) => {
            let text = report::render(&ReportInputs {
                bias: bias_res.as_ref(),
                resistance: resistance.as_ref(),
                rl: rl.as_ref(),
                inductance: inductance.as_ref(),
                breakaway: bk_res.as_ref(),
                ladder: Some(&ladder),
                ..Default::default()
            });
            println!("{text}");
            std::fs::write(dir.join("report.txt"), &text)?;
            p.save(&path)?;
            println!("params: {}", path.display());
            bail!(
                "no gains: the inertia declined ({why}), and the gains are built on its B; \
                 report.txt and params.json in {} keep what did fit - bias, winding, breakaway \
                 and ladder - but there is no gain set to write: run `osc ident run` again",
                dir.display()
            );
        }
    };

    let sigma_theta = bias_res.as_ref().map_or(1.0, |b| b.sigma_theta);
    let sigma_from = match bias_res {
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
        r_vpc: w.r_taps_vpc,
        r_loop_vpc: w.r_loop_vpc,
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

    p.inertia = Some(InertiaJson::new(&inertia, from.as_str()));
    p.plant = Some(PlantJson::new(&plant, &t, &w, sigma_from.as_str()));
    p.gains = GainJson::set(&encoded);
    p.save(&path)?;
    println!("params: {}", path.display());
    println!(
        "next: ident write {} --save, then hold and step by hand at mid travel",
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
        r_loop_vpc: pick(p.r_loop_vpc, p.r_vpc),
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
        r_taps_vpc: plant.r_vpc,
        r_from: Source::Default,
        r_loop_vpc: plant.r_loop_vpc,
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

fn render_partial(inputs: ReportInputs<'_>) -> String {
    report::render(&inputs)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// After a write the owner is sent to a hold and a few steps at mid
    /// travel, not to a drive into both stops.
    #[test]
    fn write_hints_the_hold_and_step_check() {
        let hint = hold_and_step_hint((432, 3626));
        assert_eq!(
            hint,
            "hold and step by hand at mid travel: `osc set mode Position`, `osc set \
             goal_position 2029`, `osc set torque_enable on`, a few goal_position steps inside \
             532..3526, then `osc set torque_enable off`"
        );
        assert!(!hint.contains("verify"));
    }

    fn ctx(out: PathBuf) -> Ctx {
        Ctx {
            baud: "auto".into(),
            out,
            guard: (None, None),
            slip: None,
            i_abort: None,
            l_henries: gains::DEFAULT_L_HENRIES,
            step_periods: 20,
            burst_pct: None,
            burst_repeats: 5,
            burst_i_max: None,
            burst_chans: Chans::Driven,
            burst_hold_pct: None,
            burst_stops: None,
            burst_hold_repeats: 4,
            inertia_ms: 150,
            gear_ratio: None,
            f_ci: 1000.0,
            f_cv: 200.0,
            f_cp: 25.0,
            f_o: 15.0,
            lut: None,
            drive: None,
        }
    }

    /// A run's front as it lands on disk: the stop ladder's dwells for the
    /// winding R, and params.json with the bias, breakaway and sense.
    fn record_front(tag: &str) -> (PathBuf, csvio::OutDir, f64) {
        use osc_ident::exp::WindowSample;
        use osc_ident::exp::resistance::DwellSample;

        let dir = std::env::temp_dir().join(format!("ident-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let out = csvio::OutDir(dir.clone());
        let r = 7270.0 / 4096.0;
        let mut dwells = Vec::new();
        for (k, duty) in [(0u32, 16_000.0), (1, -16_000.0)] {
            for i in [100.0, 150.0, 200.0] {
                let i = i * f64::signum(duty);
                dwells.push(DwellSample {
                    dwell: k,
                    dir: f64::signum(duty) as i8,
                    w: WindowSample {
                        t_ms: k as f64 * 100.0 + i.abs(),
                        i,
                        vdiff: r * i.abs() * 32767.0 / duty,
                        duty_q15: duty,
                    },
                });
            }
        }
        csvio::write_dwell_samples(&out, &dwells).unwrap();
        let bias = BiasJson {
            sigma_theta: 1.2,
            sigma_raw: 1.2,
            gain: 1.0,
            rest: 2029.0,
            tel_n: 10_000,
            pos_mean: 2029.0,
            i_noise: 1.5,
            i_bias_delta: 0.0,
            vbus_mean: 3204.0,
            vbus_sd: 2.0,
            n: 200,
            warnings: Vec::new(),
        };
        let breakaway = BreakawayJson {
            duty_bk_fwd: Some(2621),
            duty_bk_rev: Some(2800),
            fric_fwd_counts: Some(80.0),
            fric_rev_counts: Some(85.0),
            model_derived: false,
            asymmetry: Some(0.06),
        };
        ParamsFile {
            bias: Some(bias),
            breakaway: Some(breakaway),
            sense: Some(SenseJson {
                shunt_r_mohm: 60,
                gain_milli: 15_000,
                vmotor_div_top: 6_800,
                vmotor_div_bot: 3_300,
                tick_hz: 20_100,
                vdd_mv: 3_300,
                vbus_div_top_ohm: 15_000,
                vbus_div_bot_ohm: 10_000,
            }),
            ..Default::default()
        }
        .save(&dir.join("params.json"))
        .unwrap();

        (dir, out, r)
    }

    /// A run on the bench servo that fitted R, breakaway and the ladder,
    /// whose inertia steps gave nothing: the fit writes the report and
    /// params.json with all of it, names what is missing and what that
    /// means for the gains, and ends in an error only once they are on
    /// disk.
    #[test]
    fn a_failed_inertia_fit_keeps_the_rest_of_the_run() {
        use osc_ident::exp::ladder::RungSummary;

        let (dir, out, r) = record_front("inertia");
        let rungs: Vec<RungSummary> = [1500.0, 3000.0, 4500.0, -1500.0, -3000.0, -4500.0]
            .into_iter()
            .map(|omega: f64| {
                let i = omega.signum() * (80.0 + 0.004 * omega.abs());
                RungSummary {
                    duty_q15: (omega / 5.0) as i16,
                    omega,
                    omega_r2: 0.99,
                    i,
                    v: r * i + 0.1472 * omega,
                    windows: 30,
                    used: true,
                    note: None,
                    omega_bemf: None,
                }
            })
            .collect();
        csvio::write_rungs(&out, &rungs).unwrap();
        csvio::write_step_series(&out, &[]).unwrap();

        let err = fit_dir(&ctx(dir.clone()), dir.clone()).unwrap_err();
        assert_eq!(
            err.to_string(),
            format!(
                "no gains: the inertia declined (no inertia step was captured to fit), and the \
                 gains are built on its B; report.txt and params.json in {} keep what did fit - \
                 bias, winding, breakaway and ladder - but there is no gain set to write: run \
                 `osc ident run` again",
                dir.display()
            )
        );
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        assert!(p.bias.is_some() && p.breakaway.is_some());
        let res = p.resistance.expect("the winding R");
        assert!((res.r_vpc - r).abs() < 1e-6, "{}", res.r_vpc);
        let ladder = p.ladder.expect("the ladder");
        assert!((ladder.ke_vpc - 0.1472).abs() < 1e-6, "{}", ladder.ke_vpc);
        assert!(p.inertia.is_none() && p.plant.is_none() && p.gains.is_empty());
        assert!(std::fs::read_to_string(dir.join("report.txt")).is_ok_and(|t| !t.is_empty()));

        // steps whose current decays read B from 0.08 to 0.13: declined, and
        // the rest of the run kept the same way
        use osc_ident::fits::StepSeries;
        let scattered: Vec<(StepSeries, bool)> = [0.08, 0.13, 0.10, 0.12, 0.09, 0.11]
            .iter()
            .enumerate()
            .map(|(k, b)| {
                let sgn = if k % 2 == 0 { 1.0 } else { -1.0 };
                let tau = 1.0 / (b * 2010.0 * (0.004 + 0.1472 / r));
                let t: Vec<f64> = (0..3000).map(|k| k as f64 / 20_100.0).collect();
                let i = t
                    .iter()
                    .map(|t| sgn * (80.0 + 90.0 * (-t / tau).exp()))
                    .collect();
                let s = StepSeries {
                    mask: vec![true; t.len()],
                    pos: vec![2029.0; t.len()],
                    t,
                    i,
                    duty_q15: sgn * 9000.0,
                };
                (s, true)
            })
            .collect();
        csvio::write_step_series(&out, &scattered).unwrap();
        let err = fit_dir(&ctx(dir.clone()), dir.clone()).unwrap_err();
        assert!(
            err.to_string().starts_with(
                "no gains: the inertia declined (B spreads 18% across the 6 inertia steps, over \
                 the 10% one fit may: the fit is not trusted)"
            ),
            "{err}"
        );
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        assert!(p.ladder.is_some() && p.inertia.is_none() && p.gains.is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A run on the bench servo that fitted R and breakaway, whose ladder
    /// declined: the fit writes the report and params.json with what did
    /// fit, says why the ladder gave nothing and that the gains need it,
    /// and ends in an error only once they are on disk.
    #[test]
    fn a_declined_ladder_keeps_the_rest_of_the_run() {
        use osc_ident::exp::ladder::Declined;

        let (dir, _, r) = record_front("ladder");
        let why = Declined::Thin { rungs: 2 }.to_string();
        std::fs::write(dir.join(LADDER_DECLINED), format!("{why}\n")).unwrap();
        let err = fit_dir(&ctx(dir.clone()), dir.clone()).unwrap_err();
        assert_eq!(
            err.to_string(),
            format!(
                "no gains: the ladder declined (2 of the ladder's rungs ran both ways inside the \
                 travel, and the fit needs 3), and the gains are built on its Ke and friction \
                 line; report.txt and params.json in {} keep what did fit - bias, winding and \
                 breakaway - but there is no gain set to write: run `osc ident run` again",
                dir.display()
            )
        );
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        assert!(p.bias.is_some() && p.breakaway.is_some());
        let res = p.resistance.expect("the winding R");
        assert!((res.r_vpc - r).abs() < 1e-6, "{}", res.r_vpc);
        assert!(p.ladder.is_none() && p.inertia.is_none());
        assert!(p.plant.is_none() && p.gains.is_empty());
        let report = std::fs::read_to_string(dir.join("report.txt")).unwrap();
        assert!(!report.is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A run whose ladder declined on a servo that carries a Ke and
    /// friction line: inertia never takes the ladder's, the report and
    /// params.json give B read against the stored ones, and the gains still
    /// wait on a ladder.
    #[test]
    fn a_declined_ladder_reads_inertia_against_the_stored_motion() {
        use osc_ident::exp::ladder::Declined;
        use osc_ident::fits::StepSeries;

        let (dir, out, r) = record_front("stored-motion");
        let why = Declined::Disagrees { off: -0.16 }.to_string();
        std::fs::write(dir.join(LADDER_DECLINED), format!("{why}\n")).unwrap();
        let mut p = ParamsFile::load(&dir.join("params.json")).unwrap();
        p.stored_motion = Some(StoredMotionJson {
            ke_vpc: 0.1417,
            fc: 53.0,
            fv: 0.0051,
        });
        p.save(&dir.join("params.json")).unwrap();
        let b = 0.335;
        let tau = 1.0 / (b * 2010.0 * (0.0051 + 0.1417 / r));
        let steps: Vec<(StepSeries, bool)> = [9000.0, -9000.0, 11000.0, -11000.0]
            .into_iter()
            .map(|duty: f64| {
                let t: Vec<f64> = (0..3000).map(|k| k as f64 / 20_100.0).collect();
                let i = t
                    .iter()
                    .map(|t| duty.signum() * (80.0 + 90.0 * (-t / tau).exp()))
                    .collect();
                let s = StepSeries {
                    mask: vec![true; t.len()],
                    pos: vec![2029.0; t.len()],
                    t,
                    i,
                    duty_q15: duty,
                };
                (s, true)
            })
            .collect();
        csvio::write_step_series(&out, &steps).unwrap();
        let err = fit_dir(&ctx(dir.clone()), dir.clone()).unwrap_err();
        assert!(
            err.to_string()
                .starts_with(&format!("no gains: the ladder declined ({why})")),
            "{err}"
        );
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        let inertia = p.inertia.expect("B for the report");
        assert_eq!(inertia.priors, "the servo's stored Ke and friction");
        assert!(
            (inertia.b_best / b - 1.0).abs() < 0.01,
            "{}",
            inertia.b_best
        );
        assert!(p.ladder.is_none() && p.plant.is_none() && p.gains.is_empty());
        let report = std::fs::read_to_string(dir.join("report.txt")).unwrap();
        assert!(
            report.contains("warn: B read against the servo's stored Ke and friction"),
            "{report}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

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

    /// No stretch of travel is left out of the fit unless the owner names
    /// one: without the flags there is no slip zone, and the rig masks
    /// nothing. Named, it takes both ends.
    #[test]
    fn no_slip_zone_by_default() {
        use clap::Parser;

        #[derive(Parser)]
        struct Osc {
            #[command(flatten)]
            args: Args,
        }
        let parse = |argv: &[&str]| Osc::try_parse_from(argv).map(|o| o.args);
        let args = parse(&["osc", "show"]).unwrap();
        assert_eq!(args.slip_lo.zip(args.slip_hi), None);
        let rig = RigParams::new(Some((532, 3526)), 350);
        assert_eq!(rig.slip, None);
        assert!((0..=4095).all(|p| !rig.in_slip(p)));

        let args = parse(&["osc", "--slip-lo", "1250", "--slip-hi", "1650", "show"]).unwrap();
        assert_eq!(args.slip_lo.zip(args.slip_hi), Some((1250, 1650)));
        assert!(parse(&["osc", "--slip-lo", "1250", "show"]).is_err());
        assert!(parse(&["osc", "--slip-hi", "1650", "show"]).is_err());

        let burst = |argv: &[&str]| match parse(argv).unwrap().cmd {
            Cmd::Burst { static_load } => static_load,
            other => panic!("{other:?}"),
        };
        assert!(!burst(&["osc", "burst"]));
        assert!(burst(&[
            "osc",
            "--burst-pct",
            "25,40",
            "burst",
            "--static-load"
        ]));
    }

    /// The burst channels stay `driven` unless named; `diff` interleaves
    /// both terminals behind the shunt (mask 11), and a mask past 15 is
    /// refused.
    #[test]
    fn burst_chans_default_to_driven_and_take_diff() {
        use clap::Parser;
        use osc_ident::burst::CHANS_DIFF;

        #[derive(Parser)]
        struct Osc {
            #[command(flatten)]
            args: Args,
        }
        let chans = |argv: &[&str]| Osc::try_parse_from(argv).map(|o| o.args.burst_chans);
        assert_eq!(chans(&["osc", "burst"]).unwrap(), Chans::Driven);
        let diff = chans(&["osc", "--burst-chans", "diff", "burst"]).unwrap();
        assert_eq!(diff, Chans::Diff);
        assert_eq!(diff.for_step(-8520, true), CHANS_DIFF);
        assert_eq!(CHANS_DIFF, 11);
        assert!(chans(&["osc", "--burst-chans", "16", "burst"]).is_err());
    }

    /// The note a run prints once when it falls back to the winding the
    /// servo carries: what the burst failed, the stored R and L in ohm and
    /// mH, why they hold, and a warning only when the burst's rough R is
    /// far from them.
    #[test]
    fn the_reuse_note_says_why_the_stored_winding_holds() {
        let w = Winding {
            r_ohm: Some(4.9),
            r_vpc: 7270.0 / 4096.0,
            r_taps_vpc: 7270.0 / 4096.0,
            r_from: Source::Stored,
            r_loop_vpc: 7270.0 / 4096.0,
            l_h: 0.62e-3,
            l_from: Source::Stored,
        };
        let why = Over::Declined("burst");
        let quiet = reuse_note(&w, why, "capture-agreement, split-halves", None);
        assert_eq!(
            quiet,
            [
                "[winding] the burst measured no winding R (capture-agreement, split-halves), so the \
                 run uses the R and L this servo already carries from an earlier identification: \
                 R 4.90 ohm, L 0.620 mH",
                "  they still hold: the winding belongs to the motor and does not change with \
                 the position table or the gear train",
            ]
        );
        let stale = reuse_note(&w, why, "split-halves", Some(3.5));
        assert_eq!(stale.len(), 3);
        assert_eq!(
            stale[2],
            "warning: the burst's rough R, 3.50 ohm, is 29% away from the stored 4.90 ohm, so \
             the stored winding may be stale; `osc ident run --stall-ladder` measures it again \
             from stop stalls, on a lower supply (USB) when this one leaves the stop ladder no \
             room"
        );
        let room = Over::NoLadderRoom {
            floor: 0.133,
            cap: 0.155,
        };
        assert!(reuse_note(&w, room, "split-halves", None)[0].starts_with(
            "[winding] the burst measured no winding R (split-halves) and the resistance \
                 stop ladder has no room on this supply (the servo reads current and terminal \
                 voltage from 13.3% duty, the current limit allows 15.5% at a stop), so the run uses"
        ));
    }

    /// A free fit that declines on the fake servo, its bursts blind to the
    /// driven terminal: the held route sizes its duties from the R the
    /// servo stores and says so, the duties inside the limit by that R;
    /// with nothing stored it says it is skipped and why.
    #[test]
    fn a_declined_free_fit_names_the_held_routes_r_or_why_it_is_skipped() {
        use osc_ident::exp::testkit::{FakeServo, board_d_scales, pump, rig};
        let sc = board_d_scales();
        let i_lim_a = 280.0 * sc.amps_per_count;
        let mut servo = FakeServo::new(7270.0 / 4096.0);
        servo.dynamic = true;
        let cfg = InductanceCfg {
            repeats: 1,
            i_max_a: 1.0,
            chans: Chans::Fixed(0),
            fit: FitCfg::default().with_limit(i_lim_a),
            ..InductanceCfg::default()
        };
        let mut e8 = Guarded::new(Inductance::new(cfg, &rig(), sc), rig());
        pump(&mut e8, &mut servo, 200_000);
        let fit = e8.into_inner().fit().expect("the bursts fit");
        assert!(!fit.promotable());

        let stored = 7270.0 / 4096.0;
        let (r, line) = held_r(Some(&fit), Some(stored), &sc).expect("the stored R");
        assert_eq!(r, stored);
        assert_eq!(
            line,
            format!(
                "[burst, held] the free fit declined on waveform (no capture from rest sampled \
                 the driven terminal): hold and stop duties from the R stored on the servo, \
                 {:.2} ohm",
                stored / sc.r_vpc(1.0)
            )
        );
        let lim = ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (209, 3849),
            raw: (209, 3849),
            r_q12: 7270,
            vbus: 3204,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: sc.amps_per_count,
            drive_polarity: true,
        };
        assert_eq!(lim.r_vpc(), Some(stored));
        let plan = DutyPlan::new(&lim, r, None);
        for duty in [plan.hold, plan.stop_cap] {
            assert!(lim.check_stall_at("held", q15_floor(duty), r).is_ok());
        }

        assert_eq!(
            held_r(Some(&fit), None, &sc),
            Err(
                "[burst, held] skipped: no winding R to size the hold and stop duties (the free \
                 fit declined on waveform (no capture from rest sampled the driven terminal), \
                 nothing is stored on the servo, and the fit has no waveform R)"
                    .into()
            )
        );
        assert!(
            held_r(None, None, &sc)
                .unwrap_err()
                .contains("(the bursts gave nothing to fit, nothing is stored")
        );
    }

    /// The rest of a run as it lands on disk: ladder rungs on a winding of
    /// `r` vcounts per ccount and the inertia steps.
    /// A run that read the thermometer's anchor: the hold's windows at the
    /// low stop under the 280 limit, the ambient the host gave. The fit
    /// writes the four CALIB fields behind the gains - r0_q12 the kernel's
    /// own R at the hold, t0_cc the ambient, k_r2t_q88 copper's handbook
    /// line through them, mu_q016 a 2 s settle at the limit - and the
    /// thermometer's current floor at 7/8 of the hold; the report says
    /// where each came from.
    /// The fit writes the gains alone; the thermometer is `osc ident anchor`'s.
    #[test]
    fn the_fit_writes_no_thermometer() {
        let (dir, out, r) = record_front("no-anchor");
        record_ladder_and_steps(&out, r);
        fit_dir(&ctx(dir.clone()), dir.clone()).unwrap();
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        assert!(p.thermometer.is_none());
        assert_eq!(p.gains.len(), 19);
        assert!(p.gains.iter().all(|g| g.name != "r0_q12"));
        let report = std::fs::read_to_string(dir.join("report.txt")).unwrap();
        assert!(
            report.contains("[thermometer] osc ident anchor writes"),
            "{report}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn record_ladder_and_steps(out: &csvio::OutDir, r: f64) {
        use osc_ident::exp::ladder::RungSummary;
        use osc_ident::fits::StepSeries;

        let rungs: Vec<RungSummary> = [1500.0, 3000.0, 4500.0, -1500.0, -3000.0, -4500.0]
            .into_iter()
            .map(|omega: f64| {
                let i = omega.signum() * (80.0 + 0.004 * omega.abs());
                RungSummary {
                    duty_q15: (omega / 5.0) as i16,
                    omega,
                    omega_r2: 0.99,
                    i,
                    v: r * i + 0.1472 * omega,
                    windows: 30,
                    used: true,
                    note: None,
                    omega_bemf: None,
                }
            })
            .collect();
        csvio::write_rungs(out, &rungs).unwrap();
        let series: Vec<(StepSeries, bool)> = [6000.0, -6000.0, 8000.0, -8000.0]
            .into_iter()
            .map(|duty: f64| {
                let (sgn, omega_ss, tau) = (duty.signum(), duty.abs() / 2.0, 0.04);
                let t: Vec<f64> = (0..60).map(|k| k as f64 * 0.005).collect();
                let pos = t
                    .iter()
                    .map(|t| 2029.0 + sgn * omega_ss * (t - tau * (1.0 - (-t / tau).exp())))
                    .collect();
                let i = t
                    .iter()
                    .map(|t| sgn * (80.0 + 200.0 * (-t / tau).exp()))
                    .collect();
                let series = StepSeries {
                    mask: vec![true; t.len()],
                    t,
                    pos,
                    i,
                    duty_q15: duty,
                };
                (series, false)
            })
            .collect();
        csvio::write_step_series(out, &series).unwrap();
    }

    macro_rules! mg90_2s {
        ($($n:literal),*) => {
            [$(include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../ident/testdata/burst/mg90-2s/burst-",
                $n,
                ".csv"
            ))),*]
        };
    }

    /// The bench MG90's bursts refitted from disk: the burst promotes its
    /// line, params.json says which number went where, and the gains carry
    /// it - r_q12 is the waveform's winding R between the taps, 4.344 ohm
    /// where V/I at the current limit the run's telemetry recorded reads
    /// 5.316, i_ki the slope with the bridge, i_kp the L - the ladder fits
    /// Ke against r_q12's R, and the inertia fit takes the slope.
    #[test]
    fn a_promoted_burst_writes_the_winding_r_to_r_q12_and_the_slope_to_i_ki() {
        use osc_ident::frame::TelemetrySnapshot;

        let (dir, out, r) = record_front("burst");
        std::fs::remove_file(dir.join("resistance.csv")).unwrap();
        let caps: Vec<Capture> = mg90_2s!(
            "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"
        )
        .iter()
        .map(|t| osc_ident::burst::from_csv(t).unwrap())
        .collect();
        csvio::write_bursts(&out, &caps).unwrap();
        {
            let mut log = csvio::SnapshotLog::create(&out, "inductance_snapshots.csv").unwrap();
            let s = TelemetrySnapshot {
                i_lim_counts: 280,
                ..TelemetrySnapshot::default()
            };
            log.push(4.5, &s).unwrap();
        }
        record_ladder_and_steps(&out, r);

        fit_dir(&ctx(dir.clone()), dir.clone()).unwrap();
        let p = ParamsFile::load(&dir.join("params.json")).unwrap();
        let sc = p.sense.unwrap().scales().unwrap();
        let e8 = p.inductance.as_ref().expect("the burst");
        assert!(e8.promoted, "{:?}", e8.blocking);
        let wave = e8.wave.clone().expect("the waveform fit");
        assert_eq!(wave.set_aside.len(), 1);
        assert_eq!(wave.set_aside[0].0, 11);
        assert!((wave.i_lim_a.unwrap() - 280.0 * sc.amps_per_count).abs() < 1e-12);
        let (v_over_i, slope) = (wave.v_over_i_lim_ohm.unwrap(), wave.slope_lim_ohm.unwrap());
        let plant = p.plant.expect("a plant");
        assert_eq!(plant.r_source, "burst, free");
        assert_eq!(
            (format!("{:.3}", wave.r_ohm), format!("{v_over_i:.3}")),
            ("4.344".into(), "5.316".into())
        );
        assert!((plant.r_ohm.unwrap() - wave.r_ohm).abs() < 1e-9);
        assert!((plant.r_vpc - sc.r_vpc(wave.r_ohm)).abs() < 1e-12);
        assert!((plant.r_loop_vpc - sc.r_vpc(slope)).abs() < 1e-12);
        assert_eq!(plant.l_henries, wave.l_h);
        assert!(
            plant.winding_use.starts_with(&format!(
                "r_q12 takes r_vpc, the waveform's winding R between the terminal taps \
                 ({:.3} ohm); every stall-safe duty takes V/I at the current limit \
                 ({v_over_i:.3} ohm); the current loop's i_ki takes r_loop_vpc, the V-I line's \
                 slope with the bridge ({slope:.3} ohm)",
                wave.r_ohm
            )),
            "{}",
            plant.winding_use
        );
        // Ke pairs with r_q12's R in the back-EMF estimate
        let l = p.ladder.expect("a ladder");
        let pts = csvio::read_rung_points(&dir).unwrap();
        assert_eq!(l.ke_vpc, fits::ke_fit(&pts, plant.r_vpc).unwrap().ke_vpc);
        // the inertia fit's back-EMF damping is Ke over the loop's R
        let priors = |r_vpc: f64| InertiaPriors {
            r_vpc,
            ke_vpc: l.ke_vpc,
            fc: (l.fc_fwd.unwrap() + l.fc_rev.unwrap()) / 2.0,
            fv: (l.fv_fwd.unwrap() + l.fv_rev.unwrap()) / 2.0,
            tick_hz: 20_100.0,
        };
        let tagged = csvio::read_step_series(&dir).unwrap();
        let series: Vec<fits::StepSeries> = tagged.iter().map(|(s, _)| s.clone()).collect();
        let b = |r_vpc: f64| fit_steps(&tagged, &[], &priors(r_vpc)).unwrap().b_best;
        let exp_rise = |r_vpc: f64| fits::b_exp_fit(&series, &priors(r_vpc), 12).unwrap().b;
        let inertia = p.inertia.expect("inertia");
        assert_eq!(inertia.b_best, b(plant.r_loop_vpc));
        assert_ne!(inertia.b_best, b(plant.r_vpc));
        assert_eq!(inertia.b_exp.unwrap(), exp_rise(plant.r_loop_vpc));
        assert_eq!(inertia.priors, "the ladder");
        let raw = |name: &str| p.gains.iter().find(|g| g.name == name).unwrap().raw;
        let w_ci = std::f64::consts::TAU * 1000.0;
        assert_eq!(raw("r_q12"), (sc.r_vpc(wave.r_ohm) * 4096.0).round() as u16);
        assert!(raw("r_q12") < (sc.r_vpc(v_over_i) * 4096.0).round() as u16);
        assert_eq!(
            raw("i_ki_q412"),
            (w_ci * plant.r_loop_vpc / 20_100.0 * 4096.0).round() as u16
        );
        assert_eq!(raw("i_kp_q88"), (w_ci * plant.l_cd * 256.0).round() as u16);
        let report = std::fs::read_to_string(dir.join("report.txt")).unwrap();
        assert!(
            report.contains(&format!("  waveform      R {:.3}", wave.r_ohm)),
            "{report}"
        );
        assert!(
            report.contains("-> r_q12, the back-EMF estimate's R and the ladder's Ke"),
            "{report}"
        );
        assert!(report.contains("-> every stall-safe duty"), "{report}");
        assert!(report.contains("-> the current loop's i_ki"), "{report}");
        // the burst's rungs, each read alone, in params.json and the report
        let rungs: Vec<(String, bool, String)> = e8
            .rungs
            .iter()
            .map(|g| {
                let pct = format!("{:.0}", g.duty * 100.0);
                (pct, g.forward, format!("{:.3}", g.v_over_i_ohm))
            })
            .collect();
        assert_eq!(
            rungs,
            [
                ("25".into(), true, "5.061".into()),
                ("25".into(), false, "5.291".into()),
                ("40".into(), true, "4.921".into()),
                ("40".into(), false, "4.906".into()),
            ]
        );
        assert!(
            report.contains("  rungs         duty dir  n  I asym"),
            "{report}"
        );
        assert!(report.contains(" 40% rev  4   0.588"), "{report}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A run whose burst declined and that took the winding the servo
    /// carries: the fit synthesizes from it as from any other winding,
    /// params.json and the report name it "stored on the servo", and the
    /// gains write R and L back as they were read.
    #[test]
    fn params_name_the_stored_winding_as_their_source() {
        let (dir, out, r) = record_front("stored");
        std::fs::remove_file(dir.join("resistance.csv")).unwrap();
        let path = dir.join("params.json");
        let mut p = ParamsFile::load(&path).unwrap();
        let sc = p.sense.unwrap().scales().unwrap();
        // what an earlier identification wrote: R 7270, L 0.6 mH at 1 kHz
        let w_ci = std::f64::consts::TAU * 1000.0;
        let l_cd = gains::l_cd_from_si(0.6e-3, 60, 15_000, 6_800, 3_300).unwrap();
        let i_kp = (w_ci * l_cd * 256.0).round() as u16;
        let i_ki = (w_ci * r / 20_100.0 * 4096.0).round() as u16;
        let w = sources::stored(7270, i_kp, i_ki, 20_100, 1000.0, &sc).expect("a stored winding");
        p.stored_winding = Some(StoredWindingJson::from(&w));
        p.save(&path).unwrap();

        record_ladder_and_steps(&out, r);

        fit_dir(&ctx(dir.clone()), dir.clone()).unwrap();
        let p = ParamsFile::load(&path).unwrap();
        let plant = p.plant.expect("a plant");
        assert_eq!(plant.r_source, "stored on the servo");
        assert_eq!(plant.l_source, "stored on the servo");
        assert_eq!((plant.r_vpc, plant.l_henries), (w.r_vpc, w.l_h));
        assert!(p.resistance.is_none() && p.inductance.is_none());
        let raw = |name: &str| p.gains.iter().find(|g| g.name == name).unwrap().raw;
        assert_eq!(raw("r_q12"), 7270);
        assert_eq!(raw("i_kp_q88"), i_kp);
        assert_eq!(raw("i_ki_q412"), i_ki);
        let report = std::fs::read_to_string(dir.join("report.txt")).unwrap();
        assert!(
            report.contains("not run: R stored on the servo"),
            "{report}"
        );
        assert!(report.contains("ohm)  stored on the servo"), "{report}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
