//! osc-ident: the identification, calibration and tuning tool. A servo's
//! constant set, and the safe path that puts a set on a servo. `rules`
//! touches no servo; `check`, `plan` and `table` only read;
//! `install`, `save` and `restore` write with torque off and never drive
//! the motor; `step` runs one measurement step into a run folder; `gather`
//! turns a run folder into inputs; `tune` settles the speed gain on the
//! servo with the closed-loop check; `run` does the steps, the gather and
//! the tune in one, then the cruises for the friction line and the final
//! set's check. The tool reports measurements, calculations and their
//! spread; judging them is the user's, so a set installs whatever it
//! moves, each change printed. A step stops only where going on would
//! drive the motor blind or into harm (a pot or tap at a converter rail,
//! a horn that moves when it should rest, a fault, a drive the firmware
//! refuses, a wrong-way or blocked move) or where it has no value to
//! give; the tune's check keeps its scores, which choose the gain.

mod files;
mod finish;
mod gather;
mod run;
mod steps;
mod table;
mod tune;

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use futures_lite::future::block_on;
use osc_client::descriptor::{Descriptor, Registry, Selection};
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::{BaudRate, Client, Error, Id, mgmt};
use osc_measure::build;
use osc_measure::install::{self, Step};
use osc_measure::run::{At, Host, Store, quiet, servo_name};
use osc_measure::set::{self, FromRules};
use serde_json::json;

use crate::files::Fs;
use crate::steps::Load;

/// The table layouts this tool knows, compiled from the checked-in export.
const DESCRIPTORS: &[&str] = &[include_str!("../../../descriptors/osc-servo/0.1.json")];

#[derive(Parser, Debug)]
#[command(
    name = "osc-ident",
    version,
    about = "The identification, calibration and tuning tool: a servo's constant set, and the safe path that puts it on the servo",
    long_about = "The identification, calibration and tuning tool: a servo's constant set, and the safe path that puts it on the servo. It reports measurements, calculations and their spread; judging them is the user's, so a set installs whatever it moves, each change printed. A step stops only where going on would drive the motor blind or into harm (a pot or tap at a converter rail, a horn that moves when it should rest, a fault, a drive the firmware refuses, a wrong-way or blocked move) or where it has no value to give; every other reading is reported, and the tune's check keeps its scores, which choose the gain."
)]
struct Cli {
    /// The servo's bus id.
    #[arg(short, long, global = true, default_value_t = 1)]
    id: u8,
    /// The bus rate: `auto` finds it, else 500000, 1000000, 2000000 or 3000000.
    #[arg(long, global = true, default_value = "auto")]
    baud: String,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Measured inputs, the servo profile and a policy to a set file. Touches no servo.
    Rules {
        /// The inputs folder: measured.json, nameplate.json (the profile, either
        /// version; a version 0 nameplate with its thermal.json), policy.json,
        /// board.json (the board's facts step 0 read off the servo) and, where
        /// measured, shelf.json, pos_lut.json, wide-codes.json.
        #[arg(long)]
        inputs: PathBuf,
        /// A policy file other than the folder's.
        #[arg(long)]
        policy: Option<PathBuf>,
        /// The set's pair name; `<profile>__<servo>` if omitted, the servo
        /// as the inputs folder's board.json names it.
        #[arg(long)]
        pair: Option<String>,
        #[arg(long, default_value = "set.json")]
        out: PathBuf,
    },
    /// The servo against a set: a line a difference, exit 1 on one. Writes nothing.
    Check {
        #[arg(long)]
        set: PathBuf,
    },
    /// The servo's position table: state, stops, nonzero calibration points, local gains. Writes nothing.
    Table {
        /// The table and its report as JSON.
        #[arg(long)]
        json: bool,
    },
    /// Make the servo equal the set, volatile, every write read back.
    Install {
        #[arg(long)]
        set: PathBuf,
        #[command(flatten)]
        w: WriteArgs,
    },
    /// Back up the stored state, install, check, SAVE, reboot, read back.
    Save {
        #[arg(long)]
        set: PathBuf,
        #[command(flatten)]
        w: WriteArgs,
        /// Where the run folder with the backup goes.
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Put a recorded state back, volatile: a save's backup/stored.json.
    Restore {
        #[arg(long = "from")]
        from: PathBuf,
    },
    /// The run this tool would do on the servo, with its numbers and the five budgets. Reads only.
    Plan {
        #[command(flatten)]
        i: StepArgs,
    },
    /// One measurement step into a run folder `<out>/<servo uid>/<UTC time>/`.
    Step {
        #[command(subcommand)]
        step: StepName,
    },
    /// A run folder's step results to an inputs folder for `rules` and `tune`. Touches no servo.
    Gather {
        /// The run folder: `<out>/<servo uid>/<UTC time>/` of a `step stops` run.
        #[arg(long)]
        run: PathBuf,
        /// The servo profile with its heat model, or a version 0
        /// nameplate.json with the thermal.json beside it.
        #[arg(long, visible_alias = "profile")]
        nameplate: PathBuf,
        /// The policy file: this tool has no built-in one.
        #[arg(long)]
        policy: PathBuf,
        /// The inputs folder to write, new.
        #[arg(long)]
        out: PathBuf,
    },
    /// Settle the speed gain: the rules at a gain, the set installed volatile, the closed-loop
    /// check; each try lower until the check accepts. Drives the motor under its own loops; never saves.
    Tune {
        /// The inputs folder, as `rules` reads it.
        #[arg(long)]
        inputs: PathBuf,
        /// The run folder to write, new.
        #[arg(long)]
        out: PathBuf,
        #[arg(long, default_value_t = osc_measure::tune::TRIES)]
        tries: u32,
    },
    /// The whole identification: the steps through the spins into a run folder, then, every step
    /// accepted, the gather into its `inputs/` and the tune into its `tune/`; on the set the tune
    /// accepts, the cruise step for the friction line, the gather again into `final/inputs/` and
    /// the final set installed volatile and checked in `final/`; a final check refused on rest
    /// error alone goes on down the tune's tries on that friction line, into `tune/tries/`.
    /// Never saves. With `--resume`, a stopped run goes on in its own folder from its first
    /// step not accepted, every check against what it began on passed before anything drives.
    Run {
        /// The servo profile with its heat model, or a version 0
        /// nameplate.json with the thermal.json beside it.
        #[arg(long, visible_alias = "profile")]
        nameplate: PathBuf,
        /// The policy file.
        #[arg(long)]
        policy: PathBuf,
        /// Where the run folder `<servo uid>/<UTC time>/` goes.
        #[arg(long, required_unless_present = "resume")]
        out: Option<PathBuf>,
        /// The tune's tries at most; 6 when not given.
        #[arg(long)]
        tries: Option<u32>,
        /// The run folder of a stopped run to go on with.
        #[arg(long)]
        resume: Option<PathBuf>,
    },
}

#[derive(Subcommand, Debug)]
enum StepName {
    /// Step 1, rest and board check: nothing drives.
    Rest {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Step 2, the first probe and polarity on a free shaft: the rest step
    /// first, then shunt bursts, then open-loop pulses. Drives the motor;
    /// metal gears only, for now.
    Probe {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Step 3, the stops and the seated R and L: the rest step and the
    /// probe first, then three seats at each stop, shunt bursts at
    /// each, and the shaft left near mid travel. Presses the end stops;
    /// metal gears only, for now.
    Stops {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Step 4, the ADC's wide codes and the position table: rest, the
    /// probe and the stops first, then open-loop slow passes at the
    /// approach duty stop to stop, each ending pressed on its stop under
    /// the stall permit, and fast passes at one fifth of the rated speed
    /// guard to guard; the motor's ripple read as the ruler after the way
    /// out, the shaft left at mid travel. Presses the end stops; metal
    /// gears only, for now.
    Table {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Step 5, the window term: rest, the probe, the stops and the table
    /// first, then 24 open-loop onsets from rest near mid travel (48 when
    /// the two directions differ), fitted after the way out. Drives the
    /// motor; metal gears only, for now.
    Window {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// Step 6, free spins and their coasts: rest, the probe, the stops,
    /// the table and the window term first, then a pilot spin and fourteen spins
    /// between the stops at 25, 50 and 75 % of the stated voltage, each
    /// from rest at a guard and cut short of the far one, for Ke, J and the
    /// open-loop friction line; the shaft left at mid travel. Drives the motor fast between
    /// its stops; metal gears only, for now.
    Spins {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// The closed-loop check of the installed set: the position
    /// loop turned on, small steps, then moves of 1, 10 and 33 % of the
    /// travel scored. Drives the motor under its own loops; changes no
    /// constant.
    Verify {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
    },
    /// The friction line from closed-loop Velocity cruises at mid travel on
    /// the installed set: four speeds from 0.075 of the move speed
    /// to all of it, two legs each way a speed, each leg sized and centered
    /// from the travel, the shaft left at the middle. Drives the motor under
    /// its own loops; changes no constant.
    Cruise {
        #[command(flatten)]
        i: StepArgs,
        #[arg(long, default_value = "osc-ident-runs")]
        out: PathBuf,
        /// A run folder whose stops step gives the travel; the stops the
        /// installed set holds if omitted.
        #[arg(long)]
        run: Option<PathBuf>,
    },
}

#[derive(clap::Args, Debug)]
struct StepArgs {
    /// The servo profile (JSON, `osc.servo-profile/1`), or a version 0
    /// nameplate.json: the maker's sheet (gear class, operating voltage,
    /// no-load and stall rows), the counted gear ratio, the travel, the
    /// count's sense, the motor's brushes and rotor_segments (commutator
    /// segments), counted on a teardown or stated by the maker, and the
    /// winding's heat model (a version 0 nameplate's thermal.json beside
    /// it).
    #[arg(long, visible_alias = "profile")]
    nameplate: Option<PathBuf>,
    /// The tuning policy (JSON, `osc.tuning-policy/1`, or a version 0
    /// policy.json): the run voltage and the derates the steps read. A
    /// version 0 nameplate carries them instead.
    #[arg(long)]
    policy: Option<PathBuf>,
}

impl StepArgs {
    fn load(&self) -> Result<steps::StepInputs> {
        steps::StepInputs::load(self.nameplate.as_deref(), self.policy.as_deref())
    }
}

/// A step command's or a run's outcome with the lines of the reboot that
/// clears the unsaved bit after it ([`steps::clear_unsaved`]): appended to
/// its lines, or printed ahead of its refusal.
fn ended<T>(
    ran: Result<(Vec<String>, bool, T)>,
    cleared: Result<Vec<String>>,
) -> Result<(Vec<String>, bool, T)> {
    match (ran, cleared) {
        (Ok((mut lines, accepted, t)), Ok(more)) => {
            lines.extend(more);
            Ok((lines, accepted, t))
        }
        (Ok(_), Err(e)) => Err(e.context("the command ended, but the reboot to clear unsaved")),
        (Err(e), Ok(more)) => {
            for l in more {
                println!("{l}");
            }
            Err(e)
        }
        (Err(e), Err(r)) => {
            eprintln!("the reboot to clear unsaved: {r:#}");
            Err(e)
        }
    }
}

/// Set by Ctrl-C: a step reads it between rungs, so a running burst is
/// never cut short.
static STOP: AtomicBool = AtomicBool::new(false);

/// The name a run's records carry.
const TOOL: &str = concat!("osc-ident ", env!("CARGO_PKG_VERSION"));

fn unix_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// The run's host: the file system, the system clock, `stop`; the lines
/// are printed at the end, so the feed is quiet.
fn host<'a>(files: &'a mut dyn Store, stop: &'a AtomicBool) -> Host<'a> {
    Host {
        files,
        clock: &unix_s,
        stop,
        feed: &quiet,
        tool: TOOL,
    }
}

#[derive(clap::Args, Debug)]
struct WriteArgs {
    /// The table file of a LIVE set's points; `<pair>.pos_lut.json` beside the set if omitted.
    #[arg(long)]
    table: Option<PathBuf>,
}

fn rules(inputs: &Path, policy: Option<&Path>, pair: Option<&str>, out: &Path) -> Result<()> {
    let r = files::rules_inputs(inputs, policy)?;
    for n in &r.notes {
        eprintln!("{n}");
    }
    let pair = pair.map_or_else(
        || format!("{}__{}", r.inputs.nameplate.name, r.servo),
        str::to_string,
    );
    let (file, notes) = set::from_rules(
        &build(&r.inputs),
        &FromRules {
            pair: &pair,
            nameplate: &r.inputs.nameplate,
            counts_per_deg: r.inputs.measured.table.as_ref().map(|t| t.counts_per_deg),
            table: r.table.as_ref(),
            slots: r.slots.as_deref(),
            files: &r.files,
        },
        &descriptor_builtin()?,
    )?;
    if out.exists() {
        bail!("{} exists: a set is written to a new file", out.display());
    }
    std::fs::write(out, file.render()).with_context(|| format!("writing {}", out.display()))?;
    for t in file.terms() {
        println!("{t}");
    }
    for n in &notes {
        println!("{n}");
    }
    let (state, sha) = file.table();
    println!(
        "wrote {}: {} fields, position table {state} ({sha}), set {}",
        out.display(),
        file.values().len(),
        &file.hash()[..12]
    );
    Ok(())
}

fn descriptor_builtin() -> Result<Descriptor> {
    Ok(Descriptor::parse(DESCRIPTORS[0])?)
}

fn parse_rate(s: &str) -> Result<BaudRate> {
    let bps: u32 = s.parse().context("baud is an integer rate or `auto`")?;
    [
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ]
    .into_iter()
    .find(|r| r.as_hz() == bps)
    .ok_or_else(|| anyhow::anyhow!("unsupported baud {bps}"))
}

async fn connect(baud: &str, id: Id) -> Result<(Client<NusbPipe>, Descriptor)> {
    let mut c = Client::connect(NusbPipe::open()?).await?;
    c.set_resend(true);
    match baud {
        "auto" => match mgmt::find_bus_baud(&mut c).await? {
            Some(rate) => eprintln!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo answers at any rate"),
        },
        s => c.host_baud(parse_rate(s)?).await?,
    }
    let ping = match c.ping(id).await {
        Err(Error::Timeout { .. }) => c.ping(id).await,
        p => p,
    }
    .with_context(|| format!("servo {} does not answer", id.as_byte()))?;
    let mut reg = Registry::new();
    for s in DESCRIPTORS {
        reg.push(Descriptor::parse(s)?);
    }
    let d = match reg.select(ping.model, ping.fw) {
        Selection::Exact(d) | Selection::OlderMinor(d) => d.clone(),
        _ => bail!(
            "no descriptor of this tool is the servo's (model {:#06x}, fw {:#06x})",
            ping.model,
            ping.fw
        ),
    };
    Ok((c, d))
}

fn print_steps(steps: &[Step]) {
    for s in steps {
        println!("{s}");
    }
}

/// The servo against the set; whether it reads clean.
async fn check<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set_path: &Path,
) -> Result<bool> {
    let set = files::set(set_path)?;
    let checked = install::check(c, id, d, &set).await?;
    for line in &checked.differences {
        println!("{line}");
    }
    if !checked.unowned.is_empty() {
        println!("not in the set: {}", checked.unowned.join(", "));
    }
    println!(
        "{} differences; settings {}; unsaved {}; set {}; servo {}",
        checked.differences.len(),
        checked.settings,
        if checked.unsaved { "yes" } else { "no" },
        &set.hash()[..12.min(set.hash().len())],
        &checked.servo[..12.min(checked.servo.len())]
    );
    if checked.refused != "none" {
        println!(
            "refused {}: {}",
            checked.refused,
            osc_client::store_state::REFUSED
        );
    }
    Ok(checked.clean())
}

async fn write_verb<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor, cmd: &Cmd) -> Result<()> {
    let mut steps = Vec::new();
    let done = match cmd {
        Cmd::Install { set, w } => {
            let file = files::set(set)?;
            let table = files::table_for(set, &file, w.table.as_deref())?;
            install::install(
                c,
                id,
                d,
                &file,
                table.as_ref().map(|t| &t.points),
                &mut steps,
            )
            .await
        }
        Cmd::Save { set, w, out } => {
            let file = files::set(set)?;
            let table = files::table_for(set, &file, w.table.as_deref())?;
            let root = files::run_folder(out, &servo_name(c, id).await?)?;
            let mut sink = At {
                files: &mut Fs,
                root,
            };
            install::save(
                c,
                id,
                d,
                &file,
                table.as_ref().map(|t| &t.points),
                &mut sink,
                &mut steps,
            )
            .await
        }
        Cmd::Restore { from } => {
            let bytes = files::read(from)?;
            let text = String::from_utf8(bytes.clone()).context("a record is UTF-8")?;
            let origin = json!({"from": "a record restored", "file": from.display().to_string(), "sha1": set::file_sha1(&bytes)});
            install::restore(c, id, d, &text, origin, &mut steps).await
        }
        Cmd::Rules { .. }
        | Cmd::Check { .. }
        | Cmd::Table { .. }
        | Cmd::Plan { .. }
        | Cmd::Step { .. }
        | Cmd::Gather { .. }
        | Cmd::Tune { .. }
        | Cmd::Run { .. } => {
            bail!("not a write verb")
        }
    };
    print_steps(&steps);
    done?;
    let wrote = steps.iter().any(|s| {
        matches!(
            s,
            Step::Field { .. } | Step::Slots { .. } | Step::Table { .. }
        )
    });
    match cmd {
        Cmd::Save { .. } => {
            let s = osc_client::store_state::read(c, id, d).await?;
            println!(
                "saved: after a reboot the servo reads as the set, settings {}",
                s.settings()
            )
        }
        _ if wrote => println!("volatile, not saved: a reboot returns the saved state"),
        _ => println!("the servo holds the set: nothing written"),
    }
    Ok(())
}

fn run(cli: &Cli) -> Result<bool> {
    let id = Id::new(cli.id);
    match &cli.cmd {
        Cmd::Rules {
            inputs,
            policy,
            pair,
            out,
        } => {
            rules(inputs, policy.as_deref(), pair.as_deref(), out)?;
            Ok(true)
        }
        Cmd::Check { set } => block_on(async {
            let (mut c, d) = connect(&cli.baud, id).await?;
            check(&mut c, id, &d, set).await
        }),
        Cmd::Table { json } => block_on(async {
            let (mut c, d) = connect(&cli.baud, id).await?;
            let t = table::read(&mut c, id, &d).await?;
            if let Some(w) = t.warning() {
                eprintln!("{w}");
            }
            if *json {
                println!("{}", t.json()?);
            } else {
                print!("{}", t.text());
            }
            Ok(true)
        }),
        Cmd::Plan { i } => {
            let inputs = i.load()?;
            block_on(async {
                let (mut c, d) = connect(&cli.baud, id).await?;
                let (lines, starts) = steps::plan(&mut c, id, &d, &inputs).await?;
                for l in &lines {
                    println!("{l}");
                }
                Ok(starts)
            })
        }
        Cmd::Step { step } => {
            let (StepName::Rest { i, out }
            | StepName::Probe { i, out }
            | StepName::Stops { i, out }
            | StepName::Table { i, out }
            | StepName::Window { i, out }
            | StepName::Spins { i, out }
            | StepName::Verify { i, out }
            | StepName::Cruise { i, out, .. }) = step;
            let inputs = i.load()?;
            ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst))
                .context("installing the Ctrl-C handler")?;
            block_on(async {
                let (mut c, d) = connect(&cli.baud, id).await?;
                let servo = servo_name(&mut c, id).await?;
                let before = osc_measure::state::read(&mut c, id, &d).await?;
                let ran = async {
                    Ok::<_, anyhow::Error>(match step {
                        StepName::Rest { .. } => {
                            steps::step_rest(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Probe { .. } => {
                            steps::step_probe(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Stops { .. } => {
                            steps::step_stops(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Table { .. } => {
                            steps::step_table(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Window { .. } => {
                            steps::step_window(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Spins { .. } => {
                            steps::step_spins(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Verify { .. } => {
                            steps::step_verify(&mut c, id, &d, &inputs, out, &servo, &STOP).await?
                        }
                        StepName::Cruise { run, .. } => {
                            steps::step_cruise(
                                &mut c,
                                id,
                                &d,
                                &inputs,
                                out,
                                &servo,
                                &STOP,
                                run.as_deref(),
                            )
                            .await?
                        }
                    })
                }
                .await;
                let (lines, accepted, _) =
                    ended(ran, steps::clear_unsaved(&mut c, id, &d, &before).await)?;
                for l in &lines {
                    println!("{l}");
                }
                Ok(accepted)
            })
        }
        Cmd::Gather {
            run,
            nameplate,
            policy,
            out,
        } => {
            for l in gather::gather(run, nameplate, policy, out)? {
                println!("{l}");
            }
            Ok(true)
        }
        Cmd::Tune { inputs, out, tries } => {
            ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst))
                .context("installing the Ctrl-C handler")?;
            block_on(async {
                let (mut c, d) = connect(&cli.baud, id).await?;
                let (lines, accepted) =
                    tune::tune(&mut c, id, &d, inputs, out, *tries, &STOP).await?;
                for l in &lines {
                    println!("{l}");
                }
                Ok(accepted)
            })
        }
        Cmd::Run {
            nameplate,
            policy,
            resume: Some(root),
            out,
            tries,
        } => {
            let tries = run::resume_args(root, out.as_deref(), *tries)?;
            let a = run::RunArgs {
                nameplate,
                policy,
                out: root,
                tries,
            };
            run::admit(&a)?;
            ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst))
                .context("installing the Ctrl-C handler")?;
            block_on(async {
                let (mut c, d) = connect(&cli.baud, id).await?;
                let servo = servo_name(&mut c, id).await?;
                let before = osc_measure::state::read(&mut c, id, &d).await?;
                let ran = run::resume(&mut c, id, &d, &a, root, &servo, &STOP)
                    .await
                    .map(|(lines, accepted)| (lines, accepted, ()));
                let (lines, accepted, ()) =
                    ended(ran, steps::clear_unsaved(&mut c, id, &d, &before).await)?;
                for l in &lines {
                    println!("{l}");
                }
                Ok(accepted)
            })
        }
        Cmd::Run {
            nameplate,
            policy,
            out,
            tries,
            resume: None,
        } => {
            let out = out
                .as_deref()
                .context("--out names the run folder's place")?;
            let a = run::RunArgs {
                nameplate,
                policy,
                out,
                tries: tries.unwrap_or(osc_measure::tune::TRIES),
            };
            run::admit(&a)?;
            ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst))
                .context("installing the Ctrl-C handler")?;
            block_on(async {
                let (mut c, d) = connect(&cli.baud, id).await?;
                let servo = servo_name(&mut c, id).await?;
                let before = osc_measure::state::read(&mut c, id, &d).await?;
                let ran = run::run(&mut c, id, &d, &a, &servo, &STOP)
                    .await
                    .map(|(lines, accepted)| (lines, accepted, ()));
                let (lines, accepted, ()) =
                    ended(ran, steps::clear_unsaved(&mut c, id, &d, &before).await)?;
                for l in &lines {
                    println!("{l}");
                }
                Ok(accepted)
            })
        }
        cmd => block_on(async {
            let (mut c, d) = connect(&cli.baud, id).await?;
            write_verb(&mut c, id, &d, cmd).await?;
            Ok(true)
        }),
    }
}

fn main() -> ExitCode {
    match run(&Cli::parse()) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("refused: {e:#}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;
    use osc_client::fake::FakePipe;
    use osc_measure::set::{SetFile, TableFile};

    fn testdata(rel: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../measure/testdata/mg90-d")
            .join(rel)
    }

    fn scratch(name: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("osc-ident-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("scratch");
        p
    }

    #[test]
    fn the_arguments_parse() {
        Cli::command().debug_assert();
        let cli = Cli::try_parse_from(["osc-ident", "step", "table", "--nameplate", "np.json"])
            .expect("step table parses");
        assert!(matches!(
            cli.cmd,
            Cmd::Step {
                step: StepName::Table { .. }
            }
        ));
        let cli = Cli::try_parse_from(["osc-ident", "table", "--json"]).expect("table parses");
        assert!(matches!(cli.cmd, Cmd::Table { json: true }));
    }

    /// `--profile` names the same file `--nameplate` does, on every command
    /// that reads one.
    #[test]
    fn profile_is_another_name_for_nameplate() {
        let cli = Cli::try_parse_from([
            "osc-ident",
            "step",
            "probe",
            "--profile",
            "p.json",
            "--policy",
            "q.json",
        ])
        .expect("parses");
        let Cmd::Step {
            step: StepName::Probe { i, .. },
        } = cli.cmd
        else {
            panic!("step probe")
        };
        assert_eq!(i.nameplate.as_deref(), Some(Path::new("p.json")));
        assert_eq!(i.policy.as_deref(), Some(Path::new("q.json")));
        for verb in [
            vec!["gather", "--run", "r", "--policy", "q", "--out", "o"],
            vec!["run", "--policy", "q", "--out", "o"],
        ] {
            let mut args = vec!["osc-ident"];
            args.extend(verb);
            args.extend(["--profile", "p.json"]);
            match Cli::try_parse_from(&args).expect("parses").cmd {
                Cmd::Gather { nameplate, .. } | Cmd::Run { nameplate, .. } => {
                    assert_eq!(nameplate, PathBuf::from("p.json"))
                }
                c => panic!("{c:?}"),
            }
        }
    }

    /// A version 1 profile resolves only with its policy, and with the
    /// record's own it gives the steps the version 0 nameplate's values.
    #[test]
    fn a_version_1_profile_loads_with_its_policy() {
        let profile = testdata("profile.json");
        let e = steps::StepInputs::load(Some(&profile), None)
            .err()
            .expect("refused");
        assert!(format!("{e:#}").contains("--policy"), "{e:#}");
        let v1 = steps::StepInputs::load(Some(&profile), Some(&testdata("policy-of-record.json")))
            .expect("loads");
        let v0 = inputs();
        let (a, b) = (v1.nameplate.expect("v1"), v0.nameplate.expect("v0"));
        assert_eq!(a.name, "vorpal-mg90");
        assert_eq!(
            (a.run_v, a.derate_speed, a.derate_other, a.travel_deg),
            (b.run_v, b.derate_speed, b.derate_other, b.travel_deg)
        );
        let volts = |s: Option<osc_measure::nameplate::Stall>| s.map(|s| (s.at_v, s.current_a));
        assert_eq!(
            (a.speed_row(), volts(a.stall_row())),
            (b.speed_row(), volts(b.stall_row()))
        );
        // the profile states the stall torque a yield torque is taken over
        assert_eq!(a.stall_row().and_then(|s| s.torque_kg_cm), Some(1.8));
        let heat = v1
            .thermal
            .expect("the profile states the record's heat model");
        let value = |q: Option<osc_measure::Quantity>| q.map(|q| q.value);
        assert_eq!(
            [
                value(heat.fast_tau_s),
                value(heat.fast_gain_c_per_w),
                value(heat.slow_tau_s),
                value(heat.slow_gain_c_per_w)
            ],
            [
                Some(2.3752191114749404),
                Some(20.44571009950305),
                Some(128.92),
                Some(76.7616)
            ]
        );
    }

    /// `rules` on the mg90-d inputs writes a set that `check` reads as the
    /// servo's, on a simulated servo that holds the set of record.
    #[test]
    fn rules_on_the_record_inputs_checks_clean_against_the_set_of_record() {
        let dir = scratch("rules");
        let out = dir.join("set.json");
        rules(&testdata(""), None, None, &out).expect("rules");
        assert!(
            rules(&testdata(""), None, None, &out).is_err(),
            "never writes over a set"
        );

        let mut pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = block_on(Client::connect(pipe)).expect("connect");
        let (id, d) = (Id::new(1), descriptor_builtin().expect("descriptor"));
        let record =
            SetFile::parse(&std::fs::read_to_string(testdata("constants.json")).expect("read"))
                .expect("the set of record");
        let table =
            TableFile::parse(&std::fs::read_to_string(testdata("pos_lut.json")).expect("read"))
                .expect("table");
        block_on(install::install(
            &mut c,
            id,
            &d,
            &record,
            Some(&table.points),
            &mut Vec::new(),
        ))
        .expect("the servo takes the set of record");
        block_on(mgmt::save(&mut c, id)).expect("save");
        block_on(mgmt::reboot(&mut c, id)).expect("reboot");

        assert!(
            block_on(check(&mut c, id, &d, &out)).expect("check"),
            "clean"
        );
        assert!(block_on(check(&mut c, id, &d, &testdata("constants.json"))).expect("check"));
        let ours = files::set(&out).expect("our set");
        assert_eq!(ours.table(), record.table());
        assert_eq!(ours.values().len() + 1, record.values().len());

        // The table goes with the set: our rules' output names its points.
        let mut steps = Vec::new();
        let named = files::table_for(&out, &ours, Some(&testdata("pos_lut.json"))).expect("table");
        block_on(install::install(
            &mut c,
            id,
            &d,
            &ours,
            named.as_ref().map(|t| &t.points),
            &mut steps,
        ))
        .expect("installs");
        assert_eq!(steps, [], "the servo already holds it");
        let _ = std::fs::remove_dir_all(dir);
    }

    fn fake(rate: BaudRate) -> (Client<FakePipe>, Descriptor) {
        let mut pipe = FakePipe::new(rate, &[1]);
        pipe.seed_calibrated(0);
        let c = block_on(Client::connect(pipe)).expect("connect");
        (c, descriptor_builtin().expect("descriptor"))
    }

    /// A run folder is named by the chip id the discovery reads; with no
    /// servo of the id in its roster, no folder is named at all.
    #[test]
    fn a_run_folder_takes_the_chip_id_or_is_refused() {
        let (mut c, _) = fake(BaudRate::B1000000);
        let name = block_on(servo_name(&mut c, Id::new(1))).expect("the chip id");
        assert_eq!(name.len(), 32, "{name}");
        assert!(name.chars().all(|ch| ch.is_ascii_hexdigit()), "{name}");
        let e = block_on(servo_name(&mut c, Id::new(2))).expect_err("no servo 2");
        assert!(
            e.to_string()
                .starts_with("the discovery found no servo of id 2 (ids: 1)"),
            "{e}"
        );
    }

    fn inputs() -> steps::StepInputs {
        steps::StepInputs::load(Some(&testdata("nameplate.json")), None).expect("inputs")
    }

    /// `plan` names steps 0 and 1 with the servo's numbers and the five
    /// budgets, and none of the steps the run no longer has.
    #[test]
    fn plan_prints_steps_0_and_1_and_the_five_budgets() {
        let (mut c, d) = fake(BaudRate::B3000000);
        let (lines, starts) =
            block_on(steps::plan(&mut c, Id::new(1), &d, &inputs())).expect("plan");
        let text = lines.join("\n");
        println!("{text}");
        assert!(starts);
        assert!(
            text.contains("production image (capability_flags"),
            "{text}"
        );
        assert!(
            text.contains("step 0: connect") && text.contains("passes"),
            "{text}"
        );
        assert!(text.contains("step 1: rest and board check"), "{text}");
        for b in ["current:", "duty:", "time:", "travel:", "temperature:"] {
            assert_eq!(
                lines
                    .iter()
                    .filter(|l| l.trim_start().starts_with(b))
                    .count(),
                1,
                "{b}: {text}"
            );
        }
        assert!(text.contains("mg90-d") && text.contains("4.8 V"), "{text}");
        for gone in ["step 2", "heat slope", "spinning"] {
            assert!(!text.contains(gone), "{gone}: {text}");
        }

        let (mut c, d) = fake(BaudRate::B2000000);
        let (lines, starts) =
            block_on(steps::plan(&mut c, Id::new(1), &d, &inputs())).expect("plan");
        assert!(!starts);
        assert!(
            lines
                .iter()
                .any(|l| l.starts_with("step 0") && l.contains("REFUSED")),
            "{lines:?}"
        );
    }

    /// `step rest` keeps its run folder: the manifest, the captures and the
    /// result, the servo torque off after it whatever the verdict.
    #[test]
    fn step_rest_keeps_a_run_folder() {
        let (mut c, d) = fake(BaudRate::B3000000);
        let out = scratch("step-rest");
        let stop = AtomicBool::new(false);
        let (lines, _, root) = block_on(steps::step_rest(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        assert!(root.starts_with(out.join("uid")), "{}", root.display());
        for f in [
            "run.json",
            "results/rest.json",
            "captures/rest/01-rest.csv.gz",
            "captures/rest/02-raw.meta.json",
        ] {
            assert!(root.join(f).exists(), "{f}: {lines:?}");
        }
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["image"], "production");
        assert!(run["steps"]["rest"]["state"].is_string());
        assert!(
            lines
                .iter()
                .any(|l| l.contains("torque_enable 0, read back 0")),
            "{lines:?}"
        );
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        let kept: osc_measure::board::Record =
            serde_json::from_slice(&std::fs::read(root.join(files::BOARD)).expect("read"))
                .expect("the board record");
        let rec: osc_measure::board::Record =
            serde_json::from_slice(&std::fs::read(testdata("board.json")).expect("read"))
                .expect("the record's board");
        assert_eq!(kept.servo, "uid");
        assert_eq!(
            kept.registers, rec.registers,
            "step 0 kept what the servo publishes"
        );
        let _ = std::fs::remove_dir_all(out);
    }

    /// An image older than the board facts block (CALIB ntc_site reads 0)
    /// is refused at step 0: `plan` says so and would not start, a step
    /// makes no run folder and the servo stays torque off.
    #[test]
    fn an_image_without_board_facts_is_refused_at_step_0() {
        let older = || {
            let (mut c, d) = fake(BaudRate::B3000000);
            c.pipe_mut()
                .sim_mut()
                .servo_table_mut(0, |t| t.calib.ntc_site.ntc_site = 0);
            (c, d)
        };
        let (mut c, d) = older();
        let (lines, starts) =
            block_on(steps::plan(&mut c, Id::new(1), &d, &inputs())).expect("plan");
        assert!(!starts);
        assert!(
            lines.iter().any(|l| l.starts_with("step 0")
                && l.contains("REFUSED")
                && l.contains(osc_measure::board::OLDER_IMAGE)),
            "{lines:?}"
        );
        let (mut c, d) = older();
        let out = scratch("step-older");
        let e = block_on(steps::step_rest(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &AtomicBool::new(false),
        ))
        .expect_err("refused");
        let e = format!("{e:#}");
        assert!(
            e.contains("step 0") && e.contains(osc_measure::board::OLDER_IMAGE),
            "{e}"
        );
        assert!(!out.join("uid").exists(), "no run folder");
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        let _ = std::fs::remove_dir_all(out);
    }

    /// `step probe` runs the rest step first into the same folder, then the
    /// probe; the simulated servo runs no shunt burst, so the probe aborts
    /// at its first one, through the way out, and both steps are recorded.
    /// A servo at rest on the bench, a count or so of noise on each
    /// channel: what the rest step accepts.
    fn resting() -> Vec<osc_client::fake::TelSample> {
        use osc_client::fake::{TelSample, TelemetryTick};
        let noise = |i: u32, k: u32| {
            let mut h = i.wrapping_mul(0x9E37_79B9) ^ k.wrapping_mul(0x85EB_CA6B);
            h ^= h >> 15;
            h = h.wrapping_mul(0x2C1B_3C6D);
            h ^= h >> 12;
            (h % 3) as i32 - 1
        };
        (0..4096u32)
            .map(|i| {
                let at = |base: i32, k: u32| (base + noise(i, k)) as u16;
                TelSample {
                    tick: TelemetryTick {
                        _scan: [0; 9],
                        pos: at(2041, 1),
                        vbus_raw: at(1241, 3),
                        ntc_raw: at(2010, 4),
                        current_raw: at(1206, 5),
                        vmotor_a: at(773, 6),
                        vmotor_b: at(772, 7),
                        duty_ticks: 0,
                        current_trough: at(1206, 8),
                        i_tick_counts: 0,
                        vdiff_counts: 0,
                        vmotor_a_trough: at(773, 9),
                        vmotor_b_trough: at(772, 10),
                        tick_flags: 0,
                        sample_tick: i,
                    },
                    vbus_counts: 2000,
                }
            })
            .collect()
    }

    #[test]
    fn step_probe_runs_rest_first_into_the_same_folder() {
        let (mut c, d) = fake(BaudRate::B3000000);
        c.pipe_mut().set_track(0, resting());
        let out = scratch("step-probe");
        let stop = AtomicBool::new(false);
        let before = block_on(osc_measure::state::read(&mut c, Id::new(1), &d)).expect("state");
        assert!(!before.unsaved, "the seed is saved");
        let (lines, accepted, root) = block_on(steps::step_probe(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        for f in ["run.json", "results/rest.json", "results/probe.json"] {
            assert!(root.join(f).exists(), "{f}: {text}");
        }
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["steps"]["rest"]["state"], "accepted", "{run}");
        assert_eq!(run["steps"]["probe"]["state"], "aborted", "{run}");
        assert!(text.contains("step probe aborted"), "{text}");
        assert!(text.contains("torque_enable 0, read back 0"), "{text}");
        assert!(text.contains("end position"), "{text}");
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        // the probe's stored writes went back, 0 differences, but set the
        // unsaved bit: the command's end reboots to clear it
        let now = block_on(osc_measure::state::read(&mut c, Id::new(1), &d)).expect("state");
        assert!(now.unsaved && now.hash(&d) == before.hash(&d), "{text}");
        let more = block_on(steps::clear_unsaved(&mut c, Id::new(1), &d, &before)).expect("clears");
        assert!(
            more[0].starts_with("rebooted to clear unsaved: ")
                && more[0].ends_with("now unsaved no, 0 differences"),
            "{more:?}"
        );
        let now = block_on(osc_measure::state::read(&mut c, Id::new(1), &d)).expect("state");
        assert!(!now.unsaved && now.hash(&d) == before.hash(&d));
        assert_eq!(
            block_on(steps::clear_unsaved(&mut c, Id::new(1), &d, &before)).expect("reads"),
            Vec::<String>::new(),
            "a saved servo is not rebooted"
        );
        let _ = std::fs::remove_dir_all(out);
    }

    /// A command that leaves a stored field other than it found it, a set
    /// installed volatile, is never rebooted, and neither is one that began
    /// on a servo already unsaved: either holds a value its flash does not.
    #[test]
    fn a_volatile_value_is_never_rebooted_away() {
        let (mut c, d) = fake(BaudRate::B3000000);
        let id = Id::new(1);
        let f = d.field("current_limit_counts").expect("field").addr;
        let read = |c: &mut Client<FakePipe>| {
            block_on(osc_measure::state::read(c, id, &d)).expect("state")
        };
        let before = read(&mut c);
        let was = block_on(c.read(id, f, 2)).expect("read");
        let other = (u16::from_le_bytes([was[0], was[1]]) - 1).to_le_bytes();
        block_on(c.write(id, f, &other)).expect("write");
        let lines = block_on(steps::clear_unsaved(&mut c, id, &d, &before)).expect("reads");
        assert!(lines.is_empty(), "{lines:?}");
        let installed = read(&mut c);
        assert!(installed.unsaved);
        assert_eq!(block_on(c.read(id, f, 2)).expect("read"), other);
        // began unsaved, ends back on the flash's value: still left
        block_on(c.write(id, f, &was)).expect("write");
        let lines = block_on(steps::clear_unsaved(&mut c, id, &d, &installed)).expect("reads");
        assert!(lines.is_empty(), "{lines:?}");
        assert!(read(&mut c).unsaved);
    }

    /// `step stops` runs rest and the probe first into the same folder;
    /// the simulated servo runs no shunt burst, so the probe aborts and the
    /// stops step does not start, and it says so.
    #[test]
    fn step_stops_starts_only_on_an_accepted_probe() {
        let (mut c, d) = fake(BaudRate::B3000000);
        c.pipe_mut().set_track(0, resting());
        let out = scratch("step-stops");
        let stop = AtomicBool::new(false);
        let (lines, accepted, root) = block_on(steps::step_stops(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        assert!(root.join("results/probe.json").exists(), "{text}");
        assert!(!root.join("results/stops.json").exists(), "{text}");
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["steps"]["rest"]["state"], "accepted", "{run}");
        assert_eq!(run["steps"]["probe"]["state"], "aborted", "{run}");
        assert!(run["steps"]["stops"].is_null(), "{run}");
        assert!(
            text.contains("step stops not run: it needs an accepted probe"),
            "{text}"
        );
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        let _ = std::fs::remove_dir_all(out);

        let none = steps::StepInputs::load(None, None).expect("inputs");
        let dir = scratch("step-stops-none");
        let e = block_on(steps::step_stops(
            &mut c,
            Id::new(1),
            &d,
            &none,
            &dir,
            "uid",
            &stop,
        ))
        .expect_err("refused");
        assert!(format!("{e:#}").contains("--nameplate"), "{e:#}");
        assert!(!dir.join("uid").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// `step window` chains rest, the probe and the stops into one folder;
    /// the simulated servo runs no shunt burst, so the probe aborts and
    /// neither the stops nor the window step starts. Without a nameplate it
    /// is refused before any folder.
    #[test]
    fn step_window_starts_only_on_an_accepted_chain() {
        let (mut c, d) = fake(BaudRate::B3000000);
        c.pipe_mut().set_track(0, resting());
        let out = scratch("step-window");
        let stop = AtomicBool::new(false);
        let (lines, accepted, root) = block_on(steps::step_window(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        assert!(!root.join("results/window.json").exists(), "{text}");
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["steps"]["probe"]["state"], "aborted", "{run}");
        assert!(run["steps"]["window"].is_null(), "{run}");
        assert!(
            text.contains("step stops not run: it needs an accepted probe"),
            "{text}"
        );
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        let _ = std::fs::remove_dir_all(out);

        let none = steps::StepInputs::load(None, None).expect("inputs");
        let dir = scratch("step-window-none");
        let e = block_on(steps::step_window(
            &mut c,
            Id::new(1),
            &d,
            &none,
            &dir,
            "uid",
            &stop,
        ))
        .expect_err("refused");
        let e = format!("{e:#}");
        assert!(
            e.contains("step window") && e.contains("--nameplate"),
            "{e}"
        );
        assert!(!dir.join("uid").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// `step spins` chains rest, the probe, the stops and the window step
    /// into one folder; the simulated servo runs no shunt burst, so the
    /// probe aborts and nothing after it starts; with no nameplate, refused
    /// before the bus is touched.
    #[test]
    fn step_spins_starts_only_on_an_accepted_chain() {
        let (mut c, d) = fake(BaudRate::B3000000);
        c.pipe_mut().set_track(0, resting());
        let out = scratch("step-spins");
        let stop = AtomicBool::new(false);
        let (lines, accepted, root) = block_on(steps::step_spins(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        assert!(root.join("results/probe.json").exists(), "{text}");
        assert!(!root.join("results/spins.json").exists(), "{text}");
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["steps"]["probe"]["state"], "aborted", "{run}");
        for later in ["stops", "window", "spins"] {
            assert!(run["steps"][later].is_null(), "{later}: {run}");
        }
        assert!(
            text.contains("step stops not run: it needs an accepted probe"),
            "{text}"
        );
        assert!(!text.contains("step spins refused"), "{text}");
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);
        let _ = std::fs::remove_dir_all(out);

        let none = steps::StepInputs::load(None, None).expect("inputs");
        let dir = scratch("step-spins-none");
        let e = block_on(steps::step_spins(
            &mut c,
            Id::new(1),
            &d,
            &none,
            &dir,
            "uid",
            &stop,
        ))
        .expect_err("refused");
        assert!(
            format!("{e:#}").contains("step spins drives the motor"),
            "{e:#}"
        );
        assert!(!dir.join("uid").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// `step verify` on a servo whose shaft rests by a soft limit: refused
    /// before any write, the refusal printed and kept in the run folder's
    /// manifest; with no nameplate, refused before the bus is touched.
    #[test]
    fn step_verify_is_refused_by_a_soft_limit_and_keeps_a_run_folder() {
        let (mut c, d) = fake(BaudRate::B3000000);
        c.pipe_mut()
            .sim_mut()
            .servo_table_mut(0, |t| t.telemetry.estimates.theta_hat_q16 = 240 << 16);
        let out = scratch("step-verify");
        let stop = AtomicBool::new(false);
        let (lines, accepted, root) = block_on(steps::step_verify(
            &mut c,
            Id::new(1),
            &d,
            &inputs(),
            &out,
            "uid",
            &stop,
        ))
        .expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        assert!(
            text.contains("refused before any write") && text.contains("turn the horn"),
            "{text}"
        );
        let run: serde_json::Value =
            serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
                .expect("json");
        assert_eq!(run["steps"]["verify"]["state"], "refused", "{run}");
        assert!(!root.join("results/verify.json").exists());
        assert_eq!(block_on(c.read(Id::new(1), 0x180, 1)).expect("read"), [0]);

        let none = steps::StepInputs::load(None, None).expect("inputs");
        let e = block_on(steps::step_verify(
            &mut c,
            Id::new(1),
            &d,
            &none,
            &out,
            "other",
            &stop,
        ))
        .expect_err("refused");
        assert!(format!("{e:#}").contains("--nameplate"), "{e:#}");
        assert!(!out.join("other").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(out);
    }

    /// A user's nameplate short of a required value never loads: the
    /// refusal names the file and the field; `step probe` with none given
    /// is refused before the bus is touched.
    #[test]
    fn a_nameplate_file_missing_a_field_is_refused_naming_it() {
        let dir = scratch("nameplate");
        let mut v: serde_json::Value =
            serde_json::from_slice(&files::read(&testdata("nameplate.json")).expect("read"))
                .expect("json");
        v.as_object_mut().expect("object").remove("travel_deg");
        let p = dir.join("nameplate.json");
        std::fs::write(&p, serde_json::to_vec(&v).expect("json")).expect("write");
        let e = steps::StepInputs::load(Some(&p), None)
            .err()
            .expect("refused");
        let e = format!("{e:#}");
        assert!(
            e.contains("nameplate.json") && e.contains("no usable travel_deg"),
            "{e}"
        );

        let (mut c, d) = fake(BaudRate::B3000000);
        let none = steps::StepInputs::load(None, None).expect("inputs");
        let e = block_on(steps::step_probe(
            &mut c,
            Id::new(1),
            &d,
            &none,
            &dir,
            "uid",
            &AtomicBool::new(false),
        ))
        .expect_err("refused");
        assert!(format!("{e:#}").contains("--nameplate"), "{e:#}");
        assert!(!dir.join("uid").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(dir);
    }
}
