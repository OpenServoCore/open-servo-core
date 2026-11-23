//! osc-ident: a servo's constant set, and the safe path that puts a set on
//! a servo. `rules` touches no servo; `check` only reads; `install`,
//! `save` and `restore` write with torque off and never drive the motor.

mod files;

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use futures_lite::future::block_on;
use osc_client::descriptor::{Descriptor, Registry, Selection};
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::{BaudRate, Client, Error, Id, mgmt};
use osc_measure::build;
use osc_measure::install::{self, Step};
use osc_measure::set::{self, FromRules};
use serde_json::json;

use crate::files::DirSink;

/// The table layouts this tool knows, compiled from the checked-in export.
const DESCRIPTORS: &[&str] = &[include_str!("../../../descriptors/osc-servo/0.1.json")];

#[derive(Parser, Debug)]
#[command(
    name = "osc-ident",
    version,
    about = "A servo's constant set, and the safe path that puts it on the servo"
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
    /// Measured inputs, the nameplate and a policy to a set file. Touches no servo.
    Rules {
        /// The inputs folder: measured.json, nameplate.json, policy.json and,
        /// where measured, thermal.json, shelf.json, pos_lut.json, wide-codes.json.
        #[arg(long)]
        inputs: PathBuf,
        /// A policy file other than the folder's.
        #[arg(long)]
        policy: Option<PathBuf>,
        /// The board's description; the dev board's if omitted.
        #[arg(long)]
        board: Option<PathBuf>,
        /// The set's pair name; `<servo>__<board>` if omitted.
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
    /// Make the servo equal the set, volatile, every write read back, then stamp it.
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
        /// The protection fields the restore may set to protect less.
        #[arg(long, num_args = 1..)]
        loosen: Vec<String>,
    },
}

#[derive(clap::Args, Debug)]
struct WriteArgs {
    /// The table file of a LIVE set's points; `<pair>.pos_lut.json` beside the set if omitted.
    #[arg(long)]
    table: Option<PathBuf>,
    /// The protection fields the set may move to protect less than the servo holds.
    #[arg(long, num_args = 1..)]
    loosen: Vec<String>,
}

fn rules(
    inputs: &Path,
    policy: Option<&Path>,
    board: Option<&Path>,
    pair: Option<&str>,
    out: &Path,
) -> Result<()> {
    let r = files::rules_inputs(inputs, policy, board)?;
    let pair = pair.map_or_else(
        || format!("{}__{}", r.inputs.nameplate.name, r.inputs.board.name),
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
        "{} differences; data state {}; stamp {}; set {}",
        checked.differences.len(),
        checked.data,
        checked.stamp,
        &set.hash()[..12.min(set.hash().len())]
    );
    Ok(checked.clean())
}

/// The servo's name in a run folder: its UID where the bus says it.
async fn servo_name<P: Pipe>(c: &mut Client<P>, id: Id) -> String {
    match mgmt::discover(c).await {
        Ok(found) => match found.iter().find(|f| f.id == id) {
            Some(f) => format!("{:?}", f.uid),
            None => format!("id{}", id.as_byte()),
        },
        Err(_) => format!("id{}", id.as_byte()),
    }
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
                &w.loosen,
                &mut steps,
            )
            .await
        }
        Cmd::Save { set, w, out } => {
            let file = files::set(set)?;
            let table = files::table_for(set, &file, w.table.as_deref())?;
            let root = files::run_folder(out, &servo_name(c, id).await)?;
            let mut sink = DirSink { root };
            install::save(
                c,
                id,
                d,
                &file,
                table.as_ref().map(|t| &t.points),
                &w.loosen,
                &mut sink,
                &mut steps,
            )
            .await
        }
        Cmd::Restore { from, loosen } => {
            let bytes = files::read(from)?;
            let text = String::from_utf8(bytes.clone()).context("a record is UTF-8")?;
            let origin = json!({"from": "a record restored", "file": from.display().to_string(), "sha1": set::file_sha1(&bytes)});
            install::restore(c, id, d, &text, origin, loosen, &mut steps).await
        }
        Cmd::Rules { .. } | Cmd::Check { .. } => bail!("not a write verb"),
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
            println!("saved: after a reboot the servo reads as the set, data state clean")
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
            board,
            pair,
            out,
        } => {
            rules(
                inputs,
                policy.as_deref(),
                board.as_deref(),
                pair.as_deref(),
                out,
            )?;
            Ok(true)
        }
        Cmd::Check { set } => block_on(async {
            let (mut c, d) = connect(&cli.baud, id).await?;
            check(&mut c, id, &d, set).await
        }),
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
    }

    /// `rules` on the mg90-d inputs writes a set that `check` reads as the
    /// servo's, on a simulated servo that holds the set of record.
    #[test]
    fn rules_on_the_record_inputs_checks_clean_against_the_set_of_record() {
        let dir = scratch("rules");
        let out = dir.join("set.json");
        rules(&testdata(""), None, None, None, &out).expect("rules");
        assert!(
            rules(&testdata(""), None, None, None, &out).is_err(),
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
        let loosen: Vec<String> = record
            .values()
            .iter()
            .filter(|(n, _)| set::looser(n).is_some())
            .map(|(n, _)| n.to_string())
            .collect();
        block_on(install::install(
            &mut c,
            id,
            &d,
            &record,
            Some(&table.points),
            &loosen,
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
            &[],
            &mut steps,
        ))
        .expect("installs");
        assert_eq!(steps, [], "the servo already holds it");
        let _ = std::fs::remove_dir_all(dir);
    }
}
