//! The whole identification, the CLI's `run` and the browser's, over the
//! client's bus and a [`Host`]'s files and clock. The measurement chain
//! through the spins (`step spins`'s own: rest, probe, stops, table,
//! window, spins), then, every step accepted, the gather into
//! `<run folder>/inputs/` and the tune into `<run folder>/tune/`; on the
//! set the tune accepts, the cruise step, the gather again and the final
//! set's check into `<run folder>/final/`, backed off down the tune's
//! tries on a refusal on rest error alone ([`finish`]). Every argument is
//! checked ([`admit`]) before the bus is touched.

pub mod files;
pub mod finish;
pub mod gather;
pub mod steps;
pub mod store;
pub mod tune;

use std::path::Path;
use std::sync::atomic::AtomicBool;

use anyhow::{Context, Result, bail};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use serde_json::{Value, json};

use self::steps::StepInputs;
pub use self::store::{At, Store};

/// What a run takes from the host it runs on: its files, its clock, the
/// stop flag a step reads between rungs, and the tool's name the run's
/// records carry.
pub struct Host<'a> {
    pub files: &'a mut dyn Store,
    /// Unix seconds: the CLI's system clock, the browser's `Date.now()`.
    pub clock: &'a dyn Fn() -> f64,
    pub stop: &'a AtomicBool,
    /// `osc-ident 0.1.0`.
    pub tool: &'a str,
}

/// What `run` was given.
pub struct RunArgs<'a> {
    pub nameplate: &'a Path,
    pub policy: &'a Path,
    pub out: &'a Path,
    pub tries: u32,
}

/// The refusals that need no servo: the files parse, the profile and the
/// policy resolve, the profile states its heat model, and `--tries` is at
/// least 1. The lines said about the files read go into `said`, on a
/// refusal after the reading too.
pub fn admit(fs: &mut dyn Store, a: &RunArgs, said: &mut Vec<String>) -> Result<StepInputs> {
    if a.tries == 0 {
        bail!("--tries is at least 1");
    }
    let (i, notes) = StepInputs::read(fs, Some(a.nameplate), Some(a.policy))?;
    said.extend(notes);
    if i.thermal.is_none() {
        bail!(
            "{}: {}",
            a.nameplate.display(),
            osc_ident_inputs::no_heat_model()
        );
    }
    Ok(i)
}

/// The command line that runs `run` again as given.
fn again(id: Id, a: &RunArgs) -> String {
    format!(
        "osc-ident --id {} run --nameplate {} --policy {} --out {} --tries {}",
        id.as_byte(),
        a.nameplate.display(),
        a.policy.display(),
        a.out.display(),
        a.tries
    )
}

/// The run folder's manifest with the gather and the tune beside its
/// steps.
fn record(fs: &mut dyn Store, root: &Path, gather: Value, tune: Value) -> Result<()> {
    let p = root.join("run.json");
    let mut doc: Value = serde_json::from_slice(&files::read(fs, &p)?)
        .with_context(|| format!("parsing {}", p.display()))?;
    doc["gather"] = gather;
    doc["tune"] = tune;
    rewrite(fs, &p, &doc)
}

/// `run.json` written over with `doc`.
fn rewrite(fs: &mut dyn Store, p: &Path, doc: &Value) -> Result<()> {
    let mut text = serde_json::to_vec_pretty(doc)?;
    text.push(b'\n');
    fs.write_file(p, &text)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("writing {}", p.display()))
}

/// The cruise step and the final check beside the tune in `run.json`.
fn record_finish(fs: &mut dyn Store, root: &Path, cruise: Value, last: Value) -> Result<()> {
    let p = root.join("run.json");
    let mut doc: Value = serde_json::from_slice(&files::read(fs, &p)?)
        .with_context(|| format!("parsing {}", p.display()))?;
    doc["cruise"] = cruise;
    doc["final"] = last;
    rewrite(fs, &p, &doc)
}

fn not_run(why: &str) -> Value {
    json!({"state": "not run", "why": why})
}

/// `run` on the servo `servo` (its run folder's name) with the inputs
/// [`admit`] gave for `a` before anything went on the bus: the lines it
/// prints, and whether the tune accepted a set; the final set's verdict is
/// in the lines and in `run.json`.
pub async fn run<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    inputs: &StepInputs,
    servo: &str,
) -> Result<(Vec<String>, bool)> {
    let (mut lines, measured, root) =
        steps::step_spins(c, id, d, host, inputs, a.out, servo).await?;
    if !measured {
        let why = "not every step was accepted";
        record(host.files, &root, not_run(why), not_run(why))?;
        record_finish(host.files, &root, not_run(why), not_run(why))?;
        lines.push(format!(
            "the run stops: {why}, so nothing is gathered or tuned; to run it again: {}",
            again(id, a)
        ));
        return Ok((lines, false));
    }
    let (more, accepted) = gather_and_tune(c, id, d, host, a, &root).await?;
    lines.extend(more);
    if !accepted {
        let why = "the tune accepted no set";
        record_finish(host.files, &root, not_run(why), not_run(why))?;
        return Ok((lines, false));
    }
    match finish::finish(c, id, d, host, a, &root).await {
        Ok(f) => {
            record_finish(host.files, &root, f.cruise, f.last)?;
            lines.extend(f.lines);
        }
        Err(e) => {
            let why = format!("the run folder could not be read for the cruise: {e:#}");
            record_finish(host.files, &root, not_run(&why), not_run(&why))?;
            lines.push(format!("{why}; nothing was written, the tune's set stays"));
        }
    }
    Ok((lines, accepted))
}

/// The gather of the run folder `root`, its steps all accepted, into
/// `inputs/`, then the tune from it into `tune/`, both recorded in its
/// `run.json`.
pub async fn gather_and_tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    root: &Path,
) -> Result<(Vec<String>, bool)> {
    let mut lines = Vec::new();
    let inputs_dir = root.join("inputs");
    let t0 = (host.clock)();
    let gathered = gather::gather(host.files, root, a.nameplate, a.policy, &inputs_dir);
    let t1 = (host.clock)();
    let gather_files: Vec<String> = [
        "measured.json",
        "pos_lut.json",
        "cross-checks.json",
        "nameplate.json",
        "policy.json",
        "resolved.json",
        "thermal.json",
    ]
    .iter()
    .filter(|f| **f != "thermal.json" || host.files.exists(&inputs_dir.join(f)))
    .map(|f| format!("inputs/{f}"))
    .collect();
    match gathered {
        Err(e) => {
            let why = format!("{e:#}");
            record(
                host.files,
                root,
                json!({"state": "refused", "why": why, "started": t0, "ended": t1}),
                not_run("the gather was refused"),
            )?;
            lines.push(format!("gather refused: {why}"));
            lines.push("the run stops: nothing is tuned".into());
            return Ok((lines, false));
        }
        Ok(g) => {
            lines.push(format!("gather into {}:", inputs_dir.display()));
            lines.extend(g.into_iter().map(|l| format!("  {l}")));
        }
    }
    let gather = json!({"state": "accepted", "why": null, "started": t0, "ended": t1, "files": gather_files});

    let tune_dir = root.join("tune");
    let tuned = tune::tune(c, id, d, host, &inputs_dir, &tune_dir, a.tries).await;
    let t2 = (host.clock)();
    match tuned {
        Err(e) => {
            let why = format!("{e:#}");
            record(
                host.files,
                root,
                gather,
                json!({"state": "refused", "why": why, "started": t1, "ended": t2}),
            )?;
            lines.push(format!("tune refused: {why}"));
            lines.push(format!(
                "nothing was written; the measured inputs are kept: osc-ident --id {} tune --inputs {} --out {} --tries {} continues from them",
                id.as_byte(),
                inputs_dir.display(),
                tune_dir.display(),
                a.tries
            ));
            Ok((lines, false))
        }
        Ok((tune_lines, accepted)) => {
            let mut files = vec!["tune/tune.json".to_string()];
            if accepted {
                files.push("tune/set.json".into());
            }
            record(
                host.files,
                root,
                gather,
                json!({"state": if accepted { "accepted" } else { "refused" }, "why": null,
                       "started": t1, "ended": t2, "files": files}),
            )?;
            lines.push(format!("tune into {}:", tune_dir.display()));
            lines.extend(tune_lines);
            Ok((lines, accepted))
        }
    }
}
