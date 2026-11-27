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
use serde::Serialize;
use serde_json::{Value, json};

use self::steps::StepInputs;
pub use self::store::{At, Store};
use crate::steps::cruise;

/// The steps past the measurement chain, by their keys in `run.json`.
pub const GATHER: &str = "gather";
pub const TUNE: &str = "tune";
pub const FINAL: &str = "final";

/// What a run takes from the host it runs on: its files, its clock, the
/// stop flag a step reads between rungs, the feed its progress goes to,
/// and the tool's name the run's records carry.
pub struct Host<'a> {
    pub files: &'a mut dyn Store,
    /// Unix seconds: the CLI's system clock, the browser's `Date.now()`.
    pub clock: &'a dyn Fn() -> f64,
    pub stop: &'a AtomicBool,
    /// The browser's progress block; [`quiet`] for the CLI, which prints
    /// the lines the run returns.
    pub feed: &'a dyn Fn(&Progress),
    /// `osc-ident 0.1.0`.
    pub tool: &'a str,
}

/// What a run tells its host's feed as it goes, each step by its key in
/// `run.json`.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum Progress {
    /// The step begins; a step not run never does.
    Started { step: &'static str },
    /// Try `at` of at most `of` begins: the tune's, and the final check's
    /// down the back-off.
    Try {
        step: &'static str,
        at: u32,
        of: u32,
    },
    /// Lines as the CLI prints them, once said: a run's, in order, are
    /// the lines it returns.
    Lines { lines: Vec<String> },
    /// The step ends, in the state and why `run.json` records for it.
    Ended {
        step: &'static str,
        state: String,
        why: Option<String>,
    },
}

/// The feed of a host that prints the lines at the end.
pub fn quiet(_: &Progress) {}

/// The lines said past `told`, told.
fn tell_lines(feed: &dyn Fn(&Progress), lines: &[String], told: &mut usize) {
    if lines.len() > *told {
        feed(&Progress::Lines {
            lines: lines[*told..].to_vec(),
        });
        *told = lines.len();
    }
}

/// The end of `step` as its `run.json` entry `entry` records it.
fn tell_end(feed: &dyn Fn(&Progress), step: &'static str, entry: &Value) {
    feed(&Progress::Ended {
        step,
        state: entry["state"].as_str().unwrap_or_default().into(),
        why: entry["why"].as_str().map(Into::into),
    });
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
    doc[GATHER] = gather;
    doc[TUNE] = tune;
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
    doc[cruise::STEP] = cruise;
    doc[FINAL] = last;
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
    let feed = host.feed;
    let (mut lines, measured, root) =
        steps::step_spins(c, id, d, host, inputs, a.out, servo).await?;
    let mut told = lines.len();
    if !measured {
        let why = "not every step was accepted";
        record(host.files, &root, not_run(why), not_run(why))?;
        record_finish(host.files, &root, not_run(why), not_run(why))?;
        for step in [GATHER, TUNE, cruise::STEP, FINAL] {
            tell_end(feed, step, &not_run(why));
        }
        lines.push(format!(
            "the run stops: {why}, so nothing is gathered or tuned; to run it again: {}",
            again(id, a)
        ));
        tell_lines(feed, &lines, &mut told);
        return Ok((lines, false));
    }
    let (more, accepted) = gather_and_tune(c, id, d, host, a, &root).await?;
    lines.extend(more);
    told = lines.len();
    if !accepted {
        let why = "the tune accepted no set";
        record_finish(host.files, &root, not_run(why), not_run(why))?;
        tell_end(feed, cruise::STEP, &not_run(why));
        tell_end(feed, FINAL, &not_run(why));
        return Ok((lines, false));
    }
    match finish::finish(c, id, d, host, a, &root).await {
        Ok(f) => {
            record_finish(host.files, &root, f.cruise, f.last.clone())?;
            tell_end(feed, FINAL, &f.last);
            lines.extend(f.lines);
        }
        Err(e) => {
            let why = format!("the run folder could not be read for the cruise: {e:#}");
            record_finish(host.files, &root, not_run(&why), not_run(&why))?;
            tell_end(feed, cruise::STEP, &not_run(&why));
            tell_end(feed, FINAL, &not_run(&why));
            lines.push(format!("{why}; nothing was written, the tune's set stays"));
            tell_lines(feed, &lines, &mut told);
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
    let feed = host.feed;
    let (mut lines, mut told) = (Vec::new(), 0);
    let inputs_dir = root.join("inputs");
    feed(&Progress::Started { step: GATHER });
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
            let gather = json!({"state": "refused", "why": why, "started": t0, "ended": t1});
            let tune = not_run("the gather was refused");
            record(host.files, root, gather.clone(), tune.clone())?;
            lines.push(format!("gather refused: {why}"));
            lines.push("the run stops: nothing is tuned".into());
            tell_lines(feed, &lines, &mut told);
            tell_end(feed, GATHER, &gather);
            tell_end(feed, TUNE, &tune);
            return Ok((lines, false));
        }
        Ok(g) => {
            lines.push(format!("gather into {}:", inputs_dir.display()));
            lines.extend(g.into_iter().map(|l| format!("  {l}")));
        }
    }
    let gather = json!({"state": "accepted", "why": null, "started": t0, "ended": t1, "files": gather_files});
    tell_lines(feed, &lines, &mut told);
    tell_end(feed, GATHER, &gather);

    let tune_dir = root.join("tune");
    feed(&Progress::Started { step: TUNE });
    let tuned = tune::tune(c, id, d, host, &inputs_dir, &tune_dir, a.tries).await;
    let t2 = (host.clock)();
    match tuned {
        Err(e) => {
            let why = format!("{e:#}");
            let tune = json!({"state": "refused", "why": why, "started": t1, "ended": t2});
            record(host.files, root, gather, tune.clone())?;
            lines.push(format!("tune refused: {why}"));
            lines.push(format!(
                "nothing was written; the measured inputs are kept: osc-ident --id {} tune --inputs {} --out {} --tries {} continues from them",
                id.as_byte(),
                inputs_dir.display(),
                tune_dir.display(),
                a.tries
            ));
            tell_lines(feed, &lines, &mut told);
            tell_end(feed, TUNE, &tune);
            Ok((lines, false))
        }
        Ok((tune_lines, accepted)) => {
            let mut files = vec!["tune/tune.json".to_string()];
            if accepted {
                files.push("tune/set.json".into());
            }
            let tune = json!({"state": if accepted { "accepted" } else { "refused" }, "why": null,
                              "started": t1, "ended": t2, "files": files});
            record(host.files, root, gather, tune.clone())?;
            lines.push(format!("tune into {}:", tune_dir.display()));
            lines.extend(tune_lines);
            tell_lines(feed, &lines, &mut told);
            tell_end(feed, TUNE, &tune);
            Ok((lines, accepted))
        }
    }
}
