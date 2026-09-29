//! `osc capture` - the plant-capture campaign around `osc sweep`'s recorder.
//! A dataset is one servo on one supply under one drive rule,
//! `<root>/<servo>__<supply>__limit` for what this tool captures; `pilot`
//! measures the servo and writes the envelope the campaign sizes its rung
//! windows by, `plan` expands the session procedure against it, `session`
//! runs it on the servo, and `check` re-reads a landed capture.

mod check;
pub(crate) mod envelope;
mod front;
mod pilot;
mod plan;
mod procs;
mod run;
pub(crate) mod rungs;
pub(crate) mod store;
mod verdict;

use std::ops::RangeInclusive;
use std::path::{Path, PathBuf};

use anyhow::Result;
use clap::{Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};

use crate::sweep::Step;

/// `osc capture` args. `--baud`/`--id` come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    #[command(subcommand)]
    cmd: CaptureCmd,
}

#[derive(Subcommand, Debug)]
enum CaptureCmd {
    /// Measure the servo under its current limit and write the dataset's
    /// envelope.toml: per grid duty its climb, speed, braked stop and window,
    /// the coast ladder and every chain the session drives, each run on the
    /// servo only while it fits the runway; the grid again under fast decay
    /// for a recording that drives under it.
    Pilot(pilot::Args),
    /// Print the session procedure and, per capture, the block order, block
    /// map and each recording's schedule, expanded against the dataset's
    /// envelope. No servo needed.
    Plan(PlanArgs),
    /// Run the session procedure on the servo: battery gate, a discarded
    /// warm-up, then each capture's recordings, each accepted only when
    /// complete and clean. A rerun resumes at the first capture not landed.
    Session(run::Args),
    /// Re-read a landed capture dir: every recording must hold every segment
    /// its meta promises, every direction driven, none empty, all under the
    /// drive rule and current limit dataset.toml declares; under the limit,
    /// every grid and ends rung settled and no current window over the
    /// abort. No servo needed.
    Check(check::Args),
}

/// `osc capture plan` args.
#[derive(clap::Args, Debug)]
pub struct PlanArgs {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>__limit`.
    #[arg(long)]
    servo: String,
    /// Supply the servo runs on.
    #[arg(long, value_enum)]
    supply: Supply,
    /// Captures to plan, `n` or `a..b` inclusive; defaults to the procedure's
    /// count.
    #[arg(long, value_parser = parse_captures)]
    captures: Option<RangeInclusive<u32>>,
    /// Dataset root; defaults to notebooks/telemetry in this git checkout.
    #[arg(long)]
    root: Option<PathBuf>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Supply {
    Usb,
    #[value(name = "2s")]
    #[serde(rename = "2s")]
    TwoS,
}

impl Supply {
    fn as_str(self) -> &'static str {
        match self {
            Supply::Usb => "usb",
            Supply::TwoS => "2s",
        }
    }

    fn runway(self) -> osc_ident::runway::Supply {
        match self {
            Supply::Usb => osc_ident::runway::Supply::Usb,
            Supply::TwoS => osc_ident::runway::Supply::TwoS,
        }
    }
}

/// How the drive was held while a recording was made. Steady state does not
/// know which rule reached it; transients do, so a dataset holds one.
/// `lease`, a ceiling over the limit leased from the host, is specified and
/// not built.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Rule {
    /// Captured before the servo limited open-loop current: a meta with no
    /// `drive` block.
    #[default]
    Free,
    /// Captured under the servo's own `current_limit_counts`.
    Limit,
}

impl Rule {
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Rule::Free => "free",
            Rule::Limit => "limit",
        }
    }
}

/// What every capture this tool makes runs under: the firmware holds
/// open-loop current to the servo's limit.
pub(crate) const RULE: Rule = Rule::Limit;

/// Brake-and-hold before a rung arms; `osc sweep --settle-ms`'s default.
pub(crate) const SETTLE_MS: u32 = 300;
/// Torque-off cool-down between rungs; `osc sweep --rest-ms`'s default.
pub(crate) const REST_MS: u32 = 500;
/// Attempts per rung; `osc sweep --rung-tries`'s default.
pub(crate) const RUNG_TRIES: u32 = 3;
/// The raw six; `osc sweep --tel-mask`'s default.
pub(crate) const TEL_MASK: u16 = 0x1cd;
/// `osc sweep --window-ms`'s default. Every planned step carries its own
/// window, so this only reaches meta.json, where session.py reads it.
pub(crate) const WINDOW_MS: u32 = 150;

/// Entry from the top-level `osc capture` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    match &args.cmd {
        CaptureCmd::Pilot(a) => pilot::run(a, baud, id),
        CaptureCmd::Plan(a) => print_plan(a),
        CaptureCmd::Session(a) => run::run(a, baud, id),
        CaptureCmd::Check(a) => check::run(a),
    }
}

fn print_plan(a: &PlanArgs) -> Result<()> {
    let root = match &a.root {
        Some(r) => r.clone(),
        None => default_root()?,
    };
    let dir = dataset_dir(&root, &a.servo, a.supply);
    let env = envelope::Envelope::load(&dir)?;
    let (p, source) = procs::Procedure::load()?;
    println!("procedure: {source}");
    println!(
        "  {} captures, warm-up {}, cooldown {} s, rest {} ms, baseline {} ms, tel_mask {:#x}, \
         rotate {}",
        p.captures, p.warmup, p.cooldown_s, p.rest_ms, p.baseline_ms, p.tel_mask, p.rotate
    );
    match p.supply.get(&a.supply) {
        Some(g) => println!(
            "  battery: refuse under {} mV, warn under {} mV",
            g.cells * g.cell_floor_mv,
            g.cells * g.cell_warn_mv
        ),
        None => println!("  battery: {} is not gated", a.supply.as_str()),
    }
    let fast = env
        .fast
        .as_ref()
        .map_or("none".to_string(), |g| format!("{}%", g.top_pct));
    println!(
        "envelope: {} (grid top {}%, under fast decay {fast}, coast top {}%)",
        dir.join("envelope.toml").display(),
        env.grid.top_pct,
        env.coast.top_pct
    );
    for n in a.captures.clone().unwrap_or(1..=p.captures) {
        println!("capture-{n}");
        for r in plan::expand(&p, &env, n)? {
            let order: Vec<&str> = r.blocks.iter().map(|b| b.name.as_str()).collect();
            println!(
                "  {} ({} decay, {} steps, {:.1} s streamed each way): {}",
                r.recording,
                r.decay.as_str(),
                r.schedule.len(),
                streamed_ms(&r.schedule) as f64 / 1000.0,
                order.join(" ")
            );
            for d in &r.dropped {
                println!("    {d}");
            }
            println!("    blocks: {}", serde_json::to_string(&r.blocks)?);
            let sched: Vec<String> = r.schedule.iter().map(Step::to_string).collect();
            println!("    schedule: {}", sched.join(","));
        }
    }
    Ok(())
}

/// The TEL time a schedule streams in one direction, ms: every step's window.
fn streamed_ms(schedule: &[Step]) -> u32 {
    schedule
        .iter()
        .map(|s| match *s {
            Step::Drive(_, ms) | Step::Then(_, ms) => ms.unwrap_or(WINDOW_MS),
            Step::Coast(ms) | Step::Brake(ms) => ms,
        })
        .sum()
}

/// `n` or `a..b`, inclusive, counting from 1.
fn parse_captures(s: &str) -> Result<RangeInclusive<u32>, String> {
    let n = |v: &str| {
        v.parse::<u32>()
            .ok()
            .filter(|&n| n > 0)
            .ok_or_else(|| format!("capture {v:?} is not a count from 1"))
    };
    let (a, b) = match s.split_once("..") {
        Some((a, b)) => (n(a)?, n(b)?),
        None => (n(s)?, n(s)?),
    };
    if a > b {
        return Err(format!("{s}: first capture after last"));
    }
    Ok(a..=b)
}

pub(crate) fn default_root() -> Result<PathBuf> {
    Ok(crate::sweep::git_toplevel()?
        .join("notebooks")
        .join("telemetry"))
}

/// `<servo>__<supply>` for a `free` dataset, `<servo>__<supply>__<rule>`
/// for any other.
pub(crate) fn dataset_name(servo: &str, supply: Supply, rule: Rule) -> String {
    match rule {
        Rule::Free => format!("{servo}__{}", supply.as_str()),
        r => format!("{servo}__{}__{}", supply.as_str(), r.as_str()),
    }
}

/// The dataset this tool captures into: [`RULE`]'s.
pub(crate) fn dataset_dir(root: &Path, servo: &str, supply: Supply) -> PathBuf {
    root.join(dataset_name(servo, supply, RULE))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captures_parse_as_one_or_a_range() {
        assert_eq!(parse_captures("3"), Ok(3..=3));
        assert_eq!(parse_captures("1..5"), Ok(1..=5));
        assert_eq!(parse_captures("2..2"), Ok(2..=2));
        for bad in ["0", "0..3", "4..2", "1..", "..3", "a", "1..5..6", "-1"] {
            assert!(parse_captures(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn dataset_dir_names_servo_supply_and_rule() {
        let root = Path::new("/r");
        assert_eq!(
            dataset_dir(root, "mg90-a", Supply::TwoS),
            Path::new("/r/mg90-a__2s__limit")
        );
        assert_eq!(
            dataset_dir(root, "mg90-a", Supply::Usb),
            Path::new("/r/mg90-a__usb__limit")
        );
        assert_eq!(
            dataset_name("mg90-a", Supply::TwoS, Rule::Free),
            "mg90-a__2s"
        );
    }
}
