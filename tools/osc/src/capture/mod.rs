//! `osc capture` - the plant-capture campaign around `osc sweep`'s recorder.
//! A dataset is one servo on one supply, `<root>/<servo>__<supply>`; `pilot`
//! measures the servo and writes the envelope the campaign sizes its rung
//! windows by, `plan` expands the session procedure against it, `session`
//! runs it on the servo, and `check` re-reads a landed capture.

mod battery;
mod check;
mod envelope;
mod pilot;
mod plan;
mod procs;
mod run;
mod store;
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
    /// Measure steady-state speed per duty on this servo and write the
    /// dataset's envelope.toml: guard limits, v_ss fits, per-duty windows,
    /// and the coast block's top duty, each verified on the servo.
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
    /// its meta promises, every direction driven, none empty. No servo needed.
    Check(check::Args),
}

/// `osc capture plan` args.
#[derive(clap::Args, Debug)]
pub struct PlanArgs {
    /// Servo key; the dataset dir is `<root>/<servo>__<supply>`.
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
}

/// Seek drive, percent of full scale; the park's duty, gentle on arrival.
pub(crate) const SEEK_PCT: u8 = 15;
/// Breakout escalation ceiling; `osc sweep --seek-cap-pct`'s default.
pub(crate) const SEEK_CAP_PCT: u8 = 45;
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
        "  {} captures, warm-up {}, cooldown {} s, rest {} ms, baseline {} ms, seek {}%, \
         tel_mask {:#x}, rotate {}",
        p.captures,
        p.warmup,
        p.cooldown_s,
        p.rest_ms,
        p.baseline_ms,
        p.seek_pct,
        p.tel_mask,
        p.rotate
    );
    match p.supply.get(&a.supply) {
        Some(g) => println!(
            "  battery: refuse under {} mV, warn under {} mV",
            g.cells * g.cell_floor_mv,
            g.cells * g.cell_warn_mv
        ),
        None => println!("  battery: {} is not gated", a.supply.as_str()),
    }
    println!(
        "envelope: {} (coast top {}%)",
        dir.join("envelope.toml").display(),
        env.coast.top_pct
    );
    for n in a.captures.clone().unwrap_or(1..=p.captures) {
        println!("capture-{n}");
        for r in plan::expand(&p, &env, n)? {
            let order: Vec<&str> = r.blocks.iter().map(|b| b.name.as_str()).collect();
            println!(
                "  {} ({} decay, {} steps): {}",
                r.recording,
                r.decay.as_str(),
                r.schedule.len(),
                order.join(" ")
            );
            println!("    blocks: {}", serde_json::to_string(&r.blocks)?);
            let sched: Vec<String> = r.schedule.iter().map(Step::to_string).collect();
            println!("    schedule: {}", sched.join(","));
        }
    }
    Ok(())
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

fn default_root() -> Result<PathBuf> {
    Ok(crate::sweep::git_toplevel()?
        .join("notebooks")
        .join("telemetry"))
}

pub(crate) fn dataset_dir(root: &Path, servo: &str, supply: Supply) -> PathBuf {
    root.join(format!("{servo}__{}", supply.as_str()))
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
    fn dataset_dir_names_servo_and_supply() {
        let root = Path::new("/r");
        assert_eq!(
            dataset_dir(root, "mg90-a", Supply::TwoS),
            Path::new("/r/mg90-a__2s")
        );
        assert_eq!(
            dataset_dir(root, "mg90-a", Supply::Usb),
            Path::new("/r/mg90-a__usb")
        );
    }
}
