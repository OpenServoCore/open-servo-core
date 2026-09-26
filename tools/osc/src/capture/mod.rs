//! `osc capture` - the plant-capture campaign around `osc sweep`'s recorder.
//! A dataset is one servo on one supply, `<root>/<servo>__<supply>`; `pilot`
//! measures the servo and writes the envelope the campaign sizes its rung
//! windows by.

mod envelope;
mod pilot;

use std::path::{Path, PathBuf};

use anyhow::Result;
use clap::{Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};

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
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum, Serialize, Deserialize)]
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

/// Entry from the top-level `osc capture` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    match &args.cmd {
        CaptureCmd::Pilot(a) => pilot::run(a, baud, id),
    }
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
