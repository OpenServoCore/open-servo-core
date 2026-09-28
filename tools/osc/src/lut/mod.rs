//! `osc lut` - the pot linearization table the firmware applies per sample
//! (osc-ident's `lut`): `build` stitches one from a capture dataset's
//! settled rungs and writes the image JSON. Nothing here touches a servo.

mod build;

use anyhow::Result;
use clap::Subcommand;

/// `osc lut` args. No bus: the top-level `--baud`/`--id` are unused.
#[derive(clap::Args, Debug)]
pub struct Args {
    #[command(subcommand)]
    cmd: LutCmd,
}

#[derive(Subcommand, Debug)]
enum LutCmd {
    /// Build the table from a dataset's settled constant-duty rungs (the
    /// session grid block and the bare sweeps such as `ends`), print how
    /// it went, and write the image JSON `osc lut write` takes.
    Build(build::Args),
}

pub fn run(args: &Args) -> Result<()> {
    match &args.cmd {
        LutCmd::Build(a) => build::run(a),
    }
}
