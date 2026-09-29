//! `osc lut` - the position linearization table the firmware applies per
//! sample (osc-ident's `lut`): `build` stitches one from a capture dataset
//! and writes the image JSON, `grade` graphs an image or the servo's table,
//! `write` puts an image on the servo, `show` reads the servo's back, `clear`
//! returns it to the identity. Nothing here stamps: a new table re-defines
//! the domain the identified set was fitted in, so every write leaves
//! STAMP_MISMATCH for `osc ident` to clear.

mod build;
mod grade;
mod servo;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use clap::Subcommand;
use osc_ident::lut::{GridLut, Image};

/// `osc lut` args. `build` and `grade <image>` need no bus; the rest take
/// the top-level `--baud`/`--id`.
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
    /// Graph an image's (or, with --servo, the servo's) local gain per
    /// interval and grade it; advisory, the operator decides.
    Grade(GradeArgs),
    /// Write an image to the servo: validated against its stops here
    /// first, then STORE, COMMIT, knot-for-knot readback. Torque off.
    Write(servo::WriteArgs),
    /// The servo's table: state, band, nonzero knots, grade.
    Show(servo::ShowArgs),
    /// Return the servo to the identity: an all-zero table COMMITted LIVE.
    Clear,
}

#[derive(clap::Args, Debug)]
pub(crate) struct GradeArgs {
    /// An image JSON (from `osc lut build` or the notebook).
    #[arg(required_unless_present = "servo", conflicts_with = "servo")]
    image: Option<PathBuf>,
    /// Grade the table the servo holds against its own stops.
    #[arg(long)]
    servo: bool,
    /// The report as JSON instead of the chart and summary.
    #[arg(long)]
    json: bool,
}

pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    match &args.cmd {
        LutCmd::Build(a) => build::run(a),
        LutCmd::Grade(a) => match &a.image {
            Some(path) => {
                let img = load(path)?;
                let lut = on_grid(&img, path)?;
                let r = grade::Report::new(&lut, (img.raw_min, img.raw_max));
                if a.json {
                    println!("{}", serde_json::to_string_pretty(&r)?);
                } else {
                    println!("image: {}", describe(&img, path));
                    print!("{}{}", r.chart(), r.summary());
                }
                Ok(())
            }
            None => servo::grade(baud, id, a.json),
        },
        LutCmd::Write(a) => servo::write(a, baud, id),
        LutCmd::Show(a) => servo::show(a, baud, id),
        LutCmd::Clear => servo::clear(baud, id),
    }
}

pub(crate) fn load(path: &Path) -> Result<Image> {
    let text = std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?;
    serde_json::from_str(&text).with_context(|| format!("parse {}", path.display()))
}

pub(crate) fn on_grid(img: &Image, path: &Path) -> Result<GridLut> {
    match img.lut() {
        Some(lut) => Ok(lut),
        None => bail!(
            "{}: not on the firmware grid (grid_shift {}, {} knots; want 4 and 256)",
            path.display(),
            img.grid_shift,
            img.points.len()
        ),
    }
}

pub(crate) fn describe(img: &Image, path: &Path) -> String {
    format!(
        "{} ({}, {} rungs, covered {}..{}, stops {}..{})",
        path.display(),
        img.dataset,
        img.rungs,
        img.covered[0],
        img.covered[1],
        img.raw_min,
        img.raw_max
    )
}
