//! `osc lut build`: the dataset's settled rungs through osc-ident's grid
//! builder, a summary of what it kept and what it made, and the image JSON.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use osc_ident::lut::{self, Build, BuildError, GRID, Image, Verdict};

use crate::capture::envelope::Limits;
use crate::capture::rungs::{self, Rung};
use crate::capture::store::{Decl, Store};

/// `osc lut build` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// A dataset dir, `<root>/<servo>__<supply>`, with its dataset.toml.
    dataset: PathBuf,
    /// Where the image lands [default: <dataset>/pot-lut.json].
    #[arg(long)]
    out: Option<PathBuf>,
    /// Pot count at the low stop; with --raw-max, overrides the envelope's
    /// phys limits.
    #[arg(long, requires = "raw_max")]
    raw_min: Option<u16>,
    /// Pot count at the high stop.
    #[arg(long, requires = "raw_min")]
    raw_max: Option<u16>,
}

pub(crate) fn run(a: &Args) -> Result<()> {
    let stops = a.raw_min.zip(a.raw_max);
    let built = build(&a.dataset, stops)?;
    print!("{}", summary(&built, stops.is_none()));
    let out = a
        .out
        .clone()
        .unwrap_or_else(|| a.dataset.join("pot-lut.json"));
    write(&built.image, &out)?;
    println!("wrote {}", out.display());
    Ok(())
}

pub(crate) struct Built {
    pub(crate) stops: (u16, u16),
    pub(crate) rungs: Vec<Rung>,
    pub(crate) build: Build,
    pub(crate) image: Image,
}

/// The table from every settled rung of the dataset, against `stops` or
/// the envelope's phys limits.
pub(crate) fn build(dataset: &Path, stops: Option<(u16, u16)>) -> Result<Built> {
    let decl = Decl::load(&Store::new(dataset.to_path_buf()))?;
    let stops = match stops {
        Some(s) => s,
        None => {
            let l = Limits::load(dataset)
                .context("no stops: pass --raw-min and --raw-max")?
                .phys;
            (l[0], l[1])
        }
    };
    let rungs = rungs::gather(dataset)?;
    if rungs.is_empty() {
        bail!("{}: no settled rungs in any recording", dataset.display());
    }
    let refs: Vec<lut::Rung> = rungs
        .iter()
        .map(|r| lut::Rung {
            pos: &r.pos,
            current: &r.current,
            duty_pct: r.duty_pct,
            tick_hz: r.tick_hz,
        })
        .collect();
    let build = lut::build(&refs, stops.0, stops.1).map_err(|e| match e {
        BuildError::NoPrior => anyhow::anyhow!(
            "no coupling prior: no {}..{}% rung carries a ripple line",
            lut::PRIOR_PCT.0,
            lut::PRIOR_PCT.1
        ),
        BuildError::Uncovered => anyhow::anyhow!(
            "no stretch of travel is crossed by {} accepted rungs",
            lut::MIN_RUNG_COVER
        ),
        BuildError::Rejected(r) => anyhow::anyhow!("the table fails the firmware's check: {r:?}"),
    })?;
    let name = decl.name();
    let source = format!(
        "osc {} lut build on {name} (git {}): the grid block of every slow-decay session \
         recording and the unchained drives of the bare sweeps, both directions, knots on \
         the stretch >= {} rungs cross",
        env!("CARGO_PKG_VERSION"),
        crate::sweep::git_sha(),
        lut::MIN_RUNG_COVER
    );
    let image = build.lut.image(
        stops.0,
        stops.1,
        &name,
        build.rungs(),
        build.covered,
        &source,
    );
    Ok(Built {
        stops,
        rungs,
        build,
        image,
    })
}

fn summary(b: &Built, stops_from_envelope: bool) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let (raw_min, raw_max) = b.stops;
    let _ = writeln!(
        s,
        "dataset: {}\nstops: {raw_min}..{raw_max}{}",
        b.image.dataset,
        if stops_from_envelope {
            " (envelope.toml phys)"
        } else {
            ""
        }
    );
    let count = |f: fn(&Verdict) -> bool| b.build.verdicts.iter().filter(|v| f(v)).count();
    let _ = writeln!(
        s,
        "rungs: {} gathered; {} used, {} off-core (outside {}..{}%), {} untracked, {} short, {} coupling",
        b.rungs.len(),
        count(|v| *v == Verdict::Used),
        count(|v| *v == Verdict::OffCore),
        lut::CORE_PCT.0,
        lut::CORE_PCT.1,
        count(|v| *v == Verdict::Untracked),
        count(|v| matches!(v, Verdict::Short(_))),
        count(|v| matches!(v, Verdict::Coupling(_))),
    );
    for (r, v) in b.rungs.iter().zip(&b.build.verdicts) {
        let why = match v {
            Verdict::Used | Verdict::OffCore => continue,
            Verdict::Untracked => "untracked".to_string(),
            Verdict::Short(f) => format!("short, tracked {:.3} of it", f),
            Verdict::Coupling(r) => format!("coupling {:.3}x the median", r),
        };
        let _ = writeln!(
            s,
            "  {} capture-{} {} seg {} ({:+}%): {why}",
            r.experiment, r.cap, r.recording, r.seg, r.duty_pct
        );
    }
    let (lo, hi) = b.build.covered;
    let _ = writeln!(
        s,
        "c_prior: {:.6} cycles/count\ncovered: {lo}..{hi} ({} counts, >= {} rungs)",
        b.build.c_prior,
        hi - lo,
        lut::MIN_RUNG_COVER
    );
    let knots = &b.build.lut.knots;
    let nonzero: Vec<usize> = (0..knots.len()).filter(|&k| knots[k] != 0).collect();
    match (nonzero.first(), nonzero.last()) {
        (Some(&a), Some(&z)) => {
            let peak = knots.iter().map(|c| c.abs()).max().unwrap_or(0);
            let _ = writeln!(
                s,
                "knots: {} nonzero at raw {}..{}, |c| up to {peak}",
                nonzero.len(),
                a * GRID as usize,
                z * GRID as usize
            );
        }
        _ => s.push_str("knots: none nonzero (identity)\n"),
    }
    let gains: Vec<(f64, usize)> = knots
        .windows(2)
        .enumerate()
        .filter(|(_, w)| w[0] != 0 || w[1] != 0)
        .map(|(k, w)| {
            (
                (GRID as i32 + w[1] as i32 - w[0] as i32) as f64 / GRID as f64,
                k,
            )
        })
        .collect();
    if let (Some(max), Some(min)) = (
        gains.iter().max_by(|x, y| x.0.total_cmp(&y.0)),
        gains.iter().min_by(|x, y| x.0.total_cmp(&y.0)),
    ) {
        let at = |k: usize| format!("{}..{}", k * GRID as usize, (k + 1) * GRID as usize);
        let _ = writeln!(
            s,
            "interval gain: steepest {:.2}x at raw {}, shallowest {:.2}x at raw {}",
            max.0,
            at(max.1),
            min.0,
            at(min.1)
        );
    }
    let _ = writeln!(
        s,
        "validate against {raw_min}..{raw_max}: {}",
        match b.build.lut.validate(raw_min, raw_max) {
            Ok(()) => "ok".to_string(),
            Err(r) => format!("REJECT {r:?}"),
        }
    );
    s
}

/// The image as the notebook writes it: pretty, a trailing newline.
fn render(img: &Image) -> Result<String> {
    Ok(serde_json::to_string_pretty(img).context("serialize image")? + "\n")
}

fn write(img: &Image, path: &Path) -> Result<()> {
    std::fs::write(path, render(img)?).with_context(|| format!("write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DATASET: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../notebooks/telemetry/mg90-a__2s"
    );
    const COMMITTED: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../ident/testdata/lut/pot-lut-mg90-a-grid.json"
    ));

    /// The dataset's rungs through the builder give the notebook's image
    /// byte for byte, once the source line is its own.
    #[test]
    fn mg90_a_image_matches_the_notebook_byte_for_byte() {
        let t0 = std::time::Instant::now();
        let mut b = build(Path::new(DATASET), None).unwrap();
        eprintln!(
            "mg90-a: {} rungs built in {:.1?}",
            b.rungs.len(),
            t0.elapsed()
        );
        assert_eq!(b.stops, (209, 3849));
        assert_eq!(b.rungs.len(), 220);
        assert_eq!(b.build.rungs(), 99);
        assert_eq!(b.build.covered, (542, 3520));
        assert!(
            b.image
                .source
                .starts_with("osc 0.1.0 lut build on mg90-a__2s"),
            "{}",
            b.image.source
        );

        let want: Image = serde_json::from_str(COMMITTED).unwrap();
        b.image.source = want.source.clone();
        assert_eq!(b.image, want);
        let out = std::env::temp_dir().join(format!("osc-lut-{}.json", std::process::id()));
        write(&b.image, &out).unwrap();
        assert_eq!(std::fs::read_to_string(&out).unwrap(), COMMITTED);
        std::fs::remove_file(&out).unwrap();

        let text = summary(&b, true);
        assert!(
            text.contains("stops: 209..3849 (envelope.toml phys)"),
            "{text}"
        );
        assert!(
            text.contains("rungs: 220 gathered; 99 used, 120 off-core"),
            "{text}"
        );
        assert!(text.contains("covered: 542..3520"), "{text}");
        assert!(text.contains("validate against 209..3849: ok"), "{text}");
    }
}
