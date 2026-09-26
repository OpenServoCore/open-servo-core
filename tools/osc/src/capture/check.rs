//! `osc capture check`: re-read a landed capture and confirm every recording
//! holds every segment its own meta promises, without the servo.

use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use flate2::read::GzDecoder;
use serde::Deserialize;

use super::verdict::expected_segments;

/// `osc capture check` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// A capture dir: `<dataset>/<experiment>/capture-N`.
    dir: PathBuf,
}

/// The sweep meta keys the segment count follows from.
#[derive(Deserialize)]
struct SweepMeta {
    schedule: Vec<String>,
    dirs: Vec<i8>,
    baseline_ms: u32,
}

pub(crate) fn run(a: &Args) -> Result<()> {
    let names = recordings(&a.dir)?;
    if names.is_empty() {
        bail!("no recordings in {}", a.dir.display());
    }
    let mut failed = 0;
    for name in &names {
        match check(&a.dir, name) {
            Ok(line) => println!("{name}: {line}"),
            Err(e) => {
                failed += 1;
                println!("{name}: FAIL {e:#}");
            }
        }
    }
    if failed > 0 {
        bail!("{failed} of {} recordings failed", names.len());
    }
    println!("ok: {} recordings complete", names.len());
    Ok(())
}

/// Every recording name with a `.meta.json` or a `.csv.gz`, so a half pair
/// still gets its own failure line.
fn recordings(dir: &Path) -> Result<BTreeSet<String>> {
    let mut names = BTreeSet::new();
    for e in std::fs::read_dir(dir).with_context(|| format!("read {}", dir.display()))? {
        let file = e?.file_name().to_string_lossy().into_owned();
        if let Some(n) = file
            .strip_suffix(".meta.json")
            .or_else(|| file.strip_suffix(".csv.gz"))
        {
            names.insert(n.to_string());
        }
    }
    Ok(names)
}

fn check(dir: &Path, name: &str) -> Result<String> {
    let meta_path = dir.join(format!("{name}.meta.json"));
    let text = std::fs::read_to_string(&meta_path)
        .with_context(|| format!("read {}", meta_path.display()))?;
    let m: SweepMeta =
        serde_json::from_str(&text).with_context(|| format!("parse {}", meta_path.display()))?;
    let csv_path = dir.join(format!("{name}.csv.gz"));
    let f =
        std::fs::File::open(&csv_path).with_context(|| format!("open {}", csv_path.display()))?;
    let (rows, dirs) = count_rows(BufReader::new(GzDecoder::new(f)))
        .with_context(|| format!("read {}", csv_path.display()))?;

    let baseline = m.baseline_ms > 0;
    let want = expected_segments(baseline, m.dirs.len(), m.schedule.len());
    let expected: BTreeSet<u32> = (u32::from(!baseline)..).take(want).collect();
    let missing: Vec<u32> = expected
        .iter()
        .filter(|s| !rows.contains_key(s))
        .copied()
        .collect();
    if !missing.is_empty() {
        bail!(
            "{} of {want} segments, missing seg {missing:?}",
            want - missing.len()
        );
    }
    let extra: Vec<u32> = rows
        .keys()
        .filter(|s| !expected.contains(s))
        .copied()
        .collect();
    if !extra.is_empty() {
        bail!("seg {extra:?} beyond the {want} the schedule promises");
    }
    if let Some(d) = m.dirs.iter().find(|d| !dirs.contains(d)) {
        bail!("no rows drive dir {d:+}");
    }
    let total: usize = rows.values().sum();
    Ok(format!("{want} segments, {total} rows, dirs {:?}", m.dirs))
}

/// Rows per seg, and every dir seen; the header names the columns.
fn count_rows(r: impl BufRead) -> Result<(BTreeMap<u32, usize>, BTreeSet<i8>)> {
    let mut lines = r.lines();
    let header = lines.next().ok_or_else(|| anyhow!("empty csv"))??;
    let col = |name| {
        header
            .split(',')
            .position(|c| c == name)
            .ok_or_else(|| anyhow!("no {name} column"))
    };
    let (seg_col, dir_col) = (col("seg")?, col("dir")?);
    let mut rows = BTreeMap::new();
    let mut dirs = BTreeSet::new();
    for (i, line) in lines.enumerate() {
        let line = line?;
        let cells: Vec<&str> = line.split(',').collect();
        let field = |c: usize| cells.get(c).copied().unwrap_or_default();
        let bad = || anyhow!("row {}: bad seg/dir in {line:?}", i + 2);
        let seg: u32 = field(seg_col).parse().map_err(|_| bad())?;
        let dir: i8 = field(dir_col).parse().map_err(|_| bad())?;
        *rows.entry(seg).or_insert(0) += 1;
        if seg > 0 {
            dirs.insert(dir);
        }
    }
    Ok((rows, dirs))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::store::Store;
    use crate::capture::store::fixture::{clean, land, seg, tmp};

    #[test]
    fn a_landed_capture_checks_clean() {
        let root = tmp("check-clean");
        let dir = land(&Store::new(root.clone()), &clean());
        assert_eq!(
            check(&dir, "slow").unwrap(),
            "5 segments, 15 rows, dirs [1, -1]"
        );
        assert!(run(&Args { dir: dir.clone() }).is_ok());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn short_or_one_sided_recordings_fail() {
        let root = tmp("check-short");
        let mut segs = clean();
        segs.pop();
        let dir = land(&Store::new(root.clone()), &segs);
        let e = check(&dir, "slow").unwrap_err().to_string();
        assert_eq!(e, "4 of 5 segments, missing seg [4]");
        assert!(run(&Args { dir: dir.clone() }).is_err());

        let segs: Vec<_> = clean()
            .into_iter()
            .map(|mut s| {
                s.dir = s.dir.max(0);
                s
            })
            .collect();
        let dir = land(&Store::new(root.clone()), &segs);
        assert_eq!(
            check(&dir, "slow").unwrap_err().to_string(),
            "no rows drive dir -1"
        );

        // an empty segment writes no rows, so it reads as missing
        let mut segs = clean();
        segs[2] = seg(2, 1, 0);
        let dir = land(&Store::new(root.clone()), &segs);
        assert!(
            check(&dir, "slow")
                .unwrap_err()
                .to_string()
                .contains("missing seg [2]")
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_half_pair_fails_on_its_own_line() {
        let root = tmp("check-half");
        let dir = land(&Store::new(root.clone()), &clean());
        std::fs::copy(dir.join("slow.meta.json"), dir.join("fast.meta.json")).unwrap();
        assert_eq!(
            recordings(&dir).unwrap().into_iter().collect::<Vec<_>>(),
            ["fast", "slow"]
        );
        assert!(check(&dir, "fast").is_err());
        assert!(run(&Args { dir: dir.clone() }).is_err());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn rows_count_by_the_header() {
        let csv = "dir,x,seg\n0,a,0\n1,b,1\n1,c,1\n-1,d,2\n";
        let (rows, dirs) = count_rows(csv.as_bytes()).unwrap();
        assert_eq!(rows, [(0, 1), (1, 2), (2, 1)].into());
        assert_eq!(dirs, [1, -1].into());
        assert!(count_rows("seg,dir\nx,1\n".as_bytes()).is_err());
        assert!(count_rows("tick\n1\n".as_bytes()).is_err());
    }
}
