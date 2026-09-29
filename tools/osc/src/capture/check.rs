//! `osc capture check`: re-read a landed capture and confirm every recording
//! holds every segment its own meta promises, without the servo; over a
//! dataset or experiment dir, every capture under it, and that they were
//! all made under one position table.

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
    /// A capture dir `<dataset>/<experiment>/capture-N`, or a dataset or
    /// experiment dir holding captures.
    dir: PathBuf,
}

/// The sweep meta keys the segment count follows from, and the plant
/// block the captures must agree on.
#[derive(Deserialize)]
struct SweepMeta {
    schedule: Vec<String>,
    dirs: Vec<i8>,
    baseline_ms: u32,
    #[serde(default)]
    plant: Option<PlantMeta>,
}

/// The table a recording was made under, as its meta names it.
#[derive(Deserialize, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct PlantMeta {
    lut_state: String,
    lut_crc: String,
}

pub(crate) fn run(a: &Args) -> Result<()> {
    let dirs = capture_dirs(&a.dir)?;
    let mut total = 0;
    let mut failed = 0;
    let mut tables: BTreeMap<Option<PlantMeta>, Vec<String>> = BTreeMap::new();
    for dir in &dirs {
        let names = recordings(dir)?;
        for name in &names {
            total += 1;
            let label = match dir.strip_prefix(&a.dir) {
                Ok(rel) if !rel.as_os_str().is_empty() => format!("{}/{name}", rel.display()),
                _ => name.clone(),
            };
            match check(dir, name) {
                Ok((line, plant)) => {
                    println!("{label}: {line}");
                    tables.entry(plant).or_default().push(label);
                }
                Err(e) => {
                    failed += 1;
                    println!("{label}: FAIL {e:#}");
                }
            }
        }
    }
    if total == 0 {
        bail!("no recordings in {}", a.dir.display());
    }
    if tables.len() > 1 {
        println!("FAIL {}", mixed(&tables));
    }
    if failed > 0 {
        bail!("{failed} of {total} recordings failed");
    }
    if tables.len() > 1 {
        bail!("{}", mixed(&tables));
    }
    println!("ok: {total} recordings complete");
    Ok(())
}

/// The capture dirs under `dir`: itself when it holds recordings, else
/// every `capture-N` one or two levels down (an experiment or a dataset).
fn capture_dirs(dir: &Path) -> Result<Vec<PathBuf>> {
    if !recordings(dir)?.is_empty() {
        return Ok(vec![dir.to_path_buf()]);
    }
    let is_capture = |p: &Path| {
        p.is_dir()
            && p.file_name()
                .and_then(|f| f.to_str())
                .is_some_and(|f| f.starts_with("capture-"))
    };
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir).with_context(|| format!("read {}", dir.display()))? {
        let p = e?.path();
        if is_capture(&p) {
            out.push(p);
        } else if p.is_dir() {
            for e in std::fs::read_dir(&p).with_context(|| format!("read {}", p.display()))? {
                let p = e?.path();
                if is_capture(&p) {
                    out.push(p);
                }
            }
        }
    }
    out.sort();
    Ok(out)
}

/// The mix, one table per line with the recordings made under it.
fn mixed(tables: &BTreeMap<Option<PlantMeta>, Vec<String>>) -> String {
    let groups: Vec<String> = tables
        .iter()
        .map(|(t, names)| {
            let table = match t {
                Some(p) => format!("lut {} {}", p.lut_state, p.lut_crc),
                None => "no plant record".to_string(),
            };
            format!("{table}: {}", names.join(" "))
        })
        .collect();
    format!(
        "recordings mix {} pot tables; {}",
        tables.len(),
        groups.join("; ")
    )
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

fn check(dir: &Path, name: &str) -> Result<(String, Option<PlantMeta>)> {
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
    let plant = match &m.plant {
        Some(p) => format!(", lut {} {}", p.lut_state, p.lut_crc),
        None => String::new(),
    };
    Ok((
        format!("{want} segments, {total} rows, dirs {:?}{plant}", m.dirs),
        m.plant,
    ))
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
            check(&dir, "slow").unwrap().0,
            "5 segments, 15 rows, dirs [1, -1]"
        );
        assert!(run(&Args { dir: dir.clone() }).is_ok());
        std::fs::remove_dir_all(&root).unwrap();
    }

    /// A capture dir's recordings, or every capture under a dataset dir,
    /// must all name one table: a reboot that dropped an unsaved table, or
    /// a table rewritten between captures, shows as a mix.
    #[test]
    fn captures_under_different_tables_fail_together() {
        use crate::capture::store::fixture::{land_as, sweep_meta};
        let root = tmp("check-mix");
        let store = Store::new(root.clone());
        let live = |crc: &str| {
            let mut m = sweep_meta();
            m["plant"] = serde_json::json!({ "lut_state": "LIVE", "lut_crc": crc });
            m
        };
        land_as(&store, 1, "slow", &live("0x1a2b"), &clean());
        land_as(&store, 1, "fast", &live("0x1a2b"), &clean());
        assert_eq!(
            check(&root.join("session/capture-1"), "slow").unwrap().0,
            "5 segments, 15 rows, dirs [1, -1], lut LIVE 0x1a2b"
        );
        assert!(run(&Args { dir: root.clone() }).is_ok(), "one table");

        land_as(&store, 2, "slow", &live("0x1a2b"), &clean());
        let mut identity = sweep_meta();
        identity["plant"] = serde_json::json!({ "lut_state": "IDENTITY", "lut_crc": "0x0000" });
        land_as(&store, 2, "fast", &identity, &clean());
        assert!(
            run(&Args {
                dir: root.join("session/capture-1")
            })
            .is_ok()
        );
        let e = run(&Args {
            dir: root.join("session/capture-2"),
        })
        .unwrap_err()
        .to_string();
        assert_eq!(
            e,
            "recordings mix 2 pot tables; lut IDENTITY 0x0000: fast; lut LIVE 0x1a2b: slow"
        );
        assert!(
            run(&Args {
                dir: root.join("session")
            })
            .is_err()
        );
        assert!(run(&Args { dir: root.clone() }).is_err(), "the dataset dir");
        assert_eq!(capture_dirs(&root).unwrap().len(), 2);

        // a recording from before the plant record counts as its own kind
        land_as(&store, 3, "slow", &sweep_meta(), &clean());
        let mut tables = BTreeMap::new();
        tables.insert(None, vec!["session/capture-3/slow".to_string()]);
        tables.insert(
            Some(PlantMeta {
                lut_state: "LIVE".into(),
                lut_crc: "0x1a2b".into(),
            }),
            vec!["session/capture-1/slow".to_string()],
        );
        assert_eq!(
            mixed(&tables),
            "recordings mix 2 pot tables; no plant record: session/capture-3/slow; lut LIVE 0x1a2b: session/capture-1/slow"
        );
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
