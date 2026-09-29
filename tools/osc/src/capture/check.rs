//! `osc capture check`: re-read a landed capture and confirm every recording
//! holds every segment its own meta promises, without the servo; over a
//! dataset or experiment dir, every capture under it, and that they were
//! all made under one position table and one drive rule, the one
//! dataset.toml declares. A recording made under the limit is judged as the
//! capture verdict judged it: settled grid and ends tails, no current window
//! over its abort.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use flate2::read::GzDecoder;
use osc_ident::exp::{Applied, judge};
use serde::Deserialize;

use osc_ident::frame::TelFrame;
use osc_ident::limits::Ma;

use super::Rule;
use super::plan::Block;
use super::store::{goal_tick, tick_ms};
use super::verdict::{Abort, block_of, expected_segments, over_abort, settled};
use crate::rig::pump::BurstStats;
use crate::sweep::{Decay, Segment, Step};

/// `osc capture check` args.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// A capture dir `<dataset>/<experiment>/capture-N`, or a dataset or
    /// experiment dir holding captures.
    dir: PathBuf,
}

/// The sweep meta keys the segment count follows from, and the plant and
/// drive blocks the captures must agree on.
#[derive(Deserialize)]
struct SweepMeta {
    schedule: Vec<Step>,
    dirs: Vec<i8>,
    baseline_ms: u32,
    tick_hz: Option<f64>,
    #[serde(default)]
    plant: Option<PlantMeta>,
    #[serde(default)]
    drive: Option<DriveMeta>,
    #[serde(default)]
    vbus_counts: Option<f64>,
    #[serde(default)]
    decay: Option<Decay>,
    #[serde(default)]
    session: Option<SessionMeta>,
}

/// The block map a session recording carries.
#[derive(Deserialize)]
struct SessionMeta {
    blocks: Vec<Block>,
}

/// The table a recording was made under, as its meta names it.
#[derive(Deserialize, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct PlantMeta {
    lut_state: String,
    lut_crc: String,
}

/// The drive keys the check reads.
#[derive(Deserialize)]
struct DriveMeta {
    rule: Rule,
    current_limit_counts: u16,
    /// Per segment, in order; a bare `osc sweep` writes none.
    #[serde(default)]
    t_goal_ms: Option<Vec<Option<f64>>>,
    #[serde(default)]
    i_abort_counts: Option<f64>,
    #[serde(default)]
    window_floor_q15: Option<u16>,
    #[serde(default)]
    r_q12: Option<u16>,
}

/// The rule and limit a recording was made under, or a dataset declares.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Drive {
    rule: Rule,
    limit: Option<u16>,
}

impl Drive {
    /// A meta with no drive block was made before the servo limited
    /// open-loop current.
    fn of(m: &SweepMeta) -> Self {
        match &m.drive {
            Some(d) => Drive {
                rule: d.rule,
                limit: Some(d.current_limit_counts),
            },
            None => Drive {
                rule: Rule::Free,
                limit: None,
            },
        }
    }
}

impl fmt::Display for Drive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.rule.as_str())?;
        match self.limit {
            Some(l) => write!(f, " at {l} counts"),
            None => Ok(()),
        }
    }
}

/// dataset.toml's drive keys; one with no `rule` is `free`.
#[derive(Deserialize)]
struct DeclDrive {
    #[serde(default)]
    rule: Rule,
    #[serde(default)]
    current_limit_counts: Option<u16>,
}

/// What one recording checked as.
struct Checked {
    line: String,
    plant: Option<PlantMeta>,
    drive: Drive,
}

pub(crate) fn run(a: &Args) -> Result<()> {
    let dirs = capture_dirs(&a.dir)?;
    let declared = declared(&a.dir)?;
    let mut total = 0;
    let mut failed = 0;
    let mut tables: BTreeMap<Option<PlantMeta>, Vec<String>> = BTreeMap::new();
    let mut drives: BTreeMap<Drive, Vec<String>> = BTreeMap::new();
    for dir in &dirs {
        let names = recordings(dir)?;
        for name in &names {
            total += 1;
            let label = match dir.strip_prefix(&a.dir) {
                Ok(rel) if !rel.as_os_str().is_empty() => format!("{}/{name}", rel.display()),
                _ => name.clone(),
            };
            match check(dir, name) {
                Ok(c) => {
                    println!("{label}: {}", c.line);
                    tables.entry(c.plant).or_default().push(label.clone());
                    drives.entry(c.drive).or_default().push(label);
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
    let mut problems = Vec::new();
    if tables.len() > 1 {
        problems.push(mixed(&tables));
    }
    if drives.len() > 1 {
        problems.push(mixed_drives(&drives));
    }
    if let (Some(decl), [only]) = (declared, drives.keys().collect::<Vec<_>>().as_slice())
        && decl != **only
    {
        problems.push(format!(
            "dataset.toml declares {decl}, the recordings were made {only}"
        ));
    }
    for p in &problems {
        println!("FAIL {p}");
    }
    if failed > 0 {
        bail!("{failed} of {total} recordings failed");
    }
    if !problems.is_empty() {
        bail!("{}", problems.join("\n"));
    }
    let drive = drives
        .keys()
        .next()
        .map_or(String::new(), |d| format!(", {d}"));
    println!("ok: {total} recordings complete{drive}");
    Ok(())
}

/// The drive dataset.toml declares, from the dataset dir `dir` is or sits
/// in (a capture dir is two levels down); None where there is none.
fn declared(dir: &Path) -> Result<Option<Drive>> {
    for d in dir.ancestors().take(3) {
        let path = d.join("dataset.toml");
        if path.is_file() {
            let text = std::fs::read_to_string(&path)
                .with_context(|| format!("read {}", path.display()))?;
            let decl: DeclDrive =
                toml::from_str(&text).with_context(|| format!("parse {}", path.display()))?;
            return Ok(Some(Drive {
                rule: decl.rule,
                limit: decl.current_limit_counts,
            }));
        }
    }
    Ok(None)
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
        "recordings mix {} position tables; {}",
        tables.len(),
        groups.join("; ")
    )
}

/// The mix, one drive per line with the recordings made under it.
fn mixed_drives(drives: &BTreeMap<Drive, Vec<String>>) -> String {
    let groups: Vec<String> = drives
        .iter()
        .map(|(d, names)| format!("{d}: {}", names.join(" ")))
        .collect();
    format!(
        "recordings mix {} drive rules; {}",
        drives.len(),
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

fn check(dir: &Path, name: &str) -> Result<Checked> {
    let mut meta_path = dir.join(format!("{name}.meta.json"));
    if !meta_path.is_file() && dir.join("meta.json").is_file() {
        // a bare `osc sweep` recording: `meta.json` beside `sweep.csv.gz`
        meta_path = dir.join("meta.json");
    }
    let text = std::fs::read_to_string(&meta_path)
        .with_context(|| format!("read {}", meta_path.display()))?;
    let m: SweepMeta =
        serde_json::from_str(&text).with_context(|| format!("parse {}", meta_path.display()))?;
    let csv_path = dir.join(format!("{name}.csv.gz"));
    let f =
        std::fs::File::open(&csv_path).with_context(|| format!("open {}", csv_path.display()))?;
    let rows = read_rows(BufReader::new(GzDecoder::new(f)))
        .with_context(|| format!("read {}", csv_path.display()))?;

    let baseline = m.baseline_ms > 0;
    let want = expected_segments(baseline, m.dirs.len(), m.schedule.len());
    let expected: BTreeSet<u32> = (u32::from(!baseline)..).take(want).collect();
    let missing: Vec<u32> = expected
        .iter()
        .filter(|s| !rows.segs.contains_key(s))
        .copied()
        .collect();
    if !missing.is_empty() {
        bail!(
            "{} of {want} segments, missing seg {missing:?}",
            want - missing.len()
        );
    }
    let extra: Vec<u32> = rows
        .segs
        .keys()
        .filter(|s| !expected.contains(s))
        .copied()
        .collect();
    if !extra.is_empty() {
        bail!("seg {extra:?} beyond the {want} the schedule promises");
    }
    if let Some(d) = m.dirs.iter().find(|d| !rows.dirs.contains(d)) {
        bail!("no rows drive dir {d:+}");
    }
    let drive = Drive::of(&m);
    if let Some(t) = m.drive.as_ref().and_then(|d| d.t_goal_ms.as_deref()) {
        check_goals(&rows, t, m.tick_hz)?;
    }
    if let Some(d) = m.drive.as_ref().filter(|_| drive.rule == Rule::Limit) {
        check_limit(&rows, &m, d)?;
    }
    if drive.rule == Rule::Free {
        check_free(&rows, &m.schedule, baseline)?;
    }
    let total: usize = rows.segs.values().map(|s| s.rows).sum();
    let plant = match &m.plant {
        Some(p) => format!(", lut {} {}", p.lut_state, p.lut_crc),
        None => String::new(),
    };
    let under = match m.drive {
        Some(_) => format!(", {drive}"),
        None => String::new(),
    };
    Ok(Checked {
        line: format!(
            "{want} segments, {total} rows, dirs {:?}{plant}{under}",
            m.dirs
        ),
        plant: m.plant,
        drive,
    })
}

/// Every segment's `t_goal_ms` is the first tick its rows hold the goal.
fn check_goals(rows: &Rows, meta: &[Option<f64>], tick_hz: Option<f64>) -> Result<()> {
    let Some(hz) = tick_hz.filter(|&hz| hz > 0.0) else {
        bail!("t_goal_ms without a tick_hz");
    };
    if meta.len() != rows.segs.len() {
        bail!(
            "t_goal_ms names {} segments, the rows hold {}",
            meta.len(),
            rows.segs.len()
        );
    }
    for ((seg, s), &have) in rows.segs.iter().zip(meta) {
        let want =
            goal_tick(s.duty.iter().map(|&(t, d)| (t, Some(d))), s.cmd).map(|t| tick_ms(t, hz));
        let same = match (have, want) {
            (Some(a), Some(b)) => (a - b).abs() < 1e-6,
            (None, None) => true,
            _ => false,
        };
        if !same {
            let ms = |v: Option<f64>| v.map_or("null".to_string(), |v| format!("{v} ms"));
            bail!(
                "seg {seg}: t_goal_ms {} in the meta, {} by the rows",
                ms(have),
                ms(want)
            );
        }
    }
    Ok(())
}

/// A recording made under the limit, judged as the capture verdict judged
/// it: no window of current over the abort its meta names, and every grid
/// and ends rung's tail settled at its goal.
fn check_limit(rows: &Rows, m: &SweepMeta, d: &DriveMeta) -> Result<()> {
    let Some(hz) = m.tick_hz.filter(|&hz| hz > 0.0) else {
        bail!("a limit recording without a tick_hz");
    };
    let segs: Vec<Segment> = rows
        .segs
        .iter()
        .map(|(&seg, s)| Segment {
            seg,
            dir: s.dir,
            cmd_duty_q15: s.cmd,
            frames: s.frames.clone(),
            stats: BurstStats {
                frames: 0,
                samples: s.rows,
                holes: 0,
                garble: 0,
            },
        })
        .collect();
    let baseline = m.baseline_ms > 0;
    if let (Some(i_abort), Some(floor_q15), Some(r_q12), Some(rail)) =
        (d.i_abort_counts, d.window_floor_q15, d.r_q12, m.vbus_counts)
    {
        let abort = Abort {
            i_abort,
            floor_q15,
            rail,
            r_vpc: r_q12 as f64 / 4096.0,
            fast: m.decay == Some(Decay::Fast),
            tick_hz: hz,
            ma: Ma(0.0),
        };
        over_abort(&segs, &m.schedule, baseline, &abort).map_err(|e| anyhow!(e))?;
    }
    let Some(session) = &m.session else {
        return Ok(());
    };
    for (i, g) in segs.iter().enumerate() {
        let Some(k) = i.checked_sub(baseline as usize) else {
            continue;
        };
        let k = k % m.schedule.len();
        if g.cmd_duty_q15 != 0
            && matches!(block_of(&session.blocks, k), Some("grid" | "ends"))
            && matches!(m.schedule[k], Step::Drive(..))
        {
            settled(g, hz).map_err(|e| anyhow!("seg {}: {e}", g.seg))?;
        }
    }
    Ok(())
}

/// A recording made before the servo limited open-loop current holds no
/// sample the limit governed; one that does was made under a limit its
/// meta does not declare. Each segment is judged as the capture verdict
/// judges it: a drive slews from rest, a chained one from the duty the
/// segment before it left applied.
fn check_free(rows: &Rows, schedule: &[Step], baseline: bool) -> Result<()> {
    let mut last = 0;
    for (i, (seg, s)) in rows.segs.iter().enumerate() {
        let step = i
            .checked_sub(baseline as usize)
            .map(|k| schedule[k % schedule.len()]);
        let start = match step {
            Some(Step::Drive(..)) | None => 0,
            Some(_) if i == 0 => 0,
            Some(_) => last,
        };
        last = s.duty.last().map_or(0, |&(_, d)| d);
        if s.cmd == 0 {
            continue;
        }
        let governed = judge(s.duty.iter().copied(), s.cmd, start)
            .iter()
            .filter(|&&a| a == Applied::Governed)
            .count();
        if governed > 0 {
            bail!(
                "seg {seg}: {governed} samples held under the commanded duty in a free \
                 recording: it was made under a current limit its meta does not declare"
            );
        }
    }
    Ok(())
}

/// One segment's rows: the commanded duty, every applied-duty sample, and
/// the samples a limit recording is judged by.
#[derive(Default)]
struct SegRows {
    rows: usize,
    cmd: i16,
    dir: i8,
    duty: Vec<(u64, i16)>,
    frames: Vec<TelFrame>,
}

/// A recording's rows by seg, and every dir seen.
struct Rows {
    segs: BTreeMap<u32, SegRows>,
    dirs: BTreeSet<i8>,
}

/// The header names the columns; an empty duty cell is a sample TEL did
/// not stream.
fn read_rows(r: impl BufRead) -> Result<Rows> {
    let mut lines = r.lines();
    let header = lines.next().ok_or_else(|| anyhow!("empty csv"))??;
    let col = |name| {
        header
            .split(',')
            .position(|c| c == name)
            .ok_or_else(|| anyhow!("no {name} column"))
    };
    let (seg_col, dir_col) = (col("seg")?, col("dir")?);
    let (cmd_col, tick_col, duty_col) = (col("cmd_duty_q15")?, col("tick")?, col("duty_q15")?);
    let (pos_col, raw_col, trough_col) = (
        col("pos").ok(),
        col("current_raw").ok(),
        col("current_trough").ok(),
    );
    let mut segs: BTreeMap<u32, SegRows> = BTreeMap::new();
    let mut dirs = BTreeSet::new();
    for (i, line) in lines.enumerate() {
        let line = line?;
        let cells: Vec<&str> = line.split(',').collect();
        let field = |c: usize| cells.get(c).copied().unwrap_or_default();
        let bad = |what: &str| anyhow!("row {}: bad {what} in {line:?}", i + 2);
        let seg: u32 = field(seg_col).parse().map_err(|_| bad("seg"))?;
        let dir: i8 = field(dir_col).parse().map_err(|_| bad("dir"))?;
        let cmd: i16 = field(cmd_col).parse().map_err(|_| bad("cmd_duty_q15"))?;
        let tick: u64 = field(tick_col).parse().map_err(|_| bad("tick"))?;
        let s = segs.entry(seg).or_insert_with(|| SegRows {
            cmd,
            dir,
            ..SegRows::default()
        });
        s.rows += 1;
        let duty = match field(duty_col) {
            "" => None,
            d => Some(d.parse().map_err(|_| bad("duty_q15"))?),
        };
        if let Some(d) = duty {
            s.duty.push((tick, d));
        }
        let cell = |c: Option<usize>| c.and_then(|c| field(c).parse::<u16>().ok());
        s.frames.push(TelFrame {
            tick,
            duty_q15: duty,
            pos: cell(pos_col),
            current_raw: cell(raw_col),
            current_trough: cell(trough_col),
            ..TelFrame::default()
        });
        if seg > 0 {
            dirs.insert(dir);
        }
    }
    Ok(Rows { segs, dirs })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::plan::Plan;
    use crate::capture::store::fixture::{clean, land, land_as, seg, sweep_meta, tmp};
    use crate::capture::store::{Capture, CaptureMeta, Store};
    use crate::capture::verdict::bench;
    use crate::rig::servo::bench::Bench;
    use crate::sweep::Dirs;
    use serde_json::{Value, json};

    #[test]
    fn a_landed_capture_checks_clean() {
        let root = tmp("check-clean");
        let dir = land(&Store::new(root.clone()), &clean());
        assert_eq!(
            check(&dir, "slow").unwrap().line,
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
        let root = tmp("check-mix");
        let store = Store::new(root.clone());
        let live = |crc: &str| {
            let mut m = sweep_meta();
            m["plant"] = json!({ "lut_state": "LIVE", "lut_crc": crc });
            m
        };
        land_as(&store, 1, "slow", &live("0x1a2b"), &clean());
        land_as(&store, 1, "fast", &live("0x1a2b"), &clean());
        assert_eq!(
            check(&root.join("session/capture-1"), "slow").unwrap().line,
            "5 segments, 15 rows, dirs [1, -1], lut LIVE 0x1a2b"
        );
        assert!(run(&Args { dir: root.clone() }).is_ok(), "one table");

        land_as(&store, 2, "slow", &live("0x1a2b"), &clean());
        let mut identity = sweep_meta();
        identity["plant"] = json!({ "lut_state": "IDENTITY", "lut_crc": "0x0000" });
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
            "recordings mix 2 position tables; lut IDENTITY 0x0000: fast; lut LIVE 0x1a2b: slow"
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
            "recordings mix 2 position tables; no plant record: session/capture-3/slow; lut LIVE 0x1a2b: session/capture-1/slow"
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn short_or_one_sided_recordings_fail() {
        let root = tmp("check-short");
        let mut segs = clean();
        segs.pop();
        let dir = land(&Store::new(root.clone()), &segs);
        let e = check(&dir, "slow").err().unwrap().to_string();
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
            check(&dir, "slow").err().unwrap().to_string(),
            "no rows drive dir -1"
        );

        // an empty segment writes no rows, so it reads as missing
        let mut segs = clean();
        segs[2] = seg(2, 1, 0);
        let dir = land(&Store::new(root.clone()), &segs);
        assert!(
            check(&dir, "slow")
                .err()
                .unwrap()
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

    /// A bare `osc sweep` recording keeps `meta.json` beside `sweep.csv.gz`.
    #[test]
    fn a_bare_sweep_reads_its_meta_json() {
        let root = tmp("check-bare");
        let dir = land(&Store::new(root.clone()), &clean());
        std::fs::rename(dir.join("slow.meta.json"), dir.join("meta.json")).unwrap();
        std::fs::rename(dir.join("slow.csv.gz"), dir.join("sweep.csv.gz")).unwrap();
        assert_eq!(
            check(&dir, "sweep").unwrap().line,
            "5 segments, 15 rows, dirs [1, -1]"
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn rows_read_by_the_header() {
        let csv = "dir,tick,x,duty_q15,cmd_duty_q15,seg\n\
                   0,0,a,0,0,0\n\
                   1,0,b,,100,1\n\
                   1,1,c,100,100,1\n\
                   -1,0,d,-5,-100,2\n";
        let rows = read_rows(csv.as_bytes()).unwrap();
        let counts: Vec<(u32, usize)> = rows.segs.iter().map(|(k, s)| (*k, s.rows)).collect();
        assert_eq!(counts, [(0, 1), (1, 2), (2, 1)]);
        assert_eq!(rows.dirs, [1, -1].into());
        assert_eq!(rows.segs[&1].cmd, 100);
        assert_eq!(rows.segs[&1].duty, [(1, 100)]);
        assert_eq!(rows.segs[&2].duty, [(0, -5)]);
        let bad = "seg,dir,tick,cmd_duty_q15,duty_q15\nx,1,0,0,0\n";
        assert!(read_rows(bad.as_bytes()).is_err());
        assert!(read_rows("tick\n1\n".as_bytes()).is_err());
    }

    /// A drive rung of `goal` whose applied duty climbs `rate` per tick
    /// from 0 and is held at `held`.
    fn rung(seg_n: u32, dir: i8, goal: i16, held: i16, rate: i16, n: u64) -> Segment {
        let mut s = seg(seg_n, dir, n as usize);
        s.cmd_duty_q15 = goal;
        for (t, f) in s.frames.iter_mut().enumerate() {
            let climb = (rate as i64 * t as i64).min(held.unsigned_abs() as i64) as i16;
            f.duty_q15 = Some(climb * held.signum());
        }
        s
    }

    /// Baseline, then the fixture plan's `20@80`, `coast:400` both ways.
    fn recording(fwd_held: i16) -> Vec<Segment> {
        let coast = |seg_n: u32, dir: i8| {
            let mut s = seg(seg_n, dir, 3);
            for f in &mut s.frames {
                f.duty_q15 = Some(0);
            }
            s
        };
        let mut base = seg(0, 0, 3);
        base.frames.iter_mut().for_each(|f| f.duty_q15 = Some(0));
        vec![
            base,
            rung(1, 1, 6553, fwd_held, 3000, 100),
            coast(2, 1),
            rung(3, -1, -6553, -6553, 3000, 100),
            coast(4, -1),
        ]
    }

    fn limit_meta(limit: u16) -> Value {
        let mut m = sweep_meta();
        m["drive"] = json!({ "rule": "limit", "current_limit_counts": limit });
        m
    }

    fn decl(root: &Path, body: &str) {
        std::fs::write(root.join("dataset.toml"), body).unwrap();
    }

    #[test]
    fn a_recording_without_a_drive_block_is_free() {
        let root = tmp("check-free");
        let dir = land(&Store::new(root.clone()), &recording(6553));
        let c = check(&dir, "slow").unwrap();
        assert_eq!(
            c.drive,
            Drive {
                rule: Rule::Free,
                limit: None
            }
        );
        assert_eq!(c.line, "5 segments, 209 rows, dirs [1, -1]");
        // a dataset.toml with no rule declares free too
        decl(
            &root,
            "servo = \"mg90-a\"\nsupply = \"2s\"\ncaptured = \"x\"\n",
        );
        assert_eq!(declared(&dir).unwrap(), Some(c.drive));
        assert!(run(&Args { dir: root.clone() }).is_ok());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_dataset_never_mixes_drive_rules() {
        let root = tmp("check-rules");
        let store = Store::new(root.clone());
        land_as(&store, 1, "slow", &limit_meta(280), &recording(6553));
        let c = check(&root.join("session/capture-1"), "slow").unwrap();
        assert_eq!(
            c.line,
            "5 segments, 209 rows, dirs [1, -1], limit at 280 counts"
        );
        // the store stamps each segment's goal, and the check reads it back
        let text = std::fs::read_to_string(root.join("session/capture-1/slow.meta.json")).unwrap();
        let meta: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(
            meta["drive"]["t_goal_ms"],
            json!([0.0, 0.15, 0.0, 0.15, 0.0])
        );
        decl(
            &root,
            "servo = \"mg90-a\"\nsupply = \"2s\"\nrule = \"limit\"\ncurrent_limit_counts = 280\ncaptured = \"x\"\n",
        );
        assert!(run(&Args { dir: root.clone() }).is_ok());

        land_as(&store, 2, "slow", &sweep_meta(), &recording(6553));
        assert_eq!(
            run(&Args { dir: root.clone() }).unwrap_err().to_string(),
            "recordings mix 2 drive rules; free: session/capture-2/slow; limit at 280 counts: \
             session/capture-1/slow"
        );
        std::fs::remove_dir_all(root.join("session/capture-2")).unwrap();

        land_as(&store, 2, "slow", &limit_meta(300), &recording(6553));
        assert!(
            run(&Args { dir: root.clone() })
                .unwrap_err()
                .to_string()
                .starts_with("recordings mix 2 drive rules")
        );
        std::fs::remove_dir_all(root.join("session/capture-2")).unwrap();

        // one rule across the recordings, another in dataset.toml
        decl(
            &root,
            "servo = \"mg90-a\"\nsupply = \"2s\"\ncaptured = \"x\"\n",
        );
        assert_eq!(
            run(&Args { dir: root.clone() }).unwrap_err().to_string(),
            "dataset.toml declares free, the recordings were made limit at 280 counts"
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn governed_samples_in_a_free_recording_fail_the_check() {
        let root = tmp("check-governed");
        let store = Store::new(root.clone());
        // held at 15% under a 20% goal: the limit governed it
        let held = recording(4915);
        let dir = land_as(&store, 1, "slow", &sweep_meta(), &held);
        let e = check(&dir, "slow").err().unwrap().to_string();
        assert!(
            e.starts_with("seg 1: 46 samples held under the commanded duty in a free recording"),
            "{e}"
        );
        assert!(run(&Args { dir: root.clone() }).is_err());

        // the same rows under a declared limit are what the rule predicts
        let dir = land_as(&store, 1, "slow", &limit_meta(280), &held);
        assert!(check(&dir, "slow").is_ok());
        let text = std::fs::read_to_string(dir.join("slow.meta.json")).unwrap();
        assert_eq!(
            serde_json::from_str::<Value>(&text).unwrap()["drive"]["t_goal_ms"][1],
            Value::Null
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_t_goal_the_rows_disagree_with_fails() {
        let root = tmp("check-tgoal");
        let dir = land_as(
            &Store::new(root.clone()),
            1,
            "slow",
            &limit_meta(280),
            &recording(6553),
        );
        let path = dir.join("slow.meta.json");
        let mut meta: Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        meta["drive"]["t_goal_ms"][1] = json!(0.5);
        std::fs::write(&path, meta.to_string()).unwrap();
        assert_eq!(
            check(&dir, "slow").err().unwrap().to_string(),
            "seg 1: t_goal_ms 0.5 ms in the meta, 0.15 ms by the rows"
        );
        std::fs::remove_dir_all(&root).unwrap();
    }

    /// A grid rung recorded on the bench servo, landed as a limit recording
    /// with the settings its meta names.
    fn land_grid(root: &Path, n: u32, step: Step, set: impl FnOnce(&mut Bench)) -> PathBuf {
        let c = bench::cfg(vec![step], Dirs::Both);
        let segs = bench::record(&c, set);
        let mut meta = sweep_meta();
        meta["schedule"] = json!([step.to_string()]);
        meta["baseline_ms"] = json!(c.baseline_ms);
        meta["decay"] = json!("slow");
        meta["vbus_counts"] = json!(3204);
        meta["drive"] = json!({
            "rule": "limit",
            "current_limit_counts": 280,
            "i_abort_counts": 350,
            "window_floor_q15": 4356,
            "r_q12": 7270,
        });
        let store = Store::new(root.to_path_buf());
        let cap = Capture::open(&store, "session", n).unwrap();
        let mut t = cap.begin("slow", &meta).unwrap();
        for s in &segs.segments {
            t.on_seg(s).unwrap();
        }
        let plan = Plan {
            recording: "slow".into(),
            decay: Decay::Slow,
            schedule: vec![step],
            blocks: bench::one_block("grid", &c),
            dropped: Vec::new(),
        };
        t.accept(&CaptureMeta {
            supply: crate::capture::Supply::TwoS,
            plan: &plan,
            attempt: 1,
            governed: &[],
        })
        .unwrap();
        cap.dir().to_path_buf()
    }

    /// A limit recording read back is judged as the capture verdict judged
    /// it: a grid rung whose tail held its goal checks clean; one whose
    /// window ended 48 ms past its goal, or whose servo held no limit,
    /// fails on the rows alone.
    #[test]
    fn a_limit_recording_checks_its_tails_and_its_current() {
        let root = tmp("check-limit");
        let dir = land_grid(&root, 1, Step::Drive(40, Some(361)), |_| {});
        let c = check(&dir, "slow").unwrap();
        assert!(c.line.ends_with(", limit at 280 counts"), "{}", c.line);

        let dir = land_grid(&root, 2, Step::Drive(40, Some(120)), |_| {});
        let e = check(&dir, "slow").err().unwrap().to_string();
        assert!(
            e.starts_with("seg 1: the applied duty held its goal for the last 4"),
            "{e}"
        );

        let dir = land_grid(&root, 3, Step::Drive(60, Some(272)), |b| {
            b.servo.current_limit = None
        });
        let e = check(&dir, "slow").err().unwrap().to_string();
        assert!(
            e.starts_with(
                "the servo is not holding its current limit: seg 1 drew a 16-sample mean of"
            ),
            "{e}"
        );
        assert!(e.ends_with("over the abort of 350 counts"), "{e}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn goal_tick_skips_a_frame_with_no_duty() {
        let f = TelFrame {
            tick: 3,
            duty_q15: None,
            ..TelFrame::default()
        };
        assert_eq!(goal_tick([(f.tick, f.duty_q15), (4, Some(7))], 7), Some(4));
    }
}
