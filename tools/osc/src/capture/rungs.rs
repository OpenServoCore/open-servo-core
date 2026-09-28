//! The settled constant-duty rungs of a dataset: every driven segment that
//! opened on a still shaft and held one duty to the end of its window, the
//! input `osc lut build` stitches. Fast-decay recordings and stall ladders
//! never qualify. A session block map names the rungs (its `grid` block); a
//! bare `osc sweep` recording's are its drive steps that feed no chained
//! step. The current is the recording's own torque-off baseline removed;
//! the duty carries the direction's sign.

use std::collections::{BTreeMap, BTreeSet};
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use flate2::read::GzDecoder;
use serde::Deserialize;

use super::plan::Block;
use crate::sweep::{Decay, Step, feeds};

pub(crate) struct Rung {
    pub(crate) experiment: String,
    pub(crate) cap: u32,
    pub(crate) recording: String,
    pub(crate) seg: u32,
    pub(crate) duty_pct: i32,
    pub(crate) tick_hz: f64,
    pub(crate) pos: Vec<u16>,
    pub(crate) current: Vec<f64>,
}

/// The sweep meta keys the selection follows from.
#[derive(Deserialize)]
struct Meta {
    schedule: Vec<Step>,
    dirs: Vec<i8>,
    decay: Decay,
    tick_hz: f64,
    #[serde(default)]
    stall: bool,
    session: Option<Session>,
}

#[derive(Deserialize)]
struct Session {
    blocks: Vec<Block>,
}

/// Every rung of the dataset: experiments in name order, captures in
/// number order, each recording's segments in file order.
pub(crate) fn gather(dataset: &Path) -> Result<Vec<Rung>> {
    let mut out = Vec::new();
    for experiment in experiments(dataset)? {
        let name = experiment
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        for (cap, dir) in captures(&experiment)? {
            for (recording, meta, csv) in recordings(&dir)? {
                let text = std::fs::read_to_string(&meta)
                    .with_context(|| format!("read {}", meta.display()))?;
                let m: Meta = serde_json::from_str(&text)
                    .with_context(|| format!("parse {}", meta.display()))?;
                let keep = settled(&m);
                if keep.is_empty() {
                    continue;
                }
                let f =
                    std::fs::File::open(&csv).with_context(|| format!("open {}", csv.display()))?;
                let segs = read_segments(BufReader::new(GzDecoder::new(f)), &keep)
                    .with_context(|| format!("read {}", csv.display()))?;
                out.extend(segs.into_iter().map(|(seg, s)| Rung {
                    experiment: name.clone(),
                    cap,
                    recording: recording.clone(),
                    seg,
                    duty_pct: (s.q15 as f64 * 100.0 / i16::MAX as f64).round() as i32,
                    tick_hz: m.tick_hz,
                    pos: s.pos,
                    current: s.current,
                }));
            }
        }
    }
    Ok(out)
}

/// Subdirs holding at least one `capture-N`, in name order.
fn experiments(dataset: &Path) -> Result<Vec<PathBuf>> {
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(dataset)
        .with_context(|| format!("read {}", dataset.display()))?
        .map(|e| Ok(e?.path()))
        .collect::<Result<_>>()?;
    dirs.retain(|d| d.is_dir() && captures(d).is_ok_and(|c| !c.is_empty()));
    dirs.sort();
    Ok(dirs)
}

/// `(n, dir)` of every `capture-N`, in number order.
fn captures(experiment: &Path) -> Result<Vec<(u32, PathBuf)>> {
    let mut caps = Vec::new();
    for e in
        std::fs::read_dir(experiment).with_context(|| format!("read {}", experiment.display()))?
    {
        let e = e?;
        let file = e.file_name().to_string_lossy().into_owned();
        if let Some(n) = file.strip_prefix("capture-").and_then(|n| n.parse().ok())
            && e.path().is_dir()
        {
            caps.push((n, e.path()));
        }
    }
    caps.sort();
    Ok(caps)
}

/// `(name, meta, csv)` per recording: `<name>.meta.json` + `<name>.csv.gz`
/// as the store lands them, or a bare sweep's `meta.json` + `sweep.csv.gz`.
fn recordings(dir: &Path) -> Result<Vec<(String, PathBuf, PathBuf)>> {
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir).with_context(|| format!("read {}", dir.display()))? {
        let file = e?.file_name().to_string_lossy().into_owned();
        let (name, csv) = match file.strip_suffix(".meta.json") {
            Some(n) => (n.to_string(), format!("{n}.csv.gz")),
            None if file == "meta.json" => ("sweep".to_string(), "sweep.csv.gz".to_string()),
            None => continue,
        };
        let csv = dir.join(csv);
        if !csv.is_file() {
            bail!("{}: {name} has meta but no rows", dir.display());
        }
        out.push((name, dir.join(file), csv));
    }
    out.sort();
    Ok(out)
}

/// The seg numbers of a recording's settled rungs: the grid block's steps
/// when a session block map names one, else every drive step nothing
/// chains to; each once per direction, numbered as the sweep commits them.
fn settled(m: &Meta) -> BTreeSet<u32> {
    if m.stall || m.decay != Decay::Slow {
        return BTreeSet::new();
    }
    let n = m.schedule.len();
    let steps: Vec<usize> = match &m.session {
        Some(s) => s
            .blocks
            .iter()
            .find(|b| b.name == "grid")
            .map(|b| (b.first..b.first + b.count).collect())
            .unwrap_or_default(),
        None => (0..n)
            .filter(|&k| matches!(m.schedule[k], Step::Drive(..)) && !feeds(&m.schedule, k))
            .collect(),
    };
    (0..m.dirs.len())
        .flat_map(|d| steps.iter().map(move |&k| (1 + d * n + k) as u32))
        .collect()
}

struct Segment {
    q15: i32,
    pos: Vec<u16>,
    current: Vec<f64>,
}

/// The kept segments' pos and current with seg 0's mean current removed;
/// the header names the columns.
fn read_segments(r: impl BufRead, keep: &BTreeSet<u32>) -> Result<BTreeMap<u32, Segment>> {
    let mut lines = r.lines();
    let header = lines.next().ok_or_else(|| anyhow!("empty csv"))??;
    let col = |name| {
        header
            .split(',')
            .position(|c| c == name)
            .ok_or_else(|| anyhow!("no {name} column"))
    };
    let (seg_col, q15_col, pos_col, cur_col) = (
        col("seg")?,
        col("cmd_duty_q15")?,
        col("pos")?,
        col("current_raw")?,
    );
    let mut segs: BTreeMap<u32, Segment> = BTreeMap::new();
    let (mut base_sum, mut base_n) = (0.0, 0usize);
    for (i, line) in lines.enumerate() {
        let line = line?;
        let cells: Vec<&str> = line.split(',').collect();
        let field = |c: usize| cells.get(c).copied().unwrap_or_default();
        let bad = || anyhow!("row {}: bad seg/duty/pos/current in {line:?}", i + 2);
        let seg: u32 = field(seg_col).parse().map_err(|_| bad())?;
        if seg == 0 {
            base_sum += field(cur_col).parse::<f64>().map_err(|_| bad())?;
            base_n += 1;
            continue;
        }
        if !keep.contains(&seg) {
            continue;
        }
        let q15: i32 = field(q15_col).parse().map_err(|_| bad())?;
        let pos: u16 = field(pos_col).parse().map_err(|_| bad())?;
        let cur: f64 = field(cur_col).parse().map_err(|_| bad())?;
        let s = segs.entry(seg).or_insert_with(|| Segment {
            q15,
            pos: Vec::new(),
            current: Vec::new(),
        });
        if s.q15 != q15 {
            bail!("row {}: seg {seg} changes duty mid-rung", i + 2);
        }
        s.pos.push(pos);
        s.current.push(cur);
    }
    if base_n == 0 {
        bail!("no torque-off baseline (seg 0) to take the current bias from");
    }
    if let Some(seg) = keep.iter().find(|s| !segs.contains_key(s)) {
        bail!("seg {seg} has no rows");
    }
    let bias = base_sum / base_n as f64;
    for s in segs.values_mut() {
        for c in &mut s.current {
            *c -= bias;
        }
    }
    Ok(segs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn meta(v: serde_json::Value) -> Meta {
        serde_json::from_value(v).unwrap()
    }

    #[test]
    fn a_block_map_names_the_grid_and_a_bare_sweep_its_unchained_drives() {
        let session = meta(json!({
            "schedule": ["5@1500", "10@1500", "20@80", "coast:400", "30@20", "coast:400"],
            "dirs": [1, -1],
            "decay": "slow",
            "tick_hz": 20000,
            "session": { "blocks": [
                { "name": "coast", "first": 2, "count": 2 },
                { "name": "grid", "first": 0, "count": 2 },
                { "name": "step", "first": 4, "count": 2 }
            ] }
        }));
        assert_eq!(settled(&session), [1, 2, 7, 8].into());

        let bare = meta(json!({
            "schedule": ["15@1277", "20@897", "20@80", "coast:400", "20@60", "then:-20@60"],
            "dirs": [1, -1],
            "decay": "slow",
            "tick_hz": 20000
        }));
        assert_eq!(settled(&bare), [1, 2, 7, 8].into());

        let fast = meta(json!({
            "schedule": ["15@1277"], "dirs": [1], "decay": "fast", "tick_hz": 20000
        }));
        assert!(settled(&fast).is_empty());
        let stall = meta(json!({
            "schedule": ["12", "then:15"], "dirs": [1, -1], "decay": "slow",
            "tick_hz": 20000, "stall": true
        }));
        assert!(settled(&stall).is_empty());
        let no_grid = meta(json!({
            "schedule": ["30@20", "coast:400"], "dirs": [1], "decay": "slow",
            "tick_hz": 20000, "session": { "blocks": [{ "name": "step", "first": 0, "count": 2 }] }
        }));
        assert!(settled(&no_grid).is_empty());
    }

    #[test]
    fn segments_read_by_the_header_with_the_baseline_bias_removed() {
        let csv = "seg,cmd_duty_q15,dir,pos,current_raw\n\
                   0,0,0,2000,10\n0,0,0,2000,12\n\
                   1,4915,1,2000,20\n1,4915,1,2001,21\n\
                   2,9830,1,2002,30\n\
                   3,-4915,-1,2003,40\n";
        let segs = read_segments(csv.as_bytes(), &[1, 3].into()).unwrap();
        assert_eq!(segs.keys().copied().collect::<Vec<_>>(), [1, 3]);
        assert_eq!(segs[&1].pos, [2000, 2001]);
        assert_eq!(segs[&1].current, [9.0, 10.0]);
        assert_eq!(segs[&3].q15, -4915);
        assert_eq!(segs[&3].current, [29.0]);
        assert!(read_segments(csv.as_bytes(), &[4].into()).is_err());
        let no_base = "seg,cmd_duty_q15,dir,pos,current_raw\n1,4915,1,2000,20\n";
        assert!(read_segments(no_base.as_bytes(), &[1].into()).is_err());
        let flip = "seg,cmd_duty_q15,dir,pos,current_raw\n0,0,0,2000,10\n1,4915,1,2000,20\n1,-4915,1,2000,20\n";
        assert!(read_segments(flip.as_bytes(), &[1].into()).is_err());
        assert!(read_segments("tick\n1\n".as_bytes(), &[1].into()).is_err());
    }
}
