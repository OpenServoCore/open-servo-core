//! A dataset on disk, the layout notebooks/oscnb/datasets.py reads:
//!
//!   <dataset>/dataset.toml
//!   <dataset>/<experiment>/capture-N/<recording>.csv.gz
//!                                   /<recording>.meta.json
//!
//! A recording is written into `capture-N/.tmp-<recording>/` and renamed into
//! place only once accepted, so a capture dir never holds a partial one and a
//! capture whose every `.csv.gz` is present is complete.

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use flate2::Compression;
use flate2::write::GzEncoder;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use super::Supply;
use super::plan::Plan;
use crate::sweep::{self, CSV_HEADER, Segment};

const DECL: &str = "dataset.toml";

pub(crate) struct Store {
    dir: PathBuf,
}

impl Store {
    pub(crate) fn new(dir: PathBuf) -> Self {
        Self { dir }
    }

    fn capture_dir(&self, experiment: &str, n: u32) -> PathBuf {
        self.dir.join(experiment).join(format!("capture-{n}"))
    }

    /// Recording `name` of capture `n` landed; creates nothing.
    pub(crate) fn landed(&self, experiment: &str, n: u32, name: &str) -> bool {
        self.capture_dir(experiment, n)
            .join(format!("{name}.csv.gz"))
            .is_file()
    }
}

/// What dataset.toml declares: what the recordings cannot say about
/// themselves.
#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Decl {
    pub(crate) servo: String,
    pub(crate) supply: Supply,
    pub(crate) captured: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) notes: Option<String>,
}

impl Decl {
    /// Writes dataset.toml unless the dataset has one; true when written. An
    /// existing one is the operator's and is never touched.
    pub(crate) fn save(&self, s: &Store) -> Result<bool> {
        std::fs::create_dir_all(&s.dir).with_context(|| format!("mkdir {}", s.dir.display()))?;
        let path = s.dir.join(DECL);
        let body = toml::to_string(self).context("serialize dataset.toml")?;
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(mut f) => f
                .write_all(body.as_bytes())
                .with_context(|| format!("write {}", path.display()))
                .map(|()| true),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => Ok(false),
            Err(e) => Err(e).with_context(|| format!("create {}", path.display())),
        }
    }

    pub(crate) fn load(s: &Store) -> Result<Self> {
        let path = s.dir.join(DECL);
        let text =
            std::fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
        toml::from_str(&text).with_context(|| format!("parse {}", path.display()))
    }
}

/// One capture's dir. A warm-up capture's dir is removed on drop, so its
/// recordings never land.
pub(crate) struct Capture {
    dir: PathBuf,
    n: u32,
    warmup: bool,
}

impl Capture {
    pub(crate) fn open(s: &Store, experiment: &str, n: u32) -> Result<Self> {
        let dir = s.capture_dir(experiment, n);
        std::fs::create_dir_all(&dir).with_context(|| format!("mkdir {}", dir.display()))?;
        Ok(Self {
            dir,
            n,
            warmup: false,
        })
    }

    /// Scratch dir for a warm-up running capture `n`'s plan.
    pub(crate) fn warmup(s: &Store, experiment: &str, n: u32) -> Result<Self> {
        let dir = s.dir.join(experiment).join(".warmup");
        fresh_dir(&dir)?;
        Ok(Self {
            dir,
            n,
            warmup: true,
        })
    }

    pub(crate) fn dir(&self) -> &Path {
        &self.dir
    }

    /// Remove landed recording `name`, its `.csv.gz` (the done marker) first.
    pub(crate) fn discard(&self, name: &str) -> Result<()> {
        for f in [format!("{name}.csv.gz"), format!("{name}.meta.json")] {
            let path = self.dir.join(f);
            match std::fs::remove_file(&path) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => return Err(e).with_context(|| format!("rm {}", path.display())),
            }
        }
        Ok(())
    }

    /// Start one try of recording `name` with its sweep meta.
    pub(crate) fn begin(&self, name: &str, meta: &Value) -> Result<Try<'_>> {
        let tmp = self.dir.join(format!(".tmp-{name}"));
        fresh_dir(&tmp)?;
        let meta_path = tmp.join("meta.json");
        std::fs::write(&meta_path, serde_json::to_string_pretty(meta)?)
            .with_context(|| format!("write {}", meta_path.display()))?;
        let csv_path = tmp.join("sweep.csv");
        let mut csv = BufWriter::new(
            File::create(&csv_path).with_context(|| format!("create {}", csv_path.display()))?,
        );
        writeln!(csv, "{CSV_HEADER}")?;
        Ok(Try {
            cap: self,
            name: name.to_string(),
            tmp,
            csv,
        })
    }
}

impl Drop for Capture {
    fn drop(&mut self) {
        if self.warmup {
            let _ = std::fs::remove_dir_all(&self.dir);
        }
    }
}

/// What a capture adds to a recording's sweep meta.
pub(crate) struct CaptureMeta<'a> {
    pub(crate) supply: Supply,
    pub(crate) plan: &'a Plan,
    pub(crate) attempt: u32,
}

/// One attempt at a recording, in its tmp dir until accepted or rejected.
pub(crate) struct Try<'a> {
    cap: &'a Capture,
    name: String,
    tmp: PathBuf,
    csv: BufWriter<File>,
}

impl Try<'_> {
    /// `sweep::record`'s per-segment sink.
    pub(crate) fn on_seg(&mut self, s: &Segment) -> Result<()> {
        sweep::write_rows(&mut self.csv, s)
    }

    /// Land the recording: `<name>.csv.gz` and `<name>.meta.json`, the sweep
    /// meta plus supply, session block map and capture, keys sorted.
    pub(crate) fn accept(self, extra: &CaptureMeta) -> Result<()> {
        let Try {
            cap,
            name,
            tmp,
            csv,
        } = self;
        csv.into_inner()
            .map_err(|e| e.into_error())
            .context("flush sweep.csv")?;

        let meta_path = tmp.join("meta.json");
        let text = std::fs::read_to_string(&meta_path)
            .with_context(|| format!("read {}", meta_path.display()))?;
        let mut meta: Value = serde_json::from_str(&text).context("parse sweep meta")?;
        let plan = extra.plan;
        let sched: Vec<String> = plan.schedule.iter().map(|s| s.to_string()).collect();
        if meta.get("schedule") != Some(&json!(sched)) {
            bail!("{name}: the sweep's schedule is not the plan's");
        }
        let Some(obj) = meta.as_object_mut() else {
            bail!("{name}: sweep meta is not an object");
        };
        obj.insert("supply".into(), json!(extra.supply.as_str()));
        obj.insert("session".into(), json!({ "blocks": plan.blocks }));
        let order: Vec<&str> = plan.blocks.iter().map(|b| b.name.as_str()).collect();
        obj.insert(
            "capture".into(),
            json!({ "n": cap.n, "try": extra.attempt, "warmup": cap.warmup, "order": order }),
        );

        let (meta_name, gz_name) = (format!("{name}.meta.json"), format!("{name}.csv.gz"));
        gzip(&tmp.join("sweep.csv"), &tmp.join(&gz_name))?;
        let merged = tmp.join(&meta_name);
        std::fs::write(&merged, serde_json::to_string_pretty(&meta)?)
            .with_context(|| format!("write {}", merged.display()))?;
        // The csv.gz lands last: its presence is what marks the recording done.
        for f in [meta_name, gz_name] {
            let to = cap.dir.join(&f);
            std::fs::rename(tmp.join(&f), &to)
                .with_context(|| format!("rename to {}", to.display()))?;
        }
        std::fs::remove_dir_all(&tmp).with_context(|| format!("rm {}", tmp.display()))
    }

    pub(crate) fn reject(self) -> Result<()> {
        drop(self.csv);
        std::fs::remove_dir_all(&self.tmp).with_context(|| format!("rm {}", self.tmp.display()))
    }
}

/// An empty dir at `dir`, clearing whatever an interrupted run left there.
fn fresh_dir(dir: &Path) -> Result<()> {
    match std::fs::remove_dir_all(dir) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(e).with_context(|| format!("rm {}", dir.display())),
    }
    std::fs::create_dir_all(dir).with_context(|| format!("mkdir {}", dir.display()))
}

fn gzip(from: &Path, to: &Path) -> Result<()> {
    let mut src = File::open(from).with_context(|| format!("open {}", from.display()))?;
    let dst = File::create(to).with_context(|| format!("create {}", to.display()))?;
    let mut enc = GzEncoder::new(BufWriter::new(dst), Compression::default());
    std::io::copy(&mut src, &mut enc).with_context(|| format!("gzip {}", from.display()))?;
    enc.finish()?.flush()?;
    Ok(())
}

#[cfg(test)]
pub(super) mod fixture {
    use super::*;
    use crate::capture::plan::Block;
    use crate::rig::pump::BurstStats;
    use crate::sweep::{Decay, Step};
    use osc_ident::frame::TelFrame;

    /// A fresh empty dir under the system temp dir, per test.
    pub(crate) fn tmp(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("osc-store-{}-{tag}", std::process::id()));
        fresh_dir(&d).unwrap();
        d
    }

    pub(crate) fn plan() -> Plan {
        Plan {
            recording: "slow".into(),
            decay: Decay::Slow,
            schedule: vec![Step::Drive(20, Some(80)), Step::Coast(400)],
            blocks: vec![Block {
                name: "coast".into(),
                first: 0,
                count: 2,
            }],
        }
    }

    /// What `sweep::meta` writes, trimmed to the keys the checks read plus
    /// one they must carry through.
    pub(crate) fn sweep_meta() -> Value {
        json!({
            "schedule": ["20@80", "coast:400"],
            "dirs": [1, -1],
            "baseline_ms": 1000,
            "window_ms": 150,
            "tick_hz": 20000,
        })
    }

    pub(crate) fn seg(seg: u32, dir: i8, rows: usize) -> Segment {
        Segment {
            seg,
            dir,
            cmd_duty_q15: 0,
            frames: (0..rows)
                .map(|i| TelFrame {
                    tick: i as u64,
                    pos: Some(2000),
                    ..TelFrame::default()
                })
                .collect(),
            stats: BurstStats {
                frames: 1,
                samples: rows,
                holes: 0,
                garble: 0,
            },
        }
    }

    /// Baseline, then both directions of the two-step plan.
    pub(crate) fn clean() -> Vec<Segment> {
        vec![
            seg(0, 0, 3),
            seg(1, 1, 3),
            seg(2, 1, 3),
            seg(3, -1, 3),
            seg(4, -1, 3),
        ]
    }

    /// Record `segs` as capture 1's `slow` and accept it.
    pub(crate) fn land(store: &Store, segs: &[Segment]) -> PathBuf {
        let cap = Capture::open(store, "session", 1).unwrap();
        let mut t = cap.begin("slow", &sweep_meta()).unwrap();
        for s in segs {
            t.on_seg(s).unwrap();
        }
        let plan = plan();
        let extra = CaptureMeta {
            supply: Supply::TwoS,
            plan: &plan,
            attempt: 2,
        };
        t.accept(&extra).unwrap();
        cap.dir().to_path_buf()
    }
}

#[cfg(test)]
mod tests {
    use super::fixture::*;
    use super::*;

    fn listing(dir: &Path) -> Vec<String> {
        let mut v: Vec<String> = std::fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        v.sort();
        v
    }

    #[test]
    fn decl_is_written_once() {
        let root = tmp("decl");
        let store = Store::new(root.join("mg90-a__2s"));
        let decl = Decl {
            servo: "mg90-a".into(),
            supply: Supply::TwoS,
            captured: "2026-09-25".into(),
            notes: None,
        };
        assert!(decl.save(&store).unwrap());
        let text = std::fs::read_to_string(root.join("mg90-a__2s/dataset.toml")).unwrap();
        assert!(text.contains("supply = \"2s\"") && !text.contains("notes"));
        let other = Decl {
            servo: "sg90-a".into(),
            ..decl
        };
        assert!(!other.save(&store).unwrap());
        assert_eq!(Decl::load(&store).unwrap().servo, "mg90-a");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_try_lands_by_rename_and_leaves_no_tmp() {
        let root = tmp("land");
        let store = Store::new(root.clone());
        let cap = Capture::open(&store, "session", 1).unwrap();
        let mut t = cap.begin("slow", &sweep_meta()).unwrap();
        assert_eq!(listing(cap.dir()), [".tmp-slow"]);
        assert_eq!(
            listing(&cap.dir().join(".tmp-slow")),
            ["meta.json", "sweep.csv"]
        );
        for s in clean() {
            t.on_seg(&s).unwrap();
        }
        let plan = plan();
        let extra = CaptureMeta {
            supply: Supply::TwoS,
            plan: &plan,
            attempt: 2,
        };
        t.accept(&extra).unwrap();
        assert_eq!(listing(cap.dir()), ["slow.csv.gz", "slow.meta.json"]);
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn accepted_meta_keeps_the_sweep_keys_and_adds_three() {
        let root = tmp("meta");
        let dir = land(&Store::new(root.clone()), &clean());
        let text = std::fs::read_to_string(dir.join("slow.meta.json")).unwrap();
        let meta: Value = serde_json::from_str(&text).unwrap();
        let sweep = sweep_meta();
        for (k, v) in sweep.as_object().unwrap() {
            assert_eq!(&meta[k], v, "{k}");
        }
        assert_eq!(meta["supply"], "2s");
        assert_eq!(
            meta["session"],
            json!({ "blocks": [{ "name": "coast", "first": 0, "count": 2 }] })
        );
        assert_eq!(
            meta["capture"],
            json!({ "n": 1, "try": 2, "warmup": false, "order": ["coast"] })
        );
        assert_eq!(
            meta.as_object().unwrap().len(),
            sweep.as_object().unwrap().len() + 3
        );
        // sorted keys, as the notebooks' own writers left them
        let keys: Vec<usize> = ["\"baseline_ms\"", "\"capture\"", "\"dirs\"", "\"supply\""]
            .iter()
            .map(|k| text.find(k).unwrap())
            .collect();
        assert!(keys.is_sorted(), "{text}");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_schedule_other_than_the_plans_is_refused() {
        let root = tmp("sched");
        let store = Store::new(root.clone());
        let cap = Capture::open(&store, "session", 1).unwrap();
        let mut meta = sweep_meta();
        meta["schedule"] = json!(["20@80"]);
        let t = cap.begin("slow", &meta).unwrap();
        let plan = plan();
        let extra = CaptureMeta {
            supply: Supply::Usb,
            plan: &plan,
            attempt: 1,
        };
        assert!(t.accept(&extra).is_err());
        assert!(!store.landed("session", 1, "slow"));
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_rejected_try_leaves_nothing() {
        let root = tmp("reject");
        let store = Store::new(root.clone());
        let cap = Capture::open(&store, "session", 3).unwrap();
        let mut t = cap.begin("fast", &sweep_meta()).unwrap();
        t.on_seg(&seg(0, 0, 3)).unwrap();
        t.reject().unwrap();
        assert!(listing(cap.dir()).is_empty());
        assert!(cap.dir().ends_with("session/capture-3"));
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_warmup_never_lands() {
        let root = tmp("warmup");
        let store = Store::new(root.clone());
        {
            let cap = Capture::warmup(&store, "session", 1).unwrap();
            let t = cap.begin("slow", &sweep_meta()).unwrap();
            let plan = plan();
            let extra = CaptureMeta {
                supply: Supply::TwoS,
                plan: &plan,
                attempt: 1,
            };
            t.accept(&extra).unwrap();
            let meta = std::fs::read_to_string(cap.dir().join("slow.meta.json")).unwrap();
            assert!(meta.contains("\"warmup\": true"));
            assert!(cap.dir().ends_with("session/.warmup"));
        }
        assert!(listing(&root.join("session")).is_empty());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn landed_reads_the_marker_and_creates_nothing() {
        let root = tmp("landed");
        let store = Store::new(root.clone());
        land(&store, &clean());
        assert!(store.landed("session", 1, "slow"));
        assert!(!store.landed("session", 1, "fast"));
        assert!(!store.landed("session", 2, "slow"));
        assert!(!root.join("session/capture-2").exists());
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn discard_unlands_a_recording() {
        let root = tmp("discard");
        let store = Store::new(root.clone());
        let dir = land(&store, &clean());
        let cap = Capture::open(&store, "session", 1).unwrap();
        cap.discard("slow").unwrap();
        assert!(!store.landed("session", 1, "slow"));
        assert!(listing(&dir).is_empty());
        cap.discard("slow").unwrap();
        std::fs::remove_dir_all(&root).unwrap();
    }
}
