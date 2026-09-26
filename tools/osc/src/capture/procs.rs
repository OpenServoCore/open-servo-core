//! `session.toml`: the capture session procedure, compiled in and replaced
//! whole by an operator copy in the config dir. TOML because an operator
//! edits it.

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use super::Supply;
use crate::sweep::{Decay, Step};

const DEFAULT: &str = include_str!("session.toml");

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct Procedure {
    pub(crate) captures: u32,
    pub(crate) warmup: bool,
    pub(crate) cooldown_s: u32,
    pub(crate) rest_ms: u32,
    pub(crate) baseline_ms: u32,
    pub(crate) seek_pct: u8,
    pub(crate) tel_mask: u16,
    pub(crate) rotate: bool,
    #[serde(default)]
    pub(crate) supply: BTreeMap<Supply, SupplyCfg>,
    pub(crate) recording: Vec<RecordingCfg>,
    pub(crate) block: Blocks,
}

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct SupplyCfg {
    pub(crate) cells: u32,
    pub(crate) cell_floor_mv: u32,
    pub(crate) cell_warn_mv: u32,
}

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct RecordingCfg {
    pub(crate) name: String,
    pub(crate) decay: Decay,
    pub(crate) blocks: Vec<String>,
}

/// The block kinds, by the names session.py reads them back under.
#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct Blocks {
    pub(crate) grid: Grid,
    pub(crate) coast: CoastBlock,
    pub(crate) step: Steps,
    pub(crate) reversal: Steps,
    pub(crate) breakaway: Ladder,
}

impl Blocks {
    pub(crate) const NAMES: [&str; 5] = ["grid", "coast", "step", "reversal", "breakaway"];
}

/// One rung per duty at the envelope's window for it.
#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct Grid {
    pub(crate) duties: Vec<u8>,
}

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct CoastBlock {
    pub(crate) duties: Vec<u8>,
    pub(crate) drive_ms: u32,
    pub(crate) coast_ms: u32,
}

#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct Steps {
    pub(crate) steps: Vec<Step>,
}

/// One rung per duty, all at one window.
#[derive(Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct Ladder {
    pub(crate) duties: Vec<u8>,
    pub(crate) window_ms: u32,
}

impl Procedure {
    /// The operator's session.toml when it exists, else the compiled default,
    /// with where it came from.
    pub(crate) fn load() -> Result<(Self, String)> {
        let user = config_path(
            std::env::var_os("XDG_CONFIG_HOME"),
            std::env::var_os("HOME"),
        );
        load_from(user.as_deref())
    }

    pub(crate) fn parse(text: &str) -> Result<Self> {
        let p: Self = toml::from_str(text)?;
        p.validate()?;
        Ok(p)
    }

    fn validate(&self) -> Result<()> {
        if self.captures == 0 {
            bail!("captures must be at least 1");
        }
        if self.recording.is_empty() {
            bail!("no [[recording]]");
        }
        let mut names = BTreeSet::new();
        for r in &self.recording {
            if !names.insert(&r.name) {
                bail!("recording {:?} appears twice", r.name);
            }
            if r.blocks.is_empty() {
                bail!("recording {:?} has no blocks", r.name);
            }
            let mut seen = BTreeSet::new();
            for b in &r.blocks {
                if !Blocks::NAMES.contains(&b.as_str()) {
                    bail!(
                        "recording {:?}: no block {b:?}; blocks are {:?}",
                        r.name,
                        Blocks::NAMES
                    );
                }
                if !seen.insert(b) {
                    bail!("recording {:?} lists block {b:?} twice", r.name);
                }
            }
        }
        Ok(())
    }
}

fn load_from(user: Option<&Path>) -> Result<(Procedure, String)> {
    let (text, source) = match user.filter(|p| p.exists()) {
        Some(p) => (
            std::fs::read_to_string(p).with_context(|| format!("read {}", p.display()))?,
            p.display().to_string(),
        ),
        None => (DEFAULT.to_string(), "compiled default".to_string()),
    };
    let p = Procedure::parse(&text).with_context(|| format!("session procedure {source}"))?;
    Ok((p, source))
}

fn config_path(xdg: Option<OsString>, home: Option<OsString>) -> Option<PathBuf> {
    let base = match xdg.filter(|v| !v.is_empty()) {
        Some(x) => PathBuf::from(x),
        None => PathBuf::from(home?).join(".config"),
    };
    Some(base.join("osc").join("capture").join("session.toml"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compiled_default_parses() {
        let p = Procedure::parse(DEFAULT).unwrap();
        assert_eq!(p.captures, 5);
        assert_eq!(p.tel_mask, 0x1cd);
        let recs: Vec<(&str, Decay, Vec<&str>)> = p
            .recording
            .iter()
            .map(|r| {
                let b = r.blocks.iter().map(String::as_str).collect();
                (r.name.as_str(), r.decay, b)
            })
            .collect();
        assert_eq!(
            recs,
            [
                ("slow", Decay::Slow, Blocks::NAMES.to_vec()),
                ("fast", Decay::Fast, vec!["grid"]),
            ]
        );
        let two_s = &p.supply[&Supply::TwoS];
        assert_eq!((two_s.cells, two_s.cell_floor_mv), (2, 3500));
        assert!(!p.supply.contains_key(&Supply::Usb));
        assert_eq!(p.block.step.steps[0], Step::Drive(30, Some(20)));
        assert!(DEFAULT.is_ascii());
    }

    fn with_recordings(recs: &str) -> String {
        let head = DEFAULT.split("[[recording]]").next().unwrap();
        let tail = &DEFAULT[DEFAULT.find("[block.grid]").unwrap()..];
        format!("{head}{recs}\n{tail}")
    }

    #[test]
    fn validation_names_the_problem() {
        let err = |recs: &str| {
            Procedure::parse(&with_recordings(recs))
                .unwrap_err()
                .to_string()
        };
        let rec = |name: &str, blocks: &str| {
            format!("[[recording]]\nname = \"{name}\"\ndecay = \"slow\"\nblocks = [{blocks}]\n")
        };
        assert!(Procedure::parse(&with_recordings(&rec("slow", "\"grid\""))).is_ok());
        assert!(err(&rec("slow", "\"ramp\"")).contains("no block \"ramp\""));
        assert!(err(&rec("slow", "\"grid\", \"grid\"")).contains("twice"));
        assert!(err(&rec("slow", "")).contains("no blocks"));
        let twice = format!("{}{}", rec("slow", "\"grid\""), rec("slow", "\"coast\""));
        assert!(err(&twice).contains("appears twice"));
    }

    #[test]
    fn typos_and_bad_steps_are_refused() {
        let bad = DEFAULT.replace("rest_ms = 1500", "rest_msec = 1500");
        assert!(Procedure::parse(&bad).is_err());
        let bad = DEFAULT.replace("\"30@20\"", "\"30@\"");
        let e = format!("{:#}", Procedure::parse(&bad).unwrap_err());
        assert!(e.contains("bad ms"), "{e}");
    }

    #[test]
    fn user_file_replaces_the_default_whole() {
        let dir = std::env::temp_dir().join(format!("osc-procs-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("session.toml");
        std::fs::write(&path, DEFAULT.replace("captures = 5", "captures = 2")).unwrap();
        let (p, src) = load_from(Some(&path)).unwrap();
        assert_eq!((p.captures, src), (2, path.display().to_string()));
        std::fs::remove_dir_all(&dir).unwrap();
        let (p, src) = load_from(Some(&path)).unwrap();
        assert_eq!((p.captures, src.as_str()), (5, "compiled default"));
    }

    #[test]
    fn config_path_prefers_xdg() {
        let p =
            |x: Option<&str>, h: Option<&str>| config_path(x.map(Into::into), h.map(Into::into));
        assert_eq!(
            p(Some("/x"), Some("/h")),
            Some(PathBuf::from("/x/osc/capture/session.toml"))
        );
        assert_eq!(
            p(Some(""), Some("/h")),
            Some(PathBuf::from("/h/.config/osc/capture/session.toml"))
        );
        assert_eq!(p(None, None), None);
    }
}
