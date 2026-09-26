//! `envelope.toml`: what `osc capture pilot` measured on the servo and the
//! per-duty windows the campaign drives with. TOML because an operator reads
//! and edits it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};

use super::Supply;

const FILE: &str = "envelope.toml";

const HEADER: &str = "\
# osc capture pilot envelope. Positions in pot counts, speeds in counts/ms,
# times in ms. limits: soft/phys read from the servo, guard = soft inset,
# runway = guard_hi - guard_lo. v_ss: counts/ms = slope x duty_pct + intercept,
# `used` sizes the windows. windows_ms: duty_pct = window. coast: top_pct is
# the coast block's top duty; peak_inside_soft its closest approach to soft.
";

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Envelope {
    pub(crate) supply: Supply,
    pub(crate) fw: u16,
    pub(crate) git_sha: String,
    pub(crate) measured: String,
    pub(crate) seek_pct: u8,
    pub(crate) limits: Limits,
    pub(crate) v_ss: Speed,
    pub(crate) windows_ms: BTreeMap<u8, u32>,
    pub(crate) coast: Coast,
    pub(crate) verified: Vec<Verified>,
}

#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Limits {
    pub(crate) soft: [u16; 2],
    pub(crate) phys: [u16; 2],
    pub(crate) guard: [u16; 2],
    pub(crate) center: u16,
    pub(crate) runway: u16,
}

#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Dir {
    Fwd,
    Rev,
}

impl std::fmt::Display for Dir {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Dir::Fwd => "fwd",
            Dir::Rev => "rev",
        })
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Speed {
    pub(crate) duties: Vec<u8>,
    pub(crate) used: Dir,
    pub(crate) fwd: Fit,
    pub(crate) rev: Fit,
}

impl Speed {
    pub(crate) fn used(&self) -> &Fit {
        match self.used {
            Dir::Fwd => &self.fwd,
            Dir::Rev => &self.rev,
        }
    }
}

/// Steady-state speed in counts/ms, affine in duty percent.
#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Fit {
    pub(crate) slope: f64,
    pub(crate) intercept: f64,
    pub(crate) r2: f64,
}

impl Fit {
    /// Rounded to what the file keeps, so every number derived from a fit
    /// can be re-derived from the saved envelope.
    pub(crate) fn rounded(slope: f64, intercept: f64, r2: f64) -> Self {
        Self {
            slope: round_to(slope, 4),
            intercept: round_to(intercept, 3),
            r2: round_to(r2, 4),
        }
    }

    pub(crate) fn at(&self, pct: u8) -> f64 {
        self.slope * pct as f64 + self.intercept
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Coast {
    pub(crate) probe_pct: u8,
    pub(crate) probe_entry: f64,
    pub(crate) probe_travel: u16,
    pub(crate) top_pct: u8,
    pub(crate) peak_inside_soft: i32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Verified {
    pub(crate) step: String,
    pub(crate) travel: u16,
    pub(crate) predicted: u16,
}

impl Envelope {
    pub(crate) fn save(&self, dir: &Path) -> Result<PathBuf> {
        let path = dir.join(FILE);
        let body = toml::to_string(self).context("serialize envelope")?;
        std::fs::write(&path, format!("{HEADER}{body}"))
            .with_context(|| format!("write {}", path.display()))?;
        Ok(path)
    }

    pub(crate) fn load(dir: &Path) -> Result<Self> {
        let path = dir.join(FILE);
        let s = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                bail!("no {}: run osc capture pilot first", path.display())
            }
            Err(e) => return Err(e).with_context(|| format!("read {}", path.display())),
        };
        toml::from_str(&s).with_context(|| format!("parse {}", path.display()))
    }
}

pub(crate) fn round_to(x: f64, places: i32) -> f64 {
    let k = 10f64.powi(places);
    (x * k).round() / k
}

/// UTC civil date `YYYY-MM-DD` of a unix time (Hinnant's civil_from_days).
pub(crate) fn civil_date(secs: u64) -> String {
    let z = (secs / 86_400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!("{y:04}-{m:02}-{d:02}")
}

/// The pinned mg90 numbers, as the pilot would save them.
#[cfg(test)]
pub(super) fn mg90() -> Envelope {
    let fwd = Fit::rounded(0.2275, -0.845, 0.998);
    Envelope {
        supply: Supply::TwoS,
        fw: 64,
        git_sha: "5ee25770".into(),
        measured: civil_date(1_758_758_400),
        seek_pct: super::SEEK_PCT,
        limits: Limits {
            soft: [432, 3626],
            phys: [209, 3849],
            guard: [532, 3526],
            center: 2029,
            runway: 2994,
        },
        v_ss: Speed {
            duties: vec![10, 20, 30, 40, 50],
            used: Dir::Fwd,
            fwd,
            rev: Fit::rounded(0.2141, -0.712, 0.997),
        },
        windows_ms: [(5, 1500), (10, 1500), (60, 219), (100, 140)].into(),
        coast: Coast {
            probe_pct: 40,
            probe_entry: 8.255,
            probe_travel: 310,
            top_pct: 80,
            peak_inside_soft: 690,
        },
        verified: vec![Verified {
            step: "60@219".into(),
            travel: 2310,
            predicted: 2395,
        }],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn toml_round_trips() {
        let env = mg90();
        let dir = std::env::temp_dir().join(format!("osc-envelope-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = env.save(&dir).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.starts_with("# osc capture pilot envelope."));
        assert!(text.contains("supply = \"2s\""));
        assert!(text.contains("\n[windows_ms]\n5 = 1500\n"));
        assert!(text.is_ascii());
        let back = Envelope::load(&dir).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(back, env);
    }

    #[test]
    fn civil_date_from_unix_seconds() {
        assert_eq!(civil_date(0), "1970-01-01");
        assert_eq!(civil_date(1_758_758_400), "2025-09-25");
        assert_eq!(civil_date(1_758_758_400 + 86_399), "2025-09-25");
        // leap day and the year boundary after it
        assert_eq!(civil_date(951_782_400), "2000-02-29");
        assert_eq!(civil_date(978_307_200), "2001-01-01");
    }
}
