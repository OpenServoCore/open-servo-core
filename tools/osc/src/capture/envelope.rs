//! `envelope.toml`: what `osc capture pilot` measured on the servo and the
//! per-duty windows the campaign drives with. TOML because an operator reads
//! and edits it.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use osc_ident::runway;
use serde::{Deserialize, Serialize};

use super::{Rule, Supply};

const FILE: &str = "envelope.toml";

const HEADER: &str = "\
# osc capture pilot envelope. Positions in pot counts, speeds in counts/ms,
# times in ms. rule, current_limit_counts, rail_mv: the drive rule the pilot
# ran under, the servo's current limit and its rail at rest; a capture
# refuses the envelope once they differ. limits: soft/phys read from the
# servo, guard = soft inset, runway = guard_hi - guard_lo. grid: the ladder
# climbed the grid block's duties from the bottom, both ways, each rung run
# only while its predicted travel fit the runway and its braked stop fit
# between the soft limit and the stop beyond it. Per rung and direction:
# the window it ran, t_goal_ms (the first sample whose applied duty was the
# goal), travel by then, v_ss (fit over the samples at the goal past the
# settle), and the braked stop after it. windows_ms: the window a campaign
# rung drives at each duty kept, its climb and a tail crossing 0.8 of the
# runway, cut to what still fits it. v_ss: counts/ms = slope x duty_pct +
# intercept over the rungs that moved. coast: the ladder climbed the coast
# block's duties, each driven to its goal and 20 ms on (drive_ms), then
# coast_ms, both ways, while its predicted travel x (1 + margin) fit the room
# (the far edge of the start band to soft); top_pct is its top duty. Per rung
# and direction: predicted travel (none on the first rung), entry speed, lead
# (travel to the coast's first sample), coast, travel = lead + coast, and the
# peak's distance inside soft. chains: every other chain the session drives,
# run both ways, its excursion from the start and the peak's distance inside
# soft; refused_chains did not fit. fast: the grid ladder again under fast
# decay, for a recording that drives under it.
";

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Envelope {
    pub(crate) supply: Supply,
    pub(crate) fw: u16,
    pub(crate) git_sha: String,
    pub(crate) measured: String,
    pub(crate) seek_pct: u8,
    /// Free for an envelope that names none: measured before the servo
    /// limited open-loop current.
    #[serde(default)]
    pub(crate) rule: Rule,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) current_limit_counts: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) rail_mv: Option<u32>,
    pub(crate) limits: Limits,
    pub(crate) v_ss: Speed,
    pub(crate) windows_ms: BTreeMap<u8, u32>,
    pub(crate) grid: Grid,
    pub(crate) coast: Coast,
    pub(crate) chains: Vec<ChainRun>,
    pub(crate) refused_chains: Vec<ChainRefused>,
    /// The grid ladder again under fast decay, for a recording that drives
    /// under it; none when the procedure has no such recording.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) fast: Option<Grid>,
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
    pub(crate) fwd: Fit,
    pub(crate) rev: Fit,
}

impl Speed {
    pub(crate) fn of(&self, d: Dir) -> &Fit {
        match d {
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

#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct PerDir<T> {
    pub(crate) fwd: T,
    pub(crate) rev: T,
}

impl<T> PerDir<T> {
    pub(crate) fn get(&self, d: Dir) -> &T {
        match d {
            Dir::Fwd => &self.fwd,
            Dir::Rev => &self.rev,
        }
    }
}

/// The grid ladder: every duty it kept, and the first it did not.
#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Grid {
    /// The highest duty kept both ways; 0 when none was.
    pub(crate) top_pct: u8,
    pub(crate) rungs: Vec<GridRung>,
    pub(crate) refused: Option<LadderRefused>,
}

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct GridRung {
    pub(crate) pct: u8,
    pub(crate) window_ms: u32,
    pub(crate) fwd: RungRun,
    pub(crate) rev: RungRun,
}

impl GridRung {
    pub(crate) fn get(&self, d: Dir) -> &RungRun {
        match d {
            Dir::Fwd => &self.fwd,
            Dir::Rev => &self.rev,
        }
    }
}

/// One direction of one grid rung as it ran.
#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct RungRun {
    pub(crate) ran_ms: u32,
    pub(crate) t_goal_ms: f64,
    pub(crate) travel: u16,
    pub(crate) v_ss: f64,
    pub(crate) stop: u16,
}

#[derive(Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct LadderRefused {
    pub(crate) pct: u8,
    pub(crate) dir: Dir,
    pub(crate) why: String,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Coast {
    pub(crate) coast_ms: u32,
    pub(crate) margin: f64,
    pub(crate) top_pct: u8,
    pub(crate) room: PerDir<u16>,
    pub(crate) refused: Option<Refused>,
    pub(crate) ladder: Vec<CoastRung>,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct CoastRung {
    pub(crate) pct: u8,
    pub(crate) drive_ms: u32,
    pub(crate) fwd: CoastRun,
    pub(crate) rev: CoastRun,
}

impl CoastRung {
    pub(crate) fn get(&self, d: Dir) -> &CoastRun {
        match d {
            Dir::Fwd => &self.fwd,
            Dir::Rev => &self.rev,
        }
    }
}

/// One direction of one ladder rung; distances in counts from the start.
#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct CoastRun {
    pub(crate) predicted: Option<u16>,
    pub(crate) entry: f64,
    pub(crate) lead: u16,
    pub(crate) coast: u16,
    pub(crate) travel: u16,
    pub(crate) peak_inside_soft: i32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Refused {
    pub(crate) pct: u8,
    pub(crate) why: String,
}

/// A chain the pilot ran both ways: `chain` is its steps as a schedule
/// names them.
#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct ChainRun {
    pub(crate) block: String,
    pub(crate) chain: String,
    pub(crate) predicted: u16,
    pub(crate) fwd: Excursion,
    pub(crate) rev: Excursion,
}

#[cfg(test)]
impl ChainRun {
    pub(crate) fn get(&self, d: Dir) -> &Excursion {
        match d {
            Dir::Fwd => &self.fwd,
            Dir::Rev => &self.rev,
        }
    }
}

/// How far a chain carried the shaft from where it started, and how far
/// inside the soft limit ahead its furthest sample stayed (negative past
/// it).
#[derive(Copy, Clone, Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct Excursion {
    pub(crate) travel: u16,
    pub(crate) peak_inside_soft: i32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub(crate) struct ChainRefused {
    pub(crate) block: String,
    pub(crate) chain: String,
    pub(crate) predicted: u16,
    pub(crate) why: String,
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
        toml::from_str(&s).with_context(|| {
            format!(
                "parse {}: an older pilot wrote it, run osc capture pilot again",
                path.display()
            )
        })
    }
}

impl Envelope {
    /// The chain `chain` of `block`, when the pilot ran it.
    pub(crate) fn chain(&self, block: &str, chain: &str) -> Option<&ChainRun> {
        self.chains
            .iter()
            .find(|c| c.block == block && c.chain == chain)
    }
}

impl Envelope {
    /// What a runway reads of it: the v_ss lines and every braked stop the
    /// grid ladder measured, both ways, from the speed it braked at.
    pub(crate) fn runway(&self) -> runway::Envelope {
        let line = |f: &Fit| runway::Line {
            slope: f.slope,
            intercept: f.intercept,
        };
        runway::Envelope {
            supply: self.supply.runway(),
            phys: (self.limits.phys[0], self.limits.phys[1]),
            fwd: line(&self.v_ss.fwd),
            rev: line(&self.v_ss.rev),
            coast: self
                .grid
                .rungs
                .iter()
                .flat_map(|r| [r.fwd, r.rev])
                .filter(|r| r.v_ss > 0.0)
                .map(|r| (r.v_ss, r.stop as f64))
                .collect(),
        }
    }
}

/// Every dataset under `root` holding an envelope, in name order, each as
/// read: an envelope from an older pilot does not parse.
pub(crate) fn every(root: &Path) -> Vec<(PathBuf, Result<Envelope>)> {
    let Ok(dirs) = std::fs::read_dir(root) else {
        return Vec::new();
    };
    let mut found: Vec<PathBuf> = dirs
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|d| d.join(FILE).is_file())
        .collect();
    found.sort();
    found
        .into_iter()
        .map(|d| {
            let env = Envelope::load(&d);
            (d.join(FILE), env)
        })
        .collect()
}

/// Size `runway` by the first envelope under the dataset root that
/// describes the servo on `supply` with its stops at `phys`: every
/// envelope left out before it with why, and the one that sized it.
pub(crate) fn size_runway(
    runway: &mut runway::Runway,
    supply: Option<runway::Supply>,
    phys: (i32, i32),
) -> (Vec<(PathBuf, String)>, Option<PathBuf>) {
    let found = super::default_root()
        .map(|root| every(&root))
        .unwrap_or_default();
    size_by(runway, supply, phys, found)
}

fn size_by(
    runway: &mut runway::Runway,
    supply: Option<runway::Supply>,
    phys: (i32, i32),
    found: Vec<(PathBuf, Result<Envelope>)>,
) -> (Vec<(PathBuf, String)>, Option<PathBuf>) {
    let mut left = Vec::new();
    for (path, env) in found {
        let why = match env {
            Ok(env) if env.rule == Rule::Free => {
                "it was measured before the servo limited open-loop current (run osc capture \
                 pilot again)"
                    .into()
            }
            Ok(env) => match runway.size_by(env.runway(), supply, phys) {
                Ok(()) => return (left, Some(path)),
                Err(stale) => stale.to_string(),
            },
            Err(_) => "an older pilot wrote it (run osc capture pilot again)".into(),
        };
        left.push((path, why));
    }
    (left, None)
}

impl Limits {
    /// The limits alone, whatever else the file holds: a dataset's envelope
    /// keeps naming the stops after the sections around it change shape.
    pub(crate) fn load(dir: &Path) -> Result<Self> {
        #[derive(Deserialize)]
        struct Only {
            limits: Limits,
        }
        let path = dir.join(FILE);
        let s =
            std::fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
        toml::from_str::<Only>(&s)
            .map(|o| o.limits)
            .with_context(|| format!("parse {}", path.display()))
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

/// The bench MG90 on 2S under its 280-count limit as the pilot measures it
/// on the test servo: per grid duty the climb (t_goal ms, travel), v_ss, the
/// braked stop and the window, the coast ladder and every chain.
#[cfg(test)]
pub(super) fn mg90() -> Envelope {
    const GRID: [(u8, u32, f64, u16, f64, u16, u32); 11] = [
        (5, 150, 0.0, 0, 0.0, 0, 1500),
        (10, 150, 0.0, 0, 0.0, 0, 1500),
        (15, 150, 2.9, 0, 2.255, 25, 1064),
        (20, 194, 14.85, 11, 3.307, 41, 735),
        (25, 155, 26.65, 35, 4.328, 58, 571),
        (30, 168, 40.45, 80, 5.371, 76, 471),
        (35, 182, 55.4, 145, 6.413, 94, 406),
        (40, 199, 71.75, 236, 7.456, 113, 361),
        (45, 216, 89.8, 358, 8.498, 132, 329),
        (50, 235, 108.25, 504, 9.539, 151, 305),
        (55, 254, 130.75, 707, 10.58, 170, 286),
    ];
    let rungs: Vec<GridRung> = GRID
        .iter()
        .map(|&(pct, ran_ms, t_goal_ms, travel, v_ss, stop, window_ms)| {
            let run = RungRun {
                ran_ms,
                t_goal_ms,
                travel,
                v_ss,
                stop,
            };
            GridRung {
                pct,
                window_ms,
                fwd: run,
                rev: run,
            }
        })
        .collect();
    let grid = Grid {
        top_pct: 55,
        rungs,
        refused: Some(LadderRefused {
            pct: 60,
            dir: Dir::Rev,
            why: "rev 60% would stop in about 206 counts, over the 200 between the soft limit \
                  and the stop: a rung its host abandoned would hit the stop"
                .into(),
        }),
    };
    let chain = |block: &str, chain: &str, predicted: u16, travel: u16, inside: i32| ChainRun {
        block: block.into(),
        chain: chain.into(),
        predicted,
        fwd: Excursion {
            travel,
            peak_inside_soft: inside,
        },
        rev: Excursion {
            travel,
            peak_inside_soft: inside,
        },
    };
    Envelope {
        supply: Supply::TwoS,
        fw: 64,
        git_sha: "5ee25770".into(),
        measured: civil_date(1_758_758_400),
        seek_pct: 15,
        rule: Rule::Limit,
        current_limit_counts: Some(280),
        rail_mv: Some(7899),
        limits: Limits {
            soft: [432, 3626],
            phys: [232, 3849],
            guard: [532, 3526],
            center: 2029,
            runway: 2994,
        },
        v_ss: Speed {
            duties: GRID.iter().filter(|g| g.4 > 0.0).map(|g| g.0).collect(),
            fwd: Fit::rounded(0.2081, -0.866, 1.0),
            rev: Fit::rounded(0.2081, -0.867, 1.0),
        },
        windows_ms: GRID.iter().map(|g| (g.0, g.6)).collect(),
        grid: grid.clone(),
        coast: Coast {
            coast_ms: 400,
            margin: 0.1,
            top_pct: 40,
            room: PerDir {
                fwd: 3019,
                rev: 3019,
            },
            refused: Some(Refused {
                pct: 60,
                why: "the grid ladder never kept 60%, so it has no climb to drive by".into(),
            }),
            ladder: [
                (20, 35, 2.206, 65, 73, None),
                (30, 61, 4.432, 187, 200, Some(592)),
                (40, 92, 6.675, 395, 368, Some(663)),
            ]
            .into_iter()
            .map(|(pct, drive_ms, entry, lead, coast, predicted)| {
                let run = CoastRun {
                    predicted,
                    entry,
                    lead,
                    coast,
                    travel: lead + coast,
                    peak_inside_soft: 3019 - (lead + coast) as i32,
                };
                CoastRung {
                    pct,
                    drive_ms,
                    fwd: run,
                    rev: run,
                }
            })
            .collect(),
        },
        chains: vec![
            chain("step", "30@20,coast:400", 425, 79, 2988),
            chain("step", "30@40,coast:400", 533, 230, 2837),
            chain("step", "30@60,coast:400", 640, 380, 2687),
            chain("step", "30@80,coast:400", 748, 505, 2562),
            chain("step", "30@120,coast:400", 963, 729, 2338),
            chain(
                "reversal",
                "20@60,then:-20@60,then:20@60,brake:200",
                746,
                235,
                2832,
            ),
            chain(
                "reversal",
                "40@60,then:-40@60,then:40@60,brake:200",
                1689,
                398,
                2669,
            ),
            chain("ends", "15@1277", 2940, 2877, 152),
            chain("ends", "20@897", 3044, 2910, 157),
        ],
        refused_chains: Vec::new(),
        fast: Some(grid),
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
        assert!(text.contains("rule = \"limit\"\ncurrent_limit_counts = 280\nrail_mv = 7899\n"));
        assert!(text.contains("\n[windows_ms]\n5 = 1500\n"));
        assert!(text.contains("\n[[grid.rungs]]\npct = 5\nwindow_ms = 1500\n"));
        assert!(text.contains(
            "\n[[coast.ladder]]\npct = 20\ndrive_ms = 35\n\n[coast.ladder.fwd]\nentry = 2.206\n"
        ));
        assert!(text.contains("\n[[chains]]\nblock = \"step\"\nchain = \"30@20,coast:400\"\n"));
        assert!(text.is_ascii());
        let back = Envelope::load(&dir).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(back, env);
    }

    /// The runway reads the v_ss lines and the braked stops both ways, and
    /// takes the envelope only for its supply and stops; the pilot's older
    /// file does not parse and sizes nothing.
    #[test]
    fn envelope_sizes_the_ladder_runway() {
        let r = mg90().runway();
        assert_eq!(r.supply, runway::Supply::TwoS);
        assert_eq!(r.phys, (232, 3849));
        assert_eq!(r.fwd.at(60.0), mg90().v_ss.fwd.at(60));
        assert_eq!(r.coast.len(), 18, "9 moving rungs both ways");
        assert_eq!(r.coast[17], (10.58, 170.0));
        assert!(r.check(Some(runway::Supply::TwoS), (232, 3849)).is_ok());
        assert!(r.check(Some(runway::Supply::Usb), (232, 3849)).is_err());

        let dir = std::env::temp_dir().join(format!("osc-envelopes-{}", std::process::id()));
        let (fresh, old) = (dir.join("a__2s"), dir.join("b__2s"));
        std::fs::create_dir_all(&fresh).unwrap();
        std::fs::create_dir_all(&old).unwrap();
        std::fs::create_dir_all(dir.join("c__usb")).unwrap();
        mg90().save(&fresh).unwrap();
        let text = std::fs::read_to_string(fresh.join(FILE)).unwrap();
        let probe =
            "[coast]\nprobe_pct = 40\nprobe_entry = 7.293\nprobe_travel = 551\ntop_pct = 66\n";
        let head = text.split("[coast]").next().unwrap();
        std::fs::write(old.join(FILE), format!("{head}{probe}")).unwrap();
        let found = every(&dir);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(found.len(), 2, "a dir without an envelope is skipped");
        assert_eq!(found[0].0, fresh.join(FILE));
        assert_eq!(found[0].1.as_ref().unwrap(), &mg90());
        let e = format!("{:#}", found[1].1.as_ref().unwrap_err());
        assert!(e.contains("an older pilot wrote it"), "{e}");
        assert!(every(&dir).is_empty());
    }

    /// The braked stops the runway predicts by are over three times shorter
    /// than a coast from the same speed: sized by them, a rung brakes where
    /// it must, not a coast early.
    #[test]
    fn a_runway_brakes_by_the_braked_stops() {
        let mut rw = runway::Runway::new((532, 3526));
        rw.size_by(mg90().runway(), Some(runway::Supply::TwoS), (232, 3849))
            .unwrap();
        let stop = rw.stop(10.58).unwrap();
        assert!((160.0..180.0).contains(&stop), "{stop}");
        let c = mg90().coast.ladder[2].fwd;
        let stop = rw.stop(c.entry).unwrap();
        assert!(c.coast as f64 > 3.0 * stop, "{} against {stop}", c.coast);
    }

    /// An envelope with no rule was measured before the servo limited
    /// open-loop current: nothing sizes a runway by it, and the front
    /// refuses it. A file with no grid ladder does not parse at all: it
    /// is sent back to the pilot.
    #[test]
    fn an_envelope_from_before_the_limiter_is_refused() {
        let dir = std::env::temp_dir().join(format!("osc-envelope-free-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let free = Envelope {
            rule: Rule::Free,
            current_limit_counts: None,
            rail_mv: None,
            ..mg90()
        };
        free.save(&dir).unwrap();
        let text = std::fs::read_to_string(dir.join(FILE)).unwrap();
        // one that names no rule reads as free
        std::fs::write(dir.join(FILE), text.replace("rule = \"free\"\n", "")).unwrap();
        let back = Envelope::load(&dir).unwrap();
        assert_eq!(back.rule, Rule::Free);
        let mut rw = runway::Runway::new((532, 3526));
        let (left, sized) = size_by(
            &mut rw,
            Some(runway::Supply::TwoS),
            (232, 3849),
            vec![(dir.join(FILE), Ok(back))],
        );
        assert_eq!(sized, None);
        assert_eq!(
            left[0].1,
            "it was measured before the servo limited open-loop current (run osc capture pilot \
             again)"
        );
        assert!(rw.envelope().is_none());

        let (_, sized) = size_by(
            &mut rw,
            Some(runway::Supply::TwoS),
            (232, 3849),
            vec![(dir.join(FILE), Ok(mg90()))],
        );
        assert_eq!(sized, Some(dir.join(FILE)));

        let head = text.split("[grid]").next().unwrap();
        std::fs::write(
            dir.join(FILE),
            format!("{head}[[verified]]\nstep = \"60@219\"\n"),
        )
        .unwrap();
        let e = format!("{:#}", Envelope::load(&dir).unwrap_err());
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(
            e.contains("an older pilot wrote it, run osc capture pilot again"),
            "{e}"
        );
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
