//! The tool's files: the inputs the rules read, the set and table files,
//! and the run folder a save keeps its backup in.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use osc_ident_inputs::migrate::{ProfileIn, slug};
use osc_ident_inputs::profile::HeatModel;
use osc_ident_inputs::{Resolved, read as file_of, resolve, upgrade};
use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::set::{SetFile, TableFile, file_sha1};
use osc_measure::sink::{Sink, SinkError};
use osc_measure::{Board, Inputs, Nameplate, Quantity, Source};
use serde::de::DeserializeOwned;

/// The dev board's description: the board a set is built for unless
/// `--board` names another.
const DEV_BOARD: &str = include_str!("../../../measure/boards/osc-dev-v006.json");

/// The board named, else the dev board.
pub fn board(path: Option<&Path>) -> Result<Board> {
    match path {
        Some(p) => {
            serde_json::from_slice(&read(p)?).with_context(|| format!("parsing {}", p.display()))
        }
        None => serde_json::from_str(DEV_BOARD).context("the built-in board"),
    }
}

/// A profile (or a version 0 nameplate) and the policy beside it as the
/// rules read them, with a line for each file read at an older version.
pub struct Loaded {
    pub resolved: Resolved,
    pub notes: Vec<String>,
}

/// The profile file and the policy file, either at version 0 or 1, resolved
/// over the board's facts; refused naming the files and the first value the
/// run cannot use. With no policy, a version 0 nameplate carries the three
/// choices the steps read.
pub fn inputs(profile: &Path, policy: Option<&Path>, board: &Board) -> Result<Loaded> {
    let policy = match policy {
        Some(p) => Some((p, read(p)?)),
        None => None,
    };
    inputs_of(profile, &read(profile)?, policy, board)
}

fn utf8<'a>(path: &Path, bytes: &'a [u8]) -> Result<&'a str> {
    std::str::from_utf8(bytes).with_context(|| format!("{} is not UTF-8", path.display()))
}

fn inputs_of(
    profile: &Path,
    bytes: &[u8],
    policy: Option<(&Path, Vec<u8>)>,
    board: &Board,
) -> Result<Loaded> {
    let p = file_of::profile(utf8(profile, bytes)?)
        .map_err(|e| anyhow::anyhow!("{}: {e}", profile.display()))?;
    let named = match &policy {
        Some((q, _)) => format!("{} with {}", profile.display(), q.display()),
        None => profile.display().to_string(),
    };
    let q = match &policy {
        Some((path, bytes)) => {
            let key = path
                .file_stem()
                .map_or("policy".into(), |s| slug(&s.to_string_lossy()));
            Some(
                file_of::policy(utf8(path, bytes)?, &key)
                    .map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))?,
            )
        }
        None if matches!(p, ProfileIn::V1(_)) => bail!(
            "{named}: a version 1 profile holds no run voltage or derates, which are the tuning policy's: name the policy with --policy"
        ),
        None => None,
    };
    let u = upgrade(p, q).map_err(|e| anyhow::anyhow!("{named}: {e}"))?;
    let resolved = resolve(&u.profile, &u.policy, Some(&board.facts()))
        .map_err(|e| anyhow::anyhow!("{named}: {e}"))?;
    let notes = u.notes.iter().map(|n| format!("{named}: {n}")).collect();
    Ok(Loaded { resolved, notes })
}

/// The steps' nameplate: the profile resolved with the policy, where one is
/// named.
pub fn nameplate(path: &Path, policy: Option<&Path>) -> Result<Nameplate> {
    Ok(inputs(path, policy, &board(None)?)?.resolved.nameplate)
}

/// The profile's heat model as the rules read thermal values, each named
/// by its key in the profile file.
pub fn thermal_of(h: &HeatModel, profile: &Path, sha1: &str) -> ThermalInputs {
    let q = |value: f64, unit: &str, key: &str| {
        Some(Quantity {
            value,
            lo: None,
            hi: None,
            unit: unit.into(),
            source: Source {
                file: profile.display().to_string(),
                key: key.into(),
                sha1: Some(sha1.into()),
            },
        })
    };
    ThermalInputs {
        fast_tau_s: q(h.fast_tau_s, "s", "thermal.fast_tau_s"),
        fast_gain_c_per_w: q(h.fast_gain_c_per_w, "C/W", "thermal.fast_gain_c_per_w"),
        slow_tau_s: q(h.slow_tau_s, "s", "thermal.slow_tau_s"),
        slow_gain_c_per_w: q(h.slow_gain_c_per_w, "C/W", "thermal.slow_gain_c_per_w"),
    }
}

pub fn read(path: &Path) -> Result<Vec<u8>> {
    std::fs::read(path).with_context(|| format!("reading {}", path.display()))
}

fn json<T: DeserializeOwned>(path: &Path, bytes: &[u8]) -> Result<T> {
    serde_json::from_slice(bytes).with_context(|| format!("parsing {}", path.display()))
}

pub fn set(path: &Path) -> Result<SetFile> {
    let text = String::from_utf8(read(path)?).context("a set file is UTF-8")?;
    SetFile::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))
}

pub fn table(path: &Path) -> Result<TableFile> {
    let text = String::from_utf8(read(path)?).context("a table file is UTF-8")?;
    TableFile::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))
}

/// The table file for a set: the one named, else the notebooks' layout,
/// `<pair>.pos_lut.json` beside the set, where there is one.
pub fn table_for(
    set_path: &Path,
    set: &SetFile,
    named: Option<&Path>,
) -> Result<Option<TableFile>> {
    if let Some(p) = named {
        return table(p).map(Some);
    }
    let beside = set_path
        .parent()
        .unwrap_or(Path::new("."))
        .join(format!("{}.pos_lut.json", set.pair()));
    if beside.exists() {
        table(&beside).map(Some)
    } else {
        Ok(None)
    }
}

#[derive(serde::Deserialize)]
struct Slots {
    slots: Vec<u8>,
}

/// What the rules read from an inputs folder, every file read named with
/// its sha1.
pub struct RulesInputs {
    pub inputs: Inputs,
    pub table: Option<TableFile>,
    pub slots: Option<Vec<u8>>,
    pub files: Vec<(String, String, String)>,
    /// A line for each input file read at an older version.
    pub notes: Vec<String>,
}

/// `dir` holds `measured.json`, `nameplate.json` (the servo profile, either
/// version) and `policy.json`, and where they exist `thermal.json` (else the
/// profile's heat model), `shelf.json`, `pos_lut.json` (the table, a
/// notebooks table file) and `wide-codes.json` (the slots).
pub fn rules_inputs(
    dir: &Path,
    policy: Option<&Path>,
    board: Option<&Path>,
) -> Result<RulesInputs> {
    let mut files = Vec::new();
    let mut take = |name: &str, path: PathBuf| -> Result<Option<Vec<u8>>> {
        if !path.exists() {
            return Ok(None);
        }
        let bytes = read(&path)?;
        files.push((
            name.to_string(),
            path.display().to_string(),
            file_sha1(&bytes),
        ));
        Ok(Some(bytes))
    };
    let need = |name: &str, b: Option<Vec<u8>>, path: &Path| match b {
        Some(b) => Ok(b),
        None => Err(anyhow::anyhow!("{name}: {} is not there", path.display())),
    };
    let p = dir.join("measured.json");
    let mut measured: Measured = json(&p, &need("measured", take("measured", p.clone())?, &p)?)?;
    let p = dir.join("shelf.json");
    if let Some(b) = take("shelf", p.clone())? {
        measured.shelf = Some(json::<ShelfRecord>(&p, &b)?);
    }
    let np_path = dir.join("nameplate.json");
    let np_bytes = need("nameplate", take("nameplate", np_path.clone())?, &np_path)?;
    let q = policy.map_or_else(|| dir.join("policy.json"), Path::to_path_buf);
    let q_bytes = need("policy", take("policy", q.clone())?, &q)?;
    let p = dir.join("thermal.json");
    let thermal_bytes = take("thermal", p.clone())?;
    let board: Board = match board {
        Some(p) => json(p, &need("board", take("board", p.to_path_buf())?, p)?)?,
        None => serde_json::from_str(DEV_BOARD).context("the built-in board")?,
    };
    let Loaded {
        resolved: loaded,
        notes,
    } = inputs_of(&np_path, &np_bytes, Some((&q, q_bytes)), &board)?;
    let thermal: ThermalInputs = match (thermal_bytes, &loaded.thermal) {
        (Some(b), _) => json(&p, &b)?,
        (None, Some(h)) => thermal_of(h, &np_path, &file_sha1(&np_bytes)),
        (None, None) => ThermalInputs::default(),
    };
    let p = dir.join("pos_lut.json");
    let table = match take("pos_table", p.clone())? {
        Some(b) => Some(
            TableFile::parse(std::str::from_utf8(&b).context("UTF-8")?)
                .map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?,
        ),
        None => None,
    };
    let p = dir.join("wide-codes.json");
    let slots = match take("wide_codes", p.clone())? {
        Some(b) => Some(json::<Slots>(&p, &b)?.slots),
        None => None,
    };
    Ok(RulesInputs {
        inputs: Inputs {
            board,
            measured,
            nameplate: loaded.nameplate,
            policy: loaded.policy,
            thermal,
            tuning: Tuning::default(),
        },
        table,
        slots,
        files,
        notes,
    })
}

/// A run's folder on disk. A file a run kept is never written over.
pub struct DirSink {
    pub root: PathBuf,
}

impl Sink for DirSink {
    fn write(&mut self, path: &str, bytes: &[u8]) -> Result<(), SinkError> {
        let p = self.root.join(path);
        if p.exists() {
            return Err(SinkError(format!(
                "{} exists: a run never writes over its files",
                p.display()
            )));
        }
        if let Some(dir) = p.parent() {
            std::fs::create_dir_all(dir)
                .map_err(|e| SinkError(format!("{}: {e}", dir.display())))?;
        }
        std::fs::write(&p, bytes).map_err(|e| SinkError(format!("{}: {e}", p.display())))
    }

    fn read(&mut self, path: &str) -> Result<Vec<u8>, SinkError> {
        let p = self.root.join(path);
        std::fs::read(&p).map_err(|e| SinkError(format!("{}: {e}", p.display())))
    }

    fn shown(&self, path: &str) -> String {
        self.root.join(path).display().to_string()
    }
}

/// `<out>/<servo>/<UTC time>/`, refused when it exists.
pub fn run_folder(out: &Path, servo: &str) -> Result<PathBuf> {
    let p = out.join(servo).join(utc(SystemTime::now()));
    if p.exists() {
        bail!("{} exists", p.display());
    }
    Ok(p)
}

/// `YYYYMMDDTHHMMSSZ`.
pub fn utc(t: SystemTime) -> String {
    let secs = t.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as i64;
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // days to a civil date, the proleptic Gregorian calendar in 400-year eras
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z",
        rem / 3_600,
        rem / 60 % 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn utc_names_the_civil_time() {
        let at = |s: u64| utc(UNIX_EPOCH + Duration::from_secs(s));
        assert_eq!(at(0), "19700101T000000Z");
        assert_eq!(at(951_782_400), "20000229T000000Z");
        assert_eq!(at(1_700_000_000), "20231114T221320Z");
    }

    /// The bench nameplate loads; one still stating ripple_per_rev is
    /// refused naming the file and the two counts to enter instead.
    #[test]
    fn a_nameplate_stating_ripple_per_rev_is_refused() {
        let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../measure/testdata/mg90-d/nameplate.json");
        let np = nameplate(&p, None).expect("the bench nameplate loads");
        assert_eq!(np.events_per_rev(), Ok(6));
        let mut v: serde_json::Value =
            serde_json::from_slice(&read(&p).expect("read")).expect("json");
        v["ripple_per_rev"] = 6.into();
        let board = board(None).expect("board");
        let why = inputs_of(&p, &serde_json::to_vec(&v).expect("bytes"), None, &board)
            .err()
            .expect("refused")
            .to_string();
        assert_eq!(
            why,
            format!(
                "{}: {}",
                p.display(),
                osc_measure::nameplate::RETIRED_RIPPLE_PER_REV
            )
        );
    }
}
