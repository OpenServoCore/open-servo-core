//! The tool's files: the inputs the rules read, the set and table files,
//! and the run folder a save keeps its backup in.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use osc_ident_inputs::migrate::{ProfileIn, slug};
use osc_ident_inputs::profile::HeatModel;
use osc_ident_inputs::{BoardFacts, Resolved, read as file_of, resolve, upgrade};
use osc_measure::board::{self, Record};
use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::screens::Holes;
use osc_measure::set::{SetFile, TableFile, file_sha1};
use osc_measure::sink::{Sink, SinkError};
use osc_measure::steps::table as table_step;
use osc_measure::{Board, Inputs, Nameplate, Quantity, Source};
use serde::de::DeserializeOwned;

/// The run's pot holes the closed-loop check's pot steps leave out: those
/// of the table step's result the gathered inputs `measured` (a
/// `measured.json`) name as their table's source, read again and checked
/// against its sha1. None, said, where the inputs hold no table or name no
/// table step's result. Refused where that result is gone or changed.
pub fn run_holes(measured: &serde_json::Value) -> Result<Holes> {
    let src = &measured["table"]["source"];
    let Some(file) = src["file"].as_str() else {
        return Ok(Holes::none(
            "the inputs hold no table: no pot hole left out",
        ));
    };
    if !file.ends_with(table_step::RESULT) {
        return Ok(Holes::none(format!(
            "the inputs' table is {file}, no table step's result: no pot hole left out"
        )));
    }
    let p = Path::new(file);
    let bytes = read(p).context(
        "the table result the inputs' table names, whose pot holes the check's pot steps leave out",
    )?;
    if let Some(want) = src["sha1"].as_str()
        && file_sha1(&bytes) != want
    {
        bail!("{file} changed since the inputs were gathered (sha1 {want}): gather again");
    }
    let doc: serde_json::Value = json(p, &bytes)?;
    Holes::of_table(&doc, file).map_err(|e| anyhow::anyhow!(e))
}

/// A run's board file, `inputs/board.json`: what step 0 read off the servo.
pub const BOARD: &str = "inputs/board.json";

/// The board record in the inputs folder `dir`, refused where step 0 wrote
/// none.
pub fn board_in(dir: &Path) -> Result<(Record, Board, Vec<u8>)> {
    let p = dir.join(board::FILE);
    if !p.exists() {
        bail!(
            "{} is not there: the board's facts are read off the servo, so rerun step 0 on the servo",
            p.display()
        );
    }
    let bytes = read(&p)?;
    let record: Record = json(&p, &bytes)?;
    let b = record
        .board()
        .map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?;
    Ok((record, b, bytes))
}

/// A profile (or a version 0 nameplate) and the policy beside it as the
/// rules read them, with a line for each file read at an older version and
/// for each value stored at a bound it is over.
pub struct Loaded {
    pub resolved: Resolved,
    pub notes: Vec<String>,
}

/// The profile file and the policy file, either at version 0 or 1, resolved
/// over the board's facts where they are read; refused naming the files and
/// the first value the run cannot use. With no policy, a version 0 nameplate
/// carries the three choices the steps read.
pub fn inputs(profile: &Path, policy: Option<&Path>, board: Option<&BoardFacts>) -> Result<Loaded> {
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
    board: Option<&BoardFacts>,
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
    let resolved =
        resolve(&u.profile, &u.policy, board).map_err(|e| anyhow::anyhow!("{named}: {e}"))?;
    let notes = u
        .notes
        .iter()
        .cloned()
        .chain(resolved.report.clamps())
        .map(|n| format!("{named}: {n}"))
        .collect();
    Ok(Loaded { resolved, notes })
}

/// The steps' nameplate: the profile resolved with the policy, where one is
/// named.
pub fn nameplate(path: &Path, policy: Option<&Path>) -> Result<Nameplate> {
    Ok(inputs(path, policy, None)?.resolved.nameplate)
}

/// The profile's heat model as the rules read thermal values, each named
/// by its key in the profile file.
pub fn thermal_of(h: &HeatModel, profile: &Path, sha1: &str) -> ThermalInputs {
    let q = |value: f64, unit: &str, key: &str| {
        Some(Quantity {
            value,
            lo: None,
            hi: None,
            sd: None,
            n: None,
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
    /// The servo step 0 read the board off, by its run folder's name.
    pub servo: String,
    pub table: Option<TableFile>,
    pub slots: Option<Vec<u8>>,
    pub files: Vec<(String, String, String)>,
    /// A line for each input file read at an older version.
    pub notes: Vec<String>,
}

/// `dir` holds `measured.json`, `nameplate.json` (the servo profile, either
/// version), `policy.json` and `board.json` (step 0's), and where they exist
/// `thermal.json` (else the profile's heat model), `shelf.json`,
/// `pos_lut.json` (the table, a notebooks table file) and `wide-codes.json`
/// (the slots).
pub fn rules_inputs(dir: &Path, policy: Option<&Path>) -> Result<RulesInputs> {
    let (record, board, board_bytes) = board_in(dir)?;
    let mut files = vec![(
        "board".to_string(),
        dir.join(board::FILE).display().to_string(),
        file_sha1(&board_bytes),
    )];
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
    let Loaded {
        resolved: loaded,
        notes,
    } = inputs_of(
        &np_path,
        &np_bytes,
        Some((&q, q_bytes)),
        Some(&board.facts()),
    )?;
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
        servo: record.servo,
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

/// The board file step 0 writes on the bench servo, in the run folder
/// `run`.
#[cfg(test)]
pub(crate) fn step_0(run: &Path) {
    let rec =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../measure/testdata/mg90-d/board.json");
    std::fs::create_dir_all(run.join("inputs")).expect("mkdir");
    std::fs::copy(rec, run.join(BOARD)).expect("copy");
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

    /// A value the policy states over its bound is said among the loaded
    /// inputs' lines, which the steps and the gather print, naming the files.
    #[test]
    fn a_value_stored_at_a_bound_is_said_with_the_inputs() {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../measure/testdata/mg90-d");
        let profile = dir.join("profile.json");
        let policy = dir.join("policy-of-record.json");
        let mut q: serde_json::Value =
            serde_json::from_slice(&read(&policy).expect("read")).expect("json");
        q["motion"]["move_speed"] = serde_json::json!({"deg_s": 500.0});
        let (_, board, _) = board_in(&dir).expect("board");
        let l = inputs_of(
            &profile,
            &read(&profile).expect("read"),
            Some((&policy, serde_json::to_vec(&q).expect("bytes"))),
            Some(&board.facts()),
        )
        .expect("loads");
        assert_eq!(l.resolved.policy.velocity_limit_deg_s, Some(400.0));
        assert_eq!(
            l.notes,
            [format!(
                "{} with {}: motion.move_speed: 500 deg/s is over the 400 deg/s speed ceiling, where it cannot act: stored as 400 deg/s",
                profile.display(),
                policy.display()
            )]
        );
    }

    /// The check's pot holes are those of the table step's result the
    /// inputs name as their table's source, read again: none, said, where
    /// the inputs' table is no table step's result; refused where that
    /// result is gone or changed since the gather.
    #[test]
    fn the_run_holes_are_the_table_results_the_inputs_name() {
        let dir = std::env::temp_dir().join(format!("osc-ident-holes-{}", std::process::id()));
        std::fs::create_dir_all(dir.join("results")).expect("mkdir");
        let file = dir.join(table_step::RESULT);
        let doc = serde_json::json!({"step": "table", "holes": {"pot": {"holes": [
            {"from_counts": 6.75, "to_counts": 118.8}]}}});
        let bytes = serde_json::to_vec(&doc).expect("bytes");
        std::fs::write(&file, &bytes).expect("write");
        let measured = |sha1: &str| {
            serde_json::json!({"table": {"counts_per_deg": 20.0, "source": {
                "file": file.display().to_string(), "key": "scale.counts_per_deg", "sha1": sha1}}})
        };
        let h = run_holes(&measured(&file_sha1(&bytes))).expect("holes");
        assert_eq!(h.ranges, vec![(6.75, 118.8)]);
        assert_eq!(h.source, format!("the 1 pot hole of {}", file.display()));
        let changed = run_holes(&measured("0000")).expect_err("changed");
        assert!(
            changed
                .to_string()
                .contains("changed since the inputs were gathered"),
            "{changed}"
        );
        let none = run_holes(&serde_json::json!({"r_ohm": {}})).expect("none");
        assert!(
            none.ranges.is_empty() && none.source.contains("no table"),
            "{none:?}"
        );
        let record = serde_json::json!({"table": {"source": {"file": "notebooks/x.pos_lut.json", "sha1": "1"}}});
        let h = run_holes(&record).expect("none");
        assert!(
            h.ranges.is_empty() && h.source.contains("no table step's result"),
            "{h:?}"
        );
        std::fs::remove_file(&file).expect("rm");
        assert!(run_holes(&measured(&file_sha1(&bytes))).is_err());
        let _ = std::fs::remove_dir_all(dir);
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
        let why = inputs_of(&p, &serde_json::to_vec(&v).expect("bytes"), None, None)
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
