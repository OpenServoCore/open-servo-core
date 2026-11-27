//! The tool's files: the inputs the rules read, the set and table files,
//! and the run folder a save keeps its backup in.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use osc_ident_inputs::migrate::{ProfileIn, slug};
use osc_ident_inputs::profile::{FieldSource, HeatModel};
use osc_ident_inputs::{BoardFacts, Resolved, read as file_of, resolve, upgrade_with};
use osc_measure::board::{self, Record};
use osc_measure::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use osc_measure::screens::Holes;
use osc_measure::set::{SetFile, TableFile, file_sha1};
use osc_measure::sink::{Sink, SinkError};
use osc_measure::steps::table as table_step;
use osc_measure::{Board, Inputs, Nameplate};
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
    /// The file the heat model was read from and its sha1: the profile, or
    /// the thermal.json beside a version 0 nameplate.
    pub heat_from: (PathBuf, String),
    /// That thermal.json's bytes, where a version 0 nameplate took one.
    pub thermal_json: Option<Vec<u8>>,
}

impl Loaded {
    /// The profile's heat model as the rules and the steps read it.
    pub fn thermal(&self) -> Option<ThermalInputs> {
        let (file, sha1) = &self.heat_from;
        self.resolved
            .thermal
            .map(|h| ThermalInputs::of_profile(&h, &file.display().to_string(), sha1))
    }

    /// Where the heat model came from, and its four values.
    pub fn heat_said(&self) -> String {
        let named = &self.resolved.report.profile;
        let Some(h) = self.resolved.thermal else {
            return format!(
                "heat model: none, the profile {} states no thermal: no step that drives runs",
                named.key
            );
        };
        let from = match &self.thermal_json {
            Some(_) => format!(
                "{} beside the version 0 nameplate",
                self.heat_from.0.display()
            ),
            None => format!(
                "the profile {} rev {}",
                named.key,
                named.revision.unwrap_or(0)
            ),
        };
        format!(
            "heat model: from {from}, not identified: fast {:.2} s {:.1} C/W, slow {:.1} s {:.1} C/W",
            h.fast_tau_s, h.fast_gain_c_per_w, h.slow_tau_s, h.slow_gain_c_per_w
        )
    }
}

/// The profile file and the policy file, either at version 0 or 1, resolved
/// over the board's facts where they are read; refused naming the files and
/// the first value the run cannot use. With no policy, a version 0 nameplate
/// carries the three choices the steps read. A version 0 nameplate takes
/// its heat model from the `thermal.json` beside it, where there is one.
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

/// The `thermal.json` beside a version 0 nameplate: its path, bytes and
/// heat model.
struct Beside {
    path: PathBuf,
    bytes: Vec<u8>,
    heat: HeatModel,
}

/// The `thermal.json` beside the version 0 nameplate `profile`, where
/// there is one.
fn beside_v0(profile: &Path) -> Result<Option<Beside>> {
    let path = profile.with_file_name("thermal.json");
    if !path.exists() {
        return Ok(None);
    }
    let bytes = read(&path)?;
    let heat = file_of::thermal(utf8(&path, &bytes)?)
        .map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))?;
    Ok(Some(Beside { path, bytes, heat }))
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
    let beside = match p {
        ProfileIn::V0 { .. } => beside_v0(profile)?,
        ProfileIn::V1(_) => None,
    };
    let mut heat_from = (profile.to_path_buf(), file_sha1(bytes));
    let mut thermal_json = None;
    let heat = beside.map(|b| {
        let sha1 = file_sha1(&b.bytes);
        let from = FieldSource {
            how: Some("file".into()),
            file: Some(b.path.display().to_string()),
            sha1: Some(sha1.clone()),
            ..FieldSource::default()
        };
        heat_from = (b.path, sha1);
        thermal_json = Some(b.bytes);
        (b.heat, from)
    });
    let u = upgrade_with(p, q, heat).map_err(|e| anyhow::anyhow!("{named}: {e}"))?;
    let resolved =
        resolve(&u.profile, &u.policy, board).map_err(|e| anyhow::anyhow!("{named}: {e}"))?;
    let notes = u
        .notes
        .iter()
        .cloned()
        .chain(resolved.report.clamps())
        .map(|n| format!("{named}: {n}"))
        .collect();
    Ok(Loaded {
        resolved,
        notes,
        heat_from,
        thermal_json,
    })
}

/// The steps' nameplate: the profile resolved with the policy, where one is
/// named.
pub fn nameplate(path: &Path, policy: Option<&Path>) -> Result<Nameplate> {
    Ok(inputs(path, policy, None)?.resolved.nameplate)
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
/// `thermal.json` (beside a version 0 nameplate only: a version 1 profile
/// states its heat model), `shelf.json`, `pos_lut.json` (the table, a
/// notebooks table file) and `wide-codes.json` (the slots).
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
    let l = inputs_of(
        &np_path,
        &np_bytes,
        Some((&q, q_bytes)),
        Some(&board.facts()),
    )?;
    let p = dir.join("thermal.json");
    match &l.thermal_json {
        Some(_) => {
            take("thermal", p)?;
        }
        None if p.exists() => bail!(
            "{}: the profile is the heat model's one source, and a version 1 profile states its own thermal: enter the values in the profile and take the file out",
            p.display()
        ),
        None => {}
    }
    let thermal = l.thermal().unwrap_or_default();
    let Loaded {
        resolved: loaded,
        notes,
        ..
    } = l;
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

    fn testdata(rel: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../measure/testdata")
            .join(rel)
    }

    fn scratch(name: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("osc-ident-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("scratch");
        p
    }

    /// The set the rules write from the inputs folder `dir`.
    fn set_of(dir: &Path) -> SetFile {
        let r = rules_inputs(dir, None).expect("inputs");
        let (file, _) = osc_measure::set::from_rules(
            &osc_measure::build(&r.inputs),
            &osc_measure::set::FromRules {
                pair: "pair",
                nameplate: &r.inputs.nameplate,
                counts_per_deg: r.inputs.measured.table.as_ref().map(|t| t.counts_per_deg),
                table: r.table.as_ref(),
                slots: r.slots.as_deref(),
                files: &r.files,
            },
            &crate::descriptor_builtin().expect("descriptor"),
        )
        .expect("a set");
        file
    }

    const HEAT_WORDS: [&str; 4] = [
        "thermal_tau_s",
        "thermal_tau2_s",
        "th_rise_q016",
        "th_rise2_q016",
    ];

    /// The record's inputs with the record's profile and its policy, no
    /// thermal.json: the profile's heat model binds the set of record's four
    /// thermal words.
    #[test]
    fn the_profiles_heat_model_binds_the_records_thermal_words() {
        let src = testdata("mg90-d");
        let dir = scratch("heat-record");
        for f in [
            "measured.json",
            "shelf.json",
            "board.json",
            "pos_lut.json",
            "wide-codes.json",
        ] {
            std::fs::copy(src.join(f), dir.join(f)).expect(f);
        }
        std::fs::copy(src.join("profile.json"), dir.join("nameplate.json")).expect("profile");
        std::fs::copy(src.join("policy-of-record.json"), dir.join("policy.json")).expect("policy");
        let r = rules_inputs(&dir, None).expect("inputs");
        assert!(
            !r.files.iter().any(|(n, _, _)| n == "thermal"),
            "no thermal.json read"
        );
        let set = osc_measure::build(&r.inputs);
        let word = |f: &str| match set.get(f) {
            Some(osc_measure::Outcome::Bound(e)) => e.value,
            o => panic!("{f}: {o:?}"),
        };
        // set-of-record.json: 2, 129, 434, 1631
        assert_eq!(HEAT_WORDS.map(word), [2, 129, 434, 1631]);

        // a thermal.json beside a version 1 profile is refused, not read
        std::fs::copy(src.join("thermal.json"), dir.join("thermal.json")).expect("thermal");
        let e = rules_inputs(&dir, None).err().expect("refused");
        assert!(format!("{e:#}").contains("one source"), "{e:#}");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The recorded bench run's final inputs bind its final set: as the run
    /// gathered them (the version 0 nameplate with the thermal.json beside
    /// it, folded into its heat model), the whole set; with the record's
    /// profile and policy in their place and no thermal.json, the four
    /// thermal words and the protection hash.
    #[test]
    fn the_recorded_runs_final_inputs_bind_its_final_sets_thermal_words() {
        let run = testdata("run/00000000ffffffff76e5bc220ef6abcd/20261127T130203Z");
        let recorded = set(&run.join("final/set.json")).expect("the final set");
        let words = |s: &SetFile| HEAT_WORDS.map(|f| s.value(f).and_then(|v| v.as_i64()));
        assert_eq!(
            words(&recorded),
            [Some(2), Some(129), Some(447), Some(1678)]
        );
        let protection = |s: &SetFile| s.hashes().expect("hashes")["protection"].clone();
        let dir = scratch("heat-run");
        for f in std::fs::read_dir(run.join("final/inputs")).expect("inputs") {
            let f = f.expect("entry").path();
            std::fs::copy(&f, dir.join(f.file_name().expect("name"))).expect("copy");
        }
        std::fs::copy(run.join("final/measured.json"), dir.join("measured.json"))
            .expect("measured");
        let as_run = set_of(&dir);
        assert_eq!(words(&as_run), words(&recorded));
        assert_eq!(as_run.hashes().expect("hashes")["set"], recorded.hash());

        std::fs::remove_file(dir.join("thermal.json")).expect("rm");
        std::fs::copy(testdata("mg90-d/profile.json"), dir.join("nameplate.json"))
            .expect("profile");
        std::fs::copy(
            testdata("mg90-d/policy-of-record.json"),
            dir.join("policy.json"),
        )
        .expect("policy");
        let profiled = set_of(&dir);
        assert_eq!(words(&profiled), words(&recorded));
        assert_eq!(protection(&profiled), protection(&recorded));
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
