//! The run's files: the inputs the rules read, the set and table files,
//! and the run folder a step or a save keeps its files in, all read and
//! written through a [`Store`].

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use osc_ident_inputs::migrate::{ProfileIn, slug};
use osc_ident_inputs::profile::{FieldSource, HeatModel};
use osc_ident_inputs::{BoardFacts, Resolved, read as file_of, resolve, upgrade_with};
use serde::de::DeserializeOwned;

use super::store::Store;
use crate::board::{self, Record};
use crate::inputs::{Measured, ShelfRecord, ThermalInputs, Tuning};
use crate::screens::Holes;
use crate::set::{SetFile, TableFile, file_sha1};
use crate::steps::table as table_step;
use crate::{Board, Inputs, Nameplate};

/// The run's pot holes the closed-loop check's pot steps leave out: those
/// of the table step's result the gathered inputs `measured` (a
/// `measured.json`) name as their table's source, read again and checked
/// against its sha1. None, said, where the inputs hold no table or name no
/// table step's result. Refused where that result is gone or changed.
pub fn run_holes(fs: &mut dyn Store, measured: &serde_json::Value) -> Result<Holes> {
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
    let bytes = read(fs, p).context(
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
pub fn board_in(fs: &mut dyn Store, dir: &Path) -> Result<(Record, Board, Vec<u8>)> {
    let p = dir.join(board::FILE);
    if !fs.exists(&p) {
        bail!(
            "{} is not there: the board's facts are read off the servo, so rerun step 0 on the servo",
            p.display()
        );
    }
    let bytes = read(fs, &p)?;
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
pub fn inputs(
    fs: &mut dyn Store,
    profile: &Path,
    policy: Option<&Path>,
    board: Option<&BoardFacts>,
) -> Result<Loaded> {
    let policy = match policy {
        Some(p) => Some((p, read(fs, p)?)),
        None => None,
    };
    let bytes = read(fs, profile)?;
    inputs_of(fs, profile, &bytes, policy, board)
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
fn beside_v0(fs: &mut dyn Store, profile: &Path) -> Result<Option<Beside>> {
    let path = profile.with_file_name("thermal.json");
    if !fs.exists(&path) {
        return Ok(None);
    }
    let bytes = read(fs, &path)?;
    let heat = file_of::thermal(utf8(&path, &bytes)?)
        .map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))?;
    Ok(Some(Beside { path, bytes, heat }))
}

/// [`inputs`] with the profile's bytes and the policy's read.
pub fn inputs_of(
    fs: &mut dyn Store,
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
        ProfileIn::V0 { .. } => beside_v0(fs, profile)?,
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
pub fn nameplate(fs: &mut dyn Store, path: &Path, policy: Option<&Path>) -> Result<Nameplate> {
    Ok(inputs(fs, path, policy, None)?.resolved.nameplate)
}

pub fn read(fs: &mut dyn Store, path: &Path) -> Result<Vec<u8>> {
    fs.read_file(path)
        .map_err(anyhow::Error::msg)
        .with_context(|| format!("reading {}", path.display()))
}

fn json<T: DeserializeOwned>(path: &Path, bytes: &[u8]) -> Result<T> {
    serde_json::from_slice(bytes).with_context(|| format!("parsing {}", path.display()))
}

pub fn set(fs: &mut dyn Store, path: &Path) -> Result<SetFile> {
    let text = String::from_utf8(read(fs, path)?).context("a set file is UTF-8")?;
    SetFile::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))
}

pub fn table(fs: &mut dyn Store, path: &Path) -> Result<TableFile> {
    let text = String::from_utf8(read(fs, path)?).context("a table file is UTF-8")?;
    TableFile::parse(&text).map_err(|e| anyhow::anyhow!("{}: {e}", path.display()))
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
pub fn rules_inputs(fs: &mut dyn Store, dir: &Path, policy: Option<&Path>) -> Result<RulesInputs> {
    let (record, board, board_bytes) = board_in(fs, dir)?;
    let mut files = vec![(
        "board".to_string(),
        dir.join(board::FILE).display().to_string(),
        file_sha1(&board_bytes),
    )];
    let mut take = |fs: &mut dyn Store, name: &str, path: PathBuf| -> Result<Option<Vec<u8>>> {
        if !fs.exists(&path) {
            return Ok(None);
        }
        let bytes = read(fs, &path)?;
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
    let mut measured: Measured =
        json(&p, &need("measured", take(fs, "measured", p.clone())?, &p)?)?;
    let p = dir.join("shelf.json");
    if let Some(b) = take(fs, "shelf", p.clone())? {
        measured.shelf = Some(json::<ShelfRecord>(&p, &b)?);
    }
    let np_path = dir.join("nameplate.json");
    let np_bytes = need(
        "nameplate",
        take(fs, "nameplate", np_path.clone())?,
        &np_path,
    )?;
    let q = policy.map_or_else(|| dir.join("policy.json"), Path::to_path_buf);
    let q_bytes = need("policy", take(fs, "policy", q.clone())?, &q)?;
    let l = inputs_of(
        fs,
        &np_path,
        &np_bytes,
        Some((&q, q_bytes)),
        Some(&board.facts()),
    )?;
    let p = dir.join("thermal.json");
    match &l.thermal_json {
        Some(_) => {
            take(fs, "thermal", p)?;
        }
        None if fs.exists(&p) => bail!(
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
    let table = match take(fs, "pos_table", p.clone())? {
        Some(b) => Some(
            TableFile::parse(std::str::from_utf8(&b).context("UTF-8")?)
                .map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?,
        ),
        None => None,
    };
    let p = dir.join("wide-codes.json");
    let slots = match take(fs, "wide_codes", p.clone())? {
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

/// `<out>/<servo>/<UTC time>/` at `now`, unix seconds, refused when it
/// exists.
pub fn run_folder(fs: &dyn Store, out: &Path, servo: &str, now: f64) -> Result<PathBuf> {
    let p = out.join(servo).join(utc(now));
    if fs.exists(&p) {
        bail!("{} exists", p.display());
    }
    Ok(p)
}

/// `YYYYMMDDTHHMMSSZ` of `t`, unix seconds, the second it falls in.
pub fn utc(t: f64) -> String {
    let secs = t.max(0.0) as i64;
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
