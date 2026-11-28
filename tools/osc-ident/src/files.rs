//! The tool's files on disk: the [`Store`] the run reads and writes
//! through, and the inputs, set and table files and run folders over it,
//! by their paths.

use std::path::{Path, PathBuf};

use anyhow::Result;
use osc_measure::run::Store;
use osc_measure::run::files as of;
use osc_measure::set::{SetFile, TableFile};
#[cfg(test)]
use {
    osc_ident_inputs::BoardFacts,
    osc_measure::board::Record,
    osc_measure::run::files::Loaded,
    osc_measure::screens::Holes,
    osc_measure::set::file_sha1,
    osc_measure::steps::table as table_step,
    osc_measure::{Board, Nameplate},
    std::time::{SystemTime, UNIX_EPOCH},
};

#[cfg(test)]
pub use osc_measure::run::files::BOARD;
pub use osc_measure::run::files::RulesInputs;

/// The file system, paths as the user gives them.
pub struct Fs;

impl Store for Fs {
    fn read_file(&mut self, path: &Path) -> Result<Vec<u8>, String> {
        std::fs::read(path).map_err(|e| e.to_string())
    }

    fn write_file(&mut self, path: &Path, bytes: &[u8]) -> Result<(), String> {
        std::fs::write(path, bytes).map_err(|e| e.to_string())
    }

    fn make_dirs(&mut self, path: &Path) -> Result<(), String> {
        std::fs::create_dir_all(path).map_err(|e| e.to_string())
    }

    fn exists(&self, path: &Path) -> bool {
        path.exists()
    }

    fn entries(&self, path: &Path) -> Result<Vec<(String, bool)>, String> {
        let mut out = Vec::new();
        for e in std::fs::read_dir(path).map_err(|e| e.to_string())? {
            let name = e.map_err(|e| e.to_string())?.file_name();
            let is_dir = path.join(&name).is_dir();
            out.push((name.to_string_lossy().into_owned(), is_dir));
        }
        Ok(out)
    }

    fn canonical(&self, path: &Path) -> Result<PathBuf, String> {
        path.canonicalize().map_err(|e| e.to_string())
    }
}

/// The run's pot holes the closed-loop check's pot steps leave out
/// ([`of::run_holes`]).
#[cfg(test)]
pub fn run_holes(measured: &serde_json::Value) -> Result<Holes> {
    of::run_holes(&mut Fs, measured)
}

/// The board record in the inputs folder `dir`, refused where step 0 wrote
/// none.
#[cfg(test)]
pub fn board_in(dir: &Path) -> Result<(Record, Board, Vec<u8>)> {
    of::board_in(&mut Fs, dir)
}

#[cfg(test)]
fn inputs_of(
    profile: &Path,
    bytes: &[u8],
    policy: Option<(&Path, Vec<u8>)>,
    board: Option<&BoardFacts>,
) -> Result<Loaded> {
    of::inputs_of(&mut Fs, profile, bytes, policy, board)
}

/// The steps' nameplate: the profile resolved with the policy, where one is
/// named.
#[cfg(test)]
pub fn nameplate(path: &Path, policy: Option<&Path>) -> Result<Nameplate> {
    of::nameplate(&mut Fs, path, policy)
}

pub fn read(path: &Path) -> Result<Vec<u8>> {
    of::read(&mut Fs, path)
}

pub fn set(path: &Path) -> Result<SetFile> {
    of::set(&mut Fs, path)
}

#[cfg(test)]
pub fn table(path: &Path) -> Result<TableFile> {
    of::table(&mut Fs, path)
}

/// The table file for a set: the one named, else the notebooks' layout,
/// `<pair>.pos_lut.json` beside the set, where there is one.
pub fn table_for(
    set_path: &Path,
    set: &SetFile,
    named: Option<&Path>,
) -> Result<Option<TableFile>> {
    if let Some(p) = named {
        return of::table(&mut Fs, p).map(Some);
    }
    let beside = set_path
        .parent()
        .unwrap_or(Path::new("."))
        .join(format!("{}.pos_lut.json", set.pair()));
    if beside.exists() {
        of::table(&mut Fs, &beside).map(Some)
    } else {
        Ok(None)
    }
}

/// What the rules read from the inputs folder `dir` ([`of::rules_inputs`]).
pub fn rules_inputs(dir: &Path, policy: Option<&Path>) -> Result<RulesInputs> {
    of::rules_inputs(&mut Fs, dir, policy)
}

/// `<out>/<servo>/<UTC time>/`, refused when it exists.
pub fn run_folder(out: &Path, servo: &str) -> Result<PathBuf> {
    of::run_folder(&Fs, out, servo, crate::unix_s())
}

/// `YYYYMMDDTHHMMSSZ`.
#[cfg(test)]
pub fn utc(t: SystemTime) -> String {
    of::utc(t.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs()) as f64)
}

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

    /// A version 0 policy beside a version 1 profile is refused naming both
    /// files and the pairs that load: a version 0 policy holds no run
    /// voltage, which only a version 0 nameplate carries to it.
    #[test]
    fn a_version_0_policy_beside_a_version_1_profile_is_refused_plainly() {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../measure/testdata/mg90-d");
        let profile = dir.join("profile.json");
        let policy = dir.join("policy.json");
        let e = inputs_of(
            &profile,
            &read(&profile).expect("read"),
            Some((&policy, read(&policy).expect("read"))),
            None,
        )
        .err()
        .expect("refused");
        assert_eq!(
            format!("{e:#}"),
            format!(
                "{} is a version 1 profile and {} a version 0 policy: a version 1 profile takes a version 1 policy, such as policy-of-record.json; to use a version 0 pair, pass the version 0 nameplate",
                profile.display(),
                policy.display()
            )
        );
        let record = dir.join("policy-of-record.json");
        inputs_of(
            &profile,
            &read(&profile).expect("read"),
            Some((&record, read(&record).expect("read"))),
            None,
        )
        .expect("the version 1 pair loads");
        let nameplate = dir.join("nameplate.json");
        inputs_of(
            &nameplate,
            &read(&nameplate).expect("read"),
            Some((&policy, read(&policy).expect("read"))),
            None,
        )
        .expect("the version 0 pair loads");
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
        // the run's soft pair is its policy's old share of the stops' span,
        // centered: the same window as a soft travel binds it again
        let measured: serde_json::Value =
            serde_json::from_slice(&read(&dir.join("measured.json")).expect("read")).expect("json");
        let soft = |f: &str| recorded.value(f).and_then(|v| v.as_f64()).expect(f);
        let width = (soft("pos_max_soft_counts") - soft("pos_min_soft_counts"))
            / measured["table"]["counts_per_deg"].as_f64().expect("scale");
        let mut policy: serde_json::Value =
            serde_json::from_slice(&read(&dir.join("policy.json")).expect("read")).expect("json");
        policy["soft_travel_deg"] = width.into();
        std::fs::write(dir.join("policy.json"), policy.to_string()).expect("policy");
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
