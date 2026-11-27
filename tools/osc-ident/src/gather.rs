//! `gather`: a run folder's step results to an inputs folder the rules
//! read. Touches no servo.

use std::path::Path;

use anyhow::{Context, Result, bail};
use osc_measure::board;
use osc_measure::gather::{
    self, CRUISE_RESULT, RunResults, SPINS_RESULT, StepResult, TABLE_RESULT, WINDOW_RESULT,
};
use osc_measure::inputs::ThermalInputs;
use osc_measure::sink::Sink;
use osc_measure::steps::{probe, rest, stops};
use serde::de::DeserializeOwned;

use crate::files::{self, DirSink};

fn result(run: &Path, rel: &str) -> Result<Option<StepResult>> {
    let p = run.join(rel);
    if !p.exists() {
        return Ok(None);
    }
    let bytes = files::read(&p)?;
    Ok(Some(
        StepResult::parse(&p.display().to_string(), &bytes).map_err(|e| anyhow::anyhow!("{e}"))?,
    ))
}

fn parsed<T: DeserializeOwned>(p: &Path, bytes: &[u8]) -> Result<T> {
    serde_json::from_slice(bytes).with_context(|| format!("parsing {}", p.display()))
}

/// The inputs folder `out` from the run folder `run`: `measured.json`, the
/// table `pos_lut.json` (the table step's, else straight), the table
/// step's slots `wide-codes.json` where it ran, the profile (as
/// `nameplate.json`), the policy and the thermal values as given,
/// `resolved.json`, the policy's values as this run resolved them, and
/// step 0's `board.json`, which is there already when `out` is the run's
/// own `inputs/`. The lines it prints.
pub fn gather(
    run: &Path,
    nameplate: &Path,
    thermal: Option<&Path>,
    policy: &Path,
    out: &Path,
) -> Result<Vec<String>> {
    let own = run.join("inputs");
    let (_, board, board_bytes) = files::board_in(&own)?;
    let loaded = files::inputs(nameplate, Some(policy), Some(&board.facts()))?;
    let np = &loaded.resolved.nameplate;
    let np_bytes = files::read(nameplate)?;
    let policy_bytes = files::read(policy)?;
    let mut resolved = serde_json::to_value(&loaded.resolved.report)?;
    resolved["notes"] = loaded.notes.clone().into();
    let mut resolved = serde_json::to_vec_pretty(&resolved)?;
    resolved.push(b'\n');
    let thermal_bytes = match thermal {
        Some(p) => {
            let b = files::read(p)?;
            parsed::<ThermalInputs>(p, &b)?;
            Some(b)
        }
        None => None,
    };
    let results = RunResults {
        rest: result(run, rest::RESULT)?,
        probe: result(run, probe::RESULT)?,
        stops: result(run, stops::RESULT)?,
        window: result(run, WINDOW_RESULT)?,
        spins: result(run, SPINS_RESULT)?,
        cruise: result(run, CRUISE_RESULT)?,
        table: result(run, TABLE_RESULT)?,
    };
    let g = gather::gather(&results, np).map_err(|e| anyhow::anyhow!("{e}"))?;
    let into_own = out.exists() && same(out, &own)?;
    if out.exists() && !(into_own && only_board(out)?) {
        bail!(
            "{} exists: an inputs folder is gathered into a new one",
            out.display()
        );
    }
    let mut sink = DirSink {
        root: out.to_path_buf(),
    };
    if !into_own {
        sink.write(board::FILE, &board_bytes)?;
    }
    sink.write("measured.json", gather::render(&g.measured).as_bytes())?;
    sink.write("pos_lut.json", gather::render(&g.table).as_bytes())?;
    sink.write("nameplate.json", &np_bytes)?;
    sink.write("policy.json", &policy_bytes)?;
    sink.write("resolved.json", &resolved)?;
    let mut wrote = vec![
        "measured.json",
        "pos_lut.json",
        "nameplate.json",
        "policy.json",
        "resolved.json",
    ];
    if !into_own {
        wrote.push(board::FILE);
    }
    if let Some(w) = &g.wide_codes {
        sink.write("wide-codes.json", gather::render(w).as_bytes())?;
        wrote.push("wide-codes.json");
    }
    if let Some(b) = &thermal_bytes {
        sink.write("thermal.json", b)?;
        wrote.push("thermal.json");
    }
    let mut lines = loaded.notes;
    lines.extend(g.lines);
    lines.push(format!("wrote {}: {}", out.display(), wrote.join(", ")));
    Ok(lines)
}

fn same(a: &Path, b: &Path) -> Result<bool> {
    let canon = |p: &Path| p.canonicalize().with_context(|| format!("{}", p.display()));
    Ok(b.exists() && canon(a)? == canon(b)?)
}

/// The run's own `inputs/` as step 0 left it: its board file and nothing
/// a gather writes.
fn only_board(dir: &Path) -> Result<bool> {
    for e in std::fs::read_dir(dir).with_context(|| format!("{}", dir.display()))? {
        if e?.file_name() != board::FILE {
            return Ok(false);
        }
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::*;

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

    /// A run folder of the bench servo's results gathers into an inputs
    /// folder that `rules` reads; with no stops result it is refused
    /// naming the step, and nothing is written.
    #[test]
    fn gather_writes_an_inputs_folder_the_rules_read() {
        let dir = scratch("gather");
        let run = dir.join("run");
        std::fs::create_dir_all(run.join("results")).expect("mkdir");
        files::step_0(&run);
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                run.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let out = dir.join("inputs");
        let lines = gather(
            &run,
            &testdata("mg90-d/nameplate.json"),
            Some(&testdata("mg90-d/thermal.json")),
            &testdata("mg90-d/policy.json"),
            &out,
        )
        .expect("gathers");
        let text = lines.join("\n");
        assert!(text.contains("stops 107 to 3729 counts"), "{text}");
        for f in [
            "measured.json",
            "pos_lut.json",
            "nameplate.json",
            "policy.json",
            "resolved.json",
            "thermal.json",
        ] {
            assert!(out.join(f).exists(), "{f}");
        }
        let resolved: serde_json::Value =
            serde_json::from_slice(&files::read(&out.join("resolved.json")).expect("read"))
                .expect("json");
        assert_eq!(resolved["profile"]["key"], "mg90-d");
        assert!(resolved["notes"][0].as_str().unwrap().contains("version 0"));
        let line = |field: &str| {
            resolved["fields"]
                .as_array()
                .expect("fields")
                .iter()
                .find(|l| l["field"] == field)
                .cloned()
                .unwrap_or_else(|| panic!("{field}"))
        };
        assert_eq!(line("motion.move_speed")["value"], 200.0);
        assert_eq!(
            line("protection.speed_ceiling")["value"],
            serde_json::json!([2, 3])
        );
        assert_eq!(line("oc_trip")["form"], "board");
        assert!(
            gather(
                &run,
                &testdata("mg90-d/nameplate.json"),
                None,
                &testdata("mg90-d/policy.json"),
                &out,
            )
            .is_err(),
            "never writes over an inputs folder"
        );
        let r = files::rules_inputs(&out, None).expect("the rules read it");
        assert_eq!(r.table.as_ref().map(|t| t.stops), Some((107, 3729)));
        // without a gain the speed loop is held: `rules` still writes a set
        let set_out = dir.join("set.json");
        crate::rules(&out, None, None, &set_out).expect("rules");
        let set = files::set(&set_out).expect("set");
        assert_eq!(set.table().0, "IDENTITY");
        assert!(set.value("v_kp_q88").is_none() && set.value("r_q12").is_some());

        std::fs::remove_file(run.join("results/stops.json")).expect("rm");
        let e = gather(
            &run,
            &testdata("mg90-d/nameplate.json"),
            None,
            &testdata("mg90-d/policy.json"),
            &dir.join("other"),
        )
        .expect_err("refused");
        assert!(format!("{e:#}").contains("osc-ident step stops"), "{e:#}");
        assert!(!dir.join("other").exists());
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A run folder without step 0's board file is refused before anything
    /// is written, and so is an inputs folder the rules would read.
    #[test]
    fn a_run_folder_without_step_0s_board_is_refused() {
        let dir = scratch("gather-no-board");
        let run = dir.join("run");
        std::fs::create_dir_all(run.join("results")).expect("mkdir");
        let out = dir.join("inputs");
        let e = gather(
            &run,
            &testdata("mg90-d/nameplate.json"),
            None,
            &testdata("mg90-d/policy.json"),
            &out,
        )
        .expect_err("refused");
        assert!(
            format!("{e:#}").contains("rerun step 0 on the servo"),
            "{e:#}"
        );
        assert!(!out.exists());
        let e = files::rules_inputs(&run, None).err().expect("refused");
        assert!(
            format!("{e:#}").contains("rerun step 0 on the servo"),
            "{e:#}"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The record's version 1 profile and policy gather into an inputs
    /// folder whose set is the version 0 files' own, every field byte for
    /// byte; only the input files' names and hashes differ.
    #[test]
    fn a_version_1_profile_and_policy_gather_to_the_version_0_set() {
        let dir = scratch("gather-v1");
        let run = dir.join("run");
        std::fs::create_dir_all(run.join("results")).expect("mkdir");
        files::step_0(&run);
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                run.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let mut sets = Vec::new();
        for (profile, policy, name) in [
            ("mg90-d/nameplate.json", "mg90-d/policy.json", "v0"),
            ("mg90-d/profile.json", "mg90-d/policy-of-record.json", "v1"),
        ] {
            let out = dir.join(name);
            gather(
                &run,
                &testdata(profile),
                Some(&testdata("mg90-d/thermal.json")),
                &testdata(policy),
                &out,
            )
            .expect("gathers");
            let set_out = dir.join(format!("{name}.json"));
            crate::rules(&out, None, Some("pair"), &set_out).expect("rules");
            let text = files::set(&set_out).expect("set").render();
            let fields = text.split_once("\n \"fields\": {").expect("fields").1;
            sets.push(fields.to_string());
        }
        assert_eq!(sets[0], sets[1]);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// With the table step's result in the run, the inputs folder holds
    /// its points and its slots, and `rules` writes a set whose table is
    /// LIVE and whose slots are the step's.
    #[test]
    fn gather_writes_the_table_steps_table_and_slots() {
        let dir = scratch("gather-table");
        let run = dir.join("run");
        std::fs::create_dir_all(run.join("results")).expect("mkdir");
        files::step_0(&run);
        for f in ["rest", "probe", "stops", "window", "spins", "table"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                run.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let out = dir.join("inputs");
        let lines = gather(
            &run,
            &testdata("mg90-d/nameplate.json"),
            Some(&testdata("mg90-d/thermal.json")),
            &testdata("mg90-d/policy.json"),
            &out,
        )
        .expect("gathers");
        assert!(
            lines.iter().any(|l| l.contains("wide-codes.json")),
            "{lines:?}"
        );
        let r = files::rules_inputs(&out, None).expect("the rules read it");
        let t = r.table.as_ref().expect("the table");
        assert!(t.points.iter().any(|&k| k != 0));
        assert_eq!(
            r.slots.as_deref(),
            Some(&[0, 48, 0, 80, 0, 0, 0, 88, 0, 48, 0, 0, 0, 0, 0, 0][..])
        );
        let set_out = dir.join("set.json");
        crate::rules(&out, None, None, &set_out).expect("rules");
        let set = files::set(&set_out).expect("set");
        assert_eq!(set.table().0, "LIVE");
        assert_eq!(set.slots().expect("slots").as_deref(), r.slots.as_deref());
        let _ = std::fs::remove_dir_all(dir);
    }
}
