//! `gather`: a run folder's step results to an inputs folder the rules
//! read ([`osc_measure::run::gather`]). Touches no servo.

use std::path::Path;

#[cfg(test)]
use crate::files;
use crate::files::Fs;
use anyhow::Result;

pub fn gather(run: &Path, nameplate: &Path, policy: &Path, out: &Path) -> Result<Vec<String>> {
    osc_measure::run::gather::gather(&mut Fs, run, nameplate, policy, out)
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
            &testdata("mg90-d/policy.json"),
            &out,
        )
        .expect("gathers");
        let text = lines.join("\n");
        assert!(text.contains("stops 96 to 3744 counts"), "{text}");
        assert!(
            text.contains("cross-check stall current at 4.8 V, V / R"),
            "{text}"
        );
        for f in [
            "measured.json",
            "pos_lut.json",
            "cross-checks.json",
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
                &testdata("mg90-d/policy.json"),
                &out,
            )
            .is_err(),
            "never writes over an inputs folder"
        );
        let r = files::rules_inputs(&out, None).expect("the rules read it");
        assert_eq!(r.table.as_ref().map(|t| t.stops), Some((96, 3744)));
        // without a gain the speed loop is held: `rules` still writes a set
        let set_out = dir.join("set.json");
        crate::rules(&out, None, None, &set_out).expect("rules");
        let set = files::set(&set_out).expect("set");
        assert_eq!(set.table().0, "LIVE");
        assert!(set.value("v_kp_q88").is_none() && set.value("r_q12").is_some());

        std::fs::remove_file(run.join("results/stops.json")).expect("rm");
        let e = gather(
            &run,
            &testdata("mg90-d/nameplate.json"),
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
        for f in ["rest", "probe", "stops", "window", "spins", "table"] {
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
            gather(&run, &testdata(profile), &testdata(policy), &out).expect("gathers");
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
