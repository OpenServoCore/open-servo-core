//! `gather`: a run folder's step results to an inputs folder the rules
//! read. Touches no servo.

use std::path::Path;

use anyhow::{Context, Result, bail};
use osc_measure::gather::{
    self, CRUISE_RESULT, RunResults, SPINS_RESULT, StepResult, TABLE_RESULT, WINDOW_RESULT,
};
use osc_measure::inputs::ThermalInputs;
use osc_measure::steps::{probe, rest, stops};
use osc_measure::{Policy, sink::Sink};
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
/// step's slots `wide-codes.json` where it ran, and the nameplate, the
/// policy and the thermal values as given. The lines it prints.
pub fn gather(
    run: &Path,
    nameplate: &Path,
    thermal: Option<&Path>,
    policy: &Path,
    out: &Path,
) -> Result<Vec<String>> {
    let np = files::nameplate(nameplate)?;
    let np_bytes = files::read(nameplate)?;
    let policy_bytes = files::read(policy)?;
    parsed::<Policy>(policy, &policy_bytes)?;
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
    let g = gather::gather(&results, &np).map_err(|e| anyhow::anyhow!("{e}"))?;
    if out.exists() {
        bail!(
            "{} exists: an inputs folder is gathered into a new one",
            out.display()
        );
    }
    let mut sink = DirSink {
        root: out.to_path_buf(),
    };
    sink.write("measured.json", gather::render(&g.measured).as_bytes())?;
    sink.write("pos_lut.json", gather::render(&g.table).as_bytes())?;
    sink.write("nameplate.json", &np_bytes)?;
    sink.write("policy.json", &policy_bytes)?;
    let mut wrote = vec![
        "measured.json",
        "pos_lut.json",
        "nameplate.json",
        "policy.json",
    ];
    if let Some(w) = &g.wide_codes {
        sink.write("wide-codes.json", gather::render(w).as_bytes())?;
        wrote.push("wide-codes.json");
    }
    if let Some(b) = &thermal_bytes {
        sink.write("thermal.json", b)?;
        wrote.push("thermal.json");
    }
    let mut lines = g.lines;
    lines.push(format!("wrote {}: {}", out.display(), wrote.join(", ")));
    Ok(lines)
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
            "thermal.json",
        ] {
            assert!(out.join(f).exists(), "{f}");
        }
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
        let r = files::rules_inputs(&out, None, None).expect("the rules read it");
        assert_eq!(r.table.as_ref().map(|t| t.stops), Some((107, 3729)));
        // without a gain the speed loop is held: `rules` still writes a set
        let set_out = dir.join("set.json");
        crate::rules(&out, None, None, None, &set_out).expect("rules");
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

    /// With the table step's result in the run, the inputs folder holds
    /// its points and its slots, and `rules` writes a set whose table is
    /// LIVE and whose slots are the step's.
    #[test]
    fn gather_writes_the_table_steps_table_and_slots() {
        let dir = scratch("gather-table");
        let run = dir.join("run");
        std::fs::create_dir_all(run.join("results")).expect("mkdir");
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
        let r = files::rules_inputs(&out, None, None).expect("the rules read it");
        let t = r.table.as_ref().expect("the table");
        assert!(t.points.iter().any(|&k| k != 0));
        assert_eq!(
            r.slots.as_deref(),
            Some(&[0, 48, 0, 80, 0, 0, 0, 88, 0, 48, 0, 0, 0, 0, 0, 0][..])
        );
        let set_out = dir.join("set.json");
        crate::rules(&out, None, None, None, &set_out).expect("rules");
        let set = files::set(&set_out).expect("set");
        assert_eq!(set.table().0, "LIVE");
        assert_eq!(set.slots().expect("slots").as_deref(), r.slots.as_deref());
        let _ = std::fs::remove_dir_all(dir);
    }
}
