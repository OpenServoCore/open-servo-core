//! `run`: the whole identification in one command. The measurement chain
//! through the spins (`step spins`'s own: rest, probe, stops, table,
//! window, spins), then, every step accepted, the
//! gather into `<run folder>/inputs/` and the tune into
//! `<run folder>/tune/`; on the set the tune accepts, the cruise step, the
//! gather again and the final set's check into `<run folder>/final/`
//! ([`crate::finish`]). Every argument is checked before the bus is
//! touched.

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::Policy;
use serde_json::{Value, json};

use crate::files;
use crate::steps::{self, StepInputs};

/// What `run` was given.
pub struct RunArgs<'a> {
    pub nameplate: &'a Path,
    pub thermal: &'a Path,
    pub policy: &'a Path,
    pub out: &'a Path,
    pub tries: u32,
}

fn unix_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// The refusals that need no servo: the three files parse and `--tries`
/// is at least 1.
pub fn admit(a: &RunArgs) -> Result<StepInputs> {
    if a.tries == 0 {
        bail!("--tries is at least 1");
    }
    let policy = files::read(a.policy)?;
    serde_json::from_slice::<Policy>(&policy)
        .with_context(|| format!("parsing {}", a.policy.display()))?;
    StepInputs::load(Some(a.nameplate), Some(a.thermal), None)
}

/// The command line that runs `run` again as given.
fn again(id: Id, a: &RunArgs) -> String {
    format!(
        "osc-ident --id {} run --nameplate {} --thermal {} --policy {} --out {} --tries {}",
        id.as_byte(),
        a.nameplate.display(),
        a.thermal.display(),
        a.policy.display(),
        a.out.display(),
        a.tries
    )
}

/// The run folder's manifest with the gather and the tune beside its
/// steps.
fn record(root: &Path, gather: Value, tune: Value) -> Result<()> {
    let p = root.join("run.json");
    let mut doc: Value = serde_json::from_slice(&files::read(&p)?)
        .with_context(|| format!("parsing {}", p.display()))?;
    doc["gather"] = gather;
    doc["tune"] = tune;
    let mut text = serde_json::to_vec_pretty(&doc)?;
    text.push(b'\n');
    std::fs::write(&p, text).with_context(|| format!("writing {}", p.display()))
}

/// The cruise step and the final check beside the tune in `run.json`.
fn record_finish(root: &Path, cruise: Value, last: Value) -> Result<()> {
    let p = root.join("run.json");
    let mut doc: Value = serde_json::from_slice(&files::read(&p)?)
        .with_context(|| format!("parsing {}", p.display()))?;
    doc["cruise"] = cruise;
    doc["final"] = last;
    let mut text = serde_json::to_vec_pretty(&doc)?;
    text.push(b'\n');
    std::fs::write(&p, text).with_context(|| format!("writing {}", p.display()))
}

fn not_run(why: &str) -> Value {
    json!({"state": "not run", "why": why})
}

/// `run` on the servo: the lines it prints, and whether the tune accepted
/// a set; the final set's verdict is in the lines and in `run.json`. The
/// arguments are [`admit`]ted before anything goes on the bus.
#[allow(clippy::too_many_arguments)]
pub async fn run<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    let inputs = admit(a)?;
    let (mut lines, measured, root) =
        steps::step_spins(c, id, d, &inputs, a.out, servo, stop).await?;
    if !measured {
        let why = "not every step was accepted";
        record(&root, not_run(why), not_run(why))?;
        record_finish(&root, not_run(why), not_run(why))?;
        lines.push(format!(
            "the run stops: {why}, so nothing is gathered or tuned; to run it again: {}",
            again(id, a)
        ));
        return Ok((lines, false));
    }
    let (more, accepted) = gather_and_tune(c, id, d, a, &root, stop).await?;
    lines.extend(more);
    if !accepted {
        let why = "the tune accepted no set";
        record_finish(&root, not_run(why), not_run(why))?;
        return Ok((lines, false));
    }
    match crate::finish::finish(c, id, d, a, &root, stop).await {
        Ok(f) => {
            record_finish(&root, f.cruise, f.last)?;
            lines.extend(f.lines);
        }
        Err(e) => {
            let why = format!("the run folder could not be read for the cruise: {e:#}");
            record_finish(&root, not_run(&why), not_run(&why))?;
            lines.push(format!("{why}; nothing was written, the tune's set stays"));
        }
    }
    Ok((lines, accepted))
}

/// The gather of the run folder `root`, its steps all accepted, into
/// `inputs/`, then the tune from it into `tune/`, both recorded in its
/// `run.json`.
async fn gather_and_tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    root: &Path,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    let mut lines = Vec::new();
    let inputs_dir = root.join("inputs");
    let t0 = unix_s();
    let gathered = crate::gather::gather(root, a.nameplate, Some(a.thermal), a.policy, &inputs_dir);
    let t1 = unix_s();
    let gather_files = [
        "measured.json",
        "pos_lut.json",
        "nameplate.json",
        "policy.json",
        "thermal.json",
    ]
    .map(|f| format!("inputs/{f}"));
    match gathered {
        Err(e) => {
            let why = format!("{e:#}");
            record(
                root,
                json!({"state": "refused", "why": why, "started": t0, "ended": t1}),
                not_run("the gather was refused"),
            )?;
            lines.push(format!("gather refused: {why}"));
            lines.push("the run stops: nothing is tuned".into());
            return Ok((lines, false));
        }
        Ok(g) => {
            lines.push(format!("gather into {}:", inputs_dir.display()));
            lines.extend(g.into_iter().map(|l| format!("  {l}")));
        }
    }
    let gather = json!({"state": "accepted", "why": null, "started": t0, "ended": t1, "files": gather_files});

    let tune_dir = root.join("tune");
    let tuned = crate::tune::tune(c, id, d, &inputs_dir, &tune_dir, a.tries, stop).await;
    let t2 = unix_s();
    match tuned {
        Err(e) => {
            let why = format!("{e:#}");
            record(
                root,
                gather,
                json!({"state": "refused", "why": why, "started": t1, "ended": t2}),
            )?;
            lines.push(format!("tune refused: {why}"));
            lines.push(format!(
                "nothing was written; the measured inputs are kept: osc-ident --id {} tune --inputs {} --out {} --tries {} continues from them",
                id.as_byte(),
                inputs_dir.display(),
                tune_dir.display(),
                a.tries
            ));
            Ok((lines, false))
        }
        Ok((tune_lines, accepted)) => {
            let mut files = vec!["tune/tune.json".to_string()];
            if accepted {
                files.push("tune/set.json".into());
            }
            record(
                root,
                gather,
                json!({"state": if accepted { "accepted" } else { "refused" }, "why": null,
                       "started": t1, "ended": t2, "files": files}),
            )?;
            lines.push(format!("tune into {}:", tune_dir.display()));
            lines.extend(tune_lines);
            Ok((lines, accepted))
        }
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use futures_lite::future::block_on;
    use osc_client::BaudRate;
    use osc_client::fake::{FakePipe, TelSample, TelemetryTick};

    use super::*;

    const ID: Id = Id::new(1);

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

    /// A servo at rest, a count or so of noise on each channel.
    fn resting() -> Vec<TelSample> {
        let noise = |i: u32, k: u32| {
            let mut h = i.wrapping_mul(0x9E37_79B9) ^ k.wrapping_mul(0x85EB_CA6B);
            h ^= h >> 15;
            h = h.wrapping_mul(0x2C1B_3C6D);
            h ^= h >> 12;
            (h % 3) as i32 - 1
        };
        (0..4096u32)
            .map(|i| {
                let at = |base: i32, k: u32| (base + noise(i, k)) as u16;
                TelSample {
                    tick: TelemetryTick {
                        _scan: [0; 9],
                        pos: at(2041, 1),
                        vbus_raw: at(1241, 3),
                        ntc_raw: at(2010, 4),
                        current_raw: at(1206, 5),
                        vmotor_a: at(773, 6),
                        vmotor_b: at(772, 7),
                        duty_ticks: 0,
                        current_trough: at(1206, 8),
                        i_tick_counts: 0,
                        vdiff_counts: 0,
                        vmotor_a_trough: at(773, 9),
                        vmotor_b_trough: at(772, 10),
                        tick_flags: 0,
                        sample_tick: i,
                    },
                    vbus_counts: 2000,
                }
            })
            .collect()
    }

    fn fake() -> (Client<FakePipe>, Descriptor) {
        let mut pipe = FakePipe::new(BaudRate::B3000000, &[1]);
        pipe.seed_calibrated(0);
        let mut c = block_on(Client::connect(pipe)).expect("connect");
        c.pipe_mut().set_track(0, resting());
        (c, crate::descriptor_builtin().expect("descriptor"))
    }

    struct Given {
        nameplate: PathBuf,
        thermal: PathBuf,
    }

    fn given() -> Given {
        Given {
            nameplate: testdata("mg90-d/nameplate.json"),
            thermal: testdata("mg90-d/thermal.json"),
        }
    }

    fn args<'a>(g: &'a Given, out: &'a Path, policy: &'a Path) -> RunArgs<'a> {
        RunArgs {
            nameplate: &g.nameplate,
            thermal: &g.thermal,
            policy,
            out,
            tries: 6,
        }
    }

    /// The simulated servo runs no shunt burst, so the probe aborts: the
    /// run stops there, gathers and tunes nothing, records both as not run,
    /// and names the command that runs it again.
    #[test]
    fn a_refused_step_stops_the_run_with_nothing_gathered() {
        let (mut c, d) = fake();
        let dir = scratch("run-stops");
        let policy = testdata("mg90-d/policy.json");
        let g = given();
        let a = args(&g, &dir, &policy);
        let (lines, accepted) =
            block_on(run(&mut c, ID, &d, &a, "uid", &AtomicBool::new(false))).expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        let last = lines.last().expect("a line");
        assert!(
            last.contains("nothing is gathered or tuned")
                && last.contains(&format!(
                    "osc-ident --id 1 run --nameplate {}",
                    testdata("mg90-d/nameplate.json").display()
                ))
                && last.contains("--tries 6"),
            "{last}"
        );
        let root = std::fs::read_dir(dir.join("uid"))
            .expect("the run folder")
            .next()
            .expect("one")
            .expect("entry")
            .path();
        assert!(!root.join("inputs").exists() && !root.join("tune").exists());
        let m: Value = serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
            .expect("json");
        assert_eq!(m["steps"]["probe"]["state"], "aborted", "{m}");
        assert_eq!(m["gather"]["state"], "not run", "{m}");
        assert_eq!(m["tune"]["state"], "not run", "{m}");
        assert_eq!(block_on(c.read(ID, 0x180, 1)).expect("read"), [0]);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The run's steps are `step spins`'s own: on two like servos both
    /// print the same step lines.
    #[test]
    fn the_run_measures_with_the_chain_step_spins_runs() {
        let dir = scratch("run-same");
        let policy = testdata("mg90-d/policy.json");
        let (mut c, d) = fake();
        let g = given();
        let out = dir.join("run");
        let a = args(&g, &out, &policy);
        let (ran, _) =
            block_on(run(&mut c, ID, &d, &a, "uid", &AtomicBool::new(false))).expect("runs");
        let (mut c, d) = fake();
        let inputs = StepInputs::load(
            Some(&testdata("mg90-d/nameplate.json")),
            Some(&testdata("mg90-d/thermal.json")),
            None,
        )
        .expect("inputs");
        let (stepped, _, root) = block_on(steps::step_spins(
            &mut c,
            ID,
            &d,
            &inputs,
            &dir.join("step"),
            "uid",
            &AtomicBool::new(false),
        ))
        .expect("runs");
        let run_root = std::fs::read_dir(dir.join("run/uid"))
            .expect("run folder")
            .next()
            .expect("one")
            .expect("entry")
            .path();
        let plain = |lines: &[String], root: &Path| -> Vec<String> {
            let r = root.display().to_string();
            lines.iter().map(|l| l.replace(&r, "<root>")).collect()
        };
        let ran = plain(&ran[..ran.len() - 1], &run_root);
        let stepped = plain(&stepped, &root);
        assert!(stepped.len() > 10, "{stepped:?}");
        assert_eq!(ran, stepped);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A run whose steps were all accepted gathers into `inputs/` and
    /// tunes into `tune/`; a tune refused before any write (torque on)
    /// ends with the command that continues without measuring again, and
    /// `run.json` records both.
    #[test]
    fn a_tune_refused_before_any_write_names_the_command_that_continues() {
        let dir = scratch("run-continue");
        let root = dir.join("uid/run");
        std::fs::create_dir_all(root.join("results")).expect("mkdir");
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                root.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        std::fs::write(
            root.join("run.json"),
            r#"{"tool": "osc-ident", "steps": {}}"#,
        )
        .expect("manifest");
        let g = given();
        let policy = testdata("mg90-d/policy.json");
        let (mut c, d) = fake();
        let torque = d.field("torque_enable").expect("torque").addr;
        block_on(c.write(ID, torque, &[1])).expect("torque on");
        let a = args(&g, &dir, &policy);
        let (lines, accepted) = block_on(gather_and_tune(
            &mut c,
            ID,
            &d,
            &a,
            &root,
            &AtomicBool::new(false),
        ))
        .expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(!accepted);
        assert!(root.join("inputs/measured.json").exists());
        assert!(!root.join("tune").exists(), "nothing tuned");
        let last = lines.last().expect("a line");
        let want = format!(
            "nothing was written; the measured inputs are kept: osc-ident --id 1 tune --inputs {} --out {} --tries 6 continues from them",
            root.join("inputs").display(),
            root.join("tune").display()
        );
        assert_eq!(last, &want);
        assert!(text.contains("torque_enable is on"), "{text}");
        let m: Value = serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
            .expect("json");
        assert_eq!(m["gather"]["state"], "accepted", "{m}");
        assert_eq!(m["gather"]["files"][0], "inputs/measured.json", "{m}");
        assert_eq!(m["tune"]["state"], "refused", "{m}");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A policy that is not there, or no tries: refused before anything
    /// goes on the bus, and no run folder.
    #[test]
    fn the_arguments_are_refused_before_any_bus_traffic() {
        let dir = scratch("run-args");
        let policy = testdata("mg90-d/policy.json");
        let g = given();
        let (mut c, d) = fake();
        c.pipe_mut().take_frames();
        let refused = |c: &mut Client<FakePipe>, a: &RunArgs, word: &str| {
            let e =
                block_on(run(c, ID, &d, a, "uid", &AtomicBool::new(false))).expect_err("refused");
            assert!(format!("{e:#}").contains(word), "{e:#}");
            assert!(c.pipe_mut().take_frames().is_empty(), "the bus was used");
        };
        let missing = dir.join("none.json");
        refused(&mut c, &args(&g, &dir, &missing), "none.json");
        let mut a = args(&g, &dir, &policy);
        a.tries = 0;
        refused(&mut c, &a, "--tries");
        admit(&args(&g, &dir, &policy)).expect("admitted");
        assert!(!dir.join("uid").exists(), "no run folder");
        let _ = std::fs::remove_dir_all(dir);
    }
}
