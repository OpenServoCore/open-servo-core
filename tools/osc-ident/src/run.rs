//! `run` on the file system: [`osc_measure::run`], the run folder under
//! `--out`.

use std::sync::atomic::AtomicBool;

use anyhow::Result;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::run as of;

pub use osc_measure::run::RunArgs;

use crate::files::Fs;
use crate::steps::StepInputs;
#[cfg(test)]
use {
    crate::files,
    crate::steps::{self, Load},
    serde_json::Value,
    std::path::Path,
};

/// The refusals that need no servo ([`of::admit`]), each line said about
/// the files read printed to stderr.
pub fn admit(a: &RunArgs) -> Result<StepInputs> {
    let mut said = Vec::new();
    let admitted = of::admit(&mut Fs, a, &mut said);
    for n in &said {
        eprintln!("{n}");
    }
    admitted
}

/// `run` on the servo ([`of::run`]), the arguments [`admit`]ted first.
pub async fn run<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    let inputs = admit(a)?;
    of::run(c, id, d, &mut crate::host(&mut Fs, stop), a, &inputs, servo).await
}

#[cfg(test)]
async fn gather_and_tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    root: &Path,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    of::gather_and_tune(c, id, d, &mut crate::host(&mut Fs, stop), a, root).await
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::path::PathBuf;
    use std::sync::atomic::Ordering;

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
    }

    fn given() -> Given {
        Given {
            nameplate: testdata("mg90-d/nameplate.json"),
        }
    }

    fn args<'a>(g: &'a Given, out: &'a Path, policy: &'a Path) -> RunArgs<'a> {
        RunArgs {
            nameplate: &g.nameplate,
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
        let before = block_on(osc_measure::state::read(&mut c, ID, &d)).expect("state");
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
        assert!(root.join(files::BOARD).exists(), "step 0 kept the board");
        assert!(!root.join("inputs/measured.json").exists() && !root.join("tune").exists());
        let m: Value = serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read"))
            .expect("json");
        assert_eq!(m["steps"]["probe"]["state"], "aborted", "{m}");
        assert_eq!(m["gather"]["state"], "not run", "{m}");
        assert_eq!(m["tune"]["state"], "not run", "{m}");
        assert_eq!(block_on(c.read(ID, 0x180, 1)).expect("read"), [0]);
        // the steps put every stored field back: the run's end reboots to
        // clear the unsaved bit their writes set
        let more = block_on(steps::clear_unsaved(&mut c, ID, &d, &before)).expect("clears");
        assert!(
            more[0].ends_with("now unsaved no, 0 differences"),
            "{more:?}"
        );
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
        let inputs =
            StepInputs::load(Some(&testdata("mg90-d/nameplate.json")), None).expect("inputs");
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
        files::step_0(&root);
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

    /// A run a page runs names no command: a tune refused before any write
    /// keeps the measured inputs, said without the command that continues,
    /// and a run whose steps were not all accepted stops without the one
    /// that runs it again.
    #[test]
    fn a_run_on_a_page_names_no_command() {
        let dir = scratch("run-page");
        let root = dir.join("uid/run");
        std::fs::create_dir_all(root.join("results")).expect("mkdir");
        files::step_0(&root);
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                root.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        std::fs::write(
            root.join("run.json"),
            r#"{"tool": "osc-client-web-ident test", "steps": {}}"#,
        )
        .expect("manifest");
        let g = given();
        let policy = testdata("mg90-d/policy.json");
        let a = args(&g, &dir, &policy);
        let stop = AtomicBool::new(false);
        let mut fs = Fs;
        let mut host = crate::host(&mut fs, &stop);
        host.tool = "osc-client-web-ident test";
        let (mut c, d) = fake();
        let torque = d.field("torque_enable").expect("torque").addr;
        block_on(c.write(ID, torque, &[1])).expect("torque on");
        let (lines, accepted) =
            block_on(of::gather_and_tune(&mut c, ID, &d, &mut host, &a, &root)).expect("runs");
        assert!(!accepted);
        assert_eq!(
            lines.last().map(String::as_str),
            Some("nothing was written; the measured inputs are kept")
        );

        let (mut c, d) = fake();
        let out = dir.join("whole");
        let a = args(&g, &out, &policy);
        let inputs = admit(&a).expect("admitted");
        let (lines, accepted) =
            block_on(of::run(&mut c, ID, &d, &mut host, &a, &inputs, "uid")).expect("runs");
        assert!(!accepted);
        assert_eq!(
            lines.last().map(String::as_str),
            Some("the run stops: not every step was accepted, so nothing is gathered or tuned")
        );
        assert!(
            !lines.iter().any(|l| l.contains("osc-ident --id")),
            "{lines:?}"
        );
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

    /// A profile with no heat model is refused before the bus, naming its
    /// four values; the record profile, which states one, is admitted, and
    /// so is the version 0 nameplate with the thermal.json beside it.
    #[test]
    fn a_profile_with_no_heat_model_is_refused_before_any_bus_traffic() {
        let dir = scratch("run-no-heat");
        let policy = testdata("mg90-d/policy-of-record.json");
        let mut v: Value =
            serde_json::from_slice(&files::read(&testdata("mg90-d/profile.json")).expect("read"))
                .expect("json");
        v["thermal"] = Value::Null;
        let blank = Given {
            nameplate: dir.join("profile.json"),
        };
        std::fs::write(&blank.nameplate, serde_json::to_vec(&v).expect("bytes")).expect("write");
        let (mut c, d) = fake();
        c.pipe_mut().take_frames();
        let e = block_on(run(
            &mut c,
            ID,
            &d,
            &args(&blank, &dir, &policy),
            "uid",
            &AtomicBool::new(false),
        ))
        .expect_err("refused");
        assert_eq!(
            format!("{e:#}"),
            format!(
                "{}: {}",
                blank.nameplate.display(),
                osc_ident_inputs::no_heat_model()
            )
        );
        assert!(c.pipe_mut().take_frames().is_empty(), "the bus was used");
        assert!(!dir.join("uid").exists(), "no run folder");
        let record = Given {
            nameplate: testdata("mg90-d/profile.json"),
        };
        let i = admit(&args(&record, &dir, &policy)).expect("admitted");
        let heat = i.thermal.expect("the profile's heat model");
        assert_eq!(heat.slow_tau_s.map(|q| q.value), Some(128.92));
        let i = admit(&args(&given(), &dir, &testdata("mg90-d/policy.json"))).expect("v0");
        assert_eq!(
            i.thermal.and_then(|t| t.fast_tau_s).map(|q| q.source.file),
            Some(testdata("mg90-d/thermal.json").display().to_string())
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A run on the simulated servo as a page runs it.
    struct Fed {
        dir: PathBuf,
        root: PathBuf,
        lines: Vec<String>,
        told: Vec<of::Progress>,
        c: Client<FakePipe>,
        d: Descriptor,
    }

    /// The run with a feed that keeps what it is told, and raises the stop
    /// flag as the step `stop_at` starts where one is given.
    fn fed(name: &str, stop_at: Option<&str>) -> Fed {
        let (mut c, d) = fake();
        let dir = scratch(name);
        let policy = testdata("mg90-d/policy.json");
        let g = given();
        let a = args(&g, &dir, &policy);
        let inputs = admit(&a).expect("admitted");
        let stop = AtomicBool::new(false);
        let told = RefCell::new(Vec::new());
        let feed = |p: &of::Progress| {
            if matches!(p, of::Progress::Started { step } if Some(*step) == stop_at) {
                stop.store(true, Ordering::SeqCst);
            }
            told.borrow_mut().push(p.clone());
        };
        let (lines, _) = {
            let mut fs = Fs;
            let mut host = crate::host(&mut fs, &stop);
            host.feed = &feed;
            block_on(of::run(&mut c, ID, &d, &mut host, &a, &inputs, "uid")).expect("runs")
        };
        let root = std::fs::read_dir(dir.join("uid"))
            .expect("the run folder")
            .next()
            .expect("one")
            .expect("entry")
            .path();
        Fed {
            dir,
            root,
            lines,
            told: told.into_inner(),
            c,
            d,
        }
    }

    fn manifest(root: &Path) -> Value {
        serde_json::from_slice(&std::fs::read(root.join("run.json")).expect("read")).expect("json")
    }

    /// Each step's end as the feed was told it, in order.
    fn ends(told: &[of::Progress]) -> Vec<(String, String, Option<String>)> {
        told.iter()
            .filter_map(|p| match p {
                of::Progress::Ended { step, state, why } => {
                    Some((step.to_string(), state.clone(), why.clone()))
                }
                _ => None,
            })
            .collect()
    }

    fn at(told: &[of::Progress], p: &of::Progress) -> usize {
        told.iter().position(|t| t == p).expect("told")
    }

    /// The simulated servo's run stops at the probe: the feed is told each
    /// step's start and its end in the state and why `run.json` records,
    /// in its order, every step after the probe not run and never begun;
    /// each step's lines as it ends, all of them, in order, the lines the
    /// run returns.
    #[test]
    fn the_feed_tells_each_step_as_run_json_records_it() {
        let f = fed("feed-steps", None);
        let m = manifest(&f.root);
        let entry = |k: &str, v: &Value| {
            (
                k.to_string(),
                v["state"].as_str().expect("state").to_string(),
                v["why"].as_str().map(str::to_string),
            )
        };
        let mut want: Vec<_> = m["steps"]
            .as_object()
            .expect("steps")
            .iter()
            .map(|(k, v)| entry(k, v))
            .collect();
        for k in ["gather", "tune", "cruise", "final"] {
            want.push(entry(k, &m[k]));
        }
        let ended = ends(&f.told);
        assert_eq!(ended, want, "{m}");
        let states: Vec<(&str, &str)> = ended
            .iter()
            .map(|(s, w, _)| (s.as_str(), w.as_str()))
            .collect();
        assert_eq!(
            states,
            [
                ("rest", "accepted"),
                ("probe", "aborted"),
                ("gather", "not run"),
                ("tune", "not run"),
                ("cruise", "not run"),
                ("final", "not run")
            ]
        );
        let begun: Vec<&str> = f
            .told
            .iter()
            .filter_map(|p| match p {
                of::Progress::Started { step } => Some(*step),
                _ => None,
            })
            .collect();
        assert_eq!(begun, ["rest", "probe"]);
        let end_of = |step: &'static str| {
            f.told
                .iter()
                .position(|p| matches!(p, of::Progress::Ended { step: s, .. } if *s == step))
                .expect("ended")
        };
        let rest = at(&f.told, &of::Progress::Started { step: "rest" });
        let probe = at(&f.told, &of::Progress::Started { step: "probe" });
        assert!(rest < end_of("rest") && end_of("rest") < probe && probe < end_of("probe"));
        let said_before = |i: usize| -> Vec<String> {
            f.told[..i]
                .iter()
                .filter_map(|p| match p {
                    of::Progress::Lines { lines } => Some(lines.clone()),
                    _ => None,
                })
                .flatten()
                .collect()
        };
        assert!(
            said_before(probe).iter().any(|l| l == "step rest accepted"),
            "the rest step's lines are told before the probe starts"
        );
        assert_eq!(
            said_before(f.told.len()),
            f.lines,
            "the lines the run returns"
        );
        assert_eq!(
            serde_json::to_value(&f.told[0]).expect("json"),
            serde_json::json!({"event": "started", "step": "rest"})
        );
        let _ = std::fs::remove_dir_all(f.dir);
    }

    /// The stop flag raised as the probe starts, as Ctrl-C raises the
    /// CLI's: the probe stops before its first burst and ends through its
    /// way out, torque off and read back; `run.json` and the feed record
    /// it interrupted, and the run stops there.
    #[test]
    fn a_stop_raised_mid_run_ends_the_step_through_its_way_out() {
        let mut f = fed("feed-stop", Some("probe"));
        let m = manifest(&f.root);
        let why = "interrupted: the step stopped between rungs";
        assert_eq!(m["steps"]["rest"]["state"], "accepted", "{m}");
        assert_eq!(m["steps"]["probe"]["state"], "interrupted", "{m}");
        assert_eq!(m["steps"]["probe"]["why"], why, "{m}");
        assert_eq!(m["gather"]["state"], "not run", "{m}");
        assert_eq!(
            ends(&f.told)[1],
            ("probe".into(), "interrupted".into(), Some(why.into()))
        );
        let out = f
            .lines
            .iter()
            .rposition(|l| l == "the way out:")
            .expect("the probe's way out");
        let end = f
            .lines
            .iter()
            .position(|l| *l == format!("step probe interrupted: {why}"))
            .expect("the probe's verdict");
        assert!(
            f.lines[out..end]
                .iter()
                .any(|l| l == "  torque_enable 0, read back 0"),
            "{:?}",
            &f.lines[out..end]
        );
        let torque = f.d.field("torque_enable").expect("torque").addr;
        assert_eq!(block_on(f.c.read(ID, torque, 1)).expect("read"), [0]);
        assert!(
            f.lines
                .last()
                .is_some_and(|l| l.starts_with("the run stops: not every step was accepted")),
            "{:?}",
            f.lines.last()
        );
        let _ = std::fs::remove_dir_all(f.dir);
    }
}

/// The run into memory, as the browser keeps it, against the same run on
/// disk, as the CLI keeps it.
#[cfg(test)]
mod parity {
    use std::cell::Cell;
    use std::path::{Path, PathBuf};
    use std::sync::atomic::AtomicBool;

    use futures_lite::future::block_on;
    use osc_client::BaudRate;
    use osc_client::descriptor::Descriptor;
    use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
    use osc_client::{Client, Id, mgmt};
    use osc_measure::guard::{Exit, Way};
    use osc_measure::install::{self, Refused};
    use osc_measure::run::{self as of, Host, RunArgs, Store};
    use osc_measure::scorecard::REST;
    use osc_measure::sink::MemSink;
    use osc_measure::steps::cruise::{self, CruiseInputs, Cruised};
    use osc_measure::steps::verify::{self, Verified, VerifyInputs};
    use osc_measure::steps::{Check, Ended, Rig, Verdict};
    use serde_json::{Value, json};

    use crate::files::{self, Fs};

    const ID: Id = Id::new(1);
    const TOOL: &str = "osc-ident parity";

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

    /// A clock that reads a quarter second later each time, from the same
    /// instant on every run.
    fn clock() -> impl Fn() -> f64 {
        let t = Cell::new(1_764_000_000.0);
        move || {
            let now = t.get();
            t.set(now + 0.25);
            now
        }
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

    /// Every file under `dir` on disk, by its full path, with its bytes.
    fn on_disk(dir: &Path) -> Vec<(String, Vec<u8>)> {
        let mut names = Vec::new();
        of::finish::tree(&Fs, dir, "", &mut names).expect("tree");
        names
            .into_iter()
            .map(|n| {
                let p = dir.join(&n);
                (p.display().to_string(), std::fs::read(&p).expect("read"))
            })
            .collect()
    }

    /// Every file of `m` under `dir`, as [`on_disk`] lists them.
    fn in_memory(m: &MemSink, dir: &Path) -> Vec<(String, Vec<u8>)> {
        let prefix = format!("{}/", dir.display());
        m.files
            .iter()
            .filter(|(k, _)| k.starts_with(&prefix))
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// A memory store holding the given files from disk at their paths.
    fn holding(paths: &[PathBuf]) -> MemSink {
        let mut m = MemSink::default();
        for p in paths {
            m.write_file(p, &std::fs::read(p).expect("read"))
                .expect("write");
        }
        m
    }

    fn given() -> [PathBuf; 3] {
        [
            testdata("mg90-d/nameplate.json"),
            testdata("mg90-d/policy.json"),
            testdata("mg90-d/thermal.json"),
        ]
    }

    /// `run` on the simulated servo, which runs no shunt burst, so the run
    /// stops at the probe: into memory it says the same lines and writes
    /// the same files, path for path and byte for byte, as on disk.
    #[test]
    fn a_run_into_a_mem_sink_writes_what_the_cli_writes() {
        let dir = scratch("parity-run");
        let out = dir.join("out");
        let [nameplate, policy, _] = given();
        let a = RunArgs {
            nameplate: &nameplate,
            policy: &policy,
            out: &out,
            tries: 6,
        };
        let run = |files: &mut dyn Store| {
            let (mut c, d) = fake();
            let mut said = Vec::new();
            let inputs = of::admit(files, &a, &mut said).expect("admitted");
            let stop = AtomicBool::new(false);
            let clock = clock();
            let mut host = Host {
                files,
                clock: &clock,
                stop: &stop,
                feed: &of::quiet,
                tool: TOOL,
            };
            let ran = block_on(of::run(&mut c, ID, &d, &mut host, &a, &inputs, "uid"));
            (said, ran.expect("runs"))
        };
        let disk = run(&mut Fs);
        let mut m = holding(&given());
        let mem = run(&mut m);
        assert!(disk.0.len() >= 2, "{:?}", disk.0);
        assert!(!disk.1.1, "the simulated servo's run stops at the probe");
        assert_eq!(mem, disk, "the same lines");
        let files = on_disk(&out);
        let names: Vec<&str> = files.iter().map(|(n, _)| n.as_str()).collect();
        for f in ["run.json", "inputs/board.json", "results/probe.json"] {
            assert!(names.iter().any(|n| n.ends_with(f)), "{f}: {names:?}");
        }
        assert!(names.iter().any(|n| n.contains("/captures/rest/")));
        assert_eq!(in_memory(&m, &out), files, "the same files");
        let _ = std::fs::remove_dir_all(dir);
    }

    fn ended(way: Way) -> Ended {
        Ended {
            way,
            why: (way != Way::Done).then(|| "scripted".into()),
            exit: Exit {
                lines: vec![],
                torque_off: true,
                rebooted: false,
            },
            lines: vec![],
        }
    }

    /// The servo of record holding the tune's set `set`, volatile.
    fn tuned_servo(
        set: &osc_measure::set::SetFile,
        points: Option<&[i16; osc_client::pos_lut::INTERVALS]>,
    ) -> Client<FakePipe> {
        let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
        inner.seed_calibrated(0);
        let mut c = block_on(Client::connect(inner)).expect("connect");
        let d = crate::descriptor_builtin().expect("descriptor");
        let rec = files::set(&testdata("mg90-d/constants.json")).expect("record");
        let table = files::table(&testdata("mg90-d/pos_lut.json")).expect("table");
        block_on(install::install(
            &mut c,
            ID,
            &d,
            &rec,
            Some(&table.points),
            &mut Vec::new(),
        ))
        .expect("the record installs");
        block_on(mgmt::save(&mut c, ID)).expect("save");
        block_on(install::install(
            &mut c,
            ID,
            &d,
            set,
            points,
            &mut Vec::new(),
        ))
        .expect("the tune's set installs");
        c
    }

    /// The finish on a run folder as the tune leaves it, its cruise and
    /// checks scripted (refused on rest error, then accepted, so it backs
    /// off once): into memory it says the same lines and writes the same
    /// `final/` and `tune/` files as on disk.
    #[test]
    fn a_finish_into_a_mem_sink_writes_what_the_cli_writes() {
        let dir = scratch("parity-finish");
        let root = dir.join("uid/run");
        std::fs::create_dir_all(root.join("results")).expect("mkdir");
        files::step_0(&root);
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                root.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let [nameplate, policy, _] = given();
        let inputs = root.join("inputs");
        crate::gather::gather(&root, &nameplate, &policy, &inputs).expect("gathers");
        let d = crate::descriptor_builtin().expect("descriptor");
        let r = files::rules_inputs(&inputs, None).expect("inputs");
        let measured: Value =
            serde_json::from_slice(&std::fs::read(inputs.join("measured.json")).expect("read"))
                .expect("json");
        let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
        let k0 = osc_measure::rules::speed_loop::starting_gain(&r.inputs).expect("k0");
        let (set, _, _) = crate::tune::set_at(
            &r,
            &measured,
            k0,
            &crate::tune::Checked::default(),
            &pair,
            &d,
        )
        .expect("the tune's set");
        std::fs::create_dir_all(root.join("tune")).expect("mkdir");
        std::fs::write(root.join("tune/set.json"), set.render()).expect("set");
        std::fs::write(
            root.join("tune/tune.json"),
            json!({"starting_gain": k0, "tries": [{"try": 1, "kp": k0, "next": "accept"}],
                   "accepted": {"try": 1, "kp": k0, "speed_kp_verified": k0, "set": set.hash()}})
            .to_string(),
        )
        .expect("tune.json");
        // the tune's accepted check, as the tune keeps it; a file not there
        // is said in the host's own words, which differ between the stores
        let check = root.join(format!("tune/tries/01/{}", verify::RESULT));
        std::fs::create_dir_all(check.parent().expect("parent")).expect("mkdir");
        std::fs::write(
            &check,
            json!({"step": "verify", "verdict": "accepted"}).to_string(),
        )
        .expect("verify.json");
        let mut before: Vec<PathBuf> = on_disk(&dir).into_iter().map(|(n, _)| n.into()).collect();
        before.extend(given());
        let mut m = holding(&before);

        let points = r.table.as_ref().map(|t| t.points);
        let cruise_doc: Value =
            serde_json::from_slice(&std::fs::read(testdata("gather/cruise.json")).expect("read"))
                .expect("json");
        let a = RunArgs {
            nameplate: &nameplate,
            policy: &policy,
            out: &dir,
            tries: 6,
        };
        let finish = |files: &mut dyn Store| {
            let mut c = tuned_servo(&set, points.as_ref());
            let stop = AtomicBool::new(false);
            let clock = clock();
            let mut host = Host {
                files,
                clock: &clock,
                stop: &stop,
                feed: &of::quiet,
                tool: TOOL,
            };
            let mut checks = 0;
            let f = block_on(of::finish::finish_with(
                &mut c,
                ID,
                &d,
                &mut host,
                &a,
                &root,
                async |rig: &mut Rig<'_, FakePipe>, _: &CruiseInputs<'_>| {
                    rig.sink
                        .write(cruise::RESULT, cruise_doc.to_string().as_bytes())
                        .map_err(|e| Refused(e.0))?;
                    Ok::<_, Refused>(Cruised {
                        ended: ended(Way::Done),
                        checks: vec![],
                        result: Some(cruise_doc.clone()),
                    })
                },
                async |rig: &mut Rig<'_, FakePipe>, _: &VerifyInputs<'_>| {
                    checks += 1;
                    let (way, failed): (Way, &[&str]) = if checks == 1 {
                        (Way::Refused, &[REST])
                    } else {
                        (Way::Done, &[])
                    };
                    rig.sink
                        .write(
                            verify::RESULT,
                            json!({"verdict": way.name(), "failed_on": failed})
                                .to_string()
                                .as_bytes(),
                        )
                        .map_err(|e| Refused(e.0))?;
                    let verdict = if way == Way::Done {
                        Verdict::Pass
                    } else {
                        Verdict::Fail
                    };
                    let name = failed.first().copied().unwrap_or("scripted");
                    Ok::<_, Refused>(Verified {
                        ended: ended(way),
                        checks: vec![Check {
                            name: name.into(),
                            value: "scripted".into(),
                            bound: "scripted".into(),
                            verdict,
                        }],
                        result: None,
                    })
                },
            ))
            .expect("finishes");
            (f.lines, f.cruise, f.last)
        };
        let disk = finish(&mut Fs);
        let mem = finish(&mut m);
        assert_eq!(disk.2["state"], "accepted", "{}", disk.2);
        assert_eq!(disk.2["checks"], json!([2, 3]), "{}", disk.2);
        assert_eq!(mem, disk, "the same lines and records");
        let files = on_disk(&dir);
        for f in [
            "final/set.json",
            "final/inputs/measured.json",
            "tune/back-off.json",
        ] {
            assert!(files.iter().any(|(n, _)| n.ends_with(f)), "{f}");
        }
        assert_eq!(in_memory(&m, &dir), files, "the same files");
        let _ = std::fs::remove_dir_all(dir);
    }
}
