//! `tune` on the file system ([`osc_measure::run::tune`]): the speed gain
//! a closed-loop check settles, into the run folder `--out`.

use std::path::Path;
use std::sync::atomic::AtomicBool;

use anyhow::Result;
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::run::tune as of;

use crate::files::Fs;
#[cfg(test)]
use {
    crate::files,
    osc_measure::install::{self, Refused},
    osc_measure::set::SetFile,
    osc_measure::steps::Rig,
    osc_measure::steps::verify::{self, Verified, VerifyInputs},
    osc_measure::tune,
    serde_json::{Value, json},
};

#[cfg(test)]
pub(crate) use osc_measure::run::tune::{Checked, set_at};

pub async fn tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    inputs: &Path,
    out: &Path,
    tries: u32,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    of::tune(
        c,
        id,
        d,
        &mut crate::host(&mut Fs, stop),
        inputs,
        out,
        tries,
    )
    .await
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
async fn tune_with<P: Pipe, F>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    inputs: &Path,
    out: &Path,
    tries: u32,
    stop: &AtomicBool,
    check: F,
) -> Result<(Vec<String>, bool)>
where
    F: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    of::tune_with(
        c,
        id,
        d,
        &mut crate::host(&mut Fs, stop),
        inputs,
        out,
        tries,
        check,
    )
    .await
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::path::PathBuf;
    use std::rc::Rc;
    use std::time::Duration;

    use futures_lite::future::block_on;
    use osc_client::fake::{FakePipe, TelSample, TelemetryTick};
    use osc_client::pipe::PipeError;
    use osc_client::{BaudRate, Inst, MgmtOp, Opcode, mgmt};
    use osc_measure::guard::{Exit, Way};
    use osc_measure::scorecard::{self as sc, Lin};
    use osc_measure::state;
    use osc_measure::steps::{Check, Ended, Unit, Verdict};

    use super::*;

    const ID: Id = Id::new(1);
    const OSC_EXCHANGE: u8 = 0x20;
    const CONTROL: u16 = 0x180;
    const CHUNK0: u16 = 0x1e2;
    /// A raw pot near the middle of the set of record's soft range.
    const MID_POT: u16 = 1964;

    type Log = Rc<RefCell<Vec<Ev>>>;
    /// A hook on every TEL arm, its number from 1.
    type Hook = Box<dyn FnMut(usize, &mut FakePipe)>;

    /// The fake adapter, every write and MGMT logged. The simulated servo
    /// runs no estimator and a reboot zeroes its estimate, so the estimate
    /// is held at `theta_q16`, the reading of the resting shaft.
    struct Spy {
        inner: FakePipe,
        log: Log,
        arms: usize,
        hook: Hook,
        theta_q16: Option<i32>,
        /// Run on the servo as each REBOOT is acked.
        rebooted: Box<dyn FnMut(&mut FakePipe)>,
    }

    impl Pipe for Spy {
        async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
            if let [
                _,
                _,
                OSC_EXCHANGE,
                _,
                _,
                _,
                _,
                inst,
                _,
                _,
                _,
                _,
                payload @ ..,
            ] = bytes
            {
                match Inst(*inst).opcode() {
                    Some(Opcode::Write) if payload.len() >= 2 => {
                        let addr = u16::from_le_bytes([payload[0], payload[1]]);
                        self.log
                            .borrow_mut()
                            .push(Ev::Write(addr, payload[2..].to_vec()));
                        if addr == CHUNK0 && payload.len() >= 12 {
                            self.arms += 1;
                            (self.hook)(self.arms, &mut self.inner);
                        }
                    }
                    Some(Opcode::Mgmt) => self.log.borrow_mut().push(Ev::Mgmt(payload[0])),
                    _ => {}
                }
            }
            let reboot = matches!(bytes, [_, _, OSC_EXCHANGE, _, _, _, _, inst, _, _, _, _, op, ..]
                if Inst(*inst).opcode() == Some(Opcode::Mgmt) && *op == MgmtOp::Reboot as u8);
            let sent = self.inner.send(bytes).await;
            if reboot {
                (self.rebooted)(&mut self.inner);
            }
            if let Some(q) = self.theta_q16 {
                self.inner
                    .sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.estimates.theta_hat_q16 = q);
            }
            sent
        }

        async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
            self.inner.recv().await
        }

        async fn pause(&mut self, d: Duration) {
            self.inner.pause(d).await
        }
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

    fn descriptor() -> Descriptor {
        crate::descriptor_builtin().expect("descriptor")
    }

    fn record() -> SetFile {
        files::set(&testdata("mg90-d/constants.json")).expect("the set of record")
    }

    fn resting(pot: u16) -> Vec<TelSample> {
        (0..4096u32)
            .map(|i| TelSample {
                tick: TelemetryTick {
                    _scan: [0; 9],
                    pos: pot,
                    vbus_raw: 1241,
                    ntc_raw: 2010,
                    current_raw: 1206,
                    vmotor_a: 773,
                    vmotor_b: 772,
                    duty_ticks: 0,
                    current_trough: 1206,
                    i_tick_counts: 0,
                    vdiff_counts: 0,
                    vmotor_a_trough: 773,
                    vmotor_b_trough: 772,
                    tick_flags: 0,
                    sample_tick: i,
                },
                vbus_counts: 2000,
            })
            .collect()
    }

    /// The servo of record at 3 Mbps, the set of record installed and
    /// saved, at rest near the middle, its estimate its own reading.
    fn servo(hook: Hook) -> (Client<Spy>, Log) {
        servo_at(BaudRate::B3000000, hook)
    }

    /// [`servo`] with its saved baud `saved`, the bus then moved to 3M
    /// volatile, as `osc-bus baud 3000000` does after a FACTORY.
    fn servo_at(saved: BaudRate, hook: Hook) -> (Client<Spy>, Log) {
        let mut inner = FakePipe::new(saved, &[1]);
        inner.seed_calibrated(0);
        let log = Log::default();
        let mut c = block_on(Client::connect(Spy {
            inner,
            log: log.clone(),
            arms: 0,
            hook: Box::new(|_, _| {}),
            theta_q16: None,
            rebooted: Box::new(|_| {}),
        }))
        .expect("connect");
        let d = descriptor();
        let rec = record();
        let table = files::table(&testdata("mg90-d/pos_lut.json")).expect("table");
        block_on(install::install(
            &mut c,
            ID,
            &d,
            &rec,
            Some(&table.points),
            &mut Vec::new(),
        ))
        .expect("the set of record installs");
        block_on(mgmt::save(&mut c, ID)).expect("save");
        if saved != BaudRate::B3000000 {
            let roster = block_on(mgmt::set_baud(&mut c, &[ID], BaudRate::B3000000));
            assert_eq!(roster.expect("baud"), [(ID, true)]);
        }
        c.pipe_mut().inner.set_track(0, resting(MID_POT));
        let s = block_on(state::read(&mut c, ID, &d)).expect("state");
        let lin = Lin::new(s.lut.effective(), s.slots(&d).as_deref());
        let q = (lin.counts(MID_POT) * 65536.0).round() as i32;
        c.pipe_mut().theta_q16 = Some(q);
        c.pipe_mut().hook = hook;
        log.borrow_mut().clear();
        (c, log)
    }

    /// A servo off the line, never saved: board data only, every image
    /// reason of a factory-fresh servo, saved at 1M and the bus moved to
    /// 3M volatile, at rest near the middle, its estimate the reading the
    /// set of record's table gives.
    fn fresh_servo() -> (Client<Spy>, Log) {
        let mut inner = FakePipe::new(BaudRate::B1000000, &[1]);
        inner.seed_board(0);
        let log = Log::default();
        let mut c = block_on(Client::connect(Spy {
            inner,
            log: log.clone(),
            arms: 0,
            hook: Box::new(|_, _| {}),
            theta_q16: None,
            rebooted: Box::new(|_| {}),
        }))
        .expect("connect");
        let roster = block_on(mgmt::set_baud(&mut c, &[ID], BaudRate::B3000000));
        assert_eq!(roster.expect("baud"), [(ID, true)]);
        c.pipe_mut().inner.set_track(0, resting(MID_POT));
        let table = files::table(&testdata("mg90-d/pos_lut.json")).expect("table");
        let slots = record().slots().expect("slots");
        let lin = Lin::new(Some(&table.points), slots.as_deref());
        c.pipe_mut().theta_q16 = Some((lin.counts(MID_POT) * 65536.0).round() as i32);
        log.borrow_mut().clear();
        (c, log)
    }

    fn run(
        c: &mut Client<Spy>,
        inputs: &Path,
        out: &Path,
        tries: u32,
    ) -> Result<(Vec<String>, bool)> {
        block_on(tune(
            c,
            ID,
            &descriptor(),
            inputs,
            out,
            tries,
            &AtomicBool::new(false),
        ))
    }

    fn mgmt_ops(log: &Log) -> Vec<u8> {
        log.borrow()
            .iter()
            .filter_map(|e| match e {
                Ev::Mgmt(op) => Some(*op),
                _ => None,
            })
            .collect()
    }

    fn json_at(p: &Path) -> Value {
        serde_json::from_slice(&std::fs::read(p).expect("read")).expect("json")
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Ev {
        Write(u16, Vec<u8>),
        Mgmt(u8),
        Check,
    }

    /// A check's verdict as the step ends: `failed` the checks it failed,
    /// its result kept in the try's folder.
    fn verified(rig: &mut Rig<'_, Spy>, way: Way, failed: &[&str]) -> Result<Verified, Refused> {
        rig.c.pipe_mut().log.borrow_mut().push(Ev::Check);
        let checks: Vec<Check> = failed
            .iter()
            .map(|n| Check {
                name: n.to_string(),
                value: "scripted".into(),
                bound: "scripted".into(),
                verdict: Verdict::Fail,
            })
            .collect();
        let result = json!({
            "verdict": way.name(),
            "failed_on": failed,
            "bounds": {"screen_counts": 199.0},
            "moves": [{"overshoot_deg": 1.5, "overshoot_bound_deg": 1.0, "tail_driven_share": 0.01,
                       "rest_error_deg": 0.7, "follow_peak_counts": 120.0}],
        });
        rig.sink
            .write(verify::RESULT, result.to_string().as_bytes())
            .map_err(|e| Refused(e.0))?;
        Ok(Verified {
            ended: Ended {
                way,
                why: (way != Way::Done).then(|| format!("scripted: {}", failed.join(", "))),
                exit: Exit {
                    lines: vec![],
                    torque_off: true,
                    rebooted: false,
                },
                lines: vec![],
            },
            checks,
            result: Some(result),
        })
    }

    fn run_scripted(
        c: &mut Client<Spy>,
        out: &Path,
        tries: u32,
        script: Vec<(Way, Vec<&'static str>)>,
    ) -> Result<(Vec<String>, bool)> {
        run_scripted_on(c, &testdata("mg90-d"), out, tries, script)
    }

    fn run_scripted_on(
        c: &mut Client<Spy>,
        inputs: &Path,
        out: &Path,
        tries: u32,
        script: Vec<(Way, Vec<&'static str>)>,
    ) -> Result<(Vec<String>, bool)> {
        let mut at = 0;
        let check = async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
            let (way, failed) = script[at.min(script.len() - 1)].clone();
            at += 1;
            verified(rig, way, &failed)
        };
        block_on(tune_with(
            c,
            ID,
            &descriptor(),
            inputs,
            out,
            tries,
            &AtomicBool::new(false),
            check,
        ))
    }

    /// The feed is told each try as it begins, of the tries allowed.
    #[test]
    fn the_feed_is_told_each_try_as_it_begins() {
        use osc_measure::run::Progress;
        let (mut c, _) = servo(Box::new(|_, _| {}));
        let dir = scratch("tune-feed");
        let told = RefCell::new(Vec::new());
        let feed = |p: &Progress| told.borrow_mut().push(p.clone());
        let stop = AtomicBool::new(false);
        let mut fs = Fs;
        let mut host = crate::host(&mut fs, &stop);
        host.feed = &feed;
        let check = async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
            for at in 1..=2 {
                (rig.tell)(Unit {
                    at,
                    of: 2,
                    what: "move",
                });
            }
            verified(rig, Way::Refused, &[sc::QUIET])
        };
        let (_, accepted) = block_on(of::tune_with(
            &mut c,
            ID,
            &descriptor(),
            &mut host,
            &testdata("mg90-d"),
            &dir.join("run"),
            3,
            check,
        ))
        .expect("runs");
        assert!(!accepted);
        let unit = |at| Progress::Unit {
            step: "tune",
            at,
            of: 2,
            what: "move",
        };
        let tries: Vec<Progress> = (1..=3)
            .flat_map(|at| {
                [
                    Progress::Try {
                        step: "tune",
                        at,
                        of: 3,
                    },
                    unit(1),
                    unit(2),
                ]
            })
            .collect();
        assert_eq!(told.take(), tries);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Each try's set from the rules, then its install, then the check, in
    /// that order; refused on its scores, each try four fifths of the gain
    /// before. After the last try the servo is rebooted onto its stored
    /// state, the backup's; no SAVE is ever sent.
    #[test]
    fn a_tune_refused_on_its_scores_lowers_each_try_then_puts_the_servo_back() {
        let (mut c, log) = servo(Box::new(|_, _| {}));
        let dir = scratch("tune-scores");
        let out = dir.join("run");
        let script = vec![
            (Way::Refused, vec![sc::REST, sc::FOLLOW]),
            (Way::Refused, vec![sc::OVERSHOOT]),
            (Way::Refused, vec![sc::QUIET]),
        ];
        let (lines, accepted) = run_scripted(&mut c, &out, 3, script).expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(!accepted, "{text}");
        let doc = json_at(&out.join("tune.json"));
        let tries = doc["tries"].as_array().expect("tries");
        assert_eq!(tries.len(), 3, "{doc}");
        let k0 = doc["starting_gain"].as_f64().expect("k0");
        for (n, t) in tries.iter().enumerate() {
            let kp = t["kp"].as_f64().expect("kp");
            assert!((kp - k0 * tune::SHARE.powi(n as i32)).abs() < 1e-12, "{t}");
            assert_eq!(t["verdict"], "refused", "{t}");
            assert_eq!(t["next"], "lower", "{t}");
            assert_eq!(t["worst"]["overshoot_share_of_bound"], 1.5, "{t}");
            let dir = out.join(format!("tries/{:02}", n + 1));
            for f in ["set.json", "measured.json", "results/verify.json"] {
                assert!(dir.join(f).exists(), "{f}");
            }
            let m = json_at(&dir.join("measured.json"));
            assert_eq!(m["speed_kp_verified"].as_f64(), Some(kp));
        }
        assert!(out.join("backup/stored.json").exists());
        assert!(!out.join("set.json").exists(), "no set is the tune's");
        assert_eq!(lines.iter().filter(|l| l.starts_with("try ")).count(), 3);
        assert!(text.contains("worst overshoot 150% of its bound"), "{text}");
        assert!(
            text.contains("put back: rebooted onto the stored configuration"),
            "{text}"
        );
        assert_eq!(
            lines.last().map(String::as_str),
            Some("live on the servo: its stored configuration; no tried set is installed")
        );

        // the reboot for the backup and the one that puts it back, no SAVE
        let reboot = MgmtOp::Reboot as u8;
        assert_eq!(mgmt_ops(&log), [reboot, reboot]);
        let d = descriptor();
        let kp_at = d.field("v_kp_q88").expect("v_kp").addr;
        let l = log.borrow();
        let mut at = 0;
        for n in 0..tries.len() {
            let set = files::set(&out.join(format!("tries/{:02}/set.json", n + 1))).expect("set");
            let want = (set.value("v_kp_q88").and_then(Value::as_u64).expect("v_kp") as u16)
                .to_le_bytes()
                .to_vec();
            let install = l[at..]
                .iter()
                .position(|e| *e == Ev::Write(kp_at, want.clone()))
                .map(|k| k + at)
                .unwrap_or_else(|| panic!("try {}: no install of its set's v_kp: {l:?}", n + 1));
            let checked = l[install..]
                .iter()
                .position(|e| *e == Ev::Check)
                .map(|k| k + install)
                .unwrap_or_else(|| panic!("try {}: no check after the install", n + 1));
            at = checked + 1;
        }
        assert_eq!(l.iter().filter(|e| **e == Ev::Check).count(), 3);
        // between a try's check and the next install only the speed gains
        // move: every other field is the same set's
        let checks: Vec<usize> = (0..l.len()).filter(|&k| l[k] == Ev::Check).collect();
        let name_at = |a: u16| {
            d.fields
                .iter()
                .find(|f| f.addr == a)
                .map_or(format!("{a:#x}"), |f| f.name.clone())
        };
        for w in checks.windows(2) {
            let mut moved: Vec<String> = l[w[0]..w[1]]
                .iter()
                .filter_map(|e| match e {
                    Ev::Write(a, _) if *a < CONTROL => Some(name_at(*a)),
                    _ => None,
                })
                .collect();
            moved.sort();
            moved.dedup();
            assert_eq!(
                moved,
                ["p_kp_q88", "v_kaw_q412", "v_ki_q412", "v_kp_q88"],
                "{moved:?}"
            );
        }
        let last_reboot = l
            .iter()
            .rposition(|e| *e == Ev::Mgmt(reboot))
            .expect("put back");
        assert!(last_reboot >= at, "the put-back follows the last check");
        drop(l);

        let checked = block_on(install::check(&mut c, ID, &d, &record())).expect("check");
        assert!(checked.clean(), "{:?}", checked.differences);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The first set the check accepts is the tune's: kept as set.json and
    /// left installed volatile, the command that stores it printed; nothing
    /// saved, nothing put back.
    #[test]
    fn the_first_accepted_set_is_the_result_and_is_never_saved() {
        let (mut c, log) = servo(Box::new(|_, _| {}));
        let dir = scratch("tune-accept");
        let out = dir.join("run");
        let script = vec![(Way::Refused, vec![sc::REST]), (Way::Done, vec![])];
        let (lines, accepted) = run_scripted(&mut c, &out, 4, script).expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(accepted, "{text}");
        let doc = json_at(&out.join("tune.json"));
        assert_eq!(doc["tries"].as_array().map(Vec::len), Some(2), "{doc}");
        assert_eq!(doc["accepted"]["try"], 2);
        let k0 = doc["starting_gain"].as_f64().expect("k0");
        assert_eq!(
            doc["accepted"]["speed_kp_verified"].as_f64(),
            Some(k0 * tune::SHARE)
        );
        assert_eq!(
            std::fs::read(out.join("set.json")).expect("set"),
            std::fs::read(out.join("tries/02/set.json")).expect("try 2")
        );
        assert!(text.contains("accepted on try 2") && text.contains("80% of the starting gain"));
        let save = format!(
            "to store it: osc-ident --id 1 save --set {} --table {}",
            out.join("set.json").display(),
            testdata("mg90-d").join("pos_lut.json").display()
        );
        assert!(lines.contains(&save), "{text}");
        assert!(
            lines
                .last()
                .is_some_and(|l| l.starts_with("live on the servo, volatile: the accepted set")),
            "{text}"
        );
        assert_eq!(
            mgmt_ops(&log),
            [MgmtOp::Reboot as u8],
            "the backup's reboot alone"
        );
        let set = files::set(&out.join("set.json")).expect("set");
        let checked = block_on(install::check(&mut c, ID, &descriptor(), &set)).expect("check");
        assert!(checked.clean(), "{:?}", checked.differences);

        // a check that aborts is no score: the tune stops there
        let (mut c, _) = servo(Box::new(|_, _| {}));
        let out = dir.join("aborted");
        let script = vec![(Way::Error, vec![]), (Way::Done, vec![])];
        let (lines, accepted) = run_scripted(&mut c, &out, 4, script).expect("runs");
        assert!(!accepted);
        let doc = json_at(&out.join("tune.json"));
        assert_eq!(doc["tries"].as_array().map(Vec::len), Some(1), "{doc}");
        assert!(lines.iter().any(|l| l.contains("put back: rebooted")));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A tune a page runs says the CLI's lines but the command that stores
    /// the accepted set: a page has no command line.
    #[test]
    fn a_tune_on_a_page_names_no_command() {
        let dir = scratch("tune-page");
        let out = dir.join("run");
        let script = vec![(Way::Refused, vec![sc::REST]), (Way::Done, vec![])];
        let (mut c, _) = servo(Box::new(|_, _| {}));
        let (cli, _) = run_scripted(&mut c, &out, 4, script.clone()).expect("runs");
        std::fs::remove_dir_all(&out).expect("a new folder for the page's");
        let (mut c, _) = servo(Box::new(|_, _| {}));
        let stop = AtomicBool::new(false);
        let mut fs = Fs;
        let mut host = crate::host(&mut fs, &stop);
        host.tool = "osc-client-web-ident test";
        let mut at = 0;
        let check = async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
            let (way, failed) = script[at.min(script.len() - 1)].clone();
            at += 1;
            verified(rig, way, &failed)
        };
        let (page, accepted) = block_on(of::tune_with(
            &mut c,
            ID,
            &descriptor(),
            &mut host,
            &testdata("mg90-d"),
            &out,
            4,
            check,
        ))
        .expect("runs");
        assert!(accepted);
        let store = |l: &String| l.starts_with("to store it: osc-ident ");
        assert_eq!(cli.iter().filter(|l| store(l)).count(), 1, "{cli:?}");
        let want: Vec<String> = cli.iter().filter(|l| !store(l)).cloned().collect();
        assert_eq!(page, want);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A field the rules store at a bound is said in the tune's lines,
    /// under the starting gain: a move speed at the ceiling in deg/s lands
    /// a count over the ceiling, which rounds down.
    #[test]
    fn a_field_stored_at_a_bound_is_said_in_the_tunes_lines() {
        let dir = scratch("tune-clamp");
        let inputs = dir.join("inputs");
        std::fs::create_dir_all(&inputs).expect("mkdir");
        for f in [
            "measured.json",
            "shelf.json",
            "nameplate.json",
            "thermal.json",
            "pos_lut.json",
            "wide-codes.json",
            "board.json",
        ] {
            std::fs::copy(testdata(&format!("mg90-d/{f}")), inputs.join(f)).expect("copy");
        }
        let mut policy = json_at(&testdata("mg90-d/policy.json"));
        policy["velocity_limit_deg_s"] = 400.0.into();
        std::fs::write(inputs.join("policy.json"), policy.to_string()).expect("policy");
        let (mut c, _) = servo(Box::new(|_, _| {}));
        let out = dir.join("run");
        let (lines, accepted) =
            run_scripted_on(&mut c, &inputs, &out, 1, vec![(Way::Done, vec![])]).expect("runs");
        assert!(accepted, "{lines:?}");
        assert!(lines[0].starts_with("starting gain: "), "{lines:?}");
        assert_eq!(
            lines[1],
            "velocity_limit_cps: the move speed gives 8051 count/s, over speed_ceiling_cps 8050, where it cannot act: stored as 8050"
        );
        let set = files::set(&out.join("set.json")).expect("set");
        assert_eq!(set.value("velocity_limit_cps"), Some(&8050.into()));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A fault latched under the first enable is no score: the tune stops
    /// after that try, refused, and the servo is put back.
    #[test]
    fn a_fault_at_the_first_enable_stops_the_tune_after_one_try() {
        let (mut c, log) = servo(Box::new(|n, p| {
            if n == 1 {
                p.sim_mut()
                    .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x10);
            }
        }));
        let dir = scratch("tune-fault");
        let out = dir.join("run");
        let (lines, accepted) = run(&mut c, &testdata("mg90-d"), &out, 4).expect("runs");
        let text = lines.join("\n");
        assert!(!accepted, "{text}");
        let doc = json_at(&out.join("tune.json"));
        let tries = doc["tries"].as_array().expect("tries");
        assert_eq!(tries.len(), 1, "{doc}");
        assert_eq!(tries[0]["next"], "stop");
        assert!(
            doc["ended"]
                .as_str()
                .is_some_and(|e| e.contains("a lower gain is no answer")),
            "{doc}"
        );
        assert!(!out.join("tries/02").exists());
        assert!(text.contains("put back: rebooted"), "{text}");
        let reboot = MgmtOp::Reboot as u8;
        assert_eq!(mgmt_ops(&log), [reboot, reboot]);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A factory-fresh servo, never saved, tunes as decided: each try's set
    /// installed volatile and the check run on it, with no SAVE before it
    /// and none after. The stored state its images left is information: the
    /// check's first enable is reached (its first burst armed). A `config`
    /// fault latched there, as a firmware that refuses the mode latches
    /// it, ends the tune as any fault does, named, and the servo is put
    /// back.
    #[test]
    fn a_tune_on_a_factory_fresh_servo_reaches_the_check_unsaved() {
        let (mut c, log) = fresh_servo();
        let d = descriptor();
        let fresh = block_on(state::read(&mut c, ID, &d)).expect("state");
        assert_ne!(
            fresh.store.status & osc_client::store_state::CONFIG_DEFAULTS,
            0,
            "{}",
            fresh.settings_said()
        );
        let armed = Rc::new(RefCell::new(0));
        let seen = armed.clone();
        c.pipe_mut().hook = Box::new(move |n, p| {
            *seen.borrow_mut() = n;
            if n == 1 {
                p.sim_mut().servo_table_mut(0, |t| {
                    t.telemetry.common.fault_flags = 0x40;
                    t.telemetry.mode.fault_code = osc_client::store_state::fault::CONFIG;
                });
            }
        });
        let dir = scratch("tune-factory");
        let out = dir.join("run");
        let (lines, accepted) = run(&mut c, &testdata("mg90-d"), &out, 4).expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(!accepted, "{text}");
        assert!(
            *armed.borrow() >= 1,
            "the check reached its first enable: {text}"
        );
        let doc = json_at(&out.join("tune.json"));
        let tries = doc["tries"].as_array().expect("tries");
        assert_eq!(tries.len(), 1, "{doc}");
        assert_eq!(tries[0]["install"], "installed", "{doc}");
        assert_eq!(
            tries[0]["failed_on"],
            json!(["the first enable holds still"]),
            "{doc}"
        );
        assert!(
            tries[0]["why"]
                .as_str()
                .is_some_and(|w| w.contains("fault_code 7 (config)")),
            "{doc}"
        );
        assert!(text.contains("put back: rebooted"), "{text}");
        let stored = json_at(&out.join("backup/stored.json"));
        assert_eq!(stored["settings"], "defaults: CONFIG, CALIB", "{stored}");
        assert!(
            !mgmt_ops(&log).contains(&(MgmtOp::Save as u8)),
            "{:?}",
            mgmt_ops(&log)
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Refused with nothing written and no run folder: torque on, a latched
    /// fault. A set that moves protections from the servo's is no refusal:
    /// every install takes it.
    #[test]
    fn the_tune_refuses_before_any_write() {
        let dir = scratch("tune-refused");
        let d = descriptor();
        let torque = d.field("torque_enable").expect("torque").addr;
        let lut_page = d.field("pos_lut_page").map(|f| f.addr);
        let (mut c, log) = servo(Box::new(|_, _| {}));
        let nothing_written = |log: &Log| {
            log.borrow().iter().all(|e| match e {
                Ev::Write(a, b) => Some(*a) == lut_page && b.get(1) == Some(&2),
                _ => false,
            })
        };

        block_on(c.write(ID, torque, &[1])).expect("torque on");
        log.borrow_mut().clear();
        let out = dir.join("torque");
        let e = run(&mut c, &testdata("mg90-d"), &out, 4).expect_err("refused");
        assert!(format!("{e:#}").contains("torque_enable is on"), "{e:#}");
        assert!(nothing_written(&log), "{:?}", log.borrow());
        assert!(!out.exists());
        block_on(c.write(ID, torque, &[0])).expect("torque off");

        c.pipe_mut()
            .inner
            .sim_mut()
            .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0x01);
        log.borrow_mut().clear();
        let out = dir.join("fault");
        let e = run(&mut c, &testdata("mg90-d"), &out, 4).expect_err("refused");
        assert!(format!("{e:#}").contains("a fault is latched"), "{e:#}");
        assert!(nothing_written(&log), "{:?}", log.borrow());
        assert!(!out.exists());
        c.pipe_mut()
            .inner
            .sim_mut()
            .servo_table_mut(0, |t| t.telemetry.common.fault_flags = 0);

        // the bench servo's run gathered: its nominal scales move protections
        let run_dir = dir.join("bench-run");
        std::fs::create_dir_all(run_dir.join("results")).expect("mkdir");
        files::step_0(&run_dir);
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                run_dir.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let inputs = dir.join("inputs");
        crate::gather::gather(
            &run_dir,
            &testdata("mg90-d/nameplate.json"),
            &testdata("mg90-d/policy.json"),
            &inputs,
        )
        .expect("gathers");
        log.borrow_mut().clear();
        let out = dir.join("moved");
        let script = vec![(Way::Done, vec![])];
        let (lines, accepted) = run_scripted_on(&mut c, &inputs, &out, 6, script).expect("runs");
        let text = lines.join("\n");
        assert!(accepted, "{text}");
        let installed: Vec<&str> = [
            "stall_omega_max_cps",
            "yield_current_counts",
            "rail_fold_q8",
        ]
        .into_iter()
        .filter(|f| text.contains(&format!("{f}: ")))
        .collect();
        assert_eq!(
            installed.len(),
            3,
            "the install's lines name each move: {text}"
        );
        let save = format!(
            "to store it: osc-ident --id 1 save --set {}",
            out.join("set.json").display()
        );
        assert!(text.contains(&save), "{text}");
        let set = files::set(&out.join("set.json")).expect("set");
        let checked = block_on(install::check(&mut c, ID, &d, &set)).expect("check");
        assert!(checked.clean(), "{:?}", checked.differences);
        assert_eq!(
            mgmt_ops(&log),
            [MgmtOp::Reboot as u8],
            "the backup's reboot alone"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    const MOVED: &str =
        "after the reboot the servo answers at 1M, its saved baud; moved it to 3M, volatile";

    /// A servo whose saved baud is 1M on a bus moved to 3M volatile comes
    /// back from each of the tool's reboots at 1M: it is found there, its
    /// stored state read there, and moved back to 3M, a line each reboot.
    #[test]
    fn a_reboot_onto_a_saved_baud_off_the_run_rate_is_found_and_moved_back() {
        let dir = scratch("tune-baud");
        let three = BaudRate::B3000000.as_hz();

        let (mut c, log) = servo_at(BaudRate::B1000000, Box::new(|_, _| {}));
        let out = dir.join("accept");
        let script = vec![(Way::Refused, vec![sc::REST]), (Way::Done, vec![])];
        let (lines, accepted) = run_scripted(&mut c, &out, 2, script).expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(accepted, "{text}");
        let reboot = MgmtOp::Reboot as u8;
        let reboots = |log: &Log| mgmt_ops(log).iter().filter(|o| **o == reboot).count();
        assert_eq!(lines.iter().filter(|l| *l == MOVED).count(), 1, "{text}");
        assert_eq!(reboots(&log), 1);
        let stored = json_at(&out.join("backup/stored.json"));
        assert_eq!(stored["fields"]["baud_rate_idx"], 1, "the saved baud");
        assert_eq!(c.info().bps, three);
        block_on(c.ping(ID)).expect("the servo answers at 3M");
        let set = files::set(&out.join("set.json")).expect("set");
        let checked = block_on(install::check(&mut c, ID, &descriptor(), &set)).expect("check");
        assert!(checked.clean(), "{:?}", checked.differences);

        let (mut c, log) = servo_at(BaudRate::B1000000, Box::new(|_, _| {}));
        let out = dir.join("refused");
        let script = vec![(Way::Refused, vec![sc::REST])];
        let (lines, accepted) = run_scripted(&mut c, &out, 1, script).expect("runs");
        let text = lines.join("\n");
        println!("{text}");
        assert!(!accepted, "{text}");
        assert_eq!(reboots(&log), 2);
        assert_eq!(lines.iter().filter(|l| *l == MOVED).count(), 2, "{text}");
        let doc = json_at(&out.join("tune.json"));
        assert_eq!(doc["put_back"]["differences"], json!([]), "{doc}");
        assert!(
            text.contains("put back: rebooted onto the stored configuration"),
            "{text}"
        );
        assert_eq!(c.info().bps, three);
        block_on(c.ping(ID)).expect("the servo answers at 3M");
        let checked =
            block_on(install::check(&mut c, ID, &descriptor(), &record())).expect("check");
        assert!(checked.clean(), "{:?}", checked.differences);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A servo that answers at no rate after the backup's reboot: the tune
    /// says so, the rates it was looked for at named, and the host is left
    /// at the run's rate.
    #[test]
    fn a_servo_silent_at_every_rate_after_the_reboot_is_refused_plainly() {
        let (mut c, _) = servo(Box::new(|_, _| {}));
        c.pipe_mut().rebooted = Box::new(|p| p.sim_mut().start_in_bootloader(0));
        let dir = scratch("tune-silent");
        let out = dir.join("run");
        let e = run_scripted(&mut c, &out, 1, vec![(Way::Done, vec![])]).expect_err("refused");
        let e = format!("{e:#}");
        assert_eq!(
            e,
            "no answer at 3M after the REBOOT it acked (10 pings over 5 s), and no ENUM answer at 1M, 0.5M, 2M or 3M either: the servo answers at no rate on this bus"
        );
        assert_eq!(c.info().bps, BaudRate::B3000000.as_hz());
        assert!(!out.join("backup/stored.json").exists());
        let _ = std::fs::remove_dir_all(dir);
    }
}
