//! `run`'s finish ([`osc_measure::run::finish`]: the cruise, the gather
//! again into `final/inputs/` and the final checks, on the set the tune
//! accepted) on the file system, as its tests drive it; `run` reaches it
//! through [`osc_measure::run::run`].

#[cfg(test)]
use {
    crate::files::{self, Fs},
    crate::run::RunArgs,
    crate::tune::Checked,
    anyhow::Result,
    osc_client::descriptor::Descriptor,
    osc_client::pipe::Pipe,
    osc_client::{Client, Id},
    osc_measure::gather::POT_STEP4,
    osc_measure::install::{self, Refused},
    osc_measure::set::SetFile,
    osc_measure::steps::Rig,
    osc_measure::steps::cruise::{self, CruiseInputs, Cruised},
    osc_measure::steps::verify::{self, Verified, VerifyInputs},
    osc_measure::tune,
    serde_json::{Value, json},
    std::path::Path,
    std::sync::atomic::AtomicBool,
};

#[cfg(test)]
pub use osc_measure::run::finish::Finished;

#[cfg(test)]
fn json_at(p: &Path) -> Result<Value> {
    osc_measure::run::finish::json_at(&mut Fs, p)
}

#[cfg(test)]
fn final_set(
    inputs_dir: &Path,
    kp: f64,
    checked: &Checked,
    d: &Descriptor,
) -> Result<(
    SetFile,
    Vec<u8>,
    Option<[i16; osc_client::pos_lut::INTERVALS]>,
)> {
    osc_measure::run::finish::final_set(&mut Fs, inputs_dir, kp, checked, d)
}

#[cfg(test)]
fn tree(dir: &Path, below: &str, out: &mut Vec<String>) -> Result<(), String> {
    osc_measure::run::finish::tree(&Fs, dir, below, out)
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
async fn finish_with<P: Pipe, C, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    root: &Path,
    stop: &AtomicBool,
    run_cruise: C,
    check: V,
) -> Result<Finished>
where
    C: AsyncFnMut(&mut Rig<'_, P>, &CruiseInputs<'_>) -> Result<Cruised, Refused>,
    V: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    let mut fs = Fs;
    let mut host = crate::host(&mut fs, stop);
    osc_measure::run::finish::finish_with(c, id, d, &mut host, a, root, run_cruise, check).await
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::path::PathBuf;
    use std::rc::Rc;
    use std::time::Duration;

    use futures_lite::future::block_on;
    use osc_client::fake::FakePipe;
    use osc_client::pipe::PipeError;
    use osc_client::{BaudRate, Inst, MgmtOp, Opcode, mgmt};
    use osc_measure::guard::{Exit, Way};
    use osc_measure::steps::{Check, Ended, Verdict};

    use super::*;

    const ID: Id = Id::new(1);
    const OSC_EXCHANGE: u8 = 0x20;

    /// The fake adapter, every MGMT op logged.
    struct Spy {
        inner: FakePipe,
        mgmt: Rc<RefCell<Vec<u8>>>,
    }

    impl Pipe for Spy {
        async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
            if let [_, _, OSC_EXCHANGE, _, _, _, _, inst, _, _, _, _, op, ..] = bytes
                && Inst(*inst).opcode() == Some(Opcode::Mgmt)
            {
                self.mgmt.borrow_mut().push(*op);
            }
            self.inner.send(bytes).await
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

    struct Given {
        nameplate: PathBuf,
        policy: PathBuf,
    }

    fn given() -> Given {
        Given {
            nameplate: testdata("mg90-d/nameplate.json"),
            policy: testdata("mg90-d/policy.json"),
        }
    }

    fn args<'a>(g: &'a Given, out: &'a Path) -> RunArgs<'a> {
        RunArgs {
            nameplate: &g.nameplate,
            policy: &g.policy,
            out,
            tries: 6,
        }
    }

    /// [`tuned_at`] with the tune's first try accepted.
    fn tuned(dir: &Path, g: &Given) -> (PathBuf, SetFile, Client<Spy>, Rc<RefCell<Vec<u8>>>) {
        tuned_at(dir, g, 1)
    }

    /// A run folder as the tune leaves it: the bench servo's results,
    /// gathered into `inputs/`, the tune's set of try `accepted` (from 1)
    /// in `tune/`, its tune.json the tune's own fields; and the servo of
    /// record holding that set, volatile over the stored record.
    fn tuned_at(
        dir: &Path,
        g: &Given,
        accepted: u32,
    ) -> (PathBuf, SetFile, Client<Spy>, Rc<RefCell<Vec<u8>>>) {
        let root = dir.join("uid/run");
        std::fs::create_dir_all(root.join("results")).expect("mkdir");
        files::step_0(&root);
        std::fs::write(
            root.join("run.json"),
            r#"{"tool": "osc-ident", "steps": {}}"#,
        )
        .expect("manifest");
        for f in ["rest", "probe", "stops", "window", "spins", "table"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                root.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let inputs = root.join("inputs");
        crate::gather::gather(&root, &g.nameplate, &g.policy, &inputs).expect("gathers");
        let d = crate::descriptor_builtin().expect("descriptor");
        let r = files::rules_inputs(&inputs, None).expect("inputs");
        let measured = json_at(&inputs.join("measured.json")).expect("measured");
        let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
        let k0 = osc_measure::rules::speed_loop::starting_gain(&r.inputs).expect("k0");
        let kp = tune::gain(k0, accepted - 1);
        let (set, _, _) = crate::tune::set_at(&r, &measured, kp, &Checked::default(), &pair, &d)
            .expect("the tune's set");
        std::fs::create_dir_all(root.join("tune")).expect("mkdir");
        std::fs::write(root.join("tune/set.json"), set.render()).expect("set");
        let tries: Vec<Value> = (1..=accepted)
            .map(|t| {
                json!({"try": t, "kp": tune::gain(k0, t - 1),
                       "next": if t == accepted { "accept" } else { "lower" }})
            })
            .collect();
        std::fs::write(
            root.join("tune/tune.json"),
            json!({"starting_gain": k0, "share": tune::SHARE, "tries_allowed": 6, "tries": tries,
                   "accepted": {"try": accepted, "kp": kp, "speed_kp_verified": kp, "set": set.hash()}})
                .to_string(),
        )
        .expect("tune.json");

        let mut inner = FakePipe::new(BaudRate::B3000000, &[1]);
        inner.seed_calibrated(0);
        let log = Rc::new(RefCell::new(Vec::new()));
        let mut c = block_on(Client::connect(Spy {
            inner,
            mgmt: log.clone(),
        }))
        .expect("connect");
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
            &set,
            r.table.as_ref().map(|t| &t.points),
            &mut Vec::new(),
        ))
        .expect("the tune's set installs");
        log.borrow_mut().clear();
        (root, set, c, log)
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

    /// A cruise step that keeps `doc` as its result and ends `way`.
    fn cruised(rig: &mut Rig<'_, Spy>, way: Way, doc: &Value) -> Result<Cruised, Refused> {
        rig.sink
            .write(cruise::RESULT, doc.to_string().as_bytes())
            .map_err(|e| Refused(e.0))?;
        Ok(Cruised {
            ended: ended(way),
            checks: vec![],
            result: Some(doc.clone()),
        })
    }

    fn checked(rig: &mut Rig<'_, Spy>, way: Way) -> Result<Verified, Refused> {
        checked_on(rig, way, &[])
    }

    /// The result a scripted check keeps.
    fn checked_doc(way: Way, failed: &[&str]) -> Value {
        json!({"verdict": way.name(), "failed_on": failed})
    }

    /// Every file under `dir`, by its path below it, with its bytes.
    fn tree_of(dir: &Path) -> Vec<(String, Vec<u8>)> {
        let mut names = Vec::new();
        tree(dir, "", &mut names).expect("tree");
        names
            .into_iter()
            .map(|n| {
                let b = std::fs::read(dir.join(&n)).expect("read");
                (n, b)
            })
            .collect()
    }

    /// A check that ends `way`, failing the checks `failed` (none named: a
    /// scripted one, failed unless the check is accepted).
    fn checked_on(rig: &mut Rig<'_, Spy>, way: Way, failed: &[&str]) -> Result<Verified, Refused> {
        rig.sink
            .write(
                verify::RESULT,
                checked_doc(way, failed).to_string().as_bytes(),
            )
            .map_err(|e| Refused(e.0))?;
        let check = |name: &str, verdict| Check {
            name: name.into(),
            value: "scripted".into(),
            bound: "scripted".into(),
            verdict,
        };
        let checks = if failed.is_empty() {
            vec![check(
                "scripted",
                if way == Way::Done {
                    Verdict::Pass
                } else {
                    Verdict::Fail
                },
            )]
        } else {
            failed.iter().map(|n| check(n, Verdict::Fail)).collect()
        };
        Ok(Verified {
            ended: ended(way),
            checks,
            result: None,
        })
    }

    /// The finish after an accepted cruise, its checks ending as `script`
    /// says in turn; the finish and how many checks ran.
    fn finish_script(
        c: &mut Client<Spy>,
        a: &RunArgs,
        root: &Path,
        script: &[(Way, &[&str])],
    ) -> (Finished, usize) {
        let d = crate::descriptor_builtin().expect("descriptor");
        let doc = cruise_doc();
        let mut at = 0;
        let f = block_on(finish_with(
            c,
            ID,
            &d,
            a,
            root,
            &AtomicBool::new(false),
            async |rig: &mut Rig<'_, Spy>, _: &CruiseInputs<'_>| cruised(rig, Way::Done, &doc),
            async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
                let (way, failed) = script[at.min(script.len() - 1)];
                at += 1;
                checked_on(rig, way, failed)
            },
        ))
        .expect("finishes");
        (f, at)
    }

    fn finish_scripted(
        c: &mut Client<Spy>,
        a: &RunArgs,
        root: &Path,
        cruise_way: Way,
        check_way: Way,
    ) -> (Finished, usize) {
        let d = crate::descriptor_builtin().expect("descriptor");
        let doc = json_at(&testdata("gather/cruise.json")).expect("cruise.json");
        let mut checks = 0;
        let f = block_on(finish_with(
            c,
            ID,
            &d,
            a,
            root,
            &AtomicBool::new(false),
            async |rig: &mut Rig<'_, Spy>, _: &CruiseInputs<'_>| cruised(rig, cruise_way, &doc),
            async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
                checks += 1;
                checked(rig, check_way)
            },
        ))
        .expect("finishes");
        (f, checks)
    }

    fn holds(c: &mut Client<Spy>, set: &SetFile) -> bool {
        let d = crate::descriptor_builtin().expect("descriptor");
        block_on(install::check(c, ID, &d, set))
            .expect("check")
            .clean()
    }

    /// The cruise accepted: the gather again takes its friction line into
    /// `final/inputs/`, the rules at the tune's gain give the final set,
    /// installed volatile and checked; accepted, it stays on the servo,
    /// the command that stores it printed, nothing saved. Accepted the
    /// first time, nothing is backed off: no try is added to the tune's.
    /// The final set takes the still speed of the tune's accepted check as
    /// a measured term naming its result; a tune whose check holds none
    /// leaves it out and says so.
    #[test]
    fn the_final_set_takes_the_still_speed_of_the_tunes_check() {
        let dir = scratch("finish-still");
        let g = given();
        let (root, _, mut c, _) = tuned(&dir, &g);
        let at = root.join(format!("tune/tries/01/{}", verify::RESULT));
        std::fs::create_dir_all(at.parent().expect("parent")).expect("mkdir");
        let check = json!({"step": "verify", "verdict": "accepted", "estimates": {"still_speed_cps":
            {"value": 115.0, "sd": null, "n": 6, "unit": "count/s", "source": "the still holds"}}});
        std::fs::write(&at, check.to_string()).expect("verify.json");
        let (f, _) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Done);
        let text = f.lines.join("\n");
        assert!(
            text.contains("still_speed_cps 115.0 count/s: the largest observer speed over the still holds of the tune's check, try 1"),
            "{text}"
        );
        let m = json_at(&root.join("final/measured.json")).expect("m");
        let q = &m["still_speed_cps"];
        assert_eq!(
            (q["value"].as_f64(), q["n"].as_u64()),
            (Some(115.0), Some(6))
        );
        assert!(
            q["source"]["file"]
                .as_str()
                .is_some_and(|f| f.ends_with("tune/tries/01/results/verify.json")),
            "{q}"
        );

        let _ = std::fs::remove_dir_all(dir);
        let dir = scratch("finish-no-still");
        let (root, _, mut c, _) = tuned(&dir, &g);
        let at = root.join(format!("tune/tries/01/{}", verify::RESULT));
        std::fs::create_dir_all(at.parent().expect("parent")).expect("mkdir");
        std::fs::write(
            &at,
            json!({"step": "verify", "verdict": "accepted"}).to_string(),
        )
        .expect("verify.json");
        let (f, _) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Done);
        assert!(
            f.lines
                .iter()
                .any(|l| l == "still_speed_cps: the tune's check, try 1, holds no still holds"),
            "{:?}",
            f.lines
        );
        let m = json_at(&root.join("final/measured.json")).expect("m");
        assert!(m.get("still_speed_cps").is_none(), "{m}");
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The final set's pot statistic is the worst over the whole
    /// population: the spins' and the table's as gathered, and the tune's
    /// accepted check's, its wide-code holds too; the larger term, named by
    /// its result.
    #[test]
    fn the_final_pot_steps_are_the_worst_of_the_gather_and_the_tunes_check() {
        let g = given();
        let check = |v: f64| {
            json!({"step": "verify", "verdict": "accepted", "estimates": {"pot_step4_counts":
                {"value": v, "sd": null, "n": 34, "unit": "raw counts", "source": "the check"}}})
        };
        let finished = |name: &str, spins: Option<f64>, checked: f64| {
            let dir = scratch(name);
            let (root, _, mut c, _) = tuned(&dir, &g);
            if let Some(v) = spins {
                let p = root.join("results/spins.json");
                let mut doc = json_at(&p).expect("spins.json");
                doc["estimates"]["pot_step4_counts"] =
                    json!({"value": v, "sd": null, "n": 12, "unit": "raw counts"});
                std::fs::write(&p, doc.to_string()).expect("spins.json");
            }
            let at = root.join(format!("tune/tries/01/{}", verify::RESULT));
            std::fs::create_dir_all(at.parent().expect("parent")).expect("mkdir");
            std::fs::write(&at, check(checked).to_string()).expect("verify.json");
            let (f, _) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Done);
            let m = json_at(&root.join("final/measured.json")).expect("m");
            let _ = std::fs::remove_dir_all(dir);
            (f.lines.join("\n"), m[POT_STEP4].clone())
        };
        let (text, q) = finished("finish-step4-check", Some(15.0), 52.0);
        assert_eq!(
            (q["value"].as_f64(), q["n"].as_u64()),
            (Some(52.0), Some(34)),
            "{q}"
        );
        assert!(
            q["source"]["file"]
                .as_str()
                .is_some_and(|f| f.ends_with("tune/tries/01/results/verify.json")),
            "{q}"
        );
        assert!(
            text.contains("pot_step4_counts 52 raw counts: the worst of the spins' and the table's, 15, and the tune's check's, try 1, its wide-code holds too, 52"),
            "{text}"
        );
        let (_, q) = finished("finish-step4-spins", Some(60.0), 52.0);
        assert_eq!(q["value"].as_f64(), Some(60.0), "{q}");
        assert!(
            q["source"]["file"]
                .as_str()
                .is_some_and(|f| f.ends_with("results/spins.json")),
            "{q}"
        );
        let (_, q) = finished("finish-step4-alone", None, 52.0);
        assert_eq!(q["value"].as_f64(), Some(52.0), "{q}");
    }

    #[test]
    fn an_accepted_cruise_installs_and_checks_the_final_set() {
        let dir = scratch("finish-accepted");
        let g = given();
        let (root, tune_set, mut c, mgmt_log) = tuned(&dir, &g);
        let tune_doc = std::fs::read(root.join("tune/tune.json")).expect("tune.json");
        let (f, checks) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Done);
        let text = f.lines.join("\n");
        println!("{text}");
        assert_eq!(checks, 1);
        assert_eq!(f.cruise["state"], "accepted");
        assert_eq!(f.last["state"], "accepted", "{}", f.last);
        let m = json_at(&root.join("final/inputs/measured.json")).expect("measured");
        assert!(
            m["cruise_coulomb_ma"]["source"]["file"]
                .as_str()
                .is_some_and(|f| f.ends_with("results/cruise.json")),
            "{m}"
        );
        assert_eq!(
            std::fs::read(root.join("tune/tune.json")).expect("tune.json"),
            tune_doc,
            "the tune's record is untouched"
        );
        assert!(!text.contains("backing off"), "{text}");
        // final/ as a check written straight into it leaves it: the gather,
        // the rules' set and measured file at the tune's gain, the check's
        // result, nothing else
        let kp = json_at(&root.join("tune/tune.json")).expect("t")["accepted"]["kp"]
            .as_f64()
            .expect("kp");
        let d = crate::descriptor_builtin().expect("descriptor");
        let (set, measured, _) =
            final_set(&root.join("final/inputs"), kp, &Checked::default(), &d).expect("set");
        let mut today: Vec<(String, Vec<u8>)> = tree_of(&root.join("final/inputs"))
            .into_iter()
            .map(|(n, b)| (format!("inputs/{n}"), b))
            .collect();
        today.extend([
            ("measured.json".to_string(), measured),
            (
                verify::RESULT.to_string(),
                checked_doc(Way::Done, &[]).to_string().into_bytes(),
            ),
            ("set.json".to_string(), set.render().into_bytes()),
        ]);
        today.sort();
        assert_eq!(tree_of(&root.join("final")), today, "final/ byte for byte");
        assert_eq!(
            tree_of(&root.join("tune/tries/02")),
            today[today.len() - 3..].to_vec(),
            "the first final check is a try after the tune's"
        );
        assert_eq!(f.last["set_file"], "final/set.json");
        assert_eq!(f.last["try"], 2);
        assert_eq!(f.last["checks"], json!([2]));
        assert_eq!(
            json_at(&root.join("tune/back-off.json")).expect("back-off")["tries"]
                .as_array()
                .map(Vec::len),
            Some(1)
        );
        assert_eq!(
            json_at(&root.join("final/measured.json")).expect("m")["speed_kp_verified"].as_f64(),
            Some(kp)
        );
        let fin = files::set(&root.join("final/set.json")).expect("final set");
        assert_ne!(
            fin.value("fric_fc_counts"),
            tune_set.value("fric_fc_counts")
        );
        assert_eq!(
            fin.value("v_kp_q88"),
            tune_set.value("v_kp_q88"),
            "the tune's gain"
        );
        assert!(holds(&mut c, &fin), "the final set is live");
        assert!(
            text.contains("friction line: from the cruise step's"),
            "{text}"
        );
        assert!(
            text.contains("the tune's check: accepted; the final check: accepted"),
            "{text}"
        );
        let save = format!(
            "to store it: osc-ident --id 1 save --set {} --table {}",
            root.join("final/set.json").display(),
            root.join("final/inputs/pos_lut.json").display()
        );
        assert!(f.lines.contains(&save), "{text}");
        let live = format!(
            "live on the servo, volatile: the final set {}",
            &fin.hash()[..12]
        );
        assert!(f.lines.contains(&live), "{text}");
        assert!(root.join("final/results/verify.json").exists());
        assert!(
            !mgmt_log.borrow().contains(&(MgmtOp::Save as u8)),
            "never saved"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A finish a page runs says the CLI's lines but the command that stores
    /// the final set: a page has no command line.
    #[test]
    fn a_finish_on_a_page_names_no_command() {
        let g = given();
        let finished = |name: &str, tool: &'static str| {
            let dir = scratch(name);
            let (root, _, mut c, _) = tuned(&dir, &g);
            let d = crate::descriptor_builtin().expect("descriptor");
            let doc = json_at(&testdata("gather/cruise.json")).expect("cruise.json");
            let stop = AtomicBool::new(false);
            let mut fs = Fs;
            let mut host = crate::host(&mut fs, &stop);
            host.tool = tool;
            let f = block_on(osc_measure::run::finish::finish_with(
                &mut c,
                ID,
                &d,
                &mut host,
                &args(&g, &dir),
                &root,
                async |rig: &mut Rig<'_, Spy>, _: &CruiseInputs<'_>| cruised(rig, Way::Done, &doc),
                async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| checked(rig, Way::Done),
            ))
            .expect("finishes");
            assert_eq!(f.last["state"], "accepted", "{}", f.last);
            let shown = dir.display().to_string();
            let lines: Vec<String> = f.lines.iter().map(|l| l.replace(&shown, "<dir>")).collect();
            let _ = std::fs::remove_dir_all(dir);
            lines
        };
        let cli = finished("finish-page-cli", "osc-ident test");
        let page = finished("finish-page", "osc-client-web-ident test");
        let store = |l: &String| l.starts_with("to store it: osc-ident ");
        assert_eq!(cli.iter().filter(|l| store(l)).count(), 1, "{cli:?}");
        let want: Vec<String> = cli.iter().filter(|l| !store(l)).cloned().collect();
        assert_eq!(page, want);
    }

    /// The final set refused by its check: the tune's set goes back on the
    /// servo, whole; nothing saved.
    #[test]
    fn a_final_set_refused_puts_the_tunes_set_back() {
        let dir = scratch("finish-refused");
        let g = given();
        let (root, tune_set, mut c, mgmt_log) = tuned(&dir, &g);
        let (f, checks) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Refused);
        let text = f.lines.join("\n");
        assert_eq!(checks, 1, "a refusal on another line is not backed off");
        assert!(!text.contains("backing off") && !root.join("tune/tries/03").exists());
        assert_eq!(
            tree_of(&root.join("final"))
                .into_iter()
                .filter(|(n, _)| !n.starts_with("inputs/"))
                .collect::<Vec<_>>(),
            tree_of(&root.join("tune/tries/02")),
            "final/ holds the refused first check, as it did"
        );
        assert_eq!(f.last["checks"], json!([2]));
        assert_eq!(f.last["state"], "refused", "{}", f.last);
        assert!(
            text.contains("the tune's check: accepted; the final check: refused"),
            "{text}"
        );
        assert!(
            text.contains("no final set: the final set is not accepted"),
            "{text}"
        );
        assert!(
            text.contains("live on the servo, volatile: the tune's set"),
            "{text}"
        );
        assert!(!text.contains("to store it"), "{text}");
        assert!(holds(&mut c, &tune_set), "the tune's set is live again");
        assert!(!mgmt_log.borrow().contains(&(MgmtOp::Save as u8)));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The cruise not accepted: no gather, no final set, no check; the
    /// tune's set stays on the servo.
    #[test]
    fn a_cruise_not_accepted_leaves_the_tunes_set() {
        let dir = scratch("finish-cruise");
        let g = given();
        let (root, tune_set, mut c, _) = tuned(&dir, &g);
        let (f, checks) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Refused, Way::Done);
        let text = f.lines.join("\n");
        assert_eq!(checks, 0);
        assert_eq!(f.cruise["state"], "refused");
        assert_eq!(f.last["state"], "not run");
        assert!(!root.join("final").exists());
        assert!(
            text.contains("the cruise step was refused: the friction line stays the spin step's")
                && text.contains("no final set: the cruise step was refused"),
            "{text}"
        );
        assert!(holds(&mut c, &tune_set));
        let _ = std::fs::remove_dir_all(dir);
    }

    fn torque_on(c: &mut Client<Spy>) {
        let d = crate::descriptor_builtin().expect("descriptor");
        let t = d.field("torque_enable").expect("torque").addr;
        block_on(c.write(ID, t, &[1])).expect("torque on");
    }

    /// The finish with the cruise result `doc` written by a cruise that ends
    /// `way`, then `after` on the servo; the check ends `check_way`, then
    /// `after_check`. The finish, and how many cruises and checks ran.
    #[allow(clippy::too_many_arguments)]
    fn finish_on(
        c: &mut Client<Spy>,
        a: &RunArgs,
        root: &Path,
        doc: Value,
        way: Way,
        after: fn(&mut Client<Spy>),
        check_way: Way,
        after_check: fn(&mut Client<Spy>),
    ) -> (Finished, usize, usize) {
        let d = crate::descriptor_builtin().expect("descriptor");
        let (mut cruises, mut checks) = (0, 0);
        let f = block_on(finish_with(
            c,
            ID,
            &d,
            a,
            root,
            &AtomicBool::new(false),
            async |rig: &mut Rig<'_, Spy>, _: &CruiseInputs<'_>| {
                cruises += 1;
                let r = cruised(rig, way, &doc);
                after(rig.c);
                r
            },
            async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
                checks += 1;
                let r = checked(rig, check_way);
                after_check(rig.c);
                r
            },
        ))
        .expect("finishes");
        (f, cruises, checks)
    }

    fn cruise_doc() -> Value {
        json_at(&testdata("gather/cruise.json")).expect("cruise.json")
    }

    fn nothing(_: &mut Client<Spy>) {}

    /// The servo rebooted onto its stored set after the tune's check: the
    /// tune's set goes back on before the cruise; where it cannot (torque
    /// on), no cruise runs, recorded.
    #[test]
    fn the_cruise_runs_on_the_tunes_set_or_not_at_all() {
        let dir = scratch("finish-live");
        let g = given();
        let (root, tune_set, mut c, _) = tuned(&dir, &g);
        let rec = files::set(&testdata("mg90-d/constants.json")).expect("record");
        let d = crate::descriptor_builtin().expect("descriptor");
        let table = files::table(&testdata("mg90-d/pos_lut.json")).expect("table");
        block_on(install::install(
            &mut c,
            ID,
            &d,
            &rec,
            Some(&table.points),
            &mut Vec::new(),
        ))
        .expect("the stored set back, as a reboot leaves it");
        assert!(!holds(&mut c, &tune_set));
        let a = args(&g, &dir);
        let (f, cruises, _) = finish_on(
            &mut c,
            &a,
            &root,
            cruise_doc(),
            Way::Done,
            nothing,
            Way::Done,
            nothing,
        );
        let text = f.lines.join("\n");
        assert_eq!(cruises, 1);
        assert!(
            f.lines[0].starts_with("live on the servo, volatile: the tune's set")
                && f.lines[0].ends_with("installed again"),
            "{text}"
        );
        assert_eq!(f.last["state"], "accepted", "{}", f.last);

        let dir2 = scratch("finish-not-live");
        let (root, _, mut c, _) = tuned(&dir2, &g);
        torque_on(&mut c);
        let (f, cruises, checks) = finish_on(
            &mut c,
            &a,
            &root,
            cruise_doc(),
            Way::Done,
            nothing,
            Way::Done,
            nothing,
        );
        let text = f.lines.join("\n");
        assert_eq!((cruises, checks), (0, 0), "{text}");
        assert_eq!(f.cruise["state"], "not run");
        assert_eq!(f.last["state"], "not run");
        assert!(text.contains("the cruise is not run"), "{text}");
        let _ = std::fs::remove_dir_all(dir);
        let _ = std::fs::remove_dir_all(dir2);
    }

    /// After an accepted cruise each refusal is a finish short of a final
    /// set, its lines kept and its state recorded, the tune's set kept:
    /// the gather refused (a cruise on another sense block), the rules
    /// refused (a gain over the starting gain), the install refused
    /// (torque left on).
    #[test]
    fn refusals_after_the_cruise_are_recorded_finishes() {
        let g = given();
        let dir = scratch("finish-gather");
        let (root, tune_set, mut c, _) = tuned(&dir, &g);
        let mut doc = cruise_doc();
        doc["scales"]["counts_per_a"] = 1135.0.into();
        let a = args(&g, &dir);
        let (f, _, checks) = finish_on(
            &mut c,
            &a,
            &root,
            doc,
            Way::Done,
            nothing,
            Way::Done,
            nothing,
        );
        let text = f.lines.join("\n");
        assert_eq!(checks, 0);
        assert_eq!(f.cruise["state"], "accepted");
        assert_eq!(f.last["state"], "refused", "{}", f.last);
        assert!(
            f.last["why"]
                .as_str()
                .is_some_and(|w| w.contains("gather") && w.contains("another sense block")),
            "{}",
            f.last
        );
        assert!(text.contains("  step cruise accepted"), "{text}");
        assert!(holds(&mut c, &tune_set));
        let _ = std::fs::remove_dir_all(&dir);

        let dir = scratch("finish-rules");
        let (root, tune_set, mut c, _) = tuned(&dir, &g);
        let p = root.join("tune/tune.json");
        let mut t = json_at(&p).expect("tune.json");
        t["accepted"]["kp"] = 1.25.into();
        std::fs::write(&p, t.to_string()).expect("tune.json");
        let a = args(&g, &dir);
        let (f, _, checks) = finish_on(
            &mut c,
            &a,
            &root,
            cruise_doc(),
            Way::Done,
            nothing,
            Way::Done,
            nothing,
        );
        assert_eq!(checks, 0);
        assert!(
            f.last["why"]
                .as_str()
                .is_some_and(|w| w.contains("the final set was not built")
                    && w.contains("over the starting gain")),
            "{}",
            f.last
        );
        assert!(holds(&mut c, &tune_set));
        let _ = std::fs::remove_dir_all(dir);

        let dir = scratch("finish-install");
        let (root, _, mut c, _) = tuned(&dir, &g);
        let a = args(&g, &dir);
        let (f, _, checks) = finish_on(
            &mut c,
            &a,
            &root,
            cruise_doc(),
            Way::Done,
            torque_on,
            Way::Done,
            nothing,
        );
        let text = f.lines.join("\n");
        assert_eq!(checks, 0);
        assert!(
            f.last["why"].as_str().is_some_and(
                |w| w.contains("install was refused") && w.contains("torque_enable is on")
            ),
            "{}",
            f.last
        );
        assert!(
            text.contains("the tune's set") && text.contains("is not live"),
            "{text}"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The final check accepted, but the final set does not go back on the
    /// servo after it (torque left on): not accepted, no save command.
    #[test]
    fn an_accepted_check_whose_set_is_not_live_is_not_accepted() {
        let dir = scratch("finish-not-kept");
        let g = given();
        let (root, _, mut c, _) = tuned(&dir, &g);
        let a = args(&g, &dir);
        let (f, _, checks) = finish_on(
            &mut c,
            &a,
            &root,
            cruise_doc(),
            Way::Done,
            nothing,
            Way::Done,
            torque_on,
        );
        let text = f.lines.join("\n");
        assert_eq!(checks, 1);
        assert_eq!(f.last["state"], "refused", "{}", f.last);
        assert_eq!(f.last["check"], "accepted");
        assert!(
            f.last["why"]
                .as_str()
                .is_some_and(|w| w.contains("not live on the servo")),
            "{}",
            f.last
        );
        assert!(!text.contains("to store it"), "{text}");
        let _ = std::fs::remove_dir_all(dir);
    }

    const REST: &[&str] = &[osc_measure::scorecard::REST];
    const ACCEPTED: (Way, &[&str]) = (Way::Done, &[]);
    const ON_REST: (Way, &[&str]) = (Way::Refused, REST);

    fn v_kp(f: &SetFile) -> u64 {
        f.value("v_kp_q88").and_then(Value::as_u64).expect("v_kp")
    }

    fn starting_gain(root: &Path) -> f64 {
        json_at(&root.join("tune/tune.json")).expect("tune.json")["starting_gain"]
            .as_f64()
            .expect("k0")
    }

    /// final/ past its gathered inputs.
    fn held(root: &Path) -> Vec<(String, Vec<u8>)> {
        tree_of(&root.join("final"))
            .into_iter()
            .filter(|(n, _)| !n.starts_with("inputs/"))
            .collect()
    }

    /// The feed is told the cruise as it starts and ends, then the final
    /// check's start and each of its tries down the back-off, of the
    /// tries the tune leaves; the lines told, in order, are the finish's.
    #[test]
    fn the_feed_is_told_the_cruise_and_each_final_check() {
        use osc_measure::run::Progress;
        let dir = scratch("finish-feed");
        let g = given();
        let (root, _, mut c, _) = tuned(&dir, &g);
        let d = crate::descriptor_builtin().expect("descriptor");
        let doc = cruise_doc();
        let script = [ON_REST, ON_REST, ACCEPTED];
        let mut at = 0;
        let told = RefCell::new(Vec::new());
        let feed = |p: &Progress| told.borrow_mut().push(p.clone());
        let stop = AtomicBool::new(false);
        let mut fs = Fs;
        let mut host = crate::host(&mut fs, &stop);
        host.feed = &feed;
        let f = block_on(osc_measure::run::finish::finish_with(
            &mut c,
            ID,
            &d,
            &mut host,
            &args(&g, &dir),
            &root,
            async |rig: &mut Rig<'_, Spy>, _: &CruiseInputs<'_>| {
                (rig.tell)(osc_measure::steps::Unit {
                    at: 1,
                    of: 4,
                    what: "speed",
                });
                cruised(rig, Way::Done, &doc)
            },
            async |rig: &mut Rig<'_, Spy>, _: &VerifyInputs<'_>| {
                let (way, failed) = script[at];
                at += 1;
                (rig.tell)(osc_measure::steps::Unit {
                    at: 1,
                    of: 20,
                    what: "move",
                });
                checked_on(rig, way, failed)
            },
        ))
        .expect("finishes");
        assert_eq!(f.last["state"], "accepted", "{}", f.last);
        let moved = Progress::Unit {
            step: "final",
            at: 1,
            of: 20,
            what: "move",
        };
        let told = told.take();
        let events: Vec<&Progress> = told
            .iter()
            .filter(|p| !matches!(p, Progress::Lines { .. }))
            .collect();
        let tried = |at| Progress::Try {
            step: "final",
            at,
            of: 6,
        };
        assert_eq!(
            events,
            [
                &Progress::Started { step: "cruise" },
                &Progress::Unit {
                    step: "cruise",
                    at: 1,
                    of: 4,
                    what: "speed"
                },
                &Progress::Ended {
                    step: "cruise",
                    state: "accepted".into(),
                    why: None
                },
                &Progress::Started { step: "final" },
                &tried(1),
                &moved,
                &tried(2),
                &moved,
                &tried(3),
                &moved,
            ]
        );
        let said: Vec<String> = told
            .iter()
            .filter_map(|p| match p {
                Progress::Lines { lines } => Some(lines.clone()),
                _ => None,
            })
            .flatten()
            .collect();
        assert_eq!(said, f.lines);
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Each final check is a try after the tune's, kept in `tune/tries/NN`
    /// and listed in `tune/back-off.json`; one refused on rest error alone
    /// goes on to the tune's next gain on the cruise's friction line, each
    /// step said plainly. The first set its check accepts is the final set:
    /// `final/` is written once, with that try's files, live, the command
    /// that stores it printed.
    #[test]
    fn a_final_check_refused_on_rest_error_backs_the_gain_off_a_try_at_a_time() {
        let dir = scratch("finish-backoff");
        let g = given();
        let (root, tune_set, mut c, mgmt_log) = tuned(&dir, &g);
        let tune_before = std::fs::read(root.join("tune/tune.json")).expect("tune.json");
        let k0 = starting_gain(&root);
        let (f, checks) = finish_script(
            &mut c,
            &args(&g, &dir),
            &root,
            &[ON_REST, ON_REST, ACCEPTED],
        );
        let text = f.lines.join("\n");
        println!("{text}");
        assert_eq!(checks, 3);
        let mut gains = Vec::new();
        let mut sets = Vec::new();
        for (t, step) in [(2u32, 0u32), (3, 1), (4, 2)] {
            let at = root.join(format!("tune/tries/{t:02}"));
            for file in ["measured.json", "set.json", "results/verify.json"] {
                assert!(at.join(file).exists(), "try {t}: {file}");
            }
            let set = files::set(&at.join("set.json")).expect("set");
            let kp = tune::gain(k0, step);
            assert_eq!(
                json_at(&at.join("measured.json")).expect("m")["speed_kp_verified"].as_f64(),
                Some(kp)
            );
            assert_eq!(v_kp(&set), (kp * 256.0).round() as u64, "try {t}");
            gains.push(v_kp(&set));
            sets.push(set);
        }
        assert_eq!(
            gains[0],
            v_kp(&tune_set),
            "the first final check is at the tune's gain"
        );
        for w in ["fric_fc_counts", "fric_fv_q016"] {
            assert_ne!(sets[0].value(w), tune_set.value(w), "{w}");
            for s in &sets[1..] {
                assert_eq!(
                    s.value(w),
                    sets[0].value(w),
                    "the cruise's friction line: {w}"
                );
            }
        }
        for w in gains.windows(2) {
            let said = format!(
                "verify refused on rest error at v_kp {}; backing off to {}",
                w[0], w[1]
            );
            assert!(f.lines.contains(&said), "{said}: {text}");
        }
        assert!(!root.join("tune/tries/05").exists());
        assert_eq!(
            held(&root),
            tree_of(&root.join("tune/tries/04")),
            "final/ holds the accepted try"
        );

        assert_eq!(
            std::fs::read(root.join("tune/tune.json")).expect("tune.json"),
            tune_before,
            "the tune's record is never written over"
        );
        let doc = json_at(&root.join("tune/back-off.json")).expect("back-off.json");
        assert_eq!(doc["starting_gain"].as_f64(), Some(k0), "{doc}");
        assert_eq!(doc["tries_allowed"], 6);
        assert!(
            doc["inputs"]
                .as_str()
                .is_some_and(|i| i.ends_with("final/inputs")),
            "{doc}"
        );
        assert_eq!(doc["after"]["accepted_try"], 1);
        assert_eq!(doc["accepted"]["try"], 4);
        assert_eq!(
            doc["accepted"]["speed_kp_verified"].as_f64(),
            Some(tune::gain(k0, 2))
        );
        assert_eq!(doc["ended"], Value::Null);
        let tries = doc["tries"].as_array().expect("tries");
        assert_eq!(tries.len(), 3, "{doc}");
        let after = [
            "the tune's check accepted its try 1".to_string(),
            format!("the check at v_kp {} refused on rest error", gains[0]),
            format!("the check at v_kp {} refused on rest error", gains[1]),
        ];
        for (k, (t, (n, next))) in tries
            .iter()
            .zip([(2, "lower"), (3, "lower"), (4, "accept")])
            .enumerate()
        {
            assert_eq!(t["try"], n, "{t}");
            assert_eq!(t["next"], next, "{t}");
            assert_eq!(t["v_kp_q88"], gains[k], "{t}");
            assert_eq!(t["set_file"], format!("tries/{n:02}/set.json"), "{t}");
            assert_eq!(
                t["check"],
                format!("tries/{n:02}/results/verify.json"),
                "{t}"
            );
            assert_eq!(t["install"], "installed", "{t}");
            assert_eq!(t["after"], after[k], "{t}");
            assert!(
                (t["share_of_start"].as_f64().expect("share") - tune::SHARE.powi(k as i32)).abs()
                    < 1e-12
            );
        }
        assert_eq!(tries[0]["failed_on"], json!([osc_measure::scorecard::REST]));
        assert_eq!(tries[2]["verdict"], "accepted");

        let won = files::set(&root.join("final/set.json")).expect("set");
        assert_eq!(won.hash(), sets[2].hash());
        assert_eq!(f.last["state"], "accepted", "{}", f.last);
        assert_eq!(f.last["try"], 4);
        assert_eq!(f.last["set"], won.hash());
        assert_eq!(f.last["set_file"], "final/set.json");
        assert_eq!(f.last["checks"], json!([2, 3, 4]));
        assert_eq!(f.last["back_off"], "tune/back-off.json");
        assert!(holds(&mut c, &won), "the final set is live");
        assert!(
            text.contains("backed off to try 4: the final check: accepted at v_kp"),
            "{text}"
        );
        let save = format!(
            "to store it: osc-ident --id 1 save --set {} --table {}",
            root.join("final/set.json").display(),
            root.join("final/inputs/pos_lut.json").display()
        );
        assert!(f.lines.contains(&save), "{text}");
        assert_eq!(
            f.lines.last().map(String::as_str),
            Some("a reboot returns the stored set")
        );
        assert!(!text.contains("no final set"), "{text}");
        assert!(
            !mgmt_log.borrow().contains(&(MgmtOp::Save as u8)),
            "never saved"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The verdicts the dsr-gate run recorded down the schedule (v_kp 123,
    /// 99 and 79 refused on rest error, 63 accepted), after a tune that
    /// accepts its first try and a final check that refuses it on rest
    /// error: the back-off settles on the fifth gain, 0.4096 of the
    /// starting gain, v_kp 63 on that run's starting gain.
    #[test]
    fn on_dsr_gates_verdicts_the_back_off_settles_at_the_fifth_gain() {
        let dsr: Vec<u64> = (0..5)
            .map(|n| (tune::gain(0.602504674776334, n) * 256.0).round() as u64)
            .collect();
        assert_eq!(dsr, [154, 123, 99, 79, 63], "dsr-gate's tries");

        let dir = scratch("finish-dsr");
        let g = given();
        let (root, _, mut c, _) = tuned(&dir, &g);
        let k0 = starting_gain(&root);
        let (f, checks) = finish_script(
            &mut c,
            &args(&g, &dir),
            &root,
            &[ON_REST, ON_REST, ON_REST, ON_REST, ACCEPTED],
        );
        let text = f.lines.join("\n");
        assert_eq!(checks, 5, "{text}");
        assert_eq!(f.last["state"], "accepted", "{}", f.last);
        assert_eq!(f.last["try"], 6);
        assert_eq!(f.last["checks"], json!([2, 3, 4, 5, 6]));
        let set = files::set(&root.join("final/set.json")).expect("set");
        assert_eq!(v_kp(&set), (tune::gain(k0, 4) * 256.0).round() as u64);
        assert!((tune::gain(k0, 4) / k0 - 0.4096).abs() < 1e-12);
        assert_eq!(
            f.lines
                .iter()
                .filter(|l| l.starts_with("verify refused on rest error at v_kp "))
                .count(),
            4,
            "{text}"
        );
        assert!(holds(&mut c, &set));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The back-off stops at the tune's last gain: refused on rest error
    /// there, no lower gain is tried, no final set, said why; the tune's
    /// set goes back on the servo and `final/` holds the first final check,
    /// as a refused one always has. A tune that accepted its last gain has
    /// no step left: the final check's refusal ends there.
    #[test]
    fn the_back_off_ends_at_the_tunes_last_gain() {
        let g = given();
        let dir = scratch("finish-floor");
        let (root, tune_set, mut c, mgmt_log) = tuned_at(&dir, &g, 2);
        let mut a = args(&g, &dir);
        a.tries = 3;
        let (f, checks) = finish_script(&mut c, &a, &root, &[ON_REST, ON_REST]);
        let text = f.lines.join("\n");
        println!("{text}");
        assert_eq!(checks, 2);
        assert_eq!(f.last["state"], "refused", "{}", f.last);
        assert_eq!(f.last["checks"], json!([3, 4]));
        let why = f.last["why"].as_str().expect("why");
        assert!(
            why.starts_with("the final set is not accepted (scripted); ")
                && why.contains("the last of the 3 gains the tune allows: no lower gain is tried"),
            "{why}"
        );
        assert!(text.contains(&format!("no final set: {why}")), "{text}");
        assert!(root.join("tune/tries/04/results/verify.json").exists());
        assert!(!root.join("tune/tries/05").exists());
        assert_eq!(held(&root), tree_of(&root.join("tune/tries/03")));
        let doc = json_at(&root.join("tune/back-off.json")).expect("back-off.json");
        assert_eq!(doc["tries"].as_array().map(Vec::len), Some(2), "{doc}");
        assert_eq!(doc["tries"][1]["try"], 4, "{doc}");
        assert_eq!(doc["tries"][1]["next"], "stop", "{doc}");
        assert_eq!(doc["accepted"], Value::Null, "{doc}");
        assert_eq!(doc["ended"], why, "{doc}");
        assert!(!text.contains("to store it"), "{text}");
        assert!(holds(&mut c, &tune_set), "the tune's set is live again");
        assert!(!mgmt_log.borrow().contains(&(MgmtOp::Save as u8)));
        let _ = std::fs::remove_dir_all(&dir);

        let dir = scratch("finish-floor-last");
        let (root, tune_set, mut c, _) = tuned_at(&dir, &g, 3);
        let mut a = args(&g, &dir);
        a.tries = 3;
        let (f, checks) = finish_script(&mut c, &a, &root, &[ON_REST]);
        assert_eq!(checks, 1);
        assert_eq!(f.last["checks"], json!([4]));
        assert!(
            f.last["why"]
                .as_str()
                .is_some_and(|w| w.contains("the last of the 3 gains the tune allows")),
            "{}",
            f.last
        );
        assert!(!root.join("tune/tries/05").exists());
        assert!(holds(&mut c, &tune_set));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Only a refusal on rest error alone backs off: a final check refused
    /// with a fault or another line beside it, or aborted, ends as it did,
    /// `final/` holding it; a back-off try refused on another line ends the
    /// back-off there.
    #[test]
    fn only_a_refusal_on_rest_error_alone_backs_off() {
        let g = given();
        use osc_measure::scorecard as sc;
        let ends: [(Way, &[&str]); 4] = [
            (Way::Refused, &[sc::REST, sc::NO_FAULT]),
            (Way::Refused, &[sc::OVERSHOOT]),
            (Way::Refused, &[sc::REST, sc::QUIET]),
            (Way::Error, REST),
        ];
        for (k, end) in ends.into_iter().enumerate() {
            let dir = scratch(&format!("finish-other-{k}"));
            let (root, tune_set, mut c, _) = tuned(&dir, &g);
            let (f, checks) = finish_script(&mut c, &args(&g, &dir), &root, &[end, ACCEPTED]);
            let text = f.lines.join("\n");
            assert_eq!(checks, 1, "{end:?}: {text}");
            assert_eq!(f.last["state"], "refused", "{end:?}");
            assert_eq!(f.last["checks"], json!([2]), "{}", f.last);
            assert!(
                f.last["why"]
                    .as_str()
                    .is_some_and(|w| w == "the final set is not accepted (scripted)"),
                "{}",
                f.last
            );
            assert!(!text.contains("backing off"), "{text}");
            assert!(!root.join("tune/tries/03").exists());
            assert_eq!(held(&root), tree_of(&root.join("tune/tries/02")));
            assert!(holds(&mut c, &tune_set));
            let _ = std::fs::remove_dir_all(dir);
        }

        let dir = scratch("finish-other-try");
        let (root, tune_set, mut c, _) = tuned(&dir, &g);
        let (f, checks) = finish_script(
            &mut c,
            &args(&g, &dir),
            &root,
            &[ON_REST, (Way::Refused, &[sc::OVERSHOOT]), ACCEPTED],
        );
        assert_eq!(checks, 2);
        assert_eq!(f.last["state"], "refused", "{}", f.last);
        assert_eq!(f.last["checks"], json!([2, 3]));
        assert!(
            f.last["why"].as_str().is_some_and(|w| w.contains(
                "the check of try 3 was refused, failing overshoot (scripted): only a refusal on rest error alone backs the gain off"
            )),
            "{}",
            f.last
        );
        let doc = json_at(&root.join("tune/back-off.json")).expect("back-off.json");
        assert_eq!(doc["tries"][1]["next"], "stop", "{doc}");
        assert_eq!(held(&root), tree_of(&root.join("tune/tries/02")));
        assert!(holds(&mut c, &tune_set));
        let _ = std::fs::remove_dir_all(dir);
    }
}
