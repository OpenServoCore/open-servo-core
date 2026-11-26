//! `run`'s finish, on the set the tune accepted: the cruise step for the
//! friction line the velocity loop needs, into the run folder; the gather
//! again, now with that line, into `final/inputs/`; the rules at the
//! tune's gain into `final/set.json`, installed volatile; the closed-loop
//! check once more into `final/`. Short of an accepted check, the tune's
//! set goes back on the servo and stays; nothing is ever saved.

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut::INTERVALS;
use osc_client::{Client, Id};
use osc_measure::install::{self, Refused};
use osc_measure::nameplate::Nameplate;
use osc_measure::set::SetFile;
use osc_measure::sink::Sink;
use osc_measure::steps::cruise::{self, CruiseInputs, Cruised};
use osc_measure::steps::verify::{self, Verified, VerifyInputs};
use osc_measure::steps::{Rig, stops};
use serde_json::{Value, json};

use crate::files::{self, DirSink};
use crate::run::RunArgs;

fn unix_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// What the finish did, for the lines and the manifest.
pub struct Finished {
    pub lines: Vec<String>,
    pub cruise: Value,
    pub last: Value,
}

fn json_at(p: &Path) -> Result<Value> {
    serde_json::from_slice(&files::read(p)?).with_context(|| format!("parsing {}", p.display()))
}

/// `set` live on the servo, volatile, installed again where the way out
/// of a step rebooted it away; the lines say what holds.
async fn keep_live<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    what: &str,
    lines: &mut Vec<String>,
) -> bool {
    let mut steps = Vec::new();
    match install::install(c, id, d, set, points, &mut steps).await {
        Ok(()) => {
            lines.push(format!(
                "live on the servo, volatile: {what} {}{}",
                &set.hash()[..12],
                if steps.is_empty() {
                    ""
                } else {
                    ", installed again"
                }
            ));
            true
        }
        Err(e) => {
            lines.push(format!(
                "{what} {} is not live ({e}): a reboot or a power cycle returns the stored set",
                &set.hash()[..12]
            ));
            false
        }
    }
}

/// The finish of the run folder `root` whose tune accepted a set.
pub async fn finish<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    root: &Path,
    stop: &AtomicBool,
) -> Result<Finished> {
    let run_cruise =
        async |rig: &mut Rig<'_, P>, i: &CruiseInputs<'_>| cruise::cruise(rig, i).await;
    let check = async |rig: &mut Rig<'_, P>, np: &Nameplate| {
        verify::verify(rig, &VerifyInputs { nameplate: np }).await
    };
    finish_with(c, id, d, a, root, stop, run_cruise, check).await
}

/// A finish that ends short of a final set: the tune's set made sure of on
/// the servo, the lines and the `final` state saying why.
#[allow(clippy::too_many_arguments)]
async fn short_of_final<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    keep: (&SetFile, Option<&[i16; INTERVALS]>),
    mut lines: Vec<String>,
    cruise: Value,
    state: &str,
    why: String,
) -> Finished {
    lines.push(format!("no final set: {why}"));
    keep_live(c, id, d, keep.0, keep.1, "the tune's set", &mut lines).await;
    Finished {
        lines,
        cruise,
        last: json!({"state": state, "why": why}),
    }
}

/// The final set from the gather with the cruise line, at the tune's gain:
/// the set, the measured file it was given with, and its table.
fn final_set(
    inputs_dir: &Path,
    kp: f64,
    d: &Descriptor,
) -> Result<(SetFile, Vec<u8>, Option<[i16; INTERVALS]>)> {
    let r = files::rules_inputs(inputs_dir, None, None)?;
    let measured = json_at(&inputs_dir.join("measured.json"))?;
    let pair = format!("{}__{}", r.inputs.nameplate.name, r.inputs.board.name);
    let (file, bytes) = crate::tune::set_at(&r, &measured, kp, &pair, d)
        .with_context(|| format!("the rules on {} at kp {kp}", inputs_dir.display()))?;
    Ok((file, bytes, r.table.as_ref().map(|t| t.points)))
}

/// [`finish`] with the cruise step and the closed-loop check given: the
/// simulated servo runs no loop, so its tests script both. Past the
/// reading of the run folder every refusal is a finish short of a final
/// set, recorded, never an error.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn finish_with<P: Pipe, C, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    a: &RunArgs<'_>,
    root: &Path,
    stop: &AtomicBool,
    mut run_cruise: C,
    mut check: V,
) -> Result<Finished>
where
    C: AsyncFnMut(&mut Rig<'_, P>, &CruiseInputs<'_>) -> Result<Cruised, Refused>,
    V: AsyncFnMut(&mut Rig<'_, P>, &Nameplate) -> Result<Verified, Refused>,
{
    let tune_dir = root.join("tune");
    let tuned = json_at(&tune_dir.join("tune.json"))?;
    let kp = tuned["accepted"]["kp"]
        .as_f64()
        .context("tune.json names no accepted gain")?;
    let tune_set = files::set(&tune_dir.join("set.json"))?;
    let first = files::rules_inputs(&root.join("inputs"), None, None)?;
    let tune_points = first.table.as_ref().map(|t| t.points);
    let np = files::nameplate(a.nameplate)?;
    let board = files::board(None)?;
    let stops_result = json_at(&root.join(stops::RESULT))?;
    let keep = (&tune_set, tune_points.as_ref());
    let clock = unix_s;

    // the tune's check may have ended through a way out that rebooted onto
    // the stored set: the cruise runs on the tune's set or not at all
    let mut lines = Vec::new();
    if !keep_live(
        c,
        id,
        d,
        &tune_set,
        tune_points.as_ref(),
        "the tune's set",
        &mut lines,
    )
    .await
    {
        lines.push("the cruise is not run: the servo does not hold the tune's set".into());
        let why = "the tune's set could not be made live for the cruise";
        return Ok(Finished {
            lines,
            cruise: json!({"state": "not run", "why": why}),
            last: json!({"state": "not run", "why": why}),
        });
    }
    lines.push(format!(
        "cruise on the tune's set {}, at kp {kp:.4}:",
        &tune_set.hash()[..12]
    ));
    let t0 = unix_s();
    let mut sink = DirSink {
        root: root.to_path_buf(),
    };
    let ran = {
        let mut rig = Rig {
            c: &mut *c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        run_cruise(
            &mut rig,
            &CruiseInputs {
                nameplate: &np,
                board: &board,
                stops: Some(&stops_result),
            },
        )
        .await
    };
    let t1 = unix_s();
    let (state, why) = match &ran {
        Err(e) => ("refused".to_string(), Some(e.0.clone())),
        Ok(r) => (r.ended.way.name().to_string(), r.ended.why.clone()),
    };
    match &ran {
        Err(e) => lines.push(format!("  step cruise refused before any write: {e}")),
        Ok(r) => {
            let mut said = Vec::new();
            crate::steps::said(cruise::STEP, &r.ended, &r.checks, &mut said);
            lines.extend(said.into_iter().map(|l| format!("  {l}")));
        }
    }
    let cruise_entry = json!({"state": state, "why": why, "started": t0, "ended": t1,
                              "files": [cruise::RESULT]});
    if !matches!(&ran, Ok(r) if r.accepted()) {
        lines.push(format!(
            "the cruise step was {state}: the friction line stays the spin step's"
        ));
        let why = format!("the cruise step was {state}");
        return Ok(short_of_final(c, id, d, keep, lines, cruise_entry, "not run", why).await);
    }

    let final_dir = root.join("final");
    let inputs_dir = final_dir.join("inputs");
    match crate::gather::gather(root, a.nameplate, Some(a.thermal), a.policy, &inputs_dir) {
        Ok(g) => {
            lines.push(format!("gather into {}:", inputs_dir.display()));
            lines.extend(g.into_iter().map(|l| format!("  {l}")));
        }
        Err(e) => {
            let why = format!("the gather with the cruise line was refused: {e:#}");
            return Ok(short_of_final(c, id, d, keep, lines, cruise_entry, "refused", why).await);
        }
    }
    let mut fsink = DirSink {
        root: final_dir.clone(),
    };
    let prepared = final_set(&inputs_dir, kp, d).and_then(|(file, bytes, points)| {
        fsink.write("measured.json", &bytes)?;
        fsink.write("set.json", file.render().as_bytes())?;
        Ok((file, points))
    });
    let (file, points) = match prepared {
        Ok(p) => p,
        Err(e) => {
            let why = format!("the final set was not built: {e:#}");
            return Ok(short_of_final(c, id, d, keep, lines, cruise_entry, "refused", why).await);
        }
    };
    let word = |f: &SetFile, n: &str| f.value(n).map_or("none".into(), |v| v.to_string());
    lines.push(format!(
        "final set {}: fric_fc_counts {} and fric_fv_q016 {} (the tune's {} and {}), kp {kp:.4}",
        &file.hash()[..12],
        word(&file, "fric_fc_counts"),
        word(&file, "fric_fv_q016"),
        word(&tune_set, "fric_fc_counts"),
        word(&tune_set, "fric_fv_q016")
    ));
    let mut steps = Vec::new();
    if let Err(e) = install::install(c, id, d, &file, points.as_ref(), &mut steps).await {
        let why = format!("the final set's install was refused: {e}");
        return Ok(short_of_final(c, id, d, keep, lines, cruise_entry, "refused", why).await);
    }
    lines.extend(steps.iter().map(|s| format!("  {s}")));
    let t2 = unix_s();
    let checked = {
        let mut rig = Rig {
            c: &mut *c,
            id,
            d,
            sink: &mut fsink,
            stop,
            clock: &clock,
        };
        check(&mut rig, &np).await
    };
    let t3 = unix_s();
    let (vstate, vwhy) = match &checked {
        Err(e) => ("refused".to_string(), Some(e.0.clone())),
        Ok(v) => (v.ended.way.name().to_string(), v.ended.why.clone()),
    };
    match &checked {
        Err(e) => lines.push(format!("  the final check refused before any write: {e}")),
        Ok(v) => {
            let mut said = Vec::new();
            crate::steps::said(verify::STEP, &v.ended, &v.checks, &mut said);
            lines.extend(said.into_iter().map(|l| format!("  {l}")));
        }
    }
    lines.push(format!(
        "the tune's check: accepted; the final check: {vstate}"
    ));
    let files = json!([
        "final/inputs/measured.json",
        "final/measured.json",
        "final/set.json",
        format!("final/{}", verify::RESULT)
    ]);
    let accepted = matches!(&checked, Ok(v) if v.accepted());
    let live = accepted
        && keep_live(
            c,
            id,
            d,
            &file,
            points.as_ref(),
            "the final set",
            &mut lines,
        )
        .await;
    if live {
        let table = match (file.table().0, &points) {
            ("LIVE", Some(_)) => format!(" --table {}", inputs_dir.join("pos_lut.json").display()),
            _ => String::new(),
        };
        lines.push(format!(
            "to store it: osc-ident --id {} save --set {}{table}",
            id.as_byte(),
            final_dir.join("set.json").display()
        ));
        lines.push("a reboot returns the stored set".into());
        return Ok(Finished {
            lines,
            cruise: cruise_entry,
            last: json!({"state": "accepted", "why": vwhy, "started": t2, "ended": t3,
                         "set": file.hash(), "files": files}),
        });
    }
    let why = if accepted {
        "the final check accepted the set, but it is not live on the servo".to_string()
    } else {
        format!(
            "the final set is not accepted ({})",
            vwhy.as_deref().unwrap_or(&vstate)
        )
    };
    let mut f = short_of_final(c, id, d, keep, lines, cruise_entry, "refused", why).await;
    f.last = json!({"state": "refused", "why": f.last["why"], "check": vstate, "started": t2,
                    "ended": t3, "set": file.hash(), "files": files});
    Ok(f)
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
        thermal: PathBuf,
        policy: PathBuf,
    }

    fn given() -> Given {
        Given {
            nameplate: testdata("mg90-d/nameplate.json"),
            thermal: testdata("mg90-d/thermal.json"),
            policy: testdata("mg90-d/policy.json"),
        }
    }

    fn args<'a>(g: &'a Given, out: &'a Path) -> RunArgs<'a> {
        RunArgs {
            nameplate: &g.nameplate,
            thermal: &g.thermal,
            policy: &g.policy,
            out,
            tries: 6,
        }
    }

    /// A run folder as the tune leaves it: the bench servo's results,
    /// gathered into `inputs/`, the tune's set at `KP` in `tune/`; and the
    /// servo of record holding that set, volatile over the stored record.
    fn tuned(dir: &Path, g: &Given) -> (PathBuf, SetFile, Client<Spy>, Rc<RefCell<Vec<u8>>>) {
        let root = dir.join("uid/run");
        std::fs::create_dir_all(root.join("results")).expect("mkdir");
        for f in ["rest", "probe", "stops", "window", "spins"] {
            std::fs::copy(
                testdata(&format!("gather/{f}.json")),
                root.join(format!("results/{f}.json")),
            )
            .expect("copy");
        }
        let inputs = root.join("inputs");
        crate::gather::gather(&root, &g.nameplate, Some(&g.thermal), &g.policy, &inputs)
            .expect("gathers");
        let d = crate::descriptor_builtin().expect("descriptor");
        let r = files::rules_inputs(&inputs, None, None).expect("inputs");
        let measured = json_at(&inputs.join("measured.json")).expect("measured");
        let pair = format!("{}__{}", r.inputs.nameplate.name, r.inputs.board.name);
        let (set, _) = crate::tune::set_at(&r, &measured, KP, &pair, &d).expect("the tune's set");
        std::fs::create_dir_all(root.join("tune")).expect("mkdir");
        std::fs::write(root.join("tune/set.json"), set.render()).expect("set");
        std::fs::write(
            root.join("tune/tune.json"),
            json!({"accepted": {"try": 1, "kp": KP, "speed_kp_verified": KP, "set": set.hash()}})
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

    /// The tune's accepted gain: under the gathered starting gain, 0.5896.
    const KP: f64 = 0.5;

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
        rig.sink
            .write(
                verify::RESULT,
                json!({"verdict": way.name()}).to_string().as_bytes(),
            )
            .map_err(|e| Refused(e.0))?;
        Ok(Verified {
            ended: ended(way),
            checks: vec![Check {
                name: "scripted".into(),
                value: "scripted".into(),
                bound: "scripted".into(),
                verdict: if way == Way::Done {
                    Verdict::Pass
                } else {
                    Verdict::Fail
                },
            }],
            result: None,
        })
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
            async |rig: &mut Rig<'_, Spy>, _: &Nameplate| {
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
    /// the command that stores it printed, nothing saved.
    #[test]
    fn an_accepted_cruise_installs_and_checks_the_final_set() {
        let dir = scratch("finish-accepted");
        let g = given();
        let (root, tune_set, mut c, mgmt_log) = tuned(&dir, &g);
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
            json_at(&root.join("final/measured.json")).expect("m")["speed_kp_verified"],
            KP
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
            "to store it: osc-ident --id 1 save --set {}",
            root.join("final/set.json").display()
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

    /// The final set refused by its check: the tune's set goes back on the
    /// servo, whole; nothing saved.
    #[test]
    fn a_final_set_refused_puts_the_tunes_set_back() {
        let dir = scratch("finish-refused");
        let g = given();
        let (root, tune_set, mut c, mgmt_log) = tuned(&dir, &g);
        let (f, _) = finish_scripted(&mut c, &args(&g, &dir), &root, Way::Done, Way::Refused);
        let text = f.lines.join("\n");
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
            async |rig: &mut Rig<'_, Spy>, _: &Nameplate| {
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
}
