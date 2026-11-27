//! `tune`: the speed gain a closed-loop check settles, from the rules'
//! starting gain down. A try at a time: the rules to a set at the try's
//! gain, the set installed volatile, the check. The first set
//! the check accepts is the result; the tune never saves. A tune that ends
//! without one reboots the servo onto its stored state.

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::build;
use osc_measure::install::{self, Refused, STORED};
use osc_measure::nameplate::Nameplate;
use osc_measure::rules::{Fail, speed_loop};
use osc_measure::set::{self, FromRules, SetFile, file_sha1};
use osc_measure::sink::Sink;
use osc_measure::steps::Rig;
use osc_measure::steps::verify::{self, Verified, VerifyInputs};
use osc_measure::tune::{self, Next};
use serde_json::{Value, json};

use crate::files::{self, DirSink, RulesInputs};

fn unix_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

/// The set the rules give at the speed gain `kp`, the measured file it was
/// given with (the inputs' own, `speed_kp_verified` set to `kp`), and a
/// line for each field the rules stored at a bound.
pub(crate) fn set_at(
    r: &RulesInputs,
    measured: &Value,
    kp: f64,
    pair: &str,
    d: &Descriptor,
) -> Result<(SetFile, Vec<u8>, Vec<String>)> {
    let mut inputs = r.inputs.clone();
    inputs.measured.speed_kp_verified = Some(kp);
    let mut doc = measured.clone();
    doc["speed_kp_verified"] = kp.into();
    let mut bytes = serde_json::to_vec_pretty(&doc)?;
    bytes.push(b'\n');
    let files: Vec<(String, String, String)> = r
        .files
        .iter()
        .map(|(n, p, s)| {
            if n == "measured" {
                (
                    n.clone(),
                    format!("{p}, speed_kp_verified {kp}"),
                    file_sha1(&bytes),
                )
            } else {
                (n.clone(), p.clone(), s.clone())
            }
        })
        .collect();
    let built = build(&inputs);
    let (file, _) = set::from_rules(
        &built,
        &FromRules {
            pair,
            nameplate: &inputs.nameplate,
            counts_per_deg: inputs.measured.table.as_ref().map(|t| t.counts_per_deg),
            table: r.table.as_ref(),
            slots: r.slots.as_deref(),
            files: &files,
        },
        d,
    )?;
    Ok((file, bytes, built.clamps()))
}

/// One try as tune.json keeps it.
fn try_json(n: u32, kp: f64, k0: f64, file: &SetFile, rest: Value) -> Value {
    let mut t = json!({
        "try": n + 1,
        "kp": kp,
        "share_of_start": kp / k0,
        "v_kp_q88": file.value("v_kp_q88"),
        "set": file.hash(),
        "set_file": format!("tries/{:02}/set.json", n + 1),
    });
    if let (Some(t), Value::Object(r)) = (t.as_object_mut(), rest) {
        t.extend(r);
    }
    t
}

/// `tune` into the run folder `out`: the lines it prints, and whether a
/// set was accepted. Refused with nothing written, and no folder, unless
/// the servo is at rest with torque off, no fault, and settings this tool
/// may write.
pub async fn tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    inputs: &Path,
    out: &Path,
    tries: u32,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool)> {
    let check = async |rig: &mut Rig<'_, P>, np: &Nameplate| {
        verify::verify(rig, &VerifyInputs { nameplate: np }).await
    };
    tune_with(c, id, d, inputs, out, tries, stop, check).await
}

/// [`tune`] with the closed-loop check `check`: the simulated servo runs
/// no loop, so its tests script the check's verdicts.
#[allow(clippy::too_many_arguments)]
async fn tune_with<P: Pipe, F>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    inputs: &Path,
    out: &Path,
    tries: u32,
    stop: &AtomicBool,
    mut check: F,
) -> Result<(Vec<String>, bool)>
where
    F: AsyncFnMut(&mut Rig<'_, P>, &Nameplate) -> Result<Verified, Refused>,
{
    if tries == 0 {
        bail!("--tries is at least 1");
    }
    let r = files::rules_inputs(inputs, None, None)?;
    let measured: Value = serde_json::from_slice(&files::read(&inputs.join("measured.json"))?)
        .context("parsing measured.json")?;
    let pair = format!("{}__{}", r.inputs.nameplate.name, r.inputs.board.name);
    let no_gain = |e: Fail| {
        let why = match e {
            Fail::Missing(m) => format!("it reads {}, which is not there", m.input),
            Fail::Refused(r) => r.why,
        };
        anyhow::anyhow!(
            "the rules give no starting speed gain on {}: {why}",
            inputs.display()
        )
    };
    let k0 = speed_loop::starting_gain(&r.inputs).map_err(no_gain)?;
    let nominal = speed_loop::nominal_starting_gain(&r.inputs).map_err(no_gain)?;
    let (first, _, clamps) = set_at(&r, &measured, k0, &pair, d)
        .with_context(|| format!("the rules on {} at the starting gain", inputs.display()))?;
    if out.exists() {
        bail!("{} exists: a tune runs into a new folder", out.display());
    }
    let (_, state) = verify::admit(c, id, d)
        .await
        .map_err(|e| anyhow::anyhow!("refused before any write: {e}"))?;
    install::writable_now(&state).map_err(|e| anyhow::anyhow!("refused before any write: {e}"))?;
    let points = r.table.as_ref().map(|t| &t.points);
    install::gate(&first, &state, d, points).map_err(|e| anyhow::anyhow!("{e}"))?;

    let root = out.to_path_buf();
    let mut sink = DirSink { root: root.clone() };
    let q88 = |k: f64| (k * 256.0).round();
    let mut lines = vec![format!(
        "starting gain: {k0:.4} (v_kp_q88 {}), the lowest of the rules' starting gain over the ends of its inputs' bands, the nominal {nominal:.4} ({}); the rules hold a verified gain under the starting gain at every end of the bands",
        q88(k0),
        q88(nominal)
    )];
    lines.extend(clamps);
    let mut steps = Vec::new();
    let stored = install::backup(c, id, d, &mut sink, &mut steps)
        .await
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    lines.extend(steps.iter().map(ToString::to_string));
    let started = unix_s();
    let np = &r.inputs.nameplate;
    let mut kept = Vec::new();
    let mut accepted: Option<(u32, f64, SetFile)> = None;
    let mut last: Option<(u32, String)> = None;
    let mut ended = format!("no try of {tries} was accepted, each refused on its scores");
    for n in 0..tries {
        if stop.load(Ordering::SeqCst) {
            ended = "interrupted between tries".into();
            break;
        }
        let kp = tune::gain(k0, n);
        let dir = format!("tries/{:02}", n + 1);
        let prepared = set_at(&r, &measured, kp, &pair, d).and_then(|(file, bytes, _)| {
            sink.write(&format!("{dir}/measured.json"), &bytes)?;
            sink.write(&format!("{dir}/set.json"), file.render().as_bytes())?;
            Ok(file)
        });
        let file = match prepared {
            Ok(f) => f,
            Err(e) => {
                ended = format!("try {}: {e:#}", n + 1);
                break;
            }
        };
        let head = format!(
            "try {}: kp {kp:.4} (v_kp_q88 {}, {:.0}% of the starting gain), set {}",
            n + 1,
            file.value("v_kp_q88")
                .map_or("none".into(), Value::to_string),
            100.0 * kp / k0,
            &file.hash()[..12]
        );
        let mut isteps = Vec::new();
        if let Err(e) = install::install(c, id, d, &file, points, &mut isteps).await {
            ended = format!("the install refused: {e}");
            lines.push(format!("{head}: the install refused; {ended}"));
            kept.push(try_json(
                n,
                kp,
                k0,
                &file,
                json!({"install": format!("refused: {e}"), "next": "stop"}),
            ));
            lines.extend(install::moves(&isteps));
            if !isteps.is_empty() {
                last = Some((n, file.hash().to_string()));
            }
            break;
        }
        last = Some((n, file.hash().to_string()));
        let mut tsink = DirSink {
            root: root.join(&dir),
        };
        let clock = unix_s;
        let ran = {
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut tsink,
                stop,
                clock: &clock,
            };
            check(&mut rig, np).await
        };
        let (verdict, failed, why, worst) = match &ran {
            Err(e) => (
                "refused".to_string(),
                vec![],
                Some(e.0.clone()),
                json!(null),
            ),
            Ok(v) => (
                v.ended.way.name().to_string(),
                tune::failed(&v.checks),
                v.ended.why.clone(),
                v.result.as_ref().map_or(Value::Null, tune::worst),
            ),
        };
        let next = match &ran {
            Err(e) => Next::Stop(format!("the check refused before any write: {e}")),
            Ok(_) => tune::next(&verdict, &failed, why.as_deref()),
        };
        lines.push(format!(
            "{head}: installed; the check {verdict}{}; {}",
            if failed.is_empty() {
                String::new()
            } else {
                format!(" on {}", failed.join(", "))
            },
            if worst.is_null() {
                "no move scored".into()
            } else {
                tune::worst_said(&worst)
            }
        ));
        lines.extend(isteps.iter().map(|s| format!("  {s}")));
        kept.push(try_json(
            n,
            kp,
            k0,
            &file,
            json!({"install": "installed", "verdict": verdict, "failed_on": failed,
                   "why": why, "worst": worst, "check": format!("{dir}/{}", verify::RESULT),
                   "next": match &next { Next::Accept => "accept", Next::Lower => "lower", Next::Stop(_) => "stop" }}),
        ));
        match next {
            Next::Accept => {
                accepted = Some((n, kp, file));
                break;
            }
            Next::Lower => {}
            Next::Stop(w) => {
                ended = w;
                break;
            }
        }
    }

    let shown = |rel: &str| root.join(rel).display().to_string();
    let mut unkept = Vec::new();
    let mut put_back = Value::Null;
    let ok = accepted.is_some();
    match &accepted {
        Some((n, kp, file)) => {
            if let Err(e) = sink.write("set.json", file.render().as_bytes()) {
                unkept.push(format!(
                    "set.json not kept ({e}): tries/{:02}/set.json is the same set",
                    n + 1
                ));
            }
            lines.push(format!(
                "accepted on try {}: kp {kp:.4}, {:.0}% of the starting gain {k0:.4}",
                n + 1,
                100.0 * kp / k0
            ));
            let table = match (file.table().0, &r.table) {
                ("LIVE", Some(_)) => format!(" --table {}", inputs.join("pos_lut.json").display()),
                _ => String::new(),
            };
            lines.push(format!(
                "to store it: osc-ident --id {} save --set {}{table}",
                id.as_byte(),
                shown("set.json")
            ));
            lines.push(format!(
                "live on the servo, volatile: the accepted set {}; a reboot returns the stored one",
                &file.hash()[..12]
            ));
        }
        None => {
            lines.push(format!("the tune is refused: {ended}"));
            let mut bsteps = Vec::new();
            let back = install::put_back(c, id, d, &stored, &mut bsteps).await;
            lines.extend(install::moves(&bsteps));
            let live = match &back {
                Ok(left) if left.is_empty() => {
                    lines.push(format!(
                        "put back: rebooted onto the stored configuration, which reads as the backup {}",
                        shown(STORED)
                    ));
                    put_back = json!({"rebooted": true, "differences": []});
                    "live on the servo: its stored configuration; no tried set is installed"
                        .to_string()
                }
                Ok(left) => {
                    lines.push(format!(
                        "put back: rebooted, but the state read is not the backup {}: {}",
                        shown(STORED),
                        left.join("; ")
                    ));
                    put_back = json!({"rebooted": true, "differences": left});
                    "live on the servo: the configuration it boots, which is not the backup; no tried set is installed".to_string()
                }
                Err(e) => {
                    lines.push(format!("put back: the reboot failed: {e}"));
                    put_back = json!({"rebooted": false, "why": e.0});
                    match &last {
                        Some((n, hash)) => format!(
                            "live on the servo: the set of try {} ({}), volatile, or a half-written one; a reboot or a power cycle returns the stored one",
                            n + 1,
                            &hash[..12]
                        ),
                        None => "live on the servo: its stored configuration, no try was installed"
                            .into(),
                    }
                }
            };
            lines.push(live);
        }
    }
    let doc = json!({
        "tool": format!("osc-ident {}", env!("CARGO_PKG_VERSION")),
        "inputs": inputs.display().to_string(),
        "started": started,
        "ended_at": unix_s(),
        "starting_gain": k0,
        "starting_gain_nominal": nominal,
        "share": tune::SHARE,
        "tries_allowed": tries,
        "backup": STORED,
        "tries": kept,
        "accepted": accepted.as_ref().map(|(n, kp, f)| json!({"try": n + 1, "kp": kp, "speed_kp_verified": kp, "set": f.hash()})),
        "ended": if ok { Value::Null } else { ended.into() },
        "put_back": put_back,
    });
    let mut bytes = serde_json::to_vec_pretty(&doc).unwrap_or_default();
    bytes.push(b'\n');
    if let Err(e) = sink.write("tune.json", &bytes) {
        unkept.push(format!("tune.json not kept: {e}"));
    }
    unkept.push(format!("run folder: {}", root.display()));
    let at = lines.len().saturating_sub(1);
    lines.splice(at..at, unkept);
    Ok((lines, ok))
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
    use osc_measure::steps::{Check, Ended, Verdict};

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
    /// volatile, as `osc baud 3000000` does after a FACTORY.
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
        let check = async |rig: &mut Rig<'_, Spy>, _: &Nameplate| {
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
            Some(&testdata("mg90-d/thermal.json")),
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
