//! `tune`: the speed gain a closed-loop check settles, from the rules'
//! starting gain down. A try at a time: the rules to a set at the try's
//! gain, the set installed volatile, the check. The first set
//! the check accepts is the result; the tune never saves. A tune that ends
//! without one reboots the servo onto its stored state.

use std::path::Path;
use std::sync::atomic::Ordering;

use anyhow::{Context, Result, bail};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use serde_json::{Value, json};

use super::files::{self, RulesInputs};
use super::store::At;
use super::{Host, Progress, TUNE};
use crate::build;
use crate::gather::POT_STEP4;
use crate::install::{self, Refused, STORED};
use crate::rules::{Fail, speed_loop};
use crate::set::{self, FromRules, SetFile, file_sha1};
use crate::sink::Sink;
use crate::steps::Rig;
use crate::steps::verify::{self, Verified, VerifyInputs};
use crate::tune::{self, Next};

/// The terms an accepted closed-loop check gives the final set: the still
/// speed, and the pot statistic over the whole population, the worst of
/// the gather's and the check's.
#[derive(Clone, Debug, Default)]
pub struct Checked {
    pub still: Option<Value>,
    pub step4: Option<Value>,
}

/// The set the rules give at the speed gain `kp`, the measured file it was
/// given with (the inputs' own, `speed_kp_verified` set to `kp`, and
/// `still_speed_cps` and `pot_step4_counts` to `checked`'s, an accepted
/// check's terms, where given), and a line for each field the rules stored
/// at a bound.
pub fn set_at(
    r: &RulesInputs,
    measured: &Value,
    kp: f64,
    checked: &Checked,
    pair: &str,
    d: &Descriptor,
) -> Result<(SetFile, Vec<u8>, Vec<String>)> {
    let mut inputs = r.inputs.clone();
    inputs.measured.speed_kp_verified = Some(kp);
    let mut doc = measured.clone();
    doc["speed_kp_verified"] = kp.into();
    if let Some(q) = &checked.still {
        inputs.measured.still_speed_cps =
            Some(serde_json::from_value(q.clone()).context("the check's still_speed_cps")?);
        doc["still_speed_cps"] = q.clone();
    }
    if let Some(q) = &checked.step4 {
        inputs.measured.pot_step4_counts =
            Some(serde_json::from_value(q.clone()).context("the pot_step4_counts")?);
        doc[POT_STEP4] = q.clone();
    }
    let mut bytes = serde_json::to_vec_pretty(&doc)?;
    bytes.push(b'\n');
    let files: Vec<(String, String, String)> = r
        .files
        .iter()
        .map(|(n, p, s)| {
            if n == "measured" {
                let still = if checked.still.is_some() {
                    ", still_speed_cps of the tune's check"
                } else {
                    ""
                };
                let step4 = if checked.step4.is_some() {
                    ", pot_step4_counts with the tune's check"
                } else {
                    ""
                };
                (
                    n.clone(),
                    format!("{p}, speed_kp_verified {kp}{still}{step4}"),
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
pub fn try_json(n: u32, kp: f64, k0: f64, file: &SetFile, rest: Value) -> Value {
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

/// A try's first line: its number, gain and set.
pub fn try_head(n: u32, kp: f64, k0: f64, file: &SetFile) -> String {
    format!(
        "try {}: kp {kp:.4} (v_kp_q88 {}, {:.0}% of the starting gain), set {}",
        n + 1,
        file.value("v_kp_q88")
            .map_or("none".into(), Value::to_string),
        100.0 * kp / k0,
        &file.hash()[..12]
    )
}

/// A try's check as the tune scores it.
pub struct Scored {
    pub verdict: String,
    pub failed: Vec<String>,
    pub why: Option<String>,
    pub worst: Value,
}

impl Scored {
    pub fn of(ran: &Result<Verified, Refused>) -> Self {
        match ran {
            Err(e) => Scored {
                verdict: "refused".into(),
                failed: vec![],
                why: Some(e.0.clone()),
                worst: Value::Null,
            },
            Ok(v) => Scored {
                verdict: v.ended.way.name().to_string(),
                failed: tune::failed(&v.checks),
                why: v.ended.why.clone(),
                worst: v.result.as_ref().map_or(Value::Null, tune::worst),
            },
        }
    }

    /// "the check refused on rest error; worst ...".
    pub fn said(&self) -> String {
        format!(
            "the check {}{}; {}",
            self.verdict,
            if self.failed.is_empty() {
                String::new()
            } else {
                format!(" on {}", self.failed.join(", "))
            },
            if self.worst.is_null() {
                "no move scored".into()
            } else {
                tune::worst_said(&self.worst)
            }
        )
    }
}

/// `tune` into the run folder `out`: the lines it prints, and whether a
/// set was accepted. Refused with nothing written, and no folder, unless
/// the servo is at rest with torque off, no fault, and settings this tool
/// may write.
pub async fn tune<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    inputs: &Path,
    out: &Path,
    tries: u32,
) -> Result<(Vec<String>, bool)> {
    let check = async |rig: &mut Rig<'_, P>, i: &VerifyInputs<'_>| verify::verify(rig, i).await;
    tune_with(c, id, d, host, inputs, out, tries, check).await
}

/// [`tune`] with the closed-loop check `check`: the simulated servo runs
/// no loop, so its tests script the check's verdicts.
#[allow(clippy::too_many_arguments)]
pub async fn tune_with<P: Pipe, F>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    inputs: &Path,
    out: &Path,
    tries: u32,
    mut check: F,
) -> Result<(Vec<String>, bool)>
where
    F: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    if tries == 0 {
        bail!("--tries is at least 1");
    }
    let r = files::rules_inputs(host.files, inputs, None)?;
    let measured: Value =
        serde_json::from_slice(&files::read(host.files, &inputs.join("measured.json"))?)
            .context("parsing measured.json")?;
    let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
    let holes = files::run_holes(host.files, &measured)
        .with_context(|| format!("the pot holes of {}", inputs.display()))?;
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
    let (first, _, clamps) = set_at(&r, &measured, k0, &Checked::default(), &pair, d)
        .with_context(|| format!("the rules on {} at the starting gain", inputs.display()))?;
    if host.files.exists(out) {
        bail!("{} exists: a tune runs into a new folder", out.display());
    }
    let (_, state) = verify::admit(c, id, d)
        .await
        .map_err(|e| anyhow::anyhow!("refused before any write: {e}"))?;
    install::writable_now(&state).map_err(|e| anyhow::anyhow!("refused before any write: {e}"))?;
    let points = r.table.as_ref().map(|t| &t.points);
    install::gate(&first, &state, d, points).map_err(|e| anyhow::anyhow!("{e}"))?;

    let root = out.to_path_buf();
    let cli = host.cli();
    let (stop, clock, tool, feed) = (host.stop, host.clock, host.tool, host.feed);
    let mut sink = At {
        files: &mut *host.files,
        root: root.clone(),
    };
    let q88 = |k: f64| (k * 256.0).round();
    let mut lines = vec![format!(
        "starting gain: {k0:.4} (v_kp_q88 {}), the lowest of the rules' starting gain over the ends of its inputs' bands, the nominal {nominal:.4} ({}); the rules hold a verified gain under the starting gain at every end of the bands",
        q88(k0),
        q88(nominal)
    )];
    lines.extend(clamps);
    lines.push(format!(
        "the check's pot steps leave out the samples inside a pot hole: {}",
        holes.source
    ));
    let mut steps = Vec::new();
    let stored = install::backup(c, id, d, &mut sink, &mut steps)
        .await
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    lines.extend(steps.iter().map(ToString::to_string));
    let started = clock();
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
        feed(&Progress::Try {
            step: TUNE,
            at: n + 1,
            of: tries,
        });
        let kp = tune::gain(k0, n);
        let dir = format!("tries/{:02}", n + 1);
        let prepared = set_at(&r, &measured, kp, &Checked::default(), &pair, d).and_then(
            |(file, bytes, _)| {
                sink.write(&format!("{dir}/measured.json"), &bytes)?;
                sink.write(&format!("{dir}/set.json"), file.render().as_bytes())?;
                Ok(file)
            },
        );
        let file = match prepared {
            Ok(f) => f,
            Err(e) => {
                ended = format!("try {}: {e:#}", n + 1);
                break;
            }
        };
        let head = try_head(n, kp, k0, &file);
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
        let ran = {
            let mut tsink = At {
                files: &mut *sink.files,
                root: root.join(&dir),
            };
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut tsink,
                stop,
                clock,
            };
            check(
                &mut rig,
                &VerifyInputs {
                    nameplate: np,
                    holes: &holes,
                },
            )
            .await
        };
        let s = Scored::of(&ran);
        let next = match &ran {
            Err(e) => Next::Stop(format!("the check refused before any write: {e}")),
            Ok(_) => tune::next(&s.verdict, &s.failed, s.why.as_deref()),
        };
        lines.push(format!("{head}: installed; {}", s.said()));
        lines.extend(isteps.iter().map(|s| format!("  {s}")));
        kept.push(try_json(
            n,
            kp,
            k0,
            &file,
            json!({"install": "installed", "verdict": s.verdict, "failed_on": s.failed,
                   "why": s.why, "worst": s.worst, "check": format!("{dir}/{}", verify::RESULT),
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
            if cli {
                let table = match (file.table().0, &r.table) {
                    ("LIVE", Some(_)) => {
                        format!(" --table {}", inputs.join("pos_lut.json").display())
                    }
                    _ => String::new(),
                };
                lines.push(format!(
                    "to store it: osc-ident --id {} save --set {}{table}",
                    id.as_byte(),
                    shown("set.json")
                ));
            }
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
        "tool": tool,
        "inputs": inputs.display().to_string(),
        "started": started,
        "ended_at": clock(),
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
