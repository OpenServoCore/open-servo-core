//! `run`'s finish, on the set the tune accepted: the cruise step for the
//! friction line the velocity loop needs, into the run folder; the gather
//! again, now with that line, into `final/inputs/`; the rules at the
//! tune's gain, installed volatile; the closed-loop check once more. Each
//! final check is a try after the tune's, kept in `tune/tries/`; one
//! refused on rest error alone goes on down the tune's schedule on the
//! cruise's friction line, to the tune's last gain. `final/` is written
//! once, at the end, with the accepted try's files (the first final
//! check's where none was accepted). Short of an accepted check, the
//! tune's set goes back on the servo and stays; nothing is ever saved.

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
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
use osc_measure::tune;
use serde_json::{Value, json};

use crate::files::{self, DirSink};
use crate::run::RunArgs;
use crate::tune::{Scored, try_head, try_json};

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
    let r = files::rules_inputs(inputs_dir, None)?;
    let measured = json_at(&inputs_dir.join("measured.json"))?;
    let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
    let (file, bytes, _) = crate::tune::set_at(&r, &measured, kp, &pair, d)
        .with_context(|| format!("the rules on {} at kp {kp}", inputs_dir.display()))?;
    Ok((file, bytes, r.table.as_ref().map(|t| t.points)))
}

fn word(f: &SetFile, n: &str) -> String {
    f.value(n).map_or("none".into(), |v| v.to_string())
}

/// The command that stores `set`, kept at `path`, with its table where the
/// set carries a live one.
fn save_line(
    id: Id,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    inputs_dir: &Path,
    path: &Path,
) -> String {
    let table = match (set.table().0, points) {
        ("LIVE", Some(_)) => format!(" --table {}", inputs_dir.join("pos_lut.json").display()),
        _ => String::new(),
    };
    format!(
        "to store it: osc-ident --id {} save --set {}{table}",
        id.as_byte(),
        path.display()
    )
}

/// The final checks' record beside tune.json, in its shape.
pub const BACK_OFF: &str = "back-off.json";

/// Every file under `dir`, by its path below it, in order.
fn tree(dir: &Path, below: &str, out: &mut Vec<String>) -> std::io::Result<()> {
    let mut names: Vec<_> = std::fs::read_dir(dir)?
        .map(|e| e.map(|e| e.file_name()))
        .collect::<std::io::Result<_>>()?;
    names.sort();
    for n in names {
        let rel = format!("{below}{}", n.to_string_lossy());
        if dir.join(&n).is_dir() {
            tree(&dir.join(&n), &format!("{rel}/"), out)?;
        } else {
            out.push(rel);
        }
    }
    Ok(())
}

/// The try folder `from` written file by file into `to`.
fn copy_into(from: &Path, to: &mut DirSink) -> Result<(), String> {
    let mut files = Vec::new();
    tree(from, "", &mut files).map_err(|e| format!("{}: {e}", from.display()))?;
    for f in files {
        let bytes = std::fs::read(from.join(&f)).map_err(|e| format!("{f}: {e}"))?;
        to.write(&f, &bytes).map_err(|e| e.0)?;
    }
    Ok(())
}

/// The first final check, at the tune's gain, as it ended.
struct First {
    file: SetFile,
    state: String,
    why: Option<String>,
    started: f64,
    ended: f64,
}

/// How the final checks ended: the try folders written, the first check,
/// the try accepted (its folder number, set and table), and why the
/// checks ended short of an accepted one.
struct Checks {
    tries: Vec<u32>,
    first: Option<First>,
    accepted: Option<(u32, SetFile, Option<[i16; INTERVALS]>)>,
    why: String,
}

/// The final checks, each a try after the tune's accepted try `from`, its
/// set from the gather with the cruise line in `inputs_dir`, kept in
/// `tune/tries/NN` as the tune keeps its own and listed in
/// `tune/back-off.json` (the run never writes over tune.json). The first
/// is at the tune's gain `kp`; one refused on rest error alone goes on to
/// the tune's next gain, down to the last gain the tune allows.
#[allow(clippy::too_many_arguments)]
async fn final_checks<P: Pipe, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    root: &Path,
    stop: &AtomicBool,
    allowed: u32,
    check: &mut V,
    np: &Nameplate,
    (from, kp, k0): (u32, f64, f64),
    inputs_dir: &Path,
    tune_set: &SetFile,
    lines: &mut Vec<String>,
) -> Checks
where
    V: AsyncFnMut(&mut Rig<'_, P>, &Nameplate) -> Result<Verified, Refused>,
{
    let mut out = Checks {
        tries: vec![],
        first: None,
        accepted: None,
        why: String::new(),
    };
    let tune_dir = root.join("tune");
    let mut sink = DirSink {
        root: tune_dir.clone(),
    };
    let clock = unix_s;
    let started = unix_s();
    let mut kept = Vec::new();
    let mut prev = String::new();
    let (mut step, mut t, mut kp_now) = (from.saturating_sub(1), from + 1, kp);
    out.why = loop {
        let first = out.tries.is_empty();
        if !first && stop.load(Ordering::SeqCst) {
            break "interrupted between tries".into();
        }
        let dir = format!("tries/{t:02}");
        let prepared = final_set(inputs_dir, kp_now, d).and_then(|(file, bytes, points)| {
            sink.write(&format!("{dir}/measured.json"), &bytes)?;
            sink.write(&format!("{dir}/set.json"), file.render().as_bytes())?;
            Ok((file, points))
        });
        let (file, points) = match prepared {
            Ok(p) => p,
            Err(e) if first => break format!("the final set was not built: {e:#}"),
            Err(e) => break format!("try {t}: {e:#}"),
        };
        out.tries.push(t);
        let now = word(&file, "v_kp_q88");
        let head = format!(
            "{}, on the cruise's friction line",
            try_head(t - 1, kp_now, k0, &file)
        );
        let after = if first {
            format!("the tune's check accepted its try {from}")
        } else {
            format!("the check at v_kp {prev} refused on rest error")
        };
        if first {
            lines.push(format!(
                "final set {}: fric_fc_counts {} and fric_fv_q016 {} (the tune's {} and {}), kp {kp:.4}",
                &file.hash()[..12],
                word(&file, "fric_fc_counts"),
                word(&file, "fric_fv_q016"),
                word(tune_set, "fric_fc_counts"),
                word(tune_set, "fric_fv_q016")
            ));
        } else {
            lines.push(format!(
                "verify refused on rest error at v_kp {prev}; backing off to {now}"
            ));
        }
        let mut isteps = Vec::new();
        if let Err(e) = install::install(c, id, d, &file, points.as_ref(), &mut isteps).await {
            kept.push(try_json(
                t - 1,
                kp_now,
                k0,
                &file,
                json!({"after": after, "install": format!("refused: {e}"), "next": "stop"}),
            ));
            if first {
                break format!("the final set's install was refused: {e}");
            }
            lines.push(format!("{head}: the install refused: {e}"));
            lines.extend(install::moves(&isteps));
            break format!("the install of try {t} refused: {e}");
        }
        if first {
            lines.extend(isteps.iter().map(|s| format!("  {s}")));
        }
        let mut tsink = DirSink {
            root: tune_dir.join(&dir),
        };
        let t2 = unix_s();
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
        let t3 = unix_s();
        let s = Scored::of(&ran);
        let accepted = matches!(&ran, Ok(v) if v.accepted());
        let lower = tune::backs_off(&s.verdict, &s.failed);
        if !first {
            lines.push(format!("{head}: installed; {}", s.said()));
            lines.extend(isteps.iter().map(|s| format!("  {s}")));
        }
        match &ran {
            Err(e) => lines.push(format!("  the final check refused before any write: {e}")),
            Ok(v) => {
                let mut said = Vec::new();
                crate::steps::said(verify::STEP, &v.ended, &v.checks, &mut said);
                lines.extend(said.into_iter().map(|l| format!("  {l}")));
            }
        }
        if first {
            lines.push(format!(
                "the tune's check: accepted; the final check: {}",
                s.verdict
            ));
            out.first = Some(First {
                file: file.clone(),
                state: s.verdict.clone(),
                why: s.why.clone(),
                started: t2,
                ended: t3,
            });
        }
        let more = lower && step + 1 < allowed;
        kept.push(try_json(
            t - 1,
            kp_now,
            k0,
            &file,
            json!({"after": after, "install": "installed", "verdict": s.verdict,
                   "failed_on": s.failed, "why": s.why, "worst": s.worst,
                   "check": format!("{dir}/{}", verify::RESULT),
                   "next": if accepted { "accept" } else if more { "lower" } else { "stop" }}),
        ));
        if accepted {
            out.accepted = Some((t, file, points));
            break String::new();
        }
        let short = |w: String| match &out.first {
            Some(f) if !first => format!(
                "the final set is not accepted ({}); {w}",
                f.why.as_deref().unwrap_or(&f.state)
            ),
            _ => w,
        };
        if !lower {
            break short(match &ran {
                _ if first => format!(
                    "the final set is not accepted ({})",
                    s.why.as_deref().unwrap_or(&s.verdict)
                ),
                Err(e) => format!("the check of try {t} refused before any write: {e}"),
                Ok(_) => format!(
                    "the check of try {t} was {}{}{}: only a refusal on rest error alone backs the gain off",
                    s.verdict,
                    if s.failed.is_empty() {
                        String::new()
                    } else {
                        format!(", failing {}", s.failed.join(", "))
                    },
                    s.why
                        .as_deref()
                        .map_or(String::new(), |w| format!(" ({w})"))
                ),
            });
        }
        if !more {
            let w = format!(
                "the check refused on rest error at v_kp {now}, the last of the {allowed} gains the tune allows: no lower gain is tried"
            );
            break match &out.first {
                Some(f) => format!(
                    "the final set is not accepted ({}); {w}",
                    f.why.as_deref().unwrap_or(&f.state)
                ),
                None => w,
            };
        }
        step += 1;
        kp_now = tune::gain(k0, step);
        prev = now;
        t += 1;
    };
    if out.tries.is_empty() {
        return out;
    }
    let doc = json!({
        "tool": format!("osc-ident {}", env!("CARGO_PKG_VERSION")),
        "inputs": inputs_dir.display().to_string(),
        "started": started,
        "ended_at": unix_s(),
        "starting_gain": k0,
        "share": tune::SHARE,
        "tries_allowed": allowed,
        "after": {"tune": "tune.json", "accepted_try": from},
        "tries": kept,
        "accepted": out.accepted.as_ref().map(|(t, f, _)| {
            json!({"try": t, "kp": kp_now, "speed_kp_verified": kp_now, "set": f.hash()})
        }),
        "ended": if out.accepted.is_some() { Value::Null } else { out.why.clone().into() },
    });
    let mut bytes = serde_json::to_vec_pretty(&doc).unwrap_or_default();
    bytes.push(b'\n');
    if let Err(e) = sink.write(BACK_OFF, &bytes) {
        lines.push(format!("{BACK_OFF} not kept: {e}"));
    }
    out
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
    let from = tuned["accepted"]["try"]
        .as_u64()
        .context("tune.json names no accepted try")? as u32;
    let k0 = tuned["starting_gain"]
        .as_f64()
        .context("tune.json names no starting gain")?;
    let tune_set = files::set(&tune_dir.join("set.json"))?;
    let first = files::rules_inputs(&root.join("inputs"), None)?;
    let tune_points = first.table.as_ref().map(|t| t.points);
    let np = files::nameplate(a.nameplate, Some(a.policy))?;
    let board = first.inputs.board.clone();
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
    let mut checks = final_checks(
        c,
        id,
        d,
        root,
        stop,
        a.tries,
        &mut check,
        &np,
        (from, kp, k0),
        &inputs_dir,
        &tune_set,
        &mut lines,
    )
    .await;
    // final/ holds one try's files, written once: the accepted one, or the
    // first final check where none was accepted
    let held = checks
        .accepted
        .as_ref()
        .map(|(t, _, _)| *t)
        .or(checks.tries.first().copied());
    let mut copied = false;
    if let Some(t) = held {
        let mut fsink = DirSink {
            root: final_dir.clone(),
        };
        match copy_into(&root.join(format!("tune/tries/{t:02}")), &mut fsink) {
            Ok(()) => copied = true,
            Err(e) => lines.push(format!("final/ not written from try {t}: {e}")),
        }
    }
    let files = json!([
        "final/inputs/measured.json",
        "final/measured.json",
        "final/set.json",
        format!("final/{}", verify::RESULT)
    ]);
    let back_off = format!("tune/{BACK_OFF}");
    let backed = checks.tries.len() > 1;
    if let Some((t, set, pts)) = checks.accepted.take() {
        if backed {
            lines.push(format!(
                "backed off to try {t}: the final check: accepted at v_kp {}",
                word(&set, "v_kp_q88")
            ));
        }
        if keep_live(c, id, d, &set, pts.as_ref(), "the final set", &mut lines).await {
            let set_file = if copied {
                "final/set.json".to_string()
            } else {
                format!("tune/tries/{t:02}/set.json")
            };
            lines.push(save_line(
                id,
                &set,
                pts.as_ref(),
                &inputs_dir,
                &root.join(&set_file),
            ));
            lines.push("a reboot returns the stored set".into());
            let (started, why) = checks
                .first
                .as_ref()
                .map_or((Value::Null, Value::Null), |f| {
                    (f.started.into(), f.why.clone().into())
                });
            return Ok(Finished {
                lines,
                cruise: cruise_entry,
                last: json!({"state": "accepted", "why": if backed { Value::Null } else { why },
                             "started": started, "ended": unix_s(), "set": set.hash(),
                             "files": files, "set_file": set_file, "try": t,
                             "checks": checks.tries, "back_off": back_off}),
            });
        }
        checks.why = if backed {
            format!("the final check accepted the set of try {t}, but it is not live on the servo")
        } else {
            "the final check accepted the set, but it is not live on the servo".into()
        };
    }
    let mut f = short_of_final(c, id, d, keep, lines, cruise_entry, "refused", checks.why).await;
    if let Some(first) = &checks.first {
        f.last = json!({"state": "refused", "why": f.last["why"], "check": first.state,
                        "started": first.started, "ended": if backed { unix_s() } else { first.ended },
                        "set": first.file.hash(), "files": files, "checks": checks.tries,
                        "back_off": back_off});
    }
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
        let r = files::rules_inputs(&inputs, None).expect("inputs");
        let measured = json_at(&inputs.join("measured.json")).expect("measured");
        let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
        let k0 = osc_measure::rules::speed_loop::starting_gain(&r.inputs).expect("k0");
        let kp = tune::gain(k0, accepted - 1);
        let (set, _, _) =
            crate::tune::set_at(&r, &measured, kp, &pair, &d).expect("the tune's set");
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
            async |rig: &mut Rig<'_, Spy>, _: &Nameplate| {
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
    /// the command that stores it printed, nothing saved. Accepted the
    /// first time, nothing is backed off: no try is added to the tune's.
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
        let (set, measured, _) = final_set(&root.join("final/inputs"), kp, &d).expect("set");
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
            "to store it: osc-ident --id 1 save --set {}",
            root.join("final/set.json").display()
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
