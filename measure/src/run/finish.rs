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
use std::sync::atomic::Ordering;

use anyhow::{Context, Result};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::pos_lut::INTERVALS;
use osc_client::{Client, Id};
use serde_json::{Value, json};

use super::files;
use super::store::{At, Store};
use super::tune::{Checked, Scored, try_head, try_json};
use super::{FINAL, Host, Progress, RunArgs, tell_end, tell_lines};
use crate::gather::{self, POT_STEP4, StepResult};
use crate::install::{self, Refused};
use crate::nameplate::Nameplate;
use crate::set::SetFile;
use crate::sink::Sink;
use crate::steps::cruise::{self, CruiseInputs, Cruised};
use crate::steps::verify::{self, Verified, VerifyInputs};
use crate::steps::{Rig, stops};
use crate::tune;

/// What the finish did, for the lines and the manifest.
pub struct Finished {
    pub lines: Vec<String>,
    pub cruise: Value,
    pub last: Value,
}

pub fn json_at(fs: &mut dyn Store, p: &Path) -> Result<Value> {
    serde_json::from_slice(&files::read(fs, p)?).with_context(|| format!("parsing {}", p.display()))
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
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    root: &Path,
) -> Result<Finished> {
    let run_cruise =
        async |rig: &mut Rig<'_, P>, i: &CruiseInputs<'_>| cruise::cruise(rig, i).await;
    let check = async |rig: &mut Rig<'_, P>, i: &VerifyInputs<'_>| verify::verify(rig, i).await;
    finish_with(c, id, d, host, a, root, run_cruise, check).await
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

/// The terms of the tune's accepted check, try `from`, the final set takes:
/// its still speed, and the pot statistic, the worst of the gather's
/// `gathered` and the check's; each said, the check's none where its
/// result holds none or cannot be read.
fn tune_checked(
    fs: &mut dyn Store,
    tune_dir: &Path,
    from: u32,
    gathered: Option<&Value>,
    lines: &mut Vec<String>,
) -> Checked {
    let p = tune_dir.join(format!("tries/{from:02}/{}", verify::RESULT));
    let r = files::read(fs, &p)
        .map_err(|e| format!("{e:#}"))
        .and_then(|bytes| StepResult::parse(&p.display().to_string(), &bytes).map_err(|e| e.0));
    let still = tune_still(r.as_ref(), from, lines);
    let check = match r
        .as_ref()
        .map_err(Clone::clone)
        .and_then(|r| gather::check_pot_step4(r).map_err(|e| e.0))
    {
        Ok(q) => q,
        Err(e) => {
            lines.push(format!("{POT_STEP4} of the tune's check: not read, {e}"));
            None
        }
    };
    let value = |q: Option<&Value>| {
        q.and_then(|q| q["value"].as_f64())
            .map_or("none".to_string(), |v| format!("{v:.0}"))
    };
    let step4 = gather::worst_pot_step4(gathered, check.as_ref());
    lines.push(format!(
        "{POT_STEP4} {} raw counts: the worst of the spins' and the table's, {}, and the tune's check's, try {from}, its wide-code holds too, {}",
        value(step4.as_ref()),
        value(gathered),
        value(check.as_ref())
    ));
    Checked { still, step4 }
}

/// The still speed of the tune's accepted check `r`, try `from`, as a
/// measured term; none, said, where its result holds none or cannot be
/// read.
fn tune_still(
    r: Result<&StepResult, &String>,
    from: u32,
    lines: &mut Vec<String>,
) -> Option<Value> {
    let read = r
        .map_err(Clone::clone)
        .and_then(|r| gather::still_speed(r).map_err(|e| e.0));
    match read {
        Ok(Some(q)) => {
            lines.push(format!(
                "still_speed_cps {:.1} count/s: the largest observer speed over the still holds of the tune's check, try {from}",
                q["value"].as_f64().unwrap_or(f64::NAN)
            ));
            Some(q)
        }
        Ok(None) => {
            lines.push(format!(
                "still_speed_cps: the tune's check, try {from}, holds no still holds"
            ));
            None
        }
        Err(e) => {
            lines.push(format!("still_speed_cps: not read, {e}"));
            None
        }
    }
}

/// The final set from the gather with the cruise line, at the tune's gain
/// and with the terms of the tune's accepted check: the set, the measured
/// file it was given with, and its table.
pub fn final_set(
    fs: &mut dyn Store,
    inputs_dir: &Path,
    kp: f64,
    checked: &Checked,
    d: &Descriptor,
) -> Result<(SetFile, Vec<u8>, Option<[i16; INTERVALS]>)> {
    let r = files::rules_inputs(fs, inputs_dir, None)?;
    let measured = json_at(fs, &inputs_dir.join("measured.json"))?;
    let pair = format!("{}__{}", r.inputs.nameplate.name, r.servo);
    let (file, bytes, _) = super::tune::set_at(&r, &measured, kp, checked, &pair, d)
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
pub fn tree(fs: &dyn Store, dir: &Path, below: &str, out: &mut Vec<String>) -> Result<(), String> {
    let mut names = fs.entries(dir)?;
    names.sort();
    for (n, is_dir) in names {
        let rel = format!("{below}{n}");
        if is_dir {
            tree(fs, &dir.join(&n), &format!("{rel}/"), out)?;
        } else {
            out.push(rel);
        }
    }
    Ok(())
}

/// The try folder `from` written file by file into `to`.
fn copy_into(from: &Path, to: &mut At) -> Result<(), String> {
    let mut files = Vec::new();
    tree(to.files, from, "", &mut files).map_err(|e| format!("{}: {e}", from.display()))?;
    for f in files {
        let bytes = to
            .files
            .read_file(&from.join(&f))
            .map_err(|e| format!("{f}: {e}"))?;
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
    host: &mut Host<'_>,
    root: &Path,
    allowed: u32,
    check: &mut V,
    np: &Nameplate,
    (from, kp, k0): (u32, f64, f64),
    inputs_dir: &Path,
    tune_set: &SetFile,
    lines: &mut Vec<String>,
) -> Checks
where
    V: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    let mut out = Checks {
        tries: vec![],
        first: None,
        accepted: None,
        why: String::new(),
    };
    let tune_dir = root.join("tune");
    let gathered = json_at(host.files, &inputs_dir.join("measured.json"));
    let checked = tune_checked(
        host.files,
        &tune_dir,
        from,
        gathered.as_ref().ok().and_then(|m| m.get(POT_STEP4)),
        lines,
    );
    let holes = match gathered.and_then(|m| files::run_holes(host.files, &m)) {
        Ok(h) => h,
        Err(e) => {
            out.why = format!("the final check's pot holes were not read: {e:#}");
            return out;
        }
    };
    let (stop, clock, tool, feed) = (host.stop, host.clock, host.tool, host.feed);
    let mut sink = At {
        files: &mut *host.files,
        root: tune_dir.clone(),
    };
    let started = clock();
    let mut kept = Vec::new();
    let mut prev = String::new();
    let (mut step, mut t, mut kp_now) = (from.saturating_sub(1), from + 1, kp);
    out.why = loop {
        let first = out.tries.is_empty();
        if !first && stop.load(Ordering::SeqCst) {
            break "interrupted between tries".into();
        }
        feed(&Progress::Try {
            step: FINAL,
            at: out.tries.len() as u32 + 1,
            of: allowed.saturating_sub(from.saturating_sub(1)),
        });
        let dir = format!("tries/{t:02}");
        let prepared = final_set(&mut *sink.files, inputs_dir, kp_now, &checked, d).and_then(
            |(file, bytes, points)| {
                sink.write(&format!("{dir}/measured.json"), &bytes)?;
                sink.write(&format!("{dir}/set.json"), file.render().as_bytes())?;
                Ok((file, points))
            },
        );
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
        let t2 = clock();
        let ran = {
            let mut tsink = At {
                files: &mut *sink.files,
                root: tune_dir.join(&dir),
            };
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut tsink,
                stop,
                clock,
                tell: &super::teller(feed, FINAL),
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
        let t3 = clock();
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
                super::steps::said(verify::STEP, &v.ended, &v.checks, &mut said);
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
        "tool": tool,
        "inputs": inputs_dir.display().to_string(),
        "started": started,
        "ended_at": clock(),
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
pub async fn finish_with<P: Pipe, C, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    root: &Path,
    run_cruise: C,
    check: V,
) -> Result<Finished>
where
    C: AsyncFnMut(&mut Rig<'_, P>, &CruiseInputs<'_>) -> Result<Cruised, Refused>,
    V: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    let (feed, mut told) = (host.feed, 0);
    let f = finish_told(c, id, d, host, a, root, run_cruise, check, &mut told).await?;
    tell_lines(feed, &f.lines, &mut told);
    Ok(f)
}

/// [`finish_with`], the lines through the cruise's end told and counted
/// in `told`.
#[allow(clippy::too_many_arguments)]
async fn finish_told<P: Pipe, C, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    root: &Path,
    mut run_cruise: C,
    mut check: V,
    told: &mut usize,
) -> Result<Finished>
where
    C: AsyncFnMut(&mut Rig<'_, P>, &CruiseInputs<'_>) -> Result<Cruised, Refused>,
    V: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    let tune_dir = root.join("tune");
    let tuned = json_at(host.files, &tune_dir.join("tune.json"))?;
    let kp = tuned["accepted"]["kp"]
        .as_f64()
        .context("tune.json names no accepted gain")?;
    let from = tuned["accepted"]["try"]
        .as_u64()
        .context("tune.json names no accepted try")? as u32;
    let k0 = tuned["starting_gain"]
        .as_f64()
        .context("tune.json names no starting gain")?;
    let tune_set = files::set(host.files, &tune_dir.join("set.json"))?;
    let first = files::rules_inputs(host.files, &root.join("inputs"), None)?;
    let tune_points = first.table.as_ref().map(|t| t.points);
    let np = files::nameplate(host.files, a.nameplate, Some(a.policy))?;
    let board = first.inputs.board.clone();
    let stops_result = json_at(host.files, &root.join(stops::RESULT))?;
    let keep = (&tune_set, tune_points.as_ref());
    let (stop, clock, feed) = (host.stop, host.clock, host.feed);

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
        let cruise = json!({"state": "not run", "why": why});
        tell_lines(feed, &lines, told);
        tell_end(feed, cruise::STEP, &cruise);
        return Ok(Finished {
            lines,
            cruise,
            last: json!({"state": "not run", "why": why}),
        });
    }
    super::put(host.files, root, cruise::STEP, super::running())?;
    feed(&Progress::Started { step: cruise::STEP });
    lines.push(format!(
        "cruise on the tune's set {}, at kp {kp:.4}:",
        &tune_set.hash()[..12]
    ));
    let t0 = clock();
    let ran = {
        let mut sink = At {
            files: &mut *host.files,
            root: root.to_path_buf(),
        };
        let mut rig = Rig {
            c: &mut *c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock,
            tell: &super::teller(feed, cruise::STEP),
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
    let t1 = clock();
    let (state, why) = match &ran {
        Err(e) => ("refused".to_string(), Some(e.0.clone())),
        Ok(r) => (r.ended.way.name().to_string(), r.ended.why.clone()),
    };
    match &ran {
        Err(e) => lines.push(format!("  step cruise refused before any write: {e}")),
        Ok(r) => {
            let mut said = Vec::new();
            super::steps::said(cruise::STEP, &r.ended, &r.checks, &mut said);
            lines.extend(said.into_iter().map(|l| format!("  {l}")));
        }
    }
    let cruise_entry = json!({"state": state, "why": why, "started": t0, "ended": t1,
                              "files": [cruise::RESULT]});
    if let Err(e) = super::put(host.files, root, cruise::STEP, cruise_entry.clone()) {
        lines.push(format!("run.json not kept: {e:#}"));
    }
    tell_lines(feed, &lines, told);
    tell_end(feed, cruise::STEP, &cruise_entry);
    if !matches!(&ran, Ok(r) if r.accepted()) {
        lines.push(format!(
            "the cruise step was {state}: the friction line stays the spin step's"
        ));
        let why = format!("the cruise step was {state}");
        return Ok(short_of_final(c, id, d, keep, lines, cruise_entry, "not run", why).await);
    }

    if let Err(e) = super::put(host.files, root, FINAL, super::running()) {
        lines.push(format!("run.json not kept: {e:#}"));
    }
    feed(&Progress::Started { step: FINAL });
    let final_dir = root.join("final");
    let inputs_dir = final_dir.join("inputs");
    match super::gather::gather(host.files, root, a.nameplate, a.policy, &inputs_dir) {
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
        host,
        root,
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
        let mut fsink = At {
            files: &mut *host.files,
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
            if host.cli() {
                lines.push(save_line(
                    id,
                    &set,
                    pts.as_ref(),
                    &inputs_dir,
                    &root.join(&set_file),
                ));
            }
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
                             "started": started, "ended": clock(), "set": set.hash(),
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
                        "started": first.started, "ended": if backed { clock() } else { first.ended },
                        "set": first.file.hash(), "files": files, "checks": checks.tries,
                        "back_off": back_off});
    }
    Ok(f)
}
