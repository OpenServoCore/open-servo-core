//! `run --resume`: a stopped run continued in its own folder from its
//! first step not accepted (plan sec 5.4). Every refusal that needs no
//! servo comes first and leaves the folder untouched; then the servo is
//! checked against what the run began on, through a reboot onto its stored
//! settings, before anything drives. The segment is recorded in `run.json`
//! before it moves or writes anything else; the stopped attempt's files
//! move, bytes untouched, under `segments/<k>/` at the same path, and the
//! run goes on from that step as a fresh run would, each step installing
//! what it needs from the folder's files. Only the tune and the final check
//! go on between tries, by their own rules.

use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use serde_json::{Value, json};

use super::steps::{self as chain, StepInputs};
use super::store::{At, Store};
use super::{
    FINAL, GATHER, Host, ORDER, Past, Progress, RunArgs, TUNE, at, end_segment, files, finish,
    manifest_of, rewrite, tune,
};
use crate::board;
use crate::envelope::RISE_MAX_C;
use crate::install::{self, Refused};
use crate::set::file_sha1;
use crate::state;
use crate::steps::cruise::{self, CruiseInputs, Cruised};
use crate::steps::rest::{self, Sense};
use crate::steps::stops::{self, StopsInputs};
use crate::steps::verify::{Verified, VerifyInputs};
use crate::steps::{Rig, probe, seek, spins, table, window};
use crate::tune as schedule;

/// The progress key a resume's own work is told under.
pub const STEP: &str = "resume";
/// The rail may read this share off the rest step's (plan sec 5.4).
pub const RAIL_SHARE: f64 = 0.05;
/// A folder whose first segment began longer ago has its high stop
/// re-seated before a restart past the stops (plan sec 5.4), s.
pub const RESEAT_AFTER_S: f64 = 600.0;
/// The re-seat's bound against the run's high stop, counts: the
/// run-to-run bound of plan sec 5.6, the stops step's own walk bound.
pub const RESEAT_COUNTS: f64 = seek::WALK_MAX_COUNTS;
/// The winding is taken as back at the board when the most any step's
/// envelope admits over the board has decayed to this, C: 0.4 % of R.
pub const COOLED_C: f64 = 1.0;
/// The cooling wait is told this often, s.
pub const COOL_TELL_S: f64 = 5.0;

/// The wait before a probe or stops step a resume runs again `since_s`
/// after the last drive ended, s: the most any step's envelope admits,
/// decaying on the heat model's slow time constant, down to
/// [`COOLED_C`] (plan sec 3.1, the cold steps first).
pub fn cooling_s(slow_tau_s: f64, since_s: f64) -> f64 {
    (slow_tau_s * (RISE_MAX_C / COOLED_C).ln() - since_s.max(0.0)).max(0.0)
}

/// The re-seat's check: the high stop seated at `seat` against the run's
/// `run` (its seats' median), within [`RESEAT_COUNTS`].
pub fn reseat_judge(seat: f64, run: f64) -> (bool, Value) {
    let apart = (seat - run).abs();
    let pass = apart <= RESEAT_COUNTS;
    (
        pass,
        json!({"stop": "high", "seat_counts": seat, "run_counts": run, "apart_counts": apart,
               "bound_counts": RESEAT_COUNTS, "verdict": if pass { "pass" } else { "fail" }}),
    )
}

/// What a resume of a run folder does, decided from its files alone.
#[derive(Clone, Debug)]
pub struct Plan {
    /// The step it restarts at, by its key in `run.json`.
    pub from: &'static str,
    /// The new segment's number.
    pub n: u32,
    /// The tune's tries, the run's.
    pub tries: u32,
    /// The stopped attempt's files, each to its place under `segments/`.
    pub moves: Vec<(String, String)>,
    pub tune: tune::Onward,
    pub last: Option<finish::Onward>,
    /// Whether the high stop is re-seated first.
    pub reseat: bool,
    /// The wait before the restart for the winding to cool, s.
    pub cool_s: f64,
    /// Why it restarts where it does, a line each.
    pub why: Vec<String>,
    pub doc: Value,
}

/// The method and revision this tool writes for a step with a result.
fn method_of(step: &str) -> Option<(&'static str, u32)> {
    Some(match step {
        rest::STEP => (rest::METHOD, rest::REVISION),
        probe::STEP => (probe::METHOD, probe::REVISION),
        stops::STEP => (stops::METHOD, stops::REVISION),
        table::STEP => (table::METHOD, table::REVISION),
        window::STEP => (window::METHOD, window::REVISION),
        spins::STEP => (spins::METHOD, spins::REVISION),
        cruise::STEP => (cruise::METHOD, cruise::REVISION),
        _ => return None,
    })
}

fn entry<'a>(doc: &'a Value, step: &str) -> &'a Value {
    if [GATHER, TUNE, cruise::STEP, FINAL].contains(&step) {
        &doc[step]
    } else {
        &doc["steps"][step]
    }
}

fn result_rel(step: &str) -> String {
    format!("results/{step}.json")
}

/// The files a step's attempt leaves in the run folder, by their paths in
/// it: the tune's try folders stay, as tries are never renumbered.
fn files_of(fs: &dyn Store, root: &Path, step: &str) -> Vec<String> {
    let mut out = match step {
        GATHER => fs
            .entries(&root.join("inputs"))
            .unwrap_or_default()
            .into_iter()
            .filter(|(n, _)| format!("inputs/{n}") != files::BOARD)
            .map(|(n, _)| format!("inputs/{n}"))
            .collect(),
        TUNE => vec![
            "tune/tune.json".into(),
            "tune/set.json".into(),
            "tune/backup".into(),
        ],
        FINAL => vec!["final".into(), format!("tune/{}", finish::BACK_OFF)],
        _ => vec![format!("captures/{step}"), result_rel(step)],
    };
    out.retain(|f| fs.exists(&root.join(f)));
    out.sort();
    out
}

fn json_at(fs: &mut dyn Store, p: &Path) -> Result<Value> {
    serde_json::from_slice(&files::read(fs, p)?).with_context(|| format!("parsing {}", p.display()))
}

/// The gain of the try folder `n` of `tune_dir`, from the measured file it
/// was given with.
fn try_kp(fs: &mut dyn Store, tune_dir: &Path, n: u32) -> Option<f64> {
    let p = tune_dir.join(format!("tries/{n:02}/measured.json"));
    json_at(fs, &p).ok()?["speed_kp_verified"].as_f64()
}

/// The tries of a tune.json or back-off.json, each under the segment it
/// ran in.
fn tries_of(doc: &Value, seg: &Value) -> Vec<Value> {
    doc["tries"]
        .as_array()
        .cloned()
        .unwrap_or_default()
        .into_iter()
        .map(|mut t| {
            if t.get("segment").is_none() {
                t["segment"] = seg.clone();
            }
            t
        })
        .collect()
}

/// Whether the check of a try was judged on its scores: refused failing a
/// check, or accepted.
fn judged(t: &Value) -> bool {
    match t["verdict"].as_str() {
        Some("accepted") => true,
        Some("refused") => t["failed_on"].as_array().is_some_and(|f| !f.is_empty()),
        _ => false,
    }
}

/// The resume of the run folder `root` with the profile and the policy of
/// `a`, at `now`: every refusal that needs no servo, the folder untouched.
pub fn plan(
    fs: &mut dyn Store,
    root: &Path,
    a: &RunArgs,
    i: &StepInputs,
    now: f64,
) -> Result<Plan> {
    let p = root.join("run.json");
    if !fs.exists(&p) {
        bail!("{}: no run.json: not a run folder", root.display());
    }
    let doc: Value = serde_json::from_slice(&files::read(fs, &p)?)
        .map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?;
    let given = &doc["given"];
    if !given.is_object() {
        bail!(
            "{} records no profile and policy: the run was written before a resume could check them; start a new run",
            p.display()
        );
    }
    if doc[FINAL]["state"] == "accepted" {
        bail!(
            "the run is complete: every step was accepted through the final check; nothing to resume"
        );
    }
    for (key, path, what) in [
        ("nameplate", a.nameplate, "the profile"),
        ("policy", a.policy, "the policy"),
    ] {
        let now = file_sha1(&files::read(fs, path)?);
        let then = given[key]["sha1"].as_str().unwrap_or_default();
        if now != then {
            bail!(
                "{} is not {what} the run began with (sha1 {now}, the run's {then})",
                path.display()
            );
        }
    }
    let heat = i.thermal.as_ref().and_then(|t| t.slow_tau_s.as_ref());
    let (heat_file, heat_now) = heat.map_or((String::new(), None), |q| {
        (q.source.file.clone(), q.source.sha1.clone())
    });
    let heat_then = given["heat"]["sha1"].as_str().map(str::to_string);
    if heat_now != heat_then {
        bail!(
            "{heat_file} is not the heat model the run began with (sha1 {}, the run's {})",
            heat_now.unwrap_or_else(|| "none".into()),
            heat_then.unwrap_or_else(|| "none".into())
        );
    }
    let slow_tau_s = heat.map_or(0.0, |q| q.value);
    let tries = given["tries"].as_u64().unwrap_or(schedule::TRIES as u64) as u32;

    let mut why = Vec::new();
    let mut from = None;
    for step in ORDER {
        let e = entry(&doc, step);
        let state = e["state"].as_str().unwrap_or("not begun");
        if step == GATHER && state == "refused" {
            bail!(
                "the gather refused on these files ({}): it is pure, so it would refuse again; start a new run",
                e["why"].as_str().unwrap_or("no why recorded")
            );
        }
        if state != "accepted" {
            why.push(format!("{step}: {state}, so the run goes on from it"));
            from = Some(step);
            break;
        }
        let named = e["files"].as_array().into_iter().flatten();
        for rel in named.filter_map(Value::as_str) {
            if !fs.exists(&root.join(rel)) {
                bail!("{rel}, which run.json records accepted, is not in the folder");
            }
        }
        let Some((method, revision)) = method_of(step) else {
            continue;
        };
        let rel = result_rel(step);
        if !fs.exists(&root.join(&rel)) {
            bail!("{rel}, which run.json records accepted, is not in the folder");
        }
        let r = json_at(fs, &root.join(&rel))?;
        let (m, v) = (r["method"].as_str(), r["revision"].as_u64());
        if (m, v) != (Some(method), Some(revision as u64)) {
            why.push(format!(
                "{rel} is {} revision {}, this tool's {method} revision {revision}: the step runs again",
                m.unwrap_or("no method"),
                v.map_or("none".into(), |v| v.to_string())
            ));
            from = Some(step);
            break;
        }
    }
    let Some(from) = from else {
        bail!(
            "the run is complete: every step was accepted through the final check; nothing to resume"
        );
    };
    let segs = doc["segments"].as_array().cloned().unwrap_or_default();
    let n = segs.len() as u32 + 1;
    let tune_dir = root.join("tune");
    let next = finish::next_free(fs, &tune_dir);

    let mut tune_on = tune::Onward {
        segment: Some(n),
        resumed: fs.exists(&tune_dir),
        next,
        ..tune::Onward::default()
    };
    if from == TUNE {
        let seg = doc[TUNE]["segment"].clone();
        let tj = tune_dir.join("tune.json");
        if fs.exists(&tj) {
            let t = json_at(fs, &tj)?;
            tune_on.kept = tries_of(&t, &seg);
            if let Some(last) = tune_on.kept.last() {
                let kp = last["kp"].as_f64();
                let lower = last["next"] == "lower";
                let k0 = t["starting_gain"].as_f64();
                if let (Some(kp), Some(k0), true) = (kp, k0, lower)
                    && tune::index_of(kp, k0) + 1 >= tries
                {
                    bail!(
                        "the tune refused every gain it allows on the moves' scores (tries 1 to {}): nothing is left to try; start a new run",
                        next - 1
                    );
                }
                tune_on.from_kp = kp;
                tune_on.lower = lower;
            }
        } else if next > 1 {
            // cut off inside a try: that try's gain again
            tune_on.from_kp = try_kp(fs, &tune_dir, next - 1);
        }
    }

    let mut last = None;
    if from == FINAL {
        let tuned = json_at(fs, &tune_dir.join("tune.json"))?;
        let k0 = tuned["starting_gain"]
            .as_f64()
            .context("tune.json names no starting gain")?;
        let tune_kp = tuned["accepted"]["kp"]
            .as_f64()
            .context("tune.json names no accepted gain")?;
        let back = tune_dir.join(finish::BACK_OFF);
        let seg = doc[FINAL]["segment"].clone();
        let (kept, index, after) = if fs.exists(&back) {
            let b = json_at(fs, &back)?;
            let kept = tries_of(&b, &seg);
            let t = kept.last().cloned().unwrap_or(Value::Null);
            let kp = t["kp"].as_f64().unwrap_or(tune_kp);
            let q88 = t["v_kp_q88"].clone();
            let n_try = t["try"].clone();
            let (index, after) = if t["next"] == "lower" {
                (
                    tune::index_of(kp, k0) + 1,
                    format!(
                        "the check at v_kp {q88} refused on rest error, then the run stopped between tries"
                    ),
                )
            } else if judged(&t) {
                let on = t["failed_on"]
                    .as_array()
                    .map(|f| {
                        f.iter()
                            .filter_map(Value::as_str)
                            .collect::<Vec<_>>()
                            .join(", ")
                    })
                    .filter(|f| !f.is_empty())
                    .unwrap_or_else(|| "none (accepted)".into());
                bail!(
                    "the final check refused the set on {on} at v_kp {q88}: a resume does not check a judged set again; the tune's set stays: osc-ident save --set {}, or start a new run",
                    tune_dir.join("set.json").display()
                );
            } else {
                (
                    tune::index_of(kp, k0),
                    format!(
                        "the check of try {n_try} was {}: its gain again",
                        t["verdict"].as_str().unwrap_or("not scored")
                    ),
                )
            };
            (kept, index, after)
        } else {
            let from_try = tuned["accepted"]["try"].clone();
            (
                Vec::new(),
                tune::index_of(tune_kp, k0),
                format!("the tune's check accepted its try {from_try}"),
            )
        };
        if index >= tries {
            bail!(
                "the final check refused on rest error at the last of the {tries} gains the tune allows: a resume does not check a judged set again; the tune's set stays: osc-ident save --set {}, or start a new run",
                tune_dir.join("set.json").display()
            );
        }
        last = Some(finish::Onward {
            next,
            index,
            kept,
            after,
        });
    }

    let mut moves = Vec::new();
    for step in &ORDER[at(from)..] {
        let seg = entry(&doc, step)["segment"].as_u64().unwrap_or(1);
        for f in files_of(fs, root, step) {
            moves.push((f.clone(), format!("segments/{seg}/{f}")));
        }
    }
    for (_, to) in &moves {
        if fs.exists(&root.join(to)) {
            bail!("{to} exists: a resume never writes over a run's files");
        }
    }

    let first_t = segs
        .first()
        .and_then(|s| s["started"].as_f64())
        .unwrap_or(now);
    let reseat = at(from) > at(stops::STEP) && now - first_t > RESEAT_AFTER_S;
    let cut_off = ORDER.iter().any(|s| entry(&doc, s)["state"] == "running");
    let ended = ORDER
        .iter()
        .filter_map(|s| entry(&doc, s)["ended"].as_f64())
        .chain(segs.iter().filter_map(|s| s["ended"].as_f64()))
        .fold(f64::NEG_INFINITY, f64::max);
    let since = if cut_off || !ended.is_finite() {
        0.0
    } else {
        now - ended
    };
    let cool_s = if at(from) <= at(stops::STEP) {
        cooling_s(slow_tau_s, since)
    } else {
        0.0
    };
    Ok(Plan {
        from,
        n,
        tries,
        moves,
        tune: tune_on,
        last,
        reseat,
        cool_s,
        why,
        doc,
    })
}

/// The command that continues the stopped run `root`.
pub fn again(id: Id, root: &Path, a: &RunArgs) -> String {
    format!(
        "to continue it: osc-ident --id {} run --resume {} --nameplate {} --policy {}",
        id.as_byte(),
        root.display(),
        a.nameplate.display(),
        a.policy.display()
    )
}

/// One on-bus check, as the segment records it.
fn check(name: &str, value: String, bound: String, pass: bool) -> Value {
    json!({"name": name, "value": value, "bound": bound, "verdict": if pass { "pass" } else { "fail" }})
}

/// The resume of the run folder `root` on the servo `servo` (its chip id
/// as the bus says it), the profile and the policy of `a` admitted into
/// `i`: the lines it prints, and whether the tune accepted a set.
#[allow(clippy::too_many_arguments)]
pub async fn resume<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    i: &StepInputs,
    root: &Path,
    servo: &str,
) -> Result<(Vec<String>, bool)> {
    let run_cruise =
        async |rig: &mut Rig<'_, P>, i: &CruiseInputs<'_>| cruise::cruise(rig, i).await;
    let check = async |rig: &mut Rig<'_, P>, i: &VerifyInputs<'_>| {
        crate::steps::verify::verify(rig, i).await
    };
    resume_with(c, id, d, host, a, i, root, servo, run_cruise, check).await
}

/// [`resume`] with the cruise and the closed-loop check given: the
/// simulated servo runs no loop, so its tests script both.
#[allow(clippy::too_many_arguments)]
pub async fn resume_with<P: Pipe, C, V>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    a: &RunArgs<'_>,
    i: &StepInputs,
    root: &Path,
    servo: &str,
    run_cruise: C,
    check: V,
) -> Result<(Vec<String>, bool)>
where
    C: AsyncFnMut(&mut Rig<'_, P>, &CruiseInputs<'_>) -> Result<Cruised, Refused>,
    V: AsyncFnMut(&mut Rig<'_, P>, &VerifyInputs<'_>) -> Result<Verified, Refused>,
{
    let (feed, clock) = (host.feed, host.clock);
    let t0 = clock();
    let p = plan(host.files, root, a, i, t0)?;
    let a = RunArgs {
        tries: p.tries,
        ..*a
    };
    for step in &ORDER[..at(p.from)] {
        super::tell_end(feed, step, entry(&p.doc, step));
    }
    feed(&Progress::Started { step: STEP });
    let mut lines = vec![format!(
        "resume of {} as segment {}, from the {} step",
        root.display(),
        p.n,
        p.from
    )];
    lines.extend(p.why.iter().map(|w| format!("  {w}")));
    let mut told = 0;
    super::tell_lines(feed, &lines, &mut told);

    let (mut seg, refused) = checks(c, id, d, &p, root, servo, host.files, &mut lines).await;
    seg["n"] = p.n.into();
    seg["tool"] = host.tool.into();
    seg["from"] = p.from.into();
    seg["started"] = t0.into();
    seg["ended"] = Value::Null;
    seg["cooled_s"] = 0.into();
    seg["moved"] = p
        .moves
        .iter()
        .map(|(f, t)| json!({"from": f, "to": t}))
        .collect();
    if let Some(w) = &refused {
        seg["refused"] = w.clone().into();
    }
    let mut doc = p.doc.clone();
    push_segment(&mut doc, seg);
    let manifest = root.join("run.json");
    let ended = |why: Option<&String>| Progress::Ended {
        step: STEP,
        state: if why.is_some() { "refused" } else { "accepted" }.into(),
        why: why.cloned(),
    };
    let gone_on = async {
        rewrite(host.files, &manifest, &doc)?;
        if let Some(w) = refused {
            bail!("{w}");
        }
        before_the_steps(c, id, d, host, i, root, &p, &mut lines, &mut told).await
    }
    .await;
    if let Err(e) = gone_on {
        let w = format!("{e:#}");
        // a refusal is in the segment already; anything else is put there
        // now, as far as the store still takes a write
        let _ = refuse_segment(host.files, &manifest, &w);
        super::tell_lines(feed, &lines, &mut told);
        feed(&ended(Some(&w)));
        bail!("{w}");
    }
    super::tell_lines(feed, &lines, &mut told);
    feed(&ended(None));

    let (more, measured) = if at(p.from) < at(GATHER) {
        let (more, measured, _) = chain::resume_spins(c, id, d, host, i, root, p.from).await?;
        (more, measured)
    } else {
        (Vec::new(), true)
    };
    lines.extend(more);
    let past = Past {
        from: at(p.from).max(at(GATHER)),
        tune: p.tune.clone(),
        last: p.last.clone(),
    };
    let (lines, accepted) = super::past_steps_with(
        c, id, d, host, &a, root, lines, measured, &past, run_cruise, check,
    )
    .await?;
    end_segment(host.files, root, clock())?;
    Ok((lines, accepted))
}

/// The resume's own work once its segment is recorded and its checks
/// passed: the cooling wait, the re-seat, the stopped attempt's files
/// moved; a refusal in the tool's words, the segment's own where it has one.
#[allow(clippy::too_many_arguments)]
async fn before_the_steps<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    i: &StepInputs,
    root: &Path,
    p: &Plan,
    lines: &mut Vec<String>,
    told: &mut usize,
) -> Result<()> {
    let (feed, stop) = (host.feed, host.stop);
    let manifest = root.join("run.json");
    if p.cool_s > 0.0 {
        let tau = i
            .thermal
            .as_ref()
            .and_then(|t| t.slow_tau_s.as_ref())
            .map_or(0.0, |q| q.value);
        lines.push(format!(
            "the winding cools before the {} step: {:.0} s, torque off ({:.1} x the slow time constant {tau:.2} s, from {RISE_MAX_C} C down to {COOLED_C} C over the board, less the time since the last drive)",
            p.from,
            p.cool_s,
            (RISE_MAX_C / COOLED_C).ln()
        ));
        super::tell_lines(feed, lines, told);
        let of = p.cool_s.ceil().max(1.0) as u32;
        // the pauses' own sum: a pause is at least as long as asked
        let mut waited = 0.0;
        while waited < p.cool_s {
            if stop.load(Ordering::SeqCst) {
                let w = "interrupted while the winding cooled: nothing drove";
                lines.push(w.into());
                bail!("{w}");
            }
            feed(&Progress::Unit {
                step: STEP,
                at: (waited as u32 + 1).min(of),
                of,
                what: "s cooling",
            });
            let step = COOL_TELL_S.min(p.cool_s - waited);
            c.pause(Duration::from_secs_f64(step)).await;
            waited += step;
        }
        let mut doc = manifest_of(host.files, root)?;
        if let Some(s) = doc["segments"].as_array_mut().and_then(|s| s.last_mut()) {
            s["cooled_s"] = p.cool_s.into();
        }
        rewrite(host.files, &manifest, &doc)?;
    }

    if p.reseat {
        feed(&Progress::Unit {
            step: STEP,
            at: 1,
            of: 1,
            what: "re-seat",
        });
        let r = reseated(c, id, d, host, i, root, p.n, lines).await?;
        let mut doc = manifest_of(host.files, root)?;
        let fail = r["verdict"] != "pass";
        let why = r["why"].as_str().map(str::to_string);
        if let Some(s) = doc["segments"].as_array_mut().and_then(|s| s.last_mut()) {
            s["reseat"] = r;
            if let Some(w) = why.as_ref().filter(|_| fail) {
                s["refused"] = w.clone().into();
            }
        }
        rewrite(host.files, &manifest, &doc)?;
        if fail {
            bail!("{}", why.unwrap_or_default());
        }
    }

    for (f, t) in &p.moves {
        let to = root.join(t);
        if let Some(dir) = to.parent() {
            host.files
                .make_dirs(dir)
                .map_err(|e| anyhow::anyhow!("{}: {e}", dir.display()))?;
        }
        host.files
            .rename(&root.join(f), &to)
            .map_err(|e| anyhow::anyhow!("moving {f} to {t}: {e}"))?;
        lines.push(format!("moved {f} to {t}"));
    }
    Ok(())
}

fn push_segment(doc: &mut Value, seg: Value) {
    match doc["segments"].as_array_mut() {
        Some(s) => s.push(seg),
        None => doc["segments"] = json!([seg]),
    }
}

/// The last segment refused with `why`, in `run.json` at `manifest`.
fn refuse_segment(fs: &mut dyn Store, manifest: &Path, why: &str) -> Result<()> {
    let mut doc: Value = serde_json::from_slice(&files::read(fs, manifest)?)?;
    if let Some(s) = doc["segments"].as_array_mut().and_then(|s| s.last_mut()) {
        s["refused"] = why.into();
    }
    rewrite(fs, manifest, &doc)
}

/// The checks on the bus, before anything drives: the segment as far as
/// they fill it, and the first refusal, in the tool's words; a check that
/// could not be made refuses with what stopped it.
#[allow(clippy::too_many_arguments)]
async fn checks<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    p: &Plan,
    root: &Path,
    servo: &str,
    fs: &mut dyn Store,
    lines: &mut Vec<String>,
) -> (Value, Option<String>) {
    let doc = &p.doc;
    let first = &doc["segments"][0];
    let mut seg = json!({"rebooted": false, "ntc_c": null, "rail_v": null, "checks": []});
    let mut said = Vec::new();
    let mut notes = Vec::new();
    let made: Result<Option<String>> = async {
        let uid = doc["servo_uid"].as_str().unwrap_or_default();
        let pass = uid == servo;
        said.push(check(
            "the servo",
            servo.into(),
            format!("the run's {uid}"),
            pass,
        ));
        if !pass {
            return Ok(Some(format!(
                "servo {} is {servo}; the run is {uid}: a resume continues on the servo it began on",
                id.as_byte()
            )));
        }
        let now = state::read(c, id, d).await?;
        let torque = install::writable_now(&now);
        said.push(check(
            "torque off",
            if now.torque { "on" } else { "off" }.into(),
            "off".into(),
            torque.is_ok(),
        ));
        if let Err(e) = torque {
            return Ok(Some(e.0));
        }
        let mut steps = Vec::new();
        let stored = install::stored_state(c, id, d, &mut steps)
            .await
            .map_err(|e| anyhow::anyhow!("the reboot onto the stored settings: {e}"))?;
        seg["rebooted"] = true.into();
        lines.push(
            "rebooted onto the stored settings: the resumed segment starts from them; the reboot restarts the winding estimate from the board NTC"
                .into(),
        );
        lines.extend(install::moves(&steps).map(|l| format!("  {l}")));
        let f = |name: &str| d.field(name).map_or(Value::Null, |f| stored.text(f));
        let (m, v, cf) = (
            f("model_number"),
            f("firmware_version"),
            f("capability_flags"),
        );
        let w = &first["firmware"];
        let pass = (&m, &v, &cf) == (&w["model"], &w["version"], &w["capability_flags"]);
        said.push(check(
            "the firmware",
            format!("model {m} firmware {v} (capability_flags {cf})"),
            format!(
                "the run's {} {} ({})",
                w["model"], w["version"], w["capability_flags"]
            ),
            pass,
        ));
        if !pass {
            return Ok(Some(format!(
                "the servo runs model {m} firmware {v} (capability_flags {cf}); the run began on {} {} ({}): start a new run",
                w["model"], w["version"], w["capability_flags"]
            )));
        }
        let (record, _, _) = files::board_in(fs, &root.join("inputs"))?;
        let words = board::Words::of(&stored, d).map_err(|e| anyhow::anyhow!("{e}"))?;
        let (was, is) = (
            serde_json::to_value(&record.registers)?,
            serde_json::to_value(&words)?,
        );
        let differ: Vec<String> = was
            .as_object()
            .map(|o| {
                o.iter()
                    .filter(|(k, v)| is.get(k.as_str()) != Some(v))
                    .map(|(k, _)| k.clone())
                    .collect()
            })
            .unwrap_or_default();
        said.push(check(
            "the board facts",
            if differ.is_empty() {
                "as inputs/board.json".into()
            } else {
                differ.join(", ")
            },
            "inputs/board.json, byte for byte".into(),
            differ.is_empty(),
        ));
        if !differ.is_empty() {
            return Ok(Some(format!(
                "the board facts differ from inputs/board.json ({}): the servo is on another board; start a new run",
                differ.join(", ")
            )));
        }
        let unsaved = first["state"]["unsaved"].as_bool().unwrap_or(true);
        said.push(check(
            "the run began on stored settings",
            format!("unsaved {}", if unsaved { "yes" } else { "no" }),
            "unsaved no".into(),
            !unsaved,
        ));
        if unsaved {
            return Ok(Some(
                "the run began on settings not saved (unsaved yes): the stored settings it measured against are not known; start a new run"
                    .into(),
            ));
        }
        let (now_h, then_h) = (
            stored.hash(d),
            first["state"]["hash"]
                .as_str()
                .unwrap_or_default()
                .to_string(),
        );
        said.push(check(
            "the stored settings",
            now_h.clone(),
            format!("the run's {then_h}"),
            now_h == then_h,
        ));
        if now_h != then_h {
            return Ok(Some(format!(
                "the servo's stored settings are not the ones the run began on (state {now_h}, began {then_h}): a SAVE, a FACTORY or another tool's install since; start a new run"
            )));
        }
        let sense = Sense::of(&stored, d).map_err(|e| anyhow::anyhow!("{e}"))?;
        let rail = rest::rail_now(c, id, d, &sense).await?;
        seg["rail_v"] = rail.into();
        seg["ntc_c"] = chain::ntc_now(c, id, d, &stored)
            .await
            .map_or(Value::Null, Into::into);
        if at(p.from) == at(rest::STEP) {
            notes.push(format!(
                "  the rail: not checked: no accepted rest step to hold {rail:.2} V to; the rest step runs again"
            ));
            return Ok(None);
        }
        let rest_r = json_at(fs, &root.join(rest::RESULT))?;
        let then = rest_r["estimates"]["rail_v"]["value"]
            .as_f64()
            .context("the rest result holds no rail_v")?;
        let pass = (rail - then).abs() <= RAIL_SHARE * then;
        said.push(check(
            "the rail",
            format!("{rail:.2} V"),
            format!("the rest step's {then:.2} V within 5 %"),
            pass,
        ));
        if !pass {
            return Ok(Some(format!(
                "the rail reads {rail:.2} V; the run's rest step read {then:.2} V: more than 5 % apart; resume on the supply the run began on"
            )));
        }
        Ok(None)
    }
    .await;
    let refused = made.unwrap_or_else(|e| {
        Some(format!(
            "a check before the resume drives could not be made: {e:#}"
        ))
    });
    for s in &said {
        lines.push(format!(
            "  {}: {}, {} ({})",
            s["name"].as_str().unwrap_or_default(),
            s["value"].as_str().unwrap_or_default(),
            s["bound"].as_str().unwrap_or_default(),
            s["verdict"].as_str().unwrap_or_default()
        ));
    }
    lines.extend(notes);
    seg["checks"] = said.into();
    (seg, refused)
}

/// The high stop re-seated and judged against the run's: the segment's
/// `reseat` block, its `why` where it is refused.
#[allow(clippy::too_many_arguments)]
async fn reseated<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    host: &mut Host<'_>,
    i: &StepInputs,
    root: &Path,
    n: u32,
    lines: &mut Vec<String>,
) -> Result<Value> {
    let np = i
        .nameplate
        .as_ref()
        .context("the re-seat drives the motor: it needs the servo's nameplate")?;
    let (_, board, _) = files::board_in(host.files, &root.join("inputs"))?;
    let rest_r = json_at(host.files, &root.join(rest::RESULT))?;
    let probe_r = json_at(host.files, &root.join(probe::RESULT))?;
    let stops_r = json_at(host.files, &root.join(stops::RESULT))?;
    let (low, high) = (
        stops_r["stops"]["low"]["median_counts"].as_f64(),
        stops_r["stops"]["high"]["median_counts"].as_f64(),
    );
    let (Some(low), Some(high)) = (low, high) else {
        bail!("{} holds no stops' medians", stops::RESULT);
    };
    let polls = format!("captures/reseat-{n}/polls.csv");
    let (stop, clock) = (host.stop, host.clock);
    let mut sink = At {
        files: &mut *host.files,
        root: PathBuf::from(root),
    };
    let ran = {
        let mut rig = Rig {
            c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock,
            tell: &crate::steps::untold,
        };
        stops::reseat(
            &mut rig,
            &StopsInputs {
                nameplate: np,
                board: &board,
                thermal: i.thermal.as_ref(),
                rest: &rest_r,
                probe: &probe_r,
            },
            low,
            &polls,
        )
        .await
    };
    let r = match ran {
        Err(e) => {
            lines.push(format!("re-seat refused before any write: {e}"));
            return Ok(
                json!({"verdict": "fail", "why": format!("the re-seat refused before any write: {e}")}),
            );
        }
        Ok(r) => r,
    };
    lines.extend(r.ended.lines.iter().map(|l| format!("  {l}")));
    lines.push("the way out:".into());
    lines.extend(r.ended.exit.lines.iter().map(|l| format!("  {l}")));
    let mut out = match (r.ended.way, r.seat) {
        (crate::guard::Way::Done, Some(seat)) => {
            let (pass, mut v) = reseat_judge(seat, high);
            if !pass {
                v["why"] = format!(
                    "the high stop seats at {seat:.0}, {:.0} counts from the run's {high:.0} ({RESEAT_COUNTS} allowed): the horn or the gears moved since the run; start a new run",
                    (seat - high).abs()
                )
                .into();
            }
            v
        }
        (way, _) => json!({"stop": "high", "run_counts": high, "verdict": "fail",
                           "why": format!("the re-seat was {}: {}", way.name(),
                                          r.ended.why.clone().unwrap_or_default())}),
    };
    out["polls"] = polls.into();
    lines.push(format!(
        "re-seat: {}",
        out["why"].as_str().map_or_else(
            || format!("the high stop within {RESEAT_COUNTS} counts of the run's: pass"),
            str::to_string
        )
    ));
    Ok(out)
}

/// The words a stopped run's folder gets for its continuation, where a
/// resume of it would not refuse on its files.
pub fn continue_line(
    fs: &mut dyn Store,
    id: Id,
    root: &Path,
    a: &RunArgs,
    i: &StepInputs,
    now: f64,
) -> Option<String> {
    plan(fs, root, a, i, now).ok().map(|_| again(id, root, a))
}
