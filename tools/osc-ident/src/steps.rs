//! `plan` and `step`: what a run would do on this servo, its numbers read
//! off it, and one step of the run into a run folder. `plan` only reads.

use std::path::{Path, PathBuf};
use std::sync::atomic::AtomicBool;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use osc_client::descriptor::Descriptor;
use osc_client::pipe::Pipe;
use osc_client::{Client, Id};
use osc_measure::capture::rows::Layout;
use osc_measure::capture::tel;
use osc_measure::envelope::{Envelope, Facts, HeatModel, Rung};
use osc_measure::inputs::ThermalInputs;
use osc_measure::nameplate::Nameplate;
use osc_measure::state;
use osc_measure::steps::cruise::{self, CruiseInputs};
use osc_measure::steps::probe::{self, ProbeInputs};
use osc_measure::steps::rest::{self, DEADMAN_S, ROWS, Sense};
use osc_measure::steps::spins::{self as spin, SpinsInputs};
use osc_measure::steps::stops::{self, StopsInputs};
use osc_measure::steps::table::{self, Schedule, TableInputs};
use osc_measure::steps::verify::{self, VerifyInputs};
use osc_measure::steps::window::{self, WindowInputs};
use osc_measure::steps::{Ended, Rig};
use osc_measure::{Board, sink::Sink};
use serde_json::{Value, json};

use crate::files::{self, DirSink};

/// What `plan` and the steps read besides the servo.
pub struct StepInputs {
    pub nameplate: Option<Nameplate>,
    pub thermal: Option<ThermalInputs>,
    pub board: Board,
}

impl StepInputs {
    pub fn load(
        nameplate: Option<&Path>,
        thermal: Option<&Path>,
        board: Option<&Path>,
    ) -> Result<Self> {
        let json = |p: &Path| -> Result<Value> {
            serde_json::from_slice(&files::read(p)?)
                .with_context(|| format!("parsing {}", p.display()))
        };
        Ok(StepInputs {
            nameplate: nameplate.map(files::nameplate).transpose()?,
            thermal: thermal
                .map(|p| Ok::<_, anyhow::Error>(serde_json::from_value(json(p)?)?))
                .transpose()?,
            board: files::board(board)?,
        })
    }
}

fn unix_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

async fn read_u16<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor, name: &str) -> Result<u16> {
    let f = d
        .field(name)
        .with_context(|| format!("the descriptor names no {name}"))?;
    let b = c.read(id, f.addr, 2).await?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

fn nameplate_said(np: Option<&Nameplate>) -> String {
    let Some(np) = np else {
        return "nameplate: none given (--nameplate): no drive is priced".into();
    };
    let rows = |v: Vec<String>| {
        if v.is_empty() {
            "none".into()
        } else {
            v.join(", ")
        }
    };
    format!(
        "nameplate: {}, gear {}, ratio {}, motor {}, operating {}, runs on {}, no-load {}, stall {}",
        np.name,
        np.gear_class
            .map_or("not given".into(), |g| format!("{g:?}").to_lowercase()),
        np.gear_ratio.map_or("not given".into(), |r| r.to_string()),
        match (np.brushes, np.rotor_segments, np.events_per_rev()) {
            (Some(b), Some(s), Ok(n)) => {
                format!("{b} brushes x {s} rotor segments, {n} ripple events a revolution")
            }
            _ => "brushes and rotor_segments not given".into(),
        },
        np.operating_v
            .map_or("not given".into(), |v| format!("{} to {} V", v.min, v.max)),
        np.run_v.map_or("not given".into(), |v| format!("{v} V")),
        rows(
            np.no_load
                .iter()
                .map(|r| format!("{} s per {} deg at {} V", r.seconds, r.per_deg, r.at_v))
                .collect()
        ),
        rows(
            np.stall
                .iter()
                .map(|r| match r.current_a {
                    Some(a) => format!("{a} A at {} V", r.at_v),
                    None => format!("at {} V, no current stated", r.at_v),
                })
                .collect()
        ),
    )
}

/// The run this tool would do on the servo, with its rail, floors and
/// nameplate, and the five budgets with their numbers; whether step 1
/// would start. Reads only.
pub async fn plan<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
) -> Result<(Vec<String>, bool)> {
    let line = tel::line(c, id).await?;
    let state = state::read(c, id, d).await?;
    let sense = Sense::of(&state, d)?;
    let rail = rest::rail_now(c, id, d, &sense).await?;
    let ntc_c = sense
        .ntc
        .c_of_raw(read_u16(c, id, d, "ntc_raw").await? as f64);
    let pos = read_u16(c, id, d, "pos").await? as f64;
    let n = |name: &str| d.field(name).map_or(Value::Null, |f| state.text(f));
    let pwm = n("pwm_period_ticks").as_f64().unwrap_or(0.0);
    let floor = |name: &str| {
        let t = n(name).as_f64().unwrap_or(0.0);
        format!(
            "{name} {t} ({:.1}% duty)",
            if pwm > 0.0 { 100.0 * t / pwm } else { 0.0 }
        )
    };
    let q15 = |v: u16| format!("{v} ({:.1}%)", 100.0 * v as f64 / 32768.0);
    let mut out = vec![
        format!(
            "servo {}: model {}, firmware {}, {}; bus at {} baud",
            id.as_byte(),
            n("model_number"),
            n("firmware_version"),
            line.said(),
            line.baud.as_hz()
        ),
        format!(
            "rail {rail:.2} V ({:.2} V at the connector), board NTC {ntc_c:.1} C, position {pos:.0} counts; settings {}, unsaved {}, refused {}",
            sense.supply_v(rail),
            state.settings_said(),
            state.unsaved_said(),
            state.refused_said()
        ),
        format!(
            "floors: {}, {}; published window_floor_q15 {}, window_v_floor_q15 {}",
            floor("i_window_min_ticks"),
            floor("v_window_min_ticks"),
            q15(read_u16(c, id, d, "window_floor_q15").await?),
            q15(read_u16(c, id, d, "window_v_floor_q15").await?)
        ),
        nameplate_said(i.nameplate.as_ref()),
    ];
    let np = i.nameplate.clone().unwrap_or_default();
    let mut facts = Facts::new(&np, rail);
    facts.ntc_c = Some(ntc_c);
    let heat = match (&facts.r, &i.thermal) {
        (_, None) => Err("no thermal values entered (--thermal)".to_string()),
        (None, _) => Err(
            "R is not known: the nameplate states no stall current, the probe and the stops step measure it".to_string(),
        ),
        (Some(r), Some(t)) => {
            HeatModel::new(&i.board, t, r.ohm, sense.counts_per_a(), sense.tick_hz)
        }
    };
    let mut env = Envelope::new(facts, heat);
    let admit = line
        .admit(Layout::Rest)
        .and_then(|()| line.admit(Layout::Raw));
    out.push(format!(
        "step 0: connect, refuse under 3 Mbps, read the board's facts, check the nameplate: {}",
        match &admit {
            Ok(()) => format!(
                "the bus runs at {} Mbaud, TEL rows of {} B: passes",
                line.baud.as_hz() / 1_000_000,
                line.row_bytes_max
            ),
            Err(e) => format!("REFUSED: {e}"),
        }
    ));
    let span = ROWS as f64 / sense.tick_hz;
    out.push(format!(
        "step 1: rest and board check, nothing drives: torque off, the rest row for {span} s ({ROWS} rows); the dead-man {DEADMAN_S} s, torque on at zero duty in OpenLoop, the raw row for {span} s ({ROWS} rows)"
    ));
    for r in [
        Rung::still("rest, torque off", span, pos),
        Rung::still("raw, torque on at zero duty", span, pos),
    ] {
        match env.admit(&r) {
            Ok(p) => out.push(format!("  {}", p.said(&r))),
            Err(e) => out.push(format!("  {e}")),
        }
    }
    out.push(format!(
        "  the servo's stored stops {}..{} are not taken: step 3 identifies them",
        n("raw_min"),
        n("raw_max")
    ));
    out.push("budgets:".into());
    out.extend(env.budgets().into_iter().map(|b| format!("  {b}")));
    Ok((out, admit.is_ok()))
}

/// One step as the manifest records it.
struct Ran<'a> {
    step: &'static str,
    ended: Option<&'a Ended>,
    result: Option<&'a Value>,
    refused: Option<String>,
    times: (f64, f64),
}

/// The run folder's minimal manifest: every step it ran, in order.
fn manifest(uid: &str, ran: &[Ran]) -> Value {
    let result = ran.iter().find_map(|r| r.result);
    let steps: serde_json::Map<String, Value> = ran
        .iter()
        .map(|r| {
            (
                r.step.to_string(),
                json!({
                    "state": r.ended.map_or("refused", |e| e.way.name()),
                    "why": r.ended.and_then(|e| e.why.clone()).or(r.refused.clone()),
                    "started": r.times.0,
                    "ended": r.times.1,
                }),
            )
        })
        .collect();
    json!({
        "tool": format!("osc-ident {}", env!("CARGO_PKG_VERSION")),
        "servo_uid": uid,
        "image": result.map_or(Value::Null, |r| r["image"].clone()),
        "capability_flags": result.map_or(Value::Null, |r| r["capability_flags"].clone()),
        "steps": steps,
    })
}

/// A step's lines as the CLI prints them: what it did, its checks, the
/// way out, its verdict.
pub(crate) fn said(
    step: &str,
    ended: &Ended,
    checks: &[osc_measure::steps::Check],
    lines: &mut Vec<String>,
) {
    lines.extend(ended.lines.iter().cloned());
    lines.extend(checks.iter().map(|c| format!("  {}", c.said())));
    lines.push("the way out:".into());
    lines.extend(ended.exit.lines.iter().map(|l| format!("  {l}")));
    lines.push(format!(
        "step {step} {}{}",
        ended.way.name(),
        ended
            .why
            .as_ref()
            .map_or(String::new(), |w| format!(": {w}"))
    ));
}

/// `step rest` into `<out>/<servo>/<UTC time>/`: the lines it prints, and
/// whether it was accepted.
pub async fn step_rest<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    let root = files::run_folder(out, servo)?;
    let mut sink = DirSink { root: root.clone() };
    let t0 = unix_s();
    let clock = unix_s;
    let ran = {
        let mut rig = Rig {
            c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        rest::rest(&mut rig, i.nameplate.as_ref()).await
    };
    let times = (t0, unix_s());
    let mut lines = Vec::new();
    let entry = Ran {
        step: rest::STEP,
        ended: ran.as_ref().ok().map(|r| &r.ended),
        result: ran.as_ref().ok().and_then(|r| r.result.as_ref()),
        refused: ran.as_ref().err().map(|e| e.0.clone()),
        times,
    };
    let mut text = serde_json::to_vec_pretty(&manifest(servo, &[entry]))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    let accepted = match ran {
        Err(e) => {
            lines.push(format!("refused before any write: {e}"));
            false
        }
        Ok(r) => {
            said(rest::STEP, &r.ended, &r.checks, &mut lines);
            r.accepted()
        }
    };
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}

/// `step probe` into `<out>/<servo>/<UTC time>/`: the rest step first, into
/// the same folder, for its zeros, pot noise and frame ripple; then the
/// probe on its result. The lines it prints, and whether both were
/// accepted.
#[allow(clippy::too_many_arguments)]
pub async fn step_probe<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    let np = i
        .nameplate
        .as_ref()
        .context("step probe drives the motor: it needs the servo's nameplate (--nameplate)")?;
    let root = files::run_folder(out, servo)?;
    let mut sink = DirSink { root: root.clone() };
    let clock = unix_s;
    let mut lines = Vec::new();
    let t0 = unix_s();
    let rested = {
        let mut rig = Rig {
            c: &mut *c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        rest::rest(&mut rig, Some(np)).await
    };
    let t1 = unix_s();
    let rest_ok = match &rested {
        Err(e) => {
            lines.push(format!("step rest refused before any write: {e}"));
            false
        }
        Ok(r) => {
            said(rest::STEP, &r.ended, &r.checks, &mut lines);
            r.accepted()
        }
    };
    let mut probed = None;
    let mut t2 = t1;
    if rest_ok {
        let text = sink.read(rest::RESULT)?;
        let rest_result: Value = serde_json::from_slice(&text).context("the rest result")?;
        let ran = {
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            probe::probe(
                &mut rig,
                &ProbeInputs {
                    nameplate: np,
                    board: &i.board,
                    thermal: i.thermal.as_ref(),
                    rest: &rest_result,
                },
            )
            .await
        };
        t2 = unix_s();
        match &ran {
            Err(e) => lines.push(format!("step probe refused before any write: {e}")),
            Ok(p) => said(probe::STEP, &p.ended, &p.checks, &mut lines),
        }
        probed = Some(ran);
    } else {
        lines.push("step probe not run: it needs an accepted rest step".into());
    }
    let mut ran = vec![Ran {
        step: rest::STEP,
        ended: rested.as_ref().ok().map(|r| &r.ended),
        result: rested.as_ref().ok().and_then(|r| r.result.as_ref()),
        refused: rested.as_ref().err().map(|e| e.0.clone()),
        times: (t0, t1),
    }];
    if let Some(p) = &probed {
        ran.push(Ran {
            step: probe::STEP,
            ended: p.as_ref().ok().map(|p| &p.ended),
            result: p.as_ref().ok().and_then(|p| p.result.as_ref()),
            refused: p.as_ref().err().map(|e| e.0.clone()),
            times: (t1, t2),
        });
    }
    let mut text = serde_json::to_vec_pretty(&manifest(servo, &ran))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    let accepted = matches!(&probed, Some(Ok(p)) if p.accepted());
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}

/// The step a chained run ends on, in the chain's order.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Last {
    Stops,
    Table,
    Window,
    Spins,
}

/// A step as the chain keeps it: its end and result, or its refusal.
type Kept = (
    &'static str,
    Option<(Ended, Option<Value>)>,
    Option<String>,
    (f64, f64),
);

fn result_of(sink: &mut DirSink, rel: &str, what: &str) -> Result<Value> {
    serde_json::from_slice(&sink.read(rel)?).with_context(|| format!("the {what} result"))
}

/// `step verify` into `<out>/<servo>/<UTC time>/`: the closed-loop check
/// of the installed set. The lines it prints, and whether the set
/// was accepted.
pub async fn step_verify<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    let np = i
        .nameplate
        .as_ref()
        .context("step verify drives the motor: it needs the servo's nameplate (--nameplate)")?;
    let root = files::run_folder(out, servo)?;
    let mut sink = DirSink { root: root.clone() };
    let t0 = unix_s();
    let clock = unix_s;
    let ran = {
        let mut rig = Rig {
            c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        verify::verify(&mut rig, &VerifyInputs { nameplate: np }).await
    };
    let entry = Ran {
        step: verify::STEP,
        ended: ran.as_ref().ok().map(|r| &r.ended),
        result: ran.as_ref().ok().and_then(|r| r.result.as_ref()),
        refused: ran.as_ref().err().map(|e| e.0.clone()),
        times: (t0, unix_s()),
    };
    let mut text = serde_json::to_vec_pretty(&manifest(servo, &[entry]))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    let mut lines = Vec::new();
    let accepted = match ran {
        Err(e) => {
            lines.push(format!("refused before any write: {e}"));
            false
        }
        Ok(r) => {
            said(verify::STEP, &r.ended, &r.checks, &mut lines);
            r.accepted()
        }
    };
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}

/// `step cruise` into `<out>/<servo>/<UTC time>/`: the friction line from
/// closed-loop cruises on the installed set, the travel the stops
/// step's of the run folder `run` where given, else the set's. The lines it
/// prints, and whether the line was accepted.
#[allow(clippy::too_many_arguments)]
pub async fn step_cruise<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
    run: Option<&Path>,
) -> Result<(Vec<String>, bool, PathBuf)> {
    let np = i
        .nameplate
        .as_ref()
        .context("step cruise drives the motor: it needs the servo's nameplate (--nameplate)")?;
    let stops = run
        .map(|r| -> Result<Value> {
            let p = r.join(stops::RESULT);
            serde_json::from_slice(&files::read(&p)?)
                .with_context(|| format!("parsing {}", p.display()))
        })
        .transpose()?;
    let root = files::run_folder(out, servo)?;
    let mut sink = DirSink { root: root.clone() };
    let t0 = unix_s();
    let clock = unix_s;
    let ran = {
        let mut rig = Rig {
            c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        cruise::cruise(
            &mut rig,
            &CruiseInputs {
                nameplate: np,
                board: &i.board,
                stops: stops.as_ref(),
            },
        )
        .await
    };
    let entry = Ran {
        step: cruise::STEP,
        ended: ran.as_ref().ok().map(|r| &r.ended),
        result: ran.as_ref().ok().and_then(|r| r.result.as_ref()),
        refused: ran.as_ref().err().map(|e| e.0.clone()),
        times: (t0, unix_s()),
    };
    let mut text = serde_json::to_vec_pretty(&manifest(servo, &[entry]))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    let mut lines = Vec::new();
    let accepted = match ran {
        Err(e) => {
            lines.push(format!("refused before any write: {e}"));
            false
        }
        Ok(r) => {
            said(cruise::STEP, &r.ended, &r.checks, &mut lines);
            r.accepted()
        }
    };
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}

/// `step stops` into `<out>/<servo>/<UTC time>/`: the rest step, the probe
/// on its result, then the stops on both, into the same folder, each
/// started only on the one before it accepted. The lines it prints, and
/// whether all three were accepted.
pub async fn step_stops<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    chain(c, id, d, i, out, servo, stop, Last::Stops).await
}

/// `step table` into `<out>/<servo>/<UTC time>/`: rest, the probe, the
/// stops, then the table step's slow and fast passes on all three, into
/// the same folder, each started only on the one before it accepted. The
/// lines it prints, and whether all four were accepted.
pub async fn step_table<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    chain(c, id, d, i, out, servo, stop, Last::Table).await
}

/// `step window` into `<out>/<servo>/<UTC time>/`: rest, the probe, the
/// stops, the table, then the window term's onsets on the first three,
/// into the same folder, each started only on the one before it accepted.
/// The lines it prints, and whether all five were accepted.
#[allow(clippy::too_many_arguments)]
pub async fn step_window<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    chain(c, id, d, i, out, servo, stop, Last::Window).await
}

/// `step spins` into `<out>/<servo>/<UTC time>/`: rest, the probe, the
/// stops, the table, the window term, then the spins, into the same
/// folder, each started only on the one before it accepted. The lines it
/// prints, and whether all six were accepted.
pub async fn step_spins<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
) -> Result<(Vec<String>, bool, PathBuf)> {
    chain(c, id, d, i, out, servo, stop, Last::Spins).await
}

#[allow(clippy::too_many_arguments)]
async fn chain<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    i: &StepInputs,
    out: &Path,
    servo: &str,
    stop: &AtomicBool,
    last: Last,
) -> Result<(Vec<String>, bool, PathBuf)> {
    let name = match last {
        Last::Stops => stops::STEP,
        Last::Table => table::STEP,
        Last::Window => window::STEP,
        Last::Spins => spin::STEP,
    };
    let np = i.nameplate.as_ref().with_context(|| {
        format!("step {name} drives the motor: it needs the servo's nameplate (--nameplate)")
    })?;
    let root = files::run_folder(out, servo)?;
    let mut sink = DirSink { root: root.clone() };
    let clock = unix_s;
    let mut lines = Vec::new();
    let mut ran: Vec<Kept> = Vec::new();
    let t0 = unix_s();
    let rested = {
        let mut rig = Rig {
            c: &mut *c,
            id,
            d,
            sink: &mut sink,
            stop,
            clock: &clock,
        };
        rest::rest(&mut rig, Some(np)).await
    };
    let t1 = unix_s();
    let rest_ok = match &rested {
        Err(e) => {
            lines.push(format!("step rest refused before any write: {e}"));
            false
        }
        Ok(r) => {
            said(rest::STEP, &r.ended, &r.checks, &mut lines);
            r.accepted()
        }
    };
    ran.push((
        rest::STEP,
        rested
            .as_ref()
            .ok()
            .map(|r| (r.ended.clone(), r.result.clone())),
        rested.as_ref().err().map(|e| e.0.clone()),
        (t0, t1),
    ));
    let mut probe_ok = false;
    let mut t2 = t1;
    if rest_ok {
        let rest_result = result_of(&mut sink, rest::RESULT, "rest")?;
        let probed = {
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            probe::probe(
                &mut rig,
                &ProbeInputs {
                    nameplate: np,
                    board: &i.board,
                    thermal: i.thermal.as_ref(),
                    rest: &rest_result,
                },
            )
            .await
        };
        t2 = unix_s();
        match &probed {
            Err(e) => lines.push(format!("step probe refused before any write: {e}")),
            Ok(p) => {
                said(probe::STEP, &p.ended, &p.checks, &mut lines);
                probe_ok = p.accepted();
            }
        }
        ran.push((
            probe::STEP,
            probed
                .as_ref()
                .ok()
                .map(|p| (p.ended.clone(), p.result.clone())),
            probed.as_ref().err().map(|e| e.0.clone()),
            (t1, t2),
        ));
    } else {
        lines.push("step probe not run: it needs an accepted rest step".into());
    }
    let mut stops_ok = false;
    let mut t3 = t2;
    if probe_ok {
        let rest_result = result_of(&mut sink, rest::RESULT, "rest")?;
        let probe_result = result_of(&mut sink, probe::RESULT, "probe")?;
        let found = {
            let mut rig = Rig {
                c: &mut *c,
                id,
                d,
                sink: &mut sink,
                stop,
                clock: &clock,
            };
            stops::stops(
                &mut rig,
                &StopsInputs {
                    nameplate: np,
                    board: &i.board,
                    thermal: i.thermal.as_ref(),
                    rest: &rest_result,
                    probe: &probe_result,
                },
            )
            .await
        };
        t3 = unix_s();
        match &found {
            Err(e) => lines.push(format!("step stops refused before any write: {e}")),
            Ok(s) => {
                said(stops::STEP, &s.ended, &s.checks, &mut lines);
                stops_ok = s.accepted();
            }
        }
        ran.push((
            stops::STEP,
            found
                .as_ref()
                .ok()
                .map(|s| (s.ended.clone(), s.result.clone())),
            found.as_ref().err().map(|e| e.0.clone()),
            (t2, t3),
        ));
    } else if rest_ok {
        lines.push("step stops not run: it needs an accepted probe".into());
    }
    let mut accepted = stops_ok;
    let mut t4 = t3;
    if last >= Last::Table {
        accepted = false;
        if stops_ok {
            let rest_result = result_of(&mut sink, rest::RESULT, "rest")?;
            let probe_result = result_of(&mut sink, probe::RESULT, "probe")?;
            let stops_result = result_of(&mut sink, stops::RESULT, "stops")?;
            let tabled = {
                let mut rig = Rig {
                    c: &mut *c,
                    id,
                    d,
                    sink: &mut sink,
                    stop,
                    clock: &clock,
                };
                table::table(
                    &mut rig,
                    &TableInputs {
                        nameplate: np,
                        board: &i.board,
                        thermal: i.thermal.as_ref(),
                        rest: &rest_result,
                        probe: &probe_result,
                        stops: &stops_result,
                        schedule: Schedule::PLAN,
                    },
                )
                .await
            };
            t4 = unix_s();
            match &tabled {
                Err(e) => lines.push(format!("step table refused before any write: {e}")),
                Ok(t) => {
                    said(table::STEP, &t.ended, &t.checks, &mut lines);
                    accepted = t.accepted();
                }
            }
            ran.push((
                table::STEP,
                tabled
                    .as_ref()
                    .ok()
                    .map(|t| (t.ended.clone(), t.result.clone())),
                tabled.as_ref().err().map(|e| e.0.clone()),
                (t3, t4),
            ));
        } else if probe_ok {
            lines.push("step table not run: it needs an accepted stops step".into());
        }
    }
    let table_ok = accepted;
    let mut t5 = t4;
    if last >= Last::Window {
        accepted = false;
        if table_ok {
            let rest_result = result_of(&mut sink, rest::RESULT, "rest")?;
            let probe_result = result_of(&mut sink, probe::RESULT, "probe")?;
            let stops_result = result_of(&mut sink, stops::RESULT, "stops")?;
            let windowed = {
                let mut rig = Rig {
                    c: &mut *c,
                    id,
                    d,
                    sink: &mut sink,
                    stop,
                    clock: &clock,
                };
                window::window(
                    &mut rig,
                    &WindowInputs {
                        nameplate: np,
                        board: &i.board,
                        thermal: i.thermal.as_ref(),
                        rest: &rest_result,
                        probe: &probe_result,
                        stops: &stops_result,
                    },
                )
                .await
            };
            t5 = unix_s();
            match &windowed {
                Err(e) => lines.push(format!("step window refused before any write: {e}")),
                Ok(w) => {
                    said(window::STEP, &w.ended, &w.checks, &mut lines);
                    accepted = w.accepted();
                }
            }
            ran.push((
                window::STEP,
                windowed
                    .as_ref()
                    .ok()
                    .map(|w| (w.ended.clone(), w.result.clone())),
                windowed.as_ref().err().map(|e| e.0.clone()),
                (t4, t5),
            ));
        } else if stops_ok {
            lines.push("step window not run: it needs an accepted table step".into());
        }
    }
    if last == Last::Spins {
        let window_ok = accepted;
        accepted = false;
        if window_ok {
            let rest_result = result_of(&mut sink, rest::RESULT, "rest")?;
            let probe_result = result_of(&mut sink, probe::RESULT, "probe")?;
            let stops_result = result_of(&mut sink, stops::RESULT, "stops")?;
            let window_result = result_of(&mut sink, window::RESULT, "window")?;
            let spun = {
                let mut rig = Rig {
                    c: &mut *c,
                    id,
                    d,
                    sink: &mut sink,
                    stop,
                    clock: &clock,
                };
                spin::spins(
                    &mut rig,
                    &SpinsInputs {
                        nameplate: np,
                        board: &i.board,
                        thermal: i.thermal.as_ref(),
                        rest: &rest_result,
                        probe: &probe_result,
                        stops: &stops_result,
                        window: Some(&window_result),
                    },
                )
                .await
            };
            let t6 = unix_s();
            match &spun {
                Err(e) => lines.push(format!("step spins refused before any write: {e}")),
                Ok(s) => {
                    said(spin::STEP, &s.ended, &s.checks, &mut lines);
                    accepted = s.accepted();
                }
            }
            ran.push((
                spin::STEP,
                spun.as_ref()
                    .ok()
                    .map(|s| (s.ended.clone(), s.result.clone())),
                spun.as_ref().err().map(|e| e.0.clone()),
                (t5, t6),
            ));
        } else if table_ok {
            lines.push("step spins not run: it needs an accepted window step".into());
        }
    }
    let entries: Vec<Ran> = ran
        .iter()
        .map(|(step, e, refused, times)| Ran {
            step,
            ended: e.as_ref().map(|(e, _)| e),
            result: e.as_ref().and_then(|(_, r)| r.as_ref()),
            refused: refused.clone(),
            times: *times,
        })
        .collect();
    let mut text = serde_json::to_vec_pretty(&manifest(servo, &entries))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}
