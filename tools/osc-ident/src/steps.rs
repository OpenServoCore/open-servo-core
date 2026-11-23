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
use osc_measure::steps::Rig;
use osc_measure::steps::rest::{self, DEADMAN_S, ROWS, Sense};
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
            nameplate: nameplate
                .map(|p| Ok::<_, anyhow::Error>(serde_json::from_value(json(p)?)?))
                .transpose()?,
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
        "nameplate: {}, gear {}, ratio {}, operating {}, runs on {}, no-load {}, stall {}",
        np.name,
        np.gear_class
            .map_or("not given".into(), |g| format!("{g:?}").to_lowercase()),
        np.gear_ratio.map_or("not given".into(), |r| r.to_string()),
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
            "rail {rail:.2} V ({:.2} V at the connector), board NTC {ntc_c:.1} C, position {pos:.0} counts; data state {}, stamp {}",
            sense.supply_v(rail),
            state.data_said(),
            state.stamp_said()
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
            "R is not known: the nameplate states no stall current, step 4 measures it".to_string(),
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

/// The run folder's minimal manifest for one step.
fn manifest(
    uid: &str,
    step: &str,
    rested: Option<&rest::Rested>,
    refused: Option<&str>,
    times: (f64, f64),
) -> Value {
    let result = rested.and_then(|r| r.result.as_ref());
    let state = rested.map_or("refused", |r| r.ended.way.name());
    json!({
        "tool": format!("osc-ident {}", env!("CARGO_PKG_VERSION")),
        "servo_uid": uid,
        "image": result.map_or(Value::Null, |r| r["image"].clone()),
        "capability_flags": result.map_or(Value::Null, |r| r["capability_flags"].clone()),
        "steps": {
            step: {
                "state": state,
                "why": rested.and_then(|r| r.ended.why.clone()).or(refused.map(str::to_string)),
                "started": times.0,
                "ended": times.1,
            }
        }
    })
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
    let (rested, refused) = match &ran {
        Ok(r) => (Some(r), None),
        Err(e) => (None, Some(e.0.as_str())),
    };
    let mut text = serde_json::to_vec_pretty(&manifest(servo, rest::STEP, rested, refused, times))?;
    text.push(b'\n');
    sink.write("run.json", &text)?;
    let accepted = match ran {
        Err(e) => {
            lines.push(format!("refused before any write: {e}"));
            false
        }
        Ok(r) => {
            lines.extend(r.ended.lines.iter().cloned());
            lines.extend(r.checks.iter().map(|c| format!("  {}", c.said())));
            lines.push("the way out:".into());
            lines.extend(r.ended.exit.lines.iter().map(|l| format!("  {l}")));
            lines.push(format!(
                "step rest {}{}",
                r.ended.way.name(),
                r.ended
                    .why
                    .as_ref()
                    .map_or(String::new(), |w| format!(": {w}"))
            ));
            r.accepted()
        }
    };
    lines.push(format!("run folder: {}", root.display()));
    Ok((lines, accepted, root))
}
