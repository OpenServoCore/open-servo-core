//! The measurement steps. A step runs on a [`Rig`]: the bus, the files,
//! the interrupt flag, the host's clock. It refuses with nothing written
//! when it cannot start, and once it has written it always ends through
//! [`crate::guard::exit`].

pub mod drive;
pub mod polarity;
pub mod probe;
pub mod rest;
pub mod runway;
pub mod seek;
pub mod shunt;
pub mod spins;
pub mod stops;
#[cfg(test)]
pub(crate) mod testkit;
pub mod verify;
pub mod window;

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use osc_client::descriptor::Descriptor;
use osc_client::{Client, Error, Id, pipe::Pipe};
use serde_json::{Map, Value, json};

use crate::capture::meta;
use crate::capture::rows::Layout;
use crate::capture::tel::{self, Burst, StreamCfg};
use crate::guard::{Exit, Way};
use crate::install::Refused;
use crate::sink::{Sink, SinkError};
use crate::state::field;

/// What a step runs on.
pub struct Rig<'a, P: Pipe> {
    pub c: &'a mut Client<P>,
    pub id: Id,
    pub d: &'a Descriptor,
    pub sink: &'a mut dyn Sink,
    /// Set by an interrupt; read between rungs, so a burst runs out.
    pub stop: &'a AtomicBool,
    /// The host's clock, unix seconds: the browser's and the CLI's own.
    pub clock: &'a dyn Fn() -> f64,
}

/// Why a step stops short.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Halt {
    Refused(String),
    Error(String),
    Interrupted,
}

impl Halt {
    pub fn way(&self) -> Way {
        match self {
            Halt::Refused(_) => Way::Refused,
            Halt::Error(_) => Way::Error,
            Halt::Interrupted => Way::Interrupted,
        }
    }

    pub fn why(&self) -> String {
        match self {
            Halt::Refused(w) | Halt::Error(w) => w.clone(),
            Halt::Interrupted => "interrupted: the step stopped between rungs".into(),
        }
    }
}

impl From<Error> for Halt {
    fn from(e: Error) -> Self {
        Halt::Error(format!("bus: {e}"))
    }
}

impl From<SinkError> for Halt {
    fn from(e: SinkError) -> Self {
        Halt::Error(format!("files: {e}"))
    }
}

impl From<Refused> for Halt {
    fn from(e: Refused) -> Self {
        Halt::Refused(e.0)
    }
}

/// How a step ended once it had written: its way, why, the way out's
/// moves, and what it did and found, a line each.
#[derive(Clone, Debug)]
pub struct Ended {
    pub way: Way,
    pub why: Option<String>,
    pub exit: Exit,
    pub lines: Vec<String>,
}

/// One acceptance check, in universal units, with its numbers.
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub name: String,
    pub value: String,
    pub bound: String,
    pub verdict: Verdict,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Verdict {
    Pass,
    /// Refuses the step.
    Fail,
    /// Said, never a refusal.
    Warn,
    /// The check has nothing to judge by; its numbers are still said.
    NotJudged,
}

impl Verdict {
    pub fn word(self) -> &'static str {
        match self {
            Verdict::Pass => "pass",
            Verdict::Fail => "FAIL",
            Verdict::Warn => "warning",
            Verdict::NotJudged => "not judged",
        }
    }
}

impl Check {
    pub fn said(&self) -> String {
        format!(
            "{}: {} ({}): {}",
            self.name,
            self.value,
            self.bound,
            self.verdict.word()
        )
    }

    pub fn json(&self) -> Value {
        json!({"name": self.name, "value": self.value, "bound": self.bound, "verdict": self.verdict.word()})
    }
}

pub(crate) fn interrupted(stop: &AtomicBool) -> Result<(), Halt> {
    if stop.load(Ordering::SeqCst) {
        Err(Halt::Interrupted)
    } else {
        Ok(())
    }
}

pub(crate) async fn write<P: Pipe>(
    rig: &mut Rig<'_, P>,
    name: &str,
    bytes: &[u8],
) -> Result<(), Halt> {
    let f = field(rig.d, name)?;
    rig.c.write(rig.id, f.addr, bytes).await?;
    Ok(())
}

/// A burst as received, waiting to be kept: the way out runs first, since
/// compressing a second of rows takes longer than a dead-man allows.
pub struct Taken {
    pub nn: u32,
    pub burst: Burst,
    pub cmd_duty_q15: i16,
    pub meta: Value,
}

/// Keep every burst a step took, in order, a line each.
pub(crate) fn keep_all(
    sink: &mut dyn Sink,
    step: &str,
    taken: &[Taken],
    lines: &mut Vec<String>,
) -> Result<(), SinkError> {
    for t in taken {
        let (csv, _) = meta::keep(sink, step, t.nn, &t.burst, t.cmd_duty_q15, &t.meta)?;
        lines.push(format!("kept {csv}"));
    }
    Ok(())
}

/// One burst read to LAST and kept in `taken` whole or not, and the
/// `fault_flags` read after it.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn take_read<P: Pipe>(
    rig: &mut Rig<'_, P>,
    sc: &StreamCfg,
    taken: &mut Vec<Taken>,
    layout: Layout,
    rows: u16,
    cmd_duty_q15: i16,
    plan: &Value,
    servo: &Map<String, Value>,
) -> Result<(Burst, u8), Halt> {
    let t0 = (rig.clock)();
    let b = tel::stream(rig.c, rig.id, rig.d, sc, layout, rows).await?;
    let t1 = (rig.clock)();
    let faults = read_u8(rig, "fault_flags").await?;
    let nn = taken.len() as u32 + 1;
    let m = meta::tel(servo, &b, plan.clone(), (t0, t1));
    taken.push(Taken {
        nn,
        burst: b.clone(),
        cmd_duty_q15,
        meta: m,
    });
    Ok((b, faults))
}

/// The step aborts when a fault latched in the burst [`take_read`] last
/// kept in `taken`.
pub(crate) async fn faulted<P: Pipe>(
    rig: &mut Rig<'_, P>,
    taken: &[Taken],
    b: &Burst,
    faults: u8,
) -> Result<(), Halt> {
    if faults == 0 && !b.alert {
        return Ok(());
    }
    let code = read_u8(rig, "fault_code").await.unwrap_or(0);
    Err(Halt::Error(format!(
        "a fault latched during the {} burst (capture {:02}): fault_flags {faults:#04x}, fault_code {code}",
        b.layout.name(),
        taken.len()
    )))
}

/// One burst read to LAST, kept in `taken` whole or not. A fault latched
/// in it aborts the step; the reasons it is not whole, if any, come back
/// with it, for the caller to repeat or abort on.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn take_once<P: Pipe>(
    rig: &mut Rig<'_, P>,
    sc: &StreamCfg,
    taken: &mut Vec<Taken>,
    layout: Layout,
    rows: u16,
    cmd_duty_q15: i16,
    plan: &Value,
    servo: &Map<String, Value>,
) -> Result<(Burst, Vec<String>), Halt> {
    let (b, faults) = take_read(rig, sc, taken, layout, rows, cmd_duty_q15, plan, servo).await?;
    faulted(rig, taken, &b, faults).await?;
    let lost = b.lost();
    Ok((b, lost))
}

/// One rung's burst, read to LAST. A fault latched in it aborts the step;
/// a burst that lost a row is no evidence: it is said so, the rung taken
/// once more, and a second such burst aborts the step. Every burst goes
/// into `taken`, the lost ones too.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn take<P: Pipe>(
    rig: &mut Rig<'_, P>,
    sc: &StreamCfg,
    taken: &mut Vec<Taken>,
    layout: Layout,
    rows: u16,
    cmd_duty_q15: i16,
    plan: &Value,
    servo: &Map<String, Value>,
    lines: &mut Vec<String>,
) -> Result<Burst, Halt> {
    let mut attempt = 0;
    loop {
        attempt += 1;
        let (b, lost) = take_once(rig, sc, taken, layout, rows, cmd_duty_q15, plan, servo).await?;
        let nn = taken.len();
        if lost.is_empty() {
            lines.push(format!(
                "{} row: {} rows, whole (capture {nn:02})",
                layout.name(),
                b.rows.len()
            ));
            return Ok(b);
        }
        lines.push(format!(
            "{} row, attempt {attempt}: lost rows ({}), no evidence (capture {nn:02})",
            layout.name(),
            lost.join(", ")
        ));
        if attempt == 2 {
            return Err(Halt::Error(format!(
                "two {} bursts in a row lost rows ({}): the line is not clean enough; check the cable and its contacts",
                layout.name(),
                lost.join(", ")
            )));
        }
        interrupted(rig.stop)?;
    }
}

pub(crate) async fn read_u8<P: Pipe>(rig: &mut Rig<'_, P>, name: &str) -> Result<u8, Halt> {
    let f = field(rig.d, name)?;
    Ok(rig
        .c
        .read(rig.id, f.addr, 1)
        .await?
        .first()
        .copied()
        .unwrap_or(0))
}

/// The longest silence a wait with torque on allows itself, s: well
/// inside the dead-man every step arms.
const KEEP_ALIVE_S: f64 = 0.2;

/// Wait `s` with torque on: a read every [`KEEP_ALIVE_S`] reloads the bus
/// dead-man, which any instruction to the servo does.
pub(crate) async fn keep_alive<P: Pipe>(rig: &mut Rig<'_, P>, s: f64) -> Result<(), Halt> {
    let mut left = s;
    while left > 0.0 {
        let step = left.min(KEEP_ALIVE_S);
        rig.c.pause(Duration::from_secs_f64(step)).await;
        read_u16(rig, "pos").await?;
        left -= step;
    }
    Ok(())
}

pub(crate) async fn read_u16<P: Pipe>(rig: &mut Rig<'_, P>, name: &str) -> Result<u16, Halt> {
    let f = field(rig.d, name)?;
    let b = rig.c.read(rig.id, f.addr, 2).await?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

/// The mean of `n` reads of a u16 register.
pub(crate) async fn mean_u16<P: Pipe>(
    rig: &mut Rig<'_, P>,
    name: &str,
    n: u32,
) -> Result<f64, Halt> {
    let mut sum = 0.0;
    for _ in 0..n {
        sum += read_u16(rig, name).await? as f64;
    }
    Ok(sum / n.max(1) as f64)
}

/// A mode's value by its name in the descriptor.
pub(crate) fn mode_value(d: &Descriptor, name: &str) -> Result<u8, Halt> {
    field(d, "mode")?
        .variants
        .iter()
        .find(|v| v.name == name)
        .map(|v| v.value)
        .ok_or_else(|| Halt::Refused(format!("the servo's mode names no {name}")))
}
