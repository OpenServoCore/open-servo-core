//! The measurement steps. A step runs on a [`Rig`]: the bus, the files,
//! the interrupt flag, the host's clock. It refuses with nothing written
//! when it cannot start, and once it has written it always ends through
//! [`crate::guard::exit`].

pub mod cruise;
pub mod drive;
pub mod polarity;
pub mod probe;
pub mod rest;
pub mod runway;
pub mod seek;
pub mod shunt;
pub mod spins;
pub mod stops;
pub mod table;
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
use crate::capture::rows::{Layout, TelFrame};
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
    /// Told each unit of the step's progress as it begins, only with the
    /// shaft at rest: it reads no clock, sends nothing and writes nothing.
    pub tell: &'a dyn Fn(Unit),
}

/// A unit of a step's progress: unit `at` of `of` begins. `at <= of`, and
/// `of` never falls within a step; a retake tells its unit again.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Unit {
    pub at: u32,
    pub of: u32,
    pub what: &'static str,
}

/// A rig's `tell` that tells no one.
pub fn untold(_: Unit) {}

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

/// The bound of a reading the tool reports and the user judges.
pub const REPORTED: &str = "reported";

impl Check {
    /// A reading said with its numbers and no range.
    pub fn reported(name: &str, value: String) -> Self {
        Check {
            name: name.into(),
            value,
            bound: REPORTED.into(),
            verdict: Verdict::NotJudged,
        }
    }

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

/// What a step's estimate stands on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Scatter {
    /// One sd over `n` repeats.
    Sd(f64, usize),
    /// `n` readings, no spread over repeats: one hold, one fit.
    Once(usize),
    /// No reading: a part's or a design value.
    Nominal,
}

impl Scatter {
    /// The sd over `n` repeats where it is a number, else `n` readings.
    pub fn of(sd: f64, n: usize) -> Self {
        if sd.is_finite() && n > 1 {
            Scatter::Sd(sd, n)
        } else {
            Scatter::Once(n)
        }
    }
}

/// A step's estimate: its value, its sd and repeat count, the band one sd
/// either side as the notebooks' `results.mean_sd` writes it, its unit and
/// source. `n` null is a nominal value.
pub fn estimate(value: f64, s: Scatter, unit: &str, source: &str) -> Value {
    let (sd, n) = match s {
        Scatter::Sd(sd, n) => (Some(sd), Some(n)),
        Scatter::Once(n) => (None, Some(n)),
        Scatter::Nominal => (None, None),
    };
    json!({"value": value, "sd": sd, "n": n, "lo": sd.map(|sd| value - sd),
           "hi": sd.map(|sd| value + sd), "unit": unit, "source": source})
}

/// A driven step's result with the worst level 4 successive RAIL-period
/// pot steps all pass over its drive bursts `driven`, raw counts, the
/// samples inside the run's pot holes `holes` left out, as
/// `estimates.pot_step4_counts` ([`crate::screens::step_run`]); nothing
/// where no burst holds a run of 4.
pub(crate) fn with_pot_step4(
    result: &mut Value,
    driven: &[&[TelFrame]],
    holes: &crate::screens::Holes,
    what: &str,
) {
    let Some(level) = crate::screens::step_run(driven, crate::screens::SCREEN_RUN, holes) else {
        return;
    };
    let src = format!(
        "the worst level {} successive RAIL-period pot steps all pass, every phase of the MEDIUM period, over the {} drive bursts of {what}; no run spans a hole in the ticks; samples inside a pot hole left out: {}",
        crate::screens::SCREEN_RUN,
        driven.len(),
        holes.source
    );
    result["estimates"][crate::gather::POT_STEP4] = estimate(
        level as f64,
        Scatter::Once(driven.len()),
        "raw counts",
        &src,
    );
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
    let b = rig.c.read(rig.id, f.addr, 1).await?;
    b.first().copied().ok_or_else(|| short(name, 1, b.len()))
}

/// A reply shorter than the field: no value, the step ends through the
/// guard rather than read a fault or a word that is not there.
fn short(name: &str, want: usize, got: usize) -> Halt {
    Halt::Error(format!(
        "the read of {name} came back with {got} of its {want} bytes"
    ))
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
    match b[..] {
        [lo, hi, ..] => Ok(u16::from_le_bytes([lo, hi])),
        _ => Err(short(name, 2, b.len())),
    }
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

#[cfg(test)]
mod tests {
    use super::{Scatter, TelFrame, estimate, with_pot_step4};
    use serde_json::json;

    /// A driven step's result carries its worst 4-step pot run as an
    /// estimate over its drive bursts, its own `estimates` block or a new
    /// one; a step with no run of 4 steps says nothing.
    #[test]
    fn a_driven_step_writes_its_worst_four_step_pot_run() {
        let burst: Vec<TelFrame> = (0..60u64)
            .map(|t| TelFrame {
                tick: t,
                pos: Some(2048 + if (t / 10) % 2 == 0 { 0 } else { 15 }),
                ..TelFrame::default()
            })
            .collect();
        let mut with = json!({"estimates": {"ke": 1}});
        let none = crate::screens::Holes::none("no table result given");
        with_pot_step4(&mut with, &[&burst, &burst[..20]], &none, "the spins");
        let e = &with["estimates"]["pot_step4_counts"];
        assert_eq!(
            (e["value"].as_f64(), e["n"].as_u64()),
            (Some(15.0), Some(2))
        );
        assert_eq!(e["unit"], "raw counts");
        assert!(
            e["source"]
                .as_str()
                .unwrap()
                .contains("over the 2 drive bursts of the spins")
        );
        assert_eq!(with["estimates"]["ke"], 1);
        let mut without = json!({"verdict": "accepted"});
        with_pot_step4(&mut without, &[&burst], &none, "the table's rungs");
        assert_eq!(without["estimates"]["pot_step4_counts"]["value"], 15.0);
        let mut short = json!({"verdict": "accepted"});
        with_pot_step4(&mut short, &[&burst[..30]], &none, "a short burst");
        assert_eq!(short, json!({"verdict": "accepted"}));
        // a burst inside a pot hole reads nothing
        let hole =
            crate::screens::Holes::new(vec![(2000.0, 2100.0)], "a hole".into()).expect("hole");
        let mut holed = json!({"verdict": "accepted"});
        with_pot_step4(&mut holed, &[&burst], &hole, "a burst in a hole");
        assert_eq!(holed, json!({"verdict": "accepted"}));
    }

    #[test]
    fn an_estimate_says_its_sd_and_n_or_that_it_is_nominal() {
        assert_eq!(
            estimate(2.0, Scatter::of(0.5, 4), "V", "s"),
            json!({"value": 2.0, "sd": 0.5, "n": 4, "lo": 1.5, "hi": 2.5, "unit": "V", "source": "s"})
        );
        assert_eq!(Scatter::of(f64::NAN, 3), Scatter::Once(3));
        assert_eq!(Scatter::of(0.5, 1), Scatter::Once(1));
        assert_eq!(
            estimate(2.0, Scatter::Once(1), "V", "s"),
            json!({"value": 2.0, "sd": null, "n": 1, "lo": null, "hi": null, "unit": "V", "source": "s"})
        );
        assert_eq!(
            estimate(2.0, Scatter::Nominal, "V", "s")["n"],
            serde_json::Value::Null
        );
    }
}

#[cfg(test)]
mod units {
    use super::*;

    /// Each step's planned units, beside its constants: the counts its
    /// progress is told over.
    #[test]
    fn each_step_plans_its_units() {
        assert_eq!(rest::UNITS, 2);
        assert_eq!(probe::units(probe::shots(5.0, 5.0).len()), 17);
        assert_eq!(stops::UNITS, 6);
        let plan = table::Schedule::PLAN;
        assert_eq!((plan.passes(), plan.passes_max()), (9, 13));
        assert_eq!(
            (window::units(1), window::units(window::SETS_MAX)),
            (24, 48)
        );
        assert_eq!(spins::units(), 15);
        assert_eq!(cruise::UNITS, 4);
        assert_eq!(verify::units(6, 8), 20);
    }
}
