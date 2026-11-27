//! The safe configuration path, async over the client: `check` reads the
//! servo against a set, `install` makes it equal the set volatile and
//! stamps it, `save` keeps a backup of the stored state and then makes the
//! set permanent, `restore` puts a recorded state back. Nothing drives the
//! motor, every write is refused under torque, and there is no path that
//! wipes the stored data: a servo whose saved settings are corrupt is
//! refused.

use std::fmt;
use std::time::Duration;

use osc_client::data_state::{
    self as data_state, CONFIG_CORRUPT, PLANT_UNSET, SAVE_CLEARS, STAMP_MISMATCH,
};
use osc_client::descriptor::{Descriptor, Field};
use osc_client::pos_lut::{self, INTERVALS};
use osc_client::{BaudRate, Client, Error, Id, mgmt, pipe::Pipe, stamp};
use serde_json::Value;

use crate::pyjson::dumps;
use crate::set::{self, NEVER, SetError, SetFile, WIDE, bytes_of, points_sha1};
use crate::sink::{Sink, SinkError};
use crate::state::{self, Record, State, state_hash};

/// Where a save keeps the state the servo had stored, and the state it
/// boots with after the SAVE.
pub const STORED: &str = "backup/stored.json";
pub const AFTER: &str = "backup/after.json";
/// The stops a table is taken against: written ahead of it.
const STOPS: [&str; 2] = ["raw_min", "raw_max"];
const BOOT_WAIT: Duration = Duration::from_millis(500);
const BOOT_TRIES: u32 = 10;
const SWEEP_QUIET: Duration = Duration::from_secs(5);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refused(pub String);

impl fmt::Display for Refused {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Refused {}

impl From<Error> for Refused {
    fn from(e: Error) -> Self {
        Refused(format!("bus: {e}"))
    }
}

impl From<SetError> for Refused {
    fn from(e: SetError) -> Self {
        Refused(e.0)
    }
}

impl From<SinkError> for Refused {
    fn from(e: SinkError) -> Self {
        Refused(e.0)
    }
}

/// What a verb did on the servo and in its files, in order.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    Slots {
        from: String,
        to: String,
    },
    Field {
        name: String,
        from: Value,
        to: Value,
    },
    Table {
        from: String,
        to: String,
    },
    Stamped(u16),
    Backup(String),
    Saved,
    After(String),
    Rebooted,
    /// After a reboot the servo answered at `at`, its saved baud, and was
    /// moved back to the run's rate `run`, volatile.
    Moved {
        at: BaudRate,
        run: BaudRate,
    },
}

impl fmt::Display for Step {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Step::Slots { from, to } => write!(f, "wide-code slots: {from} -> {to}"),
            Step::Field { name, from, to } => write!(f, "{name}: {} -> {}", dumps(from), dumps(to)),
            Step::Table { from, to } => {
                write!(f, "position table: {from} -> {to}, every page read back")
            }
            Step::Stamped(s) => write!(
                f,
                "stamped {s:#06x}: the data state reads no stamp mismatch"
            ),
            Step::Backup(p) => write!(
                f,
                "the stored state is backed up to {p}, read back the same"
            ),
            Step::Saved => write!(f, "SAVE: CONFIG and CALIB are in flash"),
            Step::After(p) => write!(f, "the state after the SAVE is in {p}"),
            Step::Rebooted => write!(f, "rebooted: the servo runs its saved state"),
            Step::Moved { at, run } => write!(
                f,
                "after the reboot the servo answers at {}, its saved baud; moved it to {}, volatile",
                mbaud(*at),
                mbaud(*run)
            ),
        }
    }
}

/// The reason names set in `flags`, joined.
fn names(flags: u8) -> String {
    data_state::DataState {
        flags,
        fault_code: data_state::fault::NONE,
    }
    .names()
}

/// The lines of the moves back to the run's rate among `steps`.
pub fn moves(steps: &[Step]) -> impl Iterator<Item = String> + '_ {
    steps
        .iter()
        .filter(|s| matches!(s, Step::Moved { .. }))
        .map(ToString::to_string)
}

fn spaced(b: &[u8]) -> String {
    b.iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// The servo against a set.
#[derive(Clone, Debug)]
pub struct Checked {
    /// A line a difference: the set's fields by name, the table, the slots.
    pub differences: Vec<String>,
    /// The fields a host writes that the set does not hold, each with the
    /// servo's value.
    pub unowned: Vec<String>,
    pub data: String,
    pub flags: u8,
    pub stamp: String,
}

impl Checked {
    pub fn clean(&self) -> bool {
        self.differences.is_empty() && self.flags == 0
    }

    /// Why a SAVE of the servo as checked is refused: the set has not
    /// landed, or its data state holds a reason the SAVE does not retire.
    /// The reasons it retires are no refusal: on a servo never saved they
    /// stand until that SAVE.
    pub fn unsaveable(&self) -> Option<String> {
        let left = self.flags & !SAVE_CLEARS;
        match (self.differences.is_empty(), left) {
            (true, 0) => None,
            (true, _) => Some(format!(
                "after the install the servo holds the set, but its data state reads {} ({} of it a SAVE does not clear)",
                self.data,
                names(left)
            )),
            (false, _) => Some(format!(
                "after the install the servo is not the set (data state {}): {}",
                self.data,
                self.differences.join("; ")
            )),
        }
    }
}

/// Where `state` is not the set, a line each.
pub fn differences(set: &SetFile, state: &State, d: &Descriptor) -> Vec<String> {
    let mut out = Vec::new();
    for (name, v) in set.values() {
        let Some(f) = d.field(name) else {
            out.push(format!(
                "{name}: the servo names no such field, set {}",
                dumps(v)
            ));
            continue;
        };
        match (bytes_of(f, v), state.bytes(f)) {
            (Ok(want), Some(have)) if want == have => {}
            (Ok(_), _) => out.push(format!(
                "{name}: servo {}, set {}",
                dumps(&state.text(f)),
                dumps(v)
            )),
            (Err(e), _) => out.push(e.0),
        }
    }
    let (want_state, want_sha) = set.table();
    if state.table_state() != want_state {
        out.push(format!(
            "position table state: servo {}, set {want_state}",
            state.table_state()
        ));
    }
    if state.points_sha1() != want_sha {
        out.push(format!(
            "position table points sha1: servo {}, set {want_sha}",
            state.points_sha1()
        ));
    }
    match (set.slots(), state.slots(d)) {
        (Ok(None), _) => {}
        (Ok(Some(want)), Some(have)) if want == have => {}
        (Ok(Some(want)), have) => out.push(format!(
            "wide-code slots: servo {}, set {}",
            have.map_or("names none".into(), |h| spaced(&h)),
            spaced(&want)
        )),
        (Err(e), _) => out.push(e.0),
    }
    out
}

fn unowned(set: &SetFile, state: &State, d: &Descriptor) -> Vec<String> {
    let rec = state.record(d);
    let writable = rec["writable"].as_array().cloned().unwrap_or_default();
    writable
        .iter()
        .filter_map(Value::as_str)
        .filter(|n| {
            set.value(n).is_none() && !(*n == WIDE && set.slots().is_ok_and(|s| s.is_some()))
        })
        .map(|n| format!("{n} = {}", dumps(&rec["fields"][n])))
        .collect()
}

pub async fn check<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
) -> Result<Checked, Refused> {
    let state = state::read(c, id, d).await?;
    Ok(Checked {
        differences: differences(set, &state, d),
        unowned: unowned(set, &state, d),
        data: state.data_said(),
        flags: state.data.flags,
        stamp: state.stamp_said(),
    })
}

/// Refused with nothing written: torque on, or saved settings no write
/// verb may touch.
pub fn writable_now(state: &State) -> Result<(), Refused> {
    if state.torque {
        return Err(Refused(
            "torque_enable is on: this tool writes a servo only with torque off".into(),
        ));
    }
    if state.data.flags & CONFIG_CORRUPT != 0 {
        return Err(Refused(
            "the servo's saved settings are unreadable (CONFIG_CORRUPT): `osc recover` is the way back, this tool refuses such a servo".into(),
        ));
    }
    Ok(())
}

/// What an install writes, every refusal found before the first write.
struct Plan<'d> {
    /// The state the plan was made against.
    state: &'d State,
    slots: Option<Vec<u8>>,
    stops: Vec<(&'d Field, Vec<u8>)>,
    table: Option<[i16; INTERVALS]>,
    fields: Vec<(&'d Field, Vec<u8>)>,
}

fn plan<'d>(
    set: &SetFile,
    state: &'d State,
    d: &'d Descriptor,
    points: Option<&[i16; INTERVALS]>,
) -> Result<Plan<'d>, Refused> {
    writable_now(state)?;
    let mut no = Vec::new();
    let mut p = Plan {
        state,
        slots: None,
        stops: Vec::new(),
        table: None,
        fields: Vec::new(),
    };
    let mut want: Vec<(&'d Field, Vec<u8>)> = Vec::new();
    for (name, v) in set.values() {
        let Some(f) = d
            .fields
            .iter()
            .find(|f| f.name == name && f.access == osc_client::descriptor::Access::Rw)
        else {
            no.push(format!(
                "the servo's descriptor lets a host write no {name}"
            ));
            continue;
        };
        match bytes_of(f, v) {
            Ok(b) => want.push((f, b)),
            Err(e) => no.push(e.0),
        }
    }
    want.sort_by_key(|(f, _)| f.addr);
    for (f, b) in want {
        let Some(have) = state.bytes(f) else {
            no.push(format!("{}: the state read holds no such field", f.name));
            continue;
        };
        if have == b.as_slice() {
            continue;
        }
        let said = format!(
            "{}: servo {}, set {}",
            f.name,
            dumps(&state.text(f)),
            dumps(&set::text_of(f, &b))
        );
        if NEVER.contains(&f.name.as_str()) {
            no.push(format!("{said}: this tool never writes it"));
            continue;
        }
        if STOPS.contains(&f.name.as_str()) {
            p.stops.push((f, b));
        } else {
            p.fields.push((f, b));
        }
    }
    match set.slots() {
        Ok(Some(slots)) => match (d.field(WIDE), state.slots(d)) {
            (Some(f), Some(have)) if f.width as usize == slots.len() => {
                if have != slots {
                    p.slots = Some(slots);
                }
            }
            _ => no.push(format!(
                "the servo's descriptor holds no {WIDE} of the set's slots"
            )),
        },
        Ok(None) => {}
        Err(e) => no.push(e.0),
    }
    let (want_state, want_sha) = set.table();
    if (state.table_state(), state.points_sha1().as_str()) != (want_state, want_sha) {
        let table = match (want_state, points) {
            ("IDENTITY", _) => Some([0i16; INTERVALS]),
            ("LIVE", Some(k)) => Some(*k),
            ("LIVE", None) => {
                no.push("the set's table is LIVE and no table points were given".into());
                None
            }
            (s, _) => {
                no.push(format!(
                    "the set's table is {s}: an install writes a LIVE or an IDENTITY table only"
                ));
                None
            }
        };
        if let Some(k) = table {
            if points_sha1(&k) == want_sha {
                p.table = Some(k);
            } else {
                no.push(format!(
                    "the table at hand holds points of sha1 {}, the set's table is {want_sha}",
                    points_sha1(&k)
                ));
            }
        }
    }
    if no.is_empty() {
        Ok(p)
    } else {
        Err(Refused(format!(
            "refused before any write:\n  {}",
            no.join("\n  ")
        )))
    }
}

/// The refusals an install of `set` over `state` makes before its first
/// write, without writing.
pub fn gate(
    set: &SetFile,
    state: &State,
    d: &Descriptor,
    points: Option<&[i16; INTERVALS]>,
) -> Result<(), Refused> {
    plan(set, state, d, points).map(|_| ())
}

/// One field written and read back. `Ok(false)`: the servo refused the
/// value, which may land once the rest is in.
async fn put<P: Pipe>(c: &mut Client<P>, id: Id, f: &Field, bytes: &[u8]) -> Result<bool, Refused> {
    match c.write(id, f.addr, bytes).await {
        Ok(()) => {}
        Err(Error::Servo(_)) => return Ok(false),
        Err(e) => return Err(e.into()),
    }
    let back = c.read(id, f.addr, bytes.len() as u16).await?;
    if back != bytes {
        return Err(Refused(format!(
            "{}: wrote {}, read back {}",
            f.name,
            spaced(bytes),
            spaced(&back)
        )));
    }
    Ok(true)
}

/// Fields the servo checks side against side as stored, so a change to
/// both goes in one write: the soft limits, whose 0 / 0, off, no side
/// reaches alone (protocol sec 5.8).
const PAIRS: [[&str; 2]; 1] = [["pos_min_soft_counts", "pos_max_soft_counts"]];

/// The plan's fields, in address order, as writes: both sides of a pair
/// of [`PAIRS`] one write, every other field its own.
fn writes(fields: Vec<(&Field, Vec<u8>)>) -> Vec<Vec<(&Field, Vec<u8>)>> {
    let mut out: Vec<Vec<(&Field, Vec<u8>)>> = Vec::new();
    for (f, b) in fields {
        let joins = matches!(
            out.last().map(Vec::as_slice),
            Some([(g, _)]) if PAIRS.iter().any(|[lo, hi]| g.name == *lo && f.name == *hi)
                && g.addr + g.width == f.addr
        );
        match out.last_mut() {
            Some(w) if joins => w.push((f, b)),
            _ => out.push(vec![(f, b)]),
        }
    }
    out
}

/// A write of [`writes`], read back: [`put`] at its first field.
async fn put_run<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    run: &[(&Field, Vec<u8>)],
) -> Result<bool, Refused> {
    let Some((f, _)) = run.first() else {
        return Ok(true);
    };
    put(
        c,
        id,
        f,
        &run.iter().flat_map(|(_, b)| b.clone()).collect::<Vec<u8>>(),
    )
    .await
}

async fn apply<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    p: Plan<'_>,
    steps: &mut Vec<Step>,
    written: &mut bool,
) -> Result<(), Refused> {
    let state = p.state;
    if let Some(slots) = &p.slots {
        let f = state::field(d, WIDE)?;
        *written = true;
        if !put(c, id, f, slots).await? {
            return Err(Refused(format!(
                "the servo refused the slots {}",
                spaced(slots)
            )));
        }
        steps.push(Step::Slots {
            from: state.slots(d).map_or("none".into(), |s| spaced(&s)),
            to: spaced(slots),
        });
    }
    let field_step = |f: &Field, b: &[u8]| Step::Field {
        name: f.name.clone(),
        from: state.text(f),
        to: set::text_of(f, b),
    };
    for (f, b) in &p.stops {
        *written = true;
        if !put(c, id, f, b).await? {
            return Err(Refused(format!(
                "{}: the servo refused {}",
                f.name,
                dumps(&set::text_of(f, b))
            )));
        }
        steps.push(field_step(f, b));
    }
    if let Some(points) = &p.table {
        *written = true;
        pos_lut::write(c, id, d, points).await?;
        steps.push(Step::Table {
            from: state.table_state().into(),
            to: set.table().0.into(),
        });
    }
    let mut pending = writes(p.fields);
    for last in [false, true] {
        let mut again = Vec::new();
        for run in pending {
            *written = true;
            if put_run(c, id, &run).await? {
                steps.extend(run.iter().map(|(f, b)| field_step(f, b)));
            } else if last {
                let names: Vec<&str> = run.iter().map(|(f, _)| f.name.as_str()).collect();
                let values: Vec<String> = run
                    .iter()
                    .map(|(f, b)| dumps(&set::text_of(f, b)))
                    .collect();
                return Err(Refused(format!(
                    "{}: the servo refused {}",
                    names.join(" and "),
                    values.join(" and ")
                )));
            } else {
                again.push(run);
            }
        }
        pending = again;
    }
    let now = if *written {
        let now = state::read(c, id, d).await?;
        let left = differences(set, &now, d);
        if !left.is_empty() {
            return Err(Refused(format!(
                "the state read back is not the set: {}",
                left.join("; ")
            )));
        }
        now
    } else {
        state.clone()
    };
    if *written || !now.stamp.matches() || now.data.flags & STAMP_MISMATCH != 0 {
        *written = true;
        let s = stamp::restamp(c, id, d).await?;
        let v = stamp::verdict(c, id, d).await?;
        let data = osc_client::data_state::read(c, id, d).await?;
        if v.stored != s || !v.matches() || data.flags & (STAMP_MISMATCH | PLANT_UNSET) != 0 {
            return Err(Refused(format!(
                "the stamp {s:#06x} did not bless the set: stamp reads {:#06x} over a live set of {:#06x}, data state {}",
                v.stored,
                v.computed,
                data.names()
            )));
        }
        steps.push(Step::Stamped(s));
    }
    Ok(())
}

/// Wait for the servo to answer after a reboot.
async fn await_boot<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(), Error> {
    let mut last = Error::Timeout { slot: 0 };
    for _ in 0..BOOT_TRIES {
        c.pause(BOOT_WAIT).await;
        match c.ping(id).await {
            Ok(_) => return Ok(()),
            Err(e @ Error::Timeout { .. }) => last = e,
            Err(e) => return Err(e),
        }
    }
    Err(last)
}

fn mbaud(r: BaudRate) -> String {
    format!("{}M", r.as_hz() as f64 / 1e6)
}

/// MGMT REBOOT, and the servo found after it. A servo boots on its saved
/// baud, which a volatile move of the bus does not change: silent at the
/// run's rate, it is looked for at the others by ENUM and must be on the
/// roster there by its id. `Some` is the rate it answers at, the host left
/// there for the caller's reads ahead of [`home`].
async fn boot<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Option<BaudRate>, Refused> {
    mgmt::reboot(c, id).await?;
    let run = c.info().bps;
    let silent = match await_boot(c, id).await {
        Ok(()) => return Ok(None),
        Err(e @ Error::Timeout { .. }) => e,
        Err(e) => return Err(e.into()),
    };
    let Some(run) = BaudRate::from_hz(run) else {
        return Err(silent.into());
    };
    let pinged = format!(
        "no answer at {} after the REBOOT it acked ({BOOT_TRIES} pings over {:.0} s)",
        mbaud(run),
        (BOOT_WAIT * BOOT_TRIES).as_secs_f64()
    );
    let Some(at) = mgmt::find_bus_baud(c).await? else {
        c.host_baud(run).await?;
        return Err(Refused(format!(
            "{pinged}, and no ENUM answer at 1M, 0.5M, 2M or 3M either: the servo answers at no rate on this bus"
        )));
    };
    let roster = mgmt::discover(c).await?;
    // The first unicast right behind a discover sweep is lost about one
    // run in twenty; one sent 5 s after it was observed clean.
    c.pause(SWEEP_QUIET).await;
    if !roster.iter().any(|f| f.id == id) {
        let ids: Vec<String> = roster.iter().map(|f| f.id.as_byte().to_string()).collect();
        c.host_baud(run).await?;
        return Err(Refused(format!(
            "{pinged}; at {} the bus answers ENUM, and its roster (ids: {}) holds no servo of id {}",
            mbaud(at),
            if ids.is_empty() {
                "none".into()
            } else {
                ids.join(", ")
            },
            id.as_byte()
        )));
    }
    Ok((at != run).then_some(at))
}

/// The servo [`boot`] found at `at` moved back to the run's rate, volatile:
/// one write at `at`, never retried, and the reunion ping at the run's rate
/// is the verdict (protocol sec 8).
async fn home<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    at: Option<BaudRate>,
    run: u32,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let (Some(at), Some(run)) = (at, BaudRate::from_hz(run)) else {
        return Ok(());
    };
    let roster = mgmt::set_baud(c, &[id], run).await?;
    if roster.iter().all(|(_, alive)| *alive) {
        steps.push(Step::Moved { at, run });
        return Ok(());
    }
    Err(Refused(format!(
        "after the reboot the servo answers at {}, its saved baud; moved to {}, volatile, it does not answer there: `osc discover` finds the rate it answers at",
        mbaud(at),
        mbaud(run)
    )))
}

pub(crate) async fn reboot<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let run = c.info().bps;
    let at = boot(c, id).await?;
    steps.push(Step::Rebooted);
    home(c, id, at, run, steps).await
}

/// The refusal of a verb that had written: the servo is rebooted onto its
/// saved state, which the refusal says, so no [`Step::Rebooted`] is kept.
async fn rebooted<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    why: Refused,
    steps: &mut Vec<Step>,
) -> Refused {
    let run = c.info().bps;
    let back = match boot(c, id).await {
        Ok(at) => home(c, id, at, run, steps).await,
        Err(e) => Err(e),
    };
    match back {
        Ok(()) => Refused(format!(
            "{why}\nrebooted: the servo is back on its saved state"
        )),
        Err(e) => Refused(format!(
            "{why}\nthe reboot did not answer ({e}): the servo holds a half-written set until it is power cycled"
        )),
    }
}

/// Make the servo equal the set, volatile, then stamp it. Refused with
/// nothing written unless torque is off, every field of the set is one
/// the servo lets a host write and this tool writes, and the table can be
/// written. Then the slots, the stops, the table, the other fields that
/// differ in table order, each read back, a field the servo refuses tried
/// once more after the rest; the whole state read back must be the set
/// before it is stamped. A refusal after the first write reboots the
/// servo onto its saved state. `points` are the table's when the set's
/// is LIVE.
pub async fn install<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let state = state::read(c, id, d).await?;
    let p = plan(set, &state, d, points)?;
    let mut written = false;
    match apply(c, id, d, set, p, steps, &mut written).await {
        Ok(()) => Ok(()),
        Err(e) if !written => Err(e),
        Err(e) => Err(rebooted(c, id, e, steps).await),
    }
}

/// Reboot the servo onto its stored state and keep the record of it in
/// [`STORED`], which must read back the same before anything else is
/// written: the state a save, and a tune that ends without a set, go back
/// to. The stored state as read at the baud it boots on, before any move
/// back to the run's rate.
pub async fn backup<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    sink: &mut dyn Sink,
    steps: &mut Vec<Step>,
) -> Result<State, Refused> {
    let run = c.info().bps;
    let at = boot(c, id).await?;
    steps.push(Step::Rebooted);
    let stored = state::read(c, id, d).await;
    home(c, id, at, run, steps).await?;
    let stored = stored?;
    let mut bytes =
        serde_json::to_vec_pretty(&stored.record(d)).map_err(|e| Refused(e.to_string()))?;
    bytes.push(b'\n');
    sink.write(STORED, &bytes)?;
    let back = sink.read(STORED)?;
    let parsed: Value = serde_json::from_slice(&back).unwrap_or(Value::Null);
    if back != bytes
        || parsed.get("hash").and_then(Value::as_str) != Some(state_hash(&parsed).as_str())
    {
        return Err(Refused(format!(
            "{} does not read back as the backup written: nothing was written",
            sink.shown(STORED)
        )));
    }
    steps.push(Step::Backup(sink.shown(STORED)));
    Ok(stored)
}

/// Put the servo back on its stored state by a reboot, which writes no
/// field, then read it against `stored`, the state [`backup`] kept: a
/// line a difference, none when it is that state. The read comes before
/// any move back to the run's rate, so the volatile baud is no difference.
pub async fn put_back<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    stored: &State,
    steps: &mut Vec<Step>,
) -> Result<Vec<String>, Refused> {
    let run = c.info().bps;
    let at = boot(c, id).await?;
    steps.push(Step::Rebooted);
    let now = state::read(c, id, d).await;
    home(c, id, at, run, steps).await?;
    let now = now?;
    if now.record(d)["hash"] == stored.record(d)["hash"] {
        return Ok(Vec::new());
    }
    let mut out: Vec<String> = d
        .fields
        .iter()
        .filter(|f| now.bytes(f).is_some() && now.bytes(f) != stored.bytes(f))
        .map(|f| {
            format!(
                "{}: stored {}, now {}",
                f.name,
                dumps(&stored.text(f)),
                dumps(&now.text(f))
            )
        })
        .collect();
    if (now.table_state(), now.points_sha1()) != (stored.table_state(), stored.points_sha1()) {
        out.push(format!(
            "position table: stored {} ({}), now {} ({})",
            stored.table_state(),
            stored.points_sha1(),
            now.table_state(),
            now.points_sha1()
        ));
    }
    if now.data_said() != stored.data_said() {
        out.push(format!(
            "data state: stored {}, now {}",
            stored.data_said(),
            now.data_said()
        ));
    }
    if out.is_empty() {
        out.push(format!(
            "the state read is not the backup's: stamp stored {}, now {}",
            stored.stamp_said(),
            now.stamp_said()
        ));
    }
    Ok(out)
}

/// Persist the set. Refused with the bus untouched for a derived set,
/// then unless torque is off. The servo is rebooted onto its stored state,
/// which is written to [`STORED`] and must read back the same before
/// anything else is written; then the install, refused as any, the
/// check, MGMT SAVE (protocol sec 9.4), a reboot, and the state it boots
/// with to [`AFTER`], which must be the set.
pub async fn save<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    sink: &mut dyn Sink,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    if let Some(dev) = set.deviation() {
        return Err(Refused(format!(
            "the set is derived ({dev} moves it): a derived set is never saved"
        )));
    }
    writable_now(&state::read(c, id, d).await?)?;
    backup(c, id, d, sink, steps).await?;
    let put_back = format!(
        "`osc-ident restore --from {}` installs it again, volatile",
        sink.shown(STORED)
    );
    if let Err(e) = install(c, id, d, set, points, steps).await {
        return Err(Refused(format!(
            "{e}\nnothing was saved: {} holds the state the servo has stored; {put_back}",
            sink.shown(STORED)
        )));
    }
    if let Some(why) = check(c, id, d, set).await?.unsaveable() {
        return Err(rebooted(c, id, Refused(why), steps).await);
    }
    if let Err(e) = mgmt::save(c, id).await {
        let why = Refused(format!(
            "the servo refused the SAVE ({e}): nothing was saved"
        ));
        return Err(rebooted(c, id, why, steps).await);
    }
    steps.push(Step::Saved);
    reboot(c, id, steps).await?;
    let after = state::read(c, id, d).await?;
    let mut bytes =
        serde_json::to_vec_pretty(&after.record(d)).map_err(|e| Refused(e.to_string()))?;
    bytes.push(b'\n');
    sink.write(AFTER, &bytes)?;
    steps.push(Step::After(sink.shown(AFTER)));
    let left = differences(set, &after, d);
    if !left.is_empty() || after.data.flags != 0 {
        return Err(Refused(format!(
            "after the SAVE and a reboot the servo is not the set (data state {}{}): flash holds the state {} records, no longer the old one. {put_back}; `osc save` then makes it permanent",
            after.data_said(),
            left.iter().map(|l| format!("; {l}")).collect::<String>(),
            sink.shown(AFTER)
        )));
    }
    Ok(())
}

/// Put back, volatile, the state a record holds (a save's [`STORED`], or
/// the notebooks' `servo.json`): the install of the set adopted from it,
/// the table from its own points, refused as any install.
pub async fn restore<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    record: &str,
    origin: Value,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let r = Record::parse(record)?;
    let set = r.adopted(origin)?;
    install(c, id, d, &set, Some(&r.points), steps).await
}
