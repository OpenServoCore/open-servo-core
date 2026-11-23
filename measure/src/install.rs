//! The safe configuration path, async over the client: `check` reads the
//! servo against a set, `install` makes it equal the set volatile and
//! stamps it, `save` keeps a backup of the stored state and then makes the
//! set permanent, `restore` puts a recorded state back. Nothing drives the
//! motor, every write is refused under torque, and there is no path that
//! wipes the stored data: a servo whose saved settings are corrupt is
//! refused.

use std::fmt;
use std::time::Duration;

use osc_client::data_state::{CONFIG_CORRUPT, PLANT_UNSET, STAMP_MISMATCH};
use osc_client::descriptor::{Descriptor, Field, Kind};
use osc_client::pos_lut::{self, INTERVALS};
use osc_client::{Client, Error, Id, mgmt, pipe::Pipe, stamp};
use serde_json::Value;

use crate::pyjson::dumps;
use crate::set::{self, Looser, NEVER, SetError, SetFile, WIDE, bytes_of, points_sha1};
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
        }
    }
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
    pub stamp: String,
}

impl Checked {
    pub fn clean(&self) -> bool {
        self.differences.is_empty() && self.data == "clean"
    }
}

fn int_of(f: &Field, b: &[u8]) -> i64 {
    let raw = b
        .iter()
        .enumerate()
        .fold(0u64, |v, (i, &x)| v | (x as u64) << (8 * i));
    let shift = 64 - 8 * (b.len().clamp(1, 8) as u32);
    match f.kind {
        Kind::Int => ((raw << shift) as i64) >> shift,
        _ => raw as i64,
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
        stamp: state.stamp_said(),
    })
}

/// Refused with nothing written: torque on, or saved settings no write
/// verb may touch.
fn writable_now(state: &State) -> Result<(), Refused> {
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

/// The protection gate is in here: a protection field the set would move
/// to protect less is written only when `loosen` names it.
fn plan<'d>(
    set: &SetFile,
    state: &'d State,
    d: &'d Descriptor,
    points: Option<&[i16; INTERVALS]>,
    loosen: &[String],
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
        if let Some(way) = set::looser(&f.name) {
            let (from, to) = (int_of(f, have), int_of(f, &b));
            let less = match way {
                Looser::Judged => true,
                Looser::Up => to > from,
                Looser::Down => to < from,
            };
            if less && !loosen.contains(&f.name) {
                let how = match way {
                    Looser::Judged => "judged, taken as protecting less".to_string(),
                    w => format!("protects less (less is {})", w.word()),
                };
                no.push(format!("{said}, {how}: name it with --loosen {}", f.name));
                continue;
            }
        }
        if STOPS.contains(&f.name.as_str()) {
            p.stops.push((f, b));
        } else {
            p.fields.push((f, b));
        }
    }
    for n in loosen {
        if set::looser(n).is_none() || set.value(n).is_none() {
            no.push(format!("{n} is not a protection field of the set"));
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

/// One field written and read back. `Ok(false)`: the servo refused the
/// value, which may land once the rest is in.
async fn put<P: Pipe>(c: &mut Client<P>, id: Id, f: &Field, bytes: &[u8]) -> Result<bool, Refused> {
    match c.write(id, f.addr, bytes).await {
        Ok(()) => {}
        Err(Error::Servo(_)) => return Ok(false),
        Err(e) => return Err(e.into()),
    }
    let back = c.read(id, f.addr, f.width).await?;
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
    let mut pending = p.fields;
    for last in [false, true] {
        let mut again = Vec::new();
        for (f, b) in pending {
            *written = true;
            if put(c, id, f, &b).await? {
                steps.push(field_step(f, &b));
            } else if last {
                return Err(Refused(format!(
                    "{}: the servo refused {}",
                    f.name,
                    dumps(&set::text_of(f, &b))
                )));
            } else {
                again.push((f, b));
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

pub(crate) async fn reboot<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    steps: &mut Vec<Step>,
) -> Result<(), Error> {
    mgmt::reboot(c, id).await?;
    await_boot(c, id).await?;
    steps.push(Step::Rebooted);
    Ok(())
}

/// The refusal of a verb that had written: the servo is rebooted onto its
/// saved state.
async fn rebooted<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    why: Refused,
    steps: &mut Vec<Step>,
) -> Refused {
    match reboot(c, id, steps).await {
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
/// the servo lets a host write and this tool writes, the table can be
/// written, and the protection gate passes. Then the slots, the stops, the
/// table, the other fields that differ in table order, each read back, a
/// field the servo refuses tried once more after the rest; the whole state
/// read back must be the set before it is stamped. A refusal after the
/// first write reboots the servo onto its saved state. `points` are the
/// table's when the set's is LIVE.
pub async fn install<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    loosen: &[String],
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let state = state::read(c, id, d).await?;
    let p = plan(set, &state, d, points, loosen)?;
    let mut written = false;
    match apply(c, id, d, set, p, steps, &mut written).await {
        Ok(()) => Ok(()),
        Err(e) if !written => Err(e),
        Err(e) => Err(rebooted(c, id, e, steps).await),
    }
}

/// Persist the set. Refused with the bus untouched for a derived set,
/// then unless torque is off. The servo is rebooted onto its stored state,
/// which is written to [`STORED`] and must read back the same before
/// anything else is written; then the install, its gate as for any, the
/// check, MGMT SAVE (protocol sec 9.4), a reboot, and the state it boots
/// with to [`AFTER`], which must be the set.
#[allow(clippy::too_many_arguments)]
pub async fn save<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    set: &SetFile,
    points: Option<&[i16; INTERVALS]>,
    loosen: &[String],
    sink: &mut dyn Sink,
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    if let Some(dev) = set.deviation() {
        return Err(Refused(format!(
            "the set is derived ({dev} moves it): a derived set is never saved"
        )));
    }
    writable_now(&state::read(c, id, d).await?)?;
    reboot(c, id, steps).await?;
    let stored = state::read(c, id, d).await?;
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
    let put_back = format!(
        "`osc-ident restore --from {}` installs it again, volatile",
        sink.shown(STORED)
    );
    if let Err(e) = install(c, id, d, set, points, loosen, steps).await {
        return Err(Refused(format!(
            "{e}\nnothing was saved: {} holds the state the servo has stored; {put_back}",
            sink.shown(STORED)
        )));
    }
    let checked = check(c, id, d, set).await?;
    if !checked.clean() {
        let why = Refused(format!(
            "after the install the servo is not the set (data state {}): {}",
            checked.data,
            checked.differences.join("; ")
        ));
        return Err(rebooted(c, id, why, steps).await);
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
/// the table from its own points, behind the same gate.
pub async fn restore<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    record: &str,
    origin: Value,
    loosen: &[String],
    steps: &mut Vec<Step>,
) -> Result<(), Refused> {
    let r = Record::parse(record)?;
    let set = r.adopted(origin)?;
    install(c, id, d, &set, Some(&r.points), loosen, steps).await
}
