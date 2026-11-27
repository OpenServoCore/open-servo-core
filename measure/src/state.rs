//! The servo's constant state as it holds it now, and the record of it a
//! save keeps before anything is written: the notebooks' state record
//! (`scorecard.servo_state`), so `oscnb.constants restore` reads it too.

use osc_client::descriptor::{Access, Descriptor, Field};
use osc_client::pos_lut::{self, INTERVALS, PosLut};
use osc_client::store_state::{self, StoreState};
use osc_client::{Client, Error, Id, pipe::Pipe};
use serde_json::{Map, Value, json};

use crate::pyjson::{self, hex};
use crate::set::{self, COMMS, NEVER, SetError, SetFile, WIDE, points_sha1, text_of};

/// The first field of CONTROL: CONFIG and CALIB, the regions a SAVE keeps,
/// are the table from its first field up to this one.
const CONTROL_FIRST: &str = "torque_enable";
const ORIGIN: &str = "servo-state";
/// The register a record read on CALIB layout 7 holds and no servo has
/// since: a restore never writes it.
const OLD_STAMP: &str = "plant_stamp";
/// Registers a record names as the servo it was read on did, by today's
/// names: the bytes are the same.
const RENAMED: [(&str, &str); 1] = [("i_cont_counts", "yield_current_counts")];

pub(crate) fn field<'a>(d: &'a Descriptor, name: &str) -> Result<&'a Field, Error> {
    d.field(name)
        .ok_or_else(|| Error::Descriptor(format!("no {name} in {}", d.model)))
}

#[derive(Clone, Debug)]
pub struct State {
    /// CONFIG and CALIB from `base`.
    pub base: u16,
    pub raw: Vec<u8>,
    pub lut: PosLut,
    /// The store bits, the unsaved bit and the refused modes.
    pub store: StoreState,
    /// `status_flags` UNSAVED: what the servo runs differs from what a
    /// reboot loads.
    pub unsaved: bool,
    pub torque: bool,
}

pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<State, Error> {
    let end = field(d, CONTROL_FIRST)?.addr;
    let base = d
        .fields
        .iter()
        .map(|f| f.addr)
        .filter(|&a| a < end)
        .min()
        .unwrap_or(end);
    let raw = c.read_span(id, base, end).await?;
    let lut = pos_lut::read(c, id, d).await?;
    let store = store_state::read(c, id, d).await?;
    let unsaved = store.unsaved();
    let torque = field(d, CONTROL_FIRST)?;
    let torque = c
        .read(id, torque.addr, 1)
        .await?
        .first()
        .is_some_and(|&b| b != 0);
    Ok(State {
        base,
        raw,
        lut,
        store,
        unsaved,
        torque,
    })
}

impl State {
    pub fn bytes(&self, f: &Field) -> Option<&[u8]> {
        let lo = (f.addr as usize).checked_sub(self.base as usize)?;
        self.raw.get(lo..lo + f.width as usize)
    }

    pub fn text(&self, f: &Field) -> Value {
        self.bytes(f).map_or(Value::Null, |b| text_of(f, b))
    }

    pub fn slots(&self, d: &Descriptor) -> Option<Vec<u8>> {
        d.field(WIDE)
            .and_then(|f| self.bytes(f))
            .map(<[u8]>::to_vec)
    }

    pub fn table_state(&self) -> &'static str {
        self.lut.state_name().unwrap_or("UNKNOWN")
    }

    pub fn points_sha1(&self) -> String {
        points_sha1(&self.lut.points)
    }

    /// The settings word as `osc-bus status` says it: `loaded`, or
    /// `defaults: CONFIG, CALIB`.
    pub fn settings_said(&self) -> String {
        self.store.settings()
    }

    /// The modes the servo refuses now, as `osc-bus status` says them.
    pub fn refused_said(&self) -> String {
        self.store.refused_said()
    }

    pub fn unsaved_said(&self) -> &'static str {
        if self.unsaved { "yes" } else { "no" }
    }

    /// The servo's fingerprint: its record's hash.
    pub fn hash(&self, d: &Descriptor) -> String {
        self.record(d)["hash"].as_str().unwrap_or_default().into()
    }

    /// The record of this state, its hash the notebooks' `state_hash`.
    pub fn record(&self, d: &Descriptor) -> Value {
        let end = self.base as usize + self.raw.len();
        let ours: Vec<&Field> = d
            .fields
            .iter()
            .filter(|f| (f.addr as usize) < end)
            .collect();
        let fields: Map<String, Value> = ours
            .iter()
            .map(|f| (f.name.clone(), self.text(f)))
            .collect();
        let writable: Vec<&str> = ours
            .iter()
            .filter(|f| f.access == Access::Rw)
            .map(|f| f.name.as_str())
            .collect();
        let points: Vec<u8> = self
            .lut
            .points
            .iter()
            .flat_map(|k| k.to_le_bytes())
            .collect();
        let mut doc = json!({
            "addr": self.base,
            "raw": hex(&self.raw),
            "fields": fields,
            "writable": writable,
            "pos_table": {"state": self.table_state(), "points_sha1": self.points_sha1(), "points": hex(&points)},
            "wide_codes": self.slots(d).map(|s| hex(&s)),
            "settings": self.settings_said(),
            "refused_modes": self.refused_said(),
            "unsaved": self.unsaved,
        });
        let hash = state_hash(&doc);
        if let Some(m) = doc.as_object_mut() {
            m.insert("hash".into(), hash.into());
        }
        doc
    }
}

/// What a state record's hash covers: the table bytes and the position
/// table. Not the decoded fields, which `raw` holds, the descriptor's
/// writable list, the points, which their sha1 stands for, or what the
/// servo says of its state (`settings`, `refused_modes`, `unsaved`).
pub fn state_hash(doc: &Value) -> String {
    pyjson::hash(&hashed(doc))
}

fn hashed(doc: &Value) -> Value {
    let get = |k: &str| doc.get(k).cloned().unwrap_or(Value::Null);
    let table = doc.get("pos_table");
    let t = |k: &str| table.and_then(|t| t.get(k)).cloned().unwrap_or(Value::Null);
    json!({
        "addr": get("addr"),
        "raw": get("raw"),
        "pos_table": {"state": t("state"), "points_sha1": t("points_sha1")},
        "wide_codes": get("wide_codes"),
    })
}

/// The hash a record written while servos carried a plant stamp holds:
/// the same and the `stamp` and `data_state` lines.
fn stamped_hash(doc: &Value) -> String {
    let mut h = hashed(doc);
    if let Some(m) = h.as_object_mut() {
        for k in ["stamp", "data_state"] {
            m.insert(k.into(), doc.get(k).cloned().unwrap_or(Value::Null));
        }
    }
    pyjson::hash(&h)
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(s.get(i..i + 2)?, 16).ok())
        .collect()
}

/// A state record read back: what a restore puts on the servo.
#[derive(Clone, Debug)]
pub struct Record {
    pub doc: Value,
    pub points: [i16; INTERVALS],
}

impl Record {
    /// Refused when its hash is not its content's or when its points are
    /// not the ones its sha1 names. The stored state it was read in is no
    /// refusal: a record of a servo on its defaults restores like any. A
    /// record from a servo with a plant stamp reads under the hash it was
    /// written with.
    pub fn parse(text: &str) -> Result<Self, SetError> {
        let bad = |why: &str| SetError(format!("the record {why}"));
        let doc: Value = serde_json::from_str(text).map_err(|e| SetError(e.to_string()))?;
        let held = doc.get("hash").and_then(Value::as_str);
        let stamped = doc.get("stamp").is_some();
        let want = if stamped {
            stamped_hash(&doc)
        } else {
            state_hash(&doc)
        };
        if held != Some(want.as_str()) {
            return Err(bad("is not what its hash says"));
        }
        let table = doc.get("pos_table");
        let bytes = table
            .and_then(|t| t.get("points"))
            .and_then(Value::as_str)
            .and_then(unhex)
            .filter(|b| b.len() == 2 * INTERVALS)
            .ok_or_else(|| bad("holds no table points"))?;
        let mut points = [0i16; INTERVALS];
        for (k, pair) in points.iter_mut().zip(bytes.as_chunks::<2>().0) {
            *k = i16::from_le_bytes(*pair);
        }
        if table
            .and_then(|t| t.get("points_sha1"))
            .and_then(Value::as_str)
            != Some(points_sha1(&points).as_str())
        {
            return Err(bad("holds points that are not the ones its sha1 names"));
        }
        if doc.get("writable").and_then(Value::as_array).is_none()
            || doc.get("fields").and_then(Value::as_object).is_none()
        {
            return Err(bad("names no fields"));
        }
        Ok(Record { doc, points })
    }

    /// The set that puts the record back: every field it holds that this
    /// tool writes, of class `none`, adopted from the record.
    pub fn adopted(&self, origin: Value) -> Result<SetFile, SetError> {
        let fields = &self.doc["fields"];
        let own = |e: &mut Map<String, Value>| {
            e.insert("class".into(), "none".into());
            e.insert("origin".into(), ORIGIN.into());
        };
        let mut out = Map::new();
        for name in self.doc["writable"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
        {
            if COMMS.contains(&name) || NEVER.contains(&name) || name == WIDE || name == OLD_STAMP {
                continue;
            }
            let value = fields.get(name).cloned().unwrap_or(Value::Null);
            let name = RENAMED
                .iter()
                .find(|(old, _)| *old == name)
                .map_or(name, |(_, new)| new);
            let mut e = Map::new();
            e.insert("value".into(), value);
            own(&mut e);
            e.insert(
                "protection".into(),
                set::looser(name).map_or(Value::Null, |l| json!({"looser": l.word()})),
            );
            out.insert(name.into(), Value::Object(e));
        }
        let mut table = json!({
            "state": self.doc["pos_table"]["state"],
            "points_sha1": self.doc["pos_table"]["points_sha1"],
        });
        let mut wide = match self.doc.get("wide_codes").and_then(Value::as_str) {
            Some(h) => {
                json!({"slots": unhex(h).ok_or_else(|| SetError("the record's slots are not hex".into()))?})
            }
            None => Value::Null,
        };
        for e in [table.as_object_mut(), wide.as_object_mut()]
            .into_iter()
            .flatten()
        {
            own(e);
        }
        let mut doc = Map::new();
        doc.insert("format".into(), set::FORMAT.into());
        doc.insert("pair".into(), "restored".into());
        doc.insert("inputs".into(), json!({}));
        doc.insert("origins".into(), json!({ ORIGIN: origin }));
        doc.insert("wide_codes".into(), wide);
        doc.insert("pos_table".into(), table);
        doc.insert("fields".into(), Value::Object(out));
        doc.insert("previous".into(), Value::Null);
        SetFile::sealed(doc)
    }
}
