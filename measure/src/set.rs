//! The set file: the notebooks' format 1 (`oscnb.constants`), so either
//! tool reads what the other writes. Every CONFIG and CALIB field a host
//! writes past the bus front, each with its value as `osc-bus dump` prints it,
//! its class and where it came from, with the wide-code slots, the
//! position table's state and the sha1 of its points, and the hashes of
//! that content. A set is generated, never edited: a file whose hashes
//! are not its content's is refused.

use std::fmt;

use osc_client::descriptor::{Descriptor, Field, Kind};
use osc_client::pos_lut::{INTERVALS, PAGE_POINTS, PAGES};
use serde_json::{Map, Value, json};
use sha1::{Digest, Sha1};

use crate::nameplate::Nameplate;
use crate::pyjson::{self, dumps, hex};
use crate::rules::{self, Class, Outcome};

pub const FORMAT: u64 = 1;
const CLASSES: [&str; 6] = ["measured", "bench", "datasheet", "policy", "pinned", "none"];
/// The classes of a set written before a class said where a value comes
/// from; such a set loads as it was written.
const WAS: [&str; 2] = ["held", "adopted"];

/// Where a servo answers, not what it is (protocol sec 5): in no set.
pub const COMMS: [&str; 3] = ["id", "baud_rate_idx", "response_deadline_us"];
pub const WIDE: &str = "adc_wide_mid_q4";
/// What this tool never writes, whatever a set holds: the bus front, the
/// supply word a capture is matched to its board by, and the pot force
/// terms, which are tuning for the load.
pub const NEVER: [&str; 6] = [
    "id",
    "baud_rate_idx",
    "response_deadline_us",
    "vdd_mv",
    "pot_steady_counts_per_a",
    "pot_fast_counts_per_a",
];

/// The way a protection field's value moves to protect less.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Looser {
    Up,
    Down,
    /// No one direction says it: any change is taken as protecting less.
    Judged,
}

impl Looser {
    pub fn word(self) -> &'static str {
        match self {
            Looser::Up => "up",
            Looser::Down => "down",
            Looser::Judged => "judged",
        }
    }
}

/// The fields whose change is a change of what protects the motor, the
/// train and the rail, as the notebooks list them (`LOOSER`), read off the
/// kernel's use of each.
const LOOSER: [(&str, Looser); 25] = [
    ("current_limit_counts", Looser::Up),
    ("stall_tau_trip_counts", Looser::Up),
    // the press arms over the yield current and folds to it; 0 reads as the cap
    ("yield_current_counts", Looser::Judged),
    ("stall_release_counts", Looser::Up),
    ("stall_response", Looser::Judged),
    ("stall_time_ticks", Looser::Up),
    ("stall_omega_max_cps", Looser::Down),
    ("oc_trip_counts", Looser::Up),
    ("oc_trip_ticks", Looser::Up),
    ("duty_max_q15", Looser::Up),
    ("cutoff_cc", Looser::Up),
    // 0 turns the fold off
    ("rail_fold_q8", Looser::Judged),
    ("pos_error_counts", Looser::Up),
    ("pos_error_time_ticks", Looser::Up),
    ("speed_ceiling_cps", Looser::Up),
    ("sensor_delta_max", Looser::Up),
    ("sensor_bad_count", Looser::Up),
    // one curve of four words: a coefficient moves the reading up on one side of the reference
    ("ntc_raw_ref", Looser::Judged),
    ("ntc_t_ref_cc", Looser::Judged),
    ("ntc_k1_q88", Looser::Judged),
    ("ntc_k2_q24", Looser::Judged),
    // a longer leak heats later and cools later; a body time of 0 drops the body node
    ("thermal_tau_s", Looser::Judged),
    ("thermal_tau2_s", Looser::Judged),
    ("th_rise_q016", Looser::Down),
    ("th_rise2_q016", Looser::Down),
];
/// A protection field a firmware change took out of the table: a set
/// written while it stood still names it.
const RETIRED: [(&str, Looser); 2] = [
    ("derate_start_cc", Looser::Up),
    ("v_undervolt_counts", Looser::Down),
];

/// The fields each bounding field bounds: none is ever over it.
const BOUNDS: [(&str, &[&str]); 1] = [("speed_ceiling_cps", &["velocity_limit_cps"])];

const TOP: [&str; 13] = [
    "format",
    "pair",
    "inputs",
    "origins",
    "wide_codes",
    "pos_table",
    "scale",
    "scales",
    "limits",
    "fields",
    "hash",
    "previous",
    "deviation",
];
const BLOCKS: [&str; 5] = ["inputs", "origins", "scales", "limits", "fields"];
const LEAD: [&str; 10] = [
    "value",
    "slots",
    "state",
    "kind",
    "counts_per_deg",
    "points_sha1",
    "class",
    "reason",
    "needs",
    "datasheet_candidate",
];

/// Whether the firmware took `name` out of the table: a set written while it
/// stood still names it, and nothing writes or checks it.
pub fn retired(name: &str) -> bool {
    RETIRED.iter().any(|(n, _)| *n == name)
}

pub fn looser(name: &str) -> Option<Looser> {
    LOOSER
        .iter()
        .chain(RETIRED.iter())
        .find(|(n, _)| *n == name)
        .map(|(_, l)| *l)
}

fn protection_of(name: &str) -> Value {
    match looser(name) {
        Some(l) => json!({"looser": l.word()}),
        None => Value::Null,
    }
}

/// The sha1 a table's 256 points are named by: i16 LE in a row.
pub fn points_sha1(points: &[i16]) -> String {
    let bytes: Vec<u8> = points.iter().flat_map(|k| k.to_le_bytes()).collect();
    hex(&Sha1::digest(&bytes))
}

pub fn file_sha1(bytes: &[u8]) -> String {
    hex(&Sha1::digest(bytes))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SetError(pub String);

impl fmt::Display for SetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for SetError {}

fn err<T>(why: impl Into<String>) -> Result<T, SetError> {
    Err(SetError(why.into()))
}

fn object<'a>(v: Option<&'a Value>, what: &str) -> Result<&'a Map<String, Value>, SetError> {
    v.and_then(Value::as_object)
        .ok_or_else(|| SetError(format!("{what} is not an object")))
}

/// A set as the file holds it.
#[derive(Clone, Debug, PartialEq)]
pub struct SetFile {
    doc: Map<String, Value>,
}

impl SetFile {
    /// The notebooks' `load`: refused unless it is a set as the tool
    /// writes one, its hashes those of its content.
    pub fn parse(text: &str) -> Result<Self, SetError> {
        let doc: Value = serde_json::from_str(text).map_err(|e| SetError(e.to_string()))?;
        let Value::Object(doc) = doc else {
            return err("a set file is one JSON object");
        };
        if let Some(extra) = doc.keys().find(|k| !TOP.contains(&k.as_str())) {
            return err(format!("format {FORMAT} holds no {extra}"));
        }
        if doc.get("format").and_then(Value::as_u64) != Some(FORMAT) {
            return err(format!(
                "format {}, this tool reads {FORMAT}",
                doc.get("format").map(dumps).unwrap_or_default()
            ));
        }
        let set = SetFile { doc };
        set.loaded()?;
        Ok(set)
    }

    fn loaded(&self) -> Result<(), SetError> {
        let doc = &self.doc;
        let fields = object(doc.get("fields"), "fields")?;
        let table = object(doc.get("pos_table"), "pos_table")?;
        let empty = Map::new();
        let scales = doc
            .get("scales")
            .and_then(Value::as_object)
            .unwrap_or(&empty);
        let origins = doc
            .get("origins")
            .and_then(Value::as_object)
            .unwrap_or(&empty);
        for (name, e) in fields.iter().chain(scales) {
            let class = e.get("class").and_then(Value::as_str).unwrap_or("");
            if e.get("value").is_none() || !CLASSES.iter().chain(&WAS).any(|c| *c == class) {
                return err(format!(
                    "{name} holds no value of one class of {}",
                    CLASSES.join(", ")
                ));
            }
        }
        let none = Value::Null;
        let marked = fields.iter().map(|(n, e)| (n.as_str(), e)).chain([
            ("wide-code slots", doc.get("wide_codes").unwrap_or(&none)),
            ("position table", &doc["pos_table"]),
        ]);
        for (name, e) in marked {
            if let Some(o) = e.get("origin")
                && !o.as_str().is_some_and(|o| origins.contains_key(o))
            {
                return err(format!(
                    "{name} is adopted from {}, which the file does not hold",
                    dumps(o)
                ));
            }
            let class = e.get("class").and_then(Value::as_str).unwrap_or("");
            let inputs = e
                .get("inputs")
                .and_then(Value::as_array)
                .is_some_and(|a| !a.is_empty());
            if (class == "bench" && !inputs)
                || (class == "datasheet" && !(inputs && e.get("document").is_some()))
            {
                return err(format!(
                    "{name} is of class {class} and names no record or no document"
                ));
            }
        }
        let scale_sha = doc.get("scale").and_then(|s| s.get("points_sha1"));
        if doc
            .get("scale")
            .and_then(|s| s.get("kind"))
            .and_then(Value::as_str)
            == Some("table")
            && scale_sha != table.get("points_sha1")
        {
            return err("its scale is that of another table than its own");
        }
        let limits = doc
            .get("limits")
            .and_then(Value::as_object)
            .unwrap_or(&empty);
        for (name, e) in fields {
            if e.get("protection") != Some(&protection_of(name)) {
                return err(format!(
                    "{name} says {} of protection, the tool {}",
                    e.get("protection").map(dumps).unwrap_or("nothing".into()),
                    dumps(&protection_of(name))
                ));
            }
        }
        for (name, e) in fields.iter().chain(limits) {
            if e.get("pos_unit").and_then(Value::as_i64).unwrap_or(0) != 0
                && e.get("scale_points_sha1") != scale_sha
            {
                return err(format!(
                    "{name} was computed for another table than the set's scale"
                ));
            }
        }
        for (name, e) in limits {
            let bounds = BOUNDS.iter().find(|(b, _)| *b == name.as_str());
            let class = e.get("class").and_then(Value::as_str).unwrap_or("");
            let names: Option<Vec<&str>> = e
                .get("bounds")
                .and_then(Value::as_array)
                .map(|a| a.iter().filter_map(Value::as_str).collect());
            if bounds.is_none_or(|(_, f)| names.as_deref() != Some(*f))
                || e.get("value").is_none()
                || !CLASSES.contains(&class)
            {
                return err(format!("{name} is no limit as the tool writes one"));
            }
        }
        for (limit, bounded) in BOUNDS {
            let Some(cap) = fields
                .get(limit)
                .or_else(|| limits.get(limit))
                .and_then(|e| e.get("value"))
                .and_then(Value::as_i64)
            else {
                continue;
            };
            for name in bounded {
                if let Some(v) = fields
                    .get(*name)
                    .and_then(|e| e.get("value"))
                    .and_then(Value::as_i64)
                    && v > cap
                {
                    return err(format!("{name}: {v} is over the set's {limit}, {cap}"));
                }
            }
        }
        if doc.get("hash") != Some(&Value::Object(self.hashes()?)) {
            return err(
                "its hashes are not those of its content: a set is generated, never edited",
            );
        }
        if let Some(dev) = doc.get("deviation").filter(|d| !d.is_null()) {
            let ok = dev.as_object().is_some_and(|d| {
                d.len() == 3
                    && ["file", "sha1", "fields"]
                        .iter()
                        .all(|k| d.contains_key(*k))
                    && d["fields"].as_object().is_some_and(|moved| {
                        !moved.is_empty()
                            && moved.iter().all(|(n, pair)| {
                                fields.get(n).and_then(|e| e.get("value")) == pair.get(1)
                            })
                    })
            });
            if !ok {
                return err(
                    "its deviation is not the fields it moved, each as [of record, deviated]",
                );
            }
        }
        Ok(())
    }

    /// The set from its document, the hashes computed over it.
    pub fn sealed(mut doc: Map<String, Value>) -> Result<Self, SetError> {
        doc.remove("hash");
        let mut set = SetFile { doc };
        let hashes = set.hashes()?;
        set.doc.insert("hash".into(), Value::Object(hashes));
        Ok(set)
    }

    /// The set's hashes: `plant` over every plant field's value with the
    /// table's state and points and the slots, `protection` over the
    /// protection fields' values, `set` over the two.
    pub fn hashes(&self) -> Result<Map<String, Value>, SetError> {
        let fields = object(self.doc.get("fields"), "fields")?;
        let table = object(self.doc.get("pos_table"), "pos_table")?;
        let (mut plant, mut protection) = (Map::new(), Map::new());
        for (name, e) in fields {
            let v = e.get("value").cloned().unwrap_or(Value::Null);
            if looser(name).is_some() {
                protection.insert(name.clone(), v);
            } else {
                plant.insert(name.clone(), v);
            }
        }
        let slots = match self.doc.get("wide_codes") {
            Some(Value::Object(w)) => w.get("slots").cloned().unwrap_or(Value::Null),
            _ => Value::Null,
        };
        let plant = json!({
            "fields": plant,
            "pos_table": {"state": table.get("state"), "points_sha1": table.get("points_sha1")},
            "wide_codes": slots,
        });
        let (plant, protection) = (
            pyjson::hash(&plant),
            pyjson::hash(&Value::Object(protection)),
        );
        let mut out = Map::new();
        out.insert("plant".into(), plant.clone().into());
        out.insert("protection".into(), protection.clone().into());
        out.insert(
            "set".into(),
            pyjson::hash(&json!({"plant": plant, "protection": protection})).into(),
        );
        for block in ["scales", "limits"] {
            if let Some(Value::Object(b)) = self.doc.get(block)
                && !b.is_empty()
            {
                let values: Map<String, Value> = b
                    .iter()
                    .map(|(n, e)| (n.clone(), e.get("value").cloned().unwrap_or(Value::Null)))
                    .collect();
                out.insert(block.into(), pyjson::hash(&Value::Object(values)).into());
            }
        }
        Ok(out)
    }

    /// The file, byte for byte as the notebooks render it: the keys in one
    /// order, the fields in the table's, an entry a line.
    pub fn render(&self) -> String {
        let mut lines = Vec::new();
        for key in TOP {
            let Some(v) = self.doc.get(key) else {
                continue;
            };
            match v {
                Value::Object(block) if BLOCKS.contains(&key) && !block.is_empty() => {
                    let mut names: Vec<&String> = block.keys().collect();
                    if key != "fields" {
                        names.sort();
                    }
                    let body: Vec<String> = names
                        .into_iter()
                        .map(|n| {
                            format!(
                                "  {}: {}",
                                dumps(&Value::String(n.clone())),
                                entry(&block[n.as_str()])
                            )
                        })
                        .collect();
                    lines.push(format!(" \"{key}\": {{\n{}\n }}", body.join(",\n")));
                }
                _ => lines.push(format!(" \"{key}\": {}", entry(v))),
            }
        }
        format!("{{\n{}\n}}\n", lines.join(",\n"))
    }

    pub fn pair(&self) -> &str {
        self.doc.get("pair").and_then(Value::as_str).unwrap_or("")
    }

    pub fn hash(&self) -> &str {
        self.doc
            .get("hash")
            .and_then(|h| h.get("set"))
            .and_then(Value::as_str)
            .unwrap_or("")
    }

    /// Every field's value, in the set's order.
    pub fn values(&self) -> Vec<(&str, &Value)> {
        self.doc
            .get("fields")
            .and_then(Value::as_object)
            .map(|f| {
                f.iter()
                    .filter_map(|(n, e)| Some((n.as_str(), e.get("value")?)))
                    .collect()
            })
            .unwrap_or_default()
    }

    pub fn value(&self, name: &str) -> Option<&Value> {
        self.doc.get("fields")?.get(name)?.get("value")
    }

    /// A line a field: its value, unit and spread, `+/- p %` of its band
    /// or `nominal` with none.
    pub fn terms(&self) -> Vec<String> {
        let Some(fields) = self.doc.get("fields").and_then(Value::as_object) else {
            return Vec::new();
        };
        fields
            .iter()
            .map(|(name, e)| {
                let band = e
                    .get("band")
                    .and_then(Value::as_array)
                    .and_then(|b| Some((b.first()?.as_i64()?, b.get(1)?.as_i64()?)));
                let value = e.get("value").and_then(Value::as_i64);
                let spread = match (value, band) {
                    (Some(v), Some(b)) => match spread_pct(v, b) {
                        Some(p) => format!("+/- {p:.2} %"),
                        None => format!("band {} to {}", b.0, b.1),
                    },
                    _ => "nominal".into(),
                };
                format!(
                    "{name} {} {} {spread}",
                    e.get("value").map_or(String::new(), dumps),
                    e.get("unit").and_then(Value::as_str).unwrap_or("")
                )
            })
            .collect()
    }

    /// The slots, none when the set holds none.
    pub fn slots(&self) -> Result<Option<Vec<u8>>, SetError> {
        match self.doc.get("wide_codes") {
            None | Some(Value::Null) => Ok(None),
            Some(w) => {
                let slots = w.get("slots").and_then(Value::as_array);
                let bytes: Option<Vec<u8>> = slots.map(|a| {
                    a.iter()
                        .filter_map(|v| v.as_u64().and_then(|v| u8::try_from(v).ok()))
                        .collect()
                });
                match bytes {
                    Some(b) if slots.is_some_and(|a| a.len() == b.len()) => Ok(Some(b)),
                    _ => err("its wide-code slots are not bytes"),
                }
            }
        }
    }

    /// The table's state name and the sha1 of its points.
    pub fn table(&self) -> (&str, &str) {
        let t = self.doc.get("pos_table");
        let get = |k: &str| {
            t.and_then(|t| t.get(k))
                .and_then(Value::as_str)
                .unwrap_or("")
        };
        (get("state"), get("points_sha1"))
    }

    /// The deviation file a derived set names: such a set is never saved.
    pub fn deviation(&self) -> Option<&str> {
        self.doc
            .get("deviation")
            .and_then(|d| d.get("file"))
            .and_then(Value::as_str)
    }
}

/// One entry on a line: what it holds first, then its class, the rest by
/// name.
fn entry(v: &Value) -> String {
    let Value::Object(e) = v else {
        return dumps(v);
    };
    let mut keys: Vec<&String> = e.keys().filter(|k| !LEAD.contains(&k.as_str())).collect();
    keys.sort();
    let lead = LEAD
        .iter()
        .filter_map(|k| e.get_key_value(*k).map(|(k, _)| k));
    let ordered: Map<String, Value> = lead
        .chain(keys)
        .map(|k| (k.clone(), e[k.as_str()].clone()))
        .collect();
    dumps(&Value::Object(ordered))
}

/// A field's value as `osc-bus dump` prints it and a set holds it: a number,
/// `on`/`off`, an enum as `Name (n)`, the gear class as `n (name)`, bytes
/// as spaced hex.
pub fn text_of(f: &Field, bytes: &[u8]) -> Value {
    let raw = bytes
        .iter()
        .take(f.width.min(8) as usize)
        .enumerate()
        .fold(0u64, |v, (i, &b)| v | (b as u64) << (8 * i));
    match f.kind {
        Kind::Bool => Value::String(if raw != 0 { "on" } else { "off" }.into()),
        Kind::Enum => match f.variants.iter().find(|v| v.value as u64 == raw) {
            Some(v) => Value::String(format!("{} ({raw})", v.name)),
            None => raw.into(),
        },
        Kind::Int => {
            let shift = 64 - 8 * f.width.min(8) as u32;
            (((raw << shift) as i64) >> shift).into()
        }
        Kind::Uint if f.name == "gear_class" => {
            let name = osc_client::calib::GearClass::from_byte(raw as u8).name();
            Value::String(format!("{raw} ({name})"))
        }
        Kind::Uint => raw.into(),
        Kind::Bytes => Value::String(
            bytes
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<Vec<_>>()
                .join(" "),
        ),
    }
}

/// The table bytes a set's value of `f` is, refused when the value is not
/// one the field holds.
pub fn bytes_of(f: &Field, v: &Value) -> Result<Vec<u8>, SetError> {
    let bad = || {
        SetError(format!(
            "{}: {} is no value the field holds",
            f.name,
            dumps(v)
        ))
    };
    // `osc-bus dump`'s text less the note in parentheses after a name or a number
    let plain = match v {
        Value::String(s) => match s.find(" (") {
            Some(at) if s.ends_with(')') => s[..at].to_string(),
            _ => s.clone(),
        },
        Value::Number(n) => n.to_string(),
        Value::Bool(b) => if *b { "on" } else { "off" }.to_string(),
        _ => return Err(bad()),
    };
    let int = |s: &str| s.parse::<i64>().map_err(|_| bad());
    let raw: i64 = match f.kind {
        Kind::Bool => match plain.as_str() {
            "on" | "true" | "1" => 1,
            "off" | "false" | "0" => 0,
            _ => return Err(bad()),
        },
        Kind::Enum => match f
            .variants
            .iter()
            .find(|x| x.name.eq_ignore_ascii_case(&plain))
        {
            Some(x) => x.value as i64,
            None => int(&plain)?,
        },
        Kind::Uint | Kind::Int => int(&plain)?,
        Kind::Bytes => {
            let b: Result<Vec<u8>, _> = plain
                .split_whitespace()
                .map(|h| u8::from_str_radix(h, 16))
                .collect();
            return b
                .ok()
                .filter(|b| b.len() == f.width as usize)
                .ok_or_else(bad);
        }
    };
    let w = f.width as u32;
    let (lo, hi) = match f.kind {
        Kind::Int => (-(1i64 << (8 * w - 1)), (1i64 << (8 * w - 1)) - 1),
        _ => (0, (1i64 << (8 * w)) - 1),
    };
    if !(lo..=hi).contains(&raw) || !(1..=4).contains(&w) {
        return Err(bad());
    }
    Ok(raw.to_le_bytes()[..w as usize].to_vec())
}

/// A position table file of the notebooks (`postable`): the 256 points by
/// page and the stops it was built against.
#[derive(Clone, Debug)]
pub struct TableFile {
    pub points: [i16; INTERVALS],
    pub stops: (i64, i64),
    pub counts_per_deg: Option<f64>,
}

impl TableFile {
    pub fn parse(text: &str) -> Result<Self, SetError> {
        let doc: Value = serde_json::from_str(text).map_err(|e| SetError(e.to_string()))?;
        let pages = doc.get("points_by_page").and_then(Value::as_array);
        let mut points = [0i16; INTERVALS];
        let mut n = 0;
        for (p, page) in pages.into_iter().flatten().enumerate() {
            for (k, v) in page.as_array().into_iter().flatten().enumerate() {
                let v = v.as_i64().and_then(|v| i16::try_from(v).ok());
                match (v, points.get_mut(p * PAGE_POINTS + k)) {
                    (Some(v), Some(slot)) if k < PAGE_POINTS => *slot = v,
                    _ => return err("a table file's points are not 8 pages of 32 i16"),
                }
                n += 1;
            }
        }
        if n != PAGES * PAGE_POINTS {
            return err("a table file's points are not 8 pages of 32 i16");
        }
        let stops = doc.get("inputs").and_then(|i| i.get("stops"));
        let stop = |k: &str| stops.and_then(|s| s.get(k)).and_then(Value::as_i64);
        let (Some(lo), Some(hi)) = (stop("raw_min"), stop("raw_max")) else {
            return err("a table file names no stops it was built against");
        };
        Ok(TableFile {
            points,
            stops: (lo, hi),
            counts_per_deg: doc
                .get("target")
                .and_then(|t| t.get("counts_per_deg"))
                .and_then(Value::as_f64),
        })
    }
}

/// What a set from the rules is made of besides the rules' set.
pub struct FromRules<'a> {
    pub pair: &'a str,
    pub nameplate: &'a Nameplate,
    /// The scale the rules read (`measured.table`).
    pub counts_per_deg: Option<f64>,
    pub table: Option<&'a TableFile>,
    pub slots: Option<&'a [u8]>,
    /// Every input file by name: `(name, path, sha1)`.
    pub files: &'a [(String, String, String)],
}

/// A derived field's spread: half its band (the rule again at each end of
/// every banded input, a measured one's band one sd either side) over its
/// value, percent to 0.01.
pub fn spread_pct(value: i64, band: (i64, i64)) -> Option<f64> {
    (value != 0).then(|| (5000.0 * (band.1 - band.0) as f64 / (value as f64).abs()).round() / 100.0)
}

/// A set file from what the rules gave: a refused field refuses the set;
/// a held field and a tuning field are left out, and a field stored at a
/// bound is kept there, each said in a note.
pub fn from_rules(
    set: &rules::Set,
    r: &FromRules,
    d: &Descriptor,
) -> Result<(SetFile, Vec<String>), SetError> {
    let refused: Vec<String> = set
        .fields
        .iter()
        .filter_map(|(_, o)| match o {
            Outcome::Refused(r) => Some(r.why.clone()),
            _ => None,
        })
        .collect();
    if !refused.is_empty() {
        return err(format!("the rules refuse:\n  {}", refused.join("\n  ")));
    }
    let mut notes = Vec::new();
    let mut fields = Map::new();
    for f in &d.fields {
        let Some(outcome) = set.get(&f.name) else {
            continue;
        };
        let e = match outcome {
            Outcome::Bound(e) if e.class == Class::Tuning => {
                notes.push(format!(
                    "{}: left out, tuning for the load the tool never writes",
                    f.name
                ));
                continue;
            }
            Outcome::Bound(e) => {
                if let Some(c) = &e.clamp {
                    notes.push(format!("{}: {c}", f.name));
                }
                e
            }
            Outcome::Held(m) => {
                notes.push(format!(
                    "{}: left out, its rule reads {}, which is not there",
                    f.name, m.input
                ));
                continue;
            }
            Outcome::Refused(_) => continue,
        };
        let w = f.width as usize;
        let text = text_of(f, &e.value.to_le_bytes()[..w.min(8)]);
        let class = match e.class {
            Class::Measured => "measured",
            Class::Policy => "policy",
            Class::Nameplate if r.nameplate.document.is_some() => "datasheet",
            Class::Nameplate | Class::Board | Class::User | Class::Tuning => "pinned",
        };
        let mut entry = Map::new();
        entry.insert("value".into(), text);
        entry.insert("class".into(), class.into());
        if let Some((lo, hi)) = e.band {
            entry.insert("band".into(), json!([lo, hi]));
            if let Some(p) = spread_pct(e.value, (lo, hi)) {
                entry.insert("spread_pct".into(), p.into());
            }
        }
        if class == "datasheet" {
            entry.insert("document".into(), r.nameplate.document.clone().into());
        }
        entry.insert("inputs".into(), json!(e.inputs));
        entry.insert("protection".into(), protection_of(&f.name));
        entry.insert("rule".into(), e.rule.into());
        entry.insert("unit".into(), e.unit.into());
        fields.insert(f.name.clone(), Value::Object(entry));
    }
    let stops = (set.value("raw_min"), set.value("raw_max"));
    let (pos_table, scale) = match r.table {
        Some(t) => {
            if stops != (Some(t.stops.0), Some(t.stops.1)) {
                return err(format!(
                    "the table file was built against stops {} and {}, the set's are {:?} and {:?}",
                    t.stops.0, t.stops.1, stops.0, stops.1
                ));
            }
            if t.counts_per_deg != r.counts_per_deg {
                return err("the table file is not the one the inputs' scale was read from");
            }
            let sha = points_sha1(&t.points);
            let state = if t.points.iter().all(|&k| k == 0) {
                "IDENTITY"
            } else {
                "LIVE"
            };
            (
                json!({"state": state, "points_sha1": sha, "class": "measured", "inputs": ["pos_table"], "rule": "table.pos_lut"}),
                json!({"kind": "table", "counts_per_deg": r.counts_per_deg, "points_sha1": sha}),
            )
        }
        None => (
            json!({"state": "IDENTITY", "points_sha1": points_sha1(&[0; INTERVALS]), "class": "measured", "rule": "table.none"}),
            json!({"kind": "raw", "counts_per_deg": r.counts_per_deg, "points_sha1": null}),
        ),
    };
    let wide_codes = match r.slots {
        Some(s) => {
            json!({"slots": s, "class": "measured", "inputs": ["wide_codes"], "rule": "slots.wide_codes"})
        }
        None => Value::Null,
    };
    let inputs: Map<String, Value> = r
        .files
        .iter()
        .map(|(n, file, sha)| (n.clone(), json!({"file": file, "sha1": sha})))
        .collect();
    let mut doc = Map::new();
    doc.insert("format".into(), FORMAT.into());
    doc.insert("pair".into(), r.pair.into());
    doc.insert("inputs".into(), Value::Object(inputs));
    doc.insert("origins".into(), json!({}));
    doc.insert("wide_codes".into(), wide_codes);
    doc.insert("pos_table".into(), pos_table);
    if r.counts_per_deg.is_some() {
        doc.insert("scale".into(), scale);
    }
    doc.insert("fields".into(), Value::Object(fields));
    doc.insert("previous".into(), Value::Null);
    let set = SetFile::sealed(doc)?;
    set.loaded()?;
    Ok((set, notes))
}
