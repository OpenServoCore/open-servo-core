//! Descriptor-driven typed register access: the operator names
//! `goal_position` instead of `0x0184`. Types, selection and the value codec
//! live in `osc_client::descriptor`; this layer supplies the JSON (built-ins
//! compiled in, operator overrides from the config dir) and the string
//! formatting. Operators drop model-matching JSON in their config dir to
//! override or add.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

pub use osc_client::descriptor::{Descriptor, Field, Registry};
use osc_client::descriptor::{Kind, Selection, Value};
use osc_protocol::version::unpack_version;

use crate::{hex, parse_hex};

/// One built-in descriptor, compiled from the checked-in export. A second
/// built-in is one more entry.
const BUILTINS: &[&str] = &[include_str!("../../../descriptors/osc-servo/0.1.json")];

/// The compiled built-ins overlaid by every external descriptor; an
/// external file sharing a built-in's model and major.minor replaces it.
pub fn load() -> Result<Registry> {
    let mut reg = Registry::new();
    for s in BUILTINS {
        reg.push(Descriptor::parse(s).context("built-in descriptor")?);
    }
    for path in external_paths()? {
        let text = std::fs::read_to_string(&path)
            .with_context(|| format!("reading descriptor {}", path.display()))?;
        reg.push(
            Descriptor::parse(&text)
                .with_context(|| format!("parsing descriptor {}", path.display()))?,
        );
    }
    Ok(reg)
}

/// Pick the descriptor for a servo-reported (model, packed fw). Returns the
/// selection plus any advisory note to print at the call site.
pub fn select(reg: &Registry, model: u16, fw: u16) -> Result<(&Descriptor, Option<String>)> {
    let (major, minor, _) = unpack_version(fw);
    match reg.select(model, fw) {
        Selection::Exact(d) => return Ok((d, None)),
        Selection::OlderMinor(d) => {
            let note = format!(
                "note: descriptor {} is fw {}.{}, servo reports fw {major}.{minor}; fields added since are unnamed",
                d.model, d.firmware_major, d.firmware_minor
            );
            return Ok((d, Some(note)));
        }
        Selection::Incompatible { newest: (a, b) } => bail!(
            "no descriptor for model {model:#06x} fw {major}.{minor}; newest known {a}.{b}\ndrop a matching *.json in {}",
            external_dir().display(),
        ),
        Selection::UnknownModel => {}
    }
    let known: Vec<&Descriptor> = reg.iter().collect();
    // model 0 is unassigned (unprovisioned or dev firmware). With exactly
    // one descriptor known there is no ambiguity to resolve.
    if model == 0
        && let [only] = known.as_slice()
    {
        let note = format!(
            "note: servo model unassigned; assuming {} (the only known descriptor)",
            only.model
        );
        return Ok((only, Some(note)));
    }
    let mut known: Vec<String> = known
        .iter()
        .map(|d| {
            format!(
                "{} {}.{} ({:#06x})",
                d.model, d.firmware_major, d.firmware_minor, d.model_number
            )
        })
        .collect();
    known.sort();
    bail!(
        "no descriptor for model {model:#06x}; known: {}\ndrop a matching *.json in {}",
        known.join(", "),
        external_dir().display(),
    );
}

/// External descriptor directory: `$OSC_CONFIG_DIR/descriptors` else
/// `$HOME/.config/osc/descriptors`.
fn external_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("OSC_CONFIG_DIR") {
        return Path::new(&dir).join("descriptors");
    }
    let home = std::env::var_os("HOME").unwrap_or_default();
    Path::new(&home).join(".config/osc/descriptors")
}

/// Every `*.json` under the external dir, sorted for deterministic overlay.
/// A missing dir is not an error (operators need not have one).
fn external_paths() -> Result<Vec<PathBuf>> {
    let dir = external_dir();
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(e).with_context(|| format!("reading {}", dir.display())),
    };
    let mut paths: Vec<PathBuf> = Vec::new();
    for entry in entries {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "json") {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

/// Find a field by exact name; on a miss, suggest near matches so a typo
/// is one correction away.
pub fn field<'a>(d: &'a Descriptor, name: &str) -> Result<&'a Field> {
    if let Some(f) = d.field(name) {
        return Ok(f);
    }
    let near: Vec<&str> = d
        .fields
        .iter()
        .filter(|f| f.name.contains(name) || name.contains(f.name.as_str()))
        .map(|f| f.name.as_str())
        .collect();
    if near.is_empty() {
        bail!("no field named {name:?}");
    }
    bail!("no field named {name:?}; did you mean: {}", near.join(", "));
}

/// Decode raw table bytes into an operator-facing string per the field's kind.
pub fn decode(field: &Field, bytes: &[u8]) -> Result<String> {
    let v = osc_client::descriptor::decode(field, bytes)
        .with_context(|| format!("field {}", field.name))?;
    Ok(match v {
        Value::Uint(n) => n.to_string(),
        Value::Int(n) => n.to_string(),
        Value::Bool(on) => if on { "on" } else { "off" }.into(),
        Value::Enum(n) => match field.variant(n) {
            Some(v) => format!("{} ({n})", v.name),
            None => n.to_string(),
        },
        Value::Bytes(b) => hex(&b),
    })
}

/// Encode an operator string into exactly `width` table bytes per the kind.
/// Bounds are not pre-checked: the servo owns validation and its reject reply
/// is the truth.
pub fn encode(field: &Field, s: &str) -> Result<Vec<u8>> {
    let v = match field.kind {
        Kind::Uint => Value::Uint(parse_u64(s)?),
        Kind::Int => Value::Int(parse_i64(s)?),
        Kind::Bool => Value::Bool(match s.to_ascii_lowercase().as_str() {
            "on" | "true" | "1" => true,
            "off" | "false" | "0" => false,
            _ => bail!("bool wants on|off|true|false|1|0, got {s:?}"),
        }),
        Kind::Enum => Value::Enum(enum_value(field, s)?),
        Kind::Bytes => Value::Bytes(parse_hex(s)?),
    };
    osc_client::descriptor::encode(field, &v).with_context(|| format!("field {}", field.name))
}

/// Resolve an enum arg: variant name (exact, then case-insensitive), else a
/// raw discriminant number.
fn enum_value(field: &Field, s: &str) -> Result<u8> {
    if let Some(v) = field.variants.iter().find(|v| v.name == s) {
        return Ok(v.value);
    }
    if let Some(v) = field
        .variants
        .iter()
        .find(|v| v.name.eq_ignore_ascii_case(s))
    {
        return Ok(v.value);
    }
    if let Ok(n) = parse_u64(s) {
        return u8::try_from(n).with_context(|| format!("enum discriminant {n} overflows u8"));
    }
    let names: Vec<&str> = field.variants.iter().map(|v| v.name.as_str()).collect();
    bail!(
        "enum {} takes a variant name ({}) or number, got {s:?}",
        field.name,
        names.join(", ")
    );
}

fn parse_u64(s: &str) -> Result<u64> {
    match s.strip_prefix("0x") {
        Some(h) => u64::from_str_radix(h, 16).with_context(|| format!("bad hex {s:?}")),
        None => s.parse().with_context(|| format!("bad integer {s:?}")),
    }
}

fn parse_i64(s: &str) -> Result<i64> {
    if let Some(h) = s.strip_prefix("0x") {
        return i64::from_str_radix(h, 16).with_context(|| format!("bad hex {s:?}"));
    }
    if let Some(h) = s.strip_prefix("-0x") {
        return Ok(-i64::from_str_radix(h, 16).with_context(|| format!("bad hex {s:?}"))?);
    }
    s.parse().with_context(|| format!("bad integer {s:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_client::descriptor::Access;

    fn builtin() -> Descriptor {
        Descriptor::parse(BUILTINS[0]).unwrap()
    }

    // Tests build the registry from the built-in only: `load` would read the
    // developer's real external descriptor dir.
    fn registry() -> Registry {
        let mut reg = Registry::new();
        reg.push(builtin());
        reg
    }

    #[test]
    fn builtin_parses() {
        let d = builtin();
        assert_eq!(d.model_number, 257);
        assert_eq!(d.model, "osc-servo");
        assert!(d.fields.len() > 50);
        let id = field(&d, "id").unwrap();
        assert_eq!(id.addr, 16);
        assert_eq!(id.access, Access::Rw);
    }

    #[test]
    fn uint_round_trips() {
        let d = builtin();
        let f = field(&d, "response_deadline_us").unwrap(); // u16
        let bytes = encode(f, "1000").unwrap();
        assert_eq!(bytes, vec![0xE8, 0x03]);
        assert_eq!(decode(f, &bytes).unwrap(), "1000");
        // hex input
        assert_eq!(encode(f, "0x03e8").unwrap(), vec![0xE8, 0x03]);
    }

    #[test]
    fn int_round_trips_and_signs() {
        let d = builtin();
        let f = field(&d, "goal_position").unwrap(); // i32
        let bytes = encode(f, "-100000").unwrap();
        assert_eq!(decode(f, &bytes).unwrap(), "-100000");
        assert_eq!(encode(f, "-0x10").unwrap(), (-16i32).to_le_bytes().to_vec());
    }

    #[test]
    fn bool_round_trips() {
        let d = builtin();
        let f = field(&d, "torque_enable").unwrap();
        assert_eq!(encode(f, "on").unwrap(), vec![1]);
        assert_eq!(encode(f, "true").unwrap(), vec![1]);
        assert_eq!(encode(f, "0").unwrap(), vec![0]);
        assert_eq!(decode(f, &[1]).unwrap(), "on");
        assert_eq!(decode(f, &[0]).unwrap(), "off");
    }

    #[test]
    fn enum_by_name_and_number() {
        let d = builtin();
        let f = field(&d, "mode").unwrap();
        assert_eq!(encode(f, "Position").unwrap(), vec![3]);
        assert_eq!(encode(f, "position").unwrap(), vec![3]); // case-insensitive
        assert_eq!(encode(f, "3").unwrap(), vec![3]);
        assert_eq!(decode(f, &[3]).unwrap(), "Position (3)");
        // off-registry discriminant falls back to the raw number
        assert_eq!(decode(f, &[7]).unwrap(), "7");
        assert!(encode(f, "Nonsense").is_err());
    }

    #[test]
    fn bytes_round_trips() {
        let d = builtin();
        let f = field(&d, "words").unwrap(); // 64 B
        let hexstr = "aa bb ".to_string() + &"00 ".repeat(62);
        let bytes = encode(f, &hexstr).unwrap();
        assert_eq!(bytes.len(), 64);
        assert_eq!(bytes[0], 0xaa);
        assert!(decode(f, &bytes).unwrap().starts_with("aa bb"));
        assert!(encode(f, "aabb").is_err());
    }

    #[test]
    fn overflow_rejects() {
        let d = builtin();
        let f = field(&d, "response_deadline_us").unwrap(); // u16
        assert!(encode(f, "70000").is_err());
        let g = field(&d, "goal_current").unwrap(); // i16
        assert!(encode(g, "40000").is_err());
    }

    #[test]
    fn ro_set_rejects() {
        let d = builtin();
        let f = field(&d, "pos").unwrap(); // ro
        let err = format!("{:#}", encode(f, "0").unwrap_err());
        assert!(err.contains("read-only"));
    }

    #[test]
    fn field_miss_suggests() {
        let d = builtin();
        let err = field(&d, "goal_pos").unwrap_err().to_string();
        assert!(err.contains("goal_position"));
    }

    #[test]
    fn select_exact_match() {
        let reg = registry();
        let b = builtin();
        let fw = osc_protocol::version::pack_version(b.firmware_major, b.firmware_minor, 7);
        let (d, note) = select(&reg, 257, fw).unwrap();
        assert_eq!(d.model_number, 257);
        assert!(note.is_none());
    }

    #[test]
    fn select_older_minor_notes() {
        let reg = registry();
        let b = builtin();
        let fw = osc_protocol::version::pack_version(b.firmware_major, b.firmware_minor + 1, 0);
        let (_, note) = select(&reg, 257, fw).unwrap();
        assert!(note.unwrap().contains("fw"));
    }

    #[test]
    fn select_other_major_bails() {
        let reg = registry();
        let b = builtin();
        let fw = osc_protocol::version::pack_version(b.firmware_major + 1, 0, 0);
        let err = select(&reg, 257, fw).unwrap_err().to_string();
        assert!(err.contains("newest known"));
    }

    #[test]
    fn select_model_zero_single_descriptor() {
        // The built-in set is a single descriptor, so model 0 resolves to it.
        let reg = registry();
        let (d, note) = select(&reg, 0, 1).unwrap();
        assert_eq!(d.model_number, 257);
        assert!(note.unwrap().contains("unassigned"));
    }

    #[test]
    fn select_unknown_model_bails() {
        let err = select(&registry(), 0x0999, 1).unwrap_err().to_string();
        assert!(err.contains("known"));
        assert!(err.contains("osc-servo"));
    }
}
