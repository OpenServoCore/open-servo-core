//! Descriptor-driven typed register access. A descriptor is the firmware's
//! exported device description (descriptors/osc-servo.json); it maps register
//! names to table addresses, widths, access, and value kinds so the operator
//! names `goal_position` instead of `0x0184`. Built-ins are compiled in;
//! operators drop model-matching JSON in their config dir to override or add.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::{hex, parse_hex};

/// One built-in descriptor, compiled from the checked-in export. A second
/// built-in is one more entry.
const BUILTINS: &[&str] = &[include_str!("../../../descriptors/osc-servo.json")];

#[derive(Debug, Clone, Deserialize)]
pub struct Descriptor {
    pub model: String,
    pub model_number: u16,
    pub firmware_version: u8,
    pub table_size: u16,
    pub fields: Vec<Field>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Field {
    pub name: String,
    pub addr: u16,
    pub width: u16,
    pub access: Access,
    pub kind: Kind,
    #[serde(default)]
    pub variants: Vec<Variant>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Access {
    Ro,
    Rw,
}

impl Access {
    pub fn as_str(self) -> &'static str {
        match self {
            Access::Ro => "ro",
            Access::Rw => "rw",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    Uint,
    Int,
    Bool,
    Enum,
    Bytes,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Variant {
    pub name: String,
    /// Discriminants are u8 by construction (the derive requires repr(u8)).
    pub value: u8,
}

/// The known descriptor set: built-ins overlaid by external files sharing a
/// model number.
pub struct Registry {
    descriptors: Vec<Descriptor>,
}

impl Registry {
    /// Parse the compiled built-ins, then overlay every external descriptor.
    /// A built-in that fails to parse is a build-time bug and panics; an
    /// external file that fails to parse is a hard error naming the file.
    pub fn load() -> Result<Self> {
        let mut descriptors: Vec<Descriptor> = BUILTINS
            .iter()
            .map(|s| serde_json::from_str(s).expect("built-in descriptor parses"))
            .collect();
        for path in external_paths()? {
            let text = std::fs::read_to_string(&path)
                .with_context(|| format!("reading descriptor {}", path.display()))?;
            let d: Descriptor = serde_json::from_str(&text)
                .with_context(|| format!("parsing descriptor {}", path.display()))?;
            // Same model number overrides the built-in: the operator's escape
            // hatch for a firmware whose table diverged from the shipped one.
            descriptors.retain(|e| e.model_number != d.model_number);
            descriptors.push(d);
        }
        Ok(Registry { descriptors })
    }

    /// Pick the descriptor for a servo-reported (model, fw). Returns the
    /// selection plus any advisory note to print at the call site.
    pub fn select(&self, model: u16, fw: u8) -> Result<(&Descriptor, Option<String>)> {
        if let Some(d) = self.descriptors.iter().find(|d| d.model_number == model) {
            let note = (d.firmware_version != fw).then(|| {
                format!(
                    "warning: descriptor {} is fw {}, servo reports fw {} (common block is protocol-stable)",
                    d.model, d.firmware_version, fw
                )
            });
            return Ok((d, note));
        }
        // model 0 is unassigned (unprovisioned or dev firmware). With exactly
        // one descriptor known there is no ambiguity to resolve.
        if model == 0
            && let [only] = self.descriptors.as_slice()
        {
            let note = format!(
                "note: servo model unassigned; assuming {} (the only known descriptor)",
                only.model
            );
            return Ok((only, Some(note)));
        }
        let mut known: Vec<String> = self
            .descriptors
            .iter()
            .map(|d| format!("{} ({:#06x})", d.model, d.model_number))
            .collect();
        known.sort();
        bail!(
            "no descriptor for model {:#06x}; known: {}\ndrop a matching *.json in {}",
            model,
            known.join(", "),
            external_dir().display(),
        );
    }
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

impl Descriptor {
    /// Find a field by exact name; on a miss, suggest near matches so a typo
    /// is one correction away.
    pub fn field(&self, name: &str) -> Result<&Field> {
        if let Some(f) = self.fields.iter().find(|f| f.name == name) {
            return Ok(f);
        }
        let near: Vec<&str> = self
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
}

impl Field {
    /// Exclusive end address of this field's bytes.
    pub fn end(&self) -> u16 {
        self.addr + self.width
    }
}

/// Decode raw table bytes into an operator-facing string per the field's kind.
pub fn decode(field: &Field, bytes: &[u8]) -> Result<String> {
    match field.kind {
        Kind::Uint => Ok(read_uint(field, bytes)?.to_string()),
        Kind::Int => Ok(read_int(field, bytes)?.to_string()),
        Kind::Bool => Ok(if bytes.first().copied().unwrap_or(0) != 0 {
            "on".into()
        } else {
            "off".into()
        }),
        Kind::Enum => {
            // Unsigned: a discriminant past 127 must not sign-extend.
            let n = read_uint(field, bytes)?;
            Ok(match field.variants.iter().find(|v| v.value as u64 == n) {
                Some(v) => format!("{} ({})", v.name, n),
                None => n.to_string(),
            })
        }
        Kind::Bytes => Ok(hex(bytes)),
    }
}

/// Encode an operator string into exactly `width` table bytes per the kind.
/// Bounds are not pre-checked: the servo owns validation and its reject reply
/// is the truth.
pub fn encode(field: &Field, s: &str) -> Result<Vec<u8>> {
    if field.access == Access::Ro {
        bail!(
            "field {} is {} (read-only)",
            field.name,
            field.access.as_str()
        );
    }
    match field.kind {
        Kind::Uint => write_uint(field, parse_u64(s)?),
        Kind::Int => write_int(field, parse_i64(s)?),
        Kind::Bool => {
            let on = match s.to_ascii_lowercase().as_str() {
                "on" | "true" | "1" => true,
                "off" | "false" | "0" => false,
                _ => bail!("bool wants on|off|true|false|1|0, got {s:?}"),
            };
            Ok(vec![on as u8])
        }
        Kind::Enum => {
            let n = enum_value(field, s)?;
            write_uint(field, n as u64)
        }
        Kind::Bytes => {
            let bytes = parse_hex(s)?;
            if bytes.len() != field.width as usize {
                bail!(
                    "field {} takes {} hex bytes, got {}",
                    field.name,
                    field.width,
                    bytes.len()
                );
            }
            Ok(bytes)
        }
    }
}

fn read_uint(field: &Field, bytes: &[u8]) -> Result<u64> {
    scalar_le(field, bytes)
}

fn read_int(field: &Field, bytes: &[u8]) -> Result<i64> {
    let raw = scalar_le(field, bytes)?;
    // Sign-extend from the field width.
    Ok(match field.width {
        1 => raw as u8 as i8 as i64,
        2 => raw as u16 as i16 as i64,
        4 => raw as u32 as i32 as i64,
        w => bail!("field {} has unsupported scalar width {w}", field.name),
    })
}

/// Read a little-endian unsigned scalar of the field's width from `bytes`.
fn scalar_le(field: &Field, bytes: &[u8]) -> Result<u64> {
    let w = field.width as usize;
    if !matches!(w, 1 | 2 | 4) {
        bail!("field {} has unsupported scalar width {w}", field.name);
    }
    if bytes.len() < w {
        bail!("field {} wants {w} B, got {}", field.name, bytes.len());
    }
    let mut v = 0u64;
    for (i, &b) in bytes[..w].iter().enumerate() {
        v |= (b as u64) << (8 * i);
    }
    Ok(v)
}

fn write_uint(field: &Field, v: u64) -> Result<Vec<u8>> {
    let max = match field.width {
        1 => u8::MAX as u64,
        2 => u16::MAX as u64,
        4 => u32::MAX as u64,
        w => bail!("field {} has unsupported scalar width {w}", field.name),
    };
    if v > max {
        bail!(
            "value {v} overflows u{} field {}",
            field.width * 8,
            field.name
        );
    }
    Ok(v.to_le_bytes()[..field.width as usize].to_vec())
}

fn write_int(field: &Field, v: i64) -> Result<Vec<u8>> {
    let (lo, hi) = match field.width {
        1 => (i8::MIN as i64, i8::MAX as i64),
        2 => (i16::MIN as i64, i16::MAX as i64),
        4 => (i32::MIN as i64, i32::MAX as i64),
        w => bail!("field {} has unsupported scalar width {w}", field.name),
    };
    if v < lo || v > hi {
        bail!(
            "value {v} overflows i{} field {}",
            field.width * 8,
            field.name
        );
    }
    Ok(v.to_le_bytes()[..field.width as usize].to_vec())
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

    fn builtin() -> Descriptor {
        serde_json::from_str(BUILTINS[0]).unwrap()
    }

    fn field(d: &Descriptor, name: &str) -> Field {
        d.field(name).unwrap().clone()
    }

    #[test]
    fn builtin_parses() {
        let d = builtin();
        assert_eq!(d.model_number, 257);
        assert_eq!(d.model, "osc-servo");
        assert!(d.fields.len() > 50);
        let id = d.field("id").unwrap();
        assert_eq!(id.addr, 16);
        assert_eq!(id.access, Access::Rw);
    }

    #[test]
    fn uint_round_trips() {
        let d = builtin();
        let f = field(&d, "response_deadline_us"); // u16
        let bytes = encode(&f, "1000").unwrap();
        assert_eq!(bytes, vec![0xE8, 0x03]);
        assert_eq!(decode(&f, &bytes).unwrap(), "1000");
        // hex input
        assert_eq!(encode(&f, "0x03e8").unwrap(), vec![0xE8, 0x03]);
    }

    #[test]
    fn int_round_trips_and_signs() {
        let d = builtin();
        let f = field(&d, "goal_position"); // i32
        let bytes = encode(&f, "-100000").unwrap();
        assert_eq!(decode(&f, &bytes).unwrap(), "-100000");
        assert_eq!(
            encode(&f, "-0x10").unwrap(),
            (-16i32).to_le_bytes().to_vec()
        );
    }

    #[test]
    fn bool_round_trips() {
        let d = builtin();
        let f = field(&d, "torque_enable");
        assert_eq!(encode(&f, "on").unwrap(), vec![1]);
        assert_eq!(encode(&f, "true").unwrap(), vec![1]);
        assert_eq!(encode(&f, "0").unwrap(), vec![0]);
        assert_eq!(decode(&f, &[1]).unwrap(), "on");
        assert_eq!(decode(&f, &[0]).unwrap(), "off");
    }

    #[test]
    fn enum_by_name_and_number() {
        let d = builtin();
        let f = field(&d, "mode");
        assert_eq!(encode(&f, "PositionPid").unwrap(), vec![1]);
        assert_eq!(encode(&f, "positionpid").unwrap(), vec![1]); // case-insensitive
        assert_eq!(encode(&f, "1").unwrap(), vec![1]);
        assert_eq!(decode(&f, &[1]).unwrap(), "PositionPid (1)");
        // off-registry discriminant falls back to the raw number
        assert_eq!(decode(&f, &[7]).unwrap(), "7");
    }

    #[test]
    fn bytes_round_trips() {
        let d = builtin();
        let f = field(&d, "words"); // 64 B
        let hexstr = "aa bb ".to_string() + &"00 ".repeat(62);
        let bytes = encode(&f, &hexstr).unwrap();
        assert_eq!(bytes.len(), 64);
        assert_eq!(bytes[0], 0xaa);
        assert!(decode(&f, &bytes).unwrap().starts_with("aa bb"));
    }

    #[test]
    fn overflow_rejects() {
        let d = builtin();
        let f = field(&d, "response_deadline_us"); // u16
        assert!(encode(&f, "70000").is_err());
        let g = field(&d, "goal_effort"); // i16
        assert!(encode(&g, "40000").is_err());
    }

    #[test]
    fn enum_high_discriminant_stays_unsigned() {
        let f = Field {
            name: "grade".into(),
            addr: 0,
            width: 1,
            access: Access::Rw,
            kind: Kind::Enum,
            variants: vec![Variant {
                name: "Hi".into(),
                value: 200,
            }],
        };
        assert_eq!(encode(&f, "Hi").unwrap(), vec![200]);
        assert_eq!(encode(&f, "200").unwrap(), vec![200]);
        assert_eq!(decode(&f, &[200]).unwrap(), "Hi (200)");
    }

    #[test]
    fn enum_unknown_name_rejects() {
        let d = builtin();
        let f = field(&d, "mode");
        assert!(encode(&f, "Nonsense").is_err());
    }

    #[test]
    fn ro_set_rejects() {
        let d = builtin();
        let f = field(&d, "present_position"); // ro
        let err = encode(&f, "0").unwrap_err().to_string();
        assert!(err.contains("read-only"));
    }

    #[test]
    fn bytes_wrong_length_rejects() {
        let d = builtin();
        let f = field(&d, "words"); // 64 B
        assert!(encode(&f, "aabb").is_err());
    }

    #[test]
    fn field_miss_suggests() {
        let d = builtin();
        let err = d.field("goal_pos").unwrap_err().to_string();
        assert!(err.contains("goal_position"));
    }

    // Selection tests build the registry from the built-in only: Registry::load
    // would read the developer's real external descriptor dir.
    #[test]
    fn select_exact_match() {
        let reg = Registry {
            descriptors: vec![builtin()],
        };
        let (d, note) = reg.select(257, 1).unwrap();
        assert_eq!(d.model_number, 257);
        assert!(note.is_none());
    }

    #[test]
    fn select_fw_mismatch_warns() {
        let reg = Registry {
            descriptors: vec![builtin()],
        };
        let (_, note) = reg.select(257, 9).unwrap();
        assert!(note.unwrap().contains("fw"));
    }

    #[test]
    fn select_model_zero_single_descriptor() {
        // The built-in set is a single descriptor, so model 0 resolves to it.
        let reg = Registry {
            descriptors: vec![builtin()],
        };
        let (d, note) = reg.select(0, 1).unwrap();
        assert_eq!(d.model_number, 257);
        assert!(note.unwrap().contains("unassigned"));
    }

    #[test]
    fn select_unknown_model_bails() {
        let reg = Registry {
            descriptors: vec![builtin()],
        };
        let err = reg.select(0x0999, 1).unwrap_err().to_string();
        assert!(err.contains("known"));
        assert!(err.contains("osc-servo"));
    }
}
