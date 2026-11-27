//! Device descriptors: the firmware's exported table description
//! (descriptors/<model>/<major>.<minor>.json, format 2) mapping register
//! names to addresses, widths, access and value kinds. Nothing is compiled
//! in: the caller supplies the JSON text (a CLI embeds its copy, a GUI
//! fetches the file for the model and version the servo reports) and this
//! module parses, selects and codes values.

use std::fmt;

use serde::Deserialize;

use osc_protocol::version::unpack_version;

pub const FORMAT: u8 = 2;

#[derive(Debug, Clone, Deserialize)]
pub struct Descriptor {
    pub format: u8,
    pub model: String,
    pub model_number: u16,
    pub firmware_major: u8,
    pub firmware_minor: u8,
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
    pub min: Option<i64>,
    #[serde(default)]
    pub max: Option<i64>,
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

#[derive(Debug)]
pub enum ParseError {
    Json(serde_json::Error),
    Format(u8),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::Json(e) => write!(f, "{e}"),
            ParseError::Format(v) => write!(f, "descriptor format {v}, want {FORMAT}"),
        }
    }
}

impl std::error::Error for ParseError {}

impl Descriptor {
    pub fn parse(json: &str) -> Result<Self, ParseError> {
        let d: Descriptor = serde_json::from_str(json).map_err(ParseError::Json)?;
        if d.format != FORMAT {
            return Err(ParseError::Format(d.format));
        }
        Ok(d)
    }

    pub fn field(&self, name: &str) -> Option<&Field> {
        self.fields.iter().find(|f| f.name == name)
    }
}

impl Field {
    /// Exclusive end address of this field's bytes.
    pub fn end(&self) -> u16 {
        self.addr + self.width
    }

    pub fn variant(&self, value: u8) -> Option<&Variant> {
        self.variants.iter().find(|v| v.value == value)
    }
}

/// The known descriptor set. Each entry names one (model, major.minor)
/// layout; minors of one model coexist so selection can pick the newest at
/// or below what the servo reports.
#[derive(Debug, Default)]
pub struct Registry {
    descriptors: Vec<Descriptor>,
}

#[derive(Debug, Clone, Copy)]
pub enum Selection<'a> {
    Exact(&'a Descriptor),
    /// Same major, older minor: a valid subset of the servo's table.
    OlderMinor(&'a Descriptor),
    /// Model known but no descriptor at or below the servo's major.minor
    /// (different major, or every known minor is newer); `newest` is the
    /// highest version known for the model.
    Incompatible {
        newest: (u8, u8),
    },
    UnknownModel,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a descriptor, replacing one with the same model number and
    /// major.minor.
    pub fn push(&mut self, d: Descriptor) {
        self.descriptors.retain(|e| {
            (e.model_number, e.firmware_major, e.firmware_minor)
                != (d.model_number, d.firmware_major, d.firmware_minor)
        });
        self.descriptors.push(d);
    }

    pub fn iter(&self) -> impl Iterator<Item = &Descriptor> {
        self.descriptors.iter()
    }

    /// Pick the descriptor for a servo-reported (model, packed fw): same
    /// major, largest known minor at or below the servo's. Patch is
    /// irrelevant to the layout.
    pub fn select(&self, model: u16, fw: u16) -> Selection<'_> {
        let (major, minor, _) = unpack_version(fw);
        let mut known = self.descriptors.iter().filter(|d| d.model_number == model);
        let Some(first) = known.next() else {
            return Selection::UnknownModel;
        };
        let mut newest = first;
        let mut best: Option<&Descriptor> = None;
        for d in std::iter::once(first).chain(known) {
            if (d.firmware_major, d.firmware_minor) > (newest.firmware_major, newest.firmware_minor)
            {
                newest = d;
            }
            if d.firmware_major == major
                && d.firmware_minor <= minor
                && best.is_none_or(|b| d.firmware_minor > b.firmware_minor)
            {
                best = Some(d);
            }
        }
        match best {
            Some(d) if d.firmware_minor == minor => Selection::Exact(d),
            Some(d) => Selection::OlderMinor(d),
            None => Selection::Incompatible {
                newest: (newest.firmware_major, newest.firmware_minor),
            },
        }
    }
}

/// A decoded register value; the variant follows the field's [`Kind`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Value {
    Uint(u64),
    Int(i64),
    Bool(bool),
    /// The raw discriminant; [`Field::variant`] names it.
    Enum(u8),
    Bytes(Vec<u8>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    ReadOnly,
    /// The value's variant is not the field's kind.
    Kind,
    /// Scalar width outside {1, 2, 4}.
    Width(u16),
    /// Byte count does not fit the field.
    Length {
        want: u16,
        got: usize,
    },
    Overflow,
}

impl fmt::Display for CodecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CodecError::ReadOnly => write!(f, "read-only"),
            CodecError::Kind => write!(f, "value does not match the field kind"),
            CodecError::Width(w) => write!(f, "unsupported scalar width {w}"),
            CodecError::Length { want, got } => write!(f, "wants {want} B, got {got}"),
            CodecError::Overflow => write!(f, "value overflows the field"),
        }
    }
}

impl std::error::Error for CodecError {}

/// Decode raw table bytes per the field's kind.
pub fn decode(field: &Field, bytes: &[u8]) -> Result<Value, CodecError> {
    Ok(match field.kind {
        Kind::Uint => Value::Uint(scalar_le(field, bytes)?),
        Kind::Int => {
            let raw = scalar_le(field, bytes)?;
            Value::Int(match field.width {
                1 => raw as u8 as i8 as i64,
                2 => raw as u16 as i16 as i64,
                _ => raw as u32 as i32 as i64,
            })
        }
        Kind::Bool => Value::Bool(bytes.first().copied().unwrap_or(0) != 0),
        // Unsigned: a discriminant past 127 must not sign-extend.
        Kind::Enum => {
            Value::Enum(u8::try_from(scalar_le(field, bytes)?).map_err(|_| CodecError::Overflow)?)
        }
        Kind::Bytes => Value::Bytes(bytes.to_vec()),
    })
}

/// Encode a value into exactly `width` table bytes. Only the type width is
/// checked: the servo owns range validation and its reject reply is the
/// truth.
pub fn encode(field: &Field, value: &Value) -> Result<Vec<u8>, CodecError> {
    if field.access == Access::Ro {
        return Err(CodecError::ReadOnly);
    }
    match (field.kind, value) {
        (Kind::Uint, Value::Uint(v)) => write_uint(field, *v),
        (Kind::Int, Value::Int(v)) => {
            let (lo, hi) = match field.width {
                1 => (i8::MIN as i64, i8::MAX as i64),
                2 => (i16::MIN as i64, i16::MAX as i64),
                4 => (i32::MIN as i64, i32::MAX as i64),
                w => return Err(CodecError::Width(w)),
            };
            if *v < lo || *v > hi {
                return Err(CodecError::Overflow);
            }
            Ok(v.to_le_bytes()[..field.width as usize].to_vec())
        }
        (Kind::Bool, Value::Bool(on)) => Ok(vec![*on as u8]),
        (Kind::Enum, Value::Enum(v)) => write_uint(field, *v as u64),
        (Kind::Bytes, Value::Bytes(b)) => {
            if b.len() != field.width as usize {
                return Err(CodecError::Length {
                    want: field.width,
                    got: b.len(),
                });
            }
            Ok(b.clone())
        }
        _ => Err(CodecError::Kind),
    }
}

/// Little-endian unsigned scalar of the field's width.
fn scalar_le(field: &Field, bytes: &[u8]) -> Result<u64, CodecError> {
    let w = field.width as usize;
    if !matches!(w, 1 | 2 | 4) {
        return Err(CodecError::Width(field.width));
    }
    if bytes.len() < w {
        return Err(CodecError::Length {
            want: field.width,
            got: bytes.len(),
        });
    }
    Ok(bytes[..w]
        .iter()
        .enumerate()
        .fold(0u64, |v, (i, &b)| v | ((b as u64) << (8 * i))))
}

fn write_uint(field: &Field, v: u64) -> Result<Vec<u8>, CodecError> {
    let max = match field.width {
        1 => u8::MAX as u64,
        2 => u16::MAX as u64,
        4 => u32::MAX as u64,
        w => return Err(CodecError::Width(w)),
    };
    if v > max {
        return Err(CodecError::Overflow);
    }
    Ok(v.to_le_bytes()[..field.width as usize].to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_protocol::version::pack_version;

    const MINIMAL: &str = r#"{
        "format": 2, "model": "osc-servo", "class": "servo", "model_number": 257,
        "firmware_major": 0, "firmware_minor": 1, "table_size": 1024,
        "generator": "test",
        "fields": [
            {"name": "id", "addr": 16, "width": 1, "access": "rw", "kind": "uint", "min": 1, "max": 249},
            {"name": "goal_position", "addr": 388, "width": 4, "access": "rw", "kind": "int"},
            {"name": "torque_enable", "addr": 384, "width": 1, "access": "rw", "kind": "bool"},
            {"name": "mode", "addr": 385, "width": 1, "access": "rw", "kind": "enum",
             "variants": [{"name": "OpenLoop", "value": 0}, {"name": "Position", "value": 3}]},
            {"name": "words", "addr": 512, "width": 4, "access": "rw", "kind": "bytes"},
            {"name": "pos", "addr": 600, "width": 2, "access": "ro", "kind": "uint"}
        ]
    }"#;

    fn minimal() -> Descriptor {
        Descriptor::parse(MINIMAL).unwrap()
    }

    fn at(model: u16, major: u8, minor: u8) -> Descriptor {
        let mut d = minimal();
        d.model_number = model;
        d.firmware_major = major;
        d.firmware_minor = minor;
        d
    }

    fn version(d: &Descriptor) -> (u8, u8) {
        (d.firmware_major, d.firmware_minor)
    }

    #[test]
    fn parses_format_2() {
        let d = minimal();
        assert_eq!(d.model_number, 257);
        assert_eq!((d.firmware_major, d.firmware_minor), (0, 1));
        let id = d.field("id").unwrap();
        assert_eq!((id.addr, id.width, id.end()), (16, 1, 17));
        assert_eq!((id.min, id.max), (Some(1), Some(249)));
        assert_eq!(id.access, Access::Rw);
        assert!(d.field("goal_pos").is_none());
        assert_eq!(
            d.field("mode").unwrap().variant(3).unwrap().name,
            "Position"
        );
    }

    #[test]
    fn rejects_other_formats() {
        let json = MINIMAL.replacen("\"format\": 2", "\"format\": 1", 1);
        assert!(matches!(
            Descriptor::parse(&json),
            Err(ParseError::Format(1))
        ));
        assert!(matches!(Descriptor::parse("{"), Err(ParseError::Json(_))));
    }

    #[test]
    fn push_replaces_same_layout_only() {
        let mut reg = Registry::new();
        reg.push(at(257, 0, 1));
        reg.push(at(257, 0, 1));
        reg.push(at(257, 0, 2));
        reg.push(at(258, 0, 1));
        assert_eq!(reg.iter().count(), 3);
    }

    #[test]
    fn selection_matrix() {
        let mut reg = Registry::new();
        reg.push(at(257, 1, 0));
        reg.push(at(257, 1, 2));
        reg.push(at(257, 2, 0));
        let sel = |major, minor| reg.select(257, pack_version(major, minor, 9));

        assert!(matches!(sel(1, 2), Selection::Exact(d) if version(d) == (1, 2)));
        assert!(matches!(sel(1, 5), Selection::OlderMinor(d) if version(d) == (1, 2)));
        assert!(matches!(sel(1, 1), Selection::OlderMinor(d) if version(d) == (1, 0)));
        assert!(matches!(sel(2, 0), Selection::Exact(d) if version(d) == (2, 0)));
        assert!(matches!(
            sel(3, 0),
            Selection::Incompatible { newest: (2, 0) }
        ));
        assert!(matches!(
            reg.select(0x0999, pack_version(1, 2, 0)),
            Selection::UnknownModel
        ));
    }

    #[test]
    fn newer_minors_only_is_incompatible() {
        let mut reg = Registry::new();
        reg.push(at(257, 1, 3));
        assert!(matches!(
            reg.select(257, pack_version(1, 1, 0)),
            Selection::Incompatible { newest: (1, 3) }
        ));
    }

    #[test]
    fn uint_round_trips() {
        let d = minimal();
        let f = d.field("pos").unwrap();
        assert_eq!(decode(f, &[0xE8, 0x03]).unwrap(), Value::Uint(1000));
        let f = d.field("id").unwrap();
        assert_eq!(encode(f, &Value::Uint(7)).unwrap(), vec![7]);
        assert_eq!(encode(f, &Value::Uint(256)), Err(CodecError::Overflow));
        assert_eq!(encode(f, &Value::Int(7)), Err(CodecError::Kind));
    }

    #[test]
    fn int_round_trips_and_signs() {
        let d = minimal();
        let f = d.field("goal_position").unwrap();
        let bytes = encode(f, &Value::Int(-100000)).unwrap();
        assert_eq!(bytes, (-100000i32).to_le_bytes());
        assert_eq!(decode(f, &bytes).unwrap(), Value::Int(-100000));
        assert_eq!(encode(f, &Value::Int(1 << 40)), Err(CodecError::Overflow));
    }

    #[test]
    fn bool_round_trips() {
        let d = minimal();
        let f = d.field("torque_enable").unwrap();
        assert_eq!(encode(f, &Value::Bool(true)).unwrap(), vec![1]);
        assert_eq!(decode(f, &[0]).unwrap(), Value::Bool(false));
    }

    #[test]
    fn enum_round_trips_unsigned() {
        let d = minimal();
        let f = d.field("mode").unwrap();
        assert_eq!(encode(f, &Value::Enum(200)).unwrap(), vec![200]);
        assert_eq!(decode(f, &[200]).unwrap(), Value::Enum(200));
    }

    #[test]
    fn bytes_round_trips_exact_length() {
        let d = minimal();
        let f = d.field("words").unwrap();
        let v = Value::Bytes(vec![0xaa, 0xbb, 0, 0]);
        assert_eq!(encode(f, &v).unwrap(), vec![0xaa, 0xbb, 0, 0]);
        assert_eq!(decode(f, &[0xaa, 0xbb, 0, 0]).unwrap(), v);
        assert_eq!(
            encode(f, &Value::Bytes(vec![0xaa])),
            Err(CodecError::Length { want: 4, got: 1 })
        );
    }

    #[test]
    fn ro_and_short_reads_reject() {
        let d = minimal();
        let f = d.field("pos").unwrap();
        assert_eq!(encode(f, &Value::Uint(0)), Err(CodecError::ReadOnly));
        assert_eq!(decode(f, &[1]), Err(CodecError::Length { want: 2, got: 1 }));
    }
}
