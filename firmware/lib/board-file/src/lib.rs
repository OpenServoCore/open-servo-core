//! A servo board's `board.toml`, written as the `BoardConfig` literal its
//! app passes to `run!` (driver-pattern sec 2.6).
//!
//! The Rust types are the schema. This crate checks the file's form only
//! (integers, strings that name a variant, arrays where the config holds a
//! pair or a table) and leaves every other check to rustc on the literal:
//! an unknown or missing field, a value over its type's range, a pin that
//! does not exist. Every value rule is a const fn on its type, run in
//! `run!`. Each emitted line ends in `// <file>:<line> <key>`, which rustc's
//! snippet prints, so its errors name the key and its line.
//!
//! The literal is one expression with no arithmetic: every conversion stays
//! in the core's const fns. The same file feeds the sim's mirror of the
//! board ([`emit_sim_mirror`]), so the sim and the image seed one board.

use std::fmt;
use std::fmt::Write as _;
use std::path::Path;

use toml::Spanned;
use toml::de::{DeTable, DeValue};

const PRELUDE: &str = "::osc_servo_ch32::prelude";
const CORE: &str = "::osc_servo_core";
const CORE_CONFIG: &str = "::osc_servo_core::regions::config";

/// What the build script writes to OUT_DIR.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Generated {
    /// The `BoardConfig` literal, one expression.
    pub config: String,
    /// The tinyboot compatibility key, one string literal: the app and the
    /// bootloader both take it from `[board] compat`.
    pub compat: String,
}

/// A board file the generator refuses, as `<file>:<line>:<col>: <key>: <rule>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    pub file: String,
    pub at: Option<(usize, usize)>,
    pub key: Option<String>,
    pub rule: String,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.file)?;
        if let Some((line, col)) = self.at {
            write!(f, ":{line}:{col}")?;
        }
        if let Some(key) = &self.key {
            write!(f, ": {key}")?;
        }
        write!(f, ": {}", self.rule)
    }
}

impl std::error::Error for Error {}

/// Reads `path` and writes its literal and compatibility key.
pub fn emit_board_config(path: &Path) -> Result<Generated, Error> {
    let (file, text) = read(path)?;
    generate(&text, &file)
}

/// [`emit_board_config`] over text already read; `file` names it in the
/// line comments and the errors.
pub fn generate(text: &str, file: &str) -> Result<Generated, Error> {
    let doc = Doc {
        text,
        file,
        types: PRELUDE,
        consts: PRELUDE,
    };
    let root = doc.parse()?;
    let root = root.get_ref();

    let mut compat = None;
    let mut fields = Vec::new();
    for (k, v) in in_file_order(root) {
        let name = k.get_ref().as_ref();
        if name != "board" {
            fields.push(Field::new(k, v, name.to_owned(), name.to_owned()));
            continue;
        }
        let DeValue::Table(board) = v.get_ref() else {
            return Err(doc.err(k.span().start, "board", "[board] is a table"));
        };
        for (bk, bv) in in_file_order(board) {
            let bname = bk.get_ref().as_ref();
            let key = format!("board.{bname}");
            if bname == "compat" {
                let DeValue::String(s) = bv.get_ref() else {
                    return Err(doc.err(
                        bv.span().start,
                        &key,
                        "the tinyboot compatibility key is a string",
                    ));
                };
                compat = Some(format!("{:?}", s.as_ref()));
            } else {
                fields.push(Field::new(bk, bv, key, bname.to_owned()));
            }
        }
    }
    let compat = compat.ok_or_else(|| Error {
        file: file.to_owned(),
        at: Some((1, 1)),
        key: Some("board.compat".to_owned()),
        rule: "missing: the app and the bootloader take the tinyboot compatibility key from it"
            .to_owned(),
    })?;

    let mut config = format!("{PRELUDE}::BoardConfig {{\n");
    doc.emit_struct(&mut config, "", &fields, 0)?;
    config.push_str("}\n");
    Ok(Generated { config, compat })
}

/// Reads `path` and writes the sim's mirror of the board: items named
/// `<prefix>_SENSE`, `_SENSE_EXT`, `_PROTECTION`, `_BEMF_MIN_TICKS`,
/// `_I_SETTLE_GAIN` and `_DEFAULTS` over the core's host types.
pub fn emit_sim_mirror(path: &Path, prefix: &str) -> Result<String, Error> {
    let (file, text) = read(path)?;
    sim_mirror(&text, &file, prefix)
}

/// [`emit_sim_mirror`] over text already read. The file passes the image's
/// form checks first, so the sim never takes a file the image refuses.
/// The mapping onto CALIB's fields is stated here once; the bias count is
/// the core's own const fn, so the mirror holds no arithmetic either.
pub fn sim_mirror(text: &str, file: &str, prefix: &str) -> Result<String, Error> {
    generate(text, file)?;
    let doc = Doc {
        text,
        file,
        types: CORE,
        consts: CORE_CONFIG,
    };
    let root = doc.parse()?;
    let m = Mirror {
        doc: &doc,
        root: root.get_ref(),
    };
    let mut out = String::new();

    out.push_str("/// The board's sense chain, as CALIB publishes it.\n");
    let _ = writeln!(
        out,
        "pub const {prefix}_SENSE: {CORE}::CalibSense = {CORE}::CalibSense {{"
    );
    m.leaf(&mut out, "shunt_r_mohm", "calibration.shunt_r_mohm")?;
    m.leaf(&mut out, "gain_milli", "wiring.current_sense.gain_milli")?;
    m.leaf(
        &mut out,
        "vmotor_div_top",
        "calibration.vmotor_divider.top_ohm",
    )?;
    m.leaf(
        &mut out,
        "vmotor_div_bot",
        "calibration.vmotor_divider.bot_ohm",
    )?;
    m.leaf(&mut out, "vdd_mv", "calibration.vdd_mv")?;
    let _ = writeln!(out, "    tick_hz: {CORE}::kernel::TICK_HZ as u16,");
    m.leaf(
        &mut out,
        "i_window_min_ticks",
        "calibration.i_window_min_ticks",
    )?;
    m.leaf(
        &mut out,
        "v_window_min_ticks",
        "calibration.v_window_min_ticks",
    )?;
    out.push_str("};\n\n");

    out.push_str("/// The rest of the board's CALIB facts.\n");
    let _ = writeln!(
        out,
        "pub const {prefix}_SENSE_EXT: {CORE}::CalibSenseExt = {CORE}::CalibSenseExt {{"
    );
    m.leaf(
        &mut out,
        "vbus_div_top_ohm",
        "calibration.vbus_divider.top_ohm",
    )?;
    m.leaf(
        &mut out,
        "vbus_div_bot_ohm",
        "calibration.vbus_divider.bot_ohm",
    )?;
    m.leaf(&mut out, "ntc_pullup_ohm", "calibration.ntc.pullup_ohm")?;
    m.leaf(&mut out, "ntc_r25_ohm", "calibration.ntc.r25_ohm")?;
    m.leaf(&mut out, "ntc_beta", "calibration.ntc.beta")?;
    let _ = writeln!(
        out,
        "    vmotor_bias_nom_counts: {CORE_CONFIG}::vmotor_counts("
    );
    for key in [
        "calibration.vdd_mv",
        "calibration.vmotor_bias.top_ohm",
        "calibration.vmotor_bias.bot_ohm",
        "calibration.vdd_mv",
    ] {
        let (v, tag) = m.value(key)?;
        let _ = writeln!(out, "        {v}, {tag}");
    }
    out.push_str("    ),\n");
    m.leaf(&mut out, "rail_drop_mv", "calibration.rail_drop_mv")?;
    m.leaf(
        &mut out,
        "vmotor_bias_top_ohm",
        "calibration.vmotor_bias.top_ohm",
    )?;
    m.leaf(
        &mut out,
        "vmotor_bias_bot_ohm",
        "calibration.vmotor_bias.bot_ohm",
    )?;
    out.push_str("};\n\n");

    out.push_str("/// The board's protection defaults.\n");
    m.table_item(
        &mut out,
        &format!("{prefix}_PROTECTION"),
        &format!("{CORE}::ProtectionDefaults"),
        "protection",
    )?;

    out.push_str(
        "/// The back-EMF observer's window floor, which the board hands the\n\
         /// kernel through `KernelTiming` and not through CALIB.\n",
    );
    let (v, tag) = m.value("calibration.bemf_window_min_ticks")?;
    let _ = writeln!(out, "pub const {prefix}_BEMF_MIN_TICKS: u16 = {v}; {tag}\n");

    out.push_str(
        "/// The shunt amplifier's settle bands, which the board hands the\n\
         /// kernel through `KernelTiming`.\n",
    );
    m.table_item(
        &mut out,
        &format!("{prefix}_I_SETTLE_GAIN"),
        &format!("{CORE}::estimator::window::SettleGain"),
        "calibration.i_settle_gain",
    )?;

    out.push_str("/// The boot seed's comms and travel.\n");
    m.table_item(
        &mut out,
        &format!("{prefix}_DEFAULTS"),
        &format!("{CORE}::ConfigDefaults"),
        "defaults",
    )?;
    Ok(out)
}

/// The leaves the mirror picks by key, rendered as the image's literal
/// renders them.
struct Mirror<'d, 'a, 'i> {
    doc: &'d Doc<'d>,
    root: &'a DeTable<'i>,
}

impl<'a, 'i> Mirror<'_, 'a, 'i> {
    fn get(&self, key: &str) -> Result<(usize, &'a Spanned<DeValue<'i>>), Error> {
        let missing = || Error {
            file: self.doc.file.to_owned(),
            at: Some((1, 1)),
            key: Some(key.to_owned()),
            rule: "missing: the sim's mirror of the board reads it".to_owned(),
        };
        let mut table = self.root;
        let mut keys = key.split('.').peekable();
        while let Some(k) = keys.next() {
            let (name, v) = table.get_key_value(k).ok_or_else(missing)?;
            if keys.peek().is_none() {
                return Ok((name.span().start, v));
            }
            let DeValue::Table(t) = v.get_ref() else {
                return Err(missing());
            };
            table = t;
        }
        Err(missing())
    }

    /// A number or a flag and its line comment.
    fn value(&self, key: &str) -> Result<(String, String), Error> {
        let (at, v) = self.get(key)?;
        let text = match v.get_ref() {
            DeValue::Integer(i) => integer(i).map(|n| n.to_string()),
            DeValue::Boolean(b) => Some(b.to_string()),
            _ => None,
        };
        let text = text.ok_or_else(|| {
            self.doc.err(
                v.span().start,
                key,
                "the sim's mirror takes a number or a flag here",
            )
        })?;
        Ok((text, self.doc.tag(at, key)))
    }

    fn leaf(&self, out: &mut String, field: &str, key: &str) -> Result<(), Error> {
        let (v, tag) = self.value(key)?;
        let _ = writeln!(out, "    {field}: {v}, {tag}");
        Ok(())
    }

    /// A struct the file states field for field, walked as the image's
    /// literal walks it.
    fn table_item(&self, out: &mut String, name: &str, ty: &str, key: &str) -> Result<(), Error> {
        let (at, v) = self.get(key)?;
        let DeValue::Table(t) = v.get_ref() else {
            return Err(self
                .doc
                .err(v.span().start, key, "the sim's mirror takes a table here"));
        };
        let _ = writeln!(
            out,
            "pub const {name}: {ty} = {ty} {{ {}",
            self.doc.tag(at, key)
        );
        let fields = self.doc.children(t, key, key);
        self.doc.emit_struct(out, key, &fields, 0)?;
        out.push_str("};\n\n");
        Ok(())
    }
}

fn read(path: &Path) -> Result<(String, String), Error> {
    let text = std::fs::read_to_string(path).map_err(|e| Error {
        file: path.display().to_string(),
        at: None,
        key: None,
        rule: format!("reading the board file: {e}"),
    })?;
    Ok((file_name(path), text))
}

/// How a key path's value is written.
enum Shape {
    Struct(&'static str),
    Enum(&'static str),
    /// A pair of one enum's variants.
    Tuple(&'static str),
    Slice,
}

/// The generator's one statement of the config's shape, by Rust field
/// path; rustc checks every name here on each build.
fn shape(path: &str) -> Option<Shape> {
    use Shape::*;
    Some(match path {
        "wiring" => Struct("BoardWiring"),
        "wiring.drv_en" => Struct("DrvEn"),
        "wiring.drv_en.pin" => Enum("DigitalPin"),
        "wiring.drv_en.active" | "wiring.stat_led_active" => Enum("Level"),
        "wiring.current_sense" => Struct("CurrentSenseConfig"),
        "wiring.current_sense.opa" => Struct("opa::Config"),
        "wiring.current_sense.opa.pos" => Enum("opa::PositiveInput"),
        "wiring.current_sense.opa.neg" => Enum("opa::NegativeInput"),
        "wiring.current_sense.opa.out" => Enum("opa::Output"),
        "wiring.sensors" => Struct("AdcPins"),
        "wiring.sensors.pos" | "wiring.sensors.vbus" | "wiring.sensors.ntc" => {
            Enum("AnalogChannel")
        }
        "wiring.sensors.vmotor" => Tuple("AnalogChannel"),
        "calibration" => Struct("Calibration"),
        "calibration.vmotor_divider" | "calibration.vbus_divider" | "calibration.vmotor_bias" => {
            Struct("Divider")
        }
        "calibration.ntc" => Struct("Ntc"),
        "calibration.i_settle_gain" => Struct("SettleGain"),
        "calibration.i_settle_gain.q15" => Slice,
        "defaults" => Struct("ConfigDefaults"),
        "defaults.baud" => Enum("BaudRate"),
        "protection" => Struct("ProtectionDefaults"),
        _ => return None,
    })
}

/// Leaves the generator writes from core and protocol constants, by the
/// struct path that holds them: the table-ABI model is no PCB fact, and the
/// response deadline is a protocol default.
const OWNED: [(&str, &str, &str); 2] = [
    ("", "model", "MODEL_OSC_SERVO"),
    (
        "defaults",
        "response_deadline_us",
        "DEFAULT_RESPONSE_DEADLINE_US",
    ),
];

struct Field<'a, 'i> {
    span: usize,
    value: &'a Spanned<DeValue<'i>>,
    /// The dotted TOML key, for the comments and the errors.
    key: String,
    /// The Rust field path, for the shape map; `[board]`'s keys are the
    /// root struct's fields.
    path: String,
}

impl<'a, 'i> Field<'a, 'i> {
    fn new(
        k: &Spanned<std::borrow::Cow<'i, str>>,
        value: &'a Spanned<DeValue<'i>>,
        key: String,
        path: String,
    ) -> Self {
        Self {
            span: k.span().start,
            value,
            key,
            path,
        }
    }

    fn name(&self) -> &str {
        self.path.rsplit('.').next().unwrap_or(&self.path)
    }
}

type Entry<'a, 'i> = (
    &'a Spanned<std::borrow::Cow<'i, str>>,
    &'a Spanned<DeValue<'i>>,
);

fn in_file_order<'a, 'i>(t: &'a DeTable<'i>) -> Vec<Entry<'a, 'i>> {
    let mut v: Vec<_> = t.iter().collect();
    v.sort_by_key(|(k, _)| k.span().start);
    v
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

struct Doc<'t> {
    text: &'t str,
    file: &'t str,
    /// The module the shape map's type names resolve in.
    types: &'t str,
    /// The module the generator-owned constants resolve in.
    consts: &'t str,
}

impl<'t> Doc<'t> {
    fn parse(&self) -> Result<Spanned<DeTable<'t>>, Error> {
        DeTable::parse(self.text).map_err(|e| Error {
            file: self.file.to_owned(),
            at: e.span().map(|s| self.line_col(s.start)),
            key: None,
            rule: e.message().to_owned(),
        })
    }

    fn children<'a, 'i>(&self, t: &'a DeTable<'i>, key: &str, path: &str) -> Vec<Field<'a, 'i>> {
        in_file_order(t)
            .into_iter()
            .map(|(k, v)| {
                let child = k.get_ref().as_ref();
                Field::new(k, v, format!("{key}.{child}"), format!("{path}.{child}"))
            })
            .collect()
    }

    fn line_col(&self, at: usize) -> (usize, usize) {
        let before = &self.text[..at.min(self.text.len())];
        let line = before.matches('\n').count() + 1;
        let col = before.rsplit('\n').next().unwrap_or("").chars().count() + 1;
        (line, col)
    }

    fn err(&self, at: usize, key: &str, rule: &str) -> Error {
        Error {
            file: self.file.to_owned(),
            at: Some(self.line_col(at)),
            key: Some(key.to_owned()),
            rule: rule.to_owned(),
        }
    }

    fn tag(&self, at: usize, key: &str) -> String {
        format!("// {}:{} {key}", self.file, self.line_col(at).0)
    }

    fn emit_struct(
        &self,
        out: &mut String,
        path: &str,
        fields: &[Field<'_, '_>],
        depth: usize,
    ) -> Result<(), Error> {
        let pad = "    ".repeat(depth + 1);
        for f in fields {
            if let Some((_, _, konst)) = OWNED
                .iter()
                .find(|(at, name, _)| *at == path && *name == f.name())
            {
                return Err(self.err(
                    f.span,
                    &f.key,
                    &format!("the generator writes this field from `{konst}`: leave it out"),
                ));
            }
        }
        for f in fields {
            self.emit_field(out, f, &pad, depth)?;
        }
        for (at, name, konst) in OWNED {
            if at == path {
                let _ = writeln!(out, "{pad}{name}: {}::{konst},", self.consts);
            }
        }
        Ok(())
    }

    fn emit_field(
        &self,
        out: &mut String,
        f: &Field<'_, '_>,
        pad: &str,
        depth: usize,
    ) -> Result<(), Error> {
        let name = f.name();
        let tag = self.tag(f.span, &f.key);
        let at = f.value.span().start;
        match (f.value.get_ref(), shape(&f.path)) {
            (DeValue::Table(t), Some(Shape::Struct(ty))) => {
                let _ = writeln!(out, "{pad}{name}: {}::{ty} {{ {tag}", self.types);
                let children = self.children(t, &f.key, &f.path);
                self.emit_struct(out, &f.path, &children, depth + 1)?;
                let _ = writeln!(out, "{pad}}},");
            }
            (DeValue::Table(_), None) => {
                return Err(self.err(at, &f.key, "the board config holds no table by this name"));
            }
            (DeValue::Table(_), Some(_)) => {
                return Err(self.err(at, &f.key, "a table where the board config takes one value"));
            }
            (DeValue::Integer(i), _) => {
                let n = integer(i).ok_or_else(|| {
                    self.err(
                        at,
                        &f.key,
                        "the integer is out of any firmware field's range",
                    )
                })?;
                let _ = writeln!(out, "{pad}{name}: {n}, {tag}");
            }
            (DeValue::Float(_), _) => {
                return Err(self.err(
                    at,
                    &f.key,
                    "integers only: the firmware's units are whole mA, mV, ohm, us, ticks",
                ));
            }
            (DeValue::Boolean(b), _) => {
                let _ = writeln!(out, "{pad}{name}: {b}, {tag}");
            }
            (DeValue::Datetime(_), _) => {
                return Err(self.err(at, &f.key, "a datetime is no board value"));
            }
            (DeValue::String(s), Some(Shape::Enum(ty))) => {
                let v = self.variant(at, &f.key, s)?;
                let _ = writeln!(out, "{pad}{name}: {}::{ty}::{v}, {tag}", self.types);
            }
            (DeValue::String(_), _) => {
                return Err(self.err(
                    at,
                    &f.key,
                    "a string where the board config takes a number or a flag: strings only name a variant",
                ));
            }
            (DeValue::Array(a), Some(Shape::Tuple(ty))) => {
                let mut items = Vec::new();
                for e in a.iter() {
                    let DeValue::String(s) = e.get_ref() else {
                        return Err(self.err(
                            e.span().start,
                            &f.key,
                            "the pair names two variants, as strings",
                        ));
                    };
                    let v = self.variant(e.span().start, &f.key, s)?;
                    items.push(format!("{}::{ty}::{v}", self.types));
                }
                let _ = writeln!(out, "{pad}{name}: ({}), {tag}", items.join(", "));
            }
            (DeValue::Array(a), Some(Shape::Slice)) => {
                let mut items = Vec::new();
                for e in a.iter() {
                    let n = match e.get_ref() {
                        DeValue::Integer(i) => integer(i),
                        _ => None,
                    };
                    let n = n.ok_or_else(|| {
                        self.err(e.span().start, &f.key, "the table holds integers only")
                    })?;
                    items.push(n.to_string());
                }
                let _ = writeln!(out, "{pad}{name}: &[{}], {tag}", items.join(", "));
            }
            (DeValue::Array(_), _) => {
                return Err(self.err(
                    at,
                    &f.key,
                    "an array where the board config takes one value",
                ));
            }
        }
        Ok(())
    }

    fn variant<'s>(&self, at: usize, key: &str, s: &'s str) -> Result<&'s str, Error> {
        let mut chars = s.chars();
        let ident = chars
            .next()
            .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
            && chars.all(|c| c.is_ascii_alphanumeric() || c == '_');
        if ident {
            Ok(s)
        } else {
            Err(self.err(
                at,
                key,
                &format!("`{s}` is no variant name: write it as the firmware spells it"),
            ))
        }
    }
}

fn integer(i: &toml::de::DeInteger<'_>) -> Option<i128> {
    i128::from_str_radix(i.as_str(), i.radix()).ok()
}
