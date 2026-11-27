//! The rules: one per field a host writes past the bus front, each a
//! function that reads only through a [`Ctx`], so a field names every
//! input it read, and a missing input holds the field and names it. One
//! encoder rounds a rule's value into the field's format: a value past the
//! firmware's own bound is stored at it and said, a value its storage
//! cannot hold is refused.

use crate::inputs::{Inputs, Quantity};

pub mod current_loop;
pub mod encode;
pub mod limits;
pub mod motion;
pub mod motor;
pub mod observer;
pub mod speed_loop;
pub mod thermal;
pub mod units;

/// Where a field's value comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    /// A rule over what identification measured.
    Measured,
    /// A fact of the board, from its description file.
    Board,
    /// What the user typed from the maker's sheet.
    Nameplate,
    /// A value the user entered with no measurement behind it.
    User,
    /// A rule over a decision of the policy file.
    Policy,
    /// Tuning for the load at hand: the user's, or off.
    Tuning,
}

/// The storage the firmware declares for a field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Bool,
    /// The variants the field allows, `(name, value)`.
    Enum(&'static [(&'static str, i64)]),
    U8,
    U16,
    I16,
    U32,
    I32,
}

impl Kind {
    pub fn width(self) -> u32 {
        match self {
            Kind::Bool | Kind::Enum(_) | Kind::U8 => 1,
            Kind::U16 | Kind::I16 => 2,
            Kind::U32 | Kind::I32 => 4,
        }
    }

    pub fn signed(self) -> bool {
        matches!(self, Kind::I16 | Kind::I32)
    }

    pub fn range(self) -> (i64, i64) {
        match self {
            Kind::Bool => (0, 1),
            Kind::Enum(variants) => variants
                .iter()
                .fold((i64::MAX, i64::MIN), |(lo, hi), &(_, v)| {
                    (lo.min(v), hi.max(v))
                }),
            Kind::U8 => (0, u8::MAX as i64),
            Kind::U16 => (0, u16::MAX as i64),
            Kind::I16 => (i16::MIN as i64, i16::MAX as i64),
            Kind::U32 => (0, u32::MAX as i64),
            Kind::I32 => (i32::MIN as i64, i32::MAX as i64),
        }
    }
}

/// How a field holds a value: its storage, its fraction bits, and the
/// firmware's own bounds where it sets tighter ones than the storage.
#[derive(Clone, Copy, Debug)]
pub struct Format {
    pub kind: Kind,
    pub q: u32,
    pub min: Option<i64>,
    pub max: Option<i64>,
}

impl Format {
    pub const fn of(kind: Kind) -> Self {
        Format {
            kind,
            q: 0,
            min: None,
            max: None,
        }
    }

    pub const fn q(kind: Kind, q: u32) -> Self {
        Format {
            kind,
            q,
            min: None,
            max: None,
        }
    }

    pub const fn within(kind: Kind, min: Option<i64>, max: Option<i64>) -> Self {
        Format {
            kind,
            q: 0,
            min,
            max,
        }
    }

    /// The values the field takes.
    pub fn bounds(&self) -> (i64, i64) {
        let (lo, hi) = self.kind.range();
        (
            self.min.unwrap_or(lo).max(lo),
            self.max.unwrap_or(hi).min(hi),
        )
    }
}

/// How the encoder rounds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rounding {
    /// Half away from zero.
    Nearest,
    /// Down: a cap derated from a maker's value never rounds past its derate.
    Down,
}

pub type RuleFn = fn(&mut Ctx) -> Result<f64, Fail>;

/// `table![field: "rule", format, "unit", Class, function;]`, a rule a
/// line; `cap` after the function rounds its value down.
macro_rules! table {
    ($($field:ident: $name:literal, $fmt:expr, $unit:expr, $class:ident, $f:ident $($cap:ident)?;)*)
    => {
        &[$(
            $crate::rules::Rule::new(
                stringify!($field), $name, $fmt, $unit, $crate::rules::Class::$class, $f,
            )$(.$cap())?,
        )*]
    };
}

/// What binds one field.
#[derive(Clone, Copy)]
pub struct Rule {
    pub field: &'static str,
    pub name: &'static str,
    pub format: Format,
    pub unit: &'static str,
    pub class: Class,
    pub rounding: Rounding,
    pub value: RuleFn,
}

impl Rule {
    pub const fn new(
        field: &'static str,
        name: &'static str,
        format: Format,
        unit: &'static str,
        class: Class,
        value: RuleFn,
    ) -> Self {
        Rule {
            field,
            name,
            format,
            unit,
            class,
            rounding: Rounding::Nearest,
            value,
        }
    }

    pub const fn cap(self) -> Self {
        Rule {
            rounding: Rounding::Down,
            ..self
        }
    }
}

/// An input a rule reads is not there.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Missing {
    /// The input, as `block.name`.
    pub input: String,
}

/// A value the field cannot hold, or inputs no rule may bind from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refusal {
    pub why: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Fail {
    Missing(Missing),
    Refused(Refusal),
}

impl Fail {
    pub fn missing(input: impl Into<String>) -> Self {
        Fail::Missing(Missing {
            input: input.into(),
        })
    }

    pub fn refused(why: impl Into<String>) -> Self {
        Fail::Refused(Refusal { why: why.into() })
    }
}

/// A bound field: its value as the field holds it, and what gave it.
#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub value: i64,
    pub rule: &'static str,
    pub class: Class,
    pub unit: &'static str,
    /// What the rule read, in the order it read it.
    pub inputs: Vec<&'static str>,
    /// The rule again at each end of every banded input; none with no
    /// banded input.
    pub band: Option<(i64, i64)>,
    /// Where the rule's value cannot be stored or cannot act: what it gave
    /// and the bound it is stored at.
    pub clamp: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Bound(Entry),
    /// The servo keeps what it holds: the named input is not there.
    Held(Missing),
    /// Never written: the value does not fit, or the inputs contradict.
    Refused(Refusal),
}

/// Every field a rule binds.
#[derive(Clone, Debug)]
pub struct Set {
    pub fields: Vec<(&'static str, Outcome)>,
}

impl Set {
    pub fn get(&self, field: &str) -> Option<&Outcome> {
        self.fields
            .iter()
            .find(|(name, _)| *name == field)
            .map(|(_, outcome)| outcome)
    }

    pub fn value(&self, field: &str) -> Option<i64> {
        match self.get(field) {
            Some(Outcome::Bound(e)) => Some(e.value),
            _ => None,
        }
    }

    fn refuse(&mut self, field: &str, why: String) {
        if let Some((_, outcome)) = self.fields.iter_mut().find(|(name, _)| *name == field) {
            *outcome = Outcome::Refused(Refusal { why });
        }
    }

    /// A bound `field` over `to` stored at `to`, band and all, and said.
    fn clamp_down(&mut self, field: &str, to: i64, why: String) {
        if let Some((_, Outcome::Bound(e))) =
            self.fields.iter_mut().find(|(name, _)| *name == field)
            && e.value > to
        {
            e.value = to;
            e.band = e.band.map(|(lo, hi)| (lo.min(to), hi.min(to)));
            e.clamp = Some(match e.clamp.take() {
                Some(was) => format!("{was}; {why}"),
                None => why,
            });
        }
    }

    /// A line for each field stored at a bound: `field: what it gave and
    /// where it is stored`.
    pub fn clamps(&self) -> Vec<String> {
        self.fields
            .iter()
            .filter_map(|(name, o)| match o {
                Outcome::Bound(Entry { clamp: Some(c), .. }) => Some(format!("{name}: {c}")),
                _ => None,
            })
            .collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum End {
    Lo,
    Hi,
}

#[derive(Clone, Copy)]
struct Read {
    name: &'static str,
    banded: bool,
}

/// What a rule reads through: every read is recorded as it is made, and a
/// banded input reads its `lo` or `hi` while the rule is evaluated at that
/// end of its band.
pub struct Ctx<'a> {
    pub(crate) inputs: &'a Inputs,
    reads: Vec<Read>,
    side: Option<(&'static str, End)>,
}

impl<'a> Ctx<'a> {
    fn new(inputs: &'a Inputs) -> Self {
        Ctx {
            inputs,
            reads: Vec::new(),
            side: None,
        }
    }

    /// A quantity, at the end of its band the evaluation asks for.
    pub fn q(
        &mut self,
        name: &'static str,
        get: fn(&Inputs) -> &Option<Quantity>,
    ) -> Result<f64, Fail> {
        let Some(q) = get(self.inputs) else {
            return Err(Fail::missing(name));
        };
        self.reads.push(Read {
            name,
            banded: q.lo.is_some() || q.hi.is_some(),
        });
        Ok(match self.side {
            Some((side, End::Lo)) if side == name => q.lo.unwrap_or(q.value),
            Some((side, End::Hi)) if side == name => q.hi.unwrap_or(q.value),
            _ => q.value,
        })
    }

    /// A value with no band: the policy's, the nameplate's or the user's.
    pub fn get<T>(&mut self, name: &'static str, value: Option<T>) -> Result<T, Fail> {
        let v = value.ok_or_else(|| Fail::missing(name))?;
        self.reads.push(Read {
            name,
            banded: false,
        });
        Ok(v)
    }

    /// A value that is always there: the board's.
    pub fn fact<T>(&mut self, name: &'static str, value: T) -> T {
        self.reads.push(Read {
            name,
            banded: false,
        });
        value
    }
}

/// `f` on `inputs` at its value and at each end of every banded input it
/// reads, the lowest of them: the rules evaluate a field at every end of
/// its inputs' bands, so a value one compares against must sit under
/// each.
pub(crate) fn lowest(inputs: &Inputs, f: RuleFn) -> Result<f64, Fail> {
    let mut c = Ctx::new(inputs);
    let mut low = f(&mut c)?;
    let mut banded: Vec<&'static str> = Vec::new();
    for read in &c.reads {
        if read.banded && !banded.contains(&read.name) {
            banded.push(read.name);
        }
    }
    for name in banded {
        for end in [End::Lo, End::Hi] {
            c.side = Some((name, end));
            low = low.min(f(&mut c)?);
        }
    }
    Ok(low)
}

/// `q!(c, block, name)`: the quantity `inputs.block.name`, read as
/// `block.name`.
macro_rules! q {
    ($c:expr, $block:ident, $name:ident) => {
        $c.q(concat!(stringify!($block), ".", stringify!($name)), |i| {
            &i.$block.$name
        })
    };
}

/// `get!(c, block, name)`: the unbanded `inputs.block.name`.
macro_rules! get {
    ($c:expr, $block:ident, $name:ident) => {{
        let v = $c.inputs.$block.$name.clone();
        $c.get(concat!(stringify!($block), ".", stringify!($name)), v)
    }};
}

/// `board!(c, name)`: the board description's `name`.
macro_rules! board {
    ($c:expr, $name:ident) => {{
        let v = $c.inputs.board.$name;
        $c.fact(concat!("board.", stringify!($name)), v)
    }};
}

pub(crate) use {board, get, q, table};

/// Every rule, by module. An install orders its writes by the descriptor,
/// not by this.
pub fn rules() -> impl Iterator<Item = &'static Rule> {
    [
        motor::RULES,
        observer::RULES,
        current_loop::RULES,
        speed_loop::RULES,
        motion::RULES,
        limits::RULES,
        thermal::RULES,
    ]
    .into_iter()
    .flatten()
}

fn evaluate(rule: &Rule, inputs: &Inputs) -> Outcome {
    let mut c = Ctx::new(inputs);
    let (value, clamp) = match (rule.value)(&mut c).and_then(|x| encode::encode(rule, x)) {
        Ok(v) => v,
        Err(Fail::Missing(m)) => return Outcome::Held(m),
        Err(Fail::Refused(r)) => return Outcome::Refused(r),
    };
    let reads = core::mem::take(&mut c.reads);
    let mut names: Vec<&'static str> = Vec::new();
    let mut banded: Vec<&'static str> = Vec::new();
    for read in &reads {
        if !names.contains(&read.name) {
            names.push(read.name);
        }
        if read.banded && !banded.contains(&read.name) {
            banded.push(read.name);
        }
    }
    let mut band: Option<(i64, i64)> = None;
    for name in banded {
        for end in [End::Lo, End::Hi] {
            c.side = Some((name, end));
            let at = match (rule.value)(&mut c) {
                Ok(x) => encode::held(rule, encode::rounded(rule, x)),
                Err(Fail::Missing(m)) => return Outcome::Held(m),
                Err(Fail::Refused(r)) => return Outcome::Refused(r),
            };
            let (lo, hi) = band.unwrap_or((value, value));
            band = Some((lo.min(at), hi.max(at)));
        }
    }
    Outcome::Bound(Entry {
        value,
        rule: rule.name,
        class: rule.class,
        unit: rule.unit,
        inputs: names,
        band,
        clamp,
    })
}

/// The set the rules give from `inputs`: every field, bound, held or
/// refused. A field over the field that bounds it, where it cannot act, is
/// stored at that bound and said ([`limits::bound_over_bounds`]).
pub fn build(inputs: &Inputs) -> Set {
    let mut set = Set {
        fields: rules().map(|r| (r.field, evaluate(r, inputs))).collect(),
    };
    limits::bound_over_bounds(&mut set);
    set
}
