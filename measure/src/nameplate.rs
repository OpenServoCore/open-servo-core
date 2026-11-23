//! What the user types from the maker's sheet, once per servo type, and
//! the few answers about this unit no measurement gives.

use serde::Deserialize;

#[derive(Clone, Debug, Default, Deserialize)]
pub struct Nameplate {
    pub name: String,
    #[serde(default)]
    pub document: Option<String>,
    pub gear_class: Option<GearClass>,
    /// Motor revolutions per output revolution, counted teeth.
    pub gear_ratio: Option<f64>,
    /// Whether the count rises as the horn turns counterclockwise, facing
    /// it: which stop is the counterclockwise end.
    pub count_rises_ccw: Option<bool>,
    pub operating_v: Option<VoltRange>,
    #[serde(default)]
    pub no_load: Vec<NoLoad>,
    #[serde(default)]
    pub stall: Vec<Stall>,
    /// The voltage the servo will run on: the stated row nearest it binds.
    pub run_v: Option<f64>,
    /// The share of the stated no-load speed the servo may ever turn at,
    /// `[numerator, denominator]`.
    pub derate_speed: Option<[u32; 2]>,
    /// The share any other value of the sheet is taken at.
    pub derate_other: Option<f64>,
}

/// The row the maker states at the stated voltage nearest the one the
/// servo runs on (`run_v`), the lower of two as near, of the rows inside
/// the operating range; with a single operating voltage, the row at it.
/// None when no one row binds.
pub fn stated<T: Copy>(
    op: VoltRange,
    run_v: Option<f64>,
    rows: &[T],
    volts_of: fn(&T) -> f64,
) -> Option<T> {
    let inside: Vec<T> = rows
        .iter()
        .copied()
        .filter(|r| (op.min..=op.max).contains(&volts_of(r)))
        .collect();
    let volts = if op.min == op.max {
        op.min
    } else {
        let run = run_v?;
        let near = |v: f64| ((v - run).abs() * 1e9).round_ties_even();
        inside
            .iter()
            .map(volts_of)
            .min_by(|a, b| near(*a).total_cmp(&near(*b)).then(a.total_cmp(b)))?
    };
    let mut at = inside.iter().filter(|r| volts_of(r) == volts);
    match (at.next(), at.next()) {
        (Some(row), None) => Some(*row),
        _ => None,
    }
}

impl Nameplate {
    /// The no-load row that binds ([`stated`]).
    pub fn speed_row(&self) -> Option<NoLoad> {
        stated(self.operating_v?, self.run_v, &self.no_load, |r| r.at_v)
    }

    /// The stall row that binds ([`stated`]).
    pub fn stall_row(&self) -> Option<Stall> {
        stated(self.operating_v?, self.run_v, &self.stall, |r| r.at_v)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GearClass {
    Unknown,
    Plastic,
    Metal,
}

impl GearClass {
    /// The byte the firmware stores (client calib.rs GearClass).
    pub fn byte(self) -> u8 {
        match self {
            GearClass::Unknown => 0,
            GearClass::Plastic => 1,
            GearClass::Metal => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct VoltRange {
    pub min: f64,
    pub max: f64,
}

/// The sheet's no-load speed as makers state it: `seconds` per `per_deg`.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct NoLoad {
    pub at_v: f64,
    pub seconds: f64,
    pub per_deg: f64,
}

/// A stated stall: the voltage, and the current where the sheet gives one.
#[derive(Clone, Copy, Debug, Deserialize)]
pub struct Stall {
    pub at_v: f64,
    #[serde(default)]
    pub current_a: Option<f64>,
}
