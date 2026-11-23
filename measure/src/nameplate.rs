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
