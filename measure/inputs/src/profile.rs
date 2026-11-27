//! The servo profile, `osc.servo-profile/1`: the facts about a servo model
//! that no measurement on the bench gives. One profile serves every servo
//! of that model.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::nameplate::{GearClass, NoLoad, VoltRange};
use crate::why;

pub const SCHEMA: &str = "osc.servo-profile/1";

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ServoProfile {
    pub schema: String,
    /// `<maker>-<model>[-<variant>]`: `[a-z0-9-]`, at most 40 characters.
    pub key: String,
    #[serde(default)]
    pub revision: u32,
    #[serde(default)]
    pub model: Model,
    pub gear_class: Option<GearClass>,
    /// Motor revolutions per output revolution, counted teeth.
    pub gear_ratio: Option<f64>,
    /// The count behind `gear_ratio`, `[pinion, wheel]` a stage: documentation.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gear_stages: Vec<[u32; 2]>,
    pub brushes: Option<u32>,
    pub rotor_segments: Option<u32>,
    /// The output's travel from stop to stop, deg.
    pub travel_deg: Option<f64>,
    /// Whether the count rises as the horn turns counterclockwise, facing it.
    pub count_rises_ccw: Option<bool>,
    pub operating_v: Option<VoltRange>,
    #[serde(default)]
    pub no_load: Vec<NoLoad>,
    #[serde(default)]
    pub stall: Vec<StallRow>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub output_stopper: Option<Stopper>,
    /// The winding's heat model while thermal identification is out of the
    /// tool; none leaves the servo's heat values as they are.
    #[serde(default)]
    pub thermal: Option<HeatModel>,
    #[serde(default)]
    pub provenance: Provenance,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    #[serde(default)]
    pub maker: String,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub variant: String,
}

/// The maker's stall rating at a stated voltage: the torque converts a
/// policy's yield torque into current; the current is optional.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StallRow {
    pub at_v: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque_kg_cm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub current_a: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Stopper {
    Plastic,
    Metal,
    None,
}

/// Each node's time constant and its gain over the board NTC per watt of
/// copper loss.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HeatModel {
    pub fast_tau_s: f64,
    pub fast_gain_c_per_w: f64,
    pub slow_tau_s: f64,
    pub slow_gain_c_per_w: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Provenance {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub document: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub atlas: Option<AtlasRef>,
    /// Where a value came from, by field.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub fields: BTreeMap<String, FieldSource>,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AtlasRef {
    pub id: String,
    pub revision: u32,
    pub sha256: String,
}

/// `how` is one of `datasheet`, `counted`, `measured by hand`,
/// `ident run <esig>/<UTC>`, `atlas <id>@<revision>`, `user`, or `file`
/// for a file reference a version 0 `source` block carried.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FieldSource {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub how: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub document: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub file: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub key: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sha1: Option<String>,
}

/// A profile or policy key: what names its file and the set's pair.
pub fn check_key(file: &str, key: &str) -> Result<(), String> {
    let ok = !key.is_empty()
        && key.len() <= 40
        && key
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-');
    if ok {
        Ok(())
    } else {
        Err(format!(
            "The key names the {file}'s file and the set built from it: the {file}'s key {key:?} is not 1 to 40 of a-z, 0-9 and -"
        ))
    }
}

impl ServoProfile {
    /// Every fact the run needs is there, else refused naming the first
    /// one missing, the consequence first. Which rating row binds depends
    /// on the policy's run voltage, so the rows are checked at
    /// [`crate::resolve`].
    pub fn check(&self) -> Result<(), String> {
        check_key("profile", &self.key)?;
        if self.gear_class.is_none() {
            return Err(why::gear_class());
        }
        if !self.gear_ratio.is_some_and(|r| r.is_finite() && r > 0.0) {
            return Err(why::gear_ratio());
        }
        if !self
            .travel_deg
            .is_some_and(|t| t.is_finite() && t > 0.0 && t <= 360.0)
        {
            return Err(why::travel_deg());
        }
        if self.count_rises_ccw.is_none() {
            return Err(why::count_rises_ccw());
        }
        match (self.brushes, self.rotor_segments) {
            (Some(b), Some(s)) if b > 0 && s > 0 && b.checked_mul(s).is_some() => {}
            _ => return Err(why::COUNTS_MISSING.into()),
        }
        if self
            .operating_v
            .is_none_or(|o| !(o.min > 0.0 && o.min <= o.max))
        {
            return Err(why::operating_v());
        }
        if let Some(h) = self.thermal {
            let all = [
                h.fast_tau_s,
                h.fast_gain_c_per_w,
                h.slow_tau_s,
                h.slow_gain_c_per_w,
            ];
            if !all.iter().all(|v| v.is_finite() && *v > 0.0) {
                return Err("The winding temperature estimate needs every heat value over 0: the profile's thermal states one that is not; leave thermal out to keep the servo's own".into());
            }
        }
        Ok(())
    }
}
