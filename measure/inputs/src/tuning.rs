//! The tuning policy, `osc.tuning-policy/1`: every choice of how to run a
//! servo. A value is a share of something the run knows, resolved per run
//! from the profile and what the run measures, or the same key fixed to an
//! absolute unit, which ties the policy to one servo.

use std::collections::BTreeMap;

use serde::de::IgnoredAny;
use serde::{Deserialize, Serialize};

use crate::policy::{StallResponse, Switch};
use crate::profile::{FieldSource, check_key};
use crate::why;

pub const SCHEMA: &str = "osc.tuning-policy/1";

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(deny_unknown_fields)]
pub struct TuningPolicy {
    pub schema: String,
    /// Keyed by application (`default`, `arm-joints`), not by servo.
    pub key: String,
    #[serde(default)]
    pub revision: u32,
    #[serde(default)]
    pub label: String,
    #[serde(default)]
    pub protection: Protection,
    #[serde(default)]
    pub motion: Motion,
    #[serde(default)]
    pub loops: Loops,
    #[serde(default)]
    pub sensor: Sensor,
    #[serde(default)]
    pub behavior: Behavior,
    /// Each value's reason, by its key (`protection.stall_release`).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub why: BTreeMap<String, String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<FieldSource>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default, deny_unknown_fields)]
pub struct Protection {
    /// The voltage the servo runs on: it picks which of the maker's
    /// ratings applies.
    pub run_v: Option<f64>,
    pub speed_ceiling: Option<SpeedCeiling>,
    pub current_limit: Option<CurrentLimit>,
    #[serde(rename = "yield")]
    pub yield_: Option<Yield>,
    pub plastic_torque_limit_kg_cm: Option<f64>,
    #[serde(default)]
    pub above_plastic_limit: bool,
    pub stall_release: Option<StallRelease>,
    pub stall_response: Option<StallResponse>,
    pub stall_time_ms: Option<f64>,
    pub stall_speed: Option<StallSpeed>,
    pub cutoff_c: Option<f64>,
    pub position_error: Option<PositionError>,
    pub duty_max_share: Option<f64>,
    pub collision_trip: Option<Switch>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default, deny_unknown_fields)]
pub struct Motion {
    pub move_speed: Option<MoveSpeed>,
    pub speed_up: Option<SpeedUp>,
    /// The soft window's width, output deg, centered on 0 deg; none is the
    /// profile's rated travel.
    pub soft_travel_deg: Option<f64>,
    /// A field this tool no longer reads: its presence is refused, naming
    /// `soft_travel_deg`.
    #[serde(rename = "soft_margin_share", skip_serializing)]
    pub(crate) retired_soft_margin_share: Option<IgnoredAny>,
    /// Where the application's 0 deg sits from the servo's center of
    /// travel, output deg counterclockwise; none is the center.
    pub center_offset_deg: Option<f64>,
    /// A field this tool no longer reads: its presence is refused, naming
    /// `center_offset_deg`.
    #[serde(rename = "angle_cw_stop_deg", skip_serializing)]
    pub(crate) retired_angle_cw_stop_deg: Option<IgnoredAny>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default, deny_unknown_fields)]
pub struct Loops {
    pub current_loop: Option<CurrentLoop>,
    pub observer: Option<Observer>,
    pub velocity_em_ratio: Option<f64>,
    pub velocity_zero_ratio: Option<f64>,
    pub velocity_shelf_margin_db: Option<f64>,
    pub position_split: Option<f64>,
    pub hold_band_noise_multiple: Option<f64>,
    pub hold_band_floor_counts: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default, deny_unknown_fields)]
pub struct Sensor {
    pub pot_jump: Option<PotJump>,
    pub bad_samples: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default, deny_unknown_fields)]
pub struct Behavior {
    pub openloop_zero_brake: Option<Switch>,
    pub brush_term: Option<Switch>,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(deny_unknown_fields)]
pub struct PositionError {
    #[serde(default)]
    pub deg: Option<f64>,
    #[serde(default)]
    pub time_ms: Option<f64>,
}

/// The share of the stated no-load speed the servo may ever turn at.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum SpeedCeiling {
    ShareOfNoLoad([u32; 2]),
}

/// The share of the stall current, the stated volts over R.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum CurrentLimit {
    ShareOfStall(f64),
}

/// How hard a stalled horn keeps pushing: a max yield torque (none: the
/// cap on metal, the plastic torque limit on plastic), or a fixed current.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum Yield {
    TorqueKgCm(Option<f64>),
    Ma(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum StallRelease {
    ShareOfYield(f64),
    Ma(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum StallSpeed {
    MultipleOfStillSpeed(f64),
    DegS(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum MoveSpeed {
    ShareOfCeiling(f64),
    DegS(f64),
}

/// The time to reach the move speed, or a fixed acceleration.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum SpeedUp {
    S(f64),
    DegS2(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum CurrentLoop {
    MultipleOfElectricalCorner(f64),
    Hz(f64),
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum Observer {
    MultipleOfMechanicalCorner(f64),
    Hz(f64),
}

/// The sensor screen's bound on one RAIL period's pot step.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "snake_case")]
pub enum PotJump {
    MultipleOfStep4(f64),
    Counts(f64),
}

impl TuningPolicy {
    /// A policy with nothing in it but its key.
    pub fn empty(key: &str) -> Self {
        TuningPolicy {
            schema: SCHEMA.into(),
            key: key.into(),
            revision: 0,
            label: String::new(),
            protection: Protection::default(),
            motion: Motion::default(),
            loops: Loops::default(),
            sensor: Sensor::default(),
            behavior: Behavior::default(),
            why: BTreeMap::new(),
            source: None,
        }
    }

    /// The choices the run cannot start without are there, else refused
    /// naming the first one missing, the consequence first. Any other value
    /// left out leaves its field as the servo holds it.
    pub fn check(&self) -> Result<(), String> {
        check_key("policy", &self.key)?;
        if self.motion.retired_soft_margin_share.is_some() {
            return Err(why::RETIRED_SOFT_MARGIN_SHARE.into());
        }
        if self.motion.retired_angle_cw_stop_deg.is_some() {
            return Err(why::RETIRED_ANGLE_CW_STOP_DEG.into());
        }
        if let Some(deg) = self.motion.center_offset_deg
            && !deg.is_finite()
        {
            return Err(why::center_offset_deg(deg));
        }
        if let Some(deg) = self.motion.soft_travel_deg
            && !why::within_a_turn(deg)
        {
            return Err(why::soft_travel_deg(deg));
        }
        let p = &self.protection;
        if !p.run_v.is_some_and(|v| v.is_finite() && v > 0.0) {
            return Err(why::run_v());
        }
        if p.speed_ceiling
            .is_none_or(|SpeedCeiling::ShareOfNoLoad([n, d])| n == 0 || n > d)
        {
            return Err(why::speed_ceiling());
        }
        if p.current_limit
            .is_none_or(|CurrentLimit::ShareOfStall(s)| !(s > 0.0 && s <= 1.0))
        {
            return Err(why::current_limit());
        }
        if p.plastic_torque_limit_kg_cm
            .is_some_and(|t| !(t.is_finite() && t > 0.0))
        {
            return Err("A plastic torque limit of 0 or under would never let a plastic horn push: the policy's protection.plastic_torque_limit_kg_cm is not over 0; leave it out for none".into());
        }
        if let Some(Yield::TorqueKgCm(Some(t))) = p.yield_
            && !(t.is_finite() && t > 0.0)
        {
            return Err(format!(
                "A yield torque of 0 or under would read as the cap, no fold: the policy's protection.yield is {{\"torque_kg_cm\": {t}}}; state null for the default"
            ));
        }
        let rules = [
            (
                "protection.stall_release",
                match p.stall_release {
                    Some(StallRelease::ShareOfYield(s)) => Some(s),
                    _ => None,
                },
            ),
            (
                "protection.stall_speed",
                match p.stall_speed {
                    Some(StallSpeed::MultipleOfStillSpeed(m)) => Some(m),
                    _ => None,
                },
            ),
            (
                "loops.current_loop",
                match self.loops.current_loop {
                    Some(CurrentLoop::MultipleOfElectricalCorner(m)) => Some(m),
                    _ => None,
                },
            ),
            (
                "loops.observer",
                match self.loops.observer {
                    Some(Observer::MultipleOfMechanicalCorner(m)) => Some(m),
                    _ => None,
                },
            ),
            (
                "sensor.pot_jump",
                match self.sensor.pot_jump {
                    Some(PotJump::MultipleOfStep4(m)) => Some(m),
                    _ => None,
                },
            ),
        ];
        for (field, x) in rules {
            if x.is_some_and(|x| !(x.is_finite() && x > 0.0)) {
                return Err(format!(
                    "A share or a multiple of 0 or under sets nothing the servo can run on: the policy's {field} is {}; state one over 0",
                    x.unwrap_or_default()
                ));
            }
        }
        Ok(())
    }
}
