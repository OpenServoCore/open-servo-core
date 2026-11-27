//! The choices that are no measurement, in the absolute units the rules
//! read: a version 0 `policy.json` is this shape, and [`crate::resolve`]
//! gives one from a tuning policy. Six values are each one form, a map of
//! one key: the fixed form, or the rule the run binds it by, as the tuning
//! policy states them. Code holds no servo's number.

use core::fmt;
use core::marker::PhantomData;

use serde::de::{self, IgnoredAny, MapAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize};

use crate::tuning::{CurrentLoop, Observer, PotJump, StallRelease, StallSpeed, Yield};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "lowercase")]
pub enum Switch {
    Off,
    On,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(rename_all = "lowercase")]
pub enum StallResponse {
    Fault,
    Yield,
}

impl Switch {
    pub fn name(self) -> &'static str {
        match self {
            Switch::Off => "off",
            Switch::On => "on",
        }
    }
}

impl StallResponse {
    pub fn name(self) -> &'static str {
        match self {
            StallResponse::Fault => "fault",
            StallResponse::Yield => "yield",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[cfg_attr(
    feature = "tsify",
    derive(tsify::Tsify),
    tsify(missing_as_null, hashmap_as_object)
)]
#[serde(default)]
pub struct Policy {
    /// The angle map's value at the clockwise stop.
    pub angle_cw_stop_deg: Option<f64>,
    #[serde(deserialize_with = "form")]
    pub observer: Option<Observer>,
    #[serde(deserialize_with = "form")]
    pub current_loop: Option<CurrentLoop>,
    /// The electromechanical frequency over the speed loop's crossover.
    pub velocity_em_ratio: Option<f64>,
    /// The speed loop's crossover over its PI's zero.
    pub velocity_zero_ratio: Option<f64>,
    pub velocity_shelf_margin_db: Option<f64>,
    /// The speed loop's crossover over the position gain.
    pub position_split: Option<f64>,
    /// The press trigger and Yield fold level.
    #[serde(rename = "yield", deserialize_with = "form")]
    pub yield_: Option<Yield>,
    #[serde(deserialize_with = "form")]
    pub stall_release: Option<StallRelease>,
    /// The overcurrent detector's window, samples: in a version 0 file
    /// only; [`crate::resolve`] takes it from the board's `oc_trip_us`.
    pub oc_trip_samples: Option<f64>,
    pub collision_trip: Option<Switch>,
    pub stall_response: Option<StallResponse>,
    pub stall_time_ms: Option<f64>,
    pub cutoff_c: Option<f64>,
    pub duty_max_share: Option<f64>,
    pub openloop_zero_brake: Option<Switch>,
    pub brush_term: Option<Switch>,
    pub velocity_limit_deg_s: Option<f64>,
    pub accel_limit_deg_s2: Option<f64>,
    #[serde(deserialize_with = "form")]
    pub stall_speed: Option<StallSpeed>,
    pub pos_error_deg: Option<f64>,
    pub pos_error_time_ms: Option<f64>,
    /// The share of the stops' span each soft limit sits inside its stop.
    pub soft_margin_share: Option<f64>,
    pub hold_band_noise_multiple: Option<f64>,
    pub hold_band_floor_counts: Option<f64>,
    #[serde(deserialize_with = "form")]
    pub pot_jump: Option<PotJump>,
    pub sensor_bad_samples: Option<f64>,
}

/// A value of one of the six forms: its two keys, the fixed form's first,
/// and the value each takes, a number (the yield torque's may be null).
pub trait Forms: Sized {
    const KEYS: [&'static str; 2];
    fn of(fixed: bool, v: Option<f64>) -> Option<Self>;
}

macro_rules! forms {
    ($($form:ident: $fixed:ident $fk:literal, $rule:ident $rk:literal;)*) => {$(
        impl Forms for $form {
            const KEYS: [&'static str; 2] = [$fk, $rk];
            fn of(fixed: bool, v: Option<f64>) -> Option<Self> {
                Some(if fixed { $form::$fixed(v?) } else { $form::$rule(v?) })
            }
        }
    )*};
}

forms! {
    StallRelease: Ma "ma", ShareOfYield "share_of_yield";
    StallSpeed: DegS "deg_s", MultipleOfStillSpeed "multiple_of_still_speed";
    PotJump: Counts "counts", MultipleOfStep4 "multiple_of_step4";
    CurrentLoop: Hz "hz", MultipleOfElectricalCorner "multiple_of_electrical_corner";
    Observer: Hz "hz", MultipleOfMechanicalCorner "multiple_of_mechanical_corner";
}

impl Forms for Yield {
    const KEYS: [&'static str; 2] = ["ma", "torque_kg_cm"];
    fn of(fixed: bool, v: Option<f64>) -> Option<Self> {
        if fixed {
            Some(Yield::Ma(v?))
        } else {
            Some(Yield::TorqueKgCm(v))
        }
    }
}

/// `{"<key>": ...}` for each of a form's two keys.
pub fn forms_of<T: Forms>() -> String {
    format!("{{\"{}\": ...}}, {{\"{}\": ...}}", T::KEYS[0], T::KEYS[1])
}

/// A form as its one key states it (`{"hz": 1000}`); null is none, and
/// anything else is refused naming the two forms.
fn form<'de, D, T>(d: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Forms,
{
    struct Of<T>(PhantomData<T>);

    impl<'de, T: Forms> Visitor<'de> for Of<T> {
        type Value = Option<T>;

        fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
            write!(f, "one of {}", forms_of::<T>())
        }

        fn visit_unit<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }

        fn visit_none<E: de::Error>(self) -> Result<Self::Value, E> {
            Ok(None)
        }

        fn visit_some<D: Deserializer<'de>>(self, d: D) -> Result<Self::Value, D::Error> {
            d.deserialize_any(self)
        }

        fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<Self::Value, A::Error> {
            let refused = |what: &str| {
                <A::Error as de::Error>::custom(format!("{what}: it is one of {}", forms_of::<T>()))
            };
            let Some(key) = map.next_key::<String>()? else {
                return Err(refused("an empty map"));
            };
            let v: Option<f64> = map.next_value()?;
            if map.next_key::<IgnoredAny>()?.is_some() {
                return Err(refused(&format!("a map of more keys than {key:?}")));
            }
            let fixed = match T::KEYS.iter().position(|k| *k == key) {
                Some(i) => i == 0,
                None => return Err(refused(&format!("{key:?}"))),
            };
            T::of(fixed, v)
                .map(Some)
                .ok_or_else(|| refused(&format!("{key:?} with no value")))
        }
    }

    d.deserialize_any(Of(PhantomData))
}

/// A version 0 file as it is read: its values, and the six as a file
/// written before the forms names them, each a bare number in the fixed
/// form's unit (one before the yield current's rename names it
/// `i_cont_ma`).
#[derive(Deserialize)]
pub struct PolicyFile {
    #[serde(flatten)]
    policy: Policy,
    #[serde(flatten)]
    unitized: Unitized,
}

#[derive(Default, Deserialize)]
#[serde(default)]
struct Unitized {
    observer_hz: Option<f64>,
    current_loop_hz: Option<f64>,
    #[serde(alias = "i_cont_ma")]
    yield_current_ma: Option<f64>,
    stall_release_ma: Option<f64>,
    stall_omega_max_deg_s: Option<f64>,
    sensor_delta_max_counts: Option<f64>,
}

fn take<T: Forms>(
    slot: &mut Option<T>,
    old: Option<f64>,
    key: &str,
    old_key: &str,
) -> Result<(), String> {
    match (old, slot.is_some()) {
        (Some(_), true) => Err(format!(
            "The policy states {key} twice, as {key} and as {old_key}, and it is unclear which was meant: keep {key}"
        )),
        (Some(v), false) => {
            *slot = T::of(true, Some(v));
            Ok(())
        }
        (None, _) => Ok(()),
    }
}

impl PolicyFile {
    /// The values, each of the six named the old way in its fixed form; one
    /// stated both ways is refused.
    pub fn policy(self) -> Result<Policy, String> {
        let (mut p, u) = (self.policy, self.unitized);
        take(&mut p.observer, u.observer_hz, "observer", "observer_hz")?;
        take(
            &mut p.current_loop,
            u.current_loop_hz,
            "current_loop",
            "current_loop_hz",
        )?;
        take(
            &mut p.yield_,
            u.yield_current_ma,
            "yield",
            "yield_current_ma",
        )?;
        take(
            &mut p.stall_release,
            u.stall_release_ma,
            "stall_release",
            "stall_release_ma",
        )?;
        take(
            &mut p.stall_speed,
            u.stall_omega_max_deg_s,
            "stall_speed",
            "stall_omega_max_deg_s",
        )?;
        take(
            &mut p.pot_jump,
            u.sensor_delta_max_counts,
            "pot_jump",
            "sensor_delta_max_counts",
        )?;
        Ok(p)
    }
}
