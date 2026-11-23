//! The choices that are no measurement, in physical units: a file beside
//! the servo's input file. Code holds no servo's number.

use serde::Deserialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Switch {
    Off,
    On,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum StallResponse {
    Fault,
    Yield,
}

#[derive(Clone, Debug, Default, Deserialize)]
pub struct Policy {
    /// The angle map's value at the clockwise stop.
    pub angle_cw_stop_deg: Option<f64>,
    pub observer_hz: Option<f64>,
    pub current_loop_hz: Option<f64>,
    /// The electromechanical frequency over the speed loop's crossover.
    pub velocity_em_ratio: Option<f64>,
    /// The speed loop's crossover over its PI's zero.
    pub velocity_zero_ratio: Option<f64>,
    pub velocity_shelf_margin_db: Option<f64>,
    /// The speed loop's crossover over the position gain.
    pub position_split: Option<f64>,
    pub i_cont_ma: Option<f64>,
    pub stall_release_ma: Option<f64>,
    pub oc_trip_samples: Option<f64>,
    pub collision_trip: Option<Switch>,
    pub stall_response: Option<StallResponse>,
    pub stall_time_ms: Option<f64>,
    pub cutoff_c: Option<f64>,
    pub duty_max_share: Option<f64>,
    pub v_undervolt_v: Option<f64>,
    /// The supply's source resistance the rail fold is designed for.
    pub rail_source_ohm: Option<f64>,
    pub openloop_zero_brake: Option<Switch>,
    pub brush_term: Option<Switch>,
    pub velocity_limit_deg_s: Option<f64>,
    pub accel_limit_deg_s2: Option<f64>,
    pub stall_omega_max_deg_s: Option<f64>,
    pub pos_error_deg: Option<f64>,
    pub pos_error_time_ms: Option<f64>,
    /// The share of the stops' span each soft limit sits inside its stop.
    pub soft_margin_share: Option<f64>,
    pub hold_band_noise_multiple: Option<f64>,
    pub hold_band_floor_counts: Option<f64>,
    pub sensor_delta_max_counts: Option<f64>,
    pub sensor_bad_samples: Option<f64>,
}
