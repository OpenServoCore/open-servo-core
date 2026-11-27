//! What the rules read: measured quantities with their band and source,
//! and the values a user enters.

use serde::Deserialize;

use crate::{Board, Nameplate, Policy};

/// A value with its band, its unit and where it came from. A rule is
/// evaluated again at each end of every banded input it reads. `sd` is one
/// sd over the `n` repeats it was taken over; `n` none is a nominal value.
#[derive(Clone, Debug, Deserialize)]
pub struct Quantity {
    pub value: f64,
    #[serde(default)]
    pub lo: Option<f64>,
    #[serde(default)]
    pub hi: Option<f64>,
    #[serde(default)]
    pub sd: Option<f64>,
    #[serde(default)]
    pub n: Option<u32>,
    pub unit: String,
    pub source: Source,
}

#[derive(Clone, Debug, Default, Deserialize)]
pub struct Source {
    pub file: String,
    #[serde(default)]
    pub key: String,
    #[serde(default)]
    pub sha1: Option<String>,
}

/// A reading with no band.
#[derive(Clone, Debug, Deserialize)]
pub struct Reading<T> {
    pub value: T,
    pub source: Source,
}

/// The pot's two mechanical stops, raw counts.
#[derive(Clone, Debug, Deserialize)]
pub struct Stops {
    pub raw_min: i64,
    pub raw_max: i64,
    pub source: Source,
}

/// The position table's scale: its counts per output degree.
#[derive(Clone, Debug, Deserialize)]
pub struct Table {
    pub counts_per_deg: f64,
    pub source: Source,
}

/// Margin sweeps of the speed loop at the shelf's rungs, each with the PI
/// it ran, and the registers of the set the record measured.
#[derive(Clone, Debug, Deserialize)]
pub struct ShelfRecord {
    pub sweeps: Vec<ShelfSweep>,
    pub check: ShelfCheck,
}

#[derive(Clone, Debug, Deserialize)]
pub struct ShelfSweep {
    pub capture: String,
    pub cruise_cps: f64,
    pub v_kp_q88: u16,
    pub v_ki_q412: u16,
    /// `[hz, re, im]`: the speed loop's L at one rung.
    pub points: Vec<[f64; 3]>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct ShelfCheck {
    pub capture: String,
    pub v_kp_q88: u16,
    pub v_ki_q412: u16,
    pub v_kaw_q412: u16,
}

/// What identification measured, the board's sense chain with it (its
/// stage 1 scales, or the parts' nominal on a board nobody measured).
#[derive(Clone, Debug, Default, Deserialize)]
pub struct Measured {
    pub current_counts_per_a: Option<Quantity>,
    pub terminal_counts_per_v: Option<Quantity>,
    pub rail_counts_per_v: Option<Quantity>,
    pub rail_offset_counts: Option<Quantity>,
    /// Positive duty raises the count.
    pub drive_polarity: Option<Reading<bool>>,
    pub stops: Option<Stops>,
    pub table: Option<Table>,
    /// The winding's resistance at `r_board_c`, the board NTC's reading.
    pub r_ohm: Option<Quantity>,
    pub r_board_c: Option<Quantity>,
    pub l_mh: Option<Quantity>,
    pub ke_v_s_per_rad: Option<Quantity>,
    pub j_kg_m2: Option<Quantity>,
    pub g_window_ms: Option<Quantity>,
    pub pot_rest_sd_counts: Option<Quantity>,
    /// The current the velocity loop holds in a cruise: intercept, mA, and
    /// slope, mA per motor rad/s.
    pub cruise_coulomb_ma: Option<Quantity>,
    pub cruise_viscous_ma_s_per_rad: Option<Quantity>,
    /// The current at which a slow driven ramp first moves the horn.
    pub breakaway_ma: Option<Quantity>,
    pub shelf: Option<ShelfRecord>,
    /// The speed gain a closed-loop check settled on, at or under the
    /// starting gain the rules give (`rules::speed_loop::starting_kp`).
    pub speed_kp_verified: Option<f64>,
    /// The largest |omega_hat| over the still holds of a closed-loop check,
    /// count/s ([`crate::screens::still_speed_cps`]).
    pub still_speed_cps: Option<Quantity>,
    /// The worst level 4 successive RAIL-period pot steps all pass over the
    /// run's driven rows, raw counts ([`crate::screens::step_run`]).
    pub pot_step4_counts: Option<Quantity>,
}

/// The winding model, entered by the user: each node's time constant and
/// its gain over the board NTC per watt of copper loss.
#[derive(Clone, Debug, Default, Deserialize)]
pub struct ThermalInputs {
    pub fast_tau_s: Option<Quantity>,
    pub fast_gain_c_per_w: Option<Quantity>,
    pub slow_tau_s: Option<Quantity>,
    pub slow_gain_c_per_w: Option<Quantity>,
}

/// Tuning for the load at hand, never identified: each is 0, off, unless
/// the user sets it.
#[derive(Clone, Debug, Default, Deserialize)]
pub struct Tuning {
    pub pot_steady_counts_per_a: Option<f64>,
    pub pot_fast_counts_per_a: Option<f64>,
}

#[derive(Clone, Debug)]
pub struct Inputs {
    pub board: Board,
    pub measured: Measured,
    pub nameplate: Nameplate,
    pub policy: Policy,
    pub thermal: ThermalInputs,
    pub tuning: Tuning,
}
