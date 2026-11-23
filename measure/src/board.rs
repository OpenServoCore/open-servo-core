//! What a board is, as far as the rules need it: the facts of the PCB the
//! servo cannot report. One data file per board design; a second board is
//! a second file.

use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
pub struct Board {
    pub name: String,
    pub tick_hz: f64,
    /// The nominal chip supply: captures are matched to their board by it,
    /// so a measured supply is reported and never written.
    pub vdd_mv: f64,
    pub oc_trip_ma: f64,
    pub ntc_site: NtcSite,
    /// The board NTC's fixed-point quadratic as the CALIB words hold it.
    pub ntc_curve: NtcCurve,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum NtcSite {
    /// In room air: the winding estimate needs a body node over it.
    Air,
    /// On the motor can: the NTC is the body node, so the model has none.
    Can,
}

#[derive(Clone, Copy, Debug, Deserialize)]
pub struct NtcCurve {
    pub raw_ref: f64,
    pub t_ref_cc: f64,
    pub k1_q88: f64,
    pub k2_q24: f64,
}
