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
    /// The bridge's shortest pulse, timer ticks of drive width: a board
    /// constant compiled into the image, with no register.
    #[serde(default)]
    pub bridge_min_ticks: Option<f64>,
    /// The bridge's slow-decay edges a shunt burst is read by.
    #[serde(default)]
    pub burst_edges: Option<BurstEdges>,
    /// What the window-term fit reads a crest shunt sample by.
    #[serde(default)]
    pub window_read: Option<WindowRead>,
}

/// The crest sample's place, the loop the winding decays through between
/// pulses, and the amplifier's settle gain by drive width: board facts
/// with no register, the last one compiled into the image.
#[derive(Clone, Debug, Deserialize)]
pub struct WindowRead {
    /// Where the crest shunt sample closes after the crest, timer ticks.
    pub sample_ticks: f64,
    /// The low-side loop of the bridge's brake state.
    pub r_brake_ohm: f64,
    pub settle_start_ticks: u16,
    /// Q15 gains, one per 8-tick band of drive width from the start.
    pub settle_q15: Vec<u16>,
}

/// The bridge's edges in timer ticks after the commanded ones: where the
/// current turns on and where the driven tap does, where both turn off,
/// and how long the shunt takes to settle from the on edge.
#[derive(Copy, Clone, Debug, Deserialize)]
pub struct BurstEdges {
    pub on_ticks: f64,
    pub tap_on_ticks: f64,
    pub off_ticks: f64,
    pub settle_ticks: f64,
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
