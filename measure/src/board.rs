//! What a board is, as far as the steps and the rules need it: the facts
//! of the PCB the servo publishes in CALIB (protocol sec 5.5, "Board
//! facts"), read off the servo a run measures. Step 0 keeps the words it
//! read in the run's `inputs/board.json`, and every stage after it decodes
//! that file the same way.

use osc_client::descriptor::Descriptor;
use serde::{Deserialize, Serialize};

use crate::fit::window::Settle;
use crate::install::Refused;
use crate::ntc::{self, Parts};
use crate::state::State;

/// A run's board file, beside the other inputs.
pub const FILE: &str = "board.json";

/// What an image older than the board facts block is refused with.
pub const OLDER_IMAGE: &str =
    "this servo's image publishes no board facts (CALIB ntc_site reads 0): flash the current image";

/// The words the board's facts are read from, each named by its register.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Words {
    pub tick_hz: u16,
    pub vdd_mv: u16,
    pub ntc_pullup_ohm: u16,
    pub ntc_r25_ohm: u16,
    pub ntc_beta: u16,
    pub ntc_site: u8,
    pub bridge_min_ticks: u16,
    pub i_settle_start_ticks: u16,
    pub i_settle_q15: Vec<u16>,
    pub oc_trip_ma: u16,
    pub oc_trip_us: u16,
    pub rail_source_mohm: u16,
    pub window_scale_q15: u16,
    pub terminal_offset_q4: i16,
    pub brake_loop_mohm: u16,
    pub crest_sample_ticks: u16,
    pub shunt_on_lag_q4: u16,
    pub tap_on_lag_q4: u16,
    pub off_lag_q4: u16,
    pub shunt_settle_q4: u16,
}

impl Words {
    /// The words as the state read holds them.
    pub fn of(state: &State, d: &Descriptor) -> Result<Self, Refused> {
        let bytes = |name: &str, width: Option<usize>| -> Result<&[u8], Refused> {
            let f = d
                .field(name)
                .ok_or_else(|| Refused(format!("the descriptor names no {name}")))?;
            let b = state
                .bytes(f)
                .ok_or_else(|| Refused(format!("the state read holds no {name}")))?;
            match width {
                Some(n) if b.len() != n => {
                    Err(Refused(format!("{name} is {} B, not {n}", b.len())))
                }
                None if b.len() % 2 != 0 => {
                    Err(Refused(format!("{name} is {} B, not whole words", b.len())))
                }
                _ => Ok(b),
            }
        };
        let u16_of = |name: &str| bytes(name, Some(2)).map(|b| u16::from_le_bytes([b[0], b[1]]));
        Ok(Words {
            tick_hz: u16_of("tick_hz")?,
            vdd_mv: u16_of("vdd_mv")?,
            ntc_pullup_ohm: u16_of("ntc_pullup_ohm")?,
            ntc_r25_ohm: u16_of("ntc_r25_ohm")?,
            ntc_beta: u16_of("ntc_beta")?,
            ntc_site: bytes("ntc_site", Some(1))?[0],
            bridge_min_ticks: u16_of("bridge_min_ticks")?,
            i_settle_start_ticks: u16_of("i_settle_start_ticks")?,
            i_settle_q15: bytes("i_settle_q15", None)?
                .as_chunks::<2>()
                .0
                .iter()
                .map(|w| u16::from_le_bytes(*w))
                .collect(),
            oc_trip_ma: u16_of("oc_trip_ma")?,
            oc_trip_us: u16_of("oc_trip_us")?,
            rail_source_mohm: u16_of("rail_source_mohm")?,
            window_scale_q15: u16_of("window_scale_q15")?,
            terminal_offset_q4: u16_of("terminal_offset_q4")? as i16,
            brake_loop_mohm: u16_of("brake_loop_mohm")?,
            crest_sample_ticks: u16_of("crest_sample_ticks")?,
            shunt_on_lag_q4: u16_of("shunt_on_lag_q4")?,
            tap_on_lag_q4: u16_of("tap_on_lag_q4")?,
            off_lag_q4: u16_of("off_lag_q4")?,
            shunt_settle_q4: u16_of("shunt_settle_q4")?,
        })
    }
}

/// `inputs/board.json`: the words step 0 read, and the servo it read them
/// off by the name its run folder carries.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Record {
    pub servo: String,
    pub registers: Words,
}

impl Record {
    pub fn render(&self) -> String {
        let mut text = serde_json::to_string_pretty(self).expect("a record serializes");
        text.push('\n');
        text
    }

    pub fn board(&self) -> Result<Board, Refused> {
        Board::decode(&self.registers)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Board {
    pub tick_hz: f64,
    /// The nominal chip supply: captures are matched to their board by it,
    /// so a measured supply is reported and never written.
    pub vdd_mv: f64,
    pub oc_trip_ma: f64,
    /// The overcurrent detector's window, us.
    pub oc_trip_us: f64,
    /// The supply's source resistance the rail fold is designed for: the
    /// board's default, which the image seeds the fold's slope from.
    pub rail_source_mohm: f64,
    pub ntc_site: NtcSite,
    /// The board NTC's fixed-point quadratic, designed from CALIB's NTC
    /// parts as the rest step designs it.
    pub ntc_curve: NtcCurve,
    /// The bridge's shortest pulse, timer ticks of drive width; none where
    /// the board states it unmeasured.
    pub bridge_min_ticks: Option<f64>,
    /// The amplifier's settle gain by drive width.
    pub settle: Settle,
    /// The board's stage 1 characterization; none where it states none.
    pub characterization: Option<Characterization>,
}

/// What stage 1 measured on the board: the terms a burst and the window
/// term are read through.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Characterization {
    /// The window's own `V_on / i` over the 100 % one, which a seated
    /// burst's R and L are divided by.
    pub window_scale: f64,
    /// The tap difference at zero volts in the driven frame, counts: a
    /// seated burst's volts are read over it.
    pub terminal_offset_counts: f64,
    pub burst_edges: BurstEdges,
    pub window_read: WindowRead,
}

impl Board {
    /// The board the servo publishes, refused on an image older than the
    /// block.
    pub fn of(state: &State, d: &Descriptor) -> Result<Self, Refused> {
        Self::decode(&Words::of(state, d)?)
    }

    /// The board the words give: the Q15, Q4 and mOhm words in their units.
    pub fn decode(w: &Words) -> Result<Self, Refused> {
        let ntc_site = match w.ntc_site {
            0 => return Err(Refused(OLDER_IMAGE.into())),
            1 => NtcSite::Air,
            2 => NtcSite::Can,
            n => {
                return Err(Refused(format!(
                    "CALIB ntc_site reads {n}, a site this tool does not know (1 air, 2 can)"
                )));
            }
        };
        let parts = Parts {
            pullup_ohm: w.ntc_pullup_ohm as f64,
            r25_ohm: w.ntc_r25_ohm as f64,
            beta: w.ntc_beta as f64,
        };
        let c = ntc::design(&parts, &ntc::STAGE_1)
            .map_err(|e| Refused(format!("the board's NTC curve: {e}")))?;
        let q4 = |v: f64| v / 16.0;
        let characterization = (w.window_scale_q15 != 0).then(|| Characterization {
            window_scale: w.window_scale_q15 as f64 / 32768.0,
            terminal_offset_counts: q4(w.terminal_offset_q4 as f64),
            burst_edges: BurstEdges {
                on_ticks: q4(w.shunt_on_lag_q4 as f64),
                tap_on_ticks: q4(w.tap_on_lag_q4 as f64),
                off_ticks: q4(w.off_lag_q4 as f64),
                settle_ticks: q4(w.shunt_settle_q4 as f64),
            },
            window_read: WindowRead {
                sample_ticks: w.crest_sample_ticks as f64,
                r_brake_ohm: w.brake_loop_mohm as f64 / 1000.0,
            },
        });
        Ok(Board {
            tick_hz: w.tick_hz as f64,
            vdd_mv: w.vdd_mv as f64,
            oc_trip_ma: w.oc_trip_ma as f64,
            oc_trip_us: w.oc_trip_us as f64,
            rail_source_mohm: w.rail_source_mohm as f64,
            ntc_site,
            ntc_curve: NtcCurve {
                raw_ref: c.raw_ref as f64,
                t_ref_cc: c.t_ref_cc as f64,
                k1_q88: c.k1_q88 as f64,
                k2_q24: c.k2_q24 as f64,
            },
            bridge_min_ticks: (w.bridge_min_ticks != 0).then_some(w.bridge_min_ticks as f64),
            // 0 ends the table in CALIB; past it the gain is unity
            settle: Settle {
                start_ticks: w.i_settle_start_ticks,
                q15: w
                    .i_settle_q15
                    .iter()
                    .copied()
                    .take_while(|&g| g != 0)
                    .collect(),
            },
            characterization,
        })
    }

    /// The characterization, or the refusal of the reading `what` that
    /// needs it.
    pub fn stage_1(&self, what: &str) -> Result<&Characterization, Refused> {
        self.characterization.as_ref().ok_or_else(|| {
            Refused(format!(
                "the servo's board publishes no stage 1 characterization (CALIB window_scale_q15 reads 0): {what}"
            ))
        })
    }

    /// What a tuning policy resolves from.
    pub fn facts(&self) -> osc_ident_inputs::BoardFacts {
        osc_ident_inputs::BoardFacts {
            tick_hz: self.tick_hz,
            oc_trip_us: Some(self.oc_trip_us),
        }
    }
}

/// The crest sample's place and the loop the winding decays through
/// between pulses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowRead {
    /// Where the crest shunt sample closes after the crest, timer ticks.
    pub sample_ticks: f64,
    /// The low-side loop of the bridge's brake state.
    pub r_brake_ohm: f64,
}

/// The bridge's edges in timer ticks after the commanded ones: where the
/// current turns on and where the driven tap does, where both turn off,
/// and how long the shunt takes to settle from the on edge.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BurstEdges {
    pub on_ticks: f64,
    pub tap_on_ticks: f64,
    pub off_ticks: f64,
    pub settle_ticks: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NtcSite {
    /// In room air: the winding estimate needs a body node over it.
    Air,
    /// On the motor can: the NTC is the body node, so the model has none.
    Can,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NtcCurve {
    pub raw_ref: f64,
    pub t_ref_cc: f64,
    pub k1_q88: f64,
    pub k2_q24: f64,
}
