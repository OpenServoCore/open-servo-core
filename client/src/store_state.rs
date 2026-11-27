//! Stored state and refused modes, the host half of the servo's
//! `status_flags` store bits and its `refused_modes` (protocol sec 5.7):
//! which persisted images boot did not load, whether the table in RAM
//! changed since SAVE, and which modes the kernel's belt refuses on the
//! constants it holds. The store bits are information and refuse nothing;
//! only the belt refuses a mode, and it latches `fault::CONFIG` when one is
//! asked for. Bits and codes mirror the servo core (pinned against it by
//! the fake-adapter test); the texts are what an operator sees.

use osc_protocol::table;
use osc_protocol::wire::Id;

use crate::client::Client;
use crate::descriptor::Descriptor;
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;

pub use osc_protocol::table::{
    STATUS_FLAG_CALIB_DEFAULTS as CALIB_DEFAULTS, STATUS_FLAG_CONFIG_DEFAULTS as CONFIG_DEFAULTS,
    STATUS_FLAG_UNSAVED as UNSAVED, STATUS_SAVE_CLEARS as SAVE_CLEARS,
};

/// `fault_code`: the kernel's latched fault kinds.
pub mod fault {
    pub const NONE: u8 = 0;
    pub const OVER_CURRENT: u8 = 1;
    pub const OVER_TEMP: u8 = 2;
    pub const STALL: u8 = 3;
    pub const POSITION_ERROR: u8 = 4;
    pub const SENSOR: u8 = 5;
    pub const UNDER_VOLT: u8 = 6;
    /// An enable refused by the kernel's belt: Ke, the current cap or the
    /// speed ceiling unset.
    pub const CONFIG: u8 = 7;

    pub fn name(code: u8) -> Option<&'static str> {
        Some(match code {
            NONE => "none",
            OVER_CURRENT => "over_current",
            OVER_TEMP => "over_temp",
            STALL => "stall",
            POSITION_ERROR => "position_error",
            SENSOR => "sensor",
            UNDER_VOLT => "under_volt",
            CONFIG => "config",
            _ => return None,
        })
    }
}

/// The operator line for an image on its defaults: whichever it is, and
/// whatever the cause (blank, unreadable or another layout), the action is
/// the same.
pub const DEFAULTS: &str =
    "settings not identified (defaults); run an identification: osc-ident run, then osc-ident save";

/// The operator line for unsaved changes.
pub const NOT_SAVED: &str = "changes not saved: a reboot loses them";

/// The operator line for a refused mode: the belt refuses it on a constant
/// it cannot run on.
pub const REFUSED: &str = "closed mode refused: Ke, the current cap or the speed ceiling is 0, unset; run an identification: osc-ident run";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StoreState {
    /// `status_flags`, whole.
    pub status: u8,
    /// `refused_modes`: bit `mode` set refuses that mode.
    pub refused: u8,
    pub fault_code: u8,
    /// The refused modes by the descriptor's `mode` names, lowest first.
    pub refused_modes: Vec<String>,
}

impl StoreState {
    /// What the servo runs differs from what a reboot loads.
    pub fn unsaved(&self) -> bool {
        self.status & UNSAVED != 0
    }

    /// The images on their defaults, by name.
    pub fn defaults(&self) -> Vec<&'static str> {
        [(CONFIG_DEFAULTS, "CONFIG"), (CALIB_DEFAULTS, "CALIB")]
            .into_iter()
            .filter(|&(bit, _)| self.status & bit != 0)
            .map(|(_, name)| name)
            .collect()
    }

    /// Both images loaded.
    pub fn loaded(&self) -> bool {
        self.status & (CONFIG_DEFAULTS | CALIB_DEFAULTS) == 0
    }

    /// The settings word: `loaded`, or `defaults: CONFIG, CALIB` naming
    /// each image on its defaults.
    pub fn settings(&self) -> String {
        match self.defaults().as_slice() {
            [] => "loaded".into(),
            names => format!("defaults: {}", names.join(", ")),
        }
    }

    /// Whether the belt refuses `mode` (its `mode` value) now.
    pub fn refuses(&self, mode: u8) -> bool {
        mode < 8 && self.refused >> mode & 1 != 0
    }

    /// The refused modes, joined, or `none`.
    pub fn refused_said(&self) -> String {
        if self.refused_modes.is_empty() {
            "none".into()
        } else {
            self.refused_modes.join(", ")
        }
    }

    /// The operator lines, most urgent first: a refusal (latched or
    /// standing), the defaults, unsaved changes. Empty when nothing needs
    /// doing.
    pub fn messages(&self) -> Vec<&'static str> {
        let mut out = Vec::new();
        if self.fault_code == fault::CONFIG || self.refused != 0 {
            out.push(REFUSED);
        }
        if !self.loaded() {
            out.push(DEFAULTS);
        }
        if self.unsaved() {
            out.push(NOT_SAVED);
        }
        out
    }
}

/// The refused mode names in `refused`, through the descriptor's `mode`
/// variants (`mode N` for one it does not name).
pub fn mode_names(refused: u8, d: &Descriptor) -> Vec<String> {
    let mode = d.field("mode");
    (0..8u8)
        .filter(|m| refused >> m & 1 != 0)
        .map(|m| {
            mode.and_then(|f| f.variant(m))
                .map_or_else(|| format!("mode {m}"), |v| v.name.clone())
        })
        .collect()
}

pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<StoreState, Error> {
    let refused = field(d, "refused_modes")?;
    let code = field(d, "fault_code")?;
    let lo = table::STATUS_FLAGS;
    let hi = refused.end().max(code.end());
    let b = c.read_span(id, lo, hi).await?;
    let at = |addr: u16| {
        b.get((addr - lo) as usize)
            .copied()
            .ok_or_else(|| Error::Link(LinkError::Desync("short stored state".into())))
    };
    let refused = at(refused.addr)?;
    Ok(StoreState {
        status: at(table::STATUS_FLAGS)?,
        refused,
        fault_code: at(code.addr)?,
        refused_modes: mode_names(refused, d),
    })
}

fn field<'a>(d: &'a Descriptor, name: &str) -> Result<&'a crate::descriptor::Field, Error> {
    d.field(name)
        .ok_or_else(|| Error::Descriptor(format!("no {name} in {}", d.model)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(status: u8, refused: u8, fault_code: u8) -> StoreState {
        StoreState {
            status,
            refused,
            fault_code,
            refused_modes: Vec::new(),
        }
    }

    #[test]
    fn settings_name_each_image_on_its_defaults() {
        assert_eq!(state(0, 0, 0).settings(), "loaded");
        assert_eq!(state(UNSAVED, 0, 0).settings(), "loaded");
        assert_eq!(state(CALIB_DEFAULTS, 0, 0).settings(), "defaults: CALIB");
        assert_eq!(
            state(CONFIG_DEFAULTS | CALIB_DEFAULTS, 0, 0).settings(),
            "defaults: CONFIG, CALIB"
        );
        assert!(state(UNSAVED, 0, 0).unsaved());
        assert!(!state(CONFIG_DEFAULTS, 0, 0).unsaved());
    }

    #[test]
    fn messages_lead_with_the_refusal() {
        assert!(state(0, 0, fault::NONE).messages().is_empty());
        assert_eq!(state(UNSAVED, 0, 0).messages(), [NOT_SAVED]);
        assert_eq!(
            state(CONFIG_DEFAULTS | UNSAVED, 0b1100, 0).messages(),
            [REFUSED, DEFAULTS, NOT_SAVED]
        );
        assert_eq!(state(0, 0, fault::CONFIG).messages(), [REFUSED]);
        assert!(state(0, 0b1100, 0).refuses(3));
        assert!(!state(0, 0b1100, 0).refuses(0));
        assert!(!state(0, 0xFF, 0).refuses(9));
        assert_eq!(fault::name(fault::CONFIG), Some("config"));
        assert_eq!(fault::name(8), None);
    }

    #[test]
    fn save_clears_unsaved_and_both_store_bits() {
        assert_eq!(SAVE_CLEARS, UNSAVED | CONFIG_DEFAULTS | CALIB_DEFAULTS);
        assert_eq!(SAVE_CLEARS, 0b111);
    }

    /// The bits and codes are the servo core's, not a second opinion.
    #[cfg(feature = "fake-adapter")]
    #[test]
    fn bits_and_codes_are_the_servo_cores() {
        use osc_servo_core::kernel::belt;
        use osc_servo_core::kernel::phases::loops::faults as fwf;
        use osc_servo_core::regions::control::Mode;
        assert_eq!(
            belt::KE_UNSET,
            1 << Mode::Velocity as u8 | 1 << Mode::Position as u8
        );
        assert_eq!(
            belt::LIMITS_UNSET,
            belt::KE_UNSET | 1 << Mode::Current as u8
        );
        assert_eq!(fault::NONE, fwf::CODE_NONE);
        assert_eq!(fault::OVER_CURRENT, fwf::CODE_OVER_CURRENT);
        assert_eq!(fault::OVER_TEMP, fwf::CODE_OVER_TEMP);
        assert_eq!(fault::STALL, fwf::CODE_STALL);
        assert_eq!(fault::POSITION_ERROR, fwf::CODE_POSITION_ERROR);
        assert_eq!(fault::SENSOR, fwf::CODE_SENSOR);
        assert_eq!(fault::UNDER_VOLT, fwf::CODE_UNDER_VOLT);
        assert_eq!(fault::CONFIG, fwf::CODE_CONFIG);
    }
}
