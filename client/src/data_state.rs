//! Data state, the host half of the firmware's `data_state` module:
//! `data_flags` names every reason the persisted images and the
//! identified set behind the closed loops are not the servo's own, as
//! information. No reason refuses a mode: the kernel's belt latches
//! `fault::DATA`, on the constants a loop would run on. Bits and codes
//! mirror the servo core (pinned against it by the fake-adapter test); the
//! texts are what an operator sees.

use osc_protocol::wire::Id;

use crate::client::Client;
use crate::descriptor::Descriptor;
use crate::error::{Error, LinkError};
use crate::pipe::Pipe;

/// Boot found both CONFIG slots erased.
pub const CONFIG_VIRGIN: u8 = 1 << 0;
/// Boot found CONFIG bytes that parse under no version; board defaults
/// run, as on a blank servo.
pub const CONFIG_CORRUPT: u8 = 1 << 1;
pub const CALIB_VIRGIN: u8 = 1 << 2;
pub const CALIB_CORRUPT: u8 = 1 << 3;
/// The last checkpoint's recompute differed from `plant_stamp`, or a
/// covered field was written since.
pub const STAMP_MISMATCH: u8 = 1 << 4;
/// `recip_ke_q == 0 || ke_vpc_q == 0` at the last checkpoint.
pub const PLANT_UNSET: u8 = 1 << 5;
/// A CRC-valid CONFIG image of another layout version, handled like a
/// fresh servo.
pub const CONFIG_STALE: u8 = 1 << 6;
pub const CALIB_STALE: u8 = 1 << 7;

/// The reasons a successful SAVE retires, every image reason.
pub const SAVE_CLEARS: u8 =
    CONFIG_VIRGIN | CONFIG_CORRUPT | CALIB_VIRGIN | CALIB_CORRUPT | CONFIG_STALE | CALIB_STALE;

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
    pub const DATA: u8 = 7;

    pub fn name(code: u8) -> Option<&'static str> {
        Some(match code {
            NONE => "none",
            OVER_CURRENT => "over_current",
            OVER_TEMP => "over_temp",
            STALL => "stall",
            POSITION_ERROR => "position_error",
            SENSOR => "sensor",
            UNDER_VOLT => "under_volt",
            DATA => "data",
            _ => return None,
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Reason {
    ConfigCorrupt,
    CalibCorrupt,
    ConfigStale,
    CalibStale,
    ConfigVirgin,
    CalibVirgin,
    StampMismatch,
    PlantUnset,
}

impl Reason {
    /// Every reason, most urgent first: the order a host reports them in.
    pub const ALL: [Reason; 8] = [
        Reason::ConfigCorrupt,
        Reason::CalibCorrupt,
        Reason::ConfigStale,
        Reason::CalibStale,
        Reason::ConfigVirgin,
        Reason::CalibVirgin,
        Reason::StampMismatch,
        Reason::PlantUnset,
    ];

    pub const fn bit(self) -> u8 {
        match self {
            Reason::ConfigCorrupt => CONFIG_CORRUPT,
            Reason::CalibCorrupt => CALIB_CORRUPT,
            Reason::ConfigStale => CONFIG_STALE,
            Reason::CalibStale => CALIB_STALE,
            Reason::ConfigVirgin => CONFIG_VIRGIN,
            Reason::CalibVirgin => CALIB_VIRGIN,
            Reason::StampMismatch => STAMP_MISMATCH,
            Reason::PlantUnset => PLANT_UNSET,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Reason::ConfigCorrupt => "CONFIG_CORRUPT",
            Reason::CalibCorrupt => "CALIB_CORRUPT",
            Reason::ConfigStale => "CONFIG_STALE",
            Reason::CalibStale => "CALIB_STALE",
            Reason::ConfigVirgin => "CONFIG_VIRGIN",
            Reason::CalibVirgin => "CALIB_VIRGIN",
            Reason::StampMismatch => "STAMP_MISMATCH",
            Reason::PlantUnset => "PLANT_UNSET",
        }
    }

    /// What it means and what to do about it. Information: no reason
    /// refuses a mode.
    pub const fn text(self) -> &'static str {
        match self {
            Reason::ConfigCorrupt
            | Reason::CalibCorrupt
            | Reason::ConfigStale
            | Reason::CalibStale
            | Reason::ConfigVirgin
            | Reason::CalibVirgin => {
                "Settings not identified (defaults); run an identification: osc-ident run, then osc-ident save."
            }
            Reason::StampMismatch => {
                "Changed since a tool stamped the set (edited, rebuilt or partly written); it runs as written. osc-ident check compares it with a set."
            }
            Reason::PlantUnset => {
                "Motor not identified (Ke 0): closed loop refused. Run an identification: osc-ident run."
            }
        }
    }
}

/// The reasons set in `flags`, most urgent first.
pub fn reasons(flags: u8) -> Vec<Reason> {
    Reason::ALL
        .into_iter()
        .filter(|r| flags & r.bit() != 0)
        .collect()
}

/// The operator line for the `data` fault: the kernel's belt refused the
/// mode on a constant it cannot run on.
pub const REFUSED: &str = "closed mode refused: Ke, the current cap or the speed ceiling is 0, unset; run an identification: osc-ident run";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataState {
    pub flags: u8,
    pub fault_code: u8,
}

impl DataState {
    pub fn reasons(&self) -> Vec<Reason> {
        reasons(self.flags)
    }

    /// The set reason names, joined.
    pub fn names(&self) -> String {
        self.reasons()
            .iter()
            .map(|r| r.name())
            .collect::<Vec<_>>()
            .join(" | ")
    }

    /// The operator line: the refusal when the kernel's belt latched one,
    /// else the most urgent reason's text. `None` when nothing is wrong.
    pub fn message(&self) -> Option<String> {
        if self.fault_code == fault::DATA {
            return Some(REFUSED.into());
        }
        Some(self.reasons().into_iter().next()?.text().to_string())
    }
}

pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<DataState, Error> {
    let flags = field(d, "data_flags")?;
    let code = field(d, "fault_code")?;
    let lo = flags.addr.min(code.addr);
    let hi = flags.end().max(code.end());
    let b = c.read_span(id, lo, hi).await?;
    let at = |addr: u16| {
        b.get((addr - lo) as usize)
            .copied()
            .ok_or_else(|| Error::Link(LinkError::Desync("short data state".into())))
    };
    Ok(DataState {
        flags: at(flags.addr)?,
        fault_code: at(code.addr)?,
    })
}

fn field<'a>(d: &'a Descriptor, name: &str) -> Result<&'a crate::descriptor::Field, Error> {
    d.field(name)
        .ok_or_else(|| Error::Descriptor(format!("no {name} in {}", d.model)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_bit_is_one_reason_and_reasons_come_most_urgent_first() {
        assert_eq!(Reason::ALL.iter().fold(0, |m, r| m | r.bit()), 0xFF);
        for (i, a) in Reason::ALL.iter().enumerate() {
            for b in &Reason::ALL[i + 1..] {
                assert_ne!(a.bit(), b.bit());
            }
        }
        assert_eq!(reasons(0), []);
        assert_eq!(
            reasons(PLANT_UNSET | CALIB_STALE | STAMP_MISMATCH),
            [
                Reason::CalibStale,
                Reason::StampMismatch,
                Reason::PlantUnset
            ]
        );
        assert_eq!(reasons(0xFF)[0], Reason::ConfigCorrupt);
    }

    #[test]
    fn message_leads_with_the_urgent_reason_and_marks_a_refusal() {
        let s = DataState {
            flags: 0,
            fault_code: fault::NONE,
        };
        assert_eq!(s.message(), None);
        assert_eq!(s.names(), "");
        let s = DataState {
            flags: CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET,
            fault_code: fault::NONE,
        };
        assert_eq!(s.names(), "CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET");
        assert_eq!(s.message().unwrap(), Reason::CalibStale.text());
        for flags in [0, PLANT_UNSET] {
            let s = DataState {
                flags,
                fault_code: fault::DATA,
            };
            assert_eq!(s.message().unwrap(), REFUSED, "{flags:#04x}");
        }
        assert_eq!(fault::name(fault::DATA), Some("data"));
        assert_eq!(fault::name(9), None);
    }

    #[test]
    fn save_clears_every_image_reason() {
        assert_eq!(SAVE_CLEARS, !(STAMP_MISMATCH | PLANT_UNSET));
    }

    /// The bits and codes are the servo core's, not a second opinion.
    #[cfg(feature = "fake-adapter")]
    #[test]
    fn bits_and_codes_are_the_servo_cores() {
        use osc_servo_core::data_state as fw;
        use osc_servo_core::kernel::phases::loops::faults as fwf;
        assert_eq!(CONFIG_VIRGIN, fw::CONFIG_VIRGIN);
        assert_eq!(CONFIG_CORRUPT, fw::CONFIG_CORRUPT);
        assert_eq!(CALIB_VIRGIN, fw::CALIB_VIRGIN);
        assert_eq!(CALIB_CORRUPT, fw::CALIB_CORRUPT);
        assert_eq!(STAMP_MISMATCH, fw::STAMP_MISMATCH);
        assert_eq!(PLANT_UNSET, fw::PLANT_UNSET);
        assert_eq!(CONFIG_STALE, fw::CONFIG_STALE);
        assert_eq!(CALIB_STALE, fw::CALIB_STALE);
        assert_eq!(SAVE_CLEARS, fw::SAVE_CLEARS);
        assert_eq!(fault::NONE, fwf::CODE_NONE);
        assert_eq!(fault::OVER_CURRENT, fwf::CODE_OVER_CURRENT);
        assert_eq!(fault::OVER_TEMP, fwf::CODE_OVER_TEMP);
        assert_eq!(fault::STALL, fwf::CODE_STALL);
        assert_eq!(fault::POSITION_ERROR, fwf::CODE_POSITION_ERROR);
        assert_eq!(fault::SENSOR, fwf::CODE_SENSOR);
        assert_eq!(fault::UNDER_VOLT, fwf::CODE_UNDER_VOLT);
        assert_eq!(fault::DATA, fwf::CODE_DATA);
    }
}
