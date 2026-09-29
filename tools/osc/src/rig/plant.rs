//! The pot table as the kernel applies it, read off the servo for the
//! experiments (osc-ident's `Pot`) and for the records that say which
//! counts a run was fitted in.

use anyhow::Result;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::descriptor::Descriptor;
use osc_client::nusb::NusbPipe;
use osc_client::pot_lut::{INTERVALS, state};
use osc_ident::lut::{GRID, GridLut, KNOTS};
use osc_ident::pot::Pot;
use osc_protocol::crc::osc_crc;

/// The effective table: the array while LIVE, the identity otherwise.
pub(crate) struct Lut {
    pub(crate) state: u8,
    pub(crate) knots: [i16; INTERVALS],
}

impl Lut {
    pub(crate) fn read(c: &mut Client<NusbPipe>, id: Id, d: &Descriptor) -> Result<Self> {
        let s = c.lut_state(id, d)?;
        let knots = if s == state::LIVE {
            c.pot_lut(id, d)?.knots
        } else {
            [0; INTERVALS]
        };
        Ok(Self { state: s, knots })
    }

    pub(crate) fn live(&self) -> bool {
        self.state == state::LIVE
    }

    pub(crate) fn state_name(&self) -> String {
        match state::name(self.state) {
            Some(n) => n.to_string(),
            None => format!("state {}", self.state),
        }
    }

    pub(crate) fn grid(&self) -> GridLut {
        let mut knots = [0i16; KNOTS];
        knots[..INTERVALS].copy_from_slice(&self.knots);
        GridLut { knots }
    }

    /// What the experiments fit through.
    pub(crate) fn pot(&self) -> Pot {
        if self.live() {
            Pot::live(self.grid())
        } else {
            Pot::RAW
        }
    }

    /// CRC-16/ARC over the effective knots LE, the bytes as the stamp
    /// hashes them; the identity has its own value.
    pub(crate) fn crc(&self) -> u16 {
        let bytes: Vec<u8> = self.knots.iter().flat_map(|k| k.to_le_bytes()).collect();
        osc_crc(&bytes)
    }

    pub(crate) fn nonzero(&self) -> usize {
        self.knots.iter().filter(|&&k| k != 0).count()
    }

    /// Raw counts of the first and last nonzero knot.
    pub(crate) fn band(&self) -> Option<(u16, u16)> {
        let first = self.knots.iter().position(|&k| k != 0)?;
        let last = self.knots.iter().rposition(|&k| k != 0)?;
        Some((first as u16 * GRID, last as u16 * GRID))
    }

    pub(crate) fn describe(&self) -> String {
        if !self.live() {
            return format!("{} (the kernel applies the identity)", self.state_name());
        }
        match self.band() {
            Some((lo, hi)) => format!(
                "LIVE (crc {:#06x}, {} nonzero knots at raw {lo}..{hi})",
                self.crc(),
                self.nonzero()
            ),
            None => format!("LIVE (crc {:#06x}, every knot 0)", self.crc()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bent() -> Lut {
        let mut knots = [0i16; INTERVALS];
        for (k, c) in knots.iter_mut().enumerate().take(220).skip(35) {
            *c = (k as i16 - 35) % 7 + 1;
        }
        Lut {
            state: state::LIVE,
            knots,
        }
    }

    #[test]
    fn identity_and_a_table_describe_themselves() {
        let id = Lut {
            state: state::IDENTITY,
            knots: [0; INTERVALS],
        };
        assert_eq!(id.describe(), "IDENTITY (the kernel applies the identity)");
        assert_eq!(id.pot(), Pot::RAW);
        assert_eq!(id.band(), None);
        let rejected = Lut {
            state: state::REJECT_ENDS,
            ..bent()
        };
        assert_eq!(rejected.pot(), Pot::RAW);
        assert!(rejected.describe().starts_with("REJECT_ENDS"));

        let b = bent();
        assert_eq!(b.nonzero(), 185);
        assert_eq!(b.band(), Some((560, 3504)));
        assert!(b.pot().is_live());
        assert_eq!(b.pot().counts(560), 561.0);
        let crc = b.crc();
        assert_ne!(crc, id.crc());
        assert_eq!(
            b.describe(),
            format!("LIVE (crc {crc:#06x}, 185 nonzero knots at raw 560..3504)")
        );
        let zero = Lut {
            state: state::LIVE,
            knots: [0; INTERVALS],
        };
        assert_eq!(
            zero.crc(),
            id.crc(),
            "an all-zero LIVE table hashes like the identity"
        );
        assert!(zero.pot().is_live());
    }
}
