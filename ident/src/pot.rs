//! The pot as the kernel reads it. With a table LIVE the kernel controls on
//! linearized counts (core pot_lut.rs): theta_hat, the loops, the limits
//! and every plant constant fitted against pot motion (Ke, friction, B,
//! sigma_theta) live in those counts, so the experiments fit in them too:
//! the same table applied host-side to a polled `pos`, and the kernel's own
//! `pos_lin` where a TEL stream carries it. At the identity both are the
//! raw count, bit for bit, so a servo without a table fits as before.

use crate::frame::TEL_BIT_POS_LIN;
use crate::lut::GridLut;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Pot {
    lut: Option<GridLut>,
}

impl Pot {
    /// No table: the kernel controls on the raw count.
    pub const RAW: Pot = Pot { lut: None };

    /// The table the servo holds LIVE.
    pub fn live(lut: GridLut) -> Self {
        Self { lut: Some(lut) }
    }

    pub fn is_live(&self) -> bool {
        self.lut.is_some()
    }

    pub fn lut(&self) -> Option<&GridLut> {
        self.lut.as_ref()
    }

    /// The counts the kernel controls on for a polled raw sample.
    pub fn counts(&self, raw: u16) -> f64 {
        match &self.lut {
            Some(lut) => lut.counts(raw),
            None => raw as f64,
        }
    }

    /// `mask` plus the linearized field while a table is live, so a stream
    /// carries the kernel's own word instead of a host recomputation.
    pub fn tel_mask(&self, mask: u16) -> u16 {
        if self.is_live() {
            mask | TEL_BIT_POS_LIN
        } else {
            mask
        }
    }

    pub fn label(&self) -> &'static str {
        if self.is_live() { "linearized" } else { "raw" }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::TelFrame;
    use crate::lut::{GRID, KNOTS};

    fn bent() -> GridLut {
        let mut lut = GridLut::IDENTITY;
        for (k, c) in lut.knots.iter_mut().enumerate().take(240).skip(20) {
            *c = ((k as i32 - 20) * 2).min(100) as i16;
        }
        lut
    }

    #[test]
    fn raw_is_the_count_itself_and_live_is_the_tables_word() {
        assert_eq!(Pot::RAW.counts(2421), 2421.0);
        assert!(!Pot::RAW.is_live());
        let pot = Pot::live(bent());
        assert!(pot.is_live());
        assert_eq!(pot.counts(100), 100.0, "identity below the first knot");
        // raw 496 = knot 31 exactly: raw + c[31] = 496 + 22
        assert_eq!(pot.counts(496), 518.0);
        // mid-interval keeps the Q4 fraction: raw 500 sits 4/16 into
        // interval 31 whose gain is 18/16
        assert_eq!(pot.counts(500), 518.0 + 4.0 * 18.0 / 16.0);
        assert_eq!(pot.counts(500), bent().q4(500) as f64 / GRID as f64);
        assert_eq!(Pot::live(GridLut::IDENTITY).counts(2421), 2421.0);
        assert_eq!(KNOTS, 257);
    }

    #[test]
    fn tel_mask_adds_pos_lin_only_while_live() {
        assert_eq!(Pot::RAW.tel_mask(0x1B), 0x1B);
        assert_eq!(Pot::live(GridLut::IDENTITY).tel_mask(0x1B), 0x81B);
        assert_eq!(Pot::RAW.label(), "raw");
        assert_eq!(Pot::live(bent()).label(), "linearized");
    }

    #[test]
    fn frame_counts_prefer_the_kernels_word() {
        let f = TelFrame {
            pos: Some(2000),
            pos_lin: Some(2010 * 16 + 8),
            ..Default::default()
        };
        assert_eq!(f.counts(), Some(2010.5));
        let f = TelFrame {
            pos: Some(2000),
            ..Default::default()
        };
        assert_eq!(f.counts(), Some(2000.0));
        assert_eq!(TelFrame::default().counts(), None);
    }
}
