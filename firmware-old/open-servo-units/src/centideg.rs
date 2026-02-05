//! Angular position in centidegrees.

use crate::helpers::mul_div_round_i32;
use crate::macros::impl_unit_int_ops;

/// Angle in centidegrees (1 LSB = 0.01°)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentiDeg(pub i16);

impl_unit_int_ops!(CentiDeg);

impl CentiDeg {
    #[inline]
    pub const fn from_cdeg(cdeg: i16) -> Self {
        Self(cdeg)
    }

    #[inline]
    pub const fn as_cdeg(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_deg(deg: i16) -> Self {
        Self(deg * 100)
    }

    #[inline]
    pub const fn as_deg(self) -> i16 {
        self.0 / 100
    }

    /// Convert to milliradians for trig calculations if needed
    #[inline]
    pub fn to_mrad(self) -> i32 {
        // mrad = cdeg * 174.5329 / 1000  (π/180 * 100)
        mul_div_round_i32(self.0 as i32, 175, 1000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_deg() {
        assert_eq!(CentiDeg::from_deg(90).as_cdeg(), 9000);
        assert_eq!(CentiDeg::from_deg(0).as_cdeg(), 0);
        assert_eq!(CentiDeg::from_deg(-45).as_cdeg(), -4500);
    }

    #[test]
    fn test_as_deg() {
        assert_eq!(CentiDeg::from_cdeg(9000).as_deg(), 90);
        assert_eq!(CentiDeg::from_cdeg(9050).as_deg(), 90); // truncates
        assert_eq!(CentiDeg::from_cdeg(-4500).as_deg(), -45);
    }

    #[test]
    fn test_saturating_add() {
        let a = CentiDeg::from_cdeg(i16::MAX - 100);
        let b = CentiDeg::from_cdeg(200);
        assert_eq!((a + b).as_cdeg(), i16::MAX);
    }

    #[test]
    fn test_saturating_sub() {
        let a = CentiDeg::from_cdeg(i16::MIN + 100);
        let b = CentiDeg::from_cdeg(200);
        assert_eq!((a - b).as_cdeg(), i16::MIN);
    }

    #[test]
    fn test_neg() {
        assert_eq!((-CentiDeg::from_cdeg(100)).as_cdeg(), -100);
        assert_eq!((-CentiDeg::from_cdeg(-100)).as_cdeg(), 100);
    }
}
