//! Angular position in centidegrees with i32 backing.

use crate::centideg::CentiDeg;
use core::ops::{Add, Neg, Sub};

/// Angle in centidegrees with i32 backing for overflow-safe internal math.
/// Use CentiDeg (i16) for public APIs and wire formats.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentiDeg32(pub i32);

impl CentiDeg32 {
    /// Create from raw i32 centidegrees
    #[inline]
    pub const fn from_cdeg(cdeg: i32) -> Self {
        Self(cdeg)
    }

    /// Get raw i32 centidegrees value
    #[inline]
    pub const fn as_cdeg(self) -> i32 {
        self.0
    }

    /// Saturate to i16 range
    #[inline]
    pub const fn to_cdeg_i16_sat(self) -> i16 {
        if self.0 > i16::MAX as i32 {
            i16::MAX
        } else if self.0 < i16::MIN as i32 {
            i16::MIN
        } else {
            self.0 as i16
        }
    }

    /// Convert to CentiDeg (i16) with saturation
    #[inline]
    pub const fn to_centi_deg_sat(self) -> CentiDeg {
        CentiDeg(self.to_cdeg_i16_sat())
    }
}

impl From<CentiDeg> for CentiDeg32 {
    #[inline]
    fn from(cd: CentiDeg) -> Self {
        Self(cd.0 as i32)
    }
}

impl From<i32> for CentiDeg32 {
    #[inline]
    fn from(val: i32) -> Self {
        Self(val)
    }
}

impl Add for CentiDeg32 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }
}

impl Sub for CentiDeg32 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }
}

impl Neg for CentiDeg32 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self(self.0.saturating_neg())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_positive_overflow_saturates() {
        let cd32 = CentiDeg32::from_cdeg(40000);
        assert_eq!(cd32.to_cdeg_i16_sat(), i16::MAX);
        assert_eq!(cd32.to_centi_deg_sat().as_cdeg(), i16::MAX);
    }

    #[test]
    fn test_negative_overflow_saturates() {
        let cd32 = CentiDeg32::from_cdeg(-40000);
        assert_eq!(cd32.to_cdeg_i16_sat(), i16::MIN);
        assert_eq!(cd32.to_centi_deg_sat().as_cdeg(), i16::MIN);
    }

    #[test]
    fn test_subtraction_no_wrap() {
        // 30000 - (-30000) = 60000, which exceeds i16 range but fits in i32
        let sp = CentiDeg32::from_cdeg(30000);
        let pv = CentiDeg32::from_cdeg(-30000);
        let err = sp - pv;
        assert_eq!(err.as_cdeg(), 60000);
        assert_eq!(err.to_cdeg_i16_sat(), i16::MAX);
    }

    #[test]
    fn test_roundtrip() {
        let cd = CentiDeg::from_cdeg(9000);
        let cd32 = CentiDeg32::from(cd);
        let cd_back = cd32.to_centi_deg_sat();
        assert_eq!(cd, cd_back);

        let cd = CentiDeg::from_cdeg(-4500);
        let cd32 = CentiDeg32::from(cd);
        let cd_back = cd32.to_centi_deg_sat();
        assert_eq!(cd, cd_back);
    }

    #[test]
    fn test_saturating_arithmetic() {
        let a = CentiDeg32::from_cdeg(i32::MAX - 100);
        let b = CentiDeg32::from_cdeg(200);
        assert_eq!((a + b).as_cdeg(), i32::MAX);

        let a = CentiDeg32::from_cdeg(i32::MIN + 100);
        let b = CentiDeg32::from_cdeg(200);
        assert_eq!((a - b).as_cdeg(), i32::MIN);

        let a = CentiDeg32::from_cdeg(i32::MIN);
        assert_eq!((-a).as_cdeg(), i32::MAX);
    }
}
