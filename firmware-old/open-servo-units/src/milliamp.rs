//! Current measurement in milliamps.

use crate::macros::impl_unit_int_ops;

/// Current in milliamps (1 LSB = 1 mA)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliAmp(pub i16);

impl_unit_int_ops!(MilliAmp);

impl MilliAmp {
    #[inline]
    pub const fn from_ma(ma: i16) -> Self {
        Self(ma)
    }

    #[inline]
    pub const fn as_ma(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_amps(a: i16) -> Self {
        Self(a * 1000)
    }

    /// Get absolute value of current (for threshold comparisons)
    #[inline]
    pub fn abs(self) -> Self {
        Self(self.0.saturating_abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_abs() {
        assert_eq!(MilliAmp::from_ma(100).abs().as_ma(), 100);
        assert_eq!(MilliAmp::from_ma(-100).abs().as_ma(), 100);
        assert_eq!(MilliAmp::from_ma(0).abs().as_ma(), 0);
    }
}
