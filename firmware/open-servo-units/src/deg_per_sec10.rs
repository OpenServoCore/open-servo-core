//! Angular velocity in 0.1 deg/s.

use crate::macros::impl_unit_int_ops;

/// Angular velocity in 0.1 deg/s (1 LSB = 0.1 deg/s)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DegPerSec10(pub i16);

impl_unit_int_ops!(DegPerSec10);

impl DegPerSec10 {
    #[inline]
    pub const fn from_dps10(dps10: i16) -> Self {
        Self(dps10)
    }

    #[inline]
    pub const fn as_dps10(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_dps(dps: i16) -> Self {
        Self(dps * 10)
    }

    #[inline]
    pub const fn as_dps(self) -> i16 {
        self.0 / 10
    }

    #[inline]
    pub fn from_rpm(rpm: i16) -> Self {
        // dps10 = rpm * 60 / 10 = rpm * 6
        Self(rpm.saturating_mul(6))
    }

    #[inline]
    pub fn as_rpm(self) -> i16 {
        // rpm = dps10 * 10 / 60 = dps10 / 6
        self.0 / 6
    }
}
