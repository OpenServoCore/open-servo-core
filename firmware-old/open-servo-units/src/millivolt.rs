//! Voltage measurement in millivolts.

use crate::macros::impl_unit_int_ops;

/// Voltage in millivolts (1 LSB = 1 mV)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliVolt(pub i16);

impl_unit_int_ops!(MilliVolt);

impl MilliVolt {
    #[inline]
    pub const fn from_mv(mv: i16) -> Self {
        Self(mv)
    }

    #[inline]
    pub const fn as_mv(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_volts(v: i16) -> Self {
        Self(v * 1000)
    }
}
