//! Frequency in Hertz.

use crate::helpers::div_round_u32;
use crate::MicroSecond;

/// Frequency in Hertz (1 LSB = 1 Hz)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Hertz(pub u32);

impl Hertz {
    #[inline]
    pub const fn from_hz(hz: u32) -> Self {
        Self(hz)
    }

    #[inline]
    pub const fn as_hz(self) -> u32 {
        self.0
    }

    #[inline]
    pub const fn from_khz(khz: u32) -> Self {
        Self(khz * 1000)
    }

    #[inline]
    pub const fn as_khz(self) -> u32 {
        self.0 / 1000
    }

    #[inline]
    pub const fn from_mhz(mhz: u32) -> Self {
        Self(mhz * 1_000_000)
    }

    #[inline]
    pub const fn as_mhz(self) -> u32 {
        self.0 / 1_000_000
    }

    /// Convert frequency to period in microseconds (rounded).
    /// Returns None if frequency is zero.
    #[inline]
    pub const fn to_period_us(self) -> Option<MicroSecond> {
        if self.0 == 0 {
            None
        } else {
            Some(MicroSecond::from_us(div_round_u32(1_000_000, self.0)))
        }
    }

    /// Create from period in microseconds (rounded).
    /// Returns None if period is zero.
    #[inline]
    pub const fn from_period_us(period: MicroSecond) -> Option<Self> {
        if period.0 == 0 {
            None
        } else {
            Some(Self(div_round_u32(1_000_000, period.0)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conversions() {
        assert_eq!(Hertz::from_khz(1).as_hz(), 1000);
        assert_eq!(Hertz::from_mhz(1).as_hz(), 1_000_000);
        assert_eq!(Hertz::from_hz(1500).as_khz(), 1);
    }

    #[test]
    fn test_period_conversion() {
        // 1 kHz = 1000 µs period (exact)
        assert_eq!(
            Hertz::from_khz(1).to_period_us(),
            Some(MicroSecond::from_us(1000))
        );

        // 50 Hz = 20000 µs period (exact)
        assert_eq!(
            Hertz::from_hz(50).to_period_us(),
            Some(MicroSecond::from_us(20000))
        );

        // 60 Hz = 16667 µs period (rounded from 16666.67)
        assert_eq!(
            Hertz::from_hz(60).to_period_us(),
            Some(MicroSecond::from_us(16667))
        );

        // 0 Hz has no period
        assert_eq!(Hertz::from_hz(0).to_period_us(), None);

        // Round trip
        let freq = Hertz::from_hz(1000);
        let period = freq.to_period_us().unwrap();
        assert_eq!(Hertz::from_period_us(period), Some(freq));
    }
}
