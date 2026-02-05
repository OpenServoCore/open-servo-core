//! Time duration in microseconds.

/// Time duration in microseconds (1 LSB = 1 µs)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MicroSecond(pub u32);

impl MicroSecond {
    /// Zero duration
    pub const ZERO: Self = Self(0);

    #[inline]
    pub const fn from_us(us: u32) -> Self {
        Self(us)
    }

    #[inline]
    pub const fn as_us(self) -> u32 {
        self.0
    }

    #[inline]
    pub const fn from_ms(ms: u32) -> Self {
        Self(ms * 1000)
    }

    #[inline]
    pub const fn as_ms(self) -> u32 {
        self.0 / 1000
    }

    #[inline]
    pub const fn from_secs(s: u32) -> Self {
        Self(s * 1_000_000)
    }

    #[inline]
    pub const fn as_secs(self) -> u32 {
        self.0 / 1_000_000
    }

    #[inline]
    pub const fn saturating_add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }

    #[inline]
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }
}

impl core::ops::Add for MicroSecond {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self.saturating_add(rhs)
    }
}

impl core::ops::Sub for MicroSecond {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self.saturating_sub(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conversions() {
        assert_eq!(MicroSecond::from_ms(1).as_us(), 1000);
        assert_eq!(MicroSecond::from_secs(1).as_us(), 1_000_000);
        assert_eq!(MicroSecond::from_us(1500).as_ms(), 1);
        assert_eq!(MicroSecond::from_us(1_500_000).as_secs(), 1);
    }

    #[test]
    fn test_saturating_arithmetic() {
        let a = MicroSecond::from_us(u32::MAX - 100);
        let b = MicroSecond::from_us(200);
        assert_eq!((a + b).as_us(), u32::MAX);

        let a = MicroSecond::from_us(100);
        let b = MicroSecond::from_us(200);
        assert_eq!((a - b).as_us(), 0);
    }
}
