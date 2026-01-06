//! Monotonic timestamp in microseconds.

use crate::MicroSecond;

/// Monotonic timestamp in microseconds (wrapping is OK).
///
/// This is distinct from [`MicroSecond`], which represents a **duration**.
/// Keeping timestamps and durations separate avoids many bugs.
///
/// Use [`wrapping_since`](Self::wrapping_since) to compute elapsed time.
#[repr(transparent)]
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimeStampUs(pub u32);

impl TimeStampUs {
    /// Create a timestamp from raw microseconds.
    #[inline]
    pub const fn from_us(us: u32) -> Self {
        Self(us)
    }

    /// Get the raw microsecond value.
    #[inline]
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Wrapping difference (newer - older), returned as a duration.
    ///
    /// This is safe across wrap if elapsed intervals are "reasonably small"
    /// compared to the wrap period (~71 minutes for u32 microseconds).
    #[inline]
    pub fn wrapping_since(self, older: TimeStampUs) -> MicroSecond {
        MicroSecond(self.0.wrapping_sub(older.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrapping_since() {
        let t1 = TimeStampUs(100);
        let t2 = TimeStampUs(200);
        assert_eq!(t2.wrapping_since(t1), MicroSecond(100));
    }

    #[test]
    fn test_wrapping_since_wrap() {
        // t2 wrapped around (t2 < t1 numerically, but t2 is "later")
        let t1 = TimeStampUs(u32::MAX - 50);
        let t2 = TimeStampUs(50);
        // Elapsed = 50 + 51 = 101
        assert_eq!(t2.wrapping_since(t1), MicroSecond(101));
    }
}
