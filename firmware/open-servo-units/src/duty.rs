//! PWM duty cycle.

use crate::helpers::mul_div_round_i32;
use crate::macros::impl_unit_int_ops;

/// PWM duty cycle in normalized units (-32768 to 32767)
/// -32768 = full reverse, 0 = stopped, 32767 = full forward
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Duty(pub i16);

impl_unit_int_ops!(Duty);

impl Duty {
    /// Maximum duty cycle (full forward)
    pub const MAX: Self = Self(i16::MAX);
    /// Minimum duty cycle (full reverse)
    pub const MIN: Self = Self(i16::MIN);
    /// Zero duty cycle (stopped)
    pub const ZERO: Self = Self(0);

    /// Convert to percentage (-100 to 100)
    #[inline]
    pub fn to_percentage(self) -> i8 {
        // Scale from i16 range to -100..100
        mul_div_round_i32(self.0 as i32, 100, i16::MAX as i32) as i8
    }

    #[inline]
    pub const fn from_raw(raw: i16) -> Self {
        Self(raw)
    }

    #[inline]
    pub const fn as_raw(self) -> i16 {
        self.0
    }

    /// Scale to hardware PWM value given max value
    #[inline]
    pub fn scale_to(self, max: u16) -> i32 {
        // Scale from -32768..32767 to -max..max
        mul_div_round_i32(self.0 as i32, max as i32, 32768)
    }

    /// Create from hardware PWM value given max value
    #[inline]
    pub fn from_hw(hw_value: i32, max: u16) -> Self {
        let normalized = mul_div_round_i32(hw_value, 32768, max as i32);
        Self(normalized.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    /// Get absolute value
    #[inline]
    pub fn abs(self) -> Self {
        Self(self.0.saturating_abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_percentage_rounding() {
        // i16::MAX * 100 / i16::MAX = 100 exactly
        assert_eq!(Duty::MAX.to_percentage(), 100);
        // 0 -> 0
        assert_eq!(Duty::ZERO.to_percentage(), 0);
        // Half duty: 16383 * 100 / 32767 = 49.998... -> 50
        assert_eq!(Duty::from_raw(16384).to_percentage(), 50);
        // Negative half: -16384 * 100 / 32767 = -50.001... -> -50
        assert_eq!(Duty::from_raw(-16384).to_percentage(), -50);
    }

    #[test]
    fn test_scale_to_rounding() {
        // Full duty with max=1000: 32767 * 1000 / 32768 = 999.97 -> 1000
        assert_eq!(Duty::MAX.scale_to(1000), 1000);
        // Half duty: 16384 * 1000 / 32768 = 500 exactly
        assert_eq!(Duty::from_raw(16384).scale_to(1000), 500);
        // Negative: -16384 * 1000 / 32768 = -500 exactly
        assert_eq!(Duty::from_raw(-16384).scale_to(1000), -500);
        // Rounding case: 1 * 1000 / 32768 = 0.0305 -> 0
        assert_eq!(Duty::from_raw(1).scale_to(1000), 0);
        // 17 * 1000 / 32768 = 0.519 -> 1
        assert_eq!(Duty::from_raw(17).scale_to(1000), 1);
    }

    #[test]
    fn test_from_hw_rounding() {
        // 500 * 32768 / 1000 = 16384 exactly
        assert_eq!(Duty::from_hw(500, 1000).as_raw(), 16384);
        // 1 * 32768 / 1000 = 32.768 -> 33
        assert_eq!(Duty::from_hw(1, 1000).as_raw(), 33);
        // -1 * 32768 / 1000 = -32.768 -> -33
        assert_eq!(Duty::from_hw(-1, 1000).as_raw(), -33);
    }
}
