//! PWM duty cycle.

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
        ((self.0 as i32 * 100) / i16::MAX as i32) as i8
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
        ((self.0 as i32) * (max as i32)) / 32768
    }

    /// Create from hardware PWM value given max value
    #[inline]
    pub fn from_hw(hw_value: i32, max: u16) -> Self {
        let normalized = (hw_value * 32768) / (max as i32);
        Self(normalized.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    /// Get absolute value
    #[inline]
    pub fn abs(self) -> Self {
        Self(self.0.saturating_abs())
    }
}
