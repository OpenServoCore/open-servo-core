//! Normalized actuator effort.
//!
//! `Effort` is a **dimensionless normalized command** representing actuator authority.
//! It is intentionally unit-free: boards map it to their physical actuator interface
//! (PWM duty, FOC Iq request, etc.).
//!
//! # Semantics
//!
//! - **Range:** full `i16` range: -32768..32767 (Q15-ish)
//! - **Sign:** negative = reverse, positive = forward, zero = no command
//! - **Meaning:** "how much actuator authority to apply", not current/torque/voltage
//!
//! # Board Mapping
//!
//! - **BDC (H-bridge):** typically mapped to PWM duty cycle
//! - **BLDC (FOC):** typically mapped to torque-producing current (Iq) request
//!
//! The kernel produces `Effort`; the board translates it to electrical commands.

use crate::macros::impl_unit_int_ops;

/// Dimensionless normalized actuator command.
///
/// Range: -32768..32767 (full `i16`).
///
/// - Negative = reverse direction
/// - Zero = no commanded effort
/// - Positive = forward direction
///
/// This is **not** a physical unit (not current, not torque, not duty).
/// Boards map `Effort` to their actuator interface (PWM duty, FOC Iq, etc.).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Effort(pub i16);

impl_unit_int_ops!(Effort);

impl Effort {
    /// Maximum effort (full forward)
    pub const MAX: Self = Self(i16::MAX);
    /// Minimum effort (full reverse)
    pub const MIN: Self = Self(i16::MIN);
    /// Zero effort (stopped)
    pub const ZERO: Self = Self(0);

    /// Convert to percentage (-100 to 100)
    #[inline]
    pub fn to_percentage(self) -> i8 {
        use crate::helpers::mul_div_round_i32;
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
        assert_eq!(Effort::MAX.to_percentage(), 100);
        // 0 -> 0
        assert_eq!(Effort::ZERO.to_percentage(), 0);
        // Half effort: 16383 * 100 / 32767 = 49.998... -> 50
        assert_eq!(Effort::from_raw(16384).to_percentage(), 50);
        // Negative half: -16384 * 100 / 32767 = -50.001... -> -50
        assert_eq!(Effort::from_raw(-16384).to_percentage(), -50);
    }
}
