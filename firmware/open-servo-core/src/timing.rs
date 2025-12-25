//! Timing conversion utilities for tick-rate independence.
//!
//! Converts time-based durations (microseconds, milliseconds) to tick counts
//! based on the actual tick period. This removes hardcoded tick-rate assumptions.

/// Convert a duration in microseconds to tick count at the given dt (rounds up).
///
/// # Arguments
/// * `dt_us` - Tick period in microseconds (e.g., 100 for 10kHz)
/// * `duration_us` - Duration to convert in microseconds
///
/// # Returns
/// Number of ticks, rounded up to ensure the full duration is covered.
#[inline]
pub const fn ticks_from_us(dt_us: u32, duration_us: u32) -> u32 {
    // Ceiling division: (duration + dt - 1) / dt
    (duration_us + dt_us - 1) / dt_us
}

/// Convert a duration in milliseconds to tick count at the given dt.
///
/// Convenience wrapper around `ticks_from_us`.
#[inline]
pub const fn ticks_from_ms(dt_us: u32, duration_ms: u32) -> u32 {
    ticks_from_us(dt_us, duration_ms * 1000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ticks_from_us_exact() {
        // 1000us at 100us/tick = exactly 10 ticks
        assert_eq!(ticks_from_us(100, 1000), 10);
    }

    #[test]
    fn test_ticks_from_us_rounds_up() {
        // 150us at 100us/tick = 2 ticks (rounds up from 1.5)
        assert_eq!(ticks_from_us(100, 150), 2);
        // 101us at 100us/tick = 2 ticks (rounds up from 1.01)
        assert_eq!(ticks_from_us(100, 101), 2);
    }

    #[test]
    fn test_ticks_from_us_zero() {
        // 0us = 0 ticks
        assert_eq!(ticks_from_us(100, 0), 0);
    }

    #[test]
    fn test_ticks_from_ms() {
        // 1ms at 100us/tick = 10 ticks
        assert_eq!(ticks_from_ms(100, 1), 10);
        // 500ms at 100us/tick = 5000 ticks
        assert_eq!(ticks_from_ms(100, 500), 5000);
    }

    #[test]
    fn test_ticks_from_us_different_rates() {
        // 1ms at 1000us/tick (1kHz) = 1 tick
        assert_eq!(ticks_from_us(1000, 1000), 1);
        // 1ms at 10000us/tick (100Hz) = 1 tick (rounds up from 0.1)
        assert_eq!(ticks_from_us(10000, 1000), 1);
    }
}
