//! Gain type for PID controller tuning.
//!
//! Stores gains as fixed-point (scaled by 100) to avoid float parsing/formatting
//! code bloat on embedded targets without FPU.

use core::str::FromStr;

/// A gain value stored as fixed-point (scaled by 100).
/// Parses from strings like "40", "-3.5", "0.12" without linking float parsing code.
/// Displays as "40.00" using integer-only formatting (no core::fmt::float).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Gain(i32); // scaled by 100

impl Gain {
    /// Create a Gain from an f32 value.
    #[inline]
    pub fn from_f32(val: f32) -> Self {
        let scaled = if val >= 0.0 {
            (val * 100.0 + 0.5) as i32
        } else {
            (val * 100.0 - 0.5) as i32
        };
        Gain(scaled)
    }

    /// Create a Gain from a raw scaled value (×100).
    #[inline]
    pub const fn from_raw(raw: i32) -> Self {
        Gain(raw)
    }

    /// Get the gain as f32.
    #[inline]
    pub fn as_f32(self) -> f32 {
        self.0 as f32 / 100.0
    }

    /// Get the raw scaled value (×100).
    #[inline]
    pub fn raw(self) -> i32 {
        self.0
    }

    /// Convert to Q8.8 fixed-point for use in PID controller.
    /// Clamps to i16 range.
    #[inline]
    pub fn to_q8_8(self) -> i16 {
        // raw is scaled by 100, Q8.8 is scaled by 256
        // so multiply by 256/100 = 2.56
        // Use i32 math - works for gains up to ~8000 (80.0) before overflow
        // Clamp input to avoid i32 overflow: max safe is i32::MAX / 256 ≈ 8388607
        let clamped = self.0.clamp(-838860, 838860);
        let scaled = (clamped * 256) / 100;
        scaled.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

impl core::fmt::Display for Gain {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let scaled = self.0;
        let sign = if scaled < 0 { "-" } else { "" };
        let scaled = scaled.abs();
        let int_part = scaled / 100;
        let frac_part = scaled % 100;
        write!(f, "{}{}.{:02}", sign, int_part, frac_part)
    }
}

/// Error returned when parsing a gain value fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainParseError {
    /// Input string was empty
    Empty,
    /// Input contained invalid characters
    Invalid,
    /// Value overflowed i32
    Overflow,
}

impl FromStr for Gain {
    type Err = GainParseError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        parse_gain_2dp(s).map(Gain)
    }
}

/// Parse a simple decimal like "40", "-3.5", "0.12"
/// into an i32 scaled by 100 (2 decimal places).
fn parse_gain_2dp(input: &str) -> Result<i32, GainParseError> {
    use GainParseError::*;

    let bytes = input.as_bytes();
    if bytes.is_empty() {
        return Err(Empty);
    }

    let mut i = 0;
    let mut sign = 1;

    if bytes[0] == b'-' {
        sign = -1;
        i += 1;
        if i == bytes.len() {
            return Err(Invalid);
        }
    } else if bytes[0] == b'+' {
        i += 1;
        if i == bytes.len() {
            return Err(Invalid);
        }
    }

    let mut int_part: i32 = 0;
    let mut frac_part: i32 = 0;
    let mut frac_digits: i32 = 0;
    let mut seen_dot = false;

    while i < bytes.len() {
        let b = bytes[i];
        i += 1;

        if b == b'.' {
            if seen_dot {
                return Err(Invalid);
            }
            seen_dot = true;
            continue;
        }

        if !(b'0'..=b'9').contains(&b) {
            return Err(Invalid);
        }

        let d = (b - b'0') as i32;

        if !seen_dot {
            int_part = int_part
                .checked_mul(10)
                .and_then(|x| x.checked_add(d))
                .ok_or(Overflow)?;
        } else if frac_digits < 2 {
            // only keep up to 2 fractional digits
            frac_part = frac_part
                .checked_mul(10)
                .and_then(|x| x.checked_add(d))
                .ok_or(Overflow)?;
            frac_digits += 1;
        }
        // ignore extra fractional digits
    }

    // scale frac_part to 2 digits
    while frac_digits < 2 {
        frac_part *= 10;
        frac_digits += 1;
    }

    let scaled = int_part
        .checked_mul(100)
        .and_then(|x| x.checked_add(frac_part))
        .ok_or(Overflow)?;

    Ok(sign * scaled)
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use alloc::format;

    #[test]
    fn test_gain_integer() {
        let g: Gain = "40".parse().unwrap();
        assert_eq!(g.raw(), 4000);
        assert!((g.as_f32() - 40.0).abs() < 0.01);
    }

    #[test]
    fn test_gain_decimal() {
        let g: Gain = "3.5".parse().unwrap();
        assert_eq!(g.raw(), 350);
        assert!((g.as_f32() - 3.5).abs() < 0.01);
    }

    #[test]
    fn test_gain_small_decimal() {
        let g: Gain = "0.12".parse().unwrap();
        assert_eq!(g.raw(), 12);
    }

    #[test]
    fn test_gain_negative() {
        let g: Gain = "-3.5".parse().unwrap();
        assert_eq!(g.raw(), -350);
    }

    #[test]
    fn test_gain_single_frac_digit() {
        let g: Gain = "1.5".parse().unwrap();
        assert_eq!(g.raw(), 150);
    }

    #[test]
    fn test_gain_extra_frac_digits() {
        // Truncates to 2 decimal places
        let g: Gain = "1.234".parse().unwrap();
        assert_eq!(g.raw(), 123);
    }

    #[test]
    fn test_gain_no_frac() {
        let g: Gain = "100".parse().unwrap();
        assert_eq!(g.raw(), 10000);
    }

    #[test]
    fn test_gain_with_plus() {
        let g: Gain = "+5.5".parse().unwrap();
        assert_eq!(g.raw(), 550);
    }

    #[test]
    fn test_gain_display() {
        let g = Gain::from_f32(40.0);
        assert_eq!(format!("{}", g), "40.00");

        let g = Gain::from_f32(-3.5);
        assert_eq!(format!("{}", g), "-3.50");

        let g = Gain::from_f32(0.12);
        assert_eq!(format!("{}", g), "0.12");
    }

    #[test]
    fn test_gain_to_q8_8() {
        let g = Gain::from_f32(40.0);
        // 40 * 256 = 10240
        assert_eq!(g.to_q8_8(), 10240);

        let g = Gain::from_f32(0.5);
        // 0.5 * 256 = 128
        assert_eq!(g.to_q8_8(), 128);
    }

    #[test]
    fn test_gain_from_raw() {
        let g = Gain::from_raw(4000);
        assert_eq!(g.raw(), 4000);
        assert!((g.as_f32() - 40.0).abs() < 0.01);
    }
}
