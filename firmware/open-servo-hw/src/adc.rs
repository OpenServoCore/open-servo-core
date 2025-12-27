//! ADC resolution vocabulary.
//!
//! This enum is provided for documentation and ergonomics. Use it as a const generic
//! parameter for calibration functions:
//!
//! ```ignore
//! use open_servo_hw::AdcResolution;
//! use open_servo_hw_utils::calibration::*;
//!
//! // Using the enum for clarity (compiles to same code as ::<12>)
//! let current = current_from_ipropi::<{AdcResolution::Bits12 as u8}>(raw, vdda, 1500, 1500);
//!
//! // Or use the literal directly
//! let vdda = vdda_from_vrefint::<12>(vrefint_raw, vrefint_cal, 3300);
//! ```

/// ADC resolution for const generic calibration functions.
///
/// The `as u8` cast gives the bit count, which can be used as a const generic parameter.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcResolution {
    /// 8-bit ADC (max value 255)
    Bits8 = 8,
    /// 10-bit ADC (max value 1023)
    Bits10 = 10,
    /// 12-bit ADC (max value 4095)
    Bits12 = 12,
    /// 14-bit ADC (max value 16383)
    Bits14 = 14,
    /// 16-bit ADC (max value 65535)
    Bits16 = 16,
}

impl AdcResolution {
    /// Maximum ADC value for this resolution.
    #[inline]
    pub const fn max_value(self) -> u32 {
        (1u32 << (self as u8)) - 1
    }

    /// Bit count for this resolution.
    #[inline]
    pub const fn bits(self) -> u8 {
        self as u8
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for AdcResolution {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}bit", self.bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_max_values() {
        assert_eq!(AdcResolution::Bits8.max_value(), 255);
        assert_eq!(AdcResolution::Bits10.max_value(), 1023);
        assert_eq!(AdcResolution::Bits12.max_value(), 4095);
        assert_eq!(AdcResolution::Bits14.max_value(), 16383);
        assert_eq!(AdcResolution::Bits16.max_value(), 65535);
    }

    #[test]
    fn test_bits() {
        assert_eq!(AdcResolution::Bits12.bits(), 12);
        assert_eq!(AdcResolution::Bits12 as u8, 12);
    }
}
