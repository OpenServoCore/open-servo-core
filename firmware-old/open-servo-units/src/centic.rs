//! Temperature in centi-degrees Celsius.

use crate::helpers::{div_round_i32, mul_div_round_i32};
use crate::macros::impl_unit_int_ops;

/// Temperature in 0.01°C (1 LSB = 0.01°C, centi-degrees Celsius)
/// Range: -327.68°C to +327.67°C
/// Used for all temperature measurements and thermal calculations
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentiC(pub i16);

impl_unit_int_ops!(CentiC);

impl CentiC {
    /// Create from raw centi-degrees value
    #[inline]
    pub const fn from_centi_c(cc: i16) -> Self {
        Self(cc)
    }

    /// Get raw centi-degrees value
    #[inline]
    pub const fn as_centi_c(self) -> i16 {
        self.0
    }

    /// Create from whole degrees Celsius
    #[inline]
    pub const fn from_celsius(c: i16) -> Self {
        Self(c * 100)
    }

    /// Convert to whole degrees Celsius (truncates)
    #[inline]
    pub const fn as_celsius(self) -> i16 {
        self.0 / 100
    }

    /// Create from deci-degrees (0.1°C)
    #[inline]
    pub const fn from_deci_c(dc: i16) -> Self {
        Self(dc * 10)
    }

    /// Convert to deci-degrees (0.1°C, truncates)
    #[inline]
    pub const fn as_deci_c(self) -> i16 {
        self.0 / 10
    }

    /// Create from Kelvin temperature
    #[inline]
    pub fn from_kelvin(k: u16) -> Self {
        // centiC = (K - 273.15) * 100
        let cc = ((k as i32 - 273) * 100) - 15;
        CentiC(cc.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    pub fn to_kelvin(self) -> u16 {
        // K = centiC / 100 + 273.15
        let k = div_round_i32(self.0 as i32 + 27315, 100);
        k.clamp(0, u16::MAX as i32) as u16
    }

    #[inline]
    pub fn from_fahrenheit(f: i16) -> Self {
        // C = (F - 32) * 5/9
        // centiC = (F - 32) * 500 / 9
        let cc = mul_div_round_i32(f as i32 - 32, 500, 9);
        CentiC(cc.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    pub fn as_fahrenheit(self) -> i16 {
        // F = C * 9/5 + 32
        // F = centiC * 9 / 500 + 32
        let f = mul_div_round_i32(self.0 as i32, 9, 500) + 32;
        f.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_celsius_roundtrip() {
        assert_eq!(CentiC::from_celsius(25).as_celsius(), 25);
        assert_eq!(CentiC::from_celsius(-10).as_celsius(), -10);
        assert_eq!(CentiC::from_celsius(0).as_celsius(), 0);
    }

    #[test]
    fn test_kelvin_conversion() {
        let cc = CentiC::from_celsius(0);
        assert!((cc.to_kelvin() as i32 - 273).abs() <= 1);

        let cc = CentiC::from_kelvin(273);
        assert!(cc.as_celsius().abs() <= 1);
    }

    #[test]
    fn test_kelvin_rounding() {
        // 27350 centiC + 27315 = 54665, / 100 = 546.65 -> rounds to 547
        assert_eq!(CentiC::from_centi_c(27350).to_kelvin(), 547);
        // 27340 centiC + 27315 = 54655, / 100 = 546.55 -> rounds to 547
        assert_eq!(CentiC::from_centi_c(27340).to_kelvin(), 547);
        // 27330 centiC + 27315 = 54645, / 100 = 546.45 -> rounds to 546
        assert_eq!(CentiC::from_centi_c(27330).to_kelvin(), 546);
    }

    #[test]
    fn test_fahrenheit_conversion() {
        let cc = CentiC::from_fahrenheit(32);
        assert!(cc.as_celsius().abs() <= 1);

        let cc = CentiC::from_fahrenheit(212);
        assert!((cc.as_celsius() - 100).abs() <= 1);

        let f = CentiC::from_celsius(0).as_fahrenheit();
        assert!((f - 32).abs() <= 1);
    }
}
