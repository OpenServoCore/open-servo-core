//! Simple integer-based units for servo control
//!
//! All units use integer scaling to avoid floating point and minimize binary size.
//! The scaling factors are chosen to provide sufficient precision for servo control
//! while fitting comfortably in i16/u16 types.

use core::ops::{Add, Div, Mul, Neg, Sub};

/// Voltage in millivolts (1 LSB = 1 mV)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliVolt(pub i16);

/// Current in milliamps (1 LSB = 1 mA)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliAmp(pub i16);

/// Angle in centidegrees (1 LSB = 0.01°)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentiDeg(pub i16);

/// Angular velocity in 0.1 deg/s (1 LSB = 0.1 deg/s)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DegPerSec10(pub i16);

/// Temperature in 0.01°C (1 LSB = 0.01°C, centi-degrees Celsius)
/// Range: -327.68°C to +327.67°C
/// Used for all temperature measurements and thermal calculations
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CentiC(pub i16);

/// Raw 12-bit ADC value
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Adc12(pub u16);

/// Raw encoder count
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EncoderCount(pub u16);

/// PWM duty cycle in normalized units (-32768 to 32767)
/// -32768 = full reverse, 0 = stopped, 32767 = full forward
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Duty(pub i16);

// Macro to implement basic arithmetic operations for unit types
macro_rules! impl_unit_int_ops {
    ($name:ident) => {
        impl Add for $name {
            type Output = Self;
            #[inline]
            fn add(self, rhs: Self) -> Self {
                Self(self.0.saturating_add(rhs.0))
            }
        }

        impl Sub for $name {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: Self) -> Self {
                Self(self.0.saturating_sub(rhs.0))
            }
        }

        impl Neg for $name {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                Self(self.0.saturating_neg())
            }
        }

        impl Mul<i16> for $name {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: i16) -> Self {
                Self(self.0.saturating_mul(rhs))
            }
        }

        impl Div<i16> for $name {
            type Output = Self;
            #[inline]
            fn div(self, rhs: i16) -> Self {
                Self(self.0 / rhs)
            }
        }
    };
}

impl_unit_int_ops!(MilliVolt);
impl_unit_int_ops!(MilliAmp);
impl_unit_int_ops!(CentiDeg);
impl_unit_int_ops!(DegPerSec10);
impl_unit_int_ops!(CentiC);
impl_unit_int_ops!(Duty);

// Helper function for integer multiplication and division with rounding
#[inline]
pub fn mul_div_i32(val: i32, num: i32, den: i32) -> i32 {
    (val * num + den / 2) / den
}

// ============= Conversion implementations =============

impl MilliVolt {
    #[inline]
    pub const fn from_mv(mv: i16) -> Self {
        Self(mv)
    }

    #[inline]
    pub const fn as_mv(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_volts(v: i16) -> Self {
        Self(v * 1000)
    }

    #[inline]
    pub fn from_adc12(code: Adc12, vref_mv: u16) -> Self {
        // mv = code * vref_mv / 4095
        let raw = code.0 as i32;
        let vref = vref_mv as i32;
        let mv = mul_div_i32(raw, vref, 4095);
        MilliVolt(mv.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
}

impl MilliAmp {
    #[inline]
    pub const fn from_ma(ma: i16) -> Self {
        Self(ma)
    }

    #[inline]
    pub const fn as_ma(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_amps(a: i16) -> Self {
        Self(a * 1000)
    }

    /// Convert from DRV8231A IPROPI ADC reading
    /// IPROPI = 1500µA/A through 2.2kΩ resistor
    /// Results in approximately 0.244 mA per ADC count
    #[inline]
    pub fn from_ipropi_adc(code: Adc12) -> Self {
        // ma = code * 244 / 1000 (approximately 0.244 mA/count)
        let raw = code.0 as i32;
        let ma = mul_div_i32(raw, 244, 1000);
        MilliAmp(ma.clamp(0, i16::MAX as i32) as i16)
    }

    /// Get absolute value of current (for threshold comparisons)
    #[inline]
    pub fn abs(self) -> Self {
        Self(self.0.saturating_abs())
    }
}

impl CentiDeg {
    #[inline]
    pub const fn from_cdeg(cdeg: i16) -> Self {
        Self(cdeg)
    }

    #[inline]
    pub const fn as_cdeg(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_deg(deg: i16) -> Self {
        Self(deg * 100)
    }

    #[inline]
    pub const fn as_deg(self) -> i16 {
        self.0 / 100
    }

    /// Convert from raw ADC reading for potentiometer position
    /// Maps 0-4095 to -500 to 18500 centidegrees (-5° to 185°)
    #[inline]
    pub fn from_pot_adc(code: Adc12) -> Self {
        // Map 0-4095 to -500 to 18500 (190° range in centidegrees)
        let raw = code.0 as i32;
        let cdeg = -500 + mul_div_i32(raw, 19000, 4095);
        CentiDeg(cdeg.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    /// Convert to milliradians for trig calculations if needed
    #[inline]
    pub fn to_mrad(self) -> i32 {
        // mrad = cdeg * 174.5329 / 1000  (π/180 * 100)
        mul_div_i32(self.0 as i32, 175, 1000)
    }
}

impl DegPerSec10 {
    #[inline]
    pub const fn from_dps10(dps10: i16) -> Self {
        Self(dps10)
    }

    #[inline]
    pub const fn as_dps10(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_dps(dps: i16) -> Self {
        Self(dps * 10)
    }

    #[inline]
    pub const fn as_dps(self) -> i16 {
        self.0 / 10
    }

    #[inline]
    pub fn from_rpm(rpm: i16) -> Self {
        // dps10 = rpm * 60 / 10 = rpm * 6
        Self(rpm.saturating_mul(6))
    }

    #[inline]
    pub fn as_rpm(self) -> i16 {
        // rpm = dps10 * 10 / 60 = dps10 / 6
        self.0 / 6
    }
}

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
        let k = (self.0 as i32 + 27315) / 100;
        k.clamp(0, u16::MAX as i32) as u16
    }

    #[inline]
    pub fn from_fahrenheit(f: i16) -> Self {
        // C = (F - 32) * 5/9
        // centiC = (F - 32) * 500 / 9
        let cc = mul_div_i32(f as i32 - 32, 500, 9);
        CentiC(cc.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    pub fn as_fahrenheit(self) -> i16 {
        // F = C * 9/5 + 32
        // F = centiC * 9 / 500 + 32
        let f = mul_div_i32(self.0 as i32, 9, 500) + 32;
        f.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

impl Adc12 {
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw & 0x0FFF) // Ensure 12-bit
    }

    #[inline]
    pub const fn as_raw(self) -> u16 {
        self.0
    }
}

impl EncoderCount {
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    #[inline]
    pub const fn as_raw(self) -> u16 {
        self.0
    }
}

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

#[cfg(test)]
mod tests {
    use super::*;

    // ========== CentiDeg tests ==========

    #[test]
    fn test_centideg_from_deg() {
        assert_eq!(CentiDeg::from_deg(90).as_cdeg(), 9000);
        assert_eq!(CentiDeg::from_deg(0).as_cdeg(), 0);
        assert_eq!(CentiDeg::from_deg(-45).as_cdeg(), -4500);
    }

    #[test]
    fn test_centideg_as_deg() {
        assert_eq!(CentiDeg::from_cdeg(9000).as_deg(), 90);
        assert_eq!(CentiDeg::from_cdeg(9050).as_deg(), 90); // truncates
        assert_eq!(CentiDeg::from_cdeg(-4500).as_deg(), -45);
    }

    #[test]
    fn test_centideg_from_pot_adc_endpoints() {
        // ADC 0 -> -500 cdeg (-5°)
        assert_eq!(CentiDeg::from_pot_adc(Adc12::from_raw(0)).as_cdeg(), -500);
        // ADC 4095 -> 18500 cdeg (185°)
        assert_eq!(CentiDeg::from_pot_adc(Adc12::from_raw(4095)).as_cdeg(), 18500);
    }

    #[test]
    fn test_centideg_from_pot_adc_midpoint() {
        // ADC ~2048 should be ~90° (9000 cdeg)
        let mid = CentiDeg::from_pot_adc(Adc12::from_raw(2048));
        // Expected: -500 + (2048 * 19000 / 4095) ≈ -500 + 9502 = 9002
        assert!((mid.as_cdeg() - 9000).abs() < 50);
    }

    // ========== MilliAmp tests ==========

    #[test]
    fn test_milliamp_from_ipropi_adc() {
        // ADC 0 -> 0 mA
        assert_eq!(MilliAmp::from_ipropi_adc(Adc12::from_raw(0)).as_ma(), 0);
        // ADC 1000 -> ~244 mA
        let ma = MilliAmp::from_ipropi_adc(Adc12::from_raw(1000)).as_ma();
        assert!((ma - 244).abs() < 2);
        // ADC 4095 -> ~1000 mA
        let ma = MilliAmp::from_ipropi_adc(Adc12::from_raw(4095)).as_ma();
        assert!((ma - 1000).abs() < 5);
    }

    #[test]
    fn test_milliamp_abs() {
        assert_eq!(MilliAmp::from_ma(100).abs().as_ma(), 100);
        assert_eq!(MilliAmp::from_ma(-100).abs().as_ma(), 100);
        assert_eq!(MilliAmp::from_ma(0).abs().as_ma(), 0);
    }

    // ========== MilliVolt tests ==========

    #[test]
    fn test_millivolt_from_adc12() {
        // ADC 0 -> 0 mV
        assert_eq!(MilliVolt::from_adc12(Adc12::from_raw(0), 3300).as_mv(), 0);
        // ADC 4095 -> 3300 mV (full scale)
        assert_eq!(MilliVolt::from_adc12(Adc12::from_raw(4095), 3300).as_mv(), 3300);
        // ADC 2048 -> ~1650 mV (midpoint)
        let mv = MilliVolt::from_adc12(Adc12::from_raw(2048), 3300).as_mv();
        assert!((mv - 1650).abs() < 2);
    }

    // ========== CentiC tests ==========

    #[test]
    fn test_centi_c_celsius_roundtrip() {
        assert_eq!(CentiC::from_celsius(25).as_celsius(), 25);
        assert_eq!(CentiC::from_celsius(-10).as_celsius(), -10);
        assert_eq!(CentiC::from_celsius(0).as_celsius(), 0);
    }

    #[test]
    fn test_centi_c_kelvin_conversion() {
        // 0°C = 273K
        let cc = CentiC::from_celsius(0);
        assert!((cc.to_kelvin() as i32 - 273).abs() <= 1);

        // 273K -> ~0°C
        let cc = CentiC::from_kelvin(273);
        assert!(cc.as_celsius().abs() <= 1);
    }

    #[test]
    fn test_centi_c_fahrenheit_conversion() {
        // 32°F = 0°C
        let cc = CentiC::from_fahrenheit(32);
        assert!(cc.as_celsius().abs() <= 1);

        // 212°F = 100°C
        let cc = CentiC::from_fahrenheit(212);
        assert!((cc.as_celsius() - 100).abs() <= 1);

        // 0°C -> 32°F
        let f = CentiC::from_celsius(0).as_fahrenheit();
        assert!((f - 32).abs() <= 1);
    }

    // ========== Arithmetic tests ==========

    #[test]
    fn test_saturating_add() {
        let a = CentiDeg::from_cdeg(i16::MAX - 100);
        let b = CentiDeg::from_cdeg(200);
        assert_eq!((a + b).as_cdeg(), i16::MAX); // saturates
    }

    #[test]
    fn test_saturating_sub() {
        let a = CentiDeg::from_cdeg(i16::MIN + 100);
        let b = CentiDeg::from_cdeg(200);
        assert_eq!((a - b).as_cdeg(), i16::MIN); // saturates
    }

    #[test]
    fn test_neg() {
        assert_eq!((-CentiDeg::from_cdeg(100)).as_cdeg(), -100);
        assert_eq!((-CentiDeg::from_cdeg(-100)).as_cdeg(), 100);
    }

    // ========== Adc12 tests ==========

    #[test]
    fn test_adc12_masks_to_12bit() {
        assert_eq!(Adc12::from_raw(0xFFFF).as_raw(), 0x0FFF);
        assert_eq!(Adc12::from_raw(0x1234).as_raw(), 0x0234);
    }
}
