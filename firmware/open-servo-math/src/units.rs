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

/// Temperature in 0.1°C (1 LSB = 0.1°C)
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeciC(pub i16);

/// Raw 12-bit ADC value
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Adc12(pub u16);

/// Raw encoder count
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EncoderCount(pub u16);

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
impl_unit_int_ops!(DeciC);

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

impl DeciC {
    #[inline]
    pub const fn from_dc(dc: i16) -> Self {
        Self(dc)
    }

    #[inline]
    pub const fn as_dc(self) -> i16 {
        self.0
    }

    #[inline]
    pub const fn from_celsius(c: i16) -> Self {
        Self(c * 10)
    }

    #[inline]
    pub const fn as_celsius(self) -> i16 {
        self.0 / 10
    }

    #[inline]
    pub fn from_kelvin(k: u16) -> Self {
        // deciC = (K - 273.15) * 10
        let dc = (k as i32 - 273) * 10 - 15;
        DeciC(dc.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    pub fn to_kelvin(self) -> u16 {
        // K = deciC / 10 + 273.15
        let k = (self.0 as i32 + 2732) / 10;
        k.clamp(0, u16::MAX as i32) as u16
    }

    #[inline]
    pub fn from_fahrenheit(f: i16) -> Self {
        // C = (F - 32) * 5/9
        // deciC = (F - 32) * 50 / 9
        let dc = mul_div_i32(f as i32 - 32, 50, 9);
        DeciC(dc.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    pub fn as_fahrenheit(self) -> i16 {
        // F = C * 9/5 + 32
        // F = deciC * 9 / 50 + 32
        let f = mul_div_i32(self.0 as i32, 9, 50) + 32;
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
