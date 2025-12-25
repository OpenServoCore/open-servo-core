//! Sensor reading implementations for STM32F301 board.

use open_servo_math::{CentiC, CentiDeg, MilliAmp, MilliVolt};
#[cfg(feature = "temp-sense-motor")]
use open_servo_math::ntc_lut_to_centi_celsius;

use crate::adc_sample::AdcSample;
use crate::calibration::{BoardCalibration, convert_vdda_mv};
#[cfg(feature = "temp-sense-mcu")]
use crate::calibration::convert_temperature_cc;
#[cfg(feature = "voltage-sense-motor")]
use crate::calibration::motor_voltage_from_adc;

/// Motor temperature NTC lookup table
/// 10K B3950 NTC with 10K fixed resistor, low-side configuration:
/// VCC → 10K fixed → ADC (PA4) → NTC → GND
/// Sorted by descending ADC (ascending temperature).
/// Values are in centi-degrees (0.01°C)
#[cfg(feature = "temp-sense-motor")]
const MOTOR_NTC_LUT: [(u16, i16); 13] = [
    (3469, -1000), // -10°C
    (3133, 0),     //   0°C
    (2725, 1000),  //  10°C
    (2274, 2000),  //  20°C
    (2048, 2500),  //  25°C
    (1826, 3000),  //  30°C
    (1421, 4000),  //  40°C
    (1078, 5000),  //  50°C
    (810, 6000),   //  60°C
    (609, 7000),   //  70°C
    (459, 8000),   //  80°C
    (351, 9000),   //  90°C
    (271, 10000),  // 100°C
];

/// Sensor reading functions for the STM32F301 board
pub struct SensorReader {
    calibration: BoardCalibration,
}

impl SensorReader {
    pub fn new() -> Self {
        Self {
            calibration: BoardCalibration::default(),
        }
    }

    /// Get the latest ADC sample
    #[inline(always)]
    fn get_adc_sample() -> AdcSample {
        AdcSample::read()
    }

    // Position sensor methods
    
    /// Read raw position ADC value
    pub fn position_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.position_raw
    }

    /// Read position in centidegrees
    pub fn position(&self) -> CentiDeg {
        let raw = self.position_raw();
        self.calibration.position_from_adc(raw)
    }

    // Current sensor methods
    
    /// Read raw current ADC value
    #[cfg(feature = "current-sense-bus")]
    pub fn current_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.current_raw
    }

    /// Read bus current in milliamps
    #[cfg(feature = "current-sense-bus")]
    pub fn current(&self) -> MilliAmp {
        let sample = Self::get_adc_sample();
        let vdda_mv = convert_vdda_mv(sample.vrefint_raw);
        self.calibration.current_from_adc(sample.current_raw, vdda_mv)
    }

    // Voltage sensor methods
    
    /// Read raw VREFINT ADC value (used to calculate VDDA)
    pub fn vdda_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.vrefint_raw
    }

    /// Read VDDA (MCU analog supply voltage) in millivolts
    /// Note: This is the MCU logic rail (~3.3V), NOT the motor bus voltage
    pub fn vdda(&self) -> MilliVolt {
        let sample = Self::get_adc_sample();
        let vdda_mv = convert_vdda_mv(sample.vrefint_raw);
        MilliVolt::from_mv(vdda_mv as i16)
    }

    /// Read raw motor terminal voltage ADC values
    /// Returns (V+ raw, V- raw)
    #[cfg(feature = "voltage-sense-motor")]
    pub fn motor_voltage_raw(&self) -> (u16, u16) {
        let sample = Self::get_adc_sample();
        (sample.motor_vpos_raw, sample.motor_vneg_raw)
    }

    /// Read both motor terminal voltages (V+, V-) in millivolts
    #[cfg(feature = "voltage-sense-motor")]
    pub fn motor_voltage(&self) -> (MilliVolt, MilliVolt) {
        let (vpos_raw, vneg_raw) = self.motor_voltage_raw();
        motor_voltage_from_adc(vpos_raw, vneg_raw)
    }

    // Temperature sensor methods
    
    /// Read raw MCU internal temperature sensor ADC value
    #[cfg(feature = "temp-sense-mcu")]
    pub fn mcu_temperature_raw(&self) -> Option<u16> {
        let sample = Self::get_adc_sample();
        Some(sample.mcu_temp_raw)
    }

    /// Read MCU internal temperature in centi-Celsius
    #[cfg(feature = "temp-sense-mcu")]
    pub fn mcu_temperature(&self) -> Option<CentiC> {
        let sample = Self::get_adc_sample();
        let vdda_mv = convert_vdda_mv(sample.vrefint_raw);
        let temp_cc = convert_temperature_cc(sample.mcu_temp_raw, vdda_mv);
        Some(temp_cc)
    }

    /// Read raw motor temperature NTC ADC value
    #[cfg(feature = "temp-sense-motor")]
    pub fn motor_temperature_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.motor_temp_raw
    }

    /// Read motor temperature in centi-Celsius
    #[cfg(feature = "temp-sense-motor")]
    pub fn motor_temperature(&self) -> CentiC {
        let raw = self.motor_temperature_raw();
        let cc = ntc_lut_to_centi_celsius(raw, &MOTOR_NTC_LUT);
        CentiC::from_centi_c(cc)
    }

    /// Get safety current reading (returns None if no current sensor)
    pub fn safety_current(&self) -> Option<MilliAmp> {
        #[cfg(feature = "current-sense-bus")]
        return Some(self.current());
        
        #[cfg(not(feature = "current-sense-bus"))]
        return None;
    }

    /// Get safety MCU temperature reading (returns None if no temperature sensor)
    pub fn safety_mcu_temp(&self) -> Option<CentiC> {
        #[cfg(feature = "temp-sense-mcu")]
        return self.mcu_temperature();
        
        #[cfg(not(feature = "temp-sense-mcu"))]
        return None;
    }

    /// Get safety motor temperature reading (returns None if no temperature sensor)
    pub fn safety_motor_temp(&self) -> Option<CentiC> {
        #[cfg(feature = "temp-sense-motor")]
        return Some(self.motor_temperature());
        
        #[cfg(not(feature = "temp-sense-motor"))]
        return None;
    }

    /// Get safety motor voltage reading (returns None if no voltage sensor)
    pub fn safety_motor_voltage(&self) -> Option<(MilliVolt, MilliVolt)> {
        #[cfg(feature = "voltage-sense-motor")]
        return Some(self.motor_voltage());
        
        #[cfg(not(feature = "voltage-sense-motor"))]
        return None;
    }
}