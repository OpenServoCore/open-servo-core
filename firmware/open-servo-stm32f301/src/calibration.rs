//! ADC calibration and conversion functions for STM32F301 board.

#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;
#[cfg(feature = "temp-sense-mcu")]
use open_servo_math::CentiC;
use open_servo_math::{mul_div_i32, CentiDeg, MilliVolt};

// ADC calibration memory addresses
pub const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;
#[cfg(feature = "temp-sense-mcu")]
pub const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;
#[cfg(feature = "temp-sense-mcu")]
pub const TS_CAL2_ADDR: u32 = 0x1FFFF7C2;

/// Board-specific calibration constants for ADC conversions
pub struct BoardCalibration {
    // Current sensing (DRV8231A IPROPI)
    #[cfg(feature = "current-sense-bus")]
    pub ripropi_ohms: u32,           // IPROPI sense resistor value
    #[cfg(feature = "current-sense-bus")]
    pub ipropi_gain_ua_per_a: u32,   // DRV8231A current mirror gain
    
    // Position sensing (potentiometer)
    pub pot_min_cdeg: i16,           // Minimum position in centidegrees
    pub pot_max_cdeg: i16,           // Maximum position in centidegrees
    
    // ADC configuration
    pub adc_max: u16,                // Maximum ADC value (12-bit = 4095)
}

impl Default for BoardCalibration {
    fn default() -> Self {
        Self {
            #[cfg(feature = "current-sense-bus")]
            ripropi_ohms: 1500,           // 1.5kΩ on this board
            #[cfg(feature = "current-sense-bus")]
            ipropi_gain_ua_per_a: 1500,   // DRV8231A datasheet value
            
            pot_min_cdeg: -500,           // -5° minimum
            pot_max_cdeg: 18500,          // 185° maximum
            
            adc_max: 4095,                // 12-bit ADC
        }
    }
}

impl BoardCalibration {
    /// Convert IPROPI ADC reading to motor current using calibrated VDDA
    #[cfg(feature = "current-sense-bus")]
    pub fn current_from_adc(&self, raw: u16, vdda_mv: u16) -> MilliAmp {
        // Steps:
        // 1. ADC code → IPROPI voltage: V = raw * vdda_mv / adc_max
        // 2. IPROPI voltage → IPROPI current: I = V / RIPROPI
        // 3. IPROPI current → motor current: I_motor = I / (gain_uA/A * 1e-6)
        
        let raw = raw as u64;
        let vdda = vdda_mv as u64;
        
        // Calculate: I_motor_mA = raw * vdda_mv * 1000000 / (adc_max * RIPROPI * gain_uA/A)
        let num = raw * vdda * 1_000_000u64;
        let den = (self.adc_max as u64) * (self.ripropi_ohms as u64) * (self.ipropi_gain_ua_per_a as u64);
        
        let ma = if den > 0 { (num / den) as u16 } else { 0 };
        MilliAmp::from_ma(ma as i16)
    }
    
    /// Convert potentiometer ADC reading to position
    pub fn position_from_adc(&self, raw: u16) -> CentiDeg {
        // Map ADC range [0, adc_max] to position range [pot_min_cdeg, pot_max_cdeg]
        let raw = raw as i32;
        let range = (self.pot_max_cdeg - self.pot_min_cdeg) as i32;
        let cdeg = self.pot_min_cdeg as i32 + mul_div_i32(raw, range, self.adc_max as i32);
        CentiDeg::from_cdeg(cdeg.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    
    /// Convert ADC reading to voltage
    pub fn voltage_from_adc(&self, raw: u16, vdda_mv: u16) -> MilliVolt {
        // V = raw * vdda_mv / adc_max
        let mv = mul_div_i32(raw as i32, vdda_mv as i32, self.adc_max as i32);
        MilliVolt::from_mv(mv.clamp(0, i16::MAX as i32) as i16)
    }
}

/// Convert VREFINT ADC reading to VDDA voltage in millivolts
pub fn convert_vdda_mv(vref_raw: u16) -> u16 {
    // Check for invalid vref_raw to prevent divide by zero
    if vref_raw == 0 {
        // Return nominal voltage if ADC hasn't read yet
        return 3300;
    }
    let vrefint_cal: u16 = unsafe { core::ptr::read(VREFINT_CAL_ADDR as *const u16) };
    // vdda_mv = 3300 * vrefint_cal / vref_raw
    mul_div_i32(3300, vrefint_cal as i32, vref_raw as i32) as u16
}

/// Convert MCU internal temperature sensor ADC reading to temperature
#[cfg(feature = "temp-sense-mcu")]
pub fn convert_temperature_cc(adc_value: u16, vdda_mv: u16) -> CentiC {
    let ts_cal1: u16 = unsafe { core::ptr::read(TS_CAL1_ADDR as *const u16) };
    let ts_cal2: u16 = unsafe { core::ptr::read(TS_CAL2_ADDR as *const u16) };

    // Calibration points: 30°C and 110°C at 3.3V
    // Calculate actual temperature using linear interpolation
    // temp_c = 30 + (adc - ts_cal1) * 80 / (ts_cal2 - ts_cal1) * 3300 / vdda_mv

    // Ensure calibration values are valid
    if ts_cal2 <= ts_cal1 || vdda_mv == 0 {
        // Invalid calibration data or vdda, return room temperature
        return CentiC::from_celsius(25);
    }

    let adc_normalized = mul_div_i32(adc_value as i32, 3300, vdda_mv as i32);
    let cal_diff = (ts_cal2 as i32) - (ts_cal1 as i32);
    let adc_diff = adc_normalized - (ts_cal1 as i32);

    let temp_c = 30 + mul_div_i32(adc_diff, 80, cal_diff);

    // Clamp to reasonable range
    let temp_c = temp_c.clamp(-40, 125);
    CentiC::from_celsius(temp_c as i16)
}

/// Convert motor voltage ADC readings to millivolts
/// Assumes voltage divider maps 0-5V motor voltage to full ADC range
#[cfg(feature = "voltage-sense-motor")]
pub fn motor_voltage_from_adc(vpos_raw: u16, vneg_raw: u16) -> (MilliVolt, MilliVolt) {
    // Each terminal: 0-4095 ADC = 0-5000mV
    let vpos_mv = mul_div_i32(vpos_raw as i32, 5000, 4095);
    let vneg_mv = mul_div_i32(vneg_raw as i32, 5000, 4095);
    (MilliVolt::from_mv(vpos_mv as i16), MilliVolt::from_mv(vneg_mv as i16))
}

#[cfg(all(test, feature = "current-sense-bus"))]
mod tests {
    use super::*;

    #[test]
    fn test_current_conversion() {
        let cal = BoardCalibration::default();
        // Test with nominal VDDA = 3.3V
        let vdda_mv = 3300;
        
        // Test cases with expected values for 1.5kΩ RIPROPI
        // At 200mA: IPROPI = 200mA * 1500µA/A = 0.3mA through 1.5kΩ = 0.45V
        // ADC = 0.45V / 3.3V * 4095 = 559
        let ma = cal.current_from_adc(559, vdda_mv);
        assert!((ma.as_ma() - 200).abs() < 5, "200mA test failed: got {}mA", ma.as_ma());
        
        // At 800mA: IPROPI = 800mA * 1500µA/A = 1.2mA through 1.5kΩ = 1.8V
        // ADC = 1.8V / 3.3V * 4095 = 2234
        let ma = cal.current_from_adc(2234, vdda_mv);
        assert!((ma.as_ma() - 800).abs() < 5, "800mA test failed: got {}mA", ma.as_ma());
        
        // At 0mA: ADC = 0
        let ma = cal.current_from_adc(0, vdda_mv);
        assert_eq!(ma.as_ma(), 0, "0mA test failed");
        
        // Test with different VDDA (3.0V)
        let vdda_mv = 3000;
        // At 500mA with 3.0V VDDA: 0.75V / 3.0V * 4095 = 1024
        let ma = cal.current_from_adc(1024, vdda_mv);
        assert!((ma.as_ma() - 500).abs() < 5, "500mA @ 3.0V test failed: got {}mA", ma.as_ma());
    }
    
    #[test]
    fn test_position_conversion() {
        let cal = BoardCalibration::default();
        
        // Test endpoints
        assert_eq!(cal.position_from_adc(0).as_cdeg(), -500, "Min position");
        assert_eq!(cal.position_from_adc(4095).as_cdeg(), 18500, "Max position");
        
        // Test midpoint (should be around 90°)
        let mid = cal.position_from_adc(2048);
        assert!((mid.as_cdeg() - 9000).abs() < 50, "Midpoint position");
    }
}