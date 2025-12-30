//! ADC to physical unit conversions.
//!
//! Board-specific calibration constants and conversion functions.
//! Uses factory calibration data from STM32F301 flash for accurate readings.

use open_servo_units::CentiDeg32;

// =============================================================================
// ADC Configuration
// =============================================================================

/// ADC resolution (12-bit).
pub const ADC_MAX: u16 = 4095;

// =============================================================================
// Factory Calibration Addresses (STM32F301)
// =============================================================================

/// Factory VREFINT calibration value address (measured at 3.3V, 30°C).
pub const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;

/// Factory temperature sensor calibration at 30°C.
#[cfg(feature = "temp-sense-mcu")]
pub const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;

/// Factory temperature sensor calibration at 110°C.
#[cfg(feature = "temp-sense-mcu")]
pub const TS_CAL2_ADDR: u32 = 0x1FFFF7C2;

// =============================================================================
// DRV8231A Current Sense Configuration
// =============================================================================

/// IPROPI sense resistor value in ohms.
#[cfg(feature = "current-sense-bus")]
pub const RIPROPI_OHMS: u32 = 1500; // 1.5kΩ

/// DRV8231A current mirror gain in µA/A.
#[cfg(feature = "current-sense-bus")]
pub const IPROPI_GAIN_UA_PER_A: u32 = 1500;

// =============================================================================
// Position Sensor Configuration
// =============================================================================

/// Potentiometer minimum position in centidegrees.
pub const POT_MIN_CDEG: i32 = -500; // -5°

/// Potentiometer maximum position in centidegrees.
pub const POT_MAX_CDEG: i32 = 18500; // 185°

// =============================================================================
// Conversion Functions
// =============================================================================

/// Convert VREFINT ADC reading to VDDA voltage in millivolts.
///
/// Uses factory calibration value for accuracy.
/// Returns i16 to match MilliVolt backing type (max 32.767V, sufficient for MCU VDD).
pub fn vdd_from_vrefint(vref_raw: u16) -> i16 {
    if vref_raw == 0 {
        return 3300; // Fallback to nominal
    }
    // Read factory calibration value (measured at 3.3V)
    let vrefint_cal: u16 = unsafe { core::ptr::read(VREFINT_CAL_ADDR as *const u16) };
    // vdda_mv = 3300 * vrefint_cal / vref_raw
    let result = (3300u32 * vrefint_cal as u32) / vref_raw as u32;
    // Safe: VDD is always < 5V = 5000mV, well within i16 range
    result as i16
}

/// Convert potentiometer ADC reading to position in centidegrees.
pub fn adc_to_position(adc: u16) -> CentiDeg32 {
    // Map ADC range [0, ADC_MAX] to position range [POT_MIN_CDEG, POT_MAX_CDEG]
    let adc = adc as i32;
    let range = POT_MAX_CDEG - POT_MIN_CDEG;
    let cdeg = POT_MIN_CDEG + (adc * range) / ADC_MAX as i32;
    CentiDeg32::from_cdeg(cdeg)
}

/// Convert IPROPI ADC reading to motor current in milliamps.
///
/// Uses VDDA compensation for accuracy.
#[cfg(feature = "current-sense-bus")]
pub fn adc_to_current_ma(adc: u16, vdda_mv: i16) -> i16 {
    // DRV8231A IPROPI: motor current mirrors to IPROPI pin with gain
    // I_ipropi = I_motor * (IPROPI_GAIN_UA_PER_A * 1e-6)
    // V_ipropi = I_ipropi * RIPROPI_OHMS
    // ADC = V_ipropi / VDDA * ADC_MAX
    //
    // Solving for I_motor:
    // I_motor_mA = ADC * VDDA_mV * 1_000_000 / (ADC_MAX * RIPROPI * GAIN)
    let adc = adc as u64;
    let vdda = vdda_mv as u64;
    let num = adc * vdda * 1_000_000;
    let den = (ADC_MAX as u64) * (RIPROPI_OHMS as u64) * (IPROPI_GAIN_UA_PER_A as u64);

    if den > 0 {
        (num / den) as i16
    } else {
        0
    }
}

/// Convert MCU internal temperature sensor ADC reading to centi-Celsius.
///
/// Uses factory calibration points for accuracy.
#[cfg(feature = "temp-sense-mcu")]
pub fn adc_to_mcu_temp(adc: u16, vdda_mv: i16) -> i16 {
    // Factory calibration points: 30°C and 110°C at 3.3V
    let ts_cal1: u16 = unsafe { core::ptr::read(TS_CAL1_ADDR as *const u16) };
    let ts_cal2: u16 = unsafe { core::ptr::read(TS_CAL2_ADDR as *const u16) };

    // Normalize ADC reading to 3.3V reference
    if vdda_mv == 0 || ts_cal2 <= ts_cal1 {
        return 2500; // Fallback to 25.00°C
    }

    let adc_normalized = (adc as i32 * 3300) / vdda_mv as i32;
    let cal_diff = ts_cal2 as i32 - ts_cal1 as i32;
    let adc_diff = adc_normalized - ts_cal1 as i32;

    // Linear interpolation: temp_c = 30 + (adc - cal1) * 80 / (cal2 - cal1)
    let temp_c = 30 + (adc_diff * 80) / cal_diff;

    // Return in centi-Celsius, clamped to reasonable range
    let temp_cc = temp_c * 100;
    temp_cc.clamp(-4000, 12500) as i16
}

/// Convert motor voltage ADC reading to millivolts.
///
/// Assumes voltage divider maps 0-5V motor voltage to full ADC range.
/// Returns i16 to match MilliVolt backing type.
#[cfg(feature = "voltage-sense-motor")]
pub fn adc_to_motor_mv(adc: u16, _vdda_mv: i16) -> i16 {
    // Each terminal: 0-4095 ADC = 0-5000mV (assuming 5V max with divider)
    let mv = (adc as u32 * 5000) / ADC_MAX as u32;
    // Safe: motor voltage < 5V = 5000mV, well within i16 range
    mv as i16
}

/// NTC thermistor lookup table for 10K B3950.
///
/// Table entries: (ADC value, temperature in centi-Celsius)
/// Assumes 10K NTC with 10K fixed resistor, 3.3V reference.
#[cfg(feature = "temp-sense-motor")]
const NTC_LUT: [(u16, i16); 13] = [
    (100, 12500),   // ~125°C
    (200, 10500),   // ~105°C
    (400, 8500),    // ~85°C
    (600, 7200),    // ~72°C
    (900, 6000),    // ~60°C
    (1200, 5000),   // ~50°C
    (1600, 4000),   // ~40°C
    (2048, 3000),   // ~30°C (midpoint = equal R)
    (2500, 2200),   // ~22°C
    (2900, 1500),   // ~15°C
    (3300, 800),    // ~8°C
    (3700, 0),      // ~0°C
    (4000, -1000),  // ~-10°C
];

/// Convert NTC thermistor ADC reading to temperature in centi-Celsius.
///
/// Uses linear interpolation between lookup table entries.
#[cfg(feature = "temp-sense-motor")]
pub fn adc_to_ntc_temp(adc: u16, _vdda_mv: i16) -> i16 {
    // Handle edge cases
    if adc <= NTC_LUT[0].0 {
        return NTC_LUT[0].1;
    }
    if adc >= NTC_LUT[NTC_LUT.len() - 1].0 {
        return NTC_LUT[NTC_LUT.len() - 1].1;
    }

    // Find bracketing entries
    for i in 0..NTC_LUT.len() - 1 {
        let (adc_lo, temp_lo) = NTC_LUT[i];
        let (adc_hi, temp_hi) = NTC_LUT[i + 1];

        if adc >= adc_lo && adc < adc_hi {
            // Linear interpolation
            let adc_range = (adc_hi - adc_lo) as i32;
            let temp_range = temp_hi as i32 - temp_lo as i32;
            let adc_offset = (adc - adc_lo) as i32;

            let temp = temp_lo as i32 + (adc_offset * temp_range) / adc_range;
            return temp as i16;
        }
    }

    // Fallback (shouldn't reach here)
    3000 // 30°C
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_position_endpoints() {
        assert_eq!(adc_to_position(0).as_cdeg(), POT_MIN_CDEG);
        assert_eq!(adc_to_position(ADC_MAX).as_cdeg(), POT_MAX_CDEG);
    }

    #[test]
    fn test_position_midpoint() {
        let mid = adc_to_position(ADC_MAX / 2);
        let expected = (POT_MIN_CDEG + POT_MAX_CDEG) / 2;
        assert!((mid.as_cdeg() - expected).abs() < 100);
    }

    #[test]
    fn test_vdd_fallback() {
        assert_eq!(vdd_from_vrefint(0), 3300);
    }
}
