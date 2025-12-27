//! ADC to physical unit conversions.
//!
//! Board-specific calibration constants and conversion functions.

use open_servo_units::CentiDeg32;

/// ADC resolution (12-bit).
pub const ADC_MAX: u16 = 4095;

/// Position sensor calibration.
///
/// Maps ADC reading to centidegrees.
/// Stage-0: linear mapping assuming full-range potentiometer.
///
/// Position range: 0-300 degrees (30000 centidegrees)
pub fn adc_to_position(adc: u16) -> CentiDeg32 {
    // Linear interpolation: 0 → 0°, 4095 → 300°
    // position_cdeg = adc * 30000 / 4095
    let cdeg = (adc as i32 * 30000) / ADC_MAX as i32;
    CentiDeg32::from_cdeg(cdeg)
}

/// Current sense calibration (if enabled).
///
/// Maps ADC reading to milliamps.
#[cfg(feature = "current-sense-bus")]
pub fn adc_to_current_ma(adc: u16) -> i32 {
    // Stage-0: placeholder calibration.
    // Actual calibration depends on shunt resistor and op-amp gain.
    // Example: 0.1Ω shunt, 20x gain, 3.3V reference
    // I = (adc / 4095) * 3.3V / 20 / 0.1Ω = adc * 40.27 mA
    (adc as i32 * 40) / 1000
}

/// VREFINT to supply voltage.
///
/// Use VREFINT calibration to compute actual Vdd.
pub fn vrefint_to_vdd_mv(vrefint_adc: u16) -> u16 {
    // VREFINT typical = 1.20V
    // vrefint_adc = (1.2V / Vdd) * 4095
    // Vdd = 1.2V * 4095 / vrefint_adc
    // Vdd_mV = 1200 * 4095 / vrefint_adc
    if vrefint_adc == 0 {
        return 3300; // Fallback
    }
    (1200 * 4095 / vrefint_adc as u32) as u16
}
