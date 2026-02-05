//! NTC thermistor temperature conversion using lookup table.
//!
//! No floating point - pure integer math with linear interpolation.
//!
//! The lookup table is board-specific and should be defined in the board crate.
//! Tables can be generated at compile-time using the `ntc_gen` module.

/// Convert ADC reading to temperature in centi-Celsius using a lookup table.
///
/// Assumes low-side NTC configuration:
/// - VCC → R_fixed → ADC → NTC → GND
/// - ADC decreases as temperature increases
/// - Table is sorted by descending ADC (ascending temperature)
///
/// # Arguments
/// * `adc_raw` - 12-bit ADC reading (0-4095)
/// * `lut` - Lookup table of (ADC value, temperature in centi-Celsius) pairs,
///           sorted by descending ADC value
///
/// # Returns
/// Temperature in centi-Celsius (e.g., 2500 = 25.00°C)
/// Clamped to the table's min/max range.
pub fn ntc_lut_to_centi_celsius(adc_raw: u16, lut: &[(u16, i16)]) -> i16 {
    // Handle empty table
    if lut.is_empty() {
        return 2500; // Default to 25°C
    }

    // Handle out-of-range: ADC too high = very cold
    if adc_raw >= lut[0].0 {
        return lut[0].1;
    }
    // Handle out-of-range: ADC too low = very hot
    if adc_raw <= lut[lut.len() - 1].0 {
        return lut[lut.len() - 1].1;
    }

    // Find bracketing entries (table sorted by descending ADC)
    for i in 0..lut.len() - 1 {
        let (adc_hi, temp_lo) = lut[i];
        let (adc_lo, temp_hi) = lut[i + 1];

        if adc_raw <= adc_hi && adc_raw > adc_lo {
            // Linear interpolation:
            // temp = temp_lo + (temp_hi - temp_lo) * (adc_hi - adc_raw) / (adc_hi - adc_lo)
            let adc_range = (adc_hi - adc_lo) as i32;
            let temp_range = (temp_hi - temp_lo) as i32;
            let adc_offset = (adc_hi - adc_raw) as i32;

            let temp = temp_lo as i32 + (temp_range * adc_offset / adc_range);
            return temp as i16;
        }
    }

    // Fallback (shouldn't reach here with valid input)
    2500 // 25.00°C
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test lookup table: 10K B3950 NTC with 10K fixed resistor, low-side
    const TEST_LUT: [(u16, i16); 5] = [
        (3133, 0),    //   0°C
        (2274, 2000), //  20°C
        (2048, 2500), //  25°C
        (1826, 3000), //  30°C
        (1078, 5000), //  50°C
    ];

    #[test]
    fn test_exact_table_values() {
        assert_eq!(ntc_lut_to_centi_celsius(2048, &TEST_LUT), 2500); // 25°C
        assert_eq!(ntc_lut_to_centi_celsius(3133, &TEST_LUT), 0); // 0°C
        assert_eq!(ntc_lut_to_centi_celsius(1078, &TEST_LUT), 5000); // 50°C
    }

    #[test]
    fn test_interpolation() {
        // Test interpolation between 20°C (2274) and 25°C (2048)
        // Midpoint ADC = (2274 + 2048) / 2 = 2161
        // Expected temp = (2000 + 2500) / 2 = 2250 (22.50°C)
        let temp = ntc_lut_to_centi_celsius(2161, &TEST_LUT);
        assert!((temp - 2250).abs() <= 50, "Expected ~2250, got {}", temp);
    }

    #[test]
    fn test_out_of_range_cold() {
        // ADC higher than table max = clamp to coldest
        assert_eq!(ntc_lut_to_centi_celsius(4000, &TEST_LUT), 0); // 0°C (coldest in table)
    }

    #[test]
    fn test_out_of_range_hot() {
        // ADC lower than table min = clamp to hottest
        assert_eq!(ntc_lut_to_centi_celsius(200, &TEST_LUT), 5000); // 50°C (hottest in table)
    }

    #[test]
    fn test_empty_table() {
        assert_eq!(ntc_lut_to_centi_celsius(2048, &[]), 2500); // Default 25°C
    }
}
