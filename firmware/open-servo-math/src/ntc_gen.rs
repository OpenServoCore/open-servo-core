//! Compile-time NTC thermistor lookup table generation.
//!
//! This module provides const functions to generate NTC thermistor lookup tables
//! at compile time based on thermistor parameters (R25, B constant) and circuit
//! configuration (pullup resistor value).
//!
//! Since these are const functions that run at compile time, we can use f32
//! for accurate calculations. The final output is integer-based for runtime use.

/// Generate NTC lookup table at compile time.
///
/// Uses the Beta equation: R(T) = R25 * exp(B * (1/T - 1/T25))
///
/// # Type Parameters
/// * `N` - Number of entries in the lookup table
///
/// # Arguments
/// * `r25` - Resistance at 25°C in ohms
/// * `b` - B constant in Kelvin
/// * `r_pullup` - Pullup resistor in ohms
/// * `temp_min` - Min temperature in °C
/// * `temp_max` - Max temperature in °C
///
/// # Returns
/// Array of (ADC value, temperature in centi-Celsius) pairs,
/// sorted by descending ADC value (ascending temperature).
///
/// # Example
/// ```
/// const LUT: [(u16, i16); 13] = generate_ntc_lut::<13>(
///     10_000.0,  // 10kΩ at 25°C
///     3380.0,    // B constant
///     10_000.0,  // 10kΩ pullup
///     -40.0,     // -40°C min
///     125.0,     // 125°C max
/// );
/// ```
pub const fn generate_ntc_lut<const N: usize>(
    r25: f32,
    b: f32,
    r_pullup: f32,
    temp_min: f32,
    temp_max: f32,
) -> [(u16, i16); N] {
    let mut lut = [(0u16, 0i16); N];
    
    let temp_range = temp_max - temp_min;
    
    let mut i = 0;
    while i < N {
        // Calculate temperature for this table entry (evenly spaced)
        let temp_c = if N > 1 {
            temp_min + (temp_range * i as f32) / (N - 1) as f32
        } else {
            temp_min
        };
        
        // Convert to Kelvin
        let temp_k = temp_c + 273.15;
        
        // Calculate NTC resistance using Beta equation
        // R(T) = R25 * exp(B * (1/T - 1/298.15))
        let exponent = b * (1.0 / temp_k - 1.0 / 298.15);
        let r_ntc = r25 * exp_f32(exponent);
        
        // Calculate ADC value for voltage divider
        // ADC = 4095 * R_ntc / (R_pullup + R_ntc)
        let adc_f = 4095.0 * r_ntc / (r_pullup + r_ntc);
        let adc = if adc_f > 4095.0 {
            4095
        } else if adc_f < 0.0 {
            0
        } else {
            adc_f as u16
        };
        
        // Store temperature in centi-Celsius (multiply by 100)
        let temp_cc = (temp_c * 100.0) as i16;
        
        lut[i] = (adc, temp_cc);
        i += 1;
    }
    
    // Sort by descending ADC value (ascending temperature)
    // Using simple bubble sort since it's const-compatible
    let mut sorted = lut;
    let mut i = 0;
    while i < N {
        let mut j = 0;
        while j < N - 1 - i {
            if sorted[j].0 < sorted[j + 1].0 {
                let temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
            j += 1;
        }
        i += 1;
    }
    
    sorted
}

/// Const function to calculate e^x using Taylor series.
/// Good accuracy for typical NTC calculations.
const fn exp_f32(x: f32) -> f32 {
    // For const context, we'll use a Taylor series approximation
    // exp(x) = 1 + x + x²/2! + x³/3! + x⁴/4! + x⁵/5! + ...
    // This gives good accuracy for the typical range we need
    
    if x == 0.0 {
        return 1.0;
    }
    
    let mut result = 1.0;
    let mut term = 1.0;
    let mut i = 1;
    
    // Use 12 terms for good accuracy
    while i <= 12 {
        term = term * x / i as f32;
        result += term;
        i += 1;
    }
    
    result
}

/// Pre-configured generator for common NTC thermistors.
pub mod presets {
    use super::generate_ntc_lut;
    
    /// Generate LUT for 10K NTC with B=3380 (Murata NCP15XH103F03RC / LCSC C77131).
    /// 
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// const LUT: [(u16, i16); 16] = ntc_10k_b3380::<16>(10_000.0);
    /// ```
    pub const fn ntc_10k_b3380<const N: usize>(r_pullup: f32) -> [(u16, i16); N] {
        generate_ntc_lut::<N>(
            10_000.0,  // 10kΩ at 25°C
            3380.0,    // B=3380K
            r_pullup,
            -40.0,     // -40°C min
            125.0,     // 125°C max
        )
    }
    
    /// Generate LUT for 10K NTC with B=3950 (common thermistor).
    /// 
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// const LUT: [(u16, i16); 16] = ntc_10k_b3950::<16>(10_000.0);
    /// ```
    pub const fn ntc_10k_b3950<const N: usize>(r_pullup: f32) -> [(u16, i16); N] {
        generate_ntc_lut::<N>(
            10_000.0,  // 10kΩ at 25°C
            3950.0,    // B=3950K
            r_pullup,
            -40.0,     // -40°C min
            125.0,     // 125°C max
        )
    }
    
    /// Generate LUT with custom temperature range.
    /// 
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// const LUT: [(u16, i16); 20] = ntc_custom::<20>(
    ///     10_000.0, 3380.0, 10_000.0, 0.0, 100.0
    /// );
    /// ```
    pub const fn ntc_custom<const N: usize>(
        r25: f32,
        b: f32,
        r_pullup: f32,
        temp_min: f32,
        temp_max: f32,
    ) -> [(u16, i16); N] {
        generate_ntc_lut::<N>(r25, b, r_pullup, temp_min, temp_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_exp_f32() {
        // Test our const exp function
        const E0: f32 = exp_f32(0.0);
        assert!((E0 - 1.0).abs() < 0.01);
        
        const E1: f32 = exp_f32(1.0);
        assert!((E1 - 2.71828).abs() < 0.01);
        
        const E_NEG1: f32 = exp_f32(-1.0);
        assert!((E_NEG1 - 0.36788).abs() < 0.01);
    }
    
    #[test]
    fn test_generate_small_lut() {
        const LUT: [(u16, i16); 5] = generate_ntc_lut::<5>(
            10_000.0,  // 10kΩ at 25°C
            3380.0,    // B constant
            10_000.0,  // 10kΩ pullup
            0.0,       // 0°C min
            100.0,     // 100°C max
        );
        
        // Check that table is sorted by descending ADC
        for i in 0..LUT.len() - 1 {
            assert!(LUT[i].0 >= LUT[i + 1].0);
        }
        
        // Check temperature range
        assert_eq!(LUT[0].1, 0);        // First entry should be 0°C
        assert_eq!(LUT[4].1, 10000);    // Last entry should be 100°C
        
        // Check that 25°C gives approximately 2048 ADC (half scale with equal resistors)
        // Find the entry closest to 25°C (2500 centi-celsius)
        let closest_25c = LUT.iter()
            .min_by_key(|(_, temp)| (*temp as i32 - 2500).abs())
            .unwrap();
        
        // Should be close to 2048 (half of 4095)
        assert!((closest_25c.0 as i32 - 2048).abs() < 100);
    }
    
    #[test]
    fn test_presets() {
        const LUT_3380: [(u16, i16); 8] = presets::ntc_10k_b3380::<8>(10_000.0);
        const LUT_3950: [(u16, i16); 8] = presets::ntc_10k_b3950::<8>(10_000.0);
        
        // Tables should be different due to different B constants
        assert_ne!(LUT_3380[4], LUT_3950[4]);
        
        // Both should be sorted
        for i in 0..7 {
            assert!(LUT_3380[i].0 >= LUT_3380[i + 1].0);
            assert!(LUT_3950[i].0 >= LUT_3950[i + 1].0);
        }
    }
}