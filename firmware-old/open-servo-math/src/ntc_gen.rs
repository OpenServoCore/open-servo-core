//! Compile-time NTC thermistor lookup table generation with ADC-spaced entries.
//!
//! This module provides const functions to generate NTC thermistor lookup tables
//! at compile time based on thermistor parameters (R25, B constant) and circuit
//! configuration (pullup resistor value).
//!
//! Tables are spaced evenly by ADC codes, providing uniform ADC resolution
//! across the entire temperature range. Since temperature readings always come
//! from ADC values, this approach makes the most sense.

/// Generate NTC lookup table with evenly-spaced ADC values.
///
/// Uses the Beta equation: R(T) = R25 * exp(B * (1/T - 1/T25))
///
/// This spaces entries evenly across ADC codes, which gives:
/// - Uniform ADC resolution across entire range
/// - More temperature points in hot region (where curve is flat)
/// - Fewer temperature points in cold region (where curve is steep)
/// - Optimal for ADC-based temperature readings
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
/// use open_servo_math::ntc_gen::generate_ntc_lut;
/// const LUT: [(u16, i16); 16] = generate_ntc_lut::<16>(
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

    // First, calculate ADC values at min and max temperatures
    let temp_min_k = temp_min + 273.15;
    let temp_max_k = temp_max + 273.15;

    let r_min = r25 * exp_f32(b * (1.0 / temp_max_k - 1.0 / 298.15)); // Hot = low R
    let r_max = r25 * exp_f32(b * (1.0 / temp_min_k - 1.0 / 298.15)); // Cold = high R

    let adc_min = (4095.0 * r_min / (r_pullup + r_min)) as u16;
    let adc_max = (4095.0 * r_max / (r_pullup + r_max)) as u16;

    let mut i = 0;
    while i < N {
        // Calculate evenly spaced ADC value
        let adc = if N > 1 {
            let adc_f = adc_max as f32 - ((adc_max - adc_min) as f32 * i as f32 / (N - 1) as f32);
            adc_f as u16
        } else {
            (adc_min + adc_max) / 2
        };

        // Back-calculate temperature from ADC value
        // ADC = 4095 * R_ntc / (R_pullup + R_ntc)
        // Solve for R_ntc: R_ntc = ADC * R_pullup / (4095 - ADC)
        let r_ntc = if adc < 4095 {
            (adc as f32 * r_pullup) / (4095.0 - adc as f32)
        } else {
            1000000.0 // Very high resistance
        };

        // Back-calculate temperature from resistance
        // R(T) = R25 * exp(B * (1/T - 1/298.15))
        // ln(R/R25) = B * (1/T - 1/298.15)
        // 1/T = ln(R/R25)/B + 1/298.15
        let ln_ratio = ln_f32(r_ntc / r25);
        let inv_t = ln_ratio / b + 1.0 / 298.15;
        let temp_k = 1.0 / inv_t;
        let temp_c = temp_k - 273.15;

        // Clamp to specified range
        let temp_c_clamped = if temp_c < temp_min {
            temp_min
        } else if temp_c > temp_max {
            temp_max
        } else {
            temp_c
        };

        let temp_cc = (temp_c_clamped * 100.0) as i16;

        lut[i] = (adc, temp_cc);
        i += 1;
    }

    lut // Already sorted by construction (descending ADC)
}

/// Const function to calculate e^x using Taylor series.
/// Good accuracy for typical NTC calculations.
const fn exp_f32(x: f32) -> f32 {
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

/// Const function to calculate ln(x) using Taylor series.
/// Used for back-calculating temperature from resistance.
const fn ln_f32(x: f32) -> f32 {
    if x <= 0.0 {
        return -1000.0; // Return large negative for invalid input
    }

    if x == 1.0 {
        return 0.0;
    }

    // For better convergence, use ln(x) = ln(1 + y) where y = x - 1
    // But first scale x to be near 1 by using: ln(x) = ln(x/2^n) + n*ln(2)
    let mut scaled_x = x;
    let mut scale_count = 0;

    // Scale down if x >= 2 (keep in range [0.5, 2.0) for better convergence)
    while scaled_x >= 2.0 {
        scaled_x = scaled_x / 2.0;
        scale_count += 1;
    }

    // Scale up if x < 0.5
    while scaled_x < 0.5 {
        scaled_x = scaled_x * 2.0;
        scale_count -= 1;
    }

    // Now scaled_x is in [0.5, 2.0], use Taylor series
    // ln(1 + y) = y - y²/2 + y³/3 - y⁴/4 + ...
    let y = scaled_x - 1.0;
    let mut result = 0.0;
    let mut term = y;
    let mut i = 1;

    while i <= 20 {
        if i % 2 == 1 {
            result += term / i as f32;
        } else {
            result -= term / i as f32;
        }
        term = term * y;
        i += 1;

        // Stop if term becomes very small
        if term.abs() < 0.00001 {
            break;
        }
    }

    // Add back the scaling: ln(x) = result + scale_count * ln(2)
    const LN2: f32 = 0.693147;
    result + (scale_count as f32 * LN2)
}

/// Pre-configured generators for common NTC thermistors.
pub mod presets {
    use super::generate_ntc_lut;

    /// Generate LUT for 10K NTC with B=3380 (Murata NCP15XH103F03RC / LCSC C77131).
    ///
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// use open_servo_math::ntc_gen::presets::ntc_10k_b3380;
    /// const LUT: [(u16, i16); 16] = ntc_10k_b3380::<16>(10_000.0);
    /// ```
    pub const fn ntc_10k_b3380<const N: usize>(r_pullup: f32) -> [(u16, i16); N] {
        generate_ntc_lut::<N>(
            10_000.0, // 10kΩ at 25°C
            3380.0,   // B=3380K
            r_pullup, -40.0, // -40°C min
            125.0, // 125°C max
        )
    }

    /// Generate LUT for 10K NTC with B=3950 (common thermistor).
    ///
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// use open_servo_math::ntc_gen::presets::ntc_10k_b3950;
    /// const LUT: [(u16, i16); 16] = ntc_10k_b3950::<16>(10_000.0);
    /// ```
    pub const fn ntc_10k_b3950<const N: usize>(r_pullup: f32) -> [(u16, i16); N] {
        generate_ntc_lut::<N>(
            10_000.0, // 10kΩ at 25°C
            3950.0,   // B=3950K
            r_pullup, -40.0, // -40°C min
            125.0, // 125°C max
        )
    }

    /// Generate LUT with custom parameters.
    ///
    /// # Type Parameters
    /// * `N` - Number of table entries
    ///
    /// # Example
    /// ```
    /// use open_servo_math::ntc_gen::presets::ntc_custom;
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
    fn test_ln_f32() {
        // Test our const ln function
        const LN1: f32 = ln_f32(1.0);
        assert_eq!(LN1, 0.0);

        const LN_E: f32 = ln_f32(2.71828);
        assert!((LN_E - 1.0).abs() < 0.01);

        const LN2: f32 = ln_f32(2.0);
        assert!((LN2 - 0.693).abs() < 0.01);
    }

    #[test]
    fn test_adc_spaced_lut() {
        const LUT: [(u16, i16); 5] = generate_ntc_lut::<5>(
            10_000.0, // 10kΩ at 25°C
            3380.0,   // B constant
            10_000.0, // 10kΩ pullup
            0.0,      // 0°C min
            100.0,    // 100°C max
        );

        // Check that ADC values are evenly spaced
        let spacing = (LUT[0].0 - LUT[4].0) / 4;
        for i in 0..4 {
            let expected_spacing = (LUT[i].0 - LUT[i + 1].0) as i32;
            let diff = (expected_spacing - spacing as i32).abs();
            assert!(diff <= 1); // Allow ±1 for rounding
        }

        // Check temperature is monotonic
        for i in 0..LUT.len() - 1 {
            assert!(LUT[i].1 <= LUT[i + 1].1); // Temperature increases
            assert!(LUT[i].0 >= LUT[i + 1].0); // ADC decreases
        }
    }

    #[test]
    fn test_presets() {
        const LUT_3380: [(u16, i16); 8] = presets::ntc_10k_b3380::<8>(10_000.0);
        const LUT_3950: [(u16, i16); 8] = presets::ntc_10k_b3950::<8>(10_000.0);

        // Tables should be different due to different B constants
        assert_ne!(LUT_3380[4], LUT_3950[4]);

        // Both should be sorted by descending ADC
        for i in 0..7 {
            assert!(LUT_3380[i].0 >= LUT_3380[i + 1].0);
            assert!(LUT_3950[i].0 >= LUT_3950[i + 1].0);
        }
    }

    #[test]
    fn test_edge_cases() {
        // Single entry table
        const LUT_1: [(u16, i16); 1] =
            generate_ntc_lut::<1>(10_000.0, 3380.0, 10_000.0, 25.0, 25.0);
        assert_eq!(LUT_1[0].1, 2500); // 25°C in centi-celsius

        // Check reasonable ADC value at 25°C (should be near 2048)
        assert!((LUT_1[0].0 as i32 - 2048).abs() < 100);
    }
}
