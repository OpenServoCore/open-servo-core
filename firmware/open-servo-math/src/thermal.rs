//! Thermal model for resistive heating and cooling.
//!
//! Estimates temperature using:
//! - Heating: I²R losses (Joule heating)
//! - Cooling: Newton's law of cooling
//! - Thermal mass: Heat capacity dynamics
//!
//! All calculations use fixed-point arithmetic for embedded efficiency.

/// Thermal physics model using Q16 fixed-point math.
///
/// Models temperature dynamics of a resistive element (e.g., motor windings)
/// based on current flow and environmental cooling.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThermalModel {
    /// Electrical resistance in milliohms (5000 = 5.0Ω)
    resistance_mohm: i16,
    
    /// Thermal resistance in centi-°C/W (1000 = 10°C/W)
    /// Higher value = harder to dissipate heat
    thermal_resistance_cw: i16,
    
    /// Thermal capacity in centi-J/°C (1500 = 15 J/°C)
    /// Higher value = slower to heat up/cool down
    thermal_capacity_cj: i16,
    
    /// Estimated temperature in Q16.16 centi-degrees
    /// (2500 << 16 = 25.00°C)
    /// Q16 preserves sub-centi-degree precision
    temp_q16_cdeg: i32,
    
    /// Ambient temperature in centi-degrees
    ambient_temp_cdeg: i16,
    
    /// I² accumulator for averaging between slow ticks
    i_squared_accumulator: u64,
    
    /// Number of I² samples accumulated
    i_squared_samples: u16,
}

impl ThermalModel {
    /// Create a new thermal model with custom parameters.
    ///
    /// # Arguments
    /// * `resistance_mohm` - Electrical resistance in milliohms
    /// * `thermal_resistance_cw` - Thermal resistance in centi-°C/W
    /// * `thermal_capacity_cj` - Heat capacity in centi-J/°C
    pub fn new(
        resistance_mohm: i16,
        thermal_resistance_cw: i16,
        thermal_capacity_cj: i16,
    ) -> Self {
        Self {
            resistance_mohm,
            thermal_resistance_cw,
            thermal_capacity_cj,
            temp_q16_cdeg: 2500 << 16,  // Start at 25°C
            ambient_temp_cdeg: 2500,
            i_squared_accumulator: 0,
            i_squared_samples: 0,
        }
    }
    
    /// Accumulate current measurement (fast tick - typically 10kHz).
    ///
    /// Call this at high frequency to accumulate I² samples for averaging.
    /// The accumulated values will be processed in `update_slow`.
    ///
    /// # Arguments
    /// * `current_ma` - Current in milliamps (None if no measurement)
    #[inline]
    pub fn update_fast(&mut self, current_ma: Option<i16>) {
        if let Some(i_ma) = current_ma {
            let i_squared = (i_ma as i32 * i_ma as i32) as u64;
            self.i_squared_accumulator = self.i_squared_accumulator.saturating_add(i_squared);
            self.i_squared_samples = self.i_squared_samples.saturating_add(1);
        }
    }
    
    /// Apply thermal physics and return temperature (slow tick - typically 100Hz).
    ///
    /// Processes accumulated I² samples, applies heating/cooling physics,
    /// and updates the temperature estimate.
    ///
    /// # Arguments
    /// * `ambient_cdeg` - Ambient temperature in centi-degrees
    ///
    /// # Returns
    /// Current temperature estimate in centi-degrees
    pub fn update_slow(&mut self, ambient_cdeg: i16) -> i16 {
        self.ambient_temp_cdeg = ambient_cdeg;
        
        // Calculate average I² if we have samples
        let avg_i_squared_ma2 = if self.i_squared_samples > 0 {
            (self.i_squared_accumulator / self.i_squared_samples as u64) as u32
        } else {
            0
        };
        
        // Reset accumulators for next period
        self.i_squared_accumulator = 0;
        self.i_squared_samples = 0;
        
        // Calculate heating power in milli-watts
        // P_mW = I²_mA² × R_mΩ / 1,000,000
        let power_mw = ((avg_i_squared_ma2 as i64) * (self.resistance_mohm as i64)) / 1_000_000;
        
        // Calculate cooling power in milli-watts
        // Get current temp in centi-degrees from Q16
        let temp_cdeg = (self.temp_q16_cdeg >> 16) as i16;
        let temp_delta = temp_cdeg.saturating_sub(ambient_cdeg) as i32;
        
        // P_out_mW = ΔT_cdeg × 1000 / R_cw
        let power_out_mw = (temp_delta * 1000) / (self.thermal_resistance_cw as i32);
        
        // Net power in milli-watts
        let net_power_mw = power_mw as i32 - power_out_mw;
        
        // Temperature change for 10ms period (100Hz tick)
        // ΔT_cdeg = P_mW × 10 / C_cJ
        // In Q16: ΔT_q16 = (P_mW × 10 << 16) / C_cJ
        let temp_change_q16 = ((net_power_mw * 10) << 16) / (self.thermal_capacity_cj as i32);
        
        // Update temperature in Q16
        self.temp_q16_cdeg = self.temp_q16_cdeg.saturating_add(temp_change_q16);
        
        // Clamp to reasonable range
        self.temp_q16_cdeg = self.temp_q16_cdeg.clamp(-4000 << 16, 20000 << 16);
        
        // Return current temperature
        (self.temp_q16_cdeg >> 16) as i16
    }
    
    /// Get current temperature estimate in centi-degrees.
    pub fn temperature_cdeg(&self) -> i16 {
        (self.temp_q16_cdeg >> 16) as i16
    }
    
    /// Get temperature in degrees Celsius.
    pub fn temperature_deg(&self) -> i16 {
        self.temperature_cdeg() / 100
    }
    
    /// Get temperature rise above ambient in degrees Celsius.
    pub fn temp_rise_deg(&self) -> i16 {
        (self.temperature_cdeg() - self.ambient_temp_cdeg) / 100
    }
    
    /// Initialize temperature to ambient.
    ///
    /// Call this on startup or after extended idle period.
    pub fn init(&mut self, ambient_cdeg: i16) {
        self.ambient_temp_cdeg = ambient_cdeg;
        self.temp_q16_cdeg = (ambient_cdeg as i32) << 16;
        self.i_squared_accumulator = 0;
        self.i_squared_samples = 0;
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_basic_heating() {
        let mut model = ThermalModel::new(5000, 1000, 1500);
        model.init(2500); // 25°C ambient
        
        // Apply 500mA for 100 fast ticks (10ms at 10kHz)
        for _ in 0..100 {
            model.update_fast(Some(500));
        }
        
        let initial = model.temperature_cdeg();
        let final_temp = model.update_slow(2500);
        
        // Should heat up slightly
        assert!(final_temp > initial);
    }
    
    #[test]
    fn test_cooling() {
        let mut model = ThermalModel::new(5000, 1000, 1500);
        model.temp_q16_cdeg = 5000 << 16; // Start at 50°C
        model.ambient_temp_cdeg = 2500; // 25°C ambient
        
        // No current for multiple slow ticks
        for _ in 0..500 {
            // 100 fast ticks with no current
            for _ in 0..100 {
                model.update_fast(None);
            }
            model.update_slow(2500);
        }
        
        let final_temp = model.temperature_cdeg();
        assert!(final_temp < 4000, "Should cool down from 50°C (got {} cdeg)", final_temp);
        assert!(final_temp > 2500, "Should not cool below ambient");
    }
    
    #[test]
    fn test_steady_state() {
        let mut model = ThermalModel::new(5000, 1000, 1500);
        model.init(2500); // 25°C
        
        // Apply constant 200mA for many cycles
        for _ in 0..10000 {
            // 100 fast ticks
            for _ in 0..100 {
                model.update_fast(Some(200));
            }
            model.update_slow(2500);
        }
        
        let rise = model.temp_rise_deg();
        // 200mA² × 5Ω = 0.2W, with 10°C/W = 2°C rise
        assert!(rise >= 1 && rise <= 3, "Should reach ~2°C steady state (got {}°C)", rise);
    }
    
    #[test]
    fn test_no_samples() {
        let mut model = ThermalModel::new(5000, 1000, 1500);
        model.init(3000); // 30°C
        
        // Call slow update without any fast updates
        let temp = model.update_slow(2500); // 25°C ambient
        
        // Should cool toward ambient
        assert!(temp < 3000);
    }
}