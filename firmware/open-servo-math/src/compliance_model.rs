//! Compliance limiting model for motor current protection.
//!
//! Pure mathematical model that calculates duty cycle limits based on
//! motor current measurements. Protects mechanical components (e.g. plastic
//! gears) by limiting torque through current control to achieve compliance.
//!
//! This module contains only the mathematical state machine. Hardware
//! integration and safety logic are handled by the ComplianceLimiter in
//! the safety module.

use crate::{Duty, MilliAmp};

/// State of the compliance limiting system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LimitState {
    /// Normal operation, no limiting
    Normal,
    /// Actively limiting due to over-current
    Limiting,
    /// Recovering after limit event
    Recovering,
}

/// Configuration for compliance limiting behavior.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ComplianceConfig {
    /// Current limit threshold in milliamps
    pub limit_ma: i16,
    
    /// Hysteresis for recovery (recovery starts at limit_ma - hysteresis_ma)
    pub hysteresis_ma: i16,
    
    /// Number of consecutive over-limit samples before triggering
    pub deglitch_samples: u8,
    
    /// Backoff factor in Q8 format (256 = 1.0, 225 = 0.88)
    pub backoff_factor_q8: u16,
    
    /// Recovery rate in duty units per second
    /// E.g., 3277 = 10% of full scale per second
    pub recovery_rate: i16,
}

impl Default for ComplianceConfig {
    fn default() -> Self {
        Self {
            limit_ma: 600,              // 600mA limit - balanced for normal operation
            hysteresis_ma: 50,          // 50mA hysteresis band
            deglitch_samples: 3,        // 3 consecutive samples
            backoff_factor_q8: 230,     // 0.9 backoff factor (gentler)
            recovery_rate: 3277,        // 10% per second recovery
        }
    }
}

/// Pure mathematical compliance limiting model.
///
/// Calculates duty cycle limits based on current measurements.
/// No hardware dependencies or side effects.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ComplianceModel {
    config: ComplianceConfig,
    
    /// Current duty cycle limit (0 to Duty::MAX)
    duty_cap: i16,
    
    /// Consecutive over-current sample count
    over_cnt: u8,
    
    /// Current limiting state
    state: LimitState,
}

impl ComplianceModel {
    /// Create a new compliance model with the given configuration.
    pub fn new(config: ComplianceConfig) -> Self {
        Self {
            config,
            duty_cap: Duty::MAX.as_raw(),
            over_cnt: 0,
            state: LimitState::Normal,
        }
    }
    
    /// Create with default configuration.
    pub fn default() -> Self {
        Self::new(ComplianceConfig::default())
    }
    
    /// Update the model with a new current reading.
    ///
    /// # Parameters
    /// - `current`: Current reading in milliamps (None if sensor unavailable)
    /// - `dt_us`: Time delta in microseconds since last update
    ///
    /// # Returns
    /// Updated duty cycle limits as (min, max) tuple
    pub fn update(&mut self, current: Option<MilliAmp>, dt_us: u32) -> (i32, i32) {
        // No limiting if no current sensor
        let Some(current_ma) = current else {
            return self.get_limits();
        };
        
        let abs_current = current_ma.abs().as_ma();
        let recovery_threshold = self.config.limit_ma - self.config.hysteresis_ma;
        
        match self.state {
            LimitState::Normal => {
                if abs_current > self.config.limit_ma {
                    self.over_cnt = self.over_cnt.saturating_add(1);
                    
                    if self.over_cnt >= self.config.deglitch_samples {
                        // Trigger limiting
                        self.apply_backoff();
                        self.state = LimitState::Limiting;
                        self.over_cnt = 0;
                    }
                } else {
                    // Reset counter if current is OK
                    self.over_cnt = 0;
                }
            }
            
            LimitState::Limiting => {
                // Check if we can start recovery
                if abs_current < recovery_threshold {
                    self.state = LimitState::Recovering;
                } else if abs_current > self.config.limit_ma {
                    // Still over limit, apply backoff again
                    self.apply_backoff();
                }
                // Hold state if in hysteresis band
            }
            
            LimitState::Recovering => {
                if abs_current > self.config.limit_ma {
                    // Over limit again, back to limiting
                    self.over_cnt = 1;
                    self.state = LimitState::Normal;
                } else if abs_current < recovery_threshold {
                    // Continue recovery
                    self.apply_recovery(dt_us);
                    
                    // Check if fully recovered
                    if self.duty_cap >= Duty::MAX.as_raw() {
                        self.duty_cap = Duty::MAX.as_raw();
                        self.state = LimitState::Normal;
                        self.over_cnt = 0;
                    }
                }
                // Hold state if in hysteresis band (450-500mA)
            }
        }
        
        self.get_limits()
    }
    
    /// Apply backoff by reducing duty cap.
    fn apply_backoff(&mut self) {
        // Multiply by backoff factor using Q8 arithmetic
        let new_cap = ((self.duty_cap as i32 * self.config.backoff_factor_q8 as i32) >> 8) as i16;
        
        // Ensure minimum cap (at least 10% to maintain some control)
        const MIN_CAP: i16 = 3277; // 10% of Duty::MAX
        self.duty_cap = new_cap.max(MIN_CAP);
    }
    
    /// Apply recovery by increasing duty cap.
    fn apply_recovery(&mut self, dt_us: u32) {
        // Calculate recovery step based on time delta
        // recovery_step = recovery_rate * dt_seconds
        // = recovery_rate * dt_us / 1_000_000
        let recovery_step = ((self.config.recovery_rate as i32 * dt_us as i32) / 1_000_000) as i16;
        
        // Apply recovery
        self.duty_cap = self.duty_cap.saturating_add(recovery_step);
        
        // Cap at maximum
        if self.duty_cap > Duty::MAX.as_raw() {
            self.duty_cap = Duty::MAX.as_raw();
        }
    }
    
    /// Get current duty cycle limits.
    ///
    /// Returns (min, max) tuple for flexibility.
    /// Currently symmetric but supports future asymmetric limits.
    pub fn get_limits(&self) -> (i32, i32) {
        let limit = self.duty_cap as i32;
        (-limit, limit)
    }
    
    /// Get current limiting state.
    pub fn state(&self) -> LimitState {
        self.state
    }
    
    /// Check if compliance is currently being limited.
    pub fn is_limited(&self) -> bool {
        self.state != LimitState::Normal || self.duty_cap < Duty::MAX.as_raw()
    }
    
    /// Get current duty cap value (for telemetry).
    pub fn duty_cap(&self) -> i16 {
        self.duty_cap
    }
    
    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.duty_cap = Duty::MAX.as_raw();
        self.over_cnt = 0;
        self.state = LimitState::Normal;
    }
    
    /// Update configuration.
    pub fn set_config(&mut self, config: ComplianceConfig) {
        self.config = config;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sustained_overcurrent_triggers_limiting() {
        let mut model = ComplianceModel::default();
        
        // First two samples shouldn't trigger (deglitch = 3)
        model.update(Some(MilliAmp::from_ma(600)), 100);
        assert_eq!(model.state(), LimitState::Normal);
        
        model.update(Some(MilliAmp::from_ma(600)), 100);
        assert_eq!(model.state(), LimitState::Normal);
        
        // Third consecutive sample should trigger
        model.update(Some(MilliAmp::from_ma(600)), 100);
        assert_eq!(model.state(), LimitState::Limiting);
        
        // Duty should be backed off
        let (min, max) = model.get_limits();
        assert!(max < Duty::MAX.as_raw() as i32);
        assert_eq!(min, -max);
    }
    
    #[test]
    fn test_deglitch_prevents_single_spike() {
        let mut model = ComplianceModel::default();
        
        // Single spike
        model.update(Some(MilliAmp::from_ma(600)), 100);
        assert_eq!(model.state(), LimitState::Normal);
        
        // Back to normal current resets counter
        model.update(Some(MilliAmp::from_ma(400)), 100);
        assert_eq!(model.state(), LimitState::Normal);
        
        // Another spike doesn't trigger (counter was reset)
        model.update(Some(MilliAmp::from_ma(600)), 100);
        assert_eq!(model.state(), LimitState::Normal);
    }
    
    #[test]
    fn test_recovery_at_correct_rate() {
        let mut model = ComplianceModel::default();
        
        // Trigger limiting
        for _ in 0..3 {
            model.update(Some(MilliAmp::from_ma(600)), 100);
        }
        let limited_cap = model.duty_cap();
        
        // Start recovery at low current
        model.update(Some(MilliAmp::from_ma(400)), 100);
        assert_eq!(model.state(), LimitState::Recovering);
        
        // Recovery at 10% per second = 3277 per second
        // 100ms = 0.1 seconds = ~328 units recovery
        let dt_us = 100_000; // 100ms
        model.update(Some(MilliAmp::from_ma(400)), dt_us);
        let recovered_cap = model.duty_cap();
        
        // Should have recovered by approximately 328 units
        let recovery = recovered_cap - limited_cap;
        assert!(recovery > 250 && recovery < 400, "Recovery was {} units", recovery);
    }
    
    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut model = ComplianceModel::default();
        
        // Trigger limiting
        for _ in 0..3 {
            model.update(Some(MilliAmp::from_ma(600)), 100);
        }
        assert_eq!(model.state(), LimitState::Limiting);
        let limited_cap = model.duty_cap();
        
        // Current in hysteresis band (450-500mA) - should hold state
        model.update(Some(MilliAmp::from_ma(475)), 100);
        assert_eq!(model.state(), LimitState::Limiting);
        assert_eq!(model.duty_cap(), limited_cap); // No change
        
        // Drop below hysteresis - should start recovery
        model.update(Some(MilliAmp::from_ma(440)), 100);
        assert_eq!(model.state(), LimitState::Recovering);
        
        // Back in hysteresis band - should hold recovery state
        model.update(Some(MilliAmp::from_ma(475)), 100);
        assert_eq!(model.state(), LimitState::Recovering);
        let recovering_cap = model.duty_cap();
        assert_eq!(model.duty_cap(), recovering_cap); // No change
    }
    
    #[test]
    fn test_no_limiting_without_sensor() {
        let mut model = ComplianceModel::default();
        
        // No sensor (None) should never limit
        let (min, max) = model.update(None, 100);
        assert_eq!(min, -Duty::MAX.as_raw() as i32);
        assert_eq!(max, Duty::MAX.as_raw() as i32);
        assert_eq!(model.state(), LimitState::Normal);
    }
    
    #[test]
    fn test_minimum_duty_cap() {
        let mut config = ComplianceConfig::default();
        config.backoff_factor_q8 = 128; // 0.5 - aggressive backoff
        let mut model = ComplianceModel::new(config);
        
        // Keep triggering to drive duty down
        for _ in 0..20 {
            for _ in 0..3 {
                model.update(Some(MilliAmp::from_ma(600)), 100);
            }
        }
        
        // Should never go below 10% (3277)
        assert!(model.duty_cap() >= 3277);
    }
}