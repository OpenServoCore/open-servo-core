//! Compliance limiter for motor current protection.
//!
//! Safety wrapper around ComplianceModel that integrates with the servo
//! control system. Handles direction change blanking and provides
//! telemetry information.

use open_servo_math::{MilliAmp, ComplianceModel, ComplianceConfig, LimitState};

/// Direction change blanking duration in control ticks.
/// At 10kHz, 10 ticks = 1ms blanking after direction change.
const DIRECTION_CHANGE_BLANKING_TICKS: u8 = 10;

/// Compliance limiter with safety integration.
///
/// Wraps the pure mathematical ComplianceModel with additional
/// safety features like direction change blanking.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ComplianceLimiter {
    /// Mathematical compliance model
    model: ComplianceModel,
    
    /// Direction change blanking counter
    blanking_cnt: u8,
    
    /// Last PWM direction for change detection
    last_direction: Direction,
    
    /// Flag indicating if compliance is currently limited
    compliance_limited: bool,
}

/// PWM direction for change detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Direction {
    Forward,
    Reverse,
    Stopped,
}

impl ComplianceLimiter {
    /// Create a new compliance limiter with the given configuration.
    pub fn new(config: ComplianceConfig) -> Self {
        Self {
            model: ComplianceModel::new(config),
            blanking_cnt: 0,
            last_direction: Direction::Stopped,
            compliance_limited: false,
        }
    }
    
    
    /// Update the limiter with new current reading and PWM direction.
    ///
    /// # Parameters
    /// - `current`: Current reading in milliamps (None if sensor unavailable)
    /// - `pwm_direction`: Sign of PWM command for direction change detection
    /// - `dt_us`: Time delta in microseconds since last update
    pub fn update(&mut self, current: Option<MilliAmp>, pwm_direction: i16, dt_us: u32) {
        // Detect direction change
        let new_direction = if pwm_direction > 0 {
            Direction::Forward
        } else if pwm_direction < 0 {
            Direction::Reverse
        } else {
            Direction::Stopped
        };
        
        // Start blanking on direction change
        if new_direction != self.last_direction && new_direction != Direction::Stopped {
            self.blanking_cnt = DIRECTION_CHANGE_BLANKING_TICKS;
        }
        self.last_direction = new_direction;
        
        // Apply blanking - ignore current during transients
        let effective_current = if self.blanking_cnt > 0 {
            self.blanking_cnt = self.blanking_cnt.saturating_sub(1);
            None // Ignore current during blanking
        } else {
            current
        };
        
        // Update model with effective current
        self.model.update(effective_current, dt_us);
        
        // Update telemetry flag
        self.compliance_limited = self.model.is_limited();
    }
    
    /// Get current duty cycle limits.
    ///
    /// Returns (min, max) tuple for PID output limits.
    pub fn get_limits(&self) -> (i32, i32) {
        self.model.get_limits()
    }
    
    /// Check if compliance is currently being limited.
    pub fn is_limited(&self) -> bool {
        self.compliance_limited
    }
    
    /// Get current limiting state for telemetry.
    pub fn state(&self) -> LimitState {
        self.model.state()
    }
    
    /// Get current duty cap value for telemetry.
    pub fn duty_cap(&self) -> i16 {
        self.model.duty_cap()
    }
    
    /// Check if currently in blanking period.
    pub fn is_blanking(&self) -> bool {
        self.blanking_cnt > 0
    }
    
    /// Reset limiter to initial state.
    pub fn reset(&mut self) {
        self.model.reset();
        self.blanking_cnt = 0;
        self.last_direction = Direction::Stopped;
        self.compliance_limited = false;
    }
    
    /// Update configuration.
    pub fn set_config(&mut self, config: ComplianceConfig) {
        self.model.set_config(config);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_direction_change_blanking() {
        let config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        let mut limiter = ComplianceLimiter::new(config);
        
        // High current during direction change should be ignored
        limiter.update(Some(MilliAmp::from_ma(600)), 100, 100);
        assert!(limiter.is_blanking());
        assert!(!limiter.is_limited()); // Should not limit during blanking
        
        // After blanking period, current should be processed
        for _ in 0..DIRECTION_CHANGE_BLANKING_TICKS {
            limiter.update(Some(MilliAmp::from_ma(650)), 100, 100); // Above 600mA limit
        }
        assert!(!limiter.is_blanking());

        // Now sustained overcurrent should trigger limiting (after deglitch)
        limiter.update(Some(MilliAmp::from_ma(650)), 100, 100);
        limiter.update(Some(MilliAmp::from_ma(650)), 100, 100);
        assert!(limiter.is_limited());
    }
    
    #[test]
    fn test_direction_change_resets_blanking() {
        let config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        let mut limiter = ComplianceLimiter::new(config);
        
        // Forward direction
        limiter.update(Some(MilliAmp::from_ma(400)), 100, 100);
        assert!(limiter.is_blanking());
        
        // Consume some blanking ticks
        for _ in 0..5 {
            limiter.update(Some(MilliAmp::from_ma(400)), 100, 100);
        }
        
        // Change to reverse - should reset blanking
        limiter.update(Some(MilliAmp::from_ma(400)), -100, 100);
        assert!(limiter.is_blanking());
        
        // Should need full blanking period again
        for _ in 0..DIRECTION_CHANGE_BLANKING_TICKS - 1 {
            limiter.update(Some(MilliAmp::from_ma(400)), -100, 100);
        }
        assert!(!limiter.is_blanking());
    }
    
    #[test]
    fn test_stopped_to_moving_triggers_blanking() {
        let config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        let mut limiter = ComplianceLimiter::new(config);
        
        // Start from stopped
        limiter.update(Some(MilliAmp::from_ma(0)), 0, 100);
        assert!(!limiter.is_blanking());
        
        // Move forward - should trigger blanking
        limiter.update(Some(MilliAmp::from_ma(400)), 100, 100);
        assert!(limiter.is_blanking());
    }
    
    #[test]
    fn test_telemetry_flags() {
        let config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        let mut limiter = ComplianceLimiter::new(config);
        
        // Initially not limited
        assert!(!limiter.is_limited());
        assert_eq!(limiter.state(), LimitState::Normal);
        
        // Trigger limiting (skip blanking for test)
        for _ in 0..DIRECTION_CHANGE_BLANKING_TICKS {
            limiter.update(Some(MilliAmp::from_ma(100)), 100, 100);
        }
        
        // Now trigger actual limiting (current must be > limit, not >=)
        for _ in 0..3 {
            limiter.update(Some(MilliAmp::from_ma(650)), 100, 100); // Above 600mA limit
        }

        assert!(limiter.is_limited());
        assert_eq!(limiter.state(), LimitState::Limiting);
        assert!(limiter.duty_cap() < 32767);
    }
}