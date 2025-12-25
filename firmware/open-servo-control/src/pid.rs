use crate::traits::ControlLoop;
use open_servo_math::{CentiDeg, DerivativeMode, Duty, FilterI32, Gain, MilliAmp, MilliVolt, PidControllerI16};

/// PID configuration with human-friendly gains.
///
/// This is the source of truth for PID tuning. The internal PidI16 is
/// rebuilt atomically from this config via `apply_config()`.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidConfig {
    /// Proportional gain
    pub kp: Gain,
    /// Integral gain
    pub ki: Gain,
    /// Derivative gain
    pub kd: Gain,
    /// Derivative calculation mode
    pub derivative_mode: DerivativeMode,
    /// Position filter strength (0 = no filter, higher = more filtering)
    /// Alpha = 1 / 2^position_filter_shift
    pub position_filter_shift: u8,
}

impl PidConfig {
    // =======================================================================
    // Recommended starting gains for BDC servo position control
    // =======================================================================

    /// Default Kp tuned for responsive control.
    /// At 1° error: ~0.5 PWM counts. At 90° error: ~450 PWM counts.
    pub const KP_DEFAULT: Gain = Gain::from_raw(500); // 5.0

    /// Ki disabled to prevent integral windup at 10kHz control rate.
    /// Enable only if steady-state error correction is needed.
    pub const KI_DEFAULT: Gain = Gain::from_raw(0); // 0.0

    /// Kd for overshoot damping.
    /// Set to match Kp for balanced response.
    pub const KD_DEFAULT: Gain = Gain::from_raw(500); // 5.0
}

impl PidConfig {
    /// Create a new PidConfig with default settings.
    pub fn new() -> Self {
        Self {
            kp: Self::KP_DEFAULT,
            ki: Self::KI_DEFAULT,
            kd: Self::KD_DEFAULT,
            derivative_mode: DerivativeMode::OnMeasurement,
            position_filter_shift: 3,  // Default moderate filtering (alpha = 0.125)
        }
    }
}

pub struct PidController {
    config: PidConfig,
    pid: PidControllerI16,
    setpoint: Option<CentiDeg>,
    last_position: CentiDeg,
    last_current: MilliAmp,
    position_filter: FilterI32,
}

impl PidController {
    pub fn new(config: PidConfig) -> Self {
        let kp = config.kp.to_q8_8();
        let ki = config.ki.to_q8_8();
        let kd = config.kd.to_q8_8();

        let pid = PidControllerI16::new_auto_anti_windup(
            kp,
            ki,
            kd,
            config.derivative_mode,
            i16::MIN as i32,
            i16::MAX as i32,
        );

        Self {
            config,
            pid,
            setpoint: None, // No setpoint when disengaged
            last_position: CentiDeg::from_cdeg(0),
            last_current: MilliAmp::from_ma(0),
            position_filter: FilterI32::new(config.position_filter_shift),
        }
    }

    /// Atomically rebuild PidI16 from current config.
    /// Call this after modifying config fields.
    pub fn apply_config(&mut self) {
        let kp = self.config.kp.to_q8_8();
        let ki = self.config.ki.to_q8_8();
        let kd = self.config.kd.to_q8_8();

        self.pid = PidControllerI16::new_auto_anti_windup(
            kp,
            ki,
            kd,
            self.config.derivative_mode,
            i16::MIN as i32,
            i16::MAX as i32,
        );
        
        // Update filter if shift changed
        if self.position_filter.alpha_shift() != self.config.position_filter_shift {
            self.position_filter = FilterI32::new(self.config.position_filter_shift);
        }
    }

    /// Get read-only reference to config.
    pub fn config(&self) -> &PidConfig {
        &self.config
    }

    pub fn set_setpoint(&mut self, setpoint: CentiDeg) {
        self.setpoint = Some(setpoint);
    }

    pub fn get_setpoint(&self) -> Option<CentiDeg> {
        self.setpoint
    }
    
    pub fn get_setpoint_or_default(&self) -> CentiDeg {
        // Return setpoint if set, otherwise return 0 as default
        self.setpoint.unwrap_or(CentiDeg::from_cdeg(0))
    }
    
    pub fn clear_setpoint(&mut self) {
        self.setpoint = None;
    }
    
    pub fn has_setpoint(&self) -> bool {
        self.setpoint.is_some()
    }

    pub fn get_last_position(&self) -> CentiDeg {
        self.last_position
    }

    pub fn get_last_current(&self) -> MilliAmp {
        self.last_current
    }

    pub fn step(&mut self, current: MilliAmp, position: CentiDeg, _bus_voltage: MilliVolt) -> Duty {
        self.last_position = position;
        self.last_current = current;

        // If no setpoint, output zero (motor disengaged)
        let Some(setpoint) = self.setpoint else {
            return Duty::ZERO;
        };

        let sp = setpoint.as_cdeg() as i32;
        let pv = position.as_cdeg() as i32;

        let output = self.pid.step(sp, pv);
        Duty::from_raw(output.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    fn reset_pid(&mut self) {
        self.pid.reset();
    }
}

impl ControlLoop for PidController {
    fn compute(
        &mut self,
        setpoint: CentiDeg,
        position: CentiDeg,
        current: Option<MilliAmp>,
    ) -> Duty {
        // If no internal setpoint, return zero (motor disengaged)
        if self.setpoint.is_none() {
            return Duty::ZERO;
        }
        
        // Filter the position measurement (convert to i32 first)
        let position_i32 = position.as_cdeg() as i32;
        let filtered_i32 = self.position_filter.update(position_i32);
        
        // Store as CentiDeg for compatibility (clamp to i16 range)
        self.last_position = CentiDeg::from_cdeg(filtered_i32.clamp(i16::MIN as i32, i16::MAX as i32) as i16);
        if let Some(c) = current {
            self.last_current = c;
        }

        let sp = setpoint.as_cdeg() as i32;
        let pv = filtered_i32;

        let output = self.pid.step(sp, pv);
        Duty::from_raw(output.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    fn reset(&mut self) {
        self.reset_pid();
        self.position_filter.reset();
    }

    fn set_setpoint(&mut self, setpoint: CentiDeg) {
        self.setpoint = Some(setpoint);
    }

    fn get_setpoint(&self) -> CentiDeg {
        // Return setpoint if set, otherwise 0 for compatibility
        self.setpoint.unwrap_or(CentiDeg::from_cdeg(0))
    }
    
    fn has_setpoint(&self) -> bool {
        self.setpoint.is_some()
    }
    
    fn clear_setpoint(&mut self) {
        self.setpoint = None;
    }
    
    fn set_output_limits(&mut self, min: i32, max: i32) {
        self.pid.set_output_limits(min, max);
    }

    fn pid_config(&self) -> Option<&PidConfig> {
        Some(&self.config)
    }

    fn with_pid_config_mut<F>(&mut self, f: F) -> bool
    where
        F: FnOnce(&mut PidConfig),
    {
        f(&mut self.config);
        self.apply_config();
        true // PID controller always has config, so always returns true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_config_values() {
        let config = PidConfig::new();
        assert_eq!(config.kp, PidConfig::KP_DEFAULT);
        assert_eq!(config.ki, PidConfig::KI_DEFAULT);
        assert_eq!(config.kd, PidConfig::KD_DEFAULT);
        assert_eq!(config.derivative_mode, DerivativeMode::OnMeasurement);
    }

    #[test]
    fn test_new_initializes_correctly() {
        let ctrl = PidController::new(PidConfig::new());
        assert_eq!(ctrl.get_setpoint_or_default().as_cdeg(), 0); // No setpoint initially, returns 0
        assert!(!ctrl.has_setpoint()); // Should have no setpoint
    }

    #[test]
    fn test_setpoint_get_set() {
        let mut ctrl = PidController::new(PidConfig::new());
        ctrl.set_setpoint(CentiDeg::from_cdeg(4500)); // 45°
        assert_eq!(ctrl.get_setpoint_or_default().as_cdeg(), 4500);
        assert!(ctrl.has_setpoint()); // Should have a setpoint now
    }


    #[test]
    fn test_with_pid_config_mut_calls_apply() {
        let mut ctrl = PidController::new(PidConfig::new());
        let original_kp = ctrl.config().kp;

        ctrl.with_pid_config_mut(|cfg| {
            cfg.kp = Gain::from_raw(5000); // 50.0
        });

        // Config should be updated
        assert_ne!(ctrl.config().kp, original_kp);
        assert_eq!(ctrl.config().kp, Gain::from_raw(5000));
    }

    #[test]
    fn test_reset_clears_pid_state() {
        let mut ctrl = PidController::new(PidConfig::new());

        // Run a few steps to build up integral
        ctrl.set_setpoint(CentiDeg::from_cdeg(9000));
        for _ in 0..100 {
            ctrl.compute(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0), None);
        }

        // Get output before reset
        let before = ctrl.compute(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0), None);

        // Reset
        ctrl.reset();

        // Output after reset should be different (no integral accumulation)
        let after = ctrl.compute(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0), None);

        // The accumulated integral was adding to output, so after reset output should be smaller
        // (pure proportional response vs proportional + accumulated integral)
        assert!(after.abs() <= before.abs());
    }

    #[test]
    fn test_compute_produces_output() {
        let mut ctrl = PidController::new(PidConfig::new());
        ctrl.set_setpoint(CentiDeg::from_cdeg(9000)); // 90°

        // With position at 0° and setpoint at 90°, we should get positive output
        let output = ctrl.compute(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0), None);
        assert!(output.as_raw() > 0);

        // Output should be clamped to i16 range
        assert!(output.as_raw() <= i16::MAX);
        assert!(output.as_raw() >= i16::MIN);
    }
}
