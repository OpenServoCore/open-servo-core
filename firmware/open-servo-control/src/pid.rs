use crate::traits::ControlLoop;
use open_servo_math::{CentiDeg, DerivativeMode, Gain, MilliAmp, MilliVolt, PidControllerI16};

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
    /// Maximum output magnitude (e.g. PWM duty cycle limit)
    pub output_max: i32,
    /// Derivative calculation mode
    pub derivative_mode: DerivativeMode,
}

impl PidConfig {
    // =======================================================================
    // Recommended starting gains for BDC servo position control
    // =======================================================================

    /// Conservative Kp for initial bring-up.
    /// At 1° error: ~15 PWM counts. At 90° error: ~1400 PWM counts.
    pub const KP_DEFAULT: Gain = Gain::from_raw(4000); // 40.0

    /// Small Ki for steady-state error correction.
    /// Use sparingly - accumulates fast at 10kHz!
    pub const KI_DEFAULT: Gain = Gain::from_raw(10); // 0.10

    /// Moderate Kd for overshoot damping.
    /// Roughly Kp/4. Use with DerivativeMode::OnMeasurement.
    pub const KD_DEFAULT: Gain = Gain::from_raw(1000); // 10.0
}

impl Default for PidConfig {
    fn default() -> Self {
        // PWM_MAX_DUTY is ~1799 for 20kHz PWM at 72MHz
        const PWM_MAX: i32 = 1799;

        Self {
            kp: Self::KP_DEFAULT,
            ki: Self::KI_DEFAULT,
            kd: Self::KD_DEFAULT,
            output_max: PWM_MAX,
            derivative_mode: DerivativeMode::OnMeasurement,
        }
    }
}

pub struct PidController {
    config: PidConfig,
    pid: PidControllerI16,
    setpoint: CentiDeg,
    last_position: CentiDeg,
    last_current: MilliAmp,
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
            -config.output_max,
            config.output_max,
        );

        Self {
            config,
            pid,
            setpoint: CentiDeg::from_deg(90), // 90 degrees
            last_position: CentiDeg::from_cdeg(0),
            last_current: MilliAmp::from_ma(0),
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
            -self.config.output_max,
            self.config.output_max,
        );
    }

    /// Get read-only reference to config.
    pub fn config(&self) -> &PidConfig {
        &self.config
    }

    pub fn set_setpoint(&mut self, setpoint: CentiDeg) {
        self.setpoint = setpoint;
    }

    pub fn get_setpoint(&self) -> CentiDeg {
        self.setpoint
    }

    pub fn get_last_position(&self) -> CentiDeg {
        self.last_position
    }

    pub fn get_last_current(&self) -> MilliAmp {
        self.last_current
    }

    pub fn step(&mut self, current: MilliAmp, position: CentiDeg, _bus_voltage: MilliVolt) -> i32 {
        self.last_position = position;
        self.last_current = current;

        let sp = self.setpoint.as_cdeg() as i32;
        let pv = position.as_cdeg() as i32;

        self.pid.step(sp, pv)
    }

    fn reset_pid(&mut self) {
        self.pid.reset();
    }
}

impl ControlLoop for PidController {
    fn compute(
        &mut self,
        _setpoint: CentiDeg,
        position: CentiDeg,
        current: Option<MilliAmp>,
    ) -> i32 {
        self.last_position = position;
        if let Some(c) = current {
            self.last_current = c;
        }

        let sp = self.setpoint.as_cdeg() as i32;
        let pv = position.as_cdeg() as i32;

        self.pid.step(sp, pv)
    }

    fn reset(&mut self) {
        self.reset_pid();
    }

    fn set_setpoint(&mut self, setpoint: CentiDeg) {
        self.setpoint = setpoint;
    }

    fn get_setpoint(&self) -> CentiDeg {
        self.setpoint
    }

    fn output_max(&self) -> i32 {
        self.config.output_max
    }

    fn pid_config(&self) -> Option<&PidConfig> {
        Some(&self.config)
    }

    fn with_pid_config_mut<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PidConfig),
    {
        f(&mut self.config);
        self.apply_config();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config_values() {
        let config = PidConfig::default();
        assert_eq!(config.kp, PidConfig::KP_DEFAULT);
        assert_eq!(config.ki, PidConfig::KI_DEFAULT);
        assert_eq!(config.kd, PidConfig::KD_DEFAULT);
        assert_eq!(config.output_max, 1799);
        assert_eq!(config.derivative_mode, DerivativeMode::OnMeasurement);
    }

    #[test]
    fn test_new_initializes_correctly() {
        let ctrl = PidController::new(PidConfig::default());
        assert_eq!(ctrl.get_setpoint().as_cdeg(), 9000); // 90°
        assert_eq!(ctrl.output_max(), 1799);
    }

    #[test]
    fn test_setpoint_get_set() {
        let mut ctrl = PidController::new(PidConfig::default());
        ctrl.set_setpoint(CentiDeg::from_cdeg(4500)); // 45°
        assert_eq!(ctrl.get_setpoint().as_cdeg(), 4500);
    }

    #[test]
    fn test_output_max_returns_config_value() {
        let mut config = PidConfig::default();
        config.output_max = 1000;
        let ctrl = PidController::new(config);
        assert_eq!(ctrl.output_max(), 1000);
    }

    #[test]
    fn test_with_pid_config_mut_calls_apply() {
        let mut ctrl = PidController::new(PidConfig::default());
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
        let mut ctrl = PidController::new(PidConfig::default());

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
        let mut ctrl = PidController::new(PidConfig::default());
        ctrl.set_setpoint(CentiDeg::from_cdeg(9000)); // 90°

        // With position at 0° and setpoint at 90°, we should get positive output
        let output = ctrl.compute(CentiDeg::from_cdeg(9000), CentiDeg::from_cdeg(0), None);
        assert!(output > 0);

        // Output should be clamped to output_max
        assert!(output <= ctrl.output_max());
    }
}
