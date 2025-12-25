use crate::traits::{ControlInput, ControlLoop, ControlOutput, PidTunable};
use open_servo_math::{
    CentiDeg, DerivativeMode, Duty, FilterI32, Gain, MilliAmp, PidControllerI16, TickCtx,
};

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
            position_filter_shift: 3, // Default moderate filtering (alpha = 0.125)
        }
    }
}

impl Default for PidConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// PID position controller.
///
/// Stage 1 refactor: Controller owns only algorithm state.
/// - No setpoint storage (passed via ControlInput)
/// - No output limits storage (passed via ControlInput)
/// - Returns saturated flag for stall detection
pub struct PidController {
    config: PidConfig,
    pid: PidControllerI16,
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

    /// Get last filtered position.
    pub fn get_last_position(&self) -> CentiDeg {
        self.last_position
    }

    /// Get last current reading.
    pub fn get_last_current(&self) -> MilliAmp {
        self.last_current
    }
}

// =============================================================================
// ControlLoop Implementation
// =============================================================================

impl ControlLoop for PidController {
    fn reset(&mut self) {
        self.pid.reset();
        self.position_filter.reset();
    }

    fn fast_tick(&mut self, _ctx: &TickCtx, input: &ControlInput) -> ControlOutput {
        // Filter the position measurement
        let position_i32 = input.position.as_cdeg() as i32;
        let filtered_i32 = self.position_filter.update(position_i32);

        // Store filtered position (clamp to i16 range)
        self.last_position =
            CentiDeg::from_cdeg(filtered_i32.clamp(i16::MIN as i32, i16::MAX as i32) as i16);

        // Store current if available
        if let Some(c) = input.current {
            self.last_current = c;
        }

        let sp = input.setpoint.as_cdeg() as i32;
        let pv = filtered_i32;

        // Apply per-tick limits to PID core (updates integral anti-windup too)
        let min = input.limits.min.as_raw() as i32;
        let max = input.limits.max.as_raw() as i32;
        self.pid.set_output_limits_auto_anti_windup(min, max);

        // Compute output (now properly clamped by PID core)
        let raw_output = self.pid.step(sp, pv);

        // Saturation detection (PID core already clamped to [min, max])
        let saturated = raw_output == min || raw_output == max;

        // Safe cast to i16 (should already be in range, but defensive)
        let duty_raw = raw_output.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

        ControlOutput {
            duty: Duty::from_raw(duty_raw),
            saturated,
        }
    }

    fn medium_tick(&mut self, _ctx: &TickCtx, _input: &ControlInput) {
        // No-op for PID controller
    }

    fn slow_tick(&mut self, _ctx: &TickCtx, _input: &ControlInput) {
        // No-op for PID controller
    }
}

// =============================================================================
// PidTunable Implementation
// =============================================================================

impl PidTunable for PidController {
    fn pid_config(&self) -> &PidConfig {
        &self.config
    }

    fn with_pid_config_mut<F: FnOnce(&mut PidConfig)>(&mut self, f: F) {
        f(&mut self.config);
        self.apply_config();
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::DutyLimits;
    use open_servo_math::TickDomain;

    /// Create a dummy TickCtx for testing.
    fn test_ctx() -> TickCtx {
        TickCtx {
            domain: TickDomain::ControlFast,
            dt_us: 100,
            seq: 0,
        }
    }

    fn make_input(setpoint: i16, position: i16) -> ControlInput {
        ControlInput {
            setpoint: CentiDeg::from_cdeg(setpoint),
            position: CentiDeg::from_cdeg(position),
            velocity: None,
            current: None,
            bus_voltage: None,
            limits: DutyLimits::full(),
        }
    }

    #[test]
    fn test_new_config_values() {
        let config = PidConfig::new();
        assert_eq!(config.kp, PidConfig::KP_DEFAULT);
        assert_eq!(config.ki, PidConfig::KI_DEFAULT);
        assert_eq!(config.kd, PidConfig::KD_DEFAULT);
        assert_eq!(config.derivative_mode, DerivativeMode::OnMeasurement);
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
        let ctx = test_ctx();

        // Run a few steps to build up integral
        let input = make_input(9000, 0);
        for _ in 0..100 {
            ctrl.fast_tick(&ctx, &input);
        }

        // Get output before reset
        let before = ctrl.fast_tick(&ctx, &input);

        // Reset
        ctrl.reset();

        // Output after reset should be different (no integral accumulation)
        let after = ctrl.fast_tick(&ctx, &input);

        // The accumulated integral was adding to output, so after reset output should be smaller
        // (pure proportional response vs proportional + accumulated integral)
        assert!(after.duty.abs() <= before.duty.abs());
    }

    #[test]
    fn test_fast_tick_produces_output() {
        let mut ctrl = PidController::new(PidConfig::new());
        let ctx = test_ctx();

        // With position at 0° and setpoint at 90°, we should get positive output
        let input = make_input(9000, 0);
        let output = ctrl.fast_tick(&ctx, &input);
        assert!(output.duty.as_raw() > 0);

        // Output should be clamped to i16 range
        assert!(output.duty.as_raw() <= i16::MAX);
        assert!(output.duty.as_raw() >= i16::MIN);
    }

    #[test]
    fn test_saturation_flag() {
        let mut ctrl = PidController::new(PidConfig::new());
        let ctx = test_ctx();

        // Create input with tight limits
        let input = ControlInput {
            setpoint: CentiDeg::from_cdeg(9000),
            position: CentiDeg::from_cdeg(0),
            velocity: None,
            current: None,
            bus_voltage: None,
            limits: DutyLimits::new(Duty::from_raw(-100), Duty::from_raw(100)),
        };

        // With large error, output should saturate
        let output = ctrl.fast_tick(&ctx, &input);
        assert!(output.saturated);
        assert!(output.duty.as_raw() == 100 || output.duty.as_raw() == -100);
    }

    #[test]
    fn test_no_saturation_with_small_error() {
        let mut ctrl = PidController::new(PidConfig::new());
        let ctx = test_ctx();

        // Create input with small error (position close to setpoint)
        let input = ControlInput {
            setpoint: CentiDeg::from_cdeg(100),
            position: CentiDeg::from_cdeg(99),
            velocity: None,
            current: None,
            bus_voltage: None,
            limits: DutyLimits::full(),
        };

        // With tiny error, output should not saturate
        let output = ctrl.fast_tick(&ctx, &input);
        assert!(!output.saturated);
    }
}
