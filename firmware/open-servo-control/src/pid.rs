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
