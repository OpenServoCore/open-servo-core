use crate::traits::ControlLoop;
use crate::units::*;
use pid::Pid;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub kp_max: f32,
    pub ki_max: f32,
    pub kd_max: f32,
    pub output_max: f32,
    pub reverse: bool,
}

impl Default for PidConfig {
    fn default() -> Self {
        // PWM_MAX_DUTY is ~1799 for 20kHz PWM at 72MHz
        const PWM_MAX: f32 = 1799.0;

        Self {
            kp: 40.0,
            ki: 0.0,
            kd: 0.0,
            kp_max: PWM_MAX,
            ki_max: PWM_MAX * 0.8,
            kd_max: PWM_MAX * 0.8,
            output_max: PWM_MAX,
            reverse: false,
        }
    }
}

pub struct PidController {
    pid: Pid<f32>,
    config: PidConfig,
    setpoint: CentiDeg,
    last_position: CentiDeg,
    last_current: MilliAmp,
}

impl PidController {
    pub fn new(config: PidConfig) -> Self {
        let mut pid = Pid::new(0.0, config.output_max);
        pid.p(config.kp, config.kp_max);
        pid.i(config.ki, config.ki_max);
        pid.d(config.kd, config.kd_max);

        Self {
            pid,
            config,
            setpoint: CentiDeg::from_deg(90), // 90 degrees
            last_position: CentiDeg::from_cdeg(0),
            last_current: MilliAmp::from_ma(0),
        }
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

        // Convert to float for PID library (use raw centidegrees as float)
        let setpoint_f32 = self.setpoint.as_cdeg() as f32;
        let position_f32 = position.as_cdeg() as f32;

        // Run PID control on position
        let pid_output = self
            .pid
            .setpoint(setpoint_f32)
            .next_control_output(position_f32);

        let mut output = pid_output.output;

        if self.config.reverse {
            output = -output;
        }

        output as i32
    }

    /// Update PID gains at runtime
    pub fn update_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.config.kp = kp;
        self.config.ki = ki;
        self.config.kd = kd;

        self.pid.p(kp, self.config.kp_max);
        self.pid.i(ki, self.config.ki_max);
        self.pid.d(kd, self.config.kd_max);
    }

    fn reset_pid(&mut self) {
        // Recreate PID controller to reset integral state
        self.pid = Pid::new(0.0, self.config.output_max);
        self.pid.p(self.config.kp, self.config.kp_max);
        self.pid.i(self.config.ki, self.config.ki_max);
        self.pid.d(self.config.kd, self.config.kd_max);
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

        // Convert to float for PID library
        let setpoint_f32 = self.setpoint.as_cdeg() as f32;
        let position_f32 = position.as_cdeg() as f32;

        // Run PID control on position (in centidegrees)
        let pid_output = self
            .pid
            .setpoint(setpoint_f32)
            .next_control_output(position_f32);

        let mut output = pid_output.output;

        if self.config.reverse {
            output = -output;
        }

        output as i32
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
}
