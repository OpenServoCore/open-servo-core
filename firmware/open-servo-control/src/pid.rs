use pid::Pid;
use crate::traits::ControlLoop;

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
    setpoint: u16,
    last_position: u16,
    last_current: u16,
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
            setpoint: 2048, // Center position for 12-bit ADC
            last_position: 0,
            last_current: 0,
        }
    }

    pub fn set_setpoint(&mut self, setpoint: u16) {
        self.setpoint = setpoint;
    }

    pub fn get_setpoint(&self) -> u16 {
        self.setpoint
    }

    pub fn get_last_position(&self) -> u16 {
        self.last_position
    }

    pub fn get_last_current(&self) -> u16 {
        self.last_current
    }

    pub fn step(&mut self, current_ma: u16, position: u16, _bus_voltage_mv: u16) -> i32 {
        self.last_position = position;
        self.last_current = current_ma;

        // Run PID control on position
        let pid_output = self.pid
            .setpoint(self.setpoint)
            .next_control_output(position as f32);

        let mut duty = pid_output.output as i32;
        
        if self.config.reverse {
            duty = -duty;
        }

        duty
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
    fn compute(&mut self, _setpoint: u16, position: u16, _current: Option<u16>) -> i32 {
        self.last_position = position;
        
        // Run PID control on position
        let pid_output = self.pid
            .setpoint(self.setpoint)
            .next_control_output(position as f32);

        let mut duty = pid_output.output as i32;
        
        if self.config.reverse {
            duty = -duty;
        }

        duty
    }
    
    fn reset(&mut self) {
        self.reset_pid();
    }
    
    fn set_setpoint(&mut self, setpoint: u16) {
        self.setpoint = setpoint;
    }
    
    fn get_setpoint(&self) -> u16 {
        self.setpoint
    }
}