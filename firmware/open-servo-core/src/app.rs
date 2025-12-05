use open_servo_hw::BdcMotorDriver;
use open_servo_control::{ControlLoop, PositionSensor, CurrentSensor, VoltageSensor, TemperatureSensor};
use super::fault::{FaultKind, FaultState};
use super::event::Event;

/// System state information for debugging/telemetry
#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SystemState {
    pub setpoint: u16,
    pub position: u16,
    pub pwm_duty: i32,
    pub current_ma: u16,
    pub bus_voltage_mv: u16,
    pub temperature_dk: Option<u16>,
}

impl SystemState {
    fn new() -> Self {
        SystemState {
            setpoint: 0,
            position: 0,
            pwm_duty: 0,
            current_ma: 0,
            bus_voltage_mv: 0,
            temperature_dk: None,
        }
    }
}

/// Main application state machine
pub struct App<C: ControlLoop> {
    controller: C,
    fault_state: FaultState,
    system_state: SystemState,
}

impl<C: ControlLoop> App<C> {
    pub fn new(controller: C) -> Self {
        App {
            controller,
            fault_state: FaultState::new(),
            system_state: SystemState::new(),
        }
    }

    /// Hard real-time control tick (called from DMA/Timer ISR)
    /// This must execute deterministically without blocking
    pub fn on_control_tick<H>(&mut self, hw: &mut H)
    where
        H: BdcMotorDriver + PositionSensor + CurrentSensor + VoltageSensor + TemperatureSensor
    {
        // If faulted, ensure motor is safe and exit early
        if self.fault_state.is_faulted() {
            self.fault_state.apply_safety(hw);
            return;
        }

        // Read sensors
        let current = hw.read_current();
        let position = hw.read_position();
        let bus_voltage = hw.read_voltage();
        let temperature = hw.read_temperature();

        // Run control loop
        let pwm_command = self.controller.compute(self.controller.get_setpoint(), position, Some(current));

        // Apply PWM command
        hw.set_pwm(pwm_command);

        // Update system state for telemetry
        self.system_state.setpoint = self.controller.get_setpoint();
        self.system_state.position = position;
        self.system_state.pwm_duty = pwm_command;
        self.system_state.current_ma = current;
        self.system_state.bus_voltage_mv = bus_voltage;
        self.system_state.temperature_dk = temperature;
    }

    /// Handle soft events (called from main loop)
    pub fn handle_event<H>(&mut self, _hw: &mut H, event: Event)
    where
        H: BdcMotorDriver
    {
        match event {
            Event::UartRx { port: _, byte: _ } => {
                // TODO: Handle UART protocol in V1
            }
            Event::SlowTick => {
                // Log system state on slow ticks
                #[cfg(feature = "defmt")]
                defmt::info!("State: {}", self.system_state);
            }
            Event::Fault(kind) => {
                // Log fault event (safety response already happened)
                #[cfg(feature = "defmt")]
                defmt::warn!("Fault event logged: {:?}", kind);
            }
        }
    }

    /// Raise a fault (called from safety ISR or fault detection)
    pub fn raise_fault<H: BdcMotorDriver>(&mut self, hw: &mut H, kind: FaultKind) {
        self.fault_state.raise(kind);
        self.fault_state.apply_safety(hw);
        
        // Reset controller to clear integral windup
        self.controller.reset();
    }

    /// Clear fault (called via debug command or protocol)
    pub fn clear_fault<H: BdcMotorDriver>(&mut self, _hw: &mut H) {
        self.fault_state.clear();
        self.controller.reset();
    }

    /// Get current system state
    pub fn get_system_state(&self) -> SystemState {
        self.system_state
    }

    /// Check if system is faulted
    pub fn is_faulted(&self) -> bool {
        self.fault_state.is_faulted()
    }

    /// Set controller setpoint
    pub fn set_setpoint(&mut self, setpoint: u16) {
        self.controller.set_setpoint(setpoint);
    }
}