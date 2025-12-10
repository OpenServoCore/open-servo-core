//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.

use open_servo_control::ControlLoop;
use open_servo_math::{CentiDeg, DeciC, MilliAmp, MilliVolt};

use crate::fault::{FaultKind, FaultState};
use crate::inputs::FastInputs;
use crate::outputs::FastOutputs;
use crate::safety::SafetyManager;

/// System state snapshot for telemetry and debugging.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SystemState {
    /// Current setpoint
    pub setpoint: CentiDeg,
    /// Current position feedback
    pub position: CentiDeg,
    /// Current PWM duty cycle
    pub pwm_duty: i32,
    /// Current reading (if available)
    pub current: Option<MilliAmp>,
    /// Bus voltage reading (if available)
    pub bus_voltage: Option<MilliVolt>,
    /// Temperature reading (if available)
    pub temperature: Option<DeciC>,
}

/// Pure servo control core - no hardware dependencies.
///
/// ServoCore owns:
/// - The control loop algorithm (PID, cascade, etc.)
/// - Fault state management
/// - Safety monitoring via SafetyManager
/// - System state for telemetry
///
/// ## Usage
///
/// ```ignore
/// let mut core = ServoCore::new(PidController::default());
///
/// // In control ISR:
/// let inputs = FastInputs { position, current, bus_voltage, temperature };
/// let outputs = core.fast_tick(inputs);
/// // Apply outputs to hardware
///
/// // In slow tick:
/// if let Some(fault) = core.slow_tick() {
///     // Handle fault
/// }
/// ```
pub struct ServoCore<C: ControlLoop> {
    controller: C,
    fault_state: FaultState,
    safety: SafetyManager,
    system_state: SystemState,
}

impl<C: ControlLoop> ServoCore<C> {
    /// Create a new ServoCore with the given controller.
    pub fn new(controller: C) -> Self {
        Self {
            controller,
            fault_state: FaultState::new(),
            safety: SafetyManager::new(),
            system_state: SystemState::default(),
        }
    }

    /// Hard real-time control tick (10kHz).
    ///
    /// Pure function: takes sensor inputs, returns motor outputs.
    /// No hardware access - fully host-testable.
    ///
    /// ## Safety Checks (in order)
    ///
    /// 1. If already faulted, return safe outputs
    /// 2. Validate position sensor (skip tick on bad reading)
    /// 3. Check current threshold (if sensor available)
    /// 4. Run control loop with clamped setpoint
    /// 5. Check for motor stall (PWM saturated + no movement)
    #[inline]
    pub fn fast_tick(&mut self, inputs: FastInputs) -> FastOutputs {
        // If already faulted, return safe state
        if self.fault_state.is_faulted() {
            return FastOutputs::safe();
        }

        // Cache temperature for slow tick check
        self.safety.update_temperature(inputs.temperature);

        // 1. Validate position sensor
        let position = match self.safety.validate_position(inputs.position) {
            Ok(pos) => pos,
            Err(_count) => {
                if self.safety.is_sensor_fault() {
                    self.raise_fault(FaultKind::EncoderFault);
                    return FastOutputs::fault(FaultKind::EncoderFault);
                }
                // Skip this tick with bad position - don't actuate
                return FastOutputs::skip();
            }
        };

        // 2. Check current threshold (if sensor available)
        if let Some(fault) = self.safety.check_current(inputs.current) {
            self.raise_fault(fault);
            return FastOutputs::fault(fault);
        }

        // 3. Clamp setpoint to position bounds and run control loop
        let clamped_setpoint = self.safety.clamp_setpoint(self.controller.get_setpoint());
        let pwm_command = self
            .controller
            .compute(clamped_setpoint, position, inputs.current);

        // 4. Check for stall (PWM saturated but no movement)
        let output_max = self.controller.output_max();
        let pwm_saturated = pwm_command.abs() >= output_max;
        if let Some(fault) = self.safety.check_stall(position, pwm_saturated) {
            self.raise_fault(fault);
            return FastOutputs::fault(fault);
        }

        // 5. Update system state for telemetry
        self.system_state = SystemState {
            setpoint: clamped_setpoint,
            position,
            pwm_duty: pwm_command,
            current: inputs.current,
            bus_voltage: inputs.bus_voltage,
            temperature: inputs.temperature,
        };

        FastOutputs::normal(pwm_command)
    }

    /// Slow monitoring tick (100Hz).
    ///
    /// Performs less critical checks that don't need 10kHz rate.
    /// Returns fault kind if a fault was raised.
    pub fn slow_tick(&mut self) -> Option<FaultKind> {
        // Check temperature using cached value from fast tick
        if let Some(fault) = self
            .safety
            .check_temperature(self.safety.last_temperature())
        {
            self.raise_fault(fault);
            return Some(fault);
        }

        // Check for persistent position error (supervisory, doesn't need 10kHz)
        let setpoint = self.system_state.setpoint;
        let position = self.system_state.position;
        if let Some(fault) = self.safety.check_position_error(setpoint, position) {
            self.raise_fault(fault);
            return Some(fault);
        }

        None
    }

    /// Raise a fault internally.
    fn raise_fault(&mut self, kind: FaultKind) {
        self.fault_state.raise(kind);
        self.controller.reset();
    }

    /// Clear fault state and reset safety monitoring.
    pub fn clear_fault(&mut self) {
        self.fault_state.clear();
        self.safety.reset();
        self.controller.reset();
    }

    /// Check if the core is currently faulted.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        self.fault_state.is_faulted()
    }

    /// Get the current fault state.
    pub fn fault_state(&self) -> &FaultState {
        &self.fault_state
    }

    /// Get the current system state snapshot.
    pub fn system_state(&self) -> SystemState {
        self.system_state
    }

    /// Get reference to safety manager.
    pub fn safety(&self) -> &SafetyManager {
        &self.safety
    }

    /// Get mutable reference to safety manager.
    pub fn safety_mut(&mut self) -> &mut SafetyManager {
        &mut self.safety
    }

    /// Set the control setpoint (in centidegrees).
    ///
    /// The setpoint is clamped to position bounds.
    pub fn set_setpoint(&mut self, setpoint: CentiDeg) {
        let clamped = self.safety.clamp_setpoint(setpoint);
        self.controller.set_setpoint(clamped);
    }

    /// Get the current setpoint.
    pub fn get_setpoint(&self) -> CentiDeg {
        self.controller.get_setpoint()
    }

    /// Get mutable reference to the controller.
    pub fn controller_mut(&mut self) -> &mut C {
        &mut self.controller
    }

    /// Get reference to the controller.
    pub fn controller(&self) -> &C {
        &self.controller
    }
}
