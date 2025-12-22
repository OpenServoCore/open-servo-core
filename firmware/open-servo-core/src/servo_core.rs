//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.

use open_servo_control::ControlLoop;
use open_servo_math::{CentiC, CentiDeg, Duty, MilliVolt};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;

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
    pub pwm_duty: Duty,
    /// Current reading (requires `current-sense` feature)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage reading (if available)
    pub bus_voltage: Option<MilliVolt>,
    /// Temperature reading (if available)
    pub temperature: Option<CentiC>,
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
        
        // Accumulate I² for thermal model (actual update happens in slow tick)
        self.safety.accumulate_thermal_i_squared(inputs.current());

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

        // 2. Check current threshold (no-op if current-sense disabled)
        if let Some(fault) = self.safety.check_current(inputs.current()) {
            self.raise_fault(fault);
            return FastOutputs::fault(fault);
        }
        
        // 3. Clamp setpoint to position bounds and run control loop
        let clamped_setpoint = self.safety.clamp_setpoint(self.controller.get_setpoint());
        let pwm_command = self
            .controller
            .compute(clamped_setpoint, position, inputs.current());

        // 4. Check for stall (PWM saturated but no movement)
        let pwm_saturated = pwm_command.abs() >= Duty::MAX.abs();
        if let Some(fault) = self.safety.check_stall(position, pwm_saturated) {
            self.raise_fault(fault);
            return FastOutputs::fault(fault);
        }

        // 5. Update system state for telemetry
        self.system_state = SystemState {
            setpoint: clamped_setpoint,
            position,
            pwm_duty: pwm_command,
            #[cfg(feature = "current-sense-bus")]
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
        // Update thermal model with accumulated I² from fast ticks
        self.safety.update_thermal_slow();
        
        // Check MCU temperature using cached value from fast tick
        if let Some(fault) = self
            .safety
            .check_mcu_temperature(self.safety.last_temperature())
        {
            self.raise_fault(fault);
            return Some(fault);
        }
        
        // Check motor temperature (thermal model)
        if let Some(fault) = self.safety.check_motor_temperature() {
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

#[cfg(test)]
mod tests {
    use super::*;
    use open_servo_math::MilliAmp;

    /// Minimal mock controller for testing ServoCore.
    struct MockController {
        setpoint: CentiDeg,
        output: Duty,
        reset_called: bool,
    }

    impl MockController {
        fn new() -> Self {
            Self {
                setpoint: CentiDeg::from_cdeg(9000),
                output: Duty::ZERO,
                reset_called: false,
            }
        }

        fn set_output(&mut self, output: Duty) {
            self.output = output;
        }
    }

    impl ControlLoop for MockController {
        fn compute(
            &mut self,
            _setpoint: CentiDeg,
            _position: CentiDeg,
            _current: Option<MilliAmp>,
        ) -> Duty {
            self.output
        }

        fn reset(&mut self) {
            self.reset_called = true;
            self.output = Duty::ZERO;
        }

        fn set_setpoint(&mut self, setpoint: CentiDeg) {
            self.setpoint = setpoint;
        }

        fn get_setpoint(&self) -> CentiDeg {
            self.setpoint
        }
    }

    fn make_inputs(position: i16) -> FastInputs {
        FastInputs {
            position: CentiDeg::from_cdeg(position),
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: None,
        }
    }

    // ========== fast_tick tests ==========

    #[test]
    fn test_fast_tick_returns_safe_when_faulted() {
        let mut core = ServoCore::new(MockController::new());

        // Manually fault the core
        core.fault_state.raise(FaultKind::OverCurrent);
        assert!(core.is_faulted());

        // fast_tick should return safe outputs
        let outputs = core.fast_tick(make_inputs(9000));
        assert_eq!(outputs.pwm_command, Duty::ZERO);
        assert!(!outputs.motor_enable);
    }

    #[test]
    fn test_fast_tick_skips_on_bad_position() {
        let mut core = ServoCore::new(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(500)); // Would produce output if position was valid

        // First tick initializes sensor health at position 9000
        core.fast_tick(make_inputs(9000));

        // Sudden large jump should be rejected (delta > max_delta threshold)
        // Skip keeps motor enabled but outputs PWM 0 (hold position, don't actuate on bad data)
        let outputs = core.fast_tick(make_inputs(0)); // Jump from 9000 to 0
        assert_eq!(outputs.pwm_command, Duty::ZERO);
        assert!(outputs.motor_enable); // Skip keeps motor enabled, just sends 0 PWM
    }

    #[cfg(feature = "current-sense-bus")]
    #[test]
    fn test_fast_tick_faults_on_overcurrent() {
        let mut core = ServoCore::new(MockController::new());

        // Initialize with normal position
        core.fast_tick(make_inputs(9000));

        // Trigger overcurrent (default limit is 800mA)
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            current: Some(MilliAmp::from_ma(1000)), // Over 800mA limit
            bus_voltage: None,
            temperature: None,
        };

        let outputs = core.fast_tick(inputs);
        assert!(core.is_faulted());
        assert_eq!(core.fault_state().fault_kind(), Some(FaultKind::OverCurrent));
        assert_eq!(outputs.pwm_command, Duty::ZERO);
    }

    #[test]
    fn test_fast_tick_normal_operation() {
        let mut core = ServoCore::new(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(500));

        // Initialize position
        core.fast_tick(make_inputs(9000));

        // Normal tick should produce output
        let outputs = core.fast_tick(make_inputs(9000));
        assert!(outputs.motor_enable);
        assert_eq!(outputs.pwm_command, Duty::from_raw(500));
    }

    // ========== slow_tick tests ==========

    #[test]
    fn test_slow_tick_faults_on_overtemp() {
        let mut core = ServoCore::new(MockController::new());

        // Cache high temperature via fast_tick
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: Some(CentiC::from_centi_c(9000)), // 90°C, over 80°C limit
        };
        core.fast_tick(inputs);

        // slow_tick should detect MCU overtemp
        let fault = core.slow_tick();
        assert_eq!(fault, Some(FaultKind::McuOverTemp));
        assert!(core.is_faulted());
    }

    #[test]
    fn test_slow_tick_no_fault_when_normal() {
        let mut core = ServoCore::new(MockController::new());

        // Cache normal temperature
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: Some(CentiC::from_centi_c(2500)), // 25°C, well under limit
        };
        core.fast_tick(inputs);

        // slow_tick should return None
        assert_eq!(core.slow_tick(), None);
        assert!(!core.is_faulted());
    }

    // ========== clear_fault tests ==========

    #[test]
    fn test_clear_fault_resets_all_state() {
        let mut core = ServoCore::new(MockController::new());

        // Fault the core
        core.fault_state.raise(FaultKind::Stall);
        assert!(core.is_faulted());

        // Clear fault
        core.clear_fault();

        // Should be unfaulted and controller reset
        assert!(!core.is_faulted());
        assert!(core.controller_mut().reset_called);
    }

    // ========== setpoint tests ==========

    #[test]
    fn test_setpoint_clamped_to_bounds() {
        let mut core = ServoCore::new(MockController::new());

        // Try to set setpoint above max (default max is 18000 cdeg = 180°)
        core.set_setpoint(CentiDeg::from_cdeg(20000));
        assert_eq!(core.get_setpoint().as_cdeg(), 18000);

        // Try to set setpoint below min (default min is 0)
        core.set_setpoint(CentiDeg::from_cdeg(-1000));
        assert_eq!(core.get_setpoint().as_cdeg(), 0);
    }

    // ========== system state tests ==========

    #[test]
    fn test_system_state_updated_after_tick() {
        let mut core = ServoCore::new(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(750));
        core.controller_mut().setpoint = CentiDeg::from_cdeg(9000);

        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(8000),
            #[cfg(feature = "current-sense-bus")]
            current: Some(MilliAmp::from_ma(200)),
            bus_voltage: Some(MilliVolt::from_mv(3300)),
            temperature: Some(CentiC::from_centi_c(3000)),
        };

        // Need two ticks - first initializes position sensor
        core.fast_tick(inputs);
        core.fast_tick(inputs);

        let state = core.system_state();
        assert_eq!(state.position.as_cdeg(), 8000);
        assert_eq!(state.pwm_duty, Duty::from_raw(750));
        #[cfg(feature = "current-sense-bus")]
        assert_eq!(state.current, Some(MilliAmp::from_ma(200)));
        assert_eq!(state.bus_voltage, Some(MilliVolt::from_mv(3300)));
        assert_eq!(state.temperature, Some(CentiC::from_centi_c(3000)));
    }
}
