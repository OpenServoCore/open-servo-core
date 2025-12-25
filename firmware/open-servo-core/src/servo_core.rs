//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.

use open_servo_control::{ControlInput, ControlLoop, DutyLimits};
use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
use open_servo_math::{CentiC, CentiDeg, Duty, MilliVolt, DegPerSec10, ComplianceConfig, ThermalModel};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;

use crate::fault::{FaultKind, FaultState};
use crate::inputs::FastInputs;
use crate::outputs::FastOutputs;
use crate::safety::{SafetyManager, SafetyThresholds};

/// Servo operating mode for compliance behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServoMode {
    /// Actively moving to setpoint
    Move,
    /// Holding position with reduced force
    Hold,
    /// Yielding to external force (backdrive detected)
    Yield,
}

/// Control loop frequency - MUST match how often fast_tick() is called
/// This is NOT the PWM frequency or ADC sample rate
const CONTROL_HZ: u32 = 10000;  // 10kHz fast_tick rate
const CONTROL_DT_US: u32 = 1_000_000 / CONTROL_HZ;

/// Velocity computation
const VELOCITY_DECIMATE: u16 = 10;  // Update every 1ms

/// Mode thresholds with hysteresis
const HOLD_ENTER_ERROR_CDEG: i16 = 500;   // 5°
const HOLD_EXIT_ERROR_CDEG: i16 = 700;    // 7°
const HOLD_ENTER_VEL_DPS10: i16 = 100;    // 10°/s
const HOLD_EXIT_VEL_DPS10: i16 = 150;     // 15°/s
const BACKDRIVE_VEL_THRESHOLD: i16 = 300;  // 30°/s

/// Timing (derived from CONTROL_HZ)
const SETPOINT_SETTLE_TICKS: u32 = (400 * CONTROL_HZ) / 1000;  // 400ms
const HOLD_ENTRY_TICKS: u32 = (300 * CONTROL_HZ) / 1000;       // 300ms
const YIELD_DURATION_TICKS: u32 = (200 * CONTROL_HZ) / 1000;   // 200ms
const YIELD_COAST_TICKS: u32 = (100 * CONTROL_HZ) / 1000;      // 100ms

/// Backdrive detection
const U_DEADBAND: i16 = 1638;  // 5% for sign comparison
const BACKDRIVE_PERSIST: u8 = 5;  // Require 5 consecutive detections

/// Hold duty cap curve parameters
const HOLD_ERROR_START: i16 = 500;   // 5° - start ramping
const HOLD_ERROR_END: i16 = 1500;    // 15° - max cap
const HOLD_DUTY_MIN: i16 = 6553;     // 20% at small error
const HOLD_DUTY_MAX: i16 = 14746;    // 45% at large error

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
    /// Compliance is currently being limited
    pub compliance_limited: bool,
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
    motor_engaged: bool,

    // Setpoint owned by ServoCore (Stage 1 refactor)
    setpoint: Option<CentiDeg>,

    // Compliance state
    mode: ServoMode,
    tick_counter: u32,
    
    // Velocity tracking (decimated)
    velocity_update_counter: u16,
    measured_velocity: DegPerSec10,
    prev_position: CentiDeg,
    
    // Setpoint tracking
    prev_setpoint: CentiDeg,
    setpoint_unchanged_ticks: u32,
    hold_conditions_met_ticks: u32,
    
    // Backdrive detection
    prev_pwm_command: i16,
    prev_error: i16,
    backdrive_detect_count: u8,
    yield_enter_tick: u32,
    yield_until_tick: u32,
    
    // Dual compliance configs
    move_compliance_config: ComplianceConfig,
    hold_compliance_config: ComplianceConfig,
}

impl<C: ControlLoop> ServoCore<C> {
    /// Create a new ServoCore with the given controller and board configuration.
    pub fn new(
        controller: C,
        safety_config: BoardSafetyConfig,
        thermal_config: BoardThermalConfig,
        move_compliance_config: ComplianceConfig,
        hold_compliance_config: ComplianceConfig,
    ) -> Self {
        // Create safety thresholds from board config
        let thresholds = SafetyThresholds::new(
            safety_config.current_limit_ma,
            safety_config.mcu_temp_limit_cc,
            safety_config.position_max_delta_cdeg,
            safety_config.sensor_fault_count,
            safety_config.position_min_cdeg,
            safety_config.position_max_cdeg,
            safety_config.stall_timeout_ticks,
            safety_config.stall_position_tolerance_cdeg,
            safety_config.position_error_limit_cdeg,
            safety_config.position_error_timeout_ticks,
        );
        
        // Create thermal model from board config
        let thermal_model = ThermalModel::new(
            thermal_config.resistance_mohm,
            thermal_config.thermal_resistance_cw,
            thermal_config.thermal_capacity_cj,
        );
        
        Self {
            controller,
            fault_state: FaultState::new(),
            safety: SafetyManager::new(thresholds, thermal_model, move_compliance_config.clone()),
            system_state: SystemState::default(),
            motor_engaged: false, // Start disengaged

            // Setpoint owned by ServoCore
            setpoint: None,

            // Compliance state
            mode: ServoMode::Move,
            tick_counter: 0,
            
            // Velocity tracking
            velocity_update_counter: 0,
            measured_velocity: DegPerSec10::from_dps10(0),
            prev_position: CentiDeg::from_cdeg(0),
            
            // Setpoint tracking
            prev_setpoint: CentiDeg::from_cdeg(0),
            setpoint_unchanged_ticks: 0,
            hold_conditions_met_ticks: 0,
            
            // Backdrive detection
            prev_pwm_command: 0,
            prev_error: 0,
            backdrive_detect_count: 0,
            yield_enter_tick: 0,
            yield_until_tick: 0,
            
            // Compliance configs
            move_compliance_config,
            hold_compliance_config,
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
        // Always increment master tick counter
        self.tick_counter = self.tick_counter.wrapping_add(1);
        
        // If motor is disengaged, return safe state (motor disabled)
        if !self.motor_engaged {
            return FastOutputs::safe();
        }
        
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
        
        // Update velocity (decimated with countdown)
        self.update_velocity(position);

        // Get setpoint - if None, return safe (motor disengaged or no target)
        let Some(setpoint) = self.setpoint else {
            return FastOutputs::safe();
        };

        // Clamp setpoint to bounds and track changes
        let clamped_setpoint = self.safety.clamp_setpoint(setpoint);
        self.update_setpoint_tracking(clamped_setpoint);

        // Calculate error for mode decisions
        let error = (clamped_setpoint - position).as_cdeg();

        // Update compliance mode
        let mode_changed = self.update_compliance_mode(position, clamped_setpoint, error);

        // Switch compliance config on mode transitions
        if mode_changed {
            match self.mode {
                ServoMode::Move => {
                    self.safety
                        .compliance_limiter_mut()
                        .set_config(self.move_compliance_config);
                }
                ServoMode::Hold | ServoMode::Yield => {
                    self.safety
                        .compliance_limiter_mut()
                        .set_config(self.hold_compliance_config);
                }
            }
        }

        // 3. Update compliance limiter with current reading
        self.safety
            .compliance_limiter_mut()
            .update(inputs.current(), error, CONTROL_DT_US);

        // 4. Get base limits from compliance limiter
        let (mut min_duty, mut max_duty) = self.safety.compliance_limiter().get_limits();

        // Apply mode-specific limits
        match self.mode {
            ServoMode::Move => {
                // Use full torque limiter limits
            }

            ServoMode::Hold => {
                // Apply error-based duty cap curve
                let duty_cap = self.calculate_hold_duty_cap(error.abs());
                min_duty = min_duty.max(-duty_cap as i32);
                max_duty = max_duty.min(duty_cap as i32);
            }

            ServoMode::Yield => {
                let yield_elapsed = self.tick_counter.saturating_sub(self.yield_enter_tick);

                if yield_elapsed < YIELD_COAST_TICKS {
                    // Pure coast for first 100ms
                    min_duty = 0;
                    max_duty = 0;
                } else {
                    // Optional small duty to "feel alive"
                    min_duty = -1638; // 5%
                    max_duty = 1638;
                }

                // Reset controller on YIELD entry
                if yield_elapsed == 0 {
                    self.controller.reset();
                }
            }
        }

        // 5. Build ControlInput and run controller
        let input = ControlInput {
            setpoint: clamped_setpoint,
            position,
            velocity: Some(self.measured_velocity),
            current: inputs.current(),
            bus_voltage: inputs.bus_voltage,
            limits: DutyLimits {
                min: Duty::from_raw(min_duty.clamp(i16::MIN as i32, i16::MAX as i32) as i16),
                max: Duty::from_raw(max_duty.clamp(i16::MIN as i32, i16::MAX as i32) as i16),
            },
        };

        let output = self.controller.fast_tick(&input);

        // Update tracking for next tick
        self.prev_pwm_command = output.duty.as_raw();
        self.prev_error = error;

        // 6. Check for stall using output.saturated (per-tick limits, not Duty::MAX)
        if let Some(fault) = self.safety.check_stall(position, output.saturated) {
            self.raise_fault(fault);
            return FastOutputs::fault(fault);
        }

        // 7. Update system state for telemetry
        self.system_state = SystemState {
            setpoint: clamped_setpoint,
            position,
            pwm_duty: output.duty,
            #[cfg(feature = "current-sense-bus")]
            current: inputs.current,
            bus_voltage: inputs.bus_voltage,
            temperature: inputs.temperature,
            compliance_limited: self.safety.compliance_limiter().is_limited(),
        };

        FastOutputs::normal(output.duty)
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
        self.setpoint = Some(clamped);
    }

    /// Get the current setpoint.
    pub fn get_setpoint(&self) -> Option<CentiDeg> {
        self.setpoint
    }

    /// Clear the setpoint (for disengagement).
    pub fn clear_setpoint(&mut self) {
        self.setpoint = None;
    }

    /// Get mutable reference to the controller.
    pub fn controller_mut(&mut self) -> &mut C {
        &mut self.controller
    }

    /// Get reference to the controller.
    pub fn controller(&self) -> &C {
        &self.controller
    }
    
    /// Engage the motor (enable control)
    /// If there's no setpoint, sets it to the current position to hold
    pub fn engage(&mut self, current_position: CentiDeg) {
        self.motor_engaged = true;

        // If no setpoint (was disengaged), set it to current position
        // This prevents jumping to zero on engage
        if self.setpoint.is_none() {
            self.setpoint = Some(current_position);
        }
    }

    /// Disengage the motor (disable control, motor will coast)
    pub fn disengage(&mut self) {
        self.motor_engaged = false;
        // Clear the setpoint so next engage will capture current position
        self.setpoint = None;
        // Reset controller state (clear integrator, derivative history)
        self.controller.reset();
    }
    
    /// Check if the motor is engaged
    pub fn is_engaged(&self) -> bool {
        self.motor_engaged
    }
    
    /// Update velocity estimation (decimated)
    fn update_velocity(&mut self, position: CentiDeg) {
        // Countdown pattern: updates exactly every N ticks
        if self.velocity_update_counter == 0 {
            let delta_cdeg = (position - self.prev_position).as_cdeg() as i32;
            
            // Direct calculation to avoid truncation
            // vel_dps10 = (delta_cdeg * CONTROL_HZ) / (10 * VELOCITY_DECIMATE)
            let vel_dps10_raw = (delta_cdeg * CONTROL_HZ as i32) / (10 * VELOCITY_DECIMATE as i32);
            let vel_dps10 = vel_dps10_raw.clamp(-32767, 32767);
            
            // IIR filter in i32 space to avoid overflow
            let old = self.measured_velocity.as_dps10() as i32;
            let new = vel_dps10;
            let filtered = (new + 3 * old) / 4;  // 0.25 new + 0.75 old
            
            self.measured_velocity = DegPerSec10::from_dps10(
                filtered.clamp(-32767, 32767) as i16
            );
            
            self.prev_position = position;
            self.velocity_update_counter = VELOCITY_DECIMATE - 1;
        } else {
            self.velocity_update_counter -= 1;
        }
    }
    
    /// Update setpoint tracking
    fn update_setpoint_tracking(&mut self, setpoint: CentiDeg) {
        if setpoint != self.prev_setpoint {
            // Setpoint changed
            self.setpoint_unchanged_ticks = 0;
            self.prev_setpoint = setpoint;
        } else {
            // Setpoint unchanged, increment counter (saturating)
            self.setpoint_unchanged_ticks = self.setpoint_unchanged_ticks.saturating_add(1);
        }
    }
    
    /// Check for backdrive condition
    fn check_backdrive(&mut self, velocity: DegPerSec10, error: i16) -> bool {
        // Only in HOLD mode
        if self.mode != ServoMode::Hold {
            return false;
        }
        
        let vel = velocity.as_dps10();
        let vel_abs = vel.abs();
        
        // Must exceed velocity threshold
        if vel_abs <= BACKDRIVE_VEL_THRESHOLD {
            self.backdrive_detect_count = 0;
            return false;
        }
        
        // Sign mismatch with deadband
        let u = self.prev_pwm_command;
        let u_active = u.abs() > U_DEADBAND;
        let opposing = u_active && ((vel > 0) != (u > 0));
        
        // Error growing (with small deadband)
        let error_growing = error.abs() > self.prev_error.abs() + 10;
        
        // Require either condition
        if opposing || error_growing {
            self.backdrive_detect_count = self.backdrive_detect_count.saturating_add(1);
            if self.backdrive_detect_count >= BACKDRIVE_PERSIST {
                return true;
            }
        } else {
            self.backdrive_detect_count = 0;
        }
        
        false
    }
    
    /// Update compliance mode based on conditions
    fn update_compliance_mode(&mut self, _position: CentiDeg, _setpoint: CentiDeg, error: i16) -> bool {
        let prev_mode = self.mode;
        let error_abs = error.abs();
        let vel_abs = self.measured_velocity.as_dps10().abs();
        
        match self.mode {
            ServoMode::Move => {
                // Check if conditions met for HOLD
                let hold_conditions = 
                    self.setpoint_unchanged_ticks >= SETPOINT_SETTLE_TICKS &&
                    error_abs < HOLD_ENTER_ERROR_CDEG &&
                    vel_abs < HOLD_ENTER_VEL_DPS10;
                    
                if hold_conditions {
                    self.hold_conditions_met_ticks = self.hold_conditions_met_ticks.saturating_add(1);
                    if self.hold_conditions_met_ticks >= HOLD_ENTRY_TICKS {
                        self.mode = ServoMode::Hold;
                        self.hold_conditions_met_ticks = 0;
                    }
                } else {
                    self.hold_conditions_met_ticks = 0;
                }
            }
            
            ServoMode::Hold => {
                // Check exit conditions (with hysteresis)
                if self.setpoint_unchanged_ticks < 100 ||  // Recent change
                   error_abs > HOLD_EXIT_ERROR_CDEG ||
                   vel_abs > HOLD_EXIT_VEL_DPS10 {
                    
                    self.mode = ServoMode::Move;
                    self.backdrive_detect_count = 0;
                }
                // Check for backdrive
                else if self.check_backdrive(self.measured_velocity, error) {
                    self.mode = ServoMode::Yield;
                    self.yield_enter_tick = self.tick_counter;
                    self.yield_until_tick = self.tick_counter + YIELD_DURATION_TICKS;
                    self.backdrive_detect_count = 0;
                }
            }
            
            ServoMode::Yield => {
                if self.tick_counter >= self.yield_until_tick {
                    self.mode = ServoMode::Hold;
                }
            }
        }
        
        prev_mode != self.mode
    }
    
    /// Calculate hold duty cap based on error
    fn calculate_hold_duty_cap(&self, error_cdeg: i16) -> i16 {
        if error_cdeg <= HOLD_ERROR_START {
            HOLD_DUTY_MIN
        } else if error_cdeg >= HOLD_ERROR_END {
            HOLD_DUTY_MAX
        } else {
            // Linear interpolation
            let range = (HOLD_ERROR_END - HOLD_ERROR_START) as i32;
            let progress = (error_cdeg - HOLD_ERROR_START) as i32;
            let duty_range = (HOLD_DUTY_MAX - HOLD_DUTY_MIN) as i32;
            HOLD_DUTY_MIN + ((duty_range * progress) / range) as i16
        }
    }
    
    /// Set move mode current limit
    pub fn set_move_current_limit(&mut self, ma: i16) {
        self.move_compliance_config.limit_ma = ma;
        if self.mode == ServoMode::Move {
            self.safety.compliance_limiter_mut().set_config(self.move_compliance_config);
        }
    }
    
    /// Set hold mode current limit
    pub fn set_hold_current_limit(&mut self, ma: i16) {
        self.hold_compliance_config.limit_ma = ma;
        if self.mode == ServoMode::Hold || self.mode == ServoMode::Yield {
            self.safety.compliance_limiter_mut().set_config(self.hold_compliance_config);
        }
    }
    
    /// Get current compliance mode
    pub fn compliance_mode(&self) -> ServoMode {
        self.mode
    }
    
    /// Get measured velocity
    pub fn measured_velocity(&self) -> DegPerSec10 {
        self.measured_velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use open_servo_control::ControlOutput;
    use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
    use open_servo_math::ComplianceConfig;

    /// Minimal mock controller for testing ServoCore.
    struct MockController {
        output: Duty,
        reset_called: bool,
    }

    impl MockController {
        fn new() -> Self {
            Self {
                output: Duty::ZERO,
                reset_called: false,
            }
        }

        fn set_output(&mut self, output: Duty) {
            self.output = output;
        }
    }

    impl ControlLoop for MockController {
        fn reset(&mut self) {
            self.reset_called = true;
            self.output = Duty::ZERO;
        }

        fn fast_tick(&mut self, _input: &ControlInput) -> ControlOutput {
            ControlOutput {
                duty: self.output,
                saturated: false,
            }
        }

        fn medium_tick(&mut self, _input: &ControlInput) {}
        fn slow_tick(&mut self, _input: &ControlInput) {}
    }

    /// Create a ServoCore with default test configuration.
    fn make_core(controller: MockController) -> ServoCore<MockController> {
        let safety_config = BoardSafetyConfig {
            current_limit_ma: 800,
            mcu_temp_limit_cc: 8000,
            position_max_delta_cdeg: 500,
            sensor_fault_count: 10,
            position_min_cdeg: 0,
            position_max_cdeg: 18000,
            stall_timeout_ticks: 1000,
            stall_position_tolerance_cdeg: 10,
            position_error_limit_cdeg: 3000,
            position_error_timeout_ticks: 50,
        };
        let thermal_config = BoardThermalConfig {
            resistance_mohm: 5000,        // 5.0Ω
            thermal_resistance_cw: 1000,  // 10°C/W
            thermal_capacity_cj: 1500,    // 15 J/°C
        };
        let compliance_config = ComplianceConfig::new(600, 50, 3, 230, 3277);
        ServoCore::new(
            controller,
            safety_config,
            thermal_config,
            compliance_config.clone(),
            compliance_config,
        )
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
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

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
        let mut core = make_core(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(500)); // Would produce output if position was valid
        core.engage(CentiDeg::from_cdeg(9000));

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
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

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
        let mut core = make_core(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(500));
        core.engage(CentiDeg::from_cdeg(9000));

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
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000)); // Must engage for fast_tick to cache temperature

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
        let mut core = make_core(MockController::new());

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
        let mut core = make_core(MockController::new());

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
        let mut core = make_core(MockController::new());

        // Try to set setpoint above max (default max is 18000 cdeg = 180°)
        core.set_setpoint(CentiDeg::from_cdeg(20000));
        assert_eq!(core.get_setpoint().unwrap().as_cdeg(), 18000);

        // Try to set setpoint below min (default min is 0)
        core.set_setpoint(CentiDeg::from_cdeg(-1000));
        assert_eq!(core.get_setpoint().unwrap().as_cdeg(), 0);
    }

    // ========== system state tests ==========

    #[test]
    fn test_system_state_updated_after_tick() {
        let mut core = make_core(MockController::new());
        core.controller_mut().set_output(Duty::from_raw(750));
        core.set_setpoint(CentiDeg::from_cdeg(9000)); // Use core's setpoint now
        core.engage(CentiDeg::from_cdeg(8000)); // Must engage for tick to run controller

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
