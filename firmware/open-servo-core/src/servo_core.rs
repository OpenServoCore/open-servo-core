//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.

use open_servo_control::{ControlInput, ControlLoop, DutyLimits};
use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;
use open_servo_math::{
    CentiC, CentiDeg, CentiDeg32, ComplianceConfig, DegPerSec10, Duty, MilliVolt, ThermalModel,
};

use crate::accumulator::FastAccumulator;
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

/// Time-based constants (converted to ticks at runtime via update_fast_dt_us)
const VELOCITY_PERIOD_US: u32 = 1000; // 1ms velocity window
const BACKDRIVE_PERSIST_US: u32 = 500; // 0.5ms for backdrive detection

/// Mode timing (microseconds)
const SETPOINT_SETTLE_US: u32 = 400_000; // 400ms
const HOLD_ENTRY_US: u32 = 300_000; // 300ms
const YIELD_DURATION_US: u32 = 200_000; // 200ms
const YIELD_COAST_US: u32 = 100_000; // 100ms
const SETPOINT_RECENT_CHANGE_US: u32 = 10_000; // 10ms hysteresis

/// Mode thresholds with hysteresis
const HOLD_ENTER_ERROR_CDEG: i16 = 500; // 5°
const HOLD_EXIT_ERROR_CDEG: i16 = 700; // 7°
const HOLD_ENTER_VEL_DPS10: i16 = 100; // 10°/s
const HOLD_EXIT_VEL_DPS10: i16 = 150; // 15°/s
const BACKDRIVE_VEL_THRESHOLD: i16 = 300; // 30°/s

/// Backdrive detection
const U_DEADBAND: i16 = 1638; // 5% for sign comparison

/// Hold duty cap curve parameters
const HOLD_ERROR_START: i16 = 500; // 5° - start ramping
const HOLD_ERROR_END: i16 = 1500; // 15° - max cap
const HOLD_DUTY_MIN: i16 = 6553; // 20% at small error
const HOLD_DUTY_MAX: i16 = 14746; // 45% at large error

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
    setpoint: Option<CentiDeg32>,

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
    backdrive_detect_count: u16,
    yield_enter_tick: u32,
    yield_until_tick: u32,

    // Dual compliance configs
    move_compliance_config: ComplianceConfig,
    hold_compliance_config: ComplianceConfig,

    /// Cached input from last successful fast_tick (for medium/slow tick use).
    last_input: Option<ControlInput>,

    /// Fast-tick accumulator for windowed statistics (medium tick).
    fast_accum: FastAccumulator,

    // Cached fast_dt_us and derived tick counts
    fast_dt_us: u32,
    velocity_decimate_ticks: u16,
    backdrive_persist_ticks: u16,
    setpoint_settle_ticks: u32,
    hold_entry_ticks: u32,
    yield_duration_ticks: u32,
    yield_coast_ticks: u32,
    setpoint_recent_change_ticks: u32,
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
            safety_config.position_error_timeout_us,
        );

        // Create thermal model from board config
        let thermal_model = ThermalModel::new(
            thermal_config.resistance_mohm,
            thermal_config.thermal_resistance_cw,
            thermal_config.thermal_capacity_cj,
        );

        let mut servo = Self {
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

            // Cached input for medium/slow tick
            last_input: None,

            // Fast-tick accumulator for medium tick
            fast_accum: FastAccumulator::new(),

            // Derived timing: placeholders, computed below via update_fast_dt_us()
            fast_dt_us: 0,
            velocity_decimate_ticks: 1,
            backdrive_persist_ticks: 1,
            setpoint_settle_ticks: 1,
            hold_entry_ticks: 1,
            yield_duration_ticks: 1,
            yield_coast_ticks: 1,
            setpoint_recent_change_ticks: 1,
        };

        // Initialize derived timing from SafetyManager's default fast_dt_us
        servo.update_fast_dt_us(servo.safety.fast_dt_us());
        servo
    }

    /// Hard real-time fast control tick (rate determined by board DT).
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

        // Accumulate for medium-tick windowed stats (only when engaged and not faulted)
        if self.can_run_control_ticks() {
            self.fast_accum.observe(position, inputs.current());
        }

        // Get setpoint - if None, return safe (motor disengaged or no target)
        let Some(sp32) = self.setpoint else {
            return FastOutputs::safe();
        };

        // Saturate i32 -> i16 for safety clamping
        let sp_sat = sp32.to_centi_deg_sat();

        // Clamp setpoint to bounds and track changes
        let clamped_setpoint = self.safety.clamp_setpoint(sp_sat);
        // Store the effective clamped setpoint so internal state remains within safety bounds
        // and doesn't repeatedly re-clamp each tick.
        self.setpoint = Some(CentiDeg32::from(clamped_setpoint));
        self.update_setpoint_tracking(clamped_setpoint);

        // i32 math for error - prevents overflow
        let sp = CentiDeg32::from(clamped_setpoint);
        let pv = CentiDeg32::from(position);
        let err32 = sp - pv;

        // For mode + limiter APIs (expect i16), saturate err32 -> i16
        let error: i16 = err32.to_cdeg_i16_sat();

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
        let fast_dt_us = self.safety.fast_dt_us();
        self.safety
            .compliance_limiter_mut()
            .update(inputs.current(), error, fast_dt_us);

        // 4. Get base limits from compliance limiter
        let (mut min_duty, mut max_duty) = self.safety.compliance_limiter().get_limits();

        // Apply mode-specific limits
        match self.mode {
            ServoMode::Move => {
                // Use full torque limiter limits
            }

            ServoMode::Hold => {
                // Apply error-based duty cap curve
                let duty_cap = self.calculate_hold_duty_cap(error.saturating_abs());
                min_duty = min_duty.max(-duty_cap as i32);
                max_duty = max_duty.min(duty_cap as i32);
            }

            ServoMode::Yield => {
                let yield_elapsed = self.tick_counter.saturating_sub(self.yield_enter_tick);

                if yield_elapsed < self.yield_coast_ticks {
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

        // Cache input for medium/slow tick (ControlMedium domain)
        self.last_input = Some(input);

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

        // Call controller's slow tick with same gating as medium_tick
        if self.can_run_control_ticks() {
            if let Some(ref input) = self.last_input {
                self.controller.slow_tick(input);
            }
        }

        None
    }

    /// ControlMedium tick - runs at decimated rate from fast tick.
    /// Stays aligned to ADC samples (derived from ControlFast domain).
    ///
    /// Uses windowed statistics from FastAccumulator:
    /// - velocity: derived from position delta over window
    /// - current: average over window
    pub fn control_medium_tick(&mut self) {
        if !self.can_run_control_ticks() {
            return;
        }

        // Take snapshot (computes window velocity/avg current, resets accumulator)
        if let Some(snap) = self.fast_accum.take_snapshot(self.safety.fast_dt_us()) {
            if let Some(ref base_input) = self.last_input {
                let medium_input = ControlInput {
                    setpoint: base_input.setpoint,
                    position: snap.pos_last,
                    velocity: Some(snap.velocity_dps10),
                    current: snap.current_avg,
                    bus_voltage: base_input.bus_voltage,
                    limits: base_input.limits,
                };
                self.controller.medium_tick(&medium_input);
            }
        }
    }

    /// Raise a fault internally.
    pub fn raise_fault(&mut self, kind: FaultKind) {
        self.fault_state.raise(kind);
        self.controller.reset();
        self.last_input = None;
        self.fast_accum.reset();
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
        self.setpoint = Some(CentiDeg32::from(clamped));
    }

    /// Get the current setpoint.
    pub fn get_setpoint(&self) -> Option<CentiDeg> {
        self.setpoint.map(|v| v.to_centi_deg_sat())
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
        if self.setpoint.is_none() {
            let clamped = self.safety.clamp_setpoint(current_position);
            self.setpoint = Some(CentiDeg32::from(clamped));
        }
    }

    /// Disengage the motor (disable control, motor will coast)
    pub fn disengage(&mut self) {
        self.motor_engaged = false;
        // Clear the setpoint so next engage will capture current position
        self.setpoint = None;
        // Reset controller state (clear integrator, derivative history)
        self.controller.reset();
        // Clear cached input (no medium/slow tick while disengaged)
        self.last_input = None;
        // Reset accumulator (no partial windows across state transitions)
        self.fast_accum.reset();
    }

    /// Check if the motor is engaged
    pub fn is_engaged(&self) -> bool {
        self.motor_engaged
    }

    /// Check if control ticks (medium/slow) should run.
    /// Centralized gating for controller tick methods.
    #[inline]
    fn can_run_control_ticks(&self) -> bool {
        self.motor_engaged && !self.fault_state.is_faulted()
    }

    /// Update velocity estimation (decimated)
    fn update_velocity(&mut self, position: CentiDeg) {
        // Countdown pattern: updates exactly every N ticks
        if self.velocity_update_counter == 0 {
            let delta_cdeg =
                (CentiDeg32::from(position) - CentiDeg32::from(self.prev_position)).as_cdeg();

            // Tick-rate-independent velocity: delta_cdeg * 100_000 / window_us
            // window_us = velocity_decimate_ticks * fast_dt_us
            // Use i64 to avoid overflow (delta_cdeg * 100_000 can exceed i32)
            let window_us = (self.velocity_decimate_ticks as u32).saturating_mul(self.fast_dt_us);
            let vel_dps10_raw = if window_us > 0 {
                ((delta_cdeg as i64) * 100_000 / (window_us as i64)) as i32
            } else {
                0
            };
            let vel_dps10 = vel_dps10_raw.clamp(-32767, 32767);

            // IIR filter in i32 space to avoid overflow
            let old = self.measured_velocity.as_dps10() as i32;
            let filtered = (vel_dps10 + 3 * old) / 4; // 0.25 new + 0.75 old

            self.measured_velocity = DegPerSec10::from_dps10(filtered.clamp(-32767, 32767) as i16);

            self.prev_position = position;
            self.velocity_update_counter = self.velocity_decimate_ticks.saturating_sub(1);
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
        let vel_abs = vel.saturating_abs();

        // Must exceed velocity threshold
        if vel_abs <= BACKDRIVE_VEL_THRESHOLD {
            self.backdrive_detect_count = 0;
            return false;
        }

        // Sign mismatch with deadband
        let u = self.prev_pwm_command;
        let u_active = u.saturating_abs() > U_DEADBAND;
        let opposing = u_active && ((vel > 0) != (u > 0));

        // Error growing (with small deadband)
        let error_abs = error.saturating_abs();
        let prev_error_abs = self.prev_error.saturating_abs();
        let error_growing = error_abs > prev_error_abs.saturating_add(10);

        // Require either condition
        if opposing || error_growing {
            self.backdrive_detect_count = self.backdrive_detect_count.saturating_add(1);
            if self.backdrive_detect_count >= self.backdrive_persist_ticks {
                return true;
            }
        } else {
            self.backdrive_detect_count = 0;
        }

        false
    }

    /// Update compliance mode based on conditions
    fn update_compliance_mode(
        &mut self,
        _position: CentiDeg,
        _setpoint: CentiDeg,
        error: i16,
    ) -> bool {
        let prev_mode = self.mode;
        let error_abs = error.saturating_abs();
        let vel_abs = self.measured_velocity.as_dps10().saturating_abs();

        match self.mode {
            ServoMode::Move => {
                // Check if conditions met for HOLD
                let hold_conditions = self.setpoint_unchanged_ticks >= self.setpoint_settle_ticks
                    && error_abs < HOLD_ENTER_ERROR_CDEG
                    && vel_abs < HOLD_ENTER_VEL_DPS10;

                if hold_conditions {
                    self.hold_conditions_met_ticks =
                        self.hold_conditions_met_ticks.saturating_add(1);
                    if self.hold_conditions_met_ticks >= self.hold_entry_ticks {
                        self.mode = ServoMode::Hold;
                        self.hold_conditions_met_ticks = 0;
                    }
                } else {
                    self.hold_conditions_met_ticks = 0;
                }
            }

            ServoMode::Hold => {
                // Check exit conditions (with hysteresis)
                if self.setpoint_unchanged_ticks < self.setpoint_recent_change_ticks
                    || error_abs > HOLD_EXIT_ERROR_CDEG
                    || vel_abs > HOLD_EXIT_VEL_DPS10
                {
                    self.mode = ServoMode::Move;
                    self.backdrive_detect_count = 0;
                }
                // Check for backdrive
                else if self.check_backdrive(self.measured_velocity, error) {
                    self.mode = ServoMode::Yield;
                    self.yield_enter_tick = self.tick_counter;
                    self.yield_until_tick = self.tick_counter + self.yield_duration_ticks;
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
            self.safety
                .compliance_limiter_mut()
                .set_config(self.move_compliance_config);
        }
    }

    /// Set hold mode current limit
    pub fn set_hold_current_limit(&mut self, ma: i16) {
        self.hold_compliance_config.limit_ma = ma;
        if self.mode == ServoMode::Hold || self.mode == ServoMode::Yield {
            self.safety
                .compliance_limiter_mut()
                .set_config(self.hold_compliance_config);
        }
    }

    /// Update the fast-tick period and recompute all derived timing fields.
    ///
    /// Called by the app layer when the board's actual tick rate is known or changes.
    /// At dt_us=100 (10kHz), derived values match historical defaults.
    /// The function returns early if dt_us hasn't changed.
    pub fn update_fast_dt_us(&mut self, dt_us: u32) {
        use crate::timing::ticks_from_us;

        let dt_us = dt_us.max(1); // Defensive: clamp to at least 1us
        if dt_us == self.fast_dt_us {
            return;
        }

        self.fast_dt_us = dt_us;

        // Recompute all derived ticks (clamp to at least 1)
        self.velocity_decimate_ticks = ticks_from_us(dt_us, VELOCITY_PERIOD_US).max(1) as u16;
        self.backdrive_persist_ticks = ticks_from_us(dt_us, BACKDRIVE_PERSIST_US).max(1) as u16;
        self.setpoint_settle_ticks = ticks_from_us(dt_us, SETPOINT_SETTLE_US).max(1);
        self.hold_entry_ticks = ticks_from_us(dt_us, HOLD_ENTRY_US).max(1);
        self.yield_duration_ticks = ticks_from_us(dt_us, YIELD_DURATION_US).max(1);
        self.yield_coast_ticks = ticks_from_us(dt_us, YIELD_COAST_US).max(1);
        self.setpoint_recent_change_ticks = ticks_from_us(dt_us, SETPOINT_RECENT_CHANGE_US).max(1);

        // Sanity checks (debug only): all derived ticks must be >= 1
        debug_assert!(self.velocity_decimate_ticks >= 1);
        debug_assert!(self.backdrive_persist_ticks >= 1);
    }

    /// Get current compliance mode
    pub fn compliance_mode(&self) -> ServoMode {
        self.mode
    }

    /// Get measured velocity
    pub fn measured_velocity(&self) -> DegPerSec10 {
        self.measured_velocity
    }

    #[cfg(test)]
    fn set_setpoint_i32_for_test(&mut self, sp: i32) {
        self.setpoint = Some(CentiDeg32::from_cdeg(sp));
    }

    #[cfg(test)]
    fn set_prev_error_for_test(&mut self, prev_error: i16) {
        self.prev_error = prev_error;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{make_core, make_inputs, MockController};

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
        assert_eq!(
            core.fault_state().fault_kind(),
            Some(FaultKind::OverCurrent)
        );
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

    #[test]
    fn test_get_setpoint_saturates_large_i32() {
        let mut core = make_core(MockController::new());
        core.set_setpoint_i32_for_test(40000);
        assert_eq!(core.get_setpoint().unwrap().as_cdeg(), i16::MAX);
    }

    #[test]
    fn test_get_setpoint_saturates_negative_i32() {
        let mut core = make_core(MockController::new());
        core.set_setpoint_i32_for_test(-40000);
        assert_eq!(core.get_setpoint().unwrap().as_cdeg(), i16::MIN);
    }

    #[test]
    fn test_fast_tick_error_no_overflow() {
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(-30000));
        core.set_setpoint_i32_for_test(30000);
        let inputs = make_inputs(-30000);
        let outputs = core.fast_tick(inputs);
        assert!(outputs.motor_enable);
    }

    #[test]
    fn test_fast_tick_persists_clamped_setpoint() {
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(0));

        // Put an out-of-range internal setpoint
        core.set_setpoint_i32_for_test(40000);

        // Tick once to trigger clamping + persistence
        let _ = core.fast_tick(make_inputs(0));

        // Internal setpoint should now be clamped to configured position_max
        let expected_max = CentiDeg32::from(core.safety().thresholds().position_max);
        assert_eq!(core.setpoint, Some(expected_max));
    }

    #[test]
    fn test_saturating_abs_prevents_i16_min_bug() {
        // Regression test: i16::MIN.abs() returns i16::MIN (negative!) due to overflow.
        // The old logic: cur.abs() > prev.abs() + 10 would give wrong results.
        // let prev = i16::MIN;
        // let cur: i16 = 0;
        // let old_logic = cur.abs() > prev.abs() + 10; // WRONG: prev.abs() is -32768

        let prev = i16::MIN;
        let cur: i16 = 0;
        let new_logic = cur.saturating_abs() > prev.saturating_abs().saturating_add(10);
        // cur.saturating_abs() = 0
        // prev.saturating_abs() = 32767 (saturated from MIN)
        // prev.saturating_abs().saturating_add(10) = 32767 (saturated)
        // 0 > 32767 = false
        assert!(!new_logic);
    }

    #[test]
    fn check_backdrive_saturating_abs_regression() {
        // Regression test: actually execute check_backdrive() with prev_error = i16::MIN
        // to ensure the saturating_abs() fix is exercised in real code path.
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Force HOLD mode (required for check_backdrive to pass first gate)
        core.mode = ServoMode::Hold;

        // Set prev_error to i16::MIN - the edge case that would overflow with .abs()
        core.set_prev_error_for_test(i16::MIN);

        // Velocity exceeds threshold (300), error is small
        let velocity = DegPerSec10::from_dps10(500);
        let error: i16 = 0;

        // Execute check_backdrive - this DEFINITELY executes the error_growing line
        // If saturating_abs() wasn't used, i16::MIN.abs() would overflow to -32768
        let result = core.check_backdrive(velocity, error);

        // Error is NOT growing:
        //   error_abs = 0
        //   prev_error_abs = 32767 (saturated from i16::MIN)
        //   0 > 32767.saturating_add(10) = false
        // opposing = false (prev_pwm_command = 0, so u_active = false)
        // Neither condition met, so result = false
        assert!(!result);
        assert_eq!(core.backdrive_detect_count, 0);
    }

    // ========== Derived timing tests ==========

    #[test]
    fn test_derived_timing_at_default_dt() {
        // new() uses SafetyManager's default fast_dt_us (100us), so values match 10kHz defaults
        let mut core = make_core(MockController::new());
        assert_eq!(core.fast_dt_us, 100);
        assert_eq!(core.velocity_decimate_ticks, 10);
        assert_eq!(core.backdrive_persist_ticks, 5);
        assert_eq!(core.setpoint_settle_ticks, 4000);
        assert_eq!(core.hold_entry_ticks, 3000);
        assert_eq!(core.yield_duration_ticks, 2000);
        assert_eq!(core.yield_coast_ticks, 1000);
        assert_eq!(core.setpoint_recent_change_ticks, 100);

        // Calling update_fast_dt_us with same value is a no-op
        core.update_fast_dt_us(100);
        assert_eq!(core.fast_dt_us, 100);
        assert_eq!(core.velocity_decimate_ticks, 10);
    }

    #[test]
    fn test_derived_timing_at_different_rate() {
        // With dt_us=125 (8kHz)
        let mut core = make_core(MockController::new());
        core.update_fast_dt_us(125);

        assert_eq!(core.fast_dt_us, 125);
        assert_eq!(core.velocity_decimate_ticks, 8); // 1000/125 = 8
        assert_eq!(core.backdrive_persist_ticks, 4); // 500/125 = 4
        assert_eq!(core.setpoint_settle_ticks, 3200); // 400_000/125 = 3200
        assert_eq!(core.hold_entry_ticks, 2400); // 300_000/125 = 2400
        assert_eq!(core.yield_duration_ticks, 1600); // 200_000/125 = 1600
        assert_eq!(core.yield_coast_ticks, 800); // 100_000/125 = 800
        assert_eq!(core.setpoint_recent_change_ticks, 80); // 10_000/125 = 80
    }

    #[test]
    fn test_derived_timing_rounds_up() {
        // With dt_us=150 (6.67kHz), test rounding up
        let mut core = make_core(MockController::new());
        core.update_fast_dt_us(150);

        // 1000/150 = 6.67 -> rounds up to 7
        assert_eq!(core.velocity_decimate_ticks, 7);
        // 500/150 = 3.33 -> rounds up to 4
        assert_eq!(core.backdrive_persist_ticks, 4);
    }

    #[test]
    fn test_derived_timing_clamps_to_min_1() {
        // With very slow tick rate, ensure minimum 1 tick
        let mut core = make_core(MockController::new());
        core.update_fast_dt_us(2_000_000); // 2 second ticks (absurd but tests clamp)

        assert!(core.velocity_decimate_ticks >= 1);
        assert!(core.backdrive_persist_ticks >= 1);
        assert!(core.setpoint_settle_ticks >= 1);
        assert!(core.hold_entry_ticks >= 1);
        assert!(core.yield_duration_ticks >= 1);
        assert!(core.yield_coast_ticks >= 1);
        assert!(core.setpoint_recent_change_ticks >= 1);
    }

    #[test]
    fn test_derived_timing_clamps_dt_us_to_min_1() {
        // Zero dt_us should be clamped to 1
        let mut core = make_core(MockController::new());
        core.update_fast_dt_us(0);

        // dt_us should be clamped to 1
        assert_eq!(core.fast_dt_us, 1);
        // At 1us/tick, ticks = duration_us (with rounding)
        assert_eq!(core.velocity_decimate_ticks, 1000);
        assert_eq!(core.backdrive_persist_ticks, 500);
    }

    #[test]
    fn test_derived_timing_idempotent() {
        // Calling update_fast_dt_us with same value leaves fields unchanged
        let mut core = make_core(MockController::new());
        let before = (
            core.velocity_decimate_ticks,
            core.backdrive_persist_ticks,
            core.setpoint_settle_ticks,
        );
        core.update_fast_dt_us(100);
        core.update_fast_dt_us(100); // Second call
        assert_eq!(
            (
                core.velocity_decimate_ticks,
                core.backdrive_persist_ticks,
                core.setpoint_settle_ticks,
            ),
            before
        );
    }
}
