//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.

use open_servo_control::{ControlInput, ControlLoop, DutyLimits};
use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;
use open_servo_math::TickCtx;
use open_servo_math::{
    CentiC, CentiDeg, CentiDeg32, ComplianceConfig, DegPerSec10, Duty, MilliVolt, ThermalModel,
};

use crate::accumulator::FastAccumulator;
use crate::fault::{FaultKind, FaultState};
use crate::inputs::{build_medium_control_input, FastInputs};
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

/// Time-based constants for policy/state machine logic
const BACKDRIVE_PERSIST_US: u32 = 500; // 0.5ms for backdrive detection

/// Mode timing (microseconds)
const SETPOINT_SETTLE_US: u32 = 400_000; // 400ms
const HOLD_ENTRY_US: u32 = 300_000; // 300ms
const YIELD_DURATION_US: u32 = 200_000; // 200ms
const YIELD_COAST_US: u32 = 100_000; // 100ms
/// 5% alive clamp during Yield to keep loop responsive
const YIELD_ALIVE_DUTY_MAX: i16 = 1638;
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
/// let ctx = TickCtx { domain: TickDomain::ControlFast, dt_us: 100, seq: 0 };
/// let inputs = FastInputs { position, current, bus_voltage, temperature };
/// let outputs = core.fast_tick(&ctx, inputs);
/// // Apply outputs to hardware
///
/// // In slow tick:
/// let ctx = TickCtx { domain: TickDomain::System, dt_us: 10000, seq: 0 };
/// if let Some(fault) = core.slow_tick(&ctx) {
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
    /// Monotonic sequence number for fast ticks (not for timing, telemetry/debug only)
    fast_seq: u32,

    // Velocity (from MediumSnapshot, no decimation counter needed)
    measured_velocity: DegPerSec10,

    // Setpoint tracking (time-based)
    prev_setpoint: CentiDeg,
    setpoint_unchanged_us: u32,
    hold_conditions_met_us: u32,

    // Backdrive detection (time-based)
    prev_pwm_command: i16,
    prev_error: i16,
    backdrive_elapsed_us: u32,
    // Yield timing (time-based, accumulated in medium tick)
    yield_elapsed_us: u32,
    yield_max_duty: i16, // 0 = coast, YIELD_ALIVE_DUTY_MAX = alive (set by medium tick)
    yield_needs_reset: bool,

    // Dual compliance configs
    move_compliance_config: ComplianceConfig,
    hold_compliance_config: ComplianceConfig,

    /// Cached input from last successful fast_tick (for medium/slow tick use).
    last_input: Option<ControlInput>,

    /// Fast-tick accumulator for windowed statistics (medium tick).
    fast_accum: FastAccumulator,

    // Cached fast_dt_us for per-sample timing
    fast_dt_us: u32,
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
            fast_seq: 0,

            // Velocity (from MediumSnapshot)
            measured_velocity: DegPerSec10::from_dps10(0),

            // Setpoint tracking (time-based, updated in medium tick)
            prev_setpoint: CentiDeg::from_cdeg(0),
            setpoint_unchanged_us: 0,
            hold_conditions_met_us: 0,

            // Backdrive detection (time-based, updated in medium tick)
            prev_pwm_command: 0,
            prev_error: 0,
            backdrive_elapsed_us: 0,
            yield_elapsed_us: 0,
            yield_max_duty: 0,
            yield_needs_reset: false,

            // Compliance configs
            move_compliance_config,
            hold_compliance_config,

            // Cached input for medium/slow tick
            last_input: None,

            // Fast-tick accumulator for medium tick
            fast_accum: FastAccumulator::new(),

            // Cached fast_dt_us for per-sample timing
            fast_dt_us: 0,
        };

        // Initialize fast_dt_us from SafetyManager's default
        servo.update_fast_dt_us(servo.safety.fast_dt_us());
        servo
    }

    /// Hard real-time fast control tick (rate determined by board DT).
    ///
    /// Sample, observe, safety, actuate. **No timing decisions.**
    ///
    /// Pure function: takes sensor inputs, returns motor outputs.
    /// No hardware access - fully host-testable. All policy/FSM logic
    /// is in medium tick; fast tick only applies cached values (e.g., yield_max_duty).
    ///
    /// ## Allowed Operations
    ///
    /// - Sample accumulation (position, current)
    /// - Immediate safety latches (current threshold, stall, thermal)
    /// - Controller update (fast_tick on ControlLoop)
    /// - Output limit application from cached policy values
    ///
    /// ## NOT Allowed (must be in medium_tick)
    ///
    /// - Mode transitions (Move/Hold/Yield)
    /// - Timing comparisons against constants
    /// - FSM state changes
    ///
    /// ## Safety Checks (in order)
    ///
    /// 1. If already faulted, return safe outputs
    /// 2. Validate position sensor (skip tick on bad reading)
    /// 3. Check current threshold (if sensor available)
    /// 4. Run control loop with clamped setpoint
    /// 5. Check for motor stall (PWM saturated + no movement)
    #[inline]
    pub fn fast_tick(&mut self, ctx: &TickCtx, inputs: FastInputs) -> FastOutputs {
        // Increment sequence counter (telemetry/debug only, not for timing)
        self.fast_seq = self.fast_seq.wrapping_add(1);

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

        // Accumulate for medium-tick windowed stats (only when engaged and not faulted)
        // Velocity is now computed in control_medium_tick from MediumSnapshot
        if self.can_run_control_ticks() {
            self.fast_accum.observe(position, inputs.current());
        }

        // Get setpoint - if None, return safe (motor disengaged or no target)
        let Some(sp32) = self.setpoint else {
            return FastOutputs::safe();
        };

        // Saturate i32 -> i16 for safety clamping
        let sp_sat = sp32.to_centi_deg_sat();

        // Clamp setpoint to bounds
        let clamped_setpoint = self.safety.clamp_setpoint(sp_sat);
        // Store the effective clamped setpoint so internal state remains within safety bounds
        // and doesn't repeatedly re-clamp each tick.
        self.setpoint = Some(CentiDeg32::from(clamped_setpoint));

        // i32 math for error - prevents overflow
        let sp = CentiDeg32::from(clamped_setpoint);
        let pv = CentiDeg32::from(position);
        let err32 = sp - pv;

        // For mode + limiter APIs (expect i16), saturate err32 -> i16
        let error: i16 = err32.to_cdeg_i16_sat();

        // Note: Setpoint tracking and compliance mode transitions happen in control_medium_tick
        // to use time-based thresholds. Fast_tick uses the current mode for limits.

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
                // Use yield_max_duty set by medium tick (no timing decisions in fast tick)
                min_duty = -(self.yield_max_duty as i32);
                max_duty = self.yield_max_duty as i32;

                // Reset controller on YIELD entry (once)
                if self.yield_needs_reset {
                    self.controller.reset();
                    self.yield_needs_reset = false;
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

        let output = self.controller.fast_tick(ctx, &input);

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
    pub fn slow_tick(&mut self, ctx: &TickCtx) -> Option<FaultKind> {
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
                self.controller.slow_tick(ctx, input);
            }
        }

        None
    }

    /// ControlMedium tick - runs at decimated rate from fast tick.
    /// Stays aligned to ADC samples (derived from ControlFast domain).
    ///
    /// **Policy FSM using time-based accumulators (window_dt_us).**
    ///
    /// Uses windowed statistics from FastAccumulator:
    /// - velocity: derived from position delta over window
    /// - current: average over window
    ///
    /// Runs time-based policy logic (all using wall-clock microseconds):
    /// - setpoint tracking (time unchanged)
    /// - compliance mode transitions (Move/Hold/Yield)
    /// - backdrive detection
    ///
    /// Timing behavior is independent of tick rate and sample_count;
    /// only total elapsed microseconds (window_dt_us) affects transitions.
    pub fn control_medium_tick(&mut self, ctx: &TickCtx) {
        if !self.can_run_control_ticks() {
            return;
        }

        // Take snapshot (computes window velocity/avg current, resets accumulator)
        let Some(snap) = self.fast_accum.take_snapshot(self.safety.fast_dt_us()) else {
            return;
        };

        let window_dt_us = snap.window_dt_us;

        // Update velocity from snapshot
        self.measured_velocity = snap.velocity_dps10;

        // Copy values from last_input to avoid borrow conflicts
        let Some(base_input) = self.last_input else {
            return;
        };

        // Update setpoint tracking (time-based)
        self.update_setpoint_tracking_medium(base_input.setpoint, window_dt_us);

        // Compute error for mode transitions
        let error = (base_input.setpoint.as_cdeg() as i32 - snap.pos_last.as_cdeg() as i32)
            .clamp(i16::MIN as i32, i16::MAX as i32) as i16;

        // Update compliance mode (may transition Move/Hold/Yield)
        let mode_changed = self.update_compliance_mode_medium(error, window_dt_us);

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

        // Build medium input and call controller
        let medium_input = build_medium_control_input(&base_input, &snap);
        self.controller.medium_tick(ctx, &medium_input);
    }

    /// Raise a fault internally.
    pub fn raise_fault(&mut self, kind: FaultKind) {
        self.fault_state.raise(kind);
        self.controller.reset();
        self.last_input = None;
        self.fast_accum.reset();
        self.yield_elapsed_us = 0;
        self.yield_max_duty = 0;
        self.yield_needs_reset = false;
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
        // Clear yield timing state
        self.yield_elapsed_us = 0;
        self.yield_max_duty = 0;
        self.yield_needs_reset = false;
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

    /// Update setpoint tracking (time-based, called from medium tick)
    fn update_setpoint_tracking_medium(&mut self, setpoint: CentiDeg, window_dt_us: u32) {
        if setpoint != self.prev_setpoint {
            // Setpoint changed - reset accumulated time
            self.setpoint_unchanged_us = 0;
            self.prev_setpoint = setpoint;
        } else {
            // Setpoint unchanged - accumulate time
            self.setpoint_unchanged_us = self.setpoint_unchanged_us.saturating_add(window_dt_us);
        }
    }

    /// Check for backdrive condition (time-based, called from medium tick)
    fn check_backdrive_medium(
        &mut self,
        velocity: DegPerSec10,
        error: i16,
        window_dt_us: u32,
    ) -> bool {
        // Only in HOLD mode
        if self.mode != ServoMode::Hold {
            return false;
        }

        let vel = velocity.as_dps10();
        let vel_abs = vel.saturating_abs();

        // Must exceed velocity threshold
        if vel_abs <= BACKDRIVE_VEL_THRESHOLD {
            self.backdrive_elapsed_us = 0;
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

        // Require either condition to accumulate time
        if opposing || error_growing {
            self.backdrive_elapsed_us = self.backdrive_elapsed_us.saturating_add(window_dt_us);
            if self.backdrive_elapsed_us >= BACKDRIVE_PERSIST_US {
                return true;
            }
        } else {
            self.backdrive_elapsed_us = 0;
        }

        false
    }

    /// Update compliance mode based on conditions (time-based, called from medium tick)
    fn update_compliance_mode_medium(&mut self, error: i16, window_dt_us: u32) -> bool {
        let prev_mode = self.mode;
        let error_abs = error.saturating_abs();
        let vel_abs = self.measured_velocity.as_dps10().saturating_abs();

        match self.mode {
            ServoMode::Move => {
                // Check if conditions met for HOLD (all thresholds are now in microseconds)
                let hold_conditions = self.setpoint_unchanged_us >= SETPOINT_SETTLE_US
                    && error_abs < HOLD_ENTER_ERROR_CDEG
                    && vel_abs < HOLD_ENTER_VEL_DPS10;

                if hold_conditions {
                    self.hold_conditions_met_us =
                        self.hold_conditions_met_us.saturating_add(window_dt_us);
                    if self.hold_conditions_met_us >= HOLD_ENTRY_US {
                        self.mode = ServoMode::Hold;
                        self.hold_conditions_met_us = 0;
                    }
                } else {
                    self.hold_conditions_met_us = 0;
                }
            }

            ServoMode::Hold => {
                // Check exit conditions (with hysteresis, time-based)
                if self.setpoint_unchanged_us < SETPOINT_RECENT_CHANGE_US
                    || error_abs > HOLD_EXIT_ERROR_CDEG
                    || vel_abs > HOLD_EXIT_VEL_DPS10
                {
                    self.mode = ServoMode::Move;
                    self.backdrive_elapsed_us = 0;
                }
                // Check for backdrive
                else if self.check_backdrive_medium(self.measured_velocity, error, window_dt_us) {
                    self.mode = ServoMode::Yield;
                    self.yield_elapsed_us = 0; // Reset on entry
                    self.yield_max_duty = 0; // Coast phase on entry
                    self.yield_needs_reset = true;
                    self.backdrive_elapsed_us = 0;
                }
            }

            ServoMode::Yield => {
                // Accumulate time-based elapsed (only when window_dt_us > 0, clamp to duration)
                if window_dt_us > 0 {
                    self.yield_elapsed_us = self
                        .yield_elapsed_us
                        .saturating_add(window_dt_us)
                        .min(YIELD_DURATION_US);
                }
                // Update duty phase based on elapsed time
                self.yield_max_duty = if self.yield_elapsed_us < YIELD_COAST_US {
                    0 // Coast phase
                } else {
                    YIELD_ALIVE_DUTY_MAX // Alive phase
                };
                if self.yield_elapsed_us >= YIELD_DURATION_US {
                    self.mode = ServoMode::Hold;
                    self.yield_elapsed_us = 0; // Reset on exit
                    self.yield_max_duty = 0;
                    self.yield_needs_reset = false;
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

    /// Update cached fast_dt_us when tick rate changes.
    ///
    /// Called by the app layer when the board's actual tick rate is known or changes.
    /// Returns early if dt_us hasn't changed. Clamps to at least 1us.
    pub fn update_fast_dt_us(&mut self, dt_us: u32) {
        let dt_us = dt_us.max(1); // Defensive: clamp to at least 1us
        if dt_us == self.fast_dt_us {
            return;
        }

        self.fast_dt_us = dt_us;
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

    #[cfg(test)]
    fn set_mode_for_test(&mut self, mode: ServoMode) {
        self.mode = mode;
    }

    #[cfg(test)]
    fn set_yield_state_for_test(&mut self, needs_reset: bool) {
        self.yield_elapsed_us = 0;
        self.yield_max_duty = 0;
        self.yield_needs_reset = needs_reset;
    }

    #[cfg(test)]
    fn yield_elapsed_us(&self) -> u32 {
        self.yield_elapsed_us
    }

    #[cfg(test)]
    fn set_fast_seq_for_test(&mut self, val: u32) {
        self.fast_seq = val;
    }

    #[cfg(test)]
    fn yield_max_duty(&self) -> i16 {
        self.yield_max_duty
    }

    #[cfg(test)]
    fn fast_seq(&self) -> u32 {
        self.fast_seq
    }

    #[cfg(test)]
    fn mode(&self) -> ServoMode {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{make_core, make_inputs, MockController};
    use open_servo_math::TickDomain;

    /// Create a dummy TickCtx for testing.
    fn test_ctx() -> TickCtx {
        TickCtx {
            domain: TickDomain::ControlFast,
            dt_us: 100,
            seq: 0,
        }
    }

    // ========== fast_tick tests ==========

    #[test]
    fn test_fast_tick_returns_safe_when_faulted() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(9000));

        // Manually fault the core
        core.fault_state.raise(FaultKind::OverCurrent);
        assert!(core.is_faulted());

        // fast_tick should return safe outputs
        let outputs = core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(outputs.pwm_command, Duty::ZERO);
        assert!(!outputs.motor_enable);
    }

    #[test]
    fn test_fast_tick_skips_on_bad_position() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.controller_mut().set_output(Duty::from_raw(500)); // Would produce output if position was valid
        core.engage(CentiDeg::from_cdeg(9000));

        // First tick initializes sensor health at position 9000
        core.fast_tick(&ctx, make_inputs(9000));

        // Sudden large jump should be rejected (delta > max_delta threshold)
        // Skip keeps motor enabled but outputs PWM 0 (hold position, don't actuate on bad data)
        let outputs = core.fast_tick(&ctx, make_inputs(0)); // Jump from 9000 to 0
        assert_eq!(outputs.pwm_command, Duty::ZERO);
        assert!(outputs.motor_enable); // Skip keeps motor enabled, just sends 0 PWM
    }

    #[cfg(feature = "current-sense-bus")]
    #[test]
    fn test_fast_tick_faults_on_overcurrent() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(9000));

        // Initialize with normal position
        core.fast_tick(&ctx, make_inputs(9000));

        // Trigger overcurrent (default limit is 800mA)
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            current: Some(MilliAmp::from_ma(1000)), // Over 800mA limit
            bus_voltage: None,
            temperature: None,
        };

        let outputs = core.fast_tick(&ctx, inputs);
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
        let ctx = test_ctx();
        core.controller_mut().set_output(Duty::from_raw(500));
        core.engage(CentiDeg::from_cdeg(9000));

        // Initialize position
        core.fast_tick(&ctx, make_inputs(9000));

        // Normal tick should produce output
        let outputs = core.fast_tick(&ctx, make_inputs(9000));
        assert!(outputs.motor_enable);
        assert_eq!(outputs.pwm_command, Duty::from_raw(500));
    }

    // ========== slow_tick tests ==========

    #[test]
    fn test_slow_tick_faults_on_overtemp() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(9000)); // Must engage for fast_tick to cache temperature

        // Cache high temperature via fast_tick
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: Some(CentiC::from_centi_c(9000)), // 90°C, over 80°C limit
        };
        core.fast_tick(&ctx, inputs);

        // slow_tick should detect MCU overtemp
        let fault = core.slow_tick(&ctx);
        assert_eq!(fault, Some(FaultKind::McuOverTemp));
        assert!(core.is_faulted());
    }

    #[test]
    fn test_slow_tick_no_fault_when_normal() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();

        // Cache normal temperature
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: Some(CentiC::from_centi_c(2500)), // 25°C, well under limit
        };
        core.fast_tick(&ctx, inputs);

        // slow_tick should return None
        assert_eq!(core.slow_tick(&ctx), None);
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
        let ctx = test_ctx();
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
        core.fast_tick(&ctx, inputs);
        core.fast_tick(&ctx, inputs);

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
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(-30000));
        core.set_setpoint_i32_for_test(30000);
        let inputs = make_inputs(-30000);
        let outputs = core.fast_tick(&ctx, inputs);
        assert!(outputs.motor_enable);
    }

    #[test]
    fn test_fast_tick_persists_clamped_setpoint() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(0));

        // Put an out-of-range internal setpoint
        core.set_setpoint_i32_for_test(40000);

        // Tick once to trigger clamping + persistence
        let _ = core.fast_tick(&ctx, make_inputs(0));

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
        // Regression test: actually execute check_backdrive_medium() with prev_error = i16::MIN
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
        let window_dt_us: u32 = 1000; // 1ms window

        // Execute check_backdrive_medium - this DEFINITELY executes the error_growing line
        // If saturating_abs() wasn't used, i16::MIN.abs() would overflow to -32768
        let result = core.check_backdrive_medium(velocity, error, window_dt_us);

        // Error is NOT growing:
        //   error_abs = 0
        //   prev_error_abs = 32767 (saturated from i16::MIN)
        //   0 > 32767.saturating_add(10) = false
        // opposing = false (prev_pwm_command = 0, so u_active = false)
        // Neither condition met, so result = false
        assert!(!result);
        assert_eq!(core.backdrive_elapsed_us, 0);
    }

    // ========== update_fast_dt_us tests ==========

    #[test]
    fn test_update_fast_dt_us_clamps_zero() {
        // Zero dt_us should be clamped to 1
        let mut core = make_core(MockController::new());
        core.update_fast_dt_us(0);

        // dt_us should be clamped to 1
        assert_eq!(core.fast_dt_us, 1);
    }

    // ========== Time-based policy tests ==========

    #[test]
    fn test_hold_entry_time_based() {
        // Hold mode should be entered when hold_conditions_met_us >= HOLD_ENTRY_US
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Set state so hold conditions are met
        core.setpoint_unchanged_us = SETPOINT_SETTLE_US; // >= 400ms
        core.mode = ServoMode::Move;

        // Error and velocity below thresholds
        let error: i16 = 100; // < HOLD_ENTER_ERROR_CDEG (500)
        let window_dt_us: u32 = 100_000; // 100ms per medium tick (hypothetical)

        // First call: accumulates 100ms (not enough for HOLD_ENTRY_US = 300ms)
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.mode, ServoMode::Move);
        assert_eq!(core.hold_conditions_met_us, 100_000);

        // Second call: accumulates another 100ms (200ms total, still not enough)
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.mode, ServoMode::Move);
        assert_eq!(core.hold_conditions_met_us, 200_000);

        // Third call: accumulates another 100ms (300ms total, equals HOLD_ENTRY_US)
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.mode, ServoMode::Hold);
        assert_eq!(core.hold_conditions_met_us, 0); // Reset after transition
    }

    #[test]
    fn test_backdrive_time_based() {
        // Backdrive should trigger when backdrive_elapsed_us >= BACKDRIVE_PERSIST_US (500us)
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));
        core.mode = ServoMode::Hold;
        core.prev_pwm_command = 5000; // Active PWM (must exceed U_DEADBAND = 1638)

        let velocity = DegPerSec10::from_dps10(-500); // Opposing velocity (>300 threshold)
        let error: i16 = 0;
        let window_dt_us: u32 = 200; // 200us per medium tick

        // First call: accumulates 200us (not enough for 500us threshold)
        let result = core.check_backdrive_medium(velocity, error, window_dt_us);
        assert!(!result);
        assert_eq!(core.backdrive_elapsed_us, 200);

        // Second call: accumulates another 200us (400us total, still not enough)
        let result = core.check_backdrive_medium(velocity, error, window_dt_us);
        assert!(!result);
        assert_eq!(core.backdrive_elapsed_us, 400);

        // Third call: accumulates another 200us (600us total, exceeds 500us)
        let result = core.check_backdrive_medium(velocity, error, window_dt_us);
        assert!(result);
    }

    #[test]
    fn test_setpoint_settle_time_based() {
        // setpoint_unchanged_us should accumulate via window_dt_us
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));
        core.prev_setpoint = CentiDeg::from_cdeg(9000);

        let setpoint = CentiDeg::from_cdeg(9000); // Same setpoint
        let window_dt_us: u32 = 100_000; // 100ms

        // First call: accumulates 100ms
        core.update_setpoint_tracking_medium(setpoint, window_dt_us);
        assert_eq!(core.setpoint_unchanged_us, 100_000);

        // Second call: accumulates another 100ms
        core.update_setpoint_tracking_medium(setpoint, window_dt_us);
        assert_eq!(core.setpoint_unchanged_us, 200_000);

        // Change setpoint - should reset
        let new_setpoint = CentiDeg::from_cdeg(9100);
        core.update_setpoint_tracking_medium(new_setpoint, window_dt_us);
        assert_eq!(core.setpoint_unchanged_us, 0);
    }

    // ========== Yield entry reset tests ==========

    #[test]
    fn test_yield_entry_resets_controller() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(9000));

        // Initialize position sensor
        core.fast_tick(&ctx, make_inputs(9000));

        // Clear any reset calls from engage
        core.controller_mut().reset_count = 0;
        core.controller_mut().clear_reset_flag();

        // Force into Yield mode with reset flag set
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(true);

        // First fast_tick after yield entry should call reset
        core.fast_tick(&ctx, make_inputs(9000));
        assert!(
            core.controller().reset_called,
            "controller.reset() should be called on first Yield fast_tick"
        );
        assert_eq!(
            core.controller().reset_count,
            1,
            "reset should be called exactly once"
        );

        // Clear flag to track additional calls
        core.controller_mut().clear_reset_flag();

        // Additional fast_ticks should NOT call reset again
        for _ in 0..10 {
            core.fast_tick(&ctx, make_inputs(9000));
        }
        assert!(
            !core.controller().reset_called,
            "controller.reset() should not be called on subsequent Yield ticks"
        );
        assert_eq!(
            core.controller().reset_count,
            1,
            "reset count should still be 1"
        );
    }

    #[test]
    fn test_yield_elapsed_wraps_safely() {
        // Test that yield_elapsed_us clamps to YIELD_DURATION_US and doesn't overflow
        // (replaces old tick-wrap test since timing is now time-based)
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Force into Yield mode
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(true);

        let error: i16 = 100;

        // Use a large window_dt_us to test clamping
        let window_dt_us: u32 = YIELD_DURATION_US / 2; // 100ms per tick

        // First tick: should accumulate 100ms
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), YIELD_DURATION_US / 2);
        assert_eq!(core.mode(), ServoMode::Yield);

        // Second tick: should clamp to YIELD_DURATION_US and exit
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.mode(), ServoMode::Hold);
        assert_eq!(core.yield_elapsed_us(), 0); // Reset on exit

        // Re-enter Yield and test with very large dt to ensure clamping
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(true);

        // Single tick with huge dt should clamp and exit
        let huge_dt = u32::MAX / 2;
        core.update_compliance_mode_medium(error, huge_dt);
        assert_eq!(core.mode(), ServoMode::Hold);
        assert_eq!(core.yield_elapsed_us(), 0);
    }

    #[test]
    fn test_yield_duration_time_based() {
        // Test that Yield exits after YIELD_DURATION_US (200ms) using time-based accumulation
        // Vary window_dt_us to prove independence from tick rate

        for window_dt_us in [500u32, 1000, 2000] {
            let mut core = make_core(MockController::new());
            core.engage(CentiDeg::from_cdeg(9000));

            // Force into Yield mode
            core.set_mode_for_test(ServoMode::Yield);
            core.set_yield_state_for_test(true);

            let error: i16 = 100;
            let mut accumulated_us: u32 = 0;
            let mut tick_count = 0;

            // Simulate medium ticks until we reach YIELD_DURATION_US
            while accumulated_us < YIELD_DURATION_US {
                core.update_compliance_mode_medium(error, window_dt_us);
                accumulated_us += window_dt_us;
                tick_count += 1;

                if accumulated_us < YIELD_DURATION_US {
                    assert_eq!(
                        core.mode(),
                        ServoMode::Yield,
                        "window_dt_us={}: Should remain in Yield at tick {} (accumulated {}us < {}us)",
                        window_dt_us,
                        tick_count,
                        accumulated_us,
                        YIELD_DURATION_US
                    );
                }
            }

            // Should have transitioned to Hold on the exact tick that crossed threshold
            assert_eq!(
                core.mode(),
                ServoMode::Hold,
                "window_dt_us={}: Should exit Yield to Hold when accumulated >= YIELD_DURATION_US",
                window_dt_us
            );
        }
    }

    #[test]
    fn test_yield_coast_phase_time_based() {
        // Test that coast phase ends exactly at YIELD_COAST_US (100ms)
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();
        core.engage(CentiDeg::from_cdeg(9000));

        // Initialize position sensor
        core.fast_tick(&ctx, make_inputs(9000));

        // Force into Yield mode
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(false); // needs_reset=false to isolate coast test

        let error: i16 = 100;
        let window_dt_us: u32 = 10_000; // 10ms per medium tick
        let mut accumulated_us: u32 = 0;

        // Coast phase: yield_elapsed_us < YIELD_COAST_US
        while accumulated_us < YIELD_COAST_US {
            core.update_compliance_mode_medium(error, window_dt_us);
            accumulated_us += window_dt_us;

            // yield_elapsed_us should match accumulated time (clamped)
            assert_eq!(
                core.yield_elapsed_us(),
                accumulated_us.min(YIELD_DURATION_US),
                "yield_elapsed_us should track accumulated time"
            );

            if accumulated_us < YIELD_COAST_US {
                // Still in coast phase - fast_tick should apply zero duty
                assert!(
                    core.yield_elapsed_us() < YIELD_COAST_US,
                    "Should be in coast phase at {}us < {}us",
                    core.yield_elapsed_us(),
                    YIELD_COAST_US
                );
            }
        }

        // After crossing YIELD_COAST_US, should be in alive phase
        assert!(
            core.yield_elapsed_us() >= YIELD_COAST_US,
            "Should be in alive phase at {}us >= {}us",
            core.yield_elapsed_us(),
            YIELD_COAST_US
        );
    }

    #[test]
    fn test_yield_elapsed_reset_on_entry_and_exit() {
        // Test that yield_elapsed_us is reset on entry and exit, and stays 0 in non-Yield modes
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        let error: i16 = 100;
        let window_dt_us: u32 = 10_000; // 10ms

        // Start in Move mode
        core.set_mode_for_test(ServoMode::Move);
        assert_eq!(
            core.yield_elapsed_us(),
            0,
            "yield_elapsed_us should be 0 in Move"
        );

        // Run several medium ticks in Move - yield_elapsed_us should remain 0
        for _ in 0..10 {
            core.update_compliance_mode_medium(error, window_dt_us);
            // Mode might transition to Hold due to conditions, but yield_elapsed_us should stay 0
        }
        assert_eq!(
            core.yield_elapsed_us(),
            0,
            "yield_elapsed_us should remain 0 in Move/Hold"
        );

        // Force into Yield mode (simulating backdrive detection)
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(true);
        assert_eq!(
            core.yield_elapsed_us(),
            0,
            "yield_elapsed_us should be 0 on Yield entry"
        );

        // Accumulate some time in Yield
        for _ in 0..5 {
            core.update_compliance_mode_medium(error, window_dt_us);
        }
        assert!(
            core.yield_elapsed_us() > 0,
            "yield_elapsed_us should accumulate in Yield mode"
        );

        // Run until Yield exits to Hold
        while core.mode() == ServoMode::Yield {
            core.update_compliance_mode_medium(error, window_dt_us);
        }
        assert_eq!(core.mode(), ServoMode::Hold);
        assert_eq!(
            core.yield_elapsed_us(),
            0,
            "yield_elapsed_us should be reset to 0 on Yield exit"
        );

        // Stay in Hold for several medium ticks - yield_elapsed_us should remain 0
        for _ in 0..10 {
            core.update_compliance_mode_medium(error, window_dt_us);
        }
        assert_eq!(
            core.yield_elapsed_us(),
            0,
            "yield_elapsed_us should remain 0 while in Hold"
        );
    }

    #[test]
    fn test_yield_coast_zero_duty() {
        // During coast phase (first 100ms), yield_max_duty should be 0
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Force into Yield mode
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(false);

        let error: i16 = 100;
        let window_dt_us: u32 = 10_000; // 10ms
        let mut accumulated_us: u32 = 0;

        // Coast phase: 0 to YIELD_COAST_US (100ms)
        while accumulated_us < YIELD_COAST_US {
            core.update_compliance_mode_medium(error, window_dt_us);
            accumulated_us += window_dt_us;

            if accumulated_us < YIELD_COAST_US {
                assert_eq!(
                    core.yield_max_duty(),
                    0,
                    "yield_max_duty should be 0 during coast phase at {}us",
                    accumulated_us
                );
            }
        }
    }

    #[test]
    fn test_yield_alive_limited_duty() {
        // After coast phase (100ms), yield_max_duty should be YIELD_ALIVE_DUTY_MAX
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Force into Yield mode
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(false);

        let error: i16 = 100;

        // Advance to exactly YIELD_COAST_US
        core.update_compliance_mode_medium(error, YIELD_COAST_US);
        assert_eq!(
            core.yield_max_duty(),
            YIELD_ALIVE_DUTY_MAX,
            "yield_max_duty should be YIELD_ALIVE_DUTY_MAX after coast phase"
        );

        // Continue in alive phase until near exit
        core.update_compliance_mode_medium(error, YIELD_COAST_US / 2); // 50ms more
        assert_eq!(
            core.yield_max_duty(),
            YIELD_ALIVE_DUTY_MAX,
            "yield_max_duty should remain YIELD_ALIVE_DUTY_MAX in alive phase"
        );
    }

    #[test]
    fn test_yield_max_duty_set_by_medium_tick() {
        // Test that yield_max_duty transitions exactly at YIELD_COAST_US boundary
        let mut core = make_core(MockController::new());
        core.engage(CentiDeg::from_cdeg(9000));

        // Force into Yield mode
        core.set_mode_for_test(ServoMode::Yield);
        core.set_yield_state_for_test(false);

        let error: i16 = 100;
        let window_dt_us: u32 = 25_000; // 25ms per tick

        // After 25ms: coast phase (0 < 100ms)
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), 25_000);
        assert_eq!(core.yield_max_duty(), 0);

        // After 50ms: still coast phase
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), 50_000);
        assert_eq!(core.yield_max_duty(), 0);

        // After 75ms: still coast phase
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), 75_000);
        assert_eq!(core.yield_max_duty(), 0);

        // After 100ms: transition to alive phase
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), 100_000);
        assert_eq!(
            core.yield_max_duty(),
            YIELD_ALIVE_DUTY_MAX,
            "yield_max_duty should transition to YIELD_ALIVE_DUTY_MAX exactly at YIELD_COAST_US"
        );

        // After 125ms: still alive phase
        core.update_compliance_mode_medium(error, window_dt_us);
        assert_eq!(core.yield_elapsed_us(), 125_000);
        assert_eq!(core.yield_max_duty(), YIELD_ALIVE_DUTY_MAX);
    }

    #[test]
    fn test_fast_seq_increments() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();

        // Initial fast_seq should be 0
        assert_eq!(core.fast_seq(), 0);

        // fast_seq increments on each fast_tick
        core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(core.fast_seq(), 1);

        core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(core.fast_seq(), 2);

        core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(core.fast_seq(), 3);
    }

    #[test]
    fn test_fast_seq_wraps_correctly() {
        let mut core = make_core(MockController::new());
        let ctx = test_ctx();

        // Set fast_seq near u32::MAX to test wrapping
        core.set_fast_seq_for_test(u32::MAX - 1);
        assert_eq!(core.fast_seq(), u32::MAX - 1);

        // First tick: MAX-1 -> MAX
        core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(core.fast_seq(), u32::MAX);

        // Second tick: wraps to 0
        core.fast_tick(&ctx, make_inputs(9000));
        assert_eq!(core.fast_seq(), 0);
    }

    // ========== dt/sample_count independence tests ==========

    /// Test that Yield duration is independent of window_dt_us granularity.
    /// The FSM should exit Yield when total elapsed time >= YIELD_DURATION_US,
    /// regardless of how that time is accumulated (many small ticks vs few large).
    #[test]
    fn test_yield_duration_independent_of_dt_us() {
        // Test with different window_dt_us values: 500, 1000, 2000
        for &window_dt_us in &[500u32, 1000, 2000] {
            let mut core = make_core(MockController::new());
            core.engage(CentiDeg::from_cdeg(9000));
            core.set_setpoint(CentiDeg::from_cdeg(9000));

            // Force into Yield mode
            core.set_mode_for_test(ServoMode::Yield);
            core.set_yield_state_for_test(true);

            // Calculate exact number of ticks needed
            let ticks_needed = (YIELD_DURATION_US + window_dt_us - 1) / window_dt_us;
            let error: i16 = 0;

            // Run ticks_needed - 1 ticks: should still be in Yield
            for _ in 0..(ticks_needed - 1) {
                core.update_compliance_mode_medium(error, window_dt_us);
            }
            assert_eq!(
                core.mode(),
                ServoMode::Yield,
                "Should still be in Yield before reaching threshold (window_dt_us={})",
                window_dt_us
            );

            // Final tick should exit Yield
            core.update_compliance_mode_medium(error, window_dt_us);
            assert_eq!(
                core.mode(),
                ServoMode::Hold,
                "Should transition to Hold exactly at threshold (window_dt_us={})",
                window_dt_us
            );
        }
    }

    /// Test that Yield duration is independent of sample_count (number of fast ticks
    /// accumulated per medium tick). Only total elapsed microseconds matters.
    #[test]
    fn test_yield_duration_independent_of_sample_count() {
        // Same total window_dt_us (10000us = 10ms), different implied sample counts
        // (sample_count is implicit in how window_dt_us is computed, but FSM only sees window_dt_us)
        let window_dt_us: u32 = 10_000; // 10ms per medium tick

        for trial in 0..3 {
            let mut core = make_core(MockController::new());
            core.engage(CentiDeg::from_cdeg(9000));
            core.set_setpoint(CentiDeg::from_cdeg(9000));

            // Force into Yield mode
            core.set_mode_for_test(ServoMode::Yield);
            core.set_yield_state_for_test(true);

            // Calculate ticks needed: YIELD_DURATION_US / window_dt_us, rounded up
            let ticks_needed = (YIELD_DURATION_US + window_dt_us - 1) / window_dt_us;
            let error: i16 = 0;

            // Run until just before threshold
            for _ in 0..(ticks_needed - 1) {
                core.update_compliance_mode_medium(error, window_dt_us);
            }
            assert_eq!(
                core.mode(),
                ServoMode::Yield,
                "Trial {}: Should be in Yield before threshold",
                trial
            );

            // Final tick exits
            core.update_compliance_mode_medium(error, window_dt_us);
            assert_eq!(
                core.mode(),
                ServoMode::Hold,
                "Trial {}: Should exit Yield at threshold",
                trial
            );
        }
    }

    /// Test that Hold entry timing is independent of window_dt_us granularity.
    /// Should enter Hold when total hold_conditions_met_us >= HOLD_ENTRY_US.
    #[test]
    fn test_hold_entry_independent_of_dt_us() {
        // Test with different window_dt_us values
        for &window_dt_us in &[50_000u32, 100_000, 150_000] {
            let mut core = make_core(MockController::new());
            core.engage(CentiDeg::from_cdeg(9000));
            core.set_setpoint(CentiDeg::from_cdeg(9000));

            // Set preconditions for hold_conditions to be true:
            // - setpoint_unchanged_us >= SETPOINT_SETTLE_US (400ms)
            // - error < HOLD_ENTER_ERROR_CDEG (set via error parameter)
            // - velocity < HOLD_ENTER_VEL_DPS10 (0 by default)
            core.setpoint_unchanged_us = SETPOINT_SETTLE_US;

            // Start in Move mode (default after engage)
            assert_eq!(core.mode(), ServoMode::Move);

            // Small error (within HOLD_ENTER_ERROR_CDEG) triggers hold condition accumulation
            let error: i16 = 100;

            // Calculate ticks needed for HOLD_ENTRY_US (300ms)
            let ticks_needed = (HOLD_ENTRY_US + window_dt_us - 1) / window_dt_us;

            // Run ticks_needed - 1 ticks: should still be in Move
            for _ in 0..(ticks_needed - 1) {
                core.update_compliance_mode_medium(error, window_dt_us);
            }
            assert_eq!(
                core.mode(),
                ServoMode::Move,
                "Should still be in Move before threshold (window_dt_us={})",
                window_dt_us
            );

            // Final tick should enter Hold
            core.update_compliance_mode_medium(error, window_dt_us);
            assert_eq!(
                core.mode(),
                ServoMode::Hold,
                "Should transition to Hold exactly at threshold (window_dt_us={})",
                window_dt_us
            );
        }
    }
}
