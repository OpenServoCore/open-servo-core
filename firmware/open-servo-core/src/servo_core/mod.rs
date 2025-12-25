//! ServoCore - Pure servo control logic.
//!
//! ServoCore is the "brain" of the servo, containing all control logic
//! without any hardware dependencies. It can be fully tested on a host
//! system by feeding it synthetic inputs.
//!
//! The core is organized as a thin orchestrator that delegates to pipeline
//! functions in fast.rs, medium.rs, and slow.rs modules.

pub mod config;
pub mod fast;
pub mod features;
pub mod internal;
pub mod medium;
pub mod runtime;
pub mod slow;

pub use config::CoreConfig;
pub use internal::CoreInternal;
pub use runtime::CoreRuntime;

use open_servo_control::ControlLoop;
use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
use open_servo_math::TickCtx;
use open_servo_math::{CentiDeg, CentiDeg32, ComplianceConfig, ThermalModel};

use crate::fault::{FaultKind, FaultState};
use crate::inputs::FastInputs;
use crate::outputs::FastOutputs;
use crate::safety::SafetyThresholds;

/// Servo operating mode for compliance behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServoMode {
    /// Actively moving to setpoint
    #[default]
    Move,
    /// Holding position with reduced force
    Hold,
    /// Yielding to external force (backdrive detected)
    Yield,
}

/// Pure servo control core - no hardware dependencies.
///
/// ServoCore is a thin orchestrator that delegates tick logic to:
/// - `fast::run_fast_tick()` - sample + observe + safety + actuate
/// - `medium::run_medium_tick()` - windowed stats + policy FSM + soft limits
/// - `slow::run_slow_tick()` - thermal model + supervisory checks
///
/// State is organized into:
/// - `config`: Writable configuration (limits, safety, compliance, thermal)
/// - `internal`: Mutable bookkeeping (safety, thermal, compliance, backdrive, trackers)
/// - `runtime`: Read-only telemetry (position, velocity, duty, etc.)
///
/// ## Usage
///
/// ```ignore
/// let core = ServoCore::new(controller, safety_config, thermal_config, move_cfg, hold_cfg);
///
/// // In control ISR:
/// let ctx = TickCtx { domain: TickDomain::ControlFast, dt_us: 100, seq: 0 };
/// let outputs = core.fast_tick(&ctx, inputs);
///
/// // In slow tick:
/// let ctx = TickCtx { domain: TickDomain::System, dt_us: 10000, seq: 0 };
/// if let Some(fault) = core.slow_tick(&ctx) {
///     // Handle fault
/// }
/// ```
pub struct ServoCore<C: ControlLoop> {
    /// Control loop algorithm (PID, cascade, etc.)
    controller: C,
    /// Latched fault state
    fault_state: FaultState,
    /// Writable configuration (organized by feature)
    config: CoreConfig,
    /// Mutable internal bookkeeping (organized by feature)
    internal: CoreInternal,
    /// Read-only telemetry snapshot
    runtime: CoreRuntime,
    /// Whether motor is engaged
    motor_engaged: bool,
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

        // Create hierarchical limits from board config
        // Sensor limits default to full range, mechanical = user = board config limits
        use crate::kinematics::{MechanicalLimits, SensorLimits, UserLimits};
        let limits = features::LimitsConfig::new(
            SensorLimits::new(0, 36000), // Full sensor range (0-360°)
            MechanicalLimits::new(
                safety_config.position_min_cdeg as i32,
                safety_config.position_max_cdeg as i32,
            ),
            UserLimits::new(
                safety_config.position_min_cdeg as i32,
                safety_config.position_max_cdeg as i32,
            ),
        )
        .unwrap_or_default(); // Fall back to default if limits invalid

        // Build CoreConfig from feature configs
        let config = CoreConfig::new(
            limits,
            features::SafetyConfig::new(thresholds),
            features::ComplianceConfig::new(move_compliance_config.clone(), hold_compliance_config),
            features::ThermalConfig::default(),
        );

        // Build CoreInternal with feature state
        let safety_state = features::SafetyState::new();
        let thermal_state = features::ThermalState::new(thermal_model, &config.thermal);
        let internal = CoreInternal::new(safety_state, thermal_state, move_compliance_config);

        Self {
            controller,
            fault_state: FaultState::new(),
            config,
            internal,
            runtime: CoreRuntime::default(),
            motor_engaged: false, // Start disengaged
        }
    }

    /// Hard real-time fast control tick (rate determined by board DT).
    ///
    /// Delegates to `fast::run_fast_tick()` which implements:
    /// - Sample accumulation (position, current)
    /// - Immediate safety latches (current threshold, stall)
    /// - Controller update
    /// - Output limit application
    #[inline]
    pub fn fast_tick(&mut self, ctx: &TickCtx, inputs: FastInputs) -> FastOutputs {
        // If motor is disengaged or faulted, return safe state
        if !self.motor_engaged || self.fault_state.is_faulted() {
            return FastOutputs::safe();
        }

        // Delegate to pipeline function
        let result = fast::run_fast_tick(
            &mut self.internal,
            &self.config,
            &mut self.runtime,
            ctx,
            inputs,
            &mut self.controller,
        );

        // Latch any fault from pipeline
        if let Some(fault) = result.fault {
            self.raise_fault(fault);
        }

        result.outputs
    }

    /// Slow monitoring tick (100Hz).
    ///
    /// Delegates to `slow::run_slow_tick()` which performs:
    /// - Thermal model physics update
    /// - MCU/motor temperature checks
    /// - Position error supervisory check
    pub fn slow_tick(&mut self, ctx: &TickCtx) -> Option<FaultKind> {
        // Delegate to pipeline function
        let result = slow::run_slow_tick(
            &mut self.internal,
            &self.config,
            &self.runtime,
            ctx,
            &mut self.controller,
        );

        // Latch any fault from pipeline
        if let Some(fault) = result.fault_kind() {
            self.raise_fault(fault);
            return Some(fault);
        }

        None
    }

    /// ControlMedium tick - runs at decimated rate from fast tick.
    ///
    /// Delegates to `medium::run_medium_tick()` which performs:
    /// - Windowed statistics aggregation
    /// - Policy FSM (Move/Hold/Yield transitions)
    /// - Compliance config switching
    pub fn control_medium_tick(&mut self, ctx: &TickCtx) {
        if !self.can_run_control_ticks() {
            return;
        }

        // Delegate to pipeline function
        medium::run_medium_tick(&mut self.internal, &self.config, ctx, &mut self.controller);
    }

    /// Raise a fault internally.
    pub fn raise_fault(&mut self, kind: FaultKind) {
        self.fault_state.raise(kind);
        self.controller.reset();
        self.internal.reset();
    }

    /// Clear fault state and reset safety monitoring.
    pub fn clear_fault(&mut self) {
        self.fault_state.clear();
        self.internal.safety.reset();
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

    /// Get the current runtime state snapshot.
    pub fn runtime(&self) -> &CoreRuntime {
        &self.runtime
    }

    /// Get reference to configuration.
    pub fn config(&self) -> &CoreConfig {
        &self.config
    }

    /// Get mutable reference to configuration.
    pub fn config_mut(&mut self) -> &mut CoreConfig {
        &mut self.config
    }

    /// Get reference to internal state (for debugging).
    pub fn internal(&self) -> &CoreInternal {
        &self.internal
    }

    /// Get reference to limits configuration.
    pub fn limits(&self) -> &features::LimitsConfig {
        &self.config.limits
    }

    /// Get mutable reference to limits configuration.
    pub fn limits_mut(&mut self) -> &mut features::LimitsConfig {
        &mut self.config.limits
    }

    // ============= Safety accessors =============

    /// Get reference to safety thresholds.
    pub fn safety_thresholds(&self) -> &SafetyThresholds {
        &self.config.safety.thresholds
    }

    /// Get mutable reference to safety thresholds.
    pub fn safety_thresholds_mut(&mut self) -> &mut SafetyThresholds {
        &mut self.config.safety.thresholds
    }

    /// Get reference to sensor health state (for debugging).
    pub fn sensor_health(&self) -> &crate::safety::SensorHealth {
        &self.internal.safety.sensor_health
    }

    /// Get estimated motor temperature in degrees Celsius.
    pub fn motor_temp_deg(&self) -> i16 {
        self.internal.thermal.model.temperature_deg()
    }

    /// Get motor temperature rise above ambient in degrees Celsius.
    pub fn motor_temp_rise_deg(&self) -> i16 {
        self.internal.thermal.model.temp_rise_deg()
    }

    /// Update fast tick timing (no-op: timing now comes from TickCtx).
    ///
    /// This method exists for backward compatibility but does nothing.
    /// Timing is passed via TickCtx.dt_us to each tick function.
    #[inline]
    pub fn update_fast_dt_us(&mut self, _dt_us: u32) {
        // No-op: timing now comes from TickCtx
    }

    /// Set the control setpoint (in centidegrees).
    ///
    /// The setpoint is clamped to hierarchical position limits.
    pub fn set_setpoint(&mut self, setpoint: CentiDeg) {
        let sp32 = CentiDeg32::from(setpoint);
        let clamped = self.config.limits.clamp_setpoint(sp32);
        self.internal.setpoint = Some(clamped);
    }

    /// Get the current setpoint.
    pub fn get_setpoint(&self) -> Option<CentiDeg> {
        self.internal.setpoint.map(|v| v.to_centi_deg_sat())
    }

    /// Clear the setpoint (for disengagement).
    pub fn clear_setpoint(&mut self) {
        self.internal.setpoint = None;
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
        if self.internal.setpoint.is_none() {
            let pos32 = CentiDeg32::from(current_position);
            let clamped = self.config.limits.clamp_setpoint(pos32);
            self.internal.setpoint = Some(clamped);
        }
    }

    /// Disengage the motor (disable control, motor will coast)
    pub fn disengage(&mut self) {
        self.motor_engaged = false;
        // Clear the setpoint so next engage will capture current position
        self.internal.setpoint = None;
        // Reset controller state (clear integrator, derivative history)
        self.controller.reset();
        // Reset internal state
        self.internal.reset();
    }

    /// Check if the motor is engaged
    pub fn is_engaged(&self) -> bool {
        self.motor_engaged
    }

    /// Get the current operating mode.
    pub fn mode(&self) -> ServoMode {
        self.internal.mode
    }

    /// Check if control ticks (medium/slow) should run.
    /// Centralized gating for controller tick methods.
    #[inline]
    fn can_run_control_ticks(&self) -> bool {
        self.motor_engaged && !self.fault_state.is_faulted()
    }
}

#[cfg(test)]
impl<C: ControlLoop> ServoCore<C> {
    /// Test helper: directly set internal setpoint to an i32 value.
    pub fn set_setpoint_i32_for_test(&mut self, value: i32) {
        self.internal.setpoint = Some(CentiDeg32::from_cdeg(value));
    }

    /// Test helper: get internal setpoint directly.
    pub fn internal_setpoint(&self) -> Option<CentiDeg32> {
        self.internal.setpoint
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_support::{make_core, make_inputs, MockController};
    use open_servo_math::{CentiC, Duty, MilliVolt, TickDomain};

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

    // ========== runtime tests ==========

    #[test]
    fn test_runtime_updated_after_tick() {
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

        let rt = core.runtime();
        assert_eq!(rt.position.as_cdeg(), 8000);
        assert_eq!(rt.pwm_duty, Duty::from_raw(750));
        #[cfg(feature = "current-sense-bus")]
        assert_eq!(rt.current, Some(MilliAmp::from_ma(200)));
        assert_eq!(rt.bus_voltage, Some(MilliVolt::from_mv(3300)));
        assert_eq!(rt.temperature, Some(CentiC::from_centi_c(3000)));
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
        let expected_max = CentiDeg32::from(core.safety_thresholds().position_max);
        assert_eq!(core.internal_setpoint(), Some(expected_max));
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
}
