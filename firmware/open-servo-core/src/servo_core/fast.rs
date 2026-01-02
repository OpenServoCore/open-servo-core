//! Fast tick pipeline (10kHz hard real-time).
//!
//! Pipeline stages: observe → protect → control → actuate
//!
//! Returns `FastTickResult` with outputs and optional fault.
//! Caller latches faults into FaultState.

use crate::fault::FaultKind;
use crate::inputs::FastInputs;
use crate::outputs::FastOutputs;
use crate::servo_core::config::CoreConfig;
use crate::servo_core::features::{compliance, safety, thermal};
use crate::servo_core::internal::CoreInternal;
use crate::servo_core::runtime::CoreRuntime;
use crate::servo_core::ServoMode;
use open_servo_control::{ControlInput, ControlLoop, EffortLimits};
use open_servo_math::{CentiDeg, CentiDeg32, Effort, MilliVolt, TickCtx};

/// Data observed from fast tick inputs.
#[derive(Debug, Clone)]
pub struct FastObservation {
    /// Raw position reading
    pub position_raw: CentiDeg,
    /// Current reading (if available)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage (if available)
    pub bus_voltage: Option<MilliVolt>,
    /// Temperature (if available)
    pub temperature: Option<open_servo_math::CentiC>,
}

/// State after protection checks pass.
#[derive(Debug, Clone)]
pub struct ProtectedState {
    /// Validated position
    pub position: CentiDeg,
    /// Clamped setpoint (as i32)
    pub setpoint32: CentiDeg32,
    /// Clamped setpoint (as i16)
    pub setpoint: CentiDeg,
    /// Error (setpoint - position) as i16
    pub error: i16,
    /// Current reading (if available)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage
    pub bus_voltage: Option<MilliVolt>,
    /// Temperature
    pub temperature: Option<open_servo_math::CentiC>,
}

/// Result of protection stage.
pub enum ProtectResult {
    /// Protection passed, proceed with control
    Ok(ProtectedState),
    /// Protection failed, abort with fault
    Fault(FaultKind),
    /// Skip this tick (bad sensor reading but not yet faulted)
    Skip,
    /// No setpoint set, return safe outputs
    NoSetpoint,
}

/// Result of fast tick pipeline.
pub struct FastTickResult {
    /// PWM outputs
    pub outputs: FastOutputs,
    /// Fault if one was detected (caller should latch)
    pub fault: Option<FaultKind>,
}

impl FastTickResult {
    /// Create result with fault.
    pub fn fault(kind: FaultKind) -> Self {
        Self {
            outputs: FastOutputs::fault(kind),
            fault: Some(kind),
        }
    }

    /// Create skip result (no actuation).
    pub fn skip() -> Self {
        Self {
            outputs: FastOutputs::skip(),
            fault: None,
        }
    }

    /// Create safe result (motor disabled).
    pub fn safe() -> Self {
        Self {
            outputs: FastOutputs::safe(),
            fault: None,
        }
    }

    /// Create normal result with outputs.
    pub fn normal(outputs: FastOutputs) -> Self {
        Self {
            outputs,
            fault: None,
        }
    }
}

/// Observe stage - gather data from inputs.
pub fn observe_fast(inputs: &FastInputs) -> FastObservation {
    FastObservation {
        position_raw: inputs.position,
        #[cfg(feature = "current-sense-bus")]
        current: inputs.current,
        bus_voltage: inputs.bus_voltage,
        temperature: inputs.temperature,
    }
}

/// Protection stage - validate sensors, check limits, clamp setpoint.
///
/// Returns ProtectedState if all checks pass.
pub fn protect_fast(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    obs: &FastObservation,
) -> ProtectResult {
    // Validate position sensor
    let position =
        match safety::check_position(&mut internal.safety, &config.safety, obs.position_raw) {
            Ok(pos) => pos,
            Err(_fault) => {
                if safety::is_sensor_fault(&internal.safety, &config.safety) {
                    return ProtectResult::Fault(FaultKind::EncoderFault);
                }
                // Use last good position but skip this tick
                return ProtectResult::Skip;
            }
        };

    // Check current threshold
    #[cfg(feature = "current-sense-bus")]
    if let Some(fault) = safety::check_current(&config.safety, obs.current) {
        return ProtectResult::Fault(fault);
    }

    // Get setpoint - if None, no target set
    let Some(sp32) = internal.setpoint else {
        return ProtectResult::NoSetpoint;
    };

    // Clamp setpoint to hierarchical limits
    let clamped32 = config.limits.clamp_setpoint(sp32);
    // Store effective clamped setpoint
    internal.setpoint = Some(clamped32);

    // Convert to i16 for APIs
    let clamped_setpoint = clamped32.to_centi_deg_sat();

    // Compute error (i32 math prevents overflow)
    let pv = CentiDeg32::from(position);
    let err32 = clamped32 - pv;
    let error = err32.to_cdeg_i16_sat();

    ProtectResult::Ok(ProtectedState {
        position,
        setpoint32: clamped32,
        setpoint: clamped_setpoint,
        error,
        #[cfg(feature = "current-sense-bus")]
        current: obs.current,
        bus_voltage: obs.bus_voltage,
        temperature: obs.temperature,
    })
}

/// Control stage - run controller with limits.
///
/// Returns the ControlOutput from the controller.
pub fn control_fast<C: ControlLoop>(
    controller: &mut C,
    config: &CoreConfig,
    internal: &mut CoreInternal,
    protected: &ProtectedState,
    ctx: &TickCtx,
) -> open_servo_control::ControlOutput {
    // Update compliance limiter with current reading
    compliance::update_limiter(
        &mut internal.compliance,
        #[cfg(feature = "current-sense-bus")]
        protected.current,
        #[cfg(not(feature = "current-sense-bus"))]
        None,
        protected.error,
        ctx.dt_us,
    );

    // Get base limits from compliance limiter
    let (mut min_effort, mut max_effort) = compliance::get_limits(&internal.compliance);

    // Apply mode-specific limits
    match internal.mode {
        ServoMode::Move => {
            // Use full torque limiter limits
        }

        ServoMode::Hold => {
            // Apply error-based effort cap curve
            let effort_cap = compliance::compute_hold_effort_cap(
                protected.error.saturating_abs(),
                &config.policy,
            );
            min_effort = min_effort.max(-effort_cap as i32);
            max_effort = max_effort.min(effort_cap as i32);
        }

        ServoMode::Yield => {
            // Use yield_max_effort set by medium tick
            min_effort = -(internal.backdrive.max_effort as i32);
            max_effort = internal.backdrive.max_effort as i32;

            // Reset controller on YIELD entry (once)
            if internal.backdrive.needs_reset {
                controller.reset();
                internal.backdrive.needs_reset = false;
            }
        }
    }

    // Build ControlInput
    let input = ControlInput {
        setpoint: protected.setpoint,
        position: protected.position,
        velocity: Some(internal.measured_velocity),
        #[cfg(feature = "current-sense-bus")]
        current: protected.current,
        #[cfg(not(feature = "current-sense-bus"))]
        current: None,
        bus_voltage: protected.bus_voltage,
        limits: EffortLimits {
            min: Effort::from_raw(min_effort.clamp(i16::MIN as i32, i16::MAX as i32) as i16),
            max: Effort::from_raw(max_effort.clamp(i16::MIN as i32, i16::MAX as i32) as i16),
        },
    };

    // Cache input for medium/slow tick
    internal.last_input = Some(input.clone());

    // Run controller
    controller.fast_tick(ctx, &input)
}

/// Actuate stage - check for stall, update runtime.
///
/// Mutates runtime in place.
pub fn actuate_fast(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    runtime: &mut CoreRuntime,
    protected: &ProtectedState,
    ctrl_output: &open_servo_control::ControlOutput,
) -> FastTickResult {
    // Update tracking for backdrive detection
    internal.backdrive_detector.prev_effort = ctrl_output.effort.as_raw();
    internal.backdrive_detector.prev_error = protected.error;

    // Check for stall
    if let Some(fault) = safety::check_stall(
        &mut internal.safety,
        &config.safety,
        protected.position,
        ctrl_output.saturated,
    ) {
        return FastTickResult::fault(fault);
    }

    // Update runtime in place
    runtime.setpoint = protected.setpoint;
    runtime.position = protected.position;
    runtime.effort = ctrl_output.effort;
    #[cfg(feature = "current-sense-bus")]
    {
        runtime.current = protected.current;
    }
    runtime.bus_voltage = protected.bus_voltage;
    runtime.temperature = protected.temperature;
    runtime.mode = internal.mode;
    runtime.velocity = internal.measured_velocity;
    runtime.compliance_limited = internal.compliance.is_limited();
    runtime.fast_seq = internal.fast_seq;
    runtime.engaged = true; // We only get here if engaged

    FastTickResult::normal(FastOutputs::normal(ctrl_output.effort))
}

/// Execute complete fast tick pipeline.
///
/// Pipeline: observe → protect → control → actuate
///
/// Mutates `runtime` in place. Returns outputs and optional fault.
pub fn run_fast_tick<C: ControlLoop>(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    runtime: &mut CoreRuntime,
    ctx: &TickCtx,
    inputs: FastInputs,
    controller: &mut C,
) -> FastTickResult {
    // Increment sequence counter
    internal.fast_seq = internal.fast_seq.wrapping_add(1);

    // Observe
    let obs = observe_fast(&inputs);

    // Cache temperature for slow tick
    internal.thermal.update_temperature(obs.temperature);

    // Accumulate I² for thermal model
    thermal::accumulate_i_squared(&mut internal.thermal, inputs.current());

    // Accumulate for medium-tick windowed stats
    internal
        .fast_accum
        .observe(obs.position_raw, inputs.current());

    // Protect
    let protected = match protect_fast(internal, config, &obs) {
        ProtectResult::Ok(state) => state,
        ProtectResult::Fault(kind) => return FastTickResult::fault(kind),
        ProtectResult::Skip => return FastTickResult::skip(),
        ProtectResult::NoSetpoint => return FastTickResult::safe(),
    };

    // Control
    let ctrl_output = control_fast(controller, config, internal, &protected, ctx);

    // Actuate (updates runtime in place)
    actuate_fast(internal, config, runtime, &protected, &ctrl_output)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observe_fast() {
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: Some(MilliAmp::from_ma(500)),
            bus_voltage: Some(MilliVolt::from_mv(12000)),
            temperature: Some(open_servo_math::CentiC::from_centi_c(2500)),
        };

        let obs = observe_fast(&inputs);
        assert_eq!(obs.position_raw.as_cdeg(), 9000);
        #[cfg(feature = "current-sense-bus")]
        assert_eq!(obs.current.unwrap().as_ma(), 500);
    }

    #[test]
    fn test_protect_result_variants() {
        // Just verify the enum compiles correctly
        let _ok = ProtectResult::Ok(ProtectedState {
            position: CentiDeg::from_cdeg(9000),
            setpoint32: CentiDeg32::from_cdeg(9000),
            setpoint: CentiDeg::from_cdeg(9000),
            error: 0,
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: None,
        });
        let _fault = ProtectResult::Fault(FaultKind::OverCurrent);
        let _skip = ProtectResult::Skip;
        let _no_sp = ProtectResult::NoSetpoint;
    }

    #[test]
    fn test_fast_tick_result_variants() {
        let fault_result = FastTickResult::fault(FaultKind::Stall);
        assert!(fault_result.fault.is_some());

        let skip_result = FastTickResult::skip();
        assert!(skip_result.fault.is_none());

        let safe_result = FastTickResult::safe();
        assert!(safe_result.fault.is_none());
    }
}
