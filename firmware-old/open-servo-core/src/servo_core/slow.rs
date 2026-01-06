//! Slow tick pipeline (100Hz supervision).
//!
//! Performs thermal model updates and supervisory safety checks
//! that don't need 10kHz rate.

use crate::fault::FaultKind;
use crate::servo_core::config::CoreConfig;
use crate::servo_core::features::{safety, thermal};
use crate::servo_core::internal::CoreInternal;
use crate::servo_core::runtime::CoreRuntime;
use open_servo_control::ControlLoop;
use open_servo_math::TickCtx;

/// Result of slow tick supervision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlowTickResult {
    /// No issues detected
    Ok,
    /// Fault detected - caller should latch
    Fault(FaultKind),
}

impl SlowTickResult {
    /// Convert to Option<FaultKind> for compatibility.
    pub fn fault_kind(&self) -> Option<FaultKind> {
        match self {
            SlowTickResult::Ok => None,
            SlowTickResult::Fault(kind) => Some(*kind),
        }
    }
}

/// Execute slow tick supervision pipeline.
///
/// Performs:
/// 1. Thermal model physics update (with dt_us)
/// 2. MCU temperature check (from cached fast tick reading)
/// 3. Motor temperature check (from thermal model)
/// 4. Position error supervisory check (uses runtime.position)
/// 5. Controller slow_tick callback
///
/// Uses CoreRuntime for supervisory checks (position, setpoint) rather than
/// internal.last_input, ensuring checks use the actual system state.
pub fn run_slow_tick<C: ControlLoop>(
    internal: &mut CoreInternal,
    config: &CoreConfig,
    runtime: &CoreRuntime,
    ctx: &TickCtx,
    controller: &mut C,
) -> SlowTickResult {
    // 1. Update thermal model physics (I² averaging + heat/cooling dynamics)
    thermal::update_slow(&mut internal.thermal, &config.thermal, ctx.dt_us);

    // 2. Check MCU temperature (using cached value from fast tick)
    if let Some(fault) =
        safety::check_mcu_temperature(&config.safety, internal.thermal.last_temperature)
    {
        return SlowTickResult::Fault(fault);
    }

    // 3. Check motor temperature (from thermal model)
    if let Some(fault) = thermal::check_motor_temp(&mut internal.thermal) {
        return SlowTickResult::Fault(fault);
    }

    // 4. Check position error (supervisory check using runtime.position)
    // Convert position_error_timeout_us to ticks using ctx.dt_us
    let timeout_ticks = if ctx.dt_us > 0 {
        config.safety.thresholds.position_error_timeout_us / ctx.dt_us
    } else {
        0
    };

    if let Some(fault) = safety::check_position_error(
        &mut internal.safety,
        &config.safety,
        runtime.setpoint,
        runtime.position,
        timeout_ticks,
    ) {
        return SlowTickResult::Fault(fault);
    }

    // 5. Call controller slow_tick (if we have a valid input)
    if let Some(ref base_input) = internal.last_input {
        // Build a snapshot-like input for the controller
        // Use runtime position and internal velocity
        let mut slow_input = base_input.clone();
        slow_input.position = runtime.position;
        slow_input.velocity = Some(internal.measured_velocity);
        controller.slow_tick(ctx, &slow_input);
    }

    SlowTickResult::Ok
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slow_tick_result_ok() {
        let result = SlowTickResult::Ok;
        assert_eq!(result.fault_kind(), None);
    }

    #[test]
    fn test_slow_tick_result_fault() {
        let result = SlowTickResult::Fault(FaultKind::McuOverTemp);
        assert_eq!(result.fault_kind(), Some(FaultKind::McuOverTemp));
    }
}
