//! Fast tick pipeline (10kHz hard real-time).
//!
//! Pipeline stages: observe → protect → control → actuate
//!
//! Returns `(FastOutputs, Option<FaultKind>)` - caller latches faults.
//! No `?` operator - faults are returned explicitly.

use crate::fault::FaultKind;
use crate::inputs::FastInputs;
use open_servo_math::{CentiDeg, Duty, MilliAmp};

/// Data observed from fast tick inputs.
#[derive(Debug, Clone)]
pub struct FastObservation {
    /// Raw position reading
    pub position_raw: CentiDeg,
    /// Current reading (if available)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage (if available)
    pub bus_voltage: Option<open_servo_math::MilliVolt>,
    /// Temperature (if available)
    pub temperature: Option<open_servo_math::CentiC>,
}

/// State after protection checks pass.
#[derive(Debug, Clone)]
pub struct ProtectedState {
    /// Validated position
    pub position: CentiDeg,
    /// Clamped setpoint
    pub setpoint: CentiDeg,
    /// Error (setpoint - position) as i16
    pub error: i16,
    /// Current reading (if available)
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,
    /// Bus voltage
    pub bus_voltage: Option<open_servo_math::MilliVolt>,
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
}

/// Output from control stage.
#[derive(Debug, Clone)]
pub struct ControlOutput {
    /// Commanded duty cycle
    pub duty: Duty,
    /// Whether output was saturated against limits
    pub saturated: bool,
}

/// Observe stage - gather data from inputs.
///
/// Called first in the fast tick pipeline.
pub fn observe_fast(inputs: &FastInputs) -> FastObservation {
    FastObservation {
        position_raw: inputs.position,
        #[cfg(feature = "current-sense-bus")]
        current: inputs.current,
        bus_voltage: inputs.bus_voltage,
        temperature: inputs.temperature,
    }
}

// Note: protect_fast, control_fast, actuate_fast, and run_fast_tick
// will be implemented in Commit 4 when we wire up to CoreConfig/CoreInternal.
// For now, we define the types and observe_fast as the foundation.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observe_fast() {
        let inputs = FastInputs {
            position: CentiDeg::from_cdeg(9000),
            #[cfg(feature = "current-sense-bus")]
            current: Some(MilliAmp::from_ma(500)),
            bus_voltage: Some(open_servo_math::MilliVolt::from_mv(12000)),
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
            setpoint: CentiDeg::from_cdeg(9000),
            error: 0,
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: None,
        });
        let _fault = ProtectResult::Fault(FaultKind::OverCurrent);
        let _skip = ProtectResult::Skip;
    }
}
