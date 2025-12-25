//! Input structures for the servo control loop.

use crate::accumulator::MediumSnapshot;
use open_servo_control::ControlInput;
use open_servo_math::{CentiC, CentiDeg, MilliAmp, MilliVolt};

/// Inputs for the ControlFast tick.
///
/// Optional fields are `None` when the board doesn't have that sensor.
/// SafetyManager skips corresponding checks for `None` values.
///
/// The `current` field is only available when the `current-sense` feature is enabled.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastInputs {
    /// Position feedback (required - always need position for control).
    pub position: CentiDeg,

    /// Bus current reading (requires `current-sense` feature).
    #[cfg(feature = "current-sense-bus")]
    pub current: Option<MilliAmp>,

    /// Bus voltage reading. `None` = no under-voltage protection.
    pub bus_voltage: Option<MilliVolt>,

    /// Temperature reading. `None` = no over-temp protection.
    pub temperature: Option<CentiC>,
}

impl FastInputs {
    /// Create new inputs with all available sensors.
    #[cfg(feature = "current-sense-bus")]
    pub fn new(
        position: CentiDeg,
        current: Option<MilliAmp>,
        bus_voltage: Option<MilliVolt>,
        temperature: Option<CentiC>,
    ) -> Self {
        Self {
            position,
            current,
            bus_voltage,
            temperature,
        }
    }

    /// Create new inputs (without current sensing).
    #[cfg(not(feature = "current-sense-bus"))]
    pub fn new(
        position: CentiDeg,
        bus_voltage: Option<MilliVolt>,
        temperature: Option<CentiC>,
    ) -> Self {
        Self {
            position,
            bus_voltage,
            temperature,
        }
    }

    /// Create inputs with position only (minimal board).
    pub fn position_only(position: CentiDeg) -> Self {
        Self {
            position,
            #[cfg(feature = "current-sense-bus")]
            current: None,
            bus_voltage: None,
            temperature: None,
        }
    }

    /// Get current reading if available.
    ///
    /// Returns `None` if:
    /// - The `current-sense` feature is disabled
    /// - The field is `None` (no reading available this tick)
    ///
    /// This method always exists, avoiding `#[cfg]` at call sites.
    #[inline]
    pub fn current(&self) -> Option<MilliAmp> {
        #[cfg(feature = "current-sense-bus")]
        {
            self.current
        }
        #[cfg(not(feature = "current-sense-bus"))]
        {
            None
        }
    }
}

/// Build a ControlInput for medium-tick processing from last fast-tick input and snapshot.
///
/// Merges windowed sensor data from `MediumSnapshot` (position, velocity, current_avg)
/// with metadata from `last_input` (setpoint, limits, bus_voltage).
///
/// This is a pure function with no side effects.
pub fn build_medium_control_input(
    last_input: &ControlInput,
    snap: &MediumSnapshot,
) -> ControlInput {
    ControlInput {
        setpoint: last_input.setpoint,
        position: snap.pos_last,
        velocity: Some(snap.velocity_dps10),
        current: snap.current_avg,
        bus_voltage: last_input.bus_voltage,
        limits: last_input.limits,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use open_servo_control::DutyLimits;
    use open_servo_math::{DegPerSec10, Duty};

    #[test]
    fn test_build_medium_control_input() {
        let last_input = ControlInput {
            setpoint: CentiDeg::from_cdeg(9000),
            position: CentiDeg::from_cdeg(8500), // Will be overwritten by snapshot
            velocity: None,                      // Will be overwritten by snapshot
            current: None,                       // Will be overwritten by snapshot
            bus_voltage: Some(MilliVolt::from_mv(12000)),
            limits: DutyLimits::new(Duty::from_raw(-5000), Duty::from_raw(5000)),
        };

        let snap = MediumSnapshot {
            velocity_dps10: DegPerSec10::from_dps10(150),
            current_avg: Some(MilliAmp::from_ma(300)),
            current_peak_abs: Some(MilliAmp::from_ma(400)),
            pos_last: CentiDeg::from_cdeg(8600),
            sample_count: 10,
            window_dt_us: 1000,
        };

        let result = build_medium_control_input(&last_input, &snap);

        // From last_input (metadata)
        assert_eq!(result.setpoint.as_cdeg(), 9000);
        assert_eq!(result.bus_voltage.unwrap().as_mv(), 12000);
        assert_eq!(result.limits.min.as_raw(), -5000);
        assert_eq!(result.limits.max.as_raw(), 5000);

        // From snapshot (sensor data)
        assert_eq!(result.position.as_cdeg(), 8600);
        assert_eq!(result.velocity.unwrap().as_dps10(), 150);
        assert_eq!(result.current.unwrap().as_ma(), 300);
    }
}
