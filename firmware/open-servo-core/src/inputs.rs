//! Input structures for the servo control loop.

use open_servo_math::{CentiDeg, DeciC, MilliAmp, MilliVolt};

/// Inputs for the fast (10kHz) control tick.
///
/// Optional fields are `None` when the board doesn't have that sensor.
/// SafetyManager skips corresponding checks for `None` values.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastInputs {
    /// Position feedback (required - always need position for control).
    pub position: CentiDeg,

    /// Bus current reading. `None` = no over-current protection.
    pub current: Option<MilliAmp>,

    /// Bus voltage reading. `None` = no under-voltage protection.
    pub bus_voltage: Option<MilliVolt>,

    /// Temperature reading. `None` = no over-temp protection.
    pub temperature: Option<DeciC>,
}

impl FastInputs {
    /// Create new inputs with all sensors.
    pub fn new(
        position: CentiDeg,
        current: Option<MilliAmp>,
        bus_voltage: Option<MilliVolt>,
        temperature: Option<DeciC>,
    ) -> Self {
        Self {
            position,
            current,
            bus_voltage,
            temperature,
        }
    }

    /// Create inputs with position only (minimal board).
    pub fn position_only(position: CentiDeg) -> Self {
        Self {
            position,
            current: None,
            bus_voltage: None,
            temperature: None,
        }
    }
}
