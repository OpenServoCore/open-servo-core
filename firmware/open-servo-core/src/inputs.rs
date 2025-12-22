//! Input structures for the servo control loop.

use open_servo_math::{CentiC, CentiDeg, MilliAmp, MilliVolt};

/// Inputs for the fast (10kHz) control tick.
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
