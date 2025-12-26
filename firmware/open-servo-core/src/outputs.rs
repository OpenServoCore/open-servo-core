//! Output structures from the servo control loop.

use crate::fault::FaultKind;
use open_servo_math::Effort;

/// Outputs from the ControlFast tick.
///
/// Tells the App how to actuate the motor and whether a fault occurred.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastOutputs {
    /// Motor effort command (normalized).
    pub effort: Effort,

    /// Whether the motor should be enabled.
    pub motor_enable: bool,

    /// Fault raised during this tick, if any.
    pub fault: Option<FaultKind>,
}

impl FastOutputs {
    /// Create outputs for normal operation.
    pub fn normal(effort: Effort) -> Self {
        Self {
            effort,
            motor_enable: true,
            fault: None,
        }
    }

    /// Create safe outputs - motor disabled, zero effort.
    ///
    /// Used when faulted or during transient sensor issues.
    pub fn safe() -> Self {
        Self {
            effort: Effort::ZERO,
            motor_enable: false,
            fault: None,
        }
    }

    /// Create safe outputs with a fault indication.
    pub fn fault(kind: FaultKind) -> Self {
        Self {
            effort: Effort::ZERO,
            motor_enable: false,
            fault: Some(kind),
        }
    }

    /// Create outputs for skipping actuation (bad sensor reading).
    ///
    /// Motor stays enabled but no effort command issued.
    pub fn skip() -> Self {
        Self {
            effort: Effort::ZERO,
            motor_enable: true,
            fault: None,
        }
    }
}
