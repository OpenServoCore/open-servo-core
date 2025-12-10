//! Output structures from the servo control loop.

use crate::fault::FaultKind;

/// Outputs from the fast (10kHz) control tick.
///
/// Tells the App how to actuate the motor and whether a fault occurred.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FastOutputs {
    /// PWM duty cycle command (-max to +max).
    pub pwm_command: i32,

    /// Whether the motor should be enabled.
    pub motor_enable: bool,

    /// Fault raised during this tick, if any.
    pub fault: Option<FaultKind>,
}

impl FastOutputs {
    /// Create outputs for normal operation.
    pub fn normal(pwm_command: i32) -> Self {
        Self {
            pwm_command,
            motor_enable: true,
            fault: None,
        }
    }

    /// Create safe outputs - motor disabled, zero PWM.
    ///
    /// Used when faulted or during transient sensor issues.
    pub fn safe() -> Self {
        Self {
            pwm_command: 0,
            motor_enable: false,
            fault: None,
        }
    }

    /// Create safe outputs with a fault indication.
    pub fn fault(kind: FaultKind) -> Self {
        Self {
            pwm_command: 0,
            motor_enable: false,
            fault: Some(kind),
        }
    }

    /// Create outputs for skipping actuation (bad sensor reading).
    ///
    /// Motor stays enabled but no PWM command issued.
    pub fn skip() -> Self {
        Self {
            pwm_command: 0,
            motor_enable: true,
            fault: None,
        }
    }
}
