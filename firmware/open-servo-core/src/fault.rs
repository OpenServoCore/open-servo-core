//! Fault types and state management.

use open_servo_hw::BdcMotorDriver;

/// Types of faults that can occur during servo operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultKind {
    /// Motor current exceeded threshold
    OverCurrent,
    /// Temperature exceeded threshold
    OverTemp,
    /// Bus voltage too low
    UnderVoltage,
    /// Event queue overflow (system overloaded)
    QueuePressure,
    /// Position sensor giving invalid readings
    EncoderFault,
}

/// Fault state - either OK or latched with a specific fault kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultState {
    /// No fault active
    Ok,
    /// Fault latched - first fault wins
    Latched(FaultKind),
}

impl FaultState {
    /// Create a new fault state (no fault).
    pub fn new() -> Self {
        FaultState::Ok
    }

    /// Check if a fault is currently active.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        !matches!(self, FaultState::Ok)
    }

    /// Get the latched fault kind, if any.
    pub fn fault_kind(&self) -> Option<FaultKind> {
        match self {
            FaultState::Ok => None,
            FaultState::Latched(kind) => Some(*kind),
        }
    }

    /// Raise a fault (first fault wins - subsequent faults ignored).
    pub fn raise(&mut self, kind: FaultKind) {
        if matches!(self, FaultState::Ok) {
            *self = FaultState::Latched(kind);
            #[cfg(feature = "defmt")]
            defmt::error!("Fault raised: {:?}", kind);
        }
    }

    /// Clear the fault state.
    pub fn clear(&mut self) {
        if self.is_faulted() {
            #[cfg(feature = "defmt")]
            defmt::info!("Fault cleared");
            *self = FaultState::Ok;
        }
    }

    /// Apply safety response - disable motor immediately.
    ///
    /// Called when a fault is raised to ensure safe state.
    pub fn apply_safety<H: BdcMotorDriver>(&self, hw: &mut H) {
        if self.is_faulted() {
            hw.set_pwm(0);
            hw.set_enable(false);
        }
    }
}

impl Default for FaultState {
    fn default() -> Self {
        Self::new()
    }
}
