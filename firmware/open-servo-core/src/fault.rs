//! Fault types and state management.

/// Types of faults that can occur during servo operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultKind {
    /// Motor current exceeded threshold
    OverCurrent,
    /// MCU/PCB temperature exceeded threshold
    /// Fault action: full shutdown
    McuOverTemp,
    /// Motor winding temperature exceeded threshold (future)
    /// Fault action: derate torque/PWM
    MotorOverTemp,
    /// Driver IC temperature exceeded threshold (future)
    /// Fault action: stop driving, MCU can still log
    DriverOverTemp,
    /// Bus voltage too low
    UnderVoltage,
    /// Event queue overflow (system overloaded)
    QueuePressure,
    /// Position sensor giving invalid readings
    EncoderFault,
    /// Motor stalled (PWM saturated but no movement)
    Stall,
    /// Position error too large for too long
    PositionError,
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
}

impl Default for FaultState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_starts_ok() {
        let state = FaultState::new();
        assert!(!state.is_faulted());
        assert_eq!(state, FaultState::Ok);
    }

    #[test]
    fn test_default_is_ok() {
        let state = FaultState::default();
        assert!(!state.is_faulted());
    }

    #[test]
    fn test_raise_transitions_to_latched() {
        let mut state = FaultState::new();
        state.raise(FaultKind::OverCurrent);
        assert!(state.is_faulted());
        assert_eq!(state.fault_kind(), Some(FaultKind::OverCurrent));
    }

    #[test]
    fn test_first_fault_wins() {
        let mut state = FaultState::new();
        state.raise(FaultKind::OverCurrent);
        state.raise(FaultKind::McuOverTemp); // should be ignored
        state.raise(FaultKind::Stall); // should be ignored

        // First fault (OverCurrent) should still be latched
        assert_eq!(state.fault_kind(), Some(FaultKind::OverCurrent));
    }

    #[test]
    fn test_clear_transitions_to_ok() {
        let mut state = FaultState::new();
        state.raise(FaultKind::Stall);
        assert!(state.is_faulted());

        state.clear();
        assert!(!state.is_faulted());
        assert_eq!(state.fault_kind(), None);
    }

    #[test]
    fn test_fault_kind_returns_correct_value() {
        let mut state = FaultState::new();
        assert_eq!(state.fault_kind(), None);

        state.raise(FaultKind::PositionError);
        assert_eq!(state.fault_kind(), Some(FaultKind::PositionError));

        state.clear();
        assert_eq!(state.fault_kind(), None);
    }

    #[test]
    fn test_clear_on_ok_is_noop() {
        let mut state = FaultState::new();
        state.clear(); // should not panic or change state
        assert!(!state.is_faulted());
    }
}
