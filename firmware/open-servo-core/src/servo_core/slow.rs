//! Slow tick pipeline (100Hz supervision).
//!
//! Performs thermal model updates and supervisory safety checks
//! that don't need 10kHz rate.

use crate::fault::FaultKind;

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

// Note: run_slow_tick will be implemented in Commit 4 when we wire up
// to CoreConfig/CoreInternal. The slow tick checks:
// 1. MCU temperature (from cached fast tick reading)
// 2. Motor temperature (from thermal model)
// 3. Position error (supervisory check)

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
