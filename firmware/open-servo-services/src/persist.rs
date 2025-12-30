//! Persistence service task.
//!
//! This embassy task handles saving configuration to non-volatile storage
//! (EEPROM or flash). It is triggered by [`ServiceOp::PersistRequest`].
//!
//! ## Architecture
//!
//! ```text
//! dxl_req_task → Signal<PersistRequest> → persist_task → EEPROM/Flash
//! ```
//!
//! ## Safety
//!
//! Persistence is only allowed when the servo is disengaged (torque off).
//! The task checks this condition before starting a write operation.
//!
//! ## Write Strategy
//!
//! The shadow table's EEPROM region is compared against stored values.
//! Only changed bytes are written to minimize wear.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Signal type for persist request (req task → persist task).
pub type PersistSignal = Signal<CriticalSectionRawMutex, ()>;

/// Persist result.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PersistResult {
    /// Successfully persisted.
    Ok,
    /// Refused because servo is engaged.
    Busy,
    /// Write failed (hardware error).
    WriteFailed,
}

/// Persistence task state.
///
/// This is a stub - the actual EEPROM/flash driver will be added later.
pub struct PersistTask {
    /// Last persist result.
    last_result: Option<PersistResult>,
}

impl PersistTask {
    /// Create a new persist task.
    pub const fn new() -> Self {
        Self { last_result: None }
    }

    /// Get the last persist result.
    pub const fn last_result(&self) -> Option<PersistResult> {
        self.last_result
    }
}

impl Default for PersistTask {
    fn default() -> Self {
        Self::new()
    }
}

// TODO: Add embassy task function.
// The task will:
// 1. Wait for PersistSignal
// 2. Check if servo is disengaged (via shadow table read)
// 3. If engaged: set last_result = Busy, continue
// 4. Read EEPROM region from shadow table
// 5. Compare with stored values
// 6. Write changed bytes to EEPROM/flash
// 7. Set last_result = Ok or WriteFailed
// 8. Loop
