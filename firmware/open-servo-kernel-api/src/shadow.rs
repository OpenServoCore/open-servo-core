//! Shadow table for host-visible register/telemetry access.
//!
//! This module re-exports the shadow table mechanism from `open_servo_shadow`
//! and defines the kernel commit contract.

// Re-export all shadow mechanism types
pub use open_servo_shadow::*;

// ============================================================================
// Kernel Commit Contract
// ============================================================================

/// Result of a shadow→live commit attempt.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommitResult {
    /// Commit succeeded, state updated.
    Ok,
    /// No dirty data to commit.
    NothingToCommit,
    /// Commit refused due to system state (e.g., engaged, busy).
    Busy,
    /// Validation failed for a field at the given offset.
    ///
    /// Dirty bit for this field remains set so host can fix and retry.
    ValidationError {
        /// Offset of the invalid field.
        offset: u16,
    },
}

/// Field descriptor for commit processing.
///
/// Used by kernels to define their field layout for commit logic.
#[derive(Copy, Clone, Debug)]
pub struct FieldDesc {
    /// Offset in shadow table.
    pub offset: u16,
    /// Field length in bytes.
    pub len: u8,
}

impl FieldDesc {
    /// Create a new field descriptor.
    pub const fn new(offset: u16, len: u8) -> Self {
        Self { offset, len }
    }

    /// Check if this field overlaps a dirty range.
    ///
    /// Used to determine if a field needs to be committed.
    pub const fn overlaps(&self, dirty_start: u16, dirty_len: u16) -> bool {
        if dirty_len == 0 || self.len == 0 {
            return false;
        }
        let field_end = self.offset.saturating_add(self.len as u16);
        let dirty_end = dirty_start.saturating_add(dirty_len);
        // Overlap if: field_start < dirty_end && dirty_start < field_end
        self.offset < dirty_end && dirty_start < field_end
    }
}

/// Kernel commit contract.
///
/// Kernels implement this to:
/// 1. Check which dirty ranges overlap known fields via `view.is_range_dirty()`
/// 2. Read shadow bytes via `view.read_ctrl()`
/// 3. Validate and apply to live kernel config/state
/// 4. Clear dirty bits via `view.clear_range_dirty()` for each successfully committed field
///
/// # Error Handling
///
/// On validation error:
/// - Do NOT blindly clear all dirty bits
/// - Clear only fields that were successfully applied
/// - Return `CommitResult::ValidationError { offset }` with dirty bits intact
/// - Host can fix the invalid value and retry
pub trait ShadowKernel {
    /// Publish telemetry data to shadow table.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    /// Writes kernel state to telemetry region via `view.write_telem()`.
    ///
    /// # Note
    ///
    /// Telemetry writes do NOT mark dirty bits. This is read-only data for the host.
    fn publish_telemetry(&self, view: &mut KernelView<'_>);

    /// Commit dirty shadow data to live kernel state.
    ///
    /// Called from ISR context at a safe tick boundary (e.g., ControlMedium).
    fn commit_shadow(&mut self, view: &mut KernelView<'_>) -> CommitResult;
}
