//! Shadow table commit types for kernel-api.
//!
//! This module defines the commit contract types. The actual shadow table
//! mechanism and `ShadowKernel` trait live in `open_servo_shadow`.

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
