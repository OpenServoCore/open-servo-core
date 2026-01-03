//! Shared register byte-ops API for RPC and future DXL.
//!
//! Provides transport-agnostic register operations:
//! - `read_range` / `write_range` for immediate access
//! - `reg_write_range` / `action` for staged (ACTION-deferred) writes

use crate::ShadowStorage;
use open_servo_kernel_api::shadow::{ShadowError, StageResult};

/// Shared register byte-ops API.
///
/// Wraps `ShadowStorage` to provide RPC/DXL-style register operations.
/// All methods use critical-section-protected host access.
pub struct RegOps<'a, const N: usize> {
    shadow: &'a ShadowStorage<N>,
}

impl<'a, const N: usize> RegOps<'a, N> {
    /// Create a new RegOps wrapper.
    pub fn new(shadow: &'a ShadowStorage<N>) -> Self {
        Self { shadow }
    }

    /// Read bytes from shadow table.
    pub fn read_range(&self, addr: u16, buf: &mut [u8]) -> Result<(), ShadowError> {
        self.shadow.host_read(addr, buf)
    }

    /// Write bytes immediately (marks dirty).
    pub fn write_range(&self, addr: u16, data: &[u8]) -> Result<(), ShadowError> {
        self.shadow.host_write(addr, data)
    }

    /// Stage write for later ACTION (does NOT mark dirty).
    pub fn reg_write_range(&self, addr: u16, data: &[u8]) -> StageResult {
        self.shadow
            .host_with_view(|view| view.stage_write_range(addr, data))
    }

    /// Apply all staged writes (marks dirty). Returns count applied.
    pub fn action(&self) -> usize {
        self.shadow.host_with_view(|view| view.action())
    }
}

// ============================================================================
// StreamPlan
// ============================================================================

/// Stream snapshot errors.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StreamError {
    /// Output buffer too small to hold all fields.
    OutputTooSmall,
    /// Shadow table read error.
    Shadow(ShadowError),
}

impl From<ShadowError> for StreamError {
    fn from(e: ShadowError) -> Self {
        StreamError::Shadow(e)
    }
}

/// Transport-agnostic streaming read plan.
///
/// Stores a list of (addr, len) pairs to read.
/// Interval/timing is owned by the transport layer.
pub struct StreamPlan<const MAX: usize> {
    /// Fields to read: (address, length).
    pub fields: heapless::Vec<(u16, u8), MAX>,
}

impl<const MAX: usize> StreamPlan<MAX> {
    /// Create an empty stream plan.
    pub const fn new() -> Self {
        Self {
            fields: heapless::Vec::new(),
        }
    }

    /// Read all fields into output buffer.
    ///
    /// Returns total bytes written on success.
    /// Returns `OutputTooSmall` if output buffer is smaller than `total_len()`.
    /// Returns `Shadow(err)` if any read fails.
    pub fn snapshot<const N: usize>(
        &self,
        ops: &RegOps<'_, N>,
        out: &mut [u8],
    ) -> Result<usize, StreamError> {
        let mut offset = 0usize;
        for &(addr, len) in &self.fields {
            let len = len as usize;
            if offset + len > out.len() {
                return Err(StreamError::OutputTooSmall);
            }
            ops.read_range(addr, &mut out[offset..offset + len])?;
            offset += len;
        }
        Ok(offset)
    }

    /// Total bytes that snapshot() will produce.
    pub fn total_len(&self) -> usize {
        self.fields.iter().map(|(_, len)| *len as usize).sum()
    }
}

impl<const MAX: usize> Default for StreamPlan<MAX> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_range_marks_dirty() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        // Initially not dirty
        assert!(!storage.kernel_any_dirty());

        // Write range marks dirty
        ops.write_range(0x50, &[1, 2, 3, 4]).unwrap();
        assert!(storage.kernel_any_dirty());

        // Verify data written
        let mut buf = [0u8; 4];
        ops.read_range(0x50, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn test_reg_write_range_no_dirty() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        // Initially not dirty
        assert!(!storage.kernel_any_dirty());

        // Staging alone does NOT mark dirty
        assert_eq!(ops.reg_write_range(0x60, &[5, 6]), StageResult::Ok);
        assert!(!storage.kernel_any_dirty());

        // Data not yet applied
        let mut buf = [0u8; 2];
        ops.read_range(0x60, &mut buf).unwrap();
        assert_eq!(buf, [0, 0]); // Still zeros
    }

    #[test]
    fn test_action_marks_dirty_and_applies() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        // Stage multiple writes
        assert_eq!(ops.reg_write_range(0x70, &[10, 11]), StageResult::Ok);
        assert_eq!(ops.reg_write_range(0x74, &[12, 13, 14]), StageResult::Ok);
        assert!(!storage.kernel_any_dirty());

        // ACTION applies staged writes and marks dirty
        let count = ops.action();
        assert_eq!(count, 2);
        assert!(storage.kernel_any_dirty());

        // Data now applied
        let mut buf = [0u8; 2];
        ops.read_range(0x70, &mut buf).unwrap();
        assert_eq!(buf, [10, 11]);

        let mut buf = [0u8; 3];
        ops.read_range(0x74, &mut buf).unwrap();
        assert_eq!(buf, [12, 13, 14]);
    }

    #[test]
    fn test_action_returns_zero_when_nothing_staged() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        // No staged writes
        let count = ops.action();
        assert_eq!(count, 0);
        assert!(!storage.kernel_any_dirty());
    }

    // ========================================================================
    // StreamPlan tests
    // ========================================================================

    #[test]
    fn test_stream_plan_snapshot_two_fields() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        // Write some test data
        ops.write_range(0x10, &[1, 2]).unwrap();
        ops.write_range(0x20, &[3, 4, 5]).unwrap();

        // Create plan
        let mut plan = StreamPlan::<4>::new();
        plan.fields.push((0x10, 2)).unwrap();
        plan.fields.push((0x20, 3)).unwrap();

        // Snapshot
        let mut out = [0u8; 10];
        let len = plan.snapshot(&ops, &mut out).unwrap();
        assert_eq!(len, 5);
        assert_eq!(&out[..5], &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_stream_plan_snapshot_buffer_too_small() {
        let storage = ShadowStorage::<1024>::new();
        let ops = RegOps::new(&storage);

        let mut plan = StreamPlan::<4>::new();
        plan.fields.push((0x10, 4)).unwrap();
        plan.fields.push((0x20, 4)).unwrap();

        // Buffer only 6 bytes, need 8
        let mut out = [0u8; 6];
        let result = plan.snapshot(&ops, &mut out);
        assert_eq!(result, Err(StreamError::OutputTooSmall));
    }

    #[test]
    fn test_stream_plan_total_len() {
        let mut plan = StreamPlan::<4>::new();
        plan.fields.push((0x10, 2)).unwrap();
        plan.fields.push((0x20, 4)).unwrap();
        assert_eq!(plan.total_len(), 6);
    }
}
