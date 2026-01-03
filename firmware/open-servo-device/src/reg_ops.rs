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
}
