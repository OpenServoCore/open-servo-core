//! Service operations for RPC and DXL services.
//!
//! Provides transport-agnostic register operations:
//! - `read_range` / `write_range` for immediate access
//! - `stage` / `action` for staged (ACTION-deferred) writes
//! - `snapshot` for batched reads
//! - `factory_reset` / `trigger_save` for persistence control
//! - `ResetLevel` for factory reset field selection

use open_servo_hw::v2::{PersistW, ResetW};
use open_servo_shadow::{HostShadow, ShadowError, StageResult};

// Re-export ResetLevel for convenience
pub use open_servo_hw::v2::ResetLevel;

/// Snapshot errors.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SnapshotError {
    /// Output buffer too small to hold all fields.
    OutputTooSmall,
    /// Shadow table read error.
    Shadow(ShadowError),
}

impl From<ShadowError> for SnapshotError {
    fn from(e: ShadowError) -> Self {
        SnapshotError::Shadow(e)
    }
}

/// Service operations on shadow table.
///
/// Wraps any [`HostShadow`] implementor to provide RPC/DXL-style register operations.
/// All methods use the implementor's thread-safety mechanism (typically critical sections).
pub struct ServiceOps<'a, S: HostShadow> {
    shadow: &'a S,
}

impl<'a, S: HostShadow> ServiceOps<'a, S> {
    /// Create a new ServiceOps wrapper.
    pub fn new(shadow: &'a S) -> Self {
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
    pub fn stage(&self, addr: u16, data: &[u8]) -> StageResult {
        self.shadow.host_stage(addr, data)
    }

    /// Apply all staged writes (marks dirty). Returns count applied.
    pub fn action(&self) -> usize {
        self.shadow.host_action()
    }

    /// Read multiple fields into output buffer.
    ///
    /// Each field is (addr, len). Returns total bytes written.
    pub fn snapshot(&self, fields: &[(u16, u8)], out: &mut [u8]) -> Result<usize, SnapshotError> {
        let mut offset = 0usize;
        for &(addr, len) in fields {
            let len = len as usize;
            if offset + len > out.len() {
                return Err(SnapshotError::OutputTooSmall);
            }
            self.read_range(addr, &mut out[offset..offset + len])?;
            offset += len;
        }
        Ok(offset)
    }

    /// Trigger EEPROM save to flash.
    ///
    /// Signals the persist service to save all dirty EEPROM fields to flash.
    /// Uses a signal (not a channel), so multiple save requests coalesce into one.
    pub fn trigger_save<Sig: PersistW>(signal: &Sig) {
        signal.signal(());
    }

    /// Trigger factory reset.
    ///
    /// Signals the persist service to delete non-preserved EEPROM keys from
    /// flash based on the reset level, then trigger a soft reset.
    pub fn factory_reset<Sig: ResetW>(signal: &Sig, level: ResetLevel) {
        signal.signal(level);
    }
}

// Tests for ServiceOps live in open-servo-runtime where ShadowStorage is available.
