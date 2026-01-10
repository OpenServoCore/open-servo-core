//! Service operations for RPC and DXL services.
//!
//! Provides transport-agnostic register operations:
//! - `read_range` / `write_range` for immediate access
//! - `snapshot` for batched reads
//! - `factory_reset` / `trigger_save` for persistence control
//! - `ResetLevel` for factory reset field selection
//!
//! ## Note on Staging
//!
//! Stage/action operations for Dynamixel ACTION instruction are handled
//! separately via embedded-shadow's PatchStagingBuffer at the firmware level.

use embedded_shadow::ShadowError;
use open_servo_hw::v2::{PersistW, ResetW};

// Re-export ResetLevel for convenience
pub use open_servo_hw::v2::ResetLevel;

/// Trait for host-side shadow table operations.
///
/// This trait abstracts over the concrete shadow storage type,
/// allowing services to work with any compatible implementation.
pub trait HostOps {
    /// Read bytes from shadow table.
    fn host_read(&self, addr: u16, buf: &mut [u8]) -> Result<(), ShadowError>;

    /// Write bytes immediately (marks dirty, may trigger persist).
    fn host_write(&self, addr: u16, data: &[u8]) -> Result<(), ShadowError>;
}

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
/// Wraps any [`HostOps`] implementor to provide RPC/DXL-style register operations.
/// All methods use the implementor's thread-safety mechanism (typically critical sections).
pub struct ServiceOps<'a, S: HostOps> {
    shadow: &'a S,
}

impl<'a, S: HostOps> ServiceOps<'a, S> {
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
