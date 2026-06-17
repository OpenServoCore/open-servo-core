//! Slave-originated reply shapes -- encoder inputs.

use super::id::Id;
use super::status::StatusError;

/// Ping reply payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PingStatus {
    pub model: u16,
    pub fw_version: u8,
}

/// One slave's contribution to a Fast Sync/Bulk Read reply chain. Used by
/// [`SlotEncoder`] for chain emission and as the per-slot view a master
/// reconstructs from streaming events.
///
/// [`SlotEncoder`]: crate::SlotEncoder
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Slot<'a> {
    pub id: Id,
    pub error: StatusError,
    pub data: &'a [u8],
}

/// Position of one slot in a coalesced Fast Status response. Header-emitting
/// variants carry the DXL `Length` of the whole multi-slot frame; `Last`
/// carries the chain CRC.
///
/// Chain producers don't know the running CRC at build time (it depends on
/// every prior slave's wire bytes), so they pass a placeholder and let the
/// fire-time ISR overwrite the trailing two bytes. Callers that *do* know
/// the CRC (single-slot tests, replay tools, sniffers) pass the real value.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SlotPosition {
    /// Single-slot response -- emits full header and locally-computed CRC.
    Only { packet_length: u16 },
    /// First of N -- emits header + body, no CRC (successors continue).
    First { packet_length: u16 },
    /// Body only.
    Middle,
    /// Body + caller-supplied CRC bytes.
    Last { crc: u16 },
}

/// Unified slave-originated reply. Encoder input for [`StatusEncoder::emit`].
///
/// [`StatusEncoder::emit`]: crate::StatusEncoder::emit
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Status<'a> {
    /// Write-style ack -- no payload. Covers Write/RegWrite/Action/Reboot/
    /// FactoryReset/SyncWrite/BulkWrite replies and the short-payload-Ping
    /// fallback.
    Empty { id: Id, error: StatusError },

    Ping {
        id: Id,
        error: StatusError,
        status: PingStatus,
    },

    /// Read/SyncRead/BulkRead reply -- opaque register bytes.
    Read {
        id: Id,
        error: StatusError,
        data: &'a [u8],
    },

    /// Fast Sync Read coalesced reply. Slaves participating in a chain emit
    /// one slot at a time via [`SlotEncoder`] instead; this variant exists for
    /// relays, sniffers, or single-slave-with-all-data masters.
    ///
    /// [`SlotEncoder`]: crate::SlotEncoder
    FastSyncRead {
        id: Id,
        error: StatusError,
        payload: &'a [u8],
    },

    FastBulkRead {
        id: Id,
        error: StatusError,
        payload: &'a [u8],
    },

    /// Encode-only escape hatch -- arbitrary chip-defined payload (e.g. OSC
    /// `Calibrate` reply).
    Raw {
        id: Id,
        error: StatusError,
        payload: &'a [u8],
    },
}
