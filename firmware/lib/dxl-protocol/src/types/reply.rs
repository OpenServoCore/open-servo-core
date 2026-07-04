//! Slave-originated reply shapes -- encoder inputs.

use super::id::Id;
use super::status::StatusError;

/// Ping reply payload.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Slot<'a> {
    pub id: Id,
    pub error: StatusError,
    pub data: &'a [u8],
}

/// Position of one slot in a coalesced Fast Status response.
///
/// Per the official Fast Sync/Bulk Read layout, EVERY slot's block ends
/// with a CRC — the cumulative packet CRC from the header through that
/// slot's data (the e-manual: "CRC values are used for internal
/// calculation in DYNAMIXEL to confirm packet integrity between
/// DYNAMIXELs"). That leaves exactly two wire shapes: slot 0 emits the
/// Status header and can compute its CRC locally (every covered byte is
/// its own output); every later slot appends a block whose CRC depends on
/// prior slaves' wire bytes, so chain producers pass a placeholder and
/// let the fire-time ISR overwrite the trailing two bytes. Callers that
/// *do* know the chain CRC (tests, replay tools, sniffers) pass the real
/// value.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlotPosition {
    /// Slot 0 — Status header + own block + locally-computed CRC.
    /// `packet_length` is the DXL `Length` of the whole chain frame
    /// (degenerates to the single-block value when the chain is one slot).
    First { packet_length: u16 },
    /// Slot k > 0 — block only (`err, id, data, crc`), appended
    /// mid-packet with a caller-supplied cumulative CRC.
    Successor { crc: u16 },
}

/// Unified slave-originated reply. Encoder input for [`StatusEncoder::emit`].
///
/// [`StatusEncoder::emit`]: crate::StatusEncoder::emit
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
