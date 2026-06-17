//! Event taxonomy emitted by [`Parser`](super::Parser).
//!
//! Flow: Sync -> Header -> Payload* -> Crc, with Resync on malformed
//! boundaries. Chunk variants carry `(offset, length)` into the parser's
//! current `feed()` input slice; the driver translates them to ring
//! slices. Chunks are contiguous on the wire and split only at end of
//! input. Sum of chunk lengths in a region equals the prior header's
//! `length`.

use crate::types::{Id, Instruction, StatusError};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// `FF FF FD 00` matched; classifier walker re-anchors here.
    Sync,
    Header(HeaderEvent),
    Payload(PayloadEvent),
    /// Good CRC verdict; bad CRC routes through [`Event::Resync`].
    Crc,
    Resync(ResyncKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum CrcResult {
    Good,
    Bad,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResyncKind {
    BadLength,
    BadCrc,
    Overflow,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HeaderEvent {
    Instruction(InstructionHeader),
    Status(StatusHeader),
}

/// `length` is register-byte count for Read-family, opaque data-byte
/// count for Write-family / `Clear` / `ControlTableBackup` / `Raw`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InstructionHeader {
    Ping {
        id: Id,
    },
    Read {
        id: Id,
        address: u16,
        length: u16,
    },
    Write {
        id: Id,
        address: u16,
        length: u16,
    },
    RegWrite {
        id: Id,
        address: u16,
        length: u16,
    },
    Action {
        id: Id,
    },
    Reboot {
        id: Id,
    },
    FactoryReset {
        id: Id,
        mode: u8,
    },
    Clear {
        id: Id,
        length: u16,
    },
    ControlTableBackup {
        id: Id,
        length: u16,
    },
    SyncRead {
        id: Id,
        address: u16,
        length: u16,
    },
    SyncWrite {
        id: Id,
        address: u16,
        length: u16,
    },
    BulkRead {
        id: Id,
    },
    BulkWrite {
        id: Id,
    },
    FastSyncRead {
        id: Id,
        address: u16,
        length: u16,
    },
    FastBulkRead {
        id: Id,
    },
    /// Non-standard instruction byte; body is opaque to the parser.
    Raw {
        id: Id,
        instr: u8,
        length: u16,
    },
}

impl InstructionHeader {
    /// `params` holds up to 4 bytes after the instruction byte (unused slots
    /// zero). Status is invalid here -- callers route it via
    /// [`HeaderEvent::Status`].
    pub fn from_wire(kind: Instruction, id: Id, wire_len: u16, params: &[u8; 4]) -> Self {
        let addr = u16::from_le_bytes([params[0], params[1]]);
        let plen = u16::from_le_bytes([params[2], params[3]]);
        let write_body = wire_len.saturating_sub(5);
        let opaque_body = wire_len.saturating_sub(3);
        use Instruction as I;
        match kind {
            I::Ping => Self::Ping { id },
            I::Read => Self::Read {
                id,
                address: addr,
                length: plen,
            },
            I::Write => Self::Write {
                id,
                address: addr,
                length: write_body,
            },
            I::RegWrite => Self::RegWrite {
                id,
                address: addr,
                length: write_body,
            },
            I::Action => Self::Action { id },
            I::Reboot => Self::Reboot { id },
            I::FactoryReset => Self::FactoryReset {
                id,
                mode: params[0],
            },
            I::Clear => Self::Clear {
                id,
                length: opaque_body,
            },
            I::ControlTableBackup => Self::ControlTableBackup {
                id,
                length: opaque_body,
            },
            I::SyncRead => Self::SyncRead {
                id,
                address: addr,
                length: plen,
            },
            I::SyncWrite => Self::SyncWrite {
                id,
                address: addr,
                length: plen,
            },
            I::BulkRead => Self::BulkRead { id },
            I::BulkWrite => Self::BulkWrite { id },
            I::FastSyncRead => Self::FastSyncRead {
                id,
                address: addr,
                length: plen,
            },
            I::FastBulkRead => Self::FastBulkRead { id },
            I::Ext(b) => Self::Raw {
                id,
                instr: b,
                length: opaque_body,
            },
            // SAFETY: the HeaderStage routes I::Status to HeaderEvent::Status
            // before this match runs, so this arm is dead in practice. Kept as
            // a defensive Raw fallback (rather than `unreachable!()`) so a
            // future regression in the routing layer can never panic the ISR
            // — motor firmware must keep running.
            I::Status => Self::Raw {
                id,
                instr: I::Status.as_u8(),
                length: opaque_body,
            },
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusHeader {
    pub id: Id,
    pub error: StatusError,
    pub length: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadEvent {
    Instruction(InstructionPayload),
    Status(StatusPayload),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InstructionPayload {
    /// `address` / `length` come from the prior `SyncRead` / `SyncWrite` /
    /// `FastSyncRead` header (shared across slots).
    SyncSlot {
        id: Id,
        index: u8,
    },
    /// `address` / `length` are per-slot (`BulkRead` / `BulkWrite` /
    /// `FastBulkRead`).
    BulkSlot {
        id: Id,
        index: u8,
        address: u16,
        length: u16,
    },
    WriteDataChunk {
        offset: u16,
        length: u16,
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StatusPayload {
    Ping {
        model: u16,
        fw_version: u8,
    },
    ReadDataChunk {
        offset: u16,
        length: u16,
    },
    /// Slot 0's `error` is hoisted from the prior `StatusHeader` on the
    /// wire; the parser normalizes it here so consumers see a uniform
    /// shape across slots.
    FastReadSlot {
        id: Id,
        index: u8,
        error: StatusError,
    },
    FastReadDataChunk {
        offset: u16,
        length: u16,
    },
}
