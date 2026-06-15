//! Event taxonomy emitted by [`Parser`](super::Parser).
//!
//! Flow: Sync -> Header -> Payload* -> Crc, with Resync on malformed
//! boundaries. Chunk variants carry `(offset, length)` into the parser's
//! current `feed()` input slice; the driver translates them to ring
//! slices. Chunks are contiguous on the wire and split only at end of
//! input. Sum of chunk lengths in a region equals the prior header's
//! `length`.

use crate::packet::{Id, StatusError};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Event {
    /// `FF FF FD 00` matched; classifier walker re-anchors here.
    Sync,
    Header(HeaderEvent),
    Payload(PayloadEvent),
    Crc(CrcResult),
    Resync(ResyncKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CrcResult {
    Good,
    Bad,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResyncKind {
    BadLength,
    BadCrc,
    Overflow,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HeaderEvent {
    Instruction(InstructionHeader),
    Status(StatusHeader),
}

/// `length` is register-byte count for Read-family, opaque data-byte
/// count for Write-family / `Clear` / `ControlTableBackup` / `Raw`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StatusHeader {
    pub id: Id,
    pub error: StatusError,
    pub length: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PayloadEvent {
    Instruction(InstructionPayload),
    Status(StatusPayload),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
