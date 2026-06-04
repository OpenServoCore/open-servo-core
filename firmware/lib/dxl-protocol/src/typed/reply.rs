use crate::wire::{CrcUmts, WriteBuf, WriteError};

use super::fast::{FastPosition, write_fast};
use super::instruction::Instruction;
use super::status_error::StatusError;
use super::writer::write_packet;

/// Typed slave-side reply. One variant per logical response shape; the chip's
/// `Bus::send` matches on this to pick wire layout and fire mechanism.
///
/// Sync Read / Bulk Read / Read share wire bytes — separate variants are kept
/// for caller-side intent. Likewise FastSyncRead / FastBulkRead are wire-
/// identical; `FastError` collapses both error paths since the chip-emitted
/// zero payload is the same either way.
#[derive(Copy, Clone, Debug)]
pub enum StatusReply<'a> {
    // ── Data-bearing replies (Status-family wire shape) ──
    Ping {
        id: u8,
        model: u16,
        firmware: u8,
    },
    Read {
        id: u8,
        data: &'a [u8],
    },
    SyncRead {
        id: u8,
        data: &'a [u8],
    },
    BulkRead {
        id: u8,
        data: &'a [u8],
    },
    /// Chip emits `zeros_count` zero bytes — no caller-side scratch needed.
    Calibrate {
        id: u8,
        zeros_count: u16,
    },

    // ── Fast Read success (Sync/Bulk wire-identical) ──
    FastSyncRead {
        position: FastPosition,
        id: u8,
        data: &'a [u8],
    },
    FastBulkRead {
        position: FastPosition,
        id: u8,
        data: &'a [u8],
    },

    /// Fast Read failure — chip emits `length` zero bytes for the data field
    /// so chain stays length-aligned; the error byte carries the code.
    FastError {
        position: FastPosition,
        id: u8,
        error: StatusError,
        length: u16,
    },

    // ── Empty-payload acks (Status-family) ──
    Write {
        id: u8,
    },
    RegWrite {
        id: u8,
    },
    Action {
        id: u8,
    },
    Reboot {
        id: u8,
    },

    // ── Empty-payload error reply (Status-family) ──
    Error {
        id: u8,
        error: StatusError,
    },
}

pub(crate) fn write_status_reply<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    reply: &StatusReply<'_>,
) -> Result<(), WriteError> {
    match *reply {
        StatusReply::Ping {
            id,
            model,
            firmware,
        } => {
            let m = model.to_le_bytes();
            let payload = [m[0], m[1], firmware];
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, payload.iter().copied())
        }
        StatusReply::Read { id, data }
        | StatusReply::SyncRead { id, data }
        | StatusReply::BulkRead { id, data } => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, data.iter().copied())
        }
        StatusReply::Calibrate { id, zeros_count } => write_status_frame::<W, _, CRC>(
            out,
            id,
            StatusError::None,
            core::iter::repeat_n(0u8, zeros_count as usize),
        ),
        StatusReply::Write { id }
        | StatusReply::RegWrite { id }
        | StatusReply::Action { id }
        | StatusReply::Reboot { id } => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, core::iter::empty())
        }
        StatusReply::Error { id, error } => {
            write_status_frame::<W, _, CRC>(out, id, error, core::iter::empty())
        }
        StatusReply::FastSyncRead { position, id, data }
        | StatusReply::FastBulkRead { position, id, data } => write_fast::<W, _, CRC>(
            out,
            position,
            id,
            StatusError::None,
            &mut data.iter().copied(),
        ),
        StatusReply::FastError {
            position,
            id,
            error,
            length,
        } => write_fast::<W, _, CRC>(
            out,
            position,
            id,
            error,
            &mut core::iter::repeat_n(0u8, length as usize),
        ),
    }
}

fn write_status_frame<W: WriteBuf, I: Iterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    id: u8,
    error: StatusError,
    payload: I,
) -> Result<(), WriteError> {
    let mut params = core::iter::once(error.as_u8()).chain(payload);
    write_packet::<W, _, CRC>(out, id, Instruction::Status, &mut params)
}
