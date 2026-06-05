use crate::wire::{CrcUmts, RawFrame, WriteBuf, WriteError, write_raw};

use super::fast::{FastPosition, write_fast};
use super::instruction::Instruction;
use super::status_ext::{NoStatusExt, StatusExt};
use super::status_error::StatusError;

#[derive(Copy, Clone, Debug)]
pub struct PingStatus {
    pub id: u8,
    pub model: u16,
    pub firmware: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ReadStatus<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadStatus<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadStatus<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadStatus<'a> {
    pub position: FastPosition,
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadStatus<'a> {
    pub position: FastPosition,
    pub id: u8,
    pub data: &'a [u8],
}

/// Fast Read failure — chip emits `length` zero bytes for the data field so
/// the chain stays length-aligned; the error byte carries the code.
#[derive(Copy, Clone, Debug)]
pub struct FastErrorStatus {
    pub position: FastPosition,
    pub id: u8,
    pub error: StatusError,
    pub length: u16,
}

#[derive(Copy, Clone, Debug)]
pub struct WriteStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RegWriteStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ActionStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RebootStatus {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ErrorStatus {
    pub id: u8,
    pub error: StatusError,
}

/// Typed slave-side reply. One variant per logical response shape; the chip's
/// `Bus::send` matches on this to pick wire layout and fire mechanism.
///
/// Sync Read / Bulk Read / Read share wire bytes — separate variants are kept
/// for caller-side intent. Likewise FastSyncRead / FastBulkRead are wire-
/// identical; `FastError` collapses both error paths since the chip-emitted
/// zero payload is the same either way.
///
/// The optional `R` parameter plugs in a vendor reply extension (see
/// [`StatusExt`]); pure-DXL callers leave it at the [`NoStatusExt`] default,
/// which makes [`Status::Ext`] statically uninhabited.
#[derive(Copy, Clone, Debug)]
pub enum Status<'a, R: StatusExt = NoStatusExt> {
    // ── Data-bearing replies (Status-family wire shape) ──
    Ping(PingStatus),
    Read(ReadStatus<'a>),
    SyncRead(SyncReadStatus<'a>),
    BulkRead(BulkReadStatus<'a>),
    // ── Fast Read success (Sync/Bulk wire-identical) ──
    FastSyncRead(FastSyncReadStatus<'a>),
    FastBulkRead(FastBulkReadStatus<'a>),
    FastError(FastErrorStatus),
    // ── Empty-payload acks (Status-family) ──
    Write(WriteStatus),
    RegWrite(RegWriteStatus),
    Action(ActionStatus),
    Reboot(RebootStatus),
    // ── Empty-payload error reply (Status-family) ──
    Error(ErrorStatus),
    Ext(R::Variant<'a>),
}

pub(crate) fn write_status<W: WriteBuf, CRC: CrcUmts, R: StatusExt>(
    out: &mut W,
    status: &Status<'_, R>,
) -> Result<(), WriteError> {
    match *status {
        Status::Ping(PingStatus {
            id,
            model,
            firmware,
        }) => {
            let m = model.to_le_bytes();
            let payload = [m[0], m[1], firmware];
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, payload.iter().copied())
        }
        Status::Read(ReadStatus { id, data })
        | Status::SyncRead(SyncReadStatus { id, data })
        | Status::BulkRead(BulkReadStatus { id, data }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, data.iter().copied())
        }
        Status::Write(WriteStatus { id })
        | Status::RegWrite(RegWriteStatus { id })
        | Status::Action(ActionStatus { id })
        | Status::Reboot(RebootStatus { id }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, core::iter::empty())
        }
        Status::Error(ErrorStatus { id, error }) => {
            write_status_frame::<W, _, CRC>(out, id, error, core::iter::empty())
        }
        Status::FastSyncRead(FastSyncReadStatus { position, id, data })
        | Status::FastBulkRead(FastBulkReadStatus { position, id, data }) => write_fast::<W, _, CRC>(
            out,
            position,
            id,
            StatusError::None,
            &mut data.iter().copied(),
        ),
        Status::FastError(FastErrorStatus {
            position,
            id,
            error,
            length,
        }) => write_fast::<W, _, CRC>(
            out,
            position,
            id,
            error,
            &mut core::iter::repeat_n(0u8, length as usize),
        ),
        Status::Ext(ref v) => R::write::<W, CRC>(v, out),
    }
}

fn write_status_frame<W: WriteBuf, I: IntoIterator<Item = u8>, CRC: CrcUmts>(
    out: &mut W,
    id: u8,
    error: StatusError,
    payload: I,
) -> Result<(), WriteError> {
    write_raw::<W, _, CRC>(
        out,
        RawFrame {
            id,
            instruction: Instruction::Status.as_u8(),
            params: core::iter::once(error.as_u8()).chain(payload),
        },
    )
}
