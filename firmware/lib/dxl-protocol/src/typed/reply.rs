use crate::wire::{CrcUmts, RawFrame, WriteBuf, WriteError, write_raw};

use super::fast::{FastPosition, write_fast};
use super::instruction::Instruction;
use super::reply_ext::{NoReplyExt, ReplyExt};
use super::status_error::StatusError;

#[derive(Copy, Clone, Debug)]
pub struct PingReply {
    pub id: u8,
    pub model: u16,
    pub firmware: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ReadReply<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct SyncReadReply<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct BulkReadReply<'a> {
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastSyncReadReply<'a> {
    pub position: FastPosition,
    pub id: u8,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug)]
pub struct FastBulkReadReply<'a> {
    pub position: FastPosition,
    pub id: u8,
    pub data: &'a [u8],
}

/// Fast Read failure — chip emits `length` zero bytes for the data field so
/// the chain stays length-aligned; the error byte carries the code.
#[derive(Copy, Clone, Debug)]
pub struct FastErrorReply {
    pub position: FastPosition,
    pub id: u8,
    pub error: StatusError,
    pub length: u16,
}

#[derive(Copy, Clone, Debug)]
pub struct WriteReply {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RegWriteReply {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ActionReply {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct RebootReply {
    pub id: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct ErrorReply {
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
/// [`ReplyExt`]); pure-DXL callers leave it at the [`NoReplyExt`] default,
/// which makes [`Reply::Ext`] statically uninhabited.
#[derive(Copy, Clone, Debug)]
pub enum Reply<'a, R: ReplyExt = NoReplyExt> {
    // ── Data-bearing replies (Status-family wire shape) ──
    Ping(PingReply),
    Read(ReadReply<'a>),
    SyncRead(SyncReadReply<'a>),
    BulkRead(BulkReadReply<'a>),
    // ── Fast Read success (Sync/Bulk wire-identical) ──
    FastSyncRead(FastSyncReadReply<'a>),
    FastBulkRead(FastBulkReadReply<'a>),
    FastError(FastErrorReply),
    // ── Empty-payload acks (Status-family) ──
    Write(WriteReply),
    RegWrite(RegWriteReply),
    Action(ActionReply),
    Reboot(RebootReply),
    // ── Empty-payload error reply (Status-family) ──
    Error(ErrorReply),
    Ext(R::Variant<'a>),
}

pub(crate) fn write_reply<W: WriteBuf, CRC: CrcUmts, R: ReplyExt>(
    out: &mut W,
    reply: &Reply<'_, R>,
) -> Result<(), WriteError> {
    match *reply {
        Reply::Ping(PingReply {
            id,
            model,
            firmware,
        }) => {
            let m = model.to_le_bytes();
            let payload = [m[0], m[1], firmware];
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, payload.iter().copied())
        }
        Reply::Read(ReadReply { id, data })
        | Reply::SyncRead(SyncReadReply { id, data })
        | Reply::BulkRead(BulkReadReply { id, data }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, data.iter().copied())
        }
        Reply::Write(WriteReply { id })
        | Reply::RegWrite(RegWriteReply { id })
        | Reply::Action(ActionReply { id })
        | Reply::Reboot(RebootReply { id }) => {
            write_status_frame::<W, _, CRC>(out, id, StatusError::None, core::iter::empty())
        }
        Reply::Error(ErrorReply { id, error }) => {
            write_status_frame::<W, _, CRC>(out, id, error, core::iter::empty())
        }
        Reply::FastSyncRead(FastSyncReadReply { position, id, data })
        | Reply::FastBulkRead(FastBulkReadReply { position, id, data }) => write_fast::<W, _, CRC>(
            out,
            position,
            id,
            StatusError::None,
            &mut data.iter().copied(),
        ),
        Reply::FastError(FastErrorReply {
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
        Reply::Ext(ref v) => R::write::<W, CRC>(v, out),
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
