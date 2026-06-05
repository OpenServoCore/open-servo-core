use crate::wire::{CrcUmts, RawFrame, WriteBuf, WriteError, write_raw};

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

/// Typed view of a Status-instruction frame. One variant per logical response
/// shape; pairs with a known request instruction so the master can decode the
/// raw status payload, and the slave can construct typed replies.
///
/// Sync Read / Bulk Read / Read share wire bytes — separate variants are kept
/// for caller-side intent. Fast Sync / Fast Bulk Read responses are NOT
/// represented here: those are coalesced multi-slot frames, emitted by the
/// slave one [`Slot`](crate::Slot) at a time via [`write_slot`](crate::write_slot),
/// and decoded by the master via `decode_status` returning a Fast*Status that
/// exposes a `slots()` iterator.
///
/// The optional `S` parameter plugs in a vendor status extension (see
/// [`StatusExt`]); pure-DXL callers leave it at the [`NoStatusExt`] default,
/// which makes [`Status::Ext`] statically uninhabited.
#[derive(Copy, Clone, Debug)]
pub enum Status<'a, S: StatusExt = NoStatusExt> {
    Ping(PingStatus),
    Read(ReadStatus<'a>),
    SyncRead(SyncReadStatus<'a>),
    BulkRead(BulkReadStatus<'a>),
    Write(WriteStatus),
    RegWrite(RegWriteStatus),
    Action(ActionStatus),
    Reboot(RebootStatus),
    Error(ErrorStatus),
    Ext(S::Variant<'a>),
}

pub(crate) fn write_status<W: WriteBuf, CRC: CrcUmts, S: StatusExt>(
    out: &mut W,
    status: &Status<'_, S>,
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
        Status::Ext(ref v) => S::write::<W, CRC>(v, out),
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
