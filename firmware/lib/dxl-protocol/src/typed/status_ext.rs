use crate::wire::{CrcUmts, WriteBuf, WriteError};

use super::DecodeError;
use super::packet::RawStatus;

/// Vendor-extension trait for [`Status`](crate::typed::Status) — mirror of
/// [`InstructionExt`] on the reply side. Lets a downstream crate add custom
/// slave→master typed status shapes (e.g. an OSC calibration ack) without
/// modifying `dxl-protocol`.
///
/// The crate defines a unit struct marker, implements [`StatusExt`] on it with
/// its `Variant<'_>` enum, and binds [`Codec`](crate::Codec) over that marker.
/// - [`decode`](Self::decode) is called by
///   [`decode_status`](crate::decode_status) for instruction bytes outside the
///   standard set; the master-side second-stage parser dispatches custom
///   typed status flavors here.
/// - [`write`](Self::write) is called by `Codec::write_status` to serialize
///   the typed extension reply back to wire bytes.
///
/// [`InstructionExt`]: crate::typed::InstructionExt
pub trait StatusExt {
    type Variant<'a>: Copy + 'a;

    /// Decode a raw status whose preceding request instruction `instr` may
    /// belong to this extension. Mirrors
    /// [`InstructionExt::decode`](crate::typed::InstructionExt::decode):
    ///
    /// - `None` — "not my instruction byte" — caller falls through to
    ///   [`DecodeError::UnknownInstruction`].
    /// - `Some(Ok(v))` — parsed successfully.
    /// - `Some(Err(_))` — instruction is mine but params are malformed.
    fn decode<'a>(instr: u8, raw: RawStatus<'a>) -> Option<Result<Self::Variant<'a>, DecodeError>>;

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        out: &mut W,
    ) -> Result<(), WriteError>;
}

/// Default reply extension — no custom replies. `Variant<'a> = Infallible`
/// makes `Status::Ext` statically uninhabited for pure-DXL builds.
#[derive(Copy, Clone, Debug)]
pub struct NoStatusExt;

impl StatusExt for NoStatusExt {
    type Variant<'a> = core::convert::Infallible;

    fn decode<'a>(
        _instr: u8,
        _raw: RawStatus<'a>,
    ) -> Option<Result<Self::Variant<'a>, DecodeError>> {
        None
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        _: &mut W,
    ) -> Result<(), WriteError> {
        match *v {}
    }
}
