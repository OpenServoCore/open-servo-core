use crate::wire::{Bytes, CrcUmts, WriteBuf, WriteError};

use super::DecodeError;
use super::packet::RawStatus;

/// Vendor-extension trait that lets a downstream crate add custom DXL
/// instructions without modifying `dxl-protocol`. The crate defines a unit
/// struct (the "marker"), implements [`InstructionExt`] on it to register a
/// `Variant<'_>` enum of its verbs, and binds parse/write helpers over that
/// marker — [`parse_packet`](crate::parse_packet) returns
/// [`Packet::Ext`](crate::Packet::Ext) for any instruction byte that `decode`
/// claims, and [`write_packet`](crate::write_packet) dispatches `Packet::Ext`
/// back through [`InstructionExt::write`].
///
/// Reserve instruction bytes in the vendor block (`0xE0..0xEF`) to avoid
/// colliding with future Robotis revisions.
pub trait InstructionExt {
    /// Enum of custom verbs. Borrows from the raw frame at lifetime `'a`,
    /// the same way standard `Packet` variants do.
    type Variant<'a>: Copy + 'a;

    /// Decode a raw frame whose instruction byte may belong to this
    /// extension. `instr` is the wire instruction byte; `id` is the frame's
    /// ID; `params` is the (unstuffed) parameter region.
    ///
    /// - `None` — "not my instruction byte", surfaces as
    ///   [`ParseError::BadInstruction`](crate::ParseError::BadInstruction).
    /// - `Some(Ok(v))` — parsed successfully.
    /// - `Some(Err(_))` — instruction is mine but params are malformed;
    ///   surfaces as [`ParseError::BadLength`](crate::ParseError::BadLength).
    fn decode<'a>(
        instr: u8,
        id: u8,
        params: Bytes<'a>,
    ) -> Option<Result<Self::Variant<'a>, DecodeError>>;

    /// Serialize a variant to wire bytes — typical impls call
    /// [`write_ext`](crate::write_ext) with the extension's instruction byte
    /// and a params iterator.
    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        out: &mut W,
    ) -> Result<(), WriteError>;
}

/// Default extension — no custom verbs. `Variant<'a> = Infallible` makes the
/// `Packet::Ext` arm statically uninhabited, so pure-DXL callers don't need
/// to write a match arm for it (use `Packet::Ext(v) => match v {}` if the
/// compiler requires the arm to be listed for exhaustiveness).
#[derive(Copy, Clone, Debug)]
pub struct NoInstructionExt;

impl InstructionExt for NoInstructionExt {
    type Variant<'a> = core::convert::Infallible;

    fn decode<'a>(
        _instr: u8,
        _id: u8,
        _params: Bytes<'a>,
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

/// Vendor-extension trait for [`Status`](crate::Status) — mirror of
/// [`InstructionExt`] on the reply side. Lets a downstream crate add custom
/// slave→master typed status shapes (e.g. an OSC calibration ack) without
/// modifying `dxl-protocol`.
///
/// - [`decode`](Self::decode) is called by [`decode_status`](crate::decode_status)
///   for instruction bytes outside the standard set; the master-side
///   second-stage parser dispatches custom typed status flavors here.
/// - [`write`](Self::write) is called by [`write_status`](crate::write_status)
///   to serialize the typed extension reply back to wire bytes.
pub trait StatusExt {
    type Variant<'a>: Copy + 'a;

    /// Decode a raw status whose preceding request instruction `instr` may
    /// belong to this extension. Mirrors [`InstructionExt::decode`]:
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
