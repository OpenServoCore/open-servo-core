use crate::wire::{Bytes, CrcUmts, RawFrame, WriteBuf, WriteError};

use super::DecodeError;

/// Vendor-extension trait that lets a downstream crate add custom DXL
/// instructions without modifying `dxl-protocol`. The crate defines a unit
/// struct (the "marker"), implements [`InstructionExt`] on it to register a
/// `Variant<'_>` enum of its verbs, and binds [`Codec`](crate::Codec) over
/// that marker — `Codec::parse_one` then returns
/// [`Packet::Ext`](crate::typed::Packet::Ext) for any instruction byte that
/// `decode` claims, and `Codec::write` dispatches `Packet::Ext` back through
/// [`InstructionExt::write`].
///
/// Reserve instruction bytes in the vendor block (`0xE0..0xEF`) to avoid
/// colliding with future Robotis revisions.
pub trait InstructionExt {
    /// Enum of custom verbs. Borrows from the raw frame at lifetime `'a`,
    /// the same way standard `Packet` variants do.
    type Variant<'a>: Copy + 'a;

    /// Decode a raw frame whose instruction byte may belong to this
    /// extension.
    ///
    /// - `None` — "not my instruction byte", surfaces as
    ///   [`ParseError::BadInstruction`](crate::wire::ParseError::BadInstruction).
    /// - `Some(Ok(v))` — parsed successfully.
    /// - `Some(Err(_))` — instruction is mine but params are malformed;
    ///   surfaces as [`ParseError::BadLength`](crate::wire::ParseError::BadLength).
    fn decode<'a>(raw: RawFrame<Bytes<'a>>) -> Option<Result<Self::Variant<'a>, DecodeError>>;

    /// Serialize a variant to wire bytes. Mirrors
    /// [`wire::write_raw`](crate::wire::write_raw) — typical impls build a
    /// `RawFrame` with the extension's instruction byte and forward.
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

    fn decode<'a>(_: RawFrame<Bytes<'a>>) -> Option<Result<Self::Variant<'a>, DecodeError>> {
        None
    }

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        _: &mut W,
    ) -> Result<(), WriteError> {
        match *v {}
    }
}
