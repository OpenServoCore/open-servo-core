use crate::wire::{CrcUmts, WriteBuf, WriteError};

/// Vendor-extension trait for `Reply` — mirror of [`InstructionExt`] on the
/// reply side. Lets a downstream crate add custom slave→master reply shapes
/// (e.g. an OSC calibration ack) without modifying `dxl-protocol`.
///
/// The crate defines a unit struct marker, implements [`ReplyExt`] on it
/// with its `Variant<'_>` enum, and binds [`Codec`](crate::Codec) over that
/// marker — `Codec::write_reply` then dispatches
/// [`Reply::Ext`](crate::typed::Reply::Ext) through
/// [`ReplyExt::write`].
///
/// [`InstructionExt`]: crate::typed::InstructionExt
pub trait ReplyExt {
    type Variant<'a>: Copy + 'a;

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        out: &mut W,
    ) -> Result<(), WriteError>;
}

/// Default reply extension — no custom replies. `Variant<'a> = Infallible`
/// makes `Reply::Ext` statically uninhabited for pure-DXL builds.
#[derive(Copy, Clone, Debug)]
pub struct NoReplyExt;

impl ReplyExt for NoReplyExt {
    type Variant<'a> = core::convert::Infallible;

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        _: &mut W,
    ) -> Result<(), WriteError> {
        match *v {}
    }
}
