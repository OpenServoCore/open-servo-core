use crate::wire::{CrcUmts, WriteBuf, WriteError};

/// Vendor-extension trait for `Status` — mirror of [`InstructionExt`] on the
/// reply side. Lets a downstream crate add custom slave→master reply shapes
/// (e.g. an OSC calibration ack) without modifying `dxl-protocol`.
///
/// The crate defines a unit struct marker, implements [`StatusExt`] on it
/// with its `Variant<'_>` enum, and binds [`Codec`](crate::Codec) over that
/// marker — `Codec::write_status` then dispatches
/// [`Status::Ext`](crate::typed::Status::Ext) through
/// [`StatusExt::write`].
///
/// [`InstructionExt`]: crate::typed::InstructionExt
pub trait StatusExt {
    type Variant<'a>: Copy + 'a;

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

    fn write<'a, W: WriteBuf, CRC: CrcUmts>(
        v: &Self::Variant<'a>,
        _: &mut W,
    ) -> Result<(), WriteError> {
        match *v {}
    }
}
