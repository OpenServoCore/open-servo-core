use core::marker::PhantomData;

use crate::typed::{self, DecodeError, Packet, StatusReply};
use crate::wire::{self, CrcUmts, ParseError, RxView, WriteBuf, WriteError};

/// Typed namespace for the protocol's parse/write entry points. Bind it once
/// per crate (`type Codec = dxl_protocol::Codec<SoftwareCrcUmts>;`) and the
/// `CRC` parameter disappears from every call site.
pub struct Codec<CRC>(PhantomData<CRC>);

impl<CRC: CrcUmts> Codec<CRC> {
    /// Parse the first DXL frame in `head` then `tail`, treated as one
    /// logically contiguous byte sequence (so a ring-buffer wrap can be
    /// passed without copying). Non-ring callers pass `tail = &[]`.
    pub fn parse_one<'a>(
        head: &'a [u8],
        tail: &'a [u8],
    ) -> Result<(Packet<'a>, usize), ParseError> {
        let (raw, consumed) = wire::parse_raw::<CRC>(RxView::ring(head, tail))?;
        let packet = typed::decode(raw).map_err(|e| match e {
            DecodeError::UnknownInstruction => ParseError::BadInstruction { skip: consumed },
            DecodeError::BadParams => ParseError::BadLength { skip: consumed },
        })?;
        Ok((packet, consumed))
    }

    /// Encode any `Packet` to wire bytes. Primarily used by test fixtures to
    /// construct inbound frames; production code on the slave side emits
    /// replies via `write_status_reply` instead.
    pub fn write<W: WriteBuf>(out: &mut W, packet: &Packet<'_>) -> Result<(), WriteError> {
        typed::write::<W, CRC>(out, packet)
    }

    pub fn write_status_reply<W: WriteBuf>(
        out: &mut W,
        reply: &StatusReply<'_>,
    ) -> Result<(), WriteError> {
        typed::write_status_reply::<W, CRC>(out, reply)
    }
}
