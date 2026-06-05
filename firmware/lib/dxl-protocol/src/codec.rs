use core::marker::PhantomData;

use crate::typed::{self, DecodeError, InstructionExt, NoInstructionExt, NoStatusExt, Packet, Reply, StatusExt};
use crate::wire::{self, CrcUmts, ParseError, RxView, WriteBuf, WriteError};

/// Typed namespace for the protocol's parse/write entry points. Bind it once
/// per crate (`type Codec = dxl_protocol::Codec<SoftwareCrcUmts>;`) and the
/// `CRC` parameter disappears from every call site.
///
/// `X` plugs in a request-side vendor extension (see [`InstructionExt`]); `R`
/// plugs in a reply-side extension (see [`StatusExt`]). Both default to the
/// no-extension sentinels, so pure-DXL bindings stay `Codec<CRC>`.
pub struct Codec<CRC, X = NoInstructionExt, R = NoStatusExt>(PhantomData<(CRC, X, R)>);

impl<CRC: CrcUmts, X: InstructionExt, R: StatusExt> Codec<CRC, X, R> {
    /// Parse the first DXL frame in `head` then `tail`, treated as one
    /// logically contiguous byte sequence (so a ring-buffer wrap can be
    /// passed without copying). Non-ring callers pass `tail = &[]`.
    ///
    /// Instruction bytes outside the standard set are routed to
    /// [`InstructionExt::decode`]: `None` surfaces as
    /// [`ParseError::BadInstruction`], `Some(Err)` as [`ParseError::BadLength`].
    pub fn parse_one<'a>(
        head: &'a [u8],
        tail: &'a [u8],
    ) -> Result<(Packet<'a, X>, usize), ParseError> {
        let (raw, consumed) = wire::parse_raw::<CRC>(RxView::ring(head, tail))?;
        match typed::decode::<X>(raw) {
            Ok(packet) => Ok((packet, consumed)),
            Err(DecodeError::UnknownInstruction) => match X::decode(raw) {
                Some(Ok(v)) => Ok((Packet::Ext(v), consumed)),
                Some(Err(_)) => Err(ParseError::BadLength { skip: consumed }),
                None => Err(ParseError::BadInstruction { skip: consumed }),
            },
            Err(DecodeError::BadParams) => Err(ParseError::BadLength { skip: consumed }),
        }
    }

    /// Encode any `Packet` to wire bytes. Primarily used by test fixtures to
    /// construct inbound frames; production code on the slave side emits
    /// replies via `write_reply` instead.
    pub fn write<W: WriteBuf>(out: &mut W, packet: &Packet<'_, X>) -> Result<(), WriteError> {
        typed::write::<W, CRC, X>(out, packet)
    }

    pub fn write_reply<W: WriteBuf>(out: &mut W, reply: &Reply<'_, R>) -> Result<(), WriteError> {
        typed::write_reply::<W, CRC, R>(out, reply)
    }
}
