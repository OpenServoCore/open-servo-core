use core::marker::PhantomData;

use crate::buf::WriteBuf;
use crate::crc::CrcUmts;
use crate::packet::Packet;
use crate::parser::ParseError;
use crate::reply::StatusReply;
use crate::writer::WriteError;
use crate::{parser, reply, writer};

/// Typed namespace for the protocol's parse/write entry points. Bind it once
/// per crate (`type Codec = dxl_protocol::Codec<SoftwareCrcUmts>;`) and the
/// `CRC` parameter disappears from every call site.
pub struct Codec<CRC>(PhantomData<CRC>);

impl<CRC: CrcUmts> Codec<CRC> {
    pub fn parse_one(input: &[u8]) -> Result<(Packet<'_>, usize), ParseError> {
        parser::parse_one::<CRC>(input)
    }

    /// Encode any `Packet` to wire bytes. Primarily used by test fixtures to
    /// construct inbound frames; production code on the slave side emits
    /// replies via `write_status_reply` instead.
    pub fn write<W: WriteBuf>(out: &mut W, packet: &Packet<'_>) -> Result<(), WriteError> {
        writer::write::<W, CRC>(out, packet)
    }

    pub fn write_status_reply<W: WriteBuf>(
        out: &mut W,
        reply: &StatusReply<'_>,
    ) -> Result<(), WriteError> {
        reply::write_status_reply::<W, CRC>(out, reply)
    }
}
