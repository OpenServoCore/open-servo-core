use core::marker::PhantomData;

use crate::buf::WriteBuf;
use crate::crc::CrcUmts;
use crate::fast::FastSlot;
use crate::packet::Packet;
use crate::parser::ParseError;
use crate::writer::WriteError;
use crate::{fast, parser, writer};

/// Typed namespace for the protocol's parse/write entry points. Bind it once
/// per crate (`type Codec = dxl_protocol::Codec<SoftwareCrcUmts>;`) and the
/// `CRC` parameter disappears from every call site.
pub struct Codec<CRC>(PhantomData<CRC>);

impl<CRC: CrcUmts> Codec<CRC> {
    pub fn parse_one(input: &[u8]) -> Result<(Packet<'_>, usize), ParseError> {
        parser::parse_one::<CRC>(input)
    }

    pub fn write<W: WriteBuf>(out: &mut W, packet: &Packet<'_>) -> Result<(), WriteError> {
        writer::write::<W, CRC>(out, packet)
    }

    pub fn write_fast_slot<W: WriteBuf>(
        out: &mut W,
        slot: &FastSlot<'_>,
    ) -> Result<(), WriteError> {
        fast::write_fast_slot::<W, CRC>(out, slot)
    }
}
