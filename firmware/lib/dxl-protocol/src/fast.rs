use crate::Instruction;
use crate::buf::WriteBuf;
use crate::packet::{BROADCAST_ID, HEADER};
use crate::writer::WriteError;

pub struct FastSlotBody<'a> {
    pub error: u8,
    pub id: u8,
    pub data: &'a [u8],
}

pub enum FastSlot<'a> {
    First {
        header_length: u16,
        body: FastSlotBody<'a>,
    },
    Middle(FastSlotBody<'a>),
    Last(FastSlotBody<'a>),
    Only {
        header_length: u16,
        body: FastSlotBody<'a>,
    },
}

/// CRC slot placeholder. Fire ISR overwrites in-flight during the DMA
/// pre-fetch race; on race loss these bytes appear on the wire and the
/// master sees a CRC mismatch (intentional — surfaces the failure).
const CRC_PLACEHOLDER: [u8; 2] = [0xAA, 0xBB];

pub fn write_fast_slot<W: WriteBuf>(out: &mut W, slot: &FastSlot<'_>) -> Result<(), WriteError> {
    let start = out.len();
    match write_fast_slot_inner(out, slot) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn write_fast_slot_inner<W: WriteBuf>(out: &mut W, slot: &FastSlot<'_>) -> Result<(), WriteError> {
    match slot {
        FastSlot::First {
            header_length,
            body,
        } => {
            write_header(out, *header_length)?;
            write_body(out, body)?;
        }
        FastSlot::Middle(body) => {
            write_body(out, body)?;
        }
        FastSlot::Last(body) => {
            write_body(out, body)?;
            reserve_crc(out)?;
        }
        FastSlot::Only {
            header_length,
            body,
        } => {
            write_header(out, *header_length)?;
            write_body(out, body)?;
            reserve_crc(out)?;
        }
    }
    Ok(())
}

fn write_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(BROADCAST_ID)?;
    let lb = length.to_le_bytes();
    out.push(lb[0])?;
    out.push(lb[1])?;
    out.push(Instruction::Status.as_u8())?;
    Ok(())
}

fn write_body<W: WriteBuf>(out: &mut W, body: &FastSlotBody<'_>) -> Result<(), WriteError> {
    out.push(body.error)?;
    out.push(body.id)?;
    for &b in body.data {
        out.push(b)?;
    }
    Ok(())
}

fn reserve_crc<W: WriteBuf>(out: &mut W) -> Result<(), WriteError> {
    out.push(CRC_PLACEHOLDER[0])?;
    out.push(CRC_PLACEHOLDER[1])?;
    Ok(())
}
