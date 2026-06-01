use crate::Instruction;
use crate::bytes::{ByteIter, Bytes};
use crate::crc::crc16;
#[cfg(feature = "osc")]
use crate::packet::CalibratePacket;
use crate::packet::{
    ActionPacket, BROADCAST_ID, BulkReadPacket, BulkWritePacket, ClearPacket,
    ControlTableBackupPacket, FactoryResetPacket, FastBulkReadPacket, FastSyncReadPacket, HEADER,
    MAX_LENGTH, Packet, PingPacket, ReadPacket, RebootPacket, RegWritePacket, StatusPacket,
    SyncReadPacket, SyncWritePacket, WritePacket,
};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    Incomplete,
    Resync { skip: usize },
    BadCrc { skip: usize },
    BadInstruction { skip: usize },
    BadLength { skip: usize },
}

/// On error variants other than `Incomplete`, `skip` is the number of bytes
/// the caller should drop before retrying.
pub fn parse_one(input: &[u8]) -> Result<(Packet<'_>, usize), ParseError> {
    let header_off = match find_header(input) {
        Some(0) => 0,
        Some(n) => return Err(ParseError::Resync { skip: n }),
        None => {
            // Keep longest suffix that's a proper HEADER prefix — only those
            // bytes could still complete a valid header. HEADER has no internal
            // repetition, so candidates are 3, 2, 1, 0 bytes.
            let keep = if input.ends_with(&HEADER[..3]) {
                3
            } else if input.ends_with(&HEADER[..2]) {
                2
            } else if input.ends_with(&HEADER[..1]) {
                1
            } else {
                0
            };
            let skip = input.len() - keep;
            if skip == 0 {
                return Err(ParseError::Incomplete);
            }
            return Err(ParseError::Resync { skip });
        }
    };

    let frame = &input[header_off..];

    if frame.len() < 7 {
        return Err(ParseError::Incomplete);
    }

    let id = frame[4];
    let length = u16::from_le_bytes([frame[5], frame[6]]) as usize;

    // Bad length: don't trust this header. Step past 4 bytes rather than
    // honoring its claimed size, so a phantom can't mask a real frame after.
    if !(3..=MAX_LENGTH).contains(&length) {
        return Err(ParseError::BadLength { skip: HEADER.len() });
    }

    let frame_len = 7 + length;
    if frame.len() < frame_len {
        // Fast First/Only chain headers use BROADCAST_ID + Status with a
        // length covering the WHOLE multi-slot reply. When such a header
        // shows up incomplete, the missing bytes never land here — they're
        // on the wire during another node's TX. Returning Incomplete would
        // wedge the caller's poll loop; resync past this phantom header.
        if frame.len() >= 8 && id == BROADCAST_ID && frame[7] == Instruction::Status.as_u8() {
            return Err(ParseError::BadInstruction { skip: HEADER.len() });
        }
        return Err(ParseError::Incomplete);
    }

    let crc_pos = frame_len - 2;
    let computed = crc16(&frame[..crc_pos]);
    let received = u16::from_le_bytes([frame[crc_pos], frame[crc_pos + 1]]);
    if computed != received {
        // Could be a corrupted real frame *or* a phantom header — drop past
        // the header and let resync find the next one.
        return Err(ParseError::BadCrc { skip: HEADER.len() });
    }

    let instruction = match Instruction::from_u8(frame[7]) {
        Some(i) => i,
        None => return Err(ParseError::BadInstruction { skip: frame_len }),
    };
    let params_stuffed = &frame[8..crc_pos];

    let packet = decode(instruction, id, params_stuffed)
        .map_err(|_| ParseError::BadLength { skip: frame_len })?;

    Ok((packet, frame_len))
}

fn find_header(input: &[u8]) -> Option<usize> {
    if input.len() < 4 {
        return None;
    }
    let mut i = 0;
    while i + 4 <= input.len() {
        if input[i..i + 4] == HEADER {
            return Some(i);
        }
        i += 1;
    }
    None
}

#[derive(Copy, Clone, Debug)]
struct DecodeError;

fn decode<'a>(
    instruction: Instruction,
    id: u8,
    params: &'a [u8],
) -> Result<Packet<'a>, DecodeError> {
    use Instruction::*;
    match instruction {
        Ping => need_empty(params).map(|_| Packet::Ping(PingPacket { id })),
        Read => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            if it.next().is_some() {
                return Err(DecodeError);
            }
            Ok(Packet::Read(ReadPacket {
                id,
                address,
                length,
            }))
        }
        Write => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Packet::Write(WritePacket {
                id,
                address,
                data: it.rest_bytes(),
            }))
        }
        RegWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Packet::RegWrite(RegWritePacket {
                id,
                address,
                data: it.rest_bytes(),
            }))
        }
        Action => need_empty(params).map(|_| Packet::Action(ActionPacket { id })),
        FactoryReset => {
            let mut it = ByteIter::stuffed(params);
            let mode = it.next().ok_or(DecodeError)?;
            if it.next().is_some() {
                return Err(DecodeError);
            }
            Ok(Packet::FactoryReset(FactoryResetPacket { id, mode }))
        }
        Reboot => need_empty(params).map(|_| Packet::Reboot(RebootPacket { id })),
        #[cfg(feature = "osc")]
        Calibrate => {
            let mut it = ByteIter::stuffed(params);
            let count = take_u16_le(&mut it)?;
            if it.next().is_some() {
                return Err(DecodeError);
            }
            Ok(Packet::Calibrate(CalibratePacket { id, count }))
        }
        Clear => Ok(Packet::Clear(ClearPacket {
            id,
            body: Bytes::stuffed(params),
        })),
        ControlTableBackup => Ok(Packet::ControlTableBackup(ControlTableBackupPacket {
            id,
            body: Bytes::stuffed(params),
        })),
        Status => {
            let mut it = ByteIter::stuffed(params);
            let error = it.next().ok_or(DecodeError)?;
            Ok(Packet::Status(StatusPacket {
                id,
                error,
                params: it.rest_bytes(),
            }))
        }
        SyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncRead(SyncReadPacket {
                address,
                length,
                ids: it.rest_bytes(),
            }))
        }
        SyncWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncWrite(SyncWritePacket {
                address,
                length,
                body: it.rest_bytes(),
            }))
        }
        FastSyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::FastSyncRead(FastSyncReadPacket {
                address,
                length,
                ids: it.rest_bytes(),
            }))
        }
        BulkRead => Ok(Packet::BulkRead(BulkReadPacket {
            body: Bytes::stuffed(params),
        })),
        BulkWrite => Ok(Packet::BulkWrite(BulkWritePacket {
            body: Bytes::stuffed(params),
        })),
        FastBulkRead => Ok(Packet::FastBulkRead(FastBulkReadPacket {
            body: Bytes::stuffed(params),
        })),
    }
}

fn need_empty(s: &[u8]) -> Result<(), DecodeError> {
    if !s.is_empty() {
        return Err(DecodeError);
    }
    Ok(())
}

fn take_u16_le<I: Iterator<Item = u8>>(it: &mut I) -> Result<u16, DecodeError> {
    let lo = it.next().ok_or(DecodeError)?;
    let hi = it.next().ok_or(DecodeError)?;
    Ok(u16::from_le_bytes([lo, hi]))
}
