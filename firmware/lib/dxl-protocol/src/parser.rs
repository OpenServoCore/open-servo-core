use crate::Instruction;
use crate::bytes::{ByteIter, Bytes};
use crate::crc::crc16;

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const BROADCAST_ID: u8 = 0xFE;

/// Cap on the wire `Length` field. The field is u16 (up to 65535) but real
/// DXL frames stay well under this. The cap bounds how long the parser will
/// wait on a phantom header in random byte traffic.
pub const MAX_LENGTH: usize = 1024;

#[derive(Copy, Clone, Debug)]
pub enum Packet<'a> {
    Ping {
        id: u8,
    },
    Read {
        id: u8,
        address: u16,
        length: u16,
    },
    Write {
        id: u8,
        address: u16,
        data: Bytes<'a>,
    },
    RegWrite {
        id: u8,
        address: u16,
        data: Bytes<'a>,
    },
    Action {
        id: u8,
    },
    FactoryReset {
        id: u8,
        mode: u8,
    },
    Reboot {
        id: u8,
    },
    Clear {
        id: u8,
        body: Bytes<'a>,
    },
    ControlTableBackup {
        id: u8,
        body: Bytes<'a>,
    },
    Status {
        id: u8,
        error: u8,
        params: Bytes<'a>,
    },
    SyncRead {
        address: u16,
        length: u16,
        ids: Bytes<'a>,
    },
    SyncWrite {
        address: u16,
        length: u16,
        body: Bytes<'a>,
    },
    BulkRead {
        body: Bytes<'a>,
    },
    BulkWrite {
        body: Bytes<'a>,
    },
    FastSyncRead {
        address: u16,
        length: u16,
        ids: Bytes<'a>,
    },
    FastBulkRead {
        body: Bytes<'a>,
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    Incomplete,
    Resync { skip: usize },
    BadCrc { skip: usize },
    BadInstruction { skip: usize },
    BadLength { skip: usize },
}

/// Parse a single Protocol 2.0 frame from `input`.
///
/// On success returns the decoded `Packet` and the number of input bytes
/// consumed. On error variants other than `Incomplete`, `skip` is the
/// number of bytes the caller should drop before retrying.
pub fn parse_one(input: &[u8]) -> Result<(Packet<'_>, usize), ParseError> {
    let header_off = match find_header(input) {
        Some(0) => 0,
        Some(n) => return Err(ParseError::Resync { skip: n }),
        None => {
            // Keep only the longest suffix of `input` that is a proper
            // prefix of HEADER — those are the only bytes that could yet
            // form the start of a valid frame. HEADER has no internal
            // repetition, so the candidate prefixes are 3, 2, 1, 0 bytes.
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

    // A bad length means we don't trust this header — step past it (4 bytes)
    // rather than honoring its claimed frame size, so a spurious header in
    // random traffic can't mask a real frame that follows.
    if !(3..=MAX_LENGTH).contains(&length) {
        return Err(ParseError::BadLength { skip: HEADER.len() });
    }

    let frame_len = 7 + length;
    if frame.len() < frame_len {
        return Err(ParseError::Incomplete);
    }

    let crc_pos = frame_len - 2;
    let computed = crc16(&frame[..crc_pos]);
    let received = u16::from_le_bytes([frame[crc_pos], frame[crc_pos + 1]]);
    if computed != received {
        // Same reasoning: CRC failure could mean a corrupted real frame *or*
        // a phantom header — drop just past the header and let the resync
        // path find the next one.
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
        Ping => need_empty(params).map(|_| Packet::Ping { id }),
        Read => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            if it.next().is_some() {
                return Err(DecodeError);
            }
            Ok(Packet::Read {
                id,
                address,
                length,
            })
        }
        Write => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Packet::Write {
                id,
                address,
                data: it.rest_bytes(),
            })
        }
        RegWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Packet::RegWrite {
                id,
                address,
                data: it.rest_bytes(),
            })
        }
        Action => need_empty(params).map(|_| Packet::Action { id }),
        FactoryReset => {
            let mut it = ByteIter::stuffed(params);
            let mode = it.next().ok_or(DecodeError)?;
            if it.next().is_some() {
                return Err(DecodeError);
            }
            Ok(Packet::FactoryReset { id, mode })
        }
        Reboot => need_empty(params).map(|_| Packet::Reboot { id }),
        Clear => Ok(Packet::Clear {
            id,
            body: Bytes::stuffed(params),
        }),
        ControlTableBackup => Ok(Packet::ControlTableBackup {
            id,
            body: Bytes::stuffed(params),
        }),
        Status => {
            let mut it = ByteIter::stuffed(params);
            let error = it.next().ok_or(DecodeError)?;
            Ok(Packet::Status {
                id,
                error,
                params: it.rest_bytes(),
            })
        }
        SyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncRead {
                address,
                length,
                ids: it.rest_bytes(),
            })
        }
        SyncWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::SyncWrite {
                address,
                length,
                body: it.rest_bytes(),
            })
        }
        FastSyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Packet::FastSyncRead {
                address,
                length,
                ids: it.rest_bytes(),
            })
        }
        BulkRead => Ok(Packet::BulkRead {
            body: Bytes::stuffed(params),
        }),
        BulkWrite => Ok(Packet::BulkWrite {
            body: Bytes::stuffed(params),
        }),
        FastBulkRead => Ok(Packet::FastBulkRead {
            body: Bytes::stuffed(params),
        }),
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
