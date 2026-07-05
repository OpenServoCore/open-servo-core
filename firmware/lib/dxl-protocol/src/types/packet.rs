//! Typed decode over a [`RawFrame`]: instruction packets (host->servo),
//! single Status replies (servo->host), and the FAST chain reply walker.
//!
//! Param regions destuff lazily — every borrow-typed field is a [`Bytes`]
//! that only unstuffs when the caller iterates it.

use crate::crc::crc16_umts_continue;
use crate::frame::{FrameKind, RawFrame};
use crate::types::{Id, Instruction as Op, StatusError};
use crate::unstuff::{ByteIter, Bytes};
use crate::wire::{CRC_BYTES, HEADER};

/// A decoded host->servo instruction packet. Borrow-typed: data-carrying
/// variants hold a [`Bytes`] view into the frame body.
#[derive(Copy, Clone, Debug)]
pub enum Instruction<'a> {
    Ping {
        id: Id,
    },
    Read {
        id: Id,
        address: u16,
        length: u16,
    },
    Write {
        id: Id,
        address: u16,
        data: Bytes<'a>,
    },
    RegWrite {
        id: Id,
        address: u16,
        data: Bytes<'a>,
    },
    Action {
        id: Id,
    },
    FactoryReset {
        id: Id,
        mode: u8,
    },
    Reboot {
        id: Id,
    },
    Clear {
        id: Id,
        body: Bytes<'a>,
    },
    ControlTableBackup {
        id: Id,
        body: Bytes<'a>,
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
    /// Unknown / vendor-extension instruction byte. Params are opaque; the
    /// caller's extension layer decodes them.
    Ext {
        id: Id,
        instruction: u8,
        params: Bytes<'a>,
    },
}

/// A decoded single-target Status reply (servo->host). The coalesced FAST
/// chain reply decodes via [`ChainStatusBlocks`] instead.
#[derive(Copy, Clone, Debug)]
pub struct StatusReply<'a> {
    pub id: Id,
    pub error: StatusError,
    pub params: Bytes<'a>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// A fixed-layout field ran past the end of the params, or trailing bytes
    /// remained after a fixed-size instruction.
    Layout,
    /// The frame's kind doesn't match this decoder (e.g. a Status frame handed
    /// to [`decode_instruction`], or an instruction handed to
    /// [`decode_status`]).
    WrongKind,
}

/// Decode a host->servo instruction frame.
pub fn decode_instruction<'a>(frame: &RawFrame<'a>) -> Result<Instruction<'a>, DecodeError> {
    let id = Id::new(frame.id);
    let params = frame.body;
    match Op::from_u8(frame.instruction) {
        Op::Ping => empty(params).map(|()| Instruction::Ping { id }),
        Op::Read => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            end(&mut it)?;
            Ok(Instruction::Read {
                id,
                address,
                length,
            })
        }
        Op::Write => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Instruction::Write {
                id,
                address,
                data: it.rest_bytes(),
            })
        }
        Op::RegWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            Ok(Instruction::RegWrite {
                id,
                address,
                data: it.rest_bytes(),
            })
        }
        Op::Action => empty(params).map(|()| Instruction::Action { id }),
        Op::Reboot => empty(params).map(|()| Instruction::Reboot { id }),
        Op::FactoryReset => {
            let mut it = ByteIter::stuffed(params);
            let mode = it.next().ok_or(DecodeError::Layout)?;
            end(&mut it)?;
            Ok(Instruction::FactoryReset { id, mode })
        }
        Op::Clear => Ok(Instruction::Clear {
            id,
            body: Bytes::stuffed(params),
        }),
        Op::ControlTableBackup => Ok(Instruction::ControlTableBackup {
            id,
            body: Bytes::stuffed(params),
        }),
        Op::SyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Instruction::SyncRead {
                address,
                length,
                ids: it.rest_bytes(),
            })
        }
        Op::SyncWrite => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Instruction::SyncWrite {
                address,
                length,
                body: it.rest_bytes(),
            })
        }
        Op::FastSyncRead => {
            let mut it = ByteIter::stuffed(params);
            let address = take_u16_le(&mut it)?;
            let length = take_u16_le(&mut it)?;
            Ok(Instruction::FastSyncRead {
                address,
                length,
                ids: it.rest_bytes(),
            })
        }
        Op::BulkRead => Ok(Instruction::BulkRead {
            body: Bytes::stuffed(params),
        }),
        Op::BulkWrite => Ok(Instruction::BulkWrite {
            body: Bytes::stuffed(params),
        }),
        Op::FastBulkRead => Ok(Instruction::FastBulkRead {
            body: Bytes::stuffed(params),
        }),
        Op::Status => Err(DecodeError::WrongKind),
        Op::Ext(b) => Ok(Instruction::Ext {
            id,
            instruction: b,
            params: Bytes::stuffed(params),
        }),
    }
}

/// Decode a single-target Status reply. `frame.body[0]` is the error byte;
/// the remainder is the stuffed param region.
pub fn decode_status<'a>(frame: &RawFrame<'a>) -> Result<StatusReply<'a>, DecodeError> {
    if frame.kind != FrameKind::Status {
        return Err(DecodeError::WrongKind);
    }
    let mut it = ByteIter::stuffed(frame.body);
    let error = it.next().ok_or(DecodeError::Layout)?;
    Ok(StatusReply {
        id: Id::new(frame.id),
        error: StatusError::from_byte(error),
        params: it.rest_bytes(),
    })
}

/// One block of a decoded FAST chain reply.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ChainStatusBlock<'a> {
    pub id: Id,
    pub error: StatusError,
    pub data: &'a [u8],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChainStatusError {
    /// Block `block`'s cumulative checkpoint CRC didn't match the wire; the
    /// walker fuses here (later checkpoints chain off this one).
    Checkpoint { block: usize },
    /// The frame ended before block `block` was fully present.
    Truncated { block: usize },
}

enum Lengths<'b> {
    Uniform(usize),
    Each(core::slice::Iter<'b, u16>),
}

/// Walks a [`FrameKind::ChainStatus`] frame block by block, verifying each
/// cumulative checkpoint against the wire.
///
/// Official FAST format: each block is `err(1) + id(1) + data(L_k) + crc(2)`
/// with NO byte stuffing inside blocks, where `crc` is the cumulative CRC-16
/// from the frame's sync header through that block's data. The per-block data
/// lengths come from the caller (the host knows its request); the uniform case
/// derives the block count from the frame length.
///
/// The iterator yields `Ok` per good block and fuses with `Err` at the first
/// bad checkpoint or a truncated frame — so failures report the block index.
pub struct ChainStatusBlocks<'a, 'b> {
    body: &'a [u8],
    crc: u16,
    lengths: Lengths<'b>,
    pos: usize,
    index: usize,
    done: bool,
}

impl<'a> ChainStatusBlocks<'a, 'static> {
    /// Uniform per-block data length; block count derived from the frame.
    pub fn uniform(frame: &RawFrame<'a>, data_len: u16) -> Self {
        Self::seed(frame, Lengths::Uniform(data_len as usize))
    }
}

impl<'a, 'b> ChainStatusBlocks<'a, 'b> {
    /// Explicit per-block data lengths (host knows its request layout).
    pub fn with_lengths(frame: &RawFrame<'a>, lengths: &'b [u16]) -> Self {
        Self::seed(frame, Lengths::Each(lengths.iter()))
    }

    fn seed(frame: &RawFrame<'a>, lengths: Lengths<'b>) -> Self {
        // The checkpoint CRC is cumulative from the sync header. A ChainStatus
        // body spans instruction+1 .. end, so LENGTH = body.len() + 1, letting
        // us reconstruct the 8-byte header prefix the CRC is seeded over.
        let length = (frame.body.len() + 1) as u16;
        let lb = length.to_le_bytes();
        let header = [
            HEADER[0],
            HEADER[1],
            HEADER[2],
            HEADER[3],
            frame.id,
            lb[0],
            lb[1],
            frame.instruction,
        ];
        Self {
            body: frame.body,
            crc: crc16_umts_continue(0, &header),
            lengths,
            pos: 0,
            index: 0,
            done: false,
        }
    }
}

impl<'a> Iterator for ChainStatusBlocks<'a, '_> {
    type Item = Result<ChainStatusBlock<'a>, ChainStatusError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        let data_len = match &mut self.lengths {
            Lengths::Uniform(l) => {
                if self.pos >= self.body.len() {
                    return None;
                }
                *l
            }
            Lengths::Each(it) => *it.next()? as usize,
        };

        let block = self.index;
        // err(1) + id(1) + data(L) + crc(2).
        let need = 2 + data_len + CRC_BYTES;
        if self.body.len() - self.pos < need {
            self.done = true;
            return Some(Err(ChainStatusError::Truncated { block }));
        }

        let start = self.pos;
        let error = self.body[start];
        let id = self.body[start + 1];
        let data = &self.body[start + 2..start + 2 + data_len];

        // Fold err+id+data, compare the checkpoint, then fold the checkpoint
        // bytes themselves — later blocks' cumulative CRCs cover them too.
        self.crc = crc16_umts_continue(self.crc, &self.body[start..start + 2 + data_len]);
        let cpos = start + 2 + data_len;
        let wire = u16::from_le_bytes([self.body[cpos], self.body[cpos + 1]]);
        let ok = self.crc == wire;
        self.crc = crc16_umts_continue(self.crc, &self.body[cpos..cpos + CRC_BYTES]);

        self.pos = cpos + CRC_BYTES;
        self.index += 1;

        if !ok {
            self.done = true;
            return Some(Err(ChainStatusError::Checkpoint { block }));
        }
        Some(Ok(ChainStatusBlock {
            id: Id::new(id),
            error: StatusError::from_byte(error),
            data,
        }))
    }
}

fn empty(params: &[u8]) -> Result<(), DecodeError> {
    // An empty logical body is empty on the wire too — stuffing never
    // produces bytes from nothing.
    if params.is_empty() {
        Ok(())
    } else {
        Err(DecodeError::Layout)
    }
}

fn end(it: &mut ByteIter<'_>) -> Result<(), DecodeError> {
    if it.next().is_some() {
        Err(DecodeError::Layout)
    } else {
        Ok(())
    }
}

fn take_u16_le(it: &mut ByteIter<'_>) -> Result<u16, DecodeError> {
    let lo = it.next().ok_or(DecodeError::Layout)?;
    let hi = it.next().ok_or(DecodeError::Layout)?;
    Ok(u16::from_le_bytes([lo, hi]))
}
