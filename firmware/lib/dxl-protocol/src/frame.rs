//! Stateless DXL 2.0 framing over byte slices.
//!
//! The crate holds no parser state: drivers own their cursors and countdowns,
//! hosts hold whole buffers. [`probe`] classifies the frame at the head of a
//! slice from its first 8 bytes; [`parse`] is the host/DES convenience that
//! also checks completeness and (for normal frames) the trailing CRC.

use crate::crc::CrcUmts;
use crate::types::Instruction;
use crate::wire::{BROADCAST_ID, CRC_BYTES, HEADER, PACKET_LEN_GUARD, PACKET_LEN_MIN};

/// Bytes up to and including the wire `Length` field: `HEADER(4) + ID(1) +
/// LENGTH(2)`. A frame's total wire size is this plus the `Length` value.
const HEADER_THROUGH_LENGTH: usize = HEADER.len() + 1 + 2;

/// `HEADER_THROUGH_LENGTH + INSTRUCTION(1)` — the fixed prefix `probe` reads.
const HEADER_THROUGH_INSTRUCTION: usize = HEADER_THROUGH_LENGTH + 1;

/// What kind of frame sits at the head of the slice.
///
/// `ChainStatus` is the FAST Sync/Bulk Read coalesced reply: id `0xFE`
/// (broadcast) with the `Status` instruction, whose `Length` spans a whole
/// multi-block chain. It is deliberately its own kind because, unlike a normal
/// frame, it does NOT end in a single CRC covering the whole payload — each
/// block carries its own cumulative checkpoint, so CRC validation is the
/// chain walker's job, not [`parse`]'s.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FrameKind {
    Instruction,
    Status,
    ChainStatus,
}

/// Result of classifying the head of a slice. `total = HEADER_THROUGH_LENGTH +
/// Length` is the full wire size of the frame (`probe` does not require the
/// slice to actually hold that many bytes).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Probe {
    /// Not enough bytes yet to classify; feed more.
    NeedMore,
    /// The leading `skip` bytes cannot begin a valid frame — drop them and
    /// retry. Covers both non-header noise and a header with an untrustworthy
    /// `Length`.
    Junk { skip: usize },
    Frame {
        total: usize,
        id: u8,
        instruction: u8,
        kind: FrameKind,
    },
}

/// Every non-`Incomplete` variant carries `skip`: the number of bytes the
/// caller should drop before retrying, so a stream never wedges.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    Incomplete,
    Junk { skip: usize },
    BadLength { skip: usize },
    BadCrc { skip: usize },
}

/// A validated frame's addressing bytes plus its body slice.
///
/// `body` is the stuffed params region: everything after the instruction byte
/// and before the trailing CRC — except for [`FrameKind::ChainStatus`], where
/// `body` runs through the end of the frame so the chain walker sees every
/// block and every embedded checkpoint. For [`FrameKind::Status`], `body[0]`
/// is the error byte (see [`decode_status`](crate::types::packet::decode_status)).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RawFrame<'a> {
    pub kind: FrameKind,
    pub id: u8,
    pub instruction: u8,
    pub body: &'a [u8],
}

/// Classify the frame at the head of `bytes` from its first 8 bytes.
pub fn probe(bytes: &[u8]) -> Probe {
    match classify(bytes) {
        Classified::NeedMore => Probe::NeedMore,
        Classified::Junk { skip } | Classified::BadLength { skip } => Probe::Junk { skip },
        Classified::Frame {
            total,
            id,
            instruction,
            kind,
        } => Probe::Frame {
            total,
            id,
            instruction,
            kind,
        },
    }
}

/// Probe + completeness + CRC validation over a flat buffer, returning the
/// frame and the number of bytes it consumed.
///
/// Normal frames (`Instruction` / `Status`) are CRC-checked over
/// `[..total - CRC_BYTES]` against the trailing 2 bytes (little-endian).
/// `ChainStatus` frames are returned unchecked: their trailing 2 bytes are the
/// LAST block's cumulative checkpoint and intermediate checkpoints are
/// embedded, so a whole-frame-minus-2 CRC is meaningless — per-block
/// validation belongs to [`ChainStatusBlocks`](crate::types::packet::ChainStatusBlocks).
///
/// `C` selects the CRC implementation; firmware passes its chip provider.
pub fn parse<C: CrcUmts>(bytes: &[u8]) -> Result<(RawFrame<'_>, usize), ParseError> {
    let (total, id, instruction, kind) = match classify(bytes) {
        Classified::NeedMore => return Err(ParseError::Incomplete),
        Classified::Junk { skip } => return Err(ParseError::Junk { skip }),
        Classified::BadLength { skip } => return Err(ParseError::BadLength { skip }),
        Classified::Frame {
            total,
            id,
            instruction,
            kind,
        } => (total, id, instruction, kind),
    };

    if bytes.len() < total {
        return Err(ParseError::Incomplete);
    }

    if kind == FrameKind::ChainStatus {
        let body = &bytes[HEADER_THROUGH_INSTRUCTION..total];
        return Ok((
            RawFrame {
                kind,
                id,
                instruction,
                body,
            },
            total,
        ));
    }

    let crc_pos = total - CRC_BYTES;
    let mut crc = C::new();
    crc.update(&bytes[..crc_pos]);
    let received = u16::from_le_bytes([bytes[crc_pos], bytes[crc_pos + 1]]);
    if crc.finalize() != received {
        // Could be a corrupt real frame or a phantom header — drop the header
        // and let the next probe resync onto the following frame.
        return Err(ParseError::BadCrc { skip: HEADER.len() });
    }

    let body = &bytes[HEADER_THROUGH_INSTRUCTION..crc_pos];
    Ok((
        RawFrame {
            kind,
            id,
            instruction,
            body,
        },
        total,
    ))
}

/// Shared header inspection. `probe` folds `BadLength` into `Junk`; `parse`
/// keeps them distinct.
enum Classified {
    NeedMore,
    Junk {
        skip: usize,
    },
    BadLength {
        skip: usize,
    },
    Frame {
        total: usize,
        id: u8,
        instruction: u8,
        kind: FrameKind,
    },
}

fn classify(bytes: &[u8]) -> Classified {
    match find_header(bytes) {
        HeaderScan::At(0) => {}
        HeaderScan::At(skip) => return Classified::Junk { skip },
        HeaderScan::None { keep } => {
            let skip = bytes.len() - keep;
            return if skip == 0 {
                Classified::NeedMore
            } else {
                Classified::Junk { skip }
            };
        }
    }

    // Header at 0. Need id + length before classifying anything.
    if bytes.len() < HEADER_THROUGH_LENGTH {
        return Classified::NeedMore;
    }
    let id = bytes[HEADER.len()];
    let length = u16::from_le_bytes([bytes[HEADER.len() + 1], bytes[HEADER.len() + 2]]) as usize;

    // Untrustworthy length: distrust the whole header and step past it rather
    // than honoring a claimed size that could mask a real frame behind it.
    if !(PACKET_LEN_MIN..=PACKET_LEN_GUARD).contains(&length) {
        return Classified::BadLength { skip: HEADER.len() };
    }

    if bytes.len() < HEADER_THROUGH_INSTRUCTION {
        return Classified::NeedMore;
    }
    let instruction = bytes[HEADER_THROUGH_INSTRUCTION - 1];

    Classified::Frame {
        total: HEADER_THROUGH_LENGTH + length,
        id,
        instruction,
        kind: frame_kind(id, instruction),
    }
}

fn frame_kind(id: u8, instruction: u8) -> FrameKind {
    if instruction == Instruction::Status.as_u8() {
        if id == BROADCAST_ID {
            FrameKind::ChainStatus
        } else {
            FrameKind::Status
        }
    } else {
        FrameKind::Instruction
    }
}

enum HeaderScan {
    At(usize),
    /// No full header; `keep` is the longest trailing HEADER prefix that could
    /// still complete into one (candidates 3/2/1/0 — HEADER has no internal
    /// repetition).
    None {
        keep: usize,
    },
}

fn find_header(bytes: &[u8]) -> HeaderScan {
    if let Some(i) = bytes.windows(HEADER.len()).position(|w| w == HEADER) {
        return HeaderScan::At(i);
    }
    let keep = if bytes.ends_with(&HEADER[..3]) {
        3
    } else if bytes.ends_with(&HEADER[..2]) {
        2
    } else if bytes.ends_with(&HEADER[..1]) {
        1
    } else {
        0
    };
    HeaderScan::None { keep }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc::SoftwareCrcUmts;

    type Crc = SoftwareCrcUmts;

    const PING: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];

    #[test]
    fn probe_classifies_a_ping_instruction() {
        assert_eq!(
            probe(PING),
            Probe::Frame {
                total: 10,
                id: 0x01,
                instruction: Instruction::Ping.as_u8(),
                kind: FrameKind::Instruction,
            }
        );
    }

    #[test]
    fn probe_distinguishes_status_from_chain_status() {
        // Single-target Status: id 0x01.
        let status = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0, 0];
        match probe(&status) {
            Probe::Frame { kind, .. } => assert_eq!(kind, FrameKind::Status),
            other => panic!("expected Frame, got {other:?}"),
        }
        // Broadcast + Status = coalesced FAST chain reply.
        let chain = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x04, 0x00, 0x55, 0x00, 0, 0];
        match probe(&chain) {
            Probe::Frame { kind, .. } => assert_eq!(kind, FrameKind::ChainStatus),
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn probe_reports_needmore_on_partial_prefix() {
        for partial in [&[0xFFu8][..], &[0xFF, 0xFF][..], &[0xFF, 0xFF, 0xFD][..]] {
            assert_eq!(probe(partial), Probe::NeedMore);
        }
    }

    #[test]
    fn probe_folds_bad_length_into_junk_skip_four() {
        let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
        assert_eq!(probe(&bad), Probe::Junk { skip: 4 });
    }

    #[test]
    fn parse_round_trips_a_ping() {
        let (frame, n) = parse::<Crc>(PING).unwrap();
        assert_eq!(n, PING.len());
        assert_eq!(frame.kind, FrameKind::Instruction);
        assert_eq!(frame.id, 0x01);
        assert_eq!(frame.instruction, Instruction::Ping.as_u8());
        assert!(frame.body.is_empty());
    }

    #[test]
    fn parse_reports_bad_length_distinct_from_junk() {
        let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
        assert_eq!(parse::<Crc>(&bad), Err(ParseError::BadLength { skip: 4 }));
    }

    #[test]
    fn parse_flags_corrupt_crc() {
        let mut frame = PING.to_vec();
        *frame.last_mut().unwrap() ^= 0xFF;
        assert_eq!(parse::<Crc>(&frame), Err(ParseError::BadCrc { skip: 4 }));
    }

    #[test]
    fn parse_is_incomplete_at_every_truncation() {
        for n in 0..PING.len() {
            assert_eq!(parse::<Crc>(&PING[..n]), Err(ParseError::Incomplete));
        }
    }
}
