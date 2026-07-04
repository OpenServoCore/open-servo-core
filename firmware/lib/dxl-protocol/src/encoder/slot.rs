//! One slave's slice of a Fast Sync/Bulk Read coalesced reply chain.

use crate::buf::{Chunk, WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Id, Slot, SlotPosition, StatusError};

use super::{emit_slot_body, emit_slot_body_chunked, emit_slot_header};

pub struct SlotEncoder<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    crc: CRC,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> SlotEncoder<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            crc: CRC::new(),
        }
    }

    /// Slot 0's emission: Status header + slot body + locally-computed
    /// cumulative CRC. Every byte the CRC covers is slot 0's own output,
    /// so no placeholder/patch dance is needed. `packet_length` is the
    /// whole chain frame's DXL `Length` (single-slot chains degenerate to
    /// the one-block value).
    pub fn first(&mut self, slot: &Slot<'_>, packet_length: u16) -> Result<(), WriteError> {
        let start = self.out.len();
        match self.first_inner(slot, packet_length, start) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    fn first_inner(
        &mut self,
        slot: &Slot<'_>,
        packet_length: u16,
        start: usize,
    ) -> Result<(), WriteError> {
        emit_slot_header(self.out, packet_length)?;
        emit_slot_body(self.out, slot)?;
        self.crc.reset();
        self.crc.update(&self.out.as_slice()[start..]);
        self.out.push_slice(&self.crc.finalize().to_le_bytes())?;
        Ok(())
    }

    /// Slot k > 0's emission: body + caller-supplied cumulative CRC bytes.
    /// Chain producers don't know the running CRC at build time (prior
    /// slaves' wire bytes only exist at fire time), so chip-side callers
    /// pass a sentinel and patch the trailing two bytes at fire time.
    pub fn successor(&mut self, slot: &Slot<'_>, crc: u16) -> Result<(), WriteError> {
        let start = self.out.len();
        let emit =
            emit_slot_body(self.out, slot).and_then(|()| self.out.push_slice(&crc.to_le_bytes()));
        match emit {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    /// Dispatch on [`SlotPosition`]. Every variant is reachable; no
    /// programmer-error route.
    pub fn emit(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        match position {
            SlotPosition::First { packet_length } => self.first(slot, packet_length),
            SlotPosition::Successor { crc } => self.successor(slot, crc),
        }
    }

    /// Streamed counterpart of [`Self::emit`]: same wire shape, but the
    /// slot body bytes come from a chunk iterator instead of `Slot::data`.
    /// Used by dispatchers that read straight from a control-table iterator
    /// into the TX buffer with no intermediate scratch.
    pub fn emit_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        position: SlotPosition,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        let start = self.out.len();
        match self.emit_chunked_inner(id, error, position, chunks, start) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    fn emit_chunked_inner<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        position: SlotPosition,
        chunks: I,
        start: usize,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        match position {
            SlotPosition::First { packet_length } => {
                emit_slot_header(self.out, packet_length)?;
                emit_slot_body_chunked(self.out, id, error, chunks)?;
                self.crc.reset();
                self.crc.update(&self.out.as_slice()[start..]);
                self.out.push_slice(&self.crc.finalize().to_le_bytes())?;
            }
            SlotPosition::Successor { crc } => {
                emit_slot_body_chunked(self.out, id, error, chunks)?;
                self.out.push_slice(&crc.to_le_bytes())?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_util::{Crc, assert_crc_good, parse_events, status_header};
    use crate::types::{Id, Instruction, StatusError};
    use heapless::Vec;

    type Buf = Vec<u8, 256>;

    #[test]
    fn slot_first_alone_round_trips_as_single_slot_reply() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: Id::new(7),
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEncoder::<_, Crc>::new(&mut buf)
            .first(&slot, 9)
            .unwrap();

        let evs = parse_events(&buf);
        let h = status_header(&evs);
        assert_eq!(h.id, Id::BROADCAST);
        assert_eq!(h.error, StatusError::OK);
        assert_crc_good(&evs);
    }

    #[test]
    fn slot_chain_emits_expected_bytes() {
        let mut buf = Buf::new();
        let s0 = Slot {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[0xAA, 0xBB],
        };
        let s1 = Slot {
            id: Id::new(2),
            error: StatusError::from_byte(0x05),
            data: &[0xCC, 0xDD],
        };
        let s2 = Slot {
            id: Id::new(3),
            error: StatusError::OK,
            data: &[0xEE, 0xFF],
        };
        // Middle/Last take caller-supplied (normally placeholder) CRCs.
        let crc1 = u16::from_le_bytes([0x11, 0x22]);
        let crc2 = u16::from_le_bytes([0xAA, 0xBB]);
        let mut w = SlotEncoder::<_, Crc>::new(&mut buf);
        // len = INST(1) + 3 blocks × (err+id+data2+crc2 = 6) = 19.
        w.first(&s0, 19).unwrap();
        w.successor(&s1, crc1).unwrap();
        w.successor(&s2, crc2).unwrap();

        let prefix = [
            0xFF,
            0xFF,
            0xFD,
            0x00, // sync
            Id::BROADCAST.as_byte(),
            0x13,
            0x00,                        // len = 19
            Instruction::Status.as_u8(), // 0x55
            0x00,
            1,
            0xAA,
            0xBB, // slot 0: err, id, data
        ];
        let crc0 = crate::test_util::crc_oneshot(0, &prefix);
        let mut expected: Buf = Buf::new();
        expected.extend_from_slice(&prefix).unwrap();
        expected.extend_from_slice(&crc0.to_le_bytes()).unwrap(); // slot 0 CRC (locally computed)
        expected
            .extend_from_slice(&[0x05, 2, 0xCC, 0xDD, 0x11, 0x22]) // slot 1: err, id, data, crc
            .unwrap();
        expected
            .extend_from_slice(&[0x00, 3, 0xEE, 0xFF, 0xAA, 0xBB]) // slot 2: err, id, data, crc
            .unwrap();
        assert_eq!(&buf[..], &expected[..]);
    }

    /// Golden vector straight from the ROBOTIS e-manual's Fast Sync Read
    /// example (3 devices, 4-byte reads): the encoder must reproduce the
    /// official status packet byte-for-byte — including slot 0's
    /// locally-computed cumulative CRC (84 08).
    #[test]
    fn slot_chain_reproduces_the_emanual_fast_sync_read_example() {
        let mut buf = Buf::new();
        let mut w = SlotEncoder::<_, Crc>::new(&mut buf);
        w.first(
            &Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[0xA6, 0x00, 0x00, 0x00],
            },
            25,
        )
        .unwrap();
        // Cumulative CRCs as printed in the manual (LE on the wire:
        // 16 CA and D1 9E).
        w.successor(
            &Slot {
                id: Id::new(7),
                error: StatusError::OK,
                data: &[0x1F, 0x08, 0x00, 0x00],
            },
            0xCA16,
        )
        .unwrap();
        w.successor(
            &Slot {
                id: Id::new(4),
                error: StatusError::OK,
                data: &[0xFF, 0x03, 0x00, 0x00],
            },
            0x9ED1,
        )
        .unwrap();

        let expected = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, // header, len=25, inst
            0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, // ID3 block
            0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, // ID7 block
            0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E, // ID4 block
        ];
        assert_eq!(&buf[..], &expected[..]);
    }

    #[test]
    fn slot_successor_emits_body_and_crc_slot() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: Id::new(5),
            error: StatusError::from_byte(0x07),
            data: &[0x10, 0x20],
        };
        SlotEncoder::<_, Crc>::new(&mut buf)
            .successor(&slot, 0x0000)
            .unwrap();
        assert_eq!(&buf[..], &[0x07, 0x05, 0x10, 0x20, 0x00, 0x00]);
    }

    #[test]
    fn slot_emit_dispatches_each_position() {
        let mut emitted_via_emit = Buf::new();
        let s0 = Slot {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[0xAA, 0xBB],
        };
        let s1 = Slot {
            id: Id::new(2),
            error: StatusError::from_byte(0x05),
            data: &[0xCC, 0xDD],
        };
        let s2 = Slot {
            id: Id::new(3),
            error: StatusError::OK,
            data: &[0xEE, 0xFF],
        };
        let crc = u16::from_le_bytes([0xAA, 0xBB]);

        let mut e = SlotEncoder::<_, Crc>::new(&mut emitted_via_emit);
        e.emit(&s0, SlotPosition::First { packet_length: 19 })
            .unwrap();
        e.emit(&s1, SlotPosition::Successor { crc: 0x2211 })
            .unwrap();
        e.emit(&s2, SlotPosition::Successor { crc }).unwrap();

        let mut expected = Buf::new();
        let mut w = SlotEncoder::<_, Crc>::new(&mut expected);
        w.first(&s0, 19).unwrap();
        w.successor(&s1, 0x2211).unwrap();
        w.successor(&s2, crc).unwrap();

        assert_eq!(&emitted_via_emit[..], &expected[..]);
    }

    #[test]
    fn slot_emit_first_round_trips_single_slot() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: Id::new(7),
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEncoder::<_, Crc>::new(&mut buf)
            .emit(&slot, SlotPosition::First { packet_length: 9 })
            .unwrap();
        let evs = parse_events(&buf);
        let _ = status_header(&evs);
        assert_crc_good(&evs);
    }

    #[test]
    fn slot_emit_chunked_matches_data_slice_path_for_each_position() {
        let id = Id::new(0x0A);
        let error = StatusError::from_byte(0x07);
        let data = [0x11, 0x00, 0x00, 0x44, 0x55];
        let slot = Slot {
            id,
            error,
            data: &data,
        };

        for position in [
            SlotPosition::First { packet_length: 13 },
            SlotPosition::Successor { crc: 0x1234 },
        ] {
            let mut by_slice = Buf::new();
            SlotEncoder::<_, Crc>::new(&mut by_slice)
                .emit(&slot, position)
                .unwrap();

            let mut by_chunks = Buf::new();
            SlotEncoder::<_, Crc>::new(&mut by_chunks)
                .emit_chunked(
                    id,
                    error,
                    position,
                    [
                        Chunk::Slice(&data[..1]),
                        Chunk::Zero(2),
                        Chunk::Slice(&data[3..]),
                    ],
                )
                .unwrap();

            assert_eq!(
                by_slice.as_slice(),
                by_chunks.as_slice(),
                "position={position:?}",
            );
        }
    }

    #[test]
    fn slot_emit_chunked_only_round_trips() {
        let mut buf = Buf::new();
        SlotEncoder::<_, Crc>::new(&mut buf)
            .emit_chunked(
                Id::new(7),
                StatusError::OK,
                SlotPosition::First { packet_length: 9 },
                [Chunk::Slice(&[0xAA, 0xBB]), Chunk::Zero(2)],
            )
            .unwrap();
        let evs = parse_events(&buf);
        let _ = status_header(&evs);
        assert_crc_good(&evs);
    }
}
