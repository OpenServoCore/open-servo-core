//! One slave's slice of a Fast Sync/Bulk Read coalesced reply chain.

#![allow(dead_code)]

use crate::SlotPosition;
use crate::buf::{WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::packet::Slot;

use super::{emit_slot_body, emit_slot_header};

pub struct SlotEmitter<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    crc: CRC,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> SlotEmitter<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            crc: CRC::new(),
        }
    }

    /// Single-slot reply: Status header + slot body + locally-computed CRC.
    pub fn only(&mut self, slot: &Slot<'_>, packet_length: u16) -> Result<(), WriteError> {
        let start = self.out.len();
        match self.only_inner(slot, packet_length, start) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    fn only_inner(
        &mut self,
        slot: &Slot<'_>,
        packet_length: u16,
        start: usize,
    ) -> Result<(), WriteError> {
        emit_slot_header(self.out, packet_length)?;
        emit_slot_body(self.out, slot)?;
        self.crc.reset();
        self.crc.update(&self.out.as_slice()[start..]);
        let crc = self.crc.finalize();
        self.out.push(crc as u8)?;
        self.out.push((crc >> 8) as u8)?;
        Ok(())
    }

    /// First of N: Status header + slot body, no CRC (successors continue).
    pub fn first(&mut self, slot: &Slot<'_>, packet_length: u16) -> Result<(), WriteError> {
        let start = self.out.len();
        match self.first_inner(slot, packet_length) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    fn first_inner(&mut self, slot: &Slot<'_>, packet_length: u16) -> Result<(), WriteError> {
        emit_slot_header(self.out, packet_length)?;
        emit_slot_body(self.out, slot)?;
        Ok(())
    }

    /// Body only.
    pub fn middle(&mut self, slot: &Slot<'_>) -> Result<(), WriteError> {
        let start = self.out.len();
        match emit_slot_body(self.out, slot) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    /// Body + caller-supplied CRC bytes. Chain producers don't know the
    /// running CRC at build time (prior slaves' wire bytes only exist at
    /// fire time), so chip-side callers pass a sentinel and patch the
    /// trailing two bytes at fire time.
    pub fn last(&mut self, slot: &Slot<'_>, crc: u16) -> Result<(), WriteError> {
        let start = self.out.len();
        match self.last_inner(slot, crc) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.out.truncate(start);
                Err(e)
            }
        }
    }

    fn last_inner(&mut self, slot: &Slot<'_>, crc: u16) -> Result<(), WriteError> {
        emit_slot_body(self.out, slot)?;
        self.out.push(crc as u8)?;
        self.out.push((crc >> 8) as u8)?;
        Ok(())
    }

    /// Dispatch on [`SlotPosition`]. Every variant is reachable; no
    /// programmer-error route.
    pub fn emit(&mut self, slot: &Slot<'_>, position: SlotPosition) -> Result<(), WriteError> {
        match position {
            SlotPosition::Only { packet_length } => self.only(slot, packet_length),
            SlotPosition::First { packet_length } => self.first(slot, packet_length),
            SlotPosition::Middle => self.middle(slot),
            SlotPosition::Last { crc } => self.last(slot, crc),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::BROADCAST_ID;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::decoder::{Decoder, Step};
    use crate::packet::{Instruction, Packet, RequestKind, Status, StatusError};
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    #[test]
    fn slot_only_round_trips() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: 7,
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEmitter::<_, Crc>::new(&mut buf).only(&slot, 9).unwrap();

        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, BROADCAST_ID);
                assert_eq!(s.error(), StatusError::OK);
                let slots: Vec<_, 4> = match s.interpret(RequestKind::FastSyncRead) {
                    Status::FastSyncRead { status, .. } => status.slots(4).collect(),
                    other => panic!("expected FastSyncRead, got {other:?}"),
                };
                assert_eq!(slots.len(), 1);
                assert_eq!(slots[0].id, 7);
                assert_eq!(slots[0].error, StatusError::OK);
                assert_eq!(slots[0].data, &[0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn slot_chain_emits_expected_bytes() {
        let mut buf = Buf::new();
        let s0 = Slot {
            id: 1,
            error: StatusError::OK,
            data: &[0xAA, 0xBB],
        };
        let s1 = Slot {
            id: 2,
            error: StatusError::from_byte(0x05),
            data: &[0xCC, 0xDD],
        };
        let s2 = Slot {
            id: 3,
            error: StatusError::OK,
            data: &[0xEE, 0xFF],
        };
        let crc = u16::from_le_bytes([0xAA, 0xBB]);
        let mut w = SlotEmitter::<_, Crc>::new(&mut buf);
        w.first(&s0, 15).unwrap();
        w.middle(&s1).unwrap();
        w.last(&s2, crc).unwrap();

        let expected = [
            0xFF,
            0xFF,
            0xFD,
            0x00, // sync
            BROADCAST_ID,
            0x0F,
            0x00,                        // len = 15
            Instruction::Status.as_u8(), // 0x55
            0x00,
            1,
            0xAA,
            0xBB, // slot 0: err, id, data
            0x05,
            2,
            0xCC,
            0xDD, // slot 1
            0x00,
            3,
            0xEE,
            0xFF, // slot 2
            0xAA,
            0xBB, // crc (caller-supplied)
        ];
        assert_eq!(&buf[..], &expected[..]);
    }

    #[test]
    fn slot_middle_emits_body_only() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: 5,
            error: StatusError::from_byte(0x07),
            data: &[0x10, 0x20],
        };
        SlotEmitter::<_, Crc>::new(&mut buf).middle(&slot).unwrap();
        assert_eq!(&buf[..], &[0x07, 0x05, 0x10, 0x20]);
    }

    #[test]
    fn slot_emit_dispatches_each_position() {
        let mut emitted_via_emit = Buf::new();
        let s0 = Slot {
            id: 1,
            error: StatusError::OK,
            data: &[0xAA, 0xBB],
        };
        let s1 = Slot {
            id: 2,
            error: StatusError::from_byte(0x05),
            data: &[0xCC, 0xDD],
        };
        let s2 = Slot {
            id: 3,
            error: StatusError::OK,
            data: &[0xEE, 0xFF],
        };
        let crc = u16::from_le_bytes([0xAA, 0xBB]);

        let mut e = SlotEmitter::<_, Crc>::new(&mut emitted_via_emit);
        e.emit(&s0, SlotPosition::First { packet_length: 15 })
            .unwrap();
        e.emit(&s1, SlotPosition::Middle).unwrap();
        e.emit(&s2, SlotPosition::Last { crc }).unwrap();

        let mut expected = Buf::new();
        let mut w = SlotEmitter::<_, Crc>::new(&mut expected);
        w.first(&s0, 15).unwrap();
        w.middle(&s1).unwrap();
        w.last(&s2, crc).unwrap();

        assert_eq!(&emitted_via_emit[..], &expected[..]);
    }

    #[test]
    fn slot_emit_only_round_trips_single_slot() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: 7,
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEmitter::<_, Crc>::new(&mut buf)
            .emit(&slot, SlotPosition::Only { packet_length: 9 })
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        assert!(matches!(dec.feed(&buf).0, Step::Packet(Packet::Status(_))));
    }
}
