//! One slave's slice of a Fast Sync/Bulk Read coalesced reply chain.

#![allow(dead_code)]

use crate::buf::{WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::types::{Slot, SlotPosition};

use super::{emit_slot_body, emit_slot_header};

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
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::streaming::{Event, HeaderEvent, Parser};
    use crate::types::{Id, Instruction, StatusError};
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    #[test]
    fn slot_only_round_trips() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: Id::new(7),
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEncoder::<_, Crc>::new(&mut buf).only(&slot, 9).unwrap();

        let mut p: Parser<Crc> = Parser::new();
        let evs: alloc::vec::Vec<Event> = p.feed(&buf).collect();
        let h = evs
            .iter()
            .find_map(|e| match e {
                Event::Header(HeaderEvent::Status(h)) => Some(*h),
                _ => None,
            })
            .expect("status header");
        assert_eq!(h.id, Id::BROADCAST);
        assert_eq!(h.error, StatusError::OK);
        assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
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
        let crc = u16::from_le_bytes([0xAA, 0xBB]);
        let mut w = SlotEncoder::<_, Crc>::new(&mut buf);
        w.first(&s0, 15).unwrap();
        w.middle(&s1).unwrap();
        w.last(&s2, crc).unwrap();

        let expected = [
            0xFF,
            0xFF,
            0xFD,
            0x00, // sync
            Id::BROADCAST.as_byte(),
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
            id: Id::new(5),
            error: StatusError::from_byte(0x07),
            data: &[0x10, 0x20],
        };
        SlotEncoder::<_, Crc>::new(&mut buf).middle(&slot).unwrap();
        assert_eq!(&buf[..], &[0x07, 0x05, 0x10, 0x20]);
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
        e.emit(&s0, SlotPosition::First { packet_length: 15 })
            .unwrap();
        e.emit(&s1, SlotPosition::Middle).unwrap();
        e.emit(&s2, SlotPosition::Last { crc }).unwrap();

        let mut expected = Buf::new();
        let mut w = SlotEncoder::<_, Crc>::new(&mut expected);
        w.first(&s0, 15).unwrap();
        w.middle(&s1).unwrap();
        w.last(&s2, crc).unwrap();

        assert_eq!(&emitted_via_emit[..], &expected[..]);
    }

    #[test]
    fn slot_emit_only_round_trips_single_slot() {
        let mut buf = Buf::new();
        let slot = Slot {
            id: Id::new(7),
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotEncoder::<_, Crc>::new(&mut buf)
            .emit(&slot, SlotPosition::Only { packet_length: 9 })
            .unwrap();
        let mut p: Parser<Crc> = Parser::new();
        let evs: alloc::vec::Vec<Event> = p.feed(&buf).collect();
        assert!(
            evs.iter()
                .any(|e| matches!(e, Event::Header(HeaderEvent::Status(_))))
        );
        assert!(evs.iter().any(|e| matches!(e, Event::Crc)));
    }
}
