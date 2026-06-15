//! Request frame encoder (master -> slave).

#![allow(dead_code)]

use crate::buf::{WriteBuf, WriteError};
use crate::crc::CrcUmts;
use crate::packet::{BulkReadEntry, Id, Instruction, Packet};

use super::{bulk_entries_as_bytes, emit_frame};

pub struct InstructionEncoder<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    crc: CRC,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> InstructionEncoder<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            crc: CRC::new(),
        }
    }

    pub fn ping(&mut self, id: Id) -> Result<(), WriteError> {
        emit_frame(self.out, &mut self.crc, id, Instruction::Ping.as_u8(), &[])
    }

    pub fn read(&mut self, id: Id, addr: u16, length: u16) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Read.as_u8(),
            &[&a, &l],
        )
    }

    pub fn write(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Write.as_u8(),
            &[&a, data],
        )
    }

    pub fn reg_write(&mut self, id: Id, addr: u16, data: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::RegWrite.as_u8(),
            &[&a, data],
        )
    }

    pub fn action(&mut self, id: Id) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Action.as_u8(),
            &[],
        )
    }

    pub fn reboot(&mut self, id: Id) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Reboot.as_u8(),
            &[],
        )
    }

    pub fn factory_reset(&mut self, id: Id, mode: u8) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::FactoryReset.as_u8(),
            &[&[mode]],
        )
    }

    pub fn clear(&mut self, id: Id, body: &[u8]) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::Clear.as_u8(),
            &[body],
        )
    }

    pub fn control_table_backup(&mut self, id: Id, body: &[u8]) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            id,
            Instruction::ControlTableBackup.as_u8(),
            &[body],
        )
    }

    pub fn sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::SyncRead.as_u8(),
            &[&a, &l, ids],
        )
    }

    pub fn sync_write(&mut self, addr: u16, length: u16, body: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::SyncWrite.as_u8(),
            &[&a, &l, body],
        )
    }

    pub fn bulk_read(&mut self, entries: &[BulkReadEntry]) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::BulkRead.as_u8(),
            &[bulk_entries_as_bytes(entries)],
        )
    }

    pub fn bulk_write(&mut self, body: &[u8]) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::BulkWrite.as_u8(),
            &[body],
        )
    }

    pub fn fast_sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::FastSyncRead.as_u8(),
            &[&a, &l, ids],
        )
    }

    pub fn fast_bulk_read(&mut self, entries: &[BulkReadEntry]) -> Result<(), WriteError> {
        emit_frame(
            self.out,
            &mut self.crc,
            Id::BROADCAST,
            Instruction::FastBulkRead.as_u8(),
            &[bulk_entries_as_bytes(entries)],
        )
    }

    /// Vendor-extension escape hatch -- emit a frame with an arbitrary
    /// instruction byte (e.g. OSC `Calibrate`).
    pub fn ext(&mut self, id: Id, instruction: u8, params: &[u8]) -> Result<(), WriteError> {
        emit_frame(self.out, &mut self.crc, id, instruction, &[params])
    }

    /// Round-trip a decoded [`Packet`] back to wire bytes -- for sniffers,
    /// bridges, and replay tools. Handles every variant including
    /// [`Packet::Status`]; for slave-side typed Status construction,
    /// [`super::StatusEncoder`] is more ergonomic.
    pub fn emit(&mut self, packet: &Packet<'_>) -> Result<(), WriteError> {
        match *packet {
            Packet::Ping(p) => self.ping(p.header.id),
            Packet::Read(p) => self.read(p.header.id, p.addr.get(), p.length.get()),
            Packet::Write(p) => self.write(p.header.header.id, p.header.addr.get(), p.data),
            Packet::RegWrite(p) => self.reg_write(p.header.header.id, p.header.addr.get(), p.data),
            Packet::Action(p) => self.action(p.header.id),
            Packet::Reboot(p) => self.reboot(p.header.id),
            Packet::FactoryReset(p) => self.factory_reset(p.header.id, p.mode),
            Packet::SyncRead(p) => {
                self.sync_read(p.header.addr.get(), p.header.length.get(), p.ids)
            }
            Packet::SyncWrite(p) => {
                self.sync_write(p.header.addr.get(), p.header.length.get(), p.body)
            }
            Packet::BulkRead(p) => self.bulk_read(p.entries),
            Packet::BulkWrite(p) => self.bulk_write(p.body),
            Packet::FastSyncRead(p) => {
                self.fast_sync_read(p.header.addr.get(), p.header.length.get(), p.ids)
            }
            Packet::FastBulkRead(p) => self.fast_bulk_read(p.entries),
            Packet::Raw(r) => self.ext(
                r.header.header.id,
                r.header.header.instruction.as_byte(),
                r.params,
            ),
            Packet::Status(s) => {
                let err = [s.header.error.as_byte()];
                emit_frame(
                    self.out,
                    &mut self.crc,
                    s.header.header.id,
                    Instruction::Status.as_u8(),
                    &[&err, s.params],
                )
            }
        }
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::packet::U16Le;
    use crate::streaming::InstructionHeader as PH;
    use crate::streaming::{Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent};
    use alloc::vec;
    use alloc::vec::Vec as AVec;
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    fn parse(wire: &[u8]) -> AVec<Event> {
        let mut p: Parser<Crc> = Parser::new();
        p.feed(wire).collect()
    }

    fn instr_header(events: &[Event]) -> PH {
        events
            .iter()
            .find_map(|e| match e {
                Event::Header(HeaderEvent::Instruction(h)) => Some(*h),
                _ => None,
            })
            .expect("expected instruction header event")
    }

    fn write_chunks(events: &[Event]) -> AVec<(u16, u16)> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                    offset,
                    length,
                })) => Some((*offset, *length)),
                _ => None,
            })
            .collect()
    }

    fn sync_slots(events: &[Event]) -> AVec<(Id, u8)> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Payload(PayloadEvent::Instruction(InstructionPayload::SyncSlot {
                    id,
                    index,
                })) => Some((*id, *index)),
                _ => None,
            })
            .collect()
    }

    fn bulk_slots(events: &[Event]) -> AVec<(Id, u8, u16, u16)> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Payload(PayloadEvent::Instruction(InstructionPayload::BulkSlot {
                    id,
                    index,
                    address,
                    length,
                })) => Some((*id, *index, *address, *length)),
                _ => None,
            })
            .collect()
    }

    fn assert_crc_good(events: &[Event]) {
        assert!(
            events.iter().any(|e| matches!(e, Event::Crc)),
            "no Crc verdict in events: {events:?}"
        );
    }

    #[test]
    fn instr_ping() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .ping(Id::new(0x01))
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::Ping { id } if id == Id::new(0x01)));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_read() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .read(Id::new(0x02), 0x0084, 4)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::Read {
                id,
                address: 0x0084,
                length: 4,
            } if id == Id::new(0x02)
        ));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_write_with_data() {
        let mut buf = Buf::new();
        let data = [0xAA, 0xBB, 0xCC, 0xDD];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .write(Id::new(0x03), 0x0084, &data)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::Write {
                id,
                address: 0x0084,
                length: 4,
            } if id == Id::new(0x03)
        ));
        let chunks = write_chunks(&evs);
        let total: u16 = chunks.iter().map(|(_, l)| *l).sum();
        assert_eq!(total, 4);
        for (off, len) in chunks {
            assert_eq!(
                &buf[off as usize..(off + len) as usize],
                &data[..len as usize]
            );
        }
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_reg_write() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .reg_write(Id::new(0x04), 0x0030, &data)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::RegWrite {
                id,
                address: 0x0030,
                length: 2,
            } if id == Id::new(0x04)
        ));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_action() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .action(Id::new(0x05))
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::Action { id } if id == Id::new(0x05)));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_reboot() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .reboot(Id::new(0x06))
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::Reboot { id } if id == Id::new(0x06)));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_factory_reset() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .factory_reset(Id::new(0x07), 0x02)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::FactoryReset { id, mode: 0x02 } if id == Id::new(0x07)
        ));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_clear_decodes_as_clear_with_body_length() {
        let mut buf = Buf::new();
        let body = [0x01, 0x44, 0x58, 0x4C, 0x22];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .clear(Id::new(0x08), &body)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::Clear { id, length: 5 } if id == Id::new(0x08)
        ));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_control_table_backup_decodes_as_ctb_with_body_length() {
        let mut buf = Buf::new();
        let body = [0xAA, 0xBB];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .control_table_backup(Id::new(0x09), &body)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::ControlTableBackup { id, length: 2 } if id == Id::new(0x09)
        ));
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_sync_read() {
        let mut buf = Buf::new();
        let ids = [0x01, 0x02, 0x03];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .sync_read(0x0084, 4, &ids)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::SyncRead {
                id,
                address: 0x0084,
                length: 4,
            } if id == Id::BROADCAST
        ));
        let slots = sync_slots(&evs);
        assert_eq!(
            slots,
            vec![(Id::new(0x01), 0), (Id::new(0x02), 1), (Id::new(0x03), 2),]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_sync_write() {
        let mut buf = Buf::new();
        let body = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .sync_write(0x0080, 2, &body)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::SyncWrite {
                id,
                address: 0x0080,
                length: 2,
            } if id == Id::BROADCAST
        ));
        assert_eq!(
            sync_slots(&evs),
            vec![(Id::new(0x01), 0), (Id::new(0x02), 1)]
        );
        let chunks = write_chunks(&evs);
        assert_eq!(chunks.len(), 2);
        assert_eq!(
            &buf[chunks[0].0 as usize..(chunks[0].0 + 2) as usize],
            &[0xAA, 0xBB]
        );
        assert_eq!(
            &buf[chunks[1].0 as usize..(chunks[1].0 + 2) as usize],
            &[0xCC, 0xDD]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_bulk_read() {
        let mut buf = Buf::new();
        let entries = [
            BulkReadEntry {
                id: Id::new(1),
                addr: U16Le::from_u16(0x0084),
                length: U16Le::from_u16(4),
            },
            BulkReadEntry {
                id: Id::new(2),
                addr: U16Le::from_u16(0x0090),
                length: U16Le::from_u16(2),
            },
        ];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .bulk_read(&entries)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::BulkRead { id } if id == Id::BROADCAST));
        assert_eq!(
            bulk_slots(&evs),
            vec![(Id::new(1), 0, 0x0084, 4), (Id::new(2), 1, 0x0090, 2),]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_bulk_write() {
        let mut buf = Buf::new();
        let body = [
            0x01, 0x84, 0x00, 0x02, 0x00, 0xAA, 0xBB, 0x02, 0x90, 0x00, 0x01, 0x00, 0xCC,
        ];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .bulk_write(&body)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::BulkWrite { id } if id == Id::BROADCAST));
        assert_eq!(
            bulk_slots(&evs),
            vec![(Id::new(0x01), 0, 0x0084, 2), (Id::new(0x02), 1, 0x0090, 1),]
        );
        let chunks = write_chunks(&evs);
        assert_eq!(chunks.len(), 2);
        assert_eq!(
            &buf[chunks[0].0 as usize..(chunks[0].0 + 2) as usize],
            &[0xAA, 0xBB]
        );
        assert_eq!(
            &buf[chunks[1].0 as usize..(chunks[1].0 + 1) as usize],
            &[0xCC]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_fast_sync_read() {
        let mut buf = Buf::new();
        let ids = [0x01, 0x02];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .fast_sync_read(0x0084, 4, &ids)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::FastSyncRead {
                id,
                address: 0x0084,
                length: 4,
            } if id == Id::BROADCAST
        ));
        assert_eq!(
            sync_slots(&evs),
            vec![(Id::new(0x01), 0), (Id::new(0x02), 1)]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_fast_bulk_read() {
        let mut buf = Buf::new();
        let entries = [
            BulkReadEntry {
                id: Id::new(1),
                addr: U16Le::from_u16(0x0084),
                length: U16Le::from_u16(4),
            },
            BulkReadEntry {
                id: Id::new(2),
                addr: U16Le::from_u16(0x0090),
                length: U16Le::from_u16(2),
            },
        ];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .fast_bulk_read(&entries)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(instr_header(&evs), PH::FastBulkRead { id } if id == Id::BROADCAST));
        assert_eq!(
            bulk_slots(&evs),
            vec![(Id::new(1), 0, 0x0084, 4), (Id::new(2), 1, 0x0090, 2),]
        );
        assert_crc_good(&evs);
    }

    #[test]
    fn instr_ext_decodes_as_raw() {
        let mut buf = Buf::new();
        let params = [0xDE, 0xAD, 0xBE, 0xEF];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .ext(Id::new(0x0A), 0xE0, &params)
            .unwrap();
        let evs = parse(&buf);
        assert!(matches!(
            instr_header(&evs),
            PH::Raw {
                id,
                instr: 0xE0,
                length: 4,
            } if id == Id::new(0x0A)
        ));
        assert_crc_good(&evs);
    }
}
