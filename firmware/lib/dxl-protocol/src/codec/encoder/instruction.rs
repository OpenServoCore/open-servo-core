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
mod tests {
    use super::*;
    use crate::codec::decoder::{Decoder, Step};
    use crate::codec::encoder::StatusEncoder;
    use crate::crc_software::SoftwareCrcUmts;
    use crate::packet::{Packet, StatusError, U16Le};
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    #[test]
    fn instr_ping() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .ping(Id::new(0x01))
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(&buf);
        assert_eq!(n, buf.len());
        match step {
            Step::Packet(Packet::Ping(p)) => {
                assert_eq!(p.header.id, Id::new(0x01));
                assert_eq!(p.header.instruction.kind(), Instruction::Ping);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn instr_read() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .read(Id::new(0x02), 0x0084, 4)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Read(p)) => {
                assert_eq!(p.header.id, Id::new(0x02));
                assert_eq!(p.addr.get(), 0x0084);
                assert_eq!(p.length.get(), 4);
            }
            other => panic!("expected Read, got {other:?}"),
        }
    }

    #[test]
    fn instr_write_with_data() {
        let mut buf = Buf::new();
        let data = [0xAA, 0xBB, 0xCC, 0xDD];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .write(Id::new(0x03), 0x0084, &data)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Write(w)) => {
                assert_eq!(w.header.header.id, Id::new(0x03));
                assert_eq!(w.header.addr.get(), 0x0084);
                assert_eq!(w.data, &data);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }

    #[test]
    fn instr_reg_write() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .reg_write(Id::new(0x04), 0x0030, &data)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::RegWrite(w)) => {
                assert_eq!(w.header.header.id, Id::new(0x04));
                assert_eq!(w.header.addr.get(), 0x0030);
                assert_eq!(w.data, &data);
            }
            other => panic!("expected RegWrite, got {other:?}"),
        }
    }

    #[test]
    fn instr_action() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .action(Id::new(0x05))
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Action(p)) => assert_eq!(p.header.id, Id::new(0x05)),
            other => panic!("expected Action, got {other:?}"),
        }
    }

    #[test]
    fn instr_reboot() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .reboot(Id::new(0x06))
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Reboot(p)) => assert_eq!(p.header.id, Id::new(0x06)),
            other => panic!("expected Reboot, got {other:?}"),
        }
    }

    #[test]
    fn instr_factory_reset() {
        let mut buf = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .factory_reset(Id::new(0x07), 0x02)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FactoryReset(p)) => {
                assert_eq!(p.header.id, Id::new(0x07));
                assert_eq!(p.mode, 0x02);
            }
            other => panic!("expected FactoryReset, got {other:?}"),
        }
    }

    #[test]
    fn instr_clear_dispatches_as_raw() {
        let mut buf = Buf::new();
        let body = [0x01, 0x44, 0x58, 0x4C, 0x22];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .clear(Id::new(0x08), &body)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, Id::new(0x08));
                assert_eq!(r.header.header.instruction.kind(), Instruction::Clear);
                assert_eq!(r.params, &body);
            }
            other => panic!("expected Raw(Clear), got {other:?}"),
        }
    }

    #[test]
    fn instr_control_table_backup_dispatches_as_raw() {
        let mut buf = Buf::new();
        let body = [0xAA, 0xBB];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .control_table_backup(Id::new(0x09), &body)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, Id::new(0x09));
                assert_eq!(
                    r.header.header.instruction.kind(),
                    Instruction::ControlTableBackup
                );
                assert_eq!(r.params, &body);
            }
            other => panic!("expected Raw(ControlTableBackup), got {other:?}"),
        }
    }

    #[test]
    fn instr_sync_read() {
        let mut buf = Buf::new();
        let ids = [0x01, 0x02, 0x03];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .sync_read(0x0084, 4, &ids)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::SyncRead(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                assert_eq!(p.header.addr.get(), 0x0084);
                assert_eq!(p.header.length.get(), 4);
                assert_eq!(p.ids, &ids);
            }
            other => panic!("expected SyncRead, got {other:?}"),
        }
    }

    #[test]
    fn instr_sync_write() {
        let mut buf = Buf::new();
        let body = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .sync_write(0x0080, 2, &body)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::SyncWrite(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                assert_eq!(p.header.addr.get(), 0x0080);
                assert_eq!(p.header.length.get(), 2);
                let entries: Vec<_, 4> = p.entries().collect();
                assert_eq!(entries.len(), 2);
                assert_eq!(entries[0].id, Id::new(0x01));
                assert_eq!(entries[0].data, &[0xAA, 0xBB]);
                assert_eq!(entries[1].id, Id::new(0x02));
                assert_eq!(entries[1].data, &[0xCC, 0xDD]);
            }
            other => panic!("expected SyncWrite, got {other:?}"),
        }
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
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::BulkRead(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                assert_eq!(p.entries.len(), 2);
                assert_eq!(p.entries[0].id, Id::new(1));
                assert_eq!(p.entries[0].addr.get(), 0x0084);
                assert_eq!(p.entries[0].length.get(), 4);
                assert_eq!(p.entries[1].id, Id::new(2));
                assert_eq!(p.entries[1].addr.get(), 0x0090);
                assert_eq!(p.entries[1].length.get(), 2);
            }
            other => panic!("expected BulkRead, got {other:?}"),
        }
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
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::BulkWrite(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                let entries: Vec<_, 4> = p.entries().collect();
                assert_eq!(entries.len(), 2);
                assert_eq!(entries[0].id, Id::new(0x01));
                assert_eq!(entries[0].addr, 0x0084);
                assert_eq!(entries[0].data, &[0xAA, 0xBB]);
                assert_eq!(entries[1].id, Id::new(0x02));
                assert_eq!(entries[1].addr, 0x0090);
                assert_eq!(entries[1].data, &[0xCC]);
            }
            other => panic!("expected BulkWrite, got {other:?}"),
        }
    }

    #[test]
    fn instr_fast_sync_read() {
        let mut buf = Buf::new();
        let ids = [0x01, 0x02];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .fast_sync_read(0x0084, 4, &ids)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FastSyncRead(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                assert_eq!(p.header.addr.get(), 0x0084);
                assert_eq!(p.header.length.get(), 4);
                assert_eq!(p.ids, &ids);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        }
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
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FastBulkRead(p)) => {
                assert_eq!(p.header.header.id, Id::BROADCAST);
                assert_eq!(p.entries.len(), 2);
                assert_eq!(p.entries[0].id, Id::new(1));
                assert_eq!(p.entries[1].id, Id::new(2));
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }

    #[test]
    fn instr_ext_dispatches_as_raw() {
        let mut buf = Buf::new();
        let params = [0xDE, 0xAD, 0xBE, 0xEF];
        InstructionEncoder::<_, Crc>::new(&mut buf)
            .ext(Id::new(0x0A), 0xE0, &params)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, Id::new(0x0A));
                assert_eq!(r.header.header.instruction.kind(), Instruction::Ext(0xE0));
                assert_eq!(r.params, &params);
            }
            other => panic!("expected Raw(Ext), got {other:?}"),
        }
    }

    #[test]
    fn instruction_emit_round_trips_read_through_decoder() {
        let mut a = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut a)
            .read(Id::new(0x02), 0x0084, 4)
            .unwrap();

        let mut dec: Decoder<32, Crc> = Decoder::new();
        let pkt = match dec.feed(&a).0 {
            Step::Packet(p) => p,
            other => panic!("expected Packet, got {other:?}"),
        };

        let mut b = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut b)
            .emit(&pkt)
            .unwrap();
        assert_eq!(&a[..], &b[..]);
    }

    #[test]
    fn instruction_emit_round_trips_write_with_data() {
        let mut a = Buf::new();
        let data = [0xAA, 0xBB, 0xCC, 0xDD];
        InstructionEncoder::<_, Crc>::new(&mut a)
            .write(Id::new(0x03), 0x0084, &data)
            .unwrap();

        let mut dec: Decoder<64, Crc> = Decoder::new();
        let pkt = match dec.feed(&a).0 {
            Step::Packet(p) => p,
            other => panic!("expected Packet, got {other:?}"),
        };

        let mut b = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut b)
            .emit(&pkt)
            .unwrap();
        assert_eq!(&a[..], &b[..]);
    }

    #[test]
    fn instruction_emit_round_trips_raw_extension() {
        let mut a = Buf::new();
        let params = [0xDE, 0xAD, 0xBE, 0xEF];
        InstructionEncoder::<_, Crc>::new(&mut a)
            .ext(Id::new(0x0A), 0xE0, &params)
            .unwrap();

        let mut dec: Decoder<32, Crc> = Decoder::new();
        let pkt = match dec.feed(&a).0 {
            Step::Packet(p) => p,
            other => panic!("expected Packet, got {other:?}"),
        };

        let mut b = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut b)
            .emit(&pkt)
            .unwrap();
        assert_eq!(&a[..], &b[..]);
    }

    #[test]
    fn instruction_emit_forwards_status_packet() {
        let mut a = Buf::new();
        StatusEncoder::<_, Crc>::new(&mut a)
            .read(Id::new(0x07), StatusError::OK, &[0x10, 0x20, 0x30])
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let pkt = match dec.feed(&a).0 {
            Step::Packet(p) => p,
            other => panic!("expected Packet, got {other:?}"),
        };
        let mut b = Buf::new();
        InstructionEncoder::<_, Crc>::new(&mut b)
            .emit(&pkt)
            .unwrap();
        assert_eq!(&a[..], &b[..]);
    }
}
