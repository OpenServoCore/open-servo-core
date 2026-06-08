//! Frame writers. Three structs cover the three frame shapes a master or
//! slave emits onto a DXL 2.0 bus:
//!
//! - [`InstructionWriter`] — request frames from a master.
//! - [`StatusWriter`]      — Status reply frames from a slave.
//! - [`SlotWriter`]        — one slave's slice of a Fast Sync/Bulk Read
//!                           coalesced reply chain.
//!
//! Each writer borrows the caller's TX buffer for its lifetime and emits
//! one frame per method call. The CRC type parameter is bound at
//! construction so call sites don't repeat the generic.

#![allow(dead_code)]

use core::marker::PhantomData;

use crate::instruction::Instruction;
use crate::packet::{BulkReadEntry, Slot, StatusError};
use crate::wire::{
    BROADCAST_ID, CrcUmts, HEADER, STUFFING_BYTE, STUFFING_TRIGGER, WriteBuf, WriteError,
};

// ─────────────────────────── InstructionWriter ───────────────────────────

pub struct InstructionWriter<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    _crc: PhantomData<fn() -> CRC>,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> InstructionWriter<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            _crc: PhantomData,
        }
    }

    pub fn ping(&mut self, id: u8) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(self.out, id, Instruction::Ping.as_u8(), &[])
    }

    pub fn read(&mut self, id: u8, addr: u16, length: u16) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame::<_, CRC>(self.out, id, Instruction::Read.as_u8(), &[&a, &l])
    }

    pub fn write(&mut self, id: u8, addr: u16, data: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        emit_frame::<_, CRC>(self.out, id, Instruction::Write.as_u8(), &[&a, data])
    }

    pub fn reg_write(&mut self, id: u8, addr: u16, data: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        emit_frame::<_, CRC>(self.out, id, Instruction::RegWrite.as_u8(), &[&a, data])
    }

    pub fn action(&mut self, id: u8) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(self.out, id, Instruction::Action.as_u8(), &[])
    }

    pub fn reboot(&mut self, id: u8) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(self.out, id, Instruction::Reboot.as_u8(), &[])
    }

    pub fn factory_reset(&mut self, id: u8, mode: u8) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(
            self.out,
            id,
            Instruction::FactoryReset.as_u8(),
            &[&[mode]],
        )
    }

    pub fn clear(&mut self, id: u8, body: &[u8]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(self.out, id, Instruction::Clear.as_u8(), &[body])
    }

    pub fn control_table_backup(&mut self, id: u8, body: &[u8]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(
            self.out,
            id,
            Instruction::ControlTableBackup.as_u8(),
            &[body],
        )
    }

    pub fn sync_read(&mut self, addr: u16, length: u16, ids: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::SyncRead.as_u8(),
            &[&a, &l, ids],
        )
    }

    pub fn sync_write(&mut self, addr: u16, length: u16, body: &[u8]) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::SyncWrite.as_u8(),
            &[&a, &l, body],
        )
    }

    pub fn bulk_read(&mut self, entries: &[BulkReadEntry]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::BulkRead.as_u8(),
            &[bulk_entries_as_bytes(entries)],
        )
    }

    pub fn bulk_write(&mut self, body: &[u8]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::BulkWrite.as_u8(),
            &[body],
        )
    }

    pub fn fast_sync_read(
        &mut self,
        addr: u16,
        length: u16,
        ids: &[u8],
    ) -> Result<(), WriteError> {
        let a = addr.to_le_bytes();
        let l = length.to_le_bytes();
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::FastSyncRead.as_u8(),
            &[&a, &l, ids],
        )
    }

    pub fn fast_bulk_read(&mut self, entries: &[BulkReadEntry]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(
            self.out,
            BROADCAST_ID,
            Instruction::FastBulkRead.as_u8(),
            &[bulk_entries_as_bytes(entries)],
        )
    }

    /// Vendor-extension escape hatch: emit a frame with an arbitrary
    /// instruction byte. Chip-side extensions (e.g. OSC `Calibrate`) reach
    /// the wire through this.
    pub fn ext(&mut self, id: u8, instruction: u8, params: &[u8]) -> Result<(), WriteError> {
        emit_frame::<_, CRC>(self.out, id, instruction, &[params])
    }
}

// ────────────────────────────── StatusWriter ──────────────────────────────

pub struct StatusWriter<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    _crc: PhantomData<fn() -> CRC>,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> StatusWriter<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            _crc: PhantomData,
        }
    }

    pub fn empty(&mut self, id: u8, error: StatusError) -> Result<(), WriteError> {
        self.frame(id, error, &[])
    }

    pub fn ping(
        &mut self,
        id: u8,
        error: StatusError,
        model: u16,
        fw_version: u8,
    ) -> Result<(), WriteError> {
        let m = model.to_le_bytes();
        let err = [error.as_byte()];
        let fw = [fw_version];
        emit_frame::<_, CRC>(
            self.out,
            id,
            Instruction::Status.as_u8(),
            &[&err, &m, &fw],
        )
    }

    pub fn read(&mut self, id: u8, error: StatusError, data: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, data)
    }

    /// Vendor-extension escape hatch: emit a Status reply with a
    /// chip-defined payload (e.g. OSC `Calibrate` reply).
    pub fn ext(&mut self, id: u8, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
        self.frame(id, error, payload)
    }

    fn frame(&mut self, id: u8, error: StatusError, payload: &[u8]) -> Result<(), WriteError> {
        let err = [error.as_byte()];
        emit_frame::<_, CRC>(
            self.out,
            id,
            Instruction::Status.as_u8(),
            &[&err, payload],
        )
    }
}

// ─────────────────────────────── SlotWriter ───────────────────────────────

pub struct SlotWriter<'a, W: WriteBuf, CRC: CrcUmts> {
    out: &'a mut W,
    _crc: PhantomData<fn() -> CRC>,
}

impl<'a, W: WriteBuf, CRC: CrcUmts> SlotWriter<'a, W, CRC> {
    pub fn new(out: &'a mut W) -> Self {
        Self {
            out,
            _crc: PhantomData,
        }
    }

    /// Single-slot reply: full Status header + slot body + locally-
    /// computed CRC over the bytes just emitted.
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
        let crc = CRC::accumulate(0, &self.out.as_slice()[start..]);
        self.out.push(crc as u8)?;
        self.out.push((crc >> 8) as u8)?;
        Ok(())
    }

    /// First of N slots: full Status header + slot body, no CRC
    /// (successors continue).
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

    /// Middle slot in an N-slot chain: slot body only.
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

    /// Last slot in an N-slot chain: slot body + caller-supplied CRC bytes.
    /// Chain producers accumulate the running CRC across prior slaves' wire
    /// bytes (which only exist at slot-fire time), so chip-side callers pass
    /// a sentinel here at build time and patch the trailing two bytes with
    /// the real CRC at fire time.
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
}

// ───────────────────────────── shared helpers ─────────────────────────────

/// Emit one standalone DXL 2.0 frame: header + id + length + instruction +
/// stuffed params + CRC. `params` is a slice of chunks concatenated in order
/// to form the logical (pre-stuffing) parameter region; byte stuffing of the
/// `0xFF 0xFF 0xFD` trigger is applied inline as bytes are pushed. The CRC
/// runs over the wire bytes (header through stuffed params). On failure
/// `out` is truncated back to its entry length so prior frames stay intact.
fn emit_frame<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    id: u8,
    instruction: u8,
    params: &[&[u8]],
) -> Result<(), WriteError> {
    // `id == 0xFF` would let the unstuffed header..instruction prefix itself
    // complete a stuffing trigger, breaking framing.
    if id == 0xFF {
        return Err(WriteError::Invalid);
    }
    let start = out.len();
    match emit_frame_inner::<W, CRC>(out, start, id, instruction, params) {
        Ok(()) => Ok(()),
        Err(e) => {
            out.truncate(start);
            Err(e)
        }
    }
}

fn emit_frame_inner<W: WriteBuf, CRC: CrcUmts>(
    out: &mut W,
    start: usize,
    id: u8,
    instruction: u8,
    params: &[&[u8]],
) -> Result<(), WriteError> {
    out.push(HEADER[0])?;
    out.push(HEADER[1])?;
    out.push(HEADER[2])?;
    out.push(HEADER[3])?;
    out.push(id)?;
    let len_pos = out.len();
    out.push(0)?;
    out.push(0)?;
    out.push(instruction)?;

    // Seed the sliding window with the instruction byte so a trigger that
    // straddles the instr→params boundary still emits the stuffing FD.
    let mut last2 = [0u8, instruction];
    for chunk in params {
        for &b in *chunk {
            out.push(b)?;
            if [last2[0], last2[1], b] == STUFFING_TRIGGER {
                out.push(STUFFING_BYTE)?;
                last2 = [last2[1], STUFFING_BYTE];
            } else {
                last2 = [last2[1], b];
            }
        }
    }

    let stuffed_params_len = out.len() - (len_pos + 2 + 1);
    let length_value = (1 + stuffed_params_len + 2) as u16;
    let len_bytes = length_value.to_le_bytes();
    out.set(len_pos, len_bytes[0]);
    out.set(len_pos + 1, len_bytes[1]);

    let crc = CRC::accumulate(0, &out.as_slice()[start..]);
    let crc_bytes = crc.to_le_bytes();
    out.push(crc_bytes[0])?;
    out.push(crc_bytes[1])?;

    Ok(())
}

fn emit_slot_header<W: WriteBuf>(out: &mut W, length: u16) -> Result<(), WriteError> {
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

fn emit_slot_body<W: WriteBuf>(out: &mut W, slot: &Slot<'_>) -> Result<(), WriteError> {
    out.push(slot.error.as_byte())?;
    out.push(slot.id)?;
    for &b in slot.data.iter() {
        out.push(b)?;
    }
    Ok(())
}

fn bulk_entries_as_bytes(entries: &[BulkReadEntry]) -> &[u8] {
    // SAFETY: BulkReadEntry is #[repr(C)] align 1 with three byte-sized
    // fields (u8, U16Le, U16Le) = exactly 5 bytes, no padding. The byte
    // view of the slice is sound.
    unsafe {
        core::slice::from_raw_parts(
            entries.as_ptr() as *const u8,
            entries.len() * core::mem::size_of::<BulkReadEntry>(),
        )
    }
}

// ─────────────────────────────────── tests ───────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::{Decoder, Step};
    use crate::packet::{Packet, RequestKind, Status, U16Le};
    use crate::wire::SoftwareCrcUmts;
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    // ── InstructionWriter ──

    #[test]
    fn instr_ping() {
        let mut buf = Buf::new();
        InstructionWriter::<_, Crc>::new(&mut buf).ping(0x01).unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(&buf);
        assert_eq!(n, buf.len());
        match step {
            Step::Packet(Packet::Ping(p)) => {
                assert_eq!(p.header.id, 0x01);
                assert_eq!(p.header.instruction.kind(), Instruction::Ping);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn instr_read() {
        let mut buf = Buf::new();
        InstructionWriter::<_, Crc>::new(&mut buf)
            .read(0x02, 0x0084, 4)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Read(p)) => {
                assert_eq!(p.header.id, 0x02);
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
        InstructionWriter::<_, Crc>::new(&mut buf)
            .write(0x03, 0x0084, &data)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Write(w)) => {
                assert_eq!(w.header.header.id, 0x03);
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
        InstructionWriter::<_, Crc>::new(&mut buf)
            .reg_write(0x04, 0x0030, &data)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::RegWrite(w)) => {
                assert_eq!(w.header.header.id, 0x04);
                assert_eq!(w.header.addr.get(), 0x0030);
                assert_eq!(w.data, &data);
            }
            other => panic!("expected RegWrite, got {other:?}"),
        }
    }

    #[test]
    fn instr_action() {
        let mut buf = Buf::new();
        InstructionWriter::<_, Crc>::new(&mut buf)
            .action(0x05)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Action(p)) => assert_eq!(p.header.id, 0x05),
            other => panic!("expected Action, got {other:?}"),
        }
    }

    #[test]
    fn instr_reboot() {
        let mut buf = Buf::new();
        InstructionWriter::<_, Crc>::new(&mut buf)
            .reboot(0x06)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Reboot(p)) => assert_eq!(p.header.id, 0x06),
            other => panic!("expected Reboot, got {other:?}"),
        }
    }

    #[test]
    fn instr_factory_reset() {
        let mut buf = Buf::new();
        InstructionWriter::<_, Crc>::new(&mut buf)
            .factory_reset(0x07, 0x02)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FactoryReset(p)) => {
                assert_eq!(p.header.id, 0x07);
                assert_eq!(p.mode, 0x02);
            }
            other => panic!("expected FactoryReset, got {other:?}"),
        }
    }

    #[test]
    fn instr_clear_dispatches_as_raw() {
        let mut buf = Buf::new();
        let body = [0x01, 0x44, 0x58, 0x4C, 0x22];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .clear(0x08, &body)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, 0x08);
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
        InstructionWriter::<_, Crc>::new(&mut buf)
            .control_table_backup(0x09, &body)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, 0x09);
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
        InstructionWriter::<_, Crc>::new(&mut buf)
            .sync_read(0x0084, 4, &ids)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::SyncRead(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
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
        // addr=0x80, length=2; two entries [id, d0, d1].
        let body = [0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .sync_write(0x0080, 2, &body)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::SyncWrite(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
                assert_eq!(p.header.addr.get(), 0x0080);
                assert_eq!(p.header.length.get(), 2);
                let entries: Vec<_, 4> = p.entries().collect();
                assert_eq!(entries.len(), 2);
                assert_eq!(entries[0].id, 0x01);
                assert_eq!(entries[0].data, &[0xAA, 0xBB]);
                assert_eq!(entries[1].id, 0x02);
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
                id: 1,
                addr: U16Le::from_u16(0x0084),
                length: U16Le::from_u16(4),
            },
            BulkReadEntry {
                id: 2,
                addr: U16Le::from_u16(0x0090),
                length: U16Le::from_u16(2),
            },
        ];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .bulk_read(&entries)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::BulkRead(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
                assert_eq!(p.entries.len(), 2);
                assert_eq!(p.entries[0].id, 1);
                assert_eq!(p.entries[0].addr.get(), 0x0084);
                assert_eq!(p.entries[0].length.get(), 4);
                assert_eq!(p.entries[1].id, 2);
                assert_eq!(p.entries[1].addr.get(), 0x0090);
                assert_eq!(p.entries[1].length.get(), 2);
            }
            other => panic!("expected BulkRead, got {other:?}"),
        }
    }

    #[test]
    fn instr_bulk_write() {
        let mut buf = Buf::new();
        // Two entries: (id, addr_le, len_le, ...data)
        let body = [
            0x01, 0x84, 0x00, 0x02, 0x00, 0xAA, 0xBB, 0x02, 0x90, 0x00, 0x01, 0x00, 0xCC,
        ];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .bulk_write(&body)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::BulkWrite(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
                let entries: Vec<_, 4> = p.entries().collect();
                assert_eq!(entries.len(), 2);
                assert_eq!(entries[0].id, 0x01);
                assert_eq!(entries[0].addr, 0x0084);
                assert_eq!(entries[0].data, &[0xAA, 0xBB]);
                assert_eq!(entries[1].id, 0x02);
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
        InstructionWriter::<_, Crc>::new(&mut buf)
            .fast_sync_read(0x0084, 4, &ids)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FastSyncRead(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
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
                id: 1,
                addr: U16Le::from_u16(0x0084),
                length: U16Le::from_u16(4),
            },
            BulkReadEntry {
                id: 2,
                addr: U16Le::from_u16(0x0090),
                length: U16Le::from_u16(2),
            },
        ];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .fast_bulk_read(&entries)
            .unwrap();
        let mut dec: Decoder<64, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::FastBulkRead(p)) => {
                assert_eq!(p.header.header.id, BROADCAST_ID);
                assert_eq!(p.entries.len(), 2);
                assert_eq!(p.entries[0].id, 1);
                assert_eq!(p.entries[1].id, 2);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }

    #[test]
    fn instr_ext_dispatches_as_raw() {
        let mut buf = Buf::new();
        let params = [0xDE, 0xAD, 0xBE, 0xEF];
        InstructionWriter::<_, Crc>::new(&mut buf)
            .ext(0x0A, 0xE0, &params)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.id, 0x0A);
                assert_eq!(r.header.header.instruction.kind(), Instruction::Ext(0xE0));
                assert_eq!(r.params, &params);
            }
            other => panic!("expected Raw(Ext), got {other:?}"),
        }
    }

    // ── StatusWriter ──

    #[test]
    fn status_empty_round_trips() {
        let mut buf = Buf::new();
        StatusWriter::<_, Crc>::new(&mut buf)
            .empty(0x01, StatusError::OK)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x01);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &[]);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_empty_carries_error_byte() {
        let mut buf = Buf::new();
        let err = StatusError::from_byte(0x83);
        StatusWriter::<_, Crc>::new(&mut buf).empty(0x02, err).unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x02);
                assert_eq!(s.error(), err);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_ping_round_trips_through_interpret() {
        let mut buf = Buf::new();
        StatusWriter::<_, Crc>::new(&mut buf)
            .ping(0x03, StatusError::OK, 0x0203, 0x10)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Ping) {
                Status::Ping { id, error, status } => {
                    assert_eq!(id, 0x03);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(status.model.get(), 0x0203);
                    assert_eq!(status.fw_version, 0x10);
                }
                other => panic!("expected Status::Ping, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_read_round_trips() {
        let mut buf = Buf::new();
        let data = [0x10, 0x20, 0x30, 0x40];
        StatusWriter::<_, Crc>::new(&mut buf)
            .read(0x04, StatusError::OK, &data)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => match s.interpret(RequestKind::Read) {
                Status::Read { id, error, data: d } => {
                    assert_eq!(id, 0x04);
                    assert_eq!(error, StatusError::OK);
                    assert_eq!(d, &data);
                }
                other => panic!("expected Status::Read, got {other:?}"),
            },
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn status_ext_round_trips() {
        let mut buf = Buf::new();
        let payload = [0xCA, 0xFE, 0xBA, 0xBE];
        StatusWriter::<_, Crc>::new(&mut buf)
            .ext(0x05, StatusError::OK, &payload)
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        match dec.feed(&buf).0 {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x05);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &payload);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    // ── SlotWriter ──

    #[test]
    fn slot_only_round_trips() {
        // Single-slot frame: full Status header + 1 slot + local CRC.
        // length=4 payload per slot; packet_length = 3 + 1*(2+4) = 9.
        let mut buf = Buf::new();
        let slot = Slot {
            id: 7,
            error: StatusError::OK,
            data: &[0xAA, 0xBB, 0xCC, 0xDD],
        };
        SlotWriter::<_, Crc>::new(&mut buf).only(&slot, 9).unwrap();

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
        // Three slots, length=2 each. packet_length = 3 + 3*(2+2) = 15.
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
        let mut w = SlotWriter::<_, Crc>::new(&mut buf);
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
            0x00,                          // len = 15
            Instruction::Status.as_u8(),   // 0x55
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
        SlotWriter::<_, Crc>::new(&mut buf).middle(&slot).unwrap();
        assert_eq!(&buf[..], &[0x07, 0x05, 0x10, 0x20]);
    }
}
