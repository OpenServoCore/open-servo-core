//! Streaming DXL 2.0 packet decoder.
//!
//! Fed wire bytes one chunk at a time; emits one of:
//!   - [`Step::NeedMore`] — more bytes required
//!   - [`Step::Packet`]   — a typed [`Packet`] borrowing the decoder's buffer
//!   - [`Step::Resync`]   — frame failed validation (length, CRC, overflow)
//!
//! The decoder owns its accumulator; emitted [`Packet`] variants overlay
//! the (post-unstuff) buffer for the duration of the [`feed`] call. The
//! caller advances by the returned byte count and re-enters with the
//! next chunk; multi-packet input is handled across calls, not within a
//! single call.

#![allow(dead_code)]

use core::marker::PhantomData;
use core::mem::{MaybeUninit, offset_of, size_of};

use crate::instruction::*;
use crate::packet::*;
use crate::wire::{
    CRC_BYTES, CrcUmts, HEADER, PACKET_LEN_GUARD, PACKET_LEN_MIN, STUFFING_BYTE, STUFFING_TRIGGER,
};

const HDR_LEN_OFFSET: usize = offset_of!(Header, len);
const HDR_INSTR_OFFSET: usize = offset_of!(Header, instruction);
const HDR_SIZE: usize = size_of::<Header>();

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResyncKind {
    BadLength,
    BadCrc,
    Overflow,
}

#[derive(Debug)]
pub enum Step<'a> {
    NeedMore,
    Packet(Packet<'a>),
    Resync(ResyncKind),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Stage {
    Sync,
    Header,
    Payload,
    Crc,
    Done,
}

enum StepResult {
    Continue,
    PacketReady,
    Resync(ResyncKind),
}

pub struct Decoder<const M: usize, CRC: CrcUmts> {
    buf: [MaybeUninit<u8>; M],
    logical_n: u16,
    wire_n: u16,
    expected_wire_total: u16,
    sync_matched: u8,
    crc_running: u16,
    received_crc: [u8; 2],
    unstuff_last3: [u8; 3],
    stage: Stage,
    _crc: PhantomData<fn() -> CRC>,
}

impl<const M: usize, CRC: CrcUmts> Decoder<M, CRC> {
    pub const fn new() -> Self {
        Self {
            buf: [MaybeUninit::<u8>::uninit(); M],
            logical_n: 0,
            wire_n: 0,
            expected_wire_total: 0,
            sync_matched: 0,
            crc_running: 0,
            received_crc: [0; 2],
            unstuff_last3: [0; 3],
            stage: Stage::Sync,
            _crc: PhantomData,
        }
    }

    pub fn reset(&mut self) {
        self.logical_n = 0;
        self.wire_n = 0;
        self.expected_wire_total = 0;
        self.sync_matched = 0;
        self.crc_running = 0;
        self.received_crc = [0; 2];
        self.unstuff_last3 = [0; 3];
        self.stage = Stage::Sync;
    }

    pub fn feed<'s>(&'s mut self, chunk: &[u8]) -> (Step<'s>, usize) {
        if self.stage == Stage::Done {
            self.reset();
        }
        let mut consumed = 0;
        for &b in chunk {
            consumed += 1;
            match self.step(b) {
                StepResult::Continue => {}
                StepResult::PacketReady => return (Step::Packet(self.dispatch()), consumed),
                StepResult::Resync(k) => return (Step::Resync(k), consumed),
            }
        }
        (Step::NeedMore, consumed)
    }

    #[inline]
    fn write_byte(&mut self, offset: usize, b: u8) {
        debug_assert!(offset < M);
        // SAFETY: offset is bounded by M (debug_assert); writing a u8 into
        // a MaybeUninit<u8> slot is always sound.
        unsafe {
            let base = self.buf.as_mut_ptr() as *mut u8;
            base.add(offset).write(b);
        }
    }

    #[inline]
    fn read_byte(&self, offset: usize) -> u8 {
        debug_assert!(offset < M);
        // SAFETY: caller ensures `offset` has been written via write_byte
        // before this read (every read site is gated by an FSM stage
        // that guarantees the corresponding write happened).
        unsafe {
            let base = self.buf.as_ptr() as *const u8;
            *base.add(offset)
        }
    }

    fn step(&mut self, b: u8) -> StepResult {
        match self.stage {
            Stage::Sync => self.step_sync(b),
            Stage::Header => self.step_header(b),
            Stage::Payload => self.step_payload(b),
            Stage::Crc => self.step_crc(b),
            Stage::Done => unreachable!(),
        }
    }

    fn step_sync(&mut self, b: u8) -> StepResult {
        // KMP-style backoff for HEADER = FF FF FD 00 (failure function:
        // f[1]=0, f[2]=1, f[3]=0). On mismatch, fall back to the longest
        // proper prefix of HEADER that's still a suffix of the bytes
        // seen, so a `FF FF FF FD 00` run still locks onto the embedded
        // header.
        let m = self.sync_matched as usize;
        if b == HEADER[m] {
            self.write_byte(m, b);
            let new_m = m + 1;
            self.sync_matched = new_m as u8;
            if new_m == HEADER.len() {
                self.crc_running = CRC::accumulate(0, &HEADER);
                self.wire_n = HEADER.len() as u16;
                self.stage = Stage::Header;
            }
        } else {
            match m {
                2 if b == HEADER[0] => { /* keep matched=2; buf[0..2] already FF FF */ }
                3 if b == HEADER[0] => {
                    self.write_byte(0, HEADER[0]);
                    self.sync_matched = 1;
                }
                _ => {
                    if b == HEADER[0] {
                        self.write_byte(0, b);
                        self.sync_matched = 1;
                    } else {
                        self.sync_matched = 0;
                    }
                }
            }
        }
        StepResult::Continue
    }

    fn step_header(&mut self, b: u8) -> StepResult {
        let pos = self.wire_n as usize;
        self.write_byte(pos, b);
        self.crc_running = CRC::accumulate(self.crc_running, &[b]);
        self.wire_n += 1;

        if self.wire_n as usize == HDR_INSTR_OFFSET {
            let len = u16::from_le_bytes([
                self.read_byte(HDR_LEN_OFFSET),
                self.read_byte(HDR_LEN_OFFSET + 1),
            ]) as usize;
            if !(PACKET_LEN_MIN..=PACKET_LEN_GUARD).contains(&len) {
                self.reset();
                return StepResult::Resync(ResyncKind::BadLength);
            }
            // Bytes preceding the Length-counted region: HEADER + id + len_field.
            let expected = HDR_INSTR_OFFSET + len;
            if expected > M {
                self.reset();
                return StepResult::Resync(ResyncKind::Overflow);
            }
            self.expected_wire_total = expected as u16;
        } else if self.wire_n as usize == HDR_SIZE {
            let instr = self.read_byte(HDR_INSTR_OFFSET);
            self.logical_n = HDR_SIZE as u16;
            self.unstuff_last3 = [0, 0, instr];
            if self.wire_n as usize + CRC_BYTES == self.expected_wire_total as usize {
                self.stage = Stage::Crc;
            } else {
                self.stage = Stage::Payload;
            }
        }
        StepResult::Continue
    }

    fn step_payload(&mut self, b: u8) -> StepResult {
        self.crc_running = CRC::accumulate(self.crc_running, &[b]);
        self.wire_n += 1;

        let trigger = self.unstuff_last3 == STUFFING_TRIGGER && b == STUFFING_BYTE;
        if trigger {
            self.unstuff_last3 = [self.unstuff_last3[1], self.unstuff_last3[2], STUFFING_BYTE];
        } else {
            let off = self.logical_n as usize;
            if off >= M {
                self.reset();
                return StepResult::Resync(ResyncKind::Overflow);
            }
            self.write_byte(off, b);
            self.unstuff_last3 = [self.unstuff_last3[1], self.unstuff_last3[2], b];
            self.logical_n += 1;
        }

        if self.wire_n as usize + CRC_BYTES == self.expected_wire_total as usize {
            self.stage = Stage::Crc;
        }
        StepResult::Continue
    }

    fn step_crc(&mut self, b: u8) -> StepResult {
        let idx = self.wire_n as usize - (self.expected_wire_total as usize - CRC_BYTES);
        self.received_crc[idx] = b;
        self.wire_n += 1;
        if self.wire_n == self.expected_wire_total {
            let received = u16::from_le_bytes(self.received_crc);
            if self.crc_running != received {
                self.reset();
                return StepResult::Resync(ResyncKind::BadCrc);
            }
            self.stage = Stage::Done;
            return StepResult::PacketReady;
        }
        StepResult::Continue
    }

    /// Re-emit the packet from the decoder's accumulator without consuming
    /// or advancing state. Sound only after `feed` returned `Step::Packet`
    /// on this decoder; the underlying buffer is initialized for
    /// `logical_n` bytes by construction at that point. Lets callers reborrow
    /// the packet immutably after `feed`'s mutable borrow has been released.
    pub fn dispatch_packet<'s>(&'s self) -> Packet<'s> {
        self.dispatch()
    }

    fn dispatch<'s>(&'s self) -> Packet<'s> {
        let base = self.buf.as_ptr() as *const u8;
        let instr = self.read_byte(HDR_INSTR_OFFSET);
        let pl = self.logical_n as usize - HDR_SIZE;

        // SAFETY: each arm casts only when `pl` covers every byte in the
        // target overlay struct. The FSM has written `logical_n` bytes
        // starting at offset 0 (sync + id + len + instruction + unstuffed
        // params), so `buf[0..self.logical_n]` is fully initialized.
        // Every overlay struct has alignment 1, so casting at offset 0 is
        // always aligned. Variable-length tail slices stay within the
        // initialized region by construction.
        unsafe {
            match instr {
                INSTR_PING if pl == 0 => Packet::Ping(&*(base as *const PingPacket)),
                INSTR_READ if pl == 4 => Packet::Read(&*(base as *const ReadPacket)),
                INSTR_WRITE if pl >= 2 => Packet::Write(WritePacket {
                    header: &*(base as *const WriteHeader),
                    data: core::slice::from_raw_parts(base.add(size_of::<WriteHeader>()), pl - 2),
                }),
                INSTR_REG_WRITE if pl >= 2 => Packet::RegWrite(WritePacket {
                    header: &*(base as *const WriteHeader),
                    data: core::slice::from_raw_parts(base.add(size_of::<WriteHeader>()), pl - 2),
                }),
                INSTR_ACTION if pl == 0 => Packet::Action(&*(base as *const ActionPacket)),
                INSTR_REBOOT if pl == 0 => Packet::Reboot(&*(base as *const RebootPacket)),
                INSTR_FACTORY_RESET if pl == 1 => {
                    Packet::FactoryReset(&*(base as *const FactoryResetPacket))
                }
                INSTR_STATUS if pl >= 1 => Packet::Status(StatusPacket {
                    header: &*(base as *const StatusHeader),
                    params: core::slice::from_raw_parts(
                        base.add(size_of::<StatusHeader>()),
                        pl - 1,
                    ),
                }),
                INSTR_SYNC_READ if pl >= 4 => Packet::SyncRead(SyncReadPacket {
                    header: &*(base as *const SyncReadHeader),
                    ids: core::slice::from_raw_parts(base.add(size_of::<SyncReadHeader>()), pl - 4),
                }),
                INSTR_SYNC_WRITE if pl >= 4 => Packet::SyncWrite(SyncWritePacket {
                    header: &*(base as *const SyncWriteHeader),
                    body: core::slice::from_raw_parts(
                        base.add(size_of::<SyncWriteHeader>()),
                        pl - 4,
                    ),
                }),
                INSTR_BULK_READ => Packet::BulkRead(BulkReadPacket {
                    header: &*(base as *const BulkReadHeader),
                    entries: core::slice::from_raw_parts(
                        base.add(size_of::<BulkReadHeader>()) as *const BulkReadEntry,
                        pl / size_of::<BulkReadEntry>(),
                    ),
                }),
                INSTR_BULK_WRITE => Packet::BulkWrite(BulkWritePacket {
                    header: &*(base as *const BulkWriteHeader),
                    body: core::slice::from_raw_parts(base.add(size_of::<BulkWriteHeader>()), pl),
                }),
                INSTR_FAST_SYNC_READ if pl >= 4 => Packet::FastSyncRead(FastSyncReadPacket {
                    header: &*(base as *const FastSyncReadHeader),
                    ids: core::slice::from_raw_parts(
                        base.add(size_of::<FastSyncReadHeader>()),
                        pl - 4,
                    ),
                }),
                INSTR_FAST_BULK_READ => Packet::FastBulkRead(FastBulkReadPacket {
                    header: &*(base as *const FastBulkReadHeader),
                    entries: core::slice::from_raw_parts(
                        base.add(size_of::<FastBulkReadHeader>()) as *const BulkReadEntry,
                        pl / size_of::<BulkReadEntry>(),
                    ),
                }),
                _ => Packet::Raw(RawPacket {
                    header: &*(base as *const RawHeader),
                    params: core::slice::from_raw_parts(base.add(HDR_SIZE), pl),
                }),
            }
        }
    }
}

impl<const M: usize, CRC: CrcUmts> Default for Decoder<M, CRC> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{RawFrame, SoftwareCrcUmts, write_raw};
    use heapless::Vec;

    type Crc = SoftwareCrcUmts;
    type Buf = Vec<u8, 256>;

    fn encode(id: u8, instruction: u8, params: &[u8]) -> Buf {
        let mut out = Buf::new();
        write_raw::<_, _, Crc>(
            &mut out,
            RawFrame {
                id,
                instruction,
                params: params.iter().copied(),
            },
        )
        .unwrap();
        out
    }

    #[test]
    fn ping_single_chunk() {
        let frame = encode(0x01, INSTR_PING, &[]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(&frame);
        assert_eq!(n, frame.len());
        match step {
            Step::Packet(Packet::Ping(p)) => {
                assert_eq!(p.header.id, 0x01);
                assert_eq!(p.header.instruction, INSTR_PING);
                assert_eq!(p.header.sync, [0xFF, 0xFF, 0xFD, 0x00]);
                assert_eq!(p.header.len.get(), 3);
            }
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn ping_byte_at_a_time() {
        let frame = encode(0x07, INSTR_PING, &[]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let last = frame.len() - 1;
        for (i, &b) in frame.iter().enumerate().take(last) {
            let (step, n) = dec.feed(&[b]);
            assert_eq!(n, 1);
            assert!(
                matches!(step, Step::NeedMore),
                "early termination at byte {i}"
            );
        }
        let (step, n) = dec.feed(&[frame[last]]);
        assert_eq!(n, 1);
        match step {
            Step::Packet(Packet::Ping(p)) => assert_eq!(p.header.id, 0x07),
            other => panic!("expected Ping, got {other:?}"),
        }
    }

    #[test]
    fn header_two_byte_chunks() {
        let frame = encode(0x02, INSTR_READ, &[0x84, 0x00, 0x04, 0x00]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let n_chunks = frame.len().div_ceil(2);
        let mut completed = false;
        for (i, c) in frame.chunks(2).enumerate() {
            let (step, n) = dec.feed(c);
            assert_eq!(n, c.len());
            match step {
                Step::NeedMore => assert!(i + 1 < n_chunks),
                Step::Packet(Packet::Read(p)) => {
                    completed = true;
                    assert_eq!(p.header.id, 0x02);
                    assert_eq!(p.addr.get(), 0x0084);
                    assert_eq!(p.length.get(), 0x0004);
                }
                other => panic!("unexpected step at chunk {i}: {other:?}"),
            }
        }
        assert!(completed);
    }

    #[test]
    fn write_with_data() {
        let payload = [0x84, 0x00, 0xAA, 0xBB, 0xCC, 0xDD];
        let frame = encode(0x03, INSTR_WRITE, &payload);
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::Write(w)) => {
                assert_eq!(w.header.header.id, 0x03);
                assert_eq!(w.header.addr.get(), 0x0084);
                assert_eq!(w.data, &[0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }

    #[test]
    fn stuffing_escape_inside_payload() {
        // Logical payload contains the FF FF FD trigger; the writer
        // inserts a stuffing FD that the decoder must strip.
        let payload = [0x84, 0x00, 0xFF, 0xFF, 0xFD, 0x42];
        let frame = encode(0x04, INSTR_WRITE, &payload);
        // Frame should contain the stuffing byte on the wire.
        assert!(
            frame.windows(4).any(|w| w == [0xFF, 0xFF, 0xFD, 0xFD]),
            "expected stuffing byte in wire frame, got {frame:02X?}",
        );
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, n) = dec.feed(&frame);
        assert_eq!(n, frame.len());
        match step {
            Step::Packet(Packet::Write(w)) => {
                assert_eq!(w.data, &[0xFF, 0xFF, 0xFD, 0x42]);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }

    #[test]
    fn status_response_dispatch() {
        let frame = encode(0x05, INSTR_STATUS, &[0x00, 0xAA, 0xBB]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::Status(s)) => {
                assert_eq!(s.header.header.id, 0x05);
                assert_eq!(s.error(), StatusError::OK);
                assert_eq!(s.params, &[0xAA, 0xBB]);
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn factory_reset_dispatch() {
        let frame = encode(0x06, INSTR_FACTORY_RESET, &[0xFF]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::FactoryReset(p)) => assert_eq!(p.mode, 0xFF),
            other => panic!("expected FactoryReset, got {other:?}"),
        }
    }

    #[test]
    fn action_and_reboot() {
        for (instr, name) in [(INSTR_ACTION, "Action"), (INSTR_REBOOT, "Reboot")] {
            let frame = encode(0x08, instr, &[]);
            let mut dec: Decoder<32, Crc> = Decoder::new();
            let (step, _) = dec.feed(&frame);
            match step {
                Step::Packet(Packet::Action(_)) if instr == INSTR_ACTION => {}
                Step::Packet(Packet::Reboot(_)) if instr == INSTR_REBOOT => {}
                other => panic!("expected {name}, got {other:?}"),
            }
        }
    }

    #[test]
    fn sync_read_dispatch() {
        let frame = encode(
            0xFE,
            INSTR_SYNC_READ,
            &[0x84, 0x00, 0x04, 0x00, 0x01, 0x02, 0x03],
        );
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::SyncRead(sr)) => {
                assert_eq!(sr.header.addr.get(), 0x0084);
                assert_eq!(sr.header.length.get(), 0x0004);
                assert_eq!(sr.ids, &[0x01, 0x02, 0x03]);
            }
            other => panic!("expected SyncRead, got {other:?}"),
        }
    }

    #[test]
    fn sync_write_entries() {
        // addr=0x0080, length=2 → each entry is id(1) + 2 data bytes.
        let payload = [0x80, 0x00, 0x02, 0x00, 0x01, 0xAA, 0xBB, 0x02, 0xCC, 0xDD];
        let frame = encode(0xFE, INSTR_SYNC_WRITE, &payload);
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::SyncWrite(sw)) => {
                let entries: Vec<_, 4> = sw.entries().collect();
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
    fn bulk_read_dispatch() {
        // Two entries of (id, addr_le, len_le) = 5 bytes each.
        let payload = [0x01, 0x84, 0x00, 0x04, 0x00, 0x02, 0x90, 0x00, 0x02, 0x00];
        let frame = encode(0xFE, INSTR_BULK_READ, &payload);
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::BulkRead(br)) => {
                assert_eq!(br.entries.len(), 2);
                assert_eq!(br.entries[0].id, 0x01);
                assert_eq!(br.entries[0].addr.get(), 0x0084);
                assert_eq!(br.entries[0].length.get(), 0x0004);
                assert_eq!(br.entries[1].id, 0x02);
                assert_eq!(br.entries[1].addr.get(), 0x0090);
                assert_eq!(br.entries[1].length.get(), 0x0002);
            }
            other => panic!("expected BulkRead, got {other:?}"),
        }
    }

    #[test]
    fn bulk_write_entries() {
        // Two entries: (id, addr_le, len_le, ...data)
        let payload = [
            0x01, 0x84, 0x00, 0x02, 0x00, 0xAA, 0xBB, 0x02, 0x90, 0x00, 0x01, 0x00, 0xCC,
        ];
        let frame = encode(0xFE, INSTR_BULK_WRITE, &payload);
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::BulkWrite(bw)) => {
                let entries: Vec<_, 4> = bw.entries().collect();
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
    fn fast_sync_read_dispatch() {
        let frame = encode(
            0xFE,
            INSTR_FAST_SYNC_READ,
            &[0x84, 0x00, 0x04, 0x00, 0x01, 0x02],
        );
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::FastSyncRead(p)) => {
                assert_eq!(p.header.addr.get(), 0x0084);
                assert_eq!(p.header.length.get(), 0x0004);
                assert_eq!(p.ids, &[0x01, 0x02]);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        }
    }

    #[test]
    fn fast_bulk_read_dispatch() {
        let payload = [0x01, 0x84, 0x00, 0x04, 0x00, 0x02, 0x90, 0x00, 0x02, 0x00];
        let frame = encode(0xFE, INSTR_FAST_BULK_READ, &payload);
        let mut dec: Decoder<64, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::FastBulkRead(p)) => {
                assert_eq!(p.entries.len(), 2);
                assert_eq!(p.entries[0].id, 0x01);
                assert_eq!(p.entries[1].id, 0x02);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        }
    }

    #[test]
    fn raw_for_unknown_instruction() {
        let frame = encode(0x09, 0x7F, &[0xDE, 0xAD]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::Raw(r)) => {
                assert_eq!(r.header.header.instruction, 0x7F);
                assert_eq!(r.params, &[0xDE, 0xAD]);
            }
            other => panic!("expected Raw, got {other:?}"),
        }
    }

    #[test]
    fn bad_crc_resyncs() {
        let mut frame = encode(0x01, INSTR_PING, &[]);
        // Corrupt the last CRC byte.
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(&frame);
        assert_eq!(n, frame.len());
        assert!(matches!(step, Step::Resync(ResyncKind::BadCrc)));
        // After Resync the decoder is reset and a follow-up valid frame parses.
        let good = encode(0x02, INSTR_PING, &[]);
        let (step2, _) = dec.feed(&good);
        assert!(matches!(step2, Step::Packet(Packet::Ping(p)) if p.header.id == 0x02));
    }

    #[test]
    fn bad_length_resyncs() {
        // Length = 1 (below the 3 minimum).
        let bytes = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x01, 0x00];
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(&bytes);
        assert_eq!(n, 7);
        assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
    }

    #[test]
    fn overflow_resyncs_when_frame_exceeds_buffer() {
        // length = 100; M=16 — way too small.
        let len = 100u16.to_le_bytes();
        let bytes = [0xFF, 0xFF, 0xFD, 0x00, 0x01, len[0], len[1], INSTR_PING];
        let mut dec: Decoder<16, Crc> = Decoder::new();
        let (step, n) = dec.feed(&bytes);
        assert_eq!(n, 7);
        assert!(matches!(step, Step::Resync(ResyncKind::Overflow)));
    }

    #[test]
    fn payload_overflow_resyncs() {
        // A real frame whose unstuffed length exceeds M's logical room.
        // M=12 covers 8 header + 4 params; encode a 5-byte param.
        let frame = encode(0x01, INSTR_WRITE, &[0x84, 0x00, 0xAA, 0xBB, 0xCC]);
        let mut dec: Decoder<12, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        assert!(matches!(step, Step::Resync(ResyncKind::Overflow)));
    }

    #[test]
    fn back_to_back_frames_across_feeds() {
        let f1 = encode(0x01, INSTR_PING, &[]);
        let f2 = encode(0x02, INSTR_PING, &[]);
        let mut cat: Vec<u8, 64> = Vec::new();
        cat.extend_from_slice(&f1).unwrap();
        cat.extend_from_slice(&f2).unwrap();

        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (s1, n1) = dec.feed(&cat);
        assert_eq!(n1, f1.len());
        assert!(matches!(s1, Step::Packet(Packet::Ping(p)) if p.header.id == 0x01));

        let (s2, n2) = dec.feed(&cat[n1..]);
        assert_eq!(n2, f2.len());
        assert!(matches!(s2, Step::Packet(Packet::Ping(p)) if p.header.id == 0x02));
    }

    #[test]
    fn sync_finds_embedded_header_after_extra_ff() {
        // Prefix the frame with `FF FF` so the decoder's sync sees
        // `FF FF FF FD 00 ...` — the KMP-style backoff must lock onto the
        // embedded header starting at offset 1.
        let mut bytes: Vec<u8, 64> = Vec::new();
        bytes.extend_from_slice(&[0xFF, 0xFF]).unwrap();
        bytes
            .extend_from_slice(&encode(0x01, INSTR_PING, &[]))
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&bytes);
        assert!(matches!(step, Step::Packet(Packet::Ping(p)) if p.header.id == 0x01));
    }

    #[test]
    fn sync_skips_noise_then_locks_on() {
        let mut bytes: Vec<u8, 64> = Vec::new();
        bytes
            .extend_from_slice(&[0x00, 0x12, 0xFF, 0x34, 0xFF, 0xFD])
            .unwrap();
        bytes
            .extend_from_slice(&encode(0x09, INSTR_PING, &[]))
            .unwrap();
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&bytes);
        assert!(matches!(step, Step::Packet(Packet::Ping(p)) if p.header.id == 0x09));
    }

    #[test]
    fn reset_clears_in_progress_state() {
        let frame = encode(0x01, INSTR_PING, &[]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        // Feed only half the frame.
        let half = frame.len() / 2;
        let (step, _) = dec.feed(&frame[..half]);
        assert!(matches!(step, Step::NeedMore));
        dec.reset();
        // Now feed a full frame; should parse cleanly.
        let (step2, n) = dec.feed(&frame);
        assert_eq!(n, frame.len());
        assert!(matches!(step2, Step::Packet(Packet::Ping(_))));
    }

    #[test]
    fn status_interpret_ping_response() {
        // Slave's reply to PING: error=0, model=0x0203, fw_version=0x10.
        let frame = encode(0x01, INSTR_STATUS, &[0x00, 0x03, 0x02, 0x10]);
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, _) = dec.feed(&frame);
        match step {
            Step::Packet(Packet::Status(s)) => {
                let interpreted = s.interpret(RequestKind::Ping);
                match interpreted {
                    Status::Ping { id, error, status } => {
                        assert_eq!(id, 0x01);
                        assert_eq!(error, StatusError::OK);
                        assert_eq!(status.model.get(), 0x0203);
                        assert_eq!(status.fw_version, 0x10);
                    }
                    other => panic!("expected Ping status, got {other:?}"),
                }
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }
}
