//! Shared helpers for parser/encoder tests. Exposed publicly under the
//! `test-util` feature so downstream test crates (e.g. `tests/*.rs` in this
//! crate, and future consumers once the crate breaks out) can reuse them
//! rather than redefining them.

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec as AVec;

use crate::crc::{CrcUmts, SoftwareCrcUmts};
use crate::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, Parser, PayloadEvent, StatusHeader,
    StatusPayload,
};
use crate::types::{Id, Instruction};

pub type Crc = SoftwareCrcUmts;

pub fn parse_events(wire: &[u8]) -> AVec<Event> {
    let mut p: Parser<Crc> = Parser::new();
    p.feed(wire).collect()
}

pub fn instruction_header(events: &[Event]) -> InstructionHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Instruction(h)) => Some(*h),
            _ => None,
        })
        .expect("expected instruction header event")
}

pub fn status_header(events: &[Event]) -> StatusHeader {
    events
        .iter()
        .find_map(|e| match e {
            Event::Header(HeaderEvent::Status(h)) => Some(*h),
            _ => None,
        })
        .expect("expected status header event")
}

pub fn assert_crc_good(events: &[Event]) {
    assert!(
        events
            .iter()
            .any(|e| matches!(e, Event::Crc(crate::streaming::CrcResult::Good))),
        "no Crc(Good) verdict in events: {events:?}"
    );
}

pub fn write_chunks(events: &[Event]) -> AVec<(u16, u16)> {
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

pub fn read_chunks(events: &[Event]) -> AVec<(u16, u16)> {
    events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect()
}

/// Concatenated `ReadDataChunk` bytes from `wire`. Assumes unstuffed input
/// (callers using stuffed payloads should pull `read_chunks` and unstuff).
pub fn read_data(wire: &[u8], events: &[Event]) -> AVec<u8> {
    let mut out = AVec::new();
    for (offset, length) in read_chunks(events) {
        out.extend_from_slice(&wire[offset as usize..(offset + length) as usize]);
    }
    out
}

pub fn sync_slots(events: &[Event]) -> AVec<(Id, u8)> {
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

pub fn bulk_slots(events: &[Event]) -> AVec<(Id, u8, u16, u16)> {
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

/// xorshift64*. Small deterministic RNG for adversarial-stream test cases.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn next_byte(&mut self) -> u8 {
        self.next_u64() as u8
    }
}

pub fn crc_oneshot(seed: u16, bytes: &[u8]) -> u16 {
    let mut c = SoftwareCrcUmts::new_with_state(seed);
    c.update(bytes);
    c.finalize()
}

pub fn append_crc(buf: &mut AVec<u8>) {
    let crc = crc_oneshot(0, buf);
    buf.push(crc as u8);
    buf.push((crc >> 8) as u8);
}

pub fn ping_packet(id: u8) -> AVec<u8> {
    chained_packet(id, Instruction::Ping, &[])
}

pub fn write_packet(id: u8, addr: u16, body: &[u8]) -> AVec<u8> {
    let mut params: AVec<u8> = AVec::with_capacity(2 + body.len());
    params.extend_from_slice(&addr.to_le_bytes());
    params.extend_from_slice(body);
    chained_packet(id, Instruction::Write, &params)
}

pub fn status_packet(id: u8, error: u8, body: &[u8]) -> AVec<u8> {
    let mut params: AVec<u8> = AVec::with_capacity(1 + body.len());
    params.push(error);
    params.extend_from_slice(body);
    chained_packet(id, Instruction::Status, &params)
}

/// Header + id + LE length + instruction byte + caller-supplied params + CRC.
/// Does not stuff -- callers driving stuffing paths build through the encoder.
pub fn chained_packet(id: u8, instr: Instruction, params: &[u8]) -> AVec<u8> {
    let wire_len = (1 + params.len() + 2) as u16;
    let mut buf: AVec<u8> = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    buf.extend_from_slice(&wire_len.to_le_bytes());
    buf.push(instr.as_u8());
    buf.extend_from_slice(params);
    append_crc(&mut buf);
    buf
}

/// Strip DXL 2.0 stuffing from `wire[start..end]`. Pre-warm the 3-byte
/// sliding window with the 3 wire bytes immediately before `start` so a
/// trigger straddling the address->body boundary unstuffs correctly.
pub fn unstuff_wire(wire: &[u8], start: usize, end: usize) -> AVec<u8> {
    let mut last3 = [0u8; 3];
    if start >= 3 {
        last3.copy_from_slice(&wire[start - 3..start]);
    }
    let mut out: AVec<u8> = AVec::with_capacity(end - start);
    for &b in &wire[start..end] {
        if last3 == [0xFF, 0xFF, 0xFD] && b == 0xFD {
            last3 = [last3[1], last3[2], b];
            continue;
        }
        out.push(b);
        last3 = [last3[1], last3[2], b];
    }
    out
}

/// Extract and unstuff `WriteDataChunk` payload bytes from a parsed wire.
pub fn write_data_unstuffed(wire: &[u8], events: &[Event]) -> AVec<u8> {
    let chunks = write_chunks(events);
    if chunks.is_empty() {
        return AVec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}

/// Extract and unstuff `ReadDataChunk` payload bytes from a parsed wire.
pub fn status_data_unstuffed(wire: &[u8], events: &[Event]) -> AVec<u8> {
    let chunks = read_chunks(events);
    if chunks.is_empty() {
        return AVec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}
