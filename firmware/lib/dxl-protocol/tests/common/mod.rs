//! Shared helpers for the integration test binaries. Each `tests/*.rs` is
//! its own crate, so includes via `mod common;` and tolerates unused
//! helpers per crate.

#![allow(dead_code)]

use dxl_protocol::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, Parser, PayloadEvent, StatusHeader,
    StatusPayload,
};
use dxl_protocol::types::Instruction;
use dxl_protocol::{CrcUmts, SoftwareCrcUmts};

pub type Crc = SoftwareCrcUmts;

/// xorshift64*.
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

pub fn append_crc(buf: &mut Vec<u8>) {
    let crc = crc_oneshot(0, buf);
    buf.push(crc as u8);
    buf.push((crc >> 8) as u8);
}

pub fn ping_packet(id: u8) -> Vec<u8> {
    chained_packet(id, Instruction::Ping, &[])
}

pub fn write_packet(id: u8, addr: u16, body: &[u8]) -> Vec<u8> {
    let mut params = Vec::with_capacity(2 + body.len());
    params.extend_from_slice(&addr.to_le_bytes());
    params.extend_from_slice(body);
    chained_packet(id, Instruction::Write, &params)
}

pub fn status_packet(id: u8, error: u8, body: &[u8]) -> Vec<u8> {
    let mut params = Vec::with_capacity(1 + body.len());
    params.push(error);
    params.extend_from_slice(body);
    chained_packet(id, Instruction::Status, &params)
}

/// Header + id + LE length + instruction byte + caller-supplied params + CRC.
/// Does not stuff -- callers driving stuffing paths build through the encoder.
pub fn chained_packet(id: u8, instr: Instruction, params: &[u8]) -> Vec<u8> {
    let wire_len = (1 + params.len() + 2) as u16;
    let mut buf = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    buf.extend_from_slice(&wire_len.to_le_bytes());
    buf.push(instr.as_u8());
    buf.extend_from_slice(params);
    append_crc(&mut buf);
    buf
}

pub fn parse_events(wire: &[u8]) -> Vec<Event> {
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

/// Strip DXL 2.0 stuffing from `wire[start..end]`. Pre-warm the 3-byte
/// sliding window with the 3 wire bytes immediately before `start` so a
/// trigger straddling the address->body boundary unstuffs correctly.
pub fn unstuff_wire(wire: &[u8], start: usize, end: usize) -> Vec<u8> {
    let mut last3 = [0u8; 3];
    if start >= 3 {
        last3.copy_from_slice(&wire[start - 3..start]);
    }
    let mut out = Vec::with_capacity(end - start);
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

pub fn write_data_unstuffed(wire: &[u8], events: &[Event]) -> Vec<u8> {
    let chunks: Vec<(u16, u16)> = events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect();
    if chunks.is_empty() {
        return Vec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}

pub fn status_data_unstuffed(wire: &[u8], events: &[Event]) -> Vec<u8> {
    let chunks: Vec<(u16, u16)> = events
        .iter()
        .filter_map(|e| match e {
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => Some((*offset, *length)),
            _ => None,
        })
        .collect();
    if chunks.is_empty() {
        return Vec::new();
    }
    let start = chunks[0].0 as usize;
    let end = chunks.last().map(|(o, l)| (o + l) as usize).unwrap();
    unstuff_wire(wire, start, end)
}
