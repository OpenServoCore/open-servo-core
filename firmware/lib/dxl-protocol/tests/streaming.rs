//! [`Parser`] end-to-end across multi-packet streams, byte-at-a-time
//! chunking, recovery gestures, and adversarial input. The in-source unit
//! tests stay focused on per-stage transitions.

use dxl_protocol::streaming::{
    Event, HeaderEvent, InstructionHeader, InstructionPayload, Parser, PayloadEvent, ResyncKind,
    StatusPayload,
};
use dxl_protocol::types::{Id, Instruction, StatusError};
use dxl_protocol::wire::HEADER;
use dxl_protocol::{CrcUmts, SoftwareCrcUmts};

type Crc = SoftwareCrcUmts;

/// xorshift64*.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn next_byte(&mut self) -> u8 {
        self.next_u64() as u8
    }
}

fn append_crc(buf: &mut Vec<u8>) {
    let mut c = Crc::new();
    c.update(buf);
    let crc = c.finalize();
    buf.push(crc as u8);
    buf.push((crc >> 8) as u8);
}

fn ping_packet(id: u8) -> Vec<u8> {
    let mut buf = vec![
        0xFF,
        0xFF,
        0xFD,
        0x00,
        id,
        0x03,
        0x00,
        Instruction::Ping.as_u8(),
    ];
    append_crc(&mut buf);
    buf
}

fn write_packet(id: u8, addr: u16, body: &[u8]) -> Vec<u8> {
    let wire_len = (1 + 2 + body.len() + 2) as u16;
    let mut buf = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    buf.extend_from_slice(&wire_len.to_le_bytes());
    buf.push(Instruction::Write.as_u8());
    buf.extend_from_slice(&addr.to_le_bytes());
    buf.extend_from_slice(body);
    append_crc(&mut buf);
    buf
}

fn status_packet(id: u8, error: u8, body: &[u8]) -> Vec<u8> {
    let wire_len = (1 + 1 + body.len() + 2) as u16;
    let mut buf = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    buf.extend_from_slice(&wire_len.to_le_bytes());
    buf.push(Instruction::Status.as_u8());
    buf.push(error);
    buf.extend_from_slice(body);
    append_crc(&mut buf);
    buf
}

/// Fixed-chunk drain that asserts forward progress and termination, so
/// adversarial inputs surface as panics here rather than test hangs.
fn drain_chunks(bytes: &[u8], chunk_size: usize) -> Vec<Event> {
    assert!(chunk_size >= 1);
    let mut p: Parser<Crc> = Parser::new();
    let mut events = Vec::new();
    let mut idx = 0usize;
    let max_iter = (bytes.len() + 64).saturating_mul(8).max(1024);
    let mut iter = 0usize;
    while idx < bytes.len() {
        iter += 1;
        assert!(iter <= max_iter, "non-terminating drain after {iter} iters");
        let end = (idx + chunk_size).min(bytes.len());
        let mut stream = p.feed(&bytes[idx..end]);
        let mut step_events = 0;
        for ev in &mut stream {
            events.push(ev);
            step_events += 1;
        }
        let consumed = stream.consumed();
        assert!(
            consumed >= 1 || step_events > 0,
            "feed must make forward progress (consumed={consumed} events={step_events})"
        );
        idx += consumed.max(1);
    }
    events
}

fn count_crc(events: &[Event]) -> usize {
    events.iter().filter(|e| matches!(e, Event::Crc)).count()
}

fn payload_bytes(events: &[Event]) -> u32 {
    events
        .iter()
        .map(|e| match e {
            Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
                length,
                ..
            })) => *length as u32,
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                length, ..
            })) => *length as u32,
            Event::Payload(PayloadEvent::Status(StatusPayload::FastReadDataChunk {
                length,
                ..
            })) => *length as u32,
            _ => 0,
        })
        .sum()
}

#[test]
fn status_reply_emits_sync_header_chunk_and_crc() {
    let body = [0xDE, 0xAD, 0xBE, 0xEF];
    let bytes = status_packet(0x07, 0x05, &body);
    let mut p: Parser<Crc> = Parser::new();
    let evs: Vec<Event> = p.feed(&bytes).collect();

    assert!(matches!(evs[0], Event::Sync));
    match evs[1] {
        Event::Header(HeaderEvent::Status(h)) => {
            assert_eq!(h.id, Id::new(0x07));
            assert_eq!(h.error, StatusError::from_byte(0x05));
            assert_eq!(h.length, body.len() as u16);
        }
        ref other => panic!("expected Status header, got {other:?}"),
    }
    match evs[2] {
        Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk { offset, length })) => {
            assert_eq!(length as usize, body.len());
            assert_eq!(&bytes[offset as usize..(offset + length) as usize], &body);
        }
        ref other => panic!("expected ReadDataChunk, got {other:?}"),
    }
    assert_eq!(evs[3], Event::Crc);
}

#[test]
fn back_to_back_mixed_packets_each_yield_full_event_sequence() {
    let mut stream = Vec::new();
    stream.extend(ping_packet(0x01));
    stream.extend(write_packet(0x02, 0x0050, &[0xAA, 0xBB]));
    stream.extend(status_packet(0x03, 0x00, &[0x11, 0x22, 0x33]));

    let mut p: Parser<Crc> = Parser::new();
    let evs: Vec<Event> = p.feed(&stream).collect();

    let syncs = evs.iter().filter(|e| matches!(e, Event::Sync)).count();
    let headers = evs.iter().filter(|e| matches!(e, Event::Header(_))).count();
    assert_eq!(syncs, 3);
    assert_eq!(headers, 3);
    assert_eq!(count_crc(&evs), 3);
}

/// Chunk events carry offsets into the call's own input slice, not the
/// packet, so byte-at-a-time and whole-feed don't emit byte-equal chunk
/// events. Framing shape and total payload byte count must still match.
#[test]
fn byte_at_a_time_preserves_framing_and_total_payload_length() {
    let cases: &[Vec<u8>] = &[
        ping_packet(0x09),
        write_packet(0x0A, 0x0080, &[1, 2, 3, 4, 5, 6, 7, 8]),
        status_packet(0x0B, 0x00, &[0xAA, 0xBB, 0xCC, 0xDD]),
    ];
    for wire in cases {
        let mut whole: Parser<Crc> = Parser::new();
        let baseline: Vec<Event> = whole.feed(wire).collect();
        let chunked = drain_chunks(wire, 1);

        for (label, evs) in [("whole", &baseline), ("byte-at-a-time", &chunked)] {
            assert_eq!(
                evs.iter().filter(|e| matches!(e, Event::Sync)).count(),
                1,
                "{label}"
            );
            assert_eq!(
                evs.iter().filter(|e| matches!(e, Event::Header(_))).count(),
                1,
                "{label}"
            );
            assert_eq!(count_crc(evs), 1, "{label}");
        }
        assert_eq!(
            payload_bytes(&baseline),
            payload_bytes(&chunked),
            "byte-at-a-time payload-byte total diverged"
        );
    }
}

#[test]
fn chunked_status_chunks_sum_to_body_length() {
    let body: Vec<u8> = (0..32u8).collect();
    let bytes = status_packet(0x05, 0x00, &body);
    for chunk_size in [1usize, 3, 5, 7, 11, 16, bytes.len()] {
        let evs = drain_chunks(&bytes, chunk_size);
        let total: u16 = evs
            .iter()
            .filter_map(|e| match e {
                Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                    length,
                    ..
                })) => Some(*length),
                _ => None,
            })
            .sum();
        assert_eq!(
            total as usize,
            body.len(),
            "chunk_size={chunk_size}: chunks sum {total} != body {}",
            body.len()
        );
        assert_eq!(count_crc(&evs), 1, "chunk_size={chunk_size}");
    }
}

#[test]
fn real_frames_with_random_garbage_between_recover_all() {
    let mut rng = Rng::new(0xDEAD_BEEF);
    let mut stream = Vec::new();
    let real_count = 200usize;
    for i in 0..real_count {
        let garbage_len = (rng.next_u64() % 96) as usize;
        for _ in 0..garbage_len {
            // Top-bit-masked: no garbage byte can be 0xFF or 0xFD, so the
            // noise can't synthesize the FF FF FD 00 header pattern.
            stream.push(rng.next_byte() & 0x7F);
        }
        stream.extend(ping_packet(((i % 253) + 1) as u8));
    }
    let evs = drain_chunks(&stream, 64);
    let ping_count = evs
        .iter()
        .filter(|e| {
            matches!(
                e,
                Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { .. }))
            )
        })
        .count();
    assert_eq!(ping_count, real_count);
    assert_eq!(count_crc(&evs), real_count);
}

#[test]
fn mid_packet_reset_then_clean_packet_parses() {
    let dropped = write_packet(0x03, 0x0050, &[0x11, 0x22, 0x33, 0x44]);
    let mid = dropped.len() / 2;
    let mut p: Parser<Crc> = Parser::new();
    let _ = p.feed(&dropped[..mid]).count();
    p.reset();

    let next = ping_packet(0x07);
    let evs: Vec<Event> = p.feed(&next).collect();
    assert!(matches!(evs.first(), Some(Event::Sync)));
    assert!(evs.iter().any(|e| matches!(
        e,
        Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id }))
            if *id == Id::new(0x07)
    )));
    assert_eq!(evs.last(), Some(&Event::Crc));
}

#[test]
fn parser_recovers_through_bad_crc_then_clean_packet() {
    let mut corrupted = write_packet(0x03, 0x0050, &[1, 2, 3, 4]);
    let last = corrupted.len() - 1;
    corrupted[last] ^= 0xFF;
    corrupted.extend(ping_packet(0x05));

    let mut p: Parser<Crc> = Parser::new();
    let evs: Vec<Event> = p.feed(&corrupted).collect();
    assert!(evs.contains(&Event::Resync(ResyncKind::BadCrc)));
    assert!(evs.iter().any(|e| matches!(
        e,
        Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id }))
            if *id == Id::new(0x05)
    )));
    assert_eq!(count_crc(&evs), 1);
}

#[test]
fn random_byte_stream_does_not_panic_or_wedge() {
    let mut rng = Rng::new(0xDEAD_BEEFu64);
    let mut bytes = vec![0u8; 1_000_000];
    for b in bytes.iter_mut() {
        *b = rng.next_byte();
    }
    let evs = drain_chunks(&bytes, 64);
    assert!(
        count_crc(&evs) < 5,
        "unexpected false-accept Crc events in random data"
    );
}

#[test]
fn random_byte_stream_byte_at_a_time() {
    let mut rng = Rng::new(7);
    let mut bytes = vec![0u8; 100_000];
    for b in bytes.iter_mut() {
        *b = rng.next_byte();
    }
    let evs = drain_chunks(&bytes, 1);
    assert!(count_crc(&evs) < 5, "byte-at-a-time false-accept Crc");
}

#[test]
fn all_zero_stream_no_crc_verdicts() {
    let bytes = vec![0u8; 100_000];
    let evs = drain_chunks(&bytes, 64);
    assert_eq!(count_crc(&evs), 0);
}

#[test]
fn all_ff_stream_no_crc_verdicts() {
    let bytes = vec![0xFFu8; 100_000];
    let evs = drain_chunks(&bytes, 64);
    assert_eq!(count_crc(&evs), 0);
}

#[test]
fn repeated_header_pattern_no_crc_verdicts() {
    let mut bytes = Vec::new();
    for _ in 0..10_000 {
        bytes.extend_from_slice(&HEADER);
    }
    let evs = drain_chunks(&bytes, 64);
    assert_eq!(count_crc(&evs), 0);
}
