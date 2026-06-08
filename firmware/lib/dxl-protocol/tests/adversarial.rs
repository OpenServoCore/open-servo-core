//! Streaming Decoder robustness: no panics, no wedge, rejects foreign
//! protocols, recovers real frames embedded in noise.

use dxl_protocol::decoder::{Decoder, ResyncKind, Step};
use dxl_protocol::packet as overlay;
use dxl_protocol::*;
use heapless::Vec as HVec;

type Crc = SoftwareCrcUmts;

/// Encode a legacy Packet to wire bytes. The streaming parser is what
/// changed; the writer still produces the same wire format from the
/// existing typed Packet shapes.
fn write_legacy<W: WriteBuf>(out: &mut W, p: &Packet<'_, NoInstructionExt>) -> Result<(), WriteError> {
    write_packet::<W, Crc, NoInstructionExt>(out, p)
}

fn write_ping(id: u8) -> HVec<u8, 32> {
    let mut v: HVec<u8, 32> = HVec::new();
    write_legacy(&mut v, &Packet::Ping(PingPacket { id })).unwrap();
    v
}

// xorshift64*.
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
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn next_byte(&mut self) -> u8 {
        self.next_u64() as u8
    }
    fn fill(&mut self, dst: &mut [u8]) {
        for b in dst.iter_mut() {
            *b = self.next_byte();
        }
    }
}

/// Feed `bytes` through a fresh Decoder in `chunk_size` slices, returning
/// the count of accepted packets. Asserts forward progress (every feed
/// consumes at least one byte) and termination.
fn drain_stream(bytes: &[u8], chunk_size: usize) -> usize {
    assert!(chunk_size >= 1);
    let mut dec: Decoder<2048, Crc> = Decoder::new();
    let mut idx = 0usize;
    let mut accepts = 0usize;
    let mut iter = 0usize;
    let max_iter = (bytes.len() + 64).saturating_mul(8).max(1024);

    while idx < bytes.len() {
        iter += 1;
        assert!(
            iter <= max_iter,
            "non-terminating loop after {iter} iterations on {} bytes",
            bytes.len()
        );
        let end = (idx + chunk_size).min(bytes.len());
        let chunk = &bytes[idx..end];
        let (step, n) = dec.feed(chunk);
        assert!(n >= 1, "feed must consume at least one byte from a non-empty chunk");
        assert!(n <= chunk.len(), "feed cannot consume past chunk end");
        idx += n;
        match step {
            Step::Packet(_) => accepts += 1,
            Step::NeedMore => {}
            Step::Resync(_) => {}
        }
    }
    accepts
}

#[test]
fn random_byte_stream_does_not_panic_or_wedge() {
    let mut rng = Rng::new(0xDEADBEEFu64);
    let mut bytes = vec![0u8; 1_000_000];
    rng.fill(&mut bytes);

    let accepts = drain_stream(&bytes, 64);
    // CRC-16 false-accept rate ~2^-16 conditioned on header+length+instruction;
    // expectation across 1MB is far below 1.
    assert!(
        accepts < 5,
        "unexpected false accepts in random data: {accepts}"
    );
}

#[test]
fn random_byte_stream_byte_at_a_time() {
    let mut rng = Rng::new(7);
    let mut bytes = vec![0u8; 100_000];
    rng.fill(&mut bytes);
    let accepts = drain_stream(&bytes, 1);
    assert!(accepts < 5, "byte-at-a-time false accepts: {accepts}");
}

#[test]
fn all_zero_stream_no_accepts() {
    let bytes = vec![0u8; 100_000];
    assert_eq!(drain_stream(&bytes, 64), 0);
}

#[test]
fn all_ff_stream_no_accepts() {
    let bytes = vec![0xFFu8; 100_000];
    assert_eq!(drain_stream(&bytes, 64), 0);
}

#[test]
fn repeated_header_pattern_no_accepts() {
    let mut bytes: Vec<u8> = Vec::new();
    for _ in 0..10_000 {
        bytes.extend_from_slice(&HEADER);
    }
    assert_eq!(drain_stream(&bytes, 64), 0);
}

#[test]
fn header_then_random_payload_does_not_wedge() {
    let mut rng = Rng::new(1);
    let mut stream: Vec<u8> = Vec::new();
    for _ in 0..1000 {
        stream.extend_from_slice(&HEADER);
        for _ in 0..64 {
            stream.push(rng.next_byte());
        }
    }
    let accepts = drain_stream(&stream, 64);
    assert!(accepts < 5, "phantom-header accepts: {accepts}");
}

#[test]
fn modbus_rtu_like_traffic_no_false_accept() {
    let mut rng = Rng::new(0xB0BA);
    let mut stream: Vec<u8> = Vec::new();
    for addr in 1..=247u8 {
        for func in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x17] {
            stream.push(addr);
            stream.push(func);
            for _ in 0..rng.next_u64() % 16 + 4 {
                stream.push(rng.next_byte() & 0x7F);
            }
            stream.push(rng.next_byte() & 0x7F);
            stream.push(rng.next_byte() & 0x7F);
        }
    }
    assert_eq!(drain_stream(&stream, 64), 0);
}

#[test]
fn ascii_text_traffic_no_false_accept() {
    // ASCII can't contain 0xFF/0xFD.
    let chunk = b"GET /index.html HTTP/1.1\r\nHost: example.com\r\n\r\n";
    let mut stream: Vec<u8> = Vec::new();
    for _ in 0..1000 {
        stream.extend_from_slice(chunk);
    }
    assert_eq!(drain_stream(&stream, 64), 0);
}

#[test]
fn real_frames_with_random_garbage_between_recover_all() {
    let mut rng = Rng::new(42);
    let mut stream: Vec<u8> = Vec::new();
    let real_count = 200;
    for i in 0..real_count {
        let garbage_len = (rng.next_u64() % 96) as usize;
        for _ in 0..garbage_len {
            // mask high bit so garbage can't synthesize valid frames
            stream.push(rng.next_byte() & 0x7F);
        }
        let frame = write_ping(((i % 253) + 1) as u8);
        stream.extend_from_slice(&frame);
    }
    let accepts = drain_stream(&stream, 64);
    assert_eq!(
        accepts, real_count,
        "expected to recover all {real_count} real frames, got {accepts}"
    );
}

#[test]
fn real_frames_byte_at_a_time_recover_all() {
    let mut rng = Rng::new(0xCAFEBABE);
    let mut stream: Vec<u8> = Vec::new();
    let real_count = 50;
    for i in 0..real_count {
        let garbage_len = (rng.next_u64() % 32) as usize;
        for _ in 0..garbage_len {
            stream.push(rng.next_byte() & 0x7F);
        }
        let frame = write_ping(((i % 253) + 1) as u8);
        stream.extend_from_slice(&frame);
    }
    let accepts = drain_stream(&stream, 1);
    assert_eq!(accepts, real_count);
}

#[test]
fn real_frames_with_partial_corrupt_frames_recover_clean_ones() {
    let mut rng = Rng::new(0x12345678);
    let mut stream: Vec<u8> = Vec::new();
    let valid = 100;
    let corrupt = 100;

    for i in 0..valid {
        stream.extend_from_slice(&write_ping(((i % 253) + 1) as u8));
    }
    for i in 0..corrupt {
        let mut frame = write_ping(((i % 253) + 1) as u8);
        let last = frame.len() - 1;
        frame[last] ^= rng.next_byte() | 1;
        stream.extend_from_slice(&frame);
    }

    let accepts = drain_stream(&stream, 64);
    assert_eq!(accepts, valid);
}

#[test]
fn truncated_real_frame_returns_needmore_at_each_step() {
    const PING: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];
    for n in 0..PING.len() {
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, consumed) = dec.feed(&PING[..n]);
        assert_eq!(consumed, n);
        assert!(
            matches!(step, Step::NeedMore),
            "truncation to {n} bytes: got {step:?}"
        );
    }
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, consumed) = dec.feed(PING);
    assert_eq!(consumed, PING.len());
    match step {
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, 1),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn header_byte_at_a_time() {
    // Streaming header bytes one at a time exercises the KMP sync FSM and
    // the wire_n boundary checks inside step_header.
    let frame = {
        let mut v: HVec<u8, 32> = HVec::new();
        write_legacy(
            &mut v,
            &Packet::Read(ReadPacket {
                id: 7,
                address: 0x0084,
                length: 4,
            }),
        )
        .unwrap();
        v
    };
    let mut dec: Decoder<64, Crc> = Decoder::new();
    let last = frame.len() - 1;
    for (i, &b) in frame.iter().enumerate().take(last) {
        let (step, n) = dec.feed(&[b]);
        assert_eq!(n, 1);
        assert!(matches!(step, Step::NeedMore), "byte {i}: {step:?}");
    }
    let (step, n) = dec.feed(&[frame[last]]);
    assert_eq!(n, 1);
    match step {
        Step::Packet(overlay::Packet::Read(p)) => {
            assert_eq!(p.header.id, 7);
            assert_eq!(p.addr.get(), 0x0084);
            assert_eq!(p.length.get(), 4);
        }
        other => panic!("expected Read, got {other:?}"),
    }
}

#[test]
fn random_header_id_length_does_not_lock_parser() {
    let mut rng = Rng::new(13);
    for _ in 0..500 {
        let len = (rng.next_u64() % 2048) as usize;
        let mut buf = vec![0u8; 7 + len + 2];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        let _ = drain_stream(&buf, 64);
    }
}

#[test]
fn maxlen_phantom_header_terminates() {
    // Length-field exactly at the guard cap: parser must reach a decision
    // (Packet / Resync) before the stream ends, not block.
    let mut rng = Rng::new(1234);
    for _ in 0..50 {
        let mut buf = vec![0u8; 7 + PACKET_LEN_GUARD];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        buf[5] = (PACKET_LEN_GUARD & 0xFF) as u8;
        buf[6] = ((PACKET_LEN_GUARD >> 8) & 0xFF) as u8;
        // drain_stream's max_iter / termination check is the real assertion.
        let _ = drain_stream(&buf, PACKET_LEN_GUARD + 16);
    }
}

#[test]
fn over_maxlen_header_short_circuits() {
    // Length > PACKET_LEN_GUARD must trigger BadLength on the byte that
    // completes the length field — not wait for ~64 KB of payload.
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&bad);
    assert_eq!(n, 7);
    assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
}

#[test]
fn writer_id_0xff_rejected() {
    let mut out: HVec<u8, 32> = HVec::new();
    let err = write_legacy(&mut out, &Packet::Ping(PingPacket { id: 0xFF })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
}

#[test]
fn writer_overflow_reported_and_rolled_back() {
    let mut tiny: HVec<u8, 5> = HVec::new();
    let err = write_legacy(&mut tiny, &Packet::Ping(PingPacket { id: 1 })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert!(
        tiny.is_empty(),
        "buffer should be rolled back on overflow, has {} bytes",
        tiny.len()
    );
}

#[test]
fn writer_overflow_preserves_prior_frames() {
    // DMA TX buffer holds a frame; second write overflows — first must survive
    // and re-decode cleanly through the new Decoder.
    let mut buf: HVec<u8, 16> = HVec::new();
    write_legacy(&mut buf, &Packet::Ping(PingPacket { id: 1 })).unwrap();
    let snapshot: HVec<u8, 16> = buf.clone();
    let pre_len = buf.len();

    let big = [0u8; 32];
    let err = write_legacy(
        &mut buf,
        &Packet::Status(RawStatus {
            id: 1,
            error: 0,
            params: Bytes::unstuffed(&big),
        }),
    )
    .unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert_eq!(buf.len(), pre_len, "buffer length changed on overflow");
    assert_eq!(&buf[..], &snapshot[..], "prior frame corrupted on overflow");

    let mut dec: Decoder<64, Crc> = Decoder::new();
    let (step, n) = dec.feed(&buf);
    assert_eq!(n, buf.len());
    match step {
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, 1),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn writer_invalid_id_does_not_touch_buffer() {
    let mut buf: HVec<u8, 32> = HVec::new();
    write_legacy(&mut buf, &Packet::Ping(PingPacket { id: 1 })).unwrap();
    let snapshot: HVec<u8, 32> = buf.clone();

    let err = write_legacy(&mut buf, &Packet::Ping(PingPacket { id: 0xFF })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
    assert_eq!(&buf[..], &snapshot[..]);
}

#[test]
fn pure_noise_followed_by_frame_recovers_frame() {
    // Noise without `FF` triggers stays in Sync silently; a valid frame
    // appended after surfaces as Step::Packet.
    let mut buf: Vec<u8> = vec![0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
    buf.extend_from_slice(&write_ping(1));
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&buf);
    assert_eq!(n, buf.len());
    match step {
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, 1),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn partial_header_prefix_yields_needmore() {
    for partial in [&[0xFFu8][..], &[0xFF, 0xFF][..], &[0xFF, 0xFF, 0xFD][..]] {
        let mut dec: Decoder<32, Crc> = Decoder::new();
        let (step, n) = dec.feed(partial);
        assert_eq!(n, partial.len());
        assert!(
            matches!(step, Step::NeedMore),
            "partial = {partial:02X?}: {step:?}"
        );
    }
}

/// Verify the decoded overlay variant matches the legacy `Packet` it was
/// encoded from. Field-by-field, no wire round-trip — the decoded slice
/// borrows into the Decoder, so we can't easily re-encode after the
/// feed call returns.
fn assert_overlay_matches(decoded: &overlay::Packet<'_>, legacy: &Packet<'_, NoInstructionExt>) {
    match (decoded, legacy) {
        (overlay::Packet::Ping(d), Packet::Ping(l)) => {
            assert_eq!(d.header.id, l.id);
        }
        (overlay::Packet::Read(d), Packet::Read(l)) => {
            assert_eq!(d.header.id, l.id);
            assert_eq!(d.addr.get(), l.address);
            assert_eq!(d.length.get(), l.length);
        }
        (overlay::Packet::Write(d), Packet::Write(l)) => {
            assert_eq!(d.header.header.id, l.id);
            assert_eq!(d.header.addr.get(), l.address);
            assert_bytes_eq(d.data, l.data);
        }
        (overlay::Packet::RegWrite(d), Packet::RegWrite(l)) => {
            assert_eq!(d.header.header.id, l.id);
            assert_eq!(d.header.addr.get(), l.address);
            assert_bytes_eq(d.data, l.data);
        }
        (overlay::Packet::Action(d), Packet::Action(l)) => assert_eq!(d.header.id, l.id),
        (overlay::Packet::Reboot(d), Packet::Reboot(l)) => assert_eq!(d.header.id, l.id),
        (overlay::Packet::FactoryReset(d), Packet::FactoryReset(l)) => {
            assert_eq!(d.header.id, l.id);
            assert_eq!(d.mode, l.mode);
        }
        (overlay::Packet::Status(d), Packet::Status(l)) => {
            assert_eq!(d.header.header.id, l.id);
            assert_eq!(d.error().as_byte(), l.error);
            assert_bytes_eq(d.params, l.params);
        }
        (overlay::Packet::SyncRead(d), Packet::SyncRead(l)) => {
            assert_eq!(d.header.addr.get(), l.address);
            assert_eq!(d.header.length.get(), l.length);
            assert_bytes_eq(d.ids, l.ids);
        }
        (overlay::Packet::SyncWrite(d), Packet::SyncWrite(l)) => {
            assert_eq!(d.header.addr.get(), l.address);
            assert_eq!(d.header.length.get(), l.length);
            assert_bytes_eq(d.body, l.body);
        }
        (overlay::Packet::FastSyncRead(d), Packet::FastSyncRead(l)) => {
            assert_eq!(d.header.addr.get(), l.address);
            assert_eq!(d.header.length.get(), l.length);
            assert_bytes_eq(d.ids, l.ids);
        }
        (overlay::Packet::BulkWrite(d), Packet::BulkWrite(l)) => {
            assert_bytes_eq(d.body, l.body);
        }
        // BulkRead/FastBulkRead overlays expose typed entries (5-byte
        // stride); trailing partial bytes in the legacy body are dropped
        // when the cast lands at a non-aligned length.
        (overlay::Packet::BulkRead(d), Packet::BulkRead(l)) => {
            let stride = core::mem::size_of::<overlay::BulkReadEntry>();
            assert_eq!(d.entries.len(), l.body.unstuffed_len() / stride);
        }
        (overlay::Packet::FastBulkRead(d), Packet::FastBulkRead(l)) => {
            let stride = core::mem::size_of::<overlay::BulkReadEntry>();
            assert_eq!(d.entries.len(), l.body.unstuffed_len() / stride);
        }
        (d, l) => panic!("variant mismatch: decoded={d:?} legacy={l:?}"),
    }
}

fn assert_bytes_eq(decoded: &[u8], legacy: Bytes<'_>) {
    assert_eq!(decoded.len(), legacy.unstuffed_len());
    let mut buf = [0u8; 256];
    let n = legacy.copy_to_slice(&mut buf).expect("copy_to_slice");
    assert_eq!(decoded, &buf[..n]);
}

fn feed_full<'a, const M: usize>(
    dec: &'a mut Decoder<M, Crc>,
    wire: &[u8],
) -> overlay::Packet<'a> {
    let (step, n) = dec.feed(wire);
    assert_eq!(n, wire.len(), "decoder didn't consume the full frame");
    match step {
        Step::Packet(p) => p,
        other => panic!("expected Packet, got {other:?}"),
    }
}

#[test]
fn round_trip_every_instruction() {
    let cases: &[Packet<'static>] = &[
        Packet::Ping(PingPacket { id: 1 }),
        Packet::Read(ReadPacket {
            id: 1,
            address: 132,
            length: 4,
        }),
        Packet::Write(WritePacket {
            id: 1,
            address: 116,
            data: Bytes::unstuffed(&[0, 2, 0, 0]),
        }),
        Packet::RegWrite(RegWritePacket {
            id: 1,
            address: 100,
            data: Bytes::unstuffed(&[0xAA]),
        }),
        Packet::Action(ActionPacket { id: 1 }),
        Packet::FactoryReset(FactoryResetPacket { id: 1, mode: 0xFF }),
        Packet::Reboot(RebootPacket { id: 1 }),
        Packet::Status(RawStatus {
            id: 1,
            error: 0,
            params: Bytes::unstuffed(&[0xDE, 0xAD]),
        }),
        Packet::SyncRead(SyncReadPacket {
            address: 132,
            length: 4,
            ids: Bytes::unstuffed(&[1, 2, 3]),
        }),
        Packet::SyncWrite(SyncWritePacket {
            address: 116,
            length: 4,
            body: Bytes::unstuffed(&[1, 0, 1, 0, 0]),
        }),
        Packet::FastSyncRead(FastSyncReadPacket {
            address: 132,
            length: 4,
            ids: Bytes::unstuffed(&[1, 2]),
        }),
        Packet::BulkRead(BulkReadPacket {
            body: Bytes::unstuffed(&[1, 0x84, 0x00, 0x04, 0x00]),
        }),
        Packet::BulkWrite(BulkWritePacket {
            body: Bytes::unstuffed(&[1, 2, 3]),
        }),
        Packet::FastBulkRead(FastBulkReadPacket {
            body: Bytes::unstuffed(&[1, 0x84, 0x00, 0x04, 0x00]),
        }),
    ];

    for case in cases {
        let mut wire: HVec<u8, 128> = HVec::new();
        write_legacy(&mut wire, case).unwrap();
        let mut dec: Decoder<256, Crc> = Decoder::new();
        let decoded = feed_full(&mut dec, &wire);
        assert_overlay_matches(&decoded, case);
    }
}

#[test]
fn round_trip_random_fields_across_variants() {
    let mut rng = Rng::new(0xFA1AFE1);
    const ITERS: usize = 2000;

    for _ in 0..ITERS {
        let mut body: HVec<u8, 96> = HVec::new();
        let mut ids: HVec<u8, 32> = HVec::new();

        // Variant kinds covered by the new typed Packet enum (legacy chip
        // extensions like Clear/ControlTableBackup route to Packet::Raw and
        // are exercised separately).
        let kind = rng.next_byte() % 14;
        let id = match rng.next_byte() {
            0xFF => 1,
            b => b,
        };
        let addr = (rng.next_u64() & 0xFFFF) as u16;
        let length = (rng.next_u64() & 0xFFFF) as u16;
        let mode = rng.next_byte();
        let error = rng.next_byte();

        let body_target = (rng.next_u64() % 24) as usize;
        while body.len() < body_target {
            if rng.next_byte() < 32 && body.len() + 3 <= body.capacity() {
                body.extend_from_slice(&[0xFF, 0xFF, 0xFD]).unwrap();
            } else {
                body.push(rng.next_byte()).unwrap();
            }
        }
        let ids_target = (rng.next_u64() % 8) as usize;
        for _ in 0..ids_target {
            ids.push(rng.next_byte() % 254).unwrap();
        }

        let pkt = match kind {
            0 => Packet::Ping(PingPacket { id }),
            1 => Packet::Read(ReadPacket {
                id,
                address: addr,
                length,
            }),
            2 => Packet::Write(WritePacket {
                id,
                address: addr,
                data: Bytes::unstuffed(&body),
            }),
            3 => Packet::RegWrite(RegWritePacket {
                id,
                address: addr,
                data: Bytes::unstuffed(&body),
            }),
            4 => Packet::Action(ActionPacket { id }),
            5 => Packet::FactoryReset(FactoryResetPacket { id, mode }),
            6 => Packet::Reboot(RebootPacket { id }),
            7 => Packet::Status(RawStatus {
                id,
                error,
                params: Bytes::unstuffed(&body),
            }),
            8 => Packet::SyncRead(SyncReadPacket {
                address: addr,
                length,
                ids: Bytes::unstuffed(&ids),
            }),
            9 => Packet::SyncWrite(SyncWritePacket {
                address: addr,
                length,
                body: Bytes::unstuffed(&body),
            }),
            10 => Packet::FastSyncRead(FastSyncReadPacket {
                address: addr,
                length,
                ids: Bytes::unstuffed(&ids),
            }),
            11 => Packet::BulkRead(BulkReadPacket {
                body: Bytes::unstuffed(&body),
            }),
            12 => Packet::BulkWrite(BulkWritePacket {
                body: Bytes::unstuffed(&body),
            }),
            _ => Packet::FastBulkRead(FastBulkReadPacket {
                body: Bytes::unstuffed(&body),
            }),
        };

        let mut wire: HVec<u8, 256> = HVec::new();
        write_legacy(&mut wire, &pkt).expect("write");
        let mut dec: Decoder<512, Crc> = Decoder::new();
        let decoded = feed_full(&mut dec, &wire);
        assert_overlay_matches(&decoded, &pkt);
    }
}

#[test]
fn stuffing_round_trip_for_every_body_carrying_variant() {
    let logical: &[u8] = &[0xFF, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xAA];

    let cases: &[(&str, Packet<'_>)] = &[
        (
            "Write",
            Packet::Write(WritePacket {
                id: 1,
                address: 0x0010,
                data: Bytes::unstuffed(logical),
            }),
        ),
        (
            "RegWrite",
            Packet::RegWrite(RegWritePacket {
                id: 1,
                address: 0x0010,
                data: Bytes::unstuffed(logical),
            }),
        ),
        (
            "Status",
            Packet::Status(RawStatus {
                id: 1,
                error: 0,
                params: Bytes::unstuffed(logical),
            }),
        ),
        (
            "SyncWrite",
            Packet::SyncWrite(SyncWritePacket {
                address: 0x0010,
                length: 4,
                body: Bytes::unstuffed(logical),
            }),
        ),
        (
            "BulkWrite",
            Packet::BulkWrite(BulkWritePacket {
                body: Bytes::unstuffed(logical),
            }),
        ),
    ];

    for (name, pkt) in cases {
        let mut wire: HVec<u8, 64> = HVec::new();
        write_legacy(&mut wire, pkt).unwrap_or_else(|e| panic!("{name}: write failed: {e:?}"));

        // ≥2 extra FDs expected (one per trigger).
        let payload = &wire[8..wire.len() - 2];
        let logical_fds = logical.iter().filter(|&&b| b == 0xFD).count();
        let wire_fds = payload.iter().filter(|&&b| b == 0xFD).count();
        assert!(
            wire_fds >= logical_fds + 2,
            "{name}: expected stuffing on wire, got {wire_fds} 0xFD bytes for {logical_fds} logical"
        );

        let mut dec: Decoder<128, Crc> = Decoder::new();
        let decoded = feed_full(&mut dec, &wire);
        let recovered: &[u8] = match decoded {
            overlay::Packet::Write(p) => p.data,
            overlay::Packet::RegWrite(p) => p.data,
            overlay::Packet::Status(p) => p.params,
            overlay::Packet::SyncWrite(p) => p.body,
            overlay::Packet::BulkWrite(p) => p.body,
            other => panic!("{name}: parsed into unexpected variant {other:?}"),
        };
        assert_eq!(recovered, logical, "{name}: payload mismatch after unstuffing");
    }
}
