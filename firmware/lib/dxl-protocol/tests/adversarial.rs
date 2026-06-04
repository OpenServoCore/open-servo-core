//! Parser robustness: no panics, no wedge, rejects foreign protocols, recovers
//! real frames embedded in noise. Tests use `std`; parser/writer are `no_std`.

use dxl_protocol::prelude::*;
use heapless::Vec as HVec;

type Wire = Codec<SoftwareCrcUmts>;

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

/// Asserts progress invariant (every non-Incomplete error has `skip >= 1`)
/// and termination. Returns accepted frame count.
fn drain_stream(bytes: &[u8], chunk_size: usize) -> usize {
    assert!(chunk_size >= 1);
    let mut buf: Vec<u8> = Vec::new();
    let mut feed_idx = 0usize;
    let mut accepts = 0usize;
    let mut iter = 0usize;
    let max_iter = (bytes.len() + 64).saturating_mul(8).max(1024);

    loop {
        iter += 1;
        assert!(
            iter <= max_iter,
            "non-terminating loop after {iter} iterations on {} input bytes",
            bytes.len()
        );

        match Wire::parse_one(&buf) {
            Ok((_pkt, n)) => {
                accepts += 1;
                assert!(n >= 10, "min DXL frame is 10 bytes, got {n}");
                assert!(n <= buf.len());
                buf.drain(..n);
            }
            Err(ParseError::Incomplete) => {
                if feed_idx >= bytes.len() {
                    break;
                }
                let to_pull = (bytes.len() - feed_idx).min(chunk_size);
                buf.extend_from_slice(&bytes[feed_idx..feed_idx + to_pull]);
                feed_idx += to_pull;
            }
            Err(e) => {
                let skip = match e {
                    ParseError::Resync { skip }
                    | ParseError::BadCrc { skip }
                    | ParseError::BadLength { skip }
                    | ParseError::BadInstruction { skip } => skip,
                    ParseError::Incomplete => unreachable!(),
                };
                assert!(skip >= 1, "skip invariant violated: {e:?}");
                let skip = skip.min(buf.len());
                buf.drain(..skip);
            }
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
    // CRC-16 false-accept rate ~2^-16 (conditioned on header+length+instruction);
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
    // Modbus-RTU-ish frames; mask high bit so we never emit 0xFF.
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
        let mut frame: HVec<u8, 32> = HVec::new();
        Wire::write(
            &mut frame,
            &Packet::Ping(PingPacket {
                id: ((i % 253) + 1) as u8,
            }),
        )
        .unwrap();
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
        let mut frame: HVec<u8, 32> = HVec::new();
        Wire::write(
            &mut frame,
            &Packet::Ping(PingPacket {
                id: ((i % 253) + 1) as u8,
            }),
        )
        .unwrap();
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
        let mut frame: HVec<u8, 32> = HVec::new();
        Wire::write(
            &mut frame,
            &Packet::Ping(PingPacket {
                id: ((i % 253) + 1) as u8,
            }),
        )
        .unwrap();
        stream.extend_from_slice(&frame);
    }
    for i in 0..corrupt {
        let mut frame: HVec<u8, 32> = HVec::new();
        Wire::write(
            &mut frame,
            &Packet::Ping(PingPacket {
                id: ((i % 253) + 1) as u8,
            }),
        )
        .unwrap();
        let last = frame.len() - 1;
        frame[last] ^= rng.next_byte() | 1;
        stream.extend_from_slice(&frame);
    }

    let accepts = drain_stream(&stream, 64);
    assert_eq!(accepts, valid);
}

#[test]
fn parse_one_progress_invariant_random_short_inputs() {
    // parse_one must return Ok (n>=10), Incomplete, or err with skip>=1; never panic.
    let mut rng = Rng::new(0xC0FFEE);
    for _ in 0..50_000 {
        let len = (rng.next_u64() % 64) as usize;
        let mut buf = vec![0u8; len];
        rng.fill(&mut buf);
        match Wire::parse_one(&buf) {
            Ok((_, n)) => assert!(n >= 10),
            Err(ParseError::Incomplete) => {}
            Err(ParseError::Resync { skip })
            | Err(ParseError::BadCrc { skip })
            | Err(ParseError::BadLength { skip })
            | Err(ParseError::BadInstruction { skip }) => {
                assert!(skip >= 1);
            }
        }
    }
}

#[test]
fn parse_one_progress_invariant_long_random_inputs() {
    let mut rng = Rng::new(0xC0FFEE2);
    for _ in 0..2000 {
        let len = (rng.next_u64() % (MAX_LENGTH as u64 * 2 + 64)) as usize;
        let mut buf = vec![0u8; len];
        rng.fill(&mut buf);
        match Wire::parse_one(&buf) {
            Ok((_, n)) => assert!(n >= 10 && n <= buf.len()),
            Err(ParseError::Incomplete) => {}
            Err(ParseError::Resync { skip })
            | Err(ParseError::BadCrc { skip })
            | Err(ParseError::BadLength { skip })
            | Err(ParseError::BadInstruction { skip }) => {
                assert!(skip >= 1);
            }
        }
    }
}

#[test]
fn truncated_real_frame_returns_incomplete_at_each_step() {
    const PING: &[u8] = &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];
    for n in 0..PING.len() {
        let r = Wire::parse_one(&PING[..n]);
        assert!(
            matches!(r, Err(ParseError::Incomplete)),
            "truncation to {n} bytes: got {r:?}"
        );
    }
    let (pkt, n) = Wire::parse_one(PING).unwrap();
    assert_eq!(n, PING.len());
    assert!(matches!(pkt, Packet::Ping(PingPacket { id: 1 })));
}

#[test]
fn parse_one_skips_phantom_broadcast_status_header_to_avoid_wedge() {
    // Fast First/Only chain replies use BROADCAST_ID + INSTR_STATUS with a
    // length field covering the WHOLE multi-slot reply. When such a header
    // shows up incomplete in the local RX (e.g. only the first slave's
    // contribution landed), the bytes that would complete it never arrive
    // at this offset — they're on the wire during another node's TX. The
    // parser must resync past the phantom header rather than returning
    // Incomplete and wedging the caller's poll loop.
    let frame: [u8; 14] = [
        0xFF, 0xFF, 0xFD, 0x00, // header
        0xFE, // BROADCAST_ID
        0x2B, 0x00, // length = 43 (whole multi-slot packet)
        0x55, // INSTR_STATUS
        0x00, 50, 0xAA, 0xAA, 0xAA, 0xAA, // first slot's body
    ];
    assert!(matches!(
        Wire::parse_one(&frame),
        Err(ParseError::BadInstruction { skip: 4 }),
    ));
}

#[test]
fn parse_one_returns_incomplete_on_truncated_broadcast_non_status() {
    // Regression: the phantom-header guard is specific to BROADCAST_ID +
    // INSTR_STATUS. A real broadcast Write (or any other broadcast frame)
    // that's just truncated mid-arrival must still return Incomplete so
    // the caller waits for the remaining bytes instead of dropping them.
    const BCAST_WRITE_TRUNCATED: &[u8] = &[
        0xFF, 0xFF, 0xFD, 0x00, // header
        0xFE, // BROADCAST_ID
        0x08, 0x00, // length = 8
        0x03, // INSTR_WRITE
        0x00, 0x00, // address = 0
        0xAA, 0xBB, // partial data — missing last byte + CRC
    ];
    assert!(matches!(
        Wire::parse_one(BCAST_WRITE_TRUNCATED),
        Err(ParseError::Incomplete),
    ));
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
fn maxlen_phantom_header_returns_definite_error() {
    // 7+MAX_LENGTH bytes with header and length=MAX_LENGTH: enough to verify CRC, reject.
    let mut rng = Rng::new(1234);
    for _ in 0..200 {
        let mut buf = vec![0u8; 7 + MAX_LENGTH];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        buf[5] = (MAX_LENGTH & 0xFF) as u8;
        buf[6] = ((MAX_LENGTH >> 8) & 0xFF) as u8;
        if let Err(ParseError::Incomplete) = Wire::parse_one(&buf) {
            panic!("should not be Incomplete with full frame");
        }
    }
}

#[test]
fn over_maxlen_header_short_circuits() {
    // Length > MAX_LENGTH must trigger BadLength immediately, not wait for ~64KB.
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    match Wire::parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {other:?}"),
    }
}

#[test]
fn writer_id_0xff_rejected() {
    let mut out: HVec<u8, 32> = HVec::new();
    let err = Wire::write(&mut out, &Packet::Ping(PingPacket { id: 0xFF })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
}

#[test]
fn writer_overflow_reported_and_rolled_back() {
    let mut tiny: HVec<u8, 5> = HVec::new();
    let err = Wire::write(&mut tiny, &Packet::Ping(PingPacket { id: 1 })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert!(
        tiny.is_empty(),
        "buffer should be rolled back on overflow, has {} bytes",
        tiny.len()
    );
}

#[test]
fn writer_overflow_preserves_prior_frames() {
    // DMA TX buffer holds a frame; second write overflows — first must survive.
    let mut buf: HVec<u8, 16> = HVec::new();
    Wire::write(&mut buf, &Packet::Ping(PingPacket { id: 1 })).unwrap();
    let snapshot: HVec<u8, 16> = buf.clone();
    let pre_len = buf.len();

    let big = [0u8; 32];
    let err = Wire::write(
        &mut buf,
        &Packet::Status(StatusPacket {
            id: 1,
            error: 0,
            params: Bytes::Raw(&big),
        }),
    )
    .unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert_eq!(buf.len(), pre_len, "buffer length changed on overflow");
    assert_eq!(&buf[..], &snapshot[..], "prior frame corrupted on overflow");

    let (pkt, n) = Wire::parse_one(&buf).unwrap();
    assert_eq!(n, buf.len());
    assert!(matches!(pkt, Packet::Ping(PingPacket { id: 1 })));
}

#[test]
fn writer_invalid_id_does_not_touch_buffer() {
    let mut buf: HVec<u8, 32> = HVec::new();
    Wire::write(&mut buf, &Packet::Ping(PingPacket { id: 1 })).unwrap();
    let snapshot: HVec<u8, 32> = buf.clone();

    let err = Wire::write(&mut buf, &Packet::Ping(PingPacket { id: 0xFF })).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
    assert_eq!(&buf[..], &snapshot[..]);
}

#[test]
fn no_header_skips_all_when_no_partial_prefix() {
    let buf = [0xAA, 0xBB, 0xCC, 0xDD];
    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 4),
        other => panic!("expected Resync(4), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_single_ff() {
    let buf = [0xAA, 0xBB, 0xCC, 0xFF];
    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 3),
        other => panic!("expected Resync(3), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_ff_ff() {
    let buf = [0xAA, 0xBB, 0xFF, 0xFF];
    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 2),
        other => panic!("expected Resync(2), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_ff_ff_fd() {
    let buf = [0xAA, 0xFF, 0xFF, 0xFD];
    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 1),
        other => panic!("expected Resync(1), got {other:?}"),
    }
}

#[test]
fn short_input_with_no_partial_prefix_skips_immediately() {
    // No HEADER prefix → Resync(3), not Incomplete (those bytes can never extend into header).
    let buf = [0xAA, 0xBB, 0xCC];
    match Wire::parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 3),
        other => panic!("expected Resync(3), got {other:?}"),
    }
}

#[test]
fn short_input_with_partial_prefix_returns_incomplete() {
    for partial in [&[0xFFu8][..], &[0xFF, 0xFF][..], &[0xFF, 0xFF, 0xFD][..]] {
        assert!(
            matches!(Wire::parse_one(partial), Err(ParseError::Incomplete)),
            "partial = {partial:02X?}"
        );
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
            data: Bytes::Raw(&[0, 2, 0, 0]),
        }),
        Packet::RegWrite(RegWritePacket {
            id: 1,
            address: 100,
            data: Bytes::Raw(&[0xAA]),
        }),
        Packet::Action(ActionPacket { id: 1 }),
        Packet::FactoryReset(FactoryResetPacket { id: 1, mode: 0xFF }),
        Packet::Reboot(RebootPacket { id: 1 }),
        Packet::Clear(ClearPacket {
            id: 1,
            body: Bytes::Raw(&[0x01, 0x02, 0x03]),
        }),
        Packet::ControlTableBackup(ControlTableBackupPacket {
            id: 1,
            body: Bytes::Raw(&[0xAA, 0xBB]),
        }),
        Packet::Status(StatusPacket {
            id: 1,
            error: 0,
            params: Bytes::Raw(&[0xDE, 0xAD]),
        }),
        Packet::SyncRead(SyncReadPacket {
            address: 132,
            length: 4,
            ids: Bytes::Raw(&[1, 2, 3]),
        }),
        Packet::SyncWrite(SyncWritePacket {
            address: 116,
            length: 4,
            body: Bytes::Raw(&[1, 0, 1, 0, 0]),
        }),
        Packet::FastSyncRead(FastSyncReadPacket {
            address: 132,
            length: 4,
            ids: Bytes::Raw(&[1, 2]),
        }),
        Packet::BulkRead(BulkReadPacket {
            body: Bytes::Raw(&[1, 2]),
        }),
        Packet::BulkWrite(BulkWritePacket {
            body: Bytes::Raw(&[1, 2, 3]),
        }),
        Packet::FastBulkRead(FastBulkReadPacket {
            body: Bytes::Raw(&[1, 2]),
        }),
    ];

    for case in cases {
        let mut wire1: HVec<u8, 128> = HVec::new();
        Wire::write(&mut wire1, case).unwrap();
        let (pkt, n) = Wire::parse_one(&wire1).expect("parse failed");
        assert_eq!(n, wire1.len());
        let mut wire2: HVec<u8, 128> = HVec::new();
        Wire::write(&mut wire2, &pkt).unwrap();
        assert_eq!(&wire1[..], &wire2[..], "wire mismatch on {case:?}");
    }
}

#[test]
fn round_trip_random_fields_across_variants() {
    // Randomised fields per variant — catches encoding bugs that survive a fixed fixture.
    let mut rng = Rng::new(0xFA1AFE1);
    const ITERS: usize = 5000;

    for _ in 0..ITERS {
        let mut body: HVec<u8, 96> = HVec::new();
        let mut ids: HVec<u8, 32> = HVec::new();

        let kind = rng.next_byte() % 16;
        // Writer rejects 0xFF; remap to 1 to keep the sweep dense.
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
                // ~1-in-8: insert a stuffing trigger.
                body.extend_from_slice(&[0xFF, 0xFF, 0xFD]).unwrap();
            } else {
                body.push(rng.next_byte()).unwrap();
            }
        }
        let ids_target = (rng.next_u64() % 8) as usize;
        for _ in 0..ids_target {
            // valid DXL ids are 0..=253.
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
                data: Bytes::Raw(&body),
            }),
            3 => Packet::RegWrite(RegWritePacket {
                id,
                address: addr,
                data: Bytes::Raw(&body),
            }),
            4 => Packet::Action(ActionPacket { id }),
            5 => Packet::FactoryReset(FactoryResetPacket { id, mode }),
            6 => Packet::Reboot(RebootPacket { id }),
            7 => Packet::Clear(ClearPacket {
                id,
                body: Bytes::Raw(&body),
            }),
            8 => Packet::ControlTableBackup(ControlTableBackupPacket {
                id,
                body: Bytes::Raw(&body),
            }),
            9 => Packet::Status(StatusPacket {
                id,
                error,
                params: Bytes::Raw(&body),
            }),
            10 => Packet::SyncRead(SyncReadPacket {
                address: addr,
                length,
                ids: Bytes::Raw(&ids),
            }),
            11 => Packet::SyncWrite(SyncWritePacket {
                address: addr,
                length,
                body: Bytes::Raw(&body),
            }),
            12 => Packet::FastSyncRead(FastSyncReadPacket {
                address: addr,
                length,
                ids: Bytes::Raw(&ids),
            }),
            13 => Packet::BulkRead(BulkReadPacket {
                body: Bytes::Raw(&body),
            }),
            14 => Packet::BulkWrite(BulkWritePacket {
                body: Bytes::Raw(&body),
            }),
            _ => Packet::FastBulkRead(FastBulkReadPacket {
                body: Bytes::Raw(&body),
            }),
        };

        let mut wire1: HVec<u8, 256> = HVec::new();
        Wire::write(&mut wire1, &pkt).expect("write");
        let (parsed, n) = Wire::parse_one(&wire1).expect("parse");
        assert_eq!(n, wire1.len());
        let mut wire2: HVec<u8, 256> = HVec::new();
        Wire::write(&mut wire2, &parsed).expect("re-write");
        assert_eq!(
            &wire1[..],
            &wire2[..],
            "wire mismatch on kind={kind} pkt={pkt:?}"
        );
    }
}

#[test]
fn stuffing_round_trip_for_every_body_carrying_variant() {
    // Threads a stuffing trigger through each variant — catches a `Bytes::Raw`
    // regression that the fixed-fixture round-trip test would miss.
    let logical: &[u8] = &[0xFF, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xAA];

    let cases: &[(&str, Packet<'_>)] = &[
        (
            "Write",
            Packet::Write(WritePacket {
                id: 1,
                address: 0x0010,
                data: Bytes::Raw(logical),
            }),
        ),
        (
            "RegWrite",
            Packet::RegWrite(RegWritePacket {
                id: 1,
                address: 0x0010,
                data: Bytes::Raw(logical),
            }),
        ),
        (
            "Clear",
            Packet::Clear(ClearPacket {
                id: 1,
                body: Bytes::Raw(logical),
            }),
        ),
        (
            "ControlTableBackup",
            Packet::ControlTableBackup(ControlTableBackupPacket {
                id: 1,
                body: Bytes::Raw(logical),
            }),
        ),
        (
            "Status",
            Packet::Status(StatusPacket {
                id: 1,
                error: 0,
                params: Bytes::Raw(logical),
            }),
        ),
        (
            "SyncWrite",
            Packet::SyncWrite(SyncWritePacket {
                address: 0x0010,
                length: 4,
                body: Bytes::Raw(logical),
            }),
        ),
        (
            "BulkRead",
            Packet::BulkRead(BulkReadPacket {
                body: Bytes::Raw(logical),
            }),
        ),
        (
            "BulkWrite",
            Packet::BulkWrite(BulkWritePacket {
                body: Bytes::Raw(logical),
            }),
        ),
        (
            "FastBulkRead",
            Packet::FastBulkRead(FastBulkReadPacket {
                body: Bytes::Raw(logical),
            }),
        ),
    ];

    for (name, pkt) in cases {
        let mut wire: HVec<u8, 64> = HVec::new();
        Wire::write(&mut wire, pkt).unwrap_or_else(|e| panic!("{name}: write failed: {e:?}"));

        // ≥2 extra FDs expected (one per trigger).
        let payload = &wire[8..wire.len() - 2];
        let logical_fds = logical.iter().filter(|&&b| b == 0xFD).count();
        let wire_fds = payload.iter().filter(|&&b| b == 0xFD).count();
        assert!(
            wire_fds >= logical_fds + 2,
            "{name}: expected stuffing on wire, got {wire_fds} 0xFD bytes for {logical_fds} logical"
        );

        let (parsed, n) =
            Wire::parse_one(&wire).unwrap_or_else(|e| panic!("{name}: parse failed: {e:?}"));
        assert_eq!(n, wire.len(), "{name}: short parse");

        let recovered = match parsed {
            Packet::Write(p) => p.data,
            Packet::RegWrite(p) => p.data,
            Packet::Clear(p) => p.body,
            Packet::ControlTableBackup(p) => p.body,
            Packet::Status(p) => p.params,
            Packet::SyncWrite(p) => p.body,
            Packet::BulkRead(p) => p.body,
            Packet::BulkWrite(p) => p.body,
            Packet::FastBulkRead(p) => p.body,
            other => panic!("{name}: parsed into unexpected variant {other:?}"),
        };

        assert_eq!(
            recovered.unstuffed_len(),
            logical.len(),
            "{name}: length mismatch"
        );
        let mut buf = [0u8; 32];
        let copied = recovered.copy_into(&mut buf).expect("copy_into");
        assert_eq!(
            &buf[..copied],
            logical,
            "{name}: payload mismatch after unstuffing"
        );
    }
}
