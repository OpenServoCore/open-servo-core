//! Stress tests for parser robustness:
//! - never panics on arbitrary byte streams
//! - never wedges (always makes forward progress)
//! - rejects foreign protocols without false-accepting frames
//! - real frames embedded in noise are recovered
//!
//! Tests run with `std` (in `tests/`), but parser/writer are `no_std`.

use dxl_protocol::{parse_one, write, Bytes, Packet, ParseError, HEADER, MAX_LENGTH};
use heapless::Vec as HVec;

/// Deterministic PRNG (xorshift64*).
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

/// Drives `parse_one` over `bytes` in chunks of `chunk_size`. Asserts the
/// progress invariant (every error variant other than Incomplete must report
/// `skip >= 1`) and that the loop terminates. Returns the number of accepted
/// frames.
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

        match parse_one(&buf) {
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
    // CRC-16 false-accept probability is ~2^-16 conditioned on header+length+
    // instruction also matching. Across 1MB, expectation is far below 1.
    // Allow some slack but flag a regression if accepts spike.
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
    // Synthetic Modbus-RTU-ish frames: [addr, func, payload..., crc_lo, crc_hi].
    // Loop a wide range of addresses/functions/payloads — none should ever
    // contain the DXL header pattern by accident.
    let mut rng = Rng::new(0xB0BA);
    let mut stream: Vec<u8> = Vec::new();
    for addr in 1..=247u8 {
        for func in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x10, 0x17] {
            stream.push(addr);
            stream.push(func);
            for _ in 0..rng.next_u64() % 16 + 4 {
                // Avoid producing the DXL header pattern accidentally —
                // mask the high bit out so we never emit 0xFF.
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
    // Pure ASCII can never form the DXL header (0xFF, 0xFD bytes are 8-bit).
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
            // Mask high bit so we don't accidentally synthesize valid frames
            // in the "garbage" section.
            stream.push(rng.next_byte() & 0x7F);
        }
        let mut frame: HVec<u8, 32> = HVec::new();
        write(
            &mut frame,
            &Packet::Ping {
                id: ((i % 253) + 1) as u8,
            },
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
        write(
            &mut frame,
            &Packet::Ping {
                id: ((i % 253) + 1) as u8,
            },
        )
        .unwrap();
        stream.extend_from_slice(&frame);
    }
    let accepts = drain_stream(&stream, 1);
    assert_eq!(accepts, real_count);
}

#[test]
fn real_frames_with_partial_corrupt_frames_recover_clean_ones() {
    // Mix valid frames with frames whose CRC has been bit-flipped. The
    // parser should accept every valid frame and reject every corrupt one.
    let mut rng = Rng::new(0x12345678);
    let mut stream: Vec<u8> = Vec::new();
    let valid = 100;
    let corrupt = 100;

    for i in 0..valid {
        let mut frame: HVec<u8, 32> = HVec::new();
        write(
            &mut frame,
            &Packet::Ping {
                id: ((i % 253) + 1) as u8,
            },
        )
        .unwrap();
        stream.extend_from_slice(&frame);
    }
    for i in 0..corrupt {
        let mut frame: HVec<u8, 32> = HVec::new();
        write(
            &mut frame,
            &Packet::Ping {
                id: ((i % 253) + 1) as u8,
            },
        )
        .unwrap();
        // Corrupt the CRC.
        let last = frame.len() - 1;
        frame[last] ^= rng.next_byte() | 1;
        stream.extend_from_slice(&frame);
    }

    let accepts = drain_stream(&stream, 64);
    // Each corrupt frame's body bytes can re-resync to the next real header
    // (or to themselves), so we may accept slightly more than `valid` if the
    // body bytes overlap — but for plain PINGs they don't form headers.
    assert_eq!(accepts, valid);
}

#[test]
fn parse_one_progress_invariant_random_short_inputs() {
    // For ANY input, parse_one either:
    //  - returns Ok with n >= 10
    //  - returns Incomplete
    //  - returns an error with skip >= 1
    // and never panics.
    let mut rng = Rng::new(0xC0FFEE);
    for _ in 0..50_000 {
        let len = (rng.next_u64() % 64) as usize;
        let mut buf = vec![0u8; len];
        rng.fill(&mut buf);
        match parse_one(&buf) {
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
        match parse_one(&buf) {
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
    const PING: &[u8] = &[
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E,
    ];
    for n in 0..PING.len() {
        let r = parse_one(&PING[..n]);
        assert!(
            matches!(r, Err(ParseError::Incomplete)),
            "truncation to {n} bytes: got {r:?}"
        );
    }
    let (pkt, n) = parse_one(PING).unwrap();
    assert_eq!(n, PING.len());
    assert!(matches!(pkt, Packet::Ping { id: 1 }));
}

#[test]
fn random_header_id_length_does_not_lock_parser() {
    // Construct buffers that look "headerly" but with random id+length, and
    // verify the parser always returns a definite error or completes within
    // a bounded number of steps.
    let mut rng = Rng::new(13);
    for _ in 0..500 {
        let len = (rng.next_u64() % 2048) as usize;
        let mut buf = vec![0u8; 7 + len + 2];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        // Random id and length field.
        let _ = drain_stream(&buf, 64);
    }
}

#[test]
fn maxlen_phantom_header_returns_definite_error() {
    // A buffer of exactly (7 + MAX_LENGTH) bytes with a header at start and
    // length = MAX_LENGTH should not return Incomplete — it has enough data
    // to verify CRC and reject.
    let mut rng = Rng::new(1234);
    for _ in 0..200 {
        let mut buf = vec![0u8; 7 + MAX_LENGTH];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        buf[5] = (MAX_LENGTH & 0xFF) as u8;
        buf[6] = ((MAX_LENGTH >> 8) & 0xFF) as u8;
        if let Err(ParseError::Incomplete) = parse_one(&buf) {
            panic!("should not be Incomplete with full frame");
        }
    }
}

#[test]
fn over_maxlen_header_short_circuits() {
    // Length above MAX_LENGTH must trigger BadLength immediately, not wait
    // for ~64KB of phantom data.
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    match parse_one(&bad) {
        Err(ParseError::BadLength { skip }) => assert_eq!(skip, 4),
        other => panic!("expected BadLength, got {other:?}"),
    }
}

#[test]
fn writer_id_0xff_rejected() {
    let mut out: HVec<u8, 32> = HVec::new();
    let err = write(&mut out, &Packet::Ping { id: 0xFF }).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
}

#[test]
fn writer_overflow_reported_and_rolled_back() {
    let mut tiny: HVec<u8, 5> = HVec::new();
    let err = write(&mut tiny, &Packet::Ping { id: 1 }).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert!(
        tiny.is_empty(),
        "buffer should be rolled back on overflow, has {} bytes",
        tiny.len()
    );
}

#[test]
fn writer_overflow_preserves_prior_frames() {
    // Simulate a DMA TX buffer holding a complete frame, then a second
    // write that overflows. The first frame must remain intact.
    let mut buf: HVec<u8, 16> = HVec::new();
    write(&mut buf, &Packet::Ping { id: 1 }).unwrap();
    let snapshot: HVec<u8, 16> = buf.clone();
    let pre_len = buf.len();

    // Try to write a Status whose body cannot fit in the remaining capacity.
    let big = [0u8; 32];
    let err = write(
        &mut buf,
        &Packet::Status {
            id: 1,
            error: 0,
            params: Bytes::Raw(&big),
        },
    )
    .unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Overflow);
    assert_eq!(buf.len(), pre_len, "buffer length changed on overflow");
    assert_eq!(&buf[..], &snapshot[..], "prior frame corrupted on overflow");

    // The preserved frame still parses.
    let (pkt, n) = parse_one(&buf).unwrap();
    assert_eq!(n, buf.len());
    assert!(matches!(pkt, Packet::Ping { id: 1 }));
}

#[test]
fn writer_invalid_id_does_not_touch_buffer() {
    let mut buf: HVec<u8, 32> = HVec::new();
    write(&mut buf, &Packet::Ping { id: 1 }).unwrap();
    let snapshot: HVec<u8, 32> = buf.clone();

    let err = write(&mut buf, &Packet::Ping { id: 0xFF }).unwrap_err();
    assert_eq!(err, dxl_protocol::WriteError::Invalid);
    assert_eq!(&buf[..], &snapshot[..]);
}

#[test]
fn no_header_skips_all_when_no_partial_prefix() {
    // No 0xFF anywhere — every byte can be discarded.
    let buf = [0xAA, 0xBB, 0xCC, 0xDD];
    match parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 4),
        other => panic!("expected Resync(4), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_single_ff() {
    let buf = [0xAA, 0xBB, 0xCC, 0xFF];
    match parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 3),
        other => panic!("expected Resync(3), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_ff_ff() {
    let buf = [0xAA, 0xBB, 0xFF, 0xFF];
    match parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 2),
        other => panic!("expected Resync(2), got {other:?}"),
    }
}

#[test]
fn no_header_keeps_trailing_ff_ff_fd() {
    let buf = [0xAA, 0xFF, 0xFF, 0xFD];
    match parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 1),
        other => panic!("expected Resync(1), got {other:?}"),
    }
}

#[test]
fn short_input_with_no_partial_prefix_skips_immediately() {
    // Length-3 input that can't possibly start a header → Resync(3),
    // not Incomplete (which would force the caller to wait for bytes that
    // can never extend these into a valid header).
    let buf = [0xAA, 0xBB, 0xCC];
    match parse_one(&buf) {
        Err(ParseError::Resync { skip }) => assert_eq!(skip, 3),
        other => panic!("expected Resync(3), got {other:?}"),
    }
}

#[test]
fn short_input_with_partial_prefix_returns_incomplete() {
    // Trailing prefix of HEADER — must wait for more bytes.
    for partial in [
        &[0xFFu8][..],
        &[0xFF, 0xFF][..],
        &[0xFF, 0xFF, 0xFD][..],
    ] {
        assert!(
            matches!(parse_one(partial), Err(ParseError::Incomplete)),
            "partial = {partial:02X?}"
        );
    }
}

#[test]
fn round_trip_every_instruction() {
    // Sanity: every Packet variant survives write -> parse -> write.
    let cases: &[Packet<'static>] = &[
        Packet::Ping { id: 1 },
        Packet::Read {
            id: 1,
            address: 132,
            length: 4,
        },
        Packet::Write {
            id: 1,
            address: 116,
            data: Bytes::Raw(&[0, 2, 0, 0]),
        },
        Packet::RegWrite {
            id: 1,
            address: 100,
            data: Bytes::Raw(&[0xAA]),
        },
        Packet::Action { id: 1 },
        Packet::FactoryReset { id: 1, mode: 0xFF },
        Packet::Reboot { id: 1 },
        Packet::Clear {
            id: 1,
            body: Bytes::Raw(&[0x01, 0x02, 0x03]),
        },
        Packet::ControlTableBackup {
            id: 1,
            body: Bytes::Raw(&[0xAA, 0xBB]),
        },
        Packet::Status {
            id: 1,
            error: 0,
            params: Bytes::Raw(&[0xDE, 0xAD]),
        },
        Packet::SyncRead {
            address: 132,
            length: 4,
            ids: Bytes::Raw(&[1, 2, 3]),
        },
        Packet::SyncWrite {
            address: 116,
            length: 4,
            body: Bytes::Raw(&[1, 0, 1, 0, 0]),
        },
        Packet::FastSyncRead {
            address: 132,
            length: 4,
            ids: Bytes::Raw(&[1, 2]),
        },
        Packet::BulkRead {
            body: Bytes::Raw(&[1, 2]),
        },
        Packet::BulkWrite {
            body: Bytes::Raw(&[1, 2, 3]),
        },
        Packet::FastBulkRead {
            body: Bytes::Raw(&[1, 2]),
        },
    ];

    for case in cases {
        let mut wire1: HVec<u8, 128> = HVec::new();
        write(&mut wire1, case).unwrap();
        let (pkt, n) = parse_one(&wire1).expect("parse failed");
        assert_eq!(n, wire1.len());
        let mut wire2: HVec<u8, 128> = HVec::new();
        write(&mut wire2, &pkt).unwrap();
        assert_eq!(&wire1[..], &wire2[..], "wire mismatch on {case:?}");
    }
}

#[test]
fn round_trip_random_fields_across_variants() {
    // `round_trip_every_instruction` proves the byte-identical re-write
    // property for one fixed example per variant. This randomises the
    // fields (id, address, length, payload bytes, occasional stuffing
    // triggers) and exercises every variant many times across a deterministic
    // PRNG sweep — a field-encoding bug in any variant that happens to
    // survive the one fixed example would surface here.
    let mut rng = Rng::new(0xFA1AFE1);
    const ITERS: usize = 5000;

    for _ in 0..ITERS {
        let mut body: HVec<u8, 96> = HVec::new();
        let mut ids: HVec<u8, 32> = HVec::new();

        let kind = rng.next_byte() % 16;
        // Writer rejects 0xFF; remap deterministically to keep the sweep dense.
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
                // ~1-in-8 chance of inserting a stuffing trigger.
                body.extend_from_slice(&[0xFF, 0xFF, 0xFD]).unwrap();
            } else {
                body.push(rng.next_byte()).unwrap();
            }
        }
        let ids_target = (rng.next_u64() % 8) as usize;
        for _ in 0..ids_target {
            // Real DXL ids are 0..=253; avoid 0xFE/0xFF reserved values.
            ids.push(rng.next_byte() % 254).unwrap();
        }

        let pkt = match kind {
            0 => Packet::Ping { id },
            1 => Packet::Read { id, address: addr, length },
            2 => Packet::Write { id, address: addr, data: Bytes::Raw(&body) },
            3 => Packet::RegWrite { id, address: addr, data: Bytes::Raw(&body) },
            4 => Packet::Action { id },
            5 => Packet::FactoryReset { id, mode },
            6 => Packet::Reboot { id },
            7 => Packet::Clear { id, body: Bytes::Raw(&body) },
            8 => Packet::ControlTableBackup { id, body: Bytes::Raw(&body) },
            9 => Packet::Status { id, error, params: Bytes::Raw(&body) },
            10 => Packet::SyncRead { address: addr, length, ids: Bytes::Raw(&ids) },
            11 => Packet::SyncWrite { address: addr, length, body: Bytes::Raw(&body) },
            12 => Packet::FastSyncRead { address: addr, length, ids: Bytes::Raw(&ids) },
            13 => Packet::BulkRead { body: Bytes::Raw(&body) },
            14 => Packet::BulkWrite { body: Bytes::Raw(&body) },
            _ => Packet::FastBulkRead { body: Bytes::Raw(&body) },
        };

        let mut wire1: HVec<u8, 256> = HVec::new();
        write(&mut wire1, &pkt).expect("write");
        let (parsed, n) = parse_one(&wire1).expect("parse");
        assert_eq!(n, wire1.len());
        let mut wire2: HVec<u8, 256> = HVec::new();
        write(&mut wire2, &parsed).expect("re-write");
        assert_eq!(&wire1[..], &wire2[..], "wire mismatch on kind={kind} pkt={pkt:?}");
    }
}

#[test]
fn stuffing_round_trip_for_every_body_carrying_variant() {
    // Each body-carrying variant's `decode()` arm independently chooses
    // `Bytes::stuffed()` for its payload. A regression that swapped one to
    // `Bytes::Raw` would silently corrupt that variant's payload whenever
    // a stuffing trigger appeared in the logical bytes — but
    // `round_trip_every_instruction`'s fixed fixtures contain no triggers,
    // so the bug would not surface there. This test threads a trigger
    // through each variant.
    let logical: &[u8] = &[0xFF, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xAA];

    let cases: &[(&str, Packet<'_>)] = &[
        ("Write",              Packet::Write              { id: 1, address: 0x0010, data: Bytes::Raw(logical) }),
        ("RegWrite",           Packet::RegWrite           { id: 1, address: 0x0010, data: Bytes::Raw(logical) }),
        ("Clear",              Packet::Clear              { id: 1, body: Bytes::Raw(logical) }),
        ("ControlTableBackup", Packet::ControlTableBackup { id: 1, body: Bytes::Raw(logical) }),
        ("Status",             Packet::Status             { id: 1, error: 0, params: Bytes::Raw(logical) }),
        ("SyncWrite",          Packet::SyncWrite          { address: 0x0010, length: 4, body: Bytes::Raw(logical) }),
        ("BulkRead",           Packet::BulkRead           { body: Bytes::Raw(logical) }),
        ("BulkWrite",          Packet::BulkWrite          { body: Bytes::Raw(logical) }),
        ("FastBulkRead",       Packet::FastBulkRead       { body: Bytes::Raw(logical) }),
    ];

    for (name, pkt) in cases {
        let mut wire: HVec<u8, 64> = HVec::new();
        write(&mut wire, pkt).unwrap_or_else(|e| panic!("{name}: write failed: {e:?}"));

        // Stuffing must have inserted at least 2 extra FDs (one per trigger).
        let payload = &wire[8..wire.len() - 2];
        let logical_fds = logical.iter().filter(|&&b| b == 0xFD).count();
        let wire_fds = payload.iter().filter(|&&b| b == 0xFD).count();
        assert!(
            wire_fds >= logical_fds + 2,
            "{name}: expected stuffing on wire, got {wire_fds} 0xFD bytes for {logical_fds} logical"
        );

        let (parsed, n) = parse_one(&wire).unwrap_or_else(|e| panic!("{name}: parse failed: {e:?}"));
        assert_eq!(n, wire.len(), "{name}: short parse");

        let recovered = match parsed {
            Packet::Write              { data,   .. } => data,
            Packet::RegWrite           { data,   .. } => data,
            Packet::Clear              { body,   .. } => body,
            Packet::ControlTableBackup { body,   .. } => body,
            Packet::Status             { params, .. } => params,
            Packet::SyncWrite          { body,   .. } => body,
            Packet::BulkRead           { body }       => body,
            Packet::BulkWrite          { body }       => body,
            Packet::FastBulkRead       { body }       => body,
            other => panic!("{name}: parsed into unexpected variant {other:?}"),
        };

        assert_eq!(recovered.unstuffed_len(), logical.len(), "{name}: length mismatch");
        let mut buf = [0u8; 32];
        let copied = recovered.copy_into(&mut buf).expect("copy_into");
        assert_eq!(&buf[..copied], logical, "{name}: payload mismatch after unstuffing");
    }
}
