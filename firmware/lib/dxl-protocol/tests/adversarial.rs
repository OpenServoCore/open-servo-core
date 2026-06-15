//! Streaming Decoder robustness: no panics, no wedge, rejects foreign
//! protocols, recovers real frames embedded in noise.

use dxl_protocol::codec::{Decoder, ResyncKind, Step};
use dxl_protocol::packet::{self as overlay, BulkReadEntry, ErrorCode, Id, StatusError, U16Le};
use dxl_protocol::{
    HEADER, InstructionEncoder, PACKET_LEN_GUARD, SoftwareCrcUmts, StatusEncoder, WriteBuf,
    WriteError,
};
use heapless::Vec as HVec;

type Crc = SoftwareCrcUmts;

fn write_ping(id: u8) -> HVec<u8, 32> {
    let mut v: HVec<u8, 32> = HVec::new();
    InstructionEncoder::<_, Crc>::new(&mut v)
        .ping(Id::new(id))
        .unwrap();
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
        assert!(
            n >= 1,
            "feed must consume at least one byte from a non-empty chunk"
        );
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

fn feed_full<'a, const M: usize>(dec: &'a mut Decoder<M, Crc>, wire: &[u8]) -> overlay::Packet<'a> {
    let (step, n) = dec.feed(wire);
    assert_eq!(n, wire.len(), "decoder didn't consume the full frame");
    match step {
        Step::Packet(p) => p,
        other => panic!("expected Packet, got {other:?}"),
    }
}

#[test]
fn random_byte_stream_does_not_panic_or_wedge() {
    let mut rng = Rng::new(0xDEADBEEFu64);
    let mut bytes = vec![0u8; 1_000_000];
    rng.fill(&mut bytes);

    let accepts = drain_stream(&bytes, 64);
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
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, Id::new(1)),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn header_byte_at_a_time() {
    // Streaming header bytes one at a time exercises the KMP sync FSM and
    // the wire_n boundary checks inside step_header.
    let frame = {
        let mut v: HVec<u8, 32> = HVec::new();
        InstructionEncoder::<_, Crc>::new(&mut v)
            .read(Id::new(7), 0x0084, 4)
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
            assert_eq!(p.header.id, Id::new(7));
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
    let mut rng = Rng::new(1234);
    for _ in 0..50 {
        let mut buf = vec![0u8; 7 + PACKET_LEN_GUARD];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        buf[5] = (PACKET_LEN_GUARD & 0xFF) as u8;
        buf[6] = ((PACKET_LEN_GUARD >> 8) & 0xFF) as u8;
        let _ = drain_stream(&buf, PACKET_LEN_GUARD + 16);
    }
}

#[test]
fn over_maxlen_header_short_circuits() {
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&bad);
    assert_eq!(n, 7);
    assert!(matches!(step, Step::Resync(ResyncKind::BadLength)));
}

#[test]
fn writer_id_0xff_rejected() {
    let mut out: HVec<u8, 32> = HVec::new();
    let err = InstructionEncoder::<_, Crc>::new(&mut out)
        .ping(Id::new(0xFF))
        .unwrap_err();
    assert_eq!(err, WriteError::Invalid);
}

#[test]
fn writer_overflow_reported_and_rolled_back() {
    let mut tiny: HVec<u8, 5> = HVec::new();
    let err = InstructionEncoder::<_, Crc>::new(&mut tiny)
        .ping(Id::new(1))
        .unwrap_err();
    assert_eq!(err, WriteError::Overflow);
    assert!(
        tiny.is_empty(),
        "buffer should be rolled back on overflow, has {} bytes",
        tiny.len()
    );
}

#[test]
fn writer_overflow_preserves_prior_frames() {
    // DMA TX buffer holds a frame; second write overflows -- first must survive
    // and re-decode cleanly through the new Decoder.
    let mut buf: HVec<u8, 16> = HVec::new();
    InstructionEncoder::<_, Crc>::new(&mut buf)
        .ping(Id::new(1))
        .unwrap();
    let snapshot: HVec<u8, 16> = buf.clone();
    let pre_len = buf.len();

    let big = [0u8; 32];
    let err = StatusEncoder::<_, Crc>::new(&mut buf)
        .ext(Id::new(1), StatusError::OK, &big)
        .unwrap_err();
    assert_eq!(err, WriteError::Overflow);
    assert_eq!(buf.len(), pre_len, "buffer length changed on overflow");
    assert_eq!(&buf[..], &snapshot[..], "prior frame corrupted on overflow");

    let mut dec: Decoder<64, Crc> = Decoder::new();
    let (step, n) = dec.feed(&buf);
    assert_eq!(n, buf.len());
    match step {
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, Id::new(1)),
        other => panic!("expected Ping, got {other:?}"),
    }
}

#[test]
fn writer_invalid_id_does_not_touch_buffer() {
    let mut buf: HVec<u8, 32> = HVec::new();
    InstructionEncoder::<_, Crc>::new(&mut buf)
        .ping(Id::new(1))
        .unwrap();
    let snapshot: HVec<u8, 32> = buf.clone();

    let err = InstructionEncoder::<_, Crc>::new(&mut buf)
        .ping(Id::new(0xFF))
        .unwrap_err();
    assert_eq!(err, WriteError::Invalid);
    assert_eq!(&buf[..], &snapshot[..]);
}

#[test]
fn pure_noise_followed_by_frame_recovers_frame() {
    let mut buf: Vec<u8> = vec![0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
    buf.extend_from_slice(&write_ping(1));
    let mut dec: Decoder<32, Crc> = Decoder::new();
    let (step, n) = dec.feed(&buf);
    assert_eq!(n, buf.len());
    match step {
        Step::Packet(overlay::Packet::Ping(p)) => assert_eq!(p.header.id, Id::new(1)),
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

// --- Round-trip every instruction shape. Each case carries a closure that
//     emits the request shape onto the wire and a closure that verifies the
//     decoded overlay matches. Keeping the verification right next to the
//     emit makes the cases self-contained -- no shared match table that has
//     to enumerate every variant.

fn round_trip<E, V>(label: &str, emit: E, verify: V)
where
    E: FnOnce(&mut InstructionEncoder<'_, HVec<u8, 256>, Crc>) -> Result<(), WriteError>,
    V: FnOnce(overlay::Packet<'_>),
{
    let mut wire: HVec<u8, 256> = HVec::new();
    {
        let mut w = InstructionEncoder::<_, Crc>::new(&mut wire);
        emit(&mut w).unwrap_or_else(|e| panic!("{label}: emit failed: {e:?}"));
    }
    let mut dec: Decoder<512, Crc> = Decoder::new();
    let decoded = feed_full(&mut dec, &wire);
    verify(decoded);
}

#[test]
fn round_trip_every_instruction() {
    round_trip(
        "Ping",
        |w| w.ping(Id::new(1)),
        |p| match p {
            overlay::Packet::Ping(d) => assert_eq!(d.header.id, Id::new(1)),
            other => panic!("expected Ping, got {other:?}"),
        },
    );
    round_trip(
        "Read",
        |w| w.read(Id::new(1), 132, 4),
        |p| match p {
            overlay::Packet::Read(d) => {
                assert_eq!(d.header.id, Id::new(1));
                assert_eq!(d.addr.get(), 132);
                assert_eq!(d.length.get(), 4);
            }
            other => panic!("expected Read, got {other:?}"),
        },
    );
    round_trip(
        "Write",
        |w| w.write(Id::new(1), 116, &[0, 2, 0, 0]),
        |p| match p {
            overlay::Packet::Write(d) => {
                assert_eq!(d.header.header.id, Id::new(1));
                assert_eq!(d.header.addr.get(), 116);
                assert_eq!(d.data, &[0, 2, 0, 0]);
            }
            other => panic!("expected Write, got {other:?}"),
        },
    );
    round_trip(
        "RegWrite",
        |w| w.reg_write(Id::new(1), 100, &[0xAA]),
        |p| match p {
            overlay::Packet::RegWrite(d) => {
                assert_eq!(d.header.header.id, Id::new(1));
                assert_eq!(d.header.addr.get(), 100);
                assert_eq!(d.data, &[0xAA]);
            }
            other => panic!("expected RegWrite, got {other:?}"),
        },
    );
    round_trip(
        "Action",
        |w| w.action(Id::new(1)),
        |p| match p {
            overlay::Packet::Action(d) => assert_eq!(d.header.id, Id::new(1)),
            other => panic!("expected Action, got {other:?}"),
        },
    );
    round_trip(
        "FactoryReset",
        |w| w.factory_reset(Id::new(1), 0xFF),
        |p| match p {
            overlay::Packet::FactoryReset(d) => {
                assert_eq!(d.header.id, Id::new(1));
                assert_eq!(d.mode, 0xFF);
            }
            other => panic!("expected FactoryReset, got {other:?}"),
        },
    );
    round_trip(
        "Reboot",
        |w| w.reboot(Id::new(1)),
        |p| match p {
            overlay::Packet::Reboot(d) => assert_eq!(d.header.id, Id::new(1)),
            other => panic!("expected Reboot, got {other:?}"),
        },
    );
    round_trip(
        "SyncRead",
        |w| w.sync_read(132, 4, &[1, 2, 3]),
        |p| match p {
            overlay::Packet::SyncRead(d) => {
                assert_eq!(d.header.addr.get(), 132);
                assert_eq!(d.header.length.get(), 4);
                assert_eq!(d.ids, &[1, 2, 3]);
            }
            other => panic!("expected SyncRead, got {other:?}"),
        },
    );
    round_trip(
        "SyncWrite",
        |w| w.sync_write(116, 4, &[1, 0, 1, 0, 0]),
        |p| match p {
            overlay::Packet::SyncWrite(d) => {
                assert_eq!(d.header.addr.get(), 116);
                assert_eq!(d.header.length.get(), 4);
                assert_eq!(d.body, &[1, 0, 1, 0, 0]);
            }
            other => panic!("expected SyncWrite, got {other:?}"),
        },
    );
    round_trip(
        "FastSyncRead",
        |w| w.fast_sync_read(132, 4, &[1, 2]),
        |p| match p {
            overlay::Packet::FastSyncRead(d) => {
                assert_eq!(d.header.addr.get(), 132);
                assert_eq!(d.header.length.get(), 4);
                assert_eq!(d.ids, &[1, 2]);
            }
            other => panic!("expected FastSyncRead, got {other:?}"),
        },
    );
    let bulk_entries = [BulkReadEntry {
        id: Id::new(1),
        addr: U16Le::from_u16(0x0084),
        length: U16Le::from_u16(4),
    }];
    round_trip(
        "BulkRead",
        |w| w.bulk_read(&bulk_entries),
        |p| match p {
            overlay::Packet::BulkRead(d) => {
                assert_eq!(d.entries.len(), 1);
                assert_eq!(d.entries[0].id, Id::new(1));
                assert_eq!(d.entries[0].addr.get(), 0x0084);
                assert_eq!(d.entries[0].length.get(), 4);
            }
            other => panic!("expected BulkRead, got {other:?}"),
        },
    );
    round_trip(
        "BulkWrite",
        |w| w.bulk_write(&[1, 2, 3]),
        |p| match p {
            overlay::Packet::BulkWrite(d) => assert_eq!(d.body, &[1, 2, 3]),
            other => panic!("expected BulkWrite, got {other:?}"),
        },
    );
    round_trip(
        "FastBulkRead",
        |w| w.fast_bulk_read(&bulk_entries),
        |p| match p {
            overlay::Packet::FastBulkRead(d) => {
                assert_eq!(d.entries.len(), 1);
                assert_eq!(d.entries[0].id, Id::new(1));
                assert_eq!(d.entries[0].addr.get(), 0x0084);
                assert_eq!(d.entries[0].length.get(), 4);
            }
            other => panic!("expected FastBulkRead, got {other:?}"),
        },
    );
}

#[test]
fn round_trip_random_fields_across_variants() {
    let mut rng = Rng::new(0xFA1AFE1);
    const ITERS: usize = 2000;

    for _ in 0..ITERS {
        let mut body: HVec<u8, 96> = HVec::new();
        let mut ids: HVec<u8, 32> = HVec::new();

        let kind = rng.next_byte() % 13;
        let id = Id::new(match rng.next_byte() {
            0xFF => 1,
            b => b,
        });
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

        let mut wire: HVec<u8, 256> = HVec::new();
        {
            let mut w = InstructionEncoder::<_, Crc>::new(&mut wire);
            match kind {
                0 => w.ping(id).unwrap(),
                1 => w.read(id, addr, length).unwrap(),
                2 => w.write(id, addr, &body).unwrap(),
                3 => w.reg_write(id, addr, &body).unwrap(),
                4 => w.action(id).unwrap(),
                5 => w.factory_reset(id, mode).unwrap(),
                6 => w.reboot(id).unwrap(),
                7 => {
                    // Move-discard `w` so its `&mut wire` borrow ends before
                    // StatusEncoder takes its own. `drop(w)` would do the
                    // same but trips clippy::drop_non_drop.
                    let _ = w;
                    StatusEncoder::<_, Crc>::new(&mut wire)
                        .ext(id, StatusError::from_byte(error), &body)
                        .unwrap();
                }
                8 => w.sync_read(addr, length, &ids).unwrap(),
                9 => w.sync_write(addr, length, &body).unwrap(),
                10 => w.fast_sync_read(addr, length, &ids).unwrap(),
                11 => w.bulk_write(&body).unwrap(),
                _ => w.ext(id, 0x77, &body).unwrap(),
            }
        }

        let mut dec: Decoder<512, Crc> = Decoder::new();
        let (step, n) = dec.feed(&wire);
        assert_eq!(n, wire.len(), "kind {kind}: didn't consume full frame");
        assert!(
            matches!(step, Step::Packet(_)),
            "kind {kind}: expected Packet, got {step:?}"
        );
    }
}

#[test]
fn stuffing_round_trip_for_every_body_carrying_variant() {
    let logical: &[u8] = &[0xFF, 0xFF, 0xFD, 0x42, 0xFF, 0xFF, 0xFD, 0xAA];

    type Case = (&'static str, fn(&mut HVec<u8, 64>, &[u8]));
    let cases: &[Case] = &[
        ("Write", |out, data| {
            InstructionEncoder::<_, Crc>::new(out)
                .write(Id::new(1), 0x0010, data)
                .unwrap()
        }),
        ("RegWrite", |out, data| {
            InstructionEncoder::<_, Crc>::new(out)
                .reg_write(Id::new(1), 0x0010, data)
                .unwrap()
        }),
        ("Status", |out, data| {
            StatusEncoder::<_, Crc>::new(out)
                .ext(Id::new(1), StatusError::OK, data)
                .unwrap()
        }),
        ("SyncWrite", |out, data| {
            InstructionEncoder::<_, Crc>::new(out)
                .sync_write(0x0010, 4, data)
                .unwrap()
        }),
        ("BulkWrite", |out, data| {
            InstructionEncoder::<_, Crc>::new(out)
                .bulk_write(data)
                .unwrap()
        }),
    ];

    for (name, emit) in cases {
        let mut wire: HVec<u8, 64> = HVec::new();
        emit(&mut wire, logical);

        // >=2 extra FDs expected (one per trigger).
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
        assert_eq!(
            recovered, logical,
            "{name}: payload mismatch after unstuffing"
        );
    }
}

// Status with error code is encoded via StatusEncoder::ext with a non-OK
// code; covers the StatusError byte propagation through the round trip.
#[test]
fn status_with_error_code_round_trips() {
    let mut wire: HVec<u8, 32> = HVec::new();
    StatusEncoder::<_, Crc>::new(&mut wire)
        .ext(Id::new(1), StatusError::code(ErrorCode::DataRange), &[])
        .unwrap();
    let mut dec: Decoder<64, Crc> = Decoder::new();
    match feed_full(&mut dec, &wire) {
        overlay::Packet::Status(p) => {
            assert_eq!(p.header.header.id, Id::new(1));
            assert_eq!(p.error(), StatusError::code(ErrorCode::DataRange));
            assert!(p.params.is_empty());
        }
        other => panic!("expected Status, got {other:?}"),
    }
}
