//! Ported from the baseline `adversarial.rs`: the flat codec must never panic
//! or wedge, must reject foreign protocols, and must recover real frames
//! embedded in noise. Adapted to the new API (`parse` + `Probe`/`ParseError`).
//!
//! Behavior changes vs the baseline parser are called out inline:
//!  - `Resync` is now `Junk`.
//!  - An incomplete broadcast+Status (FAST chain) header now returns
//!    `Incomplete` rather than a phantom skip — `parse` no longer bakes in a
//!    FAST policy; the caller decides (see the two phantom tests below).

use dxl_protocol::frame::{ParseError, parse};
use dxl_protocol::wire::{HEADER, PACKET_LEN_GUARD};
use dxl_protocol::{Id, InstructionEncoder, SoftwareCrcUmts as Crc};
use heapless::Vec as HVec;

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

fn ping_frame(id: u8) -> HVec<u8, 32> {
    let mut f: HVec<u8, 32> = HVec::new();
    InstructionEncoder::<_, Crc>::new(&mut f)
        .ping(Id::new(id))
        .unwrap();
    f
}

fn skip_of(e: ParseError) -> Option<usize> {
    match e {
        ParseError::Incomplete => None,
        ParseError::Junk { skip }
        | ParseError::BadLength { skip }
        | ParseError::BadCrc { skip } => Some(skip),
    }
}

/// Asserts the progress invariant (every non-`Incomplete` error skips >= 1)
/// and termination. Returns the accepted frame count.
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
            "non-terminating loop on {} bytes",
            bytes.len()
        );

        match parse::<Crc>(&buf) {
            Ok((_frame, n)) => {
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
                let skip = skip_of(e).expect("non-Incomplete error carries skip");
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
    let mut rng = Rng::new(0xDEADBEEF);
    let mut bytes = vec![0u8; 1_000_000];
    rng.fill(&mut bytes);
    let accepts = drain_stream(&bytes, 64);
    assert!(accepts < 5, "unexpected false accepts: {accepts}");
}

#[test]
fn random_byte_stream_byte_at_a_time() {
    let mut rng = Rng::new(7);
    let mut bytes = vec![0u8; 100_000];
    rng.fill(&mut bytes);
    assert!(drain_stream(&bytes, 1) < 5);
}

#[test]
fn all_zero_stream_no_accepts() {
    assert_eq!(drain_stream(&vec![0u8; 100_000], 64), 0);
}

#[test]
fn all_ff_stream_no_accepts() {
    assert_eq!(drain_stream(&vec![0xFFu8; 100_000], 64), 0);
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
    assert!(drain_stream(&stream, 64) < 5);
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
        for _ in 0..(rng.next_u64() % 96) {
            stream.push(rng.next_byte() & 0x7F);
        }
        stream.extend_from_slice(&ping_frame(((i % 253) + 1) as u8));
    }
    assert_eq!(drain_stream(&stream, 64), real_count);
}

#[test]
fn real_frames_byte_at_a_time_recover_all() {
    let mut rng = Rng::new(0xCAFEBABE);
    let mut stream: Vec<u8> = Vec::new();
    let real_count = 50;
    for i in 0..real_count {
        for _ in 0..(rng.next_u64() % 32) {
            stream.push(rng.next_byte() & 0x7F);
        }
        stream.extend_from_slice(&ping_frame(((i % 253) + 1) as u8));
    }
    assert_eq!(drain_stream(&stream, 1), real_count);
}

#[test]
fn real_frames_with_partial_corrupt_frames_recover_clean_ones() {
    let mut rng = Rng::new(0x12345678);
    let mut stream: Vec<u8> = Vec::new();
    let valid = 100;
    let corrupt = 100;
    for i in 0..valid {
        stream.extend_from_slice(&ping_frame(((i % 253) + 1) as u8));
    }
    for i in 0..corrupt {
        let mut frame = ping_frame(((i % 253) + 1) as u8);
        let last = frame.len() - 1;
        frame[last] ^= rng.next_byte() | 1;
        stream.extend_from_slice(&frame);
    }
    assert_eq!(drain_stream(&stream, 64), valid);
}

#[test]
fn progress_invariant_random_short_inputs() {
    let mut rng = Rng::new(0xC0FFEE);
    for _ in 0..50_000 {
        let len = (rng.next_u64() % 64) as usize;
        let mut buf = vec![0u8; len];
        rng.fill(&mut buf);
        match parse::<Crc>(&buf) {
            Ok((_, n)) => assert!(n >= 10),
            Err(ParseError::Incomplete) => {}
            Err(e) => assert!(skip_of(e).unwrap() >= 1),
        }
    }
}

#[test]
fn progress_invariant_long_random_inputs() {
    let mut rng = Rng::new(0xC0FFEE2);
    for _ in 0..2000 {
        let len = (rng.next_u64() % (PACKET_LEN_GUARD as u64 * 2 + 64)) as usize;
        let mut buf = vec![0u8; len];
        rng.fill(&mut buf);
        match parse::<Crc>(&buf) {
            Ok((_, n)) => assert!(n >= 10 && n <= buf.len()),
            Err(ParseError::Incomplete) => {}
            Err(e) => assert!(skip_of(e).unwrap() >= 1),
        }
    }
}

#[test]
fn truncated_real_frame_returns_incomplete_at_each_step() {
    let ping = ping_frame(1);
    for n in 0..ping.len() {
        assert_eq!(parse::<Crc>(&ping[..n]), Err(ParseError::Incomplete));
    }
    let (_frame, n) = parse::<Crc>(&ping).unwrap();
    assert_eq!(n, ping.len());
}

#[test]
fn incomplete_broadcast_status_chain_header_returns_incomplete() {
    // BEHAVIOR CHANGE vs baseline: the baseline resynced past this phantom
    // (BadInstruction skip 4) to avoid wedging a chip's poll loop. The flat
    // codec has no FAST policy — `parse` reports `Incomplete` and the caller
    // (driver) decides whether to wait or resync.
    let frame: [u8; 14] = [
        0xFF, 0xFF, 0xFD, 0x00, // header
        0xFE, // BROADCAST_ID
        0x2B, 0x00, // length = 43 (whole multi-slot packet)
        0x55, // Status
        0x00, 50, 0xAA, 0xAA, 0xAA, 0xAA, // first slot's body
    ];
    assert_eq!(parse::<Crc>(&frame), Err(ParseError::Incomplete));
}

#[test]
fn truncated_broadcast_non_status_returns_incomplete() {
    let frame: [u8; 13] = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x08, 0x00, 0x03, // header, len 8, Write
        0x00, 0x00, 0xAA, 0xBB, // partial data
        0x00,
    ];
    assert_eq!(parse::<Crc>(&frame), Err(ParseError::Incomplete));
}

#[test]
fn maxlen_phantom_header_returns_definite_error() {
    let mut rng = Rng::new(1234);
    for _ in 0..200 {
        let mut buf = vec![0u8; 7 + PACKET_LEN_GUARD];
        rng.fill(&mut buf);
        buf[0..4].copy_from_slice(&HEADER);
        buf[5] = (PACKET_LEN_GUARD & 0xFF) as u8;
        buf[6] = ((PACKET_LEN_GUARD >> 8) & 0xFF) as u8;
        // A full-length buffer must produce a verdict, never Incomplete —
        // unless it happens to be a (CRC-unchecked) broadcast+Status chain.
        assert!(
            !matches!(parse::<Crc>(&buf), Err(ParseError::Incomplete)),
            "full frame should not be Incomplete"
        );
    }
}

#[test]
fn over_maxlen_header_short_circuits() {
    let bad = [0xFFu8, 0xFF, 0xFD, 0x00, 0x01, 0xFF, 0xFF, 0x01];
    assert_eq!(parse::<Crc>(&bad), Err(ParseError::BadLength { skip: 4 }));
}

#[test]
fn no_header_skips_all_when_no_partial_prefix() {
    assert_eq!(
        parse::<Crc>(&[0xAA, 0xBB, 0xCC, 0xDD]),
        Err(ParseError::Junk { skip: 4 })
    );
}

#[test]
fn header_prefix_suffix_retention() {
    // Longest trailing HEADER prefix is retained; the rest is junk.
    assert_eq!(
        parse::<Crc>(&[0xAA, 0xBB, 0xCC, 0xFF]),
        Err(ParseError::Junk { skip: 3 })
    );
    assert_eq!(
        parse::<Crc>(&[0xAA, 0xBB, 0xFF, 0xFF]),
        Err(ParseError::Junk { skip: 2 })
    );
    assert_eq!(
        parse::<Crc>(&[0xAA, 0xFF, 0xFF, 0xFD]),
        Err(ParseError::Junk { skip: 1 })
    );
}

#[test]
fn short_input_no_prefix_skips_immediately() {
    assert_eq!(
        parse::<Crc>(&[0xAA, 0xBB, 0xCC]),
        Err(ParseError::Junk { skip: 3 })
    );
}

#[test]
fn short_input_with_partial_prefix_returns_incomplete() {
    for partial in [&[0xFFu8][..], &[0xFF, 0xFF][..], &[0xFF, 0xFF, 0xFD][..]] {
        assert_eq!(parse::<Crc>(partial), Err(ParseError::Incomplete));
    }
}
