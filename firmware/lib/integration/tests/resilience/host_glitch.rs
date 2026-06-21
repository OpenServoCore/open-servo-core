//! Host-side packet shape pathologies — the servo must recover from any
//! truncated, malformed, or cross-baud byte sequence and answer the next
//! valid request cleanly. Every test follows the same shape: send bad
//! bytes, then `assert_bus_healthy` (broadcast Ping that all servos must
//! answer in chain order) confirms the parser, dispatcher, scheduler, and
//! inflight state all returned to a known-good baseline.
//!
//! Full baud × RDT matrix — RDT primarily bounds reply-side timing, but
//! sweeping it surfaces interactions between RDT-driven scheduler state
//! and the recovery path that a baud-only sweep would miss.

use crate::support::{SYNC_PREFIX, Setup, assert_bus_healthy, encode_ping, matrix, setup_with};
use dxl_protocol::types::Id;
use osc_core::BaudRate;
use rstest::rstest;
use rstest_reuse::apply;

const TARGET: Id = Id::new(1);

/// 256 bytes of 0xFF — drains any mid-packet parser state without
/// itself producing a sync signature. Phase::Crc consumes 2 as bad
/// CRC → reset; Phase::Header completes with garbage → bad length →
/// resync; Phase::Payload/Slots advances to Crc → bad CRC → reset.
/// All paths leave the parser back at Phase::Sync after at most ~10
/// bytes, with the remaining FFs harmlessly drained.
const PARSER_FLUSH: [u8; 256] = [0xFFu8; 256];

/// Drop bytes on the wire and settle. Used by `*_recovers_immediately`
/// tests where the bad input self-drains the parser back to Phase::Sync
/// without help, so `assert_bus_healthy` can land its broadcast Ping
/// directly on top of the recovery.
fn send_and_settle(
    sim: &mut osc_integration::sim::Sim,
    host: osc_integration::sim::DeviceId,
    bytes: &[u8],
) {
    sim.with_host(host, |h| {
        h.send_raw(bytes);
        h.wait_for_reply();
    });
}

/// Drop bytes on the wire, settle, then drop the 256-byte parser flush
/// in a second `with_host` (host `now` advances past the first batch's
/// TX end — the sim's UART panics on back-to-back overlapping frames).
/// Used by `*_recovers_after_retry` tests where the parser is stuck
/// mid-state (Phase::Crc / Header / Payload waiting for bytes that
/// never arrive); the flush models the prefix of a real host's retry
/// that drains the stuck state. Without it, the recovery Ping's sync
/// prefix would be swallowed as the missing tail of the prior packet.
fn send_with_retry(
    sim: &mut osc_integration::sim::Sim,
    host: osc_integration::sim::DeviceId,
    bytes: &[u8],
) {
    sim.with_host(host, |h| {
        h.send_raw(bytes);
        h.wait_for_reply();
    });
    sim.with_host(host, |h| {
        h.send_raw(&PARSER_FLUSH);
        h.wait_for_reply();
    });
}

/// Sync-only flood — N copies of the `FF FF FD 00` signature with no
/// header bytes following. Each flips the parser to `Header` stage; the
/// next overlapping signature must keep the parser in resync rather than
/// committing to a half-formed packet. The servo's dispatcher must hold
/// no stale inflight context when the recovery broadcast lands.
#[apply(matrix)]
#[test_log::test]
fn sync_only_flood_recovers_after_retry(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let mut flood: Vec<u8> = Vec::new();
    for _ in 0..16 {
        flood.extend_from_slice(&SYNC_PREFIX);
    }
    send_with_retry(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Sync + partial header (1 to 3 of the 4 header bytes). Parser sits
/// in `Header` stage waiting for the rest; the next valid packet's Sync
/// must hard-resync without leaking inflight state from the truncated id.
#[apply(matrix)]
#[test_log::test]
fn sync_plus_partial_header_recovers_after_retry(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    for take in 1..=3 {
        let Setup {
            mut sim,
            host,
            servos,
        } = setup_with(1, baud, rdt_us);
        // Real ping bytes, truncated mid-header (after 4 sync + take bytes).
        let mut bytes = encode_ping(0x01);
        bytes.truncate(4 + take);
        send_with_retry(&mut sim, host, &bytes);
        assert_bus_healthy(&mut sim, host, &servos);
    }
}

/// Sync + complete instruction header but no CRC. For Ping (LEN=3,
/// no params), the parser reaches the CRC stage after consuming all 8
/// header-region bytes. The servo's driver has already set `inflight`
/// at `Header(Instruction(Ping))`; without the 2 CRC bytes there's no
/// `Crc` event to commit a reply, no `Resync` to clear state. The next
/// packet's Sync must clear the stale inflight by construction (own
/// addressable target or not).
#[apply(matrix)]
#[test_log::test]
fn header_without_crc_recovers_after_retry(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let mut bytes = encode_ping(TARGET.as_byte());
    let len_with_crc = bytes.len();
    bytes.truncate(len_with_crc - 2);
    send_with_retry(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Sync + header + 1 of 2 CRC bytes. Parser sits in `Crc` stage with
/// one byte buffered. Driver's `inflight` still set from the Header
/// event. Recovery hits the same Sync-resyncs-everything path.
#[apply(matrix)]
#[test_log::test]
fn header_with_partial_crc_recovers_after_retry(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let mut bytes = encode_ping(TARGET.as_byte());
    let len_with_crc = bytes.len();
    bytes.truncate(len_with_crc - 1);
    send_with_retry(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Header declaring an impossible LEN field (0xFFFF). Parser's
/// length-overflow guard must emit `Resync(BadLength)` rather than wait
/// for 65 533 body bytes. Sent as a complete-looking 10-byte packet so
/// the parser commits to the LEN before the truncation matters.
#[apply(matrix)]
#[test_log::test]
fn impossible_length_field_recovers_immediately(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    // FF FF FD 00 | ID=01 | LEN_LO=FF LEN_HI=FF | INSTR=01 (ping) | junk*2
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&SYNC_PREFIX);
    bytes.push(0x01);
    bytes.push(0xFF);
    bytes.push(0xFF);
    bytes.push(0x01);
    bytes.push(0x00);
    bytes.push(0x00);
    send_and_settle(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Near-sync patterns — three bytes shy of the signature, then garbage.
/// Verifies the sync-stage doesn't lock on a partial match.
#[apply(matrix)]
#[test_log::test]
fn near_sync_patterns_recover_immediately(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let near_patterns: &[&[u8]] = &[
        &[0xFF, 0xFF, 0xFD, 0xFF], // signature with bad 4th byte
        &[0xFF, 0xFF, 0xFE, 0x00], // signature with bad 3rd byte
        &[0xFE, 0xFF, 0xFD, 0x00], // signature with bad 1st byte
        &[0xFF, 0xFE, 0xFD, 0x00], // signature with bad 2nd byte
        &[0xFF, 0xFF, 0xFF, 0xFD], // shifted signature
    ];
    for pattern in near_patterns {
        let Setup {
            mut sim,
            host,
            servos,
        } = setup_with(1, baud, rdt_us);
        send_and_settle(&mut sim, host, pattern);
        assert_bus_healthy(&mut sim, host, &servos);
    }
}

/// All-0xFF flood — a stuck-high wire or hammered TX lane. The sync
/// stage requires the full `FF FF FD 00` so no false-trigger; the
/// dispatcher must hold no spurious state when the recovery lands.
#[apply(matrix)]
#[test_log::test]
fn all_ff_flood_recovers_immediately(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let flood = vec![0xFFu8; 256];
    send_and_settle(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// All-0x00 flood — stuck-low wire / break condition. Same recovery
/// surface as the all-0xFF case.
#[apply(matrix)]
#[test_log::test]
fn all_zero_flood_recovers_immediately(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let flood = vec![0x00u8; 256];
    send_and_settle(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Valid packet body, but the last byte's CRC bit-flipped. Parser
/// emits `Crc(Bad)`, driver drops the inflight reply. Recovery confirms
/// the bad-CRC path doesn't leak inflight state across packets.
#[apply(matrix)]
#[test_log::test]
fn bad_crc_recovers_immediately(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    let mut bytes = encode_ping(TARGET.as_byte());
    let last = bytes.len() - 1;
    bytes[last] ^= 0xFF;
    send_and_settle(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Cross-baud noise — host TXes a valid Ping at the WRONG baud (one
/// step off from the servo's). The servo's UART samples the wire at
/// its own baud, producing arbitrary garbage bytes. The garbage must
/// not coincidentally form a valid signature + header + good CRC, and
/// the dispatcher must hold no spurious state when the host returns to
/// the correct baud.
#[apply(matrix)]
#[test_log::test]
fn cross_baud_noise_recovers_immediately(baud_idx: u8, rdt_us: u32) {
    let servo_baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    // Pick a host baud that's one step away on the table; wrap so every
    // servo baud has a paired mismatch test.
    let host_baud =
        BaudRate::from_idx(if baud_idx == 0 { 1 } else { baud_idx - 1 }).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, servo_baud, rdt_us);
    // Override the host's baud after sim setup. The servo stays at
    // `servo_baud` so the wire's bit-time mismatch produces garbled bytes.
    sim.host_mut(host).set_baud(host_baud);

    let bytes = encode_ping(TARGET.as_byte());
    sim.with_host(host, |h| {
        h.send_raw(&bytes);
        h.wait_for_reply();
    });

    // Restore the host to the servo's baud and verify recovery.
    sim.host_mut(host).set_baud(servo_baud);
    assert_bus_healthy(&mut sim, host, &servos);
}
