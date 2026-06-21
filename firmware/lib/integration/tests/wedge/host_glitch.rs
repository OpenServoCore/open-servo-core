//! Host-side packet shape pathologies — the servo must never wedge on a
//! truncated, malformed, or cross-baud byte sequence. Every test follows
//! the same shape: send garbage, then `assert_bus_healthy` (broadcast
//! Ping that the servo must answer in chain order) confirms the servo's
//! parser, dispatcher, scheduler, and inflight state recovered.
//!
//! Baud sweep only — RDT is irrelevant to the wedge surface (RDT bounds
//! reply-side timing, not how the streaming parser tolerates bad input).

use crate::support::{
    SYNC_PREFIX, Setup, assert_bus_healthy, baud_matrix, encode_ping, setup_with,
};
use dxl_protocol::types::Id;
use osc_core::BaudRate;
use osc_integration::sim::DEFAULT_RDT_US;
use rstest::rstest;
use rstest_reuse::apply;

const TARGET: Id = Id::new(1);

/// 256 bytes of 0xFF — drains any mid-packet parser state without
/// itself producing a sync signature. Phase::Crc consumes 2 as bad
/// CRC → reset; Phase::Header completes with garbage → bad length →
/// resync; Phase::Payload/Slots advances to Crc → bad CRC → reset.
/// All paths leave the parser back at Phase::Sync after at most ~10
/// bytes, with the remaining FFs harmlessly drained. Mirrors real-host
/// recovery behavior: hosts retry packets that don't get a reply, and
/// each retry's prefix flushes any stuck state from the prior wedge.
const PARSER_FLUSH: [u8; 256] = [0xFFu8; 256];

/// Send a raw byte slice from the host, settle, then a parser flush in
/// a second `with_host` (so the host's `now` advances past the first
/// batch's TX end — the sim's UART panics if back-to-back bytes overlap
/// in flight). The flush models real-host retry behavior: without it a
/// truncated packet's leftover state would consume the next packet's
/// sync prefix and we'd be testing recovery latency, not whether the
/// servo wedges.
fn send_and_settle(
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
#[apply(baud_matrix)]
#[test_log::test]
fn sync_only_flood_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    let mut flood: Vec<u8> = Vec::new();
    for _ in 0..16 {
        flood.extend_from_slice(&SYNC_PREFIX);
    }
    send_and_settle(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Sync + partial header (1 to 3 of the 4 header bytes). Parser sits
/// in `Header` stage waiting for the rest; the next valid packet's Sync
/// must hard-resync without leaking inflight state from the truncated id.
#[apply(baud_matrix)]
#[test_log::test]
fn sync_plus_partial_header_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    for take in 1..=3 {
        let Setup {
            mut sim,
            host,
            servos,
        } = setup_with(1, baud, DEFAULT_RDT_US);
        // Real ping bytes, truncated mid-header (after 4 sync + take bytes).
        let mut bytes = encode_ping(0x01);
        bytes.truncate(4 + take);
        send_and_settle(&mut sim, host, &bytes);
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
#[apply(baud_matrix)]
#[test_log::test]
fn header_without_crc_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    let mut bytes = encode_ping(TARGET.as_byte());
    let len_with_crc = bytes.len();
    bytes.truncate(len_with_crc - 2);
    send_and_settle(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Sync + header + 1 of 2 CRC bytes. Parser sits in `Crc` stage with
/// one byte buffered. Driver's `inflight` still set from the Header
/// event. Recovery hits the same Sync-resyncs-everything path.
#[apply(baud_matrix)]
#[test_log::test]
fn header_with_partial_crc_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    let mut bytes = encode_ping(TARGET.as_byte());
    let len_with_crc = bytes.len();
    bytes.truncate(len_with_crc - 1);
    send_and_settle(&mut sim, host, &bytes);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Header declaring an impossible LEN field (0xFFFF). Parser's
/// length-overflow guard must emit `Resync(BadLength)` rather than wait
/// for 65 533 body bytes. Sent as a complete-looking 10-byte packet so
/// the parser commits to the LEN before the truncation matters.
#[apply(baud_matrix)]
#[test_log::test]
fn impossible_length_field_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

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
#[apply(baud_matrix)]
#[test_log::test]
fn near_sync_patterns_do_not_wedge(baud_idx: u8) {
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
        } = setup_with(1, baud, DEFAULT_RDT_US);
        send_and_settle(&mut sim, host, pattern);
        assert_bus_healthy(&mut sim, host, &servos);
    }
}

/// All-0xFF flood — a stuck-high wire or hammered TX lane. The sync
/// stage requires the full `FF FF FD 00` so no false-trigger; the
/// dispatcher must hold no spurious state when the recovery lands.
#[apply(baud_matrix)]
#[test_log::test]
fn all_ff_flood_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    let flood = vec![0xFFu8; 256];
    send_and_settle(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// All-0x00 flood — stuck-low wire / break condition. Same recovery
/// surface as the all-0xFF case.
#[apply(baud_matrix)]
#[test_log::test]
fn all_zero_flood_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    let flood = vec![0x00u8; 256];
    send_and_settle(&mut sim, host, &flood);

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Valid packet body, but the last byte's CRC bit-flipped. Parser
/// emits `Crc(Bad)`, driver drops the inflight reply. Recovery confirms
/// the bad-CRC path doesn't leak inflight state across packets.
#[apply(baud_matrix)]
#[test_log::test]
fn bad_crc_does_not_wedge(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

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
#[apply(baud_matrix)]
#[test_log::test]
fn cross_baud_noise_does_not_wedge(baud_idx: u8) {
    let servo_baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    // Pick a host baud that's one step away on the table; wrap so every
    // servo baud has a paired mismatch test.
    let host_baud =
        BaudRate::from_idx(if baud_idx == 0 { 1 } else { baud_idx - 1 }).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, servo_baud, DEFAULT_RDT_US);
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
