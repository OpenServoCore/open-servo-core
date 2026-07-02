//! Legit production timing patterns the servo must handle
//! transparently — these are NOT pathologies, they're shapes a
//! real-world host will produce. Grouped here so the "what timing edge
//! cases does the parser guarantee?" surface reads as one file:
//!
//! - **No-IDLE bursts** — per [[idle_not_packet_boundary]], the parser
//!   cursor is the sole packet boundary. A host that back-to-backs
//!   SyncWrite + Ping / Read with zero inter-packet IDLE must still
//!   parse both frames.
//! - **RDT=0 chain corners** — with no lift, slot k>0 has no reply
//!   delay budget; the chain must still land in-order.
//! - **Host baud shift** — legit re-tune after driver reconfig; the
//!   servo has no way to know until fresh bytes arrive at the right
//!   bit-time.
//!
//! Slow-host inter-byte gaps within a single packet are not tested
//! here — `Host::send_raw` queues bytes at baud-stride pacing (see
//! `Host::queue_frame`), so exposing a byte-by-byte queuing API just for
//! this scenario would be scope creep. The parser's IDLE-doesn't-boundary
//! invariant is already exercised by the no-IDLE burst tests below.

use crate::support::{
    Setup, assert_bus_healthy, encode_bulk_write, encode_sync_read, encode_sync_write, matrix,
    setup_with,
};
use dxl_protocol::types::{Id, Instruction, Status, StatusError};
use osc_core::BaudRate;
use osc_core::regions::config::addr::comms;
use osc_integration::sim::{DEFAULT_BAUD, DEFAULT_RDT_US, parse_status_stream};
use rstest::rstest;
use rstest_reuse::apply;

const TARGET: Id = Id::new(1);

/// Concatenate SyncWrite (no reply, silent broadcast) + Ping onto one
/// wire burst with no inter-packet IDLE. The parser must find the
/// second packet's sync signature from its own cursor state, not from a
/// wire-level IDLE. Baud sweep because TX pacing scales with baud.
#[apply(matrix)]
#[test_log::test]
fn back_to_back_sync_write_then_ping_no_idle_succeeds(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    // SyncWrite that no-ops on our single servo: length=1, empty body →
    // the body has no id-1 chunk, so it's a broadcast-silent write with
    // no per-servo effect. Then a Ping to the servo — must reply.
    let mut burst = encode_sync_write(comms::ID, 0, &[]);
    burst.extend_from_slice(&crate::support::encode_ping(TARGET.as_byte()));

    sim.with_host(host, |h| {
        h.send_raw(&burst);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::Ping, &rx);
    assert_eq!(
        replies.len(),
        1,
        "expected 1 Ping reply after no-IDLE burst, got {:?}",
        replies,
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Same shape with BulkWrite as the leading no-reply instruction.
/// Baud-independent seam — one representative baud is enough.
#[test_log::test]
fn back_to_back_bulk_write_then_ping_no_idle_succeeds() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, DEFAULT_BAUD, DEFAULT_RDT_US);

    // BulkWrite with a body chunk that skips the servo (id=99 doesn't
    // match), so it's a broadcast-silent no-op.
    let mut chunk = Vec::new();
    chunk.push(99u8);
    chunk.extend_from_slice(&comms::ID.to_le_bytes());
    chunk.extend_from_slice(&1u16.to_le_bytes());
    chunk.push(0);
    let mut burst = encode_bulk_write(&chunk);
    burst.extend_from_slice(&crate::support::encode_ping(TARGET.as_byte()));

    sim.with_host(host, |h| {
        h.send_raw(&burst);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::Ping, &rx);
    assert_eq!(
        replies.len(),
        1,
        "expected 1 Ping reply after no-IDLE BulkWrite+Ping burst, got {:?}",
        replies,
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

/// SyncWrite followed by SyncRead on a 3-servo chain, no IDLE between.
/// Guards that a leading no-reply instruction doesn't leave chain-pending
/// state that a following Read chain would inherit. Baud sweep exercises
/// the parser cursor at every wire pace.
#[apply(matrix)]
#[test_log::test]
fn sync_read_after_immediate_sync_write_no_idle_replies_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    // Silent-broadcast SyncWrite (length=0) followed by a SyncRead of
    // `comms::ID` across all three ids.
    let mut burst = encode_sync_write(comms::ID, 0, &[]);
    burst.extend_from_slice(&encode_sync_read(comms::ID, 1, &[1, 2, 3]));

    sim.with_host(host, |h| {
        h.send_raw(&burst);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies.len(),
        3,
        "expected 3 chained replies, got {:?}",
        replies
    );
    for (i, reply) in replies.iter().enumerate() {
        let id = (i as u8) + 1;
        assert_eq!(
            *reply,
            Status::Read {
                id: Id::new(id),
                error: StatusError::OK,
                data: &[id],
            },
        );
    }
}

/// RDT=0 across the chain — slot k>0 has no reply-delay budget. Chain
/// must still land in-order. Baud sweep only (single RDT); at high baud
/// the inter-slot gap is tightest.
#[apply(matrix)]
#[test_log::test]
fn rdt_zero_chain_replies_in_order(baud_idx: u8, rdt_us: u32) {
    let _ = rdt_us;
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, 0);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(replies.len(), 3, "expected 3 chained replies at RDT=0");
    for (i, reply) in replies.iter().enumerate() {
        let id = (i as u8) + 1;
        assert_eq!(
            *reply,
            Status::Read {
                id: Id::new(id),
                error: StatusError::OK,
                data: &[id],
            },
        );
    }
}

/// Host retunes its baud legit-ly (driver reconfig): first Ping at the
/// wrong baud is garbled bytes on the wire → servo doesn't parse a
/// packet; then the host restores the baud and the next Ping replies
/// cleanly. Same recovery path as `cross_baud_noise_recovers_immediately`
/// in `host_glitch`, framed as legit re-tune rather than glitch.
#[test_log::test]
fn host_baud_shift_recovers_when_restored() {
    let servo_baud = BaudRate::from_idx(3).expect("valid baud idx");
    let host_baud = BaudRate::from_idx(2).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, servo_baud, DEFAULT_RDT_US);

    // Host tunes down one step, sends Ping — garbled at the servo.
    sim.host_mut(host).set_baud(host_baud);
    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });

    // Restore host baud and confirm recovery.
    sim.host_mut(host).set_baud(servo_baud);
    assert_bus_healthy(&mut sim, host, &servos);
}
