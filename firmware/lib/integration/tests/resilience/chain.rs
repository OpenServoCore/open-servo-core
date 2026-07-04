//! Chain-snoop contract per [[sync_bulk_chain_snoop_contract]] — for
//! Sync/Bulk Read (Plain and Fast), slot k>0 fires only after seeing
//! slot k-1's Status frame on the wire; a silent predecessor collapses
//! the chain tail. Error replies still count as wire frames and keep
//! the chain alive; unknown ids / disconnected servos / SRL=None break
//! it.
//!
//! Tests are grouped by instruction (Plain SyncRead, Plain BulkRead,
//! FastSyncRead, FastBulkRead) so "what does the snoop contract
//! guarantee?" reads as one file rather than a hunt across
//! `tests/protocol/*.rs`. All follow the same shape: three-servo chain,
//! break slot 1 by one of {data-line disconnect, power-line disconnect,
//! SRL=None, unknown id}, observe that slot 0 still replies and slot 2
//! is silent (Plain) or the trailing Status CRC fails to patch (Fast).
//!
//! Foreign Status frames interleaved with the chain aren't tested — the
//! sim's wire model panics on wire collisions, so injecting an unrelated
//! Status frame between slots isn't cleanly expressible. The parser-side
//! defense against that scenario lives in `dxl-protocol`'s streaming
//! parser tests.

use crate::support::{Setup, assert_bus_healthy, matrix, setup_with};
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, Slot, Status, StatusError};
use osc_core::regions::config::addr::comms;
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{
    FastStatusCrc, HOST_INTER_BYTE_TIMEOUT, SimTime, parse_fast_bulk_status,
    parse_fast_sync_status, parse_status_stream,
};
use rstest::rstest;
use rstest_reuse::apply;

fn entry(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

// ---------------- Plain SyncRead ----------------

/// Silent predecessor (data-line disconnect on the middle servo) collapses
/// the chain tail — servo 1 replies, servo 3 stays armed but never sees slot
/// 1's frame. Reconnect + `assert_bus_healthy` confirms no servo's chain
/// state stays latched across the collapse (regression for
/// `dxl-streaming-rx.md` §5.3 — chain-pending reset at next instruction
/// header).
#[apply(matrix)]
#[test_log::test]
fn sync_read_data_line_disconnect_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1]).disconnect(false);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );

    sim.servo_mut(servos[1]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Power-line variant of the disconnect-collapses-tail case — one
/// representative on Plain SyncRead. Chain-pending state on the
/// bystander servos must clear at the next instruction header
/// regardless of whether the missing servo comes back with default
/// state or preserved RAM. No `assert_bus_healthy` because power-line
/// reset wipes the missing servo's id back to `DEFAULT_DXL_ID`, which
/// collides with servo 1's id on a broadcast Ping recovery check; the
/// chain-latch clearing is the load-bearing invariant and is already
/// exercised by the data-line variant's recovery.
#[test_log::test]
fn sync_read_power_line_disconnect_collapses_tail() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, BaudRate::from_idx(0).unwrap(), 0);
    sim.servo_mut(servos[1]).disconnect(true);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
}

/// Same chain-collapse mechanic as the disconnect case, but driven by
/// SRL=None on the middle servo — the dispatcher stays running, it just never
/// emits a Status frame for Sync Read, so servo 3's snoop never fires.
#[apply(matrix)]
#[test_log::test]
fn sync_read_srl_none_predecessor_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Same collapse mechanic again — the middle id simply doesn't exist on the
/// bus, so no predecessor frame ever appears for servo 3 to snoop. Servo 1
/// (slot 0, RDT-driven) replies; servo 3 stays armed and silent.
#[apply(matrix)]
#[test_log::test]
fn sync_read_unknown_id_in_chain_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, 1, &[1, 99, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

// ---------------- Plain BulkRead ----------------

/// Silent predecessor (data-line disconnect on the middle servo) collapses
/// the chain tail — servo 1 replies, servo 3 stays armed but never sees
/// slot 1's frame. Reconnect + `assert_bus_healthy` confirms no servo's
/// chain state stays latched across the collapse (same regression coverage
/// as sync_read's mirror test, for `dxl-streaming-rx.md` §5.3).
#[apply(matrix)]
#[test_log::test]
fn bulk_read_data_line_disconnect_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1]).disconnect(false);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );

    sim.servo_mut(servos[1]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// SRL=None on the middle servo — the dispatcher stays running but emits
/// no Status frame for BulkRead, so servo 3's snoop never fires.
#[apply(matrix)]
#[test_log::test]
fn bulk_read_srl_none_predecessor_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Middle entry targets an id that doesn't exist on the bus, so no
/// predecessor frame for servo 3's snoop. Servo 1 (slot 0, RDT-driven)
/// replies; servo 3 stays armed and silent.
#[apply(matrix)]
#[test_log::test]
fn bulk_read_unknown_id_in_entries_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(99, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

// ---------------- Fast SyncRead ----------------

/// Middle servo SRL=None → `handle_fast_read` returns silent → no slot 1
/// wire bytes. Slot 2 still fires (FAST slot k>0 anchors on the chain's
/// observed status START — slot 0 is alive, so the grid holds regardless
/// of slot 1's silence), so its `[err, id, data]` lands on the wire —
/// but the Fast Last fold can't reach the predecessor-bytes threshold
/// without slot 1's bytes, so the trailing Status CRC never gets patched.
/// Wire shape is slot 0 + slot 2 with broken CRC; a real host rejects
/// the Status reply via CRC validation while the decoder surfaces both
/// pieces.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_srl_none_predecessor_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_sync_status(&rx, 1);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Data-line disconnect on the middle servo — slot 1 emits no bytes. Slot
/// 2 fires anyway (status-start-anchored grid; slot 0 supplied the
/// anchor) so its frame lands on the wire,
/// but the trailing Status CRC can't be patched without slot 1's bytes
/// feeding the Fast Last fold. Wire = slot 0 + slot 2 + broken CRC; host
/// rejects via CRC validation. Reconnect + `assert_bus_healthy` confirms
/// no Fast Last state stays latched across the collapse.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_data_line_disconnect_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1]).disconnect(false);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_sync_status(&rx, 1);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );

    sim.servo_mut(servos[1]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Middle id doesn't exist on the bus — no slot 1 frame, same CRC
/// collapse as the disconnect / SRL=None cases. Slot 2 still fires via
/// the status-start grid; trailing CRC fails validation; host rejects the Status
/// reply.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_unknown_id_in_ids_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 99, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_sync_status(&rx, 1);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );
    assert_bus_healthy(&mut sim, host, &servos);
}

// ---------------- Fast BulkRead ----------------

/// Middle servo SRL=None → `handle_fast_read` returns silent → no slot 1
/// wire bytes. Slot 2 still fires (status-start-anchored grid),
/// but the Fast Last fold can't patch the trailing CRC without slot 1's
/// bytes. Same collapse mechanic as the fast_sync_read mirror.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_srl_none_predecessor_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );
    // Quiesce past the advertised chain frame's byte-skip horizon before
    // the health check — see the collapse tests in
    // `protocol/fast_bulk_read.rs` for the rationale.
    sim.with_host(host, |h| {
        h.wait_for_reply_within(SimTime::from_ms(200), HOST_INTER_BYTE_TIMEOUT);
    });
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Data-line disconnect on the middle servo — slot 1 emits no bytes. Slot
/// 2 fires anyway (status-start-anchored grid) so its frame lands on the wire, but
/// the trailing Status CRC can't be patched without slot 1's bytes feeding
/// the Fast Last fold. Reconnect + `assert_bus_healthy` confirms no Fast
/// Last state stays latched across the collapse.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_data_line_disconnect_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[1]).disconnect(false);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );

    sim.servo_mut(servos[1]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Middle entry targets an id that doesn't exist on the bus — no slot 1
/// frame, same CRC collapse as the disconnect / SRL=None cases. Slot 2
/// still fires on the status-start grid; trailing CRC fails validation.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_unknown_id_in_entries_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(99, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
    assert_eq!(status.crc, FastStatusCrc::Truncated);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );
    // Quiesce past the advertised chain frame's byte-skip horizon before
    // the health check — see the collapse tests in
    // `protocol/fast_bulk_read.rs` for the rationale.
    sim.with_host(host, |h| {
        h.wait_for_reply_within(SimTime::from_ms(200), HOST_INTER_BYTE_TIMEOUT);
    });
    assert_bus_healthy(&mut sim, host, &servos);
}

// ---------------- Fast chain: first servo silent (task #142) ----------------

/// First servo SRL=None — the chain's Status packet never starts, so no
/// slot k > 0 ever gets its anchor: the WHOLE reply stays silent (slots
/// k > 0 defer to the observed status start; no observation, no wire
/// start). A host retry must not provoke a stale fire either — each
/// parked slot's per-byte wake sees the retry's own bytes, judges the
/// stamp past the staleness window, and drops the slot instead of
/// scheduling into the host's instruction. The trailing broadcast Ping
/// (`assert_bus_healthy`) also regression-pins the un-parking: every
/// servo answers normally after two dead FAST exchanges.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_first_servo_silent_keeps_chain_silent(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[0])
        .set_status_return_level(StatusReturnLevel::None);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "no status start observed → no slot may fire",
    );

    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "host retry must not trigger a stale fire from a parked slot",
    );

    assert_bus_healthy(&mut sim, host, &servos);
}

/// Power-line variant — the first servo drops off the bus entirely
/// (chip-side reset on reconnect) instead of parsing-but-not-replying.
/// Same contract: no anchor, no tail, and the bus recovers. Runs at the
/// factory-default baud so the power-cycled servo (whose staged baud is
/// wiped by the reset) still hears the recovery Ping.
#[rstest]
#[test_log::test]
fn fast_sync_read_first_servo_power_line_disconnect_keeps_chain_silent() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, BaudRate::from_idx(3).unwrap(), 250);
    sim.servo_mut(servos[0]).disconnect(true);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "no status start observed → no slot may fire",
    );

    sim.servo_mut(servos[0]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Fast Bulk variant of the first-servo-silent contract.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_first_servo_silent_keeps_chain_silent(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);
    sim.servo_mut(servos[0])
        .set_status_return_level(StatusReturnLevel::None);

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&[entry(1, comms::ID, 1), entry(2, 0, 2), entry(3, 2, 1)]);
        h.wait_for_reply();
    });
    assert!(
        sim.host(host).rx_bytes().is_empty(),
        "no status start observed → no slot may fire",
    );

    assert_bus_healthy(&mut sim, host, &servos);
}
