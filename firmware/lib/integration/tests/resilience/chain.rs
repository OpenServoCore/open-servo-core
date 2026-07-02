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
    FastStatusCrc, parse_fast_bulk_status, parse_fast_sync_status, parse_status_stream,
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
/// wire bytes. Slot 2 still fires (Fast Sync schedules slot k>0 by
/// CC-compare, not snoop), so its `[err, id, data]` lands on the wire —
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
/// 2 fires anyway (CC-compare-driven) so its frame lands on the wire,
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
/// CC-compare; trailing CRC fails validation; host rejects the Status
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
/// wire bytes. Slot 2 still fires (Fast schedules slot k>0 by CC-compare),
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
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Data-line disconnect on the middle servo — slot 1 emits no bytes. Slot
/// 2 fires anyway (CC-compare-driven) so its frame lands on the wire, but
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
/// still fires via CC-compare; trailing CRC fails validation.
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
    assert_bus_healthy(&mut sim, host, &servos);
}
