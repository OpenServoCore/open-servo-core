use crate::support::{Setup, assert_bus_healthy, matrix, setup_with};
use dxl_protocol::types::{Id, Slot, StatusError};
use osc_core::regions::{CONFIG_REGION_SIZE, config::addr::comms};
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{FastStatusCrc, format_hex, parse_fast_sync_status};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;
const OVER_MAX_CONTROL_RW: u16 = MAX_CONTROL_RW as u16 + 1;

#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_replies_per_id_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("fast_sync_read_replies_per_id_in_order", format_hex(&rx));

    let status = parse_fast_sync_status(&rx, 1);
    assert_eq!(status.crc, FastStatusCrc::Good);
    assert_eq!(
        status.slots,
        vec![
            Slot {
                id: Id::new(1),
                error: StatusError::OK,
                data: &[1],
            },
            Slot {
                id: Id::new(2),
                error: StatusError::OK,
                data: &[2],
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[3],
            },
        ],
    );
}

#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_single_target_replies_once(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[1]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_sync_status(&rx, 1);
    assert_eq!(status.crc, FastStatusCrc::Good);
    assert_eq!(
        status.slots,
        vec![Slot {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );
}

/// `handle_fast_read` returns silent on `len == 0` BEFORE any slot is
/// emitted — no header, no Status reply at all. Differs from Plain Sync
/// Read where the dispatcher still emits a per-slot DataRange error
/// frame; for Fast, the per-slot wire shape has no room for an
/// "error-only" slot, so the whole reply is suppressed.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_zero_length_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 0, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

/// Same suppression mechanic as zero-length: `len > MAX_CONTROL_RW` short-
/// circuits before slot emission.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_length_over_cap_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, OVER_MAX_CONTROL_RW, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

/// `read_bytes` zero-fills OOB and returns Ok, so each slot emits OK +
/// zero-data. The Status reply stays whole because every slot still
/// emits its `length` bytes.
#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_across_region_boundary_returns_zeros(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(CONFIG_REGION_END_ADDR - 2, 4, &[1, 2, 3]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_sync_status(&rx, 4);
    assert_eq!(status.crc, FastStatusCrc::Good);
    let expected: Vec<Slot<'_>> = (1u8..=3)
        .map(|id| Slot {
            id: Id::new(id),
            error: StatusError::OK,
            data: &[0; 4],
        })
        .collect();
    assert_eq!(status.slots, expected);
}

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

#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_all_unknown_ids_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_sync_read(comms::ID, 1, &[99]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
