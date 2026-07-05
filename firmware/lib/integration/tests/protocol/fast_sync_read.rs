use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{Id, Slot, StatusError};
use osc_core::BaudRate;
use osc_core::regions::{CONFIG_REGION_SIZE, config::addr::comms};
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_integration::sim::{FastStatusCrc, format_hex, parse_fast_sync_status};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE;
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
