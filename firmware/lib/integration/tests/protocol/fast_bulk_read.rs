use crate::support::{Setup, assert_bus_healthy, matrix, setup_with};
use dxl_protocol::types::{BulkReadEntry, Id, Slot, StatusError};
use osc_core::BaudRate;
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, FastStatusCrc, format_hex,
    parse_fast_bulk_status,
};
use osc_integration::sim::{HOST_INTER_BYTE_TIMEOUT, SimTime};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE;
const OVER_MAX_CONTROL_RW: u16 = MAX_CONTROL_RW as u16 + 1;

fn entry(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_replies_per_entry_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    // Heterogeneous (addr, length) per entry — the bulk-specific shape.
    let entries = [
        entry(1, comms::ID, 1),
        entry(2, identity::MODEL_NUMBER, 2),
        entry(3, identity::FIRMWARE_VERSION, 1),
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("fast_bulk_read_replies_per_entry_in_order", format_hex(&rx));

    let status = parse_fast_bulk_status(&rx, &entries);
    assert_eq!(status.crc, FastStatusCrc::Good);
    let model_le = DEFAULT_MODEL_NUMBER.to_le_bytes();
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
                data: &model_le,
            },
            Slot {
                id: Id::new(3),
                error: StatusError::OK,
                data: &[DEFAULT_FIRMWARE_VERSION],
            },
        ],
    );
}

#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_single_entry_replies_once(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    let entries = [entry(1, comms::ID, 1)];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
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

/// Middle entry has `length == 0`. `handle_fast_read` returns silent on
/// `len == 0` BEFORE the slot is emitted, so slot 1 puts nothing on the
/// wire. Slot 2 still fires (CC-compare-driven) but the Fast Last fold
/// can't reach the predecessor-bytes threshold without slot 1's bytes, so
/// the trailing Status CRC never gets patched. Differs from plain Bulk
/// Read where a zero-length entry emits `Status::Empty + DataRange` and
/// keeps the chain alive — Fast has no error-only slot shape.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_zero_length_entry_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 0),
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
    // The advertised chain LENGTH stays live as the universal byte-skip's
    // give-up horizon on every listening servo even though the chain
    // collapsed — an instruction sent inside that horizon is (correctly)
    // consumed as skipped bytes; the skip is byte-count + deadline
    // driven and blind to fresh headers. Quiesce past the horizon the
    // way a real host's retry timeout does before the health check
    // (worst case: ~150 ms for a 147-byte frame at 9600).
    sim.with_host(host, |h| {
        h.wait_for_reply_within(SimTime::from_ms(200), HOST_INTER_BYTE_TIMEOUT);
    });
    assert_bus_healthy(&mut sim, host, &servos);
}

/// Same per-entry short-circuit as the zero-length case:
/// `len > MAX_CONTROL_RW` silences slot 1 → predecessor bytes missing →
/// trailing CRC stays unpatched.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_length_over_cap_entry_collapses_tail(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, OVER_MAX_CONTROL_RW),
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
    // The advertised chain LENGTH stays live as the universal byte-skip's
    // give-up horizon on every listening servo even though the chain
    // collapsed — an instruction sent inside that horizon is (correctly)
    // consumed as skipped bytes; the skip is byte-count + deadline
    // driven and blind to fresh headers. Quiesce past the horizon the
    // way a real host's retry timeout does before the health check
    // (worst case: ~150 ms for a 147-byte frame at 9600).
    sim.with_host(host, |h| {
        h.wait_for_reply_within(SimTime::from_ms(200), HOST_INTER_BYTE_TIMEOUT);
    });
    assert_bus_healthy(&mut sim, host, &servos);
}

/// One entry straddles the config region end → `read_bytes` zero-fills
/// OOB and returns Ok, so the slot still emits `length` bytes. Every slot
/// stays whole; chain Stays alive; CRC valid.
#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_entry_across_region_boundary_returns_zeros(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, CONFIG_REGION_END_ADDR - 2, 4),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
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
                data: &[0; 4],
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
fn fast_bulk_read_all_unknown_ids_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&[entry(99, comms::ID, 1)]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
