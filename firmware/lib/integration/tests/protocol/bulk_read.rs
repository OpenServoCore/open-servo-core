use crate::support::{Setup, assert_bus_healthy, matrix, setup_with};
use dxl_protocol::types::{BulkReadEntry, ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, format_hex, parse_status_stream,
};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;

fn entry(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

#[apply(matrix)]
#[test_log::test]
fn bulk_read_replies_per_entry_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    // Heterogeneous (addr, length) per entry — the bulk-specific shape.
    let entries = [
        entry(1, comms::ID, 1),
        entry(2, identity::MODEL_NUMBER, 2),
        entry(3, identity::FIRMWARE_VERSION, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("bulk_read_replies_per_entry_in_order", format_hex(&rx));

    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    let model_le = DEFAULT_MODEL_NUMBER.to_le_bytes();
    let expected = vec![
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
        Status::Read {
            id: Id::new(2),
            error: StatusError::OK,
            data: &model_le,
        },
        Status::Read {
            id: Id::new(3),
            error: StatusError::OK,
            data: &[DEFAULT_FIRMWARE_VERSION],
        },
    ];
    assert_eq!(replies, expected);
}

#[apply(matrix)]
#[test_log::test]
fn bulk_read_single_entry_replies_once(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_bulk_read(&[entry(1, comms::ID, 1)]);
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
}

/// One entry has `length=0` (DataRange), but its `Status::Empty` is still a
/// wire frame, so slot k+1 sees it and the chain stays alive — same
/// snoop-contract guarantee as sync_read.
#[apply(matrix)]
#[test_log::test]
fn bulk_read_zero_length_entry_keeps_chain_alive(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 0),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    let expected = vec![
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
        Status::Read {
            id: Id::new(2),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
        Status::Read {
            id: Id::new(3),
            error: StatusError::OK,
            data: &[3],
        },
    ];
    assert_eq!(replies, expected);
}

/// One entry straddles the config region end → `read_bytes` zeros the OOB
/// portion, the reply is `Status::Read { OK, [0; 4] }`. Other entries with
/// in-range addresses reply normally.
#[apply(matrix)]
#[test_log::test]
fn bulk_read_entry_across_region_boundary_returns_zeros(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, CONFIG_REGION_END_ADDR - 2, 4),
        entry(3, comms::ID, 1),
    ];

    sim.with_host(host, |h| {
        h.send_bulk_read(&entries);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    let expected = vec![
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
        Status::Read {
            id: Id::new(2),
            error: StatusError::OK,
            data: &[0; 4],
        },
        Status::Read {
            id: Id::new(3),
            error: StatusError::OK,
            data: &[3],
        },
    ];
    assert_eq!(replies, expected);
}

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

#[apply(matrix)]
#[test_log::test]
fn bulk_read_all_unknown_ids_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_bulk_read(&[entry(99, comms::ID, 1)]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
