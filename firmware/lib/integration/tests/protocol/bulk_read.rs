use crate::support::{Setup, assert_bus_healthy, setup};
use dxl_protocol::types::{BulkReadEntry, ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::StatusReturnLevel;
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, Host, Servo, SimTime, format_hex,
    parse_status_stream,
};

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;

fn entry(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

#[test_log::test]
fn bulk_read_replies_per_entry_in_order() {
    let Setup { mut sim, host, .. } = setup(3);

    // Heterogeneous (addr, length) per entry — the bulk-specific shape.
    let entries = [
        entry(1, comms::ID, 1),
        entry(2, identity::MODEL_NUMBER, 2),
        entry(3, identity::FIRMWARE_VERSION, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));

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

#[test_log::test]
fn bulk_read_single_entry_replies_once() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.device_mut::<Host>(host)
        .send_bulk_read(&[entry(1, comms::ID, 1)]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
#[test_log::test]
fn bulk_read_zero_length_entry_keeps_chain_alive() {
    let Setup { mut sim, host, .. } = setup(3);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 0),
        entry(3, comms::ID, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
#[test_log::test]
fn bulk_read_entry_across_region_boundary_returns_zeros() {
    let Setup { mut sim, host, .. } = setup(3);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, CONFIG_REGION_END_ADDR - 2, 4),
        entry(3, comms::ID, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
#[test_log::test]
fn bulk_read_data_line_disconnect_collapses_tail() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);
    sim.device_mut::<Servo>(servos[1]).disconnect(false);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    let replies = parse_status_stream(Instruction::BulkRead, &rx);
    assert_eq!(
        replies,
        vec![Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        }],
    );

    sim.device_mut::<Servo>(servos[1]).connect();
    assert_bus_healthy(&mut sim, host, &servos);
}

/// SRL=None on the middle servo — the dispatcher stays running but emits
/// no Status frame for BulkRead, so servo 3's snoop never fires.
#[test_log::test]
fn bulk_read_srl_none_predecessor_collapses_tail() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);
    sim.device_mut::<Servo>(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    let entries = [
        entry(1, comms::ID, 1),
        entry(2, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
#[test_log::test]
fn bulk_read_unknown_id_in_entries_collapses_tail() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(3);

    let entries = [
        entry(1, comms::ID, 1),
        entry(99, comms::ID, 1),
        entry(3, comms::ID, 1),
    ];

    sim.device_mut::<Host>(host).send_bulk_read(&entries);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

#[test_log::test]
fn bulk_read_all_unknown_ids_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(3);

    sim.device_mut::<Host>(host)
        .send_bulk_read(&[entry(99, comms::ID, 1)]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
