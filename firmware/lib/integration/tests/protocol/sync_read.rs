use crate::support::{Setup, assert_bus_healthy, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{CONFIG_REGION_SIZE, config::addr::comms};
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{Host, Servo, SimTime, format_hex, parse_status_stream};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;

#[apply(matrix)]
#[test_log::test]
fn sync_read_replies_in_id_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[1, 2, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    insta::assert_snapshot!("sync_read_replies_in_id_order", format_hex(&rx));

    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    assert_eq!(replies.len(), 3);
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

#[apply(matrix)]
#[test_log::test]
fn sync_read_single_id_replies_once(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[1]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

/// Per `sync_bulk_chain_snoop_contract`: error replies are still wire frames,
/// so slot k+1 still sees its predecessor and the chain stays alive even when
/// every slot errors. Length 0 trips DataRange on every slot.
#[apply(matrix)]
#[test_log::test]
fn sync_read_zero_length_errors_keep_chain_alive(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 0, &[1, 2, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    let expected: Vec<Status<'_>> = (1u8..=3)
        .map(|id| Status::Read {
            id: Id::new(id),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        })
        .collect();
    assert_eq!(replies, expected);
}

#[apply(matrix)]
#[test_log::test]
fn sync_read_across_region_boundary_returns_zeros_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_sync_read(CONFIG_REGION_END_ADDR - 2, 4, &[1, 2, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
    let expected: Vec<Status<'_>> = (1u8..=3)
        .map(|id| Status::Read {
            id: Id::new(id),
            error: StatusError::OK,
            data: &[0; 4],
        })
        .collect();
    assert_eq!(replies, expected);
}

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
    sim.device_mut::<Servo>(servos[1]).disconnect(false);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[1, 2, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    let replies = parse_status_stream(Instruction::SyncRead, &rx);
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
    sim.device_mut::<Servo>(servos[1])
        .set_status_return_level(StatusReturnLevel::None);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[1, 2, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[1, 99, 3]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

#[apply(matrix)]
#[test_log::test]
fn sync_read_all_unknown_ids_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, 1, &[99]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}
