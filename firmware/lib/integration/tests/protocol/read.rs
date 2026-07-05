use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::{BaudRate, RegionStorage, StatusReturnLevel};
use osc_integration::sim::{DEFAULT_MODEL_NUMBER, Host, Servo, Sim, format_hex, parse_status};
use rstest::rstest;
use rstest_reuse::apply;

const CONFIG_INTRA_GAP_ADDR: u16 = 100;
const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE;
const OVER_MAX_CONTROL_RW: u16 = 129;

#[apply(matrix)]
#[test_log::test]
fn read_returns_rw_byte_on_comms_id_addr(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_read(Id::new(1), comms::ID, 1);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("read_returns_rw_byte_on_comms_id_addr", format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_returns_ro_word_on_identity_model_number(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_read(Id::new(1), identity::MODEL_NUMBER, 2);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!(
        "read_returns_ro_word_on_identity_model_number",
        format_hex(&rx)
    );
    let model_le = DEFAULT_MODEL_NUMBER.to_le_bytes();
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &model_le,
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_zero_length_replies_data_range(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(Id::new(1), comms::ID, 0, baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_length_over_cap_replies_data_range(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(
        Id::new(1),
        identity::MODEL_NUMBER,
        OVER_MAX_CONTROL_RW,
        baud,
        rdt_us,
    );
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_across_region_boundary_returns_zero_bytes(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(Id::new(1), CONFIG_REGION_END_ADDR - 2, 4, baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[0; 4],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_in_intra_region_gap_returns_zero_bytes(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(Id::new(1), CONFIG_INTRA_GAP_ADDR, 1, baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[0],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_to_wrong_id_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(Id::new(2), comms::ID, 1, baud, rdt_us);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn read_broadcast_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_with(Id::BROADCAST, comms::ID, 1, baud, rdt_us);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn read_silent_when_srl_is_none(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_under_srl(StatusReturnLevel::None, comms::ID, 1, baud, rdt_us);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn read_replies_when_srl_is_read(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = read_under_srl(StatusReturnLevel::Read, comms::ID, 1, baud, rdt_us);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn read_on_torque_locked_config_succeeds_locks_gate_writes_only(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    sim.servo(servos[0])
        .shared()
        .table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);

    sim.with_host(host, |h| {
        h.send_read(Id::new(1), comms::ID, 1);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

fn read_with(target: Id, addr: u16, length: u16, baud: BaudRate, rdt_us: u32) -> Vec<u8> {
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);
    sim.with_host(host, |h| {
        h.send_read(target, addr, length);
        h.wait_for_reply();
    });
    sim.host(host).rx_bytes()
}

fn read_under_srl(
    level: StatusReturnLevel,
    addr: u16,
    length: u16,
    baud: BaudRate,
    rdt_us: u32,
) -> Vec<u8> {
    let mut sim = Sim::default();
    let host = sim.add_device(move |id| Host::new(id).with_baud(baud));
    sim.add_device(move |id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_baud(baud);
            s.set_rdt_us(rdt_us);
            s.set_status_return_level(level);
        })
    });

    sim.with_host(host, |h| {
        h.send_read(Id::new(1), addr, length);
        h.wait_for_reply();
    });

    sim.host(host).rx_bytes()
}
