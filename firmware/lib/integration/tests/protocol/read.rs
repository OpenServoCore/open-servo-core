use crate::support::{Setup, setup};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    CONFIG_REGION_SIZE,
    config::addr::{comms, identity},
};
use osc_core::{RegionStorage, StatusReturnLevel};
use osc_integration::sim::{
    DEFAULT_MODEL_NUMBER, Host, Servo, Sim, SimTime, format_hex, parse_status,
};

const CONFIG_INTRA_GAP_ADDR: u16 = 100;
const CONFIG_REGION_END_ADDR: u16 = CONFIG_REGION_SIZE as u16;
const OVER_MAX_CONTROL_RW: u16 = 129;

#[test_log::test]
fn read_returns_rw_byte_on_comms_id_addr() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_read(Id::new(1), comms::ID, 1);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

#[test_log::test]
fn read_returns_ro_word_on_identity_model_number() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_read(Id::new(1), identity::MODEL_NUMBER, 2);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
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

#[test_log::test]
fn read_zero_length_replies_data_range() {
    let rx = read_with(Id::new(1), comms::ID, 0);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
    );
}

#[test_log::test]
fn read_length_over_cap_replies_data_range() {
    let rx = read_with(Id::new(1), identity::MODEL_NUMBER, OVER_MAX_CONTROL_RW);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
    );
}

#[test_log::test]
fn read_over_region_boundary_replies_data_range() {
    let rx = read_with(Id::new(1), CONFIG_REGION_END_ADDR - 2, 4);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::DataRange),
            data: &[],
        },
    );
}

#[test_log::test]
fn read_into_intra_region_gap_replies_access_error() {
    let rx = read_with(Id::new(1), CONFIG_INTRA_GAP_ADDR, 1);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
            data: &[],
        },
    );
}

#[test_log::test]
fn read_to_wrong_id_yields_no_reply() {
    let rx = read_with(Id::new(2), comms::ID, 1);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn read_broadcast_yields_no_reply() {
    let rx = read_with(Id::BROADCAST, comms::ID, 1);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn read_silent_when_srl_is_none() {
    let rx = read_under_srl(StatusReturnLevel::None, comms::ID, 1);
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn read_replies_when_srl_is_read() {
    let rx = read_under_srl(StatusReturnLevel::Read, comms::ID, 1);
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

#[test_log::test]
fn read_on_torque_locked_config_succeeds_locks_gate_writes_only() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);
    sim.device::<Servo>(servos[0])
        .unwrap()
        .shared()
        .table
        .control
        .with_mut(|c| c.lifecycle.torque_enable = true);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_read(Id::new(1), comms::ID, 1);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::Read, &rx),
        Status::Read {
            id: Id::new(1),
            error: StatusError::OK,
            data: &[1],
        },
    );
}

fn read_with(target: Id, addr: u16, length: u16) -> Vec<u8> {
    let Setup { mut sim, host, .. } = setup(1);
    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_read(target, addr, length);
    });
    sim.device::<Host>(host).unwrap().rx_bytes()
}

fn read_under_srl(level: StatusReturnLevel, addr: u16, length: u16) -> Vec<u8> {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    sim.add_device(|id| {
        Servo::new(id)
            .with_dxl_id(Id::new(1))
            .with_status_return_level(level)
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_read(Id::new(1), addr, length);
    });

    sim.device::<Host>(host).unwrap().rx_bytes()
}
