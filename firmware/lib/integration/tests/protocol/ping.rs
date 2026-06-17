use crate::support::{Setup, setup};
use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use osc_core::StatusReturnLevel;
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, Host, Servo, Sim, SimTime, format_hex,
    parse_status, parse_status_stream,
};

#[test_log::test]
fn ping_to_self_id_returns_model_and_fw() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_ping(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(1),
            error: StatusError::OK,
            status: PingStatus {
                model: DEFAULT_MODEL_NUMBER,
                fw_version: DEFAULT_FIRMWARE_VERSION,
            },
        },
    );
}

#[test_log::test]
fn ping_to_wrong_id_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_ping(Id::new(2));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn ping_replies_when_srl_is_none() {
    let rx = ping_under_srl(StatusReturnLevel::None);
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(1),
            error: StatusError::OK,
            status: PingStatus {
                model: DEFAULT_MODEL_NUMBER,
                fw_version: DEFAULT_FIRMWARE_VERSION,
            },
        },
    );
}

#[test_log::test]
fn ping_replies_when_srl_is_read() {
    let rx = ping_under_srl(StatusReturnLevel::Read);
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(1),
            error: StatusError::OK,
            status: PingStatus {
                model: DEFAULT_MODEL_NUMBER,
                fw_version: DEFAULT_FIRMWARE_VERSION,
            },
        },
    );
}

#[test_log::test]
fn ping_broadcast_replies_in_id_order() {
    let Setup { mut sim, host, .. } = setup(3);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ping(Id::BROADCAST);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));

    let replies = parse_status_stream(Instruction::Ping, &rx);
    let expected: Vec<Status<'_>> = (1u8..=3)
        .map(|id| Status::Ping {
            id: Id::new(id),
            error: StatusError::OK,
            status: PingStatus {
                model: DEFAULT_MODEL_NUMBER,
                fw_version: DEFAULT_FIRMWARE_VERSION,
            },
        })
        .collect();
    assert_eq!(replies, expected);
}

fn ping_under_srl(level: StatusReturnLevel) -> Vec<u8> {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    sim.add_device(|id| {
        Servo::new(id)
            .with_dxl_id(Id::new(1))
            .with_status_return_level(level)
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_ping(Id::new(1));
    });

    sim.device::<Host>(host).unwrap().rx_bytes()
}
