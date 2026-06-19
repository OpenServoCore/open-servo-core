use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, Host, Servo, Sim, format_hex, parse_status,
    parse_status_stream,
};
use rstest::rstest;
use rstest_reuse::apply;

#[apply(matrix)]
#[test_log::test]
fn ping_to_self_id_returns_model_and_fw(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ping(Id::new(1));
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("ping_to_self_id_returns_model_and_fw", format_hex(&rx));
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

#[apply(matrix)]
#[test_log::test]
fn ping_to_wrong_id_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ping(Id::new(2));
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn ping_replies_when_srl_is_none(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = ping_under_srl(StatusReturnLevel::None, baud, rdt_us);
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

#[apply(matrix)]
#[test_log::test]
fn ping_replies_when_srl_is_read(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = ping_under_srl(StatusReturnLevel::Read, baud, rdt_us);
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

#[apply(matrix)]
#[test_log::test]
fn ping_broadcast_replies_in_id_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    insta::assert_snapshot!("ping_broadcast_replies_in_id_order", format_hex(&rx));

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

fn ping_under_srl(level: StatusReturnLevel, baud: BaudRate, rdt_us: u32) -> Vec<u8> {
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
        h.send_ping(Id::new(1));
        h.wait_for_reply();
    });

    sim.host(host).rx_bytes()
}
