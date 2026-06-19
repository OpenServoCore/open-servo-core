use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::{BaudRate, StatusReturnLevel};
use osc_integration::sim::{Host, Servo, Sim, parse_status};
use rstest::rstest;
use rstest_reuse::apply;

/// Reserved per the DXL 2.0 instruction byte table — guaranteed to surface as
/// `Instruction::Ext(_)` through the streaming parser, so `reply_unsupported`
/// is the only dispatcher path that can fire.
const UNKNOWN_INSTRUCTION_BYTE: u8 = 0xC3;

#[apply(matrix)]
#[test_log::test]
fn reboot_to_self_id_replies_ok(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_reboot(Id::new(1));
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Reboot, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn reboot_to_wrong_id_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_reboot(Id::new(2));
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn reboot_broadcast_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_reboot(Id::BROADCAST);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn reboot_silent_when_srl_is_read(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = reboot_under_srl(StatusReturnLevel::Read, baud, rdt_us);
    assert!(rx.is_empty(), "reboot requires SRL=All; got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn unknown_instruction_to_self_id_replies_instruction_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ext(Id::new(1), UNKNOWN_INSTRUCTION_BYTE, &[]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::Ext(UNKNOWN_INSTRUCTION_BYTE), &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Instruction),
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn unknown_instruction_to_wrong_id_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ext(Id::new(2), UNKNOWN_INSTRUCTION_BYTE, &[]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn unknown_instruction_broadcast_yields_no_reply(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ext(Id::BROADCAST, UNKNOWN_INSTRUCTION_BYTE, &[]);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[apply(matrix)]
#[test_log::test]
fn unknown_instruction_silent_when_srl_is_read(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let rx = ext_under_srl(StatusReturnLevel::Read, baud, rdt_us);
    assert!(
        rx.is_empty(),
        "reply_unsupported requires SRL=All; got {:?}",
        rx,
    );
}

fn reboot_under_srl(level: StatusReturnLevel, baud: BaudRate, rdt_us: u32) -> Vec<u8> {
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
        h.send_reboot(Id::new(1));
        h.wait_for_reply();
    });

    sim.host(host).rx_bytes()
}

fn ext_under_srl(level: StatusReturnLevel, baud: BaudRate, rdt_us: u32) -> Vec<u8> {
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
        h.send_ext(Id::new(1), UNKNOWN_INSTRUCTION_BYTE, &[]);
        h.wait_for_reply();
    });

    sim.host(host).rx_bytes()
}
