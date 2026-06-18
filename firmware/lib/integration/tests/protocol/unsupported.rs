use crate::support::{Setup, setup};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::StatusReturnLevel;
use osc_integration::sim::{Host, Servo, Sim, SimTime, parse_status};

/// Reserved per the DXL 2.0 instruction byte table — guaranteed to surface as
/// `Instruction::Ext(_)` through the streaming parser, so `reply_unsupported`
/// is the only dispatcher path that can fire.
const UNKNOWN_INSTRUCTION_BYTE: u8 = 0xC3;

#[test_log::test]
fn reboot_to_self_id_replies_ok() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reboot(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::Reboot, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
}

#[test_log::test]
fn reboot_to_wrong_id_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reboot(Id::new(2));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn reboot_broadcast_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reboot(Id::BROADCAST);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn reboot_silent_when_srl_is_read() {
    let rx = reboot_under_srl(StatusReturnLevel::Read);
    assert!(rx.is_empty(), "reboot requires SRL=All; got {:?}", rx);
}

#[test_log::test]
fn unknown_instruction_to_self_id_replies_instruction_error() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ext(Id::new(1), UNKNOWN_INSTRUCTION_BYTE, &[]);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::Ext(UNKNOWN_INSTRUCTION_BYTE), &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Instruction),
        },
    );
}

#[test_log::test]
fn unknown_instruction_to_wrong_id_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ext(Id::new(2), UNKNOWN_INSTRUCTION_BYTE, &[]);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn unknown_instruction_broadcast_yields_no_reply() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_ext(
            Id::BROADCAST,
            UNKNOWN_INSTRUCTION_BYTE,
            &[],
        );
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test]
fn unknown_instruction_silent_when_srl_is_read() {
    let rx = ext_under_srl(StatusReturnLevel::Read);
    assert!(
        rx.is_empty(),
        "reply_unsupported requires SRL=All; got {:?}",
        rx,
    );
}

fn reboot_under_srl(level: StatusReturnLevel) -> Vec<u8> {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_status_return_level(level);
        })
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reboot(Id::new(1));
    });

    sim.device::<Host>(host).unwrap().rx_bytes()
}

fn ext_under_srl(level: StatusReturnLevel) -> Vec<u8> {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_status_return_level(level);
        })
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ext(Id::new(1), UNKNOWN_INSTRUCTION_BYTE, &[]);
    });

    sim.device::<Host>(host).unwrap().rx_bytes()
}
