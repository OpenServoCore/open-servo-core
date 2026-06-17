use crate::fixtures::{OneServo, one_servo};
use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use osc_core::{RegionStorage, StatusReturnLevel};
use osc_integration::sim::{Host, Servo, SimTime, format_hex, parse_status};
use rstest::rstest;

#[test_log::test(rstest)]
fn ping_to_dxl_id_zero_returns_model_and_fw(one_servo: OneServo) {
    let OneServo { mut sim, host, .. } = one_servo;

    sim.advance(SimTime::from_ms(5), |sim, now| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ping(now, Id::new(0));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(0),
            error: StatusError::OK,
            status: PingStatus {
                model: 0,
                fw_version: 0,
            },
        },
    );
}

#[test_log::test(rstest)]
fn ping_to_wrong_id_yields_no_reply(one_servo: OneServo) {
    let OneServo { mut sim, host, .. } = one_servo;

    sim.advance(SimTime::from_ms(5), |sim, now| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ping(now, Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "expected silent drop, got {:?}", rx);
}

#[test_log::test(rstest)]
fn ping_replies_when_srl_is_none(one_servo: OneServo) {
    let OneServo {
        mut sim,
        host,
        servo,
    } = one_servo;
    set_srl(&mut sim, servo, StatusReturnLevel::None);

    sim.advance(SimTime::from_ms(5), |sim, now| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ping(now, Id::new(0));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(0),
            error: StatusError::OK,
            status: PingStatus {
                model: 0,
                fw_version: 0,
            },
        },
    );
}

#[test_log::test(rstest)]
fn ping_replies_when_srl_is_read(one_servo: OneServo) {
    let OneServo {
        mut sim,
        host,
        servo,
    } = one_servo;
    set_srl(&mut sim, servo, StatusReturnLevel::Read);

    sim.advance(SimTime::from_ms(5), |sim, now| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_ping(now, Id::new(0));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::Ping, &rx),
        Status::Ping {
            id: Id::new(0),
            error: StatusError::OK,
            status: PingStatus {
                model: 0,
                fw_version: 0,
            },
        },
    );
}

fn set_srl(
    sim: &mut osc_integration::sim::Sim,
    servo: osc_integration::sim::DeviceId,
    level: StatusReturnLevel,
) {
    sim.device::<Servo>(servo)
        .unwrap()
        .shared()
        .table
        .config
        .with_mut(|c| c.comms.status_return_level = level);
}
