use crate::fixtures::{OneServo, one_servo};
use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use osc_integration::sim::{Host, SimTime, format_hex, parse_status};
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
