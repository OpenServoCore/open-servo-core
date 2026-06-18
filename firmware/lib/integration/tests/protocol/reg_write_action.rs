use crate::support::{Setup, setup};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    config::addr::{comms, identity},
    control::{Mode, addr::lifecycle},
};
use osc_core::{RegionStorage, StatusReturnLevel};
use osc_integration::sim::{
    Host, Servo, Sim, SimTime, format_hex, parse_status, parse_status_stream,
};

const NEW_ID: u8 = 2;
const TORQUE_ON: &[u8] = &[1];
const MODE_PID: &[u8] = &[Mode::PositionPid as u8];

#[test_log::test]
fn reg_write_alone_replies_ok_and_does_not_mutate_table() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::RegWrite, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        !sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "RegWrite must stage only — table mutation belongs to Action",
    );
}

#[test_log::test]
fn action_alone_with_no_staged_writes_replies_ok() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    insta::assert_snapshot!(format_hex(&rx));
    assert_eq!(
        parse_status(Instruction::Action, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        !sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[test_log::test]
fn reg_write_then_action_commits_to_table() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    let replies = parse_status_stream(Instruction::RegWrite, &rx);
    assert_eq!(replies.len(), 2, "expected RegWrite + Action replies");
    assert_eq!(
        replies[0],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert_eq!(
        replies[1],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[test_log::test]
fn chain_of_two_reg_writes_commits_atomically_on_action() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reg_write(Id::new(1), lifecycle::MODE, MODE_PID);
    });
    sim.advance(SimTime::from_ms(15), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    let replies = parse_status_stream(Instruction::RegWrite, &rx);
    assert_eq!(replies.len(), 3);
    for r in &replies {
        assert_eq!(
            *r,
            Status::Empty {
                id: Id::new(1),
                error: StatusError::OK,
            },
        );
    }
    let lc = sim
        .device::<Servo>(servos[0])
        .unwrap()
        .shared()
        .table
        .control
        .with(|c| c.lifecycle);
    assert!(lc.torque_enable);
    assert_eq!(lc.mode, Mode::PositionPid);
}

#[test_log::test]
fn action_clears_staged_queue_subsequent_action_is_noop() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });
    assert!(
        sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );

    sim.device::<Servo>(servos[0])
        .unwrap()
        .set_torque_enabled(false);

    sim.advance(SimTime::from_ms(15), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    assert!(
        !sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "Action #2 must commit an empty queue; A's staged entry must not replay",
    );
}

#[test_log::test]
fn reg_write_to_ro_field_replies_access_error() {
    let Setup { mut sim, host, .. } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            identity::FIRMWARE_VERSION,
            &[0x42],
        );
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert_eq!(
        parse_status(Instruction::RegWrite, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
}

#[test_log::test]
fn prior_staged_writes_survive_a_failed_reg_write() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            identity::FIRMWARE_VERSION,
            &[0x42],
        );
    });
    sim.advance(SimTime::from_ms(15), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    let replies = parse_status_stream(Instruction::RegWrite, &rx);
    assert_eq!(replies.len(), 3);
    assert_eq!(
        replies[0],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert_eq!(
        replies[1],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
    assert_eq!(
        replies[2],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "A's staged TORQUE_ENABLE=1 must survive B's rejection and commit on Action",
    );
}

#[test_log::test]
fn reg_write_broadcast_stages_silently_action_broadcast_commits_silently() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::BROADCAST,
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::BROADCAST);
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(rx.is_empty(), "broadcast RegWrite + Action must be silent");
    assert!(
        sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[test_log::test]
fn reg_write_silent_when_srl_is_read_but_action_still_commits() {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servo = sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_status_return_level(StatusReturnLevel::Read);
        })
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    let rx_after_reg = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(
        rx_after_reg.is_empty(),
        "RegWrite reply only at SRL=All, got {:?}",
        rx_after_reg,
    );

    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });
    let rx_after_action = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(
        rx_after_action.is_empty(),
        "Action reply only at SRL=All, got {:?}",
        rx_after_action,
    );
    assert!(
        sim.device::<Servo>(servo)
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[test_log::test]
fn action_silent_when_srl_is_read_after_visible_reg_write() {
    let mut sim = Sim::default();
    let host = sim.add_device(Host::new);
    let servo = sim.add_device(|id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_status_return_level(StatusReturnLevel::Read);
        })
    });

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    assert!(
        rx.is_empty(),
        "expected no replies at SRL=Read, got {:?}",
        rx
    );
    assert!(
        sim.device::<Servo>(servo)
            .unwrap()
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[test_log::test]
fn reg_write_under_torque_lock_replies_access_error() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);
    sim.device::<Servo>(servos[0])
        .unwrap()
        .set_torque_enabled(true);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reg_write(Id::new(1), comms::ID, &[NEW_ID]);
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    let replies = parse_status_stream(Instruction::RegWrite, &rx);
    assert_eq!(replies.len(), 2);
    assert_eq!(
        replies[0],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
    assert_eq!(
        replies[1],
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert_eq!(
        sim.device::<Servo>(servos[0])
            .unwrap()
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        1,
        "Action must not replay the rejected RegWrite",
    );
}

#[test_log::test]
fn inline_write_between_reg_writes_does_not_drain_staged_queue() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup(1);

    sim.advance(SimTime::from_ms(5), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_reg_write(
            Id::new(1),
            lifecycle::TORQUE_ENABLE,
            TORQUE_ON,
        );
    });
    sim.advance(SimTime::from_ms(10), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_reg_write(Id::new(1), lifecycle::MODE, MODE_PID);
    });
    let velocity_payload = 1_000i32.to_le_bytes();
    sim.advance(SimTime::from_ms(15), |sim, _| {
        sim.device_mut::<Host>(host).unwrap().send_write(
            Id::new(1),
            lifecycle::GOAL_VELOCITY,
            &velocity_payload,
        );
    });
    sim.advance(SimTime::from_ms(20), |sim, _| {
        sim.device_mut::<Host>(host)
            .unwrap()
            .send_action(Id::new(1));
    });

    let rx = sim.device::<Host>(host).unwrap().rx_bytes();
    let replies = parse_status_stream(Instruction::Write, &rx);
    assert_eq!(replies.len(), 4);
    for r in &replies {
        assert_eq!(
            *r,
            Status::Empty {
                id: Id::new(1),
                error: StatusError::OK,
            },
        );
    }
    let lc = sim
        .device::<Servo>(servos[0])
        .unwrap()
        .shared()
        .table
        .control
        .with(|c| c.lifecycle);
    assert!(lc.torque_enable, "RegWrite A must commit on Action");
    assert_eq!(
        lc.mode,
        Mode::PositionPid,
        "RegWrite B must commit on Action"
    );
    assert_eq!(lc.goal_velocity, 1_000, "inline Write C lands immediately");
}
