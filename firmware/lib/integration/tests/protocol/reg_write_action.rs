use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{ErrorCode, Id, Instruction, Status, StatusError};
use osc_core::regions::{
    config::addr::{comms, identity},
    control::{Mode, addr::lifecycle},
};
use osc_core::{BaudRate, RegionStorage, StatusReturnLevel};
use osc_integration::sim::{
    Host, Servo, Sim, SimTime, format_hex, parse_status, parse_status_stream,
};
use rstest::rstest;
use rstest_reuse::apply;

const NEW_ID: u8 = 2;
const TORQUE_ON: &[u8] = &[1];
const MODE_PID: &[u8] = &[Mode::PositionPid as u8];

#[apply(matrix)]
#[test_log::test]
fn reg_write_alone_replies_ok_and_does_not_mutate_table(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    insta::assert_snapshot!(
        "reg_write_alone_replies_ok_and_does_not_mutate_table",
        format_hex(&rx)
    );
    assert_eq!(
        parse_status(Instruction::RegWrite, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        !sim.device::<Servo>(servos[0])
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "RegWrite must stage only — table mutation belongs to Action",
    );
}

#[apply(matrix)]
#[test_log::test]
fn action_alone_with_no_staged_writes_replies_ok(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    insta::assert_snapshot!(
        "action_alone_with_no_staged_writes_replies_ok",
        format_hex(&rx)
    );
    assert_eq!(
        parse_status(Instruction::Action, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::OK,
        },
    );
    assert!(
        !sim.device::<Servo>(servos[0])
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[apply(matrix)]
#[test_log::test]
fn reg_write_then_action_commits_to_table(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));

    let rx = sim.device::<Host>(host).rx_bytes();
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
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[apply(matrix)]
#[test_log::test]
fn chain_of_two_reg_writes_commits_atomically_on_action(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::MODE, MODE_PID);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(15));

    let rx = sim.device::<Host>(host).rx_bytes();
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
        .shared()
        .table
        .control
        .with(|c| c.lifecycle);
    assert!(lc.torque_enable);
    assert_eq!(lc.mode, Mode::PositionPid);
}

#[apply(matrix)]
#[test_log::test]
fn action_clears_staged_queue_subsequent_action_is_noop(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));
    assert!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );

    sim.device::<Servo>(servos[0]).set_torque_enabled(false);

    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(15));

    assert!(
        !sim.device::<Servo>(servos[0])
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "Action #2 must commit an empty queue; A's staged entry must not replay",
    );
}

#[apply(matrix)]
#[test_log::test]
fn reg_write_to_ro_field_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), identity::FIRMWARE_VERSION, &[0x42]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert_eq!(
        parse_status(Instruction::RegWrite, &rx),
        Status::Empty {
            id: Id::new(1),
            error: StatusError::code(ErrorCode::Access),
        },
    );
}

#[apply(matrix)]
#[test_log::test]
fn prior_staged_writes_survive_a_failed_reg_write(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), identity::FIRMWARE_VERSION, &[0x42]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(15));

    let rx = sim.device::<Host>(host).rx_bytes();
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
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
        "A's staged TORQUE_ENABLE=1 must survive B's rejection and commit on Action",
    );
}

#[apply(matrix)]
#[test_log::test]
fn reg_write_broadcast_stages_silently_action_broadcast_commits_silently(
    baud_idx: u8,
    rdt_us: u32,
) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::BROADCAST, lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host).send_action(Id::BROADCAST);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(rx.is_empty(), "broadcast RegWrite + Action must be silent");
    assert!(
        sim.device::<Servo>(servos[0])
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[apply(matrix)]
#[test_log::test]
fn reg_write_silent_when_srl_is_read_but_action_still_commits(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::default();
    let host = sim.add_device(move |id| Host::new(id).with_baud(baud));
    let servo = sim.add_device(move |id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_baud(baud);
            s.set_rdt_us(rdt_us);
            s.set_status_return_level(StatusReturnLevel::Read);
        })
    });

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    let rx_after_reg = sim.device::<Host>(host).rx_bytes();
    assert!(
        rx_after_reg.is_empty(),
        "RegWrite reply only at SRL=All, got {:?}",
        rx_after_reg,
    );

    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));
    let rx_after_action = sim.device::<Host>(host).rx_bytes();
    assert!(
        rx_after_action.is_empty(),
        "Action reply only at SRL=All, got {:?}",
        rx_after_action,
    );
    assert!(
        sim.device::<Servo>(servo)
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[apply(matrix)]
#[test_log::test]
fn action_silent_when_srl_is_read_after_visible_reg_write(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::default();
    let host = sim.add_device(move |id| Host::new(id).with_baud(baud));
    let servo = sim.add_device(move |id| {
        Servo::setup(id, |s| {
            s.set_dxl_id(Id::new(1));
            s.set_baud(baud);
            s.set_rdt_us(rdt_us);
            s.set_status_return_level(StatusReturnLevel::Read);
        })
    });

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));

    let rx = sim.device::<Host>(host).rx_bytes();
    assert!(
        rx.is_empty(),
        "expected no replies at SRL=Read, got {:?}",
        rx
    );
    assert!(
        sim.device::<Servo>(servo)
            .shared()
            .table
            .control
            .with(|c| c.lifecycle.torque_enable),
    );
}

#[apply(matrix)]
#[test_log::test]
fn reg_write_under_torque_lock_replies_access_error(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    sim.device::<Servo>(servos[0]).set_torque_enabled(true);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), comms::ID, &[NEW_ID]);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));

    let rx = sim.device::<Host>(host).rx_bytes();
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
            .shared()
            .table
            .config
            .with(|c| c.comms.id),
        1,
        "Action must not replay the rejected RegWrite",
    );
}

#[apply(matrix)]
#[test_log::test]
fn inline_write_between_reg_writes_does_not_drain_staged_queue(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);

    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::TORQUE_ENABLE, TORQUE_ON);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(5));
    sim.device_mut::<Host>(host)
        .send_reg_write(Id::new(1), lifecycle::MODE, MODE_PID);
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(10));
    let velocity_payload = 1_000i32.to_le_bytes();
    sim.device_mut::<Host>(host).send_write(
        Id::new(1),
        lifecycle::GOAL_VELOCITY,
        &velocity_payload,
    );
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(15));
    sim.device_mut::<Host>(host).send_action(Id::new(1));
    sim.device_mut::<Host>(host).wait_for_status();
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
