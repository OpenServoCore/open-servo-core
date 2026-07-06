//! Instruction-set semantics (`docs/osc-native-protocol.md` §5): each case
//! builds a sealed host instruction, runs the sim, and asserts on the decoded
//! status frame + table state. Reads as the spec — plain assertions, no
//! snapshots.

use osc_core::BaudRate;
use osc_core::regions::config::addr::comms::{BAUD_RATE_IDX, ID};
use osc_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, MgmtOp, Opcode, ResultCode};

const ID5: u8 = 5;

/// Servo frames on the wire, in order.
fn servo_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

/// Assert exactly one well-formed servo reply and return it.
fn sole_reply(frames: &[WireFrame]) -> &WireFrame {
    let replies = servo_frames(frames);
    assert_eq!(replies.len(), 1, "expected one servo reply: {frames:#?}");
    assert_valid(replies[0]);
    replies[0]
}

#[test]
fn ping_reports_model_and_fw() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert!(inst.is_status());
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let (model, fw) = sim.servo_table(s, |t| {
        (
            t.config.identity.model_number,
            t.config.identity.firmware_version,
        )
    });
    let m = model.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], fw]);
    assert!(
        inst.pad(),
        "model(2) + fw(1) = 3-byte payload is odd → padded"
    );
}

#[test]
fn read_returns_table_bytes() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // READ config identity span [0, 4): model(2) + fw(1) + reserved(1).
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let (model, fw) = sim.servo_table(s, |t| {
        (
            t.config.identity.model_number,
            t.config.identity.firmware_version,
        )
    });
    let m = model.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], fw, 0]);
}

#[test]
fn read_count_zero_is_range() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 0, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Range));
    assert!(payload.is_empty());
}

#[test]
fn read_over_ceiling_is_limit() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    // count 253 exceeds the 252 B frame ceiling (§5.1).
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 253, 0]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Limit));
}

#[test]
fn write_acks_and_applies() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &[a[0], a[1], 1]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[test]
fn write_noreply_is_silent_but_applies() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        Inst::FLAG_NOREPLY,
        &[a[0], a[1], 1],
    ));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[test]
fn broadcast_write_is_silent_but_applies() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    sim.host_send(&instruction(0xFE, Opcode::Write, 0, &[a[0], a[1], 1]));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[test]
fn hold_then_commit_applies_atomically() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);
    let a = TORQUE_ENABLE.to_le_bytes();

    // WRITE + HOLD stages; the live field stays untouched, ack Ok.
    sim.host_send(&instruction(
        ID5,
        Opcode::Write,
        Inst::FLAG_HOLD,
        &[a[0], a[1], 1],
    ));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(!sim.servo_table(s, |t| t.control.lifecycle.torque_enable));

    // Broadcast COMMIT applies the staged write silently.
    sim.host_send(&instruction(0xFE, Opcode::Commit, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));

    // A unicast COMMIT acks.
    sim.host_send(&instruction(ID5, Opcode::Commit, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}

#[test]
fn mgmt_reboot_acks_then_surfaces() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[MgmtOp::Reboot as u8]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert!(sim.take_reboot(s).is_some());
}

#[test]
fn mgmt_save_is_instruction_error() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    // SAVE (§9.4) is not yet implemented → instruction-level rejection.
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[MgmtOp::Save as u8]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
}

#[test]
fn read_with_profile_flag_is_instruction_error() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    // READ + HOLD names a profile slot (§5.2), deferred → instruction error.
    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        Inst::FLAG_HOLD,
        &[0, 0, 4, 0],
    ));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
}

#[test]
fn corrupt_crc_gets_no_reply_and_counts() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    let mut frame = instruction(ID5, Opcode::Write, 0, &[a[0], a[1], 1]);
    frame[4] ^= 0xFF; // corrupt a payload byte after sealing → CRC no longer matches

    sim.host_send(&frame);
    let frames = sim.run();

    // §5.3 layer 1: a corrupt frame is dropped silently and counted.
    assert!(servo_frames(&frames).is_empty());
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 1);
    assert_eq!(d.framing_drop_count, 0);
    assert!(!sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[test]
fn wrong_id_is_ignored() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    // Well-formed frame addressed to id 9; our servo is id 5.
    sim.host_send(&instruction(9, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

#[test]
fn alert_bit_mirrors_fault_flags() {
    let mut sim = Sim::new(BaudRate::B1000000);
    let s = sim.add_servo(ID5);

    sim.servo_table_mut(s, |t| t.telemetry.fault.fault_flags = 1);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert!(inst.alert(), "ALERT set while fault_flags nonzero (§5.3)");

    sim.servo_table_mut(s, |t| t.telemetry.fault.fault_flags = 0);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert!(!inst.alert());
}

#[test]
fn id_change_acks_old_then_answers_new() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    // WRITE the id field → 7; the ack leaves from the OLD id, then the id lands.
    let a = ID.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &[a[0], a[1], 7]));
    let frames = sim.run();
    let r = sole_reply(&frames);
    assert_eq!(
        r.from,
        Source::Servo(ID5),
        "ack carries the old responder id"
    );
    let (inst, _) = status(r);
    assert_eq!(inst.result(), Some(ResultCode::Ok));

    // A ping at the new id is answered.
    sim.host_send(&instruction(7, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert_eq!(sole_reply(&frames).from, Source::Servo(7));

    // A ping at the old id is silent.
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());
}

#[test]
fn baud_change_acks_then_switches() {
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(ID5);

    // WRITE baud_rate_idx → 3M. The ack leaves at the OLD 1M baud: the deferred
    // apply lands only after the reply fully drains (§4.2).
    let a = BAUD_RATE_IDX.to_le_bytes();
    sim.host_send_at(
        0,
        &instruction(
            ID5,
            Opcode::Write,
            0,
            &[a[0], a[1], BaudRate::B3000000 as u8],
        ),
    );
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));

    // Host follows the servo to 3M; a ping at the new baud is answered, proving
    // the switch took effect after the ack drained.
    //
    // The reciprocal "a ping still at the old 1M baud now garbles" holds on
    // hardware but is not asserted here: the sim's baud-mismatch model only
    // drops the slower-servo direction (a 3M servo's generous recheck plateau
    // absorbs the 1M byte cadence and the intact bytes still pass CRC). That
    // garble-after-switch path is exercised down-shift by
    // `resilience::rescue_pulse_drops_to_500k`.
    sim.set_host_baud(BaudRate::B3000000);
    sim.host_send_at(600, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}
