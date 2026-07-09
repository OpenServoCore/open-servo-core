//! Instruction-set semantics (`docs/osc-native-protocol.md` §5): each case
//! builds a sealed host instruction, runs the sim, and asserts on the decoded
//! status frame + table state. Reads as the spec — plain assertions, no
//! snapshots.

use osc_core::BaudRate;
use osc_core::regions::CALIB_BASE_ADDR;
use osc_core::regions::config::addr::comms::{BAUD_RATE_IDX, ID};
use osc_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, MgmtOp, Opcode, ResultCode};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{matrix, sim};

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

#[apply(matrix)]
fn ping_reports_model_and_fw(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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
}

#[apply(matrix)]
fn read_returns_table_bytes(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn read_front_loaded_reply_matches_and_leaves_table(baud_idx: u8) {
    // A READ is dispatched at covered-complete and CRC-verified at the frame end
    // (the front-loaded path): the reply is byte-identical and the read-only op
    // never touches the table.
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    let before = sim.servo_table(s, |t| t.config.identity.model_number);

    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let m = before.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], 0x56, 0]);
    assert_eq!(sim.servo_diag(s).crc_fail_count, 0);
    assert_eq!(
        sim.servo_table(s, |t| t.config.identity.model_number),
        before,
        "a read leaves the table untouched"
    );
}

#[apply(matrix)]
fn read_count_zero_is_range(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 0, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Range));
    assert!(payload.is_empty());
}

#[apply(matrix)]
fn read_over_ceiling_is_limit(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // count 253 exceeds the 252 B frame ceiling (§5.1).
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 253, 0]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Limit));
}

#[apply(matrix)]
fn write_acks_and_applies(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &[a[0], a[1], 1]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[apply(matrix)]
fn write_noreply_is_silent_but_applies(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn broadcast_write_is_silent_but_applies(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    let a = TORQUE_ENABLE.to_le_bytes();
    sim.host_send(&instruction(0xFE, Opcode::Write, 0, &[a[0], a[1], 1]));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    assert!(sim.servo_table(s, |t| t.control.lifecycle.torque_enable));
}

#[apply(matrix)]
fn hold_then_commit_applies_atomically(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn large_write_stages_and_applies(baud_idx: u8) {
    // LEN is the only size limit (§5.1): a >128 B write stages like any other
    // (the buffer fits the largest legal write) and commits at its verdict.
    // Target the calib pot-LUT span (writable, no field rules).
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    let mut data = vec![0u8; 130];
    data[0..2].copy_from_slice(&0x1234u16.to_le_bytes()); // raw_min
    data[2..4].copy_from_slice(&0x5678u16.to_le_bytes()); // raw_max
    data[4] = 0xAB; // first LUT byte

    let a = CALIB_BASE_ADDR.to_le_bytes();
    let mut payload = vec![a[0], a[1]];
    payload.extend_from_slice(&data);
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &payload));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(sim.servo_diag(s).crc_fail_count, 0);
    let (raw_min, raw_max, lut0) = sim.servo_table(s, |t| {
        (
            t.calib.pot_lut.raw_min,
            t.calib.pot_lut.raw_max,
            t.calib.pot_lut.lut[0],
        )
    });
    assert_eq!(raw_min, 0x1234);
    assert_eq!(raw_max, 0x5678);
    assert_eq!((lut0 as u32) & 0xFF, 0xAB, "the >128 B payload landed");
}

#[apply(matrix)]
fn mgmt_reboot_acks_then_surfaces(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[MgmtOp::Reboot as u8]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert!(sim.take_reboot(s).is_some());
}

#[apply(matrix)]
fn mgmt_save_is_instruction_error(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // SAVE (§9.4) is not yet implemented → instruction-level rejection.
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[MgmtOp::Save as u8]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
}

#[apply(matrix)]
fn read_with_profile_flag_is_instruction_error(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn corrupt_crc_gets_no_reply_and_counts(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn wrong_id_is_ignored(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // Well-formed frame addressed to id 9; our servo is id 5.
    sim.host_send(&instruction(9, Opcode::Ping, 0, &[]));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    let d = sim.servo_diag(s);
    assert_eq!(d.crc_fail_count, 0);
    assert_eq!(d.framing_drop_count, 0);
}

#[apply(matrix)]
fn alert_bit_mirrors_fault_flags(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

#[apply(matrix)]
fn id_change_acks_old_then_answers_new(baud_idx: u8) {
    let mut sim = sim(baud_idx);
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

    // A ping still at the old 1M baud garbles at the now-3M servo: silence.
    sim.host_send_at(600, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert!(
        servo_frames(&frames).is_empty(),
        "old-baud ping must garble after the switch: {frames:#?}"
    );

    // Host follows the servo to 3M; a ping at the new baud is answered, proving
    // the switch took effect after the ack drained.
    sim.set_host_baud(BaudRate::B3000000);
    sim.host_send_at(1200, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
}
