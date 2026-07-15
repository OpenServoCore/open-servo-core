//! Instruction-set semantics (`docs/osc-native-protocol.md` sec 5): each case
//! builds a sealed host instruction, runs the sim, and asserts on the decoded
//! status frame + table state. Reads as the spec -- plain assertions, no
//! snapshots.

use osc_integration::sim::{
    HandlerCost, Sim, Source, WireFrame, assert_valid, instruction, status,
};
use osc_protocol::wire::{Id, Inst, MgmtOp, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::config::addr::common::{BAUD_RATE_IDX, ID};
use osc_servo_core::regions::control::addr::lifecycle::TORQUE_ENABLE;
use osc_servo_core::regions::profile::span_word;
use osc_servo_core::regions::{CALIB_BASE_ADDR, PROFILE_BASE_ADDR};
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
            t.config.common.model_number,
            t.config.common.firmware_version,
        )
    });
    let m = model.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], fw]);
}

#[apply(matrix)]
fn read_returns_table_bytes(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // READ config identity span [0, 4): model(2) + fw(1) + hardware_revision(1).
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let (model, fw, hw) = sim.servo_table(s, |t| {
        (
            t.config.common.model_number,
            t.config.common.firmware_version,
            t.config.common.hardware_revision,
        )
    });
    let m = model.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], fw, hw]);
}

#[apply(matrix)]
fn read_front_loaded_reply_matches_and_leaves_table(baud_idx: u8) {
    // A READ is dispatched at covered-complete and CRC-verified at the frame end
    // (the front-loaded path): the reply is byte-identical and the read-only op
    // never touches the table.
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);
    let before = sim.servo_table(s, |t| t.config.common.model_number);

    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[0, 0, 4, 0]));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let m = before.to_le_bytes();
    // Identity span [0,4): model(2) + the sim's seeded fw(1) + hardware_revision(1),
    // both 1 from `seed_identity`.
    assert_eq!(payload, &[m[0], m[1], 0x01, 0x01]);
    assert_eq!(sim.servo_diag(s).crc_fail_count, 0);
    assert_eq!(
        sim.servo_table(s, |t| t.config.common.model_number),
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

    // count 253 exceeds the 252 B frame ceiling (sec 5.1).
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
    // LEN is the only size limit (sec 5.1): a >128 B write stages like any other
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
fn mgmt_save_without_store_is_hardware_error(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // SAVE (sec 9.4) on a servo with no persistence store seeded -> `hardware`.
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &[MgmtOp::Save as u8]));
    let frames = sim.run();

    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Hardware));
}

/// Broadcast MGMT ENUM args: `prefix_len` bits + the prefix bytes (sec 9.2).
fn enum_args(prefix_len: u8, prefix: &[u8]) -> Vec<u8> {
    let mut args = vec![MgmtOp::Enum as u8, prefix_len];
    args.extend_from_slice(prefix);
    args
}

/// Broadcast MGMT ASSIGN args: `uid(12)` + `new_id` (sec 9.2).
fn assign_args(uid: [u8; 16], new_id: u8) -> Vec<u8> {
    let mut args = vec![MgmtOp::Assign as u8];
    args.extend_from_slice(&uid);
    args.push(new_id);
    args
}

const BROADCAST: u8 = Id::BROADCAST.as_byte();

#[apply(matrix)]
fn mgmt_enum_prefix_selects_the_matching_servo(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);
    let b = sim.add_servo(6);
    // Distinct on bit 0 (the LSB-first stream's first bit, sec 9.2).
    sim.seed_servo_uid(0, [0x00; 16]);
    sim.seed_servo_uid(b, [0x01, 0xBB, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &enum_args(1, &[0x01]),
    ));
    let frames = sim.run();

    let reply = sole_reply(&frames);
    assert_eq!(
        reply.from,
        Source::Servo(6),
        "only the prefix match answers"
    );
    let (inst, payload) = status(reply);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload, sim.servo_uid(b), "the reply is the full UID");
}

#[apply(matrix)]
fn mgmt_enum_empty_prefix_is_the_tree_root(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // prefix_len 0 matches every servo -- with one on the bus, a clean reply.
    sim.host_send(&instruction(BROADCAST, Opcode::Mgmt, 0, &enum_args(0, &[])));
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(payload, sim.servo_uid(s));
}

/// sec 9.2: a staged ENUM reply is collision-tolerant -- a peer's break landing
/// in its reply gap (exactly the transport's staged-reply-kill evidence)
/// must not silence it. Before the ENUM exemption the kill made the laggard
/// of two matchers yield, the wire carried one clean frame, and the walk
/// pruned the laggard's subtree.
#[apply(matrix)]
fn mgmt_enum_reply_survives_a_peer_break_in_the_reply_gap(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(BROADCAST, Opcode::Mgmt, 0, &enum_args(0, &[])));
    // The instruction's wire end: a 14-bit break + 7 data bytes (8-byte ring
    // image). A stray break dropped 6 us later sits inside the stage->trigger
    // window at every baud: the frozen packet-end estimate is late by under
    // a byte-time, and the trigger rides 12 us behind it.
    let hz = BaudRate::from_idx(baud_idx).unwrap().as_hz() as u64;
    let end_us = (14 + 7 * 10) * 1_000_000 / hz;
    sim.inject_break_at(end_us + 6);
    let frames = sim.run();

    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert_eq!(
        payload,
        sim.servo_uid(s),
        "the reply survived the peer break"
    );
}

/// Two matchers, one trigger lagging past the leader's break (the handler
/// cost stands in for the silicon's dispatch-latency spread): the wire must
/// carry the collision -- never one clean frame with the laggard silent
/// (sec 9.2 "garbage IS the collision signal"; the yield hid id 1 from
/// 65% of fleet walks at 1M).
#[test_log::test]
fn mgmt_enum_two_matchers_collide_never_one_clean_frame() {
    let mut sim = sim(1); // 1M
    let a = sim.add_servo(ID5);
    let b = sim.add_servo(6);
    let mut uid_a = [0u8; 16];
    uid_a[0] = 0x03;
    uid_a[1] = 0xAA;
    let mut uid_b = [0u8; 16];
    uid_b[0] = 0x03;
    uid_b[1] = 0xBB;
    sim.seed_servo_uid(a, uid_a);
    sim.seed_servo_uid(b, uid_b);
    // The laggard's deadline bodies cost 30 us: its trigger pends past the
    // leader's whole break and fires mid-frame.
    sim.set_handler_cost(
        b,
        HandlerCost {
            on_deadline_us: 30,
            ..Default::default()
        },
    );

    // Both UIDs begin 0x03 -- an 8-bit prefix both match.
    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &enum_args(8, &[0x03]),
    ));
    let frames = sim.run();

    let replies = servo_frames(&frames);
    assert!(!replies.is_empty(), "the laggard must not yield silently");
    assert!(
        replies.iter().all(|f| f.collided),
        "two matchers -> colliding energy on the wire, never a clean lone frame: {replies:#?}"
    );
}

#[apply(matrix)]
fn mgmt_enum_mismatch_draws_silence(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);
    sim.seed_servo_uid(0, [0x00; 16]);

    // An empty subtree answers with a timeout, never a nack (sec 9.2).
    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &enum_args(8, &[0xFF]),
    ));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
}

#[apply(matrix)]
fn mgmt_enum_malformed_is_silent_on_broadcast_error_unicast(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // prefix_len over 96 bits: broadcast stays silent (a nack storm is the
    // one reply a broadcast must never draw)...
    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &enum_args(129, &[]),
    ));
    let frames = sim.run();
    assert!(servo_frames(&frames).is_empty());

    // ...while the same malformed query unicast gets the sec 5.3 layer-2 verdict.
    sim.host_send(&instruction(ID5, Opcode::Mgmt, 0, &enum_args(129, &[])));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Instruction));
}

#[apply(matrix)]
fn mgmt_assign_takes_the_id_and_acks_from_it(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &assign_args([ID5; 16], 42),
    ));
    let frames = sim.run();

    let reply = sole_reply(&frames);
    assert_eq!(reply.bytes[1], 42, "the ack already leaves from the new id");
    let (inst, payload) = status(reply);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    assert!(payload.is_empty());
    assert_eq!(
        sim.servo_table(s, |t| t.config.common.id),
        42,
        "mirrored into the config ID register for a later SAVE"
    );

    // The new id answers; the old id is nobody.
    sim.host_send(&instruction(42, Opcode::Ping, 0, &[]));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    assert!(servo_frames(&sim.run()).is_empty());
}

#[apply(matrix)]
fn mgmt_assign_foreign_uid_is_silent_and_inert(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &assign_args([0xEE; 16], 42),
    ));
    let frames = sim.run();

    assert!(servo_frames(&frames).is_empty());
    assert_eq!(sim.servo_table(s, |t| t.config.common.id), ID5);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let (inst, _) = status(sole_reply(&sim.run()));
    assert_eq!(inst.result(), Some(ResultCode::Ok), "old id still answers");
}

#[apply(matrix)]
fn mgmt_assign_invalid_id_nacks_validation(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // The sole UID match may nack without colliding (sec 9.2); 250 is past the
    // unicast ceiling (sec 3.1).
    sim.host_send(&instruction(
        BROADCAST,
        Opcode::Mgmt,
        0,
        &assign_args([ID5; 16], 250),
    ));
    let frames = sim.run();

    let reply = sole_reply(&frames);
    assert_eq!(reply.bytes[1], ID5, "the nack leaves from the unchanged id");
    let (inst, _) = status(reply);
    assert_eq!(inst.result(), Some(ResultCode::Validation));
    assert_eq!(sim.servo_table(s, |t| t.config.common.id), ID5);
}

#[apply(matrix)]
fn profile_read_gathers_configured_spans(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo(ID5);

    // Configure slot 0 over the wire (sec 5.2: ordinary WRITEs, no new
    // instruction): model+fw (3 B at 0x000, odd length -- no parity
    // constraint) then the comms id byte.
    let words = [span_word(0, 3), span_word(ID, 1)];
    let mut payload = PROFILE_BASE_ADDR.to_le_bytes().to_vec();
    for w in words {
        payload.extend_from_slice(&w.to_le_bytes());
    }
    sim.host_send(&instruction(ID5, Opcode::Write, 0, &payload));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));

    // READ + PROFILE names the slot; the reply is the spans' concatenation.
    sim.host_send(&instruction(ID5, Opcode::Read, Inst::FLAG_PROFILE, &[0]));
    let frames = sim.run();
    let (inst, payload) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let (model, fw) = sim.servo_table(s, |t| {
        (
            t.config.common.model_number,
            t.config.common.firmware_version,
        )
    });
    let m = model.to_le_bytes();
    assert_eq!(payload, &[m[0], m[1], fw, ID5]);
}

#[apply(matrix)]
fn profile_read_unconfigured_slot_is_range(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // Slot 1 was never configured (all words disabled) -> `range` (sec 5.3).
    sim.host_send(&instruction(ID5, Opcode::Read, Inst::FLAG_PROFILE, &[1]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Range));

    // A slot index past the region is the same error.
    sim.host_send(&instruction(ID5, Opcode::Read, Inst::FLAG_PROFILE, &[9]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert_eq!(inst.result(), Some(ResultCode::Range));
}

#[apply(matrix)]
fn profile_read_malformed_payload_is_instruction_error(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // PROFILE payload is exactly one slot byte; an addr+count shape rejects.
    sim.host_send(&instruction(
        ID5,
        Opcode::Read,
        Inst::FLAG_PROFILE,
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
    frame[4] ^= 0xFF; // corrupt a payload byte after sealing -> CRC no longer matches

    sim.host_send(&frame);
    let frames = sim.run();

    // sec 5.3 layer 1: a corrupt frame is dropped silently and counted.
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

    sim.servo_table_mut(s, |t| t.telemetry.common.fault_flags = 1);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert!(
        inst.alert(),
        "ALERT set while fault_flags nonzero (sec 5.3)"
    );

    sim.servo_table_mut(s, |t| t.telemetry.common.fault_flags = 0);
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let (inst, _) = status(sole_reply(&frames));
    assert!(!inst.alert());
}

#[apply(matrix)]
fn id_change_acks_old_then_answers_new(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    sim.add_servo(ID5);

    // WRITE the id field -> 7; the ack leaves from the OLD id, then the id lands.
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

    // WRITE baud_rate_idx -> 3M. The ack leaves at the OLD 1M baud: the deferred
    // apply lands only after the reply fully drains (sec 4.2).
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
