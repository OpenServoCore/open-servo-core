//! Coordinated group reads/writes over the osc-native bus (§5, §6). Every case
//! drives the REAL `ServoBus` + `osc_core` dispatch through the discrete-event
//! `Sim` and asserts on the decoded shape of the recorded wire frames. Group
//! payloads are hand-built per §5's tables and cross-checked against the
//! `osc_protocol::group` parsers (the layout authority).

use osc_core::regions::config::addr::identity::{FIRMWARE_VERSION, MODEL_NUMBER};
use osc_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use osc_core::regions::control::addr::streaming::{STREAM_DECIMATION, STREAM_FIELD_MASK};
use osc_core::regions::profile::span_word;
use osc_integration::sim::{Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{byte_ticks, matrix, sim};

/// The default RESPONSE_DEADLINE (60 µs) works at every baud: §6 keys reclaim
/// off the predecessor's *break* (its trigger → break lead), and an observed
/// break suspends the window while the frame plays out. The baud sweep is the
/// regression for that — at 1 M a short reply spends ~84 µs on the wire, so
/// frame-end-keyed reclaim (the original defect) would falsely reclaim every
/// present-but-slow predecessor.
const CHAIN_DEADLINE_US: u16 = 60;

/// Broadcast frame ID for group ops (§5: the id-list, not the frame ID, selects
/// responders).
const BCAST: u8 = 0xFE;

// --- group payload builders (§5 tables, verified against osc_protocol::group) -

fn gread_uniform(addr: u16, count: u16, ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&count.to_le_bytes());
    p.extend_from_slice(ids);
    p
}

fn gread_per_target(entries: &[(u8, u16, u16)]) -> Vec<u8> {
    let mut p = Vec::new();
    for &(id, addr, count) in entries {
        p.push(id);
        p.extend_from_slice(&addr.to_le_bytes());
        p.extend_from_slice(&count.to_le_bytes());
    }
    p
}

fn gwrite_uniform(addr: u16, count: u8, entries: &[(u8, &[u8])]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.push(count);
    for &(id, data) in entries {
        assert_eq!(data.len(), count as usize, "uniform GWRITE stride");
        p.push(id);
        p.extend_from_slice(data);
    }
    p
}

fn gwrite_per_target(entries: &[(u8, u16, &[u8])]) -> Vec<u8> {
    let mut p = Vec::new();
    for &(id, addr, data) in entries {
        p.push(id);
        p.extend_from_slice(&addr.to_le_bytes());
        p.push(data.len() as u8);
        p.extend_from_slice(data);
    }
    p
}

// --- helpers ----------------------------------------------------------------

fn replies(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

fn responder(f: &WireFrame) -> u8 {
    f.bytes[1]
}

/// Decoded (result, payload) of a status reply, with well-formedness checked.
fn decoded(f: &WireFrame) -> (ResultCode, Vec<u8>) {
    assert_valid(f);
    let (inst, payload) = status(f);
    assert!(inst.is_status(), "reply is a status frame");
    (inst.result().expect("valid result code"), payload.to_vec())
}

// --- reads (§6 status chains) -----------------------------------------------

#[apply(matrix)]
fn gread_uniform_chains_in_list_order(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    // Distinct model per servo so each reply's data is traceable to its owner.
    let models = [0xAA01u16, 0xBB02, 0xCC03];
    for (i, m) in models.iter().enumerate() {
        sim.servo_table_mut(i, |t| t.config.identity.model_number = *m);
    }

    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(MODEL_NUMBER, 2, &[1, 2, 3]),
    ));
    let frames = sim.run();
    let reps = replies(&frames);

    assert_eq!(reps.len(), 3, "one status per listed servo: {frames:#?}");
    // Responder ids in list order.
    assert_eq!(
        reps.iter().map(|f| responder(f)).collect::<Vec<_>>(),
        vec![1, 2, 3]
    );
    for (f, m) in reps.iter().zip(models.iter()) {
        let (result, payload) = decoded(f);
        assert_eq!(result, ResultCode::Ok);
        assert_eq!(payload, m.to_le_bytes(), "each reply carries its own span");
    }
    // Every inter-reply gap respects reply gap (§6/§7).
    let reply_gap = support::reply_gap_ticks();
    for w in reps.windows(2) {
        let gap = w[1].at - w[0].end;
        assert!(gap >= reply_gap, "chain gap {gap} < reply gap {reply_gap}");
    }
}

#[apply(matrix)]
fn gread_list_order_beats_id_order(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(MODEL_NUMBER, 2, &[3, 1, 2]),
    ));
    let frames = sim.run();
    let order: Vec<u8> = replies(&frames).iter().map(|f| responder(f)).collect();
    assert_eq!(
        order,
        vec![3, 1, 2],
        "replies follow list order, not id order"
    );
}

#[apply(matrix)]
fn gread_profile_uniform_chains_gathered_replies(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    // Each servo's slot 0 gathers model(2) + fw(1); distinct values per servo.
    let models = [0x1122u16, 0x3344];
    for (i, m) in models.iter().enumerate() {
        sim.servo_table_mut(i, |t| {
            t.config.identity.model_number = *m;
            t.config.identity.firmware_version = 0x50 + i as u8;
            t.profile.slots.words[0] = span_word(MODEL_NUMBER, 2);
            t.profile.slots.words[1] = span_word(FIRMWARE_VERSION, 1);
        });
    }

    // GREAD + PROFILE uniform: slot byte, then the id-list (§5.2).
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        Inst::FLAG_PROFILE,
        &[0, 1, 2],
    ));
    let frames = sim.run();
    let reps = replies(&frames);

    assert_eq!(reps.len(), 2, "one status per listed servo: {frames:#?}");
    assert_eq!(
        reps.iter().map(|f| responder(f)).collect::<Vec<_>>(),
        vec![1, 2]
    );
    for (i, (f, m)) in reps.iter().zip(models.iter()).enumerate() {
        let (result, payload) = decoded(f);
        assert_eq!(result, ResultCode::Ok);
        let mb = m.to_le_bytes();
        assert_eq!(payload, &[mb[0], mb[1], 0x50 + i as u8]);
    }
}

#[apply(matrix)]
fn gread_profile_per_target_selects_distinct_slots(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.servo_table_mut(0, |t| {
        t.config.identity.model_number = 0x1122;
        t.profile.slots.words[0] = span_word(MODEL_NUMBER, 2);
    });
    sim.servo_table_mut(1, |t| {
        t.config.identity.firmware_version = 0x77;
        // Servo 2 answers from slot 3 — per-target slot selection.
        t.profile.slots.words[3 * 8] = span_word(FIRMWARE_VERSION, 1);
    });

    // [id, slot]× (§5.2).
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        Inst::FLAG_PROFILE | Inst::FLAG_PER_TARGET,
        &[1, 0, 2, 3],
    ));
    let frames = sim.run();
    let reps = replies(&frames);

    assert_eq!(reps.len(), 2, "one status per listed servo: {frames:#?}");
    let (r1, p1) = decoded(reps[0]);
    assert_eq!((r1, p1.as_slice()), (ResultCode::Ok, &[0x22, 0x11][..]));
    let (r2, p2) = decoded(reps[1]);
    assert_eq!((r2, p2.as_slice()), (ResultCode::Ok, &[0x77][..]));
}

#[apply(matrix)]
fn gread_per_target_reads_distinct_spans(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.servo_table_mut(0, |t| t.config.identity.model_number = 0x1122);
    sim.servo_table_mut(1, |t| t.config.identity.firmware_version = 0x5A);
    sim.servo_table_mut(2, |t| t.config.identity.model_number = 0x3344);

    // Each servo reads its own addr/count.
    let payload = gread_per_target(&[
        (1, MODEL_NUMBER, 2),
        (2, FIRMWARE_VERSION, 1),
        (3, MODEL_NUMBER, 3),
    ]);
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        Inst::FLAG_PER_TARGET,
        &payload,
    ));
    let frames = sim.run();
    let reps = replies(&frames);
    assert_eq!(reps.len(), 3, "{frames:#?}");

    let by_id = |id: u8| decoded(reps.iter().find(|f| responder(f) == id).unwrap());
    // Servo 1: 2-byte model.
    assert_eq!(by_id(1).1, 0x1122u16.to_le_bytes());
    // Servo 2: 1-byte firmware version.
    assert_eq!(by_id(2).1, vec![0x5A]);
    // Servo 3: 3-byte model + firmware (default fw 0x56).
    assert_eq!(by_id(3).1, vec![0x44, 0x33, 0x56]);
}

#[apply(matrix)]
fn missing_servo_reclaims_with_flag(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    // Only 1 and 3 present; 2 is absent from the bus.
    let s1 = sim.add_servo_with(1, 0, CHAIN_DEADLINE_US);
    let s3 = sim.add_servo_with(3, 0, CHAIN_DEADLINE_US);
    sim.servo_table_mut(s1, |t| t.config.identity.model_number = 0x0101);
    sim.servo_table_mut(s3, |t| t.config.identity.model_number = 0x0303);

    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(MODEL_NUMBER, 2, &[1, 2, 3]),
    ));
    let frames = sim.run();
    let reps = replies(&frames);
    assert_eq!(
        reps.len(),
        2,
        "silent servo 2 produces no frame: {frames:#?}"
    );

    let r1 = reps.iter().find(|f| responder(f) == 1).unwrap();
    let r3 = reps.iter().find(|f| responder(f) == 3).unwrap();

    // Servo 3 reclaims slot 2 after servo 2's window lapses: predecessor-silent,
    // but its own read data is still present.
    let (res3, data3) = decoded(r3);
    assert_eq!(res3, ResultCode::PredecessorSilent);
    assert_eq!(
        data3,
        0x0303u16.to_le_bytes(),
        "reclaimed slot keeps its data"
    );
    let (res1, _) = decoded(r1);
    assert_eq!(res1, ResultCode::Ok);

    // Reclaim window: servo 3 fires no earlier than servo 1's end + reply gap +
    // RESPONSE_DEADLINE, and reasonably close to it.
    let bt = byte_ticks(baud_idx);
    let reclaim = CHAIN_DEADLINE_US as u64 * 48;
    let floor = r1.end + support::reply_gap_ticks() + reclaim;
    assert!(r3.at >= floor, "reclaim too early: {} < {}", r3.at, floor);
    assert!(
        r3.at <= floor + 8 * bt,
        "reclaim not close to window: {} vs {}",
        r3.at,
        floor
    );
}

#[apply(matrix)]
fn error_status_keeps_chain_alive(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.servo_table_mut(2, |t| t.config.identity.model_number = 0x9999);

    // Per-target: servo 2's span is out of bounds → Range error, empty payload.
    let payload = gread_per_target(&[(1, MODEL_NUMBER, 2), (2, 0xFFFE, 4), (3, MODEL_NUMBER, 2)]);
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        Inst::FLAG_PER_TARGET,
        &payload,
    ));
    let frames = sim.run();
    let reps = replies(&frames);
    assert_eq!(reps.len(), 3, "error keeps the chain alive: {frames:#?}");

    let by_id = |id: u8| decoded(reps.iter().find(|f| responder(f) == id).unwrap());
    assert_eq!(by_id(1).0, ResultCode::Ok);
    let (res2, data2) = by_id(2);
    assert_eq!(res2, ResultCode::Range, "out-of-range span → error status");
    assert!(data2.is_empty(), "error status has empty payload");
    // Slot 2 follows normally — only silence reclaims (§6).
    let (res3, data3) = by_id(3);
    assert_eq!(res3, ResultCode::Ok);
    assert_eq!(data3, 0x9999u16.to_le_bytes());
}

// --- writes (§5) ------------------------------------------------------------

#[apply(matrix)]
fn gwrite_hold_commit_is_atomic_fleet_update(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo(id);
    }
    let vals: [i32; 3] = [0x0011_2233, 0x0044_5566, 0x0077_1020];

    // Uniform GWRITE + HOLD: staged, no reply, live table untouched.
    let bytes: Vec<[u8; 4]> = vals.iter().map(|v| v.to_le_bytes()).collect();
    let entries: Vec<(u8, &[u8])> = (1u8..=3)
        .zip(bytes.iter())
        .map(|(id, d)| (id, &d[..]))
        .collect();
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gwrite,
        Inst::FLAG_HOLD,
        &gwrite_uniform(GOAL_VELOCITY, 4, &entries),
    ));
    let staged = sim.run();
    assert!(replies(&staged).is_empty(), "GWRITE is silent: {staged:#?}");
    for i in 0..3 {
        assert_eq!(
            sim.servo_table(i, |t| t.control.lifecycle.goal_velocity),
            0,
            "HOLD does not touch the live table"
        );
    }

    // Broadcast COMMIT: all servos apply atomically, still silent.
    sim.host_send(&instruction(BCAST, Opcode::Commit, 0, &[]));
    let committed = sim.run();
    assert!(
        replies(&committed).is_empty(),
        "COMMIT is silent: {committed:#?}"
    );
    for (i, v) in vals.iter().enumerate() {
        assert_eq!(
            sim.servo_table(i, |t| t.control.lifecycle.goal_velocity),
            *v,
            "COMMIT applied servo {i}'s staged value"
        );
    }
}

#[apply(matrix)]
fn gwrite_per_target_applies_distinct_fields(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo(id);
    }
    let gv: i32 = 0x1234_5678;
    let mask: u32 = 0xDEAD_BEEF;
    let deci: u8 = 7;

    let payload = gwrite_per_target(&[
        (1, GOAL_VELOCITY, &gv.to_le_bytes()),
        (2, STREAM_FIELD_MASK, &mask.to_le_bytes()),
        (3, STREAM_DECIMATION, &[deci]),
    ]);
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gwrite,
        Inst::FLAG_PER_TARGET,
        &payload,
    ));
    let frames = sim.run();
    assert!(replies(&frames).is_empty(), "GWRITE is silent: {frames:#?}");

    assert_eq!(
        sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
        gv
    );
    assert_eq!(
        sim.servo_table(1, |t| t.control.streaming.stream_field_mask),
        mask
    );
    assert_eq!(
        sim.servo_table(2, |t| t.control.streaming.stream_decimation),
        deci
    );
}

#[apply(matrix)]
fn back_to_back_instructions_all_land(baud_idx: u8) {
    // §7: no inter-frame gap is required. Each unicast WRITE is acked before the
    // next is sent (a plain WRITE's ack shares the half-duplex wire, so the host
    // cannot physically overlap the next frame with the ack); `run` advances the
    // clock past each ack, so the follow-up frame lands right after with no
    // inserted idle gap.
    let mut sim = sim(baud_idx);
    sim.add_servo(1);
    let gv: i32 = 0x0AA0_1BB1;
    let mask: u32 = 0x00C0_FFEE;

    let addr_gv = GOAL_VELOCITY.to_le_bytes();
    let mut w1 = vec![addr_gv[0], addr_gv[1]];
    w1.extend_from_slice(&gv.to_le_bytes());
    sim.host_send(&instruction(1, Opcode::Write, 0, &w1));
    let f1 = sim.run();
    let r1 = replies(&f1);
    assert_eq!(r1.len(), 1, "first write acked: {f1:#?}");
    let (res1, data1) = decoded(r1[0]);
    assert_eq!(res1, ResultCode::Ok);
    assert!(data1.is_empty(), "write ack is empty");

    let addr_mask = STREAM_FIELD_MASK.to_le_bytes();
    let mut w2 = vec![addr_mask[0], addr_mask[1]];
    w2.extend_from_slice(&mask.to_le_bytes());
    sim.host_send(&instruction(1, Opcode::Write, 0, &w2));
    let f2 = sim.run();
    let r2 = replies(&f2);
    assert_eq!(r2.len(), 1, "second write acked: {f2:#?}");
    assert_eq!(decoded(r2[0]).0, ResultCode::Ok);

    // Both applied, in order.
    assert_eq!(
        sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
        gv
    );
    assert_eq!(
        sim.servo_table(0, |t| t.control.streaming.stream_field_mask),
        mask
    );
}
