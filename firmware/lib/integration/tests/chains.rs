//! Coordinated group reads/writes over the osc-native bus (sec 5, 6). Every case
//! drives the REAL `ServoBus` + `osc_servo_core` dispatch through the discrete-event
//! `Sim` and asserts on the decoded shape of the recorded wire frames. Group
//! payloads are hand-built per sec 5's tables and cross-checked against the
//! `osc_protocol::group` parsers (the layout authority).

use osc_integration::sim::{
    READ32_3M_COST, Sim, Source, WireFrame, assert_valid, instruction, status, status_frame,
};
use osc_protocol::wire::{self, Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::config::addr::common::{FIRMWARE_VERSION, MODEL_NUMBER};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use osc_servo_core::regions::profile::span_word;
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::{byte_ticks, matrix, sim};

/// A 60 us RESPONSE_DEADLINE works at every baud: sec 6 keys reclaim
/// off the predecessor's *break* (its trigger -> break lead), and an observed
/// break suspends the window while the frame plays out. The baud sweep is the
/// regression for that -- at 1 M a short reply spends ~84 us on the wire, so
/// frame-end-keyed reclaim (the original defect) would falsely reclaim every
/// present-but-slow predecessor.
const CHAIN_DEADLINE_US: u16 = 60;

/// Broadcast frame ID for group ops (sec 5: the id-list, not the frame ID, selects
/// responders).
const BCAST: u8 = 0xFE;

// --- group payload builders (sec 5 tables, verified against osc_protocol::group) -

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

// --- reads (sec 6 status chains) -----------------------------------------------

#[apply(matrix)]
fn gread_uniform_chains_in_list_order(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    // Distinct model per servo so each reply's data is traceable to its owner.
    let models = [0xAA01u16, 0xBB02, 0xCC03];
    for (i, m) in models.iter().enumerate() {
        sim.servo_table_mut(i, |t| t.config.common.model_number = *m);
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
    // Every inter-reply gap respects reply gap (sec 6/7).
    let reply_gap = support::reply_gap_ticks();
    for w in reps.windows(2) {
        let gap = w[1].at - w[0].end;
        assert!(gap >= reply_gap, "chain gap {gap} < reply gap {reply_gap}");
    }
}

/// A slot waiting on its predecessor keeps the predecessor status's frame
/// end (sec 6): it answers a reply gap after that end, within a few
/// byte-times - never a starve horizon late. Bystander statuses schedule
/// nothing; a waiting slot's never loses its end.
#[apply(matrix)]
fn snooped_slot_answers_a_reply_gap_after_its_predecessor(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
    }
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(MODEL_NUMBER, 2, &[1, 2, 3]),
    ));
    let frames = sim.run();
    let reps = replies(&frames);
    assert_eq!(reps.len(), 3, "{frames:#?}");
    let reply_gap = support::reply_gap_ticks();
    for w in reps.windows(2) {
        let gap = w[1].at - w[0].end;
        assert!(gap >= reply_gap, "chain gap {gap} < reply gap {reply_gap}");
        assert!(
            gap <= reply_gap + 4 * byte_ticks(baud_idx),
            "chain gap {gap} late past reply gap {reply_gap}"
        );
    }
}

/// A uniform GREAD sizes every predecessor's status, so a waiting slot
/// drops one whose LEN says otherwise before it counts. Garbled short, the
/// status would end early and time the slot into its own tail; the slot
/// instead answers at its reclaim, flagged, after the status's real end.
#[apply(matrix)]
fn waiting_slot_drops_a_status_len_its_gread_rules_out(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    let s = sim.add_servo_with(2, 0, CHAIN_DEADLINE_US);
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(MODEL_NUMBER, 16, &[9, 2]),
    ));
    // Slot 0's status, forged by the host: 16 bytes, LEN garbled to 4.
    let mut pre = status_frame(9, ResultCode::Ok, &[0x5A; 16]);
    pre[2] = wire::len_for(4);
    assert!(!pre[1..].contains(&0));
    sim.host_send(&pre);
    let frames = sim.run();
    let pre_end = frames
        .iter()
        .find(|f| f.from == Source::Host && responder(f) == 9)
        .expect("the forged status")
        .end;
    let reps = replies(&frames);
    assert_eq!(reps.len(), 1, "{frames:#?}");
    assert!(reps[0].at > pre_end, "slot 1 fired into its predecessor");
    assert_eq!(decoded(reps[0]).0, ResultCode::PredecessorSilent);
    let d = sim.servo_diag(s);
    assert_eq!(d.framing_drop_count, 1);
    assert_eq!(d.crc_fail_count, 0);
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
    // Each servo's slot 0 gathers model(2) + the fw low byte; distinct values
    // per servo.
    let models = [0x1122u16, 0x3344];
    for (i, m) in models.iter().enumerate() {
        sim.servo_table_mut(i, |t| {
            t.config.common.model_number = *m;
            t.config.common.firmware_version = 0x50 + i as u16;
            t.profile.slots.words[0] = span_word(MODEL_NUMBER, 2);
            t.profile.slots.words[1] = span_word(FIRMWARE_VERSION, 1);
        });
    }

    // GREAD + PROFILE uniform: slot byte, then the id-list (sec 5.2).
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
        t.config.common.model_number = 0x1122;
        t.profile.slots.words[0] = span_word(MODEL_NUMBER, 2);
    });
    sim.servo_table_mut(1, |t| {
        t.config.common.firmware_version = 0x77;
        // Servo 2 answers from slot 3 -- per-target slot selection.
        t.profile.slots.words[3 * 8] = span_word(FIRMWARE_VERSION, 1);
    });

    // [id, slot]x (sec 5.2).
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
    sim.servo_table_mut(0, |t| t.config.common.model_number = 0x1122);
    sim.servo_table_mut(1, |t| t.config.common.firmware_version = 0x5A);
    sim.servo_table_mut(2, |t| t.config.common.model_number = 0x3344);

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
    // Servo 3: 3-byte model + fw low byte (the sim's seeded FIRMWARE_VERSION).
    let f = osc_servo_core::FIRMWARE_VERSION.to_le_bytes();
    assert_eq!(by_id(3).1, vec![0x44, 0x33, f[0]]);
}

#[apply(matrix)]
fn missing_servo_reclaims_with_flag(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    // Only 1 and 3 present; 2 is absent from the bus.
    let s1 = sim.add_servo_with(1, 0, CHAIN_DEADLINE_US);
    let s3 = sim.add_servo_with(3, 0, CHAIN_DEADLINE_US);
    sim.servo_table_mut(s1, |t| t.config.common.model_number = 0x0101);
    sim.servo_table_mut(s3, |t| t.config.common.model_number = 0x0303);

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
    sim.servo_table_mut(2, |t| t.config.common.model_number = 0x9999);

    // Per-target: servo 2's span is out of bounds -> Range error, empty payload.
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
    assert_eq!(res2, ResultCode::Range, "out-of-range span -> error status");
    assert!(data2.is_empty(), "error status has empty payload");
    // Slot 2 follows normally -- only silence reclaims (sec 6).
    let (res3, data3) = by_id(3);
    assert_eq!(res3, ResultCode::Ok);
    assert_eq!(data3, 0x9999u16.to_le_bytes());
}

// --- writes (sec 5) ------------------------------------------------------------

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
fn gwrite_per_target_applies_distinct_entries(baud_idx: u8) {
    let mut sim = sim(baud_idx);
    for id in [1u8, 2, 3] {
        sim.add_servo(id);
    }
    let gv: i32 = 0x1234_5678;
    let gv2: i32 = 0x5EED_F00D;
    let lo: u8 = 7;

    // Entry strides vary (4, 4, 1): the 1-byte entry pins per-entry length
    // parsing, landing in goal_velocity's low byte over a zeroed field.
    let payload = gwrite_per_target(&[
        (1, GOAL_VELOCITY, &gv.to_le_bytes()),
        (2, GOAL_VELOCITY, &gv2.to_le_bytes()),
        (3, GOAL_VELOCITY, &[lo]),
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
        sim.servo_table(1, |t| t.control.lifecycle.goal_velocity),
        gv2
    );
    assert_eq!(
        sim.servo_table(2, |t| t.control.lifecycle.goal_velocity),
        lo as i32
    );
}

#[apply(matrix)]
fn back_to_back_instructions_all_land(baud_idx: u8) {
    // sec 7: no inter-frame gap is required. Each unicast WRITE is acked before the
    // next is sent (a plain WRITE's ack shares the half-duplex wire, so the host
    // cannot physically overlap the next frame with the ack); `run` advances the
    // clock past each ack, so the follow-up frame lands right after with no
    // inserted idle gap.
    let mut sim = sim(baud_idx);
    sim.add_servo(1);
    let gv: i32 = 0x0AA0_1BB1;
    let gv2: i32 = 0x00C0_FFEE;

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
    assert_eq!(
        sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
        gv,
        "first write applied before the second lands"
    );

    let mut w2 = vec![addr_gv[0], addr_gv[1]];
    w2.extend_from_slice(&gv2.to_le_bytes());
    sim.host_send(&instruction(1, Opcode::Write, 0, &w2));
    let f2 = sim.run();
    let r2 = replies(&f2);
    assert_eq!(r2.len(), 1, "second write acked: {f2:#?}");
    assert_eq!(decoded(r2[0]).0, ResultCode::Ok);

    assert_eq!(
        sim.servo_table(0, |t| t.control.lifecycle.goal_velocity),
        gv2,
        "second write applied"
    );
}

/// A predecessor status of the largest legal frame, streamed in three arms
/// with a 50 us pause between each (a kernel body above the bus at 3M):
/// its break suspends the waiting slot's reclaim for the whole frame,
/// pauses included, and the slot answers after it, unflagged.
#[test]
fn chain_slot_survives_a_predecessor_stalled_inside_its_frame() {
    let mut sim = Sim::new(BaudRate::B3000000);
    sim.add_servo(2);
    sim.host_send(&instruction(
        BCAST,
        Opcode::Gread,
        0,
        &gread_uniform(0, wire::MAX_PAYLOAD as u16, &[9, 2]),
    ));
    let pre = status_frame(9, ResultCode::Ok, &[0x5A; wire::MAX_PAYLOAD as usize]);
    assert_eq!(pre.len(), osc_servo_drivers::bus::FRAME_MAX);
    assert!(!pre[1..].contains(&0));
    let arm = pre.len() / 3;
    sim.host_send_paused(&pre, &[(arm, 50), (2 * arm, 50)]);
    let frames = sim.run();
    let pre_end = frames
        .iter()
        .find(|f| f.from == Source::Host && responder(f) == 9)
        .expect("the predecessor status")
        .end;
    let reps = replies(&frames);
    assert_eq!(reps.len(), 1, "{frames:#?}");
    assert!(reps[0].at > pre_end, "slot 1 reclaimed into a live frame");
    assert_eq!(decoded(reps[0]).0, ResultCode::Ok);
}

/// A GREAD both slots dispatch at its covered checkpoint at 3M, whose
/// verdict each serves 600 us late (the deadline held behind a backlog,
/// the kernel above the bus): the wire-based window (end + reply gap +
/// RESPONSE_DEADLINE) has long expired when either is ready. Slot 1's
/// window counts from its own readiness instead: it sees slot 0's break
/// inside it and answers after slot 0's status, unflagged.
#[test]
fn reclaim_window_counts_from_the_slots_own_readiness() {
    const START_US: u64 = 1_000;
    const LAG_US: u64 = 600;
    let mut sim = Sim::new(BaudRate::B3000000);
    for id in [1u8, 2] {
        let s = sim.add_servo_with(id, 0, CHAIN_DEADLINE_US);
        sim.set_handler_cost(s, READ32_3M_COST);
    }
    sim.host_send_at(
        START_US,
        &instruction(
            BCAST,
            Opcode::Gread,
            0,
            &gread_uniform(GOAL_VELOCITY, 4, &[1, 2]),
        ),
    );
    let mut frames = Vec::new();
    let mut held = [false; 2];
    for t in START_US..START_US + LAG_US {
        frames.extend(sim.run_until(t));
        for (s, h) in held.iter_mut().enumerate() {
            if !*h && sim.dispatched(s) > 0 {
                sim.preempt_before_ring_read(s, LAG_US);
                *h = true;
            }
        }
    }
    assert_eq!(held, [true; 2]);
    frames.extend(sim.run());
    let gread_end = frames.iter().find(|f| f.from == Source::Host).unwrap().end;
    let reps = replies(&frames);
    assert_eq!(reps.len(), 2, "{frames:#?}");
    assert!(reps.iter().all(|f| !f.collided), "{frames:#?}");
    let window = support::reply_gap_ticks() + CHAIN_DEADLINE_US as u64 * 48;
    assert!(
        reps[0].at > gread_end + window,
        "slot 0 ready inside the window"
    );
    assert_eq!(responder(reps[0]), 1);
    assert_eq!(responder(reps[1]), 2);
    assert!(reps[1].at > reps[0].end, "slot 1 talked over slot 0");
    for f in reps {
        assert_eq!(decoded(f).0, ResultCode::Ok);
    }
}
