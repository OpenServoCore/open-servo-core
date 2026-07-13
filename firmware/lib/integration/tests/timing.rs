//! Sequencing-time properties of the osc-native transport (sec 4.1, 6, 7, 9.3).
//!
//! The `Sim` dispatches with a ZERO CPU-time model: dispatch and CRC are
//! instantaneous, so every tick here is pure wire/scheduling time. These are
//! therefore SEQUENCING assertions -- they pin reply gap behaviour, chain-gap
//! ordering, and drift tolerance against the ideal sim clock. Wall-clock
//! turnaround (the ~41 us projection of sec 7) is the bench's job, not this suite.

use osc_core::BaudRate;
use osc_core::regions::calib::addr::pot_lut::LUT;
use osc_core::regions::config::addr::identity::MODEL_NUMBER;
use osc_integration::sim::{Sim, Source, WireFrame, assert_valid, instruction, status};
use osc_protocol::wire::{Opcode, ResultCode};
use rstest::rstest;
use rstest_reuse::apply;

mod support;
use support::matrix;

const TICKS_PER_US: u64 = 48;

/// 1 byte-time (10 bits) in sim ticks at `rate`.
fn byte_ticks(rate: BaudRate) -> u64 {
    TICKS_PER_US * 1_000_000 / rate.as_hz() as u64 * 10
}

const BCAST: u8 = 0xFE;

fn gread_uniform(addr: u16, count: u16, ids: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&addr.to_le_bytes());
    p.extend_from_slice(&count.to_le_bytes());
    p.extend_from_slice(ids);
    p
}

fn host_frame(frames: &[WireFrame]) -> &WireFrame {
    frames
        .iter()
        .find(|f| f.from == Source::Host)
        .expect("host frame recorded")
}

fn replies(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

#[apply(matrix)]
fn reply_lead_respects_reply_gap(baud_idx: u8) {
    let rate = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::new(rate);
    sim.add_servo(1);
    let instr = instruction(1, Opcode::Ping, 0, &[]);
    sim.host_send(&instr);
    let frames = sim.run();

    let host = host_frame(&frames);
    let reps = replies(&frames);
    assert_eq!(reps.len(), 1, "ping -> one reply at {rate:?}: {frames:#?}");
    assert_valid(reps[0]);

    let bt = byte_ticks(rate);
    let reply_gap = support::reply_gap_ticks();
    let lead = reps[0].at - host.end;

    // Hard floor: reply gap, fixed us at every baud (sec 7).
    assert!(
        lead >= reply_gap,
        "{rate:?}: lead {lead} < reply gap {reply_gap}"
    );

    // Generous ceiling from the ring-cadence estimate (framer.rs): the
    // frozen packet-end runs late by at most the wake epsilon (1/2 bt) plus
    // the drift pad (span >> 6); two byte-times covers both with margin.
    let fp = instr.len() as u64;
    let ceil = reply_gap + 2 * bt + ((fp * bt) >> 6);
    assert!(lead <= ceil, "{rate:?}: lead {lead} > ceiling {ceil}");
}

#[apply(matrix)]
fn chain_gaps_scale_with_baud(baud_idx: u8) {
    let rate = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::new(rate);
    for id in [1u8, 2, 3] {
        sim.add_servo(id);
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

    let bt = byte_ticks(rate);
    let reply_gap = support::reply_gap_ticks();
    for w in reps.windows(2) {
        let gap = w[1].at - w[0].end;
        assert!(gap >= reply_gap, "gap {gap} < reply gap {reply_gap}");
        // Snoop-driven: a slot fires reply gap after its predecessor's status end,
        // so the gap stays within a few byte-times (no reclaim in a full chain).
        assert!(gap <= reply_gap + 6 * bt, "gap {gap} unexpectedly wide");
    }
    // Whole chain completes in well under a millisecond of wire time.
    let span = reps.last().unwrap().end - host_frame(&frames).end;
    assert!(
        span < 1_000 * TICKS_PER_US,
        "chain span {span} ticks too long"
    );
}

#[apply(matrix)]
fn skewed_servo_still_answers(baud_idx: u8) {
    // +/-1 % is the worst untrimmed-HSI throw (sec 9.3). Cursor-verified wakes absorb
    // the drift; nothing drops.
    let rate = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    for skew in [-10_000i32, 10_000] {
        let mut sim = Sim::new(rate);
        sim.add_servo_with(1, skew, 60);
        sim.host_send(&instruction(1, Opcode::Ping, 0, &[]));
        let frames = sim.run();
        let reps = replies(&frames);
        assert_eq!(reps.len(), 1, "skew {skew}: ping answered: {frames:#?}");
        assert_valid(reps[0]);
        let (inst, _) = status(reps[0]);
        assert_eq!(inst.result(), Some(ResultCode::Ok));

        let diag = sim.servo_diag(0);
        assert_eq!(diag.crc_fail_count, 0, "skew {skew}: no CRC drops");
        assert_eq!(diag.framing_drop_count, 0, "skew {skew}: no framing drops");
    }
}

#[apply(matrix)]
fn skewed_servo_survives_long_frame(baud_idx: u8) {
    // The framer's deadline-B margin scales with footprint (footprint >> 6), so
    // a long frame keeps enough slack for +1 % drift (sec 4.1 regression).
    let rate = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::new(rate);
    sim.add_servo_with(1, 10_000, 60);

    // 50 i32 words = 200 B into the rule-free calibration LUT (torque off by
    // default -> the persistent section is writable).
    let words: Vec<i32> = (0..50).map(|i| 0x0100_0000 + i).collect();
    let addr = LUT.to_le_bytes();
    let mut payload = vec![addr[0], addr[1]];
    for w in &words {
        payload.extend_from_slice(&w.to_le_bytes());
    }
    sim.host_send(&instruction(1, Opcode::Write, 0, &payload));
    let frames = sim.run();
    let reps = replies(&frames);
    assert_eq!(reps.len(), 1, "long write acked: {frames:#?}");
    let (inst, data) = status(reps[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok), "long write applies");
    assert!(data.is_empty(), "write ack is empty");

    let stored = sim.servo_table(0, |t| t.calib.pot_lut.lut);
    assert_eq!(&stored[..50], &words[..], "all 200 B landed under drift");
    assert_eq!(sim.servo_diag(0).framing_drop_count, 0);
}

#[apply(matrix)]
fn skewed_chain_sequences_correctly(baud_idx: u8) {
    let rate = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let mut sim = Sim::new(rate);
    let skews = [-10_000i32, 0, 10_000];
    for (id, skew) in (1u8..=3).zip(skews.iter()) {
        sim.add_servo_with(id, *skew, 60);
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

    // In list order, no reclaim flags despite the +/-1 % clock spread.
    assert_eq!(
        reps.iter().map(|f| f.bytes[1]).collect::<Vec<_>>(),
        vec![1, 2, 3]
    );
    for f in &reps {
        assert_valid(f);
        let (inst, _) = status(f);
        assert_eq!(
            inst.result(),
            Some(ResultCode::Ok),
            "no predecessor-silent under drift"
        );
    }
    // Gaps measured on the ideal sim clock still honour reply gap.
    let reply_gap = support::reply_gap_ticks();
    for w in reps.windows(2) {
        let gap = w[1].at - w[0].end;
        assert!(gap >= reply_gap, "gap {gap} < reply gap {reply_gap}");
    }
    for i in 0..3 {
        assert_eq!(sim.servo_diag(i).crc_fail_count, 0);
    }
}
