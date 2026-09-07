//! TEL burst semantics (protocol sec 5.3): a committed nonzero `tel_count`
//! arms a bounded burst of `Stream` status frames the servo initiates on the
//! main bus; any host break aborts it. The sim's fast-tick pump synthesizes
//! samples from `tel_sample` on the kernel's 50 us grid, so payload bytes pin
//! against the same function.
//!
//! Fixed 3 M wire, not the baud matrix: at 3M (and 2M) a full frame streams
//! faster than its 16-tick batch fills, so every synthesized sample lands on
//! the wire. At 1M and below the producer outruns the wire and drops samples
//! by design (the fast tick never blocks), which would unpin the goldens.
//!
//! DES claim note: the wire model reserves the line monotonically to the
//! latest claim end, so a queued host frame pre-reserves the wire and cannot
//! interleave with the servo's own burst frames in one `run()` (an F8
//! collision). Mid-burst host action is therefore modelled with the
//! non-claiming inject primitives (`inject_break_at`, `inject_garble_at`),
//! exactly as the resilience suite injects noise -- the break IS the abort
//! lever, and garble IS the wire fault.

use osc_integration::sim::{
    Sim, Source, WireFrame, assert_valid, frame_crc_ok, instruction, status, tel_sample,
};
use osc_protocol::wire::{Inst, Opcode, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_DUTY, TEL_COUNT, TEL_MASK};
use osc_servo_core::tel::{FLAG_LAST, STREAM_PAYLOAD_MAX, TelSample, encode_stream};

mod support;

const ID5: u8 = 5;
const ID6: u8 = 6;
/// pos + current + duty + vdiff -- the ident ladder's mask.
const MASK: u16 = 0x1B;
/// Sim ticks per TEL fast tick (50 us at 48 ticks/us).
const TICK: u64 = 50 * 48;

fn sim3m() -> Sim {
    Sim::new(BaudRate::B3000000)
}

fn write_u16(id: u8, flags: u8, addr: u16, v: u16) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let d = v.to_le_bytes();
    instruction(id, Opcode::Write, flags, &[a[0], a[1], d[0], d[1]])
}

/// Write `tel_mask`, draining its ack (a real host waits before the next
/// frame -- an unspaced second break would abort the staged reply). Leaves
/// the caller to write `tel_count` and run for the count-ack + burst.
fn prime_mask(sim: &mut Sim) {
    sim.host_send(&write_u16(ID5, 0, TEL_MASK, MASK));
    let frames = sim.run();
    assert_eq!(
        status(&frames[frames.len() - 1]).0.result(),
        Some(ResultCode::Ok)
    );
}

/// `tel_mask` then `tel_count`, each acked in turn; the burst starts after
/// the count write's ack. Returns the count-write's ack + burst frames.
fn arm(sim: &mut Sim, count: u16) -> Vec<WireFrame> {
    prime_mask(sim);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, count));
    sim.run()
}

fn servo_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    frames
        .iter()
        .filter(|f| matches!(f.from, Source::Servo(_)))
        .collect()
}

fn stream_frames(frames: &[WireFrame]) -> Vec<&WireFrame> {
    servo_frames(frames)
        .into_iter()
        .filter(|f| status(f).0.result() == Some(ResultCode::Stream))
        .collect()
}

/// Expected payload of burst frame `seq` for a `count`-sample burst whose
/// ticks ran uninterrupted from 0.
fn expect_payload(count: u32, seq: usize) -> Vec<u8> {
    let samples: Vec<TelSample> = (0..count).map(tel_sample).collect();
    let a = seq * 16;
    let b = (a + 16).min(count as usize);
    let mut buf = [0u8; STREAM_PAYLOAD_MAX];
    let n = encode_stream(
        MASK,
        seq as u8,
        b == count as usize,
        &samples[a..b],
        &mut buf,
    );
    buf[..n].to_vec()
}

#[test_log::test]
fn burst_end_to_end() {
    let mut sim = sim3m();
    sim.add_servo(ID5);

    let frames = arm(&mut sim, 40);

    // Wire order: the count-write ack first, then exactly three Stream frames.
    let servo = servo_frames(&frames);
    assert_eq!(
        servo.len(),
        4,
        "count ack + three burst frames: {frames:#?}"
    );
    assert_valid(servo[0]);
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
    for (i, f) in servo[1..].iter().enumerate() {
        assert_valid(f);
        assert!(!f.collided);
        let (inst, payload) = status(f);
        assert!(inst.is_status());
        assert_eq!(inst.result(), Some(ResultCode::Stream));
        assert!(!inst.alert());
        // seq, LAST placement, and the 16/16/8 sample split all pin here:
        // the golden encodes tick data 0..40 through the same encoder.
        assert_eq!(payload, expect_payload(40, i), "frame {i} payload");
        assert_eq!(payload[1] & FLAG_LAST != 0, i == 2, "LAST only on frame 2");
    }

    // After LAST the line frees: across a 5 ms horizon the only new traffic
    // is a late ping and its one reply.
    let at = sim.now_us() + 5_000;
    sim.host_send_at(at, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(
        servo.len(),
        1,
        "bus must stay quiet after LAST: {frames:#?}"
    );
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
}

#[test_log::test]
fn burst_hold_commit() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);

    // Stage mask, count, and a goal under HOLD: each acks, nothing applies,
    // no burst starts. Drained per write (a real host waits for each ack).
    for w in [
        write_u16(ID5, Inst::FLAG_HOLD, TEL_MASK, MASK),
        write_u16(ID5, Inst::FLAG_HOLD, TEL_COUNT, 24),
        write_u16(ID5, Inst::FLAG_HOLD, GOAL_DUTY, 0x0100),
    ] {
        sim.host_send(&w);
        let frames = sim.run();
        assert_eq!(
            status(&frames[frames.len() - 1]).0.result(),
            Some(ResultCode::Ok)
        );
        assert!(
            stream_frames(&frames).is_empty(),
            "held arm must not stream"
        );
    }
    let (mask, duty) = sim.servo_table(s, |t| {
        (t.control.lifecycle.tel_mask, t.control.lifecycle.goal_duty)
    });
    assert_eq!((mask, duty), (0, 0), "held writes stay staged");

    // One broadcast COMMIT applies all three in the same instant, silently.
    sim.host_send(&instruction(0xFE, Opcode::Commit, 0, &[]));
    let frames = sim.run();
    let commit_end = frames
        .iter()
        .find(|f| matches!(f.from, Source::Host))
        .expect("commit frame recorded")
        .end;
    let servo = servo_frames(&frames);
    let burst = stream_frames(&frames);
    assert_eq!(
        servo.len(),
        burst.len(),
        "broadcast COMMIT stays silent: {frames:#?}"
    );
    assert_eq!(burst.len(), 2, "24 samples = 16 + 8");
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(status(f).1, expect_payload(24, i), "frame {i} payload");
    }
    let (mask, duty) = sim.servo_table(s, |t| {
        (t.control.lifecycle.tel_mask, t.control.lifecycle.goal_duty)
    });
    assert_eq!((mask, duty), (MASK, 0x0100), "COMMIT applied the batch");

    // Capture began within a tick or two of the COMMIT: the first frame's
    // break lands one 16-tick batch (plus the grid-alignment tick and end
    // detection) after the commit frame drained.
    let f0 = burst[0];
    assert!(
        f0.at >= commit_end + 15 * TICK && f0.at <= commit_end + 18 * TICK,
        "first burst frame at {} vs commit end {}",
        f0.at,
        commit_end
    );
}

#[test_log::test]
fn burst_abort_on_host_break() {
    // Probe run: an unmolested 48-sample burst, to time the gap between
    // frames 0 and 1 (the sim is deterministic).
    let (frame0_end, frame1_at) = {
        let mut sim = sim3m();
        sim.add_servo(ID5);
        let frames = arm(&mut sim, 48);
        let burst = stream_frames(&frames);
        assert_eq!(burst.len(), 3);
        (burst[0].end, burst[1].at)
    };
    // The host reclaims the line between frame 0 and frame 1. A bare break
    // is the abort lever (a real host frame here can't be modelled: the DES
    // claims the wire monotonically, so a queued host frame would pre-reserve
    // the line and F8-collide with the burst's own frames -- see the module
    // note). The ping after proves the servo answers on the reclaimed line.
    let break_at_us = (frame0_end + frame1_at) / 2 / 48;

    let mut sim = sim3m();
    sim.add_servo(ID5);
    prime_mask(&mut sim);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 48));
    sim.inject_break_at(break_at_us);
    let frames = sim.run();

    // The break kills the burst: frame 0 made it out, nothing after.
    let burst = stream_frames(&frames);
    assert_eq!(
        burst.len(),
        1,
        "burst must die at the host break: {frames:#?}"
    );
    assert_eq!(status(burst[0]).1[0], 0);
    assert!(
        !stream_frames(&frames)
            .iter()
            .any(|f| f.at > break_at_us * 48),
        "no burst frame after the host break"
    );

    // The servo answers a ping on the reclaimed line.
    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1, "ping answered once: {frames:#?}");
    assert_valid(servo[0]);
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));

    // A re-arm streams a full fresh burst from seq 0 and tick 0.
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 40));
    let frames = sim.run();
    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(status(f).1, expect_payload(40, i), "re-armed frame {i}");
    }
}

#[test_log::test]
fn burst_frame_corruption_is_detectable() {
    // Probe run for frame 1's wire span (see burst_abort_on_host_break).
    let garble_at_us = {
        let mut sim = sim3m();
        sim.add_servo(ID5);
        let frames = arm(&mut sim, 48);
        let f1 = stream_frames(&frames)[1];
        (f1.at + f1.end) / 2 / 48
    };

    let mut sim = sim3m();
    sim.add_servo(ID5);
    prime_mask(&mut sim);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 48));
    sim.inject_garble_at(garble_at_us, 0xA5);
    let frames = sim.run();

    // The servo streams on oblivious; only the garbled frame fails CRC.
    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    assert!(frame_crc_ok(burst[0]));
    assert!(!frame_crc_ok(burst[1]), "garbled frame must fail CRC");
    assert!(frame_crc_ok(burst[2]));
    // The collector drops the bad frame; the seq numbering exposes the gap.
    let kept: Vec<u8> = burst
        .iter()
        .filter(|f| frame_crc_ok(f))
        .map(|f| status(f).1[0])
        .collect();
    assert_eq!(kept, [0, 2]);
    assert_eq!(status(burst[0]).1, expect_payload(48, 0));
    assert_eq!(status(burst[2]).1, expect_payload(48, 2));
}

#[test_log::test]
fn burst_alert_reflects_fault() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    // Faulted ticks span exactly the second batch window.
    sim.set_tel_fault_ticks(s, 16, 32);
    let frames = arm(&mut sim, 48);

    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    let alerts: Vec<bool> = burst.iter().map(|f| status(f).0.alert()).collect();
    assert_eq!(
        alerts,
        [false, true, false],
        "ALERT on the faulted batch only"
    );
    // Fault travels in ALERT alone (the fault contract): payloads stay the
    // synthesized data.
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(status(f).1, expect_payload(48, i), "frame {i} payload");
    }
}

#[test_log::test]
fn multi_servo_silence() {
    let mut sim = sim3m();
    sim.add_servo(ID5);
    sim.add_servo(ID6);

    // Completing at all is the drive-discipline assertion: the sim panics
    // (F8) if B ever transmits into A's burst.
    let frames = arm(&mut sim, 48);
    assert_eq!(stream_frames(&frames).len(), 3);
    assert!(
        !frames.iter().any(|f| f.from == Source::Servo(ID6)),
        "servo B must stay silent through A's burst: {frames:#?}"
    );

    // B is still live: a ping after the burst answers normally.
    sim.host_send(&instruction(ID6, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1);
    assert_eq!(servo[0].from, Source::Servo(ID6));
    assert_valid(servo[0]);
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
}
