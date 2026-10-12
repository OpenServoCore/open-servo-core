//! TEL burst semantics (protocol sec 5.6): a committed nonzero `tel_count`
//! arms a bounded burst of `Stream` status frames the servo initiates on the
//! main bus, the kernel owning the TX lane from the arm's ack to the LAST
//! frame; host traffic in that window is ignored. The sim's fast-tick pump
//! synthesizes samples from `tel_sample` on the kernel's 50 us grid, so
//! payload bytes pin against the same function.
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
//! non-claiming inject primitives (`inject_frame_at`, `inject_garble_at`),
//! exactly as the resilience suite injects noise.

use osc_integration::sim::{
    Sim, Source, TelSample, WireFrame, assert_valid, expect_tel_payload, expect_tel_payload_rows,
    frame_crc_ok, instruction, status, tel_sample,
};
use osc_protocol::tel::{StreamFrame, decode_row};
use osc_protocol::wire::{Inst, Opcode, RESCUE_PULSE_MIN_US, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::regions::control::addr::lifecycle::{GOAL_DUTY, TEL_COUNT, TEL_MASK};
use osc_servo_core::regions::telemetry::addr::sensors::POS;
use osc_servo_core::tel::{BIT_POS, BIT_POS_LIN, FLAG_LAST, STREAM_HDR};

mod support;

const ID5: u8 = 5;
const ID6: u8 = 6;
/// pos + current + duty + vdiff -- the ident ladder's mask.
const MASK: u16 = 0x1B;
/// The start rule's lead for `MASK`'s 7 B rows at 3M: a full frame breaks
/// after this row has landed (`osc_servo_drivers::tel`), and arms a tick
/// later.
const LEAD: u64 = 12;
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

fn expect_payload(count: u32, seq: usize) -> Vec<u8> {
    expect_tel_payload(MASK, count, seq)
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

/// `pos | pos_lin`: the linearized word rides the wire beside the raw pot,
/// in bit order, through the same encoder the golden pins.
#[test_log::test]
fn pos_lin_streams_beside_pos() {
    let mask = BIT_POS | BIT_POS_LIN;
    let mut sim = sim3m();
    sim.add_servo(ID5);
    sim.host_send(&write_u16(ID5, 0, TEL_MASK, mask));
    let frames = sim.run();
    assert_eq!(
        status(&frames[frames.len() - 1]).0.result(),
        Some(ResultCode::Ok)
    );
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 16));
    let frames = sim.run();
    let stream = stream_frames(&frames);
    assert_eq!(stream.len(), 1, "{frames:#?}");
    let (_, payload) = status(stream[0]);
    assert_eq!(payload, expect_tel_payload(mask, 16, 0));
    for (i, s) in payload[STREAM_HDR..STREAM_HDR + 16 * 4]
        .chunks(4)
        .enumerate()
    {
        let want = tel_sample(i as u32);
        assert_eq!(u16::from_le_bytes([s[0], s[1]]), want.pos, "sample {i}");
        assert_eq!(
            u16::from_le_bytes([s[2], s[3]]),
            want.pos_lin_q4,
            "sample {i}"
        );
    }
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

    // Capture began within a tick or two of the COMMIT (no ack to wait
    // for: the lane passes at the arm): the first frame's break follows its
    // lead row, plus the grid-alignment tick and end detection.
    let f0 = burst[0];
    assert!(
        f0.at >= commit_end + LEAD * TICK && f0.at <= commit_end + (LEAD + 3) * TICK,
        "first burst frame at {} vs commit end {}",
        f0.at,
        commit_end
    );
}

/// sec 5.6: the servo owns the line from the arm's ack to the LAST frame. A
/// host write between two burst frames is ignored - no reply, no effect,
/// the burst streams whole - and the first instruction after LAST is
/// served, the ignored one never.
#[test_log::test]
fn burst_ignores_a_host_frame_and_serves_the_next_after_last() {
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
    let at_us = (frame0_end + frame1_at) / 2 / 48;

    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    prime_mask(&mut sim);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 48));
    sim.inject_frame_at(at_us, &write_u16(ID5, 0, GOAL_DUTY, 0x0123));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 4, "the ack and the whole burst: {frames:#?}");
    for (i, f) in servo[1..].iter().enumerate() {
        assert_valid(f);
        assert_eq!(status(f).1, expect_payload(48, i), "frame {i}");
    }

    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(
        servo.len(),
        1,
        "one reply, to the ping after LAST: {frames:#?}"
    );
    assert_valid(servo[0]);
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
    assert_eq!(
        sim.servo_table(s, |t| t.control.lifecycle.goal_duty),
        0,
        "the mid-burst write never applies"
    );

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

/// Ticks lost mid-frame (any cause): the DMA reads the rows behind them
/// before they land, so the frame ships stale bytes against the CRC the
/// engine took over the fresh ones. It fails its CRC, the host sees a seq
/// hole, and the burst runs on to LAST: nothing wedges.
#[test_log::test]
fn tel_lost_ticks_mid_frame_fail_the_crc_and_leave_a_seq_hole() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    // Frame 1 arms on burst tick 16 + LEAD + 1; lose the six after it.
    let from = 16 + LEAD as u32 + 2;
    sim.lose_tel_ticks(s, from, from + 6);
    let frames = arm(&mut sim, 48);
    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3, "{frames:#?}");
    let ok: Vec<bool> = burst.iter().map(|f| frame_crc_ok(f)).collect();
    assert_eq!(ok, [true, false, true]);
    let kept: Vec<u8> = burst
        .iter()
        .filter(|f| frame_crc_ok(f))
        .map(|f| status(f).1[0])
        .collect();
    assert_eq!(kept, [0, 2], "the seq hole");
    assert_ne!(status(burst[2]).1[1] & FLAG_LAST, 0);

    sim.host_send(&instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1, "the line is the bus's again: {frames:#?}");
    assert_valid(servo[0]);
}

#[test_log::test]
fn burst_frame_corruption_is_detectable() {
    // Probe run for frame 1's wire span (see
    // burst_ignores_a_host_frame_and_serves_the_next_after_last).
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

/// INST ALERT is the device state when INST leaves: the frame's first
/// row's fault. The trailer's tail carries the OR of the batch's.
#[test_log::test]
fn burst_alert_is_the_first_row_fault_and_the_tail_the_batch_or() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    // Faulted ticks: from inside the second batch into the third's row 0.
    let faulted = 20..36;
    sim.set_tel_fault_ticks(s, faulted.start, faulted.end);
    let frames = arm(&mut sim, 48);

    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    let alerts: Vec<bool> = burst.iter().map(|f| status(f).0.alert()).collect();
    assert_eq!(alerts, [false, false, true], "ALERT where row 0 faulted");
    let tails: Vec<bool> = burst
        .iter()
        .map(|f| {
            StreamFrame::parse(MASK, status(f).1)
                .expect("a frame")
                .alert
        })
        .collect();
    assert_eq!(tails, [false, true, true], "tail where any row faulted");
    // The rows stay the synthesized data.
    let rows: Vec<TelSample> = (0..48)
        .map(|i| TelSample {
            fault: faulted.contains(&i),
            ..tel_sample(i)
        })
        .collect();
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(
            status(f).1,
            expect_tel_payload_rows(MASK, &rows, i),
            "frame {i} payload"
        );
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

/// sec 9.1 own TX: HDSEL keeps the servo's own bytes out of its ring, so a
/// main-loop line sample landing on a low bit of a burst frame finds the
/// ring frozen. The sim reads every sample during own TX low (the worst
/// case, samples phase-locked onto low bits): burst frames longer than the
/// rescue window still never declare, and a real pulse afterwards does.
#[test_log::test]
fn burst_never_reads_as_a_rescue_pulse() {
    let mut sim = sim3m();
    sim.add_servo(ID5);

    prime_mask(&mut sim);
    let t0 = sim.now_us();
    for k in 0..200 {
        sim.sample_line_at(t0 + k * 50);
    }
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 40));
    let frames = sim.run();
    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3, "{frames:#?}");
    // The two full frames outlast the window plus a sample period.
    for f in &burst[..2] {
        assert!(f.end - f.at > (RESCUE_PULSE_MIN_US as u64 + 50) * 48);
    }

    // Still at 3M: a ping answers at the operational rate.
    let at = sim.now_us() + 1_000;
    sim.host_send_at(at, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    assert_eq!(
        status(servo_frames(&frames)[0]).0.result(),
        Some(ResultCode::Ok)
    );

    // A real pulse still declares: the servo answers at the rescue rate.
    let at = sim.now_us() + 1_000;
    sim.hold_line_low_at(at, 400);
    sim.run();
    sim.set_host_baud(BaudRate::B500000);
    sim.host_send_at(at + 2_000, &instruction(ID5, Opcode::Ping, 0, &[]));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1, "{frames:#?}");
    assert_valid(servo[0]);
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
}

// --- recorded-track playback ---

/// A short track with a unique `pos` per row, so a burst's first sample
/// locates the cursor it started at.
fn track(len: u16) -> Vec<TelSample> {
    (0..len)
        .map(|i| TelSample {
            pos: 1000 + i * 7,
            current: -(i as i16) * 3,
            current_trough: 90 + i,
            duty: 50 * i as i16 - 600,
            vdiff: -(i as i16) - 40,
            vbus: 3000 + i,
            current_raw: 0x200 + i,
            vmotor_a: 800 + i,
            vmotor_b: 600 - i,
            vbus_raw: 2000 + i,
            ntc_raw: 1500 + i,
            pos_lin_q4: (1000 + i * 7) << 4,
            tick: 0,
            window_valid: !i.is_multiple_of(3),
            fault: false,
        })
        .collect()
}

#[test_log::test]
fn track_rows_stream_in_order_and_loop() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    let rows = track(24);
    sim.set_track(s, rows.clone());

    let frames = arm(&mut sim, 40);
    let servo = servo_frames(&frames);
    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    // The pump ticks on the 50 us grid: the first sample is the row at the
    // first grid point after the count write's ack armed the burst.
    let first = decode_row(MASK, &status(burst[0]).1[STREAM_HDR..]).expect("a row")[0];
    let k = rows
        .iter()
        .position(|r| r.pos == first)
        .expect("first sample is a track row");
    assert_eq!(k, (servo[0].at / TICK + 1) as usize % rows.len());
    // 40 samples over a 24-row track wrap once; the payloads pin every row
    // through the same encoder, in order, stamped by the pump's tick.
    let served: Vec<TelSample> = (0..40)
        .map(|i| TelSample {
            tick: i as u16,
            ..rows[(k + i) % rows.len()]
        })
        .collect();
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(
            status(f).1,
            expect_tel_payload_rows(MASK, &served, i),
            "frame {i} payload"
        );
    }

    // An empty track restores the synthesized samples, byte for byte.
    sim.set_track(s, Vec::new());
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 40));
    let frames = sim.run();
    for (i, f) in stream_frames(&frames).iter().enumerate() {
        assert_eq!(status(f).1, expect_payload(40, i), "synthesized frame {i}");
    }
}

#[test_log::test]
fn track_mirrors_into_the_live_table() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    let rows = track(24);
    sim.set_track(s, rows.clone());

    // Row 0 lands at set_track: the raw ADC frame into the sensors block,
    // the row's kernel conclusions into the estimates block.
    let r = rows[0];
    sim.servo_table(s, |t| {
        let sn = &t.telemetry.sensors;
        assert_eq!(
            (
                sn.pos,
                sn.current,
                sn.current_trough,
                sn.vmotor_a,
                sn.vmotor_b,
                sn.vbus_raw,
                sn.ntc_raw
            ),
            (
                r.pos,
                r.current_raw,
                r.current_trough,
                r.vmotor_a,
                r.vmotor_b,
                r.vbus_raw,
                r.ntc_raw
            )
        );
        let es = &t.telemetry.estimates;
        assert_eq!(
            (es.vbus_counts, es.duty_applied_q15, es.i_hat_counts),
            (r.vbus, osc_integration::plant::duty_q15(r.duty), r.current)
        );
    });

    // The cursor is sim time at the fast-tick rate, wrapping past the track;
    // after every exchange the table holds the cursor's row.
    for k in 1..=3 {
        sim.host_send_at(k * 1234, &instruction(ID5, Opcode::Ping, 0, &[]));
        sim.run();
        let row = sim.track_row(s).expect("track set");
        assert_eq!(row, (sim.now_us() / 50) as usize % rows.len());
        assert_eq!(
            sim.servo_table(s, |t| t.telemetry.sensors.pos),
            rows[row].pos,
            "exchange {k}"
        );
    }

    // A wire read of `pos` images the row current while the reply streamed.
    let a = POS.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[a[0], a[1], 2, 0]));
    let frames = sim.run();
    let reply = servo_frames(&frames)[0];
    let (inst, payload) = status(reply);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let got = u16::from_le_bytes([payload[0], payload[1]]);
    let on_wire: Vec<u16> = (reply.at / TICK..=reply.end / TICK)
        .map(|r| rows[r as usize % rows.len()].pos)
        .collect();
    assert!(on_wire.contains(&got), "read {got} not in {on_wire:?}");
}
