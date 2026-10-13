//! TEL burst semantics (protocol sec 5.6): a committed nonzero `tel_count`
//! arms a bounded burst of `Stream` status frames the servo initiates on the
//! main bus, the kernel owning the TX lane from the arm's ack to the LAST
//! frame; host traffic in that window is ignored. A row is the table's
//! chunks copied each tick; the sim's fast-tick pump plays `tel_sample`
//! into the table on the kernel's 50 us grid, so payload bytes pin against
//! the same function.
//!
//! TEL streams at 3M only (`tel::BAUD`): a frame of 16 rows of the longest
//! row clears the wire inside its 16-tick batch.
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
use osc_protocol::tel::StreamFrame;
use osc_protocol::wire::{Inst, Opcode, RESCUE_PULSE_MIN_US, ResultCode};
use osc_servo_core::BaudRate;
use osc_servo_core::TelemetryTick;
use osc_servo_core::regions::control::addr::lifecycle::GOAL_DUTY;
use osc_servo_core::regions::control::addr::tel::{CHUNK0_ADDR, TEL_COUNT};
use osc_servo_core::regions::telemetry::addr::{estimates, tick};
use osc_servo_core::tel::{Chunk, Chunks, FLAG_LAST, STREAM_HDR, tick_flag};

mod support;

const ID5: u8 = 5;
const ID6: u8 = 6;
/// The crest scan's shunt, taps and pos: one 8 B chunk.
const ROW: Chunks = Chunks::one(tick::CURRENT_RAW, 8);
/// The start rule's lead for `ROW`'s 8 B rows at 3M: a full frame breaks
/// after this row has landed (`osc_servo_drivers::tel`), and arms a tick
/// later.
const LEAD: u64 = 12;
/// Sim ticks per TEL fast tick (50 us at 48 ticks/us).
const TICK: u64 = 50 * 48;

fn sim3m() -> Sim {
    Sim::new(BaudRate::B3000000)
}

fn write_u16(id: u8, flags: u8, addr: u16, v: u16) -> Vec<u8> {
    write_at(id, flags, addr, &v.to_le_bytes())
}

fn write_at(id: u8, flags: u8, addr: u16, data: &[u8]) -> Vec<u8> {
    let mut p = addr.to_le_bytes().to_vec();
    p.extend_from_slice(data);
    instruction(id, Opcode::Write, flags, &p)
}

/// Write the chunks, draining the ack (a real host waits before the next
/// frame -- an unspaced second break would abort the staged reply). Leaves
/// the caller to write `tel_count` and run for the count-ack + burst.
fn prime_chunks(sim: &mut Sim, c: Chunks) {
    sim.host_send(&write_at(ID5, 0, CHUNK0_ADDR, &c.to_le_bytes()));
    let frames = sim.run();
    assert_eq!(
        status(&frames[frames.len() - 1]).0.result(),
        Some(ResultCode::Ok)
    );
}

/// The chunks then `tel_count`, each acked in turn; the burst starts after
/// the count write's ack. Returns the count-write's ack + burst frames.
fn arm(sim: &mut Sim, count: u16) -> Vec<WireFrame> {
    prime_chunks(sim, ROW);
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
    expect_tel_payload(ROW, count, seq)
}

/// A READ of `tel_count` after the burst: what the register holds.
fn read_tel_count(sim: &mut Sim) -> u16 {
    let a = TEL_COUNT.to_le_bytes();
    let at = sim.now_us() + 1_000;
    sim.host_send_at(at, &instruction(ID5, Opcode::Read, 0, &[a[0], a[1], 2, 0]));
    let frames = sim.run();
    let (inst, payload) = status(servo_frames(&frames)[0]);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    u16::from_le_bytes([payload[0], payload[1]])
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

/// A row of two chunks streams whole: each row carries its tick's table
/// bytes, the rail as last published. A row over 12 B is no row: the arm
/// acks and parks, and `tel_count` reads 0.
#[test_log::test]
fn tel_two_chunk_row_streams_whole() {
    for c in [
        Chunks::two(
            Chunk::new(tick::I_TICK_COUNTS, 6),
            Chunk::new(tick::TICK_FLAGS, 2),
        ),
        Chunks::two(
            Chunk::new(tick::I_TICK_COUNTS, 8),
            Chunk::new(estimates::VBUS_COUNTS, 2),
        ),
    ] {
        let mut sim = sim3m();
        sim.add_servo(ID5);
        prime_chunks(&mut sim, c);
        sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 40));
        let frames = sim.run();
        let stream = stream_frames(&frames);
        assert_eq!(stream.len(), 3, "{c:?}: {frames:#?}");
        for (i, f) in stream.iter().enumerate() {
            assert_valid(f);
            assert_eq!(status(f).1, expect_tel_payload(c, 40, i), "{c:?} frame {i}");
        }
    }

    let wide = Chunks::two(
        Chunk::new(tick::DUTY_TICKS, 12),
        Chunk::new(estimates::THETA_HAT_Q16, 8),
    );
    let mut sim = sim3m();
    sim.add_servo(ID5);
    prime_chunks(&mut sim, wide);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 40));
    let frames = sim.run();
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1, "the ack alone: {frames:#?}");
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
    assert_eq!(read_tel_count(&mut sim), 0);
}

/// TEL streams at 3M only: an arm at 2M acks and never streams, and
/// `tel_count` reads 0 behind it.
#[test_log::test]
fn tel_arm_below_three_megabaud_is_refused() {
    let mut sim = Sim::new(BaudRate::B2000000);
    sim.add_servo(ID5);
    let frames = arm(&mut sim, 40);
    let servo = servo_frames(&frames);
    assert_eq!(servo.len(), 1, "the ack alone: {frames:#?}");
    assert_eq!(status(servo[0]).0.result(), Some(ResultCode::Ok));
    assert_eq!(read_tel_count(&mut sim), 0);
}

/// `tel_count` reads nonzero only while a burst is armed or streams: the
/// servo zeroes it once the LAST frame is out, so a host read after the
/// burst, and a shunt-burst arm, see it finished.
#[test_log::test]
fn tel_count_reads_zero_after_last() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    let frames = arm(&mut sim, 40);
    assert_eq!(stream_frames(&frames).len(), 3);
    assert_eq!(
        sim.servo_table(s, |t| t.control.tel.tel_count),
        40,
        "LAST is out, the lane not yet back"
    );
    assert_eq!(read_tel_count(&mut sim), 0);
}

#[test_log::test]
fn burst_hold_commit() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);

    // Stage the arm (chunks and count in one write) and a goal under
    // HOLD: each acks, nothing applies, no burst starts. Drained per write
    // (a real host waits for each ack).
    let mut arm = ROW.to_le_bytes().to_vec();
    arm.extend_from_slice(&24u16.to_le_bytes());
    for w in [
        write_at(ID5, Inst::FLAG_HOLD, CHUNK0_ADDR, &arm),
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
    let (row, duty) = sim.servo_table(s, |t| {
        (t.control.tel.chunk0_len, t.control.lifecycle.goal_duty)
    });
    assert_eq!((row, duty), (0, 0), "held writes stay staged");

    // One broadcast COMMIT applies both in the same instant, silently.
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
    let (row, duty) = sim.servo_table(s, |t| {
        (t.control.tel.chunk0_len, t.control.lifecycle.goal_duty)
    });
    assert_eq!((row, duty), (8, 0x0100), "COMMIT applied the batch");

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
    prime_chunks(&mut sim, ROW);
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
    prime_chunks(&mut sim, ROW);
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
/// row's fault, which the host reads beside the row's own tick flags.
#[test_log::test]
fn burst_alert_is_the_first_rows_fault() {
    let mut sim = sim3m();
    let s = sim.add_servo(ID5);
    // Faulted ticks: from inside the second batch into the third's row 0.
    let faulted = 20..36;
    sim.set_tel_fault_ticks(s, faulted.start, faulted.end);
    let flagged = Chunks::two(
        Chunk::new(tick::I_TICK_COUNTS, 8),
        Chunk::new(tick::TICK_FLAGS, 2),
    );
    prime_chunks(&mut sim, flagged);
    sim.host_send(&write_u16(ID5, 0, TEL_COUNT, 48));
    let frames = sim.run();

    let burst = stream_frames(&frames);
    assert_eq!(burst.len(), 3);
    let alerts: Vec<bool> = burst.iter().map(|f| status(f).0.alert()).collect();
    assert_eq!(alerts, [false, false, true], "ALERT where row 0 faulted");
    let rows: Vec<TelSample> = (0..48)
        .map(|i| {
            let mut s = tel_sample(i);
            if faulted.contains(&i) {
                s.tick.tick_flags |= tick_flag::FAULT;
            }
            s
        })
        .collect();
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(
            status(f).1,
            expect_tel_payload_rows(flagged, &rows, i),
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

    prime_chunks(&mut sim, ROW);
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
            tick: TelemetryTick {
                i_tick_counts: -(i as i16) * 3,
                vdiff_counts: -(i as i16) - 40,
                duty_ticks: 50 * i as i16 - 600,
                pos: 1000 + i * 7,
                current_trough: 90 + i,
                current_raw: 0x200 + i,
                vmotor_a: 800 + i,
                vmotor_b: 600 - i,
                vbus_raw: 2000 + i,
                ntc_raw: 1500 + i,
                tick_flags: if i.is_multiple_of(3) {
                    0
                } else {
                    tick_flag::I_VALID
                },
                ..Default::default()
            },
            vbus_counts: 3000 + i,
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
    // first grid point after the count write's ack lent the lane.
    let at = STREAM_HDR + ROW.offset(tick::POS, 2).expect("pos rides the row");
    let p = status(burst[0]).1;
    let first = u16::from_le_bytes([p[at], p[at + 1]]);
    let k = rows
        .iter()
        .position(|r| r.tick.pos == first)
        .expect("first sample is a track row");
    assert_eq!(k, (servo[0].end / TICK + 1) as usize % rows.len());
    // 40 samples over a 24-row track wrap once; the payloads pin every row,
    // in order, stamped by the pump's tick.
    let served: Vec<TelSample> = (0..40).map(|i| rows[(k + i) % rows.len()]).collect();
    for (i, f) in burst.iter().enumerate() {
        assert_valid(f);
        assert_eq!(
            status(f).1,
            expect_tel_payload_rows(ROW, &served, i),
            "frame {i} payload"
        );
    }
    let f = StreamFrame::parse(ROW.row_bytes(), status(burst[1]).1).expect("a frame");
    assert_eq!((f.seq, f.tick, f.len()), (1, 16, 16));

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

    // Row 0 lands at set_track: the tick block (the stamp kept) and the
    // rail estimate.
    let r = rows[0];
    sim.servo_table(s, |t| {
        let k = t.telemetry.tick;
        assert_eq!(
            k,
            TelemetryTick {
                sample_tick: k.sample_tick,
                ..r.tick
            }
        );
        assert_eq!(t.telemetry.estimates.vbus_counts, r.vbus_counts);
    });

    // The cursor is sim time at the fast-tick rate, wrapping past the track;
    // after every exchange the table holds the cursor's row.
    for k in 1..=3 {
        sim.host_send_at(k * 1234, &instruction(ID5, Opcode::Ping, 0, &[]));
        sim.run();
        let row = sim.track_row(s).expect("track set");
        assert_eq!(row, (sim.now_us() / 50) as usize % rows.len());
        assert_eq!(
            sim.servo_table(s, |t| t.telemetry.tick.pos),
            rows[row].tick.pos,
            "exchange {k}"
        );
    }

    // A wire read of `pos` images the row current while the reply streamed.
    let a = tick::POS.to_le_bytes();
    sim.host_send(&instruction(ID5, Opcode::Read, 0, &[a[0], a[1], 2, 0]));
    let frames = sim.run();
    let reply = servo_frames(&frames)[0];
    let (inst, payload) = status(reply);
    assert_eq!(inst.result(), Some(ResultCode::Ok));
    let got = u16::from_le_bytes([payload[0], payload[1]]);
    let on_wire: Vec<u16> = (reply.at / TICK..=reply.end / TICK)
        .map(|r| rows[r as usize % rows.len()].tick.pos)
        .collect();
    assert!(on_wire.contains(&got), "read {got} not in {on_wire:?}");
}
