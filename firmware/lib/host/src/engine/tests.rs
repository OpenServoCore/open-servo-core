//! `HostBus` scenarios over recording fakes (the drivers-crate mock spirit:
//! cloneable `Rc` state companions, one clone moved into the engine).

use std::vec;
use std::vec::Vec;

use osc_protocol::build;
use osc_protocol::wire::{BaudRate, Id, Inst, Opcode, ResultCode, UID_LEN};

use super::*;
use crate::testutil::{
    FakeBaud, FakeDeadline, FakeRing, FakeWire, TestProviders, WireOp, sealed_status,
};

struct Rig {
    bus: HostBus<TestProviders>,
    ring: FakeRing,
    clock: FakeDeadline,
    wire: FakeWire,
    baud: FakeBaud,
}

fn rig() -> Rig {
    let ring = FakeRing::new();
    let clock = FakeDeadline::new();
    let wire = FakeWire::default();
    let baud = FakeBaud::default();
    let bus = HostBus::new(
        ring.clone(),
        clock.clone(),
        wire.clone(),
        baud.clone(),
        BaudRate::B1000000,
    );
    Rig {
        bus,
        ring,
        clock,
        wire,
        baud,
    }
}

fn ping(id: u8) -> (Id, Inst) {
    (Id::new(id), Inst::instruction(Opcode::Ping, 0))
}

/// Submit + drive the TX to completion (claim/break/send observed).
fn exchange(r: &mut Rig, id: Id, inst: Inst, payload: &[u8]) {
    r.bus
        .submit(Command::Exchange { id, inst, payload })
        .expect("valid exchange");
    r.bus.on_tx_complete();
}

fn expect_done(r: &mut Rig) -> Terminal {
    match r.bus.poll() {
        Some(Event::Done(t)) => t,
        other => panic!("expected Done, got {other:?}"),
    }
}

fn expect_status(r: &mut Rig) -> (u8, Id) {
    match r.bus.poll() {
        Some(Event::Status { slot, id, .. }) => (slot, id),
        other => panic!("expected Status, got {other:?}"),
    }
}

#[test]
fn ping_round_trip() {
    let mut r = rig();
    let (id, inst) = ping(1);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();

    // Claim -> law break -> one arm carrying the sealed frame sans the
    // alignment byte (the break is the wire's 0x00). The bytes are the
    // protocol sec 3.2 PING id 1 vector.
    let log = r.wire.log();
    assert_eq!(log[0], WireOp::Claim);
    assert_eq!(log[1], WireOp::Break);
    assert_eq!(log[2], WireOp::Send(vec![0x01, 0x03, 0x10, 0x50, 0xFC]));

    r.bus.on_tx_complete();
    assert_eq!(r.wire.log()[3], WireOp::Release);

    r.ring
        .frame(&sealed_status(1, ResultCode::Ok, &[0x2A, 0x00, 0x01]));
    let (slot, sid) = expect_status(&mut r);
    assert_eq!((slot, sid), (0, Id::new(1)));

    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 1);
    assert_eq!(t.evidence.garble, 0);
    assert!(r.bus.poll().is_none());

    // The engine is idle again.
    let (id, inst) = ping(5);
    assert!(
        r.bus
            .submit(Command::Exchange {
                id,
                inst,
                payload: &[]
            })
            .is_ok()
    );
}

#[test]
fn one_outstanding_command_ever() {
    let mut r = rig();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    assert_eq!(
        r.bus.submit(Command::Rescue),
        Err(SubmitError::Busy),
        "mid-exchange"
    );

    r.bus.on_tx_complete();
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
    let _ = expect_status(&mut r);
    // Terminal queued but unconsumed: still busy.
    assert_eq!(r.bus.submit(Command::Rescue), Err(SubmitError::Busy));
    let _ = expect_done(&mut r);
    assert!(r.bus.submit(Command::Rescue).is_ok());
}

#[test]
fn invalid_exchange_never_touches_the_wire() {
    let mut r = rig();
    let err = r.bus.submit(Command::Exchange {
        id: Id::new(0),
        inst: Inst::instruction(Opcode::Ping, 0),
        payload: &[],
    });
    assert_eq!(err, Err(SubmitError::Invalid(shape::InvalidReason::BadId)));
    assert!(r.wire.log().is_empty());
    // And the engine stayed idle -- a refused submit costs nothing.
    let (id, inst) = ping(1);
    assert!(
        r.bus
            .submit(Command::Exchange {
                id,
                inst,
                payload: &[]
            })
            .is_ok()
    );
}

#[test]
fn noreply_write_completes_at_wire_end() {
    let mut r = rig();
    let mut p = [0u8; 8];
    let n = build::write(&mut p, 0x0180, &[0x2C, 0x01]).unwrap();
    exchange(
        &mut r,
        Id::new(5),
        Inst::instruction(Opcode::Write, Inst::FLAG_NOREPLY),
        &p[..n],
    );
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Sent);
    assert_eq!(*r.wire.log().last().unwrap(), WireOp::Release);
}

#[test]
fn timeout_when_the_bus_stays_silent() {
    let mut r = rig();
    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);
    assert!(r.bus.poll().is_none());

    // Window: RESPONSE_DEADLINE(1000) + (10 + 16 margin) bytes x 10 us = 1260.
    r.clock.advance(1259);
    assert!(r.bus.poll().is_none());
    r.clock.advance(2);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Timeout { slot: 0 });
    assert_eq!(t.evidence.statuses, 0);
}

#[test]
fn gread_chain_streams_slots_in_order() {
    let mut r = rig();
    let mut p = [0u8; 16];
    let ids = [Id::new(1), Id::new(2), Id::new(3)];
    let n = build::gread_uniform(&mut p, 0x0084, 4, &ids).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Gread, 0),
        &p[..n],
    );

    for (k, id) in ids.iter().enumerate() {
        r.ring
            .frame(&sealed_status(id.as_byte(), ResultCode::Ok, &[0; 4]));
        let (slot, sid) = expect_status(&mut r);
        assert_eq!((slot, sid), (k as u8, *id));
    }
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 3);
}

#[test]
fn chain_timeout_names_the_missing_slot() {
    let mut r = rig();
    let mut p = [0u8; 16];
    let ids = [Id::new(1), Id::new(2), Id::new(3)];
    let n = build::gread_uniform(&mut p, 0x0084, 4, &ids).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Gread, 0),
        &p[..n],
    );

    r.ring.frame(&sealed_status(1, ResultCode::Ok, &[0; 4]));
    let _ = expect_status(&mut r);
    r.ring.frame(&sealed_status(2, ResultCode::Ok, &[0; 4]));
    let _ = expect_status(&mut r);

    r.clock.advance(10_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Timeout { slot: 2 });
    assert_eq!(t.evidence.statuses, 2);
}

#[test]
fn broadcast_enum_collects_until_quiet() {
    let mut r = rig();
    let mut p = [0u8; 24];
    let prefix = [0u8; UID_LEN];
    let n = build::mgmt_enum(&mut p, 0, &prefix).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Mgmt, 0),
        &p[..n],
    );

    r.ring
        .frame(&sealed_status(10, ResultCode::Ok, &[0xAA; UID_LEN]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 0);
    r.ring
        .frame(&sealed_status(11, ResultCode::Ok, &[0xBB; UID_LEN]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 1);

    // Quiet horizon reached: collect completes rather than timing out.
    r.clock.advance(10_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 2);
}

#[test]
fn garble_is_evidence_and_paces_the_next_exchange() {
    let mut r = rig();
    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);

    // Junk, then the clean reply: garble counted, not trailing.
    r.ring.feed(&[0xAA, 0xBB, 0xCC]);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
    let _ = expect_status(&mut r);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.garble, 3);
    assert!(!t.evidence.garble_after_last_frame);

    // The sec 3.4 pacing gap: the next exchange holds off the wire for one
    // starve horizon (64 byte-times = 640 us at 1M).
    r.wire.clear();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "paced: nothing on the wire yet");
    r.clock.advance(641);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim, "pacing gap elapsed -> TX");
}

#[test]
fn pacing_waits_for_ring_silence_and_drops_what_it_heard() {
    let mut r = rig();
    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);
    r.ring.feed(&[0xAA]);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);
    let _ = expect_done(&mut r);

    r.wire.clear();
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    // A late reply lands mid-quiet: the quiet restarts from it.
    r.clock.advance(500);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[9]));
    assert!(r.bus.poll().is_none());
    r.clock.advance(400);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "900 us in, 400 us of silence");
    r.clock.advance(241);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim, "640 us of silence -> TX");

    // The late reply was dropped at TX: only the real one is collected.
    r.bus.on_tx_complete();
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[1]));
    match r.bus.poll() {
        Some(Event::Status { payload, .. }) => assert_eq!(payload.segments().0, &[1]),
        other => panic!("expected Status, got {other:?}"),
    }
    assert_eq!(expect_done(&mut r).evidence.statuses, 1);
}

#[test]
fn cancel_mid_await_keeps_the_old_reply_window_quiet() {
    let mut r = rig();
    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);
    let window = r.bus.window_for(&Shape::derive(id, inst, &[]).unwrap());
    r.bus.cancel();
    assert!(!r.bus.busy());

    r.wire.clear();
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    // The servo may answer up to its deadline: no frame before the old
    // window ends, though the horizon alone is long past.
    r.clock.advance(window - 1);
    assert!(r.bus.poll().is_none());
    assert!(
        r.wire.log().is_empty(),
        "the old reply window is still open"
    );
    // Its late reply lands: nobody's, and the quiet re-arms by the window,
    // as the await would have.
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    assert!(r.bus.poll().is_none(), "the old reply is nobody's");
    r.clock.advance(window - 1);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty());
    r.clock.advance(2);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim);
}

#[test]
fn cancel_mid_stream_holds_the_bus_to_the_burst_window() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 50_000);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);
    r.clock.advance(1_000);
    r.bus.cancel();

    r.wire.clear();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    // Burst frames ms apart: silence between them never opens the bus.
    for _ in 0..4 {
        r.clock.advance(10_000);
        r.ring
            .frame(&sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]));
        assert!(r.bus.poll().is_none());
        assert!(r.wire.log().is_empty(), "inside the burst window");
    }
    r.clock.advance(8_999);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "the burst window has 1 us left");
    r.clock.advance(1);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim, "the burst window ended");
}

#[test]
fn cancel_mid_transmit_releases_the_wire() {
    let mut r = rig();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    r.bus.cancel();
    assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
    // The cut frame's TC changes nothing.
    r.bus.on_tx_complete();
    assert!(r.bus.poll().is_none());
    assert!(!r.bus.busy());
}

#[test]
fn cancel_when_idle_costs_no_quiet() {
    let mut r = rig();
    r.bus.cancel();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    assert_eq!(r.wire.log()[0], WireOp::Claim);
}

#[test]
fn trailing_energy_is_flagged() {
    let mut r = rig();
    let mut p = [0u8; 24];
    let prefix = [0u8; UID_LEN];
    let n = build::mgmt_enum(&mut p, 0, &prefix).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Mgmt, 0),
        &p[..n],
    );

    r.ring
        .frame(&sealed_status(10, ResultCode::Ok, &[0xAA; UID_LEN]));
    let _ = expect_status(&mut r);
    // Collision residue after the clean frame.
    r.ring.feed(&[0x48, 0x12]);
    assert!(r.bus.poll().is_none());

    r.clock.advance(10_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert!(t.evidence.garble_after_last_frame);
}

#[test]
fn parked_collision_residue_is_still_evidence() {
    // Superimposed ENUM replies can wire-AND into a plausible frame PREFIX
    // that parks the resolver (real-fleet finding): the quiet horizon then
    // completes the collect having killed nothing -- indistinguishable from
    // an empty subtree unless unresolved ring bytes count as garble.
    let mut r = rig();
    let mut p = [0u8; 24];
    let prefix = [0u8; UID_LEN];
    let n = build::mgmt_enum(&mut p, 0, &prefix).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Mgmt, 0),
        &p[..n],
    );

    // Break byte + a header claiming a 30-byte frame that never arrives.
    r.ring.frame(&[0x00, 0x05, 30, 0x80]);
    assert!(r.bus.poll().is_none());

    r.clock.advance(10_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 0);
    assert_eq!(t.evidence.garble, 4, "parked bytes are collision evidence");
    assert!(t.evidence.garble_after_last_frame);
}

#[test]
fn junk_behind_the_completing_status_is_trailing_evidence() {
    let mut r = rig();
    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);

    // Reply and tail junk arrive together: completion at the clean frame
    // must still see the tail (the sec 9.2 trailing-energy signal).
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
    r.ring.feed(&[0x00, 0x99]);
    let _ = expect_status(&mut r);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.garble, 2);
    assert!(t.evidence.garble_after_last_frame);
}

#[test]
fn cal_train_rides_a_drift_free_grid() {
    let mut r = rig();
    let mut p = [0u8; 8];
    let n = build::mgmt_cal(&mut p, 400, 3).unwrap();
    exchange(
        &mut r,
        Id::BROADCAST,
        Inst::instruction(Opcode::Mgmt, 0),
        &p[..n],
    );

    // gaps + 1 = 4 breaks, each armed at the previous target + gap -- ISR
    // lateness must not accumulate down the train.
    let mut grid = Vec::new();
    for _ in 0..4 {
        grid.push(r.clock.armed().expect("train slot armed"));
        r.bus.on_deadline();
    }
    assert_eq!(grid, vec![400, 800, 1200, 1600]);

    let breaks = r
        .wire
        .log()
        .iter()
        .filter(|op| **op == WireOp::Break)
        .count();
    assert_eq!(breaks, 5, "announce break + 4 train breaks");
    assert_eq!(*r.wire.log().last().unwrap(), WireOp::Release);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Sent);
}

#[test]
fn rescue_pulses_then_drops_to_the_rescue_rate() {
    let mut r = rig();
    r.bus.submit(Command::Rescue).unwrap();
    assert_eq!(r.wire.log(), vec![WireOp::Claim, WireOp::HoldLow]);
    assert!(r.bus.poll().is_none(), "pulse still held");

    r.clock.advance(1_001);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Sent);
    assert_eq!(*r.wire.log().last().unwrap(), WireOp::Release);
    assert_eq!(r.baud.applied(), vec![BaudRate::RESCUE]);

    // Rescued servos need the pacing gap before crisp turnarounds.
    r.wire.clear();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    assert!(r.wire.log().is_empty(), "paced after rescue");
}

#[test]
fn host_baud_applies_and_completes() {
    let mut r = rig();
    r.bus
        .submit(Command::HostBaud {
            bps: BaudRate::B3000000.as_hz(),
        })
        .unwrap();
    assert_eq!(r.baud.applied(), vec![BaudRate::B3000000]);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
}

/// Ticks per 10-bit character at `bps` on the fake clock.
fn byte_ticks_at(bps: u32) -> u32 {
    (FakeDeadline::TICKS_PER_US * 10_000_000).div_ceil(bps)
}

fn host_baud(r: &mut Rig, bps: u32) {
    r.bus.submit(Command::HostBaud { bps }).unwrap();
    assert_eq!(expect_done(r).outcome, Outcome::Complete);
}

#[test]
fn host_baud_to_a_non_osc_rate_applies_raw_and_paces_at_it() {
    let mut r = rig();
    host_baud(&mut r, 115_200);
    assert_eq!(r.baud.applied_raw(), vec![115_200]);
    assert!(r.baud.applied().is_empty());
    assert_eq!(r.bus.bps(), 115_200);

    let horizon = osc_protocol::wire::STARVE_HORIZON_BYTE_TIMES * byte_ticks_at(115_200);
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    r.clock.advance(horizon - 1);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "paced at 115200's byte-time");
    r.clock.advance(1);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim, "horizon elapsed -> TX");
}

#[test]
fn host_baud_below_the_floor_is_refused() {
    let mut r = rig();
    for bps in [0, MIN_BPS - 1] {
        assert_eq!(
            r.bus.submit(Command::HostBaud { bps }),
            Err(SubmitError::Invalid(InvalidReason::BadPayload))
        );
    }
    assert!(r.baud.applied_raw().is_empty());
    host_baud(&mut r, MIN_BPS);
    assert_eq!(r.baud.applied_raw(), vec![MIN_BPS]);
}

#[test]
fn byte_ticks_round_up_off_the_osc_rates() {
    let mut r = rig();
    // 1e7 / 921600 = 10.85 ticks at 1 tick/us.
    host_baud(&mut r, 921_600);
    assert_eq!(r.bus.byte_ticks(), 11);
}

#[test]
fn cancel_returns_a_non_osc_rate_to_the_rescue_rate() {
    let mut r = rig();
    host_baud(&mut r, 115_200);
    r.bus.cancel();
    assert_eq!(r.baud.applied(), vec![BaudRate::B500000]);
    assert_eq!(r.bus.bps(), 500_000);
}

#[test]
fn byte_time_follows_bps() {
    let mut r = rig();
    let (id, inst) = ping(5);
    let plan = Shape::derive(id, inst, &[]).unwrap();
    host_baud(&mut r, 500_000);
    let at_500k = r.bus.window_for(&plan);
    host_baud(&mut r, 115_200);
    let at_115k = r.bus.window_for(&plan);

    let bytes = plan.reply_footprint as u32 + WINDOW_MARGIN_BYTES;
    assert_eq!(
        at_115k - at_500k,
        bytes * (byte_ticks_at(115_200) - byte_ticks_at(500_000))
    );
}

#[test]
fn response_deadline_setting_stretches_the_window() {
    let mut r = rig();
    r.bus
        .submit(Command::SetResponseDeadline { us: 1_000 })
        .unwrap();
    let _ = expect_done(&mut r);

    let (id, inst) = ping(5);
    exchange(&mut r, id, inst, &[]);
    // The old 310-tick window would have expired; the widened one holds.
    r.clock.advance(700);
    assert!(r.bus.poll().is_none());
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
    let _ = expect_status(&mut r);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
}

/// Submit a stream-tagged tel_count WRITE and drive its TX out.
fn arm_stream(r: &mut Rig, id: Id, window_us: u32) {
    let mut p = [0u8; 8];
    let n = build::write(&mut p, 0x0192, &[2, 0]).unwrap();
    r.bus
        .submit(Command::ExchangeStream {
            id,
            inst: Inst::instruction(Opcode::Write, 0),
            payload: &p[..n],
            window_us,
        })
        .expect("valid stream exchange");
    r.bus.on_tx_complete();
}

#[test]
fn stream_collects_frames_until_last() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 5_000);

    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 0, "the arm's own ack streams first");
    assert!(r.bus.poll().is_none(), "the ack does not finish a stream");

    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 1);
    assert!(r.bus.poll().is_none());

    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[1, 1, 1, 0, 9, 9]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 2);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 3);
    assert_eq!(t.evidence.garble, 0);
}

#[test]
fn broadcast_stream_arm_owes_no_ack_but_still_collects() {
    let mut r = rig();
    // Broadcast COMMIT as the carrier: silent arm, the burst still streams.
    r.bus
        .submit(Command::ExchangeStream {
            id: Id::BROADCAST,
            inst: Inst::instruction(Opcode::Commit, 0),
            payload: &[],
            window_us: 5_000,
        })
        .unwrap();
    r.bus.on_tx_complete();
    assert!(r.bus.poll().is_none(), "awaiting the stream, not Sent");

    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[0, 1, 1, 0, 8, 8]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 0);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 1);
}

#[test]
fn corrupt_stream_frame_is_garble_never_a_status() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 5_000);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);

    // Frame seq 0 arrives with its CRC flipped: no status may surface.
    let mut bad = sealed_status(5, ResultCode::Stream, &[0, 2, 1, 0, 8, 8]);
    let at = bad.len() - 1;
    bad[at] ^= 0xFF;
    r.ring.frame(&bad);
    assert!(r.bus.poll().is_none(), "corrupt frame yields no status");

    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[1, 1, 1, 0, 9, 9]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 1, "the clean LAST frame is the second arrival");
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(
        t.evidence.statuses, 2,
        "ack + LAST; the corrupt frame is not counted"
    );
    assert!(t.evidence.garble > 0, "the corrupt frame is evidence");
}

#[test]
fn a_corrupt_len_right_after_a_clean_frame_never_holds_last() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 5_000);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);

    // Garble on frame seq 0's LEN: a 243-byte claim from a real break, with
    // LAST's break inside it and nothing after.
    let mut bad = sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]);
    bad[2] = 0xF0;
    r.ring.frame(&bad);
    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[1, 1, 1, 0, 9, 9]));

    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 1, "LAST, the second clean arrival");
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 2);
    assert_eq!(usize::from(t.evidence.garble), bad.len());
}

#[test]
fn a_ring_lap_mid_stream_is_counted_not_silent() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 5_000);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);

    // Nine 64 B frames land while the consumer is stalled: the first eight
    // are one whole ring, overwritten by the ninth before any walk.
    let filler = |seq: u8| {
        let mut p = [0x55; 58];
        p[0] = seq;
        p[1] = 0;
        sealed_status(5, ResultCode::Stream, &p)
    };
    assert_eq!(filler(0).len() * 8, crate::testutil::RING_LEN);
    for seq in 0..9 {
        r.ring.frame(&filler(seq));
    }
    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[9, 1, 1, 0, 9, 9]));

    let mut seqs = Vec::new();
    let t = loop {
        match r.bus.poll() {
            Some(Event::Status { payload, .. }) => seqs.push(payload.u8_at(0)),
            Some(Event::Done(t)) => break t,
            None => panic!("stream never finished"),
            Some(other) => panic!("unexpected {other:?}"),
        }
    };
    assert_eq!(
        seqs,
        [Some(8), Some(9)],
        "the walk resumes at the lapped index"
    );
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.laps, 1);
    assert_eq!(t.evidence.garble, 0);
}

#[test]
fn stream_window_never_rearms_on_progress() {
    let mut r = rig();
    arm_stream(&mut r, Id::new(5), 1_000);
    r.ring.frame(&sealed_status(5, ResultCode::Ok, &[]));
    let _ = expect_status(&mut r);

    // Frames keep arriving but LAST never does: the one wide window
    // expires on schedule despite continuous wire progress.
    r.clock.advance(600);
    r.ring
        .frame(&sealed_status(5, ResultCode::Stream, &[0, 0, 1, 0, 8, 8]));
    let _ = expect_status(&mut r);
    r.clock.advance(600);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Timeout { slot: 2 });
    assert_eq!(t.evidence.statuses, 2);
}

#[test]
fn stream_refuses_chain_and_collect_carriers() {
    let mut r = rig();
    let mut p = [0u8; 16];
    let ids = [Id::new(1), Id::new(2)];
    let n = build::gread_uniform(&mut p, 0x0084, 4, &ids).unwrap();
    assert!(matches!(
        r.bus.submit(Command::ExchangeStream {
            id: Id::BROADCAST,
            inst: Inst::instruction(Opcode::Gread, 0),
            payload: &p[..n],
            window_us: 5_000,
        }),
        Err(SubmitError::Invalid(shape::InvalidReason::BadInst))
    ));
    assert!(r.wire.log().is_empty(), "refused before the wire");
}

/// Submit a TbExchange with a fixed header around `data`.
fn tb_submit(
    r: &mut Rig,
    data: &[u8],
    expect: TbExpect,
    window_us: u32,
) -> Result<(), SubmitError> {
    r.bus.submit(Command::TbExchange {
        cmd: 3,
        addr: 0x0008_0400,
        flags: 0x01,
        data,
        expect,
        window_us,
    })
}

/// Submit a TbExchange and drive its TX out.
fn tb_exchange(r: &mut Rig, expect: TbExpect, window_us: u32) {
    tb_submit(r, &[1, 2, 3], expect, window_us).expect("valid tb exchange");
    r.bus.on_tx_complete();
}

/// A sealed tinyboot frame as it lands on the wire.
fn tb_wire(cmd: u8, status: u8, addr: u32, flags: u8, data: &[u8]) -> Vec<u8> {
    let a = addr.to_le_bytes();
    let n = (data.len() as u16).to_le_bytes();
    let mut f = vec![0xAA, 0x55, cmd, status, a[0], a[1], a[2], flags, n[0], n[1]];
    f.extend_from_slice(data);
    let crc = tb_framer::tb_crc(0xFFFF, &f);
    f.extend_from_slice(&crc.to_le_bytes());
    f
}

fn tb_reply(cmd: u8, data: &[u8]) -> Vec<u8> {
    tb_wire(cmd, 1, 0x0012_3456, 0x80, data)
}

/// `(slot, cmd, status, addr, flags, data)`.
fn expect_tb_frame(r: &mut Rig) -> (u8, u8, u8, u32, u8, Vec<u8>) {
    match r.bus.poll() {
        Some(Event::TbFrame {
            slot,
            cmd,
            status,
            addr,
            flags,
            data,
        }) => (slot, cmd, status, addr, flags, data.bytes().collect()),
        other => panic!("expected TbFrame, got {other:?}"),
    }
}

#[test]
fn tb_exchange_sends_the_sealed_frame_with_no_break() {
    let mut r = rig();
    tb_submit(&mut r, &[0xDE, 0xAD, 0xBE], TbExpect::One, 5_000).unwrap();
    let mut frame = vec![
        0xAA, 0x55, 3, 0x00, 0x00, 0x04, 0x08, 0x01, 3, 0, 0xDE, 0xAD, 0xBE,
    ];
    let crc = tb_framer::tb_crc(0xFFFF, &frame);
    frame.extend_from_slice(&crc.to_le_bytes());
    assert_eq!(r.wire.log(), vec![WireOp::Claim, WireOp::Send(frame)]);

    r.bus.on_tx_complete();
    assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
    assert!(!r.wire.log().contains(&WireOp::Break));
    assert!(r.bus.poll().is_none(), "awaiting the reply");
}

#[test]
fn tb_expect_none_is_sent_at_tx_end() {
    let mut r = rig();
    tb_submit(&mut r, &[1], TbExpect::None, 0).unwrap();
    assert!(r.bus.poll().is_none(), "in flight until TC");
    r.bus.on_tx_complete();
    assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Sent);
    assert!(!r.bus.busy());
}

#[test]
fn tb_expect_one_completes_on_the_first_clean_reply() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::One, 5_000);
    r.ring
        .feed(&tb_wire(3, 2, 0x0008_0400, 0x40, &[9, 8, 7, 6]));

    assert_eq!(
        expect_tb_frame(&mut r),
        (0, 3, 2, 0x0008_0400, 0x40, vec![9, 8, 7, 6])
    );
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 1);
    assert_eq!(t.evidence.garble, 0);
    assert!(!r.bus.busy());
}

#[test]
fn tb_expect_one_times_out_at_the_fixed_window() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::One, 1_000);

    // Garble mid-window: evidence, never a re-arm.
    r.clock.advance(600);
    r.ring.feed(&[0x12, 0x34]);
    assert!(r.bus.poll().is_none());
    r.clock.advance(399);
    assert!(r.bus.poll().is_none(), "1 us of the window left");
    r.clock.advance(1);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Timeout { slot: 0 });
    assert_eq!(t.evidence.statuses, 0);
    assert_eq!(t.evidence.garble, 2);
}

#[test]
fn tb_collect_gathers_every_reply_until_the_window_ends() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::Collect, 2_000);

    r.clock.advance(300);
    r.ring.feed(&tb_reply(4, &[0xA1; 12]));
    assert_eq!(expect_tb_frame(&mut r).0, 0);
    assert!(r.bus.poll().is_none(), "collect keeps the window running");
    r.clock.advance(300);
    r.ring.feed(&tb_reply(4, &[0xB2; 12]));
    let (slot, _, _, _, _, data) = expect_tb_frame(&mut r);
    assert_eq!((slot, data), (1, vec![0xB2; 12]));
    assert!(r.bus.poll().is_none());

    r.clock.advance(1_399);
    assert!(r.bus.poll().is_none(), "1 us of the window left");
    r.clock.advance(1);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 2);
    assert_eq!(t.evidence.garble, 0);
}

#[test]
fn tb_collect_counts_a_collision_as_garble() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::Collect, 2_000);

    // Two 24-byte Select replies 3 byte-times apart, wire-ANDed.
    let a = tb_reply(4, &[0xA1; 12]);
    let b = tb_reply(4, &[0x5E; 12]);
    let k = 3;
    let mut wire = a.clone();
    wire.extend_from_slice(&[0xFF; 3]);
    for (i, &byte) in b.iter().enumerate() {
        wire[i + k] &= byte;
    }
    r.ring.feed(&wire);
    assert!(r.bus.poll().is_none(), "no clean frame");

    r.clock.advance(2_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!(t.evidence.statuses, 0);
    assert_eq!(usize::from(t.evidence.garble), wire.len());
}

#[test]
fn tb_partial_reply_at_the_window_end_folds_into_garble() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::One, 1_000);
    r.ring.feed(&tb_reply(4, &[1, 2, 3, 4])[..8]);
    assert!(r.bus.poll().is_none(), "a header still landing");

    r.clock.advance(1_000);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Timeout { slot: 0 });
    assert_eq!(t.evidence.garble, 8);
    assert!(t.evidence.garble_after_last_frame);
}

#[test]
fn tb_submit_rejects_bad_bodies() {
    let mut r = rig();
    let long = [0u8; MAX_TB_PAYLOAD + 1];
    let bad = |reason| Err(SubmitError::Invalid(reason));
    assert_eq!(
        tb_submit(&mut r, &long, TbExpect::One, 1_000),
        bad(InvalidReason::TooLong)
    );
    assert_eq!(
        r.bus.submit(Command::TbExchange {
            cmd: 3,
            addr: 0x0100_0000,
            flags: 0,
            data: &[],
            expect: TbExpect::One,
            window_us: 1_000,
        }),
        bad(InvalidReason::BadPayload),
        "addr over 24 bits"
    );
    for (expect, window_us) in [
        (TbExpect::None, 1),
        (TbExpect::One, 0),
        (TbExpect::Collect, 0),
    ] {
        assert_eq!(
            tb_submit(&mut r, &[1], expect, window_us),
            bad(InvalidReason::BadPayload),
            "{expect:?} with window {window_us}"
        );
    }
    assert!(r.wire.log().is_empty(), "refused before the wire");

    // The largest legal data fills the TX scratch exactly.
    let max = [0u8; MAX_TB_PAYLOAD];
    tb_submit(&mut r, &max, TbExpect::None, 0).unwrap();
    match &r.wire.log()[1] {
        WireOp::Send(f) => assert_eq!(f.len(), TX_LEN),
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn tb_cancel_mid_await_keeps_the_fixed_window_quiet() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::One, 2_000);
    r.clock.advance(500);
    r.bus.cancel();
    assert!(!r.bus.busy());

    r.wire.clear();
    let (id, inst) = ping(5);
    r.bus
        .submit(Command::Exchange {
            id,
            inst,
            payload: &[],
        })
        .unwrap();
    // The device may answer until the old window ends, though the horizon
    // alone is long past.
    r.clock.advance(999);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "the old window is still open");
    // Its late reply lands: nobody's, and it restarts only the starve
    // horizon (640 us at 1M), never the fixed window.
    r.clock.advance(1);
    r.ring.feed(&tb_reply(3, &[1]));
    assert!(r.bus.poll().is_none(), "the old reply is nobody's");
    r.clock.advance(639);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "1 us of the horizon left");
    r.clock.advance(1);
    assert!(r.bus.poll().is_none());
    assert_eq!(r.wire.log()[0], WireOp::Claim);
}

#[test]
fn tb_exchange_waits_out_pacing() {
    let mut r = rig();
    tb_exchange(&mut r, TbExpect::One, 5_000);
    r.ring.feed(&[0x01, 0x02]);
    r.ring.feed(&tb_reply(3, &[]));
    let _ = expect_tb_frame(&mut r);
    assert_eq!(expect_done(&mut r).evidence.garble, 2);

    r.wire.clear();
    tb_submit(&mut r, &[1], TbExpect::One, 5_000).unwrap();
    assert!(r.bus.poll().is_none());
    r.clock.advance(639);
    assert!(r.bus.poll().is_none());
    assert!(r.wire.log().is_empty(), "paced: 1 us of the horizon left");
    r.clock.advance(1);
    assert!(r.bus.poll().is_none());
    let log = r.wire.log();
    assert_eq!(log[0], WireOp::Claim, "horizon elapsed -> TX");
    assert!(matches!(log[1], WireOp::Send(_)), "no break");
}

/// The largest window whose ticks the deadline compare can still see.
const MAX_WINDOW_US: u32 = u32::MAX / 2 / FakeDeadline::TICKS_PER_US;

#[test]
fn tb_window_past_the_tick_horizon_is_refused() {
    let mut r = rig();
    for expect in [TbExpect::One, TbExpect::Collect] {
        assert_eq!(
            tb_submit(&mut r, &[1], expect, MAX_WINDOW_US + 1),
            Err(SubmitError::Invalid(InvalidReason::BadPayload)),
            "{expect:?}"
        );
    }
    assert_eq!(
        tb_submit(&mut r, &[1], TbExpect::One, u32::MAX),
        Err(SubmitError::Invalid(InvalidReason::BadPayload))
    );
    assert!(r.wire.log().is_empty(), "refused before the wire");

    tb_exchange(&mut r, TbExpect::One, MAX_WINDOW_US);
    r.clock.advance(MAX_WINDOW_US - 1);
    assert!(r.bus.poll().is_none(), "the window still runs");
    r.clock.advance(1);
    assert_eq!(expect_done(&mut r).outcome, Outcome::Timeout { slot: 0 });
}

#[test]
fn stream_window_past_the_tick_horizon_is_refused() {
    let mut r = rig();
    let mut p = [0u8; 8];
    let n = build::write(&mut p, 0x0192, &[2, 0]).unwrap();
    for window_us in [MAX_WINDOW_US + 1, u32::MAX] {
        assert_eq!(
            r.bus.submit(Command::ExchangeStream {
                id: Id::new(5),
                inst: Inst::instruction(Opcode::Write, 0),
                payload: &p[..n],
                window_us,
            }),
            Err(SubmitError::Invalid(InvalidReason::BadPayload)),
            "{window_us}"
        );
    }
    assert!(r.wire.log().is_empty(), "refused before the wire");

    arm_stream(&mut r, Id::new(5), MAX_WINDOW_US);
    r.clock.advance(MAX_WINDOW_US - 1);
    assert!(r.bus.poll().is_none(), "the window still runs");
    r.clock.advance(1);
    assert_eq!(expect_done(&mut r).outcome, Outcome::Timeout { slot: 0 });
}

#[test]
fn osc_and_tinyboot_exchanges_alternate_cleanly() {
    let mut r = rig();
    let (id, inst) = ping(5);
    let osc_round = |r: &mut Rig, data: u8| {
        r.wire.clear();
        exchange(r, id, inst, &[]);
        assert_eq!(r.wire.log()[1], WireOp::Break, "osc frames ride a break");
        r.ring.frame(&sealed_status(5, ResultCode::Ok, &[data]));
        match r.bus.poll() {
            Some(Event::Status { slot, payload, .. }) => {
                assert_eq!((slot, payload.segments().0), (0, &[data][..]))
            }
            other => panic!("expected Status, got {other:?}"),
        }
        let t = expect_done(r);
        assert_eq!(t.outcome, Outcome::Complete);
        assert_eq!((t.evidence.statuses, t.evidence.garble), (1, 0));
    };

    osc_round(&mut r, 0x11);

    r.wire.clear();
    tb_exchange(&mut r, TbExpect::One, 5_000);
    assert!(!r.wire.log().contains(&WireOp::Break));
    r.ring.feed(&tb_reply(3, &[0x22]));
    let (slot, cmd, _, _, _, data) = expect_tb_frame(&mut r);
    assert_eq!((slot, cmd, data), (0, 3, vec![0x22]));
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
    assert_eq!((t.evidence.statuses, t.evidence.garble), (1, 0));

    osc_round(&mut r, 0x33);
}

#[cfg(feature = "bench")]
mod wire {
    use super::*;

    fn expect_wire_done(r: &mut Rig) -> u32 {
        match r.bus.poll() {
            Some(Event::WireDone { tick }) => tick,
            other => panic!("expected WireDone, got {other:?}"),
        }
    }

    #[test]
    fn wire_send_is_break_plus_raw_bytes_then_wire_done() {
        let mut r = rig();
        // Deliberately not a legal frame -- raw means raw.
        r.bus.wire_send(&[0xDE, 0xAD]).unwrap();
        let log = r.wire.log();
        assert_eq!(log[0], WireOp::Claim);
        assert_eq!(log[1], WireOp::Break);
        assert_eq!(log[2], WireOp::Send(vec![0xDE, 0xAD]));
        assert!(r.bus.poll().is_none(), "in flight until TC");

        r.clock.advance(70);
        r.bus.on_tx_complete();
        assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
        assert_eq!(expect_wire_done(&mut r), 70, "tick = wire release moment");
        assert!(r.bus.poll().is_none(), "consumed");
    }

    #[test]
    fn wire_burst_chains_frames_on_tc() {
        let mut r = rig();
        r.bus.wire_burst(&[2, 0xAA, 0xBB, 1, 0xCC]).unwrap();
        let log = r.wire.log();
        assert_eq!(
            log,
            vec![WireOp::Claim, WireOp::Break, WireOp::Send(vec![0xAA, 0xBB])]
        );

        r.bus.on_tx_complete();
        let log = r.wire.log();
        assert_eq!(&log[3..], &[WireOp::Break, WireOp::Send(vec![0xCC])]);
        assert!(r.bus.poll().is_none(), "second frame still in flight");

        r.bus.on_tx_complete();
        assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
        let _ = expect_wire_done(&mut r);
    }

    #[test]
    fn wire_pulse_holds_low_until_its_deadline() {
        let mut r = rig();
        r.bus.wire_pulse_low(300).unwrap();
        let log = r.wire.log();
        assert_eq!(log, vec![WireOp::Claim, WireOp::HoldLow]);

        r.clock.advance(299);
        assert!(r.bus.poll().is_none(), "still held");
        r.clock.advance(1);
        let tick = expect_wire_done(&mut r);
        assert_eq!(tick, 300);
        assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
    }

    #[test]
    fn wire_ops_and_commands_reject_each_other_as_busy() {
        let mut r = rig();
        r.bus.wire_send(&[0x55]).unwrap();
        let (id, inst) = ping(1);
        assert_eq!(
            r.bus.submit(Command::Exchange {
                id,
                inst,
                payload: &[]
            }),
            Err(SubmitError::Busy),
            "command rejected while a wire op owns the wire"
        );
        assert_eq!(r.bus.wire_send(&[0x55]), Err(SubmitError::Busy));
        r.bus.on_tx_complete();
        assert_eq!(
            r.bus.wire_pulse_low(10),
            Err(SubmitError::Busy),
            "unconsumed WireDone still owns the engine"
        );
        let _ = expect_wire_done(&mut r);
        r.bus.wire_pulse_low(10).unwrap();

        // And the mirror: a command in flight rejects wire ops.
        let mut r = rig();
        r.bus
            .submit(Command::Exchange {
                id,
                inst,
                payload: &[],
            })
            .unwrap();
        assert_eq!(r.bus.wire_send(&[0x55]), Err(SubmitError::Busy));
    }

    #[test]
    fn malformed_wire_ops_reject_without_touching_the_wire() {
        let mut r = rig();
        assert!(matches!(r.bus.wire_send(&[]), Err(SubmitError::Invalid(_))));
        assert!(matches!(
            r.bus.wire_burst(&[]),
            Err(SubmitError::Invalid(_))
        ));
        assert!(matches!(
            r.bus.wire_burst(&[0]),
            Err(SubmitError::Invalid(_)),
        ));
        assert!(matches!(
            r.bus.wire_burst(&[3, 0xAA]),
            Err(SubmitError::Invalid(_)),
        ));
        assert!(matches!(
            r.bus.wire_pulse_low(0),
            Err(SubmitError::Invalid(_))
        ));
        assert!(r.wire.log().is_empty(), "nothing reached the wire");
    }

    #[test]
    fn wire_train_paces_breaks_on_the_grid_after_the_announce() {
        let mut r = rig();
        // Announce says 392, the wire paces 400: the lying-train injector the
        // engine's own CAL path cannot express.
        let mut p = [0u8; 8];
        let n = build::mgmt_cal(&mut p, 392, 3).unwrap();
        r.bus.wire_train(&p[..n], 400, 4).unwrap();
        assert_eq!(r.wire.log()[..2], [WireOp::Claim, WireOp::Break]);
        assert!(r.bus.poll().is_none(), "announce still in flight");

        // The announce's TC anchors the grid.
        r.bus.on_tx_complete();
        let mut grid = Vec::new();
        for _ in 0..4 {
            grid.push(r.clock.armed().expect("train slot armed"));
            r.bus.on_deadline();
        }
        assert_eq!(grid, vec![400, 800, 1200, 1600]);

        let breaks = r
            .wire
            .log()
            .iter()
            .filter(|op| **op == WireOp::Break)
            .count();
        assert_eq!(breaks, 5, "announce break + 4 train breaks");
        assert_eq!(*r.wire.log().last().unwrap(), WireOp::Release);
        let _ = expect_wire_done(&mut r);
    }

    #[test]
    fn wire_train_rejects_empty_and_busy() {
        let mut r = rig();
        assert!(matches!(
            r.bus.wire_train(&[], 400, 4),
            Err(SubmitError::Invalid(_))
        ));
        assert!(matches!(
            r.bus.wire_train(&[0xFE], 0, 4),
            Err(SubmitError::Invalid(_))
        ));
        assert!(matches!(
            r.bus.wire_train(&[0xFE], 400, 0),
            Err(SubmitError::Invalid(_))
        ));
        r.bus.wire_send(&[0x55]).unwrap();
        assert_eq!(r.bus.wire_train(&[0xFE], 400, 4), Err(SubmitError::Busy));
    }

    #[test]
    fn wire_baud_applies_raw_and_completes_immediately() {
        let mut r = rig();
        // One BRR step off 1M - the trim suite's host-detune probe rate.
        r.bus.wire_baud(993_103).unwrap();
        assert_eq!(r.baud.applied_raw(), vec![993_103]);
        let _ = expect_wire_done(&mut r);

        // The change paces like any baud change.
        let (id, inst) = ping(5);
        r.bus
            .submit(Command::Exchange {
                id,
                inst,
                payload: &[],
            })
            .unwrap();
        assert!(r.wire.log().is_empty(), "paced after the rate change");
    }

    #[test]
    fn cancel_returns_a_detuned_uart_to_the_rescue_rate() {
        let mut r = rig();
        r.bus.wire_baud(993_103).unwrap();
        let _ = expect_wire_done(&mut r);
        assert_eq!(r.bus.bps(), 993_103);
        r.bus.cancel();
        assert_eq!(r.bus.bps(), 500_000);
        assert_eq!(r.baud.applied().last(), Some(&BaudRate::B500000));
    }

    #[test]
    fn cancel_mid_pulse_ends_the_low_hold() {
        let mut r = rig();
        r.bus.wire_pulse_low(5_000).unwrap();
        r.bus.cancel();
        assert_eq!(r.wire.log().last(), Some(&WireOp::Release));
        r.clock.advance(5_000);
        assert!(r.bus.poll().is_none(), "no WireDone for a cancelled pulse");
        assert!(!r.bus.busy());
    }

    #[test]
    fn wire_baud_rejects_zero_and_busy() {
        let mut r = rig();
        assert!(matches!(r.bus.wire_baud(0), Err(SubmitError::Invalid(_))));
        r.bus.wire_send(&[0x55]).unwrap();
        assert_eq!(r.bus.wire_baud(1_000_000), Err(SubmitError::Busy));
    }

    #[test]
    fn wire_baud_below_the_floor_is_refused() {
        let mut r = rig();
        for bps in [0, MIN_BPS - 1] {
            assert_eq!(
                r.bus.wire_baud(bps),
                Err(SubmitError::Invalid(InvalidReason::BadPayload))
            );
        }
        assert!(r.baud.applied_raw().is_empty());
        r.bus.wire_baud(MIN_BPS).unwrap();
        assert_eq!(r.baud.applied_raw(), vec![MIN_BPS]);
    }
}
