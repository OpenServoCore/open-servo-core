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
        .feed(&sealed_status(1, ResultCode::Ok, &[0x2A, 0x00, 0x01]));
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
    r.ring.feed(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
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

    // Window: RESPONSE_DEADLINE(60) + (9 + 16 margin) bytes x 10 us = 310.
    r.clock.advance(311);
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
            .feed(&sealed_status(id.as_byte(), ResultCode::Ok, &[0; 4]));
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

    r.ring.feed(&sealed_status(1, ResultCode::Ok, &[0; 4]));
    let _ = expect_status(&mut r);
    r.ring.feed(&sealed_status(2, ResultCode::Ok, &[0; 4]));
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
        .feed(&sealed_status(10, ResultCode::Ok, &[0xAA; UID_LEN]));
    let (slot, _) = expect_status(&mut r);
    assert_eq!(slot, 0);
    r.ring
        .feed(&sealed_status(11, ResultCode::Ok, &[0xBB; UID_LEN]));
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
    r.ring.feed(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
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
        .feed(&sealed_status(10, ResultCode::Ok, &[0xAA; UID_LEN]));
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
    r.ring.feed(&[0x00, 0x05, 30, 0x80]);
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
    r.ring.feed(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
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
    r.bus.submit(Command::HostBaud(BaudRate::B3000000)).unwrap();
    assert_eq!(r.baud.applied(), vec![BaudRate::B3000000]);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
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
    r.ring.feed(&sealed_status(5, ResultCode::Ok, &[1, 2, 3]));
    let _ = expect_status(&mut r);
    let t = expect_done(&mut r);
    assert_eq!(t.outcome, Outcome::Complete);
}
