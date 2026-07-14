//! Client -> records -> production LinkServer -> production engine -> sim
//! wire -> production servo stack, all in-process (fake-adapter backend).
//! Exercised through the blocking facade so both wrappers stay covered.

#![cfg(feature = "fake-adapter")]

use osc_client::blocking::Client;
use osc_client::common::{Health, Identity};
use osc_client::cyclic::{Cycle, Group, Telemetry};
use osc_client::fake::FakePipe;
use osc_client::mgmt::Uid;
use osc_client::{BaudRate, Error, Id, LinkError, RejectReason};
use osc_protocol::table;
use osc_protocol::wire::UID_LEN;

/// V006 map fact used by read/write round trips (control.lifecycle
/// goal_velocity); the common block is the only protocol-fixed address space.
const GOAL_VELOCITY: u16 = 392;
/// Slot 0 of the profile region (protocol sec 5.2 pin).
const PROFILE_SLOT0: u16 = 0x280;

/// Span word encoding, protocol sec 5.2: `[addr:10][count:6]`.
const fn span_word(addr: u16, count: u16) -> u16 {
    (addr << 6) | count
}

fn fleet(ids: &[u8]) -> Client<FakePipe> {
    Client::connect(FakePipe::new(BaudRate::B1000000, ids)).expect("connect")
}

#[test]
fn connect_reports_link_info() {
    let c = fleet(&[1]);
    assert_eq!(c.info().version, 1);
    assert!(c.info().ticks_per_us > 0);
}

#[test]
fn ping_digests_model_and_fw() {
    let mut c = fleet(&[5]);
    let ping = c.ping(Id::new(5)).expect("ping");
    assert!(!ping.alert);
    // Model/fw come from the servo's identity block; nonzero model is the
    // seeded default.
    assert!(ping.model > 0);
}

#[test]
fn write_reads_back() {
    let mut c = fleet(&[5]);
    let val = 0x0B0B_0B0Bu32.to_le_bytes();
    c.write(Id::new(5), GOAL_VELOCITY, &val).expect("write");
    let got = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(got, val);
}

#[test]
fn noreply_write_applies_silently() {
    let mut c = fleet(&[5]);
    let val = 0x1122_3344u32.to_le_bytes();
    c.write_noreply(Id::new(5), GOAL_VELOCITY, &val)
        .expect("noreply write");
    let got = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(got, val);
}

#[test]
fn hold_then_commit_applies_atomically() {
    let mut c = fleet(&[5]);
    let val = 0x0505_0505u32.to_le_bytes();
    c.write_hold(Id::new(5), GOAL_VELOCITY, &val).expect("hold");
    let before = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_ne!(before, val, "held write must not apply before COMMIT");
    c.commit().expect("commit");
    let after = c.read(Id::new(5), GOAL_VELOCITY, 4).expect("read");
    assert_eq!(after, val);
}

#[test]
fn gread_chains_in_slot_order() {
    let mut c = fleet(&[1, 2, 3]);
    let ids = [Id::new(1), Id::new(2), Id::new(3)];
    let chain = c.gread(&ids, GOAL_VELOCITY, 4).expect("gread");
    assert_eq!(chain.timeout_slot, None);
    let order: Vec<(u8, u8)> = chain.statuses.iter().map(|s| (s.slot, s.id)).collect();
    assert_eq!(order, vec![(0, 1), (1, 2), (2, 3)]);
}

#[test]
fn gwrite_fans_out_per_target_values() {
    let mut c = fleet(&[1, 2]);
    let a = 0x0000_00AAu32.to_le_bytes();
    let b = 0x0000_00BBu32.to_le_bytes();
    c.gwrite(
        GOAL_VELOCITY,
        4,
        &[(Id::new(1), &a[..]), (Id::new(2), &b[..])],
    )
    .expect("gwrite");
    assert_eq!(c.read(Id::new(1), GOAL_VELOCITY, 4).expect("read"), a);
    assert_eq!(c.read(Id::new(2), GOAL_VELOCITY, 4).expect("read"), b);
}

#[test]
fn absent_servo_times_out() {
    let mut c = fleet(&[5]);
    match c.ping(Id::new(9)) {
        Err(Error::Timeout { slot: 0 }) => {}
        other => panic!("expected timeout, got {other:?}"),
    }
}

#[test]
fn invalid_id_rejects_before_the_wire() {
    let mut c = fleet(&[5]);
    match c.ping(Id::new(0)) {
        Err(Error::Link(LinkError::Rejected(RejectReason::BadId))) => {}
        other => panic!("expected BadId rejection, got {other:?}"),
    }
}

#[test]
fn servo_result_codes_surface_as_errors() {
    let mut c = fleet(&[5]);
    // A read past the table answers `range` at the instruction layer.
    match c.read(Id::new(5), 0x3FF, 64) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Servo(Range), got {other:?}"),
    }
}

#[test]
fn rescue_reunites_at_the_rescue_rate() {
    let mut c = fleet(&[5]);
    let roster = c.rescue_sweep(&[Id::new(5)]).expect("rescue sweep");
    assert_eq!(roster, vec![(Id::new(5), true)]);
}

#[test]
fn cal_trains_complete_and_stay_wire_invisible() {
    let mut c = fleet(&[5]);
    c.cal(2, 400, 4).expect("cal");
    // sec 9.3: the train is invisible to the link counters.
    let diag = c.pipe_mut().sim_mut().servo_diag(0);
    assert_eq!(diag.crc_fail_count, 0);
    assert_eq!(diag.framing_drop_count, 0);
    c.ping(Id::new(5)).expect("ping after cal");
}

#[test]
fn discover_walks_out_every_uid() {
    let mut c = fleet(&[1, 2, 3]);
    let mut want = Vec::new();
    for (i, seed) in [0x11u8, 0x2E, 0x93].into_iter().enumerate() {
        let mut uid = [0u8; UID_LEN];
        uid[0] = seed;
        uid[5] = 0xC0 | i as u8;
        c.pipe_mut().sim_mut().seed_servo_uid(i, uid);
        want.push(Uid(uid));
    }
    want.sort();
    let found = c.discover().expect("discover");
    assert_eq!(found, want);
}

#[test]
fn assign_moves_the_matcher_to_its_new_id() {
    let mut c = fleet(&[1]);
    let uid = [0xA7u8; UID_LEN];
    c.pipe_mut().sim_mut().seed_servo_uid(0, uid);
    c.assign(&Uid(uid), Id::new(9)).expect("assign");
    c.ping(Id::new(9)).expect("ping at the new id");
    match c.ping(Id::new(1)) {
        Err(Error::Timeout { .. }) => {}
        other => panic!("old id must be vacant, got {other:?}"),
    }
}

#[test]
fn rails_and_bootloader_ack() {
    let mut c = fleet(&[1]);
    c.set_rails(true, false).expect("rails");
    c.enter_bootloader().expect("bootloader");
}

#[test]
fn identity_decodes_the_config_front() {
    let mut c = fleet(&[5]);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.config.common.hardware_revision = 3;
        t.config.common.capability_flags = 0x8000_0001;
    });
    let got = c.identity(Id::new(5)).expect("identity");
    assert_eq!(
        got,
        Identity {
            model: 0x1234, // the sim's seeded default
            fw: 0x56,
            hw: 3,
            capabilities: 0x8000_0001,
        }
    );
}

#[test]
fn health_reads_the_telemetry_front_and_tracks_dirty() {
    let mut c = fleet(&[5]);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.common.fault_flags = 0x04;
        t.telemetry.common.trim_steps = -3;
        t.telemetry.common.crc_fail_count = 7;
        t.telemetry.common.framing_drop_count = 9;
    });
    // fault_flags nonzero sets ALERT on the reply; the digest must not
    // mistake that for an error.
    let got = c.health(Id::new(5)).expect("health");
    assert_eq!(
        got,
        Health {
            fault_flags: 0x04,
            config_dirty: false,
            trim_steps: -3,
            crc_fail_count: 7,
            framing_drop_count: 9,
        }
    );
    c.write(
        Id::new(5),
        table::RESPONSE_DEADLINE_US,
        &60u16.to_le_bytes(),
    )
    .expect("config write");
    assert!(c.health(Id::new(5)).expect("health").config_dirty);
}

#[test]
fn clear_counters_zeroes_both_in_one_write() {
    let mut c = fleet(&[5]);
    c.pipe_mut().sim_mut().servo_table_mut(0, |t| {
        t.telemetry.common.crc_fail_count = 41;
        t.telemetry.common.framing_drop_count = 8;
    });
    c.clear_counters(Id::new(5)).expect("clear");
    let h = c.health(Id::new(5)).expect("health");
    assert_eq!((h.crc_fail_count, h.framing_drop_count), (0, 0));
}

/// The park mechanism itself is pinned in osc-integration's `cross_baud`
/// suite -- the link-mode rig cannot hold a parked resolver across
/// commands (every exchange drains the sim queue = unbounded quiet), so
/// this level pins the paced choreography only.
#[test]
fn set_baud_migrates_servo_first_and_reunites() {
    let mut c = fleet(&[1, 2]);
    let ids = [Id::new(1), Id::new(2)];
    let roster = c.set_baud(&ids, BaudRate::B3000000).expect("set_baud");
    assert_eq!(roster, vec![(Id::new(1), true), (Id::new(2), true)]);
    let chain = c.gread(&ids, GOAL_VELOCITY, 4).expect("gread at 3M");
    assert_eq!(chain.timeout_slot, None);
    assert_eq!(chain.statuses.len(), 2);
}

#[test]
fn cal_verify_traces_every_train() {
    let mut c = fleet(&[5]);
    let traces = c.cal_verify(&[Id::new(5)], 3, 400, 4).expect("cal_verify");
    assert_eq!(traces.len(), 1);
    assert_eq!(traces[0].id, Id::new(5));
    assert_eq!(traces[0].trims.len(), 3);
    // Sim clocks don't drift, so the applied total never moves -- the
    // convergence verdict is what silicon exercises.
    assert!(traces[0].converged());
}

#[test]
fn cycle_step_commits_the_writes_the_chain_echoes() {
    let mut c = fleet(&[1, 2]);
    let ids = vec![Id::new(1), Id::new(2)];
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: ids.clone(),
        }],
        Telemetry::Span {
            ids: ids.clone(),
            addr: GOAL_VELOCITY,
            count: 4,
        },
    );
    let a = 0x0000_00AAu32.to_le_bytes();
    let b = 0x0000_00BBu32.to_le_bytes();
    let chain = c.step(&cycle, &[&a, &b]).expect("step");
    assert_eq!(chain.timeout_slot, None);
    let echoed: Vec<&[u8]> = chain.statuses.iter().map(|s| &s.payload[..]).collect();
    assert_eq!(echoed, vec![&a[..], &b[..]]);
}

#[test]
fn cycle_step_rejects_a_malformed_payload_batch() {
    let mut c = fleet(&[1]);
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: vec![Id::new(1)],
        }],
        Telemetry::Span {
            ids: vec![Id::new(1)],
            addr: GOAL_VELOCITY,
            count: 4,
        },
    );
    match c.step(&cycle, &[]) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Range on a missing slot, got {other:?}"),
    }
    match c.step(&cycle, &[&[0u8; 2][..]]) {
        Err(Error::Servo(osc_client::ResultCode::Range)) => {}
        other => panic!("expected Range on a narrow slice, got {other:?}"),
    }
}

#[test]
fn cycle_profile_telemetry_streams_the_slot() {
    let mut c = fleet(&[5]);
    let ids = vec![Id::new(5)];
    // Slot 0: the 4-byte goal echo plus the alarm/status pair -- scattered
    // registers one slot byte fetches per step.
    let words = [
        span_word(GOAL_VELOCITY, 4),
        span_word(table::FAULT_FLAGS, 2),
    ];
    let mut spans = Vec::new();
    for w in words {
        spans.extend_from_slice(&w.to_le_bytes());
    }
    c.write(Id::new(5), PROFILE_SLOT0, &spans)
        .expect("configure slot");
    let cycle = Cycle::new(
        vec![Group {
            addr: GOAL_VELOCITY,
            count: 4,
            ids: ids.clone(),
        }],
        Telemetry::Profile { ids, slot: 0 },
    );
    let goal = 0x0000_0C0Cu32.to_le_bytes();
    let chain = c.step(&cycle, &[&goal]).expect("step");
    assert_eq!(chain.timeout_slot, None);
    let [status] = chain.statuses.as_slice() else {
        panic!("one status, got {}", chain.statuses.len());
    };
    assert_eq!(status.payload.len(), 6);
    assert_eq!(&status.payload[..4], &goal);
}

// --- wire instrument (0x6x family) ---

#[test]
fn wire_send_releases_and_reports_tick() {
    let mut c = fleet(&[5]);
    // A CRC-broken ping: rings at the servo, dies at its verdict, draws no
    // reply -- raw means raw, the engine never validated it.
    let t1 = c.wire_send(&[0x01, 0x03, 0x10, 0xFF, 0xFF]).expect("send");
    let t2 = c.wire_send(&[0x55, 0xAA]).expect("send again");
    assert!(t2 > t1, "release ticks advance with sim time");
}

#[test]
fn engine_commands_stay_healthy_across_instrument_use() {
    let mut c = fleet(&[5]);
    c.wire_send(&[0x01, 0x03, 0x10, 0xFF, 0xFF]).expect("send");
    c.wire_pulse_low(50).expect("pulse");
    // The pulse reads as a break at the fleet (any >=10-bit low span), so
    // the servo parks on a phantom candidate -- instrument traffic obeys
    // the sec 8 pacing rule like any other garble source: one starve
    // horizon of quiet before expecting crisp turnarounds.
    c.pause(std::time::Duration::from_millis(1));
    let ping = c.ping(Id::new(5)).expect("ping after instrument ops");
    assert!(ping.model > 0);
}

#[test]
fn wire_burst_chains_frames() {
    let mut c = fleet(&[5]);
    let f1 = [0x01, 0x03, 0x10, 0xFF, 0xFF];
    let f2 = [0x02, 0x03, 0x10, 0xFF, 0xFF];
    c.wire_burst(&[&f1, &f2]).expect("burst");
}

#[test]
fn wire_burst_rejects_unencodable_frames_client_side() {
    let mut c = fleet(&[5]);
    let err = c.wire_burst(&[&[]]).unwrap_err();
    assert_eq!(
        err,
        Error::Link(LinkError::Rejected(RejectReason::Malformed))
    );
    let long = vec![0u8; 256];
    let err = c.wire_burst(&[&long]).unwrap_err();
    assert_eq!(
        err,
        Error::Link(LinkError::Rejected(RejectReason::Malformed))
    );
}

#[test]
fn empty_wire_send_is_rejected_by_the_adapter() {
    let mut c = fleet(&[5]);
    // A bare break never raises TC, so the engine refuses to wedge on it --
    // pinned here as the round-trip REJECTED path.
    let err = c.wire_send(&[]).unwrap_err();
    assert_eq!(
        err,
        Error::Link(LinkError::Rejected(RejectReason::Malformed))
    );
}

#[test]
fn edge_drain_and_reset_round_trip() {
    let mut c = fleet(&[5]);
    // The sim has no edge model (capture is silicon-only); the shape and
    // the acks are what this pins.
    let drain = c.drain_edges().expect("drain");
    assert!(drain.edges.is_empty());
    assert!(!drain.overflow);
    c.reset_capture().expect("reset");
    let drain = c.drain_edges().expect("drain after reset");
    assert!(drain.edges.is_empty());
}
