//! Client -> records -> production LinkServer -> production engine -> sim
//! wire -> production servo stack, all in-process (fake-adapter backend).
//! Exercised through the blocking facade so both wrappers stay covered.

#![cfg(feature = "fake-adapter")]

use osc_client::blocking::Client;
use osc_client::fake::FakePipe;
use osc_client::mgmt::Uid;
use osc_client::{BaudRate, Error, Id, LinkError, RejectReason};
use osc_protocol::wire::UID_LEN;

/// V006 map fact used by read/write round trips (control.lifecycle
/// goal_velocity); replaced by common-block consts where applicable in
/// landing 3.
const GOAL_VELOCITY: u16 = 392;

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
