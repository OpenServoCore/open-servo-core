use bench::osc::{build_read, build_write};
use osc_servo_core::regions::control::addr::lifecycle::GOAL_VELOCITY;
use serial_test::serial;

use crate::support::bench;

/// WRITE -> empty OK ack, and the value lands. goal_velocity is a plain i32
/// with no range rule and no kernel consumer yet, so an arbitrary value is a
/// clean round-trip probe. The original is restored (state discipline).
#[serial]
#[test]
fn write_goal_velocity_round_trips() {
    let mut b = bench();
    let id = b.id();

    let orig = b.status_ok(&build_read(id, GOAL_VELOCITY, 4)).payload;
    assert_eq!(orig.len(), 4, "goal_velocity is 4 bytes");

    let probe: u32 = 0x1234_ABCD;
    let ack = b.status_ok(&build_write(id, GOAL_VELOCITY, &probe.to_le_bytes()));
    assert!(ack.payload.is_empty(), "a write ack is empty");

    let after = b.status_ok(&build_read(id, GOAL_VELOCITY, 4)).payload;
    assert_eq!(after, probe.to_le_bytes(), "the write applied");

    b.status_ok(&build_write(id, GOAL_VELOCITY, &orig));
}
