//! Negative-space: chip stays silent on bad-CRC / foreign-id / broadcast
//! Write, but a broadcast Write still applies to persistent state.

use bench::{build_ping, build_read, build_write};
use dxl_protocol::types::Id;
use serial_test::serial;

use super::support::{
    BROADCAST_ID, CONTROL_BASE_ADDR, bus, byte_time_us, expect_status_ok, servo_id,
};

const FOREIGN_UNUSED_ID: u8 = 123;

#[test]
#[serial]
fn foreign_id_ping_silent() {
    let mut bus = bus();
    let baud = bus.baud();
    let frame = build_ping(Id::new(FOREIGN_UNUSED_ID)).unwrap();
    let idle_us = 14 * byte_time_us(baud) + 20_000;
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "chip replied to foreign Ping",
    );
}

#[test]
#[serial]
fn bad_crc_ping_silent() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let mut frame: Vec<u8> = build_ping(id).unwrap().to_vec();
    *frame.last_mut().unwrap() ^= 0xFF;
    let idle_us = 14 * byte_time_us(baud) + 20_000;
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "chip replied to bad-CRC Ping",
    );
}

/// Broadcast Write is silent on the wire but its side effect (mutating
/// torque_enable) must land — verified by a follow-up unicast Read.
#[test]
#[serial]
fn broadcast_write_applies_silently() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    // Known starting state.
    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap()).unwrap();

    let bcast_frame = build_write(Id::new(BROADCAST_ID), CONTROL_BASE_ADDR, &[1]).unwrap();
    let idle_us = 14 * byte_time_us(baud) + 20_000;
    assert!(
        bus.xfer_reply(&bcast_frame, idle_us).unwrap().is_none(),
        "broadcast Write got a reply",
    );

    let reply = expect_status_ok(&mut bus, &build_read(id, CONTROL_BASE_ADDR, 1).unwrap()).unwrap();
    assert_eq!(reply.data, vec![1u8], "broadcast Write didn't apply");

    expect_status_ok(&mut bus, &build_write(id, CONTROL_BASE_ADDR, &[0]).unwrap()).unwrap();
}
