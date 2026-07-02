use bench::{DEFAULT_IDLE_US, build_bulk_read, parse_status_reply};
use dxl_protocol::types::{BulkReadEntry, Id};
use serial_test::serial;

use super::support::{FOREIGN_A, FOREIGN_B, bus, servo_id, silent_window_us};

fn entry(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

#[test]
#[serial]
fn bulk_read_solo() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let frame = build_bulk_read(&[entry(id.as_byte(), 0, 2)]).unwrap();
    let reply = bus
        .xfer_reply(&frame, DEFAULT_IDLE_US)
        .unwrap()
        .expect("no reply");
    let parsed = parse_status_reply(&reply, None).unwrap();
    assert_eq!(parsed.id, id);
    assert_eq!(parsed.data.len(), 2);
}

/// Chain-tail collapse: see the equivalent SyncRead test for the
/// snoop-contract rationale.
#[test]
#[serial]
fn bulk_read_middle_slot_silent_when_predecessor_absent() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_bulk_read(&[
        entry(FOREIGN_A, 0, 4),
        entry(id.as_byte(), 0, 2),
        entry(FOREIGN_B, 0, 8),
    ])
    .unwrap();
    let idle_us = silent_window_us(baud, 2, 8);
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "chip fired slot 1 without observing slot 0",
    );
}

#[test]
#[serial]
fn bulk_read_id_not_in_list_silent() {
    let mut bus = bus();
    let baud = bus.baud();
    let frame = build_bulk_read(&[entry(FOREIGN_A, 0, 2), entry(FOREIGN_B, 0, 2)]).unwrap();
    let idle_us = silent_window_us(baud, 1, 2);
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "BulkRead w/o our id got a reply",
    );
}
