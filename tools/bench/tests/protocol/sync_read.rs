use bench::{DEFAULT_IDLE_US, build_sync_read, parse_status_reply};
use serial_test::serial;

use super::support::{FOREIGN_A, FOREIGN_B, bus, servo_id, silent_window_us};

#[test]
#[serial]
fn sync_read_solo() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let frame = build_sync_read(0, 2, &[id.as_byte()]).unwrap();
    let reply = bus
        .xfer_reply(&frame, DEFAULT_IDLE_US)
        .unwrap()
        .expect("no reply");
    let parsed = parse_status_reply(&reply, None).unwrap();
    assert_eq!(parsed.id, id);
    assert_eq!(parsed.data.len(), 2);
}

/// Chain-tail collapse: chip in slot 1 (foreigns bracketing) stays silent
/// when the preceding foreign never emits a Status frame. This mirrors
/// the sync/bulk chain-snoop-contract — slot k>0 fires only after
/// observing slot k-1's status on the wire; a silent predecessor
/// collapses the tail. Positive-direction (real predecessor via pirate
/// injection) would land in a chain-timing suite once #142 lands.
#[test]
#[serial]
fn sync_read_middle_slot_silent_when_predecessor_absent() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_sync_read(0, 2, &[FOREIGN_A, id.as_byte(), FOREIGN_B]).unwrap();
    let idle_us = silent_window_us(baud, 2, 2);
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "chip fired slot 1 without observing slot 0",
    );
}

/// Servo id absent from the list: bus must stay silent.
#[test]
#[serial]
fn sync_read_id_not_in_list_silent() {
    let mut bus = bus();
    let baud = bus.baud();
    let frame = build_sync_read(0, 2, &[FOREIGN_A, FOREIGN_B]).unwrap();
    let idle_us = silent_window_us(baud, 1, 2);
    assert!(
        bus.xfer_reply(&frame, idle_us).unwrap().is_none(),
        "SyncRead w/o our id got a reply",
    );
}
