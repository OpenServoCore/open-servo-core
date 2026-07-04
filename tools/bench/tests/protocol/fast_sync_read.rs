//! FastSyncRead — solo + per-position wire shape.
//!
//! Foreigns in the id list are absent from the bus, so only the chip's
//! bytes appear on the wire. Length + prefix tell us which
//! `SlotPosition` the dispatcher emitted:
//! - **First**: header + body (no CRC)
//! - **Middle / Last**: SILENT — slots k > 0 anchor on the observed
//!   status start (#142); with slot 0 absent the Status packet never
//!   begins, so the chip never fires (single-DUT collapse contract).

use bench::{DEFAULT_IDLE_US, build_fast_sync_read, build_ping, parse_fast_response};
use dxl_protocol::wire::HEADER;
use serial_test::serial;

use super::support::{
    BROADCAST_ID, FOREIGN_A, FOREIGN_B, bus, expect_no_reply, expect_status_ok, servo_id,
    silent_window_us,
};

const INSTR_STATUS: u8 = 0x55;

#[test]
#[serial]
fn fast_sync_read_solo() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let frame = build_fast_sync_read(0, 2, &[id.as_byte()]).unwrap();
    let reply = bus
        .xfer_reply(&frame, DEFAULT_IDLE_US)
        .unwrap()
        .expect("no reply");
    let slots = parse_fast_response(&reply, &[2]).unwrap();
    assert_eq!(slots.len(), 1);
    assert_eq!(slots[0].id, id);
    assert_eq!(slots[0].data.len(), 2);
}

#[test]
#[serial]
fn fast_sync_read_first_emits_header_body_and_checkpoint() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[id.as_byte(), FOREIGN_A, FOREIGN_B]).unwrap();
    let chunk = bus
        .xfer_reply(&frame, silent_window_us(baud, 2, 2))
        .unwrap()
        .expect("no reply");
    // Official layout: slot 0 emits header + block + its cumulative CRC.
    assert_eq!(chunk.len(), 14, "expected 14 bytes, got {}", chunk.len());
    assert_eq!(&chunk[..4], &HEADER);
    assert_eq!(chunk[4], BROADCAST_ID);
    let packet_length = u16::from_le_bytes([chunk[5], chunk[6]]);
    assert_eq!(packet_length, 19, "LEN field = {packet_length}, want 19");
    assert_eq!(chunk[7], INSTR_STATUS);
    assert_eq!(chunk[8], 0, "err = 0x{:02X}", chunk[8]);
    assert_eq!(chunk[9], id.as_byte());
    let wire_crc = u16::from_le_bytes([chunk[12], chunk[13]]);
    assert_eq!(
        dxl_protocol::crc16_umts_continue(0, &chunk[..12]),
        wire_crc,
        "slot 0 checkpoint must be the cumulative packet CRC",
    );
}

/// Slot 1 with an absent slot 0: no Status packet ever starts, so the
/// chip has no anchor and must stay silent (#142 collapse contract).
/// Under the old `packet_end + RDT + offset` formula this fired 4 bytes
/// into a dead window; a follow-up ping pins that the parked slot
/// un-parks cleanly.
#[test]
#[serial]
fn fast_sync_read_middle_with_silent_first_stays_silent() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[FOREIGN_A, id.as_byte(), FOREIGN_B]).unwrap();
    expect_no_reply(&mut bus, &frame, silent_window_us(baud, 2, 2));
    let ping = build_ping(id).expect("encode ping");
    let reply = expect_status_ok(&mut bus, &ping).expect("servo recovers after collapsed chain");
    assert_eq!(reply.id, id);
}

/// Last slot with both predecessors absent — same collapse contract as
/// Middle, plus it pins that the fold pipeline never engages (a parked
/// Last wait gates polls; silence must not deafen the servo).
#[test]
#[serial]
fn fast_sync_read_last_with_silent_predecessors_stays_silent() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[FOREIGN_A, FOREIGN_B, id.as_byte()]).unwrap();
    expect_no_reply(&mut bus, &frame, silent_window_us(baud, 2, 2));
    let ping = build_ping(id).expect("encode ping");
    let reply = expect_status_ok(&mut bus, &ping).expect("servo recovers after collapsed chain");
    assert_eq!(reply.id, id);
}

#[test]
#[serial]
fn fast_sync_read_id_not_in_list_silent() {
    let mut bus = bus();
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[FOREIGN_A, FOREIGN_B]).unwrap();
    assert!(
        bus.xfer_reply(&frame, silent_window_us(baud, 1, 2))
            .unwrap()
            .is_none(),
    );
}
