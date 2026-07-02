//! FastSyncRead — solo + per-position wire shape.
//!
//! Foreigns in the id list are absent from the bus, so only the chip's
//! bytes appear on the wire. Length + prefix tell us which
//! `SlotPosition` the dispatcher emitted:
//! - **First**: header + body (no CRC)
//! - **Middle**: body only
//! - **Last**: body + self-CRC (no chain predecessor to fold over)

use bench::{DEFAULT_IDLE_US, build_fast_sync_read, parse_fast_response};
use dxl_protocol::wire::HEADER;
use dxl_protocol::{CrcUmts, SoftwareCrcUmts};
use serial_test::serial;

use super::support::{BROADCAST_ID, FOREIGN_A, FOREIGN_B, bus, servo_id, silent_window_us};

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
fn fast_sync_read_first_emits_header_and_body_only() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[id.as_byte(), FOREIGN_A, FOREIGN_B]).unwrap();
    let chunk = bus
        .xfer_reply(&frame, silent_window_us(baud, 2, 2))
        .unwrap()
        .expect("no reply");
    assert_eq!(chunk.len(), 12, "expected 12 bytes, got {}", chunk.len());
    assert_eq!(&chunk[..4], &HEADER);
    assert_eq!(chunk[4], BROADCAST_ID);
    let packet_length = u16::from_le_bytes([chunk[5], chunk[6]]);
    assert_eq!(packet_length, 15, "LEN field = {packet_length}, want 15");
    assert_eq!(chunk[7], INSTR_STATUS);
    assert_eq!(chunk[8], 0, "err = 0x{:02X}", chunk[8]);
    assert_eq!(chunk[9], id.as_byte());
}

#[test]
#[serial]
fn fast_sync_read_middle_emits_body_only() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[FOREIGN_A, id.as_byte(), FOREIGN_B]).unwrap();
    let chunk = bus
        .xfer_reply(&frame, silent_window_us(baud, 2, 2))
        .unwrap()
        .expect("no reply");
    assert_eq!(chunk.len(), 4, "expected 4 bytes, got {}", chunk.len());
    assert_eq!(chunk[0], 0, "err = 0x{:02X}", chunk[0]);
    assert_eq!(chunk[1], id.as_byte());
}

/// Chip fires slot k>0 on RDT timing even when the preceding foreigns
/// are absent (chain-snoop anchoring is pending #142). Its fold path
/// then races the placeholder-CRC deadline against ghost predecessor
/// bytes and emits `00 00` instead of the real chain CRC. Unignore once
/// #134 (HW-fired kickoff) + #142 (anchor from observed status-start)
/// land — expected to pass then without further test changes.
#[test]
#[serial]
#[ignore = "chip fires + CRC-patches slot k>0 without predecessor observation; pending #134 + #142"]
fn fast_sync_read_last_emits_body_and_self_crc() {
    let mut bus = bus();
    let id = servo_id(&bus);
    let baud = bus.baud();
    let frame = build_fast_sync_read(0, 2, &[FOREIGN_A, FOREIGN_B, id.as_byte()]).unwrap();
    let chunk = bus
        .xfer_reply(&frame, silent_window_us(baud, 2, 2))
        .unwrap()
        .expect("no reply");
    assert_eq!(chunk.len(), 6, "expected 6 bytes, got {}", chunk.len());
    assert_eq!(chunk[0], 0);
    assert_eq!(chunk[1], id.as_byte());
    let mut crc = SoftwareCrcUmts::new();
    crc.update(&chunk[..4]);
    let expected = crc.finalize();
    let observed = u16::from_le_bytes([chunk[4], chunk[5]]);
    assert_eq!(
        observed, expected,
        "CRC over body alone = 0x{observed:04X}, want 0x{expected:04X}",
    );
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
