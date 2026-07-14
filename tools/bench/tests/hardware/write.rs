use bench::osc::{build_read, build_write};
use osc_servo_core::regions::control::addr::streaming::STREAM_FIELD_MASK;
use serial_test::serial;

use crate::support::bench;

/// WRITE -> empty OK ack, and the value lands. stream_field_mask is a plain u32
/// bitmask (no range rule, no motion side-effect), so an arbitrary value is a
/// clean round-trip probe. The original is restored (state discipline).
#[serial]
#[test]
fn write_field_mask_round_trips() {
    let mut b = bench();
    let id = b.id();

    let orig = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    assert_eq!(orig.len(), 4, "stream_field_mask is 4 bytes");

    let probe: u32 = 0x1234_ABCD;
    let ack = b.status_ok(&build_write(id, STREAM_FIELD_MASK, &probe.to_le_bytes()));
    assert!(ack.payload.is_empty(), "a write ack is empty");

    let after = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    assert_eq!(after, probe.to_le_bytes(), "the write applied");

    b.status_ok(&build_write(id, STREAM_FIELD_MASK, &orig));
}
