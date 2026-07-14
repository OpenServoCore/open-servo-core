use bench::osc::{build_instruction, build_read, build_write};
use osc_protocol::wire::{Inst, Opcode};
use osc_servo_core::regions::control::addr::streaming::STREAM_FIELD_MASK;
use serial_test::serial;

use crate::support::bench;

const BROADCAST: u8 = 0xFE;

/// A NOREPLY write draws no status frame (protocol sec 5 NOREPLY flag). We write the field's
/// current value back -- inert, but the write still executes silently.
#[serial]
#[test]
fn noreply_write_is_silent() {
    let mut b = bench();
    let id = b.id();
    let orig = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;

    let mut payload = STREAM_FIELD_MASK.to_le_bytes().to_vec();
    payload.extend_from_slice(&orig);
    b.expect_no_reply(&build_instruction(
        id,
        Opcode::Write,
        Inst::FLAG_NOREPLY,
        &payload,
    ));
}

/// A broadcast write is silent by addressing (protocol sec 5): no servo replies to 0xFE.
#[serial]
#[test]
fn broadcast_write_is_silent() {
    let mut b = bench();
    let id = b.id();
    let orig = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    b.expect_no_reply(&build_write(BROADCAST, STREAM_FIELD_MASK, &orig));
}
