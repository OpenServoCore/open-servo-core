use bench::osc::{build_instruction, build_read, build_write};
use osc_core::regions::control::addr::streaming::STREAM_FIELD_MASK;
use osc_protocol::wire::{Inst, Opcode};
use serial_test::serial;

use crate::support::bench;

const BROADCAST: u8 = 0xFE;

/// WRITE+HOLD stages the value (the live field stays put); a broadcast COMMIT
/// applies it. osc collapses DXL's RegWrite/Action into HOLD + COMMIT (protocol sec 5). The
/// original is restored (state discipline).
#[test]
#[serial]
fn hold_then_commit_applies() {
    let mut b = bench();
    let id = b.id();

    let orig = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    let staged: u32 = 0x0BAD_F00D;

    // WRITE + HOLD: acks OK, but the live field keeps its old value.
    let mut payload = STREAM_FIELD_MASK.to_le_bytes().to_vec();
    payload.extend_from_slice(&staged.to_le_bytes());
    b.status_ok(&build_instruction(
        id,
        Opcode::Write,
        Inst::FLAG_HOLD,
        &payload,
    ));

    let held = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    assert_eq!(held, orig, "HOLD does not touch the live field");

    // Broadcast COMMIT is silent and applies the staged write.
    b.expect_no_reply(&build_instruction(BROADCAST, Opcode::Commit, 0, &[]));
    let after = b.status_ok(&build_read(id, STREAM_FIELD_MASK, 4)).payload;
    assert_eq!(
        after,
        staged.to_le_bytes(),
        "COMMIT applied the staged write"
    );

    b.status_ok(&build_write(id, STREAM_FIELD_MASK, &orig));
}
