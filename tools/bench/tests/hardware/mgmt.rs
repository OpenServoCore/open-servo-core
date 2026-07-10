use bench::osc::{build_assign, build_enum, build_ping};
use osc_protocol::wire::UID_LEN;
use serial_test::serial;

use crate::support::bench;

/// Broadcast ENUM at the tree root (prefix_len 0) — the one query a
/// single-servo bench can always answer cleanly. The reply is the fixed
/// 16-byte UID field (§9.2): the V006's 96-bit ESIG in the low 12 bytes
/// (not the all-zero pattern a missed ESIG read would seed), zero pad above.
#[test]
#[serial]
fn enum_root_reports_the_esig_uid() {
    let mut b = bench();

    let status = b.status_ok(&build_enum(0, &[]));
    assert_eq!(status.id, b.id(), "the reply carries the responder's id");
    assert_eq!(status.payload.len(), UID_LEN);
    assert_ne!(status.payload[..12], [0u8; 12], "ESIG read seeded nothing");
    assert_eq!(status.payload[12..], [0u8; 4], "pad above the 96-bit ESIG");
    println!("ESIG UID: {:02x?}", status.payload);

    // An exact-match prefix (all 128 bits) selects the same servo; one
    // flipped bit selects nobody.
    let uid: [u8; UID_LEN] = status.payload.clone().try_into().expect("16 bytes");
    let full = b.status_ok(&build_enum(128, &uid));
    assert_eq!(full.payload, status.payload);
    let mut miss = uid;
    miss[0] ^= 1;
    b.expect_no_reply(&build_enum(128, &miss));
}

/// Broadcast ASSIGN moves the servo to a new id — the ack already leaves from
/// it (§9.2) — then back. PINGs prove the swap on the wire both ways.
#[test]
#[serial]
fn assign_moves_the_id_and_acks_from_it() {
    let mut b = bench();
    let home = b.id();
    let away = if home == 42 { 43 } else { 42 };

    let uid: [u8; UID_LEN] = b
        .status_ok(&build_enum(0, &[]))
        .payload
        .try_into()
        .expect("16 bytes");

    let ack = b.status_ok(&build_assign(&uid, away));
    assert_eq!(ack.id, away, "the ack leaves from the new id");
    b.status_ok(&build_ping(away));
    b.expect_no_reply(&build_ping(home));

    // Restore before asserting anything else — a failure here must not leave
    // the bench off its configured id.
    let back = b.status_ok(&build_assign(&uid, home));
    assert_eq!(back.id, home);
    b.status_ok(&build_ping(home));
}
