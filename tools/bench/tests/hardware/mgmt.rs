use bench::osc::{
    REBOOT_SETTLE_MS, SAVE_SETTLE_MS, build_assign, build_enum, build_factory, build_ping,
    build_read, build_reboot, build_save, build_write,
};
use osc_protocol::wire::UID_LEN;
use osc_servo_core::regions::config::addr::comms::RESPONSE_DEADLINE_US;
use osc_servo_core::regions::telemetry::addr::fault::CONFIG_DIRTY;
use serial_test::serial;

use crate::support::bench;

/// ENUM prefix matching, fleet-safe: the DUT's UID comes from the prefix
/// walk (collision descent -- a root query collides on a fleet BY DESIGN,
/// protocol sec 9.2, so no test may assert a clean root reply). The UID is the fixed
/// 16-byte field: the V006's 96-bit ESIG in the low 12 bytes (not the
/// all-zero pattern a missed ESIG read would seed), zero pad above. An
/// exact 128-bit prefix selects exactly the DUT; one flipped bit nobody.
#[test]
#[serial]
fn enum_prefix_selects_exactly_the_dut() {
    let mut b = bench();

    let uid = b.dut_uid();
    assert_eq!(uid.len(), UID_LEN);
    assert_ne!(uid[..12], [0u8; 12], "ESIG read seeded nothing");
    assert_eq!(uid[12..], [0u8; 4], "pad above the 96-bit ESIG");
    println!("ESIG UID: {uid:02x?}");

    let full = b.status_ok(&build_enum(128, &uid));
    assert_eq!(full.id, b.id(), "the reply carries the responder's id");
    assert_eq!(full.payload, uid);
    let mut miss = uid;
    miss[0] ^= 1;
    b.expect_no_reply(&build_enum(128, &miss));
}

/// Broadcast ASSIGN moves the servo to a new id -- the ack already leaves from
/// it (protocol sec 9.2) -- then back. PINGs prove the swap on the wire both ways.
/// Fleet-safe: ASSIGN is UID-addressed (one matcher on any bus), and the
/// away id sits outside every bench and fleet id map.
#[test]
#[serial]
fn assign_moves_the_id_and_acks_from_it() {
    let mut b = bench();
    let home = b.id();
    let away = if home == 42 { 43 } else { 42 };

    let uid = b.dut_uid();

    let ack = b.status_ok(&build_assign(&uid, away));
    assert_eq!(ack.id, away, "the ack leaves from the new id");
    b.status_ok(&build_ping(away));
    b.expect_no_reply(&build_ping(home));

    // Restore before asserting anything else -- a failure here must not leave
    // the bench off its configured id.
    let back = b.status_ok(&build_assign(&uid, home));
    assert_eq!(back.id, home);
    b.status_ok(&build_ping(home));
}

/// The full protocol sec 9.4/protocol sec 9.5 silicon round trip: a config marker survives SAVE +
/// REBOOT (real erase, page program, and boot overlay on real flash), then
/// FACTORY wipes both slots and its self-reboot restores board defaults.
/// Fleet-safe tail: factory also resets the DUT's ID to the board default
/// (which may collide with a live fleet id), so the first post-factory
/// exchange is a UID-addressed ASSIGN back home, then a SAVE re-persists
/// the identity -- the bus leaves exactly as found. Reads are collected
/// first and asserted only after the restore, so a bad marker never
/// strands a saved image on the bench.
#[test]
#[serial]
fn save_persists_across_reboot_until_factory() {
    const MARKER: u16 = 123; // response_deadline_us; board default is 60

    let mut b = bench();
    let id = b.id();
    let uid = b.dut_uid();

    b.status_ok(&build_write(
        id,
        RESPONSE_DEADLINE_US,
        &MARKER.to_le_bytes(),
    ));
    let dirty_before = b.status_ok(&build_read(id, CONFIG_DIRTY, 1)).payload[0];

    b.status_ok_within(&build_save(id), SAVE_SETTLE_MS);
    let dirty_after = b.status_ok(&build_read(id, CONFIG_DIRTY, 1)).payload[0];

    // REBOOT acks first, then resets; the marker must come back from flash.
    b.status_ok(&build_reboot(id));
    std::thread::sleep(std::time::Duration::from_millis(REBOOT_SETTLE_MS));
    let persisted = b
        .status_ok(&build_read(id, RESPONSE_DEADLINE_US, 2))
        .payload;

    // FACTORY wipes both slots and reboots itself back to board defaults.
    // Re-home by UID BEFORE any id-addressed exchange: the board-default id
    // the DUT booted with may belong to another servo on a fleet.
    b.status_ok_within(&build_factory(id), SAVE_SETTLE_MS);
    std::thread::sleep(std::time::Duration::from_millis(REBOOT_SETTLE_MS));
    let rehomed = b.status_ok(&build_assign(&uid, id));
    let restored = b
        .status_ok(&build_read(id, RESPONSE_DEADLINE_US, 2))
        .payload;
    b.status_ok_within(&build_save(id), SAVE_SETTLE_MS);

    assert_eq!(dirty_before, 1, "config write marks modified-since-save");
    assert_eq!(dirty_after, 0, "SAVE clears the dirty bit");
    assert_eq!(
        persisted,
        MARKER.to_le_bytes(),
        "marker survived the reboot"
    );
    assert_eq!(rehomed.id, id, "post-factory re-home acks from the id");
    assert_ne!(restored, MARKER.to_le_bytes(), "factory dropped the image");
}
