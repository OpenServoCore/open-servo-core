//! protocol sec 9.1 rescue break on silicon: a dominant low reaches a servo at ANY
//! configured baud and drops it to the rescue rate; reboot exits back to
//! the configured baud. Plus the full field-recovery flow through rescue.
//!
//! Both tests end with an UNCONDITIONAL rescue-based recovery tail (the
//! pulse reaches the servo wherever a lost ack may have left it), and all
//! asserts run after it -- a transient pirate capture dropout must never
//! strand the bench unit off its id, baud, or with a stray saved image.

use std::thread::sleep;
use std::time::Duration;

use bench::osc::{
    REBOOT_SETTLE_MS, SAVE_SETTLE_MS, build_assign, build_factory, build_ping, build_reboot,
    build_save, build_write,
};
use bench::{BOOT_BAUD, baud_index};
use osc_core::regions::config::addr::comms::BAUD_RATE_IDX;
use osc_protocol::wire::{Id, ResultCode};
use serial_test::serial;

use crate::support::bench;

/// The pulse is baud-agnostic: servo moved to 3M (volatile) with the pirate
/// back at the boot baud -- unreachable -- then one rescue pulse unifies the
/// bus at 0.5M. Reboot restores the configured (boot-default) baud.
#[test]
#[serial]
fn rescue_reaches_a_servo_at_any_baud_and_reboot_exits() {
    let mut b = bench();
    let id = b.id();
    let ping = build_ping(id);

    // Move the servo to 3M by raw exchange (not the panicking harness
    // helper): the outcome is collected so the recovery tail always runs.
    let to_3m = build_write(id, BAUD_RATE_IDX, &[baud_index(3_000_000).unwrap()]);
    let moved = b.xfer(&to_3m).map(|ex| ex.status.result);
    let lost = b.xfer(&ping).is_err();

    b.rescue_pulse();
    let rescued = b.xfer(&ping).map(|ex| ex.status.result);

    // Recovery tail: pulse again (idempotent on a rescue-parked servo) so
    // the reboot reaches it regardless of what succeeded above -- then a
    // broadcast REBOOT (silent, takes immediately, protocol sec 9.5) releases every
    // OTHER servo the bus-wide pulses parked at 0.5M; the fleet exits
    // rescue together and the bus leaves as found.
    b.rescue_pulse();
    let rebooted = b.xfer(&build_reboot(id)).map(|ex| ex.status.result);
    b.expect_no_reply(&build_reboot(Id::BROADCAST.as_byte()));
    sleep(Duration::from_millis(REBOOT_SETTLE_MS));
    b.follow_baud(BOOT_BAUD);
    let back = b.xfer(&ping).map(|ex| ex.status.result);

    assert_eq!(moved.ok().flatten(), Some(ResultCode::Ok), "baud write");
    assert!(lost, "a servo at 3M must not answer a boot-baud host");
    assert_eq!(
        rescued.ok().flatten(),
        Some(ResultCode::Ok),
        "the pulse reached the 3M servo and dropped it to 0.5M"
    );
    assert_eq!(rebooted.ok().flatten(), Some(ResultCode::Ok));
    assert_eq!(
        back.ok().flatten(),
        Some(ResultCode::Ok),
        "reboot exits rescue to the configured baud"
    );
}

/// The field-recovery story end to end: rescue pulse -> prefix-walk finds the
/// UID at 0.5M -> broadcast ASSIGN moves the id -> SAVE persists it -> REBOOT
/// exits rescue and the new id answers at the configured baud. The restore
/// tail then rescues again and FACTORYs the servo carrying OUR UID --
/// wherever a partial failure left its id or a stray saved image -- then
/// re-homes it by UID and re-persists. Fleet-safe throughout: the walk may
/// find a whole chain (only the entry at our configured id is the DUT),
/// every mutation is UID-scoped, and a silent broadcast REBOOT releases
/// the servos the bus-wide pulses parked at 0.5M.
#[test]
#[serial]
fn rescue_walk_assign_save_survives_reboot_until_factory() {
    let mut b = bench();
    let home = b.id();
    let away = if home == 42 { 43 } else { 42 };

    b.rescue_pulse();
    let found = b.walk();
    let dut = found.iter().find(|f| f.id == home);
    let walk_hit = dut.is_some();
    let uid = dut.map(|f| f.uid).unwrap_or_default();

    let assigned = b.xfer(&build_assign(&uid, away)).map(|ex| ex.status.id);
    let saved = b
        .xfer_within(&build_save(away), SAVE_SETTLE_MS)
        .map(|ex| ex.status.result);

    let _ = b.xfer(&build_reboot(away));
    sleep(Duration::from_millis(REBOOT_SETTLE_MS));
    b.follow_baud(BOOT_BAUD);
    let away_alive = b.xfer(&build_ping(away)).map(|ex| ex.status.result);

    // Restore tail: rescue-based, so it targets the state actually on the
    // bus rather than what the happy path predicts -- but UID-scoped: only
    // the servo carrying OUR uid is factory'd (a fleet's other servos are
    // bystanders). The broadcast REBOOT then releases everyone else from
    // rescue, and the factory'd DUT -- booted at the board-default id,
    // which may collide with a live fleet id -- is re-homed by UID before
    // any id-addressed exchange, then re-persisted.
    b.rescue_pulse();
    let stranded = b.walk();
    if let Some(f) = stranded.iter().find(|f| f.uid == uid) {
        let _ = b.xfer_within(&build_factory(f.id), SAVE_SETTLE_MS);
    }
    b.expect_no_reply(&build_reboot(Id::BROADCAST.as_byte()));
    sleep(Duration::from_millis(REBOOT_SETTLE_MS));
    b.follow_baud(BOOT_BAUD);
    let rehomed = b.xfer(&build_assign(&uid, home)).map(|ex| ex.status.id);
    let _ = b.xfer_within(&build_save(home), SAVE_SETTLE_MS);
    let home_back = b.xfer(&build_ping(home)).map(|ex| ex.status.result);

    assert!(
        walk_hit,
        "the walk found the bench servo id {home}: {found:?}"
    );
    assert_eq!(rehomed.ok(), Some(home), "post-factory re-home by UID");
    assert_eq!(assigned.ok(), Some(away), "ASSIGN acks from the new id");
    assert_eq!(saved.ok().flatten(), Some(ResultCode::Ok), "SAVE at 0.5M");
    assert_eq!(
        away_alive.ok().flatten(),
        Some(ResultCode::Ok),
        "the rescued-and-assigned id persisted the reboot at the boot baud"
    );
    assert_eq!(
        home_back.ok().flatten(),
        Some(ResultCode::Ok),
        "the restore tail brought the board-default id back"
    );
}
