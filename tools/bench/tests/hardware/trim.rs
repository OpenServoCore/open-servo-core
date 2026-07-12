//! Clock-trim CAL on silicon (§9.3). The lying-announce train injects a
//! known clock-offset reading without touching the chip: the announce
//! declares a shorter gap than the pirate's crystal actually paces, every
//! gap reads long by the same ratio, and the servo trims as if its own
//! clock were fast. The truthful trains that follow must pull the total
//! back — the closed-loop plant-direction proof DES cannot give (the sim
//! fakes the chip adapter; silicon 2026-07-11: an inverted HSITRIM mapping
//! railed the fleet in max-step clamps while every DES trim test stayed
//! green).

use std::thread::sleep;
use std::time::Duration;

use bench::osc::{build_cal, build_read};
use osc_core::regions::telemetry::addr::clock::TRIM_STEPS;
use serial_test::serial;

use super::support::{Bench, SETTLE_MS, bench};

/// Wire gap the pirate actually paces, and the gaps per train.
const GAP_US: u16 = 400;
const GAPS: u8 = 8;
/// Announced gap for the lying train: wire runs 400, so every gap reads
/// +8 µs ≈ +20.4k ppm of "clock fast" — far past `STEPS_MAX` at any legal
/// step effect, and still inside the per-gap T/16 sanity gate.
const LIE_GAP_US: u16 = 392;
/// One window's clamp (drivers `trim::STEPS_MAX`).
const STEPS_MAX: i32 = 4;

fn read_trim(b: &mut Bench) -> i32 {
    let status = b.status_ok(&build_read(b.id(), TRIM_STEPS, 1));
    assert_eq!(status.payload.len(), 1, "trim_steps is one byte");
    status.payload[0] as i8 as i32
}

fn train(b: &mut Bench, announce_gap_us: u16) {
    b.cal_train(
        &build_cal(announce_gap_us, GAPS),
        GAP_US as u32,
        GAPS as u32 + 1,
    );
    // The decision applies in the servo main loop between frames.
    sleep(Duration::from_millis(SETTLE_MS));
}

#[test]
#[serial]
fn lying_train_trims_and_truth_pulls_back() {
    let mut b = bench();
    let start = read_trim(&mut b);
    assert!(
        start.abs() <= 8,
        "precondition: trim near center, got {start} (reboot the servo)"
    );

    train(&mut b, LIE_GAP_US);
    let lied = read_trim(&mut b);
    assert_eq!(lied - start, STEPS_MAX, "the lie clamps at +STEPS_MAX");

    train(&mut b, GAP_US);
    train(&mut b, GAP_US);
    let back = read_trim(&mut b);
    assert!(
        (back - start).abs() <= 1,
        "truth pulls the trim back: start {start}, back {back}"
    );
}
