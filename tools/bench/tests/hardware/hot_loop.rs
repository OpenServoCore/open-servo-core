//! Zero-gap burst survival on silicon, across the baud matrix — the bench twin
//! of `firmware/lib/integration/tests/hot_loop.rs`. The DES sim proves the
//! LOGICAL zero-gap contract with zero-cost handlers; it cannot model the
//! ISR-latency window where frame N's end-of-frame work races frame N+1's break.
//! These tests bombard the real chip at every wire baud and assert the burst is
//! PERFECTLY clean — zero stale read-backs (a silently-dropped GWRITE/COMMIT/
//! WRITE) and zero missed/malformed replies.
//!
//! The framer still has an intermittent low-baud glitch: a dropped or late frame
//! that a second pass would recover (an unidentified bug, tracked as a separate
//! task). We deliberately do NOT budget around it — a run that hits it FAILS
//! here, by design, so the bench stays an honest reproducer instead of a
//! tolerance that hides the bug. Each baud is measured and printed before the
//! verdict, so a red run names exactly where and how it glitched.
//!
//! Turnaround is reported for the record but NOT gated here — the burst reply
//! latency folds in GWRITE+COMMIT work plus the ISR tail, a different metric
//! from the ping T_turn budget the `turnaround` suite owns.
//!
//! `BENCH_BURST_CYCLES` overrides the per-baud cycle count (default 2000).

use std::env;

use bench::SUPPORTED_BAUDS;
use bench::osc::{build_read, build_write};
use bench::run::{BurstReport, Stats, hot_loop_cycle, plain_flood_cycle};
use osc_core::regions::control::addr::lifecycle::GOAL_POSITION;
use serial_test::serial;

use crate::support::bench;

fn cycles() -> u32 {
    env::var("BENCH_BURST_CYCLES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(2000)
}

/// Print the tally + turnaround for `baud`; push a description if it glitched.
fn check(label: &str, baud: u32, report: &BurstReport, dirty: &mut Vec<String>) {
    println!(
        "{label} @{baud}: {} ok, {} stale, {} no-reply, {} other  (of {} cycles)",
        report.ok.len(),
        report.stale,
        report.no_reply,
        report.other,
        report.cycles(),
    );
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
    if report.failures() > 0 {
        dirty.push(format!(
            "{label} @{baud}: {} stale, {} no-reply, {} other (of {})",
            report.stale,
            report.no_reply,
            report.other,
            report.cycles(),
        ));
    }
}

/// The production hot loop `[GWRITE(HOLD), COMMIT, GREAD]` sent zero-gap at every
/// wire baud: the GREAD must read back the just-committed value on every cycle.
#[test]
#[serial]
fn hot_loop_zero_gap_survives() {
    let mut b = bench();
    let id = b.id();
    let n = cycles();

    let orig = b.status_ok(&build_read(id, GOAL_POSITION, 4)).payload;
    let mut dirty = Vec::new();
    for &baud in &SUPPORTED_BAUDS {
        let report = b.burst_at(baud, n, |v| hot_loop_cycle(id, GOAL_POSITION, v));
        check("hot loop", baud, &report, &mut dirty);
    }
    b.status_ok(&build_write(id, GOAL_POSITION, &orig));

    assert!(
        dirty.is_empty(),
        "hot loop dropped frames (low-baud glitch, tracked separately):\n{}",
        dirty.join("\n")
    );
}

/// A plain `[WRITE(NOREPLY) × 8, READ]` flood at every wire baud: no per-write
/// reply paces the framer, so the READ read-back guards every silent write. The
/// aggressive stress reproduces the low-baud glitch the most readily.
#[test]
#[serial]
fn plain_write_flood_survives() {
    let mut b = bench();
    let id = b.id();
    let n = cycles();

    let orig = b.status_ok(&build_read(id, GOAL_POSITION, 4)).payload;
    let mut dirty = Vec::new();
    for &baud in &SUPPORTED_BAUDS {
        let report = b.burst_at(baud, n, |v| plain_flood_cycle(id, GOAL_POSITION, 8, v));
        check("plain flood", baud, &report, &mut dirty);
    }
    b.status_ok(&build_write(id, GOAL_POSITION, &orig));

    assert!(
        dirty.is_empty(),
        "plain flood dropped frames (low-baud glitch, tracked separately):\n{}",
        dirty.join("\n")
    );
}
