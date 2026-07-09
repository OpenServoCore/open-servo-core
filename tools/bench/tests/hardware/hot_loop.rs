//! Zero-gap burst survival on silicon, across the baud matrix — the bench twin
//! of `firmware/lib/integration/tests/hot_loop.rs`. The DES sim proves the
//! LOGICAL zero-gap contract with zero-cost handlers; it cannot model the
//! ISR-latency window where frame N's end-of-frame work races frame N+1's break.
//! These tests bombard the real chip at every wire baud and assert the total
//! failure rate stays under a per-scenario budget. The report prints the
//! breakdown — `stale` (a GWRITE/COMMIT/WRITE the framer silently dropped)
//! vs `no-reply`/`other` (reply-merge wire artifacts, the class
//! `tool-reply-edges` drills into) — but the gate is the combined rate:
//! the framer has a known low-baud frame-loss floor (task #32, OPEN) that leaves
//! a rare stale even on the 3-frame production loop at 500k/1M, so asserting
//! `stale == 0` would flake. The budget catches a gross regression from the
//! current floor; it tightens toward zero when #32 lands. At 2M/3M the printed
//! stale is ~0 in practice.
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

/// Total-failure-rate ceiling for the production hot loop (fraction of cycles).
/// The 3-frame loop's floor is tiny — reply artifacts ≤2/3000 at 1M plus a rare
/// low-baud stale (~1/2000 at 500k, task #32) — so 0.3% catches a regression
/// with several-x margin without flaking.
const HOT_LOOP_BUDGET: f64 = 0.003;

/// Total-failure-rate ceiling for the aggressive 8-write plain flood. Its
/// low-baud frame-loss floor is heavier (worst ~0.33% at 1M — task #32), so 1%
/// documents that floor and catches a gross regression without flaking.
const FLOOD_BUDGET: f64 = 0.01;

fn cycles() -> u32 {
    env::var("BENCH_BURST_CYCLES")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(2000)
}

fn report_summary(label: &str, report: &BurstReport) {
    println!(
        "{label}: {} ok, {} stale, {} no-reply, {} other  (of {} cycles)",
        report.ok.len(),
        report.stale,
        report.no_reply,
        report.other,
        report.cycles(),
    );
    if let Some(s) = Stats::from(&report.ok) {
        s.print();
    }
}

fn assert_rate(label: &str, baud: u32, report: &BurstReport, budget: f64) {
    let rate = report.failures() as f64 / report.cycles() as f64;
    assert!(
        rate < budget,
        "{label} @{baud}: failure rate {:.3}% over budget ({} of {})",
        rate * 100.0,
        report.failures(),
        report.cycles(),
    );
}

/// The production hot loop `[GWRITE(HOLD), COMMIT, GREAD]` sent zero-gap at every
/// wire baud: the GREAD must already read back the just-committed value.
#[test]
#[serial]
fn hot_loop_zero_gap_survives() {
    let mut b = bench();
    let id = b.id();
    let n = cycles();

    let orig = b.status_ok(&build_read(id, GOAL_POSITION, 4)).payload;
    for &baud in &SUPPORTED_BAUDS {
        let report = b.burst_at(baud, n, |v| hot_loop_cycle(id, GOAL_POSITION, v));
        report_summary(&format!("hot loop @{baud}"), &report);
        assert_rate("hot loop", baud, &report, HOT_LOOP_BUDGET);
    }
    b.status_ok(&build_write(id, GOAL_POSITION, &orig));
}

/// A plain `[WRITE(NOREPLY) × 8, READ]` flood at every wire baud: no per-write
/// reply paces the framer, so the READ read-back guards every silent write. This
/// is the aggressive stress that surfaces the low-baud framer floor (task #32);
/// its budget documents that floor rather than asserting it away.
#[test]
#[serial]
fn plain_write_flood_survives() {
    let mut b = bench();
    let id = b.id();
    let n = cycles();

    let orig = b.status_ok(&build_read(id, GOAL_POSITION, 4)).payload;
    for &baud in &SUPPORTED_BAUDS {
        let report = b.burst_at(baud, n, |v| plain_flood_cycle(id, GOAL_POSITION, 8, v));
        report_summary(&format!("plain flood @{baud}"), &report);
        assert_rate("plain flood", baud, &report, FLOOD_BUDGET);
    }
    b.status_ok(&build_write(id, GOAL_POSITION, &orig));
}
