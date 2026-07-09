//! Zero-gap hot-loop survival on silicon — the bench twin of
//! `firmware/lib/integration/tests/hot_loop.rs`. The DES sim proves the LOGICAL
//! zero-gap contract with zero-cost handlers; it cannot model the ISR-latency
//! window where frame N's end-of-frame work races frame N+1's break. This test
//! bombards the real chip with the production hot loop and asserts that window
//! stays closed:
//!
//! * `stale == 0` — a stale GREAD read-back means a GWRITE or COMMIT was
//!   silently dropped by the framer. Zero tolerance: it is a correctness defect,
//!   not a wire artifact.
//! * failure rate under budget — a missed GREAD reply is a wire/RX artifact
//!   (the reply-merge class the `tool-reply-edges` forensic drills into);
//!   budgeted, not zero, so a real regression trips it without flaking.
//!
//! Turnaround is reported for the record but NOT gated here: the burst GREAD
//! reply latency folds in the GWRITE+COMMIT work (goal_position's soft-limit
//! rules, the representative worst case) plus the ISR-preemption tail, so it is
//! a different metric from the ping T_turn budget the `turnaround` suite owns.
//!
//! `BENCH_BURST_CYCLES` overrides the cycle count (default 2000) for a longer
//! soak. goal_position is restored afterwards (state discipline).

use std::env;

use bench::osc::{build_read, build_write};
use bench::run::{BurstReport, Stats, hot_loop_cycle};
use osc_core::regions::control::addr::lifecycle::GOAL_POSITION;
use serial_test::serial;

use crate::support::bench;

/// Failure-rate ceiling (fraction of cycles) for the 3-frame production loop.
/// Above the observed 1 M reply-artifact floor (~5e-4) with margin, below any
/// real framer regression.
const FAILURE_BUDGET: f64 = 0.003;

/// Cycles per run; `BENCH_BURST_CYCLES` overrides for a longer soak.
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

/// The production hot loop `[GWRITE(HOLD), COMMIT, GREAD]` sent zero-gap: the
/// GREAD must already read back the just-committed value on every cycle.
#[test]
#[serial]
fn hot_loop_zero_gap_survives() {
    let mut b = bench();
    let id = b.id();
    let n = cycles();

    let orig = b.status_ok(&build_read(id, GOAL_POSITION, 4)).payload;
    let report = b.burst(n, |v| hot_loop_cycle(id, GOAL_POSITION, v));
    b.status_ok(&build_write(id, GOAL_POSITION, &orig));

    report_summary("hot loop", &report);

    assert_eq!(
        report.stale, 0,
        "{} stale read-backs — a GWRITE or COMMIT was silently dropped",
        report.stale
    );
    let rate = report.failures() as f64 / report.cycles() as f64;
    assert!(
        rate < FAILURE_BUDGET,
        "failure rate {:.3}% over budget ({} of {} cycles)",
        rate * 100.0,
        report.failures(),
        report.cycles(),
    );
}
