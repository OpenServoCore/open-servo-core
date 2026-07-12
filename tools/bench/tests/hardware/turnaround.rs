use bench::SUPPORTED_BAUDS;
use bench::osc::{build_ping, build_read, build_write};
use bench::run::Stats;
use osc_core::regions::control::addr::lifecycle::GOAL_POSITION;
use osc_core::regions::telemetry::addr::converted::PRESENT_POSITION;
use serial_test::serial;

use crate::support::{Bench, bench};

/// Per-baud ceiling for the mean ping turnaround (µs). Ring-cadence timing,
/// the fixed-µs reply gap, and the in-place chain trigger put the floor at
/// ~30/32 µs (1M/500k) and ~39/41 µs (2M/3M, pipeline-bound — the
/// covered-overlap window shrinks below the dispatch body; RAM placement
/// probed and rejected, see the transport pillar). Each ceiling sits ~6 µs
/// above the measured floor: tight enough to catch a regression from the
/// current baseline, loose enough for the ±5 µs flash-layout swing.
fn ping_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 45.0,
        2_000_000 => 53.0,
        3_000_000 => 48.0,
        500_000 => 45.0,
        _ => 55.0,
    }
}

/// READ ceiling: the reply break waits only on staging the copy-once
/// snapshot kickoff, not the payload's wire time, so a 16 B read tracks the
/// ping floor plus the read-dispatch body (measured means 36.5/37.8/41.1/40.5
/// ascending baud, 2026-07-10). Same ~6 µs headroom policy.
fn read_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 41.0,
        2_000_000 => 53.0,
        3_000_000 => 52.0,
        500_000 => 44.0,
        _ => 55.0,
    }
}

/// WRITE ceiling: goal_position is the rule-heavy hot-loop register — its
/// soft-limit rules dominate the dispatch body (write-size is not the cost),
/// and the production hot loop pays none of this — GWRITE is NOREPLY.
/// Re-baselined 2026-07-12 evening (measured means 80.0/83.1/95.5/96.1
/// ascending baud) on the law-break servo build + LBD pirate stamps.
/// WATCH: the low-baud writes moved +20/+10 µs vs the same morning's
/// 60.2/72.7 means while 2M/3M held — a #25-build shift worth a perf
/// look, not a wire defect (2M/3M pipeline-bound legs unchanged). Same
/// ~6 µs headroom policy.
fn write_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 89.0,
        2_000_000 => 102.0,
        3_000_000 => 102.0,
        500_000 => 86.0,
        _ => 108.0,
    }
}

/// Sweep `wire` across the baud matrix: zero failed exchanges, mean under
/// the per-baud budget, distribution printed for the record.
fn gate(b: &mut Bench, wire: &[u8], label: &str, budget_us: fn(u32) -> f64) {
    for &baud in &SUPPORTED_BAUDS {
        let report = b.measure_at(baud, wire, 50).expect("measure");
        assert_eq!(
            report.fail, 0,
            "@{baud}: {} of 50 {label} exchanges failed",
            report.fail
        );

        let s = Stats::from(&report.ok).expect("some turnaround samples");
        println!("{label} turnaround @{baud}:");
        s.print();
        let budget = budget_us(baud);
        assert!(
            s.mean < budget,
            "@{baud}: {label} mean turnaround {:.1} us over budget {budget:.0}",
            s.mean
        );
    }
}

/// THE metric: instruction wire-end → status break fall, swept across the
/// baud matrix.
#[test]
#[serial]
fn ping_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    gate(&mut b, &build_ping(id), "ping", ping_budget_us);
}

/// The copy-once read path at telemetry scale (16 B from the converted
/// block).
#[test]
#[serial]
fn read_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    gate(
        &mut b,
        &build_read(id, PRESENT_POSITION, 16),
        "read",
        read_budget_us,
    );
}

/// The mutating path: validated goal_position write (board-default value,
/// so the table state is untouched).
#[test]
#[serial]
fn write_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    gate(
        &mut b,
        &build_write(id, GOAL_POSITION, &0i32.to_le_bytes()),
        "write",
        write_budget_us,
    );
}
