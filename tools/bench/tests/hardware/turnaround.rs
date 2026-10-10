use bench::SUPPORTED_BAUDS;
use bench::osc::{build_ping, build_read, build_write};
use bench::run::Stats;
use osc_servo_core::regions::control::addr::lifecycle::GOAL_POSITION;
use osc_servo_core::regions::telemetry::addr::sensors::POS;
use serial_test::serial;

use crate::support::{Bench, bench};

/// Budgets for the mean turnaround (us) with the kernel above the bus: a
/// reply waits out any kernel body in flight, so the means sit about one
/// kernel body above the dispatch floor and move with where the 50 us ticks
/// land (run-to-run spread ~2.5 us). Each ceiling sits ~8 us above the
/// measured mean: the +/-5 us flash-layout swing plus that phase spread.
/// The bound that matters to a host is RESPONSE_DEADLINE (1000 us); these
/// catch a regression from the current baseline.
///
/// Ping, measured means 62.2/71.0/78.1/77.0 ascending baud.
fn ping_budget_us(baud: u32) -> f64 {
    match baud {
        500_000 => 70.0,
        1_000_000 => 79.0,
        2_000_000 => 86.0,
        3_000_000 => 85.0,
        _ => 90.0,
    }
}

/// READ: the reply break waits only on staging the snapshot's first arm, not
/// the payload's wire time, so a 16 B read tracks the ping floor plus the
/// read-dispatch body. Measured means 60.4/73.6/77.1/77.1 ascending baud.
fn read_budget_us(baud: u32) -> f64 {
    match baud {
        500_000 => 68.0,
        1_000_000 => 82.0,
        2_000_000 => 85.0,
        3_000_000 => 85.0,
        _ => 90.0,
    }
}

/// WRITE: goal_position is the rule-heavy hot-loop register; its soft-limit
/// rules dominate the dispatch body (write size is not the cost), and the
/// production hot loop pays none of this (GWRITE is NOREPLY). The commit runs
/// before the ack is sequenced, so its CPU is on the turnaround. Measured
/// means 151.0/162.6/166.5/166.7 ascending baud.
fn write_budget_us(baud: u32) -> f64 {
    match baud {
        500_000 => 159.0,
        1_000_000 => 171.0,
        2_000_000 => 175.0,
        3_000_000 => 175.0,
        _ => 180.0,
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

/// THE metric: instruction wire-end -> status break fall, swept across the
/// baud matrix.
#[serial]
#[test]
fn ping_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    gate(&mut b, &build_ping(id), "ping", ping_budget_us);
}

/// The copy-once read path at telemetry scale (16 B from the raw block).
#[serial]
#[test]
fn read_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    gate(&mut b, &build_read(id, POS, 16), "read", read_budget_us);
}

/// The mutating path: a goal_position write the rule accepts.
#[serial]
#[test]
fn write_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();
    let goal = b.goal_mid();
    gate(
        &mut b,
        &build_write(id, GOAL_POSITION, &goal.to_le_bytes()),
        "write",
        write_budget_us,
    );
}
