use bench::SUPPORTED_BAUDS;
use bench::osc::build_ping;
use bench::run::Stats;
use serial_test::serial;

use crate::support::bench;

/// Per-baud ceiling for the mean ping turnaround (µs). Turnaround is NOT baud-
/// flat: 1 M is the tuned boot baud (measured ~35 µs, held at the 45 µs gate
/// with ±5 µs flash-swing headroom), and the metric rises at both higher and
/// lower baud (measured ~47/43/46 µs at 500k/2M/3M on this build). Each ceiling
/// sits ~6 µs above the measured floor: tight enough to catch a regression from
/// the current baseline, loose enough not to flake on the baud-dependent floor.
fn turnaround_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 45.0,
        2_000_000 => 50.0,
        3_000_000 => 52.0,
        500_000 => 53.0,
        _ => 55.0,
    }
}

/// THE metric: instruction wire-end → status break fall, swept across the baud
/// matrix. The distribution at each baud is printed for the record.
#[test]
#[serial]
fn ping_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();

    for &baud in &SUPPORTED_BAUDS {
        let report = b.measure_at(baud, &build_ping(id), 50).expect("measure");
        assert_eq!(
            report.fail, 0,
            "@{baud}: {} of 50 ping exchanges failed",
            report.fail
        );

        let s = Stats::from(&report.ok).expect("some turnaround samples");
        println!("ping turnaround @{baud}:");
        s.print();
        let budget = turnaround_budget_us(baud);
        assert!(
            s.mean < budget,
            "@{baud}: mean turnaround {:.1} us over budget {budget:.0}",
            s.mean
        );
    }
}
