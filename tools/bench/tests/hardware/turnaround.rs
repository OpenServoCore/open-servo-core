use bench::SUPPORTED_BAUDS;
use bench::osc::build_ping;
use bench::run::Stats;
use serial_test::serial;

use crate::support::bench;

/// Per-baud ceiling for the mean ping turnaround (µs). Ring-cadence timing,
/// the fixed-µs reply gap, and the in-place chain trigger put the floor at
/// ~30/32 µs (1M/500k) and ~39/41 µs (2M/3M, pipeline-bound — the
/// covered-overlap window shrinks below the dispatch body; RAM placement
/// probed and rejected, see the transport pillar). Each ceiling sits ~6 µs
/// above the measured floor: tight enough to catch a regression from the
/// current baseline, loose enough for the ±5 µs flash-layout swing.
fn turnaround_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 36.0,
        2_000_000 => 45.0,
        3_000_000 => 47.0,
        500_000 => 38.0,
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
