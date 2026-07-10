use bench::SUPPORTED_BAUDS;
use bench::osc::build_ping;
use bench::run::Stats;
use serial_test::serial;

use crate::support::bench;

/// Per-baud ceiling for the mean ping turnaround (µs). Ring-cadence timing +
/// the fixed-µs reply gap flattened the low side (measured ~34/36 µs at 1M/500k,
/// 2026-07-09); 2M/3M remain pipeline-bound (~41/44 µs — the covered-overlap
/// window shrinks below the dispatch body, the documented follow-up band).
/// Each ceiling sits ~6 µs above the measured floor: tight enough to catch a
/// regression from the current baseline, loose enough for the ±5 µs
/// flash-layout swing.
fn turnaround_budget_us(baud: u32) -> f64 {
    match baud {
        1_000_000 => 40.0,
        2_000_000 => 47.0,
        3_000_000 => 50.0,
        500_000 => 42.0,
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
