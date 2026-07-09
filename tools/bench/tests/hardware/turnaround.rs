use bench::osc::build_ping;
use bench::run::Stats;
use serial_test::serial;

use crate::support::bench;

/// THE metric: instruction wire-end → status break fall. At the 1M boot baud a
/// ping turns around in ~38 µs (`docs/osc-servo-transport.md`); assert a
/// generous ceiling so a real regression trips while the ±5 µs flash-layout
/// swing does not. The measured distribution is printed for the record.
#[test]
#[serial]
fn ping_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();

    let report = b.measure(&build_ping(id), 50).expect("measure");
    assert_eq!(
        report.fail, 0,
        "{} of 50 ping exchanges failed",
        report.fail
    );

    let s = Stats::from(&report.ok).expect("some turnaround samples");
    s.print();
    assert!(
        s.mean < 60.0,
        "mean turnaround {:.1} us over budget",
        s.mean
    );
}
