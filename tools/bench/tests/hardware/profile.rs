use bench::SUPPORTED_BAUDS;
use bench::osc::{build_profile_config, build_read, build_read_profile};
use bench::run::Stats;
use osc_core::regions::config::addr::comms::ID;
use osc_core::regions::config::addr::identity::MODEL_NUMBER;
use osc_core::regions::telemetry::addr::converted::{
    PRESENT_CURRENT, PRESENT_POSITION, PRESENT_VBUS_MV,
};
use osc_protocol::wire::ResultCode;
use serial_test::serial;

use crate::support::bench;

/// The turnaround scatter: position(4) + current(2) + vbus(2) from the
/// converted telemetry block — the §5.2 cyclic-telemetry shape.
const SCATTER: [(u16, u8); 3] = [
    (PRESENT_POSITION, 4),
    (PRESENT_CURRENT, 2),
    (PRESENT_VBUS_MV, 2),
];

/// A gathered reply is byte-identical to the concatenation of plain READs of
/// the same spans (odd interior span included — no parity constraint, §5.2).
#[test]
#[serial]
fn profile_read_matches_plain_reads() {
    let mut b = bench();
    let id = b.id();

    b.status_ok(&build_profile_config(id, 0, &[(MODEL_NUMBER, 2), (ID, 1)]));
    let gathered = b.status_ok(&build_read_profile(id, 0)).payload;

    let mut want = b.status_ok(&build_read(id, MODEL_NUMBER, 2)).payload;
    want.extend_from_slice(&b.status_ok(&build_read(id, ID, 1)).payload);
    assert_eq!(gathered, want, "gathered == concat of plain reads");

    // Restore: disable the slot.
    b.status_ok(&build_profile_config(id, 0, &[]));
}

/// Unconfigured and out-of-range slots reject `range` (§5.3).
#[test]
#[serial]
fn profile_read_bad_slot_rejects_range() {
    let mut b = bench();
    let id = b.id();

    for slot in [1u8, 9] {
        let ex = b.xfer(&build_read_profile(id, slot)).expect("exchange");
        assert_eq!(
            ex.status.result,
            Some(ResultCode::Range),
            "slot {slot}: expected range"
        );
    }
}

/// Per-baud ceiling for the mean profile-read turnaround (µs). Measured floor
/// (2026-07-10 build, 3 spans / 8 B): 39.9 / 48.2 / 46.6 / 50.5 at
/// 0.5M/1M/2M/3M — the two extra snapshot arms and the span resolution add
/// ~4-11 µs over the same bytes as one contiguous READ. Each ceiling sits
/// ~6 µs above the measured floor (the ping-budget convention: regression
/// margin plus the ±5 µs flash-layout swing).
/// Re-baselined 2026-07-13 (measured means 41.6/46.9/55.8/60.0 ascending
/// baud) on the enum-slot fleet build.
fn profile_turnaround_budget_us(baud: u32) -> f64 {
    match baud {
        500_000 => 48.0,
        1_000_000 => 53.0,
        2_000_000 => 62.0,
        3_000_000 => 66.0,
        _ => 70.0,
    }
}

/// Turnaround for the scattered-telemetry profile read, swept across the baud
/// matrix; the distribution at each baud is printed for the record. The 2M
/// leg is the regression gate for the pirate's DATAR discipline (task #7:
/// an IDLE-clear DATAR read ate the reply's final byte 1/128 exchanges).
#[test]
#[serial]
fn profile_read_turnaround_within_budget() {
    let mut b = bench();
    let id = b.id();

    b.status_ok(&build_profile_config(id, 0, &SCATTER));
    for &baud in &SUPPORTED_BAUDS {
        let report = b
            .measure_at(baud, &build_read_profile(id, 0), 50)
            .expect("measure");
        assert_eq!(
            report.fail, 0,
            "@{baud}: {} of 50 profile reads failed",
            report.fail
        );

        let s = Stats::from(&report.ok).expect("some turnaround samples");
        println!("profile-read (3 spans, 8 B) turnaround @{baud}:");
        s.print();
        let budget = profile_turnaround_budget_us(baud);
        assert!(
            s.mean < budget,
            "@{baud}: mean turnaround {:.1} us over budget {budget:.0}",
            s.mean
        );
    }
    b.status_ok(&build_profile_config(id, 0, &[]));
}
