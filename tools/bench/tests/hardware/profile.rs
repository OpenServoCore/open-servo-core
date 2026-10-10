use bench::SUPPORTED_BAUDS;
use bench::osc::{build_profile_config, build_read, build_read_profile};
use bench::run::Stats;
use osc_protocol::wire::ResultCode;
use osc_servo_core::regions::config::addr::common::ID;
use osc_servo_core::regions::config::addr::common::MODEL_NUMBER;
use osc_servo_core::regions::telemetry::addr::sensors::{ENC_A, POS, VMOTOR_A};
use serial_test::serial;

use crate::support::bench;

/// The turnaround scatter: pos+current(4) + vmotor_a(2) + enc_a(2) from the
/// raw telemetry block -- the protocol sec 5.2 cyclic-telemetry shape.
const SCATTER: [(u16, u8); 3] = [(POS, 4), (VMOTOR_A, 2), (ENC_A, 2)];

/// A gathered reply is byte-identical to the concatenation of plain READs of
/// the same spans (odd interior span included -- no parity constraint, protocol sec 5.2).
#[serial]
#[test]
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

/// Unconfigured and out-of-range slots reject `range` (protocol sec 5.3).
#[serial]
#[test]
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

/// Per-baud ceiling for the mean profile-read turnaround (us), 3 spans / 8 B:
/// the two extra snapshot arms and the span resolution add ~10-16 us over a
/// contiguous READ of the same bytes. Measured means with the kernel above
/// the bus: 72.4/87.4/89.0/93.1 ascending baud. Each ceiling sits ~8 us above
/// (the turnaround-budget convention: layout swing plus kernel-phase spread).
fn profile_turnaround_budget_us(baud: u32) -> f64 {
    match baud {
        500_000 => 80.0,
        1_000_000 => 95.0,
        2_000_000 => 97.0,
        3_000_000 => 101.0,
        _ => 105.0,
    }
}

/// Turnaround for the scattered-telemetry profile read, swept across the baud
/// matrix; the distribution at each baud is printed for the record.
#[serial]
#[test]
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
