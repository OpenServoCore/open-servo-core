//! HSI auto-trim convergence — drive Pings under ±2% simulated factory
//! drift and assert the driver-side integrator brings the chip clock back
//! toward nominal within a fixed ping budget. All-baud sweep validates
//! the magnitude-aware control law across the SNR spectrum: at 9600 the
//! per-step drift signal dwarfs sampling noise; at 3M baud `ticks_per_bit`
//! is only 16 ticks, so a single chip-stamp quantization tick is 6% of a
//! bit — the test pins whether the integrator's byte-tick threshold
//! survives that noise floor.
//!
//! Two-phase control law:
//! - **Boot** (until first batch close after `Clock::new`): 6-sample
//!   batch with full-step deadband and a 16-step emit cap. The first
//!   Ping reply supplies the 6 byte pairs; one emit lands the correction
//!   inside the ±20 000 ppm envelope so non-Fast commands work right
//!   after init.
//! - **Steady** (every subsequent batch): 20-sample batch with
//!   half-step deadband and a 4-step emit cap. Squeezes the residual
//!   gap before the host issues Fast commands.
//!
//! The driver's `ClockTrim` trait uses chip-blind ppm units; the sim's
//! `HsiClock` carries factory drift as a `(num, den)` ratio set per-test
//! and composes it with the latest absolute correction drained from
//! `MockClockTrim`. No chip-side step constants leak into either side.

use crate::support::{Setup, baud_matrix, setup_with};
use dxl_protocol::types::Id;
use osc_core::BaudRate;
use osc_integration::sim::DEFAULT_RDT_US;
use rstest::rstest;
use rstest_reuse::apply;

/// Pings to drive per test. The boot phase closes one batch on Ping 1
/// (6 byte pairs from the reply). Steady phase closes every ~3.3 pings
/// (20 / 6). 64 pings leaves ~19 steady batches of headroom over the
/// first-emit landing — enough margin to surface any regression in the
/// steady-phase deadband or residual squeeze.
const PING_BUDGET: u32 = 64;

const TARGET: Id = Id::new(1);

/// At nominal factory HSI the integrator must emit zero corrections at
/// every baud. Any spurious emit signals a threshold-compute bug — most
/// likely 1-tick chip-stamp quantization at high baud crossing the
/// half-step boundary, or a precomputed-reciprocal sign error at low baud.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_at_nominal_emits_no_trim_ops(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    let ops = sim.servo(servos[0]).trim_ops();
    assert!(
        ops.is_empty(),
        "at nominal HSI expected zero trim ops, got {:?} (baud_idx={baud_idx})",
        ops,
    );
}

/// +2% factory drift (HSI runs fast). Driver should drive the applied
/// correction negative to slow the chip back toward nominal. Asserts both
/// direction and magnitude — a final correction smaller than ~75% of the
/// drift would mean the loop got stuck (rail clipping, threshold too
/// tight, etc.) without fully cancelling.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_hot_corner_converges(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    // +2% drift = +20_000 ppm.
    sim.servo_mut(servos[0]).set_hsi_drift(1, 50);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    let ops = sim.servo(servos[0]).trim_ops();
    let final_ppm = *ops.last().expect("expected at least one trim op");

    assert!(
        final_ppm <= -15_000,
        "correction too small for +2% drift: {final_ppm} (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!("baud_idx={baud_idx} +2% drift → final_ppm={final_ppm}, ops={ops:?}",);
}

/// -2% factory drift (HSI runs slow). Symmetric to the hot-corner case.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_cold_corner_converges(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    // -2% drift = -20_000 ppm.
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 50);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    let ops = sim.servo(servos[0]).trim_ops();
    let final_ppm = *ops.last().expect("expected at least one trim op");

    assert!(
        final_ppm >= 15_000,
        "correction too small for -2% drift: {final_ppm} (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!("baud_idx={baud_idx} -2% drift → final_ppm={final_ppm}, ops={ops:?}",);
}

/// Live HCLK ppm offset from nominal. Composes the servo's factory-drift
/// ratio with the latest absolute correction the driver applied — same
/// quantity the chip would carry on real silicon.
fn live_residual_ppm(sim: &osc_integration::sim::Sim, servo: osc_integration::sim::DeviceId) -> i32 {
    let live_hz = sim.servo(servo).clock().freq_hz() as i64;
    let nominal_hz = sim.servo(servo).hsi_clock().nominal().freq_hz() as i64;
    ((live_hz - nominal_hz) * 1_000_000 / nominal_hz) as i32
}

/// +2% factory drift, one Ping. Boot phase closes a 6-sample batch
/// during the first instruction body and the magnitude-aware estimator
/// emits exactly the opposing nudge: -20 000 ppm = 8 steps × 2500 ppm.
/// The post-apply residual is `(1.02 × 0.98 - 1) × 10⁶ ≈ -400 ppm` —
/// inside the steady-phase ½-step deadband (~1250 ppm) so the integrator
/// stops emitting. After Ping 1 the chip is effectively at nominal
/// HCLK; non-Fast commands work immediately.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_hot_corner_lands_in_one_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(1, 50);

    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });

    let ops = sim.servo(servos[0]).trim_ops();
    assert_eq!(
        ops.len(),
        1,
        "expected exactly one emit after first Ping (baud_idx={baud_idx}, ops={ops:?})",
    );
    assert_eq!(
        ops[0], -20_000,
        "boot emit at +2% drift should be exactly -20_000 ppm \
         (8 steps × 2500 ppm/step); got {} (baud_idx={baud_idx})",
        ops[0],
    );
    let residual_ppm = live_residual_ppm(&sim, servos[0]);
    assert!(
        residual_ppm.abs() < 1000,
        "expected |residual_ppm| < 1000 after 1-ping boot landing, \
         got {residual_ppm} (baud_idx={baud_idx})",
    );
    log::info!(
        "baud_idx={baud_idx} +2% drift, 1 ping → emit={} residual_ppm={residual_ppm}",
        ops[0],
    );
}

/// -2% factory drift, one Ping. Symmetric to the hot-corner one-Ping case.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_cold_corner_lands_in_one_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 50);

    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });

    let ops = sim.servo(servos[0]).trim_ops();
    assert_eq!(
        ops.len(),
        1,
        "expected exactly one emit after first Ping (baud_idx={baud_idx}, ops={ops:?})",
    );
    assert_eq!(
        ops[0], 20_000,
        "boot emit at -2% drift should be exactly +20_000 ppm \
         (8 steps × 2500 ppm/step); got {} (baud_idx={baud_idx})",
        ops[0],
    );
    let residual_ppm = live_residual_ppm(&sim, servos[0]);
    assert!(
        residual_ppm.abs() < 1000,
        "expected |residual_ppm| < 1000 after 1-ping boot landing, \
         got {residual_ppm} (baud_idx={baud_idx})",
    );
    log::info!(
        "baud_idx={baud_idx} -2% drift, 1 ping → emit={} residual_ppm={residual_ppm}",
        ops[0],
    );
}

/// Steady-phase tracks a mid-operation drift shift. Boot at nominal HSI
/// (Ping 1 closes the boot batch with no emit). Then a +0.5% drift
/// appears — models the chip warming up under load — and the steady
/// integrator must squeeze it back into the deadband within a handful of
/// pings. Steady batch closes every ~5.3 pings (32 / 6); 16 pings give
/// ~3 steady batches of headroom. At 3M (worst SNR) +0.5% drift produces
/// `drift_sum_q8 / drift_per_step_q8 ≈ 2 steps` → emit -5 000 ppm,
/// residual `(1.005 × 0.995 - 1) × 10⁶ ≈ -25 ppm`.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_steady_phase_tracks_dynamic_drift(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);

    // Ping 1 at nominal closes the boot batch; no correction needed.
    // The boot→steady transition fires unconditionally on this TC.
    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });
    let ops_after_boot = sim.servo(servos[0]).trim_ops();

    // Temperature shift: chip warms up, HSI drifts +0.5% (~50 °C swing
    // at the V006's ~0.05%/°C tempco). `set_hsi_drift` doesn't touch
    // the applied correction — only the live HCLK.
    sim.servo_mut(servos[0]).set_hsi_drift(1, 200);

    for _ in 0..16 {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }

    let residual_ppm = live_residual_ppm(&sim, servos[0]);
    let ops = sim.servo(servos[0]).trim_ops();
    assert!(
        residual_ppm.abs() < 1000,
        "steady phase failed to converge: residual_ppm={residual_ppm} \
         after 16 pings under +0.5% temp shift \
         (baud_idx={baud_idx}, ops_after_boot={ops_after_boot:?}, ops={ops:?})",
    );
    assert!(
        ops.len() > ops_after_boot.len(),
        "steady phase emitted nothing under +0.5% drift \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!(
        "baud_idx={baud_idx} dynamic +0.5% drift after 16 pings → \
         residual_ppm={residual_ppm} ops={ops:?}",
    );
}
