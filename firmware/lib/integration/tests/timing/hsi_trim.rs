//! HSI auto-trim convergence — drive traffic under ±2% simulated factory
//! drift and assert the driver-side integrator brings the chip clock back
//! toward nominal.
//!
//! The integrator samples drift from NDTR/byte-count *spans*. Long bursts
//! form drain-ISR span pairs; isolated short packets (a Ping trips a single
//! drain ISR, and the same-burst gate rejects any pair straddling an
//! inter-packet gap) are covered by the RXNE cold-start *window*, which
//! accumulates one long span per burst from per-byte wakes. Both feed the
//! same integrator, so the ping-driven convergence tests below hold. The
//! span/window mechanisms are unit-tested in `osc-drivers`
//! (`clock::drift_integrator`, `codec::span`, `dxl::uart::drift_window`);
//! the `hsi_at_nominal_emits_no_trim_ops` case here is the spurious-emit
//! guard.
//!
//! Two-phase control law:
//! - **Boot** (until the first batch close after `Clock::new`): the batch
//!   closes at the first RX packet boundary holding ≥ 1 span, with a
//!   full-step deadband and a 16-step emit cap. One emit lands the
//!   correction inside the ±20 000 ppm envelope on the first exchange.
//! - **Steady** (every subsequent batch): 20-span batch with a half-step
//!   deadband and a 4-step emit cap.
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

/// Pings to drive per test. The boot phase lands its correction on Ping 1
/// (one window span) and closes the drift window; that single landing fully
/// cancels ±2% factory drift, so the surplus pings are headroom to surface
/// any regression that would perturb an already-converged loop.
const PING_BUDGET: u32 = 64;

const TARGET: Id = Id::new(1);

/// Foreign target: nothing in the bus answers this id, so the servo at id=1
/// sees the Ping as a foreign instruction and runs the byte-skip path.
/// HSI sampling must still fire across the header+CRC bytes (host HSE clocks
/// all instructions; foreign vs. own is irrelevant to the integrator).
const FOREIGN_TARGET: Id = Id::new(99);

/// At nominal factory HSI the integrator must emit zero corrections at
/// every baud. Any spurious emit signals a threshold-compute bug — most
/// likely 1-tick chip-stamp quantization at high baud crossing the
/// half-step boundary, or a precomputed-reciprocal sign error at low baud.
///
/// Also the per-byte IRQ budget guard: the boot batch closes at Ping 1's
/// packet boundary even though it lands nothing (below-deadband close =
/// drift ≈ 0 learned), and the drift window must close with it — a
/// perfectly-trimmed chip must not hold per-byte RXNE wakes open forever.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_at_nominal_emits_no_trim_ops(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    assert!(
        sim.servo(servos[0]).rx_byte_wake_watched(),
        "cold-boot drift window open before any traffic (baud_idx={baud_idx})",
    );

    sim.with_host(host, |h| {
        h.send_ping(TARGET);
        h.wait_for_reply();
    });
    assert!(
        !sim.servo(servos[0]).rx_byte_wake_watched(),
        "boot batch closed below the deadband — the drift window must \
         close with it, landed or not (baud_idx={baud_idx})",
    );

    for _ in 1..PING_BUDGET {
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
fn live_residual_ppm(
    sim: &osc_integration::sim::Sim,
    servo: osc_integration::sim::DeviceId,
) -> i32 {
    let live_hz = sim.servo(servo).clock().freq_hz() as i64;
    let nominal_hz = sim.servo(servo).hsi_clock().nominal().freq_hz() as i64;
    ((live_hz - nominal_hz) * 1_000_000 / nominal_hz) as i32
}

/// +2% factory drift, one Ping. The boot batch closes at Ping 1's packet
/// boundary holding the one window span, and the magnitude-aware estimator
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

/// Steady-phase tracks a mid-operation drift shift. Boot at nominal HSI:
/// Ping 1 closes the boot batch below the deadband (no emit) and the drift
/// window closes with it — per-byte IRQs never outlive the sampling. Then a
/// +0.5% drift appears — models the chip warming up under load. The shift
/// goes untracked until the staleness reopen (64 instructions since the
/// last accepted span), after which one 20-span steady batch measures and
/// squeezes it back into the deadband. At 3M (worst SNR) +0.5% drift
/// produces ≈ 2 steps → emit -5 000 ppm, residual
/// `(1.005 × 0.995 - 1) × 10⁶ ≈ -25 ppm`, and the batch close ends the
/// reopened window again.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_steady_phase_tracks_dynamic_drift(baud_idx: u8) {
    // Staleness reopen at instruction 65 (64 past Ping 1's accepted span),
    // then one 20-span steady batch (one span per ping) closes at ~85, plus
    // margin for the apply and the residual read.
    const STEADY_PINGS: u32 = 88;

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

    for _ in 0..STEADY_PINGS {
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
         after {STEADY_PINGS} pings under +0.5% temp shift \
         (baud_idx={baud_idx}, ops_after_boot={ops_after_boot:?}, ops={ops:?})",
    );
    assert!(
        ops.len() > ops_after_boot.len(),
        "steady phase emitted nothing under +0.5% drift \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!(
        "baud_idx={baud_idx} dynamic +0.5% drift after {STEADY_PINGS} pings → \
         residual_ppm={residual_ppm} ops={ops:?}",
    );
}

/// +2% drift, foreign-instruction sampling path. Each Ping targets id=99
/// (no servo answers) so the lone servo at id=1 parses Sync + Header then
/// drops into byte-skip for the remaining CRC. The per-byte wakes record
/// across both regions and the window settles the same one span at the
/// skip-complete boundary (foreign instructions pass the instruction gate)
/// it would get from an own-target reception. Convergence must therefore
/// match the own-target hot-corner case.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_hot_corner_converges_via_foreign_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(1, 50);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(FOREIGN_TARGET);
            h.wait_for_reply();
        });
    }
    let ops = sim.servo(servos[0]).trim_ops();
    let final_ppm = *ops.last().expect("expected at least one trim op");

    assert!(
        final_ppm <= -15_000,
        "foreign-ping correction too small for +2% drift: {final_ppm} \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!("baud_idx={baud_idx} +2% drift, foreign pings → final_ppm={final_ppm}, ops={ops:?}",);
}

/// -2% drift, foreign-instruction sampling path. Symmetric to the
/// foreign-ping hot-corner case.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_cold_corner_converges_via_foreign_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 50);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(FOREIGN_TARGET);
            h.wait_for_reply();
        });
    }
    let ops = sim.servo(servos[0]).trim_ops();
    let final_ppm = *ops.last().expect("expected at least one trim op");

    assert!(
        final_ppm >= 15_000,
        "foreign-ping correction too small for -2% drift: {final_ppm} \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    log::info!("baud_idx={baud_idx} -2% drift, foreign pings → final_ppm={final_ppm}, ops={ops:?}",);
}

/// +2% drift, one foreign Ping. The boot batch closes at the first Ping's
/// skip-complete boundary the same way it closes at an own-target Crc.
/// Emit must equal exactly -20_000 ppm — same one window span, same
/// magnitude-aware estimator, same boot-phase emit cap.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_hot_corner_lands_in_one_foreign_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(1, 50);

    sim.with_host(host, |h| {
        h.send_ping(FOREIGN_TARGET);
        h.wait_for_reply();
    });

    let ops = sim.servo(servos[0]).trim_ops();
    assert_eq!(
        ops.len(),
        1,
        "expected exactly one emit after first foreign Ping \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    assert_eq!(
        ops[0], -20_000,
        "boot emit at +2% drift via foreign Ping should be exactly -20_000 ppm \
         (8 steps × 2500 ppm/step); got {} (baud_idx={baud_idx})",
        ops[0],
    );
    let residual_ppm = live_residual_ppm(&sim, servos[0]);
    assert!(
        residual_ppm.abs() < 1000,
        "expected |residual_ppm| < 1000 after 1-foreign-ping boot landing, \
         got {residual_ppm} (baud_idx={baud_idx})",
    );
    log::info!(
        "baud_idx={baud_idx} +2% drift, 1 foreign ping → emit={} residual_ppm={residual_ppm}",
        ops[0],
    );
}

/// -2% drift, one foreign Ping. Symmetric to the foreign-ping hot-corner
/// one-Ping case.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_cold_corner_lands_in_one_foreign_ping(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, DEFAULT_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 50);

    sim.with_host(host, |h| {
        h.send_ping(FOREIGN_TARGET);
        h.wait_for_reply();
    });

    let ops = sim.servo(servos[0]).trim_ops();
    assert_eq!(
        ops.len(),
        1,
        "expected exactly one emit after first foreign Ping \
         (baud_idx={baud_idx}, ops={ops:?})",
    );
    assert_eq!(
        ops[0], 20_000,
        "boot emit at -2% drift via foreign Ping should be exactly +20_000 ppm \
         (8 steps × 2500 ppm/step); got {} (baud_idx={baud_idx})",
        ops[0],
    );
    let residual_ppm = live_residual_ppm(&sim, servos[0]);
    assert!(
        residual_ppm.abs() < 1000,
        "expected |residual_ppm| < 1000 after 1-foreign-ping boot landing, \
         got {residual_ppm} (baud_idx={baud_idx})",
    );
    log::info!(
        "baud_idx={baud_idx} -2% drift, 1 foreign ping → emit={} residual_ppm={residual_ppm}",
        ops[0],
    );
}
