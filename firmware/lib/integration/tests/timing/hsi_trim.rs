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
use osc_core::regions::config::addr::comms;
use osc_integration::sim::{DEFAULT_RDT_US, byte_time_ns};
use rstest::rstest;
use rstest_reuse::apply;

/// Pings to drive per test. The boot phase closes one batch on Ping 1
/// (6 byte pairs from the reply). Steady phase closes every ~3.3 pings
/// (20 / 6). 64 pings leaves ~19 steady batches of headroom over the
/// first-emit landing — enough margin to surface any regression in the
/// steady-phase deadband or residual squeeze.
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

/// Sub-step phase-projection assertions need a deterministic fixed-baud
/// scenario — at 1 Mbaud the integrator settles to a known state under
/// ±0.4% drift, with no cross-baud quantization variance in the residual.
/// Other bauds (especially 115200) show ±1 tick of stamp-noise jitter on
/// the same drift, so a matrix sweep can't pin the projection to an exact
/// value.
const FINE_TRIM_BAUD: BaudRate = BaudRate::B1000000;
/// 100 µs of chip-HCLK ticks at the V006's 48 MHz reference. Representative
/// of a single-slot RDT delay at typical bench RDT values.
const PHASE_PROBE_DISTANCE_HCLK: u32 = 4800;

/// At nominal HSI on 1 Mbaud, the integrator stays exactly inside the
/// half-step deadband and emits zero — the residual_q8 latches at the
/// noise floor (just inside zero) and the projection rounds to 0 over
/// the 100 µs probe distance. Any non-zero return means either an
/// unintended residual accumulation or a regression in the projection
/// math.
#[test_log::test]
fn fine_trim_phase_adjust_at_nominal_drift_is_zero() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, FINE_TRIM_BAUD, 0);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }

    let phase = sim
        .servo(servos[0])
        .projected_phase_error_hclk(PHASE_PROBE_DISTANCE_HCLK);
    assert_eq!(phase, 0, "expected zero phase adjust at nominal HSI");
}

/// +0.4% factory drift (= 1.6 trim-steps worth in the steady-phase
/// integrator window) lands the integer-step apply at -5000 ppm
/// (2 steps; the ideal correction would be -3982 ppm). The resulting
/// live HSI runs slow at `(1.004)(0.995) - 1 ≈ -1020 ppm`, the
/// integrator's residual_q8 inherits that signed offset (negative =
/// HSI slow), and the back-date projection over 100 µs lands NEGATIVE:
///
///   phase ≈ -0.4 × STEP_PPM × distance / 1e6
///         = -0.4 × 2500 × 4800 / 1e6
///         ≈ -4.8 HCLK ticks  →  -4 (signed integer truncation toward zero)
///
/// A residual-sign inversion would land near +43; a scale regression
/// would land far from -4.
#[test_log::test]
fn fine_trim_phase_adjust_under_positive_drift_is_negative() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, FINE_TRIM_BAUD, 0);
    sim.servo_mut(servos[0]).set_hsi_drift(1, 250);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }

    let phase = sim
        .servo(servos[0])
        .projected_phase_error_hclk(PHASE_PROBE_DISTANCE_HCLK);
    assert_eq!(
        phase, -4,
        "expected -4 HCLK ticks (over-corrected at +0.4% drift)",
    );
}

/// Symmetric to the positive-drift case: -0.4% factory drift sets
/// applied = +5000 ppm (2 steps), live HSI runs fast at ~+1020 ppm,
/// residual_q8 is positive (HSI fast), projection over 100 µs lands
/// POSITIVE at +4 HCLK ticks.
#[test_log::test]
fn fine_trim_phase_adjust_under_negative_drift_is_positive() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, FINE_TRIM_BAUD, 0);
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 250);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }

    let phase = sim
        .servo(servos[0])
        .projected_phase_error_hclk(PHASE_PROBE_DISTANCE_HCLK);
    assert_eq!(
        phase, 4,
        "expected +4 HCLK ticks (over-corrected at -0.4% drift)",
    );
}

/// Max realistic RDT (`u8 × 2 µs`). Phase-adjust scales by `delay_ticks`,
/// so a larger RDT gives `projected_phase_error_hclk` more leverage. At 1M
/// baud × 510 µs the uncorrected wire-bit drift under ±0.4% factory drift
/// (post-convergence ≈ ±1020 ppm live residual) is ~530 ns ≈ 0.5 bp —
/// well above the [`FINE_TRIM_WIRE_TOLERANCE_NS`] floor.
const FINE_TRIM_WIRE_RDT_US: u32 = 510;
/// Sub-bit-period tolerance for fixed-1M-baud wire-edge tests. 10× the
/// observed ~10 ns residual after phase_adjust correctly cancels ±0.4%
/// drift, 5× under the smallest catchable bug (a 25% scale error in
/// `projected_phase_error_hclk` lands ~130 ns drift; a missing wireup
/// at a scheduler call site lands ~530 ns; a sign bug lands ~1060 ns).
const FINE_TRIM_WIRE_TOLERANCE_NS: u64 = 100;

/// At nominal HSI the integrator's residual stays zero; phase_adjust is
/// zero; the reply's first wire bit lands exactly on
/// `packet_end + max(RDT, byte_time)`. Baseline against the drift cases
/// below — any non-zero phase_adjust here would point at residual
/// accumulation on a quiescent integrator.
#[test_log::test]
fn fine_trim_wire_edge_at_nominal_drift_lands_on_packet_end_plus_rdt() {
    let Setup { mut sim, host, .. } = setup_with(1, FINE_TRIM_BAUD, FINE_TRIM_WIRE_RDT_US);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("read sent");
    let starts = sim.host(host).rx_byte_starts_ns();
    let actual = *starts.first().expect("at least one reply byte");
    let rdt_ns = (FINE_TRIM_WIRE_RDT_US as u64) * 1_000;
    let expected = packet_end + rdt_ns.max(byte_time_ns(FINE_TRIM_BAUD));
    let drift = actual.abs_diff(expected);
    assert!(
        drift < FINE_TRIM_WIRE_TOLERANCE_NS,
        "actual {actual}ns, expected {expected}ns, drift {drift}ns (tol {FINE_TRIM_WIRE_TOLERANCE_NS}ns)",
    );
}

/// +0.4% factory drift → boot fires -5000 ppm, live HSI lands at ~-1020 ppm
/// (slow). Without phase_adjust the chip's TX countdown runs slow, so the
/// reply's first wire bit lands ~530 ns LATE. The negative residual_q8
/// → negative phase_adjust pulls the chip-tick deadline EARLIER, cancelling
/// the wall-clock drift.
#[test_log::test]
fn fine_trim_wire_edge_under_positive_drift_lands_on_packet_end_plus_rdt() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, FINE_TRIM_BAUD, FINE_TRIM_WIRE_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(1, 250);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("read sent");
    let starts = sim.host(host).rx_byte_starts_ns();
    let actual = *starts.first().expect("at least one reply byte");
    let rdt_ns = (FINE_TRIM_WIRE_RDT_US as u64) * 1_000;
    let expected = packet_end + rdt_ns.max(byte_time_ns(FINE_TRIM_BAUD));
    let drift = actual.abs_diff(expected);
    assert!(
        drift < FINE_TRIM_WIRE_TOLERANCE_NS,
        "actual {actual}ns, expected {expected}ns, drift {drift}ns (tol {FINE_TRIM_WIRE_TOLERANCE_NS}ns)",
    );
}

/// -0.4% factory drift, symmetric to the +drift case. Boot fires +5000 ppm,
/// live HSI lands at ~+1020 ppm (fast); without phase_adjust the chip-tick
/// countdown overshoots and the wire bit lands ~500 ns EARLY. The positive
/// residual_q8 → positive phase_adjust pushes the deadline LATER, cancelling
/// the drift.
#[test_log::test]
fn fine_trim_wire_edge_under_negative_drift_lands_on_packet_end_plus_rdt() {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, FINE_TRIM_BAUD, FINE_TRIM_WIRE_RDT_US);
    sim.servo_mut(servos[0]).set_hsi_drift(-1, 250);

    for _ in 0..PING_BUDGET {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("read sent");
    let starts = sim.host(host).rx_byte_starts_ns();
    let actual = *starts.first().expect("at least one reply byte");
    let rdt_ns = (FINE_TRIM_WIRE_RDT_US as u64) * 1_000;
    let expected = packet_end + rdt_ns.max(byte_time_ns(FINE_TRIM_BAUD));
    let drift = actual.abs_diff(expected);
    assert!(
        drift < FINE_TRIM_WIRE_TOLERANCE_NS,
        "actual {actual}ns, expected {expected}ns, drift {drift}ns (tol {FINE_TRIM_WIRE_TOLERANCE_NS}ns)",
    );
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

/// +2% drift, foreign-instruction sampling path. Each Ping targets id=99
/// (no servo answers) so the lone servo at id=1 parses Sync + Header then
/// drops into byte-skip for the remaining CRC. The byte-parser-driven
/// edge walker advances in lockstep through both regions, so the
/// integrator gets the same 6 byte pairs per Ping it would get from an
/// own-target reception. Convergence must therefore match the own-target
/// hot-corner case.
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

/// +2% drift, one foreign Ping. The boot batch closes during the skip
/// phase of the first Ping the same way it closes during the body of an
/// own-target reception. Emit must equal exactly -20_000 ppm — same byte
/// pair count, same magnitude-aware estimator, same boot-phase emit cap.
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
