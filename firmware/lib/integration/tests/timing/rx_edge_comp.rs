//! RX edge-stamp compensation invariance — picks two observable values
//! that are sensitive to whether the driver's per-baud edge-stamp
//! subtraction is wired correctly, then runs the same scenario at
//! `comp = 0` (no offset modeled, no subtraction needed) and at a
//! non-zero `comp` that matches the V006 IC-filter LUT for the test
//! baud. Both arms must produce identical observable values: comp
//! cancels in HSI byte-pair deltas and gets recovered by the driver's
//! subtraction at packet_end-tick math.
//!
//! - **`hsi_trim_invariant_under_rx_edge_comp`**: integrator runs on
//!   `(curr − prev)` byte-time deltas. Stamps shifted by `+comp` produce
//!   identical deltas → identical trim emissions. If a future change
//!   asymmetrically compensates (e.g. one stamp but not the other) this
//!   test surfaces a divergent `trim_ops` vector.
//!
//! - **`tx_fire_wall_time_invariant_under_rx_edge_comp`**: the chip-side
//!   subtraction restores wire-edge time to `packet_end_tick`, so the TX
//!   fire deadline (and thus the reply's first wire bit) lands at the
//!   same wall-clock moment regardless of comp. If a future change
//!   omits the subtraction at any read-from-ring site, comp leaks into
//!   the deadline and the first reply byte slips by `comp` chip ticks.
//!
//! Per-baud comp values mirror `osc-ch32::providers::usart_baud::
//! ic_filter_delay_ticks_for` (the chip-side source of truth) — adding a
//! `BaudRate` variant there should land matching arms in
//! [`comp_ticks_for`] below.

use crate::support::{Setup, baud_matrix, setup_with};
use dxl_protocol::types::Id;
use osc_core::BaudRate;
use osc_core::regions::config::addr::comms;
use osc_integration::sim::DEFAULT_RDT_US;
use rstest::rstest;
use rstest_reuse::apply;

const TARGET: Id = Id::new(1);

/// Pings driven across each scenario. 32 is comfortably past the boot-batch
/// close (first reply emits) and through a few steady-batch emissions,
/// giving the test a multi-emission trim_ops vector to compare against.
const HSI_PING_BUDGET: u32 = 32;

/// Per-baud sim/driver compensation value, mirroring the V006 chip-side
/// LUT in `osc-ch32::providers::usart_baud::ic_filter_delay_ticks_for`.
/// Both arms of each invariance test feed the same baud's value to the
/// sim's IC-stamp shift AND the driver's mock; the math cancels iff the
/// driver subtracts at every read-from-ring site.
fn comp_ticks_for(baud: BaudRate) -> u16 {
    match baud {
        BaudRate::B3000000 => 12,
        BaudRate::B2000000 => 16,
        BaudRate::B1000000 => 32,
        BaudRate::B115200 | BaudRate::B57600 | BaudRate::B9600 => 256,
    }
}

/// HSI integrator processes `(curr − prev)` byte-time deltas, so a uniform
/// `+comp` offset on every stamp cancels in the subtraction. Drive +2%
/// drift over a fixed ping budget at `comp = 0` and at the V006 3 Mbaud
/// comp value; the two `trim_ops` vectors must be byte-for-byte equal.
/// Sweeps every supported baud — the LUT comp varies but the cancellation
/// is universal.
#[apply(baud_matrix)]
#[test_log::test]
fn hsi_trim_invariant_under_rx_edge_comp(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let comp = comp_ticks_for(baud);
    let baseline = run_hsi(baud, DEFAULT_RDT_US, 0, HSI_PING_BUDGET);
    let with_comp = run_hsi(baud, DEFAULT_RDT_US, comp, HSI_PING_BUDGET);
    assert_eq!(
        baseline, with_comp,
        "trim_ops diverged at baud_idx={baud_idx} between comp=0 and comp={comp} \
         (baseline={baseline:?}, comp={with_comp:?})",
    );
    log::info!(
        "baud_idx={baud_idx} +2% drift → trim_ops invariant under comp={comp}: {baseline:?}",
    );
}

fn run_hsi(baud: BaudRate, rdt_us: u32, comp_ticks: u16, ping_budget: u32) -> Vec<i32> {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    // +2% factory HSI drift. Same hardware-shaped knob both arms exercise;
    // the only difference between runs is the rx-edge stamp offset.
    sim.servo_mut(servos[0]).set_hsi_drift(1, 50);
    sim.servo_mut(servos[0]).set_rx_edge_comp_ticks(comp_ticks);
    for _ in 0..ping_budget {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }
    sim.servo(servos[0]).trim_ops()
}

/// 510 µs RDT — max realistic Return Delay Time (u8 × 2 µs); large enough
/// that the chip-side `max(RDT, byte_time)` floor never floors out at the
/// faster bauds, so the deadline math under test is RDT-dominated and a
/// missing subtraction surfaces directly as wire-time drift.
const TX_FIRE_RDT_US: u32 = 510;

/// Pings to converge HSI before measuring TX fire — matches the prelude
/// in `hsi_trim.rs`'s wire-edge tests so the integrator's residual is
/// flat-zero at nominal HSI in both arms.
const PRECONVERGE_PINGS: u32 = 64;

/// The driver subtracts `comp` from every IC stamp at read-from-ring time,
/// restoring wire-edge time to `last_byte_start`. `packet_end_tick` — and
/// every downstream tick derived from it (TX deadline, phase-adjust input,
/// chain-CRC anchors) — is then identical to the comp=0 arm modulo nothing.
/// Run the same prelude + read at `comp = 0` and at the per-baud LUT comp
/// value; the first reply byte's wall clock must be **exactly** equal —
/// no tolerance. Any non-zero drift means comp leaked into the deadline
/// at some site the driver should have subtracted at. Sweeps every
/// supported baud.
#[apply(baud_matrix)]
#[test_log::test]
fn tx_fire_wall_time_invariant_under_rx_edge_comp(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let comp = comp_ticks_for(baud);
    let baseline = run_tx_fire(baud, TX_FIRE_RDT_US, 0);
    let with_comp = run_tx_fire(baud, TX_FIRE_RDT_US, comp);
    assert_eq!(
        baseline, with_comp,
        "first-byte wall time diverged at baud_idx={baud_idx} between comp=0 \
         ({baseline} ns) and comp={comp} ({with_comp} ns)",
    );
    log::info!(
        "baud_idx={baud_idx} rdt=510µs → first-byte exactly invariant under \
         comp={comp}: {baseline}ns",
    );
}

fn run_tx_fire(baud: BaudRate, rdt_us: u32, comp_ticks: u16) -> u64 {
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(1, baud, rdt_us);
    sim.servo_mut(servos[0]).set_rx_edge_comp_ticks(comp_ticks);

    // Preconverge HSI so residual phase-adjust is below the comp signal.
    for _ in 0..PRECONVERGE_PINGS {
        sim.with_host(host, |h| {
            h.send_ping(TARGET);
            h.wait_for_reply();
        });
    }

    // Fresh measurement window: a single Read whose reply's first wire byte
    // is the assertable observable. The two arms are compared directly
    // against each other, so no `expected` formula is needed — invariance
    // is the property under test.
    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_read(TARGET, comms::ID, 1);
        h.wait_for_reply();
    });

    let starts = sim.host(host).rx_byte_starts_ns();
    let actual = *starts.first().expect("at least one reply byte");
    log::debug!("comp={comp_ticks} baud={baud:?} rdt_us={rdt_us}: first reply byte at {actual}ns");
    actual
}
