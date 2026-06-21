//! Fast Sync / Bulk Read slot CC-compare timing — pin that every chain
//! reply is one contiguous baud-stride burst starting at `packet_end +
//! RDT`. Each chip arms its slot's TIM2 CC-match at `packet_end_tick +
//! RDT_ticks + bytes_before(k) × byte_ticks`; if any slot fires early
//! it collides with the predecessor's CRC patch fold and if any slot
//! fires late an IDLE gap opens in the middle of the chain. Both fault
//! shapes surface as a non-`byte_time` inter-byte delta in
//! [`Host::rx_byte_starts_ns`].
//!
//! Same `max(RDT_ns, byte_time)` poll-time floor as the single-target
//! RDT sweep — at `RDT < ~1 byte_time` the chip's deadline has
//! already passed by the time firmware polls at IDLE, and slot 0
//! fires immediately at the poll wall-clock. Chain payload integrity
//! (chain CRC, slot decoding) lives in `tests/protocol/fast_*_read.rs`.

use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{BulkReadEntry, Id};
use osc_core::BaudRate;
use osc_core::regions::config::addr::{comms, identity};
use osc_integration::sim::{DeviceId, Host, Sim, bit_period_ns, byte_time_ns};
use rstest::rstest;
use rstest_reuse::apply;

/// Assert: first reply byte lands at `packet_end + max(RDT_ns,
/// byte_time)`, then every subsequent inter-byte delta is exactly
/// `byte_time_ns`. The pair pins slot 0's CC-match plus the contiguous
/// chain stride.
fn assert_contiguous_burst(sim: &Sim, host: DeviceId, baud: BaudRate, rdt_us: u32) {
    let packet_end = sim.host(host).packet_end_ns().expect("instruction sent");
    let starts = sim.host(host).rx_byte_starts_ns();
    assert!(
        starts.len() >= 2,
        "expected ≥ 2 reply bytes, got {}: {:?}",
        starts.len(),
        starts,
    );

    let bp = bit_period_ns(baud);
    let bt = byte_time_ns(baud);
    let rdt_ns = (rdt_us as u64) * 1_000;
    let expected_first = packet_end + rdt_ns.max(bt);
    let first_drift = starts[0].abs_diff(expected_first);
    assert!(
        first_drift < bp,
        "slot 0 first byte: actual {}ns, expected {}ns, drift {}ns (bp {}ns)",
        starts[0],
        expected_first,
        first_drift,
        bp,
    );

    for (i, pair) in starts.windows(2).enumerate() {
        let delta = pair[1] - pair[0];
        let delta_drift = delta.abs_diff(bt);
        assert!(
            delta_drift < bp,
            "byte {}→{} inter-byte delta {}ns, expected {}ns (byte_time), drift {}ns (bp {}ns)",
            i,
            i + 1,
            delta,
            bt,
            delta_drift,
            bp,
        );
    }
}

/// Helper for callers that follow the standard send-fast-then-assert
/// shape; just collapses the boilerplate.
fn host_wait(sim: &mut Sim, host: DeviceId, prep: impl FnOnce(&mut Host)) {
    sim.with_host(host, |h| {
        prep(h);
        h.wait_for_reply();
    });
}

#[apply(matrix)]
#[test_log::test]
fn fast_sync_read_wire_burst_is_contiguous(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    host_wait(&mut sim, host, |h| {
        h.send_fast_sync_read(comms::ID, 4, &[1, 2, 3]);
    });

    assert_contiguous_burst(&sim, host, baud, rdt_us);
}

#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_wire_burst_is_contiguous(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    // Asymmetric per-entry lengths exercise the per-slot `bytes_before`
    // accumulator on the Bulk path (uniform-length Fast Sync covers the
    // symmetric formula above).
    let entries = [
        BulkReadEntry {
            id: Id::new(1),
            address: comms::ID,
            length: 1,
        },
        BulkReadEntry {
            id: Id::new(2),
            address: identity::MODEL_NUMBER,
            length: 2,
        },
        BulkReadEntry {
            id: Id::new(3),
            address: identity::FIRMWARE_VERSION,
            length: 1,
        },
    ];

    host_wait(&mut sim, host, |h| {
        h.send_fast_bulk_read(&entries);
    });

    assert_contiguous_burst(&sim, host, baud, rdt_us);
}
