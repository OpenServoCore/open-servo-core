//! DXL 2.0 broadcast Ping wire-timing — pin servo k's first start-bit at
//! `packet_end + DEFAULT_RDT + k × PING_STATUS_FRAME_BYTES × byte_time`.
//!
//! RDT is sourced from the uniform driver default
//! (`osc_drivers::dxl::DEFAULT_RDT_2US`), not from the per-instance
//! register-table value. The per-id `k × frame_bytes` spacer already owns
//! collision avoidance — adding self-rdt on top would let a low-id,
//! high-rdt servo extend into the next id's slot.
//!
//! `long_reply_delays.rs::broadcast_ping_at_12_servos_replies_in_order`
//! covers ordering + content; this test pins the byte-arrival schedule
//! against the spec formula.

use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::Id;
use dxl_protocol::wire::{CRC_BYTES, RESPONSE_HEADER_BYTES};
use osc_core::BaudRate;
use osc_drivers::dxl::DEFAULT_RDT_2US;
use osc_integration::sim::{RxLogKind, bit_period_ns};
use rstest::rstest;
use rstest_reuse::apply;

/// Mirrors `drivers/dxl/uart/mod.rs::PING_STATUS_FRAME_BYTES`. Re-derived
/// from the protocol layer rather than re-exported because the firmware
/// constant is chip-side; the test asserts the host observes the same
/// shape.
const PING_STATUS_FRAME_BYTES: u64 = (RESPONSE_HEADER_BYTES + 3 + CRC_BYTES) as u64;

#[apply(matrix)]
#[test_log::test]
fn broadcast_ping_slot_offsets_match_spec(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply();
    });

    let bp = bit_period_ns(baud);
    let byte_time = 10 * bp;
    // Broadcast Ping ignores per-servo register RDT (the swept `rdt_us`)
    // and uses the driver default — keeps every on-bus servo aligned even
    // with heterogeneous RDT configs.
    let rdt_ns = (DEFAULT_RDT_2US as u64) * 2 * 1_000;

    // `packet_end` = last instruction byte's start + 10·bp (stop-bit clear).
    let packet_end = sim
        .host(host)
        .tx_log()
        .last()
        .expect("instruction sent")
        .at
        .as_ns()
        + byte_time;

    // rx_log byte timestamps land at byte completion (10·bp past start
    // bit); subtract `byte_time` to recover the start-bit instant. Every
    // 14th byte is the first byte of the next slot's frame.
    let frame_starts: Vec<u64> = sim
        .host(host)
        .rx_log()
        .iter()
        .filter_map(|e| match e.kind {
            RxLogKind::Byte(_) => Some(e.at.as_ns() - byte_time),
            RxLogKind::IdleGap => None,
        })
        .enumerate()
        .filter_map(|(i, t)| (i as u64).is_multiple_of(PING_STATUS_FRAME_BYTES).then_some(t))
        .collect();

    assert_eq!(
        frame_starts.len(),
        3,
        "expected 3 ping frames, got {} (rx_log = {:?})",
        frame_starts.len(),
        sim.host(host).rx_log(),
    );

    for (idx, &actual_start) in frame_starts.iter().enumerate() {
        let k = (idx + 1) as u64;
        let expected_start = packet_end + rdt_ns + k * PING_STATUS_FRAME_BYTES * byte_time;
        let drift = actual_start.abs_diff(expected_start);
        // 1 bit_period absorbs integer-tick rounding in the firmware's
        // `bytes_to_ticks`; larger drift signals real schedule error.
        assert!(
            drift < bp,
            "slot {k}: actual {actual_start}ns, expected {expected_start}ns, \
             drift {drift}ns (bp {bp}ns, rdt_us {rdt_us}, baud_idx {baud_idx})",
        );
    }
}
