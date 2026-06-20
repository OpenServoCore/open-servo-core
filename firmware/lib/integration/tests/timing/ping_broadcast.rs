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

use crate::support::{Setup, baud_matrix, setup_with};
use dxl_protocol::types::Id;
use dxl_protocol::wire::{CRC_BYTES, RESPONSE_HEADER_BYTES};
use osc_core::BaudRate;
use osc_integration::sim::{
    DEFAULT_RDT_NS, DEFAULT_RDT_US, DeviceId, Sim, bit_period_ns, byte_time_ns,
};
use rstest::rstest;
use rstest_reuse::apply;

/// Mirrors `drivers/dxl/uart/mod.rs::PING_STATUS_FRAME_BYTES`. Re-derived
/// from the protocol layer rather than re-exported because the firmware
/// constant is chip-side; the test asserts the host observes the same
/// shape.
const PING_STATUS_FRAME_BYTES: u64 = (RESPONSE_HEADER_BYTES + 3 + CRC_BYTES) as u64;

/// Stride `rx_byte_starts_ns` by `PING_STATUS_FRAME_BYTES` to keep the
/// first byte of each slot's frame; drops the 13 intra-frame bytes that
/// follow it.
fn ping_frame_starts_ns(sim: &Sim, host: DeviceId) -> Vec<u64> {
    sim.host(host)
        .rx_byte_starts_ns()
        .into_iter()
        .enumerate()
        .filter_map(|(i, t)| {
            (i as u64)
                .is_multiple_of(PING_STATUS_FRAME_BYTES)
                .then_some(t)
        })
        .collect()
}

fn assert_slot_schedule(starts: &[u64], packet_end: u64, baud: BaudRate, baud_idx: u8) {
    let bp = bit_period_ns(baud);
    let byte_time = byte_time_ns(baud);
    for (idx, &actual_start) in starts.iter().enumerate() {
        let k = (idx + 1) as u64;
        let expected_start = packet_end + DEFAULT_RDT_NS + k * PING_STATUS_FRAME_BYTES * byte_time;
        let drift = actual_start.abs_diff(expected_start);
        // 1 bit_period absorbs integer-tick rounding in the firmware's
        // `bytes_to_ticks`; larger drift signals real schedule error.
        assert!(
            drift < bp,
            "slot {k}: actual {actual_start}ns, expected {expected_start}ns, \
             drift {drift}ns (bp {bp}ns, baud_idx {baud_idx})",
        );
    }
}

#[apply(baud_matrix)]
#[test_log::test]
fn broadcast_ping_slot_offsets_match_spec(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, DEFAULT_RDT_US);

    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("instruction sent");
    let starts = ping_frame_starts_ns(&sim, host);

    assert_eq!(
        starts.len(),
        3,
        "expected 3 ping frames, got {} (rx_log = {:?})",
        starts.len(),
        sim.host(host).rx_log(),
    );
    assert_slot_schedule(&starts, packet_end, baud, baud_idx);
}

/// Heterogeneous per-servo RDT must not leak into broadcast-Ping slot
/// timing — the driver default (`DEFAULT_RDT_2US`) is the only RDT the
/// schedule sees. Servo 1 at max RDT (510 µs) and servo 2 at zero
/// straddle the default (250 µs); any leakage would push slot 1 later
/// and slot 2 earlier.
#[apply(baud_matrix)]
#[test_log::test]
fn broadcast_ping_ignores_per_servo_rdt(baud_idx: u8) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    // Register is u8 × 2 µs → 510 µs is the bus-wide ceiling.
    const MAX_RDT_US: u32 = 510;
    let Setup {
        mut sim,
        host,
        servos,
    } = setup_with(2, baud, 0);
    sim.servo_mut(servos[0]).set_rdt_us(MAX_RDT_US);

    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("instruction sent");
    let starts = ping_frame_starts_ns(&sim, host);

    assert_eq!(
        starts.len(),
        2,
        "expected 2 ping frames, got {} (rx_log = {:?})",
        starts.len(),
        sim.host(host).rx_log(),
    );
    assert_slot_schedule(&starts, packet_end, baud, baud_idx);
}
