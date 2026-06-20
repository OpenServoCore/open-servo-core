//! Single-target RDT — pin the chip's `deadline = packet_end_tick +
//! rdt_ticks` schedule (`drivers/dxl/uart/mod.rs::send_status`) across
//! the baud × RDT matrix. Read is representative: every single-target
//! reply (Ping / Read / Write / RegWrite / Action) lands on the same
//! catch-all arm in `into_reply_context` with `slot_offset_bytes = 0`
//! and `rdt_us = ctx.rdt_us`, so one instruction covers the equation
//! for all of them.
//!
//! The wall-clock assertion is `max(packet_end + RDT, packet_end +
//! byte_time)`. The poll that arms the TX schedule runs at IDLE
//! (~10 bp = 1 byte_time after the last instruction byte's stop bit).
//! When the chip's intended deadline has already passed by then —
//! true whenever `RDT < ~10 bp`, i.e. low baud × low RDT — production
//! hardware's TIM2 CC1 match fires on the next CNT increment, so the
//! wire byte ships at the firmware's schedule wall-clock. Above that
//! horizon the tight `packet_end + RDT` formula governs.

use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::Id;
use osc_core::BaudRate;
use osc_core::regions::config::addr::comms;
use osc_integration::sim::{bit_period_ns, byte_time_ns};
use rstest::rstest;
use rstest_reuse::apply;

#[apply(matrix)]
#[test_log::test]
fn read_reply_starts_at_packet_end_plus_rdt(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(1, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_read(Id::new(1), comms::ID, 1);
        h.wait_for_reply();
    });

    let packet_end = sim.host(host).packet_end_ns().expect("instruction sent");
    let starts = sim.host(host).rx_byte_starts_ns();
    let actual_start = *starts.first().expect("at least one reply byte");
    let rdt_ns = (rdt_us as u64) * 1_000;
    let expected_start = packet_end + rdt_ns.max(byte_time_ns(baud));
    let drift = actual_start.abs_diff(expected_start);
    let bp = bit_period_ns(baud);
    // 1 bit_period absorbs integer-tick rounding in the firmware's
    // µs→tick conversion; larger drift signals real schedule error.
    assert!(
        drift < bp,
        "actual {actual_start}ns, expected {expected_start}ns, \
         drift {drift}ns (bp {bp}ns, baud_idx={baud_idx}, rdt_us={rdt_us})",
    );
}
