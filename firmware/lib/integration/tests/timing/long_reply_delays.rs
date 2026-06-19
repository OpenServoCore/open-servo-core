//! Long instruction-to-reply delay tests — pin the invariant that the
//! chip's reply (or chain slot's fire) lands correctly regardless of how
//! far past instruction-end the wire offset is. A naive integer-rollover
//! bug anywhere in the chip's scheduling math would surface here.
//!
//! Today's V006 carries TIM2 at 16-bit / 1.365 ms wrap (`u16_wrap = 65536
//! ticks ≈ 136.5 byte_times` at 1 Mbaud × 48 MHz HCLK; `bit_time = 48`,
//! `byte_time = 480`), and the offsets below are sized to clear that
//! boundary by multiple wraps. A chip with a wider timer would carry the
//! same protocol-level invariant — these tests defend the guarantee,
//! not the V006-specific math.
//!
//! **Broadcast Ping** positions servo N's reply at `N × 14` wire bytes past
//! `packet_end` (`PING_STATUS_FRAME_BYTES = RESPONSE_HEADER_BYTES(9) + 3 +
//! CRC_BYTES(2) = 14`). `setup(12)` puts the last three replies at offsets
//! solidly past one V006 wrap.
//!
//! **Fast Bulk Read at L = MAX_CONTROL_RW (128)** carries `fast_first(L) =
//! 138` and `fast_middle(L) = 130` wire bytes per slot. A 3-servo chain
//! places servo 3's fire offset ≈ 268 bytes past `packet_end` — ~2× the
//! V006 wrap.
//!
//! **Plain Sync Read at k > 0** fires sequence-driven off the predecessor's
//! skip-exhaust event (`docs/dxl-streaming-rx.md` §5.2), not deadline-driven
//! — wrap-agnostic by construction. The test pins the invariant by running
//! the same long-offset chain and verifying every slot's Status lands in
//! order.

use crate::support::{Setup, matrix, setup_with};
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, PingStatus, Status, StatusError};
use osc_core::BaudRate;
use osc_core::regions::config::addr::comms;
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, FastStatusCrc, HOST_ABSOLUTE_CAP,
    HOST_INTER_BYTE_TIMEOUT, parse_fast_bulk_status, parse_status_stream,
};
use rstest::rstest;
use rstest_reuse::apply;

#[apply(matrix)]
#[test_log::test]
fn broadcast_ping_at_12_servos_replies_in_order(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(12, baud, rdt_us);

    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply_within(HOST_ABSOLUTE_CAP, HOST_INTER_BYTE_TIMEOUT);
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::Ping, &rx);
    let expected: Vec<Status<'_>> = (1u8..=12)
        .map(|id| Status::Ping {
            id: Id::new(id),
            error: StatusError::OK,
            status: PingStatus {
                model: DEFAULT_MODEL_NUMBER,
                fw_version: DEFAULT_FIRMWARE_VERSION,
            },
        })
        .collect();
    assert_eq!(replies, expected);
}

#[apply(matrix)]
#[test_log::test]
fn fast_bulk_read_3_servos_full_payload_chain_intact(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    let len = MAX_CONTROL_RW as u16;
    let entries = [
        BulkReadEntry {
            id: Id::new(1),
            address: 0,
            length: len,
        },
        BulkReadEntry {
            id: Id::new(2),
            address: 0,
            length: len,
        },
        BulkReadEntry {
            id: Id::new(3),
            address: 0,
            length: len,
        },
    ];

    sim.with_host(host, |h| {
        h.send_fast_bulk_read(&entries);
        h.wait_for_reply_within(HOST_ABSOLUTE_CAP, HOST_INTER_BYTE_TIMEOUT);
    });

    let rx = sim.host(host).rx_bytes();
    let status = parse_fast_bulk_status(&rx, &entries);
    // Full-payload chain still patches the trailing CRC across multi-wrap
    // fire offsets — the chain-CRC fold runs on the 32-bit SysTick grid
    // and the wire fire schedules through the time-remaining decision
    // tree (direct CC3 / SysTick handoff). Each slot's data is the chip's
    // own config region; the comparison here is structural (3 OK slots
    // in id order) rather than per-byte.
    assert_eq!(status.crc, FastStatusCrc::Good);
    assert_eq!(status.slots.len(), 3);
    for (i, slot) in status.slots.iter().enumerate() {
        assert_eq!(slot.id.as_byte(), (i + 1) as u8);
        assert_eq!(slot.error, StatusError::OK);
        assert_eq!(slot.data.len(), len as usize);
    }
}

#[apply(matrix)]
#[test_log::test]
fn plain_sync_read_chain_remains_sequence_driven(baud_idx: u8, rdt_us: u32) {
    let baud = BaudRate::from_idx(baud_idx).expect("valid baud idx");
    let Setup { mut sim, host, .. } = setup_with(3, baud, rdt_us);

    // Multi-wrap-sized chain — each slot's `len` bytes of payload mean the
    // cumulative wire offset to slot 3 crosses the u16 wrap, mirroring the
    // Fast Bulk Read sibling above. The Plain k > 0 fire path arms off
    // the predecessor's skip-exhaust event (`docs/dxl-streaming-rx.md`
    // §5.2), not a deadline tick — wrap-agnostic by mechanism. This test
    // pins that invariant: if a hidden deadline path crept in, slots 2/3
    // would fire at the wrong instant and the chain would either collide
    // or stall.
    let len = MAX_CONTROL_RW as u16;
    sim.with_host(host, |h| {
        h.send_sync_read(comms::ID, len, &[1, 2, 3]);
        h.wait_for_reply_within(HOST_ABSOLUTE_CAP, HOST_INTER_BYTE_TIMEOUT);
    });

    let rx = sim.host(host).rx_bytes();
    eprintln!(
        "plain sync rx ({}b): first 32 = {:02X?}",
        rx.len(),
        &rx[..rx.len().min(32)]
    );
    let replies = parse_status_stream(Instruction::Read, &rx);
    eprintln!("parsed replies: {}", replies.len());
    assert_eq!(replies.len(), 3);
    for (i, reply) in replies.iter().enumerate() {
        match reply {
            Status::Read { id, error, data } => {
                assert_eq!(id.as_byte(), (i + 1) as u8);
                assert_eq!(*error, StatusError::OK);
                assert_eq!(data.len(), len as usize);
            }
            other => panic!("expected Status::Read, got {:?}", other),
        }
    }
}
