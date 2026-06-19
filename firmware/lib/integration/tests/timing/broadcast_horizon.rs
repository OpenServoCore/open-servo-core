//! Multi-wrap broadcast-horizon timing tests — exercise the u32 schedule
//! lift end-to-end at slot offsets that would silently truncate against
//! TIM2's 16-bit CCR3 / CNT (1.365 ms at HCLK = 48 MHz).
//!
//! Wrap math at default 1 Mbaud × 48 MHz servo HCLK:
//! `bit_time = 48 ticks` (= HCLK / baud), `byte_time = 480 ticks` (10 bits
//! per UART frame), `u16_wrap = 65536 ticks ≈ 136.5 byte_times`.
//!
//! **Broadcast Ping** positions servo N's reply at `N × 14` wire bytes past
//! `packet_end` (`PING_STATUS_FRAME_BYTES = RESPONSE_HEADER_BYTES(9) + 3 +
//! CRC_BYTES(2) = 14`). 136.5 / 14 ≈ 9.75 servos per wrap; servo 10+
//! straddles the wrap. `setup(12)` puts the last three replies solidly past
//! it.
//!
//! **Fast Bulk Read at L = MAX_CONTROL_RW (128)** carries `fast_first(L) =
//! 138` and `fast_middle(L) = 130` wire bytes per slot — ~1 slot per wrap.
//! A 3-servo chain places servo 3's fire offset ≈ 268 bytes past
//! packet_end (~2 wraps).
//!
//! **Plain Sync Read at k > 0** fires sequence-driven off the predecessor's
//! skip-exhaust event (`docs/dxl-streaming-rx.md` §5.2), not deadline-driven
//! — wrap-agnostic by construction. The test pins the invariant by running
//! the same multi-wrap-sized chain and verifying every slot's Status lands
//! in order.

use crate::support::{Setup, setup};
use dxl_protocol::types::{BulkReadEntry, Id, Instruction, PingStatus, Status, StatusError};
use osc_core::regions::config::addr::comms;
use osc_core::services::dxl::limits::MAX_CONTROL_RW;
use osc_integration::sim::{
    DEFAULT_FIRMWARE_VERSION, DEFAULT_MODEL_NUMBER, FastStatusCrc, Host, SimTime,
    parse_fast_bulk_status, parse_status_stream,
};

/// Bus drain window — at 1 Mbaud one wire byte is 10 µs, so a 12-servo
/// broadcast Ping (~168 wire bytes of replies) needs ~1.7 ms minimum.
/// Headroom for parser + IDLE settling: 10 ms.
const STATUS_WINDOW: SimTime = SimTime::from_ms(10);

#[test_log::test]
fn broadcast_ping_at_12_servos_replies_in_order_past_u16_wrap() {
    let Setup { mut sim, host, .. } = setup(12);

    sim.device_mut::<Host>(host).send_ping(Id::BROADCAST);
    sim.device_mut::<Host>(host)
        .wait_for_status_within(STATUS_WINDOW);
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

#[test_log::test]
fn fast_bulk_read_3_servos_full_payload_chain_crosses_u16_wrap() {
    let Setup { mut sim, host, .. } = setup(3);

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

    sim.device_mut::<Host>(host).send_fast_bulk_read(&entries);
    sim.device_mut::<Host>(host)
        .wait_for_status_within(STATUS_WINDOW);
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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

#[test_log::test]
fn plain_sync_read_chain_remains_sequence_driven_past_u16_wrap() {
    let Setup { mut sim, host, .. } = setup(3);

    // Multi-wrap-sized chain — each slot's `len` bytes of payload mean the
    // cumulative wire offset to slot 3 crosses the u16 wrap, mirroring the
    // Fast Bulk Read sibling above. The Plain k > 0 fire path arms off
    // the predecessor's skip-exhaust event (`docs/dxl-streaming-rx.md`
    // §5.2), not a deadline tick — wrap-agnostic by mechanism. This test
    // pins that invariant: if a hidden deadline path crept in, slots 2/3
    // would fire at the wrong instant and the chain would either collide
    // or stall.
    let len = MAX_CONTROL_RW as u16;
    sim.device_mut::<Host>(host)
        .send_sync_read(comms::ID, len, &[1, 2, 3]);
    sim.device_mut::<Host>(host)
        .wait_for_status_within(STATUS_WINDOW);
    sim.advance(SimTime::from_ms(20));

    let rx = sim.device::<Host>(host).rx_bytes();
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
