// The `matrix` template below expands to a `macro_rules!` that
// each `#[apply]` site invokes. Test binaries that include `support.rs`
// without applying the template see it as unused — silence here so
// clippy stays green across all three test binaries.
#![allow(unused_macros)]

use dxl_protocol::SoftwareCrcUmts;
use dxl_protocol::encode::encode_instruction;
use dxl_protocol::types::{Id, Instruction, PingStatus, Status, StatusError};
use dxl_protocol::wire::HEADER;
use osc_core::BaudRate;

/// Encode one instruction frame into an owned buffer via the flat emitter.
#[allow(dead_code)]
fn build_frame(id: Id, instruction: u8, params: &[&[u8]]) -> Vec<u8> {
    let raw: usize = params.iter().map(|p| p.len()).sum();
    let mut buf = vec![0u8; raw * 2 + 16];
    let n = encode_instruction::<SoftwareCrcUmts>(&mut buf, id, instruction, params)
        .expect("instruction frame encodes");
    buf.truncate(n);
    buf
}
use osc_integration::sim::{
    DEFAULT_BAUD, DEFAULT_RDT_US, DeviceId, Host, Servo, Sim, parse_status_stream,
};
#[allow(unused_imports)]
use rstest::rstest;
#[allow(unused_imports)]
use rstest_reuse::template;

/// Encode a complete Ping packet's wire bytes for `id` — header, length,
/// instruction, CRC. Resilience tests use this to obtain a canonical
/// packet and then truncate / mutate it before `Host::send_raw`, or to
/// concatenate a Ping onto a prior instruction with no inter-packet IDLE.
#[allow(dead_code)]
pub fn encode_ping(id: u8) -> Vec<u8> {
    build_frame(Id::new(id), Instruction::Ping.as_u8(), &[])
}

/// Encode a complete SyncRead packet — same shape as `Host::send_sync_read`
/// but returns the bytes for callers that want to prefix / concatenate.
#[allow(dead_code)]
pub fn encode_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8> {
    build_frame(
        Id::BROADCAST,
        Instruction::SyncRead.as_u8(),
        &[&addr.to_le_bytes(), &length.to_le_bytes(), ids],
    )
}

/// Encode a complete SyncWrite packet — returns the bytes for callers that
/// want to concatenate a Write with a following instruction (no IDLE gap).
#[allow(dead_code)]
pub fn encode_sync_write(addr: u16, length: u16, body: &[u8]) -> Vec<u8> {
    build_frame(
        Id::BROADCAST,
        Instruction::SyncWrite.as_u8(),
        &[&addr.to_le_bytes(), &length.to_le_bytes(), body],
    )
}

/// Encode a complete BulkWrite packet — returns the bytes for callers that
/// want to concatenate a Write with a following instruction (no IDLE gap).
#[allow(dead_code)]
pub fn encode_bulk_write(body: &[u8]) -> Vec<u8> {
    build_frame(Id::BROADCAST, Instruction::BulkWrite.as_u8(), &[body])
}

/// Wire-format DXL 2.0 header signature (`FF FF FD 00`). Resilience
/// tests inject this prefix when crafting partial / near-sync byte
/// sequences.
#[allow(dead_code)]
pub const SYNC_PREFIX: [u8; 4] = [HEADER[0], HEADER[1], HEADER[2], 0x00];

/// 256 bytes of `0xFF` — canonical parser-drain filler that leaves the
/// servo's streaming parser at `Phase::Sync` regardless of the mid-
/// packet state it was in. Used by resilience / disconnect tests to
/// model the prefix of a real host's retry that clears a stuck parser
/// before the recovery packet is issued.
#[allow(dead_code)]
pub const PARSER_FLUSH: [u8; 256] = [0xFFu8; 256];

// Shared by `tests/{protocol,registers,timing}/mod.rs` via
// `#[path = "../support.rs"] mod support;`. Each test binary uses a
// different subset of these helpers, so dead-code allows live on every
// public item to keep the unused-in-this-binary warnings off.

#[allow(dead_code)]
pub struct Setup {
    pub sim: Sim,
    pub host: DeviceId,
    #[allow(dead_code)]
    pub servos: Vec<DeviceId>,
}

/// Build a host + `n_servos` servos at DXL ids `1..=n_servos`, using the
/// spec-default baud + RDT.
#[allow(dead_code)]
pub fn setup(n_servos: usize) -> Setup {
    setup_with(n_servos, DEFAULT_BAUD, DEFAULT_RDT_US)
}

/// Build a host + `n_servos` servos at DXL ids `1..=n_servos`, with the
/// given bus baud and per-servo return delay (µs). Drives the
/// `matrix` sweep — see the `#[apply]` sites in
/// `tests/timing/long_reply_delays.rs`.
#[allow(dead_code)]
pub fn setup_with(n_servos: usize, baud: BaudRate, rdt_us: u32) -> Setup {
    let mut sim = Sim::default();
    let host = sim.add_device(move |id| Host::new(id).with_baud(baud));
    let servos = (1..=n_servos)
        .map(|i| {
            sim.add_device(move |id| {
                Servo::setup(id, |s| {
                    s.set_dxl_id(Id::new(i as u8));
                    s.set_baud(baud);
                    s.set_rdt_us(rdt_us);
                })
            })
        })
        .collect();
    Setup { sim, host, servos }
}

/// Shared 6 × 5 baud × RDT sweep template. Tests opt in with
/// `#[apply(matrix)]`; rstest_reuse generates one `#[test]` per
/// cross-product cell. Edit the `#[values(...)]` lists here to change
/// coverage everywhere at once.
#[template]
#[rstest]
#[allow(dead_code, unused_macros)]
pub fn matrix(
    #[values(0u8, 1, 2, 3, 4, 5)] baud_idx: u8,
    #[values(0u32, 128, 256, 384, 510)] rdt_us: u32,
) {
}

/// Baud-only sweep template for tests whose behavior is RDT-independent
/// by spec (e.g. broadcast Ping sources its RDT from the driver default,
/// not the per-servo register).
#[template]
#[rstest]
#[allow(dead_code, unused_macros)]
pub fn baud_matrix(#[values(0u8, 1, 2, 3, 4, 5)] baud_idx: u8) {}

/// Clear host logs, broadcast Ping, and assert every servo replied with an
/// OK status in chain order. Used as a recovery check at the end of tests
/// that exercise chain-collapse or error paths to confirm the bus didn't
/// lock up and no servo is stuck mid-parser.
#[allow(dead_code)]
pub fn assert_bus_healthy(sim: &mut Sim, host: DeviceId, servos: &[DeviceId]) {
    sim.host_mut(host).clear_logs();
    sim.with_host(host, |h| {
        h.send_ping(Id::BROADCAST);
        h.wait_for_reply();
    });

    let rx = sim.host(host).rx_bytes();
    let replies = parse_status_stream(Instruction::Ping, &rx);
    assert_eq!(
        replies.len(),
        servos.len(),
        "expected {} ping replies, got {}: {:?}",
        servos.len(),
        replies.len(),
        replies,
    );
    let expected_ids: Vec<u8> = servos
        .iter()
        .map(|d| sim.servo(*d).dxl_id().as_byte())
        .collect();
    for (reply, expected_id) in replies.iter().zip(expected_ids.iter()) {
        match reply {
            Status::Ping {
                id,
                error,
                status: PingStatus { .. },
            } => {
                assert_eq!(
                    id.as_byte(),
                    *expected_id,
                    "out-of-order ping reply: {:?}",
                    reply
                );
                assert_eq!(*error, StatusError::OK, "unhealthy reply: {:?}", reply);
            }
            other => panic!("expected Status::Ping, got {:?}", other),
        }
    }
}
