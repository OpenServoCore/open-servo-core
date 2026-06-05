use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::prelude::*;
use heapless::Vec;

use crate::traits::{DxlBus, Event, Schedule, ServiceEvents, ServicesIo};
use crate::{BootMode, RegionStorage, Shared, StatusReturnLevel};

use super::Dxl;
use super::osc::{CalibratePacket, OscExt, OscReplyExt, OscVariant};

/// Test-only `CrcUmts`. Production builds get the chip's impl directly; core
/// itself never references a concrete CRC engine after the trait flip.
struct TestDxlCrc;

const TEST_CRC_ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

impl CrcUmts for TestDxlCrc {
    fn accumulate(seed: u16, bytes: &[u8]) -> u16 {
        let mut digest = TEST_CRC_ENGINE.digest_with_initial(seed);
        digest.update(bytes);
        digest.finalize()
    }
}

type Wire = Codec<TestDxlCrc, OscExt, OscReplyExt>;

/// Compact summary of a `Reply` variant — the reply itself borrows from
/// dispatcher-stack storage, so tests inspect the kind here instead of cloning.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ReplyKind {
    /// Any Status-family reply (Ping/Read/SyncRead/BulkRead/Write/Ack/Error/etc.)
    Plain,
    /// FastSyncRead or FastBulkRead success — position carries packet_length.
    Fast(FastPosition),
    /// FastError — position + the chip-emitted zero-byte length.
    FastError(FastPosition, u16),
}

fn summarize(reply: &Reply<'_, OscReplyExt>) -> ReplyKind {
    match *reply {
        Reply::FastSyncRead { position, .. } | Reply::FastBulkRead { position, .. } => {
            ReplyKind::Fast(position)
        }
        Reply::FastError {
            position, length, ..
        } => ReplyKind::FastError(position, length),
        _ => ReplyKind::Plain,
    }
}

struct FakeBus {
    burst: Vec<u8, 256>,
    burst_fresh: bool,
    /// Wire bytes produced by the last `send` call — overwritten each send.
    tx: Vec<u8, 256>,
    /// Total `send` calls observed; tests assert silence with `send_count == 0`.
    send_count: u32,
    last_schedule: Option<Schedule>,
    last_kind: Option<ReplyKind>,
}

impl FakeBus {
    fn new() -> Self {
        Self {
            burst: Vec::new(),
            burst_fresh: false,
            tx: Vec::new(),
            send_count: 0,
            last_schedule: None,
            last_kind: None,
        }
    }

    /// Stash bytes as the next IDLE-anchored burst — `poll` returns the wire-
    /// end frame once, then `None` until the next `feed`.
    fn feed(&mut self, bytes: &[u8]) {
        self.burst.clear();
        self.burst.extend_from_slice(bytes).unwrap();
        self.burst_fresh = true;
    }

    /// No-op stand-in for the bus-collision / no-IDLE-gap case: bytes drop,
    /// `poll` returns `None`, the parser never runs.
    fn feed_no_idle(&mut self, _bytes: &[u8]) {}
}

impl DxlBus for FakeBus {
    fn poll(&mut self) -> Option<Packet<'static, OscExt>> {
        if !self.burst_fresh {
            return None;
        }
        self.burst_fresh = false;
        let s: &[u8] = &self.burst[..];
        // SAFETY: each test holds FakeBus for the full poll() call, so the
        // burst storage outlives the returned slice. Production contract
        // ("Packet bytes valid until next poll") is respected here too.
        let window: &'static [u8] = unsafe { core::slice::from_raw_parts(s.as_ptr(), s.len()) };
        // Mirror chip-side walk: parse forward, dispatch the frame ending
        // exactly at the wire-end; earlier frames advance the cursor.
        let n = window.len();
        let mut offset = 0;
        while offset < n {
            match Wire::parse_one(&window[offset..], &[]) {
                Ok((pkt, used)) => {
                    if offset + used == n {
                        return Some(pkt);
                    }
                    offset += used;
                }
                Err(ParseError::Incomplete) => return None,
                Err(ParseError::Resync { skip })
                | Err(ParseError::BadCrc { skip })
                | Err(ParseError::BadInstruction { skip })
                | Err(ParseError::BadLength { skip }) => {
                    offset = (offset + skip).min(n);
                }
            }
        }
        None
    }

    fn send(&mut self, reply: Reply<'_, OscReplyExt>, schedule: Schedule) {
        self.tx.clear();
        Wire::write_reply(&mut self.tx, &reply).unwrap();
        self.send_count += 1;
        self.last_schedule = Some(schedule);
        self.last_kind = Some(summarize(&reply));
    }
}

struct FakeEvents {
    reboot_count: u32,
    last_reboot_mode: Option<BootMode>,
}

impl FakeEvents {
    fn new() -> Self {
        Self {
            reboot_count: 0,
            last_reboot_mode: None,
        }
    }
}

impl ServiceEvents for FakeEvents {
    fn send(&mut self, event: Event) {
        match event {
            Event::Reboot(mode) => {
                self.reboot_count += 1;
                self.last_reboot_mode = Some(mode);
            }
            Event::SetDxlBaud(_) | Event::SetClockTrim(_) | Event::SetClockFineTrimUs(_) => {}
        }
    }
}

struct FakeIo {
    bus: FakeBus,
    events: FakeEvents,
}

impl FakeIo {
    fn new() -> Self {
        Self {
            bus: FakeBus::new(),
            events: FakeEvents::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        self.bus.feed(bytes);
    }
}

impl ServicesIo for FakeIo {
    type Bus = FakeBus;
    type Events = FakeEvents;

    fn parts(&mut self) -> (&mut FakeBus, &mut FakeEvents) {
        (&mut self.bus, &mut self.events)
    }
}

fn encode(packet: &Packet<'_, OscExt>) -> Vec<u8, 64> {
    let mut buf: Vec<u8, 64> = Vec::new();
    Wire::write(&mut buf, packet).unwrap();
    buf
}

fn parse_status(bytes: &[u8]) -> (u8, u8, Vec<u8, 64>) {
    let (pkt, used) = Wire::parse_one(bytes, &[]).unwrap();
    assert_eq!(used, bytes.len());
    match pkt {
        Packet::Status(p) => {
            let mut out: Vec<u8, 64> = Vec::new();
            for b in p.params.iter() {
                out.push(b).unwrap();
            }
            (p.id, p.error, out)
        }
        _ => panic!("not a status packet"),
    }
}

#[test]
fn ping_to_our_id_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Plain));
}

#[test]
fn ping_to_broadcast_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, _) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_other_id_silent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(17)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn read_model_number_returns_two_bytes() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn read_zero_length_rejects_with_data_range() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn unsupported_instruction_replies_instruction_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Instruction.as_u8());
}

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Access.as_u8());
    let identity = shared.table.config.with(|c| c.identity);
    assert_eq!(identity.model_number, 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0xFFFE, &[0x01])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        17,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(!lc.torque_enable);
}

#[test]
fn broadcast_write_applies_but_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn reg_write_then_action_commits_to_live_table() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error_immediately() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::Access.as_u8());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[2],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(&Packet::Write(WritePacket::new(
        0,
        CONTROL_BASE_ADDR + 1,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(&Packet::Action(ActionPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn action_with_empty_staging_replies_ok() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::None.as_u8());
}

#[test]
fn reboot_to_our_id_acks_and_calls_device_reboot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(io.events.reboot_count, 1);
    assert_eq!(io.events.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_fires_device_reboot_without_ack() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(BROADCAST_ID)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.events.reboot_count, 1);
    assert_eq!(io.events.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(17)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.events.reboot_count, 0);
    assert_eq!(io.events.last_reboot_mode, None);
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(
        0,
        BOOT_MODE,
        &[BootMode::Bootloader as u8],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.events.reboot_count, 1);
    assert_eq!(io.events.last_reboot_mode, Some(BootMode::Bootloader));
}

fn set_level(shared: &Shared, level: StatusReturnLevel) {
    shared
        .table
        .config
        .with_mut(|c| c.comms.status_return_level = level);
}

#[test]
fn return_level_none_silences_write_ack_but_ping_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_read_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_read_silences_write_ack_but_read_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, CONTROL_BASE_ADDR, &[1])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(&Packet::Read(ReadPacket::new(0, 0, 2)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_write_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Write(WritePacket::new(0, 0, &[0xAA, 0xBB])));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_unsupported_instruction_error() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FactoryReset(FactoryResetPacket::new(0, 0xFF)));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_action_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::RegWrite(RegWritePacket::new(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(&Packet::Action(ActionPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn return_level_none_silences_reboot_ack_but_reboot_still_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Reboot(RebootPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert_eq!(io.events.reboot_count, 1);
    assert_eq!(io.events.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn return_level_none_still_replies_to_ping() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ping(PingPacket::new(0)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_replies_to_read_errors() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Read(ReadPacket::new(0, 0xFFFE, 1)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_in_slot_zero_replies_with_zero_offset() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let s = io.bus.last_schedule.unwrap();
    assert_eq!(s.bytes_before, 0);
    assert_eq!(s.slot_index, 0);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn sync_read_in_later_slot_carries_bytes_before_and_slot_index() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // ids = [9, 7, 0] → our slot index = 2.
    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[9, 7, 0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let s = io.bus.last_schedule.unwrap();
    // bytes_before = 2 × (RESPONSE_HEADER_BYTES(9) + 2 + CRC_BYTES(2)) = 26
    assert_eq!(s.bytes_before, 26);
    assert_eq!(s.slot_index, 2);
}

#[test]
fn sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[5, 7, 9])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.bus.feed_no_idle(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn sync_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0xFFFE, 1, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::SyncRead(SyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_in_slot_zero_replies_with_zero_offset() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let s = io.bus.last_schedule.unwrap();
    assert_eq!(s.bytes_before, 0);
    assert_eq!(s.slot_index, 0);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_in_later_slot_sums_preceding_slot_widths() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // slot 0 id=9 len=4, slot 1 id=7 len=8, slot 2 id=0 len=2.
    let body = [9, 0, 0, 4, 0, 7, 0, 0, 8, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let s = io.bus.last_schedule.unwrap();
    // bytes_before = (9+4+2) + (9+8+2) = 34
    assert_eq!(s.bytes_before, 34);
    assert_eq!(s.slot_index, 2);
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.bus.feed_no_idle(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_data_range_error_still_replies_in_slot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0xFE, 0xFF, 1, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::DataRange.as_u8());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::BulkRead(BulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_only_slot_emits_full_frame_with_local_crc() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(FastPosition::Only { packet_length })) => packet_length,
        other => panic!("expected Fast(Only{{..}}), got {other:?}"),
    };
    // LEN = 3 + 1*(2+2) = 7 for 1-slot, 2-byte payload.
    assert_eq!(pkt_len, 7);

    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[7, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(io.bus.tx[8], 0);
    assert_eq!(io.bus.tx[9], 0);
    assert_eq!(&io.bus.tx[10..12], &[0, 0]);
    let expected_crc = TestDxlCrc::accumulate(0, &io.bus.tx[..12]).to_le_bytes();
    assert_eq!(&io.bus.tx[12..14], &expected_crc);
}

#[test]
fn fast_sync_read_first_slot_emits_header_then_body_no_crc() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[0, 7],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(FastPosition::First { packet_length })) => packet_length,
        other => panic!("expected Fast(First{{..}}), got {other:?}"),
    };
    assert_eq!(pkt_len, 11);
    assert_eq!(io.bus.last_schedule.unwrap().bytes_before, 0);

    assert_eq!(io.bus.tx.len(), 12);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[11, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(&io.bus.tx[8..12], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_middle_slot_emits_body_only_with_bytes_before() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let p = FastSyncReadPacket::new(0, 2, &[9, 0, 7]);
    let req = encode(&Packet::FastSyncRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(
        io.bus.last_kind,
        Some(ReplyKind::Fast(FastPosition::Middle))
    );
    // Slot 1: FAST_RESPONSE_SLOT0_BYTES(10) + payload(2) = 12 bytes before.
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        p.find_slot(0, 32).unwrap().bytes_before
    );

    assert_eq!(io.bus.tx.len(), 4);
    assert_eq!(&io.bus.tx[..], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_last_slot_reserves_crc_placeholder() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let p = FastSyncReadPacket::new(0, 2, &[9, 0]);
    let req = encode(&Packet::FastSyncRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(FastPosition::Last)));
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        p.find_slot(0, 32).unwrap().bytes_before
    );

    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0,
        2,
        &[5, 7, 9],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.bus.feed_no_idle(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(0, 2, &[0])));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_only_slot_emits_full_frame_with_local_crc() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(FastPosition::Only { packet_length })) => packet_length,
        other => panic!("expected Fast(Only{{..}}), got {other:?}"),
    };
    assert_eq!(pkt_len, 7);

    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[7, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(io.bus.tx[8], 0);
    assert_eq!(io.bus.tx[9], 0);
    assert_eq!(&io.bus.tx[10..12], &[0, 0]);
    let expected_crc = TestDxlCrc::accumulate(0, &io.bus.tx[..12]).to_le_bytes();
    assert_eq!(&io.bus.tx[12..14], &expected_crc);
}

#[test]
fn fast_bulk_read_first_slot_emits_header_then_body() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0, 7, 0, 0, 4, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(FastPosition::First { packet_length })) => packet_length,
        other => panic!("expected Fast(First{{..}}), got {other:?}"),
    };
    assert_eq!(pkt_len, 13);

    assert_eq!(io.bus.tx.len(), 12);
    assert_eq!(&io.bus.tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0xFE]);
    assert_eq!(&io.bus.tx[5..7], &[13, 0]);
    assert_eq!(io.bus.tx[7], 0x55);
    assert_eq!(&io.bus.tx[8..12], &[0, 0, 0, 0]);
}

#[test]
fn fast_bulk_read_middle_slot_carries_bytes_before() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 0, 0, 0, 2, 0, 7, 0, 0, 8, 0];
    let p = FastBulkReadPacket::new(&body);
    let req = encode(&Packet::FastBulkRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(
        io.bus.last_kind,
        Some(ReplyKind::Fast(FastPosition::Middle))
    );
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        p.find_slot(0, 32).unwrap().bytes_before
    );
}

#[test]
fn fast_bulk_read_last_slot_reserves_crc_placeholder() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 0, 0, 0, 2, 0];
    let p = FastBulkReadPacket::new(&body);
    let req = encode(&Packet::FastBulkRead(p));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(FastPosition::Last)));
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        p.find_slot(0, 32).unwrap().bytes_before
    );

    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(FastPosition::Last)));
    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0xAA, 0xBB]);
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.bus.feed_no_idle(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(&Packet::FastBulkRead(FastBulkReadPacket::new(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn poll_recovers_from_stale_fast_first_slot_residue_then_replies_to_ping() {
    // Production wedge (V006, 2-3M baud, Fast Bulk Read last-slave with ≥32B
    // payload): a predecessor slave's Fast First slot reply (14 wire bytes —
    // header carries length=43 for the WHOLE multi-slot packet) lands in our
    // RX ring. parse_one wants 7+43=50 bytes and only 14 will ever arrive at
    // this offset (later slots are on the wire during *our* TX). Returning
    // Incomplete here strands every future request behind these 14 bytes.
    //
    // This integration test feeds the stale residue and then the Ping; the
    // dispatcher must recover and dispatch the Ping.
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let mut residue: Vec<u8, 32> = Vec::new();
    Wire::write_reply(
        &mut residue,
        &Reply::FastSyncRead {
            position: FastPosition::First { packet_length: 43 },
            id: 50,
            data: &[0xAA, 0xAA, 0xAA, 0xAA],
        },
    )
    .unwrap();
    assert_eq!(residue.len(), 14, "Fast First slot wire shape changed?");

    let ping = encode(&Packet::Ping(PingPacket::new(0)));

    io.feed(&residue);
    io.feed(&ping);
    h.poll(&shared, &mut io);

    assert_eq!(
        io.bus.send_count, 1,
        "Ping starved behind Fast First slot residue (parse_one Incomplete \
         wedged the poll loop)",
    );
    let (id, err, _) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn calibrate_unicast_replies_with_zero_payload() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ext(OscVariant::Calibrate(CalibratePacket::new(
        0, 16,
    ))));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(params.len(), 16);
    assert!(params.iter().all(|&b| b == 0));
}

#[test]
fn calibrate_unicast_count_max_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ext(OscVariant::Calibrate(CalibratePacket::new(
        0, 128,
    ))));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let tx = &io.bus.tx[..];
    // hdr(4) + id(1) + len(2) + inst(1) + err(1) + 128 zero bytes + crc(2)
    assert_eq!(tx.len(), 139);
    assert_eq!(&tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0]);
    assert_eq!(&tx[5..7], &[132, 0]);
    assert_eq!(tx[7], 0x55);
    assert_eq!(tx[8], 0);
    assert!(tx[9..137].iter().all(|&b| b == 0));
}

#[test]
fn calibrate_unicast_count_zero_data_range_err() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ext(OscVariant::Calibrate(CalibratePacket::new(
        0, 0,
    ))));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0x04);
    assert!(params.is_empty());
}

#[test]
fn calibrate_unicast_count_over_max_data_range_err() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ext(OscVariant::Calibrate(CalibratePacket::new(
        0, 129,
    ))));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, 0x04);
    assert!(params.is_empty());
}

#[test]
fn calibrate_broadcast_silently_dropped() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(&Packet::Ext(OscVariant::Calibrate(CalibratePacket::new(
        BROADCAST_ID,
        128,
    ))));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
}

#[test]
fn fast_sync_read_error_emits_zero_payload_with_error_byte() {
    // FastError path: control-table read fails → chip emits length zero
    // bytes for the data field rather than reading garbage off a stale buf.
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // Address 0xFFFE is out of range → Err(DataRange) from read_bytes.
    let req = encode(&Packet::FastSyncRead(FastSyncReadPacket::new(
        0xFFFE,
        2,
        &[0],
    )));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert!(matches!(
        io.bus.last_kind,
        Some(ReplyKind::FastError(FastPosition::Only { .. }, 2))
    ));
    // Wire bytes: HEADER(4) + 0xFE + LEN(2) + 0x55 + error + id + 2 zeros + CRC(2)
    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(io.bus.tx[8], StatusError::DataRange.as_u8());
    assert_eq!(io.bus.tx[9], 0); // our id
    assert_eq!(&io.bus.tx[10..12], &[0, 0]); // length zero bytes
}
