use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::decoder::{Decoder, Step};
use dxl_protocol::packet::{BulkReadEntry, ErrorCode, Packet, Slot, Status, StatusError};
use dxl_protocol::{
    BROADCAST_ID, CrcUmts, InstructionEmitter, SlotEmitter, SlotPosition, StatusEmitter, WriteError,
};
use heapless::Vec;

use crate::traits::{CalSnapshot, DxlBus, Event, Schedule, ServiceEvents, ServicesIo};
use crate::{BootMode, RegionStorage, Shared, StatusReturnLevel};

use super::Dxl;
use super::osc::CALIBRATE_INSTRUCTION;

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

/// Compact summary of the last `send` / `send_slot` call — the inputs
/// themselves borrow from dispatcher-stack storage, so tests inspect the kind
/// here instead of cloning.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ReplyKind {
    /// Any Status-family reply (Ping/Read/SyncRead/BulkRead/Write/Ack/Error/etc.)
    Plain,
    /// Fast Sync/Bulk Read slot — position carries packet_length when relevant.
    Fast(SlotPosition),
    /// Fast Read failure slot — position + the zero-payload byte count.
    FastError(SlotPosition, u16),
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
    /// Stub return for `cal_snapshot`. Tests installing a non-default value
    /// drive the dispatcher's CALIB measurement path.
    cal_snapshot_stub: Option<CalSnapshot>,
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
            cal_snapshot_stub: None,
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
    type Crc = TestDxlCrc;

    fn rx_window(&mut self) -> Option<(&[u8], &[u8])> {
        if !self.burst_fresh {
            return None;
        }
        self.burst_fresh = false;
        Some((&self.burst[..], &[]))
    }

    fn snoop(&mut self) {}

    fn send(&mut self, status: Status<'_>, schedule: Schedule) {
        self.tx.clear();
        StatusEmitter::<_, TestDxlCrc>::new(&mut self.tx)
            .emit(status)
            .unwrap();
        self.send_count += 1;
        self.last_schedule = Some(schedule);
        self.last_kind = Some(ReplyKind::Plain);
    }

    fn send_slot(&mut self, slot: Slot<'_>, position: SlotPosition, schedule: Schedule) {
        self.tx.clear();
        SlotEmitter::<_, TestDxlCrc>::new(&mut self.tx)
            .emit(&slot, position)
            .unwrap();
        self.send_count += 1;
        self.last_schedule = Some(schedule);
        let length = slot.data.len() as u16;
        self.last_kind = Some(if slot.error == StatusError::OK {
            ReplyKind::Fast(position)
        } else {
            ReplyKind::FastError(position, length)
        });
    }

    fn cal_snapshot(&mut self) -> Option<CalSnapshot> {
        self.cal_snapshot_stub
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

fn encode<F>(f: F) -> Vec<u8, 256>
where
    F: FnOnce(
        &mut InstructionEmitter<'_, Vec<u8, 256>, TestDxlCrc>,
    ) -> Result<(), WriteError>,
{
    let mut buf: Vec<u8, 256> = Vec::new();
    f(&mut InstructionEmitter::<_, TestDxlCrc>::new(&mut buf)).unwrap();
    buf
}

fn encode_calibrate(id: u8, count: u16) -> Vec<u8, 256> {
    let mut params: Vec<u8, 256> = Vec::new();
    params.extend_from_slice(&count.to_le_bytes()).unwrap();
    for _ in 0..count {
        params.push(0).unwrap();
    }
    encode(|w| w.ext(id, CALIBRATE_INSTRUCTION, &params))
}

/// Cast a 5-byte-stride byte buffer to a typed `[BulkReadEntry]` slice.
/// `BulkReadEntry` is `#[repr(C)]` with align 1 (`U16Le` fields) — sound
/// under any alignment.
fn bulk_entries(body: &[u8]) -> &[BulkReadEntry] {
    assert!(body.len() % 5 == 0);
    unsafe {
        core::slice::from_raw_parts(body.as_ptr() as *const BulkReadEntry, body.len() / 5)
    }
}

fn parse_status(bytes: &[u8]) -> (u8, u8, Vec<u8, 64>) {
    let mut dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let (step, used) = dec.feed(bytes);
    assert_eq!(used, bytes.len());
    match step {
        Step::Packet(Packet::Status(p)) => {
            let id = p.header.header.id;
            let err = p.header.error;
            let mut out: Vec<u8, 64> = Vec::new();
            out.extend_from_slice(p.params).unwrap();
            (id, err, out)
        }
        _ => panic!("not a status packet"),
    }
}


#[test]
fn ping_to_our_id_replies() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(0));
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

    let req = encode(|w| w.ping(BROADCAST_ID));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, _) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_broadcast_uses_id_indexed_slot() {
    // Spec convention: each slave fires its broadcast-Ping reply in an ID-
    // indexed time slot (slot width ≥ one Ping Status frame = 14 B). With
    // our_id = 7, the schedule must reflect 7 × 14 = 98 bytes_before and
    // slot_index = 7 so the chip's per-slot margin applies.
    let shared = Shared::new();
    shared.table.config.with_mut(|c| c.comms.id = 7);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(BROADCAST_ID));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let s = io.bus.last_schedule.unwrap();
    assert_eq!(s.bytes_before, 7 * 14);
    assert_eq!(s.slot_index, 7);
}

#[test]
fn ping_to_other_id_silent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(17));
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

    let req = encode(|w| w.read(0, 0, 2));
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

    let req = encode(|w| w.read(0, 0, 0));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn unsupported_instruction_replies_instruction_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.factory_reset(0, 0xFF));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Instruction).as_byte());
}

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(0, CONTROL_BASE_ADDR, &[1]));
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

    let req = encode(|w| w.write(0, 0, &[0xAA, 0xBB]));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
    let identity = shared.table.config.with(|c| c.identity);
    assert_eq!(identity.model_number, 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range_error() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(0, 0xFFFE, &[0x01]));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(
        17,
        CONTROL_BASE_ADDR,
        &[1],
    ));
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

    let req = encode(|w| w.write(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    ));
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

    let req = encode(|w| w.reg_write(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.action(0));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error_immediately() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(0, 0, &[0xAA, 0xBB]));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(
        0,
        CONTROL_BASE_ADDR,
        &[2],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());

    let req = encode(|w| w.action(0));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(|w| w.write(
        0,
        CONTROL_BASE_ADDR + 1,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );

    let req = encode(|w| w.action(0));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(
        BROADCAST_ID,
        CONTROL_BASE_ADDR,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(|w| w.action(BROADCAST_ID));
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

    let req = encode(|w| w.action(0));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn reboot_to_our_id_acks_and_calls_device_reboot() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(0));
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

    let req = encode(|w| w.reboot(BROADCAST_ID));
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

    let req = encode(|w| w.reboot(17));
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

    let req = encode(|w| w.write(
        0,
        BOOT_MODE,
        &[BootMode::Bootloader as u8],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(|w| w.reboot(0));
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

    let req = encode(|w| w.write(0, CONTROL_BASE_ADDR, &[1]));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);
    assert!(io.bus.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.ping(0));
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

    let req = encode(|w| w.read(0, 0, 2));
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

    let req = encode(|w| w.write(0, CONTROL_BASE_ADDR, &[1]));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(io.bus.send_count, 0);

    let req = encode(|w| w.read(0, 0, 2));
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

    let req = encode(|w| w.write(0, 0, &[0xAA, 0xBB]));
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

    let req = encode(|w| w.factory_reset(0, 0xFF));
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

    let req = encode(|w| w.reg_write(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let req = encode(|w| w.action(0));
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

    let req = encode(|w| w.reboot(0));
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

    let req = encode(|w| w.ping(0));
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

    let req = encode(|w| w.read(0, 0xFFFE, 1));
    io.feed(&req);
    h.poll(&shared, &mut io);

    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn sync_read_in_slot_zero_replies_with_zero_offset() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
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
    let req = encode(|w| w.sync_read(0, 2, &[9, 7, 0]));
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

    let req = encode(|w| w.sync_read(0, 2, &[5, 7, 9]));
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

    let req = encode(|w| w.sync_read(0, 2, &[0]));
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

    let req = encode(|w| w.sync_read(0xFFFE, 1, &[0]));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
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

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(SlotPosition::Only { packet_length })) => packet_length,
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

    let req = encode(|w| w.fast_sync_read(
        0,
        2,
        &[0, 7],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(SlotPosition::First { packet_length })) => packet_length,
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

    let req = encode(|w| w.fast_sync_read(0, 2, &[9, 0, 7]));
    let mut __dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let __expected_bytes_before = match __dec.feed(&req).0 {
        Step::Packet(Packet::FastSyncRead(p)) => p.find_slot(0, 32).unwrap().bytes_before,
        _ => panic!("expected FastSyncRead"),
    };
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(
        io.bus.last_kind,
        Some(ReplyKind::Fast(SlotPosition::Middle))
    );
    // Slot 1: FAST_RESPONSE_SLOT0_BYTES(10) + payload(2) = 12 bytes before.
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        __expected_bytes_before
    );

    assert_eq!(io.bus.tx.len(), 4);
    assert_eq!(&io.bus.tx[..], &[0, 0, 0, 0]);
}

#[test]
fn fast_sync_read_last_slot_reserves_crc_placeholder() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[9, 0]));
    let mut __dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let __expected_bytes_before = match __dec.feed(&req).0 {
        Step::Packet(Packet::FastSyncRead(p)) => p.find_slot(0, 32).unwrap().bytes_before,
        _ => panic!("expected FastSyncRead"),
    };
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(SlotPosition::Last { crc: 0 })));
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        __expected_bytes_before
    );

    // Trailing 2 bytes are the reserved chain-CRC slot. The chip's
    // `patch_crc` ISR overwrites them at fire time; FakeBus mirrors the
    // chip by emitting a zero sentinel here.
    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0, 0]);
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(
        0,
        2,
        &[5, 7, 9],
    ));
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

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
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

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
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
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(SlotPosition::Only { packet_length })) => packet_length,
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
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let pkt_len = match io.bus.last_kind {
        Some(ReplyKind::Fast(SlotPosition::First { packet_length })) => packet_length,
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
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    let mut __dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let __expected_bytes_before = match __dec.feed(&req).0 {
        Step::Packet(Packet::FastBulkRead(p)) => p.find_slot(0, 32).unwrap().bytes_before,
        _ => panic!("expected FastBulkRead"),
    };
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(
        io.bus.last_kind,
        Some(ReplyKind::Fast(SlotPosition::Middle))
    );
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        __expected_bytes_before
    );
}

#[test]
fn fast_bulk_read_last_slot_reserves_crc_placeholder() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0, 0, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    let mut __dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let __expected_bytes_before = match __dec.feed(&req).0 {
        Step::Packet(Packet::FastBulkRead(p)) => p.find_slot(0, 32).unwrap().bytes_before,
        _ => panic!("expected FastBulkRead"),
    };
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(SlotPosition::Last { crc: 0 })));
    assert_eq!(
        io.bus.last_schedule.unwrap().bytes_before,
        __expected_bytes_before
    );

    // Trailing 2 bytes are the reserved chain-CRC slot. The chip's
    // `patch_crc` ISR overwrites them at fire time; FakeBus mirrors the
    // chip by emitting a zero sentinel here.
    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0, 0]);
}

#[test]
fn fast_bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert_eq!(io.bus.last_kind, Some(ReplyKind::Fast(SlotPosition::Last { crc: 0 })));
    assert_eq!(io.bus.tx.len(), 6);
    assert_eq!(&io.bus.tx[..4], &[0, 0, 0, 0]);
    assert_eq!(&io.bus.tx[4..], &[0, 0]);
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
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
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
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
    SlotEmitter::<_, TestDxlCrc>::new(&mut residue)
        .first(
            &Slot {
                id: 50,
                error: StatusError::OK,
                data: &[0xAA, 0xAA, 0xAA, 0xAA],
            },
            43,
        )
        .unwrap();
    assert_eq!(residue.len(), 14, "Fast First slot wire shape changed?");

    let ping = encode(|w| w.ping(0));

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
fn calibrate_unicast_replies_with_measurement_payload() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();
    // 1 Mbps wire: byte_time at 48 MHz HCLK ≈ 480 ticks.
    // Chip pre-computed nominal_ticks and the most-recent batched apply;
    // dispatcher just forwards. Observed slightly above nominal models a
    // fast slave.
    io.bus.cal_snapshot_stub = Some(CalSnapshot {
        observed_ticks: 13_440,
        nominal_ticks: 27 * 480, // (12 + 16 - 1) × byte_time_ticks
        applied_trim_delta: -1,
        applied_fine_trim_us: 256, // +1.0 µs in Q8.8
    });

    let req = encode_calibrate(
        0, 16,
    );
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (id, err, params) = parse_status(&io.bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    // observed(4) + nominal(4) + trim(1) + fine(2).
    assert_eq!(params.len(), 11);
    let observed = u32::from_le_bytes([params[0], params[1], params[2], params[3]]);
    let nominal = u32::from_le_bytes([params[4], params[5], params[6], params[7]]);
    assert_eq!(observed, 13_440);
    assert_eq!(nominal, 27 * 480);
    assert_eq!(params[8] as i8, -1);
    assert_eq!(i16::from_le_bytes([params[9], params[10]]), 256);
}

#[test]
fn calibrate_unicast_count_max_payload_accepted() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();
    io.bus.cal_snapshot_stub = Some(CalSnapshot {
        observed_ticks: 0,
        nominal_ticks: 139 * 480, // (12 + 128 - 1) × byte_time_ticks
        applied_trim_delta: 0,
        applied_fine_trim_us: 0,
    });

    let req = encode_calibrate(
        0, 128,
    );
    // hdr(4) + id(1) + len(2) + inst(1) + count(2) + 128 filler + crc(2) = 140
    assert_eq!(req.len(), 140);
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let tx = &io.bus.tx[..];
    // hdr(4) + id(1) + len(2) + inst(1=Status) + err(1) + measurement(11) + crc(2) = 22
    assert_eq!(tx.len(), 22);
    assert_eq!(&tx[..5], &[0xFF, 0xFF, 0xFD, 0x00, 0]);
    assert_eq!(&tx[5..7], &[15, 0]); // length = inst + err + 11 + crc(2) = 15
    assert_eq!(tx[7], 0x55);
    assert_eq!(tx[8], 0);
    let observed = u32::from_le_bytes([tx[9], tx[10], tx[11], tx[12]]);
    let nominal = u32::from_le_bytes([tx[13], tx[14], tx[15], tx[16]]);
    assert_eq!(observed, 0);
    assert_eq!(nominal, 139 * 480);
}

#[test]
fn calibrate_without_snapshot_replies_data_range_error() {
    // Chip-side first-byte tick wasn't captured (e.g. RXNE masked during a
    // chain or pre-arm boot window); dispatcher must surface the failure.
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();
    assert!(io.bus.cal_snapshot_stub.is_none());

    let req = encode_calibrate(
        0, 16,
    );
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    let (_, err, params) = parse_status(&io.bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
    assert!(params.is_empty());
}

#[test]
fn calibrate_unicast_count_zero_data_range_err() {
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode_calibrate(
        0, 0,
    );
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

    let req = encode_calibrate(
        0, 129,
    );
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

    let req = encode_calibrate(
        BROADCAST_ID,
        128,
    );
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
    let req = encode(|w| w.fast_sync_read(
        0xFFFE,
        2,
        &[0],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 1);
    assert!(matches!(
        io.bus.last_kind,
        Some(ReplyKind::FastError(SlotPosition::Only { .. }, 2))
    ));
    // Wire bytes: HEADER(4) + 0xFE + LEN(2) + 0x55 + error + id + 2 zeros + CRC(2)
    assert_eq!(io.bus.tx.len(), 14);
    assert_eq!(io.bus.tx[8], StatusError::code(ErrorCode::DataRange).as_byte());
    assert_eq!(io.bus.tx[9], 0); // our id
    assert_eq!(&io.bus.tx[10..12], &[0, 0]); // length zero bytes
}

#[test]
fn sync_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // Two slots: id=7 (not us), id=0 (us). length=1, both write 0x01 to torque_enable.
    let body = [7, 0x00, 0, 0x01];
    let req = encode(|w| w.sync_write(
        CONTROL_BASE_ADDR,
        1,
        &body,
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_to_other_ids_only_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let body = [7, 0x01, 17, 0x01];
    let req = encode(|w| w.sync_write(
        CONTROL_BASE_ADDR,
        1,
        &body,
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging_too() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(
        0,
        CONTROL_BASE_ADDR,
        &[1],
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);

    // Sync Write to mode register — should wipe the RegWrite staging.
    let body = [0, Mode::PositionPid as u8];
    let req = encode(|w| w.sync_write(
        CONTROL_BASE_ADDR + 1,
        1,
        &body,
    ));
    io.feed(&req);
    h.poll(&shared, &mut io);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );

    let req = encode(|w| w.action(0));
    io.feed(&req);
    h.poll(&shared, &mut io);
    let (_, err, _) = parse_status(&io.bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // id=0, address=CONTROL_BASE_ADDR (torque_enable), length=1, data=[1].
    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [0, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [17, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_uses_our_tuples_address_not_a_preceding_slots() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut io = FakeIo::new();
    let mut h = Dxl::new();

    // Slot for id=7 (foreign) targets torque_enable; our slot (id=0) targets
    // the mode register one byte later. Verify we honor our own address.
    let other_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let other_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let mode_addr = CONTROL_BASE_ADDR + 1;
    let our_lo = (mode_addr & 0xFF) as u8;
    let our_hi = (mode_addr >> 8) as u8;
    let body = [
        7, other_lo, other_hi, 1, 0, 1, //
        0, our_lo, our_hi, 1, 0, Mode::PositionPid as u8,
    ];
    let req = encode(|w| w.bulk_write(&body));
    io.feed(&req);
    h.poll(&shared, &mut io);

    assert_eq!(io.bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
}
