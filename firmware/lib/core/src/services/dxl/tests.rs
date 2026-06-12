use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::decoder::{Decoder, Step};
use dxl_protocol::packet::{BulkReadEntry, ErrorCode, Id, Packet, Slot, Status, StatusError};
use dxl_protocol::{
    CrcUmts, InstructionEmitter, InstructionPacket, SlotEmitter, StatusEmitter, WriteError,
};
use heapless::Vec;

const BROADCAST_ID: Id = Id::BROADCAST;

use crate::regions::config::BaudRate;
use crate::traits::{DxlBus, DxlReply};
use crate::{BootMode, RegionStorage, Shared, StatusReturnLevel};

use super::Dxl;

/// Test-only `CrcUmts`. Production builds get the chip's impl directly; core
/// itself never references a concrete CRC engine after the trait flip.
struct TestDxlCrc {
    state: u16,
}

const TEST_CRC_ENGINE: Crc<u16> = Crc::<u16>::new(&CRC_16_UMTS);

impl CrcUmts for TestDxlCrc {
    fn new() -> Self {
        Self { state: 0 }
    }

    fn update(&mut self, bytes: &[u8]) {
        let mut digest = TEST_CRC_ENGINE.digest_with_initial(self.state);
        digest.update(bytes);
        self.state = digest.finalize();
    }

    fn finalize(&self) -> u16 {
        self.state
    }

    fn reset(&mut self) {
        self.state = 0;
    }
}

/// Compact summary of the last `send_status` / `send_slot` call. Slot
/// positioning is no longer on the dispatcher path (driver computes it from
/// the cached request — see `firmware/lib/drivers/src/dxl/uart/mod.rs::ReplyContext`),
/// so the kind here only distinguishes Plain Status from Fast Slot. Position
/// assertions live in driver-crate tests.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ReplyKind {
    Plain,
    Slot,
}

/// FakeBus impls both [`DxlBus`] and [`DxlReply`]. The decoder is local to
/// `poll<F>` so the parsed packet borrows from that local — leaving `&mut
/// self` free for the closure's `&mut dyn DxlReply` handle. The recording
/// fields live as plain `pub` on `FakeBus` for ergonomic test access.
struct FakeBus {
    /// Bytes the next `poll` will hand to a local decoder. Stashed by
    /// [`Self::feed`]; cleared (`burst_fresh = false`) by `poll`.
    burst: Vec<u8, 256>,
    burst_fresh: bool,

    /// Most recent encoded TX bytes — overwritten each `send_status` /
    /// `send_slot`.
    pub tx: Vec<u8, 256>,
    pub send_count: u32,
    pub last_kind: Option<ReplyKind>,

    pub last_id_staged: Option<u8>,
    pub last_baud_staged: Option<BaudRate>,
    pub last_rdt_staged: Option<u32>,
    pub last_reboot_mode: Option<BootMode>,
    pub reboot_count: u32,
}

impl FakeBus {
    fn new() -> Self {
        Self {
            burst: Vec::new(),
            burst_fresh: false,
            tx: Vec::new(),
            send_count: 0,
            last_kind: None,
            last_id_staged: None,
            last_baud_staged: None,
            last_rdt_staged: None,
            last_reboot_mode: None,
            reboot_count: 0,
        }
    }

    /// Stash bytes as the next burst — `poll` decodes them once, then
    /// `burst_fresh` clears so a second `poll` without re-feeding is a
    /// no-op (the closure never fires).
    fn feed(&mut self, bytes: &[u8]) {
        self.burst.clear();
        self.burst.extend_from_slice(bytes).unwrap();
        self.burst_fresh = true;
    }

    /// Stand-in for the bus-collision / no-IDLE case: bytes drop, `poll`
    /// surfaces nothing.
    fn feed_no_idle(&mut self, _bytes: &[u8]) {}
}

impl DxlBus for FakeBus {
    fn poll<F>(&mut self, f: F)
    where
        F: for<'a> FnOnce(InstructionPacket<'a>, &mut dyn DxlReply),
    {
        if !self.burst_fresh {
            return;
        }
        self.burst_fresh = false;
        let mut decoder: Decoder<256, TestDxlCrc> = Decoder::new();
        let (step, _) = decoder.feed(&self.burst);
        if !matches!(step, Step::Packet(_)) {
            return;
        }
        // Packet borrows the local `decoder` (Decoder::dispatch_packet is
        // `&self`), NOT `self`. `&mut self` is then free for the closure's
        // DxlReply handle — `Self: DxlReply` coerces implicitly.
        let Some(packet) = decoder.dispatch_packet().into_instruction_packet() else {
            return;
        };
        f(packet, self);
    }
}

impl DxlReply for FakeBus {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx.clear();
        StatusEmitter::<_, TestDxlCrc>::new(&mut self.tx).emit(status)?;
        self.send_count += 1;
        self.last_kind = Some(ReplyKind::Plain);
        Ok(())
    }

    fn send_slot(&mut self, slot: &Slot<'_>) -> Result<(), WriteError> {
        // Position is a driver-side concern — encode as `Only` to keep the
        // round-trip wire bytes well-formed for assertions that inspect the
        // encoded payload.
        self.tx.clear();
        let len = (3 + slot.data.len() + 2) as u16;
        SlotEmitter::<_, TestDxlCrc>::new(&mut self.tx).emit(
            slot,
            dxl_protocol::SlotPosition::Only { packet_length: len },
        )?;
        self.send_count += 1;
        self.last_kind = Some(ReplyKind::Slot);
        Ok(())
    }

    fn stage_id(&mut self, id: u8) {
        self.last_id_staged = Some(id);
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        self.last_baud_staged = Some(baud);
    }

    fn stage_rdt(&mut self, us: u32) {
        self.last_rdt_staged = Some(us);
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        self.reboot_count += 1;
        self.last_reboot_mode = Some(mode);
    }
}

fn encode<F>(f: F) -> Vec<u8, 256>
where
    F: FnOnce(&mut InstructionEmitter<'_, Vec<u8, 256>, TestDxlCrc>) -> Result<(), WriteError>,
{
    let mut buf: Vec<u8, 256> = Vec::new();
    f(&mut InstructionEmitter::<_, TestDxlCrc>::new(&mut buf)).unwrap();
    buf
}

/// Cast a 5-byte-stride byte buffer to a typed `[BulkReadEntry]` slice.
/// `BulkReadEntry` is `#[repr(C)]` with align 1 (`U16Le` fields) — sound
/// under any alignment.
fn bulk_entries(body: &[u8]) -> &[BulkReadEntry] {
    assert!(body.len().is_multiple_of(5));
    unsafe { core::slice::from_raw_parts(body.as_ptr() as *const BulkReadEntry, body.len() / 5) }
}

fn parse_status(bytes: &[u8]) -> (u8, u8, Vec<u8, 64>) {
    let mut dec: Decoder<256, TestDxlCrc> = Decoder::new();
    let (step, used) = dec.feed(bytes);
    assert_eq!(used, bytes.len());
    match step {
        Step::Packet(Packet::Status(p)) => {
            let id = p.header.header.id.as_byte();
            let err = p.header.error.as_byte();
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
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
    assert_eq!(bus.last_kind, Some(ReplyKind::Plain));
}

#[test]
fn ping_to_broadcast_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (id, err, _) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_other_id_silent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(17)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn read_model_number_returns_two_bytes() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn read_zero_length_rejects_with_data_range() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 0));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn unsupported_instruction_replies_instruction_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.factory_reset(Id::new(0), 0xFF));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Instruction).as_byte());
}

#[test]
fn write_to_rw_address_succeeds_and_mutates() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn write_to_ro_address_replies_access_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
    let identity = shared.table.config.with(|c| c.identity);
    assert_eq!(identity.model_number, 0);
}

#[test]
fn write_to_unmapped_address_replies_data_range_error() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0xFFFE, &[0x01]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(17), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(!lc.torque_enable);
}

#[test]
fn broadcast_write_applies_but_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(BROADCAST_ID, CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    let lc = shared.table.control.with(|c| c.lifecycle);
    assert!(lc.torque_enable);
}

#[test]
fn reg_write_then_action_commits_to_live_table() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn reg_write_to_ro_address_replies_access_error_immediately() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::Access).as_byte());
}

#[test]
fn reg_write_invalid_value_rejected_at_stage_time() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[2]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR + 1, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn broadcast_reg_write_and_action_silent_but_commits() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(BROADCAST_ID, CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);

    let req = encode(|w| w.action(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn action_with_empty_staging_replies_ok() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn reboot_to_our_id_acks_and_stages_reboot() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(bus.reboot_count, 1);
    assert_eq!(bus.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_stages_reboot_without_ack() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
    assert_eq!(bus.reboot_count, 1);
    assert_eq!(bus.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(17)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
    assert_eq!(bus.reboot_count, 0);
    assert_eq!(bus.last_reboot_mode, None);
}

#[test]
fn reboot_honors_staged_boot_mode() {
    use crate::regions::control::addr::system::BOOT_MODE;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), BOOT_MODE, &[BootMode::Bootloader as u8]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reboot_count, 1);
    assert_eq!(bus.last_reboot_mode, Some(BootMode::Bootloader));
}

#[test]
fn baud_write_stages_via_reply_handle() {
    // Writing the baud-rate register forwards through the ControlTableHooks
    // shim onto reply.stage_baud — the dispatcher no longer maintains a
    // separate Event sink.
    use crate::regions::config::addr::comms::BAUD_RATE_IDX;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), BAUD_RATE_IDX, &[BaudRate::B1000000 as u8]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.last_baud_staged, Some(BaudRate::B1000000));
}

#[test]
fn id_write_stages_via_reply_handle() {
    // Writing the ID register fires the on_id_write hook, which forwards
    // through ControlTableHooks onto reply.stage_id.
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), ID, &[42]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.last_id_staged, Some(42));
}

#[test]
fn rdt_write_stages_via_reply_handle_in_us() {
    // Control-table RDT is stored in 2-µs units; the hook converts to
    // plain µs at the trait boundary so the driver doesn't repeat the math.
    use crate::regions::config::addr::comms::RETURN_DELAY_2US;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), RETURN_DELAY_2US, &[100]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.last_rdt_staged, Some(200));
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
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 1);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_read_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn return_level_read_silences_write_ack_but_read_replies() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 1);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
}

#[test]
fn return_level_none_silences_write_error_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), 0, &[0xAA, 0xBB]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_unsupported_instruction_error() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.factory_reset(Id::new(0), 0xFF));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn return_level_none_silences_action_ack() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn return_level_none_silences_reboot_ack_but_reboot_still_fires() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
    assert_eq!(bus.reboot_count, 1);
    assert_eq!(bus.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn return_level_none_still_replies_to_ping() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_replies_to_read_errors() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0xFFFE, 1));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn sync_read_in_our_slot_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[5, 7, 9]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn sync_read_data_range_error_still_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0xFFFE, 1, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn bulk_read_in_our_slot_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (id, err, params) = parse_status(&bus.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [9, 0xFE, 0xFF, 4, 0, 0, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, params) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_data_range_error_still_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0xFE, 0xFF, 1, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_sync_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    assert_eq!(bus.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[5, 7, 9]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_sync_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    assert_eq!(bus.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [5, 0, 0, 2, 0, 7, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [0, 0, 0, 2, 0];
    let req = encode(|w| w.fast_bulk_read(bulk_entries(&body)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(bus.tx.is_empty());
}

#[test]
fn fast_sync_read_error_emits_zero_payload_slot() {
    // FastError path: control-table read fails → chip emits `length` zero
    // bytes for the data field rather than reading garbage off a stale buf.
    // FakeBus encodes whatever Slot the dispatcher hands it; verify the
    // data is all-zeros and the error byte propagates.
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0xFFFE, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 1);
    assert_eq!(bus.last_kind, Some(ReplyKind::Slot));
    // Wire bytes: HEADER(4) + 0xFE + LEN(2) + 0x55 + error + id + 2 zeros + CRC(2)
    assert_eq!(bus.tx.len(), 14);
    assert_eq!(bus.tx[8], StatusError::code(ErrorCode::DataRange).as_byte());
    assert_eq!(bus.tx[9], 0); // our id
    assert_eq!(&bus.tx[10..12], &[0, 0]); // length zero bytes
}

#[test]
fn sync_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Two slots: id=7 (not us), id=0 (us). length=1, both write 0x01 to torque_enable.
    let body = [7, 0x00, 0, 0x01];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_to_other_ids_only_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let body = [7, 0x01, 17, 0x01];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_clears_pending_reg_write_staging_too() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    // Sync Write to mode register — should wipe the RegWrite staging.
    let body = [0, Mode::PositionPid as u8];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR + 1, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // id=0, address=CONTROL_BASE_ADDR (torque_enable), length=1, data=[1].
    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [0, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_other_id_silent_and_does_not_mutate() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [17, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_uses_our_tuples_address_not_a_preceding_slots() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Slot for id=7 (foreign) targets torque_enable; our slot (id=0) targets
    // the mode register one byte later. Verify we honor our own address.
    let other_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let other_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let mode_addr = CONTROL_BASE_ADDR + 1;
    let our_lo = (mode_addr & 0xFF) as u8;
    let our_hi = (mode_addr >> 8) as u8;
    let body = [
        7,
        other_lo,
        other_hi,
        1,
        0,
        1, //
        0,
        our_lo,
        our_hi,
        1,
        0,
        Mode::PositionPid as u8,
    ];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
}
