use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::streaming::{
    Event, HeaderEvent, InstructionPayload, Parser, PayloadEvent, StatusPayload,
};
use dxl_protocol::types::{BulkReadEntry, ErrorCode, Id, Slot, Status, StatusError};
use dxl_protocol::{Chunk, CrcUmts, InstructionEncoder, SlotEncoder, StatusEncoder, WriteError};
use heapless::Vec;

const BROADCAST_ID: Id = Id::BROADCAST;

use crate::regions::config::BaudRate;
use crate::traits::{DxlBus, DxlDispatcher, DxlReply};
use crate::{BootMode, RegionStorage, Shared, StatusReturnLevel};

use super::Dxl;

/// Test-only `CrcUmts` — core never picks a concrete engine in production.
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

/// Slot position is driver-side; tests here only need to tell Plain Status
/// apart from a Fast Slot reply.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ReplyKind {
    Plain,
    Slot,
}

/// Split out from `FakeBus` so `poll` can disjoint-borrow burst (parser
/// input) and reply (dispatcher output).
struct FakeReply {
    pub tx: Vec<u8, 256>,
    pub send_count: u32,
    pub last_kind: Option<ReplyKind>,
    pub last_id_staged: Option<u8>,
    pub last_baud_staged: Option<BaudRate>,
    pub last_rdt_staged: Option<u32>,
    pub last_reboot_mode: Option<BootMode>,
    pub reboot_count: u32,
}

impl FakeReply {
    fn new() -> Self {
        Self {
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
}

impl DxlReply for FakeReply {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        self.tx.clear();
        StatusEncoder::<_, TestDxlCrc>::new(&mut self.tx).emit(status)?;
        self.send_count += 1;
        self.last_kind = Some(ReplyKind::Plain);
        Ok(())
    }

    fn send_status_read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        self.tx.clear();
        StatusEncoder::<_, TestDxlCrc>::new(&mut self.tx).read_chunked(id, error, chunks)?;
        self.send_count += 1;
        self.last_kind = Some(ReplyKind::Plain);
        Ok(())
    }

    fn send_slot_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        // Match the slice-path mock: force `Only` + compute the framing length
        // by buffering the chunks into a tiny scratch first. Production never
        // routes the chunked slot through this position; the buffering is
        // bounded by the test slot payloads (sub-32 B).
        self.tx.clear();
        let mut scratch: Vec<u8, 64> = Vec::new();
        for chunk in chunks {
            match chunk {
                Chunk::Slice(s) => scratch.extend_from_slice(s).map_err(|_| WriteError::Overflow)?,
                Chunk::Zero(n) => {
                    for _ in 0..n {
                        scratch.push(0).map_err(|_| WriteError::Overflow)?;
                    }
                }
            }
        }
        let len = (3 + scratch.len() + 2) as u16;
        let slot = Slot {
            id,
            error,
            data: &scratch,
        };
        SlotEncoder::<_, TestDxlCrc>::new(&mut self.tx).emit(
            &slot,
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

struct FakeBus {
    burst: Vec<u8, 256>,
    burst_fresh: bool,
    /// Persistent across polls, mirroring production where the parser
    /// lives in the codec and a packet can straddle two `Dxl::poll` wakes
    /// (edge HT vs IDLE). Tests that split a burst across two `feed`
    /// calls rely on this carrying mid-packet state through.
    parser: Parser<TestDxlCrc>,
    reply: FakeReply,
}

impl FakeBus {
    fn new() -> Self {
        Self {
            burst: Vec::new(),
            burst_fresh: false,
            parser: Parser::<TestDxlCrc>::new(),
            reply: FakeReply::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        self.burst.clear();
        self.burst.extend_from_slice(bytes).unwrap();
        self.burst_fresh = true;
    }

    /// Models the bus-collision / no-IDLE case where bytes never surface.
    fn feed_no_idle(&mut self, _bytes: &[u8]) {}
}

impl DxlBus for FakeBus {
    fn poll<D: DxlDispatcher>(&mut self, dispatcher: &mut D) {
        if !self.burst_fresh {
            return;
        }
        self.burst_fresh = false;
        let burst: &[u8] = &self.burst;
        let reply = &mut self.reply;
        for ev in self.parser.feed(burst) {
            let chunk = chunk_for(&ev, burst);
            dispatcher.on_event(ev, chunk, reply);
        }
    }
}

/// Resolve a chunked-payload event's `(offset, length)` into a burst slice.
fn chunk_for<'a>(ev: &Event, burst: &'a [u8]) -> &'a [u8] {
    match *ev {
        Event::Payload(PayloadEvent::Instruction(InstructionPayload::WriteDataChunk {
            offset,
            length,
        })) => {
            let lo = offset as usize;
            let hi = lo + length as usize;
            &burst[lo..hi]
        }
        _ => &[],
    }
}

fn encode<F>(f: F) -> Vec<u8, 256>
where
    F: FnOnce(&mut InstructionEncoder<'_, Vec<u8, 256>, TestDxlCrc>) -> Result<(), WriteError>,
{
    let mut buf: Vec<u8, 256> = Vec::new();
    f(&mut InstructionEncoder::<_, TestDxlCrc>::new(&mut buf)).unwrap();
    buf
}

fn bre(id: u8, address: u16, length: u16) -> BulkReadEntry {
    BulkReadEntry {
        id: Id::new(id),
        address,
        length,
    }
}

/// Returns `(id, error_byte, payload)`. Payload is the post-error wire bytes:
/// data for Read-family, `(model_lo, model_hi, fw)` for Ping, empty for Empty.
fn parse_status(bytes: &[u8]) -> (u8, u8, Vec<u8, 64>) {
    let mut parser = Parser::<TestDxlCrc>::new();
    let mut id: u8 = 0;
    let mut err: u8 = 0;
    let mut data: Vec<u8, 64> = Vec::new();
    let mut saw_crc = false;
    for ev in parser.feed(bytes) {
        match ev {
            Event::Header(HeaderEvent::Status(h)) => {
                id = h.id.as_byte();
                err = h.error.as_byte();
            }
            Event::Payload(PayloadEvent::Status(StatusPayload::ReadDataChunk {
                offset,
                length,
            })) => {
                let lo = offset as usize;
                let hi = lo + length as usize;
                data.extend_from_slice(&bytes[lo..hi]).unwrap();
            }
            Event::Payload(PayloadEvent::Status(StatusPayload::Ping { model, fw_version })) => {
                data.extend_from_slice(&[(model & 0xFF) as u8, (model >> 8) as u8, fw_version])
                    .unwrap();
            }
            Event::Crc(verdict) => {
                use dxl_protocol::streaming::CrcResult;
                assert_eq!(verdict, CrcResult::Good, "status parser saw bad CRC");
                saw_crc = true;
                break;
            }
            Event::Resync(kind) => panic!("status parser resync: {kind:?}"),
            _ => {}
        }
    }
    assert!(saw_crc, "status parser did not reach Crc");
    (id, err, data)
}

#[test]
fn ping_to_our_id_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Plain));
}

/// Regression: production wakes `Dxl::poll` on edge-ring HT/TC and USART
/// IDLE; a packet whose Header lands on one wake and Crc on the next must
/// still reply. Before `inflight` was lifted onto `Dxl`, the dispatcher
/// re-zeroed it per poll and the Crc-side `commit` saw `inflight = None`,
/// silently dropping the reply.
#[test]
fn instruction_split_across_two_polls_still_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(Id::new(0)));
    // Split between the instruction byte and the CRC pair so Header lands
    // in poll #1 (creating `inflight`) and Crc lands in poll #2 (which must
    // see that same `inflight` to commit a reply).
    let split = req.len() - 2;

    bus.feed(&req[..split]);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0, "reply emitted before Crc seen");

    bus.feed(&req[split..]);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1, "split-poll Crc must commit reply");
    let (id, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
}

#[test]
fn ping_to_broadcast_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.ping(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, _) = parse_status(&bus.reply.tx);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn read_model_number_returns_two_bytes() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (id, err, params) = parse_status(&bus.reply.tx);
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

    let (_, err, _) = parse_status(&bus.reply.tx);
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

    let (_, err, _) = parse_status(&bus.reply.tx);
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

    let (id, err, params) = parse_status(&bus.reply.tx);
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

    let (_, err, _) = parse_status(&bus.reply.tx);
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

    let (_, err, _) = parse_status(&bus.reply.tx);
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

    assert_eq!(bus.reply.send_count, 0);
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

    assert_eq!(bus.reply.send_count, 0);
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
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
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
    let (_, err, _) = parse_status(&bus.reply.tx);
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
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, StatusError::code(ErrorCode::DataRange).as_byte());

    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn write_preserves_pending_reg_write_chain() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Stage torque_enable=1 via RegWrite; chain pending.
    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    // Direct Write to a different register lands immediately, leaves the
    // RegWrite chain intact.
    let req = encode(|w| w.write(Id::new(0), CONTROL_BASE_ADDR + 1, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    // Action commits the still-pending RegWrite.
    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
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
    assert_eq!(bus.reply.send_count, 0);

    let req = encode(|w| w.action(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 0);
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
    let (_, err, _) = parse_status(&bus.reply.tx);
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

    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert!(params.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_broadcast_stages_reboot_without_ack() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(BROADCAST_ID));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
}

#[test]
fn reboot_to_other_id_silent_and_no_request() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.reboot(Id::new(17)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 0);
    assert_eq!(bus.reply.last_reboot_mode, None);
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

    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::Bootloader));
}

#[test]
fn baud_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::BAUD_RATE_IDX;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), BAUD_RATE_IDX, &[BaudRate::B1000000 as u8]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_baud_staged, Some(BaudRate::B1000000));
}

#[test]
fn id_write_stages_via_reply_handle() {
    use crate::regions::config::addr::comms::ID;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), ID, &[42]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_id_staged, Some(42));
}

#[test]
fn rdt_write_stages_via_reply_handle_in_us() {
    use crate::regions::config::addr::comms::RETURN_DELAY_2US;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.write(Id::new(0), RETURN_DELAY_2US, &[100]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.last_rdt_staged, Some(200));
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
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));

    let req = encode(|w| w.ping(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1);
    let (_, err, _) = parse_status(&bus.reply.tx);
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
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
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
    assert_eq!(bus.reply.send_count, 0);

    let req = encode(|w| w.read(Id::new(0), 0, 2));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(bus.reply.send_count, 1);
    let (_, err, _) = parse_status(&bus.reply.tx);
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
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
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
    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
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

    assert_eq!(bus.reply.send_count, 0);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
    assert_eq!(bus.reply.reboot_count, 1);
    assert_eq!(bus.reply.last_reboot_mode, Some(BootMode::App));
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

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0, 0]);
}

#[test]
fn return_level_read_replies_to_unmapped_read_with_zero_bytes() {
    // Per DXL 2.0 the table is memory-shaped: unmapped addresses read back
    // as zero rather than erroring. SRL=Read still produces the reply.
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::Read);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.read(Id::new(0), 0xFFFE, 1));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0]);
}

#[test]
fn sync_read_in_our_slot_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0, 2, &[0]));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn sync_read_at_unmapped_address_replies_zero_bytes() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.sync_read(0xFFFE, 1, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (_, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0]);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn bulk_read_in_our_slot_replies() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (id, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(id, 0);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(5, 0, 2), bre(7, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn bulk_read_uses_our_tuples_address_not_a_preceding_slots() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(9, 0xFFFE, 4), bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    let (_, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0, 0]);
}

#[test]
fn bulk_read_at_unmapped_address_replies_zero_bytes() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0xFFFE, 1)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    let (_, err, params) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert_eq!(&params[..], &[0]);
}

#[test]
fn bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_sync_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[5, 7, 9]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0, 2, &[0]));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_bulk_read_in_our_slot_emits_slot_reply() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
}

#[test]
fn fast_bulk_read_silent_when_our_id_absent() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(5, 0, 2), bre(7, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_bulk_read_skips_when_no_idle_anchor() {
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed_no_idle(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_bulk_read_return_level_none_silences_reply() {
    let shared = Shared::new();
    set_level(&shared, StatusReturnLevel::None);
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let entries = [bre(0, 0, 2)];
    let req = encode(|w| w.fast_bulk_read(&entries));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
    assert!(bus.reply.tx.is_empty());
}

#[test]
fn fast_sync_read_at_unmapped_address_emits_zero_payload_slot() {
    // Per the memory-shaped read contract the slot carries `length` zero bytes
    // with error=OK, not stale buf contents.
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let req = encode(|w| w.fast_sync_read(0xFFFE, 2, &[0]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 1);
    assert_eq!(bus.reply.last_kind, Some(ReplyKind::Slot));
    // HEADER(4) + 0xFE + LEN(2) + 0x55 + error + id + 2 data + CRC(2)
    assert_eq!(bus.reply.tx.len(), 14);
    assert_eq!(bus.reply.tx[8], 0);
    assert_eq!(bus.reply.tx[9], 0);
    assert_eq!(&bus.reply.tx[10..12], &[0, 0]);
}

#[test]
fn sync_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Slots: id=7 (foreign), id=0 (us); both write 0x01 to torque_enable.
    let body = [7, 0x00, 0, 0x01];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn sync_write_preserves_pending_reg_write_chain() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Stage torque_enable=1 via RegWrite; chain pending.
    let req = encode(|w| w.reg_write(Id::new(0), CONTROL_BASE_ADDR, &[1]));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    // SyncWrite to a different register lands immediately, leaves the
    // RegWrite chain intact.
    let body = [0, Mode::PositionPid as u8];
    let req = encode(|w| w.sync_write(CONTROL_BASE_ADDR + 1, 1, &body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));

    // Action commits the still-pending RegWrite.
    let req = encode(|w| w.action(Id::new(0)));
    bus.feed(&req);
    h.poll(&shared, &mut bus);
    let (_, err, _) = parse_status(&bus.reply.tx);
    assert_eq!(err, 0);
    assert!(shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_to_our_id_mutates_and_silent() {
    use crate::regions::CONTROL_BASE_ADDR;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    let addr_lo = (CONTROL_BASE_ADDR & 0xFF) as u8;
    let addr_hi = (CONTROL_BASE_ADDR >> 8) as u8;
    let body = [0, addr_lo, addr_hi, 1, 0, 1];
    let req = encode(|w| w.bulk_write(&body));
    bus.feed(&req);
    h.poll(&shared, &mut bus);

    assert_eq!(bus.reply.send_count, 0);
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
}

#[test]
fn bulk_write_uses_our_tuples_address_not_a_preceding_slots() {
    use crate::regions::CONTROL_BASE_ADDR;
    use crate::regions::control::Mode;
    let shared = Shared::new();
    let mut bus = FakeBus::new();
    let mut h = Dxl::new();

    // Foreign slot (id=7) targets torque_enable; our slot (id=0) targets mode.
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

    assert_eq!(bus.reply.send_count, 0);
    assert!(!shared.table.control.with(|c| c.lifecycle.torque_enable));
    assert_eq!(
        shared.table.control.with(|c| c.lifecycle.mode),
        Mode::PositionPid,
    );
}
