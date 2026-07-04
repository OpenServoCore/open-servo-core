//! Unit-shaped tests for the DXL service. See
//! `firmware/lib/integration/tests/protocol/*` for the sim-driven
//! stack-level gear that covers the same spec-scenario behavior end-to-end.

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

mod bulk_read;
mod bulk_write;
mod fast_bulk_read;
mod fast_sync_read;
mod lifecycle;
mod ping;
mod read;
mod reboot;
mod reg_write_action;
mod srl_matrix;
mod sync_read;
mod sync_write;
mod write;

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
                Chunk::Slice(s) => scratch
                    .extend_from_slice(s)
                    .map_err(|_| WriteError::Overflow)?,
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
            dxl_protocol::SlotPosition::First { packet_length: len },
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

fn set_level(shared: &Shared, level: StatusReturnLevel) {
    shared
        .table
        .config
        .with_mut(|c| c.comms.status_return_level = level);
}
