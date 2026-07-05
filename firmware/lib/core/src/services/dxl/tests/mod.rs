//! Unit-shaped tests for the DXL service. See
//! `firmware/lib/integration/tests/protocol/*` for the sim-driven
//! stack-level gear that covers the same spec-scenario behavior end-to-end.

use crc::{CRC_16_UMTS, Crc};
use dxl_protocol::streaming::{Event, HeaderEvent, Parser, PayloadEvent, StatusPayload};
use dxl_protocol::types::{BulkReadEntry, ErrorCode, Id, Slot, Status, StatusError};
use dxl_protocol::{Chunk, CrcUmts, InstructionEncoder, SlotEncoder, StatusEncoder, WriteError};
use heapless::Vec;

const BROADCAST_ID: Id = Id::BROADCAST;

use dxl_protocol::frame::{ParseError, parse};
use dxl_protocol::types::packet::{Instruction, decode_instruction};
use dxl_protocol::unstuff::{ByteIter, Bytes};
use dxl_protocol::wire::BROADCAST_ID as BROADCAST_ID_BYTE;

use crate::regions::config::BaudRate;
use crate::traits::{DxlBus, DxlDispatch, DxlReply, DxlRequest, DxlRequestCtx};
use crate::{BootMode, RegionStorage, Shared, StatusReturnLevel};

use super::Dxl;

mod bulk_read;
mod bulk_write;
mod dispatch;
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

/// Stand-in DXL bus for the single-shot [`DxlDispatch`] surface. Accumulates
/// fed wire bytes and, on `poll`, frames + decodes them into one
/// [`DxlRequest`] per frame — resolving this servo's participation and its
/// own slot's data the way the production framer does — then hands each to
/// the dispatcher. Mirrors production, where a packet can straddle two
/// `Dxl::poll` wakes: bytes stay buffered until a whole frame arrives.
struct FakeBus {
    pending: Vec<u8, 256>,
    /// This servo's id — `Shared::new()` defaults the config id to 0, so the
    /// bus's addressing filter uses 0 unless a test overrides it.
    our_id: u8,
    reply: FakeReply,
}

impl FakeBus {
    fn new() -> Self {
        Self {
            pending: Vec::new(),
            our_id: 0,
            reply: FakeReply::new(),
        }
    }

    fn feed(&mut self, bytes: &[u8]) {
        self.pending.extend_from_slice(bytes).unwrap();
    }

    /// Models the bus-collision / no-IDLE case where bytes never surface.
    fn feed_no_idle(&mut self, _bytes: &[u8]) {}
}

impl DxlBus for FakeBus {
    fn poll<D: DxlDispatch>(&mut self, dispatcher: &mut D) {
        let FakeBus {
            pending,
            our_id,
            reply,
        } = self;
        let mut cursor = 0;
        loop {
            let rest = &pending[cursor..];
            match parse::<TestDxlCrc>(rest) {
                Ok((frame, n)) => {
                    let mut wr = [0u8; 128];
                    if let Ok(instr) = decode_instruction(&frame)
                        && let Some((req, ctx)) = derive_request(&instr, *our_id, &mut wr)
                    {
                        dispatcher.dispatch(req, ctx, reply);
                    }
                    cursor += n;
                }
                Err(ParseError::Incomplete) => break,
                Err(
                    ParseError::Junk { skip }
                    | ParseError::BadLength { skip }
                    | ParseError::BadCrc { skip },
                ) => cursor += skip,
            }
        }
        // Drop the consumed prefix; keep any incomplete tail for the next feed.
        if cursor > 0 {
            let tail: Vec<u8, 256> = pending[cursor..].iter().copied().collect();
            *pending = tail;
        }
    }
}

/// `Some(broadcast)` when a direct instruction targets this servo (own id or
/// broadcast); `None` when it's foreign.
fn addressed(tid: Id, our_id: u8) -> Option<bool> {
    let t = tid.as_byte();
    if t == our_id {
        Some(false)
    } else if t == BROADCAST_ID_BYTE {
        Some(true)
    } else {
        None
    }
}

fn ctx_for(broadcast: bool, may_reply: bool, slot_reply: bool) -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast,
        may_reply,
        slot_reply,
    }
}

/// Copy the next `n` logical bytes of `it` into `dst`, returning the count.
fn take_bytes(it: &mut ByteIter<'_>, n: usize, dst: &mut [u8]) -> usize {
    let mut w = 0;
    for _ in 0..n {
        match it.next() {
            Some(b) if w < dst.len() => {
                dst[w] = b;
                w += 1;
            }
            Some(_) => {}
            None => break,
        }
    }
    w
}

fn take_u16(it: &mut ByteIter<'_>) -> Option<u16> {
    let lo = it.next()?;
    let hi = it.next()?;
    Some(u16::from_le_bytes([lo, hi]))
}

fn destuff_into(data: &Bytes<'_>, wr: &mut [u8]) -> usize {
    let mut n = 0;
    for b in data.iter() {
        if n >= wr.len() {
            break;
        }
        wr[n] = b;
        n += 1;
    }
    n
}

/// Resolve a decoded instruction into the request this servo dispatches, if
/// it participates — the bus-side derivation the production framer performs.
fn derive_request<'w>(
    instr: &Instruction<'_>,
    our_id: u8,
    wr: &'w mut [u8],
) -> Option<(DxlRequest<'w>, DxlRequestCtx)> {
    match *instr {
        Instruction::Ping { id } => {
            let broadcast = addressed(id, our_id)?;
            Some((DxlRequest::Ping, ctx_for(broadcast, true, false)))
        }
        Instruction::Read {
            id,
            address,
            length,
        } => {
            let broadcast = addressed(id, our_id)?;
            Some((
                DxlRequest::Read { address, length },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::Write { id, address, data } => {
            let broadcast = addressed(id, our_id)?;
            let n = destuff_into(&data, wr);
            Some((
                DxlRequest::Write {
                    address,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::RegWrite { id, address, data } => {
            let broadcast = addressed(id, our_id)?;
            let n = destuff_into(&data, wr);
            Some((
                DxlRequest::RegWrite {
                    address,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::Action { id } => {
            let broadcast = addressed(id, our_id)?;
            Some((DxlRequest::Action, ctx_for(broadcast, !broadcast, false)))
        }
        Instruction::Reboot { id } => {
            let broadcast = addressed(id, our_id)?;
            Some((DxlRequest::Reboot, ctx_for(broadcast, !broadcast, false)))
        }
        Instruction::FactoryReset { id, mode } => {
            let broadcast = addressed(id, our_id)?;
            Some((
                DxlRequest::FactoryReset { mode },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::Clear { id, .. } => {
            let broadcast = addressed(id, our_id)?;
            Some((
                DxlRequest::Clear {
                    body: Bytes::raw(&[]),
                },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::ControlTableBackup { id, .. } => {
            let broadcast = addressed(id, our_id)?;
            Some((
                DxlRequest::ControlTableBackup {
                    body: Bytes::raw(&[]),
                },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::Ext {
            id, instruction, ..
        } => {
            let broadcast = addressed(id, our_id)?;
            Some((
                DxlRequest::Ext {
                    instruction,
                    params: Bytes::raw(&[]),
                },
                ctx_for(broadcast, !broadcast, false),
            ))
        }
        Instruction::SyncRead {
            address,
            length,
            ids,
        } => sync_read_slot(&ids, our_id).then_some((
            DxlRequest::Read { address, length },
            ctx_for(true, true, false),
        )),
        Instruction::FastSyncRead {
            address,
            length,
            ids,
        } => sync_read_slot(&ids, our_id).then_some((
            DxlRequest::Read { address, length },
            ctx_for(true, true, true),
        )),
        Instruction::SyncWrite {
            address,
            length,
            body,
        } => sync_write_slot(&body, length, our_id, wr).map(|n| {
            (
                DxlRequest::Write {
                    address,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx_for(true, false, false),
            )
        }),
        Instruction::BulkRead { body } => bulk_read_slot(&body, our_id).map(|(a, l)| {
            (
                DxlRequest::Read {
                    address: a,
                    length: l,
                },
                ctx_for(true, true, false),
            )
        }),
        Instruction::FastBulkRead { body } => bulk_read_slot(&body, our_id).map(|(a, l)| {
            (
                DxlRequest::Read {
                    address: a,
                    length: l,
                },
                ctx_for(true, true, true),
            )
        }),
        Instruction::BulkWrite { body } => bulk_write_slot(&body, our_id, wr).map(|(a, n)| {
            (
                DxlRequest::Write {
                    address: a,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx_for(true, false, false),
            )
        }),
    }
}

/// Whether `our_id` appears in a Sync Read id list.
fn sync_read_slot(ids: &Bytes<'_>, our_id: u8) -> bool {
    ids.iter().any(|slot_id| slot_id == our_id)
}

/// Capture our slot's write data (`[id, data(length)]*`); `Some(n)` when present.
fn sync_write_slot(body: &Bytes<'_>, length: u16, our_id: u8, wr: &mut [u8]) -> Option<usize> {
    let mut it = body.iter();
    while let Some(slot_id) = it.next() {
        if slot_id == our_id {
            return Some(take_bytes(&mut it, length as usize, wr));
        }
        for _ in 0..length {
            it.next()?;
        }
    }
    None
}

/// Our slot's `(address, length)` in a Bulk Read body (`[id, addr16, len16]*`).
fn bulk_read_slot(body: &Bytes<'_>, our_id: u8) -> Option<(u16, u16)> {
    let mut it = body.iter();
    while let Some(slot_id) = it.next() {
        let address = take_u16(&mut it)?;
        let length = take_u16(&mut it)?;
        if slot_id == our_id {
            return Some((address, length));
        }
    }
    None
}

/// Our slot's `(address, data_len)` in a Bulk Write body (`[id, addr16, len16,
/// data(len)]*`), capturing the data into `wr`.
fn bulk_write_slot(body: &Bytes<'_>, our_id: u8, wr: &mut [u8]) -> Option<(u16, usize)> {
    let mut it = body.iter();
    while let Some(slot_id) = it.next() {
        let address = take_u16(&mut it)?;
        let length = take_u16(&mut it)?;
        if slot_id == our_id {
            return Some((address, take_bytes(&mut it, length as usize, wr)));
        }
        for _ in 0..length {
            it.next()?;
        }
    }
    None
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
