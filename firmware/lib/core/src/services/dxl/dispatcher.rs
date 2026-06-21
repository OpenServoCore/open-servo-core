use dxl_protocol::streaming::{
    CrcResult, Event, HeaderEvent, InstructionHeader, InstructionPayload, PayloadEvent,
};
use dxl_protocol::types::{ErrorCode, Id, PingStatus, Slot, Status, StatusError};

use control_table::Snapshot;

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{DxlDispatcher, DxlReply};
use crate::{
    Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel, ValidationKind,
};

use super::limits::MAX_CONTROL_RW;

fn error_to_status(e: Error) -> StatusError {
    match e {
        Error::AccessError | Error::ValidationError(ValidationKind::Locked) => {
            StatusError::code(ErrorCode::Access)
        }
        _ => StatusError::code(ErrorCode::DataRange),
    }
}

#[derive(Copy, Clone)]
struct Ctx {
    id: Id,
    level: StatusReturnLevel,
}

impl Ctx {
    fn addressed(&self, target: Id) -> Option<(Id, bool)> {
        let out = if target == self.id {
            Some((self.id, true))
        } else if target.is_broadcast() {
            Some((self.id, false))
        } else {
            None
        };
        crate::log::trace!(
            "dispatcher: addressed my_id={:?} target={:?} → {:?}",
            self.id,
            target,
            out,
        );
        out
    }
}

struct MatchedSlot {
    address: u16,
    length: u16,
}

pub(super) struct Inflight {
    header: InstructionHeader,
    ctx: Ctx,
    matched_slot: Option<MatchedSlot>,
    /// Gates `WriteDataChunk` absorption; toggles per Sync/Bulk slot.
    accumulating: bool,
    /// Resets at each SyncSlot/BulkSlot demarcation.
    write_offset: u16,
    /// Streaming chunks land past it; commit drains via `iter_from(&snap)`
    /// then `rewind_to(snap)`, so any RegWrite chain below survives.
    snap: Snapshot,
    /// Sticky on `push` failure; commit maps to `Status::Empty{DataRange}`.
    overflowed: bool,
}

fn header_target(h: &InstructionHeader) -> Id {
    use InstructionHeader::*;
    match *h {
        Ping { id }
        | Read { id, .. }
        | Write { id, .. }
        | RegWrite { id, .. }
        | Action { id }
        | Reboot { id }
        | FactoryReset { id, .. }
        | Clear { id, .. }
        | ControlTableBackup { id, .. }
        | SyncRead { id, .. }
        | SyncWrite { id, .. }
        | BulkRead { id }
        | BulkWrite { id }
        | FastSyncRead { id, .. }
        | FastBulkRead { id }
        | Raw { id, .. } => id,
    }
}

pub(super) struct Dispatcher<'a> {
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
    /// Borrowed from `Dxl`. Persists across `Dxl::poll` calls so a packet
    /// whose Header and Crc straddle two poll wakes (edge HT vs IDLE) keeps
    /// its bookkeeping. Owning it on `Dispatcher` would re-zero `inflight`
    /// at every poll and drop the reply.
    inflight: &'a mut Option<Inflight>,
}

impl<'a> Dispatcher<'a> {
    pub(super) fn new(
        shared: &'a Shared,
        staged: &'a mut StagedWrites,
        inflight: &'a mut Option<Inflight>,
    ) -> Self {
        Self {
            shared,
            staged,
            inflight,
        }
    }

    fn snapshot_ctx(&self) -> Ctx {
        let (id, level) = self
            .shared
            .table
            .config
            .with(|c| (c.comms.id, c.comms.status_return_level));
        crate::log::trace!("dispatcher: snapshot_ctx id={} level={:?}", id, level,);
        Ctx {
            id: Id::new(id),
            level,
        }
    }
}

impl DxlDispatcher for Dispatcher<'_> {
    fn on_event<R: DxlReply>(&mut self, ev: Event, ring: &[u8], reply: &mut R) {
        crate::log::trace!(
            "dispatcher: on_event ev={:?} inflight={}",
            ev,
            self.inflight.is_some()
        );
        match ev {
            Event::Sync => self.reset(),
            Event::Header(HeaderEvent::Instruction(h)) => self.start_inflight(h),
            Event::Header(HeaderEvent::Status(_)) => {}
            Event::Payload(PayloadEvent::Instruction(p)) => self.on_instruction_payload(p, ring),
            Event::Payload(PayloadEvent::Status(_)) => {}
            Event::Crc(CrcResult::Good) => self.commit(reply),
            Event::Crc(CrcResult::Bad) | Event::Resync(_) => self.reset(),
        }
    }
}

impl Dispatcher<'_> {
    fn reset(&mut self) {
        if let Some(inflight) = self.inflight.take() {
            self.staged.rewind_to(inflight.snap);
        }
    }

    fn start_inflight(&mut self, header: InstructionHeader) {
        let ctx = self.snapshot_ctx();
        let target = header_target(&header);
        let addressed = ctx.addressed(target).is_some();
        crate::log::trace!(
            "dispatcher: start_inflight header={:?} target={:?} addressed={}",
            header,
            target,
            addressed,
        );
        let accumulating = addressed
            && matches!(
                header,
                InstructionHeader::Write { .. } | InstructionHeader::RegWrite { .. },
            );
        *self.inflight = Some(Inflight {
            header,
            ctx,
            matched_slot: None,
            accumulating,
            write_offset: 0,
            snap: self.staged.snapshot(),
            overflowed: false,
        });
    }

    fn on_instruction_payload(&mut self, p: InstructionPayload, ring: &[u8]) {
        let Some(inflight) = self.inflight.as_mut() else {
            return;
        };
        match p {
            InstructionPayload::SyncSlot { id, index: _ } => {
                if id == inflight.ctx.id {
                    let slot = match inflight.header {
                        InstructionHeader::SyncRead {
                            address, length, ..
                        }
                        | InstructionHeader::SyncWrite {
                            address, length, ..
                        }
                        | InstructionHeader::FastSyncRead {
                            address, length, ..
                        } => Some(MatchedSlot { address, length }),
                        _ => {
                            debug_assert!(false, "SyncSlot emitted under non-Sync header");
                            None
                        }
                    };
                    if let Some(slot) = slot {
                        let is_write =
                            matches!(inflight.header, InstructionHeader::SyncWrite { .. });
                        inflight.matched_slot = Some(slot);
                        inflight.accumulating = is_write;
                        inflight.write_offset = 0;
                    }
                } else {
                    inflight.accumulating = false;
                }
            }
            InstructionPayload::BulkSlot {
                id,
                index: _,
                address,
                length,
            } => {
                if id == inflight.ctx.id {
                    let is_write = matches!(inflight.header, InstructionHeader::BulkWrite { .. });
                    inflight.matched_slot = Some(MatchedSlot { address, length });
                    inflight.accumulating = is_write;
                    inflight.write_offset = 0;
                } else {
                    inflight.accumulating = false;
                }
            }
            InstructionPayload::WriteDataChunk {
                offset: _,
                length: _,
            } => {
                if !inflight.accumulating {
                    return;
                }
                let base_addr = match inflight.header {
                    InstructionHeader::Write { address, .. }
                    | InstructionHeader::RegWrite { address, .. }
                    | InstructionHeader::SyncWrite { address, .. } => address,
                    InstructionHeader::BulkWrite { .. } => inflight
                        .matched_slot
                        .as_ref()
                        .map(|s| s.address)
                        .unwrap_or(0),
                    _ => return,
                };
                let dst_addr = base_addr.wrapping_add(inflight.write_offset);
                if self.staged.push(dst_addr, ring).is_err() {
                    inflight.overflowed = true;
                }
                inflight.write_offset = inflight.write_offset.wrapping_add(ring.len() as u16);
            }
        }
    }

    fn commit<R: DxlReply>(&mut self, reply: &mut R) {
        let Some(inflight) = self.inflight.take() else {
            crate::log::debug!("dispatcher: commit drop (no inflight)");
            return;
        };
        let ctx = inflight.ctx;
        crate::log::debug!(
            "dispatcher: commit header={:?} my_id={:?} overflowed={}",
            inflight.header,
            ctx.id,
            inflight.overflowed,
        );
        match inflight.header {
            InstructionHeader::Ping { id } => self.handle_ping(&ctx, id, reply),
            InstructionHeader::Read {
                id,
                address,
                length,
            } => self.handle_read(&ctx, id, address, length, reply),
            InstructionHeader::Write {
                id,
                address,
                length,
            } => self.handle_write(&ctx, id, address, length, &inflight, reply),
            InstructionHeader::RegWrite {
                id,
                address,
                length,
            } => self.handle_reg_write(&ctx, id, address, length, &inflight, reply),
            InstructionHeader::Action { id } => self.handle_action(&ctx, id, reply),
            InstructionHeader::Reboot { id } => self.handle_reboot(&ctx, id, reply),
            InstructionHeader::FactoryReset { id, .. }
            | InstructionHeader::Clear { id, .. }
            | InstructionHeader::ControlTableBackup { id, .. }
            | InstructionHeader::Raw { id, .. } => self.reply_unsupported(&ctx, id, reply),
            InstructionHeader::SyncRead {
                address, length, ..
            } => self.handle_sync_read(&ctx, address, length, &inflight, reply),
            InstructionHeader::SyncWrite {
                address, length, ..
            } => self.handle_sync_write(address, length, &inflight, reply),
            InstructionHeader::BulkRead { .. } => self.handle_bulk_read(&ctx, &inflight, reply),
            InstructionHeader::BulkWrite { .. } => self.handle_bulk_write(&inflight, reply),
            InstructionHeader::FastSyncRead { .. } | InstructionHeader::FastBulkRead { .. } => {
                self.handle_fast_read(&ctx, &inflight, reply);
            }
        }
    }

    fn send_status<R: DxlReply>(reply: &mut R, status: Status<'_>) {
        crate::log::debug!("dispatcher: send_status status={:?}", status);
        // TX overflow is a sizing bug, not a runtime error — bench telemetry
        // catches it at integration.
        let _ = reply.send_status(status);
    }

    fn send_slot<R: DxlReply>(reply: &mut R, slot: &Slot<'_>) {
        crate::log::debug!("dispatcher: send_slot slot={:?}", slot);
        let _ = reply.send_slot(slot);
    }

    fn reply_unsupported<R: DxlReply>(&mut self, ctx: &Ctx, target: Id, reply: &mut R) {
        if ctx.level < StatusReturnLevel::All {
            return;
        }
        if let Some((id, true)) = ctx.addressed(target) {
            Self::send_status(
                reply,
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::Instruction),
                },
            );
        }
    }

    fn reply_table_result<R: DxlReply>(
        &mut self,
        ctx: &Ctx,
        id: Id,
        direct: bool,
        result: Result<(), Error>,
        reply: &mut R,
    ) {
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let r = match result {
            Ok(()) => Status::Empty {
                id,
                error: StatusError::OK,
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        Self::send_status(reply, r);
    }

    fn handle_ping<R: DxlReply>(&mut self, ctx: &Ctx, target: Id, reply: &mut R) {
        let Some((id, _)) = ctx.addressed(target) else {
            return;
        };
        let identity = self.shared.table.config.with(|c| c.identity);
        Self::send_status(
            reply,
            Status::Ping {
                id,
                error: StatusError::OK,
                status: PingStatus {
                    model: identity.model_number,
                    fw_version: identity.firmware_version,
                },
            },
        );
    }

    fn handle_read<R: DxlReply>(
        &mut self,
        ctx: &Ctx,
        target: Id,
        address: u16,
        length: u16,
        reply: &mut R,
    ) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some((id, true)) = ctx.addressed(target) else {
            return;
        };
        let len = length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            Self::send_status(
                reply,
                Status::Empty {
                    id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
            );
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let r = match self.shared.table.read_bytes(address, &mut buf[..len]) {
            Ok(()) => Status::Read {
                id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        Self::send_status(reply, r);
    }

    fn collect_chunks(staged: &StagedWrites, snap: &Snapshot, dst: &mut [u8]) -> usize {
        let mut off = 0;
        for (_, data) in staged.iter_from(snap) {
            if off >= dst.len() {
                break;
            }
            let take = (dst.len() - off).min(data.len());
            dst[off..off + take].copy_from_slice(&data[..take]);
            off += take;
        }
        off
    }

    fn handle_write<R: DxlReply>(
        &mut self,
        ctx: &Ctx,
        target: Id,
        address: u16,
        length: u16,
        inflight: &Inflight,
        reply: &mut R,
    ) {
        let Some((id, direct)) = ctx.addressed(target) else {
            self.staged.rewind_to(inflight.snap);
            return;
        };
        let len = length as usize;
        if inflight.overflowed || len > MAX_CONTROL_RW {
            self.staged.rewind_to(inflight.snap);
            if direct && ctx.level >= StatusReturnLevel::All {
                Self::send_status(
                    reply,
                    Status::Empty {
                        id,
                        error: StatusError::code(ErrorCode::DataRange),
                    },
                );
            }
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let off = Self::collect_chunks(self.staged, &inflight.snap, &mut buf);
        self.staged.rewind_to(inflight.snap);
        let result = self
            .shared
            .table
            .write_bytes(address, &buf[..off], self.staged);
        let ok = result.is_ok();
        self.reply_table_result(ctx, id, direct, result, reply);
        if ok {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .dispatch_events(address, off as u16, &mut hooks);
        }
    }

    fn handle_reg_write<R: DxlReply>(
        &mut self,
        ctx: &Ctx,
        target: Id,
        address: u16,
        length: u16,
        inflight: &Inflight,
        reply: &mut R,
    ) {
        let Some((id, direct)) = ctx.addressed(target) else {
            self.staged.rewind_to(inflight.snap);
            return;
        };
        let len = length as usize;
        if inflight.overflowed || len > MAX_CONTROL_RW {
            self.staged.rewind_to(inflight.snap);
            if direct && ctx.level >= StatusReturnLevel::All {
                Self::send_status(
                    reply,
                    Status::Empty {
                        id,
                        error: StatusError::code(ErrorCode::DataRange),
                    },
                );
            }
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let off = Self::collect_chunks(self.staged, &inflight.snap, &mut buf);
        self.staged.rewind_to(inflight.snap);
        let result = self
            .shared
            .table
            .stage_bytes(address, &buf[..off], self.staged);
        if !direct || ctx.level < StatusReturnLevel::All {
            return;
        }
        let r = match result {
            Ok(()) => Status::Empty {
                id,
                error: StatusError::OK,
            },
            Err(e) => Status::Empty {
                id,
                error: error_to_status(e),
            },
        };
        Self::send_status(reply, r);
    }

    fn handle_action<R: DxlReply>(&mut self, ctx: &Ctx, target: Id, reply: &mut R) {
        let Some((id, direct)) = ctx.addressed(target) else {
            return;
        };
        self.shared.table.commit_staged(self.staged);
        if direct && ctx.level >= StatusReturnLevel::All {
            Self::send_status(
                reply,
                Status::Empty {
                    id,
                    error: StatusError::OK,
                },
            );
        }
    }

    fn handle_reboot<R: DxlReply>(&mut self, ctx: &Ctx, target: Id, reply: &mut R) {
        let Some((id, direct)) = ctx.addressed(target) else {
            return;
        };
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        if direct && ctx.level >= StatusReturnLevel::All {
            Self::send_status(
                reply,
                Status::Empty {
                    id,
                    error: StatusError::OK,
                },
            );
        }
        reply.stage_reboot(mode);
    }

    fn handle_sync_read<R: DxlReply>(
        &mut self,
        ctx: &Ctx,
        address: u16,
        length: u16,
        inflight: &Inflight,
        reply: &mut R,
    ) {
        if ctx.level < StatusReturnLevel::Read || inflight.matched_slot.is_none() {
            return;
        }
        let len = length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            Self::send_status(
                reply,
                Status::Empty {
                    id: ctx.id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
            );
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let r = match self.shared.table.read_bytes(address, &mut buf[..len]) {
            Ok(()) => Status::Read {
                id: ctx.id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id: ctx.id,
                error: error_to_status(e),
            },
        };
        Self::send_status(reply, r);
    }

    fn handle_sync_write<R: DxlReply>(
        &mut self,
        address: u16,
        length: u16,
        inflight: &Inflight,
        reply: &mut R,
    ) {
        let len = length as usize;
        if inflight.matched_slot.is_none()
            || inflight.overflowed
            || len == 0
            || len > MAX_CONTROL_RW
        {
            self.staged.rewind_to(inflight.snap);
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let off = Self::collect_chunks(self.staged, &inflight.snap, &mut buf);
        self.staged.rewind_to(inflight.snap);
        if off < len {
            return;
        }
        if self
            .shared
            .table
            .write_bytes(address, &buf[..len], self.staged)
            .is_ok()
        {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .dispatch_events(address, len as u16, &mut hooks);
        }
    }

    fn handle_bulk_read<R: DxlReply>(&mut self, ctx: &Ctx, inflight: &Inflight, reply: &mut R) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(slot) = inflight.matched_slot.as_ref() else {
            return;
        };
        let len = slot.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            Self::send_status(
                reply,
                Status::Empty {
                    id: ctx.id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
            );
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let r = match self.shared.table.read_bytes(slot.address, &mut buf[..len]) {
            Ok(()) => Status::Read {
                id: ctx.id,
                error: StatusError::OK,
                data: &buf[..len],
            },
            Err(e) => Status::Empty {
                id: ctx.id,
                error: error_to_status(e),
            },
        };
        Self::send_status(reply, r);
    }

    fn handle_bulk_write<R: DxlReply>(&mut self, inflight: &Inflight, reply: &mut R) {
        let Some(slot) = inflight.matched_slot.as_ref() else {
            self.staged.rewind_to(inflight.snap);
            return;
        };
        let len = slot.length as usize;
        if inflight.overflowed || len == 0 || len > MAX_CONTROL_RW {
            self.staged.rewind_to(inflight.snap);
            return;
        }
        let mut buf = [0u8; MAX_CONTROL_RW];
        let off = Self::collect_chunks(self.staged, &inflight.snap, &mut buf);
        self.staged.rewind_to(inflight.snap);
        if off < len {
            return;
        }
        let address = slot.address;
        if self
            .shared
            .table
            .write_bytes(address, &buf[..len], self.staged)
            .is_ok()
        {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .dispatch_events(address, len as u16, &mut hooks);
        }
    }

    fn handle_fast_read<R: DxlReply>(&mut self, ctx: &Ctx, inflight: &Inflight, reply: &mut R) {
        if ctx.level < StatusReturnLevel::Read {
            return;
        }
        let Some(slot) = inflight.matched_slot.as_ref() else {
            return;
        };
        let len = slot.length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            return;
        }
        // On read failure: emit `length` zero bytes so the chain stays
        // positionally aligned; the error byte carries the code.
        let mut buf = [0u8; MAX_CONTROL_RW];
        let (error, data_len) = match self.shared.table.read_bytes(slot.address, &mut buf[..len]) {
            Ok(()) => (StatusError::OK, len),
            Err(e) => {
                buf[..len].fill(0);
                (error_to_status(e), len)
            }
        };
        let slot_reply = Slot {
            id: ctx.id,
            error,
            data: &buf[..data_len],
        };
        Self::send_slot(reply, &slot_reply);
    }
}
