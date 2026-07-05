use dxl_protocol::Chunk;
use dxl_protocol::types::{ErrorCode, Id, PingStatus, Status, StatusError};
use dxl_protocol::unstuff::Bytes;

use control_table::ReadChunk;

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{DxlDispatch, DxlReply, DxlRequest, DxlRequestCtx};
use crate::{
    ConfigIdentity, Error, RegionStorage, Router, Shared, StagedWrites, StatusReturnLevel,
    ValidationKind,
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

/// Config snapshot the handlers read once per request: reply id, SRL, and the
/// Ping identity. Addressing / slot geometry is resolved by the bus and rides
/// in on [`DxlRequestCtx`] instead.
struct Ctx {
    id: Id,
    level: StatusReturnLevel,
    identity: ConfigIdentity,
}

/// Stateless single-shot DXL dispatcher. Holds only borrowed shared state and
/// the RegWrite staging buffer — no per-packet reassembly; each
/// [`DxlDispatch::dispatch`] call carries its whole payload.
pub struct Dispatch<'a> {
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
}

impl<'a> Dispatch<'a> {
    pub fn new(shared: &'a Shared, staged: &'a mut StagedWrites) -> Self {
        Self { shared, staged }
    }

    fn ctx(&self) -> Ctx {
        let (id, level, identity) = self
            .shared
            .table
            .config
            .with(|c| (c.comms.id, c.comms.status_return_level, c.identity));
        Ctx {
            id: Id::new(id),
            level,
            identity,
        }
    }
}

impl DxlDispatch for Dispatch<'_> {
    fn dispatch<R: DxlReply>(&mut self, req: DxlRequest<'_>, ctx: DxlRequestCtx, reply: &mut R) {
        let c = self.ctx();
        match req {
            DxlRequest::Ping => self.ping(&c, &ctx, reply),
            DxlRequest::Read { address, length } => self.read(&c, &ctx, address, length, reply),
            DxlRequest::Write { address, data } => self.write(&c, &ctx, address, data, reply),
            DxlRequest::RegWrite { address, data } => {
                self.reg_write(&c, &ctx, address, data, reply)
            }
            DxlRequest::Action => self.action(&c, &ctx, reply),
            DxlRequest::Reboot => self.reboot(&c, &ctx, reply),
            DxlRequest::FactoryReset { .. }
            | DxlRequest::Clear { .. }
            | DxlRequest::ControlTableBackup { .. }
            | DxlRequest::Ext { .. } => self.unsupported(&c, &ctx, reply),
        }
    }
}

impl Dispatch<'_> {
    fn send_status<R: DxlReply>(reply: &mut R, status: Status<'_>) {
        // TX overflow is a sizing bug, not a runtime error — bench telemetry
        // catches it at integration.
        let _ = reply.send_status(status);
    }

    /// Ack/error reply gate shared by every write-family handler: a Status
    /// leaves only when the bus permits it and SRL is `All`.
    fn ack<R: DxlReply>(c: &Ctx, ctx: &DxlRequestCtx, result: Result<(), Error>, reply: &mut R) {
        if !ctx.may_reply || c.level < StatusReturnLevel::All {
            return;
        }
        let error = match result {
            Ok(()) => StatusError::OK,
            Err(e) => error_to_status(e),
        };
        Self::send_status(reply, Status::Empty { id: c.id, error });
    }

    fn ping<R: DxlReply>(&mut self, c: &Ctx, ctx: &DxlRequestCtx, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        Self::send_status(
            reply,
            Status::Ping {
                id: c.id,
                error: StatusError::OK,
                status: PingStatus {
                    model: c.identity.model_number,
                    fw_version: c.identity.firmware_version,
                },
            },
        );
    }

    fn read<R: DxlReply>(
        &mut self,
        c: &Ctx,
        ctx: &DxlRequestCtx,
        address: u16,
        length: u16,
        reply: &mut R,
    ) {
        if !ctx.may_reply || c.level < StatusReturnLevel::Read {
            return;
        }
        let len = length as usize;
        if len == 0 || len > MAX_CONTROL_RW {
            // A FAST slot block has no error-only form; drop it silently. Plain
            // and coordinated Reads answer with an empty DataRange Status.
            if ctx.slot_reply {
                return;
            }
            Self::send_status(
                reply,
                Status::Empty {
                    id: c.id,
                    error: StatusError::code(ErrorCode::DataRange),
                },
            );
            return;
        }
        let chunks = self.shared.table.read_iter(address, length).map(into_chunk);
        if ctx.slot_reply {
            let _ = reply.send_slot_chunked(c.id, StatusError::OK, chunks);
        } else {
            let _ = reply.send_status_read_chunked(c.id, StatusError::OK, chunks);
        }
    }

    fn write<R: DxlReply>(
        &mut self,
        c: &Ctx,
        ctx: &DxlRequestCtx,
        address: u16,
        data: Bytes<'_>,
        reply: &mut R,
    ) {
        let mut scratch = [0u8; MAX_CONTROL_RW];
        let snap = self.staged.snapshot();
        let n = match data.copy_into(&mut scratch) {
            Ok(n) => n,
            Err(_) => return Self::ack(c, ctx, Err(Error::OutOfRange), reply),
        };
        if self.staged.push(address, &scratch[..n]).is_err() {
            self.staged.rewind_to(snap);
            return Self::ack(c, ctx, Err(Error::OutOfRange), reply);
        }
        // On both success and failure `write_bytes_iter` rewinds `staged` back
        // to `snap`, so any RegWrite chain staged below survives.
        let result = self
            .shared
            .table
            .write_bytes_iter(address, self.staged, &snap);
        let ok = result.is_ok();
        Self::ack(c, ctx, result, reply);
        if ok {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .dispatch_events(address, n as u16, &mut hooks);
        }
    }

    fn reg_write<R: DxlReply>(
        &mut self,
        c: &Ctx,
        ctx: &DxlRequestCtx,
        address: u16,
        data: Bytes<'_>,
        reply: &mut R,
    ) {
        let mut scratch = [0u8; MAX_CONTROL_RW];
        let snap = self.staged.snapshot();
        let n = match data.copy_into(&mut scratch) {
            Ok(n) => n,
            Err(_) => return Self::ack(c, ctx, Err(Error::OutOfRange), reply),
        };
        if self.staged.push(address, &scratch[..n]).is_err() {
            self.staged.rewind_to(snap);
            return Self::ack(c, ctx, Err(Error::OutOfRange), reply);
        }
        // Kept staged on success for a later Action; rewound to `snap` on error.
        let result = self
            .shared
            .table
            .stage_bytes_iter(address, self.staged, &snap);
        Self::ack(c, ctx, result, reply);
    }

    fn action<R: DxlReply>(&mut self, c: &Ctx, ctx: &DxlRequestCtx, reply: &mut R) {
        self.shared.table.commit_staged(self.staged);
        Self::ack(c, ctx, Ok(()), reply);
    }

    fn reboot<R: DxlReply>(&mut self, c: &Ctx, ctx: &DxlRequestCtx, reply: &mut R) {
        let mode = self.shared.table.control.with(|c| c.system.boot_mode);
        Self::ack(c, ctx, Ok(()), reply);
        reply.stage_reboot(mode);
    }

    fn unsupported<R: DxlReply>(&mut self, c: &Ctx, ctx: &DxlRequestCtx, reply: &mut R) {
        if !ctx.may_reply || c.level < StatusReturnLevel::All {
            return;
        }
        Self::send_status(
            reply,
            Status::Empty {
                id: c.id,
                error: StatusError::code(ErrorCode::Instruction),
            },
        );
    }
}

fn into_chunk(c: ReadChunk<'_>) -> Chunk<'_> {
    match c {
        ReadChunk::Copy(s) => Chunk::Slice(s),
        ReadChunk::Zero(n) => Chunk::Zero(n),
    }
}
