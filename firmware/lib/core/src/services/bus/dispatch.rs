use control_table::{RegisterFile, Snapshot};
use heapless::Vec;
use osc_protocol::FrameBytes;
use osc_protocol::wire::{MAX_PAYLOAD, MgmtOp, ResultCode};

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{Dispatch, Dispatched, Reply, Request, RequestCtx, Status};
use crate::{Error, RegionStorage, Shared, StagedWrites, ValidationKind};
use control_table::STAGE_ENTRY_CAP;

/// Map a control-table write/stage failure onto an osc-native result code
/// (§5.3 layer 2). Read-only and torque-locked writes are `Access`; field-rule
/// rejections are `Validation`; the rest (bounds, staging-full) are `Range`.
fn error_to_result(e: Error) -> ResultCode {
    match e {
        Error::AccessError | Error::ValidationError(ValidationKind::Locked) => ResultCode::Access,
        Error::ValidationError(_) => ResultCode::Validation,
        _ => ResultCode::Range,
    }
}

/// A write staged before its CRC verdict (the dispatch spine). The bus
/// rebuilds the [`Dispatcher`] each wake, so this lives in the [`Session`]
/// and outlives the dispatcher that staged it.
///
/// [`Session`]: super::session::Session
pub(crate) struct PendingWrite {
    /// Staging watermark before this write — commit/revert operate above it.
    snap: Snapshot,
    /// A held write keeps its entries for a later COMMIT; a plain write applies
    /// them at commit time.
    hold: bool,
    /// The written span, for firing hooks at commit.
    addr: u16,
    len: u16,
}

/// Stateless single-shot dispatcher. Holds only borrowed shared state, the
/// HOLD-write staging buffer, and the pending-write slot — no per-frame
/// reassembly; each [`Dispatch::dispatch`] call carries its whole payload.
pub struct Dispatcher<'a> {
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
    pending: &'a mut Option<PendingWrite>,
}

impl<'a> Dispatcher<'a> {
    pub(crate) fn new(
        shared: &'a Shared,
        staged: &'a mut StagedWrites,
        pending: &'a mut Option<PendingWrite>,
    ) -> Self {
        Self {
            shared,
            staged,
            pending,
        }
    }

    /// Discard a pending write left dangling by a frame that died before its
    /// CRC verdict (the bus has no dispatcher in the break ISR, so cleanup is
    /// lazy). MUST run before any path that reads the staging buffer from the
    /// zero watermark — otherwise a real COMMIT would apply the phantom bytes.
    fn revert_dangling(&mut self) {
        if let Some(pending) = self.pending.take() {
            self.staged.revert_to(&pending.snap);
        }
    }

    /// Device-level ALERT bit: set on every status while the alarm register is
    /// nonzero (§5.3 layer 3).
    fn alert(&self) -> bool {
        self.shared
            .table
            .with(|t| t.telemetry.fault.fault_flags != 0)
    }
}

impl Dispatch for Dispatcher<'_> {
    fn dispatch<R: Reply>(
        &mut self,
        req: Request<'_>,
        ctx: RequestCtx,
        reply: &mut R,
    ) -> Dispatched {
        // A prior frame that died without its verdict left a dangling pending
        // write — drop it here (see revert_dangling) before this frame reads
        // or stages anything.
        self.revert_dangling();
        let alert = self.alert();
        match req {
            Request::Ping => {
                self.ping(alert, &ctx, reply);
                Dispatched::Done
            }
            Request::Read { addr, count } => {
                self.read(alert, &ctx, addr, count, reply);
                Dispatched::Done
            }
            Request::Write { addr, data, hold } => self.write(alert, &ctx, addr, data, hold, reply),
            // Verdict-first ops (trait contract): the bus CRC-checked the
            // frame before dispatching, so applying directly is sound.
            Request::Commit => {
                self.apply_commit(alert, &ctx, reply);
                Dispatched::Done
            }
            Request::Mgmt { op, .. } => {
                self.mgmt(alert, &ctx, op, reply);
                Dispatched::Done
            }
            Request::Unsupported => {
                self.instruction_error(alert, &ctx, reply);
                Dispatched::Done
            }
        }
    }

    fn commit<R: Reply>(&mut self, reply: &mut R) {
        let Some(pending) = self.pending.take() else {
            return; // defensive: nothing pending
        };
        if pending.hold {
            return; // held entries stay staged until a real COMMIT
        }
        self.shared.table.commit_from(self.staged, &pending.snap);
        let mut hooks = ControlTableHooks::new(reply);
        self.shared
            .table
            .with(|t| t.dispatch_events(pending.addr, pending.len, &mut hooks));
    }

    fn revert(&mut self) {
        if let Some(pending) = self.pending.take() {
            self.staged.revert_to(&pending.snap);
        }
    }
}

impl Dispatcher<'_> {
    fn send<R: Reply>(reply: &mut R, status: Status<'_>) {
        // TX overflow is a sizing bug, not a runtime error — bench telemetry
        // catches it at integration.
        let _ = reply.send_status(status);
    }

    /// Empty-payload status gated on the reply contract (§5.3 layer 2).
    fn ack<R: Reply>(alert: bool, ctx: &RequestCtx, result: Result<(), Error>, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        let code = match result {
            Ok(()) => ResultCode::Ok,
            Err(e) => error_to_result(e),
        };
        Self::send(
            reply,
            Status {
                result: code,
                alert,
                data: &[],
            },
        );
    }

    fn instruction_error<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        Self::send(
            reply,
            Status {
                result: ResultCode::Instruction,
                alert,
                data: &[],
            },
        );
    }

    fn ping<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        // model_number + firmware_version are contiguous at the identity base:
        // reply straight from the table so the TX engine streams in place (a
        // stack copy would force the slow copy-stage path).
        let data = RegisterFile::read(
            &self.shared.table,
            crate::regions::config::addr::identity::MODEL_NUMBER,
            3,
        )
        .unwrap_or(&[]);
        Self::send(
            reply,
            Status {
                result: ResultCode::Ok,
                alert,
                data,
            },
        );
    }

    fn read<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        addr: u16,
        count: u16,
        reply: &mut R,
    ) {
        if !ctx.may_reply {
            return;
        }
        if count == 0 {
            return Self::send(
                reply,
                Status {
                    result: ResultCode::Range,
                    alert,
                    data: &[],
                },
            );
        }
        if count > MAX_PAYLOAD as u16 {
            return Self::send(
                reply,
                Status {
                    result: ResultCode::Limit,
                    alert,
                    data: &[],
                },
            );
        }
        match RegisterFile::read(&self.shared.table, addr, count) {
            Ok(data) => Self::send(
                reply,
                Status {
                    result: ResultCode::Ok,
                    alert,
                    data,
                },
            ),
            Err(_) => Self::send(
                reply,
                Status {
                    result: ResultCode::Range,
                    alert,
                    data: &[],
                },
            ),
        }
    }

    /// Write, the spine way: validate + stage above a watermark, ack per the
    /// reply contract, but leave the live table untouched until the verdict's
    /// [`Dispatch::commit`]. A validation reject nacks with nothing staged
    /// (`Done`, no commit owed); a staging-capacity overflow (held entries
    /// filled the buffer — a COMMIT drains it) nacks `Busy`, nothing staged.
    fn write<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        addr: u16,
        data: FrameBytes<'_>,
        hold: bool,
        reply: &mut R,
    ) -> Dispatched {
        let (head, tail) = data.segments();
        let snap = self.staged.snapshot();
        match self.shared.table.stage_split(addr, head, tail, self.staged) {
            Ok(()) => {
                Self::ack(alert, ctx, Ok(()), reply);
                *self.pending = Some(PendingWrite {
                    snap,
                    hold,
                    addr,
                    len: data.len() as u16,
                });
                Dispatched::Pending
            }
            Err(Error::StagingFull) => {
                if ctx.may_reply {
                    Self::send(
                        reply,
                        Status {
                            result: ResultCode::Busy,
                            alert,
                            data: &[],
                        },
                    );
                }
                Dispatched::Done
            }
            // Bounds / rule / lock reject: validate pushed nothing. Nack now;
            // the error reply is sequenced iff the CRC passes.
            Err(e) => {
                Self::ack(alert, ctx, Err(e), reply);
                Dispatched::Done
            }
        }
    }

    fn apply_commit<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        // Capture each entry's span before commit_staged clears the buffer, so
        // hooks fire on the applied values (apply-then-events, mirroring write).
        let mut spans: Vec<(u16, u16), STAGE_ENTRY_CAP> = Vec::new();
        for (addr, data) in self.staged.iter_all() {
            let _ = spans.push((addr, data.len() as u16));
        }
        self.shared.table.commit_staged(self.staged);
        Self::ack(alert, ctx, Ok(()), reply);
        let mut hooks = ControlTableHooks::new(reply);
        for (addr, len) in spans {
            self.shared
                .table
                .with(|t| t.dispatch_events(addr, len, &mut hooks));
        }
    }

    fn mgmt<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, op: MgmtOp, reply: &mut R) {
        match op {
            MgmtOp::Reboot => {
                Self::ack(alert, ctx, Ok(()), reply);
                let mode = self.shared.table.with(|t| t.control.system.boot_mode);
                reply.stage_reboot(mode);
            }
            _ => self.instruction_error(alert, ctx, reply),
        }
    }
}
