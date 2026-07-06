use control_table::RegisterFile;
use osc_protocol::wire::{MAX_PAYLOAD, MgmtOp, ResultCode};

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{Dispatch, Reply, Request, RequestCtx, Status};
use crate::{Error, RegionStorage, Shared, StagedWrites, ValidationKind};

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

/// Stateless single-shot dispatcher. Holds only borrowed shared state and the
/// HOLD-write staging buffer — no per-frame reassembly; each
/// [`Dispatch::dispatch`] call carries its whole payload.
pub struct Dispatcher<'a> {
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
}

impl<'a> Dispatcher<'a> {
    pub fn new(shared: &'a Shared, staged: &'a mut StagedWrites) -> Self {
        Self { shared, staged }
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
    fn dispatch<R: Reply>(&mut self, req: Request<'_>, ctx: RequestCtx, reply: &mut R) {
        let alert = self.alert();
        match req {
            Request::Ping => self.ping(alert, &ctx, reply),
            Request::Read { addr, count } => self.read(alert, &ctx, addr, count, reply),
            Request::Write { addr, data, hold } => self.write(alert, &ctx, addr, data, hold, reply),
            Request::Commit => self.commit(alert, &ctx, reply),
            Request::Mgmt { op, .. } => self.mgmt(alert, &ctx, op, reply),
            Request::Unsupported => self.instruction_error(alert, &ctx, reply),
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
        let (model, fw) = self.shared.table.with(|t| {
            (
                t.config.identity.model_number,
                t.config.identity.firmware_version,
            )
        });
        let m = model.to_le_bytes();
        Self::send(
            reply,
            Status {
                result: ResultCode::Ok,
                alert,
                data: &[m[0], m[1], fw],
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

    fn write<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        addr: u16,
        data: &[u8],
        hold: bool,
        reply: &mut R,
    ) {
        if hold {
            let result = self.shared.table.stage(addr, data, self.staged);
            return Self::ack(alert, ctx, result, reply);
        }
        let result = self.shared.table.write(addr, data);
        let ok = result.is_ok();
        Self::ack(alert, ctx, result, reply);
        if ok {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .with(|t| t.dispatch_events(addr, data.len() as u16, &mut hooks));
        }
    }

    fn commit<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        self.shared.table.commit_staged(self.staged);
        Self::ack(alert, ctx, Ok(()), reply);
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
