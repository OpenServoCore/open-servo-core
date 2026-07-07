use control_table::{RegisterFile, Snapshot};
use heapless::Vec;
use osc_protocol::FrameBytes;
use osc_protocol::wire::{MAX_PAYLOAD, MgmtOp, ResultCode};

use crate::regions::hooks::ControlTableHooks;
use crate::traits::{Dispatch, Reply, Request, RequestCtx, Speculated, Status};
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

/// A speculative write staged at covered-complete, awaiting a CRC verdict. The
/// bus rebuilds the [`Dispatcher`] each wake, so this lives in the [`Session`]
/// and outlives the dispatcher that staged it.
///
/// [`Session`]: super::session::Session
pub(crate) struct SpecWrite {
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
/// HOLD-write staging buffer, and the pending speculative-write slot — no
/// per-frame reassembly; each [`Dispatch::dispatch`] call carries its whole
/// payload.
pub struct Dispatcher<'a> {
    shared: &'a Shared,
    staged: &'a mut StagedWrites,
    spec: &'a mut Option<SpecWrite>,
}

impl<'a> Dispatcher<'a> {
    pub(crate) fn new(
        shared: &'a Shared,
        staged: &'a mut StagedWrites,
        spec: &'a mut Option<SpecWrite>,
    ) -> Self {
        Self {
            shared,
            staged,
            spec,
        }
    }

    /// Discard a speculative write left dangling by a frame that died before its
    /// CRC verdict (the bus has no dispatcher in the break ISR, so cleanup is
    /// lazy). MUST run before any path that reads the staging buffer from the
    /// zero watermark — otherwise a real COMMIT would apply the phantom bytes.
    fn revert_dangling(&mut self) {
        if let Some(spec) = self.spec.take() {
            self.staged.revert_to(&spec.snap);
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
    fn dispatch<R: Reply>(&mut self, req: Request<'_>, ctx: RequestCtx, reply: &mut R) {
        // Any speculative write still pending here died without a verdict — drop
        // it so COMMIT and fresh writes see a clean buffer (see revert_dangling).
        self.revert_dangling();
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

    fn dispatch_speculative<R: Reply>(
        &mut self,
        req: Request<'_>,
        ctx: RequestCtx,
        reply: &mut R,
    ) -> Speculated {
        // A prior speculated frame that never got its verdict is dropped here
        // too (see revert_dangling) before this frame stages anything.
        self.revert_dangling();
        let alert = self.alert();
        match req {
            Request::Ping => {
                self.ping(alert, &ctx, reply);
                Speculated::Done
            }
            Request::Read { addr, count } => {
                self.read(alert, &ctx, addr, count, reply);
                Speculated::Done
            }
            Request::Write { addr, data, hold } => {
                self.write_speculative(alert, &ctx, addr, data, hold, reply)
            }
            // COMMIT applies the whole buffer, MGMT reboots, Unsupported errors:
            // none are side-effect-free before the CRC gate → full path at B.
            Request::Commit | Request::Mgmt { .. } | Request::Unsupported => Speculated::Refused,
        }
    }

    fn commit_speculation<R: Reply>(&mut self, reply: &mut R) {
        let Some(spec) = self.spec.take() else {
            return; // defensive: nothing speculated
        };
        if spec.hold {
            return; // held entries stay staged until a real COMMIT
        }
        self.shared.table.commit_from(self.staged, &spec.snap);
        let mut hooks = ControlTableHooks::new(reply);
        self.shared
            .table
            .with(|t| t.dispatch_events(spec.addr, spec.len, &mut hooks));
    }

    fn revert_speculation(&mut self) {
        if let Some(spec) = self.spec.take() {
            self.staged.revert_to(&spec.snap);
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
        // §5: read addresses must be even — replies stream zero-copy through
        // the halfword CRC engine, and an odd-addressed span breaks its
        // pairing (F12). Same rule as §5.2 profile spans.
        if count == 0 || addr & 1 == 1 {
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
        data: FrameBytes<'_>,
        hold: bool,
        reply: &mut R,
    ) {
        let (head, tail) = data.segments();
        if hold {
            let result = self.shared.table.stage_split(addr, head, tail, self.staged);
            return Self::ack(alert, ctx, result, reply);
        }
        let result = self.shared.table.write_split(addr, head, tail);
        let ok = result.is_ok();
        Self::ack(alert, ctx, result, reply);
        if ok {
            let mut hooks = ControlTableHooks::new(reply);
            self.shared
                .table
                .with(|t| t.dispatch_events(addr, data.len() as u16, &mut hooks));
        }
    }

    /// Speculative write: validate + stage above a watermark, ack per the reply
    /// contract, but leave the live table untouched until `commit_speculation`.
    /// A validation reject nacks with nothing staged (`Done`, no commit needed);
    /// a staging-capacity refusal stages nothing and stays silent (`Refused`, so
    /// the full path re-runs at frame end without a double ack).
    fn write_speculative<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        addr: u16,
        data: FrameBytes<'_>,
        hold: bool,
        reply: &mut R,
    ) -> Speculated {
        let (head, tail) = data.segments();
        let snap = self.staged.snapshot();
        match self.shared.table.stage_split(addr, head, tail, self.staged) {
            Ok(()) => {
                Self::ack(alert, ctx, Ok(()), reply);
                *self.spec = Some(SpecWrite {
                    snap,
                    hold,
                    addr,
                    len: data.len() as u16,
                });
                Speculated::Pending
            }
            // Buffer full (staging cap or payload too big): nothing was staged;
            // the full write_split path applies it at frame end.
            Err(Error::StagingFull) => Speculated::Refused,
            // Bounds / rule / lock reject: validate pushed nothing. Nack now; the
            // error reply is sequenced iff the CRC passes.
            Err(e) => {
                Self::ack(alert, ctx, Err(e), reply);
                Speculated::Done
            }
        }
    }

    fn commit<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
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
