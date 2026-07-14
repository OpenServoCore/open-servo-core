use control_table::{RegisterFile, Snapshot};
use heapless::Vec;
use osc_protocol::FrameBytes;
use osc_protocol::frame::uid_prefix_matches;
use osc_protocol::table::STATUS_FLAG_CONFIG_DIRTY;
use osc_protocol::wire::{Id, MAX_PAYLOAD, MgmtOp, ResultCode, UID_LEN};

use crate::persist::{CONFIG_LEN, PROFILE_LEN, StoreError};
use crate::regions::hooks::ControlTableHooks;
use crate::regions::{
    CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE,
};
use crate::traits::{Dispatch, Dispatched, GATHER_MAX, Reply, Request, RequestCtx, Status};
use crate::{Error, RegionStorage, Shared, StagedWrites};
use control_table::STAGE_ENTRY_CAP;

/// Map a control-table write/stage failure onto an osc-native result code
/// (sec 5.3 layer 2). Read-only writes are `Access`; field-rule rejections are
/// `Validation`; the rest (bounds, staging-full) are `Range`.
fn error_to_result(e: Error) -> ResultCode {
    match e {
        Error::AccessError => ResultCode::Access,
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
    /// Staging watermark before this write -- commit/revert operate above it.
    snap: Snapshot,
    /// A held write keeps its entries for a later COMMIT; a plain write applies
    /// them at commit time.
    hold: bool,
    /// The written span, for firing hooks at commit.
    addr: u16,
    len: u16,
}

/// Stateless single-shot dispatcher. Holds only borrowed shared state, the
/// HOLD-write staging buffer, and the pending-write slot -- no per-frame
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
    /// zero watermark -- otherwise a real COMMIT would apply the phantom bytes.
    fn revert_dangling(&mut self) {
        if let Some(pending) = self.pending.take() {
            self.staged.revert_to(&pending.snap);
        }
    }

    /// Device-level ALERT bit: set on every status while the alarm register is
    /// nonzero (sec 5.3 layer 3).
    fn alert(&self) -> bool {
        self.shared
            .table
            .with(|t| t.telemetry.common.fault_flags != 0)
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
        // write -- drop it here (see revert_dangling) before this frame reads
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
            Request::ReadProfile { slot } => {
                self.read_profile(alert, &ctx, slot, reply);
                Dispatched::Done
            }
            Request::Write { addr, data, hold } => self.write(alert, &ctx, addr, data, hold, reply),
            // Verdict-first ops (trait contract): the bus CRC-checked the
            // frame before dispatching, so applying directly is sound.
            Request::Commit => {
                self.apply_commit(alert, &ctx, reply);
                Dispatched::Done
            }
            Request::Enumerate { prefix_len, prefix } => {
                self.enumerate(alert, &ctx, prefix_len, &prefix, reply);
                Dispatched::Done
            }
            Request::Assign { uid, new_id } => {
                self.assign(alert, &ctx, &uid, new_id, reply);
                Dispatched::Done
            }
            // sec 9.3: broadcast-only (decode enforces), so no ack precedes the
            // train -- an ack's own break would count as a ruler mark.
            Request::Calibrate { gap_us, gaps } => {
                reply.begin_clock_cal(gap_us, gaps);
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
        self.mark_dirty_if_persistent(pending.addr, pending.len);
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
        // TX overflow is a sizing bug, not a runtime error -- bench telemetry
        // catches it at integration.
        let _ = reply.send_status(status);
    }

    /// Empty-payload status gated on the reply contract (sec 5.3 layer 2).
    fn ack<R: Reply>(alert: bool, ctx: &RequestCtx, result: Result<(), Error>, reply: &mut R) {
        let code = match result {
            Ok(()) => ResultCode::Ok,
            Err(e) => error_to_result(e),
        };
        Self::ack_code(alert, ctx, code, reply);
    }

    /// As [`Self::ack`] with an explicit result code (the store's `hardware`
    /// verdict has no `Error` mapping).
    fn ack_code<R: Reply>(alert: bool, ctx: &RequestCtx, code: ResultCode, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        Self::send(
            reply,
            Status {
                result: code,
                alert,
                data: &[],
            },
        );
    }

    /// sec 9.4 modified-since-save: a committed span landing in CONFIG or
    /// PROFILE sets the telemetry dirty bit (a successful SAVE clears it).
    fn mark_dirty_if_persistent(&self, addr: u16, len: u16) {
        const CONFIG_END: u16 = CONFIG_BASE_ADDR + CONFIG_REGION_SIZE;
        const PROFILE_END: u16 = PROFILE_BASE_ADDR + PROFILE_REGION_SIZE;
        let end = addr.saturating_add(len);
        // CONFIG starts at 0, so "before its end" is its whole intersect test.
        let hits = addr < CONFIG_END || (addr < PROFILE_END && end > PROFILE_BASE_ADDR);
        if hits {
            self.shared
                .table
                .with_mut(|t| t.telemetry.common.status_flags |= STATUS_FLAG_CONFIG_DIRTY);
        }
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
        // model_number + firmware_version are contiguous at the common-block front:
        // reply straight from the table so the TX engine streams in place (a
        // stack copy would force the slow copy-stage path).
        let data = RegisterFile::read(
            &self.shared.table,
            crate::regions::config::addr::common::MODEL_NUMBER,
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

    /// PROFILE read (sec 5.2): resolve the slot's span words against the live
    /// table and gather-send the concatenation. Errors are read-time (sec 5.3):
    /// a bad slot index, an empty slot, or an out-of-bounds span is `range`;
    /// a total past the frame ceiling is `limit`.
    fn read_profile<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, slot: u8, reply: &mut R) {
        if !ctx.may_reply {
            return;
        }
        let nack = |reply: &mut R, result: ResultCode| {
            Self::send(
                reply,
                Status {
                    result,
                    alert,
                    data: &[],
                },
            )
        };
        let words = self
            .shared
            .table
            .with(|t| t.profile.slot_words(slot).copied());
        let Some(words) = words else {
            return nack(reply, ResultCode::Range);
        };
        let mut spans: [&[u8]; GATHER_MAX] = [&[]; GATHER_MAX];
        let mut n = 0;
        let mut total: u16 = 0;
        for w in words {
            let Some((addr, count)) = crate::regions::profile::span_of(w) else {
                continue; // disabled word: skipped, not a terminator (sec 5.2)
            };
            let Ok(data) = RegisterFile::read(&self.shared.table, addr, count) else {
                return nack(reply, ResultCode::Range);
            };
            spans[n] = data;
            n += 1;
            total += count;
        }
        if n == 0 {
            return nack(reply, ResultCode::Range);
        }
        if total > MAX_PAYLOAD as u16 {
            return nack(reply, ResultCode::Limit);
        }
        let _ = reply.send_status_gather(ResultCode::Ok, alert, &spans[..n]);
    }

    /// Write, the spine way: validate + stage above a watermark, ack per the
    /// reply contract, but leave the live table untouched until the verdict's
    /// [`Dispatch::commit`]. A validation reject nacks with nothing staged
    /// (`Done`, no commit owed); a staging-capacity overflow (held entries
    /// filled the buffer -- a COMMIT drains it) nacks `Busy`, nothing staged.
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
            self.mark_dirty_if_persistent(addr, len);
            self.shared
                .table
                .with(|t| t.dispatch_events(addr, len, &mut hooks));
        }
    }

    /// sec 9.2 ENUM: reply with the full UID iff ours begins with the queried
    /// prefix. A mismatch stays silent -- on the broadcast wire, silence and
    /// collision garbage are the host's two tree-descent signals, and a nack
    /// would only manufacture collisions.
    fn enumerate<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        prefix_len: u8,
        prefix: &[u8; UID_LEN],
        reply: &mut R,
    ) {
        if !ctx.may_reply {
            return;
        }
        let uid = self.shared.uid();
        if !uid_prefix_matches(uid, prefix_len, prefix) {
            return;
        }
        Self::send(
            reply,
            Status {
                result: ResultCode::Ok,
                alert,
                data: uid,
            },
        );
    }

    /// sec 9.2 ASSIGN: the servo whose UID matches takes the id -- applied
    /// immediately (`set_id`, not the deferred `stage_id`) so the ack leaves
    /// from the new id, and mirrored into the config ID register so a later
    /// SAVE persists it (volatile until then). Non-matching servos stay
    /// silent; the sole matching servo can nack without colliding.
    fn assign<R: Reply>(
        &mut self,
        alert: bool,
        ctx: &RequestCtx,
        uid: &[u8; UID_LEN],
        new_id: u8,
        reply: &mut R,
    ) {
        if uid != self.shared.uid() {
            return;
        }
        if Id::try_unicast(new_id).is_none() {
            let e = Error::ValidationError(control_table::ValidationKind::Compare);
            return Self::ack(alert, ctx, Err(e), reply);
        }
        self.shared.table.with_mut(|t| {
            t.config.common.id = new_id;
            t.telemetry.common.status_flags |= STATUS_FLAG_CONFIG_DIRTY;
        });
        reply.set_id(new_id);
        Self::ack(alert, ctx, Ok(()), reply);
    }

    fn mgmt<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, op: MgmtOp, reply: &mut R) {
        match op {
            MgmtOp::Save => self.save(alert, ctx, reply),
            MgmtOp::Factory => self.factory(alert, ctx, reply),
            MgmtOp::Reboot => {
                Self::ack(alert, ctx, Ok(()), reply);
                let mode = self.shared.table.with(|t| t.control.system.boot_mode);
                reply.stage_reboot(mode);
            }
            // ENUM/ASSIGN/CAL decode to dedicated Request variants; one
            // arriving Mgmt-wrapped is answered like any unknown op.
            MgmtOp::Enum | MgmtOp::Assign | MgmtOp::Cal => {
                self.instruction_error(alert, ctx, reply)
            }
        }
    }

    /// sec 9.4 SAVE -- the only flash-touching operation. Torque gates it (the
    /// ms-scale program stall is the mid-motion hazard, not the data), and
    /// the ack leaves AFTER the store returns: ack == durable, and a failed
    /// program surfaces as `hardware` instead of a lie.
    fn save<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        if self
            .shared
            .table
            .with(|t| t.control.lifecycle.torque_enable)
        {
            return Self::ack(alert, ctx, Err(Error::AccessError), reply);
        }
        let saved = self.persist_table();
        let code = match saved {
            Ok(()) => {
                self.shared
                    .table
                    .with_mut(|t| t.telemetry.common.status_flags &= !STATUS_FLAG_CONFIG_DIRTY);
                ResultCode::Ok
            }
            Err(StoreError) => ResultCode::Hardware,
        };
        Self::ack_code(alert, ctx, code, reply);
    }

    /// Stream the persisted regions into the store -- the slices borrow the
    /// table in place (no staging copy, sec 9.4); blocking for the erase +
    /// program duration.
    fn persist_table(&self) -> Result<(), StoreError> {
        let store = self.shared.store().ok_or(StoreError)?;
        // Bounded by the region consts -- the reads cannot fail; the fallback
        // keeps the no-panic contract.
        let config: &[u8; CONFIG_LEN] =
            RegisterFile::read(&self.shared.table, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE)
                .ok()
                .and_then(|s| s.try_into().ok())
                .ok_or(StoreError)?;
        let profile: &[u8; PROFILE_LEN] =
            RegisterFile::read(&self.shared.table, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE)
                .ok()
                .and_then(|s| s.try_into().ok())
                .ok_or(StoreError)?;
        store.save(config, profile)
    }

    /// sec 9.5 FACTORY: wipe both saved slots, ack, then stage the reboot that
    /// re-seeds board defaults -- the erased store IS the factory state.
    /// Same torque gate as SAVE (erase is the same stall class, and it ends
    /// in a reboot); a failed wipe nacks `hardware` and does NOT reboot.
    fn factory<R: Reply>(&mut self, alert: bool, ctx: &RequestCtx, reply: &mut R) {
        if self
            .shared
            .table
            .with(|t| t.control.lifecycle.torque_enable)
        {
            return Self::ack(alert, ctx, Err(Error::AccessError), reply);
        }
        let wiped = self.shared.store().ok_or(StoreError).and_then(|s| s.wipe());
        match wiped {
            Ok(()) => {
                Self::ack(alert, ctx, Ok(()), reply);
                let mode = self.shared.table.with(|t| t.control.system.boot_mode);
                reply.stage_reboot(mode);
            }
            Err(StoreError) => Self::ack_code(alert, ctx, ResultCode::Hardware, reply),
        }
    }
}
