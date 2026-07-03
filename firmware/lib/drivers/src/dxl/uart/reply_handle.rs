//! The dispatcher-facing reply handle — disjoint borrows of the `DxlUart`
//! composite's send-side halves, built per parser event by
//! [`DxlUart::poll`].
//!
//! [`DxlUart::poll`]: super::DxlUart::poll

use dxl_protocol::{Chunk, Id, Slot, SlotPosition, Status, StatusError, WriteError};
use osc_core::{BaudRate, BootMode, DxlReply};

use super::clock::Clock;
use super::codec::CodecTx;
use super::fast_last::{FastLast, FastLastSchedule};
use super::send_policy::{ReplyContext, SendPolicy};
use crate::traits::dxl::{Providers, SendKind, TxScheduler};

/// The context's RDT in scheduler ticks — the single µs→ticks conversion
/// point for both the status and slot schedule paths. The factor is a
/// `TxScheduler` const, so the µs value never leaves this file's
/// scheduling math.
fn rdt_ticks<P: Providers>(ctx: &ReplyContext) -> u32 {
    ctx.rdt_us
        .wrapping_mul(<P::TxScheduler as TxScheduler>::TICKS_PER_US as u32)
}

/// A reply handle borrowed from disjoint pieces of [`DxlUart`] — the codec
/// TX half + scheduler + clock + the small set of pending-state fields. The
/// parent's closure-based [`DxlUart::poll`] hands the dispatcher one of
/// these alongside the parsed packet (which borrows the codec RX half), so
/// the dispatcher can call `send_status` / `send_slot` / `stage_*` /
/// `cancel` without a borrow conflict against the packet.
///
/// Implements [`osc_core::DxlReply`] — the chip-side `Ch32Bus::poll` forwards
/// the handle straight to the user closure as `&mut dyn DxlReply`. The
/// inherent methods stay so driver-crate tests (which don't import the trait)
/// can drive the handle directly.
pub struct ReplyHandle<'a, P: Providers, const TX_BUF_LEN: usize> {
    pub(super) tx: &'a mut CodecTx<P::Crc, TX_BUF_LEN>,
    pub(super) scheduler: &'a mut P::TxScheduler,
    pub(super) fast_last: &'a mut FastLast<P::FastLastScheduler, P::Crc>,
    pub(super) clock: &'a mut Clock<P::UsartBaud, P::ClockTrim>,
    /// Send-policy sub-composite on the parent — bus identity, the
    /// staged-config mailbox the `stage_*` commands write into, and the
    /// reply gate holding the staged [`ReplyContext`] / chain-pending
    /// wait. Live values can't shift under the handle: they only mutate
    /// at `on_tx_complete`, which can't run while the handle holds this
    /// borrow.
    pub(super) send_policy: &'a mut SendPolicy,
}

impl<P: Providers, const TX_BUF_LEN: usize> ReplyHandle<'_, P, TX_BUF_LEN> {
    /// Encode a Status reply into the codec's TX buffer and either schedule
    /// its wire start (single-target / Plain chain slot 0) or stage the
    /// chain-pending state (Plain chain slot k > 0; see
    /// `docs/dxl-streaming-rx.md` §5.2). For the scheduled path: fold RDT +
    /// slot offset (broadcast Ping / Sync / Bulk Read slot N) from the
    /// [`ReplyContext`] cached at `poll()`, convert the µs delay to
    /// scheduler ticks via `TxScheduler::TICKS_PER_US`, and hand the
    /// pre-computed `deadline_tick` to the provider. For the chain-pending
    /// path: record the predecessor's ID on the parent driver; the codec's
    /// matching `PollEvent::SkipComplete` invokes `TxBus::start_now`. If no
    /// context is staged (foreign Instruction filtered upstream) or the
    /// wire-end tick wasn't ready (and the path needs it), the encode still
    /// succeeds but no schedule is armed — the bytes simply won't ship.
    /// Context is taken on use so a double-send without a fresh `poll()`
    /// no-ops on the scheduler.
    pub fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        crate::log::trace!("dxl: send_status entry");
        self.tx.send_status(status)?;
        self.schedule_after_status_encode();
        Ok(())
    }

    /// Streamed counterpart of [`Self::send_status`] for `Status::Read`
    /// replies: the dispatcher hands a [`Chunk`] iterator from a
    /// control-table read, skipping the 128 B scratch buffer. Encode +
    /// scheduling shape are identical to [`Self::send_status`].
    pub fn send_status_read_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        crate::log::trace!("dxl: send_status_read_chunked entry");
        self.tx.send_status_read_chunked(id, error, chunks)?;
        self.schedule_after_status_encode();
        Ok(())
    }

    /// Post-encode scheduling shared by [`Self::send_status`] and the
    /// streamed variants. Plain chain k > 0 defers to predecessor; every
    /// other path lands a single `Plain` schedule entry against the
    /// cached `packet_end_tick` + RDT + slot offset.
    fn schedule_after_status_encode(&mut self) {
        let Some(ctx) = self.send_policy.take_reply_context() else {
            crate::log::debug!("dxl: send_status drop (no reply ctx)");
            return;
        };
        if let Some(pred) = ctx.predecessor_id {
            crate::log::debug!("dxl: send_status defer to predecessor={}", pred);
            self.send_policy.defer_to_predecessor(pred);
            return;
        }
        let packet_end_tick = ctx.packet_end_tick;
        let byte_count = self.tx.tx_len();
        let deadline = self.clock.compute_status_deadline(
            packet_end_tick,
            rdt_ticks::<P>(&ctx),
            ctx.slot_offset_bytes,
        );
        crate::log::debug!(
            "dxl: send_status schedule packet_end={} deadline={} byte_count={}",
            packet_end_tick,
            deadline,
            byte_count
        );
        self.scheduler
            .schedule(deadline, byte_count, SendKind::Plain);
    }

    /// Encode one Fast Sync/Bulk Read slot reply and schedule its wire
    /// start. Slot position (Only/First/Middle/Last) comes from the cached
    /// [`ReplyContext`]; Last tags the schedule with [`SendKind::FastLast`]
    /// so the provider knows to coordinate with chain-CRC catchup (M6,
    /// `#6`). Slot offset composes in tick-space directly (see
    /// [`Clock::bytes_to_ticks`]) so the inter-slot gap stays exact at
    /// every baud, including the ~3.33 µs at 3 Mbaud.
    pub fn send_slot(&mut self, slot: &Slot<'_>) -> Result<(), WriteError> {
        let Some((ctx, position)) = self.take_slot_ctx() else {
            return Ok(());
        };
        crate::log::debug!(
            "dxl[id={}]: send_slot slot={:?} position={:?}",
            self.send_policy.id(),
            slot,
            position
        );
        self.tx.send_slot(slot, position)?;
        self.schedule_after_slot_encode(ctx, position);
        Ok(())
    }

    /// Streamed counterpart of [`Self::send_slot`]: slot body bytes come
    /// from a [`Chunk`] iterator sourced directly from a control-table
    /// read. The cached `ReplyContext` provides the slot position; the
    /// post-encode schedule + Fast Last arm path is identical to
    /// [`Self::send_slot`].
    pub fn send_slot_chunked<'c, I>(
        &mut self,
        id: Id,
        error: StatusError,
        chunks: I,
    ) -> Result<(), WriteError>
    where
        I: IntoIterator<Item = Chunk<'c>>,
    {
        let Some((ctx, position)) = self.take_slot_ctx() else {
            return Ok(());
        };
        crate::log::debug!(
            "dxl[id={}]: send_slot_chunked id={} err={:?} position={:?}",
            self.send_policy.id(),
            id.as_byte(),
            error,
            position,
        );
        self.tx.send_slot_chunked(id, error, position, chunks)?;
        self.schedule_after_slot_encode(ctx, position);
        Ok(())
    }

    /// Take the cached reply context iff it carries a Fast slot position;
    /// shared between the slice and streamed slot paths. Logs and returns
    /// `None` on the two drop cases (no context, no slot).
    fn take_slot_ctx(&mut self) -> Option<(ReplyContext, SlotPosition)> {
        let ctx = self.send_policy.take_reply_context().or_else(|| {
            crate::log::debug!(
                "dxl[id={}]: send_slot drop (no reply ctx)",
                self.send_policy.id()
            );
            None
        })?;
        let Some(position) = ctx.fast_slot_position else {
            crate::log::debug!(
                "dxl[id={}]: send_slot drop (no fast slot pos)",
                self.send_policy.id()
            );
            return None;
        };
        Some((ctx, position))
    }

    /// Post-encode scheduling for a slot reply: anchor against
    /// `packet_end_tick`, fold RDT + per-source floor + slot offset into
    /// the deadline, tag the entry `FastLast` for Last, and arm the
    /// chain-CRC fold engine on Last.
    fn schedule_after_slot_encode(&mut self, ctx: ReplyContext, position: SlotPosition) {
        let byte_count = self.tx.tx_len();
        let packet_end_tick = ctx.packet_end_tick;
        // The source-floored effective RDT (see `Clock::effective_slot_rdt`)
        // keeps every slot in a Fast chain anchored off the same base above
        // `packet_end_tick`, so the wire stays contiguous instead of slot
        // k > 0 starting inside slot 0's trailing TX.
        let timing = self.clock.compute_slot_deadline(
            packet_end_tick,
            rdt_ticks::<P>(&ctx),
            ctx.slot_offset_bytes,
            ctx.src,
        );
        let kind = match position {
            SlotPosition::Last { .. } => SendKind::FastLast,
            _ => SendKind::Plain,
        };
        self.scheduler.schedule(timing.deadline, byte_count, kind);
        if matches!(position, SlotPosition::Last { .. }) {
            // `SlotTiming` carries the effective RDT the deadline above was
            // derived from, so the fold grid back-dates from the same
            // anchor by construction. `FastLastSchedule::rdt_ticks` is u16
            // per doc §10.6 — the effective RDT stays well under 16 bits
            // at every supported baud (9600 floor = 50_000 ticks).
            debug_assert!(timing.effective_rdt_ticks <= u16::MAX as u32);
            self.fast_last.start(FastLastSchedule {
                packet_end_tick,
                rdt_ticks: timing.effective_rdt_ticks as u16,
                byte_ticks: self.clock.byte_ticks(),
                predecessor_bytes: ctx.slot_offset_bytes,
                fold_start_cursor: ctx.fold_start_cursor,
            });
        }
    }

    /// Drop any scheduled TX, clear chain-pending state, and clear the
    /// staged request context so the next reply must come through a fresh
    /// `poll()`.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.send_policy.cancel();
    }

    /// Stage a deferred ID change — applies at the next `on_tx_complete`.
    pub fn stage_id(&mut self, id: u8) {
        self.send_policy.stage_id(id);
    }

    /// Stage a deferred baud-rate change. Forwarded to `Clock`; applied
    /// at its `on_tx_complete` (driven by the parent's `on_tx_complete`).
    pub fn stage_baud(&mut self, baud: BaudRate) {
        self.clock.stage_baud(baud);
    }

    /// Stage a deferred Return Delay Time change in µs — applies at the
    /// next `on_tx_complete`.
    pub fn stage_rdt(&mut self, us: u32) {
        self.send_policy.stage_rdt(us);
    }

    /// Stage a deferred reboot, honored after any in-flight TX drains.
    pub fn stage_reboot(&mut self, mode: BootMode) {
        self.send_policy.stage_reboot(mode);
    }
}

impl<P: Providers, const TX_BUF_LEN: usize> DxlReply for ReplyHandle<'_, P, TX_BUF_LEN> {
    fn send_status(&mut self, status: Status<'_>) -> Result<(), WriteError> {
        ReplyHandle::send_status(self, status)
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
        ReplyHandle::send_status_read_chunked(self, id, error, chunks)
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
        ReplyHandle::send_slot_chunked(self, id, error, chunks)
    }

    fn stage_id(&mut self, id: u8) {
        ReplyHandle::stage_id(self, id)
    }

    fn stage_baud(&mut self, baud: BaudRate) {
        ReplyHandle::stage_baud(self, baud)
    }

    fn stage_rdt(&mut self, us: u32) {
        ReplyHandle::stage_rdt(self, us)
    }

    fn stage_reboot(&mut self, mode: BootMode) {
        ReplyHandle::stage_reboot(self, mode)
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::poll_src::PollSrc;
    use crate::dxl::uart::test_support::{
        FastLastState, SEED_TICK, SchedState, TEST_ID, TEST_RDT_US, TICKS_PER_US, mk_clock_trim,
        mk_fast_last, mk_scheduler, mk_usart_baud,
    };
    use crate::mocks::{
        FastLastSchedulerOp, MockClockTrim, MockFastLastScheduler, MockTxScheduler, MockUsartBaud,
        ScheduleOp, TestProviders,
    };
    use dxl_protocol::SoftwareCrcUmts;

    const TX_BUF_LEN: usize = 140;
    const SEED: u32 = SEED_TICK as u32;
    const RDT_TICKS: u32 = TEST_RDT_US * TICKS_PER_US;
    /// 3M byte time in scheduler ticks (10 · 16).
    const BYTE_TICKS_3M: u32 = 160;

    struct Harness {
        tx: CodecTx<SoftwareCrcUmts, TX_BUF_LEN>,
        scheduler: MockTxScheduler,
        sched: SchedState,
        fast_last: FastLast<MockFastLastScheduler, SoftwareCrcUmts>,
        fl: FastLastState,
        clock: Clock<MockUsartBaud, MockClockTrim>,
        send_policy: SendPolicy,
    }

    impl Harness {
        fn new() -> Self {
            let (scheduler, sched) = mk_scheduler();
            let (fl_sched, fl) = mk_fast_last();
            Self {
                tx: CodecTx::new_for_test(),
                scheduler,
                sched,
                fast_last: FastLast::new(fl_sched),
                fl,
                clock: Clock::new(BaudRate::B3000000, mk_usart_baud().0, mk_clock_trim().0),
                send_policy: SendPolicy::new(TEST_ID, TEST_RDT_US),
            }
        }

        fn stage(&mut self, ctx: ReplyContext) {
            self.send_policy.stage_reply_context_for_test(ctx);
        }

        fn handle(&mut self) -> ReplyHandle<'_, TestProviders, TX_BUF_LEN> {
            ReplyHandle {
                tx: &mut self.tx,
                scheduler: &mut self.scheduler,
                fast_last: &mut self.fast_last,
                clock: &mut self.clock,
                send_policy: &mut self.send_policy,
            }
        }

        fn sched_ops(&self) -> alloc::vec::Vec<ScheduleOp> {
            self.sched.operations()
        }
    }

    fn ctx() -> ReplyContext {
        ReplyContext {
            packet_end_tick: SEED,
            slot_offset_bytes: 0,
            fast_slot_position: None,
            fold_start_cursor: 0,
            predecessor_id: None,
            rdt_us: TEST_RDT_US,
            src: PollSrc::ByteBatch,
        }
    }

    fn empty_status() -> Status<'static> {
        Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        }
    }

    fn own_slot(data: &[u8]) -> Slot<'_> {
        Slot {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
            data,
        }
    }

    #[test]
    fn send_status_schedules_plain_at_packet_end_plus_rdt_plus_offset() {
        let mut h = Harness::new();
        h.stage(ReplyContext {
            slot_offset_bytes: 12,
            ..ctx()
        });
        h.handle().send_status(empty_status()).expect("encode fits");

        let byte_count = h.tx.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            h.sched_ops(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: SEED + RDT_TICKS + 12 * BYTE_TICKS_3M,
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_status_defers_to_predecessor_without_scheduling() {
        let mut h = Harness::new();
        h.stage(ReplyContext {
            predecessor_id: Some(0x42),
            ..ctx()
        });
        h.handle().send_status(empty_status()).expect("encode fits");

        assert!(h.sched_ops().is_empty());
        assert_eq!(h.send_policy.awaited_predecessor(), Some(0x42));
    }

    #[test]
    fn send_status_without_context_encodes_but_never_schedules() {
        let mut h = Harness::new();
        h.handle().send_status(empty_status()).expect("encode fits");

        assert!(h.tx.tx_len() > 0);
        assert!(h.sched_ops().is_empty());
    }

    #[test]
    fn context_is_consumed_so_double_send_schedules_once() {
        let mut h = Harness::new();
        h.stage(ctx());
        let mut handle = h.handle();
        handle.send_status(empty_status()).expect("encode fits");
        handle.send_status(empty_status()).expect("encode fits");

        assert_eq!(h.sched_ops().len(), 1);
    }

    #[test]
    fn send_slot_non_last_positions_schedule_plain_and_leave_the_fold_idle() {
        for position in [
            SlotPosition::Only { packet_length: 7 },
            SlotPosition::First { packet_length: 15 },
            SlotPosition::Middle,
        ] {
            let mut h = Harness::new();
            h.stage(ReplyContext {
                fast_slot_position: Some(position),
                ..ctx()
            });
            h.handle()
                .send_slot(&own_slot(&[0x11, 0x22]))
                .expect("encode fits");

            assert_eq!(
                h.sched_ops(),
                alloc::vec![ScheduleOp::Schedule {
                    deadline: SEED + RDT_TICKS,
                    byte_count: h.tx.tx_len(),
                    kind: SendKind::Plain,
                }],
                "position {position:?}"
            );
            assert!(h.fl.operations().is_empty());
            assert!(!h.fast_last.fold_active());
        }
    }

    #[test]
    fn send_slot_without_position_drops_silently() {
        let mut h = Harness::new();
        h.stage(ctx());
        h.handle()
            .send_slot(&own_slot(&[0x11]))
            .expect("encode fits");

        assert!(h.sched_ops().is_empty());
    }

    #[test]
    fn send_slot_last_tags_fast_last_and_arms_the_grid_off_the_same_rdt() {
        let mut h = Harness::new();
        h.stage(ReplyContext {
            fast_slot_position: Some(SlotPosition::Last { crc: 0 }),
            slot_offset_bytes: 12,
            fold_start_cursor: 30,
            ..ctx()
        });
        h.handle()
            .send_slot(&own_slot(&[0xAA, 0xBB]))
            .expect("encode fits");

        assert_eq!(
            h.sched_ops(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: SEED + RDT_TICKS + 12 * BYTE_TICKS_3M,
                byte_count: h.tx.tx_len(),
                kind: SendKind::FastLast,
            }]
        );
        // The grid's stop-fold deadline composes the SAME effective RDT the
        // schedule used: packet_end + rdt + predecessor·byte − GUARD·byte.
        assert_eq!(
            h.fl.operations().first(),
            Some(&FastLastSchedulerOp::SetDeadline {
                deadline: SEED + RDT_TICKS + 12 * BYTE_TICKS_3M - BYTE_TICKS_3M,
            })
        );
        assert!(h.fast_last.fold_active());
        assert!(h.fast_last.grid_active());
    }

    #[test]
    fn line_idle_source_floors_the_slot_rdt_at_one_byte_time() {
        // rdt = 1 µs → 48 ticks, under the 160-tick byte time at 3M.
        for (src, expected_rdt_ticks) in
            [(PollSrc::LineIdle, BYTE_TICKS_3M), (PollSrc::ByteBatch, 48)]
        {
            let mut h = Harness::new();
            h.stage(ReplyContext {
                fast_slot_position: Some(SlotPosition::Only { packet_length: 7 }),
                rdt_us: 1,
                src,
                ..ctx()
            });
            h.handle()
                .send_slot(&own_slot(&[0x11]))
                .expect("encode fits");

            assert_eq!(
                h.sched_ops(),
                alloc::vec![ScheduleOp::Schedule {
                    deadline: SEED + expected_rdt_ticks,
                    byte_count: h.tx.tx_len(),
                    kind: SendKind::Plain,
                }],
                "src {src:?}"
            );
        }
    }

    #[test]
    fn cancel_forwards_to_the_scheduler_and_clears_the_staged_context() {
        let mut h = Harness::new();
        h.stage(ctx());
        h.handle().cancel();

        assert_eq!(h.sched_ops(), alloc::vec![ScheduleOp::Cancel]);
        assert!(h.send_policy.staged_reply_for_test().is_none());
    }
}
