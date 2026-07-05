//! DXL-over-UART transport. Composite over the codec (bytes ↔ packets)
//! and the clock (tick math + drift integration). The chip-side ISR layer
//! only ever reaches through `DxlUart`; cross-sub-driver routing (e.g.
//! the drift spans that feed from the codec's span tracker into the clock)
//! lives in this file per driver-pattern §4 + §10.1.
//!
//! Generic over its leaf providers AND its three storage sizes — see the
//! [`DxlUart`] doc. `firmware/ch32/src/runtime/registry.rs` binds each to
//! its V006 value. Future sub-drivers (`Tx`, `ChainCatchup`) land as
//! additional fields on this composite (TX work also extends `Codec`).

pub mod clock;
pub mod codec;
mod drift_window;
pub mod fast_last;
mod poll_router;
mod poll_src;
mod reply_handle;
mod send_policy;

pub use poll_src::PollSrc;
pub use reply_handle::ReplyHandle;

#[cfg(test)]
mod test_support;

use dxl_protocol::streaming::Event;
use osc_core::BootMode;

use crate::traits::dxl::{
    FastLastScheduler, Providers, RxDma, SendKind, Telemetry, TxBus, TxScheduler, WireClock,
};
use clock::Clock;
use codec::{Codec, PollAction, PollEvent};
use drift_window::RxWakeGate;
use fast_last::{FastLast, FoldExit};
use poll_router::PollRouter;
use send_policy::SendPolicy;

/// Bits on the wire for a single UART character: 1 start + 8 data + 1 stop
/// (8N1). Multiply by `ticks_per_bit` to get one byte's wire duration in
/// scheduler ticks. Also the IDLE detection threshold — CH32V00X RM §UART:
/// "an idle frame is 10-or-11-bit high, including the stop bit"; M=0 → 10.
pub(crate) const BITS_PER_FRAME: u16 = 10;

/// The DXL bus composite. `P` bundles the chip-side leaf interfaces this
/// driver pulls — see [`Providers`]. The const generics are storage sizes:
///
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per
///   `dxl-streaming-rx.md` §4.5).
/// - `TX_BUF_LEN`: DMA1_CH4 source-buffer depth sized to
///   `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES` (140 with the
///   default control-RW). Held by `Codec` so encoder methods write into
///   driver-owned storage instead of a chip-side static.
pub struct DxlUart<P: Providers, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize> {
    codec: Codec<P::Crc, RX_BUF_LEN, TX_BUF_LEN>,
    clock: Clock<P::UsartBaud, P::ClockTrim>,
    /// NDTR-only readback for DMA1_CH5 (the RX byte ring). Plumbed
    /// independently of the parser-path `on_rx_progress` calls so the
    /// Fast Last fold body's refresh doesn't go through the chip-side
    /// ISR — see [`Self::on_fold_step`].
    rx_dma: P::RxDma,
    scheduler: P::TxScheduler,
    /// Chip-side bus-control provider. Used by [`Self::on_tx_complete`]
    /// to release the wire driver (the scheduled take-over is pure
    /// hardware — the TX kickoff enables the TX DMA at the compare
    /// match), and by [`Self::poll`]'s SkipComplete arm for the Plain
    /// chain k > 0 sequence-driven start path
    /// (`docs/dxl-streaming-rx.md` §5.2).
    tx_bus: P::TxBus,
    /// Fast Last fold pipeline for Fast Sync / Bulk Read Last replies — a
    /// §4.3 sub-composite of the periodic-walk grid scheduler and the
    /// chain-CRC fold engine. Started at `send_slot(Last)` via `ReplyHandle`;
    /// the grid drives one body per CMP, the fold engine's per-byte hook is
    /// wired into the codec's `drain_raw` callback and finalize patches our
    /// own trailing CRC slot before DMA1_CH4 reads it.
    fast_last: FastLast<P::FastLastScheduler>,
    /// WireClock u32 readout used by `on_rx_advance`, `on_rx_idle`, and
    /// `poll` to source `now` without taking it as a parameter. The
    /// chip-side provider hides any peripheral-side detail (on V006 `now`
    /// is a direct SysTick u32 readout) — see [`WireClock`].
    wire_clock: P::WireClock,
    /// Wire-condition miss counters. The composite records at the point
    /// of detection ([`Self::poll`]'s Crc arm for anchor misses,
    /// [`Self::on_fold_step`]'s window-expired exit for patch misses);
    /// the provider owns where the counts live chip-side.
    telemetry: P::Telemetry,

    /// Send-side policy sub-composite — bus identity + staged-config
    /// mailbox (committed at [`Self::on_tx_complete`]), the parse-side
    /// Instruction tracker, and the reply gate holding the staged
    /// [`ReplyContext`] / chain-pending predecessor wait. Per
    /// driver-pattern §7.4 — driver-derivable wire shape stays in the
    /// driver, not on the trait surface.
    send_policy: SendPolicy,

    /// Per-byte RX wake reason set + drift-window lifecycle. Reconciles the
    /// FAST k > 0 status-start wait and the drift sampler, which share the
    /// one RXNEIE bit — toggled only on the OR-edge so neither closes the
    /// window the other still holds. See [`drift_window`].
    rx_wake: RxWakeGate,
}

impl<P: Providers, const RX_BUF_LEN: usize, const TX_BUF_LEN: usize>
    DxlUart<P, RX_BUF_LEN, TX_BUF_LEN>
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        codec: Codec<P::Crc, RX_BUF_LEN, TX_BUF_LEN>,
        clock: Clock<P::UsartBaud, P::ClockTrim>,
        rx_dma: P::RxDma,
        scheduler: P::TxScheduler,
        tx_bus: P::TxBus,
        fast_last: FastLast<P::FastLastScheduler>,
        wire_clock: P::WireClock,
        telemetry: P::Telemetry,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        let mut s = Self {
            codec,
            clock,
            rx_dma,
            scheduler,
            tx_bus,
            fast_last,
            wire_clock,
            telemetry,
            send_policy: SendPolicy::new(id, rdt_us),
            rx_wake: RxWakeGate::new(),
        };
        // Cold-start drift window: hold the per-byte RX wake open from boot
        // so the first short-packet exchange lands the factory-drift
        // correction (isolated pings form no drain-ISR span pairs).
        s.rx_wake.open_drift(&mut s.rx_dma, 0);
        s
    }

    // -- events -------------------------------------------------------------

    /// USART1 IDLE ISR entry — refresh the ET producer head and stash
    /// `(now, LineIdle)` for the Crc-time fallback path. Small-packet
    /// backstop for the parser drain (chip ISR follows with
    /// `services.poll` to drain any packet whose last byte arrived
    /// before RX HT/TC tripped).
    pub fn on_rx_idle(&mut self) {
        let now = self.wire_clock.now();
        let byte_ticks = self.clock.byte_ticks() as u32;
        // Publish the byte ring before stamping so the span cursor is
        // current — the IDLE path doesn't otherwise refresh it.
        self.codec.on_rx_progress(self.rx_dma.remaining());
        if let Some((d_ticks, d_bytes)) = self.codec.on_idle(now, byte_ticks) {
            if self.clock.on_span(d_ticks, d_bytes) {
                self.rx_wake.on_batch_closed(&mut self.rx_dma);
            }
            self.rx_wake
                .note_accept(self.codec.instruction_count(), false);
        }
    }

    /// DMA1_CH5 HT/TC ISR entry — clear flags, refresh the codec's view
    /// of `write_seq` from NDTR. The chip ISR follows with
    /// `services.poll` to drive the parser drain over the freshly-
    /// published bytes (see `dxl-hw-timed-transport.md` §9,
    /// `dxl-streaming-rx.md` §3 / §4.4 / §5.2). Bounds `write_seq` lag
    /// to `RX_BUF_LEN/2` so the codec's universal byte-skip can drain
    /// payload-length packets without `on_publish`'s mod-`RX_BUF_LEN`
    /// delta rounding full ring periods to zero.
    pub fn on_rx_advance(&mut self) {
        let _ = self.rx_dma.read_and_ack();
        let now = self.wire_clock.now();
        let byte_ticks = self.clock.byte_ticks() as u32;
        self.codec.on_rx_progress(self.rx_dma.remaining());
        if let Some((d_ticks, d_bytes)) = self.codec.on_byte_batch_wake(now, byte_ticks) {
            if self.clock.on_span(d_ticks, d_bytes) {
                self.rx_wake.on_batch_closed(&mut self.rx_dma);
            }
            self.rx_wake
                .note_accept(self.codec.instruction_count(), false);
        }
    }

    /// A per-byte RX wake fired (chip-side: USART1 RXNE trap). The wake is
    /// the demux point for the two consumers that hold the RXNEIE window
    /// open — served in order:
    ///
    /// 1. **Drift window** (cheap): while open, record this byte's stamp so
    ///    the burst's accumulated span can land at the packet boundary. This
    ///    is the isolated-short-packet drift path (a short packet trips one
    ///    drain ISR and forms no `SpanTracker` pair).
    /// 2. **FAST k > 0 status-start wait**: resolve the observed start tick
    ///    of the chain's single Status packet from the byte-count estimate,
    ///    schedule our
    ///    slot's wire start `slot_offset_bytes` whole bytes after it, and for
    ///    Last start the fold pipeline off the same anchor. Also called from
    ///    the `poll` epilogue as the enable-race backstop — when the reply's
    ///    leading bytes arrived before the watch opened, no further wake may
    ///    come for them.
    ///
    /// FAST guard order:
    /// 1. no wait parked → clear the FAST reason (the drift window may still
    ///    hold RXNEIE) and return;
    /// 2. no resolvable tick → keep waiting (spurious wake before any
    ///    reply byte, or an EMI-lost edge — the next byte's wake retries
    ///    via the multi-byte signature arm);
    /// 3. stamp past `latest_start_tick` → stale traffic (the host's
    ///    retry after a dead chain), never the awaited reply: drop the
    ///    slot instead of scheduling a TX into the host's instruction.
    // RAM placement is load-bearing — the resolve → schedule → inline-fold
    // path runs while the predecessor streams at wire rate; every tick of
    // flash i-fetch here is a byte of fold backlog the spin must claw back
    // before the TX DMA reads the trailing CRC slot (see
    // `FsmScheduler::on_step`).
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    #[inline(never)]
    pub fn on_rx_byte_wake(&mut self) {
        if self.rx_wake.drift_open() {
            let now = self.wire_clock.now();
            let remaining = self.rx_dma.remaining();
            let (rx, _) = self.codec.split_mut();
            rx.record_drift_wake(now, remaining);
        }
        let Some(wait) = self.send_policy.awaited_status_start() else {
            self.rx_wake.set_fast(&mut self.rx_dma, false);
            return;
        };
        let now = self.wire_clock.now();
        let byte_ticks = self.clock.byte_ticks() as u32;
        let (rx, _) = self.codec.split_mut();
        let Some(status_start) =
            rx.status_start_tick(&self.rx_dma, wait.status_start_cursor, now, byte_ticks)
        else {
            return;
        };
        if (status_start.wrapping_sub(wait.latest_start_tick) as i32) > 0 {
            crate::log::debug!(
                "dxl: status_start stale start={} latest={} — drop slot",
                status_start,
                wait.latest_start_tick
            );
            self.rx_wake.set_fast(&mut self.rx_dma, false);
            self.send_policy.on_status_start_scheduled();
            return;
        }
        let deadline = self
            .clock
            .compute_slot_start_deadline(status_start, wait.slot_offset_bytes);
        let byte_count = self.codec.tx_len();
        crate::log::debug!(
            "dxl: status_start observed start={} deadline={}",
            status_start,
            deadline
        );
        // Every parked wait is a successor slot (k > 0) under the official
        // per-block-CRC layout — the chain-CRC pipeline runs for all of
        // them, not just the chain's final slot.
        self.fast_last.start(fast_last::FastLastSchedule {
            status_start_tick: status_start,
            byte_ticks: self.clock.byte_ticks(),
            predecessor_bytes: wait.slot_offset_bytes,
            fold_start_cursor: wait.status_start_cursor,
            drain_stride_bytes: (RX_BUF_LEN / 2) as u32,
        });
        // Late observation (short predecessor at high baud): the
        // deadline is already past, so the schedule below fires the
        // hardware kickoff ASAP — and with a short own reply the TX
        // DMA reads the trailing CRC slot within a few byte-times,
        // sooner than any grid CMP body can land the patch (measured:
        // patch ~800 ticks behind the read at 3M). Every predecessor
        // byte is already on the wire (deadline past ⇒ the
        // predecessor window is over), so fold + patch inline FIRST;
        // the wire start slips by the fold's ~20 µs, which a late
        // fire-ASAP send absorbs — a valid chain CRC a little later
        // beats a placeholder on time.
        // Freshly-read clock, NOT the entry-time `now`: the stamp
        // query above can burn enough ticks that a deadline the entry
        // read saw as future is already past — and the entry read
        // deciding "not late" is exactly the case that ships a
        // placeholder CRC (measured on hardware at 3M).
        if (deadline.wrapping_sub(self.wire_clock.now()) as i32) <= 0 {
            self.on_fold_step();
        }
        self.scheduler
            .schedule(deadline, byte_count, SendKind::FastLast);
        self.rx_wake.set_fast(&mut self.rx_dma, false);
        self.send_policy.on_status_start_scheduled();
        // On-time observation with a short predecessor window: run the
        // first fold body inline instead of waiting for the CMP dispatch.
        // The fold barely out-paces the wire per byte, so the ~ENTRY-scale
        // dispatch latency is a backlog the body can never clear before
        // the window ends — the chain-CRC patch then loses to the TX DMA's
        // read of a short reply's trailing CRC slot (ledger-measured at
        // 3M). Inline entry clears the backlog mid-window. Bounded by the
        // horizon: the body busy-folds at most until the window closes,
        // and the late path above already finalized (grid inactive) when
        // the deadline was past. Runs after the watch/lifecycle bookkeeping
        // so a wake landing mid-fold sees a consistent exchange state.
        if self.fast_last.grid_active()
            && (deadline.wrapping_sub(self.wire_clock.now()) as i32)
                <= <P::FastLastScheduler as FastLastScheduler>::INLINE_FOLD_HORIZON_TICKS as i32
        {
            self.on_fold_step();
        }
    }

    /// One Fast Last periodic-walk fold body is due (chip-side SysTick
    /// CMP). Body drives [`FsmScheduler::on_step`] forward — the walker
    /// closure drains pending RX bytes raw through [`FoldEngine::on_slice`]
    /// and returns the cumulative folded count. The wire start is
    /// hardware-armed and fires in parallel; a body short of the fold
    /// target just re-arms the next CMP, and the completion body's
    /// [`FoldExit::WindowExpired`] exit (patch lost the race against the
    /// TX DMA's read of the trailing CRC slot — starved fold, or a body
    /// landing pathologically late) routes to `crc_patch_deadline_miss`.
    ///
    /// [`FsmScheduler::on_step`]: fast_last::FsmScheduler::on_step
    /// [`FoldEngine::on_slice`]: fast_last::FoldEngine::on_slice
    // RAM placement is load-bearing — see `Self::on_rx_byte_wake`.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    #[inline(never)]
    pub fn on_fold_step(&mut self) {
        let Self {
            codec,
            rx_dma,
            fast_last,
            scheduler,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        let (fl_sched, fl_crc) = fast_last.split_mut();
        let exit = fl_sched.on_step(
            || Self::checkpoint_pickup_body(rx, rx_dma, fl_crc, tx),
            || scheduler.commit_pending(),
        );
        if matches!(exit, FoldExit::WindowExpired) {
            self.telemetry.record_crc_patch_deadline_miss();
        }
    }

    /// One checkpoint-pickup attempt — the wake body's spin iteration.
    /// Returns true once the trailing CRC slot is patched (or nothing is
    /// armed). A named function rather than closure logic so the RAM
    /// placement is verifiable: a closure is its own (unsectioned) symbol,
    /// and when `poll_checkpoint`/`finalize_from_checkpoint` inline into
    /// it the whole per-iteration body lands back in flash — measured as
    /// exactly the i-fetch cost that loses the patch-vs-DMA race at 3M.
    // RAM placement is load-bearing — see `FsmScheduler::on_step`.
    #[cfg_attr(target_arch = "riscv32", unsafe(link_section = ".highcode"))]
    #[inline(never)]
    fn checkpoint_pickup_body(
        rx: &mut codec::CodecRx<P::Crc, RX_BUF_LEN>,
        rx_dma: &mut P::RxDma,
        fl_crc: &mut fast_last::FoldEngine,
        tx: &mut codec::CodecTx<P::Crc, TX_BUF_LEN>,
    ) -> bool {
        if !fl_crc.is_active() {
            // Already finalized (or never armed) — nothing left to do.
            return true;
        }
        match rx.poll_checkpoint(rx_dma, fl_crc.window_end_cursor()) {
            Some(checkpoint) => {
                fl_crc.finalize_from_checkpoint(checkpoint, tx);
                true
            }
            None => false,
        }
    }

    /// Long-horizon timer match triggered (chip-side: SysTick CMP). Two consumers
    /// share the CMP — the TX-scheduler handoff arm (multi-wrap distances
    /// where the direct hardware arm can't span the wait) and the Fast
    /// Last walk grid. The TX-scheduler reports whether the match was its
    /// own; if not, it's a Fast Last grid step. When the scheduler owned
    /// the match while the grid is active, the handoff arm clobbered the
    /// grid's pending CMP (both share the one CMP register) — run a fold
    /// body anyway so the grid re-arms its own.
    pub fn on_schedule_due(&mut self) {
        if self.scheduler.on_schedule_due() && !self.fast_last.grid_active() {
            return;
        }
        self.on_fold_step();
    }

    /// USART1 TC triggered — the reply has fully drained the wire. Release
    /// the wire driver *first* (drop TX_EN, mask TC IRQ, disable DMA) so
    /// stale TX_EN doesn't sit on the bus while pending config mutates;
    /// then drain staged writes (id / baud / trim / rdt) and surface any
    /// pending reboot to the chip-side ISR. Also resets the Fast Last
    /// state (idempotent — both halves already return to idle naturally
    /// on the successful path). Reboot is returned (rather than
    /// self-applied) because the chip controls how the reset actually
    /// happens; the driver only knows it was asked.
    pub fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.tx_bus.release_bus();
        // A pickup window canceled before its checkpoint ran still owns
        // its published bytes — dispose of them so the parser resumes at
        // the wire frontier, never inside the stale chain frame.
        if self.fast_last.fold_active() {
            let (rx, _) = self.codec.split_mut();
            rx.release_window(&self.rx_dma, self.fast_last.window_end_cursor());
        }
        self.fast_last.cancel();
        // Belt-and-suspenders sibling of the send-policy wait clear
        // below: a status-start wake window left open past our own TX
        // (the wake path closes it on every resolution, but a dropped
        // exchange may not have resolved) must not leak the FAST reason
        // into the next exchange. Clears the FAST reason only — the drift
        // window persists across our own reply.
        self.rx_wake.set_fast(&mut self.rx_dma, false);
        // Per `dxl-streaming-rx.md` §5.3: our own TX completion is a
        // packet boundary at which stale chain state must reset. The
        // §5.3 "next instruction header" trigger is too late for the
        // universal byte-skip — at slow baud the deadline-bounded skip
        // can outlive the inter-packet gap and eat the next preamble
        // before the parser ever reaches the header event.
        self.codec.cancel_skip();
        let reboot = self.send_policy.on_tx_complete();
        let baud_changed = self.clock.on_tx_complete();
        // A new divisor invalidates the in-flight drift batch — reopen the
        // window so the first packet at the new baud re-lands the trim.
        if baud_changed {
            self.rx_wake
                .open_drift(&mut self.rx_dma, self.codec.instruction_count());
        }
        reboot
    }

    // -- commands -------------------------------------------------------------

    /// Drive the codec event stream, manage wire-level state (anchor,
    /// drift gating, universal byte-skip, slot-walk, reply context) and
    /// forward every parser [`Event`] to the dispatcher closure alongside
    /// the ring slice and a [`ReplyHandle`]. The closure shape matches
    /// `osc_core::DxlDispatcher::on_event` so the chip-side
    /// `Ch32Bus::poll` forwarder is a one-liner.
    ///
    /// Per [doc §3 / §4.3]:
    /// - Instruction Header → mark packet as Instruction (drift sampling).
    /// - Instruction Header (foreign) → universal byte-skip on the body.
    /// - Status Header → universal byte-skip on the body.
    /// - SyncSlot / BulkSlot → slot-walk against the servo ID.
    /// - Crc → stamp packet-end tick, derive [`ReplyContext`].
    /// - Resync → drop inflight.
    ///
    /// Closure-based shape exists to break the borrow conflict between the
    /// parser-event borrow on the codec RX half and the `&mut` access the
    /// send path needs on the TX half. [`Codec::split_mut`] returns the two
    /// halves disjointly so the closure sees both at once.
    pub fn poll<F>(&mut self, mut f: F)
    where
        F: FnMut(Event, &[u8], &mut ReplyHandle<'_, P, TX_BUF_LEN>),
    {
        // Fold window is owned by `drain_raw` via `on_fold_step` /
        // `on_tx_start`; the parser path must not touch the ring or its
        // cursor while a Fast Last fold is in flight. IDLE-triggered and RX
        // byte-ring HT/TC-triggered polls need this entry gate to honor the
        // RX-tail ownership contract in `dxl-streaming-rx.md` §6.
        if self.fast_last.fold_active() {
            return;
        }
        // Same ownership rule one stage earlier: a deferred successor
        // slot's fold hasn't started yet (it starts at the status-start
        // observation), but the Status packet's bytes already belong to
        // it — a parser drain here would consume the leading header
        // bytes before the fold ever sees them. The per-byte wake path
        // (`on_rx_byte_wake`) is not a poll, so observation still
        // proceeds; a stale wait is bounded by the wake path's staleness
        // window, which drops it and un-gates on the next wire traffic.
        if self.send_policy.awaited_status_start().is_some() {
            return;
        }
        self.codec.on_rx_progress(self.rx_dma.remaining());
        let now = self.wire_clock.now();
        let packet_end_comp = <P::WireClock as WireClock>::PACKET_END_ENTRY_COMP_TICKS;
        let id = self.send_policy.id();
        let ticks_per_bit = self.clock.ticks_per_bit();
        let Self {
            codec,
            clock,
            rx_dma,
            scheduler,
            tx_bus,
            fast_last,
            send_policy,
            rx_wake,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        let mut router = PollRouter::<P, TX_BUF_LEN> {
            tx,
            scheduler,
            tx_bus,
            fast_last,
            clock,
            send_policy,
            rx_dma,
            rx_wake,
            id,
        };
        rx.poll(now, ticks_per_bit, packet_end_comp, |pe| match pe {
            PollEvent::Parser {
                ev,
                ring,
                next_status_pos,
                packet_end,
            } => {
                let action = router.on_parser_event(ev, next_status_pos, packet_end);
                f(ev, ring, &mut router.reply());
                // If `f()` engaged the Fast Last fold (via
                // `send_slot(Last)`), the in-flight poll must exit before
                // the parser eats any more bytes — those bytes belong to
                // the fold engine. The poll entry gate above stops FUTURE
                // polls only; without this bail-out the parser+skip path
                // consumes the predecessor's leading bytes in the same poll
                // that engaged the fold, and the fold engine starves. A
                // deferred successor wait
                // (`send_slot` k > 0, fold not yet started) owns
                // those bytes the same way — mirror of the poll entry
                // gate above, for the poll already in flight.
                if router.fast_last.fold_active()
                    || router.send_policy.awaited_status_start().is_some()
                {
                    PollAction::Stop
                } else {
                    action
                }
            }
            PollEvent::SkipComplete { id: pred } => {
                router.on_skip_complete(pred);
                PollAction::Continue
            }
        });
        // Feed the drift window's accumulated span (isolated short packet)
        // to the integrator BEFORE the packet-end close, so a boot batch
        // holds it and lands the correction on this exchange. The span can
        // itself close a steady batch — a batch close by any route ends
        // the window below.
        let mut batch_closed = false;
        let window_accepted = if let Some((d_ticks, d_bytes)) = rx.take_window_span() {
            batch_closed |= router.clock.on_span(d_ticks, d_bytes);
            true
        } else {
            false
        };
        // Apply any pending trim correction at the RX-side packet
        // boundary. Idempotent when nothing changed; necessary so
        // foreign-instruction packets — which never reach
        // `on_tx_complete` — still commit their drift samples per
        // [[drift_sampling_instruction_only]]. Returns whether the boot
        // batch closed — landed or below-deadband alike, the drift
        // window's job is done.
        batch_closed |= router.clock.on_rx_packet_end();
        // Reconcile the drift window lifecycle (close on batch close /
        // give-up, reopen on staleness). Runs after the router borrow is
        // spent.
        let instr = self.codec.instruction_count();
        self.rx_wake
            .service(&mut self.rx_dma, batch_closed, window_accepted, instr);
        // Enable-race backstop: `send_slot` (k > 0) may have deferred
        // within THIS poll while the Status packet's leading bytes were
        // already in the ring (low RDT + IDLE-late poll) — no further
        // per-byte wake comes for bytes that already arrived, so resolve
        // the observation here.
        if self.send_policy.awaited_status_start().is_some() {
            self.on_rx_byte_wake();
        }
    }

    // -- accessors ------------------------------------------------------------

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
    }

    /// Stable peripheral-memory address for DMA1_CH5's destination buffer.
    pub fn rx_buf_addr(&self) -> usize {
        self.codec.rx_buf_addr()
    }

    /// Stable peripheral-memory address for DMA1_CH4's source buffer.
    pub fn tx_buf_addr(&self) -> usize {
        self.codec.tx_buf_addr()
    }

    /// Length in bytes of the most-recent encoded packet — the DMA1_CH4
    /// transfer count for the next wire start.
    pub fn tx_len(&self) -> u16 {
        self.codec.tx_len()
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::test_support::{
        FastLastState, RxDmaState, SEED_TICK, SchedState, TEST_ID, TEST_RDT_US, TICKS_PER_BIT_3M,
        TICKS_PER_US, TelemetryState, TxBusState, WireClockState, mk_clock_trim, mk_fast_last,
        mk_rx_dma, mk_scheduler, mk_telemetry, mk_tx_bus, mk_usart_baud, mk_wire_clock, wire_ping,
        wire_status,
    };
    use super::*;
    use crate::mocks::{
        FastLastSchedulerOp, MockClockTrim, MockUsartBaud, ScheduleOp, TestProviders, TxBusOp,
    };
    use crate::traits::dxl::SendKind;
    use dxl_protocol::streaming::{HeaderEvent, InstructionHeader};
    use dxl_protocol::types::StatusError;
    use dxl_protocol::wire::{CRC_BYTES, FAST_RESPONSE_SLOT0_BYTES};
    use dxl_protocol::{Id, InstructionEncoder, Slot, SoftwareCrcUmts, Status};
    use heapless::Vec;
    use osc_core::BaudRate;

    /// Test-side storage sizing — matches V006 defaults per
    /// `dxl-streaming-rx.md` §4.5 so any drift between driver tests and
    /// chip-side reality stays visible.
    const RX_BUF_LEN: usize = 64;
    const TX_BUF_LEN: usize = 140;

    type TestCodec = Codec<SoftwareCrcUmts, RX_BUF_LEN, TX_BUF_LEN>;
    type TestBus = DxlUart<TestProviders, RX_BUF_LEN, TX_BUF_LEN>;

    struct TestState {
        sch: SchedState,
        tx_bus: TxBusState,
        rx: RxDmaState,
        fl: FastLastState,
        telemetry: TelemetryState,
        wire: WireClockState,
    }

    fn make_clock(baud: BaudRate) -> Clock<MockUsartBaud, MockClockTrim> {
        Clock::new(baud, mk_usart_baud().0, mk_clock_trim().0)
    }

    fn make_bus_with(codec: TestCodec, baud: BaudRate) -> (TestBus, TestState) {
        let (rx_dma, rx_state) = mk_rx_dma();
        let (scheduler, sch_state) = mk_scheduler();
        let (tx_bus, tx_bus_state) = mk_tx_bus();
        let (fl_sched, fl_state) = mk_fast_last();
        let (telemetry, telemetry_state) = mk_telemetry();
        let (wire_clock, wire_state) = mk_wire_clock();
        let bus = DxlUart::new(
            codec,
            make_clock(baud),
            rx_dma,
            scheduler,
            tx_bus,
            FastLast::new(fl_sched),
            wire_clock,
            telemetry,
            TEST_ID,
            TEST_RDT_US,
        );
        (
            bus,
            TestState {
                sch: sch_state,
                tx_bus: tx_bus_state,
                rx: rx_state,
                fl: fl_state,
                telemetry: telemetry_state,
                wire: wire_state,
            },
        )
    }

    fn make_bus() -> (TestBus, TestState) {
        make_bus_with(Codec::new(), BaudRate::B3000000)
    }

    /// Stash a ByteBatch drain-ISR wake at `SEED_TICK` so the codec's
    /// packet-end estimate reads `SEED_TICK` at the next Crc (ByteBatch
    /// fallback = `now`, entry-comp 0).
    fn seed_packet_end_wake(bus: &mut TestBus) {
        let byte_ticks = bus.clock.byte_ticks() as u32;
        let _ = bus.codec.on_byte_batch_wake(SEED_TICK as u32, byte_ticks);
    }

    /// Stage `bytes` into the codec's RX byte ring at sequence `at` AND
    /// publish the matching DMA1_CH5 NDTR readback through `MockRxDma` so
    /// the next `bus.poll()` entry sees `on_publish(remaining)` as a no-op
    /// delta.
    fn stage_rx(bus: &mut TestBus, state: &TestState, at: u16, bytes: &[u8]) {
        bus.codec.stage_rx_bytes_for_test(at, bytes);
        let n = RX_BUF_LEN as u16;
        let head_pos = at.wrapping_add(bytes.len() as u16) % n;
        state.rx.stage_remaining((n - head_pos) % n);
    }

    #[derive(Debug, PartialEq, Eq)]
    enum Tag {
        Sync,
        InstrPing(u8),
        InstrSyncRead,
        InstrFastSyncRead,
        StatusHeader(u8),
        Crc,
        Resync,
        Other,
    }

    fn ev_tag(ev: Event) -> Tag {
        match ev {
            Event::Sync => Tag::Sync,
            Event::Header(HeaderEvent::Instruction(InstructionHeader::Ping { id })) => {
                Tag::InstrPing(id.as_byte())
            }
            Event::Header(HeaderEvent::Instruction(InstructionHeader::SyncRead { .. })) => {
                Tag::InstrSyncRead
            }
            Event::Header(HeaderEvent::Instruction(InstructionHeader::FastSyncRead { .. })) => {
                Tag::InstrFastSyncRead
            }
            Event::Header(HeaderEvent::Status(sh)) => Tag::StatusHeader(sh.id.as_byte()),
            Event::Crc(_) => Tag::Crc,
            Event::Resync(_) => Tag::Resync,
            _ => Tag::Other,
        }
    }

    fn poll_capture(bus: &mut TestBus, state: &TestState, bytes: &[u8]) -> alloc::vec::Vec<Tag> {
        stage_rx(bus, state, 0, bytes);
        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| {
            tags.push(ev_tag(ev));
        });
        tags
    }

    fn saw_crc(tags: &[Tag]) -> bool {
        tags.iter().any(|t| matches!(t, Tag::Crc))
    }

    fn wire_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .sync_read(addr, length, ids)
            .unwrap();
        out
    }

    fn wire_fast_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 64> {
        let mut out: Vec<u8, 64> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .fast_sync_read(addr, length, ids)
            .unwrap();
        out
    }

    /// Construct a bus pre-loaded with `pkt` bytes and a stashed ByteBatch
    /// wake at [`SEED_TICK`] so the codec's packet-end estimate reads
    /// `SEED_TICK` (ByteBatch fallback = `now`, entry-comp 0) at Crc-good.
    /// Returns the bus, state, and expected packet-end tick.
    fn bus_seeded_with(pkt: &[u8]) -> (TestBus, TestState, u32) {
        let (mut bus, state) = make_bus();
        stage_rx(&mut bus, &state, 0, pkt);
        seed_packet_end_wake(&mut bus);
        let packet_end_tick = SEED_TICK as u32;
        (bus, state, packet_end_tick)
    }

    fn empty_status() -> Status<'static> {
        Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        }
    }

    /// Software estimate of the staged Status packet's first-byte start
    /// tick: the pinned wire-clock `now` ([`SEED_TICK`]) back-dated by the
    /// single `0xFF` byte [`observe_status_start`] stages past the status
    /// cursor (one whole frame).
    const STATUS_START_TICK: u32 = SEED_TICK as u32 - (BITS_PER_FRAME * TICKS_PER_BIT_3M) as u32;

    /// Stage the chain Status packet's first byte (`0xFF`) at wire cursor
    /// `start`, then drive the per-byte wake — the deferred FAST slot
    /// k > 0 resolves: the observed start estimates as `now − 1·byte_ticks`,
    /// its wire start schedules off that anchor and (for Last) the fold
    /// pipeline starts.
    fn observe_status_start(bus: &mut TestBus, state: &TestState, start: u16) {
        stage_rx(bus, state, start, &[0xFF]);
        bus.on_rx_byte_wake();
    }

    // ------------------------------------------------------------------
    // Deferred config staging
    // ------------------------------------------------------------------

    #[test]
    fn on_rx_idle_does_not_panic_with_empty_buffer() {
        let (mut bus, _) = make_bus();
        bus.on_rx_idle();
    }

    #[test]
    fn stage_baud_forwards_to_clock() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_baud(BaudRate::B1000000));
        assert_eq!(bus.clock.ticks_per_bit(), 16);
        bus.on_tx_complete();
        assert_eq!(bus.clock.ticks_per_bit(), 48);
    }

    // ------------------------------------------------------------------
    // Poll dispatch
    // ------------------------------------------------------------------

    #[test]
    fn poll_streams_instruction_addressed_to_us_through_crc() {
        let (mut bus, state) = make_bus();
        let pkt = wire_ping(TEST_ID);
        let tags = poll_capture(&mut bus, &state, &pkt);
        assert!(tags.contains(&Tag::InstrPing(TEST_ID)));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_byte_skips_foreign_instruction_body() {
        let (mut bus, state) = make_bus();
        let pkt = wire_ping(0x42);
        let tags = poll_capture(&mut bus, &state, &pkt);
        assert!(tags.contains(&Tag::InstrPing(0x42)));
        assert!(!saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_surfaces_broadcast_instruction() {
        let (mut bus, state) = make_bus();
        let pkt = wire_sync_read(0x84, 4, &[0x01, 0x02, 0x03]);
        let tags = poll_capture(&mut bus, &state, &pkt);
        assert!(tags.contains(&Tag::InstrSyncRead));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_byte_skips_status_frames() {
        let (mut bus, state) = make_bus();
        let pkt = wire_status(TEST_ID);
        let tags = poll_capture(&mut bus, &state, &pkt);
        assert!(tags.contains(&Tag::StatusHeader(TEST_ID)));
        assert!(!saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 0);
    }

    // ------------------------------------------------------------------
    // TX scheduling
    // ------------------------------------------------------------------

    #[test]
    fn send_status_after_poll_schedules_at_packet_end_plus_rdt() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, state, packet_end_tick) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            state.sch.operations(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: packet_end_tick.wrapping_add(TEST_RDT_US.wrapping_mul(TICKS_PER_US)),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_slot_last_defers_until_status_start_observation() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);

        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        // Deferred: nothing scheduled, fold idle, wake window open.
        assert!(state.sch.operations().is_empty());
        assert!(state.fl.operations().is_empty());
        assert!(!bus.fast_last.fold_active());
        assert!(state.rx.status_start_watched());

        observe_status_start(&mut bus, &state, req.len() as u16);

        // bytes_before for slot 1 = the First emission's wire bytes; the
        // deadline anchors on the OBSERVED status start, no RDT term.
        let first_bytes = (FAST_RESPONSE_SLOT0_BYTES + CRC_BYTES) as u32 + payload.len() as u32;
        let expected = STATUS_START_TICK.wrapping_add(bus.clock.bytes_to_ticks(first_bytes));
        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        // The inline fold body (short window → runs within the
        // observation) is the grid's final body, so it also commits the
        // far-horizon TX stash — a no-op here (the schedule direct-armed).
        assert_eq!(
            state.sch.operations(),
            alloc::vec![
                ScheduleOp::Schedule {
                    deadline: expected,
                    byte_count,
                    kind: SendKind::FastLast,
                },
                ScheduleOp::CommitPending,
            ]
        );
        // SetBusyWaitDeadline + first walk CMP from the grid start, then
        // the completion CMP re-armed by the inline body (only the leading
        // byte staged → short of target, spin degenerated by the mock).
        let fl_ops = state.fl.operations();
        assert!(matches!(
            fl_ops.as_slice(),
            [
                FastLastSchedulerOp::SetBusyWaitDeadline { .. },
                FastLastSchedulerOp::Schedule { .. },
                FastLastSchedulerOp::Schedule { .. },
            ]
        ));
        assert!(bus.fast_last.fold_active());
        assert!(bus.fast_last.grid_active());
        assert!(
            !bus.rx_wake.fast_for_test(),
            "FAST reason clears on resolve"
        );
        assert!(bus.send_policy.awaited_status_start().is_none());
    }

    #[test]
    fn send_slot_mid_chain_successor_defers_and_joins_the_fold_pipeline() {
        // Slot 1 of a 3-servo chain: under the official per-block-CRC
        // layout a mid-chain successor defers exactly like the final one
        // and starts the chain-CRC pipeline at observation — its own
        // block ends with a CRC that needs the fire-time patch.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID, 0x51]);
        let (mut bus, state, _) = bus_seeded_with(&req);

        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(state.sch.operations().is_empty());
        assert!(state.rx.status_start_watched());

        observe_status_start(&mut bus, &state, req.len() as u16);

        let first_bytes = (FAST_RESPONSE_SLOT0_BYTES + CRC_BYTES) as u32 + payload.len() as u32;
        let expected = STATUS_START_TICK.wrapping_add(bus.clock.bytes_to_ticks(first_bytes));
        assert!(matches!(
            state.sch.operations().as_slice(),
            [
                ScheduleOp::Schedule {
                    kind: SendKind::FastLast,
                    ..
                },
                ScheduleOp::CommitPending,
            ]
        ));
        assert_eq!(
            match state.sch.operations()[0] {
                ScheduleOp::Schedule { deadline, .. } => deadline,
                _ => unreachable!("matched above"),
            },
            expected
        );
        assert!(bus.fast_last.fold_active());
        assert!(!bus.rx_wake.fast_for_test());
    }

    #[test]
    fn spurious_wake_before_any_reply_byte_keeps_waiting() {
        // V006 residual-drain quirk: a wake with no byte past the status
        // cursor must not resolve, drop, or close the window.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        bus.on_rx_byte_wake();

        assert!(state.sch.operations().is_empty());
        assert!(bus.send_policy.awaited_status_start().is_some());
        assert!(state.rx.status_start_watched());
    }

    #[test]
    fn stale_status_start_drops_the_slot_instead_of_scheduling() {
        // Host-retry regression: the chain died, the host resent the
        // instruction, and its bytes advance the cursor. The observed
        // stamp lands past `latest_start_tick` — the wake must drop the
        // slot (close window, clear wait, no TX into the host's packet).
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        // Advance well past packet_end + rdt + slack, then present a
        // single fresh reply byte — the estimate back-dates `now` by one
        // frame, landing the observed start past `latest_start_tick`.
        let late_now = (SEED_TICK as u32) + TEST_RDT_US * TICKS_PER_US + 2 * 500 * TICKS_PER_US;
        state.wire.stage_now(late_now);
        stage_rx(&mut bus, &state, req.len() as u16, &[0xFF]);

        bus.on_rx_byte_wake();

        assert!(state.sch.operations().is_empty(), "no TX scheduled");
        assert!(bus.send_policy.awaited_status_start().is_none());
        assert!(!bus.rx_wake.fast_for_test());
        assert!(!bus.fast_last.fold_active(), "fold never engaged");
    }

    #[test]
    fn wake_with_no_wait_clears_the_fast_reason() {
        // Lifecycle cleared the wait while the FAST reason was still on —
        // the next wake self-heals by clearing it (the drift window, a
        // separate reason, keeps RXNEIE up and does not schedule anything).
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(bus.rx_wake.fast_for_test());
        let _ = bus.on_tx_complete(); // clears the wait AND the FAST reason
        assert!(!bus.rx_wake.fast_for_test());

        state.rx.stage_remaining(RX_BUF_LEN as u16); // reset for clarity
        bus.on_rx_byte_wake();
        assert!(!bus.rx_wake.fast_for_test());
        assert!(state.sch.operations().is_empty());
    }

    #[test]
    fn on_tx_complete_cancels_fast_last_state() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        observe_status_start(&mut bus, &state, req.len() as u16);
        assert!(bus.fast_last.grid_active());
        assert!(bus.fast_last.fold_active());

        let _ = bus.on_tx_complete();

        assert!(!bus.fast_last.grid_active());
        assert!(!bus.fast_last.fold_active());
        assert!(matches!(
            state.fl.operations().last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn on_tx_complete_releases_wire_before_draining_pending_config() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, state, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| {
            reply.stage_id(0x42);
            reply.stage_rdt(500);
            reply.stage_reboot(BootMode::Bootloader);
            reply.send_status(empty_status()).expect("encode fits");
        });
        state.tx_bus.clear();

        assert_eq!(bus.send_policy.id(), TEST_ID);
        assert_eq!(bus.send_policy.rdt_us(), TEST_RDT_US);

        let pending_reboot = bus.on_tx_complete();

        assert_eq!(state.tx_bus.operations(), alloc::vec![TxBusOp::ReleaseBus]);
        assert_eq!(bus.send_policy.id(), 0x42);
        assert_eq!(bus.send_policy.rdt_us(), 500);
        assert_eq!(pending_reboot, Some(BootMode::Bootloader));
    }

    // ------------------------------------------------------------------
    // Chain start for slots k > 0 (DXL streaming RX §5.2)
    // ------------------------------------------------------------------

    #[test]
    fn send_status_defers_wire_send_for_chain_k_gt_zero() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        assert!(
            state.sch.operations().is_empty(),
            "chain k > 0 does not stage a deadline",
        );
        assert!(
            state.tx_bus.operations().is_empty(),
            "wire send waits for SkipComplete",
        );
        assert_eq!(bus.send_policy.awaited_predecessor(), Some(0x42));
    }

    #[test]
    fn skip_complete_matching_predecessor_triggers_start_now() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(bus.send_policy.awaited_predecessor(), Some(0x42));

        let pred_status = wire_status(0x42);
        stage_rx(&mut bus, &state, req.len() as u16, &pred_status);
        bus.poll(|_, _, _| {});

        assert_eq!(
            state.tx_bus.operations(),
            alloc::vec![TxBusOp::StartNow { byte_count }],
        );
        assert!(
            bus.send_policy.awaited_predecessor().is_none(),
            "chain-pending cleared on match",
        );
    }

    // ------------------------------------------------------------------
    // Fast Last NDTR fold (Chunk 5)
    // ------------------------------------------------------------------

    #[test]
    fn poll_captures_fold_start_cursor_at_crc() {
        // `fold_start_cursor` lands at the host chain instruction's CRC
        // byte — the codec's wire-byte cursor past that point is where
        // the First servo's leading `0xFF` will arrive.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .send_policy
            .staged_reply_for_test()
            .expect("FastSyncRead surfaces a reply context");
        assert_eq!(ctx.fold_start_cursor, req.len() as u32);
    }

    #[test]
    fn on_fold_step_short_of_the_checkpoint_stays_armed() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        // The pipeline arms at the status-start observation (task #142).
        // Stage only part of the 14-byte predecessor window — the
        // checkpoint (its last 2 bytes) hasn't landed, so the wake body
        // exits starved (deadline arm, degenerated by the mock) without
        // patching, and the pipeline stays armed for the retry.
        let start = req.len() as u16;
        observe_status_start(&mut bus, &state, start);
        let more = [0x22_u8, 0x33, 0x44];
        stage_rx(&mut bus, &state, start + 1, &more);
        let new_head = start.wrapping_add(1 + more.len() as u16);
        state
            .rx
            .stage_remaining((RX_BUF_LEN as u16).wrapping_sub(new_head));

        bus.on_fold_step();
        assert!(
            bus.fast_last.fold_active(),
            "checkpoint not yet on the wire"
        );
        assert!(bus.fast_last.grid_active(), "retry CMP armed");
        assert_eq!(bus.codec.tx.trailing_crc_slot_for_test(), [0x00, 0x00]);
    }

    #[test]
    fn fold_step_with_full_window_patches_crc_and_finalizes() {
        // The wire start is hardware-armed at the observation; the grid
        // body's only job is landing the chain-CRC patch. Stage the whole
        // `predecessor_bytes` window so the (single-step) grid body folds
        // everything, commits the far-horizon stash, and finalizes —
        // trailing slot no longer the placeholder `[0x00, 0x00]`.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        // bytes_before for slot 1 of Fast SyncRead length=2 is
        // `FAST_RESPONSE_SLOT0_BYTES(10) + data(2) + crc(2) = 14`; the
        // observation supplies byte 0 (the leading `0xFF`), stage the
        // other 13 so finalize lands in this fold body.
        let start = req.len() as u16;
        observe_status_start(&mut bus, &state, start);
        let rest = [
            0xA2_u8, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
        ];
        stage_rx(&mut bus, &state, start + 1, &rest);
        let new_head = start.wrapping_add(1 + rest.len() as u16);
        state
            .rx
            .stage_remaining((RX_BUF_LEN as u16).wrapping_sub(new_head));

        assert_eq!(bus.codec.tx.trailing_crc_slot_for_test(), [0x00, 0x00]);

        bus.on_fold_step();

        assert_ne!(
            bus.codec.tx.trailing_crc_slot_for_test(),
            [0x00, 0x00],
            "patch_crc should overwrite the placeholder slot"
        );
        assert!(!bus.fast_last.fold_active());
        assert!(!bus.fast_last.grid_active());
        assert!(
            state.sch.operations().contains(&ScheduleOp::CommitPending),
            "final grid body commits any far-horizon TX stash",
        );
        assert_eq!(
            state.telemetry.patch_miss_count(),
            0,
            "finalize path must not bump the deadline-miss counter",
        );
    }

    #[test]
    fn late_last_observation_folds_inline_and_patches_crc() {
        // Short predecessor at high baud: the observation lands AFTER the
        // slot deadline (the whole 14-byte First is already in the ring),
        // so the hardware kickoff fires ASAP and the CRC patch races the
        // TX DMA's read of the trailing slot. `on_rx_byte_wake` must fold
        // inline — a pended grid CMP would arrive one PFIC re-entry too
        // late for a short own reply. Hardware regression pin
        // (fast_injected::fast_last at 3M shipped the placeholder CRC).
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(state.rx.status_start_watched());

        // The full injected-First emission (10 + L + crc2 = 14 bytes) plus
        // one leading byte of the next slot already in the ring; the
        // estimate back-dates `now` by all 15 whole bytes, so the observed
        // status start — and the derived slot deadline — sit before `now`:
        // the slot window is already over.
        let pred = [
            0xFF_u8, 0xFF, 0xFD, 0x00, 0xFE, 0x0B, 0x00, 0x55, 0x00, 0x32, 0xAA, 0xBB, 0x12, 0x34,
            0x56,
        ];
        stage_rx(&mut bus, &state, req.len() as u16, &pred);
        let late_now = SEED_TICK as u32 + 20_000;
        state.wire.stage_now(late_now);
        assert_eq!(bus.codec.tx.trailing_crc_slot_for_test(), [0x00, 0x00]);

        bus.on_rx_byte_wake();

        assert_ne!(
            bus.codec.tx.trailing_crc_slot_for_test(),
            [0x00, 0x00],
            "inline fold must patch before the wake returns"
        );
        assert!(!bus.fast_last.fold_active());
        assert!(!bus.fast_last.grid_active());
        // Fold-before-fire: the inline fold (whose early-exit fires the
        // commit closure) must complete BEFORE the wire start is armed.
        assert!(matches!(
            state.sch.operations().as_slice(),
            [
                ScheduleOp::CommitPending,
                ScheduleOp::Schedule {
                    kind: SendKind::FastLast,
                    ..
                },
            ]
        ));
        assert_eq!(state.telemetry.patch_miss_count(), 0);
    }

    #[test]
    fn full_window_at_observation_inline_folds_and_patches() {
        // The whole 14-byte First window is already in the ring, so the
        // byte-count estimate back-dates `now` by 14 frames — the derived
        // slot deadline lands exactly at `now` (window just ended). The
        // observation folds inline (deadline reached) and, with the full
        // window present, the CRC patch lands before `on_rx_byte_wake`
        // returns, no CMP dispatch involved.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        let start = req.len() as u16;
        let window = [
            0xFF_u8, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
        ];
        stage_rx(&mut bus, &state, start, &window);
        let full_window = window.len() as u32 * (BITS_PER_FRAME * TICKS_PER_BIT_3M) as u32;
        state.wire.stage_now(SEED_TICK as u32 + full_window);
        assert_eq!(bus.codec.tx.trailing_crc_slot_for_test(), [0x00, 0x00]);

        bus.on_rx_byte_wake();

        assert_ne!(
            bus.codec.tx.trailing_crc_slot_for_test(),
            [0x00, 0x00],
            "inline fold body must patch within the observation",
        );
        assert!(!bus.fast_last.fold_active());
        assert!(!bus.fast_last.grid_active());
        assert_eq!(state.telemetry.patch_miss_count(), 0);
    }

    #[test]
    fn on_time_observation_past_inline_horizon_leaves_fold_to_the_grid() {
        // Slot 3 of a 4-servo chain: the predecessor window (24 bytes) puts
        // the deadline past INLINE_FOLD_HORIZON_TICKS — no inline body runs;
        // the fold waits for its CMP grid.
        let req = wire_fast_sync_read(0, 2, &[0x42, 0x43, 0x44, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        observe_status_start(&mut bus, &state, req.len() as u16);

        assert!(bus.fast_last.grid_active());
        assert_eq!(
            bus.codec.tx.trailing_crc_slot_for_test(),
            [0x00, 0x00],
            "no inline body — nothing patched at observation time",
        );
    }

    #[test]
    fn fold_step_starved_reschedules_then_window_expiry_records_miss() {
        // Silent predecessor: only the observed leading byte ever
        // arrives. The final grid body hands off to a completion body
        // (fold stays active, one CMP re-armed); the completion body
        // re-arms while the patch window is open, and exits with one
        // `crc_patch_deadline_miss` once the TX drain closes it — the
        // hardware kickoff fired in parallel, so the window always
        // closes.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        let payload = [0xAA_u8, 0xBB];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        observe_status_start(&mut bus, &state, req.len() as u16);

        bus.on_fold_step();
        assert!(bus.fast_last.grid_active(), "completion body pending");
        assert!(bus.fast_last.fold_active());
        assert_eq!(state.telemetry.patch_miss_count(), 0);
        let cmp_count = state.fl.operations().len();

        bus.on_fold_step();
        assert_eq!(
            state.fl.operations().len(),
            cmp_count + 1,
            "starved completion body re-arms one CMP",
        );

        state.fl.stage_patch_window_expired(true);
        bus.on_fold_step();
        assert!(!bus.fast_last.grid_active());
        assert!(
            bus.fast_last.fold_active(),
            "fold state stays for on_tx_complete's cleanup",
        );
        assert_eq!(
            state.telemetry.patch_miss_count(),
            1,
            "expired-window exit must bump the deadline-miss counter",
        );
    }

    #[test]
    fn schedule_due_owned_match_still_runs_fold_body_when_grid_active() {
        // Far-horizon FastLast commit: the SysTick handoff arm and the
        // fold grid share the one CMP register, so a handoff arm clobbers
        // the grid's pending CMP. After the scheduler consumes its match,
        // a fold body must run so the grid re-arms its own.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &[0xAA, 0xBB],
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        observe_status_start(&mut bus, &state, req.len() as u16);
        assert!(bus.fast_last.grid_active());

        state.sch.stage_schedule_due_owned(true);
        let cmp_count = state.fl.operations().len();
        bus.on_schedule_due();
        assert!(
            state.fl.operations().len() > cmp_count,
            "grid re-armed its CMP after the handoff consumed the match",
        );
    }

    #[test]
    fn schedule_due_owned_match_without_grid_stops_there() {
        let (mut bus, state) = make_bus();
        state.sch.stage_schedule_due_owned(true);
        bus.on_schedule_due();
        assert!(
            state.fl.operations().is_empty(),
            "no fold body (not even a defensive cancel) without an active grid",
        );
    }
}
