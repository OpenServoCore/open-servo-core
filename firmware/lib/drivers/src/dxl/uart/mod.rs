//! DXL-over-UART transport. Composite over the codec (bytes ↔ packets)
//! and the clock (tick math + drift integration). The chip-side ISR layer
//! only ever reaches through `DxlUart`; cross-sub-driver routing (e.g.
//! the BT-pair walk that feeds drift samples from codec into clock) lives
//! in this file per driver-pattern §4 + §10.1.
//!
//! Generic over its leaf providers AND its three storage sizes — see the
//! [`DxlUart`] doc. `firmware/ch32/src/runtime/registry.rs` binds each to
//! its V006 value. Future sub-drivers (`Tx`, `ChainCatchup`) land as
//! additional fields on this composite (TX work also extends `Codec`).

pub mod clock;
pub mod codec;
pub mod fast_last;
mod poll_src;
mod reply_handle;
mod send_policy;

pub use poll_src::PollSrc;
pub use reply_handle::ReplyHandle;

#[cfg(test)]
mod test_support;

use dxl_protocol::streaming::{CrcResult, Event, HeaderEvent, PayloadEvent};
use osc_core::BootMode;

use crate::traits::dxl::{Providers, RxDma, TxBus, TxScheduler, WireClock};
use clock::Clock;
use codec::{Codec, PollAction, PollEvent};
use fast_last::FastLast;
use send_policy::{SendPolicy, header_target};

/// Bits on the wire for a single UART character: 1 start + 8 data + 1 stop
/// (8N1). Multiply by `ticks_per_bit` to get one byte's wire duration in
/// scheduler ticks. Also the IDLE detection threshold — CH32V00X RM §UART:
/// "an idle frame is 10-or-11-bit high, including the stop bit"; M=0 → 10.
pub(crate) const BITS_PER_FRAME: u16 = 10;

/// The DXL bus composite. `P` bundles the chip-side leaf interfaces this
/// driver pulls — see [`Providers`]. The const generics are storage sizes:
///
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per doc §8.1).
/// - `EDGE_BUF_LEN`: DMA1_CH7 edge-timestamp ring depth (typically 128 /
///   option A in doc §8.4; 64 / option B trades CPU for memory).
/// - `TX_BUF_LEN`: DMA1_CH4 source-buffer depth sized to
///   `osc_core::services::dxl::limits::DXL_TX_MAX_BYTES` (140 with the
///   default control-RW). Held by `Codec` so encoder methods write into
///   driver-owned storage instead of a chip-side static.
pub struct DxlUart<
    P: Providers,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
    const TX_BUF_LEN: usize,
> {
    codec: Codec<P::EdgeDma, P::Crc, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
    clock: Clock<P::UsartBaud, P::ClockTrim>,
    /// NDTR-only readback for DMA1_CH5 (the RX byte ring). Plumbed
    /// independently of the parser-path `on_rx_progress` calls so the
    /// Fast Last fold body's intra-loop refresh doesn't go through the
    /// chip-side ISR — see [`Self::on_fold_step`] / [`Self::on_tx_start`].
    rx_dma: P::RxDma,
    scheduler: P::TxScheduler,
    /// Chip-side bus-control provider. Used by [`Self::on_tx_start`] /
    /// [`Self::on_tx_complete`] for the scheduled wire-driver lifecycle,
    /// and by [`Self::poll`]'s SkipComplete arm for the Plain chain
    /// k > 0 sequence-driven fire path (`docs/dxl-streaming-rx.md` §5.2).
    tx_bus: P::TxBus,
    /// Fast Last fold pipeline for Fast Sync / Bulk Read Last replies — a
    /// §4.3 sub-composite of the periodic-walk grid scheduler and the
    /// chain-CRC fold engine. Armed at `send_slot(Last)` via `ReplyHandle`;
    /// the grid drives one body per CMP, the fold engine's per-byte hook is
    /// wired into the codec's `drain_raw` callback and finalize patches our
    /// own trailing CRC slot before DMA1_CH4 reads it.
    fast_last: FastLast<P::FastLastScheduler, P::Crc>,
    /// WireClock u32 readout used by `on_rx_advance`, `on_rx_idle`, and
    /// `poll` to source `now` without taking it as a parameter. The
    /// chip-side provider hides any peripheral-side composition (TIM2 u16
    /// IC stamps lifted via SysTick u32, etc.) — see [`WireClock`].
    wire_clock: P::WireClock,

    /// Send-side policy sub-composite — bus identity + staged-config
    /// mailbox (committed at [`Self::on_tx_complete`]), the parse-side
    /// Instruction tracker, and the reply gate holding the staged
    /// [`ReplyContext`] / chain-pending predecessor wait. Per
    /// driver-pattern §7.4 — driver-derivable wire shape stays in the
    /// driver, not on the trait surface.
    send_policy: SendPolicy,
}

impl<P: Providers, const RX_BUF_LEN: usize, const EDGE_BUF_LEN: usize, const TX_BUF_LEN: usize>
    DxlUart<P, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        codec: Codec<P::EdgeDma, P::Crc, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>,
        clock: Clock<P::UsartBaud, P::ClockTrim>,
        rx_dma: P::RxDma,
        scheduler: P::TxScheduler,
        tx_bus: P::TxBus,
        fast_last: FastLast<P::FastLastScheduler, P::Crc>,
        wire_clock: P::WireClock,
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
            send_policy: SendPolicy::new(id, rdt_us),
        };
        // Seed the codec's edge parser with the Clock's initial-baud
        // edge-stamp compensation. Subsequent baud changes re-publish via
        // `Self::on_tx_complete` after `Clock::on_tx_complete` applies the
        // pending baud.
        s.codec.on_baud_change(s.clock.rx_edge_comp_ticks());
        s
    }

    /// USART1 IDLE ISR entry — refresh the ET producer head and stash
    /// `(now, LineIdle)` for the Crc-time fallback path. Small-packet
    /// backstop for the parser drain (chip ISR follows with
    /// `services.poll` to drain any packet whose last byte arrived
    /// before RX HT/TC tripped).
    pub fn on_rx_idle(&mut self) {
        let now = self.wire_clock.now();
        self.codec.on_idle(now);
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
        self.codec.on_rx_progress(self.rx_dma.remaining());
        self.codec.on_byte_batch_wake(now);
    }

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
    /// - Crc → stamp packet-end tick, derive [`ReplyContext`]. Anchor
    ///   reset is codec-owned (after `walk_pairs_back` at the same Crc).
    /// - Resync → drop inflight (codec resets anchor).
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
        // cursor while a Fast Last fold is in flight. `pause_edges()`
        // stops future edge-DMA advancing, but IDLE-triggered and RX
        // byte-ring HT/TC-triggered polls still need this entry gate to
        // honor the RX-tail ownership contract in
        // `dxl-streaming-rx.md` §6.
        if self.fast_last.fold_active() {
            return;
        }
        self.codec.on_rx_progress(self.rx_dma.remaining());
        let now = self.wire_clock.now();
        let id = self.send_policy.id();
        let ticks_per_bit = self.clock.ticks_per_bit();
        let Self {
            codec,
            clock,
            scheduler,
            tx_bus,
            fast_last,
            rx_dma,
            send_policy,
            ..
        } = self;
        // Drift pairs from the codec's retroactive walk at Crc and drain
        // into `clock.on_byte_pair` after `codec.poll` returns — routing
        // them through `clock` inside the poll would force a second
        // `&mut clock` capture concurrent with the event sink's
        // `ReplyHandle { clock, .. }` capture. Bound at one DXL packet's
        // worth of body bytes (well under 32).
        let mut pair_buf: heapless::Vec<(u16, u16), 32> = heapless::Vec::new();
        let n_pairs_wanted = clock.samples_wanted_per_packet();
        let (rx, tx) = codec.split_mut();
        rx.poll(now, ticks_per_bit, n_pairs_wanted, &mut pair_buf, |pe, rx_inner| match pe {
            PollEvent::Event {
                ev,
                ring,
                next_status_pos,
            } => {
                let action = match ev {
                    Event::Header(HeaderEvent::Instruction(h)) => {
                        let skip_target = send_policy.on_instruction_header(&h);
                        crate::log::trace!(
                            "dxl[id={}]: event=header_instruction target={} addressable={}",
                            id,
                            header_target(&h).as_byte(),
                            skip_target.is_none()
                        );
                        match skip_target {
                            None => PollAction::Continue,
                            Some(target) => PollAction::Skip { id: target },
                        }
                    }
                    Event::Header(HeaderEvent::Status(sh)) => {
                        crate::log::trace!(
                            "dxl[id={}]: event=header_status status_id={}",
                            id,
                            sh.id.as_byte()
                        );
                        send_policy.on_status_header();
                        PollAction::Skip {
                            id: sh.id.as_byte(),
                        }
                    }
                    Event::Payload(PayloadEvent::Instruction(p)) => {
                        crate::log::trace!("dxl[id={}]: event=payload_instruction", id);
                        send_policy.on_slot(&p);
                        PollAction::Continue
                    }
                    Event::Payload(PayloadEvent::Status(_)) => {
                        debug_assert!(false, "Status payload should have been byte-skipped");
                        PollAction::Continue
                    }
                    Event::Crc(CrcResult::Good) => {
                        crate::log::trace!("dxl[id={}]: event=crc(good)", id);
                        if send_policy.is_tracking() {
                            // Anchor is current by construction — the codec
                            // walked the edge parser in lockstep with the byte
                            // parser through the 2 CRC bytes before emitting
                            // this event. `last_byte_start` reflects the CRC
                            // byte's start; `packet_end_tick` is one byte-time
                            // past that. Fallback fires only when interference
                            // / edge loss starved the edge parser. FAST chain
                            // ops skip the fallback — see
                            // `SendPolicy::allows_packet_end_fallback`.
                            // Both paths source `(cap_now, src)` from the most
                            // recent ISR entry — the primary path needs `src`
                            // so the u16-stamp lift picks the right drain
                            // reference (HT/TC ≈ stamp+1·bp vs IDLE ≈ stamp+
                            // 2·BITS_PER_FRAME·tpb), critical at 9600 baud
                            // where IDLE elapsed exceeds the u16 wrap.
                            let (cap_now, src) = rx_inner.last_isr_capture();
                            let packet_end_tick =
                                match rx_inner.packet_end_tick(ticks_per_bit, cap_now, src) {
                                    Some(t) => Some(t),
                                    None => {
                                        rx_dma.record_edge_anchor_miss();
                                        send_policy.allows_packet_end_fallback().then(|| {
                                            rx_inner.packet_end_tick_fallback(
                                                src,
                                                cap_now,
                                                ticks_per_bit,
                                            )
                                        })
                                    }
                                };
                            // At Crc-of-host-instruction, the codec's wire
                            // position has just walked past the request's
                            // last CRC byte — the next byte on the wire is
                            // the First predecessor's leading `0xFF`. So
                            // `next_status_pos` is exactly the fold-start
                            // cursor for the Fast Last CRC engine.
                            match send_policy.on_crc_good(packet_end_tick, next_status_pos, src)
                            {
                                Some(t) => {
                                    crate::log::debug!(
                                        "dxl[id={}]: crc packet_end_tick={}",
                                        id,
                                        t
                                    );
                                }
                                None => {
                                    crate::log::debug!(
                                        "dxl: crc anchor missing and fallback disallowed — drop reply"
                                    );
                                }
                            }
                        }
                        // Anchor reset is owned by the codec (after
                        // walk_pairs_back at the same Crc iteration) —
                        // `packet_end_tick` above was the driver's last
                        // read on this packet.
                        PollAction::Continue
                    }
                    Event::Crc(CrcResult::Bad) | Event::Resync(_) => {
                        crate::log::trace!("dxl[id={}]: event=crc(bad)/resync", id);
                        send_policy.on_resync();
                        PollAction::Continue
                    }
                    Event::Sync => PollAction::Continue,
                };
                let mut reply = ReplyHandle {
                    tx,
                    scheduler,
                    fast_last,
                    clock,
                    send_policy,
                };
                f(ev, ring, &mut reply);
                // If `f()` armed the Fast Last fold (via `send_slot(Last)`),
                // the in-flight poll must exit before the parser eats any
                // more bytes — those bytes belong to the fold engine. The
                // SysTick CMP grid's `pause_edges()` stops future polls only;
                // without this bail-out the parser+skip path consumes the
                // predecessor's leading bytes in the same poll that armed
                // the fold, and the fold engine starves.
                if fast_last.fold_active() {
                    PollAction::Stop
                } else {
                    action
                }
            }
            PollEvent::SkipComplete { id: pred } => {
                crate::log::trace!(
                    "dxl[id={}]: skip_complete pred={} predecessor_id={:?}",
                    id,
                    pred,
                    send_policy.awaited_predecessor()
                );
                if send_policy.on_skip_complete(pred) {
                    crate::log::debug!(
                        "dxl[id={}]: skip_complete match pred={} -> start_now byte_count={}",
                        id,
                        pred,
                        tx.tx_len()
                    );
                    tx_bus.start_now(tx.tx_len());
                }
                // Codec clears `packet_is_instruction` internally on
                // SkipComplete; no anchor is set between the previous
                // Crc (where the codec resets it) and here, so no
                // driver-side invalidation is required.
                PollAction::Continue
            }
        });
        // Drain the codec's retroactive-walk pairs into the drift
        // integrator. Cap is already applied inside the walk
        // (`n_pairs_wanted`), so the drain is unbounded here.
        for &(prev, curr) in pair_buf.iter() {
            clock.on_byte_pair(prev, curr);
        }
        // Apply any pending trim correction at the RX-side packet
        // boundary. Idempotent when nothing changed; necessary so
        // foreign-instruction packets — which never reach
        // `on_tx_complete` — still commit their drift samples per
        // [[drift_sampling_instruction_only]].
        clock.on_rx_packet_end();
    }

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
    }

    /// Inspection passthrough to [`Clock::projected_phase_error_hclk`] —
    /// production scheduler call sites read this directly off `self.clock`;
    /// this exists so sim/test code can sample the same value externally
    /// without exposing the integrator state itself.
    #[allow(dead_code)]
    pub fn projected_phase_error_hclk(&self, distance_hclk: u32) -> i32 {
        self.clock.projected_phase_error_hclk(distance_hclk)
    }

    /// The TX-start tick has arrived (chip-side CC3 IRQ). Activates the
    /// wire driver FIRST so the first wire bit lands on `fire_deadline`;
    /// for Fast Last replies the body then tails with the post-fire
    /// residue fold — exit policy lives in [`FastLast::on_tx_start`]; the
    /// composite supplies one drain pass per iteration. Each
    /// [`CodecRx::drain_raw`] pass refreshes the byte-ring producer head
    /// from [`RxDma::remaining`] so newly-arrived GUARD bytes become
    /// visible inside the spin (per `dxl-streaming-rx.md` §6, the
    /// `crc_patch_deadline_miss` counter is a bench-defended floor signal
    /// at the 3 Mbaud floor, not a wire-correctness failure).
    pub fn on_tx_start(&mut self) {
        self.tx_bus.handle_start();
        let Self {
            codec,
            rx_dma,
            fast_last,
            ..
        } = self;
        let (rx, tx) = codec.split_mut();
        fast_last.on_tx_start(|fl_crc| {
            rx.drain_raw(rx_dma, |slice, base_cursor| {
                fl_crc.on_slice(slice, base_cursor, tx);
            });
        });
    }

    /// One Fast Last periodic-walk fold body is due (chip-side SysTick
    /// CMP). Body drives [`FsmScheduler::on_step`] forward — the walker
    /// closure drains pending RX bytes raw through [`FoldEngine::on_slice`]
    /// and returns the cumulative folded count for the FSM's `target =
    /// predecessor_bytes − GUARD` busy-wait exit (`fast_last/`).
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
        fl_sched.on_step(
            || {
                rx.drain_raw(rx_dma, |slice, base_cursor| {
                    fl_crc.on_slice(slice, base_cursor, tx);
                });
                fl_crc.bytes_folded()
            },
            || scheduler.commit_pending(),
        );
    }

    /// Long-horizon timer match fired (chip-side: SysTick CMP). Two consumers
    /// share the CMP — the TX-scheduler handoff arm (multi-wrap distances
    /// where direct TIM2 CC3 can't span the wait) and the Fast Last walk
    /// grid. The TX-scheduler reports whether the match was its own; if so,
    /// we're done, otherwise it's a Fast Last grid step.
    pub fn on_schedule_due(&mut self) {
        if self.scheduler.on_schedule_due() {
            return;
        }
        self.on_fold_step();
    }

    /// USART1 TC fired — the reply has fully drained the wire. Release
    /// the wire driver *first* (drop TX_EN, mask TC IRQ, disable DMA) so
    /// stale TX_EN doesn't sit on the bus while pending config mutates;
    /// then drain staged writes (id / baud / trim / rdt) and surface any
    /// pending reboot to the chip-side ISR. Also disarms the Fast Last
    /// state (idempotent — both already disarmed naturally on the
    /// successful path). Reboot is returned (rather than self-applied)
    /// because the chip controls how the reset actually happens; the
    /// driver only knows it was asked.
    pub fn on_tx_complete(&mut self) -> Option<BootMode> {
        self.tx_bus.handle_tx_complete();
        self.fast_last.cancel();
        // Per `dxl-streaming-rx.md` §5.3: our own TX completion is a
        // packet boundary at which stale chain state must reset. The
        // §5.3 "next instruction header" trigger is too late for the
        // universal byte-skip — at slow baud the deadline-bounded skip
        // can outlive the inter-packet gap and eat the next preamble
        // before the parser ever reaches the header event.
        self.codec.cancel_skip();
        let reboot = self.send_policy.on_tx_complete();
        self.clock.on_tx_complete();
        // Clock just applied any pending baud — refresh the codec's edge
        // parser with the new per-baud edge-stamp compensation. Idempotent
        // when no baud change landed (the value is the same as before).
        self.codec.on_baud_change(self.clock.rx_edge_comp_ticks());
        reboot
    }

    /// Stable peripheral-memory address for DMA1_CH7's destination buffer.
    pub fn edges_addr(&self) -> usize {
        self.codec.edges_addr()
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
    /// transfer count for the next fire.
    pub fn tx_len(&self) -> u16 {
        self.codec.tx_len()
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    use super::send_policy::PING_STATUS_FRAME_BYTES;
    use super::test_support::{SEED_TICK, TEST_ID, TEST_RDT_US, TICKS_PER_BIT_3M};
    use super::*;
    use crate::mocks::{
        FastLastSchedulerOp, MockClockTrim, MockEdgeDma, MockFastLastScheduler, MockRxDma,
        MockTxBus, MockTxScheduler, MockUsartBaud, MockWireClock, ScheduleOp, TestProviders,
        TxBusOp,
    };
    use crate::traits::dxl::{DmaFlags, SendKind};
    use dxl_protocol::streaming::InstructionHeader;
    use dxl_protocol::types::StatusError;
    use dxl_protocol::wire::BROADCAST_ID;
    use dxl_protocol::{Id, InstructionEncoder, Slot, SoftwareCrcUmts, Status, StatusEncoder};
    use heapless::Vec;
    use osc_core::BaudRate;

    /// Test-side storage sizing — matches V006 defaults per doc §§8.1, 8.3,
    /// 8.4 so any drift between driver tests and chip-side reality stays
    /// visible.
    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;
    const TX_BUF_LEN: usize = 140;

    type TestCodec = Codec<MockEdgeDma, SoftwareCrcUmts, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;
    type TestBus = DxlUart<TestProviders, RX_BUF_LEN, EDGE_BUF_LEN, TX_BUF_LEN>;

    // Local state companions mirror `osc-integration::mocks::*` — clone-
    // shared Cell/RefCell handles the mock closures push into so tests
    // read observed effects out of the state rather than the mock. C5
    // hoists these into a shared `test_support` module.

    #[derive(Clone, Default)]
    struct SchedState {
        ops: Rc<RefCell<alloc::vec::Vec<ScheduleOp>>>,
    }
    impl SchedState {
        fn operations(&self) -> alloc::vec::Vec<ScheduleOp> {
            self.ops.borrow().clone()
        }
    }
    fn mk_scheduler() -> (MockTxScheduler, SchedState) {
        let state = SchedState::default();
        let mut m = MockTxScheduler::new();
        {
            let ops = state.ops.clone();
            m.expect_schedule()
                .returning_st(move |deadline, byte_count, kind| {
                    ops.borrow_mut().push(ScheduleOp::Schedule {
                        deadline,
                        byte_count,
                        kind,
                    });
                });
        }
        {
            let ops = state.ops.clone();
            m.expect_commit_pending().returning_st(move || {
                ops.borrow_mut().push(ScheduleOp::CommitPending);
            });
        }
        {
            let ops = state.ops.clone();
            m.expect_cancel().returning_st(move || {
                ops.borrow_mut().push(ScheduleOp::Cancel);
            });
        }
        m.expect_on_schedule_due().returning_st(|| false);
        (m, state)
    }

    #[derive(Clone, Default)]
    struct TxBusState {
        ops: Rc<RefCell<alloc::vec::Vec<TxBusOp>>>,
    }
    impl TxBusState {
        fn operations(&self) -> alloc::vec::Vec<TxBusOp> {
            self.ops.borrow().clone()
        }
        fn clear(&self) {
            self.ops.borrow_mut().clear();
        }
    }
    fn mk_tx_bus() -> (MockTxBus, TxBusState) {
        let state = TxBusState::default();
        let mut m = MockTxBus::new();
        {
            let ops = state.ops.clone();
            m.expect_start_now().returning_st(move |byte_count| {
                ops.borrow_mut().push(TxBusOp::StartNow { byte_count });
            });
        }
        {
            let ops = state.ops.clone();
            m.expect_handle_start().returning_st(move || {
                ops.borrow_mut().push(TxBusOp::HandleStart);
            });
        }
        {
            let ops = state.ops.clone();
            m.expect_handle_tx_complete().returning_st(move || {
                ops.borrow_mut().push(TxBusOp::HandleTxComplete);
            });
        }
        (m, state)
    }

    #[derive(Clone, Default)]
    struct RxDmaState {
        remaining: Rc<Cell<u16>>,
    }
    impl RxDmaState {
        fn stage_remaining(&self, n: u16) {
            self.remaining.set(n);
        }
    }
    fn mk_rx_dma() -> (MockRxDma, RxDmaState) {
        let state = RxDmaState::default();
        let mut m = MockRxDma::new();
        {
            let r = state.remaining.clone();
            m.expect_remaining().returning_st(move || r.get());
        }
        m.expect_read_and_ack().returning_st(DmaFlags::default);
        m.expect_record_edge_anchor_miss().returning_st(|| ());
        (m, state)
    }

    #[derive(Clone, Default)]
    struct FastLastState {
        ops: Rc<RefCell<alloc::vec::Vec<FastLastSchedulerOp>>>,
        deadline_passed: Rc<Cell<bool>>,
        patch_window_expired: Rc<Cell<bool>>,
        patch_miss_count: Rc<Cell<u32>>,
    }
    impl FastLastState {
        fn operations(&self) -> alloc::vec::Vec<FastLastSchedulerOp> {
            self.ops.borrow().clone()
        }
        fn stage_patch_window_expired(&self, v: bool) {
            self.patch_window_expired.set(v);
        }
        fn patch_miss_count(&self) -> u32 {
            self.patch_miss_count.get()
        }
    }
    fn mk_fast_last() -> (MockFastLastScheduler, FastLastState) {
        let state = FastLastState::default();
        let mut m = MockFastLastScheduler::new();
        {
            let ops = state.ops.clone();
            m.expect_set_deadline().returning_st(move |deadline| {
                ops.borrow_mut()
                    .push(FastLastSchedulerOp::SetDeadline { deadline });
            });
        }
        {
            let ops = state.ops.clone();
            m.expect_schedule().returning_st(move |deadline| {
                ops.borrow_mut()
                    .push(FastLastSchedulerOp::Schedule { deadline });
            });
        }
        {
            let dp = state.deadline_passed.clone();
            m.expect_deadline_passed().returning_st(move || dp.get());
        }
        {
            let pwe = state.patch_window_expired.clone();
            m.expect_patch_window_expired()
                .returning_st(move || pwe.get());
        }
        {
            let c = state.patch_miss_count.clone();
            m.expect_record_patch_deadline_miss().returning_st(move || {
                c.set(c.get().wrapping_add(1));
            });
        }
        {
            let ops = state.ops.clone();
            m.expect_cancel().returning_st(move || {
                ops.borrow_mut().push(FastLastSchedulerOp::Cancel);
            });
        }
        (m, state)
    }

    fn mk_wire_clock() -> MockWireClock {
        let mut m = MockWireClock::new();
        m.expect_now().returning_st(|| SEED_TICK as u32);
        m
    }

    fn mk_clock_trim() -> MockClockTrim {
        super::test_support::mk_clock_trim().0
    }

    fn mk_usart_baud() -> MockUsartBaud {
        super::test_support::mk_usart_baud().0
    }

    #[derive(Clone, Default)]
    struct EdgeDmaState {
        remaining: Rc<Cell<u16>>,
    }
    impl EdgeDmaState {
        fn stage_remaining(&self, n: u16) {
            self.remaining.set(n);
        }
    }
    fn mk_edge_dma() -> (MockEdgeDma, EdgeDmaState) {
        let state = EdgeDmaState::default();
        let mut m = MockEdgeDma::default();
        {
            let r = state.remaining.clone();
            m.expect_remaining().returning_st(move || r.get());
        }
        (m, state)
    }

    struct TestState {
        sch: SchedState,
        tx_bus: TxBusState,
        rx: RxDmaState,
        edge: EdgeDmaState,
        fl: FastLastState,
    }

    fn make_clock(baud: BaudRate) -> Clock<MockUsartBaud, MockClockTrim> {
        Clock::new(baud, mk_usart_baud(), mk_clock_trim())
    }

    fn make_bus_with(
        codec: TestCodec,
        edge_state: EdgeDmaState,
        baud: BaudRate,
    ) -> (TestBus, TestState) {
        let (rx_dma, rx_state) = mk_rx_dma();
        let (scheduler, sch_state) = mk_scheduler();
        let (tx_bus, tx_bus_state) = mk_tx_bus();
        let (fl_sched, fl_state) = mk_fast_last();
        let bus = DxlUart::new(
            codec,
            make_clock(baud),
            rx_dma,
            scheduler,
            tx_bus,
            FastLast::new(fl_sched),
            mk_wire_clock(),
            TEST_ID,
            TEST_RDT_US,
        );
        (
            bus,
            TestState {
                sch: sch_state,
                tx_bus: tx_bus_state,
                rx: rx_state,
                edge: edge_state,
                fl: fl_state,
            },
        )
    }

    fn make_bus() -> (TestBus, TestState) {
        let (edge_dma, edge_state) = mk_edge_dma();
        make_bus_with(Codec::new(edge_dma), edge_state, BaudRate::B3000000)
    }

    /// Stage a matching falling-edge signature for the last 4 bytes of `pkt`
    /// so `codec.poll`'s `anchor_at_tail` at Crc-good resolves
    /// `tail_anchor = SEED_TICK` — the composite then reads
    /// `packet_end_tick = SEED_TICK + BITS_PER_FRAME·tpb`. Also stashes the
    /// DMA-ISR capture at `SEED_TICK` so `lift`'s u16→u32 reconstruction
    /// stays in-wrap.
    fn force_anchor(bus: &mut TestBus, state: &TestState, pkt: &[u8]) {
        // TAIL_BYTES_FOR_ANCHOR = 4 — private to `codec::mod`; kept in sync
        // with the codec's Crc-time tail read.
        let tail_len = 4usize.min(pkt.len());
        let tail = &pkt[pkt.len() - tail_len..];
        let total_edges =
            bus.codec
                .stage_tail_signature_for_test(tail, TICKS_PER_BIT_3M, SEED_TICK);
        // `on_publish(remaining)` will advance `write_seq` by `total_edges`
        // (from ring position 0 to `total_edges`).
        state
            .edge
            .stage_remaining((EDGE_BUF_LEN as u16) - total_edges);
        bus.codec.on_byte_batch_wake(SEED_TICK as u32);
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

    fn wire_ping(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), StatusError::OK)
            .unwrap();
        out
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

    fn wire_bulk_read(slots: &[(u8, u16, u16)]) -> Vec<u8, 32> {
        use dxl_protocol::BulkReadEntry;
        let mut entries: Vec<BulkReadEntry, 4> = Vec::new();
        for &(id, address, length) in slots {
            entries
                .push(BulkReadEntry {
                    id: Id::new(id),
                    address,
                    length,
                })
                .unwrap();
        }
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEncoder::<_, SoftwareCrcUmts>::new(&mut out)
            .bulk_read(&entries)
            .unwrap();
        out
    }

    /// Construct a bus pre-loaded with `pkt` bytes and a forced classifier
    /// anchor at [`SEED_TICK`] so `packet_end_tick` reads `SEED_TICK + 10·tpb`
    /// at Crc-good. Returns the bus, state, and expected packet-end tick.
    fn bus_seeded_with(pkt: &[u8]) -> (TestBus, TestState, u32) {
        let (mut bus, state) = make_bus();
        stage_rx(&mut bus, &state, 0, pkt);
        force_anchor(&mut bus, &state, pkt);
        // tpb = 16 @ 3M; packet_end = SEED_TICK + 10·tpb.
        let packet_end_tick =
            (SEED_TICK as u32).wrapping_add((TICKS_PER_BIT_3M as u32).wrapping_mul(10));
        (bus, state, packet_end_tick)
    }

    fn empty_status() -> Status<'static> {
        Status::Empty {
            id: Id::new(TEST_ID),
            error: StatusError::OK,
        }
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
    fn stage_id_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_id(0x42));
        assert_eq!(bus.send_policy.id(), TEST_ID);
        let reboot = bus.on_tx_complete();
        assert_eq!(bus.send_policy.id(), 0x42);
        assert!(reboot.is_none());
    }

    #[test]
    fn stage_id_noop_when_unchanged() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_id(TEST_ID));
        assert!(bus.send_policy.pending_id().is_none());
    }

    #[test]
    fn stage_rdt_defers_until_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_rdt(500));
        assert_eq!(bus.send_policy.rdt_us(), TEST_RDT_US);
        bus.on_tx_complete();
        assert_eq!(bus.send_policy.rdt_us(), 500);
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

    #[test]
    fn stage_reboot_surfaces_through_on_tx_complete() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, _, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.stage_reboot(BootMode::Bootloader));
        assert!(bus.send_policy.pending_reboot().is_some());
        let mode = bus.on_tx_complete();
        assert_eq!(mode, Some(BootMode::Bootloader));
        assert!(bus.on_tx_complete().is_none());
    }

    #[test]
    fn rx_buf_addr_is_stable() {
        let (bus, _) = make_bus();
        let a = bus.rx_buf_addr();
        let b = bus.rx_buf_addr();
        assert_eq!(a, b);
        assert_ne!(a, 0);
    }

    // ------------------------------------------------------------------
    // Poll dispatch
    // ------------------------------------------------------------------

    #[test]
    fn poll_returns_no_events_when_no_new_bytes() {
        let (mut bus, _) = make_bus();
        let mut count = 0;
        bus.poll(|_, _, _| count += 1);
        assert_eq!(count, 0);
        bus.poll(|_, _, _| count += 1);
        assert_eq!(count, 0);
    }

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

    #[test]
    fn poll_recovers_from_bad_crc() {
        let (mut bus, state) = make_bus();
        let mut bad = wire_ping(TEST_ID);
        let crc_lo_pos = bad.len() - 2;
        bad[crc_lo_pos] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        let mut combined: Vec<u8, 32> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();

        let tags = poll_capture(&mut bus, &state, &combined);
        // Bad CRC now emits `Crc(Bad)` (was `Resync(BadCrc)` in an earlier
        // parser). Recovery is asserted via seeing a second instruction
        // header + a second `Crc` event.
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 2);
    }

    #[test]
    fn poll_handles_ring_wrap() {
        let (mut bus, state) = make_bus();
        let pkt = wire_ping(TEST_ID);
        let start = (RX_BUF_LEN as u16).wrapping_sub(4);
        bus.codec.set_rx_read_seq_for_test(start);
        stage_rx(&mut bus, &state, start, &pkt);

        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags.push(ev_tag(ev)));
        assert!(saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let (mut bus, state) = make_bus();
        let pkt = wire_ping(TEST_ID);

        let split = pkt.len() - 1;
        stage_rx(&mut bus, &state, 0, &pkt[..split]);
        let mut tags = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags.push(ev_tag(ev)));
        assert!(!saw_crc(&tags));
        assert_eq!(bus.instruction_count(), 1);

        stage_rx(&mut bus, &state, split as u16, &pkt[split..]);
        let mut tags2 = alloc::vec::Vec::new();
        bus.poll(|ev, _ring, _reply| tags2.push(ev_tag(ev)));
        assert!(saw_crc(&tags2));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn instruction_count_advances_on_every_instruction_regardless_of_id() {
        let (mut bus, state) = make_bus();
        let ours = wire_ping(TEST_ID);
        let theirs = wire_ping(0x42);
        let status = wire_status(0x42);

        let mut combined: Vec<u8, 64> = Vec::new();
        combined.extend_from_slice(&ours).unwrap();
        combined.extend_from_slice(&theirs).unwrap();
        combined.extend_from_slice(&status).unwrap();
        stage_rx(&mut bus, &state, 0, &combined);

        bus.poll(|_, _, _| {});
        assert_eq!(bus.instruction_count(), 2);
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
                deadline: packet_end_tick.wrapping_add(TEST_RDT_US.wrapping_mul(48)),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_status_drops_silently_for_foreign_instruction() {
        let ping = wire_ping(0x42);
        let (mut bus, state, _) = bus_seeded_with(&ping);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        assert!(state.sch.operations().is_empty());
    }

    #[test]
    fn send_slot_only_schedules_plain() {
        let req = wire_fast_sync_read(0, 2, &[TEST_ID]);
        let (mut bus, state, packet_end_tick) = bus_seeded_with(&req);

        let payload = [0x11_u8, 0x22];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            state.sch.operations(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: packet_end_tick.wrapping_add(TEST_RDT_US.wrapping_mul(48)),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn send_slot_last_schedules_fast_last() {
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, packet_end_tick) = bus_seeded_with(&req);

        let payload = [0xAA_u8, 0xBB];
        // First slot of 2 emits 8 (header) + 2 (error+id) + 2 (data) = 12
        // wire bytes; bytes_before for slot 1 = 12.
        let expected_ticks = TEST_RDT_US.wrapping_mul(48) + bus.clock.bytes_to_ticks(12);
        let expected = packet_end_tick.wrapping_add(expected_ticks);

        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            state.sch.operations(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: expected,
                byte_count,
                kind: SendKind::FastLast,
            }]
        );
        let fl_ops = state.fl.operations();
        assert!(matches!(
            fl_ops.as_slice(),
            [
                FastLastSchedulerOp::SetDeadline { .. },
                FastLastSchedulerOp::Schedule { .. },
            ]
        ));
        assert!(bus.fast_last.fold_active());
        assert!(bus.fast_last.grid_active());
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
    fn send_slot_without_fast_context_drops_silently() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, state, _) = bus_seeded_with(&ping);
        let payload = [0x00_u8];
        bus.poll(|_, _, reply| {
            let slot = Slot {
                id: Id::new(TEST_ID),
                error: StatusError::OK,
                data: &payload,
            };
            reply.send_slot(&slot).expect("encode fits");
        });
        assert!(state.sch.operations().is_empty());
    }

    #[test]
    fn cancel_clears_context_and_forwards() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, state, _) = bus_seeded_with(&ping);
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc(_)) {
                reply.cancel();
            }
        });
        assert_eq!(state.sch.operations(), alloc::vec![ScheduleOp::Cancel]);
        assert!(bus.send_policy.staged_reply_for_test().is_none());
    }

    #[test]
    fn send_status_consumes_ctx_so_double_send_is_silent() {
        let ping = wire_ping(TEST_ID);
        let (mut bus, state, _) = bus_seeded_with(&ping);
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc(_)) {
                reply.send_status(empty_status()).expect("encode fits");
                reply.send_status(empty_status()).expect("encode fits");
            }
        });
        assert_eq!(state.sch.operations().len(), 1);
    }

    #[test]
    fn broadcast_ping_folds_id_indexed_slot_offset() {
        let req = wire_ping(BROADCAST_ID);
        let (mut bus, state, packet_end_tick) = bus_seeded_with(&req);

        let expected_offset_ticks = bus
            .clock
            .bytes_to_ticks(TEST_ID as u32 * PING_STATUS_FRAME_BYTES);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        let byte_count = bus.codec.tx_len();
        assert!(byte_count > 0);
        assert_eq!(
            state.sch.operations(),
            alloc::vec![ScheduleOp::Schedule {
                deadline: packet_end_tick
                    .wrapping_add(TEST_RDT_US.wrapping_mul(48) + expected_offset_ticks),
                byte_count,
                kind: SendKind::Plain,
            }]
        );
    }

    #[test]
    fn on_tx_start_routes_to_handle_start() {
        let (mut bus, state) = make_bus();
        bus.on_tx_start();
        assert_eq!(state.tx_bus.operations(), alloc::vec![TxBusOp::HandleStart]);
        assert!(state.sch.operations().is_empty());
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

        assert_eq!(
            state.tx_bus.operations(),
            alloc::vec![TxBusOp::HandleTxComplete]
        );
        assert_eq!(bus.send_policy.id(), 0x42);
        assert_eq!(bus.send_policy.rdt_us(), 500);
        assert_eq!(pending_reboot, Some(BootMode::Bootloader));
    }

    // ------------------------------------------------------------------
    // Chain fire for slots k > 0 (DXL streaming RX §5.2)
    // ------------------------------------------------------------------

    #[test]
    fn sync_read_slot_zero_has_no_predecessor() {
        let req = wire_sync_read(0, 2, &[TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .send_policy
            .staged_reply_for_test()
            .expect("SyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, None);
    }

    #[test]
    fn sync_read_slot_k_gt_zero_records_immediate_predecessor() {
        let req = wire_sync_read(0, 2, &[0x42, 0x09, TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .send_policy
            .staged_reply_for_test()
            .expect("SyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, Some(0x09));
    }

    #[test]
    fn bulk_read_slot_k_gt_zero_records_immediate_predecessor() {
        let req = wire_bulk_read(&[(0x42, 0, 2), (TEST_ID, 0, 2)]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .send_policy
            .staged_reply_for_test()
            .expect("BulkRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, Some(0x42));
    }

    #[test]
    fn fast_sync_read_slot_k_gt_zero_has_no_predecessor() {
        // Fast chains carry no per-slot Status headers → chain-fire is
        // absolute-deadline, not sequence-driven. `predecessor_id` must
        // stay None per `docs/dxl-streaming-rx.md` §5.2.
        let req = wire_fast_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, _| {});
        let ctx = bus
            .send_policy
            .staged_reply_for_test()
            .expect("FastSyncRead surfaces a reply context");
        assert_eq!(ctx.predecessor_id, None);
    }

    #[test]
    fn send_status_defers_wire_send_for_chain_k_gt_zero() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        assert!(
            state.sch.operations().is_empty(),
            "chain k > 0 does not arm a deadline",
        );
        assert!(
            state.tx_bus.operations().is_empty(),
            "wire send waits for SkipComplete",
        );
        assert_eq!(bus.send_policy.awaited_predecessor(), Some(0x42));
    }

    #[test]
    fn skip_complete_matching_predecessor_fires_start_now() {
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

    #[test]
    fn skip_complete_mismatched_predecessor_does_not_fire() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, state, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));

        let other_status = wire_status(0x05);
        stage_rx(&mut bus, &state, req.len() as u16, &other_status);
        bus.poll(|_, _, _| {});

        assert!(state.tx_bus.operations().is_empty());
        assert_eq!(
            bus.send_policy.awaited_predecessor(),
            Some(0x42),
            "still waiting for the immediate predecessor",
        );
    }

    #[test]
    fn cancel_clears_chain_pending() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|ev, _, reply| {
            if matches!(ev, Event::Crc(_)) {
                reply.send_status(empty_status()).expect("encode fits");
                reply.cancel();
            }
        });
        assert!(bus.send_policy.awaited_predecessor().is_none());
    }

    #[test]
    fn on_tx_complete_clears_chain_pending() {
        let req = wire_sync_read(0, 2, &[0x42, TEST_ID]);
        let (mut bus, _, _) = bus_seeded_with(&req);
        bus.poll(|_, _, reply| reply.send_status(empty_status()).expect("encode fits"));
        assert_eq!(bus.send_policy.awaited_predecessor(), Some(0x42));

        bus.on_tx_complete();
        assert!(bus.send_policy.awaited_predecessor().is_none());
    }

    #[test]
    fn resync_clears_chain_pending() {
        // Stage chain-pending directly — the alternative is a full chain
        // + corrupted-predecessor encode, far more setup for the same
        // observable.
        let (mut bus, state) = make_bus();
        bus.send_policy.defer_to_predecessor(0x42);
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        stage_rx(&mut bus, &state, 0, &bad);
        bus.poll(|_, _, _| {});
        assert!(bus.send_policy.awaited_predecessor().is_none());
    }

    #[test]
    fn instruction_header_clears_stale_chain_pending() {
        // Per `dxl-streaming-rx.md` §5.3: any stale `predecessor_id` from
        // a prior chain whose immediate predecessor went silent must
        // clear at the next instruction-header event, or a foreign
        // Status whose id happens to match would trigger a spurious
        // `start_now`.
        let (mut bus, state) = make_bus();
        bus.send_policy.defer_to_predecessor(0x42);
        let req = wire_ping(TEST_ID);
        stage_rx(&mut bus, &state, 0, &req);
        bus.poll(|_, _, _| {});
        assert!(
            bus.send_policy.awaited_predecessor().is_none(),
            "instruction-header event must reset stale chain-pending",
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
    fn on_fold_step_drains_raw_bytes_into_fold() {
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
        // FastLast armed via `send_slot(Last)` above. Fold's start_cursor
        // = fold_start_cursor = req.len(); stage predecessor bytes at
        // that wire-byte position so the drain's cursor matches
        // `start_cursor` on the first byte folded. SyncRead has 2 slots
        // → bytes_before for our slot = 12 wire bytes; stage fewer so
        // the FSM doesn't hit its busy-wait target this poll.
        let start = req.len() as u16;
        let predecessor = [0x11_u8, 0x22, 0x33, 0x44];
        stage_rx(&mut bus, &state, start, &predecessor);
        // RxDma.remaining = N − (start + len) so on_publish leaves
        // write_seq at start+len.
        let new_head = start.wrapping_add(predecessor.len() as u16);
        state
            .rx
            .stage_remaining((RX_BUF_LEN as u16).wrapping_sub(new_head));

        bus.codec.set_rx_read_seq_for_test(start);
        let before = bus.fast_last.bytes_folded();
        bus.on_fold_step();
        let after = bus.fast_last.bytes_folded();
        assert_eq!(after - before, predecessor.len() as u32);
    }

    #[test]
    fn on_tx_start_folds_residue_and_patches_crc() {
        // CC3 fire body folds the GUARD residue and patches the trailing
        // CRC. Stage `predecessor_bytes` worth so finalize lands on the
        // last drained byte; assert the TX buffer's trailing slot is no
        // longer the placeholder `[0x00, 0x00]`.
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
        // `FAST_SLOT_HEADER_BYTES(8) + body(4) = 12`; stage exactly
        // that many so finalize lands inside on_tx_start.
        let start = req.len() as u16;
        let predecessor = [
            0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC,
        ];
        stage_rx(&mut bus, &state, start, &predecessor);
        let new_head = start.wrapping_add(predecessor.len() as u16);
        state
            .rx
            .stage_remaining((RX_BUF_LEN as u16).wrapping_sub(new_head));
        bus.codec.set_rx_read_seq_for_test(start);

        let tx_len_before = bus.codec.tx_len() as usize;
        // SAFETY: tx_buf is initialized up to tx_len_before; reading by
        // raw pointer matches the production DMA1_CH4 view.
        let trailing_before = unsafe {
            core::slice::from_raw_parts(bus.codec.tx_buf_addr() as *const u8, tx_len_before)
        }[tx_len_before - 2..]
            .to_vec();
        assert_eq!(trailing_before, [0x00, 0x00]);

        bus.on_tx_start();
        assert_eq!(
            state.tx_bus.operations().last(),
            Some(&TxBusOp::HandleStart),
            "on_tx_start must call handle_start once",
        );

        let tx_len_after = bus.codec.tx_len() as usize;
        let trailing_after = unsafe {
            core::slice::from_raw_parts(bus.codec.tx_buf_addr() as *const u8, tx_len_after)
        }[tx_len_after - 2..]
            .to_vec();
        assert_ne!(
            trailing_after,
            [0x00, 0x00],
            "patch_crc should overwrite the placeholder slot"
        );
        assert!(!bus.fast_last.fold_active());
        assert_eq!(
            state.fl.patch_miss_count(),
            0,
            "finalize path must not bump the deadline-miss counter",
        );
    }

    #[test]
    fn on_tx_start_plateau_records_miss() {
        // Bytes-starved CC3 body must not hang. With no predecessor
        // bytes staged the plateau check (no progress between drain
        // passes) exits the loop; trailing CRC stays at the placeholder.
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
        // Pin write_seq == read_seq == start. NDTR = LEN - write_seq
        // keeps on_publish a no-op (ring_pos = LEN - remaining = start,
        // prev_pos = start → delta = 0). Setting remaining = LEN would
        // advance the publisher by LEN - start bytes of garbage and
        // mask the plateau backstop under test.
        let start = req.len() as u16;
        stage_rx(&mut bus, &state, start, &[]);
        bus.codec.set_rx_read_seq_for_test(start);
        state
            .rx
            .stage_remaining((RX_BUF_LEN as u16).wrapping_sub(start));

        bus.on_tx_start();
        assert!(bus.fast_last.fold_active(), "active stays set on bail");
        assert_eq!(
            state.fl.patch_miss_count(),
            1,
            "plateau-exit must bump the deadline-miss counter",
        );
    }

    #[test]
    fn on_tx_start_window_expiry_records_miss() {
        // CH4 prefetch reached the trailing CRC slot before finalize
        // landed. The expired-window check exits the loop and bumps
        // `crc_patch_deadline_miss`.
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
        state.fl.stage_patch_window_expired(true);

        bus.on_tx_start();
        assert!(
            bus.fast_last.fold_active(),
            "active stays set on expired-window exit",
        );
        assert_eq!(
            state.fl.patch_miss_count(),
            1,
            "expired-window exit must bump the deadline-miss counter",
        );
    }
}
