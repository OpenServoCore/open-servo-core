//! Fast Last CRC fold scheduler. Owns the periodic-walk grid that drives
//! classifier + parser + fold work during a Fast Sync / Bulk Read
//! predecessor window, and the final-step busy-wait that brings
//! `bytes_walked` up to `predecessor_bytes − GUARD` before the TX-start
//! body folds the residue inline.
//!
//! The FSM lives here; the CRC engine and per-byte fold callback live on
//! the [`DxlUart`] composite (next commit, alongside `crc.rs`). The
//! driver works entirely in `(packet_end_tick, offset_ticks)` pairs — a
//! wire-clock anchor + protocol-derived offsets. Lifting the anchor into
//! the scheduler's tick domain is the chip-side provider's job; the
//! driver itself stays clock-agnostic.
//!
//! [`DxlUart`]: super::DxlUart

use crate::traits::dxl::FastLastScheduler;

/// Where in the fold pipeline we currently sit.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FastLastPhase {
    /// No reply in flight.
    Idle,
    /// CMP-match-driven grid: one body per `BYTES_PER_INTERVAL` step from
    /// `t_prior_start` to `final_anchor_offset`. The final body runs the
    /// busy-wait fold loop, then returns to [`Idle`] (the TX-start body
    /// owns the post-start residue).
    ///
    /// [`Idle`]: FastLastPhase::Idle
    PeriodicWalk,
}

/// Input to [`FastLast::start`]. `packet_end_tick` is parser-derived in
/// the wire-clock domain (u16); all timing math derives from it.
#[derive(Copy, Clone, Debug)]
pub struct FastLastSchedule {
    /// Parser-derived wire-clock value where the host's request ended
    /// (= `BT[last_byte] + 10·tpb`). Scheduler will lift this into its
    /// own tick domain at `set_deadline` time.
    pub packet_end_tick: u16,
    /// Return-delay-time, in scheduler ticks.
    pub rdt_ticks: u16,
    /// One wire byte time at the active baud, in scheduler ticks.
    pub byte_ticks: u16,
    /// Count of wire bytes the host's request will pull from servos with
    /// earlier slots than ours, before our reply slot. The busy-wait's
    /// `predecessor_bytes − GUARD` target.
    pub predecessor_bytes: u16,
}

/// FSM + grid math for the Fast Last fold pipeline.
///
/// Generic over its scheduler provider `S`. The driver works in offsets
/// from `packet_end_tick`; the scheduler owns all scheduling-domain
/// tick state internally.
pub struct FastLast<S: FastLastScheduler> {
    scheduler: S,
    phase: FastLastPhase,
    /// Wall-clock offset (from packet_end) of the body currently armed on
    /// the scheduler. Body bumps this by `interval_ticks` before re-arming
    /// — drift in any one body's ISR-entry latency doesn't propagate.
    next_anchor_offset: u32,
    /// Wall-clock offset (from packet_end) of the terminal (busy-wait)
    /// body. Set once at `start`; on equality with `next_anchor_offset`
    /// the body switches from "advance grid" to "busy-wait and return".
    final_anchor_offset: u32,
    /// `BYTES_PER_INTERVAL · byte_ticks` — grid step in scheduler ticks.
    /// Cached at `start` for fast re-arming.
    interval_ticks: u32,
    /// Cached `predecessor_bytes` for the busy-wait's
    /// `bytes_walked >= predecessor_bytes − GUARD` branch.
    predecessor_bytes: u16,
}

impl<S: FastLastScheduler> FastLast<S> {
    pub const fn new(scheduler: S) -> Self {
        Self {
            scheduler,
            phase: FastLastPhase::Idle,
            next_anchor_offset: 0,
            final_anchor_offset: 0,
            interval_ticks: 0,
            predecessor_bytes: 0,
        }
    }

    /// Compute the periodic-walk grid, stage the deadline, arm the first
    /// CMP, and flip the FSM into [`FastLastPhase::PeriodicWalk`].
    pub fn start(&mut self, p: FastLastSchedule) {
        let byte_ticks = p.byte_ticks as u32;
        let rdt_ticks = p.rdt_ticks as u32;
        let predecessor_bytes_u32 = p.predecessor_bytes as u32;
        let interval = S::BYTES_PER_INTERVAL as u32 * byte_ticks;
        let t_guard = S::GUARD_BYTES as u32 * byte_ticks;

        let t_prior_start_offset = rdt_ticks.wrapping_add(byte_ticks);
        let t_prior_end_offset = rdt_ticks.wrapping_add(predecessor_bytes_u32 * byte_ticks);
        let t_prior_duration = p.predecessor_bytes.saturating_sub(1) as u32 * byte_ticks;
        // Our reply must land at predecessor_end (no inter-slot gap).
        let fire_offset = t_prior_end_offset;

        let deadline_offset = t_prior_end_offset.wrapping_sub(t_guard);
        let final_anchor_offset = if p.predecessor_bytes > S::GUARD_BYTES {
            fire_offset
                .wrapping_sub(core::cmp::min(interval, t_prior_duration))
                .wrapping_sub(t_guard)
        } else {
            fire_offset
        };

        // Step back by `interval` until ≤ one step from t_prior_start_offset.
        // The i32 cast handles the small-predecessor case where
        // `final_anchor_offset` < `t_prior_start_offset` modular-wraps —
        // without it the loop would spin ~4G times.
        let mut anchor = final_anchor_offset;
        while (anchor.wrapping_sub(t_prior_start_offset) as i32) >= interval as i32 {
            anchor = anchor.wrapping_sub(interval);
        }
        self.next_anchor_offset = anchor;
        self.final_anchor_offset = final_anchor_offset;
        self.interval_ticks = interval;
        self.predecessor_bytes = p.predecessor_bytes;
        self.phase = FastLastPhase::PeriodicWalk;

        self.scheduler
            .set_deadline(p.packet_end_tick, deadline_offset);
        // EVERY grid CMP back-dated by FAST_LAST_ENTRY_TICKS.
        self.scheduler
            .schedule(anchor.wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32));
    }

    /// Drive one grid body. Composite calls this from its SysTick demux
    /// when our CMP fires.
    ///
    /// - `walker` runs the classifier + parser-drain + per-byte fold for
    ///   whatever bytes have landed since the last body. Called at least
    ///   once per invocation.
    /// - `bytes_walked` returns the composite-side count of predecessor
    ///   bytes folded so far. The busy-wait exits when this reaches
    ///   `predecessor_bytes − GUARD`.
    pub fn on_step<W, B>(&mut self, mut walker: W, mut bytes_walked: B)
    where
        W: FnMut(),
        B: FnMut() -> u32,
    {
        match self.phase {
            FastLastPhase::PeriodicWalk => {
                walker();
                if self.next_anchor_offset == self.final_anchor_offset {
                    // Final anchor — busy-wait. Uncontested at high baud
                    // because the composite paused DMA1_CH7 HT/TC at start
                    // time, so this body is the sole HIGH consumer in the
                    // window. Exit when `bytes_walked >= predecessor_bytes −
                    // GUARD` OR scheduler reports deadline passed; the
                    // remaining GUARD bytes are absorbed by the TX-start
                    // body's tail fold.
                    let target =
                        (self.predecessor_bytes as u32).saturating_sub(S::GUARD_BYTES as u32);
                    loop {
                        walker();
                        if bytes_walked() >= target {
                            break;
                        }
                        if self.scheduler.deadline_passed() {
                            break;
                        }
                    }
                    self.phase = FastLastPhase::Idle;
                    self.scheduler.cancel();
                    return;
                }
                self.next_anchor_offset = self.next_anchor_offset.wrapping_add(self.interval_ticks);
                self.scheduler.schedule(
                    self.next_anchor_offset
                        .wrapping_sub(S::FAST_LAST_ENTRY_TICKS as u32),
                );
            }
            FastLastPhase::Idle => {
                // Spurious CMP entry — defensive cancel.
                self.scheduler.cancel();
            }
        }
    }

    /// Drop any pending CMP and return to [`FastLastPhase::Idle`]. Idempotent.
    pub fn cancel(&mut self) {
        self.scheduler.cancel();
        self.phase = FastLastPhase::Idle;
    }

    pub fn is_active(&self) -> bool {
        !matches!(self.phase, FastLastPhase::Idle)
    }

    pub fn phase(&self) -> FastLastPhase {
        self.phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeFastLastScheduler, FastLastSchedulerOp};
    use core::cell::Cell;

    /// FAST_LAST_ENTRY_TICKS matching the mock — the back-date applied to
    /// every grid CMP.
    const ENTRY: u32 = 240;
    /// BYTES_PER_INTERVAL · byte_time at the test baud (3M: 15·160 = 2400).
    const INTERVAL_3M: u32 = 2400;
    /// 3M byte_time (10·tpb at HCLK=48M / 3MHz).
    const BYTE_TICKS_3M: u16 = 160;
    /// 3M byte_time as u32 for arithmetic on offsets.
    const BYTE_TICKS_3M_U32: u32 = BYTE_TICKS_3M as u32;

    fn fast_last() -> FastLast<FakeFastLastScheduler> {
        FastLast::new(FakeFastLastScheduler::default())
    }

    #[test]
    fn start_stages_deadline_and_first_cmp_at_anchor_minus_entry() {
        // predecessor_bytes=2 with rdt=12000 → fire_offset = 12000 + 2·160 =
        // 12320. t_prior_duration = 1·160 = 160. final_anchor_offset =
        // 12320 − min(2400, 160) − 160 = 12320 − 160 − 160 = 12000. Step-
        // back doesn't trigger (anchor − t_prior_start = 12000 − 12160 < 0).
        // deadline_offset = t_prior_end − GUARD·byte = 12320 − 160 = 12160.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 2000,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
        });

        assert_eq!(d.next_anchor_offset, 12000);
        assert_eq!(d.final_anchor_offset, 12000);
        assert_eq!(
            d.scheduler.log.as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline {
                    packet_end_tick: 2000,
                    deadline_ticks: 12160,
                },
                FastLastSchedulerOp::Schedule {
                    offset_ticks: 12000 - ENTRY,
                },
            ]
        );
    }

    #[test]
    fn start_step_back_handles_small_predecessor_modular_wrap() {
        // rdt=0, predecessor_bytes=2: t_prior_start = 160, t_prior_end =
        // 320, t_prior_duration = 160, final_anchor_offset = 320 − 160 −
        // 160 = 0. Step-back: (0 − 160 as i32) = -160 < INTERVAL → loop
        // doesn't execute. Without i32 cast the diff is ~4G ≥ 2400 → spin.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
        });

        assert_eq!(d.next_anchor_offset, 0);
        // SetDeadline + exactly one Schedule — step-back didn't spin.
        assert_eq!(d.scheduler.log.len(), 2);
    }

    #[test]
    fn intermediate_step_advances_grid_by_interval() {
        // predecessor_bytes=50 spans three anchors. final_anchor_offset =
        // 12000 + 50·160 − min(2400, 49·160) − 160 = 20000 − 2400 − 160 =
        // 17440. Step back twice: 17440 − 2400 = 15040 (15040 − 12160 =
        // 2880 ≥ 2400, step); 15040 − 2400 = 12640 (12640 − 12160 = 480 <
        // 2400, stop). next_anchor_offset = 12640.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 50,
        });
        assert_eq!(d.next_anchor_offset, 12640);
        assert_eq!(d.final_anchor_offset, 17440);

        d.on_step(|| {}, || 0);
        d.on_step(|| {}, || 0);

        assert_eq!(
            d.scheduler.log.as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline {
                    packet_end_tick: 0,
                    deadline_ticks: 12000 + 50 * BYTE_TICKS_3M_U32 - BYTE_TICKS_3M_U32,
                },
                FastLastSchedulerOp::Schedule {
                    offset_ticks: 12640 - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    offset_ticks: 15040 - ENTRY,
                },
                FastLastSchedulerOp::Schedule {
                    offset_ticks: 17440 - ENTRY,
                },
            ]
        );
        assert_eq!(d.phase(), FastLastPhase::PeriodicWalk);
        assert_eq!(d.next_anchor_offset, 17440);
    }

    #[test]
    fn final_anchor_busy_waits_until_bytes_walked_target() {
        // predecessor_bytes=5 with GUARD=1 → target=4. Pre-loop walker →
        // bytes=1; three loop iters bring bytes to 4 → break. Total walker
        // calls = 4. deadline_passed stays false the entire time.
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 5,
        });
        assert_eq!(d.next_anchor_offset, d.final_anchor_offset);

        let walker_calls = Cell::new(0u32);
        let bytes = Cell::new(0u32);
        d.on_step(
            || {
                walker_calls.set(walker_calls.get() + 1);
                bytes.set(bytes.get() + 1);
            },
            || bytes.get(),
        );

        assert_eq!(walker_calls.get(), 4);
        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(matches!(
            d.scheduler.log.last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    #[test]
    fn final_anchor_busy_waits_until_deadline_passed_when_bytes_lag() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 3,
        });
        // Drive the deadline-passed branch on the first inner-loop check.
        d.scheduler.deadline_passed_value.set(true);

        let walker_calls = Cell::new(0u32);
        d.on_step(
            || walker_calls.set(walker_calls.get() + 1),
            || 0_u32, // bytes never arrive
        );

        // Pre-loop walker + one inside the loop before the deadline branch
        // catches: 2 calls.
        assert_eq!(walker_calls.get(), 2);
        assert_eq!(d.phase(), FastLastPhase::Idle);
    }

    #[test]
    fn final_anchor_returns_to_idle_and_cancels_after_busy_wait() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 0,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 2,
        });
        let bytes = Cell::new(0u32);
        d.on_step(|| bytes.set(bytes.get() + 1), || bytes.get());

        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(!d.is_active());
        // Log: SetDeadline + Schedule (from start) then Cancel (from on_step exit).
        assert_eq!(d.scheduler.log.len(), 3);
        assert!(matches!(d.scheduler.log[2], FastLastSchedulerOp::Cancel));
    }

    #[test]
    fn cancel_from_periodic_walk_returns_to_idle() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 50,
        });
        assert_eq!(d.phase(), FastLastPhase::PeriodicWalk);

        d.cancel();

        assert_eq!(d.phase(), FastLastPhase::Idle);
        assert!(matches!(
            d.scheduler.log.last(),
            Some(FastLastSchedulerOp::Cancel)
        ));
    }

    /// Bench-traced regression at 3M GUARD=1 predecessor_bytes=14 — pins
    /// the grid math against a known-good trace. Single-step grid case
    /// (`t_prior_duration < interval`): `final_anchor_offset` and the
    /// first anchor land at 12000, scheduled CMP is `anchor − ENTRY`,
    /// deadline_offset = `t_prior_end − GUARD · byte` = 14080.
    #[test]
    fn deadline_grid_correctness_3m_predecessor_bytes_14() {
        let mut d = fast_last();
        d.start(FastLastSchedule {
            packet_end_tick: 0,
            rdt_ticks: 12000,
            byte_ticks: BYTE_TICKS_3M,
            predecessor_bytes: 14,
        });

        assert_eq!(d.final_anchor_offset, 12000);
        assert_eq!(d.next_anchor_offset, 12000);
        assert_eq!(d.interval_ticks, INTERVAL_3M);
        assert_eq!(
            d.scheduler.log.as_slice(),
            &[
                FastLastSchedulerOp::SetDeadline {
                    packet_end_tick: 0,
                    deadline_ticks: 14080,
                },
                FastLastSchedulerOp::Schedule {
                    offset_ticks: 12000 - ENTRY,
                },
            ]
        );
    }
}
