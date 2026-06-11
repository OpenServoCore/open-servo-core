//! Top-level DXL bus driver. Composes the RX timing path and the bit-rate
//! clock so the chip-side ISR layer only ever reaches through `DxlBus` —
//! cross-sub-driver routing (e.g. RX classifier needs `ticks_per_bit` from
//! the clock) lives in the composite per driver-pattern §4.2.
//!
//! Generic over its leaf providers; `firmware/ch32/src/runtime/registry.rs`
//! binds them to V006 implementations. Future sub-drivers (`DxlTx`,
//! `DxlChainCatchup`) land as additional fields.

pub mod clock;
pub mod rx;

use crate::traits::{ClockTrim, DmaRing, UsartBaud};
use clock::DxlClock;
use rx::DxlRx;

pub struct DxlBus<U: UsartBaud, T: ClockTrim, R: DmaRing> {
    rx: DxlRx<R>,
    clock: DxlClock<U, T>,
}

impl<U: UsartBaud, T: ClockTrim, R: DmaRing> DxlBus<U, T, R> {
    pub const fn new(rx: DxlRx<R>, clock: DxlClock<U, T>) -> Self {
        Self { rx, clock }
    }

    /// New RX falling-edge timestamps may be available — pull the current
    /// `ticks_per_bit` from the clock and forward to the RX classifier.
    pub fn on_rx_edge_advance(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.rx.on_edge_advance(ticks_per_bit);
    }

    /// RX wire went idle — drain any tail edges the classifier hasn't seen
    /// yet, then reset the anchor so the next burst re-seeds.
    pub fn on_rx_idle(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.rx.on_idle(ticks_per_bit);
    }

    /// Sequence number of the next BT slot to write — one past the last
    /// published BT entry.
    #[allow(dead_code)]
    pub fn byte_ts_head(&self) -> u16 {
        self.rx.byte_ts_head()
    }

    /// Stable peripheral-memory address for DMA1_CH7's destination buffer.
    /// Bringup hands this to `dma::configure(CH7, ...)`.
    pub fn edges_addr(&self) -> u32 {
        self.rx.edges_addr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeClockTrim, FakeDmaRing, FakeUsartBaud};
    use crate::traits::DmaFlags;
    use osc_core::BaudRate;

    fn make_rx(vals: &[u16], flags: DmaFlags) -> DxlRx<FakeDmaRing> {
        let mut rx = DxlRx::new(FakeDmaRing::default());
        rx.stage_edges_for_test(vals);
        rx.arm_next_flags_for_test(flags);
        rx
    }

    fn make_clock(baud: BaudRate) -> DxlClock<FakeUsartBaud, FakeClockTrim> {
        DxlClock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_3m() {
        // At 3M tpb=16 → byte=160 ticks; the 160-tick delta is a window HIT.
        // If the composite forwarded a wrong tpb (e.g. 48 from a 1M clock),
        // the same delta would fall outside [9·48, 11·48] and classify as
        // SKIP — byte_ts_head would stay at 1 instead of advancing to 2.
        let rx = make_rx(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = DxlBus::new(rx, make_clock(BaudRate::B3000000));

        bus.on_rx_edge_advance();

        assert_eq!(bus.byte_ts_head(), 2);
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_1m() {
        // Mirror of the 3M case: at tpb=48 the byte-time is 480 ticks. Two
        // edges 480 apart land as a HIT; if tpb were misrouted to 16, the
        // 480-tick delta would classify as GAP+re-anchor but still advance
        // — so we assert the HIT/SKIP boundary instead: two edges 160 apart
        // are HIT at 3M, SKIP at 1M.
        let rx = make_rx(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = DxlBus::new(rx, make_clock(BaudRate::B1000000));

        bus.on_rx_edge_advance();

        // SEED at 1000, then SKIP (intra-byte) — head stays at 1.
        assert_eq!(bus.byte_ts_head(), 1);
    }

    #[test]
    fn on_rx_idle_walks_tail_edges() {
        // IDLE backstop walks edges the HT/TC ISR hasn't drained (small
        // packets that never trip HT). Single tail edge → SEED.
        let rx = make_rx(&[500], DmaFlags::default());
        let mut bus = DxlBus::new(rx, make_clock(BaudRate::B3000000));

        bus.on_rx_idle();

        assert_eq!(bus.byte_ts_head(), 1);
    }
}
