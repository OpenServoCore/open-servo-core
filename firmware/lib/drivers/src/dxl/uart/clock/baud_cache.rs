//! Per-baud timing cache + staged-baud mailbox for the DXL transport.
//! Owns the authoritative `ticks_per_bit` (one UART bit in TIM2/HCLK ticks)
//! plus the Q16 wire byte-time and the RX edge-stamp compensation, all
//! recomputed only when the baud actually changes. Baud is staged by
//! control-table writes via [`Self::stage_baud`] and committed at
//! `on_tx_complete` (USART can't change BRR mid-frame) via
//! [`Self::on_tx_complete`].

use osc_core::BaudRate;

use super::divisor_for;
use crate::dxl::uart::BITS_PER_FRAME;
use crate::traits::dxl::UsartBaud;

pub struct BaudCache<U: UsartBaud> {
    usart: U,
    baud: BaudRate,
    ticks_per_bit: u16,
    /// Ideal wire byte-time in Q16 HCLK ticks (`BITS_PER_FRAME × CLOCK_HZ ×
    /// 2^16 / baud_hz`, ceil), cached per baud so [`Self::bytes_to_ticks`]
    /// stays divide-free on the reply-schedule hot path. Q16 (vs an integer
    /// tick count) preserves the exact truncation of the prior end-to-end
    /// `bytes × BITS × CLOCK / baud` formula at fractional bauds — an
    /// integer per-byte cache at 57600 would drift by 1 tick every 3
    /// bytes (`floor(8333.33) × 3 = 24999` vs true 25000) and blow the
    /// Fast Last CRC patch deadline. Ceil-rounded so per-byte error stays
    /// non-negative and bounded well past any packet size.
    ticks_per_byte_q16: u32,
    /// Per-baud RX edge-stamp compensation, in HCLK ticks. Refreshed from
    /// `U::rx_edge_comp_ticks(baud)` at `new` and on every committed baud
    /// change. Codec reads it via [`Self::rx_edge_comp_ticks`] once per poll
    /// and threads through to the edge walker, which subtracts it from every IC
    /// stamp at read-from-ring time so anchors, pairs, and `packet_end_tick`
    /// all live in wire-edge time.
    rx_edge_comp_ticks: u16,
    pending_baud: Option<BaudRate>,
}

impl<U: UsartBaud> BaudCache<U> {
    /// `ticks_per_bit` at the given baud and the reference clock rate. Each
    /// arm folds to a literal at monomorphization — no runtime divide
    /// (matters on RV32EC, which has no hardware `div`).
    #[inline]
    fn ticks_per_bit_at(baud: BaudRate) -> u16 {
        match baud {
            BaudRate::B9600 => const { divisor_for(U::CLOCK_HZ, BaudRate::B9600.as_hz()) as u16 },
            BaudRate::B57600 => const { divisor_for(U::CLOCK_HZ, BaudRate::B57600.as_hz()) as u16 },
            BaudRate::B115200 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B115200.as_hz()) as u16 }
            }
            BaudRate::B1000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B1000000.as_hz()) as u16 }
            }
            BaudRate::B2000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B2000000.as_hz()) as u16 }
            }
            BaudRate::B3000000 => {
                const { divisor_for(U::CLOCK_HZ, BaudRate::B3000000.as_hz()) as u16 }
            }
        }
    }

    /// Q16 wire byte-time per baud. Each arm's u64 divide happens at
    /// monomorphization (const-folded like [`Self::ticks_per_bit_at`]) so
    /// runtime stays divide-free on RV32EC+Zmmul. Exact-integer arms (9600,
    /// 1M, 2M, 3M) get zero rounding error; 57600 / 115200 ceil-round so
    /// per-byte error stays non-negative and `floor(bytes × M_q16 / 2^16)`
    /// matches the prior `bytes × BITS × CLOCK / baud` end-to-end
    /// truncation for any realistic packet size.
    #[inline]
    fn ticks_per_byte_at(baud: BaudRate) -> u32 {
        const fn per_byte_q16(clock_hz: u32, baud_hz: u32) -> u32 {
            let num = BITS_PER_FRAME as u64 * clock_hz as u64 * (1u64 << 16);
            num.div_ceil(baud_hz as u64) as u32
        }
        match baud {
            BaudRate::B9600 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B9600.as_hz()) },
            BaudRate::B57600 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B57600.as_hz()) },
            BaudRate::B115200 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B115200.as_hz()) },
            BaudRate::B1000000 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B1000000.as_hz()) },
            BaudRate::B2000000 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B2000000.as_hz()) },
            BaudRate::B3000000 => const { per_byte_q16(U::CLOCK_HZ, BaudRate::B3000000.as_hz()) },
        }
    }

    pub fn new(baud: BaudRate, usart: U) -> Self {
        let mut s = Self {
            usart,
            baud,
            ticks_per_bit: 0,
            ticks_per_byte_q16: 0,
            rx_edge_comp_ticks: 0,
            pending_baud: None,
        };
        s.apply_baud_change(baud);
        s
    }

    /// Refresh every per-baud cache from `baud`. Shared by the constructor
    /// and [`Self::on_tx_complete`] so the two never drift apart. Does
    /// not touch the USART — the constructor assumes the peripheral is
    /// already at `baud`, and the commit path drives `apply_baud` itself
    /// before calling this.
    fn apply_baud_change(&mut self, baud: BaudRate) {
        self.baud = baud;
        self.ticks_per_bit = Self::ticks_per_bit_at(baud);
        self.ticks_per_byte_q16 = Self::ticks_per_byte_at(baud);
        self.rx_edge_comp_ticks = self.usart.rx_edge_comp_ticks(baud);
    }

    // -- events -----------------------------------------------------------------

    /// Commit any staged baud to the USART and refresh the caches. Returns
    /// the new `ticks_per_bit` when a change was applied (so the composite
    /// can fan the baud change out to the drift integrator), `None` when
    /// nothing was staged. BRR can't change mid-frame, so the composite
    /// calls this at `on_tx_complete` — the first USART-idle point after our
    /// own TX.
    pub fn on_tx_complete(&mut self) -> Option<u16> {
        let baud = self.pending_baud.take()?;
        self.usart.apply_baud(baud);
        self.apply_baud_change(baud);
        Some(self.ticks_per_bit)
    }

    // -- commands ---------------------------------------------------------------

    pub fn stage_baud(&mut self, baud: BaudRate) {
        if baud != self.baud {
            self.pending_baud = Some(baud);
        }
    }

    // -- accessors --------------------------------------------------------------

    #[inline(always)]
    pub fn baud(&self) -> BaudRate {
        self.baud
    }

    #[inline(always)]
    pub fn ticks_per_bit(&self) -> u16 {
        self.ticks_per_bit
    }

    /// One wire byte's duration in HCLK ticks at the current baud
    /// (`ticks_per_bit × BITS_PER_FRAME`). u16 to match `FastLastSchedule`'s
    /// grid fields; the widest value (9600 baud → 50_000) stays under
    /// `u16::MAX`.
    #[inline(always)]
    pub fn byte_ticks(&self) -> u16 {
        self.ticks_per_bit.wrapping_mul(BITS_PER_FRAME)
    }

    /// Per-baud RX edge-stamp compensation in HCLK ticks. Codec reads once
    /// per poll and threads to the edge walker so anchors, pairs, and
    /// `packet_end_tick` live in wire-edge time. See the field doc.
    #[inline(always)]
    pub fn rx_edge_comp_ticks(&self) -> u16 {
        self.rx_edge_comp_ticks
    }

    /// Wire-byte duration in monotonic timer ticks at the current baud.
    /// Used by the composite's slot-offset math:
    /// `delay_ticks = rdt_us · TICKS_PER_US + bytes_to_ticks(offset)`.
    ///
    /// `u32 × u32 → u64` multiply + `>> 16` — no divide. Q16 factor caches
    /// per-baud via [`Self::ticks_per_byte_at`] at every baud change (cold
    /// path); the u64 divide folds at monomorphization so `__udivdi3`
    /// never appears on this hot path. Exact at 9600/1M/2M/3M; matches
    /// the prior end-to-end truncation semantics at 57600/115200 too
    /// (per-byte error bounded well past any packet size).
    pub fn bytes_to_ticks(&self, bytes: u32) -> u32 {
        ((bytes as u64 * self.ticks_per_byte_q16 as u64) >> 16) as u32
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::dxl::uart::test_support::{UsartBaudState, mk_usart_baud};
    use crate::mocks::MockUsartBaud;

    type TestCache = BaudCache<MockUsartBaud>;

    fn make(baud: BaudRate) -> (TestCache, UsartBaudState) {
        let (usart, u_state) = mk_usart_baud();
        (BaudCache::new(baud, usart), u_state)
    }

    fn cache_at(baud: BaudRate) -> TestCache {
        make(baud).0
    }

    #[test]
    fn new_computes_ticks_per_bit_from_baud() {
        assert_eq!(cache_at(BaudRate::B3000000).ticks_per_bit(), 16);
        assert_eq!(cache_at(BaudRate::B1000000).ticks_per_bit(), 48);
        assert_eq!(cache_at(BaudRate::B9600).ticks_per_bit(), 5000);
    }

    #[test]
    fn stage_baud_records_pending_change() {
        let mut c = cache_at(BaudRate::B9600);
        c.stage_baud(BaudRate::B3000000);
        assert_eq!(c.pending_baud, Some(BaudRate::B3000000));
    }

    #[test]
    fn stage_baud_is_noop_for_same_baud() {
        let mut c = cache_at(BaudRate::B9600);
        c.stage_baud(BaudRate::B9600);
        assert_eq!(c.pending_baud, None);
    }

    #[test]
    fn on_tx_complete_applies_to_usart_and_returns_new_tpb() {
        let (mut c, u_state) = make(BaudRate::B9600);
        c.stage_baud(BaudRate::B3000000);
        assert_eq!(c.on_tx_complete(), Some(16));
        assert_eq!(u_state.apply_baud_log(), alloc::vec![BaudRate::B3000000]);
        assert_eq!(c.ticks_per_bit(), 16);
        assert_eq!(c.baud(), BaudRate::B3000000);
    }

    #[test]
    fn on_tx_complete_is_noop_with_nothing_staged() {
        let (mut c, u_state) = make(BaudRate::B9600);
        assert_eq!(c.on_tx_complete(), None);
        assert!(u_state.apply_baud_log().is_empty());
    }

    #[test]
    fn bytes_to_ticks_at_1m_is_480_per_byte() {
        // 10 bits · 48 tpb = 480 ticks / byte.
        let c = cache_at(BaudRate::B1000000);
        assert_eq!(c.bytes_to_ticks(1), 480);
        assert_eq!(c.bytes_to_ticks(5), 2400);
    }

    #[test]
    fn bytes_to_ticks_at_3m_is_160_per_byte() {
        let c = cache_at(BaudRate::B3000000);
        assert_eq!(c.bytes_to_ticks(1), 160);
        assert_eq!(c.bytes_to_ticks(3), 480);
    }

    #[test]
    fn bytes_to_ticks_at_57600_matches_end_to_end_truncation() {
        // The Q16 cache's charter (see the `ticks_per_byte_q16` field doc):
        // at the fractional 57600 baud, `floor(bytes × M_q16 >> 16)` must
        // equal the prior end-to-end `bytes × BITS × CLOCK / baud`
        // truncation for every realistic packet size — an integer per-byte
        // cache drifts 1 tick every 3 bytes and blows the Fast Last CRC
        // patch deadline. Pin the whole TX-buffer-sized range.
        let c = cache_at(BaudRate::B57600);
        for bytes in 1..=140u32 {
            let exact = (bytes as u64 * BITS_PER_FRAME as u64 * 48_000_000 / 57_600) as u32;
            assert_eq!(c.bytes_to_ticks(bytes), exact, "bytes = {bytes}");
        }
        // The doc's example: integer cache would give 24_999 here.
        assert_eq!(c.bytes_to_ticks(3), 25_000);
    }

    #[test]
    fn on_tx_complete_refreshes_rx_edge_comp() {
        // Per-baud edge-stamp compensation must track the committed baud,
        // not the construction-time one — the codec re-reads it after
        // every applied change.
        let mut usart = MockUsartBaud::new();
        usart.expect_apply_baud().returning_st(|_| ());
        usart
            .expect_rx_edge_comp_ticks()
            .returning_st(|baud| match baud {
                BaudRate::B3000000 => 7,
                _ => 3,
            });
        let mut c = BaudCache::new(BaudRate::B9600, usart);
        assert_eq!(c.rx_edge_comp_ticks(), 3);
        c.stage_baud(BaudRate::B3000000);
        c.on_tx_complete();
        assert_eq!(c.rx_edge_comp_ticks(), 7);
    }
}
