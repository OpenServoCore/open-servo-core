//! DXL-over-UART transport. Composes the RX timing path and the bit-rate
//! clock so the chip-side ISR layer only ever reaches through `DxlUart` —
//! cross-sub-driver routing (e.g. RX classifier needs `ticks_per_bit` from
//! the clock) lives in the composite per driver-pattern §4.2.
//!
//! Generic over its leaf providers AND its three storage sizes
//! ([`DxlUart`] doc). `firmware/ch32/src/runtime/registry.rs` binds each
//! to its V006 value. Future sub-drivers (`Tx`, `ChainCatchup`) land as
//! additional fields.

pub mod clock;
pub mod rx;

use core::cell::SyncUnsafeCell;

use dxl_protocol::CrcUmts;
use dxl_protocol::decoder::Decoder;
use osc_core::{BaudRate, BootMode};

use crate::traits::{ClockTrim, DmaRing, UsartBaud};
use clock::Clock;
use rx::Rx;

/// The DXL bus composite. The type parameters together describe everything
/// this driver wants the chip side to supply — what peripherals to drive
/// it with, what CRC to use, and how much memory each of its three rings
/// gets:
///
/// - `U`: USART baud-rate setter (sub-driver `Clock`).
/// - `T`: HSI / HSE trim setter (sub-driver `Clock`).
/// - `R`: DMA-ring ISR surface for DMA1_CH7 (sub-driver `Rx`).
/// - `CRC`: CRC-16/UMTS engine — software impl on V006, peripheral impl
///   on a chip that has one (see `providers::dxl_crc`).
/// - `DECODER_CAP`: streaming-decoder accumulator size. Sized to hold the
///   longest unstuffed frame the dispatcher will encounter (typically 256
///   covers max-RW + header + margin); decoupled from the on-wire RX byte
///   ring because the parser drains continuously per doc §8.1.
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per doc §8.1).
///   Also drives the BT ring depth inside `Rx` — doc §8.3 requires
///   they match so byte index `i` in RX maps to `BT[i mod RX_BUF_LEN]`,
///   and the composite enforces that coupling by construction.
/// - `EDGE_BUF_LEN`: DMA1_CH7 edge-timestamp ring depth (typically 128 /
///   option A in doc §8.4; 64 / option B trades CPU for memory).
pub struct DxlUart<
    U: UsartBaud,
    T: ClockTrim,
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
> {
    rx: Rx<R, EDGE_BUF_LEN, RX_BUF_LEN>,
    clock: Clock<U, T>,

    // Lights up in the upcoming `poll()` landing; declared now so the type
    // parameter `CRC` is anchored and the chip-side registry can pin
    // `providers::dxl_crc::DxlCrc` once for the whole codec migration.
    #[allow(dead_code)]
    decoder: Decoder<DECODER_CAP, CRC>,
    /// DMA1_CH5 destination for received bytes. `SyncUnsafeCell` because
    /// USART1's DMA writes it concurrently with the parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware.
    rx_buf: SyncUnsafeCell<[u8; RX_BUF_LEN]>,

    id: u8,
    rdt_us: u32,

    pending_id: Option<u8>,
    pending_rdt_us: Option<u32>,
    pending_reboot: Option<BootMode>,
}

impl<
    U: UsartBaud,
    T: ClockTrim,
    R: DmaRing,
    CRC: CrcUmts,
    const DECODER_CAP: usize,
    const RX_BUF_LEN: usize,
    const EDGE_BUF_LEN: usize,
> DxlUart<U, T, R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>
{
    pub fn new(
        rx: Rx<R, EDGE_BUF_LEN, RX_BUF_LEN>,
        clock: Clock<U, T>,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        Self {
            rx,
            clock,
            decoder: Decoder::new(),
            rx_buf: SyncUnsafeCell::new([0; RX_BUF_LEN]),
            id,
            rdt_us,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
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

    /// USART1 TC fired — the reply has fully drained the wire. Drain
    /// staged config writes (id / baud / trim / rdt) and surface any
    /// pending reboot to the chip-side ISR. Reboot is returned (rather
    /// than self-applied) because the chip controls how the reset
    /// actually happens; the driver only knows it was asked.
    pub fn on_tx_complete(&mut self) -> Option<BootMode> {
        if let Some(id) = self.pending_id.take() {
            self.id = id;
        }
        if let Some(rdt) = self.pending_rdt_us.take() {
            self.rdt_us = rdt;
        }
        self.clock.on_tx_complete();
        self.pending_reboot.take()
    }

    pub fn stage_id(&mut self, id: u8) {
        if id != self.id {
            self.pending_id = Some(id);
        }
    }

    pub fn stage_baud(&mut self, baud: BaudRate) {
        self.clock.stage_baud(baud);
    }

    pub fn stage_rdt(&mut self, us: u32) {
        if us != self.rdt_us {
            self.pending_rdt_us = Some(us);
        }
    }

    pub fn stage_reboot(&mut self, mode: BootMode) {
        self.pending_reboot = Some(mode);
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

    /// Stable peripheral-memory address for DMA1_CH5's destination buffer.
    /// Bringup hands this to `dma::configure(CH5, ...)` so the USART byte
    /// stream lands directly in driver-owned storage (replaces the legacy
    /// `DXL_RX_BUF` static).
    pub fn rx_buf_addr(&self) -> u32 {
        self.rx_buf.get() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeClockTrim, FakeDmaRing, FakeUsartBaud};
    use crate::traits::DmaFlags;
    use dxl_protocol::SoftwareCrcUmts;
    use osc_core::BaudRate;

    /// Test-side storage sizing — matches V006 defaults per doc §§8.1, 8.3,
    /// 8.4 so any drift between driver tests and chip-side reality stays
    /// visible.
    const DECODER_CAP: usize = 256;
    const RX_BUF_LEN: usize = 64;
    const EDGE_BUF_LEN: usize = 128;

    const TEST_ID: u8 = 0x07;
    const TEST_RDT_US: u32 = 250;

    type TestRx = Rx<FakeDmaRing, EDGE_BUF_LEN, RX_BUF_LEN>;
    type TestBus = DxlUart<
        FakeUsartBaud,
        FakeClockTrim,
        FakeDmaRing,
        SoftwareCrcUmts,
        DECODER_CAP,
        RX_BUF_LEN,
        EDGE_BUF_LEN,
    >;

    fn make_rx(vals: &[u16], flags: DmaFlags) -> TestRx {
        let mut rx = Rx::new(FakeDmaRing::default());
        rx.stage_edges_for_test(vals);
        rx.arm_next_flags_for_test(flags);
        rx
    }

    fn make_clock(baud: BaudRate) -> Clock<FakeUsartBaud, FakeClockTrim> {
        Clock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    fn make_bus_with(rx: TestRx, baud: BaudRate) -> TestBus {
        DxlUart::new(rx, make_clock(baud), TEST_ID, TEST_RDT_US)
    }

    fn make_bus() -> TestBus {
        make_bus_with(Rx::new(FakeDmaRing::default()), BaudRate::B3000000)
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
        let mut bus = make_bus_with(rx, BaudRate::B3000000);

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
        let mut bus = make_bus_with(rx, BaudRate::B1000000);

        bus.on_rx_edge_advance();

        // SEED at 1000, then SKIP (intra-byte) — head stays at 1.
        assert_eq!(bus.byte_ts_head(), 1);
    }

    #[test]
    fn on_rx_idle_walks_tail_edges() {
        // IDLE backstop walks edges the HT/TC ISR hasn't drained (small
        // packets that never trip HT). Single tail edge → SEED.
        let rx = make_rx(&[500], DmaFlags::default());
        let mut bus = make_bus_with(rx, BaudRate::B3000000);

        bus.on_rx_idle();

        assert_eq!(bus.byte_ts_head(), 1);
    }

    #[test]
    fn stage_id_defers_until_tx_complete() {
        let mut bus = make_bus();
        bus.stage_id(0x42);
        assert_eq!(bus.id, TEST_ID);
        let reboot = bus.on_tx_complete();
        assert_eq!(bus.id, 0x42);
        assert!(reboot.is_none());
    }

    #[test]
    fn stage_id_noop_when_unchanged() {
        let mut bus = make_bus();
        bus.stage_id(TEST_ID);
        assert!(bus.pending_id.is_none());
    }

    #[test]
    fn stage_rdt_defers_until_tx_complete() {
        let mut bus = make_bus();
        bus.stage_rdt(500);
        assert_eq!(bus.rdt_us, TEST_RDT_US);
        bus.on_tx_complete();
        assert_eq!(bus.rdt_us, 500);
    }

    #[test]
    fn stage_baud_forwards_to_clock() {
        // Cross-driver staging: the bus forwards to the clock sub-driver,
        // which applies the BRR at its own on_tx_complete (called by bus).
        let mut bus = make_bus();
        bus.stage_baud(BaudRate::B1000000);
        // Still at 3M until the apply.
        assert_eq!(bus.clock.ticks_per_bit(), 16);
        bus.on_tx_complete();
        assert_eq!(bus.clock.ticks_per_bit(), 48);
    }

    #[test]
    fn stage_reboot_surfaces_through_on_tx_complete() {
        let mut bus = make_bus();
        bus.stage_reboot(BootMode::Bootloader);
        // Latched but not yet observed.
        assert!(bus.pending_reboot.is_some());
        let mode = bus.on_tx_complete();
        assert_eq!(mode, Some(BootMode::Bootloader));
        // Consume-on-take: next TC is clean.
        assert!(bus.on_tx_complete().is_none());
    }

    #[test]
    fn rx_buf_addr_is_stable() {
        let bus = make_bus();
        let a = bus.rx_buf_addr();
        let b = bus.rx_buf_addr();
        assert_eq!(a, b);
        assert_ne!(a, 0);
    }
}
