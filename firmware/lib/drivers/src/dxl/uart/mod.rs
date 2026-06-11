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

use dxl_protocol::decoder::{Decoder, Step};
use dxl_protocol::{BROADCAST_ID, CrcUmts, InstructionPacket};
use osc_core::{BaudRate, BootMode};

use crate::traits::{ClockTrim, DmaRing, UsartBaud};
use crate::util::DmaBuffer;
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

    decoder: Decoder<DECODER_CAP, CRC>,
    /// DMA1_CH5 destination for received bytes. `SyncUnsafeCell` because
    /// USART1's DMA writes it concurrently with the parser's reads — both
    /// reads happen at PFIC HIGH (no preemption from another consumer)
    /// and the producer is hardware. `DmaBuffer` enforces pow-2 sizing so
    /// `% RX_BUF_LEN` collapses to AND.
    rx_buf: SyncUnsafeCell<DmaBuffer<u8, RX_BUF_LEN>>,
    /// Sequence number of the next RX byte to drain into the decoder. Wraps
    /// at u16; the ring slot is `parsed_idx % RX_BUF_LEN` (doc §10.5).
    parsed_idx: u16,
    /// Monotonic count of Instruction packets the decoder emitted —
    /// regardless of target ID — across this driver's lifetime. The drift
    /// signal ([[drift_sampling_instruction_only]]) ticks on every
    /// Instruction so foreign-target instructions still calibrate, while
    /// Status frames (which use the peer's HSI) never contribute.
    instruction_count: u32,

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
            rx_buf: SyncUnsafeCell::new(DmaBuffer::new(0)),
            parsed_idx: 0,
            instruction_count: 0,
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

    /// Drain `rx_buf[parsed_idx..drain_to)` through the streaming decoder
    /// and surface the first Instruction packet addressed to us (or
    /// BROADCAST). Status frames are dropped silently (Instruction-only
    /// surface per [[standard_dxl_terminology]]); Instructions for foreign
    /// IDs are also dropped, but every emitted Instruction still bumps
    /// `instruction_count()` so the drift signal stays clean.
    ///
    /// `drain_to` is the caller-supplied RX-byte frontier (chip-side DMA
    /// head, masked with the BT frontier per doc §10.5 so the parser never
    /// reads past either ring's published cursor). On `Step::NeedMore` the
    /// parser yields with its in-progress packet state intact and resumes
    /// from the same `parsed_idx` on the next call.
    pub fn poll(&mut self, drain_to: u16) -> Option<InstructionPacket<'_>> {
        let mut matched = false;
        while self.parsed_idx != drain_to {
            // SAFETY: rx_buf is written only by DMA1_CH5 (hardware writer)
            // and read here from the same PFIC priority level as the DMA
            // HT/TC ISR, so no other consumer can `&mut` it concurrently.
            let byte = unsafe { *(*self.rx_buf.get()).at(self.parsed_idx) };
            self.parsed_idx = self.parsed_idx.wrapping_add(1);
            let (step, _) = self.decoder.feed(&[byte]);
            match step {
                Step::NeedMore | Step::Resync(_) => continue,
                Step::Packet(pkt) => {
                    let ip = match pkt.into_instruction_packet() {
                        Some(ip) => ip,
                        None => continue,
                    };
                    self.instruction_count = self.instruction_count.wrapping_add(1);
                    let id = ip.id().as_byte();
                    if id == self.id || id == BROADCAST_ID {
                        matched = true;
                        break;
                    }
                }
            }
        }
        // Re-derive the surfaced packet through the immutable
        // `dispatch_packet` path so its lifetime flows to the return
        // without holding a mid-loop `&mut self.decoder` borrow (the
        // direct path trips the borrow-checker's loop-back analysis).
        // The decoder stays in `Done` until the next `feed`, so the
        // overlay is still valid here.
        matched
            .then(|| self.decoder.dispatch_packet().into_instruction_packet())
            .flatten()
    }

    /// Monotonic count of Instruction packets seen on the wire. Used by the
    /// drift estimator as its tick source; only foreign Status frames are
    /// excluded, so this advances on every Instruction the bus decodes.
    pub fn instruction_count(&self) -> u32 {
        self.instruction_count
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
    /// Stage `bytes` into `rx_buf` starting at sequence index `at`, wrapping
    /// the ring. Mirrors the chip-side DMA1_CH5 writer for host tests.
    pub(crate) fn stage_rx_bytes_for_test(&mut self, at: u16, bytes: &[u8]) {
        let buf = unsafe { &mut *self.rx_buf.get() };
        buf.stage(at, bytes);
    }

    /// Pre-position the parser cursor (wrap-test seed). Production has no
    /// reason to set this directly — it's monotonic from `new()` onwards.
    pub(crate) fn set_parsed_idx_for_test(&mut self, idx: u16) {
        self.parsed_idx = idx;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mocks::{FakeClockTrim, FakeDmaRing, FakeUsartBaud};
    use crate::traits::DmaFlags;
    use dxl_protocol::packet::Id;
    use dxl_protocol::{InstructionEmitter, SoftwareCrcUmts, StatusEmitter};
    use heapless::Vec;
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

    /// Emit a Ping addressed to `id`.
    fn wire_ping(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .ping(Id::new(id))
            .unwrap();
        out
    }

    /// Emit a Status reply from `id` (the kind we never want to surface).
    fn wire_status(id: u8) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        StatusEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .empty(Id::new(id), dxl_protocol::packet::StatusError::OK)
            .unwrap();
        out
    }

    /// Sync Read targets BROADCAST on the wire — exercises the
    /// id-as-BROADCAST branch with a richer (non-Ping) packet shape.
    fn wire_sync_read(addr: u16, length: u16, ids: &[u8]) -> Vec<u8, 32> {
        let mut out: Vec<u8, 32> = Vec::new();
        InstructionEmitter::<_, SoftwareCrcUmts>::new(&mut out)
            .sync_read(addr, length, ids)
            .unwrap();
        out
    }

    /// Stage `bytes` at offset 0 and poll up to their end. Returns the
    /// surfaced ID byte (or None).
    fn poll_after(bus: &mut TestBus, bytes: &[u8]) -> Option<u8> {
        bus.stage_rx_bytes_for_test(0, bytes);
        bus.poll(bytes.len() as u16).map(|ip| ip.id().as_byte())
    }

    #[test]
    fn poll_returns_none_when_no_new_bytes() {
        let mut bus = make_bus();
        assert!(bus.poll(0).is_none());
        // Idempotent — calling again still drains nothing.
        assert!(bus.poll(0).is_none());
    }

    #[test]
    fn poll_surfaces_instruction_addressed_to_us() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        assert_eq!(poll_after(&mut bus, &pkt), Some(TEST_ID));
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_drops_instruction_for_other_id() {
        let mut bus = make_bus();
        let pkt = wire_ping(0x42);
        assert_eq!(poll_after(&mut bus, &pkt), None);
        // Still counted — foreign Instructions feed the drift signal.
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_surfaces_broadcast_instruction() {
        let mut bus = make_bus();
        // SyncRead targets BROADCAST on the wire — exact id == 0xFE.
        let pkt = wire_sync_read(0x84, 4, &[0x01, 0x02, 0x03]);
        bus.stage_rx_bytes_for_test(0, &pkt);
        match bus.poll(pkt.len() as u16) {
            Some(InstructionPacket::SyncRead(_)) => {}
            other => panic!("expected SyncRead, got {other:?}"),
        }
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_drops_status_silently() {
        let mut bus = make_bus();
        let pkt = wire_status(TEST_ID);
        assert_eq!(poll_after(&mut bus, &pkt), None);
        // Status frames never tick the drift signal — peer HSI is its own
        // clock domain (see [[drift_sampling_instruction_only]]).
        assert_eq!(bus.instruction_count(), 0);
    }

    #[test]
    fn poll_recovers_from_bad_crc() {
        let mut bus = make_bus();
        let mut bad = wire_ping(TEST_ID);
        // Corrupt the second-to-last byte — flips one CRC byte.
        let crc_lo_pos = bad.len() - 2;
        bad[crc_lo_pos] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        // Stage the bad packet at offset 0, then the good one right after.
        let mut combined: Vec<u8, 32> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();
        bus.stage_rx_bytes_for_test(0, &combined);

        // First poll: decoder Resyncs on the bad CRC, then decodes the
        // good packet and surfaces it.
        let surfaced = bus.poll(combined.len() as u16);
        assert!(
            surfaced.is_some(),
            "good packet should surface after resync"
        );
        // Only the good packet counts — Resync drops the bad one before
        // it ever becomes a Packet step.
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_handles_ring_wrap() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        // Position the packet so it straddles the ring boundary.
        let start = (RX_BUF_LEN as u16).wrapping_sub(4);
        bus.set_parsed_idx_for_test(start);
        bus.stage_rx_bytes_for_test(start, &pkt);

        let drain_to = start.wrapping_add(pkt.len() as u16);
        match bus.poll(drain_to) {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id.as_byte(), TEST_ID),
            other => panic!("expected Ping across wrap, got {other:?}"),
        }
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        bus.stage_rx_bytes_for_test(0, &pkt);

        // Feed everything except the last byte — decoder is mid-CRC.
        let split = (pkt.len() - 1) as u16;
        assert!(bus.poll(split).is_none());
        assert_eq!(bus.instruction_count(), 0);

        // Feed the final byte — packet now completes and surfaces.
        match bus.poll(pkt.len() as u16) {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id.as_byte(), TEST_ID),
            other => panic!("expected Ping after resume, got {other:?}"),
        }
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn instruction_count_advances_on_every_instruction_regardless_of_id() {
        let mut bus = make_bus();
        let ours = wire_ping(TEST_ID);
        let theirs = wire_ping(0x42);
        let status = wire_status(0x42);

        let mut combined: Vec<u8, 64> = Vec::new();
        combined.extend_from_slice(&ours).unwrap();
        combined.extend_from_slice(&theirs).unwrap();
        combined.extend_from_slice(&status).unwrap();
        bus.stage_rx_bytes_for_test(0, &combined);

        // First call surfaces ours; decoder pauses at packet boundary.
        let surfaced = bus.poll(combined.len() as u16);
        assert!(matches!(surfaced, Some(InstructionPacket::Ping(_))));

        // Drain the rest — foreign Ping bumps the counter, Status doesn't.
        assert!(bus.poll(combined.len() as u16).is_none());
        assert_eq!(bus.instruction_count(), 2);
    }
}
