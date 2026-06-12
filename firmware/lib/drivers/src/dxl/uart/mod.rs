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

use dxl_protocol::{BROADCAST_ID, CrcUmts, InstructionPacket};
use osc_core::{BaudRate, BootMode};

use crate::traits::{ClockTrim, DmaRing, UsartBaud};
use crate::util::Seq;
use clock::Clock;
use codec::Codec;

/// The DXL bus composite. The type parameters together describe everything
/// this driver wants the chip side to supply — what peripherals to drive
/// it with, what CRC to use, and how much memory each of its three rings
/// gets:
///
/// - `U`: USART baud-rate setter (sub-driver `Clock`).
/// - `T`: HSI / HSE trim setter (sub-driver `Clock`).
/// - `R`: DMA-ring ISR surface for DMA1_CH7 (carried by `Codec` through
///   its inner `Rx`).
/// - `CRC`: CRC-16/UMTS engine — software impl on V006, peripheral impl
///   on a chip that has one (see `providers::dxl_crc`).
/// - `DECODER_CAP`: streaming-decoder accumulator size. Sized to hold the
///   longest unstuffed frame the dispatcher will encounter (typically 256
///   covers max-RW + header + margin); decoupled from the on-wire RX byte
///   ring because the parser drains continuously per doc §8.1.
/// - `RX_BUF_LEN`: DMA1_CH5 byte-ring depth (typically 64 per doc §8.1).
///   Also drives the BT ring depth inside `Codec` — doc §8.3 requires
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
    codec: Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>,
    clock: Clock<U, T>,

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
        codec: Codec<R, CRC, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>,
        clock: Clock<U, T>,
        id: u8,
        rdt_us: u32,
    ) -> Self {
        Self {
            codec,
            clock,
            id,
            rdt_us,
            pending_id: None,
            pending_rdt_us: None,
            pending_reboot: None,
        }
    }

    /// New RX falling-edge timestamps may be available — pull the current
    /// `ticks_per_bit` from the clock and forward to the codec's RX
    /// classifier.
    pub fn on_rx_edge_advance(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.codec.on_edge_advance(ticks_per_bit);
    }

    /// RX wire went idle — drain tail edges and reset the classifier
    /// anchor for the next burst.
    pub fn on_rx_idle(&mut self) {
        let ticks_per_bit = self.clock.ticks_per_bit();
        self.codec.on_idle(ticks_per_bit);
    }

    /// USART1 RX DMA published progress — `remaining` is the channel's
    /// NDTR readback.
    pub fn on_rx_dma_advance(&mut self, remaining: u16) {
        self.codec.on_rx_dma_advance(remaining);
    }

    /// Drain the codec until an Instruction addressed to us (or BROADCAST)
    /// surfaces; route each parsed Instruction's BT pair walk into the
    /// clock's drift integrator. Foreign Instructions feed drift but
    /// don't surface; Status frames feed neither.
    ///
    /// Composite-owned routing per driver-pattern §4 + §10.1 — the wire
    /// diagram lives in this file's method bodies, codec exposes the
    /// `byte_pairs` accessor over its own BT ring, clock exposes
    /// `on_byte_pair` which does the HIT-window check + delta/10 +
    /// drift integration.
    pub fn poll(&mut self) -> Option<InstructionPacket<'_>> {
        let id = self.id;
        let codec = &mut self.codec;
        let clock = &mut self.clock;
        loop {
            let token = codec.poll_one()?;
            for (prev, curr) in codec.byte_pairs(token.start, token.end) {
                clock.on_byte_pair(prev, curr);
            }
            if token.id == id || token.id == BROADCAST_ID {
                return codec.dispatch();
            }
            // Foreign Instruction — drift fed above, drop the token and
            // continue polling.
        }
    }

    /// Monotonic count of Instruction packets seen on the wire — own and
    /// foreign IDs included; Status frames excluded. Drift estimator's
    /// tick source.
    pub fn instruction_count(&self) -> u32 {
        self.codec.instruction_count()
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
    pub fn byte_ts_head(&self) -> Seq<u16, RX_BUF_LEN> {
        self.codec.byte_ts_head()
    }

    /// Stable peripheral-memory address for DMA1_CH7's destination buffer.
    pub fn edges_addr(&self) -> u32 {
        self.codec.edges_addr()
    }

    /// Stable peripheral-memory address for DMA1_CH5's destination buffer.
    pub fn rx_buf_addr(&self) -> u32 {
        self.codec.rx_buf_addr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::codec::rx::Rx;
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
    /// One byte-time at 3 Mbaud (10·tpb).
    const BYTE_TICKS_3M: u16 = 160;

    type TestCodec = Codec<FakeDmaRing, SoftwareCrcUmts, DECODER_CAP, RX_BUF_LEN, EDGE_BUF_LEN>;
    type TestBus = DxlUart<
        FakeUsartBaud,
        FakeClockTrim,
        FakeDmaRing,
        SoftwareCrcUmts,
        DECODER_CAP,
        RX_BUF_LEN,
        EDGE_BUF_LEN,
    >;

    fn make_codec_with_edges(vals: &[u16], flags: DmaFlags) -> TestCodec {
        let mut c: TestCodec = Codec::new(FakeDmaRing::default());
        c.stage_edges_for_test(vals);
        c.arm_next_flags_for_test(flags);
        c
    }

    fn make_clock(baud: BaudRate) -> Clock<FakeUsartBaud, FakeClockTrim> {
        Clock::new(baud, FakeUsartBaud::default(), FakeClockTrim::default())
    }

    fn make_bus_with(codec: TestCodec, baud: BaudRate) -> TestBus {
        DxlUart::new(codec, make_clock(baud), TEST_ID, TEST_RDT_US)
    }

    fn make_bus() -> TestBus {
        make_bus_with(Codec::new(FakeDmaRing::default()), BaudRate::B3000000)
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_3m() {
        // At 3M tpb=16 → byte=160 ticks; the 160-tick delta is a window HIT.
        // If the composite forwarded a wrong tpb (e.g. 48 from a 1M clock),
        // the same delta would fall outside [9·48, 11·48] and classify as
        // SKIP — byte_ts_head would stay at 1 instead of advancing to 2.
        let codec = make_codec_with_edges(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B3000000);

        bus.on_rx_edge_advance();

        assert_eq!(bus.byte_ts_head().test_raw(), 2);
    }

    #[test]
    fn on_rx_edge_advance_pulls_tpb_from_clock_at_1m() {
        // Mirror of the 3M case: at tpb=48 the byte-time is 480 ticks. Two
        // edges 480 apart land as a HIT; if tpb were misrouted to 16, the
        // 480-tick delta would classify as GAP+re-anchor but still advance
        // — so we assert the HIT/SKIP boundary instead: two edges 160 apart
        // are HIT at 3M, SKIP at 1M.
        let codec = make_codec_with_edges(
            &[1000, 1160],
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B1000000);

        bus.on_rx_edge_advance();

        // SEED at 1000, then SKIP (intra-byte) — head stays at 1.
        assert_eq!(bus.byte_ts_head().test_raw(), 1);
    }

    #[test]
    fn on_rx_idle_walks_tail_edges() {
        // IDLE backstop walks edges the HT/TC ISR hasn't drained (small
        // packets that never trip HT). Single tail edge → SEED.
        let codec = make_codec_with_edges(&[500], DmaFlags::default());
        let mut bus = make_bus_with(codec, BaudRate::B3000000);

        bus.on_rx_idle();

        assert_eq!(bus.byte_ts_head().test_raw(), 1);
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

    /// Stage `bytes` at offset 0 (auto-publishing write head) and poll.
    /// Returns the surfaced ID byte (or None).
    fn poll_after(bus: &mut TestBus, bytes: &[u8]) -> Option<u8> {
        bus.codec.stage_rx_bytes_for_test(0, bytes);
        bus.poll().map(|ip| ip.id().as_byte())
    }

    #[test]
    fn poll_returns_none_when_no_new_bytes() {
        let mut bus = make_bus();
        assert!(bus.poll().is_none());
        // Idempotent — calling again still drains nothing.
        assert!(bus.poll().is_none());
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
        let pkt = wire_sync_read(0x84, 4, &[0x01, 0x02, 0x03]);
        bus.codec.stage_rx_bytes_for_test(0, &pkt);
        match bus.poll() {
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
        bus.codec.stage_rx_bytes_for_test(0, &combined);

        // First poll: decoder Resyncs on the bad CRC, then decodes the
        // good packet and surfaces it.
        let surfaced = bus.poll();
        assert!(
            surfaced.is_some(),
            "good packet should surface after resync"
        );
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_handles_ring_wrap() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);
        // Position the packet so it straddles the ring boundary.
        let start = (RX_BUF_LEN as u16).wrapping_sub(4);
        bus.codec.set_rx_read_seq_for_test(start);
        bus.codec.stage_rx_bytes_for_test(start, &pkt);

        match bus.poll() {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id.as_byte(), TEST_ID),
            other => panic!("expected Ping across wrap, got {other:?}"),
        }
        assert_eq!(bus.instruction_count(), 1);
    }

    #[test]
    fn poll_partial_packet_resumes_on_next_call() {
        let mut bus = make_bus();
        let pkt = wire_ping(TEST_ID);

        // Stage everything except the last byte — decoder is mid-CRC.
        let split = pkt.len() - 1;
        bus.codec.stage_rx_bytes_for_test(0, &pkt[..split]);
        assert!(bus.poll().is_none());
        assert_eq!(bus.instruction_count(), 0);

        // Stage the final byte (extending the producer head) — packet now
        // completes and surfaces.
        bus.codec
            .stage_rx_bytes_for_test(split as u16, &pkt[split..]);
        match bus.poll() {
            Some(InstructionPacket::Ping(p)) => assert_eq!(p.header.id.as_byte(), TEST_ID),
            other => panic!("expected Ping after resume, got {other:?}"),
        }
        assert_eq!(bus.instruction_count(), 1);
    }

    /// Stage a stream of `n_bytes` edges spaced `step` ticks apart and run
    /// the classifier so BT[0..n_bytes] is populated before the test polls.
    /// Used by the drift-sample tests to drive realistic BT pairs without
    /// faking the classifier output directly.
    fn bus_with_staged_bt(n_bytes: usize, step: u16) -> TestBus {
        let mut edges: Vec<u16, 32> = Vec::new();
        for i in 0..n_bytes as u16 {
            edges
                .push(1000_u16.wrapping_add(i.wrapping_mul(step)))
                .unwrap();
        }
        let codec = make_codec_with_edges(
            &edges,
            DmaFlags {
                ht: true,
                tc: false,
            },
        );
        let mut bus = make_bus_with(codec, BaudRate::B3000000);
        bus.on_rx_edge_advance();
        assert_eq!(bus.byte_ts_head().test_raw(), n_bytes as u16);
        bus
    }

    #[test]
    fn poll_feeds_one_drift_sample_per_bt_pair_on_instruction() {
        let ping = wire_ping(TEST_ID);
        let mut bus = bus_with_staged_bt(ping.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        let surfaced = bus.poll();
        assert!(matches!(surfaced, Some(InstructionPacket::Ping(_))));

        // 10-byte packet → 9 in-window BT pairs → 9 samples to the integrator.
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_feeds_drift_for_foreign_instruction() {
        // Foreign-ID Instruction doesn't surface but still calibrates the
        // clock — host has HSE, drift signal is valid for every Instruction.
        let ping = wire_ping(0x42);
        let mut bus = bus_with_staged_bt(ping.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        assert!(bus.poll().is_none());
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_does_not_feed_drift_for_status() {
        // Status frame uses the peer's HSI — sampling its BT pairs would
        // calibrate against another HSI ([[drift_sampling_instruction_only]]).
        let status = wire_status(TEST_ID);
        let mut bus = bus_with_staged_bt(status.len(), BYTE_TICKS_3M);
        bus.codec.stage_rx_bytes_for_test(0, &status);

        assert!(bus.poll().is_none());
        assert_eq!(bus.clock.drift_samples(), 0);
    }

    #[test]
    fn poll_filters_out_of_window_bt_pairs() {
        // Edges spaced 200 ticks > 11·tpb=176 at 3M → GAP class. BT entries
        // still land (re-anchor), but clock.on_byte_pair's HIT-window check
        // filters their pair deltas — `Clock` owns the window math now.
        let ping = wire_ping(TEST_ID);
        let mut bus = bus_with_staged_bt(ping.len(), 200);
        bus.codec.stage_rx_bytes_for_test(0, &ping);

        assert!(bus.poll().is_some());
        assert_eq!(bus.clock.drift_samples(), 0);
    }

    #[test]
    fn poll_drift_walk_anchors_at_first_consumed_byte_not_zero() {
        // Reader pre-positioned past the start of the BT ring. Packet
        // begins at RX seq 5, not 0. The drift walk must anchor on the
        // first byte poll() actually consumes — otherwise it would walk
        // BT[0..end] and either misattribute samples (if BT[0..5] are
        // stale-but-in-window) or silently early-return.
        let ping = wire_ping(TEST_ID);
        // Stage enough edges to populate BT[0..5 + ping.len()] — covers
        // both the pre-cursor slots and the packet's range.
        let mut bus = bus_with_staged_bt(5 + ping.len(), BYTE_TICKS_3M);

        bus.codec.set_rx_read_seq_for_test(5);
        bus.codec.stage_rx_bytes_for_test(5, &ping);

        assert!(bus.poll().is_some());
        // Exactly (ping.len() - 1) samples, not 5 + (ping.len() - 1).
        assert_eq!(bus.clock.drift_samples(), (ping.len() - 1) as u16);
    }

    #[test]
    fn poll_drift_walk_resumes_across_resync() {
        // A corrupt frame followed by a clean one: Resync moves the BT
        // range cursor past the bad bytes so the clean frame's BT pairs
        // (and only those) feed the integrator.
        let mut bad = wire_ping(TEST_ID);
        let crc_lo = bad.len() - 2;
        bad[crc_lo] ^= 0xFF;
        let good = wire_ping(TEST_ID);

        let total_bytes = bad.len() + good.len();
        let mut bus = bus_with_staged_bt(total_bytes, BYTE_TICKS_3M);

        let mut combined: Vec<u8, 32> = Vec::new();
        combined.extend_from_slice(&bad).unwrap();
        combined.extend_from_slice(&good).unwrap();
        bus.codec.stage_rx_bytes_for_test(0, &combined);

        assert!(bus.poll().is_some());
        // Only the good frame's pairs feed; the bad frame's bytes were
        // dropped through Resync, so its BT range never enters the walk.
        assert_eq!(bus.clock.drift_samples(), (good.len() - 1) as u16);
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
        bus.codec.stage_rx_bytes_for_test(0, &combined);

        // First call surfaces ours; decoder pauses at packet boundary.
        let surfaced = bus.poll();
        assert!(matches!(surfaced, Some(InstructionPacket::Ping(_))));

        // Drain the rest — foreign Ping bumps the counter, Status doesn't.
        assert!(bus.poll().is_none());
        assert_eq!(bus.instruction_count(), 2);
    }

    /// Sanity: `Rx` is still reachable through the codec module path for
    /// any chip-side wiring that wants direct access. The composite never
    /// reaches around the codec; this just pins the visibility contract.
    #[allow(dead_code)]
    fn _rx_path_stays_pub() {
        let _: Option<Rx<FakeDmaRing, EDGE_BUF_LEN, RX_BUF_LEN>> = None;
    }
}
