//! Timing authority for the DXL transport — a mini sub-composite of two
//! independent halves (§4.3):
//!
//! - [`BaudCache`] — the per-baud `ticks_per_bit` / byte-time cache plus the
//!   staged-baud mailbox. Everything downstream (RX classifier window, TX
//!   scheduler wire-end math, snoop tick) reads its ticks; µs conversion
//!   only at the edge (telemetry).
//! - [`DriftIntegrator`] — the boot/steady HSI drift-correction control law
//!   and the trim mailbox.
//!
//! The two inputs that determine `ticks_per_bit` are both committed at
//! `on_tx_complete` (USART can't change BRR mid-frame): baud, staged by
//! control-table writes via [`Clock::stage_baud`]; and clock-trim
//! correction, integrated from drift samples fed via [`Clock::on_byte_pair`].
//! The composite owns no state of its own — it routes each event, command,
//! and accessor to the half that owns it.

mod baud_cache;
mod drift_consts;
mod drift_integrator;

use osc_core::BaudRate;

use crate::dxl::uart::poll_src::PollSrc;
use crate::traits::dxl::{ClockTrim, UsartBaud};
use baud_cache::BaudCache;
use drift_integrator::DriftIntegrator;

/// Round-to-nearest rate divisor — `ticks_per_bit` for a baud at the
/// driver's reference clock. Folds to a literal whenever both arguments
/// are const at the call site. Numerically identical to a USART BRR
/// divisor on chips where the BRR clock equals the monotonic tick clock,
/// but the chip-side provider owns its own BRR math now.
pub(crate) const fn divisor_for(clock_hz: u32, baud_hz: u32) -> u32 {
    (clock_hz + baud_hz / 2) / baud_hz
}

pub struct Clock<U: UsartBaud, T: ClockTrim> {
    cache: BaudCache<U>,
    drift: DriftIntegrator<U, T>,
}

impl<U: UsartBaud, T: ClockTrim> Clock<U, T> {
    pub fn new(baud: BaudRate, usart: U, trim: T) -> Self {
        let cache = BaudCache::new(baud, usart);
        let drift = DriftIntegrator::new(baud, cache.ticks_per_bit(), trim);
        Self { cache, drift }
    }

    // -- events -----------------------------------------------------------------

    /// USART-idle boundary after our own TX — the only safe point to commit
    /// a staged baud (BRR can't change mid-frame). On an applied change the
    /// new `ticks_per_bit` fans out to the drift integrator so its per-baud
    /// constants and reference tpb track the wire.
    pub fn on_tx_complete(&mut self) {
        if self.cache.on_tx_complete().is_some() {
            self.drift
                .on_baud_change(self.cache.baud(), self.cache.ticks_per_bit());
        }
    }

    /// RX-side packet boundary — commits any pending trim correction and
    /// runs the drift integrator's boot→steady transition. See
    /// [`DriftIntegrator::on_rx_packet_end`].
    pub fn on_rx_packet_end(&mut self) {
        self.drift.on_rx_packet_end();
    }

    /// One classifier byte-pair for the drift integrator. See
    /// [`DriftIntegrator::on_byte_pair`].
    pub fn on_byte_pair(&mut self, prev: u16, curr: u16) {
        self.drift.on_byte_pair(prev, curr);
    }

    // -- commands ---------------------------------------------------------------

    pub fn stage_baud(&mut self, baud: BaudRate) {
        self.cache.stage_baud(baud);
    }

    // -- accessors --------------------------------------------------------------

    #[inline(always)]
    pub fn ticks_per_bit(&self) -> u16 {
        self.cache.ticks_per_bit()
    }

    /// Per-baud RX edge-stamp compensation in HCLK ticks. See
    /// [`BaudCache::rx_edge_comp_ticks`].
    #[inline(always)]
    pub fn rx_edge_comp_ticks(&self) -> u16 {
        self.cache.rx_edge_comp_ticks()
    }

    /// Wire-byte duration in monotonic timer ticks at the current baud. See
    /// [`BaudCache::bytes_to_ticks`].
    pub fn bytes_to_ticks(&self, bytes: u32) -> u32 {
        self.cache.bytes_to_ticks(bytes)
    }

    /// One wire byte's duration in HCLK ticks at the current baud. See
    /// [`BaudCache::byte_ticks`].
    #[inline(always)]
    pub fn byte_ticks(&self) -> u16 {
        self.cache.byte_ticks()
    }

    /// Effective RDT for a Fast slot: floored by the poll source's
    /// `now − packet_end` offset so slot 0 can't start before its own chip
    /// observes packet-end (at [`PollSrc::LineIdle`] that's one byte-time
    /// past `packet_end`) and the whole chain shifts together. `ByteBatch`
    /// needs no floor. The Fast Last grid reads this to back-date from the
    /// same anchor the schedule used.
    pub fn effective_slot_rdt(&self, rdt_ticks: u32, src: PollSrc) -> u32 {
        let floor = match src {
            PollSrc::ByteBatch => 0,
            PollSrc::LineIdle => self.byte_ticks() as u32,
        };
        rdt_ticks.max(floor)
    }

    /// Plain (status / non-Fast) reply deadline: anchor `rdt_ticks` + the
    /// slot-offset gap against `packet_end_tick`, folding the drift
    /// integrator's projected phase error over that distance.
    pub fn compute_status_deadline(
        &self,
        packet_end_tick: u32,
        rdt_ticks: u32,
        slot_offset_bytes: u32,
    ) -> u32 {
        let delay = rdt_ticks.wrapping_add(self.cache.bytes_to_ticks(slot_offset_bytes));
        self.deadline_after(packet_end_tick, delay)
    }

    /// Fast slot 0 (Only/First) reply deadline: RDT past the request's
    /// wire end per the DXL spec, with the source-floored effective RDT
    /// (see [`Self::effective_slot_rdt`]) so an IDLE-observed packet end
    /// can't schedule the start before this chip could have seen it.
    /// Slots k > 0 never come here — they anchor on the observed status
    /// start via [`Self::compute_slot_start_deadline`] (task #142).
    pub fn compute_slot_deadline(&self, packet_end_tick: u32, rdt_ticks: u32, src: PollSrc) -> u32 {
        let effective_rdt_ticks = self.effective_slot_rdt(rdt_ticks, src);
        self.deadline_after(packet_end_tick, effective_rdt_ticks)
    }

    /// FAST slot k > 0 wire-start deadline anchored on the observed start
    /// of the chain's Status packet: our slot starts `slot_offset_bytes`
    /// whole wire bytes after that first byte's start edge — a contiguous
    /// continuation of the single Status packet. No RDT term (RDT is
    /// single-target-reply-only per spec) and no LineIdle floor: the
    /// anchor is a hardware edge stamp strictly in the past, physically
    /// received before the query could run, so there is nothing to floor
    /// against. Folds the drift phase residual over the extrapolated
    /// distance like every other schedule site — load-bearing for long
    /// chains at low baud, where the distance spans many milliseconds.
    pub fn compute_slot_start_deadline(
        &self,
        status_start_tick: u32,
        slot_offset_bytes: u32,
    ) -> u32 {
        self.deadline_after(
            status_start_tick,
            self.cache.bytes_to_ticks(slot_offset_bytes),
        )
    }

    /// Max byte pairs per packet the driver should drain into
    /// [`Self::on_byte_pair`]. See [`DriftIntegrator::samples_wanted_per_packet`].
    pub fn samples_wanted_per_packet(&self) -> u8 {
        self.drift.samples_wanted_per_packet()
    }

    /// Project the drift integrator's pending residual onto a scheduled
    /// deadline distance. See [`DriftIntegrator::projected_phase_error_hclk`].
    pub fn projected_phase_error_hclk(&self, distance_hclk: u32) -> i32 {
        self.drift.projected_phase_error_hclk(distance_hclk)
    }

    /// Anchor a `delay_ticks` distance against `packet_end_tick` and fold in
    /// the drift integrator's projected phase error over that distance.
    /// Shared tail of both deadline computations.
    fn deadline_after(&self, packet_end_tick: u32, delay_ticks: u32) -> u32 {
        let phase_adjust = self.drift.projected_phase_error_hclk(delay_ticks);
        packet_end_tick
            .wrapping_add(delay_ticks)
            .wrapping_add_signed(phase_adjust)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::BITS_PER_FRAME;
    use crate::dxl::uart::test_support::{
        TICKS_PER_BIT_3M, TICKS_PER_BIT_9600, mk_clock_trim, mk_usart_baud,
    };
    use crate::mocks::{MockClockTrim, MockUsartBaud};

    type TestClock = Clock<MockUsartBaud, MockClockTrim>;

    fn clock_at(baud: BaudRate) -> TestClock {
        let (usart, _) = mk_usart_baud();
        let (trim, _) = mk_clock_trim();
        Clock::new(baud, usart, trim)
    }

    /// Close one boot drift batch at B9600 with per-sample drift
    /// `delta_tpb` bits — leaves a same-signed sub-step residual for the
    /// phase-fold tests (mirrors the `drift_integrator` fixtures).
    fn clock_9600_with_residual(delta_tpb: i16) -> TestClock {
        let mut c = clock_at(BaudRate::B9600);
        let observed = (TICKS_PER_BIT_9600 as i32 + delta_tpb as i32) as u16;
        for _ in 0..6 {
            c.on_byte_pair(0, observed.wrapping_mul(BITS_PER_FRAME));
        }
        c
    }

    const BYTE_TICKS_3M: u32 = TICKS_PER_BIT_3M as u32 * BITS_PER_FRAME as u32;

    // ---------- effective_slot_rdt ----------

    #[test]
    fn effective_slot_rdt_is_identity_at_byte_batch() {
        let c = clock_at(BaudRate::B3000000);
        assert_eq!(c.effective_slot_rdt(0, PollSrc::ByteBatch), 0);
        assert_eq!(c.effective_slot_rdt(100, PollSrc::ByteBatch), 100);
    }

    #[test]
    fn effective_slot_rdt_floor_engages_at_line_idle() {
        // LineIdle observes packet-end one byte-time late, so the RDT
        // floors at one byte-time (160 ticks at 3M).
        let c = clock_at(BaudRate::B3000000);
        assert_eq!(c.effective_slot_rdt(100, PollSrc::LineIdle), BYTE_TICKS_3M);
        assert_eq!(c.effective_slot_rdt(0, PollSrc::LineIdle), BYTE_TICKS_3M);
    }

    #[test]
    fn effective_slot_rdt_floor_is_inert_above_one_byte_time() {
        let c = clock_at(BaudRate::B3000000);
        assert_eq!(
            c.effective_slot_rdt(BYTE_TICKS_3M + 1, PollSrc::LineIdle),
            BYTE_TICKS_3M + 1
        );
    }

    // ---------- deadline composition ----------

    #[test]
    fn status_deadline_is_packet_end_plus_rdt_plus_offset() {
        // No drift residual → pure delay composition. 12 offset bytes at
        // 3M = 1920 ticks.
        let c = clock_at(BaudRate::B3000000);
        assert_eq!(c.compute_status_deadline(50_000, 12_000, 0), 62_000);
        assert_eq!(c.compute_status_deadline(50_000, 12_000, 12), 63_920);
    }

    #[test]
    fn slot_deadline_equals_status_deadline_with_pre_floored_rdt() {
        // `compute_slot_deadline` is `compute_status_deadline` composed
        // with the source floor — pin the equivalence at both sources.
        let c = clock_at(BaudRate::B3000000);
        for (rdt, src) in [
            (100, PollSrc::LineIdle),
            (12_000, PollSrc::LineIdle),
            (100, PollSrc::ByteBatch),
        ] {
            let floored = c.effective_slot_rdt(rdt, src);
            assert_eq!(
                c.compute_slot_deadline(50_000, rdt, src),
                c.compute_status_deadline(50_000, floored, 0)
            );
        }
    }

    #[test]
    fn slot_start_deadline_is_anchor_plus_whole_bytes_no_rdt_no_floor() {
        // 14 offset bytes at 3M = 2240 ticks past the observed status
        // start — no RDT term, no LineIdle floor, nothing else.
        let c = clock_at(BaudRate::B3000000);
        assert_eq!(c.compute_slot_start_deadline(50_000, 14), 52_240);
        assert_eq!(c.compute_slot_start_deadline(50_000, 0), 50_000);
    }

    #[test]
    fn slot_start_deadline_wraps_cleanly_near_u32_max() {
        let c = clock_at(BaudRate::B3000000);
        let anchor = u32::MAX - 100;
        assert_eq!(
            c.compute_slot_start_deadline(anchor, 12),
            anchor.wrapping_add(12 * BYTE_TICKS_3M)
        );
    }

    #[test]
    fn deadline_wraps_cleanly_near_u32_max() {
        let c = clock_at(BaudRate::B3000000);
        let packet_end = u32::MAX - 100;
        assert_eq!(
            c.compute_status_deadline(packet_end, 1_000, 0),
            packet_end.wrapping_add(1_000)
        );
    }

    // ---------- drift phase fold ----------

    #[test]
    fn deadline_folds_positive_phase_residual_later() {
        // HSI fast → positive residual → the reply must start LATER in
        // chip ticks to land on the host's reference.
        let c = clock_9600_with_residual(40);
        let delay = 120_000;
        let adjust = c.projected_phase_error_hclk(delay);
        assert!(adjust > 0, "boot batch must leave a positive residual");
        assert_eq!(
            c.compute_status_deadline(50_000, delay, 0),
            50_000 + delay + adjust as u32
        );
    }

    #[test]
    fn deadline_folds_negative_phase_residual_earlier() {
        let c = clock_9600_with_residual(-40);
        let delay = 120_000;
        let adjust = c.projected_phase_error_hclk(delay);
        assert!(adjust < 0, "boot batch must leave a negative residual");
        assert_eq!(
            c.compute_status_deadline(50_000, delay, 0),
            (50_000 + delay).wrapping_add_signed(adjust)
        );
    }

    #[test]
    fn slot_deadline_folds_the_same_phase_residual() {
        let c = clock_9600_with_residual(40);
        let rdt = 120_000;
        assert_eq!(
            c.compute_slot_deadline(50_000, rdt, PollSrc::ByteBatch),
            c.compute_status_deadline(50_000, rdt, 0),
            "slot and status paths must fold the residual identically"
        );
    }

    #[test]
    fn slot_start_deadline_folds_the_same_phase_residual() {
        // Long-chain low-baud extrapolation: the drift residual must fold
        // over the byte distance exactly as the status path folds it over
        // an equal tick delay.
        let c = clock_9600_with_residual(40);
        let offset_bytes = 3; // 150_000 ticks at 9600 — a multi-ms span
        let delay = c.bytes_to_ticks(offset_bytes);
        assert_eq!(
            c.compute_slot_start_deadline(50_000, offset_bytes),
            c.compute_status_deadline(50_000, delay, 0),
            "slot-start and status paths must fold the residual identically"
        );
    }
}
