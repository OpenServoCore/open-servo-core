//! Timing authority for the DXL transport — a mini sub-composite of two
//! independent halves (§4.3):
//!
//! - [`BaudCache`] — the per-baud `ticks_per_bit` / byte-time cache plus the
//!   staged-baud mailbox. Everything downstream (RX classifier window, fire
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
mod drift_integrator;

use osc_core::BaudRate;

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
        if self.cache.commit_pending_baud().is_some() {
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
}
