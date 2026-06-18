//! RX byte-ring DMA provider — binds `RxDma` to DMA1_CH5 (USART1 RX →
//! byte ring). The channel itself is configured + enabled in `runtime/init`;
//! this provider surfaces `remaining()` (NDTR) for the Fast Last fold body's
//! intra-loop refresh per doc §6 and `record_edge_anchor_miss` for the
//! parser-side no-anchor counter bump.

use osc_drivers::traits::dxl;

use crate::hal::dma;
use crate::runtime::statics::SHARED;

/// Production binding to DMA1_CH5 (USART1 RX → byte ring).
pub struct RxDma;

impl dxl::RxDma for RxDma {
    #[inline(always)]
    fn remaining(&self) -> u16 {
        dma::remaining(dma::Channel::CH5)
    }

    fn record_edge_anchor_miss(&mut self) {
        // Volatile RMW into the telemetry region — same pattern as
        // `record_patch_deadline_miss` on `FastLastScheduler`. Concurrent
        // host clear (via DXL bus write to `edge_anchor_miss`) can drop
        // one update per race window, accepted by the region's `rw`
        // declaration (`telemetry.rs` doc comment).
        // SAFETY: SHARED is the canonical chip-side control-table store;
        // writers other than this one fire from DXL-side ISRs at the same
        // PFIC priority, so the RMW is atomic w.r.t. itself.
        unsafe {
            let p = &raw mut (*SHARED.table.telemetry.get()).link.edge_anchor_miss;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
    }
}
