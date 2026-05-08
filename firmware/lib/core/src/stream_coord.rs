//! Cross-domain RMW counters used by the kernel ↔ services handoff. Not part
//! of the control table; lives as a separate field on `Shared`.

use portable_atomic::{AtomicU16, AtomicU32};

pub struct StreamCoord {
    /// ISR `fetch_add`s on each decimated tick; main reads + clears.
    pub pending: AtomicU32,
    /// Main writes after draining; ISR reads (diag only).
    pub consumed: AtomicU32,
    /// USART1 IDLE handler stores the DMA write index.
    pub dxl_rx_write_pos: AtomicU16,
}

impl StreamCoord {
    pub const fn const_new() -> Self {
        Self {
            pending: AtomicU32::new(0),
            consumed: AtomicU32::new(0),
            dxl_rx_write_pos: AtomicU16::new(0),
        }
    }
}
