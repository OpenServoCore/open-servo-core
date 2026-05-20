//! Kernel ↔ services handoff counters. Separate from the control table.

use portable_atomic::{AtomicU16, AtomicU32};

pub struct StreamCoord {
    /// ISR `fetch_add` on each decimated tick; main reads + clears.
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

#[cfg(test)]
mod tests {
    use super::*;
    use portable_atomic::Ordering::Relaxed;

    #[test]
    fn counters_init_zero() {
        let c = StreamCoord::const_new();
        assert_eq!(c.pending.load(Relaxed), 0);
        assert_eq!(c.consumed.load(Relaxed), 0);
        assert_eq!(c.dxl_rx_write_pos.load(Relaxed), 0);
    }

    #[test]
    fn pending_fetch_add_then_drain() {
        let c = StreamCoord::const_new();
        c.pending.fetch_add(3, Relaxed);
        c.pending.fetch_add(2, Relaxed);
        let drained = c.pending.swap(0, Relaxed);
        assert_eq!(drained, 5);
        assert_eq!(c.pending.load(Relaxed), 0);
    }
}
