//! TX-path latency compensation. Decomposed into the two clock domains
//! the TX chain crosses (TIM4 CC2 match -> wire start-bit edge):
//!
//!   TX_COMP_TICKS = TX_PIPELINE_TICKS
//!                   + (TX_BIT_TIMES_Q4 x brr) / 16
//!
//! `TX_PIPELINE_TICKS` covers the HCLK-domain pipeline:
//!   TIM4 CC2 -> DMA1_CH4 stamp -> DMA1_CH2.CR (EN=1) -> DMA fetch -> USART3.DR
//! Fixed HCLK cycles + AHB arbitration; no baud dependency.
//!
//! `TX_BIT_TIMES_Q4` covers the USART bit-clock-domain pipeline in Q4
//! (16 = 1.0 x brr). After the DR write, the byte waits 0..brr HCLK for
//! the next bit-clock edge to load TSR; TSR loads on that edge and the
//! start bit (high->low) falls at the same instant. IC1 captures the
//! falling edge, so the latency we compensate is exactly the DR->edge
//! wait -- U[0, brr] per shot, mean 0.5 x brr -> Q4 = 8.
//!
//! Both atoms are runtime-tunable via the `COMP` protocol command so
//! bench can write measured values back without a firmware rebuild.
//! `TX_COMP_TICKS` is the precomputed sum read by the send fast path --
//! recomputed by `recompute` whenever brr or either tunable changes.

use ch32_metapac::USART3;
use portable_atomic::{AtomicU32, Ordering};

/// Defaults are the converged values from `tool-pirate-tune --stage
/// tx-comp` on osc-dev-v20x at 144 MHz HCLK (median across 5 runs at
/// n=64 shots/baud; per-run wobble +/-~5 ticks on pipe, +/-1 on bit_q4 --
/// bounded by the U[0, brr] median SE floor).
const TX_PIPELINE_TICKS_DEFAULT: u32 = 45;
const TX_BIT_TIMES_Q4_DEFAULT: u32 = 8;

static TX_COMP_TICKS: AtomicU32 = AtomicU32::new(0);
static TX_PIPELINE_TICKS: AtomicU32 = AtomicU32::new(TX_PIPELINE_TICKS_DEFAULT);
static TX_BIT_TIMES_Q4: AtomicU32 = AtomicU32::new(TX_BIT_TIMES_Q4_DEFAULT);

#[inline]
pub(super) fn load() -> u32 {
    TX_COMP_TICKS.load(Ordering::Relaxed)
}

pub(super) fn recompute(brr: u32) {
    let pipe = TX_PIPELINE_TICKS.load(Ordering::Relaxed);
    let q4 = TX_BIT_TIMES_Q4.load(Ordering::Relaxed);
    TX_COMP_TICKS.store(pipe + ((q4 * brr) >> 4), Ordering::Relaxed);
}

pub fn tx_comp() -> (u32, u32) {
    (
        TX_PIPELINE_TICKS.load(Ordering::Relaxed),
        TX_BIT_TIMES_Q4.load(Ordering::Relaxed),
    )
}

/// Update one or both comp tunables, then recompute `TX_COMP_TICKS` for
/// the current baud. Held under a critical section so a send racing the
/// update can't see a half-updated comp.
pub fn set_tx_comp(pipe: Option<u32>, bit_q4: Option<u32>) {
    critical_section::with(|_| {
        if let Some(p) = pipe {
            TX_PIPELINE_TICKS.store(p, Ordering::Relaxed);
        }
        if let Some(q) = bit_q4 {
            TX_BIT_TIMES_Q4.store(q, Ordering::Relaxed);
        }
        let brr = USART3.brr().read().0;
        recompute(brr);
    });
}
