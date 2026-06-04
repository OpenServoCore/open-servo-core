//! Single-slot IDLE wire-end anchor. Master-driven timing keeps IDLEs hundreds
//! of µs apart, so single-slot suffices — no ring needed. ISR writes via
//! seqlock (even == stable, odd == writer mid-update); `seq` also doubles as
//! the freshness sentinel readers compare against.

use core::sync::atomic::Ordering;
use portable_atomic::AtomicU32;

#[derive(Copy, Clone)]
pub struct IdleAnchor {
    pub seq: u32,
    pub bytes: u32,
    pub tick: u32,
}

impl IdleAnchor {
    pub const fn empty() -> Self {
        Self {
            seq: 0,
            bytes: 0,
            tick: 0,
        }
    }
}

static SEQ: AtomicU32 = AtomicU32::new(0);
static BYTES: AtomicU32 = AtomicU32::new(0);
static TICK: AtomicU32 = AtomicU32::new(0);
static CUMULATIVE_BYTES: AtomicU32 = AtomicU32::new(0);

/// `delta_bytes` is DMA bytes since the previous IDLE; `tick` is the backdated
/// wire-end SysTick.
pub fn record(delta_bytes: u16, tick: u32) {
    let new_bytes = CUMULATIVE_BYTES
        .load(Ordering::Relaxed)
        .wrapping_add(delta_bytes as u32);
    CUMULATIVE_BYTES.store(new_bytes, Ordering::Relaxed);

    let seq = SEQ.load(Ordering::Relaxed);
    SEQ.store(seq.wrapping_add(1), Ordering::Release);
    BYTES.store(new_bytes, Ordering::Relaxed);
    TICK.store(tick, Ordering::Relaxed);
    SEQ.store(seq.wrapping_add(2), Ordering::Release);
}

/// Always returns a value; caller compares `seq` against their previously
/// consumed value to decide freshness.
pub fn snapshot() -> IdleAnchor {
    loop {
        let seq1 = SEQ.load(Ordering::Acquire);
        if seq1 & 1 != 0 {
            continue;
        }
        let bytes = BYTES.load(Ordering::Relaxed);
        let tick = TICK.load(Ordering::Relaxed);
        let seq2 = SEQ.load(Ordering::Acquire);
        if seq1 == seq2 {
            return IdleAnchor {
                seq: seq1,
                bytes,
                tick,
            };
        }
    }
}
