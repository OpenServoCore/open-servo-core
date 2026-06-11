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
    /// Packet first-byte tick captured at IDLE-handler entry. Frozen here
    /// so a later EXTI fire from the *next* packet can't overwrite the
    /// snoop view; the writer pairs it with `first_valid` inside the same
    /// seqlock so the snapshot is atomic.
    pub first_tick: u32,
    pub first_valid: bool,
    /// Bench-side instrumentation: last EXTI fire's tick + total fire
    /// count between IDLE handler invocations. `exti_fires > 1` means a
    /// glitch retriggered EXTI inside the packet; comparing `last_tick`
    /// to `first_tick` shows the glitch's edge time.
    pub last_tick: u32,
    pub exti_fires: u32,
}

impl IdleAnchor {
    pub const fn empty() -> Self {
        Self {
            seq: 0,
            bytes: 0,
            tick: 0,
            first_tick: 0,
            first_valid: false,
            last_tick: 0,
            exti_fires: 0,
        }
    }
}

static SEQ: AtomicU32 = AtomicU32::new(0);
static BYTES: AtomicU32 = AtomicU32::new(0);
static TICK: AtomicU32 = AtomicU32::new(0);
static FIRST_TICK: AtomicU32 = AtomicU32::new(0);
static FIRST_VALID: AtomicU32 = AtomicU32::new(0);
static LAST_TICK: AtomicU32 = AtomicU32::new(0);
static EXTI_FIRES: AtomicU32 = AtomicU32::new(0);
static CUMULATIVE_BYTES: AtomicU32 = AtomicU32::new(0);

/// `delta_bytes` is DMA bytes since the previous IDLE; `tick` is the backdated
/// wire-end SysTick. `first_tick` / `first_valid` are the IDLE-handler-side
/// snapshot of [`crate::dxl::statics::DXL_RX_FIRST_TICK`] /
/// [`crate::dxl::statics::DXL_RX_FIRST_VALID`] — freeze them here so a
/// later-packet EXTI fire can't poison the snoop measurement. `last_tick` /
/// `exti_fires` snapshot the per-cycle glitch counters for the bench log.
pub fn record(
    delta_bytes: u16,
    tick: u32,
    first_tick: u32,
    first_valid: bool,
    last_tick: u32,
    exti_fires: u32,
) {
    let new_bytes = CUMULATIVE_BYTES
        .load(Ordering::Relaxed)
        .wrapping_add(delta_bytes as u32);
    CUMULATIVE_BYTES.store(new_bytes, Ordering::Relaxed);

    let seq = SEQ.load(Ordering::Relaxed);
    SEQ.store(seq.wrapping_add(1), Ordering::Release);
    BYTES.store(new_bytes, Ordering::Relaxed);
    TICK.store(tick, Ordering::Relaxed);
    FIRST_TICK.store(first_tick, Ordering::Relaxed);
    FIRST_VALID.store(first_valid as u32, Ordering::Relaxed);
    LAST_TICK.store(last_tick, Ordering::Relaxed);
    EXTI_FIRES.store(exti_fires, Ordering::Relaxed);
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
        let first_tick = FIRST_TICK.load(Ordering::Relaxed);
        let first_valid = FIRST_VALID.load(Ordering::Relaxed) != 0;
        let last_tick = LAST_TICK.load(Ordering::Relaxed);
        let exti_fires = EXTI_FIRES.load(Ordering::Relaxed);
        let seq2 = SEQ.load(Ordering::Acquire);
        if seq1 == seq2 {
            return IdleAnchor {
                seq: seq1,
                bytes,
                tick,
                first_tick,
                first_valid,
                last_tick,
                exti_fires,
            };
        }
    }
}
