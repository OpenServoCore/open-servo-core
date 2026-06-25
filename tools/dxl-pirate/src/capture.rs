//! Wire-side capture: per-byte stamp pipeline per TIMING.md §3.
//!
//! Two DMA-populated buffers + a walker-populated stamp ring:
//!
//! - `falling_ring` (u16) ← TIM2_CH3 IC via DMA1_CH1; raw 16-bit captures.
//! - `rx_ring` (u8) ← USART3 DATAR via DMA1_CH3; received bytes
//!   (internal staging; not host-facing).
//! - `bytes_ring` (u8) ← walker; per-byte value captured at stamp time.
//! - `ts_ring` (u32) ← walker; per-byte start tick, lifted to tick32.
//! - `flags_ring` (u8) ← walker; per-byte anomaly bits.
//!
//! `bytes_ring`/`ts_ring`/`flags_ring` are parallel arrays indexed by the
//! same cumulative byte counter. Walker invariant: byte_head ≤ rx_total.
//! The byte VALUE is mirrored into `bytes_ring` at stamp time so drain
//! semantics are decoupled from `rx_ring`'s DMA wrap.
//!
//! Walker triggers (TIMING.md §3.2): event-driven, no cadence. Three IRQ
//! vectors fan into `walk()` at `PRIO_WALKER`:
//!
//! - `USART3` IDLE — end-of-burst drain. Fires 1 char time after the
//!   wire goes idle (~10 µs at 1M, ~3.3 µs at 3M). Catches the tail
//!   bytes of every reply; this is the path that fixes "last byte
//!   missing" symptoms the cadence walker had at packet boundaries.
//! - `DMA1_CHANNEL1` HT/TC — mid-burst drain when `falling_ring` fills.
//! - `DMA1_CHANNEL3` HT/TC — mid-burst drain when `rx_ring` fills.
//!
//! All three sources share `PRIO_WALKER` so they cannot preempt one
//! another; `walk()` runs single-threaded across them. Lift_ceiling is
//! a `read_tick32()` snapshot taken AFTER `refresh_falling_total` so
//! every IC entry the walker processes has `entry.lo ≤ lift_ceiling.lo`
//! (TIMING.md §3.3).
//!
//! Classification is **predict-and-snap PLL** per TIMING.md §3.4: the
//! first byte's start tick is anchored on the first unconsumed IC edge;
//! every subsequent byte's start is predicted at `prev_anchor + 10·bit`
//! and snapped to the closest IC entry within `±SNAP_BITS·bit` of that
//! prediction (later-edge tiebreak). On miss, the walker free-runs on
//! the prediction and flags `COUNT_UNDER` — anchor still updates, so the
//! prediction chain survives a short edge dropout.
//!
//! Per TIMING.md §3.5: three conditions flip the sticky `DESYNCED` flag —
//! two designed-impossible (`walker_late`, `ic_overrun`) and one host-paced
//! bench-script bug (`stamp_overflow`). Host commands error out until a
//! `RESET` (or `BAUD`, which implicitly resets) clears the flag.

use core::cell::SyncUnsafeCell;
use core::ptr;

use portable_atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use ch32_hal::pac::dma::vals::{Dir, Pl, Size};
use ch32_hal::pac::timer::vals::FilterValue;
use ch32_hal::pac::{DMA1, TIM2, USART3};
use dxl_pirate::parse::brr_for;
use qingke_rt::interrupt;

use crate::inject::{APB1_HZ, BaudError, DEFAULT_BAUD, read_tick32};

// IC ring per TIMING.md §3.1. With event-driven walker, DMA1_CH1 HT fires
// at FALL_LEN/2 entries — the walker drains promptly without waiting for
// a cadence tick. 256 gives ample slack for any sustained burst (3 Mbaud
// × 5 edges/byte → 1 edge per 667 ns; HT-to-drain latency is bounded by
// PFIC dispatch + walker run, both sub-µs at PRIO_WALKER).
const FALL_LEN: usize = 256;
const FALL_MASK: u32 = (FALL_LEN - 1) as u32;
const _: () = assert!(FALL_LEN.is_power_of_two());

const RX_LEN: usize = 256;
const RX_MASK: u32 = (RX_LEN - 1) as u32;
const _: () = assert!(RX_LEN.is_power_of_two());

/// Stamp ring depth. Sized to absorb realistic measurement bursts (host
/// pulls via BBATCH between fires) without overflowing while leaving 5+ KB
/// stack headroom on the V203's 20 KB SRAM. At 1024 records the three
/// rings total 6 KB (`bytes_ring` 1 KB + `ts_ring` 4 KB + `flags_ring`
/// 1 KB) — projected bss ~14.9 KB, ~5.5 KB stack room. Per TIMING.md
/// §3.5 `stamp_overflow`: past 1024 records of undrained backlog the
/// host has either gone away or has a bench-script bug; either way the
/// `DESYNCED` flag trips and host must `RESET`.
const STAMP_LEN: usize = 1024;
const STAMP_MASK: u32 = (STAMP_LEN - 1) as u32;
const _: () = assert!(STAMP_LEN.is_power_of_two());

/// Half-width of the PLL snap window in bit-times. Each byte's start is
/// predicted at `prev_anchor + 10·bit_ticks`; the walker scans
/// `[predicted − SNAP_BITS·bit_ticks, predicted + SNAP_BITS·bit_ticks]`
/// for the closest IC entry. 3 was the value that bottomed out the
/// off-line classifier comparison against the captured corpus — wide
/// enough to absorb upstream chips' worst-case 3-bit hardware idle
/// between bytes inside a Status burst, narrow enough that wire glitches
/// rarely land closer to prediction than the real start edge. Tighten
/// to 2 (or even 1) once we have wire-side telemetry showing inter-byte
/// gaps are systematically smaller in real operation.
const SNAP_BITS: u32 = 3;

/// Per-baud CC filter LUT per TIMING.md §4. fDTS pinned at HCLK = 144 MHz
/// (CKD=`DIV_1`, set in `inject::init`). Each entry is `(ICxF bits, filter
/// delay in tick32 ticks)` for every legal `ICxF` value, sorted by delay.
/// `filter_for_brr` picks the **largest delay ≤ brr/3 (≈ 0.333·bit)** at the
/// configured baud — tighter than production's natural pick because the
/// bench wiring (PB10 AF-OD + PB11 ~30 kΩ internal pull-up, no transceiver)
/// has τ ≈ 500–900 ns line rise. A 1-bit-time HIGH gap between two
/// consecutive falls (every byte whose predecessor has `b7=0`) only leaves
/// ~400 ns of stable HIGH for the filter to recognize before the next
/// fall, so the delay must fit inside that.
///
/// Each entry's delay = N / fSAMPLING, scaled to 144 MHz ticks. RM
/// `CH32FV2x_V3xRM` §14.4.7 ICxF table.
const ICF_LUT: &[(u8, u32)] = &[
    (0b0001, 2),   // fCK_INT,    N=2
    (0b0010, 4),   // fCK_INT,    N=4
    (0b0011, 8),   // fCK_INT,    N=8
    (0b0100, 12),  // fDTS/2,     N=6
    (0b0101, 16),  // fDTS/2,     N=8
    (0b0110, 24),  // fDTS/4,     N=6
    (0b0111, 32),  // fDTS/4,     N=8
    (0b1000, 48),  // fDTS/8,     N=6
    (0b1001, 64),  // fDTS/8,     N=8
    (0b1010, 80),  // fDTS/16,    N=5
    (0b1011, 96),  // fDTS/16,    N=6
    (0b1100, 128), // fDTS/16,    N=8
    (0b1101, 160), // fDTS/32,    N=5
    (0b1110, 192), // fDTS/32,    N=6
    (0b1111, 256), // fDTS/32,    N=8
];

/// Largest LUT entry whose delay is ≤ `brr/3` (≈ 0.333·bit). The earlier
/// "strictly less than `brr`" and "≤ 2·brr/3" rules ate edges
/// wire-empirically: at 1 Mbaud (brr=144) with the weak internal pull-up
/// on PB11, the line takes ~400–600 ns to cross the Schmitt threshold
/// after OD release, leaving only ~400 ns of stable HIGH inside a 1-bit
/// inter-byte gap. The filter needs `delay` ns of stable HIGH to confirm
/// state before it can detect the next fall, so `delay > 400 ns` loses
/// anchors on every byte whose predecessor has b7=0. `brr/3` puts the
/// delay safely under that window at every baud and still rejects
/// glitches narrower than ~1/3·bit, which is well below any real wire
/// noise. Linear walk is fine — const-evaluated at every set_baud site,
/// and the LUT has 15 entries.
const fn filter_for_brr(brr: u32) -> (u8, u32) {
    let mut best = ICF_LUT[0];
    let mut i = 0;
    while i < ICF_LUT.len() {
        let (bits, delay) = ICF_LUT[i];
        if delay * 3 <= brr && delay >= best.1 {
            best = (bits, delay);
        }
        i += 1;
    }
    best
}

/// Current CC filter delay in ticks, written by `set_baud` alongside the
/// IC3F register update. Walker subtracts this from each IC stamp so the
/// stored tick reads as wire-edge time, not filter-output time.
static CC_FILTER_DELAY_TICKS: AtomicU32 = AtomicU32::new(0);

#[derive(Copy, Clone)]
pub struct ByteRecord {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

pub mod flags {
    /// Walker predicted this byte's start at `prev_anchor + 10·bit_ticks`
    /// but found no IC entry in `±SNAP_BITS·bit_ticks` of the prediction;
    /// emitted `tick = predicted − cc_filter_delay` instead. Cause: real
    /// start edge eaten by CC filter, upstream chip drove an inter-byte
    /// gap exceeding the snap window, or pirate snapped on the wrong edge
    /// the byte before. Host treats this byte's `tick` as ±0.5·bit
    /// accurate, not sub-tick.
    pub const COUNT_UNDER: u8 = 1 << 0;
}

/// Sticky-fatal cause for the `DESYNCED` flag (TIMING.md §3.5). Stored
/// as `u8` in `DESYNC_CAUSE`; first trip wins (compare_exchange semantics).
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DesyncCause {
    WalkerLate = 1,
    IcOverrun = 2,
    StampOverflow = 3,
}

impl DesyncCause {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::WalkerLate),
            2 => Some(Self::IcOverrun),
            3 => Some(Self::StampOverflow),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::WalkerLate => "walker_late",
            Self::IcOverrun => "ic_overrun",
            Self::StampOverflow => "stamp_overflow",
        }
    }
}

// ── DMA-populated buffers ──────────────────────────────────────────────
static FALLING_RING: SyncUnsafeCell<[u16; FALL_LEN]> = SyncUnsafeCell::new([0; FALL_LEN]);
static RX_RING: SyncUnsafeCell<[u8; RX_LEN]> = SyncUnsafeCell::new([0; RX_LEN]);

// ── Walker-populated rings ─────────────────────────────────────────────
// `bytes_ring` mirrors the byte value at stamp time, decoupling drain from
// `rx_ring`'s DMA wrap. Without this mirror, a host that drains long after
// stamping reads through to `rx_ring[i & RX_MASK]` which DMA has since
// overwritten with a later byte → silent value corruption.
static BYTES_RING: SyncUnsafeCell<[u8; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);
static TS_RING: SyncUnsafeCell<[u32; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);
static FLAGS_RING: SyncUnsafeCell<[u8; STAMP_LEN]> = SyncUnsafeCell::new([0; STAMP_LEN]);

/// Cumulative count of bytes the walker has stamped (= ts_head). Read by
/// host-side drainers with Acquire to see the matching ring writes.
static BYTE_HEAD: AtomicU32 = AtomicU32::new(0);
/// Cumulative count of bytes the host has consumed.
static BYTE_TAIL: AtomicU32 = AtomicU32::new(0);

// ── Walker state ───────────────────────────────────────────────────────
// All written from ISR context only; single-threaded across IRQs at the
// same priority. Per TIMING.md §3.4 the walker's surface is the IC/RX
// position counters, the PLL anchor pair, and the `DESYNCED` sticky flag.
static WALKED_FALLING: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static FALLING_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LAST_FALLING_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(FALL_LEN as u16);

/// Last emitted byte's start tick (lifted u32). Source of the prediction
/// chain — next byte's start is `LAST_ANCHOR + 10·bit_ticks` ± SNAP_BITS·
/// bit_ticks. Meaningless when `HAS_ANCHOR == false`.
static LAST_ANCHOR: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
/// `false` at cold boot, after `reset_walker`, and after every `set_baud`.
/// First post-reset byte anchors on the next unconsumed IC entry (no
/// prediction yet); subsequent bytes use the PLL with `LAST_ANCHOR`.
static HAS_ANCHOR: SyncUnsafeCell<bool> = SyncUnsafeCell::new(false);

static RX_TOTAL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static LAST_RX_NDTR: SyncUnsafeCell<u16> = SyncUnsafeCell::new(RX_LEN as u16);

/// Sticky-fatal flag per TIMING.md §3.5. Set once on any of three
/// conditions: multi-cadence-flag walker entry (`walker_late`), IC ring
/// overrun (`ic_overrun`), or stamp ring overflow from host backpressure
/// (`stamp_overflow`). Once set, `walk()` is a no-op and host commands
/// return `ERR desync <cause>` until a `RESET` (or `BAUD`, which
/// implicitly resets) clears the flag.
static DESYNCED: AtomicBool = AtomicBool::new(false);
/// Sticky cause tag for the `DESYNCED` trip. `0` = not desynced; first
/// trip wins via `compare_exchange` so the cause reflects the originating
/// failure even if a later condition would also trip.
static DESYNC_CAUSE: AtomicU8 = AtomicU8::new(0);

/// Bit-time in tick32 ticks. Refreshed on `set_baud`. brr at HCLK = ticks
/// per bit (USART3 sits on APB1 which runs at HCLK).
static BIT_TICKS: AtomicU32 = AtomicU32::new(0);

// ── Walker ISR trace ring ──────────────────────────────────────────────
// One record per walker invocation, drained by host `BTRACE`. Diagnoses
// stochastic walker behaviour that doesn't trip the `DESYNCED` flag:
// trigger-source tag (`phase`), TIM2.CNT entry/exit (ISR latency +
// walker duration), and pre-walk falling_pending + edges/bytes deltas
// (per-invocation workload).

pub const TRACE_PHASE_IDLE: u8 = 0;
pub const TRACE_PHASE_RX_HT: u8 = 1;
pub const TRACE_PHASE_RX_TC: u8 = 2;
pub const TRACE_PHASE_IC_HT: u8 = 3;
pub const TRACE_PHASE_IC_TC: u8 = 4;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct WalkerTrace {
    pub phase: u8,
    /// INTFR re-read **after** the clear write; non-zero `UIF/CC1IF/CC2IF/
    /// CC4IF` bits here mean a cadence flag latched in the read→clear gap
    /// and was silently dropped — direct evidence of the race the previous
    /// agent suspected.
    pub intfr_post_clear: u8,
    pub tim2_cnt_entry: u16,
    pub tim2_cnt_exit: u16,
    /// `falling_total - walked` at ISR entry — how deep the IC ring was
    /// when the walker arrived. Computed from a live NDTR read so it
    /// reflects edges captured between the previous walker exit and this
    /// entry, not just the stale `FALLING_TOTAL` from the last refresh.
    pub falling_pending_entry: u16,
    pub edges_consumed: u8,
    pub bytes_emitted: u8,
    /// Cumulative IC edges captured at ISR entry. Compare to host-side
    /// expected edges-per-byte sum to see whether missing edges were lost
    /// upstream (CC filter / DMA) or downstream (walker classification).
    pub falling_total: u32,
    /// Cumulative RX bytes received at ISR entry.
    pub rx_total: u32,
}

const TRACE_LEN: usize = 64;
const TRACE_MASK: u32 = (TRACE_LEN - 1) as u32;
const _: () = assert!(TRACE_LEN.is_power_of_two());
const TRACE_ZERO: WalkerTrace = WalkerTrace {
    phase: 0,
    intfr_post_clear: 0,
    tim2_cnt_entry: 0,
    tim2_cnt_exit: 0,
    falling_pending_entry: 0,
    edges_consumed: 0,
    bytes_emitted: 0,
    falling_total: 0,
    rx_total: 0,
};

static TRACE_RING: SyncUnsafeCell<[WalkerTrace; TRACE_LEN]> =
    SyncUnsafeCell::new([TRACE_ZERO; TRACE_LEN]);
static TRACE_HEAD: AtomicU32 = AtomicU32::new(0);
static TRACE_TAIL: AtomicU32 = AtomicU32::new(0);

#[inline]
fn trace_push(rec: WalkerTrace) {
    let head = TRACE_HEAD.load(Ordering::Relaxed);
    let idx = (head & TRACE_MASK) as usize;
    unsafe {
        (*TRACE_RING.get())[idx] = rec;
    }
    TRACE_HEAD.store(head.wrapping_add(1), Ordering::Release);
}

pub fn trace_drain() -> Option<WalkerTrace> {
    let tail = TRACE_TAIL.load(Ordering::Relaxed);
    let head = TRACE_HEAD.load(Ordering::Acquire);
    if tail == head {
        return None;
    }
    // Drop overrun: if more than TRACE_LEN unconsumed, snap tail forward
    // to the oldest still-in-ring record. Records get overwritten head-side
    // when the host falls behind.
    let avail = head.wrapping_sub(tail);
    let effective_tail = if (avail as usize) > TRACE_LEN {
        head.wrapping_sub(TRACE_LEN as u32)
    } else {
        tail
    };
    let idx = (effective_tail & TRACE_MASK) as usize;
    let rec = unsafe { (*TRACE_RING.get())[idx] };
    TRACE_TAIL.store(effective_tail.wrapping_add(1), Ordering::Release);
    Some(rec)
}

pub fn trace_clear() {
    critical_section::with(|_| {
        let head = TRACE_HEAD.load(Ordering::Relaxed);
        TRACE_TAIL.store(head, Ordering::Release);
    });
}

pub fn init() {
    unsafe {
        // DMA1_CH1 = TIM2_CH3 IC. Circular, 16-bit. HTIE+TCIE drive the
        // walker when the IC ring half- or fully-fills (TIMING.md §3.2).
        let ch1 = DMA1.ch(0);
        ch1.par().write_value(TIM2.chcvr(2).as_ptr() as u32);
        ch1.mar().write_value((*FALLING_RING.get()).as_ptr() as u32);
        ch1.ndtr().write(|w| w.set_ndt(FALL_LEN as u16));
        ch1.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_pl(Pl::VERYHIGH); // measurement clock — never queue
            w.set_htie(true);
            w.set_tcie(true);
            w.set_en(true);
        });

        // DMA1_CH3 = USART3_RX. Circular, 8-bit. HTIE+TCIE drive the
        // walker when the RX ring half- or fully-fills.
        let ch3 = DMA1.ch(2);
        ch3.par().write_value(USART3.datar().as_ptr() as u32);
        ch3.mar().write_value((*RX_RING.get()).as_ptr() as u32);
        ch3.ndtr().write(|w| w.set_ndt(RX_LEN as u16));
        ch3.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_pl(Pl::HIGH); // RX must not drop bytes at 3 Mbaud
            w.set_htie(true);
            w.set_tcie(true);
            w.set_en(true);
        });

        BIT_TICKS.store(APB1_HZ / DEFAULT_BAUD, Ordering::Relaxed);
        apply_filter_for_brr(APB1_HZ / DEFAULT_BAUD);
        // Drop any stray IC entries deposited during USART3/PB10 init
        // transients. Without this, the first MASTER's anchor falls on a
        // stale boot-glitch edge in falling_ring.
        reset_walker();
    }
}

/// Reconfigure TIM2 IC3F for the given USART BRR and update the matching
/// `CC_FILTER_DELAY_TICKS`. Live write per V20x RM §14.4.7 — IC3F has no
/// "channel must be disabled" restriction (unlike CC3S).
fn apply_filter_for_brr(brr: u32) {
    let (icf_bits, delay_ticks) = filter_for_brr(brr);
    TIM2.chctlr_input(1).modify(|w| {
        w.set_icf(0, FilterValue::from_bits(icf_bits));
    });
    CC_FILTER_DELAY_TICKS.store(delay_ticks, Ordering::Release);
}

/// Reconfigure USART3 BIT_TICKS reference. Caller already retuned the
/// USART itself via `inject::set_baud`; this just keeps the walker's bit
/// window aligned with the new baud.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    BIT_TICKS.store(brr, Ordering::Relaxed);
    apply_filter_for_brr(brr);
    // A baud change with bytes mid-flight produces garbage anyway; drop
    // any pending IC entries so the next byte anchors on a fresh edge.
    reset_walker();
    Ok(())
}

/// Drop all in-flight walker state up to current DMA heads and clear
/// `DESYNCED` + cause. Called from init, `set_baud`, and the host `RESET`
/// command. The IC entries / RX bytes already in the rings are presumed
/// stale at these points (boot glitches, baud-bounce transients, leftover
/// packets, or the bytes that overflowed the stamp ring) — pulling the
/// walker counters forward to the DMA heads ensures the next byte
/// anchors on a fresh edge.
pub fn reset_walker() {
    critical_section::with(|_| unsafe {
        let falling_total = refresh_falling_total();
        let rx_total = refresh_rx_total();
        ptr::write_volatile(WALKED_FALLING.get(), falling_total);
        // Drop the PLL chain. The next byte the walker emits will anchor
        // on its own IC entry (no prediction) — same cold-start path as
        // boot, so post-reset behaviour matches first-packet behaviour.
        ptr::write_volatile(HAS_ANCHOR.get(), false);
        BYTE_HEAD.store(rx_total, Ordering::Release);
        BYTE_TAIL.store(rx_total, Ordering::Release);
        DESYNC_CAUSE.store(0, Ordering::Release);
        DESYNCED.store(false, Ordering::Release);
    });
}

/// First-trip-wins cause setter. `compare_exchange` keeps the originating
/// cause for diagnostic clarity if a later trip would also fire.
#[inline]
fn set_desync(cause: DesyncCause) {
    let _ = DESYNC_CAUSE.compare_exchange(0, cause as u8, Ordering::AcqRel, Ordering::Relaxed);
    DESYNCED.store(true, Ordering::Release);
}

/// Returns `Some(cause)` once any of the three trip conditions has fired,
/// `None` while READY. Recovery from desync is `RESET` (TIMING.md §3.5).
pub fn desync_cause() -> Option<DesyncCause> {
    let raw = DESYNC_CAUSE.load(Ordering::Acquire);
    DesyncCause::from_u8(raw)
}

/// Stamps queued for drain (= `byte_head - byte_tail`). Reported by
/// STATUS so the host can pace its BBATCH calls against the ring depth.
pub fn stamps_available() -> u32 {
    let head = BYTE_HEAD.load(Ordering::Acquire);
    let tail = BYTE_TAIL.load(Ordering::Relaxed);
    head.wrapping_sub(tail)
}

/// Configured baud (= `APB1_HZ / BIT_TICKS`). Returns 0 before the first
/// `set_baud` completes. Used by STATUS.
pub fn current_baud() -> u32 {
    APB1_HZ
        .checked_div(BIT_TICKS.load(Ordering::Relaxed))
        .unwrap_or(0)
}

pub fn drain_byte() -> Option<ByteRecord> {
    let tail = BYTE_TAIL.load(Ordering::Relaxed);
    let head = BYTE_HEAD.load(Ordering::Acquire);
    if tail == head {
        return None;
    }
    let idx = (tail & STAMP_MASK) as usize;
    let rec = unsafe {
        ByteRecord {
            tick: (*TS_RING.get())[idx],
            byte: (*BYTES_RING.get())[idx],
            flags: (*FLAGS_RING.get())[idx],
        }
    };
    BYTE_TAIL.store(tail.wrapping_add(1), Ordering::Release);
    Some(rec)
}

/// Drain up to `out.len()` byte records into `out`. Returns the count
/// actually written. Cheaper per-byte than `drain_byte` for the BBATCH
/// host transport.
pub fn drain_batch(out: &mut [ByteRecord]) -> usize {
    let tail0 = BYTE_TAIL.load(Ordering::Relaxed);
    let head = BYTE_HEAD.load(Ordering::Acquire);
    let avail = head.wrapping_sub(tail0) as usize;
    let n = avail.min(out.len());
    for (i, slot) in out.iter_mut().take(n).enumerate() {
        let idx = (tail0.wrapping_add(i as u32) & STAMP_MASK) as usize;
        *slot = unsafe {
            ByteRecord {
                tick: (*TS_RING.get())[idx],
                byte: (*BYTES_RING.get())[idx],
                flags: (*FLAGS_RING.get())[idx],
            }
        };
    }
    if n > 0 {
        BYTE_TAIL.store(tail0.wrapping_add(n as u32), Ordering::Release);
    }
    n
}

// ── Walker ─────────────────────────────────────────────────────────────

#[inline]
fn refresh_falling_total() -> u32 {
    let ndtr = DMA1.ch(0).ndtr().read().ndt();
    unsafe {
        let prev = *LAST_FALLING_NDTR.get();
        *LAST_FALLING_NDTR.get() = ndtr;
        let consumed = prev.wrapping_sub(ndtr) as u32 & FALL_MASK;
        let total = ptr::read_volatile(FALLING_TOTAL.get()).wrapping_add(consumed);
        ptr::write_volatile(FALLING_TOTAL.get(), total);
        total
    }
}

#[inline]
fn refresh_rx_total() -> u32 {
    let ndtr = DMA1.ch(2).ndtr().read().ndt();
    unsafe {
        let prev = *LAST_RX_NDTR.get();
        *LAST_RX_NDTR.get() = ndtr;
        let consumed = prev.wrapping_sub(ndtr) as u32 & RX_MASK;
        let total = ptr::read_volatile(RX_TOTAL.get()).wrapping_add(consumed);
        ptr::write_volatile(RX_TOTAL.get(), total);
        total
    }
}

/// Lift a 16-bit IC capture to 32-bit using single-wrap modular delta.
/// `lift_ceiling` is `read_tick32()` taken AFTER `refresh_falling_total`
/// at walker entry (TIMING.md §3.3). The snapshot ordering guarantees
/// `lift_ceiling.lo ≥ entry.lo` for every IC entry the walker processes,
/// so each entry lifts into the correct wrap. Mirrors `edge_parser::lift`
/// in production firmware.
#[inline]
fn lift(lift_ceiling: u32, entry: u16) -> u32 {
    let delta = (lift_ceiling as u16).wrapping_sub(entry) as u32;
    lift_ceiling.wrapping_sub(delta)
}

#[inline]
fn falling_at(idx: u32) -> u16 {
    unsafe { (*FALLING_RING.get())[(idx & FALL_MASK) as usize] }
}

#[inline]
fn rx_at(idx: u32) -> u8 {
    unsafe { (*RX_RING.get())[(idx & RX_MASK) as usize] }
}

fn emit(byte_idx: u32, byte: u8, tick: u32, flags: u8) {
    let i = (byte_idx & STAMP_MASK) as usize;
    unsafe {
        // Mirror the byte value at stamp time so drain semantics are
        // decoupled from `rx_ring`'s DMA wrap. The stamp record
        // `(bytes[i], ts[i], flags[i])` is self-contained.
        (*BYTES_RING.get())[i] = byte;
        (*TS_RING.get())[i] = tick;
        (*FLAGS_RING.get())[i] = flags;
    }
}

/// Predict-and-snap PLL walker per TIMING.md §3.4. Constructs
/// `lift_ceiling` internally: refresh the IC NDTR snapshot first, THEN
/// read `tick32`. Ordering guarantees `lift_ceiling.lo ≥ entry.lo` for
/// every entry in [walked, falling_total) — newer entries that land
/// between the NDTR refresh and the tick32 read aren't in this walker's
/// snapshot, so they fall to the next trigger.
pub fn walk() {
    if DESYNCED.load(Ordering::Relaxed) {
        return;
    }
    let bit_ticks = BIT_TICKS.load(Ordering::Relaxed);
    if bit_ticks == 0 {
        return;
    }
    let byte_period = 10u32.wrapping_mul(bit_ticks);
    let snap = SNAP_BITS.wrapping_mul(bit_ticks);
    let cc_filter_delay = CC_FILTER_DELAY_TICKS.load(Ordering::Relaxed);

    let falling_total = refresh_falling_total();
    let rx_total = refresh_rx_total();
    let lift_ceiling = read_tick32();
    let mut walked = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };

    // §3.5 ic_overrun: if more new entries arrived than the ring can
    // hold, we lost edges → permanent lockstep desync. Sticky-fatal.
    if falling_total.wrapping_sub(walked) > FALL_LEN as u32 {
        set_desync(DesyncCause::IcOverrun);
        return;
    }

    let mut byte_head = BYTE_HEAD.load(Ordering::Relaxed);
    let mut anchor = unsafe { ptr::read_volatile(LAST_ANCHOR.get()) };
    let mut has_anchor = unsafe { ptr::read_volatile(HAS_ANCHOR.get()) };

    while byte_head != rx_total {
        // §3.5 stamp_overflow: if the host hasn't drained, the next emit
        // would overwrite an unread stamp. Sticky-fatal; host must RESET.
        // Check is at the loop head so byte_head is NOT advanced for the
        // byte we refused to emit.
        let byte_tail = BYTE_TAIL.load(Ordering::Acquire);
        if byte_head.wrapping_sub(byte_tail) >= STAMP_LEN as u32 {
            set_desync(DesyncCause::StampOverflow);
            break;
        }
        let byte = rx_at(byte_head);

        let chosen_anchor: u32;
        let mut flags = 0u8;

        if !has_anchor {
            // Cold-start path: post-boot, post-RESET, post-set_baud. No
            // prediction available, so anchor on the first unconsumed IC
            // entry. Need `lift_ceiling` to bound `byte_period` past
            // first_edge so we don't anchor on what is actually some
            // interior edge of the same byte whose start bit is still
            // unwalked.
            if walked == falling_total {
                break;
            }
            let first_edge = lift(lift_ceiling, falling_at(walked));
            if lift_ceiling.wrapping_sub(first_edge) < byte_period {
                break;
            }
            chosen_anchor = first_edge;
            walked = walked.wrapping_add(1);
        } else {
            // Steady-state PLL. Predict the next start bit, then snap to
            // the IC entry closest to prediction within `±snap`.
            let predicted = anchor.wrapping_add(byte_period);
            let snap_high = predicted.wrapping_add(snap);
            // Yield mid-byte: `lift_ceiling` must reach past `snap_high`
            // before we can rule out a real edge that's still in flight
            // toward the IC ring. Without this guard, an edge landing at
            // (predicted, snap_high] after the walker exits would be lost
            // — we'd have already stamped a free-run miss.
            if lift_ceiling.wrapping_sub(snap_high) > u32::MAX / 2 {
                break;
            }
            let snap_low = predicted.wrapping_sub(snap);

            // Skip any leftover prev-byte interior edges (everything
            // sitting before `snap_low` is past byte N−1's start bit but
            // before byte N's snap window).
            while walked != falling_total {
                let lifted = lift(lift_ceiling, falling_at(walked));
                if lifted.wrapping_sub(snap_low) > u32::MAX / 2 {
                    walked = walked.wrapping_add(1);
                } else {
                    break;
                }
            }

            // Scan `[snap_low, snap_high]` for the closest-to-predicted
            // edge. Tiebreak prefers the later edge: real start bits come
            // AT or AFTER prediction (upstream chip inter-byte hardware
            // idle shifts the start LATER, never earlier), so on a tie
            // the later candidate is far more likely the real start than
            // the earlier one (which is then a glitch or stuck-edge
            // ringing tail).
            let mut chosen: Option<u32> = None;
            let mut chosen_walked = walked;
            let mut best_dist = u32::MAX;
            let mut probe = walked;
            while probe != falling_total {
                let lifted = lift(lift_ceiling, falling_at(probe));
                // Past snap_high → stop scanning.
                if lifted.wrapping_sub(snap_high) <= u32::MAX / 2 && lifted != snap_high {
                    break;
                }
                let dist = if lifted >= predicted {
                    lifted.wrapping_sub(predicted)
                } else {
                    predicted.wrapping_sub(lifted)
                };
                if dist < best_dist || (dist == best_dist && lifted >= predicted) {
                    best_dist = dist;
                    chosen = Some(lifted);
                    chosen_walked = probe.wrapping_add(1);
                }
                probe = probe.wrapping_add(1);
            }

            match chosen {
                Some(c) => {
                    chosen_anchor = c;
                    walked = chosen_walked;
                }
                None => {
                    // Miss. Free-run on the prediction so the chain
                    // survives one or two dropouts. `walked` was already
                    // advanced past everything below `snap_low`, so any
                    // remaining entries (between snap_high and the next
                    // byte's snap_low) stay in the ring for the next
                    // iteration's skip pass.
                    chosen_anchor = predicted;
                    flags |= flags::COUNT_UNDER;
                }
            }
        }

        let start_tick = chosen_anchor.wrapping_sub(cc_filter_delay);
        emit(byte_head, byte, start_tick, flags);
        byte_head = byte_head.wrapping_add(1);
        anchor = chosen_anchor;
        has_anchor = true;
    }

    BYTE_HEAD.store(byte_head, Ordering::Release);
    unsafe {
        ptr::write_volatile(WALKED_FALLING.get(), walked);
        ptr::write_volatile(LAST_ANCHOR.get(), anchor);
        ptr::write_volatile(HAS_ANCHOR.get(), has_anchor);
    }
}

// ── IRQ handlers ───────────────────────────────────────────────────────

/// Shared bookkeeping for every walker-driving IRQ (IDLE + DMA HT/TC).
/// Snapshots pre-walk counters for the trace ring, runs `walk()`, then
/// pushes a record reflecting actual work performed.
#[inline]
fn run_walker(phase: u8, isr_post_bits: u8) {
    let cnt_entry = TIM2.cnt().read();
    let walked_pre = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
    let byte_head_pre = BYTE_HEAD.load(Ordering::Relaxed);
    let (falling_total_now, rx_total_now, falling_pending_entry) = {
        // Live-NDTR pending count: how many IC entries are in falling_ring
        // RIGHT NOW. Does not mutate walker state; `walk()` re-reads NDTR
        // before processing.
        let prev_total = unsafe { ptr::read_volatile(FALLING_TOTAL.get()) };
        let prev_ndtr = unsafe { *LAST_FALLING_NDTR.get() };
        let curr_ndtr = DMA1.ch(0).ndtr().read().ndt();
        let new_edges = prev_ndtr.wrapping_sub(curr_ndtr) as u32 & FALL_MASK;
        let ft = prev_total.wrapping_add(new_edges);
        let prev_rx = unsafe { ptr::read_volatile(RX_TOTAL.get()) };
        let prev_rx_ndtr = unsafe { *LAST_RX_NDTR.get() };
        let curr_rx_ndtr = DMA1.ch(2).ndtr().read().ndt();
        let new_rx = prev_rx_ndtr.wrapping_sub(curr_rx_ndtr) as u32 & RX_MASK;
        let rt = prev_rx.wrapping_add(new_rx);
        (ft, rt, ft.wrapping_sub(walked_pre))
    };

    walk();

    let walked_post = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
    let byte_head_post = BYTE_HEAD.load(Ordering::Relaxed);
    let edges = walked_post.wrapping_sub(walked_pre);
    let bytes = byte_head_post.wrapping_sub(byte_head_pre);
    // Skip zero-work traces so the 64-entry ring keeps a window of real
    // bursts rather than back-to-back no-op idle wake-ups (e.g. a spurious
    // HT firing on a quiet bus).
    if edges == 0 && bytes == 0 && falling_pending_entry == 0 {
        return;
    }
    trace_push(WalkerTrace {
        phase,
        intfr_post_clear: isr_post_bits,
        tim2_cnt_entry: cnt_entry,
        tim2_cnt_exit: TIM2.cnt().read(),
        falling_pending_entry: falling_pending_entry.min(u16::MAX as u32) as u16,
        edges_consumed: edges.min(u8::MAX as u32) as u8,
        bytes_emitted: bytes.min(u8::MAX as u32) as u8,
        falling_total: falling_total_now,
        rx_total: rx_total_now,
    });
}

/// DMA1_CH1 = falling_ring (IC) HT/TC. Drains the walker so edges don't
/// accumulate past the half-ring.
#[interrupt]
fn DMA1_CHANNEL1() {
    let isr = DMA1.isr().read();
    let tc = isr.tcif(0);
    DMA1.ifcr().write(|w| {
        w.set_htif(0, true);
        w.set_tcif(0, true);
    });
    let post = DMA1.isr().read();
    let post_bits = (post.htif(0) as u8) | ((post.tcif(0) as u8) << 1);
    let phase = if tc {
        TRACE_PHASE_IC_TC
    } else {
        TRACE_PHASE_IC_HT
    };
    run_walker(phase, post_bits);
}

/// DMA1_CH3 = rx_ring (USART3 RX) HT/TC. Drains the walker so bytes don't
/// accumulate past the half-ring.
#[interrupt]
fn DMA1_CHANNEL3() {
    let isr = DMA1.isr().read();
    let tc = isr.tcif(2);
    DMA1.ifcr().write(|w| {
        w.set_htif(2, true);
        w.set_tcif(2, true);
    });
    let post = DMA1.isr().read();
    let post_bits = (post.htif(2) as u8) | ((post.tcif(2) as u8) << 1);
    let phase = if tc {
        TRACE_PHASE_RX_TC
    } else {
        TRACE_PHASE_RX_HT
    };
    run_walker(phase, post_bits);
}

#[interrupt]
fn USART3() {
    let statr = USART3.statr().read();
    if statr.idle() {
        // STATR read above + DATAR read clears IDLE.
        let _ = USART3.datar().read();
        let idle_tick = read_tick32();
        // Drain in-flight bytes BEFORE handing off to the fire scheduler.
        // IDLE means every received byte's CC3 capture + RX DMA write has
        // landed (RX DMA decrements NDTR before the USART transitions into
        // the idle window that triggers IDLE), so the walker sees a fully
        // populated tail. This catches the last-byte-of-reply case that
        // would otherwise wait for the next DMA HT/TC trigger.
        run_walker(TRACE_PHASE_IDLE, 0);
        crate::inject::on_idle(idle_tick);
    }
    crate::led::signal();
}
