//! Wire-side capture: per-byte stamp pipeline.
//!
//! Two DMA-populated IC half-rings + an RX byte ring + a walker-populated
//! stamp ring:
//!
//! - `falling_lo` (u16) ← TIM2.CCR1 via DMA1_CH5 — low 16 of each IC tick.
//! - `falling_hi` (u16) ← TIM3.CCR1 via DMA1_CH6 — high 16, latched on
//!   the same TIM2 TRGO pulse that fired CH5. Pair forms a hardware
//!   atomic 32-bit tick; no walker-side lift step.
//! - `rx_ring` (u8) ← USART3 DATAR via DMA1_CH3; received bytes
//!   (internal staging; not host-facing).
//! - `bytes_ring` (u8) ← walker; per-byte value captured at stamp time.
//! - `ts_ring` (u32) ← walker; per-byte start tick (already tick32).
//! - `flags_ring` (u8) ← walker; per-byte anomaly bits.
//!
//! `bytes_ring`/`ts_ring`/`flags_ring` are parallel arrays indexed by the
//! same cumulative byte counter. Walker invariant: byte_head ≤ rx_total.
//! The byte VALUE is mirrored into `bytes_ring` at stamp time so drain
//! semantics are decoupled from `rx_ring`'s DMA wrap.
//!
//! Walker triggers: event-driven, no cadence. Three IRQ vectors fan into
//! `walk()` at `PRIO_WALKER`:
//!
//! - `USART3` IDLE — end-of-burst drain. Fires 1 char time after the
//!   wire goes idle (~10 µs at 1M, ~3.3 µs at 3M). Catches the tail
//!   bytes of every reply; this is the path that fixes "last byte
//!   missing" symptoms the cadence walker had at packet boundaries.
//! - `DMA1_CHANNEL6` HT/TC — mid-burst drain when the IC ring fills.
//!   CH6 (TIM3 high half) is the trailing writer of the CH5/CH6 pair,
//!   so its NDTR bounds the count of fully-written 32-bit entries.
//! - `DMA1_CHANNEL3` HT/TC — mid-burst drain when `rx_ring` fills.
//!
//! All three sources share `PRIO_WALKER` so they cannot preempt one
//! another; `walk()` runs single-threaded across them. `ceiling` is a
//! `read_tick32()` snapshot taken AFTER `refresh_falling_total`. The
//! wrap-race detection on each IC pair (peek-next monotonicity for
//! interior entries; `combined > ceiling` for the tail) handles the
//! few-cycle TRC sync window per TIM2 wrap.
//!
//! Classification is **predict-and-snap PLL**: the first byte's start
//! tick is anchored on the first unconsumed IC edge; every subsequent
//! byte's start is predicted at `prev_anchor + 10·bit` and snapped to
//! the closest IC entry within `±SNAP_BITS·bit` of that prediction
//! (later-edge tiebreak). On miss, the walker free-runs on the
//! prediction and flags `COUNT_UNDER` — anchor still updates, so the
//! prediction chain survives a short edge dropout.
//!
//! Two conditions flip the sticky `DESYNCED` flag — one designed-
//! impossible (`ic_overrun`) and one host-paced bench-script bug
//! (`stamp_overflow`). Host commands error out until a `RESET` (or
//! `BAUD`, which implicitly resets) clears the flag.

use core::cell::SyncUnsafeCell;
use core::ptr;

use portable_atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::timer::vals::FilterValue;
use ch32_metapac::{DMA1, TIM2, TIM3, USART3};
use crate::parse::brr_for;
use qingke_rt::interrupt;

use crate::tick::read_tick32;
use crate::tx::{APB1_HZ, BaudError, DEFAULT_BAUD};

// IC ring. With event-driven walker, DMA1_CH1 HT fires at FALL_LEN/2
// entries — the walker drains promptly without waiting for a cadence
// tick. 256 gives ample slack for any sustained burst (3 Mbaud × 5
// edges/byte → 1 edge per 667 ns; HT-to-drain latency is bounded by
// PFIC dispatch + walker run, both sub-µs at PRIO_WALKER).
pub const FALL_LEN: usize = 256;
const FALL_MASK: u32 = (FALL_LEN - 1) as u32;
const _: () = assert!(FALL_LEN.is_power_of_two());

const RX_LEN: usize = 256;
const RX_MASK: u32 = (RX_LEN - 1) as u32;
const _: () = assert!(RX_LEN.is_power_of_two());

/// Stamp ring depth. Sized to absorb realistic measurement bursts (host
/// pulls via BBATCH between fires) without overflowing while leaving 5+ KB
/// stack headroom on the V203's 20 KB SRAM. At 1024 records the three
/// rings total 6 KB (`bytes_ring` 1 KB + `ts_ring` 4 KB + `flags_ring`
/// 1 KB) — projected bss ~14.9 KB, ~5.5 KB stack room. Past 1024 records
/// of undrained backlog the host has either gone away or has a bench-
/// script bug; either way `stamp_overflow` trips and host must `RESET`.
const STAMP_LEN: usize = 1024;
const STAMP_MASK: u32 = (STAMP_LEN - 1) as u32;
const _: () = assert!(STAMP_LEN.is_power_of_two());

/// 8N1 wire framing: 1 start + 8 data + 1 stop = 10 bit-times per byte.
const BITS_PER_BYTE_8N1: u32 = 10;

/// Half-width of the PLL snap window in bit-times. Each byte's start is
/// predicted at `prev_anchor + BITS_PER_BYTE_8N1·bit_ticks`; the walker
/// scans `[predicted − SNAP_BITS·bit_ticks, predicted + SNAP_BITS·bit_ticks]`
/// for the closest IC entry. 1 bit-time is ~50× the observed loopback
/// jitter floor (tool-pirate-tune stage 1: dev ≤ 24 ticks at brr=2500,
/// = 0.01 bit-times). Widen to 2 or 3 if real upstream chips drive more
/// inter-byte hardware idle than 1 bit-time between bytes in a chain.
const SNAP_BITS: u32 = 1;

/// Per-baud CC filter LUT. fDTS pinned at HCLK = 144 MHz (CKD=`DIV_1`,
/// set in `tick::init`). Each entry is `(ICxF bits, filter delay in
/// tick32 ticks)` for every legal `ICxF` value, sorted by delay.
/// `filter_for_brr` picks the **largest delay ≤ brr/3 (≈ 0.333·bit)** at
/// the configured baud — tighter than production's natural pick because
/// the bench wiring (PB10 AF-OD + PB11 ~30 kΩ internal pull-up, no
/// transceiver) has τ ≈ 500–900 ns line rise. A 1-bit-time HIGH gap
/// between two consecutive falls (every byte whose predecessor has
/// `b7=0`) only leaves ~400 ns of stable HIGH for the filter to
/// recognize before the next fall, so the delay must fit inside that.
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
/// noise.
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

/// Sticky-fatal cause for the `DESYNCED` flag. Stored as `u8` in
/// `DESYNC_CAUSE`; first trip wins (compare_exchange semantics).
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum DesyncCause {
    IcOverrun = 2,
    StampOverflow = 3,
}

impl DesyncCause {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            2 => Some(Self::IcOverrun),
            3 => Some(Self::StampOverflow),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::IcOverrun => "ic_overrun",
            Self::StampOverflow => "stamp_overflow",
        }
    }
}

// ── DMA-populated buffers ──────────────────────────────────────────────
// `FALLING_LO[i]` = TIM2.CCR1 captured at IC event i (DMA1_CH5).
// `FALLING_HI[i]` = TIM3.CCR1 captured at the same TIM2 TRGO pulse via
// TRC (DMA1_CH6). The pair `(hi, lo)` reads as a hardware atomic 32-bit
// tick after wrap-race correction in `falling_at`.
static FALLING_LO: SyncUnsafeCell<[u16; FALL_LEN]> = SyncUnsafeCell::new([0; FALL_LEN]);
static FALLING_HI: SyncUnsafeCell<[u16; FALL_LEN]> = SyncUnsafeCell::new([0; FALL_LEN]);
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
// same priority. The walker's surface is the IC/RX position counters,
// the PLL anchor pair, and the `DESYNCED` sticky flag.
static WALKED_FALLING: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
/// IC entry count safely written by BOTH DMA1_CH5 (lo) and DMA1_CH6
/// (hi). Tracked via CH6's NDTR — CH6 is the trailing writer (TIM3.CCR1
/// latches one TRC-sync cycle after TIM2.CCR1, and DMA1 services CH5
/// before CH6 at equal priority), so when CH6 has decremented past an
/// entry, both halves are valid.
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

/// Sticky-fatal flag. Set once on either of two conditions: IC ring
/// overrun (`ic_overrun`) or stamp ring overflow from host backpressure
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

/// `ceiling` snapshot at the walker's most recent exit. Reported by
/// `ic_snapshot` as the chip-side "now" reference so the host can place
/// the IC ring window relative to wall time, and reused as the
/// wrap-race correction `ceiling` for entries the walker has already
/// confirmed are in the ring.
static LAST_LIFT_CEILING: AtomicU32 = AtomicU32::new(0);

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
        // DMA1_CH5 = TIM2_CC1 IC → FALLING_LO. Circular, 16-bit. No IRQ;
        // CH6 is the trailing writer and drives the walker.
        let ch5 = DMA1.ch(4);
        ch5.par().write_value(TIM2.chcvr(0).as_ptr() as u32);
        ch5.mar().write_value((*FALLING_LO.get()).as_ptr() as u32);
        ch5.ndtr().write(|w| w.set_ndt(FALL_LEN as u16));
        ch5.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_pl(Pl::VERYHIGH); // measurement clock — never queue
            w.set_htie(false);
            w.set_tcie(false);
            w.set_en(true);
        });

        // DMA1_CH6 = TIM3_CC1 IC → FALLING_HI. Circular, 16-bit. HTIE+
        // TCIE drive the walker; CH6 services AFTER CH5 (lower channel
        // number wins at equal priority), so when CH6's NDTR has
        // decremented past entry i, both `(lo[i], hi[i])` are valid.
        let ch6 = DMA1.ch(5);
        ch6.par().write_value(TIM3.chcvr(0).as_ptr() as u32);
        ch6.mar().write_value((*FALLING_HI.get()).as_ptr() as u32);
        ch6.ndtr().write(|w| w.set_ndt(FALL_LEN as u16));
        ch6.cr().write(|w| {
            w.set_dir(Dir::FROMPERIPHERAL);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS16);
            w.set_psize(Size::BITS16);
            w.set_pl(Pl::VERYHIGH);
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

/// Reconfigure TIM2 IC1F for the given USART BRR and update the matching
/// `CC_FILTER_DELAY_TICKS`. Live write per V20x RM §14.4.7 — IC1F has no
/// "channel must be disabled" restriction (unlike CC1S).
fn apply_filter_for_brr(brr: u32) {
    let (icf_bits, delay_ticks) = filter_for_brr(brr);
    TIM2.chctlr_input(0).modify(|w| {
        w.set_icf(0, FilterValue::from_bits(icf_bits));
    });
    CC_FILTER_DELAY_TICKS.store(delay_ticks, Ordering::Release);
}

/// Reconfigure USART3 BIT_TICKS reference. Caller already retuned the
/// USART itself via `tx::set_baud`; this just keeps the walker's bit
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

/// Returns `Some(cause)` once any of the trip conditions has fired,
/// `None` while READY. Recovery from desync is `RESET`.
pub fn desync_cause() -> Option<DesyncCause> {
    let raw = DESYNC_CAUSE.load(Ordering::Acquire);
    DesyncCause::from_u8(raw)
}

/// Stamps queued for drain (= `byte_head - byte_tail`). Host paces BBATCH
/// calls against this.
pub fn stamps_available() -> u32 {
    let head = BYTE_HEAD.load(Ordering::Acquire);
    let tail = BYTE_TAIL.load(Ordering::Relaxed);
    head.wrapping_sub(tail)
}

/// Configured baud (= `APB1_HZ / BIT_TICKS`). Returns 0 before the first
/// `set_baud` completes.
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

/// Atomic snapshot of the IC ring + walker counters for the `BICSNAP`
/// diagnostic host command. Counters and ring contents are read under
/// critical_section so they're coherent against walker ISRs. Returns the
/// snapshot header plus the number of u32 ticks written to `out`, in
/// oldest-first order (`out[0]` is the oldest in-ring entry, `out[n-1]`
/// the newest). Each entry is the wrap-race-corrected combined
/// `(hi, lo)` pair, so the host receives sub-µs tick32 values — no
/// host-side lift required.
///
/// `falling_total` is sourced from the walker's last refresh, which
/// tracks CH6 (the trailing DMA writer). Entries past CH5's NDTR but
/// not yet CH6's are intentionally invisible here: they haven't formed
/// a fully-written 32-bit pair yet. `walk()` advances `falling_total`
/// on its next trigger; the host can re-`BICSNAP` after a `STATUS`.
#[derive(Copy, Clone)]
pub struct IcSnapshot {
    pub ref_tick: u32,
    pub falling_total: u32,
    pub walked: u32,
    pub rx_total: u32,
    pub byte_head: u32,
    pub bit_ticks: u32,
    pub cc_filter_delay: u32,
}

pub fn ic_snapshot(out: &mut [u32]) -> (IcSnapshot, usize) {
    critical_section::with(|_| {
        let falling_total = unsafe { ptr::read_volatile(FALLING_TOTAL.get()) };
        let rx_total = unsafe { ptr::read_volatile(RX_TOTAL.get()) };
        let walked = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
        let byte_head = BYTE_HEAD.load(Ordering::Relaxed);
        let ref_tick = LAST_LIFT_CEILING.load(Ordering::Acquire);
        let bit_ticks = BIT_TICKS.load(Ordering::Relaxed);
        let cc_filter_delay = CC_FILTER_DELAY_TICKS.load(Ordering::Relaxed);

        let in_ring = (falling_total as usize).min(FALL_LEN);
        let n = in_ring.min(out.len());
        let start = falling_total.wrapping_sub(n as u32);
        for (i, slot) in out.iter_mut().take(n).enumerate() {
            let probe = start.wrapping_add(i as u32);
            *slot = falling_at(probe, falling_total, ref_tick);
        }
        (
            IcSnapshot {
                ref_tick,
                falling_total,
                walked,
                rx_total,
                byte_head,
                bit_ticks,
                cc_filter_delay,
            },
            n,
        )
    })
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

/// Live-NDTR total without mutating walker state. Returns `(total, ndtr)`
/// so the committing variant can store the matching NDTR without re-
/// reading the register. CH6 (TIM3.CCR1 high half) is the trailing DMA
/// writer: CH5 (TIM2) wins same-priority arbitration AND TIM3's TRC-
/// driven capture latches one sync cycle after TIM2's, so reading CH6's
/// NDTR bounds the count of fully-written 32-bit pairs.
#[inline]
fn peek_falling_total() -> (u32, u16) {
    let ndtr = DMA1.ch(5).ndtr().read().ndt();
    unsafe {
        let prev_ndtr = *LAST_FALLING_NDTR.get();
        let consumed = prev_ndtr.wrapping_sub(ndtr) as u32 & FALL_MASK;
        let total = ptr::read_volatile(FALLING_TOTAL.get()).wrapping_add(consumed);
        (total, ndtr)
    }
}

#[inline]
fn peek_rx_total() -> (u32, u16) {
    let ndtr = DMA1.ch(2).ndtr().read().ndt();
    unsafe {
        let prev_ndtr = *LAST_RX_NDTR.get();
        let consumed = prev_ndtr.wrapping_sub(ndtr) as u32 & RX_MASK;
        let total = ptr::read_volatile(RX_TOTAL.get()).wrapping_add(consumed);
        (total, ndtr)
    }
}

#[inline]
fn refresh_falling_total() -> u32 {
    let (total, ndtr) = peek_falling_total();
    unsafe {
        *LAST_FALLING_NDTR.get() = ndtr;
        ptr::write_volatile(FALLING_TOTAL.get(), total);
    }
    total
}

#[inline]
fn refresh_rx_total() -> u32 {
    let (total, ndtr) = peek_rx_total();
    unsafe {
        *LAST_RX_NDTR.get() = ndtr;
        ptr::write_volatile(RX_TOTAL.get(), total);
    }
    total
}

/// Read the hardware-atomic 32-bit IC capture pair at `idx` and apply
/// wrap-race correction.
///
/// The phase-locked TIM3 (started one AHB write ahead of TIM2 in
/// `tick::init`) increments a few cycles BEFORE each TIM2 wrap. If a
/// capture latches inside that gap, TIM3.CCR1 latches the new high half
/// while TIM2.CCR1 still holds the pre-wrap low half — combined value
/// reads as `actual + 65536`.
///
/// Detection uses two signals depending on whether `idx` is an interior
/// entry or the tail:
///
/// - **Interior (`idx + 1 < falling_total`)**: capture times in the ring
///   are monotonically non-decreasing (DMA writes them in order), so
///   `combined > combined_next` is a disjoint signal of the race on
///   `idx`. This is timing-independent: it works regardless of how many
///   TIM2 wraps elapse between capture and walker read, which the
///   earlier `combined > ceiling` predicate didn't survive at low baud /
///   long bursts. Two consecutive races would require TIM2 to wrap twice
///   between two captures — impossible since min inter-capture spacing
///   (filter delay = 256 cycles) is far below the wrap period (65536
///   cycles).
/// - **Tail (`idx + 1 == falling_total`)**: no successor exists. `ceiling`
///   was sampled microseconds after the latest IC pair was written
///   (NDTR refresh precedes `read_tick32`), so it is within one wrap of
///   the tail entry's capture time and the original `combined > ceiling`
///   predicate is reliable here.
#[inline]
fn falling_at(idx: u32, falling_total: u32, ceiling: u32) -> u32 {
    let i = (idx & FALL_MASK) as usize;
    let lo = unsafe { (*FALLING_LO.get())[i] };
    let hi = unsafe { (*FALLING_HI.get())[i] };
    let combined = ((hi as u32) << 16) | (lo as u32);

    if idx.wrapping_add(1) != falling_total {
        let ni = (idx.wrapping_add(1) & FALL_MASK) as usize;
        let lo_n = unsafe { (*FALLING_LO.get())[ni] };
        let hi_n = unsafe { (*FALLING_HI.get())[ni] };
        let combined_next = ((hi_n as u32) << 16) | (lo_n as u32);
        if combined != combined_next && combined.wrapping_sub(combined_next) <= u32::MAX / 2 {
            return combined.wrapping_sub(1 << 16);
        }
        combined
    } else if combined != ceiling && combined.wrapping_sub(ceiling) <= u32::MAX / 2 {
        combined.wrapping_sub(1 << 16)
    } else {
        combined
    }
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

/// Predict-and-snap PLL walker. Constructs `ceiling` internally: refresh
/// the IC NDTR snapshot first, THEN read `tick32`. Ordering guarantees
/// `combined ≤ ceiling` for every non-raced entry in `[walked,
/// falling_total)` — newer entries that land between the NDTR refresh
/// and the tick32 read aren't in this walker's snapshot, so they fall
/// to the next trigger.
///
/// `idle == true` marks this invocation as USART-IDLE-triggered: the
/// wire has been quiet one character time, so the next byte (when it
/// arrives) is the start of a new burst — not contiguous with the last.
/// After draining, the walker flushes trailing interior edges and drops
/// `has_anchor` so the next byte cold-starts on its own IC edge instead
/// of free-running off a stale `last_anchor + 10·bit_ticks` prediction
/// that would sit one inter-packet gap (RDT) before the real edge. IDLE
/// is signal-only here — the next byte's tick still comes from its own
/// IC entry via the cold-start path.
pub fn walk(idle: bool) {
    if DESYNCED.load(Ordering::Relaxed) {
        return;
    }
    let bit_ticks = BIT_TICKS.load(Ordering::Relaxed);
    if bit_ticks == 0 {
        return;
    }
    let byte_period = BITS_PER_BYTE_8N1.wrapping_mul(bit_ticks);
    let snap = SNAP_BITS.wrapping_mul(bit_ticks);
    let cc_filter_delay = CC_FILTER_DELAY_TICKS.load(Ordering::Relaxed);

    let falling_total = refresh_falling_total();
    let rx_total = refresh_rx_total();
    let ceiling = read_tick32();
    LAST_LIFT_CEILING.store(ceiling, Ordering::Release);
    let mut walked = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };

    // ic_overrun: if more new entries arrived than the ring can hold,
    // we lost edges → permanent lockstep desync. Sticky-fatal.
    if falling_total.wrapping_sub(walked) > FALL_LEN as u32 {
        set_desync(DesyncCause::IcOverrun);
        return;
    }

    let mut byte_head = BYTE_HEAD.load(Ordering::Relaxed);
    let mut anchor = unsafe { ptr::read_volatile(LAST_ANCHOR.get()) };
    let mut has_anchor = unsafe { ptr::read_volatile(HAS_ANCHOR.get()) };

    while byte_head != rx_total {
        // stamp_overflow: if the host hasn't drained, the next emit
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
            // entry. Need `ceiling` to bound `byte_period` past
            // first_edge so we don't anchor on what is actually some
            // interior edge of the same byte whose start bit is still
            // unwalked.
            if walked == falling_total {
                break;
            }
            let first_edge = falling_at(walked, falling_total, ceiling);
            if ceiling.wrapping_sub(first_edge) < byte_period {
                break;
            }
            chosen_anchor = first_edge;
            walked = walked.wrapping_add(1);
        } else {
            // Steady-state PLL. Predict the next start bit, then snap to
            // the IC entry closest to prediction within `±snap`.
            let predicted = anchor.wrapping_add(byte_period);
            let snap_high = predicted.wrapping_add(snap);
            // Yield mid-byte: `ceiling` must reach past `snap_high`
            // before we can rule out a real edge that's still in flight
            // toward the IC ring. Without this guard, an edge landing at
            // (predicted, snap_high] after the walker exits would be lost
            // — we'd have already stamped a free-run miss.
            if ceiling.wrapping_sub(snap_high) > u32::MAX / 2 {
                break;
            }
            let snap_low = predicted.wrapping_sub(snap);

            // Skip any leftover prev-byte interior edges (everything
            // sitting before `snap_low` is past byte N−1's start bit but
            // before byte N's snap window).
            while walked != falling_total {
                let tick = falling_at(walked, falling_total, ceiling);
                if tick.wrapping_sub(snap_low) > u32::MAX / 2 {
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
                let tick = falling_at(probe, falling_total, ceiling);
                // Past snap_high → stop scanning.
                if tick.wrapping_sub(snap_high) <= u32::MAX / 2 && tick != snap_high {
                    break;
                }
                let dist = if tick >= predicted {
                    tick.wrapping_sub(predicted)
                } else {
                    predicted.wrapping_sub(tick)
                };
                if dist < best_dist || (dist == best_dist && tick >= predicted) {
                    best_dist = dist;
                    chosen = Some(tick);
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

    // Chain-break on IDLE-after-drain: USART IDLE means the wire just
    // went quiet. If the byte queue fully drained, any trailing IC
    // entries are interior edges of the last byte that the snap window
    // chose not to consume; they're stale once the next burst starts.
    // Drop them, drop the prediction chain, and the next byte cold-starts
    // on its own real IC edge. Without this, request/reply traffic (TX
    // echo → RDT silence → reply) tries to anchor reply byte 0 at
    // `last_echo_anchor + 10·bit_ticks` and free-runs every reply byte
    // because the real edges sit hundreds of bit-times past the snap.
    if idle && byte_head == rx_total {
        walked = falling_total;
        has_anchor = false;
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
fn run_walker(phase: u8) {
    let cnt_entry = TIM2.cnt().read();
    let walked_pre = unsafe { ptr::read_volatile(WALKED_FALLING.get()) };
    let byte_head_pre = BYTE_HEAD.load(Ordering::Relaxed);
    // Live-NDTR pending counts without mutating walker state; `walk()`
    // re-reads NDTR before processing.
    let (falling_total_now, _) = peek_falling_total();
    let (rx_total_now, _) = peek_rx_total();
    let falling_pending_entry = falling_total_now.wrapping_sub(walked_pre);

    walk(phase == TRACE_PHASE_IDLE);

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
        tim2_cnt_entry: cnt_entry,
        tim2_cnt_exit: TIM2.cnt().read(),
        falling_pending_entry: falling_pending_entry.min(u16::MAX as u32) as u16,
        edges_consumed: edges.min(u8::MAX as u32) as u8,
        bytes_emitted: bytes.min(u8::MAX as u32) as u8,
        falling_total: falling_total_now,
        rx_total: rx_total_now,
    });
}

/// DMA1_CH6 = FALLING_HI (trailing IC half) HT/TC. CH6 services after
/// CH5 (same TIM2 TRGO pulse), so HT/TC here guarantees both halves of
/// every entry up to the cursor are written. Drains the walker so edges
/// don't accumulate past the half-ring.
#[interrupt]
fn DMA1_CHANNEL6() {
    let isr = DMA1.isr().read();
    let tc = isr.tcif(5);
    DMA1.ifcr().write(|w| {
        w.set_htif(5, true);
        w.set_tcif(5, true);
    });
    let phase = if tc {
        TRACE_PHASE_IC_TC
    } else {
        TRACE_PHASE_IC_HT
    };
    run_walker(phase);
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
    let phase = if tc {
        TRACE_PHASE_RX_TC
    } else {
        TRACE_PHASE_RX_HT
    };
    run_walker(phase);
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
        run_walker(TRACE_PHASE_IDLE);
        crate::tx::on_idle(idle_tick);
    }
    crate::led::signal();
}
