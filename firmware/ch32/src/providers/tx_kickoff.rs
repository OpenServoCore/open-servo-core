//! TX-start hardware kickoff — the DMA1_CH7 time-share
//! (`docs/dxl-hw-timed-transport.md` §5/§6).
//!
//! During a scheduled send, TIM2_CH4 swaps from input capture (edge ring)
//! to output compare, and DMA1_CH7 — normally the ET-ring drain riding
//! CH4's CC4DE request line — becomes a one-transfer MEM→PER write of an
//! EN=1 config word into `DMA1_CH4.CR`. The CC4 compare match then starts
//! the USART1 TX stream with zero CPU in the deadline path. CH7's TC
//! (~5 HCLK after the match — one transfer) raises the DMA1_CH7 IRQ whose
//! body restores CH4→IC and CH7→edge-ring long before our own bytes
//! finish streaming; the RX line is silent during our TX (no self-echo),
//! so no edge is ever missed.
//!
//! Free functions + module statics rather than provider-struct state: the
//! arm/cancel sites live on [`DxlTxScheduler`] (driver-called), the
//! restore lives in the DMA1_CH7 ISR (no driver involvement), and the
//! edge-ring window latch is read by [`EdgeDma`] — three consumers, one
//! PFIC-HIGH-serialized state. Atomics are `portable-atomic` (V006 has no
//! native atomics), Relaxed throughout — same-priority no-preemption is
//! the actual synchronization.
//!
//! [`DxlTxScheduler`]: super::dxl_tx_scheduler::DxlTxScheduler
//! [`EdgeDma`]: super::edge_dma::EdgeDma

use core::cell::SyncUnsafeCell;

use ch32_metapac::DMA1;
use ch32_metapac::dma::vals::{Dir, Pl, Size};
use portable_atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use crate::hal::{dma, timer};
use crate::measurements::KICKOFF_RETRY_LEAD_TICKS;
use crate::runtime::registry::DXL_EDGE_BUF_LEN;

/// The word CH7 writes into `DMA1_CH4.CR` at the compare match: CH4's
/// bring-up TX config (`runtime/init.rs`) plus EN. Populated at every
/// [`arm`] from the same metapac field constructors so the two can't
/// drift. `SyncUnsafeCell` because DMA reads it while the CPU owns it —
/// writes happen only while CH7 is disabled.
static KICKOFF_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Edge-ring buffer address, cached at bring-up for the restore path
/// (the driver's `edges_addr()` isn't reachable from the ISR).
static EDGES_ADDR: AtomicU32 = AtomicU32::new(0);

/// True from [`arm`] until the driver consumes [`take_ring_restart`] —
/// the span where [`edge_remaining`] must return the arm-time latch.
static WINDOW_OPEN: AtomicBool = AtomicBool::new(false);

/// True once [`restore`] has re-armed the edge ring for the current
/// window; [`take_ring_restart`] hands it to the driver exactly once.
static RESTORED: AtomicBool = AtomicBool::new(false);

/// CH7 NDTR frozen at the EdgeRing→Kickoff transition. Exact for the
/// whole window: CH4 stops capturing the moment it leaves IC mode, so the
/// ring content is static until the driver resyncs.
static LATCHED_EDGE_NDTR: AtomicU16 = AtomicU16::new(0);

fn ch4_cr_with_en() -> u32 {
    let mut cr = ch32_metapac::dma::regs::Cr(0);
    cr.set_dir(Dir::FROMMEMORY);
    cr.set_circ(false);
    cr.set_pinc(false);
    cr.set_minc(true);
    cr.set_psize(Size::BITS8);
    cr.set_msize(Size::BITS8);
    cr.set_pl(Pl::LOW);
    cr.set_en(true);
    cr.0
}

/// Bring-up entry: cache the edge-ring destination and configure CH7 for
/// ET-ring duty. Also the shared restore shape — [`restore`] re-runs the
/// same configuration after each kickoff window.
pub fn install_edge_ring(edges_addr: u32) {
    EDGES_ADDR.store(edges_addr, Ordering::Relaxed);
    configure_edge_ring();
    dma::enable(dma::Channel::CH7);
}

/// CH7 as the ET ring: circular PER→MEM from TIM2.CCR4. PL=VeryHigh is
/// the only channel at that priority (ADC + USART RX/TX all sit at LOW
/// per doc §6) so CC4 capture stores can't be delayed. No HT/TC IRQ in
/// EdgeRing role — TCIE belongs to the kickoff role only.
fn configure_edge_ring() {
    let cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: Size::BITS16,
        htie: false,
        tcie: false,
        pl: Pl::VERYHIGH,
    };
    dma::configure(
        dma::Channel::CH7,
        &cfg,
        timer::tim2_ch4_capture_addr(),
        EDGES_ADDR.load(Ordering::Relaxed),
        DXL_EDGE_BUF_LEN as u16,
    );
}

/// Open the kickoff window and arm the hardware: CH7 → one-transfer
/// kickoff, CH4 → OC with CCR4 = `compare`. A re-arm inside an open
/// window (schedule without intervening cancel) keeps the original NDTR
/// latch — the kickoff-mode NDTR must never leak into it.
pub fn arm(compare: u16) {
    if !WINDOW_OPEN.load(Ordering::Relaxed) {
        LATCHED_EDGE_NDTR.store(dma::remaining(dma::Channel::CH7), Ordering::Relaxed);
        RESTORED.store(false, Ordering::Relaxed);
        WINDOW_OPEN.store(true, Ordering::Relaxed);
    }
    // Ordering is the load-bearing part. On the on-time path the
    // predecessor's bytes are STILL STREAMING during this arm — every IC
    // capture pulses the CC4 request line. The sequence must guarantee no
    // stale capture request (or stale CC4IF held level) can reach
    // kickoff-configured CH7, or the enable word lands in CH4 the moment
    // CH7 turns on and the send fires early into a not-yet-raised TX_EN
    // (bench signature: clipped/garbled emission tails, noreply at 3M):
    //
    // 1. CH7 off — capture requests pulse into a disabled channel and drop.
    // 2. CH4 leaves IC mode (captures stop), CCR4 = compare, CC4IF wiped
    //    LAST — from here the flag can only re-set on a real new-compare
    //    match.
    // 3. CH7 reconfigures and enables as the kickoff.
    //
    // A real match inside step 2-3's dead window drops its request; the
    // caller's §5.4 recheck sees CNT past CCR4 and re-aims via
    // `trigger_asap`.
    dma::disable(dma::Channel::CH7);
    timer::tim2_ch4_to_oc(compare);
    // SAFETY: CH7 is disabled — no DMA read of the word can race this.
    unsafe { *KICKOFF_WORD.get() = ch4_cr_with_en() };
    let cfg = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: false,
        pinc: false,
        minc: false,
        size: Size::BITS32,
        htie: false,
        tcie: true,
        pl: Pl::VERYHIGH,
    };
    dma::configure(
        dma::Channel::CH7,
        &cfg,
        DMA1.ch(3).cr().as_ptr() as u32,
        KICKOFF_WORD.get() as u32,
        1,
    );
    // A stale TC latch from the previous kickoff must not re-run restore
    // the moment the vector unmasks.
    dma::clear_tc_flag(dma::Channel::CH7);
    dma::enable(dma::Channel::CH7);
}

/// Past-deadline retry: re-aim CCR4 just ahead of CNT so a REAL compare
/// event fires the already-armed kickoff ASAP. (SWEVGR.CC4G would be the
/// obvious trigger, but it's dead silicon on V006 — spike-verified.)
pub fn trigger_asap() {
    timer::clear_tim2_cc4_flag();
    timer::set_tim2_ccr4(timer::tim2_cnt().wrapping_add(KICKOFF_RETRY_LEAD_TICKS));
}

/// DMA1_CH7 TC ISR body — the kickoff word landed, TX DMA is streaming.
/// Restore CH4 to input capture and CH7 to ET-ring duty; the window-flag
/// pair stays set until the driver consumes the restart.
pub fn on_kickoff_complete() {
    dma::clear_tc_flag(dma::Channel::CH7);
    if !WINDOW_OPEN.load(Ordering::Relaxed) || RESTORED.load(Ordering::Relaxed) {
        // Spurious entry — nothing armed, or already restored (a cancel
        // raced the TC). Never touch a live edge ring.
        return;
    }
    restore();
}

/// Cancel-path restore: a schedule was dropped while the window was open
/// (host retry canceling an armed reply). No-op when nothing is armed.
pub fn cancel() {
    if WINDOW_OPEN.load(Ordering::Relaxed) && !RESTORED.load(Ordering::Relaxed) {
        restore();
    }
}

fn restore() {
    dma::disable(dma::Channel::CH7);
    timer::tim2_ch4_to_ic();
    configure_edge_ring();
    dma::enable(dma::Channel::CH7);
    RESTORED.store(true, Ordering::Relaxed);
}

/// `EdgeDma::remaining` source: the live CH7 NDTR, or the arm-time latch
/// while the kickoff window is open (CH7's NDTR is the kickoff count
/// then, and after restore the ring restarted — either value would
/// corrupt the driver's ring bookkeeping until it consumes the restart).
pub fn edge_remaining() -> u16 {
    if WINDOW_OPEN.load(Ordering::Relaxed) {
        LATCHED_EDGE_NDTR.load(Ordering::Relaxed)
    } else {
        dma::remaining(dma::Channel::CH7)
    }
}

/// `EdgeDma::take_ring_restart` source: true exactly once per completed
/// window — the driver resets its ring bookkeeping and `edge_remaining`
/// goes live again.
pub fn take_ring_restart() -> bool {
    if WINDOW_OPEN.load(Ordering::Relaxed) && RESTORED.load(Ordering::Relaxed) {
        WINDOW_OPEN.store(false, Ordering::Relaxed);
        RESTORED.store(false, Ordering::Relaxed);
        true
    } else {
        false
    }
}
