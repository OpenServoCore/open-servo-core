//! Wire TX path: pre-encoded payload → DMA1_CH2 → USART3 (TX=PB10 AF OD,
//! RX=PB11 input-pullup), kicked by TIM4 OPM CC2 match chained through
//! DMA1_CH4 — no IRQ in the send path.
//!
//! USART3 runs **full-duplex**, not HDSEL: PB10 and PB11 are bridged
//! externally on the bench, putting both halves of the USART block across
//! the same wire. The chip then RX-captures both our own TX echo (useful
//! for TX_COMP autocal) and remote replies. HDSEL would internally tri-
//! state RX during TX, which is the opposite of what we need.
//!
//! Why USART3 and not USART2 on PA2/PA3: PA3 has R10=10 kΩ to 3V3 on the
//! MuseLab board (SD-card CS pull-up). With the bus connected idle-high
//! before USB plugs in, current trickles via bus → series-R → PA3 → R10
//! → chip 3V3 rail and parasitically charges VDD above POR threshold,
//! leaving the chip stuck in a half-booted state when USB later supplies
//! real power. PB10/PB11 are clean breakouts with no onboard pulls.
//!
//! TIM4 OPM, PSC=0, ARR=delta, CCR2=ARR. On CC2 compare match, DMA1_CH4
//! stamps a precomputed run-CFGR word (EN=1) over DMA1_CH2.CR; CH2 then
//! drains the prepared payload into USART3.DATAR. CCR2=ARR puts the CC2
//! event on the same edge as the OPM overflow.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::Interrupt;
use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::timer::vals::Urs;
use ch32_metapac::{AFIO, DMA1, GPIOB, RCC, TIM4, USART3};
use crate::parse::brr_for;
use crate::tick::read_tick32;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

pub const TX_BUF_LEN: usize = 1024;

/// (send_at - now) below this many `tick32` ticks bypasses TIM4 and writes
/// the run-CFGR word directly into DMA1_CH2. 256 ticks ≈ 1.8 µs at
/// 144 MHz — safely above the register-write latency of the immediate
/// path. A too-close deadline still kicks (wire-edge lands ≈ "now")
/// instead of silently missing on a wrap.
const IMMEDIATE_SEND_THRESHOLD_TICKS: u32 = 256;

/// TIM4 ARR (u16) max in tick32 units. PSC=0 → 144 MHz tick = 1:1 with
/// tick32, so ARR = delta directly and max schedule is 65 535 ticks ≈
/// 455 µs. Deadlines beyond this fall back to the immediate path (host
/// should never request them — typical wire-side slot timing is sub-ms).
const TIM4_MAX_DELTA_TICKS: u32 = u16::MAX as u32;

const TIM4_PSC: u16 = 0;

/// USART3 sits on APB1. Same rate as HCLK with prescaler DIV1.
pub const APB1_HZ: u32 = 144_000_000;
pub const DEFAULT_BAUD: u32 = 1_000_000;

/// `SEND` at/now buffer. The `at=` and no-key paths share it; the
/// no-key path routes through `schedule_send_at` internally so the same
/// commanded-tick semantics apply to both.
static TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);

/// `SEND after_idle=` uses a separate buffer so a `SEND` immediately
/// followed by a `SEND after_idle=` (echo TX, then a scheduled response
/// after the wire goes idle) keeps both payloads loaded simultaneously.
/// `on_idle()` swaps DMA1_CH2 onto IDLE_TX_BUF before kicking the send.
static IDLE_TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);
static IDLE_TX_LEN: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

/// Precomputed DMA1_CH2 CFGR word with EN=1. DMA1_CH4 copies this 32-bit
/// word over DMA1_CH2.CR on the TIM4 CC2 kickoff. Written once during
/// init, read-only thereafter so the DMA can treat it as constant memory.
static CH2_CFGR_RUN_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Set by `schedule_send_after_idle`; consumed by `on_idle`. Cleared each
/// IDLE swap.
static IDLE_SEND_PENDING: AtomicBool = AtomicBool::new(false);
static IDLE_SEND_DELAY_TICKS: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// `tick32` at the last send kickoff (commanded value for scheduled
/// sends, "now" for immediate sends). Host correlates against per-byte
/// stamps.
static LAST_SEND_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// TX-path latency compensation decomposed into the two clock domains
/// the TX chain crosses (TIM4 CC2 match → wire start-bit edge):
///
///   TX_COMP_TICKS = TX_PIPELINE_TICKS
///                   + (TX_BIT_TIMES_Q4 × brr) / 16
///
/// `TX_PIPELINE_TICKS` covers the HCLK-domain pipeline:
///   TIM4 CC2 → DMA1_CH4 stamp → DMA1_CH2.CR (EN=1) → DMA fetch → USART3.DR
/// Fixed HCLK cycles + AHB arbitration; no baud dependency.
///
/// `TX_BIT_TIMES_Q4` covers the USART bit-clock-domain pipeline in Q4
/// (16 = 1.0 × brr). After the DR write, the byte waits 0..brr HCLK for
/// the next bit-clock edge to load TSR; TSR loads on that edge and the
/// start bit (high→low) falls at the same instant. IC1 captures the
/// falling edge, so the latency we compensate is exactly the DR→edge
/// wait — U[0, brr] per shot, mean 0.5 × brr → Q4 = 8.
///
/// Both atoms are runtime-tunable via the `COMP` protocol command so
/// bench can write measured values back without a firmware rebuild.
/// `TX_COMP_TICKS` is the precomputed sum read by the send fast path —
/// recomputed by `recompute_tx_comp` whenever brr or either tunable
/// changes. Defaults below are the converged values from
/// `tool-pirate-tune --stage tx-comp` on osc-dev-v20x at 144 MHz HCLK
/// (median across 5 runs at n=64 shots/baud; per-run wobble ±~5 ticks
/// on pipe, ±1 on bit_q4 — bounded by the U[0, brr] median SE floor).
static TX_COMP_TICKS: AtomicU32 = AtomicU32::new(0);
static TX_PIPELINE_TICKS: AtomicU32 = AtomicU32::new(TX_PIPELINE_TICKS_DEFAULT);
static TX_BIT_TIMES_Q4: AtomicU32 = AtomicU32::new(TX_BIT_TIMES_Q4_DEFAULT);

const TX_PIPELINE_TICKS_DEFAULT: u32 = 45;
const TX_BIT_TIMES_Q4_DEFAULT: u32 = 8;

fn recompute_tx_comp(brr: u32) {
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
        recompute_tx_comp(brr);
    });
}

/// Headroom added to `read_tick32()` when [`send_now`] schedules its
/// near-future send through [`schedule_or_send_now`]. Sized so the
/// resulting `delta` lands comfortably above [`IMMEDIATE_SEND_THRESHOLD_TICKS`]
/// — keeping the scheduled (TIM4 OPM) path in play rather than falling
/// back to the immediate DMA-direct branch, so `LAST_SEND_TICK` matches
/// the commanded wire-start tick. 512 ticks ≈ 3.5 µs at 144 MHz —
/// negligible added latency, well above the 256-tick scheduler threshold.
const IMMEDIATE_SEND_MARGIN_TICKS: u32 = 512;

/// Upper bound on how long `wait_tx_complete` will spin for `USART3.SR.TC`.
/// 200 ms at 144 MHz. Generous enough for any payload up to TX_BUF_LEN at
/// 9600 baud (≈ 1.1 s for 1024 bytes — we cap shorter than that on
/// purpose: a stuck TC means something genuinely broke and the host should
/// see a `busy` error rather than the chip pretending to work).
const TX_WAIT_TIMEOUT_TICKS: u32 = 28_800_000;

#[inline]
fn store_last_send_tick(t: u32) {
    unsafe { ptr::write_volatile(LAST_SEND_TICK.get(), t) }
}
#[inline]
fn store_idle_send_delay(v: u32) {
    unsafe { ptr::write_volatile(IDLE_SEND_DELAY_TICKS.get(), v) }
}
#[inline]
fn load_idle_send_delay() -> u32 {
    unsafe { ptr::read_volatile(IDLE_SEND_DELAY_TICKS.get()) }
}

pub fn init() {
    init_clocks_and_remap();
    init_pins();
    init_usart3();
    init_tim4_scheduler();
    init_dma_tx();

    unsafe {
        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL6 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL3 as u8);
    }
}

/// Clocks (USART3 + TIM4 on APB1, GPIOB + AFIO on APB2, DMA1 on AHB) and
/// USART3 AFIO remap (default mapping PB10/PB11). TIM2/TIM3 enables +
/// TIM2 partial remap live in `tick::init`.
fn init_clocks_and_remap() {
    RCC.apb2pcenr().modify(|w| {
        w.set_iopben(true);
        w.set_afioen(true);
    });
    RCC.apb1pcenr().modify(|w| {
        w.set_usart3en(true);
        w.set_tim4en(true);
    });
    RCC.ahbpcenr().modify(|w| w.set_dma1en(true));

    AFIO.pcfr1().modify(|w| {
        w.set_usart3_rm(0); // 00 = default PB10/PB11
    });
}

/// PB10/PB11 (USART3_TX/RX): TX as AF push-pull (50 MHz), RX as input
/// pull-up.
///
/// Push-pull on PB10 (vs the spec-correct open-drain) is bench-only:
/// PB10↔PB11 are bridged through open air with no transceiver, so the
/// only pull on the wire is PB11's ~30 kΩ internal pull-up, giving
/// τ ≈ 500–900 ns rise time and USART RX mis-sample at ≥ 2 Mbaud. Push-
/// pull actively drives both edges. The multi-drop "no PP fighting OD"
/// concern doesn't apply to this rig.
fn init_pins() {
    // ODR high before AF lock: guards against a transient ODR-LOW
    // pulling the bus while the AF block is mid-init.
    GPIOB.outdr().modify(|w| {
        w.set_odr(10, true);
        w.set_odr(11, true); // select PB11 input pullup (see below)
    });
    //   PB10: Mode=11 (50 MHz), CNF=10 (AF PP) → 0b1011.
    //   PB11: Mode=00 (input),  CNF=10 (input w/ pull) → 0b1000;
    //         ODR(11)=1 above selects pull-up.
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 8);
        v &= !(0xF << 12);
        v |= 0b1011u32 << 8;
        v |= 0b1000u32 << 12;
        w.0 = v;
    });
}

/// USART3: 8N1, full-duplex, DMAT, DMAR, IDLEIE. Init order matters —
/// TE/RE with UE=0, then DMAT/DMAR, then UE in its own write, to avoid
/// the TX-line glitch the STM32-family USARTs throw when TE+UE land in
/// the same write.
fn init_usart3() {
    USART3.ctlr2().modify(|w| w.set_stop(0b00));
    USART3.ctlr1().modify(|w| {
        w.set_m(false);
        w.set_pce(false);
        w.set_te(true);
        w.set_re(true);
        w.set_idleie(true);
    });
    USART3.ctlr3().modify(|w| {
        w.set_dmat(true);
        w.set_dmar(true);
    });
    let brr0 = APB1_HZ / DEFAULT_BAUD;
    USART3.brr().write(|w| w.0 = brr0);
    USART3.ctlr1().modify(|w| w.set_ue(true));
    recompute_tx_comp(brr0);
}

/// TIM4 OPM. PSC=0 (1:1 with tick32). CR1: OPM=1 (auto-clear CEN after
/// first UEV), URS=1 (UG software event resets CNT without generating a
/// UEV that would spuriously kick CC2). DIER CC2DE=1 routes CC2 compare
/// match to DMA1_CH4.
fn init_tim4_scheduler() {
    TIM4.psc().write_value(TIM4_PSC);
    TIM4.atrlr().write_value(0xFFFF);
    TIM4.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_opm(true);
        w.set_urs(Urs::COUNTERONLY);
        w.set_arpe(false);
    });
    TIM4.dmaintenr().write(|w| {
        w.set_ccde(1, true); // CC2DE (channels 0-indexed)
    });
}

/// DMA1_CH2 = USART3_TX + DMA1_CH4 = TIM4_CC2 stamper that flips CH2.EN
/// on send. CH2.EN stays clear at init; `load_payload` reloads MAR/NDTR
/// with EN=0 between sends. CH4 is circular single-word: copy
/// CH2_CFGR_RUN_WORD → DMA1_CH2.CR. NDTR=1 auto-reloads each cycle
/// (CIRC=1), so we never re-program CH4 from software. CH4.EN=1
/// permanently — the CC2DE gate kicks it, not EN.
fn init_dma_tx() {
    let ch2 = DMA1.ch(1);
    ch2.par().write_value(USART3.datar().as_ptr() as u32);
    ch2.cr().write(|w| {
        w.set_dir(Dir::FROMMEMORY);
        w.set_minc(true);
        w.set_pinc(false);
        w.set_circ(false);
        w.set_msize(Size::BITS8);
        w.set_psize(Size::BITS8);
        // VERYHIGH is defensive: today only CH2 + CH4 are active so
        // inter-channel arbitration doesn't kick in, but pins the send
        // path at top priority against any future DMA channel we add.
        // (Empirically does NOT tighten TX wire-edge jitter — the
        // observed loopback variance is CPU-vs-DMA AHB arbitration,
        // which channel-priority bits don't control.)
        w.set_pl(Pl::VERYHIGH);
        w.set_tcie(false);
    });

    // Precompute the EN=1 CFGR word DMA1_CH4 stamps on send. Same bits
    // as the stopped config above, plus EN=1.
    let mut run = ch32_metapac::dma::regs::Cr(0);
    run.set_dir(Dir::FROMMEMORY);
    run.set_minc(true);
    run.set_pinc(false);
    run.set_circ(false);
    run.set_msize(Size::BITS8);
    run.set_psize(Size::BITS8);
    run.set_pl(Pl::VERYHIGH); // mirror CH2 PL above so the stamp doesn't drop priority on kickoff
    run.set_tcie(false);
    run.set_en(true);
    // SAFETY: CH2_CFGR_RUN_WORD is only read by DMA1_CH4 (set up
    // below). CH4 isn't enabled yet, so no concurrent reader.
    unsafe { ptr::write_volatile(CH2_CFGR_RUN_WORD.get(), run.0) };

    let ch4 = DMA1.ch(3);
    ch4.par().write_value(ch2.cr().as_ptr() as u32);
    ch4.mar().write_value(CH2_CFGR_RUN_WORD.get() as u32);
    ch4.ndtr().write(|w| w.set_ndt(1));
    ch4.cr().write(|w| {
        w.set_dir(Dir::FROMMEMORY);
        w.set_minc(false);
        w.set_pinc(false);
        w.set_circ(true);
        w.set_msize(Size::BITS32);
        w.set_psize(Size::BITS32);
        w.set_pl(Pl::VERYHIGH); // mirror CH2 PL above; defensive against future inter-channel contention
        w.set_tcie(false);
        w.set_en(true);
    });
}

/// Load `payload` and kick off when `tick32` reaches `at`. Held under a
/// critical section so a pending USART3 IDLE can't kick the DMA between
/// disabling EN and writing NDTR/MAR.
pub fn schedule_send_at(payload: &[u8], at: u32) -> Result<(), SendError> {
    critical_section::with(|_| {
        IDLE_SEND_PENDING.store(false, Ordering::Release);
        cancel_tim4_schedule();
        load_payload_main(payload)?;
        schedule_or_send_now(at);
        Ok(())
    })
}

/// Load `payload` into IDLE_TX_BUF and kick off `after_idle_ticks` after
/// the next USART3 IDLE assertion. Note: IDLE asserts ~1 character time
/// after wire-end; host should subtract that if sub-byte alignment
/// matters.
pub fn schedule_send_after_idle(payload: &[u8], after_idle_ticks: u32) -> Result<(), SendError> {
    critical_section::with(|_| -> Result<(), SendError> {
        IDLE_SEND_PENDING.store(false, Ordering::Release);
        if payload.len() > TX_BUF_LEN {
            return Err(SendError::TooLong);
        }
        // SAFETY: IDLE_SEND_PENDING=false above prevents on_idle from
        // reading IDLE_TX_BUF until we set true below.
        unsafe {
            let buf = &mut *IDLE_TX_BUF.get();
            buf[..payload.len()].copy_from_slice(payload);
            ptr::write_volatile(IDLE_TX_LEN.get(), payload.len() as u16);
        }
        store_idle_send_delay(after_idle_ticks);
        IDLE_SEND_PENDING.store(true, Ordering::Release);
        Ok(())
    })
}

/// Send `payload` with no precise timing — commands a near-future
/// kickoff at `now + TX_COMP_TICKS + IMMEDIATE_SEND_MARGIN_TICKS` so the
/// scheduled (TIM4 OPM) path always wins and `LAST_SEND_TICK` reflects
/// the actual wire-start tick. Wire-start lands ~3.5 µs after the call.
///
/// Preempts any pending `schedule_send_at`; coexists with a pending
/// `schedule_send_after_idle` (separate buffer). Waits for any in-flight
/// TX to drain first so a back-to-back send from the host can't truncate
/// the previous frame.
pub fn send_now(payload: &[u8]) -> Result<(), SendError> {
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    // CS keeps the `at` computation atomic with `schedule_send_at` so an
    // ISR between read_tick32 and the schedule can't eat the
    // IMMEDIATE_SEND_MARGIN and push us onto the immediate-DMA branch.
    // Nested CS (schedule_send_at opens its own) refcount-stacks under qingke.
    critical_section::with(|_| {
        let comp = TX_COMP_TICKS.load(Ordering::Relaxed);
        let at = read_tick32()
            .wrapping_add(comp)
            .wrapping_add(IMMEDIATE_SEND_MARGIN_TICKS);
        schedule_send_at(payload, at)
    })
}

fn cancel_tim4_schedule() {
    TIM4.ctlr1().modify(|w| w.set_cen(false));
    // URS=1 blocks UEV from this UG, so DMA1_CH4 isn't kicked.
    TIM4.swevgr().write(|w| w.set_ug(true));
    TIM4.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(1, false);
    });
}

/// Called from the USART3 IDLE handler. When a `send_after_idle` is
/// pending, swaps DMA1_CH2 to point at IDLE_TX_BUF and kicks off
/// `after_idle_ticks` after the IDLE-assertion tick32.
pub fn on_idle(idle_tick: u32) {
    if !IDLE_SEND_PENDING.swap(false, Ordering::AcqRel) {
        return;
    }
    let after = load_idle_send_delay();

    // A pipelined SEND(at=) + SEND(after_idle=) that races IDLE can leave
    // TIM4 with CEN=1 mid-schedule; cancel so schedule_or_send_now starts
    // from a known-idle TIM4 with our reloaded buffer.
    cancel_tim4_schedule();

    // Swap DMA1_CH2 to IDLE_TX_BUF.
    // SAFETY: USART3 IDLE ISR is the sole reader of IDLE_TX_BUF/IDLE_TX_LEN;
    // schedule_send_after_idle is the sole writer, and IDLE_SEND_PENDING
    // gating means it can't be mid-write here (the swap above ate the
    // flag).
    unsafe {
        let len = ptr::read_volatile(IDLE_TX_LEN.get());
        let ch = DMA1.ch(1);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(len));
        ch.mar().write_value((*IDLE_TX_BUF.get()).as_ptr() as u32);
    }

    let send_at = idle_tick.wrapping_add(after);
    schedule_or_send_now(send_at);
}

/// Reconfigure USART3 bit rate. Bounces UE around the BRR write, which
/// truncates any in-flight TX — so we wait for `SR.TC` first. The wait
/// runs outside the critical section to keep USB ISR latency bounded.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    wait_tx_complete().map_err(|TxTimeout| BaudError::Busy)?;
    critical_section::with(|_| {
        USART3.ctlr1().modify(|w| w.set_ue(false));
        USART3.brr().write(|w| w.0 = brr);
        USART3.ctlr1().modify(|w| w.set_ue(true));
        recompute_tx_comp(brr);
        IDLE_SEND_PENDING.store(false, Ordering::Release);
        cancel_tim4_schedule();
    });
    Ok(())
}

/// Spin until USART3 finishes transmitting the last frame, bounded by
/// `TX_WAIT_TIMEOUT_TICKS`. TC is set out of reset and after every
/// transmission, so this returns instantly on a quiesced bus.
struct TxTimeout;

fn wait_tx_complete() -> Result<(), TxTimeout> {
    let start = read_tick32();
    while !USART3.statr().read().tc() {
        let elapsed = read_tick32().wrapping_sub(start);
        if elapsed > TX_WAIT_TIMEOUT_TICKS {
            return Err(TxTimeout);
        }
    }
    Ok(())
}

pub fn last_send_tick() -> u32 {
    unsafe { ptr::read_volatile(LAST_SEND_TICK.get()) }
}

fn load_payload_main(payload: &[u8]) -> Result<(), SendError> {
    if payload.len() > TX_BUF_LEN {
        return Err(SendError::TooLong);
    }
    unsafe {
        let buf = &mut *TX_BUF.get();
        buf[..payload.len()].copy_from_slice(payload);

        let ch = DMA1.ch(1);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(payload.len() as u16));
        ch.mar().write_value(buf.as_ptr() as u32);
    }
    Ok(())
}

/// Same effect as TIM4 CC2 → DMA1_CH4 kicking CH2, just bypassing the
/// timer.
#[inline]
fn start_dma_send() {
    let run = unsafe { ptr::read_volatile(CH2_CFGR_RUN_WORD.get()) };
    DMA1.ch(1)
        .cr()
        .write_value(ch32_metapac::dma::regs::Cr(run));
}

/// Program TIM4 OPM to kick DMA1_CH2 when `tick32` reaches `at`, or send
/// immediately if the deadline is past / within scheduling overhead /
/// beyond TIM4's 16-bit ARR range.
fn schedule_or_send_now(at: u32) {
    let comp = TX_COMP_TICKS.load(Ordering::Relaxed);
    let scheduled_at = at.wrapping_sub(comp);
    let now = read_tick32();
    let delta = scheduled_at.wrapping_sub(now);
    // `delta` is u32 modular; treat very large values (= past deadline)
    // as "send now". With wrapping subtraction, "past" maps to large
    // positive deltas, so a single threshold covers both close-and-past.
    if !(IMMEDIATE_SEND_THRESHOLD_TICKS..=TIM4_MAX_DELTA_TICKS).contains(&delta) {
        store_last_send_tick(now);
        start_dma_send();
        return;
    }
    // ARR latches the next time CNT==ARR. CCR2=ARR puts CC2 on the same
    // edge as the OPM overflow.
    let arr = delta as u16;
    TIM4.atrlr().write_value(arr);
    TIM4.chcvr(1).write_value(arr); // CCR2 (0-indexed)
    store_last_send_tick(at);
    TIM4.ctlr1().modify(|w| w.set_cen(true));
}

#[derive(Copy, Clone, Debug)]
pub enum SendError {
    TooLong,
    Busy,
}

#[derive(Copy, Clone, Debug)]
pub enum BaudError {
    OutOfRange,
    Busy,
}
