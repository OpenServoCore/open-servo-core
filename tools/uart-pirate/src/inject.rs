//! DXL TX path: pre-encoded payload → DMA1_CH2 → USART3 (TX=PB10 AF OD,
//! RX=PB11 input-pullup), kicked by TIM4 OPM CC2 match chained through
//! DMA1_CH4 — no IRQ in the fire path.
//!
//! USART3 runs **full-duplex**, not HDSEL: PB10 and PB11 are bridged
//! externally on the bench, putting both halves of the USART block across
//! the single-wire DXL line. The chip then RX-captures both our own TX
//! echo (useful for FIRE_COMP autocal) and remote servo replies. HDSEL
//! would internally tri-state RX during TX, which is the opposite of
//! what we need.
//!
//! Why USART3 and not USART2 on PA2/PA3: PA3 has R10=10 kΩ to 3V3 on the
//! MuseLab board (SD-card CS pull-up). With the bus connected idle-high
//! before USB plugs in, current trickles via bus → series-R → PA3 → R10
//! → chip 3V3 rail and parasitically charges VDD above POR threshold,
//! leaving the chip stuck in a half-booted state when USB later supplies
//! real power. PB10/PB11 are clean breakouts with no onboard pulls.
//!
//! TIM4 OPM, PSC=0, ARR=delta, CCR2=ARR. On CC2 compare match, DMA1_CH4
//! stamps a precomputed armed-CFGR word (EN=1) over DMA1_CH2.CR; CH2 then
//! drains the prepared payload into USART3.DATAR. CCR2=ARR puts the CC2
//! event on the same edge as the OPM overflow.
//!
//! V20x DMA pairs are fixed: USART3_TX → DMA1_CH2, USART3_RX → DMA1_CH3.
//! TIM4_CC2 → DMA1_CH4 (same trigger as the prior USART2 build), so only
//! the stamp target moves (CH7.CR → CH2.CR). CH7 is now free.
//!
//! This module also owns TIM2 + TIM3 (the wire clock) since they need to
//! come up before USART3 / TIM4. `capture` consumes the resulting
//! `tick32` via `read_tick32()`.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::Interrupt;
use ch32_metapac::dma::vals::{Dir, Pl, Size};
use ch32_metapac::timer::vals::{CcmrInputCcs, Ckd, Mms, Urs};
use ch32_metapac::{AFIO, DMA1, GPIOA, GPIOB, RCC, TIM2, TIM3, TIM4, USART3};
use uart_pirate::parse::brr_for;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

pub const TX_BUF_LEN: usize = 1024;

/// (fire_at - now) below this many `tick32` ticks bypasses TIM4 and writes
/// the armed CFGR word directly into DMA1_CH2. 256 ticks ≈ 1.8 µs at
/// 144 MHz — safely above the arm-sequence register-write latency. A
/// too-close deadline still fires (wire-edge lands ≈ "now") instead of
/// silently missing on a wrap.
const FIRE_NOW_THRESHOLD_TICKS: u32 = 256;

/// TIM4 ARR (u16) max in tick32 units. PSC=0 → 144 MHz tick = 1:1 with
/// tick32, so ARR = delta directly and max schedule is 65 535 ticks ≈
/// 455 µs. Fires beyond this fall back to the immediate path (host should
/// never request them — DXL slot timing is sub-ms).
const TIM4_MAX_DELTA_TICKS: u32 = u16::MAX as u32;

const TIM4_PSC: u16 = 0;

/// `wire_hz`, used by the protocol layer to answer `HZ?`. TIM2/TIM3 chain
/// at HCLK; both APB buses also run at HCLK under SYSCLK_FREQ_144MHZ_HSE.
pub const WIRE_HZ: u32 = 144_000_000;
/// USART3 sits on APB1. Same rate as HCLK with prescaler DIV1.
pub const APB1_HZ: u32 = 144_000_000;
pub const DEFAULT_BAUD: u32 = 1_000_000;

/// FIRE / MASTER share this buffer. FIRE fires through TIM4 CC2 → CH4 →
/// CH7. MASTER preempts any pending FIRE by re-arming CH7 and stamping
/// directly.
static TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);

/// ARM uses a separate buffer so a MASTER+ARM chain (master TX echoes,
/// then ARM emits a faked slave slot after the bus goes idle) keeps both
/// payloads loaded simultaneously. `on_idle()` swaps DMA1_CH2 onto
/// ARM_TX_BUF before kicking the fire.
static ARM_TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);
static ARM_TX_LEN: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

/// Precomputed DMA1_CH2 CFGR word with EN=1. DMA1_CH4 copies this 32-bit
/// word over DMA1_CH2.CR on the TIM4 CC2 fire. Written once during init,
/// read-only thereafter so the DMA can treat it as constant memory.
static ARMED_CH2_CFGR_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Set by `schedule_fire_after_idle`; consumed by `on_idle`. Re-armed each
/// IDLE swap.
static ARMED_AFTER_IDLE: AtomicBool = AtomicBool::new(false);
static PENDING_AFTER_IDLE_TICKS: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// `tick32` at the last fire kickoff (commanded value for scheduled fires,
/// "now" for immediate fires). Host correlates against per-byte stamps.
static FIRED_TICK: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Fire-path latency compensation decomposed into the two clock domains
/// the TX chain crosses (TIM4 CC2 match → wire start-bit edge):
///
///   FIRE_COMP_TICKS = TX_PIPELINE_TICKS
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
/// `FIRE_COMP_TICKS` is the precomputed sum read by the fire fast path —
/// recomputed by `recompute_fire_comp` whenever brr or either tunable
/// changes. Defaults below are the converged values from
/// `tool-pirate-tune --stage fire-comp` on osc-dev-v20x at 144 MHz HCLK
/// (median across 5 runs at n=64 shots/baud; per-run wobble ±~5 ticks
/// on pipe, ±1 on bit_q4 — bounded by the U[0, brr] median SE floor).
static FIRE_COMP_TICKS: AtomicU32 = AtomicU32::new(0);
static TX_PIPELINE_TICKS: AtomicU32 = AtomicU32::new(TX_PIPELINE_TICKS_DEFAULT);
static TX_BIT_TIMES_Q4: AtomicU32 = AtomicU32::new(TX_BIT_TIMES_Q4_DEFAULT);

const TX_PIPELINE_TICKS_DEFAULT: u32 = 45;
const TX_BIT_TIMES_Q4_DEFAULT: u32 = 8;

fn recompute_fire_comp(brr: u32) {
    let pipe = TX_PIPELINE_TICKS.load(Ordering::Relaxed);
    let q4 = TX_BIT_TIMES_Q4.load(Ordering::Relaxed);
    FIRE_COMP_TICKS.store(pipe + ((q4 * brr) >> 4), Ordering::Relaxed);
}

pub fn tx_comp() -> (u32, u32) {
    (
        TX_PIPELINE_TICKS.load(Ordering::Relaxed),
        TX_BIT_TIMES_Q4.load(Ordering::Relaxed),
    )
}

/// Update one or both comp tunables, then recompute `FIRE_COMP_TICKS` for
/// the current baud. Held under a critical section so a fire racing the
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
        recompute_fire_comp(brr);
    });
}

/// Headroom added to `read_systick()` when [`fire_now_master`] schedules
/// its near-future fire through [`schedule_or_fire_now`]. Sized so the
/// resulting `delta` lands comfortably above [`FIRE_NOW_THRESHOLD_TICKS`]
/// — keeping the scheduled (TIM4 OPM) path in play rather than falling
/// back to the immediate DMA-direct branch, so `FIRED_TICK` matches the
/// commanded wire-start tick. 512 ticks ≈ 3.5 µs at 144 MHz — negligible
/// added latency, well above the 256-tick scheduler threshold.
const MASTER_MARGIN_HEADROOM_TICKS: u32 = 512;

/// Upper bound on how long `wait_tx_complete` will spin for `USART3.SR.TC`.
/// 200 ms at 144 MHz. Generous enough for any payload up to TX_BUF_LEN at
/// 9600 baud (≈ 1.1 s for 1024 bytes — we cap shorter than that on
/// purpose: a stuck TC means something genuinely broke and the host should
/// see a `busy` error rather than the chip pretending to work).
const TX_WAIT_TIMEOUT_TICKS: u32 = 28_800_000;

#[inline]
fn store_fired_tick(t: u32) {
    unsafe { ptr::write_volatile(FIRED_TICK.get(), t) }
}
#[inline]
fn store_pending_after_idle(v: u32) {
    unsafe { ptr::write_volatile(PENDING_AFTER_IDLE_TICKS.get(), v) }
}
#[inline]
fn load_pending_after_idle() -> u32 {
    unsafe { ptr::read_volatile(PENDING_AFTER_IDLE_TICKS.get()) }
}

/// Coherent (TIM2, TIM3) snapshot. Three loads in the common case, six
/// if TIM2 wraps between the first read and the TIM3 read.
#[inline]
pub fn read_tick32() -> u32 {
    loop {
        let lo_1 = TIM2.cnt().read();
        let hi = TIM3.cnt().read();
        let lo_2 = TIM2.cnt().read();
        if lo_2 >= lo_1 {
            return ((hi as u32) << 16) | (lo_2 as u32);
        }
    }
}

pub fn init() {
    init_clocks_and_remap();
    init_pins();
    init_usart3();
    init_tim2_tim3_capture();
    init_tim4_scheduler();
    init_dma_tx();

    unsafe {
        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL6 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL3 as u8);
    }
}

/// Clocks (USART3 + TIM2/3/4 on APB1, GPIOA/B + AFIO on APB2, DMA1 on
/// AHB) and AFIO remap. USART3 default mapping is PB10/PB11 (no remap).
/// TIM2 partial remap #2 puts CH3 on PB10 (CH4 on PB11) while leaving
/// CH1/CH2 on PA0/PA1. CC3 isn't itself wired to a channel — PB10's
/// falling edge propagates through TI1 via the TI1S XOR (set in
/// `init_tim2_tim3_capture`) and triggers IC1 capture.
fn init_clocks_and_remap() {
    RCC.apb2pcenr().modify(|w| {
        w.set_iopaen(true);
        w.set_iopben(true);
        w.set_afioen(true);
    });
    RCC.apb1pcenr().modify(|w| {
        w.set_usart3en(true);
        w.set_tim2en(true);
        w.set_tim3en(true);
        w.set_tim4en(true);
    });
    RCC.ahbpcenr().modify(|w| w.set_dma1en(true));

    AFIO.pcfr1().modify(|w| {
        w.set_usart3_rm(0); // 00 = default PB10/PB11
        w.set_tim2_rm(0b10); // 10 = CH3/CH4 → PB10/PB11
    });
}

/// PA0/PA1 (TIM2_RM=0b10 CH1/CH2): input pull-down so the TI1S XOR in
/// `init_tim2_tim3_capture` collapses to PB10 alone. PB10/PB11
/// (USART3_TX/RX): TX as AF push-pull (50 MHz), RX as input pull-up.
///
/// SAFETY: PA0/PA1 must be physically unrouted on the board. A future
/// variant that wires either pin to external signal will inject XOR
/// noise and break IC capture; re-evaluate this block before that
/// point.
///
/// Push-pull on PB10 (vs the spec-correct open-drain) is bench-only:
/// PB10↔PB11 are bridged through open air with no transceiver, so the
/// only pull on the wire is PB11's ~30 kΩ internal pull-up, giving
/// τ ≈ 500–900 ns rise time and USART RX mis-sample at ≥ 2 Mbaud. Push-
/// pull actively drives both edges. The DXL multi-drop "no PP fighting
/// OD" concern doesn't apply to this rig.
fn init_pins() {
    GPIOA.outdr().modify(|w| {
        w.set_odr(0, false);
        w.set_odr(1, false);
    });
    // MODE=00 (input), CNF=10 (input with pull-up/down) → 0b1000.
    // ODR(0)=0 / ODR(1)=0 above selects pull-down.
    GPIOA.cfglr().modify(|w| {
        let mut v = w.0;
        v &= !0xFF;
        v |= 0b1000u32; // PA0
        v |= 0b1000u32 << 4; // PA1
        w.0 = v;
    });

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
    recompute_fire_comp(brr0);
}

/// TIM2 master + TIM3 high-half, forming the hardware atomic 32-bit IC
/// tick32 path. TIM2 PSC=0, ARR=0xFFFF, CKD=DIV_1 pins fDTS at HCLK =
/// 144 MHz; the IC1F filter is set per-baud by
/// `capture::apply_filter_for_brr` (largest delay ≤ brr/3). No CC walker
/// cadence: the event-driven walker runs off USART3 IDLE + DMA1_CH6/CH3
/// HT/TC, not TIM2 IRQs.
///
/// Hardware atomic 32-bit IC capture:
///
/// - CTLR2.TI1S=1 routes XOR(CH1_pin, CH2_pin, CH3_pin) → TI1 internal.
///   PA0/PA1 (CH1/CH2 under TIM2_RM=0b10) are pulled down in
///   `init_pins`, so TI1 mirrors PB10's falling edges directly.
/// - CCMR1.CC1S=TI4 (= "normal" CC1 ← TI1). CCER CC1P=1 falling,
///   CC1E=1. DMAINTENR CC1DE=1 kicks DMA1_CH5 (TIM2.CCR1 → low half of
///   every IC entry).
/// - CTLR2.MMS=COMPARE_PULSE: TRGO emits a one-cycle pulse on every
///   CC1IF set, driving TIM3 TRC → TIM3.CCR1 latches the matching high
///   half. The pair forms an atomic 32-bit tick.
///
/// The 1-cycle TRC-sync race window at every TIM2 wrap leaves bad pairs
/// off-by-+65536 (combined > ceiling), which the walker detects and
/// corrects by subtracting one wrap.
///
/// Phase-lock startup: enable TIM3.CEN before TIM2.CEN so TIM3's
/// prescaler leads TIM2's counter by the AHB write gap. TIM3.CNT
/// increments a few cycles before TIM2 wraps, biasing any wrap-race IC
/// pair to off-by-+65536 which the walker detects unambiguously.
fn init_tim2_tim3_capture() {
    TIM2.psc().write_value(0);
    TIM2.atrlr().write_value(0xFFFF);
    TIM2.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
        w.set_ckd(Ckd::DIV_1);
    });
    TIM2.ctlr2().modify(|w| {
        w.set_mms(Mms::COMPARE_PULSE);
        w.set_ti1s(true);
    });
    TIM2.chctlr_input(0).modify(|w| {
        // CCMR1. CC1S in bits [1:0]. IC1F (bits [7:4]) is written
        // separately by `capture::init` from the per-baud LUT.
        w.set_ccs(0, CcmrInputCcs::TI4); // normal mapping → IC1 ← TI1
    });
    TIM2.ccer().modify(|w| {
        w.set_ccp(0, true); // CC1P=1 falling edge
        w.set_cce(0, true); // CC1E=1
    });
    TIM2.dmaintenr().write(|w| {
        w.set_ccde(0, true); // CC1DE (IC capture → DMA1_CH5)
    });
    TIM2.swevgr().write(|w| w.set_ug(true)); // load PSC/ARR
    TIM2.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(0, false);
        w.set_ccif(1, false);
        w.set_ccif(2, false);
        w.set_ccif(3, false);
    });

    // TIM3: PSC=0xFFFF → CK_CNT = HCLK/65536 = TIM2 wrap rate. No slave
    // mode (SMS=0); TS=1 keeps TRC routed to ITR1 = TIM2 TRGO.
    // CCMR1.CC1S=TRC latches TIM3.CCR1 on every TRGO pulse; CCER CC1P=0
    // picks the rising edge of the one-cycle pulse. DMAINTENR CC1DE=1
    // kicks DMA1_CH6 (TIM3.CCR1 → high half of every IC entry).
    TIM3.psc().write_value(0xFFFF);
    TIM3.atrlr().write_value(0xFFFF);
    TIM3.ctlr1().write(|w| {
        w.set_cen(false);
        w.set_arpe(false);
    });
    TIM3.smcfgr().modify(|w| {
        w.set_sms(0); // disable slave clock — TIM3 free-runs on CK_INT
        w.set_ts(1); // ITR1 = TIM2 TRGO → TRC
    });
    TIM3.chctlr_input(0).modify(|w| {
        w.set_ccs(0, CcmrInputCcs::TRC); // CC1 from TRC
    });
    TIM3.ccer().modify(|w| {
        w.set_ccp(0, false); // CC1P=0 rising edge of TRGO pulse
        w.set_cce(0, true);
    });
    TIM3.dmaintenr().write(|w| {
        w.set_ccde(0, true); // CC1DE → DMA1_CH6
    });
    TIM3.swevgr().write(|w| w.set_ug(true));
    TIM3.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(0, false);
    });

    critical_section::with(|_| {
        TIM3.ctlr1().modify(|w| w.set_cen(true));
        TIM2.ctlr1().modify(|w| w.set_cen(true));
    });
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
/// on fire. CH2.EN stays clear at init; `load_payload` reloads
/// MAR/NDTR with EN=0 between fires. CH4 is circular single-word: copy
/// ARMED_CH2_CFGR_WORD → DMA1_CH2.CR. NDTR=1 auto-reloads each cycle
/// (CIRC=1), so we never re-arm CH4 from software. CH4.EN=1
/// permanently — the CC2DE gate fires it, not EN.
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
        // inter-channel arbitration doesn't trigger, but pins the fire
        // path at top priority against any future DMA channel we add.
        // (Empirically does NOT tighten TX wire-edge jitter — the
        // observed loopback variance is CPU-vs-DMA AHB arbitration,
        // which channel-priority bits don't control.)
        w.set_pl(Pl::VERYHIGH);
        w.set_tcie(false);
    });

    // Precompute the EN=1 CFGR word DMA1_CH4 stamps on fire. Same bits
    // as the disarmed config above, plus EN=1.
    let mut armed = ch32_metapac::dma::regs::Cr(0);
    armed.set_dir(Dir::FROMMEMORY);
    armed.set_minc(true);
    armed.set_pinc(false);
    armed.set_circ(false);
    armed.set_msize(Size::BITS8);
    armed.set_psize(Size::BITS8);
    armed.set_pl(Pl::VERYHIGH); // mirror CH2 PL above so the stamp doesn't drop priority on fire
    armed.set_tcie(false);
    armed.set_en(true);
    // SAFETY: ARMED_CH2_CFGR_WORD is only read by DMA1_CH4 (set up
    // below). CH4 isn't enabled yet, so no concurrent reader.
    unsafe { ptr::write_volatile(ARMED_CH2_CFGR_WORD.get(), armed.0) };

    let ch4 = DMA1.ch(3);
    ch4.par().write_value(ch2.cr().as_ptr() as u32);
    ch4.mar().write_value(ARMED_CH2_CFGR_WORD.get() as u32);
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

/// Load `payload` and fire when `tick32` reaches `at`. Held under a
/// critical section so a pending USART3 IDLE can't fire the DMA between
/// disabling EN and writing NDTR/MAR.
pub fn schedule_fire(payload: &[u8], at: u32) -> Result<(), ArmError> {
    critical_section::with(|_| {
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        disarm_tim4();
        load_payload_main(payload)?;
        schedule_or_fire_now(at);
        Ok(())
    })
}

/// Load `payload` into ARM_TX_BUF and fire `after_idle_ticks` after the
/// next USART3 IDLE assertion. Note: IDLE asserts ~1 character time after
/// wire-end; host should subtract that if sub-byte alignment matters.
pub fn schedule_fire_after_idle(payload: &[u8], after_idle_ticks: u32) -> Result<(), ArmError> {
    critical_section::with(|_| -> Result<(), ArmError> {
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        if payload.len() > TX_BUF_LEN {
            return Err(ArmError::TooLong);
        }
        // SAFETY: ARMED_AFTER_IDLE=false above prevents on_idle from
        // reading ARM_TX_BUF until we set true below.
        unsafe {
            let buf = &mut *ARM_TX_BUF.get();
            buf[..payload.len()].copy_from_slice(payload);
            ptr::write_volatile(ARM_TX_LEN.get(), payload.len() as u16);
        }
        store_pending_after_idle(after_idle_ticks);
        ARMED_AFTER_IDLE.store(true, Ordering::Release);
        Ok(())
    })
}

/// Fire `payload` as the bus master with no precise timing — commands a
/// near-future fire at `now + FIRE_COMP_TICKS + MASTER_MARGIN_HEADROOM_TICKS`
/// so the scheduled (TIM4 OPM) path always wins and `FIRED_TICK` reflects
/// the actual wire-start tick. Wire-start lands ~3.5 µs after the call.
///
/// Preempts any pending `schedule_fire`; coexists with a pending
/// `schedule_fire_after_idle` (separate buffer). Waits for any in-flight
/// TX to drain first so a back-to-back MASTER from the host can't
/// truncate the previous frame.
pub fn fire_now_master(payload: &[u8]) -> Result<(), ArmError> {
    wait_tx_complete().map_err(|TxTimeout| ArmError::Busy)?;
    // CS keeps the `at` computation atomic with `schedule_fire` so an ISR
    // between read_tick32 and the schedule can't eat the
    // MASTER_MARGIN_HEADROOM and push us onto the immediate-fire branch.
    // Nested CS (schedule_fire opens its own) refcount-stacks under qingke.
    critical_section::with(|_| {
        let comp = FIRE_COMP_TICKS.load(Ordering::Relaxed);
        let at = read_tick32()
            .wrapping_add(comp)
            .wrapping_add(MASTER_MARGIN_HEADROOM_TICKS);
        schedule_fire(payload, at)
    })
}

fn disarm_tim4() {
    TIM4.ctlr1().modify(|w| w.set_cen(false));
    // URS=1 blocks UEV from this UG, so DMA1_CH4 isn't kicked.
    TIM4.swevgr().write(|w| w.set_ug(true));
    TIM4.intfr().write(|w| {
        w.set_uif(false);
        w.set_ccif(1, false);
    });
}

/// Called from `capture::on_usart3_idle` after an IDLE assertion. When
/// armed-after-idle, swaps DMA1_CH2 to point at ARM_TX_BUF and fires
/// `after_idle_ticks` after the IDLE-assertion tick32.
pub fn on_idle(idle_tick: u32) {
    if !ARMED_AFTER_IDLE.swap(false, Ordering::AcqRel) {
        return;
    }
    let after = load_pending_after_idle();

    // A pipelined FIRE(future) + ARM that races IDLE can leave TIM4 with
    // CEN=1 mid-schedule; disarm so schedule_or_fire_now starts from a
    // known-idle TIM4 with our reloaded buffer.
    disarm_tim4();

    // Swap DMA1_CH2 to ARM_TX_BUF.
    // SAFETY: USART3 IDLE ISR is the sole reader of ARM_TX_BUF/ARM_TX_LEN;
    // schedule_fire_after_idle is the sole writer, and ARMED_AFTER_IDLE
    // gating means it can't be mid-write here (the swap above ate the
    // flag).
    unsafe {
        let len = ptr::read_volatile(ARM_TX_LEN.get());
        let ch = DMA1.ch(1);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(len));
        ch.mar().write_value((*ARM_TX_BUF.get()).as_ptr() as u32);
    }

    let fire_at = idle_tick.wrapping_add(after);
    schedule_or_fire_now(fire_at);
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
        recompute_fire_comp(brr);
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        disarm_tim4();
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

pub fn last_fired_tick() -> u32 {
    unsafe { ptr::read_volatile(FIRED_TICK.get()) }
}

pub const fn wire_ticks_per_us() -> u32 {
    WIRE_HZ / 1_000_000
}

fn load_payload_main(payload: &[u8]) -> Result<(), ArmError> {
    if payload.len() > TX_BUF_LEN {
        return Err(ArmError::TooLong);
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

/// Same effect as TIM4 CC2 → DMA1_CH4 firing, just bypassing the timer.
#[inline]
fn fire_now_dma() {
    let armed = unsafe { ptr::read_volatile(ARMED_CH2_CFGR_WORD.get()) };
    DMA1.ch(1)
        .cr()
        .write_value(ch32_metapac::dma::regs::Cr(armed));
}

/// Program TIM4 OPM to fire DMA1_CH2 when `tick32` reaches `at`, or fire
/// immediately if the deadline is past / within arm overhead / beyond
/// TIM4's 16-bit ARR range.
fn schedule_or_fire_now(at: u32) {
    let comp = FIRE_COMP_TICKS.load(Ordering::Relaxed);
    let scheduled_at = at.wrapping_sub(comp);
    let now = read_tick32();
    let delta = scheduled_at.wrapping_sub(now);
    // `delta` is u32 modular; treat very large values (= past deadline)
    // as "fire now". With wrapping subtraction, "past" maps to large
    // positive deltas, so a single threshold covers both close-and-past.
    if !(FIRE_NOW_THRESHOLD_TICKS..=TIM4_MAX_DELTA_TICKS).contains(&delta) {
        store_fired_tick(now);
        fire_now_dma();
        return;
    }
    // ARR latches the next time CNT==ARR. CCR2=ARR puts CC2 on the same
    // edge as the OPM overflow.
    let arr = delta as u16;
    TIM4.atrlr().write_value(arr);
    TIM4.chcvr(1).write_value(arr); // CCR2 (0-indexed)
    store_fired_tick(at);
    TIM4.ctlr1().modify(|w| w.set_cen(true));
}

#[derive(Copy, Clone, Debug)]
pub enum ArmError {
    TooLong,
    Busy,
}

#[derive(Copy, Clone, Debug)]
pub enum BaudError {
    OutOfRange,
    Busy,
}
