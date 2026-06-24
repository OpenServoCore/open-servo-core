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
//! This module also owns TIM2 + TIM3 (the wire clock; see TIMING.md §1)
//! since they need to come up before USART3 / TIM4. `capture` consumes the
//! resulting `tick32` via `read_tick32()`.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_hal::pac::Interrupt;
use ch32_hal::pac::dma::vals::{Dir, Pl, Size};
use ch32_hal::pac::timer::vals::{CcmrInputCcs, Ckd, Mms, Urs};
use ch32_hal::pac::{AFIO, DMA1, GPIOB, RCC, TIM2, TIM3, TIM4, USART3};
use dxl_pirate::parse::brr_for;
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

/// TIM4 prescaler register: divide-by-1 → 144 MHz tick = 1:1 with TIM2.
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

/// Per-baud fire-path latency compensation in `tick32` ticks. Subtracted
/// from the commanded fire-at inside `schedule_or_fire_now` so the wire
/// start bit lands at the commanded tick.
///
/// Decomposition:
///   FIRE_COMP = FIRE_COMP_FLAT_TICKS (DMA chain + CC2 sync overhead)
///             + bit_time / 2          (USART start-bit BRR sync avg)
/// Retuned per `set_baud`. Phase-C autocal will retune
/// `FIRE_COMP_FLAT_TICKS` empirically against a loopback measurement.
static FIRE_COMP_TICKS: AtomicU32 = AtomicU32::new(0);

const FIRE_COMP_FLAT_TICKS: u32 = 96;

const fn fire_comp_ticks(brr: u32) -> u32 {
    FIRE_COMP_FLAT_TICKS + brr / 2
}

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

/// Coherent (TIM2, TIM3) snapshot per TIMING.md §3.3. Three loads in the
/// common case, six if TIM2 wraps between the first read and the TIM3
/// read.
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
    unsafe {
        // ── Clocks. USART3 + TIM2/3/4 on APB1; GPIOB + AFIO on APB2;
        // DMA1 on AHB.
        RCC.apb2pcenr().modify(|w| {
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

        // USART3 default mapping is PB10/PB11 (no remap). TIM2 partial
        // remap #2 moves CH3/CH4 to PB10/PB11 while leaving CH1/CH2 on
        // PA0/PA1. TIM2_CH3 IC then reads PB10 via GPIO_INDR regardless of
        // USART3's AF state; PB11 is bridged externally to PB10 so the IC
        // sees the same wire edge a hypothetical PB11 tap would.
        AFIO.pcfr1().modify(|w| {
            w.set_usart3_rm(0); // 00 = default PB10/PB11
            w.set_tim2_rm(0b10); // 10 = CH3/CH4 → PB10/PB11
        });

        // ── PB10 = USART3_TX, AF **push-pull**, 50 MHz. The bench wiring
        // has no servo, no transceiver, just PB10↔PB11 bridged through
        // open air — the only pull on the wire is PB11's ~30 kΩ internal
        // pull-up, which gives τ ≈ 500–900 ns rise time and makes USART
        // RX mis-sample at ≥ 2 Mbaud. Push-pull actively drives both
        // edges, so the wire matches transmit timing at any baud the
        // chip can clock. The DXL multi-drop "no PP fighting OD" concern
        // doesn't apply to this bench rig.
        //
        // ODR high before AF lock: guards against a transient ODR-LOW
        // pulling the bus while the AF block is mid-init.
        GPIOB.outdr().modify(|w| {
            w.set_odr(10, true);
            w.set_odr(11, true); // select PB11 input pullup (see below)
        });
        // CFGHR controls PB8..PB15.
        //   PB10 in bits [11:8]: Mode=11 (50 MHz), CNF=10 (AF PP) → 0b1011.
        //   PB11 in bits [15:12]: Mode=00 (input),   CNF=10 (input w/ pull)
        //                         → 0b1000; ODR(11)=1 above selects pull-up.
        GPIOB.cfghr().modify(|w| {
            let mut v = w.0;
            v &= !(0xF << 8);
            v &= !(0xF << 12);
            v |= 0b1011u32 << 8;
            v |= 0b1000u32 << 12;
            w.0 = v;
        });

        // ── USART3: 8N1, full-duplex, DMAT, DMAR, IDLEIE. Init order
        // matters — TE/RE with UE=0, then DMAT/DMAR, then UE in its own
        // write, to avoid the TX-line glitch the STM32-family USARTs
        // throw when TE+UE land in the same write.
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
        FIRE_COMP_TICKS.store(fire_comp_ticks(brr0), Ordering::Relaxed);

        // ── TIM2 master, low 16 of tick32. PSC=0, ARR=0xFFFF, MMS=update
        // → TRGO on each wrap drives TIM3 slave clock. No CC1/CC2/CC4
        // walker cadence: the event-driven walker (TIMING.md §3.2) runs
        // off USART3 IDLE + DMA1_CH1/CH3 HT/TC, not TIM2 IRQs. TIM2 IRQ
        // stays disabled at the PFIC.
        //
        // CC3 input capture on TI3 (= PB10 via TIM2_RM=0b10).
        // CKD=DIV_1 pins fDTS at HCLK = 144 MHz; the IC3F filter is set
        // per-baud by `capture::apply_filter_for_brr` from the LUT in
        // TIMING.md §4 (largest delay ≤ brr/3). CCER CC3P=1 → falling-
        // edge sensitivity. CC3E=1 → capture enabled. DIER CC3DE=1 →
        // each capture kicks DMA1_CH1.
        TIM2.psc().write_value(0);
        TIM2.atrlr().write_value(0xFFFF);
        TIM2.ctlr1().write(|w| {
            w.set_cen(false);
            w.set_arpe(false);
            w.set_ckd(Ckd::DIV_1);
        });
        TIM2.ctlr2().modify(|w| w.set_mms(Mms::UPDATE));
        TIM2.chctlr_input(1).modify(|w| {
            // chctlr_input(1) = CCMR2. CC3S in bits [1:0]. IC3F (bits
            // [7:4]) is written separately by `capture::init` from the
            // per-baud LUT.
            w.set_ccs(0, CcmrInputCcs::TI4); // "normal" mapping → IC3 ← TI3
        });
        TIM2.ccer().modify(|w| {
            w.set_ccp(2, true); // CC3P=1 falling edge
            w.set_cce(2, true); // CC3E=1
        });
        TIM2.dmaintenr().write(|w| {
            w.set_ccde(2, true); // CC3DE (IC capture → DMA1_CH1)
        });
        TIM2.swevgr().write(|w| w.set_ug(true)); // load PSC/ARR
        TIM2.intfr().write(|w| {
            w.set_uif(false);
            w.set_ccif(0, false);
            w.set_ccif(1, false);
            w.set_ccif(2, false);
            w.set_ccif(3, false);
        });
        TIM2.ctlr1().modify(|w| w.set_cen(true));

        // ── TIM3 slave, high 16. SMS=7 (external clock mode 1), TS=1
        // (ITR1 = TIM2 TRGO).
        TIM3.psc().write_value(0);
        TIM3.atrlr().write_value(0xFFFF);
        TIM3.ctlr1().write(|w| w.set_cen(false));
        TIM3.smcfgr().modify(|w| {
            w.set_sms(7);
            w.set_ts(1);
        });
        TIM3.swevgr().write(|w| w.set_ug(true));
        TIM3.intfr().write(|w| w.set_uif(false));
        TIM3.ctlr1().modify(|w| w.set_cen(true));

        // ── TIM4 OPM. PSC=0 (1:1 with tick32). CR1: OPM=1 (auto-clear CEN
        // after first UEV), URS=1 (UG software event resets CNT without
        // generating a UEV that would spuriously kick CC2). DIER CC2DE=1
        // routes CC2 compare match to DMA1_CH4.
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

        // ── DMA1_CH2 = USART3_TX. EN stays clear at init; CC2 stamp
        // flips it via DMA1_CH4. `load_payload` reloads MAR/NDTR with
        // EN=0 between fires.
        let ch2 = DMA1.ch(1);
        ch2.par().write_value(USART3.datar().as_ptr() as u32);
        ch2.cr().write(|w| {
            w.set_dir(Dir::FROMMEMORY);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(false);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_pl(Pl::MEDIUM); // TX paced by USART; arbiter delay is harmless
            w.set_tcie(false);
        });

        // Precompute the EN=1 CFGR word DMA1_CH4 stamps on fire. Same
        // bits as the disarmed config above, plus EN=1.
        let mut armed = ch32_hal::pac::dma::regs::Cr(0);
        armed.set_dir(Dir::FROMMEMORY);
        armed.set_minc(true);
        armed.set_pinc(false);
        armed.set_circ(false);
        armed.set_msize(Size::BITS8);
        armed.set_psize(Size::BITS8);
        armed.set_pl(Pl::MEDIUM); // mirror disarmed CH2 PL so the stamp doesn't drop priority
        armed.set_tcie(false);
        armed.set_en(true);
        ptr::write_volatile(ARMED_CH2_CFGR_WORD.get(), armed.0);

        // ── DMA1_CH4 = TIM4_CC2 trigger. Circular, single-word transfer:
        // copy ARMED_CH2_CFGR_WORD → DMA1_CH2.CR. NDTR=1 auto-reloads
        // each cycle thanks to CIRC=1, so we never re-arm CH4 from
        // software. EN=1 permanently — the CC2DE gate fires it, not EN.
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
            w.set_pl(Pl::MEDIUM); // one-shot per fire; no throughput pressure
            w.set_tcie(false);
            w.set_en(true);
        });

        qingke::pfic::enable_interrupt(Interrupt::USART3 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL1 as u8);
        qingke::pfic::enable_interrupt(Interrupt::DMA1_CHANNEL3 as u8);
    }
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
        crate::debug::clear();
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

/// Fire `payload` immediately as the bus master. Preempts any pending
/// `schedule_fire`; coexists with a pending `schedule_fire_after_idle`
/// (separate buffer). Waits for any in-flight TX to drain first so a
/// back-to-back MASTER from the host can't truncate the previous frame.
pub fn fire_now_master(payload: &[u8]) -> Result<(), ArmError> {
    wait_tx_complete().map_err(|TxTimeout| ArmError::Busy)?;
    critical_section::with(|_| -> Result<(), ArmError> {
        disarm_tim4();
        load_payload_main(payload)?;
        store_fired_tick(read_tick32());
        crate::debug::clear();
        fire_now_dma();
        Ok(())
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
        FIRE_COMP_TICKS.store(fire_comp_ticks(brr), Ordering::Relaxed);
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
        .write_value(ch32_hal::pac::dma::regs::Cr(armed));
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
        crate::debug::clear();
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
