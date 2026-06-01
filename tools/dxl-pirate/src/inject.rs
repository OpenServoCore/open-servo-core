//! DXL Fast slot injector hot path: pre-encoded payload → DMA1_CH4 → USART1
//! (HDSEL) kicked off by TIM4 OPM update event chained through DMA1_CH7 — no
//! IRQ in the fire path, ≤100 ns wire-edge jitter from the scheduled tick.
//!
//! Architecture:
//!   1. Payload pre-loaded into DMA1_CH4 (USART1_TX), channel EN=0.
//!   2. arm()/arm_after_idle() program TIM4 OPM ARR = (fire_at - now), CEN=1.
//!   3. When CNT == ARR, TIM4 UEV fires → UDE routes to DMA1_CH7 → CH7 writes
//!      precomputed armed-CFGR word (EN=1) into DMA1_CH4.CFGR.
//!   4. USART1 TXE is permanently asserted (DMAT=1), so as soon as CH4 sees
//!      EN=1 it pushes bytes into DATAR; on-wire start bit follows by ≤1 bit
//!      period.
//!
//! Timer choice: TIM1 was the original pick but TIM1_CH2 default-maps to PA9
//! (same as USART1_TX) on LQFP48, and even with CC2E=0+MOE=0 the implicit AF
//! mux lets TIM1's PP output dominate USART1's OD line — bus driven LOW.
//! TIM1 has no remap option that frees PA9 on LQFP48. TIM4 channels live on
//! PB6/PB7/PB8/PB9, no PA9 overlap; TIM4_UP routes to DMA1_CH7.
//!
//! HAL owns clocks + USB; everything else is direct metapac so the DMA chain
//! is a handful of register writes with no abstraction layers between us and
//! the bus.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_hal::pac::Interrupt;
use ch32_hal::pac::dma::vals::{Dir, Size};
use ch32_hal::pac::timer::vals::Urs;
use ch32_hal::pac::{DMA1, GPIOA, RCC, SYSTICK, TIM4, USART1};
use dxl_pirate::parse::brr_for;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
use qingke_rt::interrupt;

pub const TX_BUF_LEN: usize = 1024;

/// (fire_at - now) below this many SysTick ticks bypasses TIM4 and writes the
/// armed CFGR word directly into DMA1_CH4. Equal to (TIM4 arm overhead +
/// PFIC dispatch margin) — 32 ticks ≈ 1.8 µs at 18 MHz, safely above the
/// register-write latency for the arm sequence. A too-close deadline still
/// fires (wire-edge lands ≈ "now") instead of silently missing on a wrap.
const FIRE_NOW_THRESHOLD_TICKS: u64 = 32;

/// TIM4 ARR (u16) max in SysTick units. With PSC=7 the timer ticks at
/// HCLK/8 = 18 MHz = 1:1 with SysTick, so ARR = delta_systick directly and
/// max schedule is 65 535 ticks ≈ 3.6 ms. Fires beyond this fall back to
/// immediate (host should never request them — slot timing is sub-ms).
const TIM4_MAX_DELTA_TICKS: u64 = u16::MAX as u64;

/// TIM4 prescaler register: divide-by-8 → 18 MHz tick = 1:1 with SysTick.
const TIM4_PSC: u16 = 7;

/// FIRE / MASTER share this buffer; both fire from DMA1_CH4 (FIRE on a TIM4
/// UEV match via DMA1_CH7, MASTER on its own dispatch). MASTER preempts any
/// pending FIRE.
static TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);

/// ARM uses a separate buffer so a MASTER+ARM chain (= "fire master request,
/// then emit a faked foreign-slave slot one RDT after the master IDLE")
/// keeps both payloads loaded simultaneously. The on_listen_idle swap path
/// reloads the DMA channel to point here before kicking the fire.
static ARM_TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);
/// ARM payload length (bytes loaded into ARM_TX_BUF). ISR-only writer is
/// arm_after_idle; ISR-only reader is on_listen_idle's swap path.
static ARM_TX_LEN: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

/// Precomputed DMA1_CH4 CFGR word with EN=1; DMA1_CH7 copies this byte-pattern
/// over DMA1_CH4.CFGR on the TIM4 fire to enable the USART1_TX channel
/// without an IRQ. Init writes the value once; the cell is read-only
/// thereafter so the DMA can safely treat it as constant memory.
static ARMED_CH4_CFGR_WORD: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

// USART1 on APB2, TIM4 on APB1. With SYSCLK_FREQ_144MHZ_HSE both APB1/2 run at
// 144 MHz (prescaler DIV1) so TIM4 ticks at the same rate as USART1's BRR base.
pub const APB2_HZ: u32 = 144_000_000;
pub const APB1_HZ: u32 = 144_000_000;
pub const DEFAULT_BAUD: u32 = 1_000_000;

// SysTick on V4 ticks at HCLK/8 = 18 MHz with the 144 MHz preset, so 1 µs ≈ 18.
const SYSTICK_HZ: u32 = 144_000_000 / 8;

/// arm_after_idle arms the payload + offset and sets this; the next IDLE the
/// listener observes consumes it via swap. The swap is the only RMW —
/// FIRED_TICK_LO and PENDING_AFTER_IDLE_TICKS below are single-word,
/// single-writer-at-a-time cells accessed via volatile read/write (V203 is
/// single-core, aligned u32 stores are tear-free).
static ARMED_AFTER_IDLE: AtomicBool = AtomicBool::new(false);
static FIRED_TICK_LO: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static PENDING_AFTER_IDLE_TICKS: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// USART1 TXC stamp: SysTick.CNTL captured the moment the master's last byte
/// finishes shifting out (TC asserts post-shift, unlike TXE which fires when
/// the holding register empties). This is T_request_end in the cal model.
/// Sole writer = USART1 ISR.
static MASTER_T_REQUEST_END: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// master_send sets; listen's USART3 IDLE ISR consumes via swap and skips
/// publishing that stamp. The master's own TX echoes into the USART3 listener
/// and produces one IDLE we don't want polluting the slave-side stamp ring.
pub(crate) static SUPPRESS_NEXT_IDLE_STAMP: AtomicBool = AtomicBool::new(false);

/// Gate for the RXNE handler's T_FIRST stamp. Set ONLY by the USART3
/// IDLE-suppress branch (= master TX echo IDLE just fired). The RXNE handler
/// swap-clears it on the next received byte. Cleared (not set) by master_send
/// so a stale RXNEIE left armed by a no-reply prior trip can fire on our
/// master TX-echo byte without misattributing it as the slave-reply T0.
pub(crate) static EXPECT_FIRST_BYTE: AtomicBool = AtomicBool::new(false);

/// The USART3 IDLE handler sets this when it consumes SUPPRESS (= master TX
/// echo IDLE just fired). The next non-suppressed IDLE is the slave-reply
/// wire-end, and the handler publishes a Round stamp instead of Plain, then
/// clears this flag. master_send also resets it so a no-reply trip doesn't
/// taint the next round.
pub(crate) static EXPECT_REPLY_END_IDLE: AtomicBool = AtomicBool::new(false);

/// Set by `arm()` (= FIRE command). Consumed by the listener's RXNE handler
/// on the first byte after fire — captures `FIRE_T_FIRST` for the host
/// jitter script. Independent of the master-send `EXPECT_FIRST_BYTE` flow
/// so FIRE and MASTER paths don't collide. Cleared by the RXNE swap.
pub(crate) static EXPECT_FIRE_FIRST_BYTE: AtomicBool = AtomicBool::new(false);

/// Per-baud fire-path latency compensation in SysTicks. Subtracted from the
/// commanded `fire_at_tick` inside `schedule_or_fire_now` so the wire start
/// bit lands at the commanded tick on wall-clock terms.
///
/// Decomposition (measured by `scripts/pirate_jitter.py`):
///   FIRE_COMP = 11 (flat DMA chain + IRQ entry) + bit_time_systicks / 2
///                                                 (USART start-bit BRR sync avg)
/// The flat 11 SysTicks (~611 ns) is the TIM4 OPM CC match → DMA1_CH7 →
/// CH4.CFGR write → CH4 fires → DR write → USART start-bit chain.
/// The `bit_time / 2` is the half-bit average wait from the USART
/// synchronizer aligning to the next BRR clock edge.
///
/// Recomputed in `set_baud`; the `fire_at_tick` stored in FIRED_TICK_LO
/// stays the COMMANDED value (pre-comp) so host-side missed-schedule
/// detection still works.
static FIRE_COMP_TICKS: AtomicU32 = AtomicU32::new(0);

const FIRE_COMP_FLAT_TICKS: u32 = 11;

/// `brr` is USART1 BRR (HCLK ticks per bit at this baud). SysTick = HCLK/8,
/// so bit_time_systicks = brr / 8. Half = brr / 16.
const fn fire_comp_ticks(brr: u32) -> u32 {
    FIRE_COMP_FLAT_TICKS + brr / 16
}

#[inline]
fn store_fired_tick(t: u32) {
    unsafe { ptr::write_volatile(FIRED_TICK_LO.get(), t) }
}
#[inline]
fn store_pending_after_idle(v: u32) {
    unsafe { ptr::write_volatile(PENDING_AFTER_IDLE_TICKS.get(), v) }
}
#[inline]
fn load_pending_after_idle() -> u32 {
    unsafe { ptr::read_volatile(PENDING_AFTER_IDLE_TICKS.get()) }
}

pub fn init() {
    unsafe {
        // ── Clocks. USART1 on APB2 (default PA9/PA10 needs no AFIO); TIM4 on
        // APB1. TIM1 is forbidden here — its CH2 default-maps to PA9 and
        // fights USART1_TX on the AF mux even with CC2E=0+MOE=0 (see file
        // header). DMA1 on AHB.
        RCC.apb2pcenr().modify(|w| {
            w.set_iopaen(true);
            w.set_usart1en(true);
        });
        RCC.apb1pcenr().modify(|w| w.set_tim4en(true));
        RCC.ahbpcenr().modify(|w| w.set_dma1en(true));

        // ── PA9 = USART1_TX (default mapping), AF push-pull, 50 MHz. HDSEL
        // works with either OD or PP; we use PP because OD relies on a strong
        // external bus pull-up to recover idle-HIGH between LOW bits, and the
        // DUT's pull-up isn't strong enough to keep up at 1M+ baud — bytes
        // get clipped after the first LOW transition. Collisions aren't a
        // concern on this bench (single master + chip-side TX_EN gating), so
        // active push-pull is safe and matches the pre-TIM4 USART2 config
        // that's known to work on this hardware.
        //
        // ODR set HIGH first: belt-and-suspenders against any init phase
        // where the GPIO momentarily falls back to ODR-controlled (e.g.,
        // before USART CTLR1 enables UE, the AF block may not be actively
        // driving and ODR=0 would pull the line LOW).
        GPIOA.outdr().modify(|w| w.set_odr(9, true));
        // CFGHR controls PA8..PA15; PA9 sits in bits [7:4]. Mode=11, CNF=10.
        let cnf_mode_pa9 = 0b1011u32;
        GPIOA.cfghr().modify(|w| {
            let mut v = w.0;
            v &= !(0xF << 4);
            v |= cnf_mode_pa9 << 4;
            w.0 = v;
        });

        // ── USART1: default 1 Mbaud, 8N1, HDSEL, DMAT. Init order matters —
        // set TE/RE first (with UE=0) so the AF block knows its direction
        // before being enabled, then enable HDSEL, then UE in its own write.
        // The TE+UE-in-one-write idiom can transiently glitch the TX line
        // LOW on STM32-family USARTs (V20x included), which on AF_OD + bus
        // pull-up leaves the bus stuck LOW with the USART parked in a
        // confused state. Mirrors the ch32-hal recipe at
        // ch32-hal/src/usart/mod.rs::configure.
        USART1.ctlr2().modify(|w| w.set_stop(0b00));
        USART1.ctlr1().modify(|w| {
            w.set_m(false);
            w.set_pce(false);
            w.set_te(true);
            w.set_re(true);
        });
        USART1.ctlr3().modify(|w| {
            w.set_hdsel(true);
            w.set_dmat(true);
        });
        USART1.brr().write(|w| w.0 = APB2_HZ / DEFAULT_BAUD);
        USART1.ctlr1().modify(|w| w.set_ue(true));
        FIRE_COMP_TICKS.store(fire_comp_ticks(APB2_HZ / DEFAULT_BAUD), Ordering::Relaxed);

        // ── DMA1_CH4 = USART1_TX. metapac channels are zero-indexed: CH4 → ch(3).
        // EN stays clear at init — TIM4 fire path flips it via the DMA1_CH7
        // chain; load_payload also reloads MAR/NDTR with EN=0 between fires.
        let ch4 = DMA1.ch(3);
        ch4.par().write_value(USART1.datar().as_ptr() as u32);
        ch4.cr().write(|w| {
            w.set_dir(Dir::FROMMEMORY);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(false);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_tcie(false);
        });

        // Precompute the EN=1 CFGR word DMA1_CH7 stamps on fire. Same bits as
        // the disarmed config above, plus the enable bit. Computed via the
        // metapac builder so a future field-layout shuffle propagates here.
        let mut armed = ch32_hal::pac::dma::regs::Cr(0);
        armed.set_dir(Dir::FROMMEMORY);
        armed.set_minc(true);
        armed.set_pinc(false);
        armed.set_circ(false);
        armed.set_msize(Size::BITS8);
        armed.set_psize(Size::BITS8);
        armed.set_tcie(false);
        armed.set_en(true);
        ptr::write_volatile(ARMED_CH4_CFGR_WORD.get(), armed.0);

        // ── DMA1_CH7 = TIM4_UP. Circular, single-word transfer per trigger:
        // copy ARMED_CH4_CFGR_WORD → DMA1_CH4.CFGR. Peripheral side is the
        // CFGR MMIO register; memory side is the precomputed word in SRAM.
        // EN=1 permanently — the trigger source (TIM4_UP) gates fires, not
        // EN. NDTR=1 auto-reloads each cycle thanks to CIRC=1, so we never
        // re-arm CH7 from software.
        let ch7 = DMA1.ch(6);
        ch7.par().write_value(ch4.cr().as_ptr() as u32);
        ch7.mar().write_value(ARMED_CH4_CFGR_WORD.get() as u32);
        ch7.ndtr().write(|w| w.set_ndt(1));
        ch7.cr().write(|w| {
            w.set_dir(Dir::FROMMEMORY);
            w.set_minc(false);
            w.set_pinc(false);
            w.set_circ(true);
            w.set_msize(Size::BITS32);
            w.set_psize(Size::BITS32);
            w.set_tcie(false);
            w.set_en(true);
        });

        // ── TIM4 OPM. PSC=7 → 18 MHz tick (= 1:1 with SysTick). URS=1 so a
        // UG software event (used to reset CNT between fires) doesn't
        // generate an UEV that would spuriously kick DMA1_CH7. UDE=1 routes
        // CNT-overflow UEV to DMA. OPM=1 auto-clears CEN after the first
        // UEV, so each fire is exactly one shot.
        TIM4.psc().write_value(TIM4_PSC);
        TIM4.atrlr().write_value(0xFFFF);
        TIM4.ctlr1().write(|w| {
            w.set_cen(false);
            w.set_opm(true);
            w.set_urs(Urs::COUNTERONLY);
            w.set_arpe(false);
        });
        TIM4.dmaintenr().write(|w| {
            w.set_ude(true);
        });

        // ── SysTick: HCLK/8 upcount, free-running, NO IRQ. ch32-hal's time
        // driver isn't using SysTick (we picked time-driver-tim2), so this
        // peripheral is fully ours for tick reads (host queries, ARM/MASTER
        // stamping). The fire path lives on TIM4 + DMA chain, no CMP IRQ.
        SYSTICK.ctlr().write(|w| {
            w.set_init(true);
            w.set_ste(true);
        });
        SYSTICK.cmp().write_value(u64::MAX);
        SYSTICK.sr().write(|w| w.set_cntif(false));
        SYSTICK.ctlr().modify(|w| {
            w.set_mode(ch32_hal::pac::systick::vals::Mode::UPCOUNT);
            w.set_stre(false);
            w.set_stclk(ch32_hal::pac::systick::vals::Stclk::HCLK_DIV8);
            w.set_stie(false);
        });

        qingke::pfic::enable_interrupt(Interrupt::USART1 as u8);
    }
}

/// Load `payload` into the TX buffer and fire when `SysTick.CNT` reaches
/// `fire_at_tick` (HCLK/8 = 18 MHz). Bytes are blasted verbatim — callers own
/// the wire format.
///
/// Held under a critical section so a pending USART3 IDLE can't fire the DMA
/// between disabling EN and writing NDTR/MAR. The TIM4 schedule itself is
/// race-free because OPM auto-clears CEN after each fire.
pub fn arm(payload: &[u8], fire_at_tick: u64) -> Result<(), ArmError> {
    critical_section::with(|_| {
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        disarm_tim4();
        load_payload(payload)?;
        // Arm FIRE_T_FIRST capture so the listener's RXNE handler stamps the
        // first self-echo byte. Reset the stored value so a missed RXNE
        // can't deliver a stale stamp from the previous fire.
        crate::listen::reset_fire_t_first();
        EXPECT_FIRE_FIRST_BYTE.store(true, Ordering::Release);
        schedule_or_fire_now(fire_at_tick);
        Ok(())
    })
}

/// Load `payload` into ARM_TX_BUF and fire `after_idle_ticks` after wire-end.
/// Listener backdates by one char-time so the caller passes spec-relative
/// "ticks after end-of-frame" — no char-time compensation needed.
///
/// Uses a separate buffer from TX_BUF so a MASTER+ARM chain (master TX
/// echo IDLE → ARM fires INJ slot) doesn't need to dodge MASTER's
/// load_payload. The on_listen_idle consumer swaps the DMA channel to
/// ARM_TX_BUF before kicking the fire.
pub fn arm_after_idle(payload: &[u8], after_idle_ticks: u32) -> Result<(), ArmError> {
    critical_section::with(|_| -> Result<(), ArmError> {
        // Reset scope marker LOW for a clean re-arm window before next IDLE.
        crate::debug::clear();
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        if payload.len() > TX_BUF_LEN {
            return Err(ArmError::TooLong);
        }
        // SAFETY: ARMED_AFTER_IDLE=false above prevents on_listen_idle from
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

/// Stop a pending TIM4 OPM cleanly: clear CEN, reset CNT (URS=1 means this
/// won't generate a UEV that would kick DMA1_CH7), clear the UIF flag for
/// good measure. Idempotent — safe to call when TIM4 isn't armed.
fn disarm_tim4() {
    TIM4.ctlr1().modify(|w| w.set_cen(false));
    // UG = bit 0 of swevgr; resets CNT + PSC counter. URS=1 blocks the UEV
    // that would otherwise propagate to UDE.
    TIM4.swevgr().write(|w| w.set_ug(true));
    TIM4.intfr().modify(|w| w.set_uif(false));
}

/// Called from listen's USART3 IDLE IRQ. When armed-after-idle, swaps DMA1_CH4
/// to point at ARM_TX_BUF and fires `after_idle_ticks` after `idle_tick`.
pub fn on_listen_idle(idle_tick: u32) {
    if !ARMED_AFTER_IDLE.swap(false, Ordering::AcqRel) {
        return;
    }
    let after = load_pending_after_idle();

    // OPM normally self-clears CEN on the prior UEV, but a pipelined
    // FIRE(future)+ARM that races the IDLE here can leave TIM4 with CEN=1
    // and ARR pointing at the stale TX_BUF schedule — DMA1_CH7 would then
    // stamp EN=1 into our half-reconfigured CH4. Disarm so schedule_or_fire_now
    // below starts from a known-idle TIM4.
    disarm_tim4();

    // Reload DMA from ARM_TX_BUF so this fire emits the arm payload, not
    // whatever was last in TX_BUF (most often the MASTER request bytes).
    // SAFETY: USART3 IDLE ISR is the sole reader of ARM_TX_BUF/ARM_TX_LEN;
    // arm_after_idle is the sole writer, and ARMED_AFTER_IDLE gating means
    // it can't be mid-write here (the swap-to-false above ate the flag).
    unsafe {
        let len = ptr::read_volatile(ARM_TX_LEN.get());
        let ch = DMA1.ch(3);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(len));
        ch.mar().write_value((*ARM_TX_BUF.get()).as_ptr() as u32);
    }

    // `idle_tick` is the low 32 captured a few cycles ago, so it equals
    // `now` minus a small wrapping low-half delta — recover the u64 timeline
    // by subtracting that delta from the current u64 CNT.
    let now = SYSTICK.cnt().read();
    let elapsed = (now as u32).wrapping_sub(idle_tick);
    let fire_at = now.wrapping_sub(elapsed as u64).wrapping_add(after as u64);

    schedule_or_fire_now(fire_at);
}

/// Fire `payload` immediately as the master and capture the wire-end SysTick
/// stamp in MASTER_T_REQUEST_END via the TC IRQ. Preempts any pending
/// absolute-tick `arm` (FIRE cmd) but does NOT cancel `arm_after_idle` (ARM
/// cmd) — ARM lives in a separate buffer and an explicit MASTER+ARM chain
/// (master TX echo IDLE → ARM emits faked-foreign slot) is a supported
/// pattern. Listener suppresses the one IDLE generated by our own TX echo
/// so the stamp ring stays slave-side.
pub fn master_send(payload: &[u8]) -> Result<(), ArmError> {
    critical_section::with(|_| -> Result<(), ArmError> {
        disarm_tim4();
        load_payload(payload)?;

        SUPPRESS_NEXT_IDLE_STAMP.store(true, Ordering::Release);
        EXPECT_FIRST_BYTE.store(false, Ordering::Release);
        EXPECT_REPLY_END_IDLE.store(false, Ordering::Release);
        crate::listen::reset_t_first();

        // Clear TC before unmasking TCIE so a pre-existing TC=1 (set by the
        // previous master_send or reset state) can't fire the IRQ before our
        // burst has even started shifting.
        USART1.statr().modify(|w| w.set_tc(false));
        USART1.ctlr1().modify(|w| w.set_tcie(true));

        store_fired_tick(SYSTICK.cntl().read());
        crate::debug::clear();
        fire_now();
        Ok(())
    })
}

pub fn last_master_request_end() -> u32 {
    unsafe { ptr::read_volatile(MASTER_T_REQUEST_END.get()) }
}

/// Reconfigure USART1's bit rate. Bounces UE around the BRR write so the
/// peripheral picks up the new divisor cleanly. Caller must quiesce the bus
/// first — changing baud mid-frame will garbage anything in flight.
///
/// Held under a critical section: USART1 ISR also `.modify()`s `ctlr1` to
/// clear TCIE; without locking, this RMW races the ISR and either loses the
/// ISR's TCIE clear (leaving TCIE stuck on) or clobbers UE state. Same
/// pattern that caused the prior TX_EN-stuck-HIGH wedge.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB2_HZ, bps).ok_or(BaudError::OutOfRange)?;
    critical_section::with(|_| {
        USART1.ctlr1().modify(|w| w.set_ue(false));
        USART1.brr().write(|w| w.0 = brr);
        USART1.ctlr1().modify(|w| w.set_ue(true));
        FIRE_COMP_TICKS.store(fire_comp_ticks(brr), Ordering::Relaxed);
        // chip_tune crosses baud tiers between shots; clearing all per-trip
        // expectation flags + any pending TIM4 here prevents flags armed at
        // the previous baud from being consumed by the new baud's stamping
        // and the bench seeing cross-tier "extra_idle" or stale Round entries.
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        SUPPRESS_NEXT_IDLE_STAMP.store(false, Ordering::Release);
        EXPECT_FIRST_BYTE.store(false, Ordering::Release);
        EXPECT_REPLY_END_IDLE.store(false, Ordering::Release);
        EXPECT_FIRE_FIRST_BYTE.store(false, Ordering::Release);
        disarm_tim4();
    });
    Ok(())
}

pub fn read_systick_cnt() -> u64 {
    SYSTICK.cnt().read()
}

pub fn last_fired_tick() -> u32 {
    unsafe { ptr::read_volatile(FIRED_TICK_LO.get()) }
}

pub const fn ticks_per_us() -> u32 {
    SYSTICK_HZ / 1_000_000
}

fn load_payload(payload: &[u8]) -> Result<(), ArmError> {
    if payload.len() > TX_BUF_LEN {
        return Err(ArmError::TooLong);
    }
    unsafe {
        let buf = &mut *TX_BUF.get();
        buf[..payload.len()].copy_from_slice(payload);

        let ch = DMA1.ch(3);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(payload.len() as u16));
        ch.mar().write_value(buf.as_ptr() as u32);
    }
    Ok(())
}

/// Fire the pre-loaded DMA1_CH4 payload immediately by stamping the armed
/// CFGR word over the channel's config register. Same effect as a TIM4 UEV
/// → DMA1_CH7 chain firing, just bypassing the timer. Used by master_send
/// and by the FIRE-NOW threshold branch in schedule_or_fire_now.
#[inline]
fn fire_now() {
    let armed = unsafe { ptr::read_volatile(ARMED_CH4_CFGR_WORD.get()) };
    DMA1.ch(3)
        .cr()
        .write_value(ch32_hal::pac::dma::regs::Cr(armed));
}

/// Must be paired with a prior `load_payload`. Programs TIM4 OPM to fire the
/// DMA chain when `SYSTICK.cnt()` reaches `fire_at_tick`, or fires
/// immediately if the deadline is already past / within the arm overhead /
/// beyond TIM4's 16-bit ARR range.
fn schedule_or_fire_now(fire_at_tick: u64) {
    // FIRE_COMP_TICKS subtracts the measured pipeline (TIM4 OPM CC → wire
    // start) so the WIRE start bit lands at `fire_at_tick` on the wall clock.
    // Stored stamp is the commanded value (pre-comp) so the host-side
    // missed-schedule detection keeps working: scheduled path stamps
    // `fire_at_tick`, immediate-fire path stamps `now`.
    let comp = FIRE_COMP_TICKS.load(Ordering::Relaxed) as u64;
    let scheduled_at = fire_at_tick.wrapping_sub(comp);
    let now = SYSTICK.cnt().read();
    let delta = scheduled_at.saturating_sub(now);
    if delta < FIRE_NOW_THRESHOLD_TICKS || delta > TIM4_MAX_DELTA_TICKS {
        store_fired_tick(now as u32);
        crate::debug::clear();
        fire_now();
        return;
    }
    // ARR latches the next time CNT == ARR (URS=1 + ARPE=0 → take effect
    // immediately on next compare). Start the timer with CEN=1; OPM clears
    // it after the first UEV.
    TIM4.atrlr().write_value(delta as u16);
    store_fired_tick(fire_at_tick as u32);
    TIM4.ctlr1().modify(|w| w.set_cen(true));
}

#[derive(Copy, Clone, Debug)]
pub enum ArmError {
    TooLong,
}

#[derive(Copy, Clone, Debug)]
pub enum BaudError {
    OutOfRange,
}

#[interrupt]
fn USART1() {
    // TC asserts once the shift register drains — this is the master's
    // last-bit-out moment, i.e. T_request_end in the cal model. Stamp
    // first, then mask + clear, so a late TC re-assertion can't reorder.
    let tick = SYSTICK.cntl().read();
    unsafe {
        USART1.ctlr1().modify(|w| w.set_tcie(false));
        USART1.statr().modify(|w| w.set_tc(false));
        ptr::write_volatile(MASTER_T_REQUEST_END.get(), tick);
    }
    crate::led::signal();
}
