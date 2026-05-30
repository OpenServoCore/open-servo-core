//! DXL Fast slot injector hot path: pre-encoded payload → DMA1_CH7 → USART2
//! (HDSEL, 3 Mbaud) kicked off from a SysTick CMP IRQ for ≤1 µs slot-start
//! jitter at the wire.
//!
//! HAL owns clocks + USB; everything else is direct metapac so the ISR is a
//! handful of register writes with no abstraction layers between us and the
//! bus.

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_hal::pac::Interrupt;
use ch32_hal::pac::dma::vals::{Dir, Size};
use ch32_hal::pac::{DMA1, GPIOA, RCC, SYSTICK, USART2};
use dxl_pirate::parse::brr_for;
use portable_atomic::{AtomicBool, Ordering};
use qingke_rt::interrupt;

pub const TX_BUF_LEN: usize = 1024;

/// SysTick CMP minus current CNT below this many ticks fires immediately
/// instead of scheduling. 96 ticks ≈ 5.3 µs at 18 MHz — headroom over IRQ
/// entry + the AHB-bridge register-write latency, so a too-close deadline
/// never silently misses (next CMP-match would be 239 s later on u32 wrap).
const FIRE_NOW_THRESHOLD_TICKS: u64 = 96;

/// FIRE / MASTER share this buffer; both fire from `DMA1 CH7` immediately
/// (FIRE on a SysTick CMP match, MASTER on its own dispatch). MASTER preempts
/// any pending FIRE.
static TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);

/// ARM uses a separate buffer so a MASTER+ARM chain (= "fire master request,
/// then emit a faked foreign-slave slot one RDT after the master IDLE")
/// keeps both payloads loaded simultaneously. The `on_listen_idle` swap path
/// reloads the DMA channel to point here before kicking the fire.
static ARM_TX_BUF: SyncUnsafeCell<[u8; TX_BUF_LEN]> = SyncUnsafeCell::new([0; TX_BUF_LEN]);
/// ARM payload length (bytes loaded into `ARM_TX_BUF`). ISR-only writer is
/// `arm_after_idle`; ISR-only reader is `on_listen_idle`'s swap path.
static ARM_TX_LEN: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

// USART2 sits on APB1; with SYSCLK_FREQ_144MHZ_HSI APB1 runs at 144 MHz.
pub const APB1_HZ: u32 = 144_000_000;
pub const DEFAULT_BAUD: u32 = 1_000_000;

// SysTick on V4 ticks at HCLK/8 = 18 MHz with the 144 MHz preset, so 1 µs ≈ 18.
const SYSTICK_HZ: u32 = 144_000_000 / 8;

/// `arm_after_idle` arms the payload + offset and sets this; the next IDLE the
/// listener observes consumes it via swap. The swap is the only RMW —
/// `FIRED_TICK_LO` and `PENDING_AFTER_IDLE_TICKS` below are single-word,
/// single-writer-at-a-time cells accessed via volatile read/write (V203 is
/// single-core, aligned u32 stores are tear-free).
static ARMED_AFTER_IDLE: AtomicBool = AtomicBool::new(false);
static FIRED_TICK_LO: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);
static PENDING_AFTER_IDLE_TICKS: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// USART2 TXC stamp: SysTick.CNTL captured the moment the master's last byte
/// finishes shifting out (TC asserts post-shift, unlike TXE which fires when
/// the holding register empties). This is `T_request_end` in the cal model.
/// Sole writer = USART2 ISR.
static MASTER_T_REQUEST_END: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// `master_send` sets; listen's USART3 IDLE ISR consumes via swap and skips
/// publishing that stamp. The master's own TX echoes into the USART3 listener
/// and produces one IDLE we don't want polluting the slave-side stamp ring.
pub(crate) static SUPPRESS_NEXT_IDLE_STAMP: AtomicBool = AtomicBool::new(false);

/// Gate for the RXNE handler's `T_FIRST` stamp. Set ONLY by the USART3
/// IDLE-suppress branch (= master TX echo IDLE just fired). The RXNE handler
/// swap-clears it on the next received byte. Cleared (not set) by `master_send`
/// so a stale RXNEIE left armed by a no-reply prior trip can fire on our
/// master TX-echo byte without misattributing it as the slave-reply T0.
pub(crate) static EXPECT_FIRST_BYTE: AtomicBool = AtomicBool::new(false);

/// The USART3 IDLE handler sets this when it consumes SUPPRESS (= master TX
/// echo IDLE just fired). The next non-suppressed IDLE is the slave-reply
/// wire-end, and the handler publishes a `Round` stamp instead of `Plain`,
/// then clears this flag. `master_send` also resets it so a no-reply trip
/// doesn't taint the next round.
pub(crate) static EXPECT_REPLY_END_IDLE: AtomicBool = AtomicBool::new(false);

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
        // ── Clocks. USART2 is on APB1; default pin map (PA2/PA3) needs no AFIO.
        RCC.apb2pcenr().modify(|w| w.set_iopaen(true));
        RCC.apb1pcenr().modify(|w| w.set_usart2en(true));
        RCC.ahbpcenr().modify(|w| w.set_dma1en(true));

        // ── PA2 = USART2_TX (default mapping), AF push-pull, 50 MHz.
        // CFGLR controls PA0..PA7; PA2 sits in bits [11:8]. Mode=11, CNF=10.
        let cnf_mode_pa2 = 0b1011u32;
        GPIOA.cfglr().modify(|w| {
            let mut v = w.0;
            v &= !(0xF << 8);
            v |= cnf_mode_pa2 << 8;
            w.0 = v;
        });

        // ── USART2: default 1 Mbaud, 8N1, HDSEL, DMAT. Both TE and RE are set
        // — in HDSEL the peripheral handles direction switching on the shared
        // line, tri-stating TX between transmissions so we don't push HIGH
        // against the DUT during its slot. RX bytes go nowhere (no DMAR, no
        // RXNEIE); the OE flag just runs hot and is harmless. Host can retune
        // at runtime via `set_baud`.
        USART2.brr().write(|w| w.0 = APB1_HZ / DEFAULT_BAUD);
        USART2.ctlr2().modify(|w| w.set_stop(0b00));
        USART2.ctlr3().modify(|w| {
            w.set_hdsel(true);
            w.set_dmat(true);
        });
        USART2.ctlr1().modify(|w| {
            w.set_m(false);
            w.set_pce(false);
            w.set_te(true);
            w.set_re(true);
            w.set_ue(true);
        });

        // DMA1 CH7 = USART2_TX. metapac channels are zero-indexed: CH7 → ch(6).
        let ch = DMA1.ch(6);
        ch.par().write_value(USART2.datar().as_ptr() as u32);
        ch.cr().write(|w| {
            w.set_dir(Dir::FROMMEMORY);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_circ(false);
            w.set_msize(Size::BITS8);
            w.set_psize(Size::BITS8);
            w.set_tcie(false);
            // EN stays clear — ISR / schedule path flips it.
        });

        // ── SysTick: HCLK/8 upcount, no auto-reload, IRQ on compare match.
        // ch32-hal's time driver isn't using SysTick (we picked time-driver-tim2),
        // so this peripheral is fully ours.
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
            w.set_stie(true);
        });
        qingke::pfic::enable_interrupt(qingke_rt::CoreInterrupt::SysTick as u8);
        qingke::pfic::enable_interrupt(Interrupt::USART2 as u8);
    }
}

/// Load `payload` into the TX buffer and fire when `SysTick.CNT` reaches
/// `fire_at_tick` (HCLK/8 = 18 MHz). Bytes are blasted verbatim — callers own
/// the wire format.
///
/// Held under a critical section so a pending SysTick CMP-match or USART3
/// IDLE can't fire the DMA between disabling EN and writing NDTR/MAR.
pub fn arm(payload: &[u8], fire_at_tick: u64) -> Result<(), ArmError> {
    critical_section::with(|_| {
        ARMED_AFTER_IDLE.store(false, Ordering::Release);
        disarm_systick();
        load_payload(payload)?;
        schedule_or_fire_now(fire_at_tick);
        Ok(())
    })
}

/// Load `payload` into ARM_TX_BUF and fire `after_idle_ticks` after wire-end.
/// Listener backdates by one char-time so the caller passes spec-relative
/// "ticks after end-of-frame" — no char-time compensation needed.
///
/// Uses a separate buffer from `TX_BUF` so a MASTER+ARM chain (master TX
/// echo IDLE → ARM fires INJ slot) doesn't need to dodge MASTER's
/// `load_payload`. The `on_listen_idle` consumer swaps the DMA channel to
/// `ARM_TX_BUF` before kicking the fire.
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

/// PFIC latches SysTick pending independently of CNTIF, so clearing only
/// CNTIF would let a stale ISR fire on CS exit and kick DMA at "now" instead
/// of the new target. Order: park CMP (block new edges) → clear CNTIF (source)
/// → clear PFIC pending (latch).
fn disarm_systick() {
    SYSTICK.cmp().write_value(u64::MAX);
    SYSTICK.sr().write(|w| w.set_cntif(false));
    unsafe { qingke::pfic::unpend_interrupt(qingke_rt::CoreInterrupt::SysTick as u8) };
}

/// Called from `listen`'s USART3 IDLE IRQ. When armed-after-idle, swaps DMA1
/// CH7 to point at `ARM_TX_BUF` and fires `after_idle_ticks` after `idle_tick`.
pub fn on_listen_idle(idle_tick: u32) {
    if !ARMED_AFTER_IDLE.swap(false, Ordering::AcqRel) {
        return;
    }
    let after = load_pending_after_idle();

    // Reload DMA from ARM_TX_BUF so this fire emits the arm payload, not
    // whatever was last in TX_BUF (most often the MASTER request bytes).
    // SAFETY: USART3 IDLE ISR is the sole reader of ARM_TX_BUF/ARM_TX_LEN;
    // arm_after_idle is the sole writer, and ARMED_AFTER_IDLE gating means
    // it can't be mid-write here (the swap-to-false above ate the flag).
    unsafe {
        let len = ptr::read_volatile(ARM_TX_LEN.get());
        let ch = DMA1.ch(6);
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
/// stamp in `MASTER_T_REQUEST_END` via the TC IRQ. Preempts any pending
/// absolute-tick `arm` (FIRE cmd) but does NOT cancel `arm_after_idle` (ARM
/// cmd) — ARM lives in a separate buffer and an explicit MASTER+ARM chain
/// (master TX echo IDLE → ARM emits faked-foreign slot) is a supported
/// pattern. Listener suppresses the one IDLE generated by our own TX echo
/// so the stamp ring stays slave-side.
pub fn master_send(payload: &[u8]) -> Result<(), ArmError> {
    critical_section::with(|_| -> Result<(), ArmError> {
        disarm_systick();
        load_payload(payload)?;

        SUPPRESS_NEXT_IDLE_STAMP.store(true, Ordering::Release);
        EXPECT_FIRST_BYTE.store(false, Ordering::Release);
        EXPECT_REPLY_END_IDLE.store(false, Ordering::Release);
        crate::listen::reset_t_first();

        // Clear TC before unmasking TCIE so a pre-existing TC=1 (set by the
        // previous master_send or reset state) can't fire the IRQ before our
        // burst has even started shifting.
        USART2.statr().modify(|w| w.set_tc(false));
        USART2.ctlr1().modify(|w| w.set_tcie(true));

        store_fired_tick(SYSTICK.cntl().read());
        crate::debug::clear();
        DMA1.ch(6).cr().modify(|w| w.set_en(true));
        Ok(())
    })
}

pub fn last_master_request_end() -> u32 {
    unsafe { ptr::read_volatile(MASTER_T_REQUEST_END.get()) }
}

/// Reconfigure USART2's bit rate. Bounces UE around the BRR write so the
/// peripheral picks up the new divisor cleanly. Caller must quiesce the bus
/// first — changing baud mid-frame will garbage anything in flight.
pub fn set_baud(bps: u32) -> Result<(), BaudError> {
    let brr = brr_for(APB1_HZ, bps).ok_or(BaudError::OutOfRange)?;
    USART2.ctlr1().modify(|w| w.set_ue(false));
    USART2.brr().write(|w| w.0 = brr);
    USART2.ctlr1().modify(|w| w.set_ue(true));
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

        let ch = DMA1.ch(6);
        ch.cr().modify(|w| w.set_en(false));
        ch.ndtr().write(|w| w.set_ndt(payload.len() as u16));
        ch.mar().write_value(buf.as_ptr() as u32);
    }
    Ok(())
}

/// Must be paired with a prior `load_payload`.
fn schedule_or_fire_now(fire_at_tick: u64) {
    let now = SYSTICK.cnt().read();
    if fire_at_tick.saturating_sub(now) < FIRE_NOW_THRESHOLD_TICKS {
        store_fired_tick(now as u32);
        crate::debug::clear();
        DMA1.ch(6).cr().modify(|w| w.set_en(true));
        SYSTICK.cmp().write_value(u64::MAX);
        return;
    }
    SYSTICK.cmp().write_value(fire_at_tick);
    SYSTICK.sr().write(|w| w.set_cntif(false));
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
fn USART2() {
    // TC asserts once the shift register drains — this is the master's
    // last-bit-out moment, i.e. T_request_end in the cal model. Stamp
    // first, then mask + clear, so a late TC re-assertion can't reorder.
    let tick = SYSTICK.cntl().read();
    unsafe {
        USART2.ctlr1().modify(|w| w.set_tcie(false));
        USART2.statr().modify(|w| w.set_tc(false));
        ptr::write_volatile(MASTER_T_REQUEST_END.get(), tick);
    }
    crate::led::signal();
}

#[qingke_rt::interrupt(core)]
fn SysTick() {
    // Clear CNTIF before the DMA kick so we don't re-enter on a wraparound.
    SYSTICK.sr().write(|w| w.set_cntif(false));

    // Snapshot kickoff tick for host-side latency analytics. Low half only;
    // the high half is irrelevant at slot timescales.
    store_fired_tick(SYSTICK.cntl().read());

    crate::debug::clear();
    DMA1.ch(6).cr().modify(|w| w.set_en(true));

    // Park CMP far in the future so the IRQ doesn't keep firing on the wrap.
    // The next arm() resets it.
    SYSTICK.cmp().write_value(u64::MAX);

    crate::led::signal();
}
