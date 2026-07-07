//! Wire TX path: pre-encoded payload → DMA1_CH2 → USART3 (TX=PB10 AF PP,
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

mod comp;
mod scheduler;

use core::cell::SyncUnsafeCell;
use core::ptr;

use ch32_metapac::{AFIO, DMA1, GPIOB, RCC, USART3};
use portable_atomic::{AtomicBool, Ordering};

use crate::parse::brr_for;
use crate::tick::read_tick32;

pub use comp::{set_tx_comp, tx_comp};
pub use scheduler::last_send_tick;

pub const TX_BUF_LEN: usize = 1024;

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

/// Set by `schedule_send_after_idle`; consumed by `on_idle`. Cleared each
/// IDLE swap.
static IDLE_SEND_PENDING: AtomicBool = AtomicBool::new(false);
static IDLE_SEND_DELAY_TICKS: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

/// Upper bound on how long `wait_tx_complete` will spin for `USART3.SR.TC`.
/// 200 ms at 144 MHz. Generous enough for any payload up to TX_BUF_LEN at
/// 9600 baud (≈ 1.1 s for 1024 bytes — we cap shorter than that on
/// purpose: a stuck TC means something genuinely broke and the host should
/// see a `busy` error rather than the chip pretending to work).
const TX_WAIT_TIMEOUT_TICKS: u32 = 28_800_000;

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
    scheduler::init();
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

/// PB10/PB11 (USART3_TX/RX): TX idles as AF open-drain (released), RX
/// as input pull-up.
///
/// Drive discipline: PB10 sits open-drain (released, wire held high by
/// the bus pull-ups) whenever the pirate isn't transmitting, and flips
/// to AF push-pull only while a send shifts out — both edges actively
/// driven at 3M, then the wire is handed back. A permanently push-pull
/// idle-high PB10 clamps any direct-wire (buffer-less HDSEL) device
/// trying to talk: its GPIO can't win the fight the 74LVC2G241 used to
/// win. Rise time through the pull-ups only matters at the DC arm/
/// release boundaries, never for data edges.
///
/// Window note: scheduled sends arm push-pull at schedule time (the
/// kickoff itself is TIM4-hardware-timed with no CPU in the path), so
/// the wire is driven idle-high from schedule to wire-start. The host
/// owns the bus when it schedules, so nothing else should be talking in
/// that window.
fn init_pins() {
    // ODR high before AF lock: guards against a transient ODR-LOW
    // pulling the bus while the AF block is mid-init.
    GPIOB.outdr().modify(|w| {
        w.set_odr(10, true);
        w.set_odr(11, true); // select PB11 input pullup (see below)
    });
    //   PB10: Mode=11 (50 MHz), CNF=11 (AF OD) → 0b1111.
    //   PB11: Mode=00 (input),  CNF=10 (input w/ pull) → 0b1000;
    //         ODR(11)=1 above selects pull-up.
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 8);
        v &= !(0xF << 12);
        v |= 0b1111u32 << 8;
        v |= 0b1000u32 << 12;
        w.0 = v;
    });
}

/// PB10 CNF: AF push-pull while transmitting, AF open-drain (released)
/// otherwise.
fn pb10_drive(pp: bool) {
    set_pb10_cnf(if pp { 0b1011 } else { 0b1111 });
}

/// Arm the drive for a scheduled (DMA) send: push-pull now, and a TC
/// interrupt to hand the wire back when the last stop bit clears the
/// shifter. TC is cleared first so a stale complete flag can't release
/// the drive before this send even starts.
fn arm_drive_release() {
    pb10_drive(true);
    USART3.statr().modify(|w| w.set_tc(false));
    USART3.ctlr1().modify(|w| w.set_tcie(true));
}

/// Called from the USART3 ISR on TC while TCIE is armed: release the
/// wire and disarm.
pub fn on_tx_complete() {
    USART3.ctlr1().modify(|w| w.set_tcie(false));
    pb10_drive(false);
}

/// USART3: 8N1, full-duplex, DMAT, DMAR, IDLEIE. Plain FE break
/// handling — the LIN-mode leg (LINEN + LBDIE) is bench-toggled per
/// experiment, not the default. Init order matters — TE/RE with UE=0,
/// then DMAT/DMAR, then UE in its own write, to avoid the TX-line
/// glitch the STM32-family USARTs throw when TE+UE land in the same
/// write.
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
    comp::recompute(brr0);
}

/// Drop a pending `schedule_send_after_idle` arm without sending it.
/// Proto routes the explicit `SEND at=` command through this before
/// scheduling, so a commanded-tick send still displaces a stale idle
/// arm — but `send_now` (which routes through [`schedule_send_at`]
/// internally) does NOT, honoring its coexistence contract: arm the
/// idle send, then `send_now` the request whose trailing IDLE triggers
/// it. Clearing inside `schedule_send_at` broke exactly that sequence —
/// the host-side `inject_then_send` armed the injection and then
/// `send_now` silently killed it before the request ever shipped.
pub fn cancel_idle_send() {
    IDLE_SEND_PENDING.store(false, Ordering::Release);
}

/// Load `payload` and kick off when `tick32` reaches `at`. Held under a
/// critical section so a pending USART3 IDLE can't kick the DMA between
/// disabling EN and writing NDTR/MAR. Leaves a pending idle-send arm
/// alone — see [`cancel_idle_send`].
pub fn schedule_send_at(payload: &[u8], at: u32) -> Result<(), SendError> {
    critical_section::with(|_| {
        scheduler::cancel();
        load_payload_main(payload)?;
        arm_drive_release();
        scheduler::schedule_or_send_now(at);
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
        let at = read_tick32()
            .wrapping_add(comp::load())
            .wrapping_add(scheduler::IMMEDIATE_SEND_MARGIN_TICKS);
        schedule_send_at(payload, at)
    })
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
    scheduler::cancel();

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
    arm_drive_release();
    scheduler::schedule_or_send_now(send_at);
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
        comp::recompute(brr);
        IDLE_SEND_PENDING.store(false, Ordering::Release);
        scheduler::cancel();
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

/// Break-framing spike surface (`BREAK` / `BRKSEND` / `LOWPULSE`).
/// All three block the CDC command context; the caps keep the worst
/// case well under `wait_tx_complete`'s own 200 ms ceiling.
pub const BREAK_MAX_COUNT: u32 = 200;
pub const BREAK_MAX_GAP_US: u32 = 10_000;
pub const LOW_PULSE_MAX_US: u32 = 100_000;
/// Fits any osc-native frame (`footprint(u8::MAX)` = 258 wire bytes) with
/// slack.
pub const BRK_PAYLOAD_MAX: usize = 272;

/// Send `count` UART breaks via SBK, `gap_us` apart. Returns after the
/// last break has shifted out.
pub fn send_breaks(count: u32, gap_us: u32) -> Result<(), SendError> {
    if count == 0 || count > BREAK_MAX_COUNT || gap_us > BREAK_MAX_GAP_US {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    for i in 0..count {
        if i != 0 {
            spin_us(gap_us);
        }
        pulse_sbk();
    }
    spin_ticks(USART3.brr().read().0 * 2);
    pb10_drive(false);
    Ok(())
}

/// One 10-bit-exact break immediately followed by poll-fed `payload`
/// bytes — the osc-native host send primitive.
///
/// The break is NOT SBK: CH32 SBK stretches to ~12-14 bit-times with
/// sync slop (bench-measured on both chips), and a receiver resyncing
/// inside the stretched low swallows the first data byte (the
/// historical "01 -> 74" byte-0 loss). Instead the break is one 9-bit
/// frame of 0x00 (M=1, bit 8 = 0): start + 9 data lows = 10 low
/// bit-times, then a clean stop — exactly the shape LIN break detection
/// (LBDL=0) keys on, with a deterministic 1-frame resync point. The M
/// flip back costs a sub-frame gap at TC that never reaches the
/// receiver's IDLE threshold.
pub fn send_break_then(payload: &[u8]) -> Result<(), SendError> {
    if payload.is_empty() || payload.len() > BRK_PAYLOAD_MAX {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    let r = (|| {
        USART3.ctlr1().modify(|w| w.set_m(true));
        let r = feed_bytes(&[0x00])
            .and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
        USART3.ctlr1().modify(|w| w.set_m(false));
        r?;
        feed_bytes(payload)?;
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)
    })();
    pb10_drive(false);
    r
}

/// Cap on a `BURST` stream (length prefixes + frame bytes). Sized for a
/// hot-loop cycle or a long write bombardment with slack; well under the
/// USB command-line ceiling at 2 hex chars/byte.
pub const BURST_STREAM_MAX: usize = 640;

/// Zero-gap multi-frame bombardment: `stream` is length-prefixed frames —
/// `[len_0][frame_0][len_1][frame_1]…` — each sent as one 10-bit-exact
/// break (see [`send_break_then`]) followed back-to-back by its bytes.
/// Host-paced BRKSENDs leave USB-scale gaps between frames; this is the
/// only way to put `[break][frame][break][frame]…` on the wire with
/// sub-byte spacing, which is what the servo's frame-end-work vs
/// next-break timing needs for stress coverage.
pub fn send_burst(stream: &[u8]) -> Result<(), SendError> {
    // Validate the whole stream up front: a malformed prefix must not
    // truncate the burst mid-wire.
    let mut i = 0usize;
    while i < stream.len() {
        let n = stream[i] as usize;
        if n == 0 || i + 1 + n > stream.len() {
            return Err(SendError::TooLong);
        }
        i += 1 + n;
    }
    if stream.is_empty() {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    let r = (|| {
        let mut i = 0usize;
        while i < stream.len() {
            let n = stream[i] as usize;
            i += 1;
            USART3.ctlr1().modify(|w| w.set_m(true));
            let r = feed_bytes(&[0x00])
                .and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
            USART3.ctlr1().modify(|w| w.set_m(false));
            r?;
            feed_bytes(&stream[i..i + n])?;
            wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
            i += n;
        }
        Ok(())
    })();
    pb10_drive(false);
    r
}

/// Mid-stream framing-error injection: `pre` bytes as normal 8N1, then
/// `bad` bytes as 9-bit frames with bit 8 = 0 — an 8N1 receiver samples
/// that 9th bit at its stop position and flags FE with the byte's real
/// data levels on the wire (unlike a break's solid low) — then `post`
/// bytes as 8N1 again. The M-bit flips happen at TC, costing a sub-frame
/// gap that never reaches the receiver's IDLE threshold, so the whole
/// sequence reads as one back-to-back stream.
pub fn send_fe_inject(pre: &[u8], bad: &[u8], post: &[u8]) -> Result<(), SendError> {
    if pre.len() > FE_INJECT_MAX || bad.len() > FE_INJECT_MAX || post.len() > FE_INJECT_MAX {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    let r = (|| {
        feed_bytes(pre)?;
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
        USART3.ctlr1().modify(|w| w.set_m(true));
        let r =
            feed_bytes(bad).and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
        USART3.ctlr1().modify(|w| w.set_m(false));
        r?;
        feed_bytes(post)?;
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)
    })();
    pb10_drive(false);
    r
}

pub const FE_INJECT_MAX: usize = 32;

/// TXE-poll byte feed; bounded per byte. `dr` writes are 9 bits wide —
/// a `u8` payload always carries bit 8 = 0, which is exactly what the
/// 9-bit FE-injection frames need.
///
/// The per-byte bound is wall-clock, but this poll runs in thread mode
/// and the walker IRQs (`PRIO_WALKER`) preempt it. On a long burst the
/// TX echo floods the IC ring past its half-mark, so a DMA1_CH6-HT
/// `walk()` drains a large chunk in one shot — longer than the byte-time
/// bound. That preemption must not be read as a stuck shifter: TXE goes
/// empty in hardware while we're preempted, so re-check it once the bound
/// expires and only report `Busy` when the register is genuinely still
/// full. (Without this, a long BURST intermittently truncated mid-frame
/// with a spurious `ERR busy`.)
fn feed_bytes(payload: &[u8]) -> Result<(), SendError> {
    let bit_ticks = USART3.brr().read().0;
    for &b in payload {
        let t0 = read_tick32();
        while !USART3.statr().read().txe() {
            if read_tick32().wrapping_sub(t0) > bit_ticks * 64 && !USART3.statr().read().txe() {
                return Err(SendError::Busy);
            }
        }
        USART3.datar().write(|w| w.set_dr(b as u16));
    }
    Ok(())
}

/// Drive PB10 low as a plain GPIO for `us`, then restore AF. The
/// osc-native "rescue break" shape — a low far longer than any frame,
/// detectable by a servo EXTI at any configured baud.
pub fn low_pulse_us(us: u32) -> Result<(), SendError> {
    if us == 0 || us > LOW_PULSE_MAX_US {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    // ODR low before the CNF switch so the line drops exactly at the
    // switch; restore in the reverse order so it never glitches low.
    GPIOB.outdr().modify(|w| w.set_odr(10, false));
    set_pb10_cnf(0b0011); // GP push-pull 50 MHz
    spin_us(us);
    GPIOB.outdr().modify(|w| w.set_odr(10, true));
    spin_us(1);
    set_pb10_cnf(0b1111); // back to idle drive (AF open-drain)
    Ok(())
}

/// Set SBK and wait for the hardware clear (during the break's stop
/// bit), then one extra bit-time so back-to-back callers keep a clean
/// idle-high delimiter between breaks. Bounded at 32 bit-times.
fn pulse_sbk() {
    // BRR holds APB1 ticks per bit and tick32 runs at APB1 rate, so it
    // doubles as the bit-time in tick32 units.
    let bit_ticks = USART3.brr().read().0;
    USART3.ctlr1().modify(|w| w.set_sbk(true));
    let t0 = read_tick32();
    while USART3.ctlr1().read().sbk() {
        if read_tick32().wrapping_sub(t0) > bit_ticks * 32 {
            break;
        }
    }
    spin_ticks(bit_ticks);
}

fn set_pb10_cnf(bits: u32) {
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 8);
        v |= bits << 8;
        w.0 = v;
    });
}

fn spin_ticks(ticks: u32) {
    let t0 = read_tick32();
    while read_tick32().wrapping_sub(t0) < ticks {}
}

fn spin_us(us: u32) {
    spin_ticks(us.saturating_mul(crate::tick::wire_ticks_per_us()));
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
