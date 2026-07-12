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

/// Start a send window's RX-flag discipline: mask the idle event and
/// retire any latched IDLE flag. Call only with PB10 just driven
/// push-pull idle-high — under our own drive no byte can be
/// mid-reception, so this is the ONE place the flags' SR→DR clear pair
/// (a CPU DATAR read) is provably safe; anywhere else it kills a
/// mid-reception byte in the shifter (see `rx::isr`). The trailing
/// STATR read re-arms the SR half so the next latched flag can
/// drain-self-clear. The CS pairs the mask with the retire so a pended
/// idle service can't interleave, and closes the CTLR1 RMW tear window
/// against the vector's own CTLR1 writers (the TC release, the idle
/// mask). Each send path re-enables the event once its last CTLR1
/// write is done.
fn mask_and_retire_idle() {
    critical_section::with(|_| {
        USART3.ctlr1().modify(|w| w.set_idleie(false));
        let _ = USART3.statr().read();
        let _ = USART3.datar().read();
        let _ = USART3.statr().read();
    });
}

/// Retire-then-enable, for send paths whose window can latch a fresh
/// IDLE mid-send (SBK gaps exceed a char time): enabling over a latched
/// flag would level-pend the vector on the spot, so clear it first —
/// still under our own drive, same safety proof as
/// [`mask_and_retire_idle`].
fn retire_and_enable_idle() {
    critical_section::with(|_| {
        let _ = USART3.statr().read();
        let _ = USART3.datar().read();
        let _ = USART3.statr().read();
        USART3.ctlr1().modify(|w| w.set_idleie(true));
    });
}

/// Arm the drive for a scheduled (DMA) send: push-pull now, and a TC
/// interrupt to hand the wire back when the last stop bit clears the
/// shifter. TC is cleared first so a stale complete flag can't release
/// the drive before this send even starts. The idle event re-enables
/// here too — the flag was just retired under our drive, and a DMA
/// send performs no further thread-mode CTLR1 writes to tear against.
fn arm_drive_release() {
    pb10_drive(true);
    mask_and_retire_idle();
    clear_tc_only();
    USART3.ctlr1().modify(|w| {
        w.set_tcie(true);
        w.set_idleie(true);
    });
}

/// Clear TC with a plain all-ones-except-TC write — NEVER a
/// read-modify-write. STATR's flags are rc_w0 (write 1 = no-op, write 0
/// = clear) and the pirate's RX runs concurrently with its own TX echo:
/// an RXNE that sets between an RMW's read and its write-back gets
/// written 0 — the DMA request dies and the echo byte silently vanishes
/// from the ring (bench 2026-07-09: the GREAD id byte, ~1/1000 sends at
/// 0.5M, NoEcho). Write-1s touch nothing; only TC clears, race-free.
fn clear_tc_only() {
    USART3.statr().write(|w| {
        w.0 = u32::MAX;
        w.set_tc(false);
    });
}

/// Called from the USART3 ISR on TC while TCIE is armed: release the
/// wire and disarm.
pub fn on_tx_complete() {
    USART3.ctlr1().modify(|w| w.set_tcie(false));
    pb10_drive(false);
}

/// Arm the TC-interrupt release for a poll-fed send whose last byte is
/// already in flight. TC is NOT cleared here — the tail bytes may
/// already have drained under preemption, and that latched TC *is* the
/// release event. Only the FEINJ diagnostic still uses this (its `post`
/// leg may be empty and nothing races its wire handback); the
/// break-framed senders use [`kick_dma_and_arm_release`]. Un-CS'd RMW is
/// safe: until this write neither vector CTLR1 writer is armed (TCIE and
/// IDLEIE both sit masked since the send start). No DATAR retire here —
/// the tail echo may still be shifting in — so a mid-send-latched flag
/// costs one immediate self-masking service, never a storm.
fn arm_tc_release() {
    USART3.ctlr1().modify(|w| {
        w.set_tcie(true);
        w.set_idleie(true);
    });
}

/// Kick the preloaded DMA payload and arm the TC release as one atomic
/// step. The payload rides DMA1_CH2 — no CPU in the byte path, so a
/// walker or USB preemption can no longer open an inter-byte gap (a
/// receiver reads those as host byte cadence; bench 2026-07-11: the
/// thread-fed TX polluted the servos' clock-trim windows +30k ppm under
/// hot traffic, 15× the genuine offset and the wrong sign). The TC clear
/// retires the break's completion flag so arming can't release the drive
/// before the first payload byte; the real TC re-latches at the final
/// stop bit and the ISR hands the wire back. No DATAR retire here — the
/// break's echo may still be shifting in.
fn kick_dma_and_arm_release() {
    critical_section::with(|_| {
        clear_tc_only();
        USART3.ctlr1().modify(|w| {
            w.set_tcie(true);
            w.set_idleie(true);
        });
        scheduler::kick_now();
    });
}

/// Kick the preloaded DMA payload with the release left unarmed — the
/// burst interior, where the wire must stay driven across the frame
/// seam. TC clear first so the thread-side wait sees THIS payload's
/// completion, not the break's. Safe un-CS'd: TCIE and IDLEIE both sit
/// masked for the whole burst interior, so no vector CTLR1 writer races
/// (the [`arm_tc_release`] argument).
fn kick_dma() {
    clear_tc_only();
    scheduler::kick_now();
}

/// USART3: 8N1, full-duplex, DMAT, DMAR, IDLEIE. Plain FE break
/// handling — the LIN-mode leg (LINEN + LBDIE) is bench-toggled per
/// experiment, not the default. Init order matters — TE/RE with UE=0,
/// then DMAT/DMAR, then UE in its own write, to avoid the TX-line
/// glitch the STM32-family USARTs throw when TE+UE land in the same
/// write.
fn init_usart3() {
    USART3.ctlr2().modify(|w| {
        w.set_stop(0b00);
        // LIN break detect is the boundary recorder's wake: line-level
        // (10 dominant bits, LBDL=0), so it sees our own echo breaks —
        // which frame as valid 9-bit characters under the break TX's
        // M=1 and never raise FE — exactly as it sees servo SBK breaks.
        // (EIE/FE is silicon-dead on this die: zero ERR services with
        // EIE set, 2026-07-12.)
        w.set_linen(true);
        w.set_lbdl(false);
        w.set_lbdie(true);
    });
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
    mask_and_retire_idle();
    for i in 0..count {
        if i != 0 {
            spin_us(gap_us);
        }
        pulse_sbk();
    }
    spin_ticks(USART3.brr().read().0 * 2);
    // Inter-break gaps exceed a char time, so an IDLE latched mid-send
    // is the norm here — retire it before re-enabling, still under our
    // drive.
    retire_and_enable_idle();
    pb10_drive(false);
    Ok(())
}

/// One 10-bit-exact break immediately followed by poll-fed `payload`
/// bytes — the osc-native host send primitive, and the PROTOCOL LAW
/// shape (break = exactly one character time; decided 2026-07-12).
///
/// The break is NOT SBK: CH32 SBK stretches to ~12-14 bit-times, off
/// the law and outside the timing algebra (footprint models count the
/// break as 10 bits). Instead the break is one 9-bit frame of 0x00
/// (M=1, bit 8 = 0): start + 9 data lows = 10 low bit-times, then a
/// clean stop — exactly the shape LIN break detection (LBDL=0) keys
/// on. The M flip back costs a sub-frame gap at TC that never reaches
/// the receiver's IDLE threshold. Our own LBD receiver consumes the
/// break character (LIN treats it as a delimiter, not data); the drain
/// re-emits it with the captured tick (`rx::stamp`).
pub fn send_break_then(payload: &[u8]) -> Result<(), SendError> {
    if payload.is_empty() || payload.len() > BRK_PAYLOAD_MAX {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    mask_and_retire_idle();
    let r = (|| {
        // CH2 is shared with the scheduled-send path: a pending TIM4
        // kick landing mid-send would restart the channel over this
        // payload. (The idle-send arm stays — separate buffer, and its
        // IDLE can't assert inside a gapless payload.)
        scheduler::cancel();
        load_payload_main(payload)?;
        USART3.ctlr1().modify(|w| w.set_m(true));
        let r = feed_bytes(&[0x00])
            .and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
        USART3.ctlr1().modify(|w| w.set_m(false));
        r?;
        // The payload rides DMA — no CPU in the byte path (see
        // `kick_dma_and_arm_release`); the ISR releases the drive at
        // the final stop bit.
        kick_dma_and_arm_release();
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)
    })();
    // A failed leg abandons the send: stop the channel so it can't keep
    // feeding bytes into a released wire.
    if r.is_err() {
        DMA1.ch(1).cr().modify(|w| w.set_en(false));
    }
    // Backstop (error paths); benign after the ISR release — same CFGHR
    // value, and no other pin's config changes concurrently.
    pb10_drive(false);
    r
}

/// `CAL` train caps: the command blocks the CDC context for
/// `breaks × gap_us`, so both stay well under the host's read timeout.
pub const CAL_BREAKS_MAX: u32 = 64;
pub const CAL_GAP_MAX_US: u32 = 2_000;
/// Final-approach window per mark: the open spin hands over to a critical
/// section this far out, so a walker/USB preemption can't land between the
/// deadline check and the DATAR write. Bounds the masked span per mark.
const CAL_LEAD_TICKS: u32 = 1_440; // 10 µs at 144 MHz

/// Announce frame (break-framed, DMA-fed) followed by `breaks` bare breaks
/// whose start edges sit on an exact `gap_us` grid, paced by tick32 — the
/// osc-native MGMT CAL train (osc-native-protocol.md §9.3), spacing kept
/// by the crystal. Marks aim at absolute ticks (`t0 + k·gap`), so
/// per-mark overhead never accumulates and a late mark (preemption) never
/// shifts its successors; the receiver gates each gap independently. The
/// first mark leads by gap/2 — past the announce's dispatch, well inside
/// the receiver's 2-gap watchdog. The wire stays claimed for the whole
/// train: the gaps are bus silence, not a handback.
pub fn send_cal(announce: &[u8], gap_us: u32, breaks: u32) -> Result<(), SendError> {
    if announce.is_empty() || announce.len() > BRK_PAYLOAD_MAX {
        return Err(SendError::TooLong);
    }
    let gap_ticks = gap_us.saturating_mul(crate::tick::wire_ticks_per_us());
    // A mark is an 11-bit-time frame; 16 bit-times of gap floor keeps
    // TXE/TC re-armed well before the next mark is due.
    if !(2..=CAL_BREAKS_MAX).contains(&breaks)
        || gap_us > CAL_GAP_MAX_US
        || gap_ticks < USART3.brr().read().0 * 16
    {
        return Err(SendError::TooLong);
    }
    wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
    pb10_drive(true);
    mask_and_retire_idle();
    let r = (|| {
        // See send_break_then: CH2 is shared with the scheduled-send path.
        scheduler::cancel();
        load_payload_main(announce)?;
        USART3.ctlr1().modify(|w| w.set_m(true));
        let r = feed_bytes(&[0x00])
            .and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
        USART3.ctlr1().modify(|w| w.set_m(false));
        r?;
        // Interior kick (see send_burst): the train follows, so the drive
        // must hold across the announce's tail.
        kick_dma();
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
        // The train: every mark is the same 10-bit-exact break shape as
        // the announce's own; M stays set across the whole train.
        USART3.ctlr1().modify(|w| w.set_m(true));
        let t0 = read_tick32().wrapping_add(gap_ticks / 2);
        let mut r = Ok(());
        for k in 0..breaks {
            let due = t0.wrapping_add(k * gap_ticks);
            while (due.wrapping_sub(read_tick32()) as i32) > CAL_LEAD_TICKS as i32 {}
            let mark = critical_section::with(|_| {
                while (due.wrapping_sub(read_tick32()) as i32) > 0 {}
                feed_bytes(&[0x00])
            });
            if mark.is_err() {
                r = mark;
                break;
            }
        }
        let tail = wait_tx_complete().map_err(|TxTimeout| SendError::Busy);
        USART3.ctlr1().modify(|w| w.set_m(false));
        r.and(tail)
    })();
    // See send_break_then: a failed leg must stop the channel.
    if r.is_err() {
        DMA1.ch(1).cr().modify(|w| w.set_en(false));
    }
    // Train gaps exceed a char time, so a latched mid-train IDLE is the
    // norm — retire before re-enabling, still under our drive (see
    // send_breaks).
    retire_and_enable_idle();
    pb10_drive(false);
    r
}

/// Cap on a `BURST` stream (length prefixes + frame bytes). Sized for a
/// hot-loop cycle or a long write bombardment with slack; well under the
/// USB command-line ceiling at 2 hex chars/byte.
pub const BURST_STREAM_MAX: usize = 640;

/// A burst break is one 9-bit 0x00 frame: start + 9 data lows + stop.
const BREAK_FRAME_BITS: u32 = 11;
/// 8N1 payload byte on the wire.
const BITS_PER_BYTE_8N1: u32 = 10;
/// Grid pad between a frame's modeled end and the next break's feed, in
/// bit-times. Sized to cover the in-section handover (M flips + payload
/// kick + DMA pipeline, ~1 µs) so the next mark's TC wait never runs
/// long, while keeping the receiver-visible seam (1 + PAD bit-times —
/// the break's own extra bit plus this pad) inside the servo's
/// chain-pair gate (wire/16, §9.3) for every frame down to a bare
/// COMMIT at 3M.
const BURST_SEAM_PAD_BITS: u32 = 3;

/// Zero-gap multi-frame bombardment: `stream` is length-prefixed frames —
/// `[len_0][frame_0][len_1][frame_1]…` — each sent as one 10-bit-exact
/// break (see [`send_break_then`]) followed back-to-back by its bytes.
/// Host-paced BRKSENDs leave USB-scale gaps between frames; this is the
/// only way to put `[break][frame][break][frame]…` on the wire with
/// sub-byte spacing, which is what the servo's frame-end-work vs
/// next-break timing needs for stress coverage.
///
/// Frame starts ride an absolute tick32 grid (the [`send_cal`] mark
/// pattern): break k feeds at `t0 + Σ(frame spans + pad)`, and the whole
/// per-frame handover — previous TC, M flips, break feed, payload
/// kick — runs inside the mark's critical section. The inter-frame seam
/// is therefore constant by construction, which is what makes this
/// burst valid FOOD for the servo's differential chain-pair tracker
/// (§9.3): a CPU-shaped jittery seam disqualifies every pair.
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
    mask_and_retire_idle();
    let r = (|| {
        // See send_break_then: CH2 is shared with the scheduled-send path.
        scheduler::cancel();
        let brr = USART3.brr().read().0;
        // First mark: far enough out for the first payload load to ride
        // the open window.
        let mut due = read_tick32().wrapping_add(2 * CAL_LEAD_TICKS);
        let mut i = 0usize;
        while i < stream.len() {
            let n = stream[i] as usize;
            i += 1;
            let frame = &stream[i..i + n];
            i += n;
            let last = i == stream.len();
            while (due.wrapping_sub(read_tick32()) as i32) > CAL_LEAD_TICKS as i32 {}
            critical_section::with(|_| -> Result<(), SendError> {
                // The previous payload's TC is due PAD bits before the
                // mark; the horizon is the plateau backstop, not a wait
                // anyone expects to run.
                wait_tc_within(brr * 64)?;
                // Load strictly AFTER the previous payload's TC — the
                // reload disables CH2 mid-transfer otherwise, truncating
                // the frame on the wire (2026-07-12: 2000/2000 stale hot
                // loops, every interior frame CRC-dead). Sub-µs for hot
                // frames, inside the lead either way.
                load_payload_main(frame)?;
                USART3.ctlr1().modify(|w| w.set_m(true));
                let r = (|| {
                    while (due.wrapping_sub(read_tick32()) as i32) > 0 {}
                    feed_bytes(&[0x00])?;
                    wait_tc_within(brr * 64)
                })();
                USART3.ctlr1().modify(|w| w.set_m(false));
                r?;
                if last {
                    // The release must ride the last stop bit only — an
                    // interior release would hand the wire back mid-burst.
                    kick_dma_and_arm_release();
                } else {
                    kick_dma();
                }
                Ok(())
            })?;
            due = due.wrapping_add(
                (BREAK_FRAME_BITS + BITS_PER_BYTE_8N1 * n as u32 + BURST_SEAM_PAD_BITS) * brr,
            );
        }
        // The final payload drains under the armed TC release.
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)
    })();
    // See send_break_then: a failed leg must stop the channel.
    if r.is_err() {
        DMA1.ch(1).cr().modify(|w| w.set_en(false));
    }
    pb10_drive(false);
    r
}

/// Bounded TC spin for the burst grid's in-section waits. Plain wall
/// bound — the grid guarantees TC long before it; the bound only exists
/// so a genuinely stuck shifter surfaces as `Busy` (no-panic rule).
fn wait_tc_within(ticks: u32) -> Result<(), SendError> {
    let t0 = read_tick32();
    while !USART3.statr().read().tc() {
        if read_tick32().wrapping_sub(t0) > ticks {
            return Err(SendError::Busy);
        }
    }
    Ok(())
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
    mask_and_retire_idle();
    let r = (|| {
        feed_bytes(pre)?;
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)?;
        USART3.ctlr1().modify(|w| w.set_m(true));
        let r =
            feed_bytes(bad).and_then(|()| wait_tx_complete().map_err(|TxTimeout| SendError::Busy));
        USART3.ctlr1().modify(|w| w.set_m(false));
        r?;
        feed_bytes(post)?;
        // See send_break_then.
        arm_tc_release();
        wait_tx_complete().map_err(|TxTimeout| SendError::Busy)
    })();
    pb10_drive(false);
    r
}

pub const FE_INJECT_MAX: usize = 32;

/// TXE-poll byte feed for the CPU-fed legs only — break bytes and the
/// FEINJ diagnostic; payloads ride DMA (`kick_dma_and_arm_release`),
/// since every walker/USB preemption of this poll is an inter-byte gap
/// on the wire. `dr` writes are 9 bits wide — a `u8` payload always
/// carries bit 8 = 0, which is exactly what the 9-bit FE-injection
/// frames need.
///
/// The per-byte bound is wall-clock, but this poll runs in thread mode
/// and the walker IRQs (`PRIO_WALKER`) preempt it — a half-ring drain
/// outlasts the byte-time bound. That preemption must not be read as a
/// stuck shifter: TXE goes empty in hardware while we're preempted, so
/// re-check it once the bound expires and only report `Busy` when the
/// register is genuinely still full. (Without this, a long BURST — then
/// CPU-fed — intermittently truncated mid-frame with a spurious `ERR
/// busy`.)
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
