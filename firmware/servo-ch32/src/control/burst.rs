//! High-rate shunt capture. The normal scan samples the shunt once per PWM
//! period, which is ~6 points across a 150 us electrical time constant; a
//! burst suspends the scan and free-runs the converter at one conversion per
//! 1.083 us over a frame of the shunt plus the extras `chans` selects, steps
//! the bridge halfway through, and freezes `BURST_LEN` codes for paged
//! readback (`regions::burst`). Continuous scan mode repeats the frame with
//! no gap and one DMA request per conversion (RM sec 9.2.4, 9.2.2), so each
//! extra is sampled its slot index x 1.083 us after the shunt.
//!
//! The burst time-shares DMA1 CH1 with the scan, so its HT and TC land on the
//! kernel's own vector: `runtime::isr` routes them here while `capturing()`
//! and the kernel receives no ticks for the ~1.04 ms window. Silicon
//! constraints the swap is built around (RM sec 9.3.3, 9.2.2, and the V006
//! latching-request fact): any CTLR2 write with ADON set starts a conversion,
//! ADON off->on costs tSTAB so ADON is never touched here, and a DMA request
//! pulsed at a disabled channel latches and fires at re-enable -- so the
//! request source gated is always `ADC.CTLR2.DMA`, never `DMA1_CH1.CR.EN`.

use core::cell::SyncUnsafeCell;
use core::sync::atomic::compiler_fence;

use portable_atomic::{AtomicBool, AtomicU8, Ordering};

use osc_servo_core::regions::burst::{
    BURST_LEN, PAGE_MID_COPY, chans, dir, frame_len, page_span, state,
};
use osc_servo_core::regions::{ControlTable, DecaySelect, Mode};
use osc_servo_core::{ControlIo as _, DecayMode, Motor as _, MotorCmd, RegionStorageRaw, Shared};
use osc_units::Effort;

use crate::control::sensors::scan;
use crate::hal::clocks::HCLK_HZ;
use crate::hal::{adc, delay_cycles, dma, timer};
use crate::runtime::statics::KERNEL;

const CH: dma::Channel = dma::Channel::CH1;

/// Kernel ticks between the arm and the launch: 16 ticks = 800 us, long enough
/// for the COMMIT ack to drain off the wire and for the pre-step duty to settle
/// before the window opens.
const BURST_SETTLE_TICKS: u16 = 16;

/// `delay_cycles` iterations per microsecond. It spins on `spin_loop`, which
/// costs at least one HCLK cycle per iteration and on this core rather more,
/// so sizing a drain in HCLK cycles is a floor on the wait, never a ceiling.
const DRAIN_ITERS_PER_US: u32 = HCLK_HZ / 1_000_000;
/// One 7-slot scan retires: 182 ADCCLK at 24 MHz = 7.58 us.
const ADC_SCAN_DRAIN: u32 = 8 * DRAIN_ITERS_PER_US;
/// Per frame slot, two conversions retire: clearing CONT lets the frame in
/// flight finish, and that CTLR2 write can start one more frame of its own
/// accord. 26 ADCCLK at 24 MHz = 1.08 us each.
const ADC_SLOT_DRAIN: u32 = 3 * DRAIN_ITERS_PER_US;

/// Free-running frame capture. Not circular: the run stops itself at
/// TC and the buffer stays frozen for readback. HT is the step trigger.
const DMA_CFG: dma::Config = dma::Config {
    dir: dma::Dir::FROMPERIPHERAL,
    circ: false,
    pinc: false,
    minc: true,
    size: dma::Size::BITS16,
    htie: true,
    tcie: true,
    pl: dma::Pl::HIGH,
};

static BURST_BUF: SyncUnsafeCell<[u16; BURST_LEN]> = SyncUnsafeCell::new([0; BURST_LEN]);

/// The FSM state, atomic because the ISR prologue and the main loop both read
/// it while the DMA1 CH1 vector writes it. Written only from that vector.
static STATE: AtomicU8 = AtomicU8::new(state::IDLE);

/// Set at restore, cleared by the scan TC that stamps `restore_dir`. Same
/// vector writes and reads it; atomic so the prologue's load stands on its own.
static WITNESS_DUE: AtomicBool = AtomicBool::new(false);

/// The rest of the FSM: DMA1 CH1 vector (PFIC LOW) only, never the main loop.
struct Fsm {
    settle: u16,
    duty_q15: i16,
    decay: DecayMode,
    chans: u8,
}

static FSM: SyncUnsafeCell<Fsm> = SyncUnsafeCell::new(Fsm {
    settle: 0,
    duty_q15: 0,
    decay: DecayMode::Slow,
    chans: 0,
});

#[inline(always)]
fn fsm() -> &'static mut Fsm {
    // SAFETY: see the FSM doc -- reached only from the DMA1 CH1 vector, which
    // never preempts itself.
    unsafe { &mut *FSM.get() }
}

/// The ISR prologue's whole question: is the scan suspended?
#[inline(always)]
pub fn capturing() -> bool {
    STATE.load(Ordering::Relaxed) == state::CAPTURING
}

/// TIM1's counting phase in the published `dir` encoding.
#[inline]
fn dir_now() -> u8 {
    if timer::counting_down() {
        dir::DOWN
    } else {
        dir::UP
    }
}

/// Scan-geometry witness, and it MUST be sampled at the scan TC itself. DIR
/// answers "which scan landed second" only inside the ~17 us between the peak
/// scan's TC and the next trough trigger; the kernel body that runs after this
/// point is longer than that window, so a sample taken at the ISR tail reads a
/// later, arbitrary half of the period and says nothing (bench: UP on every
/// capture while the geometry was verifiably correct).
#[inline(always)]
pub fn witness_scan_tc(shared: &Shared) {
    if !WITNESS_DUE.load(Ordering::Relaxed) {
        return;
    }
    WITNESS_DUE.store(false, Ordering::Relaxed);
    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe {
        (&raw mut (*shared.table.region_ptr()).burst.window.restore_dir).write_volatile(dir_now())
    };
}

#[inline]
fn publish_state(p: *mut ControlTable, s: u8) {
    STATE.store(s, Ordering::Relaxed);
    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe { (&raw mut (*p).burst.window.state).write_volatile(s) };
}

/// Tail of the kernel tick: runs the arm/settle/release half of the handshake.
/// The capture half runs on the DMA events, in [`on_dma_event`].
pub fn poll_arm(shared: &Shared) {
    let p = shared.table.region_ptr();
    // SAFETY: transport-owned (PFIC HIGH) fields, read raw-volatile without
    // forming `&T` -- the kernel's own contract for CONTROL/CONFIG reads.
    let (req, life, lim_cfg, loop_cur, faults) = unsafe {
        (
            (&raw const (*p).control.burst).read_volatile(),
            (&raw const (*p).control.lifecycle).read_volatile(),
            (&raw const (*p).config.limits).read_volatile(),
            (&raw const (*p).config.loop_current).read_volatile(),
            (&raw const (*p).telemetry.common.fault_flags).read_volatile(),
        )
    };
    let f = fsm();

    match STATE.load(Ordering::Relaxed) {
        state::IDLE => {
            if req.arm != 1 {
                return;
            }
            let armable = life.torque_enable
                && life.mode == Mode::OpenLoop
                && life.tel_count == 0
                && faults == 0
                && req.chans & !chans::ALL == 0;
            if !armable {
                publish_state(p, state::REJECTED);
                return;
            }
            // The field rule already caps |duty_q15| at duty_max_q15; clamping
            // again costs one tick and cannot be skipped by a stale rule.
            let max = loop_cur.duty_max_q15.min(i16::MAX as u16) as i32;
            f.duty_q15 = (req.duty_q15 as i32).clamp(-max, max) as i16;
            f.decay = match lim_cfg.openloop_decay {
                DecaySelect::Slow => DecayMode::Slow,
                DecaySelect::Fast => DecayMode::Fast,
            };
            f.chans = req.chans;
            f.settle = BURST_SETTLE_TICKS;
            publish_state(p, state::ARMED);
        }
        state::ARMED => {
            if req.arm != 1 {
                publish_state(p, state::IDLE);
                return;
            }
            f.settle = f.settle.saturating_sub(1);
            if f.settle == 0 {
                launch(p);
            }
        }
        state::DONE | state::REJECTED => {
            if req.arm == 0 {
                publish_state(p, state::IDLE);
            }
        }
        _ => publish_state(p, state::IDLE),
    }
}

/// DMA1 CH1 events while the scan is suspended: HT applies the step, TC
/// restores the scan. Both flags are polled because a delayed vector entry can
/// find them together.
pub fn on_dma_event(shared: &Shared) {
    let p = shared.table.region_ptr();
    if dma::is_ht_flag(CH) {
        dma::clear_ht_flag(CH);
        step(p);
    }
    if dma::is_tc_flag(CH) {
        dma::clear_tc_flag(CH);
        restore();
        publish_state(p, state::DONE);
    }
}

/// Suspend the scan and open the capture. The drains below outlast the TC's
/// ~17 us of slack, which costs nothing: the tap is shut and the trigger
/// parked in the first write, so no TRGO starts anything after it. That is
/// also why `start_cnt` / `start_dir` record the phase of the launch write
/// itself rather than the phase of the TC that led to it.
fn launch(p: *mut ControlTable) {
    let f = fsm();
    // Source first, channel second: a request can only latch if it is
    // GENERATED while the channel is disabled, so shutting CTLR2.DMA before
    // DMA1_CH1.CR.EN is what makes the disable safe (the reverse order is the
    // V006 latching-request trap). Any request already generated is served
    // while the channel is still enabled.
    adc::park();
    dma::disable(CH);
    dma::clear_tc_flag(CH);
    dma::clear_ht_flag(CH);
    // The park write started a stray scan, or found one a TRGO had started;
    // either makes no DMA request with the tap shut, and it must retire
    // before RSQR changes: an RSQR write under a live conversion restarts it
    // on the new group (RM sec 9.2.2), and a frame the launch write then found
    // mid-round would deliver slot k first and rotate every frame after.
    delay_cycles(ADC_SCAN_DRAIN);
    let len = frame_len(f.chans);
    adc::set_scan_mode(len > 1);
    scan::set_burst_sequence(f.chans);
    dma::configure(
        CH,
        &DMA_CFG,
        adc::data_addr(),
        BURST_BUF.get() as u32,
        BURST_LEN as u16,
    );
    dma::enable(CH);
    adc::start_continuous_dma();

    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe {
        let w = &raw mut (*p).burst.window;
        (&raw mut (*w).start_cnt).write_volatile(timer::counter());
        (&raw mut (*w).start_dir).write_volatile(dir_now());
        (&raw mut (*w).pwm_arr).write_volatile(timer::period());
        (&raw mut (*w).samples_len).write_volatile(BURST_LEN as u16);
        (&raw mut (*w).step_index).write_volatile(0);
        (&raw mut (*w).restore_dir).write_volatile(dir::UP);
        (&raw mut (*w).chans_echo).write_volatile(f.chans);
        (&raw mut (*w).frame_len).write_volatile(len);
        // No page of this capture is published yet; a host reading before its
        // first page write must not be handed the previous capture's page.
        (&raw mut (*w).page_echo).write_volatile(PAGE_MID_COPY);
    }
    publish_state(p, state::CAPTURING);
}

/// Half-transfer: apply the step. OCPE is set, so the CCR write latches at the
/// next update event, <= 25 us out and never mid-pulse.
fn step(p: *mut ControlTable) {
    let f = fsm();
    // SAFETY: the kernel is not ticking (no scan TC reaches it while
    // capturing) and this is the vector that owns it.
    let kernel = unsafe { (*KERNEL.get()).assume_init_mut() };
    let (_sensors, motor) = kernel.io.parts();
    motor.write(MotorCmd::Drive {
        duty: Effort(f.duty_q15),
        decay: f.decay,
    });
    // Measured, not assumed: the ISR entry latency lands in the index.
    let at = BURST_LEN as u16 - dma::remaining(CH);
    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe { (&raw mut (*p).burst.window.step_index).write_volatile(at) };
}

/// Put the scan back, unconditionally.
///
/// ORDER IS LOAD-BEARING THROUGHOUT. EXTSEL stays at SWSTART, with no SWSTART
/// ever pulsed, from entry until the last write: while it does, NOTHING can
/// trigger a scan, so the converter is provably idle when SCAN and RSQR are
/// reprogrammed and no scan can be in flight when the DMA tap opens. Re-arming
/// the trigger any earlier loses that (bench: a TRGO landing between the
/// re-arm and the tap opening started a scan the DMA never delivered, the tap
/// opened mid-sequence, and every frame after came back rotated).
///
/// The tail then fixes the scan geometry (trough slots at offset 0, peak at
/// `ADC_SCAN_LEN`), which is decided by which scan the freshly-opened tap
/// delivers first. `arm_scan` is one CTLR2 write, and with ADON already on it
/// starts a scan itself; that scan is the trough scan, and UG's TRGO -- fired
/// on the very next instruction -- is swallowed by the busy converter, so the
/// next crest TRGO fills the peak half. If the ADON on->on write should ever
/// NOT start a conversion, UG's own TRGO starts the trough scan instead and
/// the geometry is the same. A preemption between the two stores is the one
/// interleave that breaks it, which is what the critical section forbids. UG
/// costs one truncated PWM period; at CNT = 0 both legs are in the normal
/// brake state, so the early brake is <= 25 us and MOE is untouched.
fn restore() {
    // Source first, channel second: a request can only latch if it is
    // GENERATED while the channel is disabled, so shutting CTLR2.DMA before
    // DMA1_CH1.CR.EN is what makes the disable safe (the reverse order is the
    // V006 latching-request trap). Any request already generated is served
    // while the channel is still enabled.
    adc::set_dma(false);
    dma::disable(CH);
    dma::clear_tc_flag(CH);
    dma::clear_ht_flag(CH);
    adc::set_continuous(false);
    delay_cycles(ADC_SLOT_DRAIN * frame_len(fsm().chans) as u32);
    adc::set_scan_mode(true);
    adc::set_sequence(scan::seq());
    scan::arm_dma();
    critical_section::with(|_| {
        adc::arm_scan(adc::Extsel::TIM1_TRGO);
        timer::force_update_event();
    });
    WITNESS_DUE.store(true, Ordering::Relaxed);
}

/// Main-loop page copy. Publishing the page under `PAGE_MID_COPY` and only
/// then stamping `page_echo` is the whole interlock: the reply snapshot reads
/// offset 0 first, so a host that observes `page_echo == page` is reading that
/// page's samples, and every other interleave reads back a value it rejects.
pub fn poll_page(shared: &Shared) {
    if STATE.load(Ordering::Relaxed) != state::DONE {
        return;
    }
    let p = shared.table.region_ptr();
    // SAFETY: `page` is transport-owned, read raw-volatile; the BURST window
    // is RO to the host and written only here and from the LOW vector, which
    // touches nothing in it while Done stands.
    unsafe {
        let page = (&raw const (*p).control.burst.page).read_volatile();
        if (&raw const (*p).burst.window.page_echo).read_volatile() == page {
            return;
        }
        let Some((from, to)) = page_span(page) else {
            return;
        };
        let w = &raw mut (*p).burst.window;
        (&raw mut (*w).page_echo).write_volatile(PAGE_MID_COPY);
        compiler_fence(Ordering::SeqCst);
        let src = BURST_BUF.get() as *const u16;
        let dst = (&raw mut (*w).samples) as *mut u16;
        for (i, at) in (from..to).enumerate() {
            dst.add(i).write_volatile(src.add(at).read_volatile());
        }
        compiler_fence(Ordering::SeqCst);
        (&raw mut (*w).page_echo).write_volatile(page);
    }
}
