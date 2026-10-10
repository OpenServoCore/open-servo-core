//! High-rate shunt capture. The normal scan samples the shunt once per PWM
//! period, which is ~6 points across a 150 us electrical time constant; a
//! burst suspends the scan and free-runs the converter at one conversion per
//! 1.083 us over a frame of the shunt plus the extras `chans` selects
//! (`chans::INTERLEAVE` gives each extra a shunt slot of its own), steps the
//! bridge halfway through, and freezes `BURST_LEN` codes for paged readback
//! (`regions::burst`). Continuous scan mode repeats the frame with no gap and
//! one DMA request per conversion (RM sec 9.2.4, 9.2.2), so each slot is
//! sampled its slot index x 1.083 us after the frame's first.
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

use portable_atomic::{AtomicU8, Ordering};

use osc_servo_core::kernel::limits::flag;
use osc_servo_core::regions::burst::{
    BURST_LEN, PAGE_MID_COPY, chans, dir, frame_len, page_span, state,
};
use osc_servo_core::regions::{ControlTable, DecaySelect, Mode};
use osc_servo_core::{ControlIo as _, DecayMode, Motor as _, MotorCmd, RegionStorageRaw, Shared};
use osc_units::Effort;

use crate::cfg::chip;
use crate::control::sensors::scan;
use crate::hal::clocks::HCLK_HZ;
use crate::hal::{adc, delay_cycles, dma, timer};
use crate::runtime::statics::KERNEL;

const CH: dma::Channel = dma::Channel::CH1;

/// Kernel ticks between the arm and the launch: 16 ticks = 800 us, long enough
/// for the COMMIT ack to drain off the wire and for the pre-step duty to settle
/// before the window opens.
const BURST_SETTLE_TICKS: u16 = 16;

/// Kernel ticks from one accepted arm to the next: 100 ms. The kernel does
/// not tick while a capture runs, so the spacing in time only grows.
const BURST_SPACING_TICKS: u16 = (chip::MOTOR_PWM_FREQ_HZ / 10) as u16;

/// `Fsm::armed_at` at boot: a full spacing before tick 0, so the first arm
/// is never refused by the spacing rule.
const ARMED_AT_BOOT: u32 = 0u32.wrapping_sub(BURST_SPACING_TICKS as u32);

const CYCLES_PER_US: u32 = HCLK_HZ / 1_000_000;
/// One 7-slot scan retires: 182 ADCCLK at 24 MHz = 7.58 us.
const ADC_SCAN_DRAIN: u32 = 8 * CYCLES_PER_US;
/// Per frame slot, two conversions retire: clearing CONT lets the frame in
/// flight finish, and that CTLR2 write can start one more frame of its own
/// accord. 26 ADCCLK at 24 MHz = 1.08 us each.
const ADC_SLOT_DRAIN: u32 = 3 * CYCLES_PER_US;

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

/// The rest of the FSM: DMA1 CH1 vector (PFIC HIGH) only, never the main loop;
/// `install` writes it once, pre-IRQ.
struct Fsm {
    /// `sample_tick` at the last accepted arm.
    armed_at: u32,
    /// Burst volts cap, vcounts. At 0, before `install`, it refuses every
    /// step but a zero one.
    v_max_counts: u16,
    settle: u16,
    /// The step in the wiring's frame: `drive_polarity` is applied at the
    /// arm, since the kernel's output negation never sees the step.
    duty_q15: i16,
    decay: DecayMode,
    chans: u8,
}

static FSM: SyncUnsafeCell<Fsm> = SyncUnsafeCell::new(Fsm {
    armed_at: ARMED_AT_BOOT,
    v_max_counts: 0,
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

/// Bringup, pre-IRQ: the board's burst volts cap
/// (`Precomputed::burst_v_max_counts`).
pub fn install(v_max_counts: u16) {
    fsm().v_max_counts = v_max_counts;
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

#[inline]
fn publish_state(p: *mut ControlTable, s: u8) {
    STATE.store(s, Ordering::Relaxed);
    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe { (&raw mut (*p).burst.window.state).write_volatile(s) };
}

/// At least the spacing has passed between the arm at `armed_at` and `now`,
/// both `sample_tick` values.
#[inline]
fn spaced(now: u32, armed_at: u32) -> bool {
    now.wrapping_sub(armed_at) >= BURST_SPACING_TICKS as u32
}

/// Tail of the kernel tick: runs the arm/settle/release half of the handshake.
/// The capture half runs on the DMA events, in [`on_dma_event`]. The idle
/// tick reads only the state and the `arm` byte.
#[inline(always)]
pub fn poll_arm(shared: &Shared) {
    let p = shared.table.region_ptr();
    let s = STATE.load(Ordering::Relaxed);
    // SAFETY: transport-owned (bus-level) field, read raw-volatile without
    // forming `&T` -- the kernel's own contract for CONTROL/CONFIG reads.
    let arm = unsafe { (&raw const (*p).control.burst.arm).read_volatile() };
    if s == state::IDLE && arm != 1 {
        return;
    }
    handshake(p, s, arm);
}

#[inline(never)]
fn handshake(p: *mut ControlTable, s: u8, arm: u8) {
    let f = fsm();
    match s {
        state::IDLE => {
            // SAFETY: as in `poll_arm`; `sample_tick` is published by the
            // tick ISR, which is this same context.
            let (req, life, lim_cfg, loop_cur, faults, now) = unsafe {
                (
                    (&raw const (*p).control.burst).read_volatile(),
                    (&raw const (*p).control.lifecycle).read_volatile(),
                    (&raw const (*p).config.limits).read_volatile(),
                    (&raw const (*p).config.loop_current).read_volatile(),
                    (&raw const (*p).telemetry.common.fault_flags).read_volatile(),
                    (&raw const (*p).telemetry.estimates.sample_tick).read_volatile(),
                )
            };
            // SAFETY: as above; `vbus_counts`, `pos` and `limit_flags` are
            // published by the kernel, which runs in this same context.
            let (vbus, pos, lo, hi, limit_flags) = unsafe {
                (
                    (&raw const (*p).telemetry.estimates.vbus_counts).read_volatile(),
                    (&raw const (*p).telemetry.sensors.pos).read_volatile() as i32,
                    (&raw const (*p).config.pos_limits.pos_min_soft_counts).read_volatile(),
                    (&raw const (*p).config.pos_limits.pos_max_soft_counts).read_volatile(),
                    (&raw const (*p).telemetry.limits.limit_flags).read_volatile(),
                )
            };
            // A burst runs past the current limit, the kernel limiter is not
            // ticking under it and a host can re-arm at will: the arm is what
            // bounds the rotor impulse (volts) and its rate (spacing), and a
            // seated burst takes the stall permit's grant, not its request.
            let volts = (req.duty_q15.unsigned_abs() as u32 * vbus as u32) >> 15;
            let armable = life.torque_enable
                && life.mode == Mode::OpenLoop
                && life.tel_count == 0
                && faults == 0
                && req.chans & !chans::ALL == 0
                && spaced(now, f.armed_at)
                && volts <= f.v_max_counts as u32
                && (limit_flags & flag::PERMIT != 0 || (pos > lo && pos < hi));
            if !armable {
                publish_state(p, state::REJECTED);
                return;
            }
            // The field rule already caps |duty_q15| at duty_max_q15; clamping
            // again costs one tick and cannot be skipped by a stale rule.
            let max = loop_cur.duty_max_q15.min(i16::MAX as u16) as i32;
            let duty = (req.duty_q15 as i32).clamp(-max, max) as i16;
            f.duty_q15 = if lim_cfg.drive_polarity { duty } else { -duty };
            f.decay = match lim_cfg.openloop_decay {
                DecaySelect::Slow => DecayMode::Slow,
                DecaySelect::Fast => DecayMode::Fast,
            };
            f.chans = req.chans;
            f.settle = BURST_SETTLE_TICKS;
            f.armed_at = now;
            publish_state(p, state::ARMED);
        }
        state::ARMED => {
            if arm != 1 {
                publish_state(p, state::IDLE);
                return;
            }
            f.settle = f.settle.saturating_sub(1);
            if f.settle == 0 {
                launch(p);
            }
        }
        state::DONE | state::REJECTED => {
            if arm == 0 {
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

/// Suspend the scan and open the capture. This body can outlast the TC's
/// ~17 us of slack, which costs nothing: the tap is shut and both triggers
/// parked in the first write, so no TRGO starts anything after it, and no
/// injected tap conversion resets a frame slot mid-capture and slips the
/// sample grid once a period. That is
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
    // The stamp is the first conversion's phase only if nothing runs between
    // the start and the reads: an ISR there reads DIR after the trough turn
    // and the host folds the whole burst 2 x CNT late.
    let (cnt, dir) = critical_section::with(|_| {
        adc::start_continuous_dma();
        (timer::counter(), dir_now())
    });

    // SAFETY: BURST is RO to the host, so this context is its sole writer.
    unsafe {
        let w = &raw mut (*p).burst.window;
        (&raw mut (*w).start_cnt).write_volatile(cnt);
        (&raw mut (*w).start_dir).write_volatile(dir);
        (&raw mut (*w).pwm_arr).write_volatile(timer::period());
        (&raw mut (*w).samples_len).write_volatile(BURST_LEN as u16);
        (&raw mut (*w).step_index).write_volatile(0);
        (&raw mut (*w).restore_dir).write_volatile(dir::DOWN);
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
    scan::program_sequence();
    scan::arm_dma();
    critical_section::with(|_| {
        adc::arm_scan(adc::Extsel::TIM1_TRGO);
        timer::force_update_event();
    });
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
    // is RO to the host and written only here and from the DMA1 CH1 vector, which
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

#[cfg(test)]
mod tests {
    extern crate std;

    use std::sync::{Mutex, MutexGuard};

    use osc_servo_core::RegionStorage as _;
    use osc_servo_core::regions::config::{BURST_MAX_MV, vmotor_counts};

    use super::*;

    /// `STATE` and `FSM` are statics: one test drives them at a time.
    static SERIAL: Mutex<()> = Mutex::new(());

    /// The osc-dev-v006 terminal taps, 6K4/1K6 at VDD 3300 mV.
    const V_MAX: u16 = vmotor_counts(BURST_MAX_MV, 6_400, 1_600, 3300);
    /// 7.9 V and 4.4 V.
    const RAIL_2S: u16 = 1961;
    const RAIL_USB: u16 = 1090;
    /// The largest step under the cap on each rail: `(d x rail) >> 15` is
    /// 794 here and 795 one count up.
    const EDGE_2S: i16 = 13_284;
    const EDGE_USB: i16 = 23_899;
    const SOFT_MIN: i32 = 300;
    const SOFT_MAX: i32 = 3800;

    /// An OpenLoop servo mid-travel on the 2S rail, the FSM fresh. No poll
    /// here holds a live arm through ARMED, so nothing reaches the hal.
    struct Rig {
        shared: Shared,
        _serial: MutexGuard<'static, ()>,
    }

    fn rig() -> Rig {
        let serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
        STATE.store(state::IDLE, Ordering::Relaxed);
        *fsm() = Fsm {
            armed_at: ARMED_AT_BOOT,
            v_max_counts: 0,
            settle: 0,
            duty_q15: 0,
            decay: DecayMode::Slow,
            chans: 0,
        };
        install(V_MAX);
        let shared = Shared::new();
        shared.table.with_mut(|t| {
            t.control.lifecycle.torque_enable = true;
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.config.limits.drive_polarity = true;
            t.config.loop_current.duty_max_q15 = i16::MAX as u16;
            t.config.pos_limits.pos_min_soft_counts = SOFT_MIN;
            t.config.pos_limits.pos_max_soft_counts = SOFT_MAX;
            t.telemetry.sensors.pos = 2048;
            t.telemetry.estimates.vbus_counts = RAIL_2S;
        });
        Rig {
            shared,
            _serial: serial,
        }
    }

    impl Rig {
        fn set(&self, f: impl FnOnce(&mut ControlTable)) {
            self.shared.table.with_mut(f);
        }

        /// One kernel tick as the ISR runs it: `sample_tick` advances, then
        /// the handshake polls.
        fn tick(&self) {
            self.set(|t| {
                let e = &mut t.telemetry.estimates;
                e.sample_tick = e.sample_tick.wrapping_add(1);
            });
            poll_arm(&self.shared);
        }

        /// One arm request at `duty`: the state it publishes.
        fn arm(&self, duty: i16) -> u8 {
            self.set(|t| {
                t.control.burst.duty_q15 = duty;
                t.control.burst.arm = 1;
            });
            self.tick();
            self.shared.table.with(|t| t.burst.window.state)
        }

        /// Drop the arm; one tick returns the FSM to IDLE.
        fn release(&self) {
            self.set(|t| t.control.burst.arm = 0);
            self.tick();
            assert_eq!(STATE.load(Ordering::Relaxed), state::IDLE);
        }

        fn idle(&self, ticks: u16) {
            for _ in 0..ticks {
                self.tick();
            }
        }
    }

    #[test]
    fn burst_at_the_cap_still_arms() {
        let r = rig();
        assert_eq!(r.arm(13_107), state::ARMED, "40% on 2S");
        drop(r);
        for (rail, edge) in [(RAIL_2S, EDGE_2S), (RAIL_USB, EDGE_USB)] {
            for duty in [edge, -edge] {
                let r = rig();
                r.set(|t| t.telemetry.estimates.vbus_counts = rail);
                assert_eq!(r.arm(duty), state::ARMED, "rail {rail} duty {duty}");
                assert_eq!(fsm().duty_q15, duty);
            }
        }
    }

    /// The step is logical like an OpenLoop duty, so under reversed wiring
    /// it drives the same way as the kernel's negated pre-step hold.
    #[test]
    fn burst_step_follows_the_drive_polarity() {
        for (polarity, wiring) in [(true, EDGE_2S), (false, -EDGE_2S)] {
            for sign in [1, -1] {
                let r = rig();
                r.set(|t| t.config.limits.drive_polarity = polarity);
                assert_eq!(r.arm(sign * EDGE_2S), state::ARMED);
                assert_eq!(fsm().duty_q15, sign * wiring, "polarity {polarity}");
            }
        }
    }

    #[test]
    fn burst_over_the_volts_cap_is_rejected() {
        for (rail, edge) in [(RAIL_2S, EDGE_2S), (RAIL_USB, EDGE_USB)] {
            for duty in [edge + 1, -(edge + 1), i16::MAX, i16::MIN] {
                let r = rig();
                r.set(|t| t.telemetry.estimates.vbus_counts = rail);
                assert_eq!(r.arm(duty), state::REJECTED, "rail {rail} duty {duty}");
                // A refused arm starts no spacing.
                r.release();
                assert_eq!(r.arm(edge), state::ARMED, "rail {rail} after {duty}");
            }
        }
    }

    #[test]
    fn burst_inside_the_spacing_is_rejected() {
        assert_eq!(
            BURST_SPACING_TICKS as u32 * 1000 / chip::MOTOR_PWM_FREQ_HZ,
            100,
            "100 ms of kernel ticks"
        );
        // Armed at tick 0, released at tick 1, asked again at tick `at`.
        let rearm_at = |at: u16| {
            let r = rig();
            assert_eq!(r.arm(EDGE_2S), state::ARMED);
            r.release();
            r.idle(at - 2);
            r.arm(EDGE_2S)
        };
        assert_eq!(rearm_at(2), state::REJECTED);
        assert_eq!(rearm_at(BURST_SPACING_TICKS - 1), state::REJECTED);
        assert_eq!(rearm_at(BURST_SPACING_TICKS), state::ARMED);
    }

    #[test]
    fn spacing_counts_kernel_ticks_since_the_last_arm() {
        let spacing = BURST_SPACING_TICKS as u32;
        for now in [0, 1, 72_000_000] {
            assert!(spaced(now, ARMED_AT_BOOT), "first arm after boot at {now}");
        }
        assert!(!spaced(1000 + spacing - 1, 1000), "inside the spacing");
        assert!(spaced(1000 + spacing, 1000), "exactly at the spacing");
        let armed_at = u32::MAX - 10;
        assert!(!spaced(armed_at.wrapping_add(spacing - 1), armed_at));
        assert!(
            spaced(armed_at.wrapping_add(spacing), armed_at),
            "across the wrap"
        );
    }

    #[test]
    fn burst_outside_the_soft_limits_needs_the_permit() {
        for pos in [0, SOFT_MIN as u16, SOFT_MAX as u16, 4095] {
            let r = rig();
            r.set(|t| t.telemetry.sensors.pos = pos);
            assert_eq!(r.arm(EDGE_2S), state::REJECTED, "pos {pos}");
            r.release();
            // The request byte alone is not the grant.
            r.set(|t| t.control.lifecycle.stall_permit = true);
            assert_eq!(r.arm(EDGE_2S), state::REJECTED, "pos {pos} requested");
            r.release();
            r.set(|t| t.telemetry.limits.limit_flags = flag::PERMIT);
            assert_eq!(r.arm(EDGE_2S), state::ARMED, "pos {pos} granted");
        }
        for pos in [SOFT_MIN as u16 + 1, SOFT_MAX as u16 - 1] {
            let r = rig();
            r.set(|t| t.telemetry.sensors.pos = pos);
            assert_eq!(r.arm(EDGE_2S), state::ARMED, "pos {pos}");
        }
    }
}
