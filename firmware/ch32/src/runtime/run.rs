//! Top-level program entry. `run!` is the board binary's main: it validates
//! the `BoardConfig` literal at compile time, calls into bringup, installs
//! the kernel + IRQs, and enters the main loop.

use osc_core::{BootMode, RegionStorageRaw};
use osc_drivers::led::Pattern;
use osc_drivers::traits::Monotonic as _;
use portable_atomic::Ordering;

use osc_drivers::bus::RESCUE_LOW_US;

use crate::cfg::{BoardConfig, Precomputed, chip};
use crate::control::Ch32ControlIo;
use crate::hal::{dma, flash, gpio, pfic, rcc};
use crate::providers::monotonic::Monotonic;

/// STAT LED talk-blink: full blink cycle while the bus is talking (the
/// pirate's cadence), and how long "talking" outlives the last wire IRQ
/// so gapped exchanges read as one episode.
const TALK_BLINK_PERIOD_US: u32 = 100_000;
const TALK_HOLD_US: u32 = 200_000;

/// Const-asserts pin-uniqueness on the `BoardConfig` literal, then runs.
#[macro_export]
macro_rules! run {
    ($cfg:expr) => {{
        const __OSC_CH32_CFG: $crate::cfg::BoardConfig = $cfg;
        const __OSC_CH32_PRE: $crate::cfg::Precomputed =
            $crate::cfg::Precomputed::compute(&__OSC_CH32_CFG);
        const _: () = __OSC_CH32_CFG.wiring.assert_valid();
        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        // INTSYSCR stays at qingke-rt's 0x3 (HPE + nesting): the "HPE
        // corrupts t0/t2" episode was a wlink write-mem artifact (see
        // runtime/isr.rs).
        unsafe { ::qingke::pfic::wfi_to_wfe(false) };
        $crate::runtime::run::__run(__OSC_CH32_CFG, __OSC_CH32_PRE)
    }};
}

#[doc(hidden)]
pub fn __run(cfg: BoardConfig, pre: Precomputed) -> ! {
    let io = Ch32ControlIo::new(cfg, pre);
    crate::runtime::statics::install(io);
    crate::runtime::isr::install_irqs();
    // Last-published transport counters: the table gets DELTAS, so a host
    // zero-write (the rw clear contract, `TelemetryBusLink`) sticks instead
    // of being clobbered by the next publish of a monotonic total.
    let mut published = osc_drivers::bus::LinkDiag {
        crc_fail_count: 0,
        framing_drop_count: 0,
    };
    let talk_hold_ticks = TALK_HOLD_US * Monotonic::TICKS_PER_US;
    let mut last_talk = Monotonic.ticks().wrapping_sub(talk_hold_ticks);
    let rescue_low_ticks = RESCUE_LOW_US * Monotonic::TICKS_PER_US;
    let mut rescue_low_since: Option<u32> = None;
    let mut rescue_ndtr: u16 = dma::remaining(dma::Channel::CH5);
    loop {
        // Transport RX/TX/deadlines are ISR-driven (USART1 + SysTick, PFIC
        // HIGH). Main loop owns LED housekeeping, the link-diagnostics
        // publish, the deferred-reboot poll, and sleep.
        //
        // STAT LED: solid when idle, blinking while the bus talks -- any
        // USART1 wire event latches `BUS_ACTIVITY`, and the blink holds
        // past the last event so an exchange reads as one episode. The
        // `wfi` wake cadence is the poll cadence: wire IRQs while talking,
        // the 20 kHz kernel tick otherwise.
        if crate::runtime::registry::BUS_ACTIVITY.swap(false, Ordering::Relaxed) {
            last_talk = Monotonic.ticks();
        }
        let talking = Monotonic.ticks().wrapping_sub(last_talk) < talk_hold_ticks;
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        let led = unsafe { crate::runtime::Drivers::stat_led() };
        led.set_pattern(if talking {
            Pattern::Blink {
                period_us: TALK_BLINK_PERIOD_US,
            }
        } else {
            Pattern::SolidOn
        });
        led.poll();

        // Publish transport health into the telemetry region (protocol sec 5.3 layer 1:
        // dropped frames are counted, never answered). The critical section
        // makes the `bus()` reach-in non-aliasing (HIGH owns it otherwise)
        // and folds the read-modify-write against a concurrent host clear
        // committing from HIGH.
        critical_section::with(|_| {
            // SAFETY: bus installed in bringup; ISRs masked by the CS.
            let diag = unsafe { crate::runtime::Drivers::bus() }.diag();
            // SAFETY: table storage is 'static; field access is volatile and
            // ISR-masked, mirroring the sample_tick idiom in `isr.rs`.
            unsafe {
                let link = &raw mut (*crate::runtime::statics::SHARED.table.region_ptr())
                    .telemetry
                    .link;
                let crc = &raw mut (*link).crc_fail_count;
                crc.write_volatile(
                    crc.read_volatile()
                        .wrapping_add(diag.crc_fail_count.wrapping_sub(published.crc_fail_count)),
                );
                let drops = &raw mut (*link).framing_drop_count;
                drops.write_volatile(
                    drops.read_volatile().wrapping_add(
                        diag.framing_drop_count
                            .wrapping_sub(published.framing_drop_count),
                    ),
                );
            }
            published = diag;
        });

        // Clock-trim loop (protocol sec 9.3): the transport measures
        // host-instruction byte cadence ISR-side; the correction lands here,
        // outside any transport ISR. A correction is <= 4 steps (~1%) -- well
        // inside the framing budget vs a crystal host, so trimming over a live
        // wire is safe (measured: the manual-knob experiment trimmed the
        // fleet mid-traffic with zero errors). The applied total mirrors
        // into telemetry, read-only, for fleet diagnosis.
        let trim = critical_section::with(|_| {
            // SAFETY: bus installed in bringup; ISRs masked by the CS.
            unsafe { crate::runtime::Drivers::bus() }.poll_clock_trim()
        });
        if let Some(total) = trim {
            rcc::apply_clock_trim(total);
            // SAFETY: table storage is 'static; single-byte volatile store
            // to a read-only field (regmap can't race it).
            unsafe {
                (&raw mut (*crate::runtime::statics::SHARED.table.region_ptr())
                    .telemetry
                    .clock
                    .trim_steps)
                    .write_volatile(total);
            }
        }

        // Deferred reboot (protocol sec 9.5), honored after the ack has drained. The
        // critical section is load-bearing: `bus()` is otherwise `&mut`-owned
        // by the HIGH transport ISRs, so masking them is what makes this
        // main-loop reach-in non-aliasing. Flash writes stay out of the ISR
        // bodies -- the stall is lethal under a live control loop.
        let reboot: Option<BootMode> =
            critical_section::with(|_| unsafe { crate::runtime::Drivers::bus() }.take_reboot());
        if let Some(mode) = reboot {
            flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
            pfic::software_reset();
        }

        // Rescue sampler (protocol sec 9.1). The break detector latches only at a
        // dominant span's END, so no transport wake can
        // observe a rescue pulse in progress -- the slow loop measures it
        // instead, which is where a 300 us-scale signal belongs. One sample
        // per wfi wake; the 20 kHz ADC tick is the idle metronome, so the
        // worst-case cadence is ~50 us against a >= 300 us window. The window
        // requires every sample low AND the RX ring frozen: any completed
        // character moves NDTR (a real dominant low delivers no start
        // edges), so UART traffic at any baud -- including the pulse's own
        // ringed 0x00, which re-anchors the window ~a byte-time in -- can
        // never impersonate a pulse. Declaration runs under a critical
        // section (the bus is otherwise HIGH-ISR-owned) while the pulse
        // still holds the line, so the driver resyncs at a provably-still
        // cursor; the window restart afterwards makes a continuing low
        // redeclare idempotently rather than repeat-fire.
        let ndtr = dma::remaining(dma::Channel::CH5);
        if !gpio::is_low(chip::BUS_LINE_PIN) || ndtr != rescue_ndtr {
            rescue_low_since = None;
            rescue_ndtr = ndtr;
        } else if let Some(t0) = rescue_low_since {
            if Monotonic.ticks().wrapping_sub(t0) >= rescue_low_ticks {
                rescue_low_since = None;
                critical_section::with(|_| {
                    // SAFETY: bus installed in bringup; ISRs masked by the CS.
                    unsafe { crate::runtime::Drivers::bus() }.on_rescue_break()
                });
            }
        } else {
            rescue_low_since = Some(Monotonic.ticks());
        }

        riscv::asm::wfi();
    }
}
