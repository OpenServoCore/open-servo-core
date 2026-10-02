//! Top-level program entry. `run!` is the board binary's main: it validates
//! the `BoardConfig` literal at compile time, calls into bringup, installs
//! the kernel + IRQs, and enters the main loop.

use osc_servo_core::{BootMode, RegionStorageRaw};
use osc_servo_drivers::traits::Monotonic as _;

use crate::cfg::{BoardConfig, Precomputed, chip};
use crate::control::Ch32ControlIo;
use crate::hal::{flash, gpio, pfic, rcc};
use crate::providers::monotonic::Monotonic;

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
    #[cfg(target_arch = "riscv32")]
    let mut stack = crate::runtime::stack::paint();
    let io = Ch32ControlIo::new(cfg, pre);
    crate::runtime::statics::install(io, pre.kernel_timing);
    crate::runtime::isr::install_irqs();
    // Last-published transport counters: the table gets DELTAS, so a host
    // zero-write (the rw clear contract, `TelemetryCommon`) sticks instead
    // of being clobbered by the next publish of a monotonic total.
    let mut published = osc_servo_drivers::bus::LinkDiag {
        crc_fail_count: 0,
        framing_drop_count: 0,
    };
    let mut tel_published: u16 = 0;
    let mut lamp_test = crate::runtime::stat::LampTest::new(Monotonic.ticks());
    loop {
        // Transport RX/TX/deadlines are ISR-driven (TIM2 + USART1 + SysTick,
        // PFIC HIGH). Main loop owns LED housekeeping, the link-diagnostics
        // publish, the TEL burst poll, the deferred-reboot poll, and sleep.
        //
        // STAT lamp (`runtime::stat`). The `wfi` wake cadence is the poll
        // cadence: the 20 kHz kernel tick at the slowest.
        // SAFETY: table storage is 'static; single-byte volatile loads, and
        // a write landing from an ISR mid-pair costs one stale pass.
        let (fault_code, data_flags) = unsafe {
            let mode = &raw const (*crate::runtime::statics::SHARED.table.region_ptr())
                .telemetry
                .mode;
            (
                (&raw const (*mode).fault_code).read_volatile(),
                (&raw const (*mode).data_flags).read_volatile(),
            )
        };
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        let led = unsafe { crate::runtime::Drivers::stat_led() };
        led.set_pattern(crate::runtime::stat::pattern(
            lamp_test.running(Monotonic.ticks()),
            fault_code,
            data_flags,
        ));
        led.poll();

        // Publish transport health into the telemetry region (protocol sec 5.3 layer 1:
        // dropped frames are counted, never answered), and the TEL rows the
        // kernel dropped. The critical section makes the `bus()` reach-in
        // non-aliasing (HIGH owns it otherwise) and folds the
        // read-modify-write against a concurrent host clear committing from
        // HIGH.
        critical_section::with(|_| {
            // SAFETY: bus installed in bringup; ISRs masked by the CS.
            let diag = unsafe { crate::runtime::Drivers::bus() }.diag();
            let tel_drops = crate::runtime::statics::TEL_CHANNEL.drops();
            // SAFETY: table storage is 'static; field access is volatile and
            // ISR-masked, mirroring the sample_tick idiom in `isr.rs`.
            unsafe {
                let telemetry =
                    &raw mut (*crate::runtime::statics::SHARED.table.region_ptr()).telemetry;
                let common = &raw mut (*telemetry).common;
                let tel = &raw mut (*telemetry).health.tel_drop_count;
                tel.write_volatile(
                    tel.read_volatile()
                        .wrapping_add(tel_drops.wrapping_sub(tel_published)),
                );
                let crc = &raw mut (*common).crc_fail_count;
                crc.write_volatile(
                    crc.read_volatile()
                        .wrapping_add(diag.crc_fail_count.wrapping_sub(published.crc_fail_count)),
                );
                let drops = &raw mut (*common).framing_drop_count;
                drops.write_volatile(
                    drops.read_volatile().wrapping_add(
                        diag.framing_drop_count
                            .wrapping_sub(published.framing_drop_count),
                    ),
                );
            }
            published = diag;
            tel_published = tel_drops;
        });

        #[cfg(target_arch = "riscv32")]
        if let Some(free) = stack.step() {
            // SAFETY: table storage is 'static; the main loop is the only
            // writer of this read-only field.
            unsafe {
                (&raw mut (*crate::runtime::statics::SHARED.table.region_ptr())
                    .telemetry
                    .health
                    .stack_free_min)
                    .write_volatile(free);
            }
        }

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
                    .common
                    .trim_steps)
                    .write_volatile(total);
            }
        }

        // TEL burst (protocol sec 5.3): stage the next kernel-encoded batch
        // onto the reply TX path when the wire is ours. Every wfi wake polls
        // (>= 20 kHz); a batch banks at most every ~800 us, so the cadence
        // has huge margin. Same reach-in contract as `take_reboot` below.
        critical_section::with(|_| {
            // SAFETY: bus installed in bringup; ISRs masked by the CS.
            unsafe { crate::runtime::Drivers::bus() }.poll_tel()
        });

        // Shunt-burst page copy: republishes one page of the frozen capture
        // into the BURST window. Main-loop side on purpose -- the copy is
        // ~120 words and has no business inside a 20 kHz ISR.
        crate::control::burst::poll_page(&crate::runtime::statics::SHARED);

        // Data job (core `data_state`): the stamp checkpoint a torque-off
        // stamp write posts and the verdict a LUT COMMIT posts, ~0.6 ms of
        // software CRC that outlasts the reply deadline in HIGH. The run is
        // preemptible; the publish masks ISRs so HIGH cannot land a write
        // between its generation check and the table stores. The posting
        // ISR's return is the wfi wake, so the job runs before the next
        // frame can arrive.
        if let Some(job) = crate::runtime::statics::SHARED.data_job_run() {
            critical_section::with(|_| crate::runtime::statics::SHARED.data_job_publish(job));
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

        // Rescue sampler (protocol sec 9.1). The break detector wakes once
        // per dominant span, a break-length in, so no transport wake can
        // measure a rescue pulse; the slow loop measures it instead, one
        // sample per wfi wake. The pin is read inside the critical section
        // that also reads the TX state and declares, so a TX release cannot
        // land between them.
        critical_section::with(|_| {
            let low = gpio::is_low(chip::BUS_USART_MAPPING.tx_pin());
            // SAFETY: bus installed in bringup; ISRs masked by the CS.
            unsafe { crate::runtime::Drivers::bus() }.sample_rescue(low)
        });

        riscv::asm::wfi();
    }
}
