//! Top-level program entry. `run!` is the board binary's main: it validates
//! the `BoardConfig` literal at compile time, calls into bringup, installs
//! the kernel + IRQs, and enters the main loop.

use osc_core::{BootMode, RegionStorageRaw};

use crate::cfg::{BoardConfig, Precomputed};
use crate::control::Ch32ControlIo;
use crate::hal::{flash, pfic, rcc};

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
    // Last-applied clock trim; table default 0 = the chip's own HSITRIM.
    let mut trim_applied: i8 = 0;
    loop {
        // Transport RX/TX/deadlines are ISR-driven (USART1 + SysTick, PFIC
        // HIGH). Main loop owns LED housekeeping, the link-diagnostics
        // publish, the deferred-reboot poll, and sleep.
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        unsafe { crate::runtime::Drivers::stat_led() }.poll();

        // Publish transport health into the telemetry region (§5.3 layer 1:
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

        // Clock-trim knob (`control.system.hsi_trim_offset`, ~0.25%/step):
        // applied here so the RCC write lands between frames, never inside
        // a transport ISR. The jump is well inside the single-sided framing
        // budget vs a crystal host, so trimming over the wire is safe.
        // SAFETY: table storage is 'static; single-byte volatile read.
        let trim = unsafe {
            (&raw const (*crate::runtime::statics::SHARED.table.region_ptr())
                .control
                .system
                .hsi_trim_offset)
                .read_volatile()
        };
        if trim != trim_applied {
            trim_applied = trim;
            rcc::apply_clock_trim_delta(trim);
        }

        // Deferred reboot (§9.5), honored after the ack has drained. The
        // critical section is load-bearing: `bus()` is otherwise `&mut`-owned
        // by the HIGH transport ISRs, so masking them is what makes this
        // main-loop reach-in non-aliasing. Flash writes stay out of the ISR
        // bodies — the stall is lethal under a live control loop.
        let reboot: Option<BootMode> =
            critical_section::with(|_| unsafe { crate::runtime::Drivers::bus() }.take_reboot());
        if let Some(mode) = reboot {
            flash::set_boot_mode(matches!(mode, BootMode::Bootloader));
            pfic::software_reset();
        }

        riscv::asm::wfi();
    }
}
