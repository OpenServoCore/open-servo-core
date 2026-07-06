//! Top-level program entry. `run!` is the board binary's main: it validates
//! the `BoardConfig` literal at compile time, calls into bringup, installs
//! the kernel + IRQs, and enters the main loop.

use osc_core::BootMode;

use crate::cfg::{BoardConfig, Precomputed};
use crate::control::Ch32ControlIo;
use crate::hal::{flash, pfic};

/// Const-asserts pin-uniqueness on the `BoardConfig` literal, then runs.
#[macro_export]
macro_rules! run {
    ($cfg:expr) => {{
        const __OSC_CH32_CFG: $crate::cfg::BoardConfig = $cfg;
        const __OSC_CH32_PRE: $crate::cfg::Precomputed =
            $crate::cfg::Precomputed::compute(&__OSC_CH32_CFG);
        const _: () = __OSC_CH32_CFG.wiring.assert_valid();
        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        unsafe { ::qingke::pfic::wfi_to_wfe(false) };
        $crate::runtime::run::__run(__OSC_CH32_CFG, __OSC_CH32_PRE)
    }};
}

#[doc(hidden)]
pub fn __run(cfg: BoardConfig, pre: Precomputed) -> ! {
    let io = Ch32ControlIo::new(cfg, pre);
    crate::runtime::statics::install(io);
    crate::runtime::isr::install_irqs();
    loop {
        // Transport RX/TX/deadlines are ISR-driven (USART1 + SysTick, PFIC
        // HIGH). Main loop owns LED housekeeping, the deferred-reboot poll,
        // and sleep.
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        unsafe { crate::runtime::Drivers::stat_led() }.poll();

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
