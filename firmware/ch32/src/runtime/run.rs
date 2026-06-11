//! Top-level program entry. `run!` is the board binary's main: it validates
//! the `BoardConfig` literal at compile time, calls into bringup, installs
//! the kernel + IRQs, and enters the main loop.

use crate::cfg::{BoardConfig, Precomputed};
use crate::control::Ch32ControlIo;

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
    crate::statics::install(io);
    crate::statics::install_irqs();
    loop {
        // SAFETY: SERVICES initialized in `install`; no ISR aliases it.
        let services = unsafe { (*crate::statics::SERVICES.get()).assume_init_mut() };
        services.poll(&crate::statics::SHARED);
        crate::dxl::tx_activity::poll();
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        unsafe { crate::runtime::Drivers::stat_led() }.poll();
        riscv::asm::wfi();
    }
}
