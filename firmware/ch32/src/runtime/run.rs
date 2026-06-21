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
    crate::runtime::statics::install(io);
    crate::runtime::isr::install_irqs();
    loop {
        // DXL parser drain runs on the three RX triggers (DMA1_CH7 HT/TC,
        // USART1 IDLE) per `dxl-streaming-rx.md` §3 / §4.4 / §5.2 — see
        // `runtime::isr::on_dma1_ch7` / `on_usart1_idle`. Main loop owns
        // only LED housekeeping + sleep.
        // SAFETY: stat_led installed in bringup; main-loop sole accessor.
        unsafe { crate::runtime::Drivers::stat_led() }.poll();
        // M2 (#33) → M5+ regression: the stat LED's TX-activity blink moved
        // here via `legacy::dxl::tx_activity::poll` against a legacy TX
        // counter. The driver doesn't expose a TX-tick accessor yet; restore
        // when M3 (#5) lands and a driver-side counter is wired.
        riscv::asm::wfi();
    }
}
