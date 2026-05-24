#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use osc_core::bp;
pub use osc_core::{BaudRate, ConfigDefaults};

pub mod board;
pub mod chip_flash;
pub(crate) mod drop_oldest_ring;
pub mod hal;
pub(crate) mod idle_ring;
pub mod irq;
pub mod log;
pub mod prelude;
pub mod services;
pub(crate) mod statics;
#[cfg(feature = "defmt")]
pub mod telemetry;

use board::{BoardConfig, Ch32Board};

/// Const-asserts pin-uniqueness on the `BoardConfig` literal, then runs.
#[macro_export]
macro_rules! run {
    ($cfg:expr) => {{
        const __OSC_CH32_CFG: $crate::board::BoardConfig = $cfg;
        const _: () = __OSC_CH32_CFG.wiring.assert_valid();
        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        unsafe { ::qingke::pfic::wfi_to_wfe(false) };
        $crate::__run(__OSC_CH32_CFG)
    }};
}

#[doc(hidden)]
pub fn __run(cfg: BoardConfig) -> ! {
    let board = Ch32Board::new(cfg);
    statics::install(board);
    statics::install_irqs();
    loop {
        // SAFETY: SERVICES initialized in `install`; no ISR aliases it.
        let services = unsafe { (*statics::SERVICES.get()).assume_init_mut() };
        services.poll(&statics::SHARED);
        #[cfg(feature = "defmt")]
        telemetry::pump();
        riscv::asm::wfi();
    }
}
