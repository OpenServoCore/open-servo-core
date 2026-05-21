#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use osc_core::bp;
pub use osc_core::{BaudRate, ConfigDefaults};

pub mod board;
pub mod chip_flash;
pub mod hal;
pub mod irq;
pub mod log;
pub mod prelude;
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
        $crate::__run(__OSC_CH32_CFG)
    }};
}

#[doc(hidden)]
pub fn __run(cfg: BoardConfig) -> ! {
    let board = Ch32Board::new(cfg);
    statics::install_kernel(board);
    loop {
        #[cfg(feature = "defmt")]
        telemetry::pump();
        riscv::asm::wfi();
    }
}
