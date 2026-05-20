#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

pub use osc_core::ConfigDefaults;
pub use osc_core::bp;

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

pub fn run(cfg: BoardConfig) -> ! {
    let board = Ch32Board::new(cfg);
    statics::install_kernel(board);
    loop {
        #[cfg(feature = "defmt")]
        telemetry::pump();
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!("wfi")
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::hint::spin_loop();
    }
}
