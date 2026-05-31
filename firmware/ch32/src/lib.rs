#![no_std]
#![feature(sync_unsafe_cell)]
#![allow(unexpected_cfgs)]

// DXL Fast fire-path mode is mutually exclusive; pick one.
#[cfg(all(feature = "dxl-systick-fire", feature = "dxl-hw-fire"))]
compile_error!("`dxl-systick-fire` and `dxl-hw-fire` are mutually exclusive");
#[cfg(not(any(feature = "dxl-systick-fire", feature = "dxl-hw-fire")))]
compile_error!("one of `dxl-systick-fire` or `dxl-hw-fire` must be enabled");

pub use osc_core::bp;
pub use osc_core::{BaudRate, ConfigDefaults};

pub mod board;
pub mod chip_flash;
pub(crate) mod drop_oldest_ring;
pub(crate) mod dxl_fast;
#[cfg(feature = "dxl-hw-fire")]
pub(crate) mod dxl_hw_fire;
pub mod hal;
pub(crate) mod idle_ring;
pub mod irq;
pub mod log;
pub mod prelude;
pub mod services;
pub(crate) mod statics;
#[cfg(feature = "defmt")]
pub mod telemetry;

use board::{BoardConfig, Ch32KernelIo, Precomputed};

/// Const-asserts pin-uniqueness on the `BoardConfig` literal, then runs.
#[macro_export]
macro_rules! run {
    ($cfg:expr) => {{
        const __OSC_CH32_CFG: $crate::board::BoardConfig = $cfg;
        const __OSC_CH32_PRE: $crate::board::Precomputed =
            $crate::board::Precomputed::compute(&__OSC_CH32_CFG);
        const _: () = __OSC_CH32_CFG.wiring.assert_valid();
        // qingke-rt sets WFITOWFE=1; undo it so `wfi` wakes on pending IRQs.
        unsafe { ::qingke::pfic::wfi_to_wfe(false) };
        $crate::__run(__OSC_CH32_CFG, __OSC_CH32_PRE)
    }};
}

#[doc(hidden)]
pub fn __run(cfg: BoardConfig, pre: Precomputed) -> ! {
    let io = Ch32KernelIo::new(cfg, pre);
    statics::install(io);
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
