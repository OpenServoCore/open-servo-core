use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_core::{Kernel, Services, Shared};

use crate::control::Ch32ControlIo;
use crate::services::Ch32Bus;

pub static SHARED: Shared = Shared::new();

/// Initialised by `install`; DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32ControlIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// Initialised by `install`; `&mut` access lives on the DXL ISR family
/// (DMA1_CH7 HT/TC and USART1 IDLE — the parser-drain triggers per
/// `dxl-streaming-rx.md` §3 / §4.4 / §5.2). All DXL-side ISRs share PFIC
/// HIGH so same-priority no-preemption serializes the access; the main
/// loop never reaches in.
pub(crate) static SERVICES: SyncUnsafeCell<MaybeUninit<Services<Ch32Bus>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

pub fn install(io: Ch32ControlIo) {
    unsafe {
        (*KERNEL.get()).write(Kernel::new(io));
        (*SERVICES.get()).write(Services::new(Ch32Bus::new()));
    }
    crate::log::info!("kernel + services installed");
}

/// `read_volatile` is load-bearing: a plain read gets hoisted out of spin loops
/// because LLVM can't see the DMA TC ISR writing this asynchronously.
pub fn read_sample_tick() -> u32 {
    unsafe {
        let p = &raw const (*SHARED.table.telemetry.get()).intermediaries.sample_tick;
        p.read_volatile()
    }
}
