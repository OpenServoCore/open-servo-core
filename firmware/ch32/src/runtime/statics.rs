use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_core::{Kernel, RegionStorageRaw, Session, Shared};

use crate::control::Ch32ControlIo;

pub static SHARED: Shared = Shared::new();

/// Initialised by `install`; the ADC DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32ControlIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// The per-servo dispatch session (HOLD-write staging). The SysTick ISR
/// borrows it to build a `Dispatcher` each time the framer hands up a
/// decoded request. `&mut` access lives on SysTick (PFIC HIGH), which shares
/// HIGH with USART1 so the two transport ISRs never preempt each other; the
/// main loop never reaches in.
pub(crate) static SESSION: SyncUnsafeCell<MaybeUninit<Session>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

pub fn install(io: Ch32ControlIo) {
    unsafe {
        (*KERNEL.get()).write(Kernel::new(io));
        (*SESSION.get()).write(Session::new());
    }
    crate::log::info!("kernel + session installed");
}

/// `read_volatile` is load-bearing: a plain read gets hoisted out of spin loops
/// because LLVM can't see the DMA TC ISR writing this asynchronously.
pub fn read_sample_tick() -> u32 {
    unsafe {
        let p = &raw const (*SHARED.table.region_ptr())
            .telemetry
            .intermediaries
            .sample_tick;
        p.read_volatile()
    }
}
