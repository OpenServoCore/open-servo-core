use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_servo_core::{Kernel, RegionStorageRaw, Session, Shared};

use crate::control::Ch32ControlIo;

pub static SHARED: Shared = Shared::new();

/// Initialised by `install`; the ADC DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32ControlIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// The per-servo dispatch session (write staging + the pending-verdict slot).
/// Borrowed only by the HIGH transport ISRs (USART1 + SysTick), which
/// materialize it per `Dispatch` call and share PFIC HIGH -- so dispatch,
/// commit, and revert never overlap (the `HighDispatcher` invariant in
/// `runtime::isr`). The main loop never reaches in.
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
