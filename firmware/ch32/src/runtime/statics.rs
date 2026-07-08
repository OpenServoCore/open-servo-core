use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_core::{Kernel, RegionStorageRaw, Session, Shared};

use crate::control::Ch32ControlIo;

pub static SHARED: Shared = Shared::new();

/// Initialised by `install`; the ADC DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Kernel<Ch32ControlIo>>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// The per-servo dispatch session (write staging + the pending-verdict slot).
/// Two borrowers, temporally exclusive (the `HighDispatcher` invariant in
/// `runtime::isr`): the HIGH transport ISRs materialize it per `Dispatch`
/// call on the spine's HIGH paths — reachable only while the dispatch handoff
/// slot is empty — and the LOW consumer vectors hold it across a claimed job,
/// during which the backpressured framer can reach no HIGH dispatch path.
/// The main loop never reaches in.
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
