use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_servo_core::{Kernel, KernelTiming, RegionStorageRaw, Session, Shared};
use osc_servo_drivers::tel::{TelChannel, TelFeed};

use crate::control::Ch32ControlIo;

pub static SHARED: Shared = Shared::new();

/// TEL burst seam, split once in `install`: the kernel keeps the `TelFeed`
/// half (its `TelStream` sink) -- fed from the PFIC LOW kernel tick, so
/// kernel ownership keeps the `&mut` single-context -- and the `TelDrain`
/// half rides in the HIGH-side bus composite (`attach_tel`).
static TEL_CHANNEL: TelChannel = TelChannel::new();

type Ch32Kernel = Kernel<Ch32ControlIo, TelFeed>;

/// Initialised by `install`; the ADC DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Ch32Kernel>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// The per-servo dispatch session (write staging + the pending-verdict slot).
/// Borrowed only by the HIGH transport ISRs (USART1 + SysTick), which
/// materialize it per `Dispatch` call and share PFIC HIGH -- so dispatch,
/// commit, and revert never overlap (the `HighDispatcher` invariant in
/// `runtime::isr`). The main loop never reaches in.
pub(crate) static SESSION: SyncUnsafeCell<MaybeUninit<Session>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

pub fn install(io: Ch32ControlIo, timing: KernelTiming) {
    let (feed, drain) = TEL_CHANNEL.split();
    unsafe {
        // SAFETY: pre-IRQ (install_irqs runs after) and `Drivers::install`
        // already built the bus in bringup, so the reach-in is single-context.
        crate::runtime::Drivers::bus().attach_tel(drain);
        (*KERNEL.get()).write(Kernel::with_tel(io, feed, timing));
        (*SESSION.get()).write(Session::new());
    }
    crate::log::info!("kernel + session installed");
}

/// `read_volatile` is load-bearing: a plain read gets hoisted out of spin loops
/// because LLVM can't see the DMA TC ISR writing this asynchronously.
pub fn read_sample_tick() -> u32 {
    unsafe {
        let p = &raw const (*SHARED.table.region_ptr()).telemetry.estimates.sample_tick;
        p.read_volatile()
    }
}
