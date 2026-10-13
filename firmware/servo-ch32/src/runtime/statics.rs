use core::cell::SyncUnsafeCell;
use core::mem::MaybeUninit;
use osc_servo_core::{Kernel, KernelTiming, Session, Shared};
use osc_servo_drivers::tel::{TelChannel, TelFeed};

use crate::control::Ch32ControlIo;
use crate::providers::crc::Crc;
use crate::providers::ring::RxRing;
use crate::providers::tx_wire::TxWire;
use crate::runtime::registry::V006Providers;

pub static SHARED: Shared = Shared::new();

/// TEL burst seam, split once in `install`: the kernel keeps the `TelFeed`
/// half (its `TelStream` sink and the TX lane while a burst streams) - fed
/// from the PFIC HIGH kernel tick, so kernel ownership keeps the `&mut`
/// single-context - and the `TelDrain` half rides in the bus composite
/// (`attach_tel`). The main loop reads only `drops`.
pub(crate) static TEL_CHANNEL: TelChannel = TelChannel::new();

type Ch32Kernel = Kernel<Ch32ControlIo, TelFeed<V006Providers>>;

/// Initialised by `install`; the ADC DMA TC IRQ is PFIC-masked until then.
pub(crate) static KERNEL: SyncUnsafeCell<MaybeUninit<Ch32Kernel>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// The per-servo dispatch session (write staging + the pending-verdict slot).
/// Borrowed only by the transport ISRs (TIM2 + SysTick), which materialize
/// it per `Dispatch` call and share the bus level - so dispatch, commit,
/// and revert never overlap (the `BusDispatcher` invariant in
/// `runtime::isr`). The main loop never reaches in.
pub(crate) static SESSION: SyncUnsafeCell<MaybeUninit<Session>> =
    SyncUnsafeCell::new(MaybeUninit::uninit());

/// Out of line: inlined, the kernel's build temporaries stay in `__run`'s
/// frame, under the painted stack, for the program's life.
#[inline(never)]
pub fn install(io: Ch32ControlIo, timing: KernelTiming) {
    let (feed, drain) = TEL_CHANNEL.split::<V006Providers>(TxWire, Crc, RxRing, &SHARED.table);
    unsafe {
        // SAFETY: pre-IRQ (install_irqs runs after) and `Drivers::install`
        // already built the bus in bringup, so the reach-in is single-context.
        crate::runtime::Drivers::bus().attach_tel(drain);
        (*KERNEL.get()).write(Kernel::with_tel(io, feed, timing));
        (*SESSION.get()).write(Session::new());
    }
    crate::log::info!("kernel + session installed");
}
