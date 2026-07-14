//! Driver registry -- one cell, one accessor: the `HostBus` engine
//! instance. The link server and USB device are main-loop locals in
//! `runtime::run` (nothing else reaches them); only the bus is shared
//! between the transport ISRs and the main loop.

use core::cell::SyncUnsafeCell;

use osc_host::engine::HostBus;
use osc_host::traits::Providers;
use osc_protocol::wire::BaudRate;

use crate::providers::deadline::Deadline;
use crate::providers::edges::Edges;
use crate::providers::ring::RxRing;
use crate::providers::tx_wire::TxWire;
use crate::providers::usart_baud::UsartBaud;

/// Bundle of the chip-side providers the engine consumes (driver-pattern
/// sec 5.4).
pub struct LinkEProviders;

impl Providers for LinkEProviders {
    type Ring = RxRing;
    type Deadline = Deadline;
    type Tx = TxWire;
    type Baud = UsartBaud;
    type Edges = Edges;
}

pub type Bus = HostBus<LinkEProviders>;

/// All `&mut` access to the engine is serialized: the USART3 and SysTick
/// vectors share one PFIC preemption class (never preempt each other), and
/// the main loop reaches in only inside `critical_section::with`. This
/// wrapper asserts that discipline so the cell can live in a `static`.
struct BusCell(SyncUnsafeCell<Option<Bus>>);
// SAFETY: see `BusCell` doc -- access is serialized by PFIC priority + CS.
unsafe impl Sync for BusCell {}

static BUS: BusCell = BusCell(SyncUnsafeCell::new(None));

pub struct Drivers;

// `SAFETY:` prose in each fn's doc comment is the project convention.
#[allow(clippy::missing_safety_doc)]
impl Drivers {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Called exactly once,
    /// after USART3 and the CH3 ring are live -- the engine assumes a
    /// receiving wire from its first poll.
    pub unsafe fn install(boot_rate: BaudRate) {
        // SAFETY: see fn doc.
        let bus = unsafe { &mut *BUS.0.get() };
        debug_assert!(bus.is_none(), "Drivers: bus already installed");
        *bus = Some(HostBus::new(
            RxRing, Deadline, TxWire, UsartBaud, Edges, boot_rate,
        ));
    }

    /// SAFETY: bringup installs the bus before any IRQ unmasks; runtime
    /// `&mut` access is from the USART3/SysTick vectors (one preemption
    /// class) or the main loop inside a critical section.
    #[inline(always)]
    pub unsafe fn bus() -> &'static mut Bus {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *BUS.0.get() };
        debug_assert!(cell.is_some(), "Drivers::bus() before install");
        // SAFETY: bringup ensures Some before any IRQ fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
