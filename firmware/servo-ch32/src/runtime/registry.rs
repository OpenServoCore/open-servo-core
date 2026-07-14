//! Driver registry -- owns the static storage for each driver *instance*
//! and exposes typed accessors. Driver types themselves stay pure (no
//! statics, no `install`, no `get`); the registry is the only place that
//! knows which instance a driver type plays in this build.
//!
//! Adding an instance is a three-line change here (cell, install line,
//! accessor) -- the driver type stays untouched.
//!
//! Each instance has its own [`SyncUnsafeCell`], so simultaneous mutable
//! access to two different instances (e.g. main loop touching `stat_led`
//! while a transport ISR touches `bus`) doesn't pass through a shared
//! `&mut Drivers` -- no aliasing UB.

use core::cell::SyncUnsafeCell;

use osc_servo_core::{BaudRate, RegionStorage};
use osc_servo_drivers::Level;
use osc_servo_drivers::bus::ServoBus;
use osc_servo_drivers::led::Led;
use osc_servo_drivers::traits::bus::Providers;
use portable_atomic::AtomicBool;

use crate::cfg::board_wiring::BoardWiring;
use crate::providers::crc::Crc;
use crate::providers::deadline::Deadline;
use crate::providers::digital_out::DigitalOut;
use crate::providers::monotonic::Monotonic;
use crate::providers::ring::RxRing;
use crate::providers::tx_wire::TxWire;
use crate::providers::usart_baud::UsartBaud;

type StatLed = Led<DigitalOut, Monotonic>;

/// Cross-cell wire-activity latch (driver-pattern sec 9.3 channel, not a
/// driver cell): the USART1 vector's tail stores, the main-loop LED
/// policy swaps. One relaxed store per wire IRQ.
pub static BUS_ACTIVITY: AtomicBool = AtomicBool::new(false);

/// Bundle of the chip-side providers the `ServoBus` composite consumes
/// (driver-pattern sec 5.4). Each associated type maps to its zero-sized
/// provider impl (one per `providers/<role>.rs`).
pub struct V006Providers;

impl Providers for V006Providers {
    type Ring = RxRing;
    type Deadline = Deadline;
    type Crc = Crc;
    type Tx = TxWire;
    type Baud = UsartBaud;
}

type Bus = ServoBus<V006Providers>;

/// `Bus` holds raw span pointers (zero-copy TX arms, driver-pattern sec 4.2) and
/// is therefore `!Sync`. All access is serialized: `&mut` only from the HIGH
/// transport ISRs (which never preempt each other), and the main loop reaches
/// in for `take_reboot` only under a critical section. This wrapper asserts
/// that discipline so the cell can live in a `static`.
struct BusCell(SyncUnsafeCell<Option<Bus>>);
// SAFETY: see `BusCell` doc -- access is serialized by PFIC priority + CS.
unsafe impl Sync for BusCell {}

struct Cells {
    dbg: SyncUnsafeCell<Option<DigitalOut>>,
    stat_led: SyncUnsafeCell<Option<StatLed>>,
    bus: BusCell,
}

static CELLS: Cells = Cells {
    dbg: SyncUnsafeCell::new(None),
    stat_led: SyncUnsafeCell::new(None),
    bus: BusCell(SyncUnsafeCell::new(None)),
};

pub struct Drivers;

// `SAFETY:` prose in each fn's doc comment is the project convention; the
// markdown-`# Safety`-section form clippy expects would add a one-off style
// island here. Keep the convention; allow the lint.
#[allow(clippy::missing_safety_doc)]
impl Drivers {
    /// SAFETY: bringup-only, pre-IRQ; sole writer. Must be called exactly
    /// once, after `runtime::init::bring_up_bus` has configured USART1, the
    /// CH5 ring, and the SPI-CRC engine, and after the table's comms block is
    /// final (defaults seeded + saved image overlaid) -- `ServoBus::new`
    /// applies the effective baud to the live BRR.
    pub unsafe fn install(w: &BoardWiring) {
        // SAFETY: see fn doc.
        let dbg = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(dbg.is_none(), "Drivers: dbg already installed");
        *dbg = Some(DigitalOut::new(w.dbg.pin(), Level::Low));

        // SAFETY: see fn doc.
        let stat_led = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(stat_led.is_none(), "Drivers: stat_led already installed");
        *stat_led = Some(Led::new(
            DigitalOut::new(crate::cfg::chip::STAT_LED_PIN, Level::High),
            Monotonic,
        ));

        // The table is the comms authority here -- a saved image's id/baud
        // must be what the bus comes up as, not the board defaults.
        let (id, baud_idx, deadline_us) = crate::runtime::statics::SHARED.table.with(|t| {
            (
                t.config.common.id,
                t.config.common.baud_rate_idx,
                t.config.common.response_deadline_us,
            )
        });
        // Image parse gates enum UB only; a corrupt-but-CRC-valid idx falls
        // back to the always-reachable rescue floor instead of panicking.
        let baud = BaudRate::from_idx(baud_idx).unwrap_or(BaudRate::RESCUE);

        // SAFETY: see fn doc.
        let bus = unsafe { &mut *CELLS.bus.0.get() };
        debug_assert!(bus.is_none(), "Drivers: bus already installed");
        #[cfg(feature = "half-duplex")]
        let tx_wire = TxWire;
        #[cfg(not(feature = "half-duplex"))]
        let tx_wire = TxWire::new(&w.bus);
        *bus = Some(ServoBus::new(
            RxRing,
            Deadline,
            Crc,
            tx_wire,
            UsartBaud,
            id,
            baud,
            deadline_us,
        ));
    }

    /// SAFETY: bringup installs `dbg` before any ISR runs; runtime access is
    /// from the ADC ISR at PFIC LOW or main-loop with IRQs masked.
    ///
    /// Only used under `--features bench`; kept always-available so the API
    /// doesn't change with the feature.
    #[inline(always)]
    #[allow(dead_code)]
    pub unsafe fn dbg() -> &'static mut DigitalOut {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.dbg.get() };
        debug_assert!(cell.is_some(), "Drivers::dbg() before install");
        // SAFETY: bringup ensures Some before any ISR fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `stat_led` before main loop runs; runtime
    /// access is from main-loop callers only.
    #[inline(always)]
    pub unsafe fn stat_led() -> &'static mut StatLed {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.stat_led.get() };
        debug_assert!(cell.is_some(), "Drivers::stat_led() before install");
        // SAFETY: bringup ensures Some before main loop runs.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }

    /// SAFETY: bringup installs `bus` before any IRQ runs; runtime `&mut`
    /// access is from the USART1 and SysTick ISRs, both at PFIC HIGH, so
    /// same-priority no-preemption serializes the composite's interior
    /// state. The main loop reaches in only for `take_reboot`, and does so
    /// inside a critical section (see `runtime::run`).
    #[inline(always)]
    pub unsafe fn bus() -> &'static mut Bus {
        // SAFETY: see fn doc.
        let cell = unsafe { &mut *CELLS.bus.0.get() };
        debug_assert!(cell.is_some(), "Drivers::bus() before install");
        // SAFETY: bringup ensures Some before any IRQ fires.
        unsafe { cell.as_mut().unwrap_unchecked() }
    }
}
