//! A simulated servo: the real `ServoBus` + real `osc_core` dispatch over the
//! sim providers, boxed for a stable address (the TX zero-copy path holds raw
//! pointers into the control table while streaming a read reply, §4.2).

use std::cell::RefCell;
use std::rc::Rc;

use osc_core::{BaudRate, BootMode, ConfigDefaults, ControlTable, RegionStorage, Session, Shared};
use osc_drivers::bus::{LinkDiag, ServoBus};

use super::core::Core;
use super::providers::{
    BaudState, DeadlineState, Handles, RingState, SimBaud, SimCrc, SimDeadline, SimLine,
    SimProviders, SimRing, SimWire,
};
use super::store::RamStore;

/// Default identity a fresh sim servo answers PING with (model + fw).
pub const DEFAULT_MODEL: u16 = 0x1234;
pub const DEFAULT_FIRMWARE: u8 = 0x56;

pub struct SimServo {
    shared: Shared,
    session: Session,
    bus: ServoBus<SimProviders>,
}

impl SimServo {
    /// Build a servo at `idx`, seed its table, and wire the providers to the
    /// shared core + returned handles. A `store` mirrors the chip bringup:
    /// boot overlay after the default seed, and the bus comes up on the
    /// table's effective comms block (a saved id/baud is what answers).
    pub fn build(
        core: &Rc<RefCell<Core>>,
        idx: usize,
        id: u8,
        rate: BaudRate,
        skew_ppm: i32,
        response_deadline_us: u16,
        store: Option<&'static RamStore>,
    ) -> (Box<SimServo>, Handles) {
        let ring = RingState::new();
        let deadline = DeadlineState::new();

        let shared = Shared::new();
        shared.table.seed_config_defaults(&ConfigDefaults {
            id,
            baud: rate,
            response_deadline_us,
            ..Default::default()
        });
        if let Some(store) = store {
            store.boot_load(&shared.table);
            shared.seed_store(store);
        }
        // Default UID: the id repeated — distinct per servo, predictable for
        // ENUM tests; override via `seed_uid` where prefix structure matters.
        shared.seed_uid([id; 16]);
        // model/fw are read-only identity fields, seeded directly (not part of
        // ConfigDefaults) so PING has something to answer with.
        shared.table.with_mut(|t| {
            t.config.identity.model_number = DEFAULT_MODEL;
            t.config.identity.firmware_version = DEFAULT_FIRMWARE;
        });

        // The table is the comms authority (registry `Drivers::install` does
        // the same read on the chip).
        let (id, rate, response_deadline_us) = shared.table.with(|t| {
            (
                t.config.comms.id,
                t.config.comms.baud_rate_idx,
                t.config.comms.response_deadline_us,
            )
        });
        let baud = BaudState::new(rate);

        let bus = ServoBus::new(
            SimRing::new(ring.clone()),
            SimDeadline::new(core.clone(), deadline.clone(), idx, skew_ppm),
            SimCrc::new(),
            SimWire::new(core.clone(), baud.clone(), idx),
            SimBaud::new(baud.clone()),
            SimLine::new(core.clone()),
            id,
            rate,
            response_deadline_us,
        );

        let servo = Box::new(SimServo {
            shared,
            session: Session::new(),
            bus,
        });
        let handles = Handles {
            ring,
            deadline,
            baud,
        };
        (servo, handles)
    }

    pub fn on_break(&mut self) {
        // A2: the break handler resolves complete frames from ring data in
        // place, so it dispatches — build the dispatcher like on_deadline.
        let mut dispatcher = self.session.dispatcher(&self.shared);
        self.bus.on_break(&mut dispatcher);
    }

    pub fn on_deadline(&mut self) {
        let mut dispatcher = self.session.dispatcher(&self.shared);
        self.bus.on_deadline(&mut dispatcher);
    }

    pub fn on_tx_complete(&mut self) {
        self.bus.on_tx_complete();
    }

    pub fn take_reboot(&mut self) -> Option<BootMode> {
        self.bus.take_reboot()
    }

    /// Replace the ESIG-stand-in UID (pre-traffic, like the chip's bringup).
    pub fn seed_uid(&self, uid: [u8; 16]) {
        self.shared.seed_uid(uid);
    }

    pub fn uid(&self) -> [u8; 16] {
        *self.shared.uid()
    }

    pub fn diag(&self) -> LinkDiag {
        self.bus.diag()
    }

    /// The chip main loop's trim poll (§9.3) — tests model the loop by
    /// calling it between exchanges.
    pub fn poll_clock_trim(&mut self) -> Option<i8> {
        self.bus.poll_clock_trim()
    }

    pub fn with_table<R>(&self, f: impl FnOnce(&ControlTable) -> R) -> R {
        self.shared.table.with(f)
    }

    /// Chip-side table mutation (e.g. a fault ISR raising `fault_flags`) —
    /// state the wire cannot set on a read-only field.
    pub fn with_table_mut<R>(&self, f: impl FnOnce(&mut ControlTable) -> R) -> R {
        self.shared.table.with_mut(f)
    }
}
