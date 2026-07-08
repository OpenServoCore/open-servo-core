//! A simulated servo: the real `ServoBus` + real `osc_core` dispatch over the
//! sim providers, boxed for a stable address (the TX zero-copy path holds raw
//! pointers into the control table while streaming a read reply, §4.2).

use std::cell::RefCell;
use std::rc::Rc;

use osc_core::{BaudRate, BootMode, ConfigDefaults, ControlTable, RegionStorage, Session, Shared};
use osc_drivers::bus::{DispatchConsumer, Handoff, LinkDiag, ServoBus};

use super::core::Core;
use super::providers::{
    BaudState, DeadlineState, Handles, RingState, SimBaud, SimCrc, SimDeadline, SimDispatchWake,
    SimLine, SimProviders, SimRing, SimSequenceWake, SimWire,
};

/// Default identity a fresh sim servo answers PING with (model + fw).
pub const DEFAULT_MODEL: u16 = 0x1234;
pub const DEFAULT_FIRMWARE: u8 = 0x56;

pub struct SimServo {
    shared: Shared,
    session: Session,
    bus: ServoBus<SimProviders>,
    consumer: DispatchConsumer<SimRing, SimSequenceWake>,
}

impl SimServo {
    /// Build a servo at `idx`, seed its table, and wire the providers to the
    /// shared core + returned handles.
    pub fn build(
        core: &Rc<RefCell<Core>>,
        idx: usize,
        id: u8,
        rate: BaudRate,
        skew_ppm: i32,
        response_deadline_us: u16,
    ) -> (Box<SimServo>, Handles) {
        let ring = RingState::new();
        let deadline = DeadlineState::new();
        let baud = BaudState::new(rate);

        let shared = Shared::new();
        shared.table.seed_config_defaults(&ConfigDefaults {
            id,
            baud: rate,
            response_deadline_us,
            ..Default::default()
        });
        // model/fw are read-only identity fields, seeded directly (not part of
        // ConfigDefaults) so PING has something to answer with.
        shared.table.with_mut(|t| {
            t.config.identity.model_number = DEFAULT_MODEL;
            t.config.identity.firmware_version = DEFAULT_FIRMWARE;
        });

        // Leaked per servo: the handoff cell is 'static on the chip.
        let handoff: &'static Handoff = Box::leak(Box::new(Handoff::new()));
        let bus = ServoBus::new(
            SimRing::new(ring.clone()),
            SimDeadline::new(core.clone(), deadline.clone(), idx, skew_ppm),
            SimCrc::new(),
            SimWire::new(core.clone(), baud.clone(), idx),
            SimBaud::new(baud.clone()),
            SimLine::new(core.clone()),
            SimDispatchWake::new(core.clone(), idx),
            handoff,
            id,
            rate,
            response_deadline_us,
        );
        let consumer = DispatchConsumer::new(handoff, SimRing::new(ring.clone()), SimSequenceWake);

        let servo = Box::new(SimServo {
            shared,
            session: Session::new(),
            bus,
            consumer,
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

    /// LOW consumer body: process the outstanding dispatch job, if any.
    /// Returns whether a job was processed (the Sim then schedules the
    /// adoption wake after the consumer's handler cost).
    pub fn on_consumer(&mut self) -> bool {
        let mut dispatcher = self.session.dispatcher(&self.shared);
        self.consumer.process(&mut dispatcher)
    }

    pub fn take_reboot(&mut self) -> Option<BootMode> {
        self.bus.take_reboot()
    }

    pub fn diag(&self) -> LinkDiag {
        self.bus.diag()
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
