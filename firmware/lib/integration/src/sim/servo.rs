//! A simulated servo: the real `ServoBus` + real `osc_servo_core` dispatch over the
//! sim providers, boxed for a stable address (the TX zero-copy path holds raw
//! pointers into the control table while streaming a read reply, sec 4.2).

use std::cell::RefCell;
use std::rc::Rc;

use osc_servo_core::tel::{TelSample, TelStream};
use osc_servo_core::{
    BaudRate, BootMode, CalibSense, CalibSenseExt, ConfigDefaults, ControlTable, RegionStorage,
    Session, Shared,
};
use osc_servo_drivers::bus::{LinkDiag, ServoBus};
use osc_servo_drivers::tel::{TelChannel, TelFeed};

use super::core::Core;
use super::providers::{
    BaudState, DeadlineState, Handles, RingState, SimBaud, SimCrc, SimDeadline, SimProviders,
    SimRing, SimWire,
};
use super::store::RamStore;

pub struct SimServo {
    shared: Shared,
    session: Session,
    bus: ServoBus<SimProviders>,
    /// Kernel-side half of the TEL channel; the sim's fast-tick pump feeds it.
    feed: TelFeed,
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
        // After boot_load, mirroring chip bringup: the calib overlay copies
        // the whole region, so RO board facts land last and win over a
        // stale saved image.
        shared.table.seed_calib_sense(
            &CalibSense {
                shunt_r_mohm: 33,
                gain_milli: 15000,
                vmotor_div_top: 10000,
                vmotor_div_bot: 10000,
                vdd_mv: 3300,
                tick_hz: 20000,
                i_window_min_ticks: 240,
                v_window_min_ticks: 300,
            },
            &CalibSenseExt {
                vbus_div_top_ohm: 20000,
                vbus_div_bot_ohm: 10000,
                ntc_pullup_ohm: 10000,
                ntc_r25_ohm: 10000,
                ntc_beta: 3950,
                vmotor_bias_nom_counts: 773,
            },
        );
        // Default UID: the id repeated -- distinct per servo, predictable for
        // ENUM tests; override via `seed_uid` where prefix structure matters.
        shared.seed_uid([id; 16]);
        // Real identity: the sim mirrors the v006 servo's table ABI, so it
        // seeds the registry model (not part of ConfigDefaults, same as the
        // chip) - keeps descriptor-keyed clients testable against the sim.
        shared
            .table
            .seed_identity(osc_protocol::models::MODEL_OSC_SERVO, 1);

        // The table is the comms authority (registry `Drivers::install` does
        // the same read on the chip).
        let (id, rate_idx, response_deadline_us) = shared.table.with(|t| {
            (
                t.config.common.id,
                t.config.common.baud_rate_idx,
                t.config.common.response_deadline_us,
            )
        });
        let rate = BaudRate::from_idx(rate_idx).expect("seeded baud idx");
        let baud = BaudState::new(rate);

        let mut bus = ServoBus::new(
            SimRing::new(ring.clone()),
            SimDeadline::new(core.clone(), deadline.clone(), idx, skew_ppm),
            SimCrc::new(),
            SimWire::new(core.clone(), baud.clone(), idx),
            SimBaud::new(baud.clone()),
            id,
            rate,
            response_deadline_us,
        );
        // Leaked like a shared RamStore: `split` wants the chip's 'static
        // channel; test-scoped, one per servo per Sim.
        let (feed, drain) = Box::leak(Box::new(TelChannel::new())).split();
        bus.attach_tel(drain);

        let servo = Box::new(SimServo {
            shared,
            session: Session::new(),
            bus,
            feed,
        });
        let handles = Handles {
            ring,
            deadline,
            baud,
        };
        (servo, handles)
    }

    /// sec 9.1: the chip main-loop sampler's declaration (thread-level, not a
    /// vector -- the sim delivers it directly at the modeled threshold tick).
    pub fn on_rescue(&mut self) {
        self.bus.on_rescue_break();
    }

    pub fn on_break(&mut self) {
        // Position from the stream: the break handler resolves complete frames
        // from ring data in place, so it dispatches -- build the dispatcher
        // like on_deadline.
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

    /// The chip main loop's trim poll (sec 9.3) -- tests model the loop by
    /// calling it between exchanges.
    pub fn poll_clock_trim(&mut self) -> Option<i8> {
        self.bus.poll_clock_trim()
    }

    /// The kernel fast tick's TEL gate (`Kernel::on_tick` checks it before
    /// building a sample) -- the sim's tick pump runs only while it holds.
    pub fn tel_active(&self) -> bool {
        self.feed.active()
    }

    /// One fast-tick sample into the kernel-side encoder.
    pub fn tel_tick(&mut self, s: &TelSample) {
        self.feed.on_tick(s);
    }

    /// The chip main loop's TEL poll (ISRs masked there); the sim calls it
    /// between handler bodies.
    pub fn poll_tel(&mut self) {
        self.bus.poll_tel();
    }

    pub fn with_table<R>(&self, f: impl FnOnce(&ControlTable) -> R) -> R {
        self.shared.table.with(f)
    }

    /// Chip-side table mutation (e.g. a fault ISR raising `fault_flags`) --
    /// state the wire cannot set on a read-only field.
    pub fn with_table_mut<R>(&self, f: impl FnOnce(&mut ControlTable) -> R) -> R {
        self.shared.table.with_mut(f)
    }
}
