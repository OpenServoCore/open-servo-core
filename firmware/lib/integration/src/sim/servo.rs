//! A simulated servo: the real `ServoBus` + real `osc_servo_core` dispatch over the
//! sim providers, boxed for a stable address (the TX zero-copy path holds raw
//! pointers into the control table while streaming a read reply, sec 4.2).

use std::cell::RefCell;
use std::rc::Rc;

use osc_servo_core::data_state::DataJob;
use osc_servo_core::pos_lut::POINTS;
use osc_servo_core::regions::config::{DEFAULT_V_UNDERVOLT_MV, vmotor_counts};
use osc_servo_core::tel::{TelSample, TelStream};
use osc_servo_core::{
    BaudRate, BootMode, CalibSense, CalibSenseExt, ConfigDefaults, ControlTable, CurrentDefaults,
    RegionStorage, Session, Shared,
};
use osc_servo_drivers::bus::{LinkDiag, ServoBus};
use osc_servo_drivers::tel::{TelChannel, TelFeed};

use super::core::Core;
use super::providers::{Handles, SimBaud, SimCrc, SimDeadline, SimProviders, SimRing, SimWire};
use super::store::RamStore;

/// The osc-dev-v006 sense chain the sim mirrors; per-servo board facts
/// override it through [`SimServo::set_sense`].
const SENSE: CalibSense = CalibSense {
    shunt_r_mohm: 60,
    gain_milli: 14884,
    vmotor_div_top: 6400,
    vmotor_div_bot: 1600,
    vdd_mv: 3300,
    tick_hz: 20000,
    i_window_min_ticks: 64,
    v_window_min_ticks: 160,
};

const SENSE_EXT: CalibSenseExt = CalibSenseExt {
    vbus_div_top_ohm: 6400,
    vbus_div_bot_ohm: 1600,
    ntc_pullup_ohm: 10000,
    ntc_r25_ohm: 10000,
    ntc_beta: 3950,
    vmotor_bias_nom_counts: 773,
    rail_drop_mv: 250,
};

/// Board travel the config defaults seed (the v006 pot's 12-bit span): what
/// the soft limits fall back to when a wiped store boots.
const PHYS_MAX_COUNTS: i32 = 4095;

pub struct SimServo {
    shared: Shared,
    session: Session,
    bus: ServoBus<SimProviders>,
    /// Kernel-side half of the TEL channel; the sim's fast-tick pump feeds it.
    feed: TelFeed,
    seed: Seed,
}

/// Everything bringup needs, kept so a staged reboot can run it again.
struct Seed {
    core: Rc<RefCell<Core>>,
    idx: usize,
    id: u8,
    rate: BaudRate,
    response_deadline_us: u16,
    store: Option<&'static RamStore>,
    handles: Handles,
    /// Board sense facts: install re-stamps them at every bringup, so they
    /// are the part of CALIB a FACTORY wipe cannot move.
    sense: CalibSense,
    sense_ext: CalibSenseExt,
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
        let handles = Handles::new(rate);
        handles.deadline.set_skew(core.borrow().now(), skew_ppm);
        let seed = Seed {
            core: core.clone(),
            idx,
            id,
            rate,
            response_deadline_us,
            store,
            handles: handles.clone(),
            sense: SENSE,
            sense_ext: SENSE_EXT,
        };
        let (shared, bus, feed) = Self::bringup(&seed, [id; 16]);
        let servo = Box::new(SimServo {
            shared,
            session: Session::new(),
            bus,
            feed,
            seed,
        });
        (servo, handles)
    }

    /// One power-on: a fresh table under the board seed, the store's overlay
    /// on top, and a driver whose comms block comes from what that left.
    fn bringup(seed: &Seed, uid: [u8; 16]) -> (Shared, ServoBus<SimProviders>, TelFeed) {
        seed.handles.ring.reset();

        let shared = Shared::new();
        shared.table.seed_config_defaults(
            &ConfigDefaults {
                id: seed.id,
                baud: seed.rate,
                response_deadline_us: seed.response_deadline_us,
                pos_max_phys_counts: PHYS_MAX_COUNTS,
                ..Default::default()
            },
            &CurrentDefaults::from_sense(
                seed.sense.shunt_r_mohm,
                seed.sense.gain_milli,
                seed.sense.vdd_mv,
            ),
            vmotor_counts(
                DEFAULT_V_UNDERVOLT_MV,
                seed.sense.vmotor_div_top as u32,
                seed.sense.vmotor_div_bot as u32,
                seed.sense.vdd_mv,
            ),
        );
        if let Some(store) = seed.store {
            store.boot_load(&shared);
            shared.seed_store(store);
        }
        // After boot_load, mirroring chip bringup: the calib overlay copies
        // the whole region, so RO board facts land last and win over a
        // stale saved image.
        shared.table.seed_calib_sense(&seed.sense, &seed.sense_ext);
        // Default UID: the id repeated -- distinct per servo, predictable for
        // ENUM tests; override via `seed_uid` where prefix structure matters.
        // A reboot passes the live one back in: the UID is silicon (ESIG on
        // the chip), so neither a reboot nor a FACTORY wipe moves it.
        shared.seed_uid(uid);
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

        let mut bus = ServoBus::new(
            SimRing::new(seed.handles.ring.clone()),
            SimDeadline::new(seed.core.clone(), seed.handles.deadline.clone(), seed.idx),
            SimCrc::new(),
            SimWire::new(seed.core.clone(), seed.handles.baud.clone(), seed.idx),
            SimBaud::new(seed.handles.baud.clone()),
            id,
            rate,
            response_deadline_us,
        );
        // Leaked like a shared RamStore: `split` wants the chip's 'static
        // channel; test-scoped, one per servo per bringup.
        let (feed, drain) = Box::leak(Box::new(TelChannel::new())).split();
        bus.attach_tel(drain);
        (shared, bus, feed)
    }

    /// Honor a staged reboot in place (sec 9.4/9.5): the store decides what
    /// the table comes back with, so a wiped store boots board defaults.
    /// Only ever called with the TX drained (`take_reboot` withholds until
    /// then), so no reply is streaming out of the table being replaced.
    pub fn reboot(&mut self) {
        let uid = *self.shared.uid();
        let (shared, bus, feed) = Self::bringup(&self.seed, uid);
        self.shared = shared;
        self.session = Session::new();
        self.bus = bus;
        self.feed = feed;
    }

    /// Replace the board sense facts and power-cycle onto them: install
    /// stamps them from the board at every bringup, so a servo that models a
    /// different board must model it from boot on. Pre-traffic.
    pub fn set_sense(&mut self, sense: CalibSense, sense_ext: CalibSenseExt) {
        self.seed.sense = sense;
        self.seed.sense_ext = sense_ext;
        self.reboot();
    }

    /// A bench SAVE without the wire (sec 9.4): the live CONFIG, PROFILE and
    /// CALIB regions and the position table land in the store, so the next
    /// boot overlays them back and FACTORY wipes them.
    pub fn persist(&self) {
        self.seed
            .store
            .expect("servo built with a store")
            .save_table(&self.shared);
    }

    /// sec 9.1: one chip main-loop line sample (thread-level, not a vector).
    pub fn sample_rescue(&mut self, low: bool) {
        self.bus.sample_rescue(low);
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

    /// The chip main loop's data-job poll (`data_state` module): run and
    /// publish what the last commit posted. The sim calls it between
    /// handler bodies; tests that split the run from its publish model a
    /// write landing mid-job.
    pub fn poll_data_job(&self) -> bool {
        self.shared.data_job_service()
    }

    pub fn data_job_run(&self) -> Option<DataJob> {
        self.shared.data_job_run()
    }

    pub fn data_job_publish(&self, job: DataJob) -> bool {
        self.shared.data_job_publish(job)
    }

    pub fn with_table<R>(&self, f: impl FnOnce(&ControlTable) -> R) -> R {
        self.shared.table.with(f)
    }

    /// Chip-side table mutation (e.g. a fault ISR raising `fault_flags`) --
    /// state the wire cannot set on a read-only field.
    pub fn with_table_mut<R>(&self, f: impl FnOnce(&mut ControlTable) -> R) -> R {
        self.shared.table.with_mut(f)
    }

    pub fn with_pos_lut<R>(&self, f: impl FnOnce(&[i16; POINTS]) -> R) -> R {
        self.shared.with_pos_lut(f)
    }
}
