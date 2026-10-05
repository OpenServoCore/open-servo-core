//! The fake adapter: the production LinkServer + HostBus over the DES sim
//! with production servo stacks, behind the [`Pipe`] trait. Full-stack CI
//! with no hardware; time is sim time, so every engine window resolves
//! instantly and deterministically.

use osc_integration::sim::{RamStore, Sim, WireFrame};
use osc_protocol::wire::BaudRate;
use osc_servo_core::stamp;

use crate::pipe::{Pipe, PipeError};

pub use osc_integration::sim::TelSample;

/// The dev board's own facts, one real SG90's calibration dumped off it and
/// a bench MG90's identified motor: what a simulated servo needs to read
/// back like a servo that left the bench calibrated, identified and
/// stamped, instead of "not calibrated".
pub mod seed {
    use osc_integration::sim::{CalibSense, CalibSenseExt, DEV_V006_SENSE, DEV_V006_SENSE_EXT};

    /// Board data, stamped by install at every bringup (the osc-dev-v006
    /// app's `Calibration`) - RO in the table, so FACTORY never moves it.
    pub const SENSE: CalibSense = DEV_V006_SENSE;
    pub const SENSE_EXT: CalibSenseExt = DEV_V006_SENSE_EXT;

    // Table state a bench calibration wrote and SAVEd: the CALIB region and
    // the CONFIG travel limits. FACTORY wipes every one of them.
    pub const RAW_MIN: u16 = 5;
    pub const RAW_MAX: u16 = 4095;
    pub const ANGLE_MIN_CDEG: i16 = 0;
    pub const ANGLE_MAX_CDEG: i16 = 20200;
    pub const GEAR_RATIO_CENTI: u16 = 25464;
    pub const POS_MIN_PHYS_COUNTS: i32 = 5;
    pub const POS_MAX_PHYS_COUNTS: i32 = 4095;
    pub const POS_MIN_SOFT_COUNTS: i32 = 228;
    pub const POS_MAX_SOFT_COUNTS: i32 = 3872;
    pub const DRIVE_POLARITY: bool = true;
    // The identified motor (an `osc ident` fit on a 2S rail): nonzero Ke is
    // what lets the closed loops open once the set is stamped.
    pub const R_Q12: u16 = 7270;
    pub const RECIP_KE_Q: u16 = 6957;
    pub const B_I_Q313: u16 = 2427;
    pub const FRIC_FC_COUNTS: u16 = 53;
    pub const FRIC_FV_Q016: u16 = 356;
    pub const KE_VPC_Q: u16 = 603;
}

pub struct FakePipe {
    sim: Sim,
    frames: Vec<WireFrame>,
}

impl FakePipe {
    pub fn new(rate: BaudRate, servo_ids: &[u8]) -> Self {
        let mut sim = Sim::new(rate);
        sim.attach_host();
        sim.attach_link();
        for &id in servo_ids {
            // One store per servo, leaked: the sim wants `&'static`, and a
            // fake fleet lives as long as the client that owns it, so the
            // leak is bounded by the roster. Without a store the servo
            // answers SAVE and FACTORY `hardware` (protocol sec 9.4).
            sim.add_servo_with_store(id, RamStore::leak());
        }
        // SAVE/FACTORY only mean anything with the reboot behind them: the
        // sim's main loop honors the staged reset, so a wiped store boots
        // board defaults and the servo answers on its default id again.
        sim.set_self_reboot(true);
        Self {
            sim,
            frames: Vec::new(),
        }
    }

    /// Reach into the rig (seed UIDs, peek tables, read diag counters).
    pub fn sim_mut(&mut self) -> &mut Sim {
        &mut self.sim
    }

    /// Servo `i` carries the dev board's sense chain as board data
    /// ([`seed`]): install re-stamps it at every bringup, so FACTORY leaves
    /// it standing. Alone, the servo is factory-fresh (every `data_flags`
    /// reason of a never-saved, never-identified servo).
    pub fn seed_board(&mut self, i: usize) {
        self.sim.set_servo_sense(i, seed::SENSE, seed::SENSE_EXT);
    }

    /// Servo `i` comes up like one off the calibration bench ([`seed`]): the
    /// dumped CALIB, the travel limits and the identified motor are
    /// written, stamped and SAVEd, then the board data goes in. So the app
    /// reads real units from the first scan with `data_flags` clear, a
    /// reboot keeps them, and FACTORY wipes them back to board defaults -
    /// exactly what the hardware does (protocol sec 9.4/9.5).
    pub fn seed_calibrated(&mut self, i: usize) {
        self.sim.servo_table_mut(i, |t| {
            t.calib.pot.raw_min = seed::RAW_MIN;
            t.calib.pot.raw_max = seed::RAW_MAX;
            t.calib.kinematics.angle_min_cdeg = seed::ANGLE_MIN_CDEG;
            t.calib.kinematics.angle_max_cdeg = seed::ANGLE_MAX_CDEG;
            t.calib.kinematics.gear_ratio_centi = seed::GEAR_RATIO_CENTI;
            t.calib.motor.r_q12 = seed::R_Q12;
            t.calib.motor.recip_ke_q = seed::RECIP_KE_Q;
            t.calib.motor.b_i_q313 = seed::B_I_Q313;
            t.calib.motor.fric_fc_counts = seed::FRIC_FC_COUNTS;
            t.calib.motor.fric_fv_q016 = seed::FRIC_FV_Q016;
            t.calib.motor.ke_vpc_q = seed::KE_VPC_Q;
            t.config.pos_limits.pos_min_phys_counts = seed::POS_MIN_PHYS_COUNTS;
            t.config.pos_limits.pos_max_phys_counts = seed::POS_MAX_PHYS_COUNTS;
            t.config.pos_limits.pos_min_soft_counts = seed::POS_MIN_SOFT_COUNTS;
            t.config.pos_limits.pos_max_soft_counts = seed::POS_MAX_SOFT_COUNTS;
            t.config.limits.drive_polarity = seed::DRIVE_POLARITY;
            t.calib.stamp.plant_stamp = stamp::compute(t, None);
        });
        self.sim.persist_servo(i);
        // The sense swap power-cycles the servo onto the images just saved:
        // the boot verdict is the firmware's own, not a flag set here.
        self.seed_board(i);
    }

    /// Servo `i` (roster index) plays `track` back at the fast-tick rate:
    /// its bursts and live telemetry registers show the recorded rows
    /// instead of the synthetic ramp (see `Sim::set_track`).
    pub fn set_track(&mut self, i: usize, track: Vec<TelSample>) {
        self.sim.set_track(i, track);
    }

    /// The host closes and reopens the adapter: every open configures it,
    /// which starts a fresh link session (see `Sim::link_reopen`).
    pub fn reopen(&mut self) {
        self.sim.link_reopen();
    }

    /// Drain the wire frames recorded across every `send`'s sim run --
    /// deterministic timing probes for injection tests.
    pub fn take_frames(&mut self) -> Vec<WireFrame> {
        std::mem::take(&mut self.frames)
    }
}

impl Pipe for FakePipe {
    async fn send(&mut self, bytes: &[u8]) -> Result<(), PipeError> {
        self.sim.link_send(bytes);
        // Play the whole exchange out; records accumulate for recv.
        self.frames.extend(self.sim.run());
        Ok(())
    }

    async fn recv(&mut self) -> Result<Vec<u8>, PipeError> {
        let out = self.sim.link_recv();
        if out.is_empty() {
            // Every send resolves its records synchronously (sim time), so
            // an empty recv means the client awaits something that will
            // never come -- fail fast instead of pending into the guard.
            return Err(PipeError::Io("fake adapter has nothing to deliver".into()));
        }
        Ok(out)
    }

    async fn pause(&mut self, d: std::time::Duration) {
        self.sim.idle(d.as_micros() as u64);
    }
}
