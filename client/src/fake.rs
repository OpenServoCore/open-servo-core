//! The fake adapter: the production LinkServer + HostBus over the DES sim
//! with production servo stacks, behind the [`Pipe`] trait. Full-stack CI
//! with no hardware; time is sim time, so every engine window resolves
//! instantly and deterministically.

use osc_integration::sim::{RamStore, Sim, WireFrame};
use osc_protocol::wire::BaudRate;

use crate::pipe::{Pipe, PipeError};

pub use osc_integration::sim::TelSample;

/// The dev board's own facts, and one real SG90's calibration dumped off it:
/// what a simulated servo needs to read back like a servo that left the
/// bench calibrated, instead of "not calibrated".
pub mod seed {
    use osc_integration::sim::{CalibSense, CalibSenseExt};

    /// Board data, stamped by install at every bringup (the osc-dev-v006
    /// app's `Calibration`) - RO in the table, so FACTORY never moves it.
    pub const SENSE: CalibSense = CalibSense {
        shunt_r_mohm: 60,
        gain_milli: 15_000,
        vmotor_div_top: 6_800,
        vmotor_div_bot: 3_300,
        vdd_mv: 3_300,
        tick_hz: 20_000,
        i_window_min_ticks: 160,
        v_window_min_ticks: 160,
    };

    pub const SENSE_EXT: CalibSenseExt = CalibSenseExt {
        vbus_div_top_ohm: 15_000,
        vbus_div_bot_ohm: 10_000,
        ntc_pullup_ohm: 10_000,
        ntc_r25_ohm: 10_000,
        ntc_beta: 3950,
        vmotor_bias_nom_counts: 779,
    };

    // Table state a bench calibration wrote and SAVEd: the CALIB region and
    // the CONFIG travel limits. FACTORY wipes every one of them.
    pub const RAW_MIN: u16 = 5;
    pub const RAW_MAX: u16 = 4095;
    pub const LUT_CORR: [i16; 55] = [
        0, 0, 421, 364, 345, 323, 317, 306, 286, 272, 279, 276, 271, 262, 267, 246, 265, 261, 250,
        239, 239, 236, 233, 255, 245, 241, 233, 226, 222, 222, 223, 235, 228, 212, 226, 213, 193,
        209, 206, 194, 192, 196, 164, 157, 153, 172, 148, 123, 119, 122, 101, 95, 91, 46, 0,
    ];
    pub const ANGLE_MIN_CDEG: i16 = 0;
    pub const ANGLE_MAX_CDEG: i16 = 20200;
    pub const GEAR_RATIO_CENTI: u16 = 25464;
    pub const POS_MIN_PHYS_COUNTS: i32 = 5;
    pub const POS_MAX_PHYS_COUNTS: i32 = 4095;
    pub const POS_MIN_SOFT_COUNTS: i32 = 228;
    pub const POS_MAX_SOFT_COUNTS: i32 = 3872;
    pub const DRIVE_POLARITY: bool = true;
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

    /// Servo `i` comes up like one off the calibration bench ([`seed`]): the
    /// board's sense chain goes in as board data, and the dumped CALIB plus
    /// travel limits are written and SAVEd. So the app reads real units from
    /// the first scan, a reboot keeps them, and FACTORY wipes them back to
    /// board defaults - exactly what the hardware does (protocol sec
    /// 9.4/9.5).
    pub fn seed_calibrated(&mut self, i: usize) {
        self.sim.set_servo_sense(i, seed::SENSE, seed::SENSE_EXT);
        self.sim.servo_table_mut(i, |t| {
            t.calib.pot_lut.raw_min = seed::RAW_MIN;
            t.calib.pot_lut.raw_max = seed::RAW_MAX;
            t.calib.pot_lut.lut_corr = seed::LUT_CORR;
            t.calib.kinematics.angle_min_cdeg = seed::ANGLE_MIN_CDEG;
            t.calib.kinematics.angle_max_cdeg = seed::ANGLE_MAX_CDEG;
            t.calib.kinematics.gear_ratio_centi = seed::GEAR_RATIO_CENTI;
            t.config.pos_limits.pos_min_phys_counts = seed::POS_MIN_PHYS_COUNTS;
            t.config.pos_limits.pos_max_phys_counts = seed::POS_MAX_PHYS_COUNTS;
            t.config.pos_limits.pos_min_soft_counts = seed::POS_MIN_SOFT_COUNTS;
            t.config.pos_limits.pos_max_soft_counts = seed::POS_MAX_SOFT_COUNTS;
            t.config.limits.drive_polarity = seed::DRIVE_POLARITY;
        });
        self.sim.persist_servo(i);
    }

    /// Servo `i` (roster index) plays `track` back at the fast-tick rate:
    /// its bursts and live telemetry registers show the recorded rows
    /// instead of the synthetic ramp (see `Sim::set_track`).
    pub fn set_track(&mut self, i: usize, track: Vec<TelSample>) {
        self.sim.set_track(i, track);
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
