//! The fake adapter: the production LinkServer + HostBus over the DES sim
//! with production servo stacks, behind the [`Pipe`] trait. Full-stack CI
//! with no hardware; time is sim time, so every engine window resolves
//! instantly and deterministically.

use osc_integration::sim::{RamStore, Sim, WireFrame};
use osc_protocol::wire::BaudRate;
use osc_servo_core::estimator::thermal::{self, NtcCfg, UNSET_CC, flag};
use osc_servo_core::kernel::phases::loops::limits::{
    FoldIn, LimitState, SLOW_PERIOD_MS, i_cont_or_cap, tau_shift,
};
use osc_servo_core::{ControlTable, stamp};

use crate::pipe::{Pipe, PipeError};

pub use osc_integration::plant::duty_counts;
pub use osc_integration::sim::{TelSample, tel_sample};
pub use osc_servo_core::TelemetryTick;
pub use osc_servo_core::regions::telemetry::tick_flag;

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
    /// The calibrated servo is an SG90: a plastic train.
    pub const GEAR_CLASS: u8 = 1;
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
    // The winding estimate's rise for the bench MG90 on this board, and
    // the 10K/10K/3950 NTC divider's curve.
    pub const TH_RISE_Q016: u16 = 1044;
    pub const NTC_RAW_REF: u16 = 2048;
    pub const NTC_T_REF_CC: i16 = 2500;
    pub const NTC_K1_Q88: i16 = 563;
    pub const NTC_K2_Q24: u16 = 5779;
    /// The NTC divider at 28 C (beta curve), where a trackless servo rests.
    pub const NTC_RAW_ROOM: u16 = 1913;
}

/// A fake servo's winding estimate.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Therm {
    /// The rise is stamped: the I2t heat of the track's mean square
    /// current over the NTC.
    #[default]
    On,
    /// No thermal block: `t_winding_cc` reads the sentinel.
    Off,
    /// The rise is stamped and the winding is pinned midway from the
    /// derate start to the cutoff.
    Derating,
}

/// The fake's stand-in for the kernel winding estimate (the sim runs no
/// kernel).
#[derive(Default)]
struct Winding {
    heat: f64,
    i_sq_mean: f64,
    forced: bool,
    at_us: u64,
}

impl Winding {
    /// Advance to `now_us` and publish what the kernel would: UNSET under
    /// the firmware's own predicate, else NTC plus the excess; `i_lim` the
    /// limit fold's electrical ceiling.
    fn publish(&mut self, t: &mut ControlTable, now_us: u64) {
        let th = t.calib.thermal;
        let raw = match t.telemetry.tick.ntc_raw {
            0 => seed::NTC_RAW_ROOM,
            r => r,
        };
        let ntc = NtcCfg {
            raw_ref: th.ntc_raw_ref,
            t_ref_cc: th.ntc_t_ref_cc,
            k1_q88: th.ntc_k1_q88,
            k2_q24: th.ntc_k2_q24,
        };
        let (lim, therm) = (t.config.limits, t.config.thermal);
        let dt_s = now_us.saturating_sub(self.at_us) as f64 * 1e-6;
        self.at_us = now_us;
        let t_ntc = thermal::ntc_cc(raw, &ntc);
        let (t_cc, flags) = match t_ntc {
            Some(base) if th.th_rise_q016 != 0 => {
                let tau_s = heat_tau_s(lim.thermal_tau_ms);
                self.heat += (self.i_sq_mean - self.heat) * -(-dt_s / tau_s).exp_m1();
                let t_cc = if self.forced {
                    (therm.derate_start_cc as i32 + therm.cutoff_cc as i32) / 2
                } else {
                    base as i32 + thermal::rise_cc(self.heat as u32, th.th_rise_q016)
                };
                (t_cc.clamp(i16::MIN as i32 + 1, i16::MAX as i32) as i16, 0)
            }
            _ => (UNSET_CC, flag::UNSET),
        };
        let mut limits = LimitState::new();
        let mut lim = lim;
        lim.i_cont_counts = i_cont_or_cap(&lim);
        limits.update_derate(t_cc, &lim, &therm);
        let fold = limits.fold(&FoldIn::default(), &lim, &t.config.pos_soft);
        t.telemetry.tick.ntc_raw = raw;
        t.telemetry.estimates.t_winding_cc = t_cc;
        t.telemetry.estimates.i_lim_counts = fold.band.lim();
        t.telemetry_ext.therm.t_ntc_cc = t_ntc.unwrap_or(0);
        t.telemetry_ext.therm.therm_flags = flags;
    }
}

/// The kernel heat EWMA's time constant, seconds: `2^tau_shift` SLOW
/// periods.
pub fn heat_tau_s(thermal_tau_ms: u16) -> f64 {
    (1u32 << tau_shift(thermal_tau_ms)) as f64 * SLOW_PERIOD_MS as f64 / 1000.0
}

pub struct FakePipe {
    sim: Sim,
    frames: Vec<WireFrame>,
    windings: Vec<Winding>,
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
            windings: servo_ids.iter().map(|_| Winding::default()).collect(),
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
        self.publish_therm();
    }

    /// Servo `i` comes up like one off the calibration bench ([`seed`]): the
    /// dumped CALIB, the travel limits and the identified motor are
    /// written, stamped and SAVEd, then the board data goes in. So the app
    /// reads real units from the first scan with `data_flags` clear, a
    /// reboot keeps them, and FACTORY wipes them back to board defaults -
    /// exactly what the hardware does (protocol sec 9.4/9.5).
    pub fn seed_calibrated(&mut self, i: usize) {
        self.seed_calibrated_with(i, Therm::On);
    }

    /// [`Self::seed_calibrated`] with the winding estimate as `therm`.
    pub fn seed_calibrated_with(&mut self, i: usize, therm: Therm) {
        self.windings[i].forced = therm == Therm::Derating;
        self.sim.servo_table_mut(i, |t| {
            t.calib.pot.raw_min = seed::RAW_MIN;
            t.calib.pot.raw_max = seed::RAW_MAX;
            t.calib.kinematics.angle_min_cdeg = seed::ANGLE_MIN_CDEG;
            t.calib.kinematics.angle_max_cdeg = seed::ANGLE_MAX_CDEG;
            t.calib.kinematics.gear_ratio_centi = seed::GEAR_RATIO_CENTI;
            t.calib.gear.gear_class = seed::GEAR_CLASS;
            t.calib.motor.r_q12 = seed::R_Q12;
            t.calib.motor.recip_ke_q = seed::RECIP_KE_Q;
            t.calib.motor.b_i_q313 = seed::B_I_Q313;
            t.calib.motor.fric_fc_counts = seed::FRIC_FC_COUNTS;
            t.calib.motor.fric_fv_q016 = seed::FRIC_FV_Q016;
            t.calib.motor.ke_vpc_q = seed::KE_VPC_Q;
            t.config.pos_phys.pos_min_phys_counts = seed::POS_MIN_PHYS_COUNTS;
            t.config.pos_phys.pos_max_phys_counts = seed::POS_MAX_PHYS_COUNTS;
            t.config.pos_soft.pos_min_soft_counts = seed::POS_MIN_SOFT_COUNTS;
            t.config.pos_soft.pos_max_soft_counts = seed::POS_MAX_SOFT_COUNTS;
            t.config.limits.drive_polarity = seed::DRIVE_POLARITY;
            if therm != Therm::Off {
                let th = &mut t.calib.thermal;
                th.th_rise_q016 = seed::TH_RISE_Q016;
                th.ntc_raw_ref = seed::NTC_RAW_REF;
                th.ntc_t_ref_cc = seed::NTC_T_REF_CC;
                th.ntc_k1_q88 = seed::NTC_K1_Q88;
                th.ntc_k2_q24 = seed::NTC_K2_Q24;
            }
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
        let n = track.len().max(1) as f64;
        self.windings[i].i_sq_mean = track
            .iter()
            .map(|s| (s.tick.i_tick_counts as f64).powi(2))
            .sum::<f64>()
            / n;
        self.sim.set_track(i, track);
        self.publish_therm();
    }

    fn publish_therm(&mut self) {
        let now = self.sim.now_us();
        for (i, w) in self.windings.iter_mut().enumerate() {
            self.sim.servo_table_mut(i, |t| w.publish(t, now));
        }
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
        self.publish_therm();
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
        self.publish_therm();
    }
}
