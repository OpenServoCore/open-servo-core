//! The fake adapter: the production LinkServer + HostBus over the DES sim
//! with production servo stacks, behind the [`Pipe`] trait. Full-stack CI
//! with no hardware; time is sim time, so every engine window resolves
//! instantly and deterministically.

use osc_integration::sim::{RamStore, Sim, WireFrame};
use osc_protocol::wire::BaudRate;
use osc_servo_core::estimator::thermal::{self, NtcCfg, UNSET_CC, flag};
use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW, LimitCfg, LimitState};
use osc_servo_core::{ControlTable, stamp};

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
    // The winding thermometer set `osc ident` writes for the MG90 family on
    // this board (ident `thermometer::Thermal::new`: tau 128 s, 63 C/W over
    // the board NTC, a 280-count hold, the 10K/10K/3950 NTC divider).
    pub const TH_ALPHA_Q24: u16 = 2097;
    pub const TH_G_Q016: u16 = 1500;
    pub const TH_MU_Q016: u16 = 7670;
    pub const NTC_RAW_REF: u16 = 2048;
    pub const NTC_T_REF_CC: i16 = 2500;
    pub const NTC_K1_Q88: i16 = 563;
    pub const NTC_K2_Q24: u16 = 5779;
    /// The NTC divider at 28 C (beta curve), where a trackless servo rests.
    pub const NTC_RAW_ROOM: u16 = 1913;
}

/// A fake servo's winding thermometer.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Therm {
    /// The ident set is stamped: a one-node model of the excess over the
    /// NTC, driven by the track's mean square current.
    #[default]
    On,
    /// No thermal block: `t_winding_cc` reads the sentinel.
    Off,
    /// The ident set is stamped and the winding is pinned midway from the
    /// derate start to the cutoff.
    Derating,
}

/// The fake's stand-in for the kernel thermometer (the sim runs no kernel).
#[derive(Default)]
struct Winding {
    excess_cc: f64,
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
        let r0 = t.calib.winding.r0_q12;
        let raw = match t.telemetry.sensors.ntc_raw {
            0 => seed::NTC_RAW_ROOM,
            r => r,
        };
        let ntc = NtcCfg {
            raw_ref: th.ntc_raw_ref,
            t_ref_cc: th.ntc_t_ref_cc,
            k1_q88: th.ntc_k1_q88,
            k2_q24: th.ntc_k2_q24,
        };
        let lim = LimitCfg {
            current_limit_counts: t.config.limits.current_limit_counts,
            derate_start_cc: t.config.thermal.derate_start_cc,
            cutoff_cc: t.config.thermal.cutoff_cc,
            ..Default::default()
        };
        let dt_s = now_us.saturating_sub(self.at_us) as f64 * 1e-6;
        self.at_us = now_us;
        let t_ntc = thermal::ntc_cc(raw, &ntc);
        let (t_cc, flags) = match t_ntc {
            Some(base) if th.th_alpha_q24 != 0 && th.th_g_q016 != 0 && r0 != 0 => {
                let slow_hz = t.calib.sense.tick_hz as f64 / (DECIM_MED as f64 * DECIM_SLOW as f64);
                let tau_s = (1u32 << 24) as f64 / (th.th_alpha_q24 as f64 * slow_hz);
                let steady = th.th_g_q016 as f64 / 65536.0 * self.i_sq_mean * r0 as f64 / 4096.0;
                self.excess_cc += (steady - self.excess_cc) * -(-dt_s / tau_s).exp_m1();
                let t_cc = if self.forced {
                    (lim.derate_start_cc as i32 + lim.cutoff_cc as i32) / 2
                } else {
                    base as i32 + self.excess_cc as i32
                };
                (t_cc.clamp(i16::MIN as i32 + 1, i16::MAX as i32) as i16, 0)
            }
            _ => (UNSET_CC, flag::UNSET),
        };
        let mut limits = LimitState::new();
        limits.update_derate(t_cc, &lim);
        limits.fold(false, 0, 0, 0, false, 0, &lim);
        t.telemetry.sensors.ntc_raw = raw;
        t.telemetry.estimates.t_winding_cc = t_cc;
        t.telemetry.estimates.i_lim_counts = limits.i_lim_counts();
        t.telemetry.therm.t_ntc_cc = t_ntc.unwrap_or(0);
        t.telemetry.therm.therm_flags = flags;
    }
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

    /// [`Self::seed_calibrated`] with the winding thermometer as `therm`.
    pub fn seed_calibrated_with(&mut self, i: usize, therm: Therm) {
        self.windings[i].forced = therm == Therm::Derating;
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
            if therm != Therm::Off {
                let th = &mut t.calib.thermal;
                th.th_alpha_q24 = seed::TH_ALPHA_Q24;
                th.th_g_q016 = seed::TH_G_Q016;
                th.th_mu_q016 = seed::TH_MU_Q016;
                th.ntc_raw_ref = seed::NTC_RAW_REF;
                th.ntc_t_ref_cc = seed::NTC_T_REF_CC;
                th.ntc_k1_q88 = seed::NTC_K1_Q88;
                th.ntc_k2_q24 = seed::NTC_K2_Q24;
                t.calib.winding.r0_q12 = seed::R_Q12;
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
            .map(|s| (s.current as f64).powi(2))
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
