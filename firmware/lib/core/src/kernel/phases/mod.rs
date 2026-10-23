//! The kernel's phases. The fast half (`fast`) runs every tick; the medium
//! half (`Medium`) reads CONTROL, runs the edges, the estimators, the outer
//! loops, the limits, the detectors and the slow block, publishes, and
//! leaves the fast half its `Command`. The medium work is split into phases
//! (`phase`), one stage of the chain each, one file each, dispatched from
//! the schedule's table; every value is
//! written by the phase that computes it and read by the later phases and
//! by the fast half.

mod control;
pub(in crate::kernel) mod fast;
mod limits;
pub mod loops;
mod observer;
mod publish;
mod rail;
mod slow;
mod trajectory;
mod velocity;

pub(in crate::kernel) use control::Control;

use self::loops::faults::{Detectors, DriveCheck};
use self::loops::limits::{IBand, LimitState};
use self::loops::trajectory::TrajGen;
use self::loops::velocity::VelocityLoop;
use super::config::KernelConfig;
use super::{DECIM_SLOW, KernelTiming};
use crate::Shared;
use crate::estimator::{FusionObs, OmegaSwitch, VbusEst, WindingTherm};
use crate::pos_lut;
use crate::regions::control::Mode;

/// The linearized pot (pos_lut module), Q4 counts: the one measurement
/// behind both seeds and the observer, so theta_hat and everything that
/// reads it (trajectory, position loop, soft limits, stall gates) are in
/// linearized counts. The raw pot stays for the sensors publish, TEL `pos`
/// and the glitch screen.
pub(in crate::kernel) fn pos_q4(shared: &Shared, lut_live: bool, raw: u16) -> u16 {
    if lut_live {
        shared.pos_lut_q4(raw)
    } else {
        pos_lut::identity_q4(raw)
    }
}

pub(in crate::kernel) struct Medium {
    /// `KernelTiming::dt_med_q32`.
    dt_med_q32: u32,
    /// `KernelTiming::recip_arr_q24`.
    recip_arr_q24: u32,
    /// Medium periods toward the next SLOW run.
    pub(in crate::kernel) slow_periods: u8,
    /// CONTROL as the CONTROL phase read it.
    ctl: Control,
    /// Torque on and no fault at the CONTROL phase: the loops run.
    run: bool,
    pub(in crate::kernel) traj: TrajGen,
    pub(in crate::kernel) fusion: FusionObs,
    /// The observer phase's picks, for the later phases.
    pub(in crate::kernel) omega_hat: i32,
    omega_bemf: Option<i32>,
    /// Linearized counts per raw count at the observer's sample.
    band_gain_q4: u16,
    vel: VelocityLoop,
    limits: LimitState,
    pub(in crate::kernel) i_band: IBand,
    limit_flags: u8,
    pub(in crate::kernel) vbus: VbusEst,
    pub(in crate::kernel) thermal: WindingTherm,
    /// Velocity-loop feedback pick: back-EMF boxcar or the observer's omega.
    pub(in crate::kernel) omega_sw: OmegaSwitch,
    /// The duty OpenLoop applies while its ceiling sits under the window
    /// floor.
    ol_base_q15: u16,
    /// SLOW ticks left on the stall permit lease, renewed when HIGH moves
    /// `Shared::permit_gen` past `permit_gen`.
    permit_ticks: u8,
    permit_gen: u8,
    det: Detectors,
    pub(in crate::kernel) drive: DriveCheck,
    te_prev: bool,
    run_prev: bool,
    mode_prev: Mode,
    /// The current command the fast half's loop tracks.
    pub(in crate::kernel) i_ref_cc: i32,
    /// Position loop output, held for the velocity step.
    pub(in crate::kernel) omega_ref_q16: i32,
    /// Anti-hunt hold from the position loop; the drive parks on it.
    pub(in crate::kernel) hold: bool,
}

impl Medium {
    pub fn new(timing: &KernelTiming) -> Self {
        Self {
            dt_med_q32: timing.dt_med_q32,
            recip_arr_q24: timing.recip_arr_q24,
            // primed so the FIRST period runs the slow block: the winding
            // estimate and the derate exist before any consumer sees them
            slow_periods: DECIM_SLOW - 1,
            ctl: Control::default(),
            run: false,
            traj: TrajGen::new(),
            fusion: FusionObs::new(),
            omega_hat: 0,
            omega_bemf: None,
            band_gain_q4: pos_lut::GRID as u16,
            vel: VelocityLoop::new(),
            limits: LimitState::new(),
            i_band: IBand { lo: 0, hi: 0 },
            limit_flags: 0,
            vbus: VbusEst::new(timing.vbus_scale_q15),
            thermal: WindingTherm::new(),
            omega_sw: OmegaSwitch::new(),
            ol_base_q15: 0,
            permit_ticks: 0,
            permit_gen: 0,
            det: Detectors::new(),
            drive: DriveCheck::new(),
            te_prev: false,
            run_prev: false,
            mode_prev: Mode::OpenLoop,
            i_ref_cc: 0,
            omega_ref_q16: 0,
            hold: false,
        }
    }

    /// First tick: the observer starts at the measurement, the winding
    /// estimate at the NTC ahead of the first SLOW tick.
    pub fn seed(&mut self, pos_q4: u16, ntc_raw: u16, cfg: &KernelConfig) {
        self.fusion.seed(pos_q4);
        self.thermal.update(ntc_raw, [0; 2], &cfg.therm);
    }
}
