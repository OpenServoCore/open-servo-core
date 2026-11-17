//! The kernel's own copy of CONFIG and CALIB: the table's span of kernel
//! inputs (`SPAN_START..SPAN_END`), block for block as the table lays it
//! out, then what the kernel derives from it. The consumers take the blocks
//! as stored, so taking a configuration is one word copy of the span; the
//! window floors come from read-only board facts and are derived once, at
//! the first tick.

use super::KernelTiming;
use super::phases::loops::limits;
use crate::estimator::{NtcCfg, ThermCfg, bemf, window};
use crate::regions::calib::{CalibMotor, CalibThermal, addr as calib_addr};
use crate::regions::config::{
    ConfigFaultCfg, ConfigFusion, ConfigLimits, ConfigLoopCurrent, ConfigLoopPosition,
    ConfigLoopVelocity, ConfigPosSoft, ConfigThermal, addr as config_addr,
};
use crate::{ControlTable, RegionStorageRaw, Shared};

/// The kernel's inputs in the table: CONFIG from the soft limits on, CALIB
/// up to its thermal block, one run across the region boundary.
pub const SPAN_START: u16 = config_addr::pos_soft::POS_MIN_SOFT_COUNTS;
pub const SPAN_END: u16 = calib_addr::thermal::TH_RISE_Q016 + CalibThermal::CT_SIZE;
const SPAN_WORDS: usize = (SPAN_END - SPAN_START) as usize / 4;

/// The physics belt's mode bits (`KernelConfig::belt`): bit `mode as u8`.
pub mod belt {
    use crate::regions::control::Mode;

    const fn bit(mode: Mode) -> u8 {
        1 << mode as u8
    }

    /// A zero Ke: the boxcar yields nothing, the current loop decouples
    /// nothing.
    pub const KE_UNSET: u8 = bit(Mode::Velocity) | bit(Mode::Position);
    /// An unset current cap or speed ceiling: every closed mode clamps
    /// against both.
    pub const LIMITS_UNSET: u8 = bit(Mode::Current) | KE_UNSET;

    /// Whether `belt` refuses `mode`: one shift, no mode match. The
    /// experiment image runs OpenLoop alone and refuses nothing.
    #[cfg(not(feature = "experiment"))]
    #[inline(always)]
    pub fn refuses(belt: u8, mode: Mode) -> bool {
        (belt >> mode as u8) & 1 != 0
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct KernelConfig {
    pub pos_soft: ConfigPosSoft,
    pub loop_current: ConfigLoopCurrent,
    pub loop_velocity: ConfigLoopVelocity,
    pub loop_position: ConfigLoopPosition,
    pub limits: ConfigLimits,
    pub thermal: ConfigThermal,
    pub fusion: ConfigFusion,
    pub fault_cfg: ConfigFaultCfg,
    pub _rsvd_config_tail: [u8; 4],
    pub motor: CalibMotor,
    pub calib_thermal: CalibThermal,
    /// The physics belt: bit `mode as u8` set refuses that mode
    /// (`belt::refuses`). A zero Ke refuses Velocity and Position, an unset
    /// current cap or speed ceiling every closed mode.
    pub belt: u8,
    /// `limit_flags`' `UNSET` bit while a limit is unset, else 0.
    pub unset_flag: u8,
    /// The leak shifts for `thermal_tau_s` and `thermal_tau2_s`
    /// (`limits::tau_shift`).
    tau_shift: u8,
    tau2_shift: u8,
    pub i_window_min_ticks: u16,
    /// CALIB `v_window_min_ticks`, the terminal floor.
    pub v_floor_ticks: u16,
    /// Smallest duty with a valid shunt window: where the OpenLoop ceiling
    /// restarts, published as `window_floor_q15`.
    pub ol_floor_q15: u16,
    /// `v_floor_ticks` as a duty, published as `window_v_floor_q15`.
    pub v_floor_q15: u16,
    /// The take's choice of the resistance the current loop commands
    /// through on a tick with no reading, Q.12 vcounts per ccount:
    /// `motor.r_q12` as stored, or the loop's own `i_kp_q88` where the
    /// winding is unidentified, so its command still lifts over the window
    /// floor. The tick reads `Command::r_loop_q12`: an identified winding
    /// at the estimate's temperature, the stand-in as chosen here
    /// (`Medium::r_hat`).
    pub r_loop_q12: u32,
    pub therm: ThermCfg,
    /// The brush term's size at full weight, `r_q12` units
    /// (`bemf::brush_rc_q12`).
    pub brush_rc_q12: i32,
    /// |speed| c/s at and under which the brush term's weight is 0.
    pub brush_lo_cps: i32,
    /// The window term's conductance per tick of the period
    /// (`window::WindowTerm::conductance`), fenced by the winding; RAIL puts
    /// it on the rail.
    pub g_arr: u32,
}

// The copy is a word copy of the table's bytes: every block sits at its
// table offset from `SPAN_START`, and the span is whole words.
const _: () = {
    use core::mem::{offset_of, size_of};
    let s = SPAN_START as usize;
    assert!(s.is_multiple_of(4) && (SPAN_END as usize).is_multiple_of(4));
    assert!(offset_of!(KernelConfig, pos_soft) + s == offset_of!(ControlTable, config.pos_soft));
    assert!(
        offset_of!(KernelConfig, loop_current) + s == offset_of!(ControlTable, config.loop_current)
    );
    assert!(
        offset_of!(KernelConfig, loop_velocity) + s
            == offset_of!(ControlTable, config.loop_velocity)
    );
    assert!(
        offset_of!(KernelConfig, loop_position) + s
            == offset_of!(ControlTable, config.loop_position)
    );
    assert!(offset_of!(KernelConfig, limits) + s == offset_of!(ControlTable, config.limits));
    assert!(offset_of!(KernelConfig, thermal) + s == offset_of!(ControlTable, config.thermal));
    assert!(offset_of!(KernelConfig, fusion) + s == offset_of!(ControlTable, config.fusion));
    assert!(offset_of!(KernelConfig, fault_cfg) + s == offset_of!(ControlTable, config.fault_cfg));
    assert!(
        offset_of!(KernelConfig, _rsvd_config_tail) + s
            == offset_of!(ControlTable, config._rsvd_tail)
    );
    assert!(offset_of!(KernelConfig, motor) + s == offset_of!(ControlTable, calib.motor));
    assert!(offset_of!(KernelConfig, calib_thermal) + s == offset_of!(ControlTable, calib.thermal));
    assert!(offset_of!(KernelConfig, calib_thermal) + size_of::<CalibThermal>() == 4 * SPAN_WORDS);
};

impl KernelConfig {
    pub fn new() -> Self {
        Self {
            pos_soft: ConfigPosSoft::new(),
            loop_current: ConfigLoopCurrent::new(),
            loop_velocity: ConfigLoopVelocity::new(),
            loop_position: ConfigLoopPosition::new(),
            limits: ConfigLimits::new(),
            thermal: ConfigThermal::new(),
            fusion: ConfigFusion::new(),
            fault_cfg: ConfigFaultCfg::new(),
            _rsvd_config_tail: [0; 4],
            motor: CalibMotor::new(),
            calib_thermal: CalibThermal::new(),
            belt: belt::KE_UNSET | belt::LIMITS_UNSET,
            unset_flag: limits::flag::UNSET,
            tau_shift: 0,
            tau2_shift: 0,
            i_window_min_ticks: 0,
            v_floor_ticks: 0,
            ol_floor_q15: 0,
            v_floor_q15: 0,
            r_loop_q12: 0,
            therm: ThermCfg::default(),
            brush_rc_q12: 0,
            brush_lo_cps: 0,
            g_arr: 0,
        }
    }

    /// The window floors. Every input is a read-only board fact
    /// (`floor_inputs_are_read_only`), so the first tick derives them once.
    pub(super) fn load_floors(&mut self, shared: &Shared, timing: &KernelTiming) {
        let p = shared.table.region_ptr();
        // SAFETY: raw-pointer volatile reads of aligned repr(C) fields in the
        // static table, no `&T` formed (single-writer contract on `Kernel`).
        let (i_window_min_ticks, v_window_min_ticks) = unsafe {
            (
                (&raw const (*p).calib.sense.i_window_min_ticks).read_volatile(),
                (&raw const (*p).calib.sense.v_window_min_ticks).read_volatile(),
            )
        };
        self.i_window_min_ticks = i_window_min_ticks;
        self.v_floor_ticks = v_window_min_ticks;
        let [i, v] = [i_window_min_ticks, v_window_min_ticks]
            .map(|ticks| window::floor_duty(ticks, timing.pwm_arr, timing.recip_arr_q24));
        self.ol_floor_q15 = i;
        self.v_floor_q15 = v;
    }

    /// The span as the table holds it, then the belt, the two speeds, the
    /// cap, the brush term, the window term's conductance and the winding
    /// estimate's config. The caller takes it only between commits.
    pub(super) fn take(&mut self, shared: &Shared, recip_arr_q24: u32) {
        let src = shared
            .table
            .region_ptr()
            .cast::<u32>()
            .wrapping_add(SPAN_START as usize / 4);
        let dst = (self as *mut Self).cast::<u32>();
        for i in 0..SPAN_WORDS {
            // SAFETY: both runs are word-aligned and SPAN_WORDS long (the
            // const asserts above); the table holds valid values of the
            // same block types at the same offsets, so `self` stays valid;
            // volatile, as every kernel read of a transport-owned region.
            unsafe { dst.add(i).write(src.add(i).read_volatile()) };
        }
        let lp = &mut self.loop_position;
        let lim = &mut self.limits;
        let ke_unset = self.motor.recip_ke_q == 0 || self.motor.ke_vpc_q == 0;
        let limits_unset = lim.current_limit_counts == 0 || lp.speed_ceiling_cps == 0;
        self.belt = (ke_unset as u8 * belt::KE_UNSET) | (limits_unset as u8 * belt::LIMITS_UNSET);
        self.unset_flag = limits_unset as u8 * limits::flag::UNSET;
        // Resolved here, so the tick tests nothing. While a limit is unset
        // the belt refuses every closed mode at the CONTROL ahead of any
        // phase that reads these (`unset_limits_never_reach_a_closed_tick`):
        // an unset ceiling stays 0 with the move speed under it, and an
        // unset cap reads as the overcurrent trip, the board's guard, which
        // OpenLoop's limiter then holds under.
        let c = lp.speed_ceiling_cps;
        let v = lp.velocity_limit_cps;
        lp.velocity_limit_cps = if v == 0 { c } else { v.min(c) };
        if lim.current_limit_counts == 0 {
            lim.current_limit_counts = lim.oc_trip_counts;
        }
        // Q8.8 to Q.12: selected here, so the tick tests nothing
        self.r_loop_q12 = match self.motor.r_q12 {
            0 => (self.loop_current.i_kp_q88 as u32) << 4,
            r => r as u32,
        };
        let m = &self.motor;
        self.brush_rc_q12 = bemf::brush_rc_q12(m.r_brush_q12, m.brush_cps, m.r_q12);
        self.brush_lo_cps = m.brush_cps as i32 - bemf::BRUSH_SPAN_CPS;
        self.g_arr = window::WindowTerm::conductance(m.g_window_q016, m.r_q12, recip_arr_q24);
        let th = &self.calib_thermal;
        // a body node needs both its words: a tau alone is a gain of 0, a
        // gain alone would ride a one-period leak
        let tau2_s = self.thermal.thermal_tau2_s;
        self.therm = ThermCfg {
            rise_q016: th.th_rise_q016,
            rise2_q016: if tau2_s == 0 { 0 } else { th.th_rise2_q016 },
            ntc: NtcCfg {
                raw_ref: th.ntc_raw_ref,
                t_ref_cc: th.ntc_t_ref_cc,
                k1_q88: th.ntc_k1_q88,
                k2_q24: th.ntc_k2_q24,
            },
        };
        // Finish the estimate's stores before the log calls, so its
        // inputs need not spill across them. The table keeps the host's 0.
        self.limits.i_cont_counts = limits::i_cont_or_cap(&self.limits);
        self.tau_shift = limits::tau_shift(self.limits.thermal_tau_s);
        self.tau2_shift = limits::tau_shift(tau2_s);
    }
}

impl KernelConfig {
    /// The two leak shifts, rotor then body (`limits::tau_shift`).
    pub fn tau_shifts(&self) -> [u8; 2] {
        [self.tau_shift, self.tau2_shift]
    }
}

impl Default for KernelConfig {
    fn default() -> Self {
        Self::new()
    }
}
