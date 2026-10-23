//! The kernel's own copy of CONFIG and CALIB: the table's span of kernel
//! inputs (`SPAN_START..SPAN_END`), block for block as the table lays it
//! out, then what the kernel derives from it. The consumers take the blocks
//! as stored, so taking a configuration is one word copy of the span; the
//! window floors come from read-only board facts and are derived once, at
//! the first tick.

use super::KernelTiming;
use super::phases::loops::limits;
use crate::estimator::{NtcCfg, ThermCfg, window};
use crate::regions::calib::{CalibMotor, CalibThermal, CalibWinding, addr as calib_addr};
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

#[repr(C)]
#[derive(Copy, Clone)]
pub struct KernelConfig {
    pub pos_soft: ConfigPosSoft,
    pub loop_current: ConfigLoopCurrent,
    pub loop_velocity: ConfigLoopVelocity,
    pub loop_position: ConfigLoopPosition,
    pub limits: ConfigLimits,
    pub thermal: ConfigThermal,
    pub _rsvd_thermal_ext: [u8; 4],
    pub fusion: ConfigFusion,
    pub fault_cfg: ConfigFaultCfg,
    pub motor: CalibMotor,
    pub winding: CalibWinding,
    pub calib_thermal: CalibThermal,
    /// `recip_ke_q == 0 || ke_vpc_q == 0`: the physics belt's verdict.
    pub ke_unset: bool,
    /// The I2t leak for `thermal_tau_ms` and `i_cont`'s octave, a nibble
    /// each (`limits::pack_shifts`): the byte the bool's padding left.
    i2t_shifts: u8,
    pub i_window_min_ticks: u16,
    /// The terminal floor per decay (`window::v_floors`), indexed by
    /// `DecayMode as usize`.
    pub v_floor_ticks: [u16; 2],
    /// Smallest duty with a valid shunt window: where the OpenLoop ceiling
    /// restarts, published as `window_floor_q15`.
    pub ol_floor_q15: u16,
    /// `v_floor_ticks` as duties, published at the OpenLoop decay.
    pub v_floor_q15: [u16; 2],
    pub therm: ThermCfg,
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
    assert!(
        offset_of!(KernelConfig, _rsvd_thermal_ext) + s
            == offset_of!(ControlTable, config._rsvd_thermal_ext)
    );
    assert!(offset_of!(KernelConfig, fusion) + s == offset_of!(ControlTable, config.fusion));
    assert!(offset_of!(KernelConfig, fault_cfg) + s == offset_of!(ControlTable, config.fault_cfg));
    assert!(offset_of!(KernelConfig, motor) + s == offset_of!(ControlTable, calib.motor));
    assert!(offset_of!(KernelConfig, winding) + s == offset_of!(ControlTable, calib.winding));
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
            _rsvd_thermal_ext: [0; 4],
            fusion: ConfigFusion::new(),
            fault_cfg: ConfigFaultCfg::new(),
            motor: CalibMotor::new(),
            winding: CalibWinding::new(),
            calib_thermal: CalibThermal::new(),
            ke_unset: true,
            i2t_shifts: 0,
            i_window_min_ticks: 0,
            v_floor_ticks: [0; 2],
            ol_floor_q15: 0,
            v_floor_q15: [0; 2],
            therm: ThermCfg::default(),
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
        let [v0, v1] = window::v_floors(v_window_min_ticks, timing.v_trough_min_ticks);
        self.v_floor_ticks = [v0, v1];
        let [i, v0, v1] = [i_window_min_ticks, v0, v1]
            .map(|ticks| window::floor_duty(ticks, timing.pwm_arr, timing.recip_arr_q24));
        self.ol_floor_q15 = i;
        self.v_floor_q15 = [v0, v1];
    }

    /// The span as the table holds it, then the belt and the winding
    /// estimate's config. The caller takes it only between commits.
    pub(super) fn take(&mut self, shared: &Shared) {
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
        self.ke_unset = self.motor.recip_ke_q == 0 || self.motor.ke_vpc_q == 0;
        let th = &self.calib_thermal;
        self.therm = ThermCfg {
            rise_q016: th.th_rise_q016,
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
        self.i2t_shifts = limits::pack_shifts(
            limits::tau_shift(self.limits.thermal_tau_ms),
            limits::i_cont_log2(self.limits.i_cont_counts),
        );
    }
}

impl KernelConfig {
    /// The I2t leak shift (`limits::tau_shift`).
    pub fn tau_shift(&self) -> u8 {
        self.i2t_shifts >> 4
    }

    /// `floor(log2(i_cont))` (`limits::i_cont_log2`).
    pub fn i_cont_log2(&self) -> u8 {
        self.i2t_shifts & 0xF
    }
}

impl Default for KernelConfig {
    fn default() -> Self {
        Self::new()
    }
}
