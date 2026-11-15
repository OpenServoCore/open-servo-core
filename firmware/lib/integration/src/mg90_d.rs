//! The bench servo of record as a rig: mg90-d on the osc-dev-v006 board.
//! [`rig`] seeds the kernel with every field of the servo's constant set
//! of record, and [`plant`] is the [`mg90`](crate::mg90) plant on the
//! motor model those fields hold ([`motor`]), so the stored model is the
//! plant's at rest by construction. Spinning, the winding is the one the
//! bench reads, which the stored model leaves out: its inductance ([`L`])
//! and its brush contact's drop ([`CONTACT`]). The pot's residual is this
//! servo's own ([`POT_RESIDUAL`]). The rest of the plant's physics is that
//! plant's own: the gear lash and the pot's noise, on the board's window
//! floors and shunt settle bands. Both sides run on the identity position
//! table, the residual standing in for what a table leaves.
//!
//! Not carried: the set's position table, whose curve the plant's pot
//! does not have, and its ADC wide-code slots, which compensate codes the
//! plant's pot never sticks at.

use osc_servo_core::kernel::DECIM_MED;
use osc_servo_core::pos_lut::POINTS;
use osc_servo_core::{ControlTable, DecayMode, RegionStorage, Shared, StallResponse};

use crate::mg90::{Contact, Mg90, Motor, stored_r};
use crate::plant::{lut_live, seed};
use crate::sim::DEV_V006_SENSE;

/// The rail in vmotor counts as the bench servo's captures read it,
/// behind the supply's lead.
pub const VBUS: u16 = 1238;
/// The set's linearized counts per degree, its table's scale.
pub const COUNTS_PER_DEG: f64 = 20.126631678008682;
/// The set's shunt scale, the board's measured counts per A.
pub const CURRENT_COUNTS_PER_A: f64 = 1135.013429521035;

/// The bench pot's direction-free departure through the set's table, as
/// `(amplitude, period)` sine lines in linearized counts at the 21, 47,
/// 86 and 191-count band centers, each sqrt(2) x the band's rms shared by
/// both directions of travel. Not in it: the wide codes, which the
/// measurement masked, and the part that flips with the travel, which is
/// shaft motion, not the pot.
pub const POT_RESIDUAL: &[(f64, f64)] = &[(0.64, 21.0), (1.29, 47.0), (0.64, 86.0), (0.95, 191.0)];

/// Set of record 324e326794b5, every field it stores, its resistance as
/// the winding at 25.00 C, the field's meaning.
pub fn set_of_record(t: &mut ControlTable) {
    let c = &mut t.config;
    c.pos_phys.pos_min_phys_counts = 82;
    c.pos_phys.pos_max_phys_counts = 3792;
    c.pos_soft.pos_min_soft_counts = 314;
    c.pos_soft.pos_max_soft_counts = 3560;
    c.loop_current.i_kp_q88 = 289;
    c.loop_current.i_ki_q412 = 1176;
    c.loop_current.i_kaw_q412 = 2352;
    c.loop_current.duty_max_q15 = 32767;
    c.loop_velocity.v_kp_q88 = 124;
    c.loop_velocity.v_ki_q412 = 311;
    c.loop_velocity.v_kaw_q412 = 623;
    c.loop_velocity.j_ff_q88 = 862;
    c.loop_position.p_kp_q88 = 5026;
    c.loop_position.pos_deadband_counts = 4;
    c.loop_position.speed_ceiling_cps = 8051;
    c.loop_position.velocity_limit_cps = 4025;
    c.loop_position.accel_limit_q88 = 10305;
    c.limits.current_limit_counts = 568;
    c.limits.stall_response = StallResponse::Yield;
    c.limits.drive_polarity = true;
    c.limits.stall_time_ticks = 10000;
    c.limits.stall_omega_max_cps = 500;
    c.limits.i_cont_counts = 284;
    c.limits.stall_release_counts = 102;
    c.limits.stall_tau_trip_counts = 65535;
    c.limits.oc_trip_counts = 2497;
    c.limits.oc_trip_ticks = 8;
    c.limits.openloop_decay = DecayMode::Slow;
    c.limits.openloop_zero_brake = false;
    c.limits.thermal_tau_s = 2;
    c.thermal.cutoff_cc = 9000;
    c.thermal.v_undervolt_counts = 922;
    c.thermal.thermal_tau2_s = 129;
    c.fault_cfg.pos_error_time_ticks = 2000;
    c.fault_cfg.pos_error_counts = 199;
    c.fault_cfg.sensor_delta_max = 64;
    c.fault_cfg.sensor_bad_count = 4;
    c.fault_cfg.rail_fold_q8 = 1467;
    c.fusion.l1_q016 = 8640;
    c.fusion.l2_q88 = 3180;
    c.fusion.l3_q88 = 84;
    let cal = &mut t.calib;
    cal.motor.r_q12 = 3725;
    cal.motor.recip_ke_q = 13315;
    cal.motor.b_i_q313 = 2434;
    cal.motor.fric_fc_counts = 54;
    cal.motor.fric_fv_q016 = 812;
    cal.motor.ke_vpc_q = 315;
    cal.thermal.th_rise_q016 = 434;
    cal.thermal.th_rise2_q016 = 1274;
    cal.thermal.ntc_raw_ref = 2048;
    cal.thermal.ntc_t_ref_cc = 2430;
    cal.thermal.ntc_k1_q88 = 474;
    cal.thermal.ntc_k2_q24 = 19934;
    cal.motor_ext.ke_uvs_per_rad = 1159;
    cal.motor_ext.fric_breakaway_counts = 0;
    cal.pot.raw_min = 82;
    cal.pot.raw_max = 3792;
    cal.kinematics.angle_min_cdeg = 0;
    cal.kinematics.angle_max_cdeg = 18433;
    cal.kinematics.gear_ratio_centi = 30805;
    cal.sense.vdd_mv = 3300;
    cal.gear.gear_class = 2;
}

/// The motor model the set's fields hold, read back the way
/// [`mg90::rig`](crate::mg90::rig) stores a plant's, on the rig's rail:
/// the winding at the temperature the set's NTC curve reads on the plant's
/// board, 24.30 C ([`stored_r`]).
pub fn motor() -> Motor {
    let sh = Shared::new();
    sh.table.with_mut(set_of_record);
    let (m, r) = sh.table.with(|t| (t.calib.motor, stored_r(t)));
    Motor {
        vbus: VBUS as f64,
        r,
        ke: m.ke_vpc_q as f64 / 4096.0,
        b: m.b_i_q313 as f64 / 8192.0 / DECIM_MED as f64,
        fc: m.fric_fc_counts as f64,
        fv: m.fric_fv_q016 as f64 / 65536.0,
    }
}

/// The bench winding's inductance, vcounts x FAST ticks per ccount: 1.11
/// mH, what the speed loop's margin sweeps at 2500 to 3500 c/s fit, where
/// the record's 0.82 mH reads 5 sigma off at 250 Hz.
pub const L: f64 = 4.858;

/// The spinning winding's drop beyond the stored resistance, as the bench
/// reads it: 2.30 ohm of slope from 37 mA to 159 mA. Small signal at 25 to
/// 333 Hz the margin sweeps fit 2.2 to 2.4 ohm over the stored 4.18 at
/// about 90 mA, fixed from 2500 to 3500 c/s. OpenLoop legs read the same
/// slope at DC with no loop over 58 to 89 mA, at a level a passive drop
/// through zero current meets only from about 37 mA up. Where the slope
/// ends the bench's break with the small-signal resistance installed
/// places, in the sweeps' own session
/// (`the_record_rig_breaks_where_the_bench_did_with_the_small_signal_resistance`);
/// the run-ups' 0.5 ohm over 0.14 to 0.39 A, another session's, would put
/// it nearer 0.2 A. Locked it reads the stored resistance.
pub const CONTACT: Contact = Contact {
    from: 42.0,
    to: 180.0,
    slope: 0.5033,
};

/// The plant at rest at `lin`, on the set's motor model with the bench
/// winding's inductance and contact, and the bench pot's residual.
pub fn plant(lin: f64, seed: u64) -> Mg90 {
    let m = motor();
    let mut p = Mg90::new(lin, [0; POINTS], seed).with_motor(m);
    p.residual = POT_RESIDUAL;
    p.tau_e = L / m.r;
    p.contact = Some(CONTACT);
    p
}

/// The set of record on the board's shunt and terminal window floors, the
/// identity table live, then `cfg`. Run it on [`mg90::kernel`](crate::mg90::kernel).
pub fn rig(cfg: impl FnOnce(&mut ControlTable)) -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        set_of_record(t);
        t.calib.sense.i_window_min_ticks = DEV_V006_SENSE.i_window_min_ticks;
        t.calib.sense.v_window_min_ticks = DEV_V006_SENSE.v_window_min_ticks;
    });
    sh.table.with_mut(cfg);
    lut_live(&sh, &[0; POINTS]);
    sh
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mg90::tests::motor_model_is_the_plants;

    /// The record rig's motor model is its plant's, under the same pin as
    /// the generic rig's.
    #[test]
    fn the_record_rigs_motor_model_is_the_plants() {
        motor_model_is_the_plants(&rig(|_| {}), || plant(2048.0, 1));
    }
}
