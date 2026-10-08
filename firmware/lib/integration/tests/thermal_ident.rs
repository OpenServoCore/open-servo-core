//! `osc ident thermal`'s fit (osc-ident `thermal`) on the real kernel: the
//! MG90-scale R-L plant seated at its low stop under the 280-count limit,
//! its winding a one-node thermal plant whose copper R follows its own
//! temperature. The kernel's thermometer reads the hold, the fit reads the
//! telemetry the CLI polls once per SLOW tick, and it lands on the
//! plant's constants, not on the table's carry.

use osc_ident::frame::TelemetrySnapshot;
use osc_ident::thermal::{self, Row};
use osc_ident::thermometer;
use osc_integration::plant::{FakeIo, RL_R_Q12, RlPlant, TIMING, duty_of, kernel, seed, stamp};
use osc_servo_core::kernel::{DECIM_MED, DECIM_SLOW};
use osc_servo_core::regions::control::addr::lifecycle::STALL_PERMIT;
use osc_servo_core::{Kernel, Mode, RegionStorage, Shared};

const LIM: u16 = 280;
/// 160 of ARR 1200: the window floor is 13.3% duty.
const I_FLOOR_TICKS: u16 = 160;
/// The low stop the hold seats against, as the anchor's does.
const STOP: u16 = 200;
/// The stall-safe cap the anchor holds at: 30%, a stall over the limit on
/// the rig's 1.775 vcounts/ccount, so the limit governs.
const HOLD_Q15: i16 = -9830;
const HOLD_S: f64 = 480.0;
const SLOW_TICKS: u32 = DECIM_MED as u32 * DECIM_SLOW as u32;
/// The host rewrites a held permit about twice per lease.
const PERMIT_EVERY_TICKS: u32 = 10_000;

/// The plant's winding: tau 96 s, steady excess 1200 / 2^16 centi-C per
/// vcount-ccount, off the table's MG90 family (128 s, 1500), so a fit
/// that read the carry back would land on the table, not the plant.
const PLANT_TAU_S: f64 = 96.0;
const PLANT_G_CC: f64 = 1200.0 / 65536.0;
const COPPER_ZERO_C: f64 = 234.5;
const AMBIENT_C: f64 = 25.0;

/// A contact factor on the winding R from `from_s`.
type Contact = &'static [(f64, f64)];

/// The torque-limit rig's MG90 observer and motor model, the family's
/// thermometer, the NTC at 25.00 C, the floor at 7/8 of the limit.
fn rig() -> Shared {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.calib.sense.i_window_min_ticks = I_FLOOR_TICKS;
        t.calib.sense.v_window_min_ticks = I_FLOOR_TICKS;
        t.calib.motor.r_q12 = RL_R_Q12;
        t.config.limits.current_limit_counts = LIM;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
        t.config.limits.openloop_zero_brake = false;
        t.config.fusion.l1_q016 = 8640;
        t.config.fusion.l2_q88 = 3180;
        t.config.fusion.l3_q88 = 84;
        t.calib.motor.b_i_q313 = 2427;
        t.calib.motor.fric_fc_counts = 53;
        t.calib.motor.ke_vpc_q = 603;
        t.calib.motor.recip_ke_q = 6957;
        t.calib.thermal.th_alpha_q24 = 2097;
        t.calib.thermal.th_g_q016 = 1500;
        t.calib.thermal.th_mu_q016 = 7670;
        t.calib.thermal.ntc_raw_ref = 2048;
        t.calib.thermal.ntc_t_ref_cc = 2500;
        t.calib.thermal.ntc_k1_q88 = 563;
        t.calib.thermal.ntc_k2_q24 = 5800;
        t.calib.winding.r0_q12 = RL_R_Q12;
        t.config.thermal.rtherm_i_min_counts = thermometer::floor_counts(LIM as f64).raw;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.ident.ident_agg = true;
    });
    stamp(&sh);
    sh
}

fn write_permit(sh: &Shared) {
    sh.table
        .with_mut(|t| t.control.lifecycle.stall_permit = true);
    sh.permit_after_commit(STALL_PERMIT, 1);
}

/// What the CLI's poll reads, once per SLOW tick.
fn read(sh: &Shared, tick: u32) -> Row {
    let o = sh.table.with(|t| TelemetrySnapshot {
        host_ms: tick as f64 * 1000.0 / TIMING.tick_hz as f64,
        t_winding_cc: t.telemetry.estimates.t_winding_cc,
        t_ntc_cc: t.telemetry.therm.t_ntc_cc,
        therm_flags: t.telemetry.therm.therm_flags,
        i_mean_counts: t.telemetry.ident.i_mean_counts,
        vdiff_mean: t.telemetry.ident.vdiff_mean,
        duty_mean_q15: t.telemetry.ident.duty_mean_q15,
        ..Default::default()
    });
    Row::from_snapshot(&o)
}

/// The seated hold at the limit with the plant's winding heating on its
/// own i^2 R; the rows the CLI would record.
fn hold(contact: Contact) -> Vec<Row> {
    let sh = rig();
    let mut k: Kernel<FakeIo> = kernel();
    let mut p = RlPlant::new(STOP);
    p.stop_lo = STOP as i32;
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_duty = HOLD_Q15;
        t.control.lifecycle.torque_enable = true;
    });
    let dt = 1.0 / TIMING.tick_hz as f64;
    let mut x_cc = 0.0f64;
    let mut rows = Vec::new();
    for tick in 0..(HOLD_S * TIMING.tick_hz as f64) as u32 {
        if tick % PERMIT_EVERY_TICKS == 0 {
            write_permit(&sh);
        }
        let duty = k.io.motor.last.map_or(0, duty_of);
        k.on_tick(p.step(duty), &sh);
        let i = p.current() as f64;
        let power = i * i * p.r_q12 as f64 / 4096.0;
        x_cc += dt / PLANT_TAU_S * (PLANT_G_CC * power - x_cc);
        if tick % SLOW_TICKS == 0 {
            let t_s = tick as f64 * dt;
            let c = contact
                .iter()
                .rev()
                .find(|(from, _)| t_s >= *from)
                .map_or(1.0, |(_, f)| *f);
            let copper = (COPPER_ZERO_C + AMBIENT_C + x_cc / 100.0) / (COPPER_ZERO_C + AMBIENT_C);
            p.r_q12 = (RL_R_Q12 as f64 * copper * c).round() as i32;
            rows.push(read(&sh, tick));
        }
    }
    rows
}

fn assert_lands(rows: &[Row]) -> thermal::Fit {
    let f = thermal::fit(rows).expect("a fit");
    let (tau, g) = (f.tau_s() / PLANT_TAU_S - 1.0, f.g_cc() / PLANT_G_CC - 1.0);
    assert!(
        tau.abs() < 0.10,
        "tau {:.1} s, {:+.1}%: {f:?}",
        f.tau_s(),
        tau * 100.0
    );
    assert!(
        g.abs() < 0.10,
        "g {:.5}, {:+.1}%: {f:?}",
        f.g_cc(),
        g * 100.0
    );
    println!(
        "tau {:.1} s ({:+.1}%), g {:.5} ({:+.1}%), {} stretches, {} chunks, {} rejected, sd \
         {:.3} centi-C/tick, {:.0} s used",
        f.tau_s(),
        tau * 100.0,
        f.g_cc(),
        g * 100.0,
        f.segments,
        f.chunks,
        f.rejected,
        f.sd / thermometer::slow_hz(TIMING.tick_hz as f64),
        f.used_s
    );
    f
}

#[test]
fn a_seated_hold_at_the_limit_fits_the_plant_s_tau_and_g_within_ten_percent() {
    let f = assert_lands(&hold(&[]));
    assert_eq!(f.segments, 1);
}

/// Run 3's brush bridge (R x0.79 for 10 s, then back) and a +3% contact
/// step inside the band: the kernel drops the base on the v step and on
/// the rise rate, the fit cuts those stretches and still lands.
#[test]
fn contact_steps_inside_the_hold_are_cut_and_the_fit_still_lands_within_ten_percent() {
    let f = assert_lands(&hold(&[(150.0, 0.79), (160.0, 1.0), (300.0, 1.03)]));
    assert!(f.segments >= 4, "{f:?}");
    // the bridge in and out; the +3% step is under the 10% a step counts at
    assert_eq!(f.r_steps, 2, "{f:?}");
}
