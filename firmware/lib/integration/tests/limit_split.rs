//! The press detector against the continuous rating on the R-L plant rig
//! (`kernel::phases::loops::limits`), over a randomized servo family: the
//! winding from 3 to 9 ohm equivalent, the cap from 250 to 1000 mA on the
//! osc-dev-v006 chain, the rating at 30 to 70% of it. OpenLoop locks the
//! rotor at the duty whose stall current sits at a stated fraction of the
//! cap, and the kernel reads that current through the shunt window one
//! tick late. Every pin runs every seed: the worst seed is the verdict.

use osc_integration::plant::{FakeIo, RL_VBUS, RlPlant, duty_of, kernel, seed, stamp};
use osc_servo_core::kernel::phases::loops::limits::flag;
use osc_servo_core::regions::config::ms_ticks;
use osc_servo_core::{Kernel, Mode, RegionStorage, Shared, StallResponse};

const MID: u16 = 2050;
const STALL_MS: u32 = 300;
/// FAST ticks per ms.
const MS: u32 = 20;
/// Bound on the observer's settle past `stall_time_ticks` before omega
/// reads slow and the timer runs out.
const SETTLE_MS: u32 = 200;
const SEEDS: u64 = 16;

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }

    /// Uniform in `lo..=hi`.
    fn range(&mut self, lo: u32, hi: u32) -> u32 {
        lo + (self.next() % (hi - lo + 1) as u64) as u32
    }
}

/// One servo of the family: its winding, cap and rating.
#[derive(Copy, Clone, Debug)]
struct Family {
    r_q12: u16,
    cap: u16,
    i_cont: u16,
}

impl Family {
    fn draw(seed: u64) -> Self {
        let mut g = Lcg(0x9E3779B97F4A7C15 ^ seed);
        // 3 to 9 ohm as the rig scales it: 7270 is 4.9 ohm
        let r_q12 = g.range(4450, 13350) as u16;
        // 250 to 1000 mA on the osc-dev-v006 chain (1108.5 counts/A)
        let cap = g.range(277, 1108) as u16;
        let i_cont = (cap as u32 * g.range(30, 70) / 100) as u16;
        Self { r_q12, cap, i_cont }
    }

    /// The duty whose locked-rotor current is `frac`/100 of the cap.
    fn duty_at(&self, frac: u32) -> i16 {
        let counts = self.cap as u32 * frac / 100;
        // duty = i x R / vbus in Q15: (counts x r_q12 >> 12) x 32767 / vbus
        let v = (counts * self.r_q12 as u32) >> 12;
        (v * 32767 / RL_VBUS as u32).min(i16::MAX as u32) as i16
    }

    fn rig(&self, response: StallResponse) -> Shared {
        let sh = Shared::new();
        seed(&sh);
        sh.table.with_mut(|t| {
            t.calib.sense.i_window_min_ticks = 64;
            t.calib.sense.v_window_min_ticks = 64;
            t.calib.motor.r_q12 = self.r_q12;
            let l = &mut t.config.limits;
            l.current_limit_counts = self.cap;
            l.i_cont_counts = self.i_cont;
            l.stall_release_counts = self.i_cont / 2;
            l.stall_response = response;
            l.stall_time_ticks = ms_ticks(STALL_MS);
            l.stall_tau_trip_counts = u16::MAX;
            // the budget is not under test here: a 66 s constant
            l.thermal_tau_s = 66;
            l.openloop_zero_brake = false;
            t.control.lifecycle.mode = Mode::OpenLoop;
            t.config.fusion.l1_q016 = 8640;
            t.config.fusion.l2_q88 = 3180;
            t.config.fusion.l3_q88 = 84;
            t.calib.motor.b_i_q313 = 2427;
            t.calib.motor.fric_fc_counts = 53;
            t.calib.motor.ke_vpc_q = 603;
            t.calib.motor.recip_ke_q = 6957;
        });
        stamp(&sh);
        sh
    }
}

fn run(k: &mut Kernel<FakeIo>, sh: &Shared, p: &mut RlPlant, ticks: u32) -> Vec<(i16, i32)> {
    (0..ticks)
        .map(|_| {
            let duty = duty_of(k);
            k.on_scan(p.step(duty), sh);
            (duty_of(k), p.current())
        })
        .collect()
}

/// A locked rotor of winding `r_q12` at `goal` from a torque-off rest.
fn locked(sh: &Shared, r_q12: u16, goal: i16) -> (Kernel<FakeIo>, RlPlant) {
    let mut p = RlPlant::new(MID);
    p.locked = true;
    p.r_q12 = r_q12;
    let mut k = kernel();
    run(&mut k, sh, &mut p, 400);
    sh.table.with_mut(|t| {
        t.control.lifecycle.goal_duty = goal;
        t.control.lifecycle.torque_enable = true;
    });
    (k, p)
}

fn i_lim(sh: &Shared) -> u16 {
    sh.table.with(|t| t.telemetry.estimates.i_lim_counts)
}

fn flags(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.limits.limit_flags)
}

fn faults(sh: &Shared) -> u8 {
    sh.table.with(|t| t.telemetry.common.fault_flags)
}

/// The tick the press folded at a locked rotor drawing `frac`/100 of the
/// cap, or None inside the timer plus the settle.
fn fold_tick(f: &Family, frac: u32, cap: u16) -> Option<u32> {
    let sh = f.rig(StallResponse::Yield);
    sh.table
        .with_mut(|t| t.config.limits.current_limit_counts = cap);
    stamp(&sh);
    let (mut k, mut p) = locked(&sh, f.r_q12, f.duty_at(frac));
    (0..(STALL_MS + SETTLE_MS) * MS).find(|_| {
        run(&mut k, &sh, &mut p, 1);
        flags(&sh) & flag::YIELD != 0
    })
}

/// a jam drawing more than the rating trips after the timer and
/// folds to the rating; the same jam under the rating never trips,
/// however long it is held.
#[test]
fn press_over_the_rating_trips_under_it_never() {
    for seed in 0..SEEDS {
        let f = Family::draw(seed);
        let what = format!("seed {seed} {f:?}");
        let over = (f.i_cont as u32 * 100).div_ceil(f.cap as u32) + 15;
        let under = f.i_cont as u32 * 85 / (f.cap as u32);
        assert!(over < 100 && under > 0, "{what}: fractions {over} {under}");
        let at = fold_tick(&f, over, f.cap).unwrap_or_else(|| panic!("{what}: never folded"));
        assert!(at >= STALL_MS * MS, "{what}: folded {} ms in", at / MS);
        let sh = f.rig(StallResponse::Yield);
        let (mut k, mut p) = locked(&sh, f.r_q12, f.duty_at(under));
        run(&mut k, &sh, &mut p, 2 * (STALL_MS + SETTLE_MS) * MS);
        assert_eq!(
            flags(&sh) & flag::YIELD,
            0,
            "{what}: tripped under the rating"
        );
        assert_eq!(i_lim(&sh), f.cap, "{what}");
        assert_eq!(faults(&sh), 0, "{what}");
    }
}

/// the cap moves the ceiling, never the trip: at twice the cap
/// the same jam folds at the same tick, within a medium period, and to
/// the same rating.
#[test]
fn raising_the_cap_does_not_move_the_trip() {
    for seed in 0..SEEDS {
        let f = Family::draw(seed);
        let what = format!("seed {seed} {f:?}");
        let over = (f.i_cont as u32 * 100).div_ceil(f.cap as u32) + 15;
        let at = fold_tick(&f, over, f.cap).unwrap_or_else(|| panic!("{what}: never folded"));
        let doubled = fold_tick(&f, over, f.cap.saturating_mul(2))
            .unwrap_or_else(|| panic!("{what}: never folded at twice the cap"));
        assert!(
            at.abs_diff(doubled) <= 10,
            "{what}: folded at {at} vs {doubled} with the cap doubled"
        );
        let sh = f.rig(StallResponse::Yield);
        sh.table
            .with_mut(|t| t.config.limits.current_limit_counts = f.cap.saturating_mul(2));
        stamp(&sh);
        let (mut k, mut p) = locked(&sh, f.r_q12, f.duty_at(over));
        run(&mut k, &sh, &mut p, (STALL_MS + SETTLE_MS) * MS);
        assert_eq!(i_lim(&sh), f.i_cont, "{what}: the fold is the rating");
    }
}

/// the permit lifts the press and the endstop, never a ceiling: a
/// permitted jam over the rating never folds on the verdict and still
/// sits under the cap.
#[test]
fn permit_lifts_the_press_not_the_cap() {
    for seed in 0..SEEDS {
        let f = Family::draw(seed);
        let what = format!("seed {seed} {f:?}");
        let sh = f.rig(StallResponse::Fault);
        let (mut k, mut p) = locked(&sh, f.r_q12, f.duty_at(100));
        let mut r = Vec::new();
        for _ in 0..4 {
            sh.table
                .with_mut(|t| t.control.lifecycle.stall_permit = true);
            sh.permit_after_commit(
                osc_servo_core::regions::control::addr::lifecycle::STALL_PERMIT,
                1,
            );
            r.extend(run(&mut k, &sh, &mut p, 250 * MS));
        }
        assert_eq!(faults(&sh), 0, "{what}");
        assert_eq!(
            flags(&sh) & (flag::YIELD | flag::PERMIT),
            flag::PERMIT,
            "{what}"
        );
        assert_eq!(i_lim(&sh), f.cap, "{what}");
        let peak = r[2_000..].iter().map(|&(_, i)| i.abs()).max().unwrap_or(0);
        assert!(peak <= f.cap as i32 * 11 / 10, "{what}: peak {peak}");
    }
}
