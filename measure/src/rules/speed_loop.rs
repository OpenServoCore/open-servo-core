//! The speed and position loops. The speed PI, C = kp + ki / (z - 1) with
//! its zero at w_z = ki x f_med / kp, drives the plant B: on that rigid
//! plant, with r = w_c / w_z, the loop crosses one at w_c when
//! kp = w_c / (B f_med) x r / sqrt(1 + r^2), its phase margin atan(r).
//!
//! The starting gain is the lower of two bounds. The lag bound puts the
//! crossover under the electromechanical frequency Ke / (2 pi sqrt(J L)),
//! where the current's L di/dt has not yet cancelled the speed the
//! back-EMF reads. The shelf cap holds the shelf the spinning brushes put
//! in the loop, kp e / Ke, the policy's margin under 0 dB on every sweep
//! of a margin record. A user has no margin record: the gains then bind
//! only from a closed-loop check that settled at or under the starting
//! gain (`Measured::speed_kp_verified`). A set the record measured
//! stands for the rule's kp within `SNAP_COUNTS` of it.
//!
//! ki and kaw follow from kp by r at its rigid crossover, kaw being ki /
//! kp, a tracking time equal to the integral time; the position gain is
//! that crossover over the policy's split.

use super::units::{b_gain, ke_v_s_per_rad, medium_hz};
use super::{Ctx, Fail, Format, Kind, Rule, get, q, table};
use crate::inputs::{Inputs, ShelfRecord, ShelfSweep};

pub const SNAP_COUNTS: f64 = 4.0;
/// What the speed gains wait for without a margin record.
const VERIFIED: &str =
    "measured.speed_kp_verified (a closed-loop check of the starting gains, or a margin record)";
const KP_ONE: f64 = 256.0;
const KI_ONE: f64 = 4096.0;
/// The shelf cap's bisection: the span about the swept kp, and its steps.
const BISECT_SPAN: f64 = 4096.0;
const BISECT_STEPS: usize = 60;

/// The lag bound's crossover, rad/s.
fn lag_rad_s(c: &mut Ctx) -> Result<f64, Fail> {
    let q = get!(c, policy, velocity_em_ratio)?;
    let ke = ke_v_s_per_rad(c)?;
    let j = q!(c, measured, j_kg_m2)?;
    let henry = q!(c, measured, l_mh)? / 1000.0;
    Ok(ke / (j * henry).sqrt() / q)
}

/// The kp that puts the rigid loop's crossover at `w` with the zero r
/// under it.
fn rigid_kp(c: &mut Ctx, w: f64) -> Result<f64, Fail> {
    let r = get!(c, policy, velocity_zero_ratio)?;
    Ok(w / (b_gain(c)? * medium_hz(c)) * r / (1.0 + r * r).sqrt())
}

fn rigid_rad_s(c: &mut Ctx, kp: f64) -> Result<f64, Fail> {
    Ok(kp / rigid_kp(c, 1.0)?)
}

#[derive(Clone, Copy)]
struct Complex(f64, f64);

impl Complex {
    fn div(self, o: Complex) -> Complex {
        let d = o.0 * o.0 + o.1 * o.1;
        Complex(
            (self.0 * o.0 + self.1 * o.1) / d,
            (self.1 * o.0 - self.0 * o.1) / d,
        )
    }

    fn mul(self, o: Complex) -> Complex {
        Complex(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }

    fn abs(self) -> f64 {
        self.0.hypot(self.1)
    }
}

/// The PI at `hz` on the MEDIUM rate.
fn pi(hz: f64, kp: f64, ki: f64, medium: f64) -> Complex {
    let (s, co) = (2.0 * core::f64::consts::PI * hz / medium).sin_cos();
    let z1 = Complex(co - 1.0, s);
    let i = Complex(ki, 0.0).div(z1);
    Complex(kp + i.0, i.1)
}

/// The mean dB over a sweep's rungs with the PI it ran swapped for
/// (kp, ki).
fn plateau_db(sweep: &ShelfSweep, kp: f64, ki: f64, medium: f64) -> f64 {
    let (run_kp, run_ki) = (
        sweep.v_kp_q88 as f64 / KP_ONE,
        sweep.v_ki_q412 as f64 / KI_ONE,
    );
    let sum: f64 = sweep
        .points
        .iter()
        .map(|&[hz, re, im]| {
            let ratio = pi(hz, kp, ki, medium).div(pi(hz, run_kp, run_ki, medium));
            20.0 * ratio.mul(Complex(re, im)).abs().log10()
        })
        .sum();
    sum / sweep.points.len() as f64
}

/// The kp whose shelf reads the policy's margin under 0 dB on the worst
/// of the record's sweeps, its ki the rule's at r, bisected.
fn shelf_cap(c: &mut Ctx, record: &ShelfRecord) -> Result<f64, Fail> {
    let m = get!(c, policy, velocity_shelf_margin_db)?;
    let r = get!(c, policy, velocity_zero_ratio)?;
    let medium = medium_hz(c);
    let per = rigid_kp(c, 1.0)?;
    let mut cap = f64::INFINITY;
    for sweep in record.sweeps.iter().filter(|s| !s.points.is_empty()) {
        let over = |kp: f64| plateau_db(sweep, kp, kp * kp / per / (r * medium), medium) + m;
        let run_kp = sweep.v_kp_q88 as f64 / KP_ONE;
        let (mut lo, mut hi) = (run_kp / BISECT_SPAN, run_kp * BISECT_SPAN);
        for _ in 0..BISECT_STEPS {
            let mid = (lo * hi).sqrt();
            if over(mid) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        cap = cap.min(lo);
    }
    if cap.is_finite() {
        Ok(cap)
    } else {
        Err(Fail::missing("measured.shelf.sweeps"))
    }
}

/// The gains the speed loop starts from: the lag bound, capped by the
/// shelf of a margin record where there is one. A closed-loop check may
/// lower them, never raise them.
pub fn starting_kp(c: &mut Ctx) -> Result<f64, Fail> {
    let w = lag_rad_s(c)?;
    let kp = rigid_kp(c, w)?;
    let shelf = c.inputs.measured.shelf.as_ref();
    match c.get("measured.shelf", shelf) {
        Ok(record) => Ok(kp.min(shelf_cap(c, record)?)),
        Err(_) => Ok(kp),
    }
}

/// [`starting_kp`] on `inputs`, outside any field, at the low end of its
/// inputs' bands: the gain a closed-loop check starts from and lowers.
/// `speed_pi` holds a verified gain against the starting gain at every
/// end of the bands, so a gain over the lowest is refused.
pub fn starting_gain(inputs: &Inputs) -> Result<f64, Fail> {
    super::lowest(inputs, starting_kp)
}

/// The speed PI and the crossover it puts the loop at.
#[derive(Clone, Copy, Debug)]
pub struct SpeedPi {
    pub kp: f64,
    pub ki: f64,
    pub kaw: f64,
    pub crossover_rad_s: f64,
}

fn pi_at(c: &mut Ctx, kp: f64) -> Result<SpeedPi, Fail> {
    let w = rigid_rad_s(c, kp)?;
    let kaw = w / (get!(c, policy, velocity_zero_ratio)? * medium_hz(c));
    Ok(SpeedPi {
        kp,
        ki: kp * kaw,
        kaw,
        crossover_rad_s: w,
    })
}

pub fn speed_pi(c: &mut Ctx) -> Result<SpeedPi, Fail> {
    let verified = c.inputs.measured.speed_kp_verified;
    if let Ok(kp) = c.get("measured.speed_kp_verified", verified) {
        let start = starting_kp(c)?;
        if kp > start {
            return Err(Fail::refused(format!(
                "speed_kp_verified {kp} is over the starting gain {start}: a check only lowers it"
            )));
        }
        return pi_at(c, kp);
    }
    let Some(record) = c.inputs.measured.shelf.as_ref() else {
        return Err(Fail::missing(VERIFIED));
    };
    let kp = starting_kp(c)?;
    let check = &record.check;
    if ((kp * KP_ONE).round_ties_even() - check.v_kp_q88 as f64).abs() <= SNAP_COUNTS {
        let kp = check.v_kp_q88 as f64 / KP_ONE;
        let w = rigid_rad_s(c, kp)?;
        return Ok(SpeedPi {
            kp,
            ki: check.v_ki_q412 as f64 / KI_ONE,
            kaw: check.v_kaw_q412 as f64 / KI_ONE,
            crossover_rad_s: w,
        });
    }
    pi_at(c, kp)
}

fn v_kp(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(speed_pi(c)?.kp)
}

fn v_ki(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(speed_pi(c)?.ki)
}

fn v_kaw(c: &mut Ctx) -> Result<f64, Fail> {
    Ok(speed_pi(c)?.kaw)
}

fn p_kp(c: &mut Ctx) -> Result<f64, Fail> {
    let w = speed_pi(c)?.crossover_rad_s;
    Ok(w / get!(c, policy, position_split)?)
}

pub const RULES: &[Rule] = table![
    v_kp_q88: "v_kp.bandwidth", Format::q(Kind::U16, 8), "ccounts/(count/s)", Policy, v_kp;
    v_ki_q412: "v_ki.bandwidth", Format::q(Kind::U16, 12), "ccounts/(count/s) per tick", Policy,
        v_ki;
    v_kaw_q412: "v_kaw.bandwidth", Format::q(Kind::U16, 12), "per tick", Policy, v_kaw;
    p_kp_q88: "p_kp.split", Format::q(Kind::U16, 8), "(count/s)/count", Policy, p_kp;
];
