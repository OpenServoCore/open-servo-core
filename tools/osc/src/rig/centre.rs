//! Into the band at mid travel (osc-ident's `Centre`), and the jam check
//! (osc-ident's `JamCheck`, [`jam_check`]): short pulses each way that show
//! the shaft moves and which way the motor turns, then toward mid travel.
//! A blocked shaft ends either, torque off.

use anyhow::{Result, bail};
use osc_client::calib::GearClass as Stored;
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::jam::{self, JamCfg, JamCheck};
use osc_ident::exp::rl::Scales;
use osc_ident::exp::{Experiment, Guarded, RigParams};
use osc_ident::limits::{GearClass, ServoLimits};
use osc_ident::regs::config;

use super::check_abort;
use super::csvio::{self, OutDir, SnapshotLog};
use super::pump::Pump;
use super::servo::{Servo, guard};

/// Pump `exp` to its end on the servo, parked safe however it ends.
pub(crate) fn drive<S: Servo>(s: &mut S, exp: &mut dyn Experiment) -> Result<()> {
    guard(s, |s| Pump::on(s, None).run(exp))
}

/// The centring pumped by `pump`, ended. A blocked shaft is an abort; a
/// centring that did not reach the band is left parked.
pub(crate) fn centre_on(
    pump: impl FnOnce(&mut Guarded<Centre>) -> Result<()>,
    cfg: CentreCfg,
    params: RigParams,
    what: &'static str,
) -> Result<Centre> {
    let mut exp = Guarded::new(Centre::new(cfg, &params), params.without_pos_guard());
    pump(&mut exp)?;
    check_abort(what, exp.abort())?;
    let exp = exp.into_inner();
    if !exp.arrived() {
        println!("  did not reach mid travel (gear slip?), left parked");
    }
    Ok(exp)
}

/// The gear class the servo holds, which sets the most current the jam
/// check may press a stop with: unknown on one `osc cal` never ran on.
pub(crate) fn gear_class<S: Servo>(s: &mut S) -> Result<GearClass> {
    let id = s.id();
    let d = crate::state::descriptor(s.client(), id)?;
    Ok(gear_class_of(s.client().gear_class(id, &d)?))
}

pub(crate) fn gear_class_of(stored: Stored) -> GearClass {
    match stored {
        Stored::Unknown => GearClass::Unknown,
        Stored::Plastic => GearClass::Plastic,
        Stored::Metal => GearClass::Metal,
    }
}

/// The jam check on `s`, by the servo's limits, scales and `class`, under
/// the `drive_polarity` in `params`: the check, the polarity it left in
/// RAM. Any end but mid travel is an error, the stored polarity written
/// back. Every pulse is printed and, with `out`, recorded there with
/// every read.
pub(crate) fn jam_check<S: Servo>(
    s: &mut S,
    lim: &ServoLimits,
    sc: &Scales,
    params: RigParams,
    class: GearClass,
    out: Option<&OutDir>,
) -> Result<JamCheck> {
    let tick_hz = s.tick_hz()?;
    if tick_hz <= 0.0 {
        bail!("the servo publishes no tick rate, and the jam check times its pulses by it");
    }
    let cfg = JamCfg::for_servo(lim, sc, class, tick_hz);
    let ma = |i: f64| cfg.ma(i).round();
    println!(
        "[jam check] short pulses each way from the current sensor's floor, {:.0} mV higher a \
         pair, up to {} mA for {} gears; then toward mid travel",
        cfg.mv(cfg.step),
        ma(cfg.i_probe),
        class.name()
    );
    if class == GearClass::Unknown {
        println!("  gear class unknown (servo not calibrated): probing at plastic limits");
    }
    let stored = params.drive_polarity;
    let mut log = out
        .map(|o| SnapshotLog::create(o, "jam_snapshots.csv"))
        .transpose()?;
    let (exp, abort) = jam::drive(JamCheck::new(cfg, &params), &params, |e| {
        guard(s, |s| Pump::on(s, log.as_mut()).run(e))
    })?;
    for p in exp.pulses() {
        println!("  {p}");
    }
    if let Some(o) = out {
        csvio::write_jam_pulses(o, exp.pulses())?;
    }
    let ended = check_abort("the jam check", abort).and_then(|()| {
        if !exp.arrived() {
            bail!("the jam check did not reach mid travel (gear slipping?)");
        }
        Ok(())
    });
    if let Err(e) = ended {
        if exp.polarity() != stored {
            let _ = s.write(config::DRIVE_POLARITY, stored as i32);
        }
        return Err(e);
    }
    if let Some(m) = exp.moved_at() {
        println!(
            "  the shaft moves at {:.1}%; a pulse moved it {} counts at most",
            m * 100.0,
            exp.pulse_travel_max()
        );
    }
    if exp.polarity() != stored {
        let latch = if exp.resumed() {
            "the servo latched drive_reversed: "
        } else {
            ""
        };
        println!(
            "  {latch}the motor turns against the stored drive_polarity {}; this run drives with \
             drive_polarity {} (RAM until a SAVE)",
            stored as u8,
            exp.polarity() as u8
        );
    }
    Ok(exp)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Supply;
    use crate::rig::blocked;
    use crate::rig::servo::bench::{self, Bench};
    use osc_ident::exp::testkit::board_d_scales;

    /// The bench MG90 at `pos` under the stored `drive_polarity` 1, its
    /// motor wired `along` it or not, the kernel's drive check on.
    fn bench_at(pos: f64, along: bool) -> Bench {
        let mut b = Bench::mg90(Supply::TwoS);
        b.servo.pos = pos;
        b.servo.drive_polarity = along;
        b.servo.drive_check = true;
        b
    }

    fn lim() -> ServoLimits {
        ServoLimits {
            i_lim: 280,
            stall_yield: 168,
            tau_trip: 280,
            soft: (432, 3626),
            phys: (232, 3849),
            raw: (232, 3849),
            r_q12: 0,
            vbus: 3204,
            window_floor_q15: 4356,
            window_v_floor_q15: 4356,
            amps_per_count: board_d_scales().amps_per_count,
            drive_polarity: true,
        }
    }

    fn params() -> RigParams {
        RigParams::new(Some((532, 3526)), 350).with_stops((232, 3849))
    }

    fn check(b: &mut Bench) -> Result<JamCheck> {
        let class = gear_class(b)?;
        jam_check(b, &lim(), &board_d_scales(), params(), class, None)
    }

    /// The class the servo holds: the bench table's, seeded calibrated, is
    /// plastic; then whatever `osc cal` writes, unknown included.
    #[test]
    fn the_gear_class_is_the_servos() {
        let mut b = bench_at(2029.0, true);
        assert_eq!(gear_class(&mut b).unwrap(), GearClass::Plastic);
        let id = b.id();
        for (byte, class) in [(2, GearClass::Metal), (0, GearClass::Unknown)] {
            bench::set(&mut b.c, id, osc_ident::regs::calib::GEAR_CLASS, byte);
            assert_eq!(gear_class(&mut b).unwrap(), class);
        }
    }

    /// Under the low soft limit with the motor reversed: the first pulse
    /// runs down, the flip goes to the servo in RAM, and the check ends at
    /// mid travel, torque off, the endstop never refusing a duty written.
    #[test]
    fn a_reversed_motor_off_mid_travel_is_flipped_in_ram() {
        let mut b = bench_at(380.0, false);
        let exp = check(&mut b).unwrap();
        assert!(!exp.polarity() && !b.servo.burst.drive_polarity);
        assert!(exp.arrived());
        let (lo, hi) = exp.band();
        assert!(
            (lo as f64..=hi as f64).contains(&b.servo.pos),
            "{}",
            b.servo.pos
        );
        assert!(!b.servo.torque && b.servo.refused_goals.is_empty());
    }

    /// A reversed motor whose pot is dead runs past the soft limit unseen
    /// and the servo latches drive_reversed: the check rests, flips,
    /// acknowledges and ends at the position sensor. It is an error that
    /// says what to check, and the stored polarity is written back.
    #[test]
    fn a_failed_check_writes_the_stored_polarity_back() {
        let mut b = bench_at(400.0, false);
        b.servo.pot_dead = Some(400);
        let Err(e) = check(&mut b) else {
            panic!("passed");
        };
        assert!(blocked(&e), "{e:#}");
        let msg = e.to_string();
        assert!(msg.contains("position sensor does not move"), "{msg}");
        assert!(msg.contains("check the pot's wiring"), "{msg}");
        assert!(b.servo.burst.drive_polarity && !b.servo.reversed);
        assert!(!b.servo.torque);
    }

    /// A locked shaft: stiff at the probe's cap, a verdict nothing may
    /// drive again, in words that say what to check.
    #[test]
    fn a_jam_ends_stiff() {
        let mut b = bench_at(2029.0, true);
        b.servo.jam = Some(2029.0);
        let Err(e) = check(&mut b) else {
            panic!("passed");
        };
        assert!(blocked(&e), "{e:#}");
        assert!(e.to_string().contains("turn the horn by hand"), "{e}");
        assert!(!b.servo.torque);
    }
}
