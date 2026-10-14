//! Into the band at mid travel (osc-ident's `Centre`); with the nudge, the
//! jam check: out and back once in the band, raising the duty while the
//! shaft does not move. A blocked shaft ends the drive, torque off. The
//! jam check's travel also shows which way the motor turns
//! ([`jam_check`]).

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::fault;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::{AbortReason, Experiment, Guarded, RigParams};
use osc_ident::regs::{config, control};

use super::pump::Pump;
use super::servo::{Servo, Wire, guard};
use super::{Aborted, check_abort};

/// Pump `exp` to its end on the servo, parked safe however it ends.
pub(crate) fn drive<S: Servo>(s: &mut S, exp: &mut dyn Experiment) -> Result<()> {
    guard(s, |s| Pump::on(s, None).run(exp))
}

/// [`centre_on`] on the servo.
pub(crate) fn centre(
    c: &mut Client<NusbPipe>,
    id: Id,
    cfg: CentreCfg,
    params: RigParams,
    what: &'static str,
) -> Result<Centre> {
    let mut s = Wire::new(c, id);
    centre_on(|exp| drive(&mut s, exp), cfg, params, what)
}

/// The centring pumped by `pump`, ended. A blocked shaft is an abort; a
/// jam check that never reached the band is an error, a centring that did
/// not is left parked.
pub(crate) fn centre_on(
    pump: impl FnOnce(&mut Guarded<Centre>) -> Result<()>,
    cfg: CentreCfg,
    params: RigParams,
    what: &'static str,
) -> Result<Centre> {
    let nudge = cfg.nudge;
    let mut exp = Guarded::new(Centre::new(cfg, &params), params.without_pos_guard());
    pump(&mut exp)?;
    check_abort(what, exp.abort())?;
    let exp = exp.into_inner();
    if !exp.arrived() {
        if nudge {
            bail!("the jam check did not reach mid travel in 5 s (gear slipping?)");
        }
        println!("  did not reach mid travel (gear slip?), left parked");
    }
    Ok(exp)
}

/// The jam check on `s` under `in_force`, the stored `drive_polarity`, and
/// the polarity the rest of a run drives under: `in_force`, unless a
/// positive duty lowered the counts - a motor plugged in the other way
/// round - then the flip, written in RAM so every seek after it drives
/// toward mid travel. Off mid travel the drive runs away from it and the
/// servo latches `drive_reversed` past a soft limit, before the travel
/// shows the direction: the flip, the latch acknowledged and the jam check
/// once more. That one failing too is an error, `in_force` written back.
pub(crate) fn jam_check<S: Servo>(
    s: &mut S,
    cfg: CentreCfg,
    params: RigParams,
    in_force: bool,
) -> Result<(Centre, bool)> {
    let poll_ms = cfg.poll_ms;
    let run = |s: &mut S| centre_on(|exp| drive(s, exp), cfg.clone(), params, "the jam check");
    let (exp, in_force) = match run(s) {
        Err(e) if drive_reversed(&e) => {
            let flipped = !in_force;
            println!(
                "  the servo latched drive_reversed: the motor turns against the stored \
                 drive_polarity {}; the jam check again with drive_polarity {} (RAM until a SAVE)",
                in_force as u8, flipped as u8
            );
            s.write(config::DRIVE_POLARITY, flipped as i32)?;
            ack(s, poll_ms)?;
            match run(s) {
                Ok(exp) => (exp, flipped),
                Err(e) => {
                    let _ = s.write(config::DRIVE_POLARITY, in_force as i32);
                    return Err(e);
                }
            }
        }
        r => (r?, in_force),
    };
    let polarity = exp.polarity_for(in_force);
    if polarity != in_force {
        println!(
            "  a positive duty lowered the counts: the motor turns against the stored \
             drive_polarity {}; this run drives with drive_polarity {} (RAM until a SAVE)",
            in_force as u8, polarity as u8
        );
        s.write(config::DRIVE_POLARITY, polarity as i32)?;
    }
    Ok((exp, polarity))
}

fn drive_reversed(e: &anyhow::Error) -> bool {
    e.downcast_ref::<Aborted>().is_some_and(|a| {
        matches!(
            a.reason,
            AbortReason::Fault {
                code: fault::DRIVE_REVERSED,
                ..
            }
        )
    })
}

/// The `torque_enable` 0 to 1 edge, the only acknowledgement of a latched
/// fault: torque on at zero duty for `poll_ms`, then off.
fn ack<S: Servo>(s: &mut S, poll_ms: u32) -> Result<()> {
    s.write(control::TORQUE_ENABLE, 1)?;
    s.sleep(poll_ms);
    s.write(control::TORQUE_ENABLE, 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::Supply;
    use crate::rig::blocked;
    use crate::rig::servo::bench::Bench;
    use osc_ident::exp::seek::SEEK_TRAVEL_MIN;

    /// The bench MG90 at `pos` under the stored `drive_polarity` 1, its
    /// motor wired `along` it or not, the kernel's drive check on.
    fn bench_at(pos: f64, along: bool) -> Bench {
        let mut b = Bench::mg90(Supply::TwoS);
        b.servo.pos = pos;
        b.servo.drive_polarity = along;
        b.servo.drive_check = true;
        b
    }

    fn cfg() -> CentreCfg {
        CentreCfg {
            duty_q15: 3119,
            cap_q15: 8290,
            nudge: true,
            ..CentreCfg::default()
        }
    }

    fn params() -> RigParams {
        RigParams::new(Some((432, 3626)), 350).with_stops((232, 3849))
    }

    /// Under the low soft limit with the motor reversed, the jam check's
    /// first drive runs away from mid travel and the servo latches
    /// drive_reversed before the travel shows the direction. The jam check
    /// flips the polarity, acknowledges the latch and drives home.
    #[test]
    fn drive_reversed_flips_the_polarity_and_checks_again() {
        let mut b = bench_at(380.0, false);
        let Err(e) = centre_on(|exp| drive(&mut b, exp), cfg(), params(), "the jam check") else {
            panic!("no latch");
        };
        assert!(drive_reversed(&e), "{e:#}");
        assert!(b.servo.pos > 232.0, "{}", b.servo.pos);

        let mut b = bench_at(380.0, false);
        let (exp, polarity) = jam_check(&mut b, cfg(), params(), true).unwrap();
        assert!(!polarity && !b.servo.burst.drive_polarity);
        assert!(exp.arrived());
        assert_eq!(exp.drive_polarity(), Some(true));
        // home where it entered the band
        let edge = exp.band().0 as f64;
        assert!(
            (b.servo.pos - edge).abs() < SEEK_TRAVEL_MIN as f64,
            "{}",
            b.servo.pos
        );
        assert!(!b.servo.torque && !b.servo.reversed);
    }

    /// A load past the low soft limit that the drive cannot hold latches
    /// drive_reversed on a motor wired the right way. The jam check after
    /// the flip fails too, the load pinning the shaft at the stop: that
    /// ends the run, the stored polarity written back, with no third run.
    #[test]
    fn a_failed_check_after_the_flip_writes_the_stored_polarity_back() {
        let loaded = || {
            let mut b = bench_at(380.0, true);
            b.servo.load = 400.0;
            b
        };
        let mut b = loaded();
        let Err(e) = centre_on(|exp| drive(&mut b, exp), cfg(), params(), "the jam check") else {
            panic!("no latch");
        };
        assert!(drive_reversed(&e), "{e:#}");

        let mut b = loaded();
        let Err(e) = jam_check(&mut b, cfg(), params(), true) else {
            panic!("the flip's check passed");
        };
        assert!(blocked(&e), "{e:#}");
        assert!(b.servo.burst.drive_polarity && b.servo.drive_polarity);
        assert!(!b.servo.torque);
    }
}
