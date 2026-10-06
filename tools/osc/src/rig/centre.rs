//! Into the band at mid travel (osc-ident's `Centre`); with the nudge, the
//! jam check: out and back once in the band, raising the duty while the
//! shaft does not move. A blocked shaft ends the drive, torque off. The
//! jam check's travel also shows which way the motor turns
//! ([`adopt_polarity`]).

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::{Experiment, Guarded, RigParams};
use osc_ident::regs::config;

use super::check_abort;
use super::pump::{Pump, write_reg};
use super::servo::{Servo, Wire, guard};

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

/// The `drive_polarity` the rest of a run drives under once the jam check,
/// `exp`, moved the shaft: `in_force`, unless a positive duty lowered the
/// counts - a motor plugged in the other way round - then the flip, written
/// in RAM so every seek after it drives toward mid travel.
pub(crate) fn adopt_polarity(
    c: &mut Client<NusbPipe>,
    id: Id,
    exp: &Centre,
    in_force: bool,
) -> Result<bool> {
    let polarity = exp.polarity_for(in_force);
    if polarity != in_force {
        println!(
            "  a positive duty lowered the counts: the motor turns against the stored \
             drive_polarity {}; this run drives with drive_polarity {} (RAM until a SAVE)",
            in_force as u8, polarity as u8
        );
        write_reg(c, id, config::DRIVE_POLARITY, polarity as i32)?;
    }
    Ok(polarity)
}
