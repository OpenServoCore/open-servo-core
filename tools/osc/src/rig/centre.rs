//! Into the band at mid travel (osc-ident's `Centre`); with the nudge, the
//! jam check: out and back once in the band, raising the duty while the
//! shaft does not move. A blocked shaft ends the drive, torque off.

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::centre::{Centre, CentreCfg};
use osc_ident::exp::{Experiment, Guarded, RigParams};

use super::check_abort;
use super::pump::{Pump, with_guard};

/// Pump `exp` to its end on the servo, parked safe however it ends.
pub(crate) fn drive(c: &mut Client<NusbPipe>, id: Id, exp: &mut dyn Experiment) -> Result<()> {
    with_guard(c, id, |c| Pump::new(c, id, None).run(exp))
}

/// [`centre_on`] on the servo.
pub(crate) fn centre(
    c: &mut Client<NusbPipe>,
    id: Id,
    cfg: CentreCfg,
    params: RigParams,
    what: &'static str,
) -> Result<Option<f64>> {
    centre_on(|exp| drive(c, id, exp), cfg, params, what)
}

/// The centring pumped by `pump`: the duty the shaft first travelled at,
/// when it travelled. A blocked shaft is an abort; a jam check that never
/// reached the band is an error, a centring that did not is left parked.
pub(crate) fn centre_on(
    pump: impl FnOnce(&mut Guarded<Centre>) -> Result<()>,
    cfg: CentreCfg,
    params: RigParams,
    what: &'static str,
) -> Result<Option<f64>> {
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
    Ok(exp.moved_at())
}
