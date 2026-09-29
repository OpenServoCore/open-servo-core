//! Shared rig plumbing for the experiment subcommands: the bus connection,
//! the servo's limits, the driver pump (TEL bursts included), table
//! snapshot/rollback, CSV record/replay, and the horn park. `ident` drives
//! these; `cal` and `sweep` reach the same set.

pub(crate) mod csvio;
pub(crate) mod limits;
pub(crate) mod park;
pub(crate) mod plant;
pub(crate) mod pump;
pub(crate) mod snapshot;

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::AbortReason;

/// A drive that stopped early, typed so a caller can tell a blocked shaft,
/// which nothing may drive again, not even to centre it, from every other
/// failure.
#[derive(Debug)]
pub(crate) struct Aborted {
    pub(crate) what: &'static str,
    pub(crate) reason: AbortReason,
}

impl std::fmt::Display for Aborted {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} aborted: {}", self.what, self.reason)?;
        if let AbortReason::Blocked { .. } = self.reason {
            write!(
                f,
                "; a blocked shaft cannot be centred, so torque is off and the shaft stays \
                 where it stopped"
            )?;
        }
        Ok(())
    }
}

impl std::error::Error for Aborted {}

/// Err when the envelope, or the experiment itself, ended the drive.
pub(crate) fn check_abort(what: &'static str, abort: Option<AbortReason>) -> Result<()> {
    match abort {
        None => Ok(()),
        Some(reason) => Err(Aborted { what, reason }.into()),
    }
}

/// The error is a blocked shaft: nothing may drive it again.
pub(crate) fn blocked(e: &anyhow::Error) -> bool {
    e.downcast_ref::<Aborted>()
        .is_some_and(|a| matches!(a.reason, AbortReason::Blocked { .. }))
}

pub(crate) fn connect(baud: &str) -> Result<Client<NusbPipe>> {
    let mut c = Client::connect(NusbPipe::open()?)?;
    match baud {
        "auto" => match c.find_bus_baud()? {
            Some(rate) => println!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => {
            use osc_client::BaudRate as B;
            let bps: u32 = s.parse().context("baud is an integer rate or `auto`")?;
            let rate = [B::B500000, B::B1000000, B::B2000000, B::B3000000]
                .into_iter()
                .find(|r| r.as_hz() == bps)
                .ok_or_else(|| anyhow::anyhow!("unsupported baud {bps}"))?;
            c.host_baud(rate)?;
        }
    }
    Ok(c)
}
