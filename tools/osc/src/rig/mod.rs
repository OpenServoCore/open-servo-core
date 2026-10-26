//! Shared rig plumbing for the experiment subcommands: the bus connection,
//! the servo's limits, the driver pump (TEL bursts included), table
//! snapshot/rollback, CSV record/replay, the centring and the horn park.
//! `ident` drives these; `cal`, `sweep` and `capture` reach the same set.

pub(crate) mod battery;
pub(crate) mod centre;
pub(crate) mod csvio;
pub(crate) mod limits;
pub(crate) mod park;
pub(crate) mod plant;
pub(crate) mod pump;
pub(crate) mod servo;
pub(crate) mod snapshot;

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::data_state::fault;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::AbortReason;
use osc_protocol::wire::DEFAULT_RESPONSE_DEADLINE_US;

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
        if let AbortReason::Fault { code, .. } = self.reason
            && let Some(name) = fault::name(code)
        {
            write!(f, ", {name}")?;
        }
        let what = match self.reason {
            AbortReason::Blocked { .. } => "",
            AbortReason::Stiff { .. } => {
                ": turn the horn by hand to feel for a jam, and if it rests at an end stop turn \
                 it toward mid travel and run again"
            }
            AbortReason::AtStop { .. } => {
                return write!(
                    f,
                    "; torque is off and the shaft stays where it stopped: move the horn toward \
                     mid travel by hand and run again"
                );
            }
            AbortReason::SensorDead { .. } => {
                ": check the pot's wiring, and for a stripped output gear"
            }
            AbortReason::MotorOpen { .. } => ": check the motor's leads and their solder joints",
            _ => return Ok(()),
        };
        write!(
            f,
            "; a blocked shaft cannot be centred, so torque is off and the shaft stays where it \
             stopped{what}"
        )
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
    e.downcast_ref::<Aborted>().is_some_and(|a| {
        matches!(
            a.reason,
            AbortReason::Blocked { .. }
                | AbortReason::Stiff { .. }
                | AbortReason::AtStop { .. }
                | AbortReason::SensorDead { .. }
                | AbortReason::MotorOpen { .. }
        )
    })
}

/// The bus every verb connects to: its rate and the host's
/// RESPONSE_DEADLINE, which the adapter mirror and every addressed servo take.
#[derive(clap::Args, Debug, Clone)]
pub(crate) struct Bus {
    /// Wire baud, or `auto` to find the bus by ENUM probe.
    #[arg(short, long, global = true, default_value = "auto")]
    pub(crate) baud: String,
    /// RESPONSE_DEADLINE in us, set on the adapter and on each addressed
    /// servo that differs (protocol sec 5.4).
    #[arg(long, global = true, default_value_t = DEFAULT_RESPONSE_DEADLINE_US)]
    pub(crate) deadline: u16,
}

/// Connect at the bus rate and bring the adapter and `ids` to the host's
/// RESPONSE_DEADLINE.
pub(crate) fn connect(bus: &Bus, ids: &[Id]) -> Result<Client<NusbPipe>> {
    let mut c = Client::connect(NusbPipe::open()?)?;
    match bus.baud.as_str() {
        "auto" => match c.find_bus_baud()? {
            Some(rate) => eprintln!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => c.host_baud(crate::parse_rate(s)?)?,
    }
    for (id, held) in c.sync_response_deadline(ids, bus.deadline)? {
        eprintln!(
            "id {}: response_deadline_us {held} -> {} (volatile until SAVE)",
            id.as_byte(),
            bus.deadline
        );
    }
    Ok(c)
}
