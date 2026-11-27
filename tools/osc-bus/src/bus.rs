//! The bus every verb connects to.

use anyhow::{Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_protocol::wire::DEFAULT_RESPONSE_DEADLINE_US;

/// The bus rate and the host's RESPONSE_DEADLINE, which the adapter mirror
/// and every addressed servo take.
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
