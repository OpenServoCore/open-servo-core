//! The firmware update session: every roster servo waits in its bootloader
//! for the whole session (link sec 5), and one broadcast reset ends it, so
//! no servo that left its app hears a tinyboot byte after it. [`update`] composes the steps
//! a bench can also run one at a time: [`enter`], [`list`],
//! [`super::install`], [`super::reset_all`], [`confirm`].

use std::time::Duration;

use osc_protocol::wire::{BaudRate, Id};

use super::{BootInfo, CRC_NS_PER_BYTE, Image, MARGIN};
use crate::client::Client;
use crate::common;
use crate::error::{Error, LinkError, RejectReason};
use crate::mgmt::{self, Found};
use crate::pipe::Pipe;
use crate::session::BusMode;

/// The bootloader's id is the chip's 96-bit ESIG, the osc UID's first 12
/// bytes (tinyboot-ch32 `hal::esig::uid`, servo-ch32 `hal::esig::uid`).
pub const CHIP_UID: usize = 12;

/// From a Reset to the app answering: tinyboot-core's hand-off CRC over the
/// stored image, at most the osc-dev-v006 APP region (boot memory.x), then
/// servo-ch32 bringup's waits, at most 65 ms (runtime/init.rs:
/// `OPA_SETTLE_MS` plus `VMOTOR_BIAS_STEPS` x `VMOTOR_BIAS_STEP_MS`).
const APP_UP: Duration =
    Duration::from_micros(MARGIN * (APP_REGION * CRC_NS_PER_BYTE + BRINGUP_NS).div_ceil(1_000));
const APP_REGION: u64 = 58_112;
const BRINGUP_NS: u64 = 65_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// The roster walk and every roster servo to its bootloader
    /// ([`enter`]).
    Enter,
    /// The bootloader walk.
    Walk,
    Install,
    /// Every bootloader back to its app at once.
    Reset,
    /// The roster walk again and each installed servo's version.
    Confirm,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress<'a> {
    Phase(Phase),
    /// After each acked write of the install on `uid`, in image bytes.
    Bytes {
        uid: &'a [u8],
        written: u32,
        total: u32,
    },
}

/// Where a session left one device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Fate {
    /// Installed, and its app reports the image's version.
    Confirmed,
    /// Installed, its version not read back: the app reported another
    /// (`Some`), or it could not be matched to one id (`None`).
    Unconfirmed(Option<u16>),
    /// The install stopped. Past the erase the device waits in its
    /// bootloader; before it, the old app runs again.
    Failed(Error),
    /// A chosen osc id not on the roster; `uid` is empty.
    NotFound,
    /// On the roster but not in the bootloader walk: still in its app, or
    /// silent.
    Missing,
    /// Reached its bootloader but is not on the roster after the session.
    NotBack,
    /// A bootloader off the roster, left as it was: [`update`] installs
    /// roster servos only.
    Waiting,
}

/// The roster servos a session installs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Chosen {
    All,
    Ids(Vec<Id>),
}

impl Chosen {
    fn has(&self, id: Id) -> bool {
        match self {
            Chosen::All => true,
            Chosen::Ids(ids) => ids.contains(&id),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Device {
    /// The tinyboot id.
    pub uid: Vec<u8>,
    /// The osc id on the session's roster; `None` off the roster.
    pub id: Option<Id>,
    pub fate: Fate,
}

/// Every device the session did not return to its app unchanged.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Report {
    pub devices: Vec<Device>,
}

/// One waiting bootloader and its Info.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BootDevice {
    pub uid: Vec<u8>,
    pub info: BootInfo,
}

/// Every rate an app may answer at: the wire's rates and the rescue rate.
fn sweep_rates() -> Vec<BaudRate> {
    let mut rates = vec![
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ];
    if !rates.contains(&BaudRate::RESCUE) {
        rates.push(BaudRate::RESCUE);
    }
    rates
}

/// Every app on the wire and the rate it answered at: a root ENUM probe at
/// each rate of [`sweep_rates`], the roster walk only where one answers.
/// The bus starts and ends in osc mode at its current rate.
pub async fn apps<P: Pipe>(c: &mut Client<P>) -> Result<Vec<(BaudRate, Found)>, Error> {
    osc_mode(c)?;
    let start = c.info().bps;
    let mut out = Vec::new();
    let r = sweep(c, &mut out).await;
    let back = c.set_bus(BusMode::Osc, start).await;
    r.and(back).map(|()| out)
}

async fn sweep<P: Pipe>(c: &mut Client<P>, out: &mut Vec<(BaudRate, Found)>) -> Result<(), Error> {
    for rate in sweep_rates() {
        c.host_baud(rate).await?;
        if mgmt::bus_present(c).await? {
            out.extend(mgmt::discover(c).await?.into_iter().map(|f| (rate, f)));
        }
    }
    Ok(())
}

/// The roster at the bus rate, each servo sent to its bootloader at `rate`
/// (an osc rate). A REBOOT that does not land leaves its servo in its
/// app, which the bootloader walk then misses. The bus must be in osc mode
/// and stays there.
pub async fn enter<P: Pipe>(c: &mut Client<P>, rate: BaudRate) -> Result<Vec<Found>, Error> {
    osc_mode(c)?;
    let roster = mgmt::discover(c).await?;
    for f in &roster {
        reboot(c, f.id, rate).await?;
    }
    Ok(roster)
}

/// The bootloader walk and each device's Info, selected by full id. The bus
/// must be in tinyboot mode.
pub async fn list<P: Pipe>(c: &mut Client<P>) -> Result<Vec<BootDevice>, Error> {
    let mut out = Vec::new();
    for uid in super::discover(c).await? {
        super::select_one(c, &uid).await?;
        let info = super::info(c).await?;
        out.push(BootDevice { uid, info });
    }
    Ok(out)
}

/// After a reset to the apps: wait for them to come up, walk the roster
/// again at the bus rate and judge each of `waiting`, the roster servos
/// that reached their bootloader. One absent is [`Fate::NotBack`]; one whose
/// tinyboot id is in `installed` has its version checked against
/// `version`; any other is back unchanged and left out. The bus must be in
/// osc mode.
pub async fn confirm<P: Pipe>(
    c: &mut Client<P>,
    waiting: &[Found],
    installed: &[Vec<u8>],
    version: u16,
) -> Result<Vec<Device>, Error> {
    c.pause(APP_UP).await;
    let after = mgmt::discover(c).await?;
    let mut out = Vec::new();
    for f in waiting {
        let uid = chip_uid(f);
        let fate = match after.iter().find(|g| chip_uid(g) == uid) {
            None => Fate::NotBack,
            Some(g) if installed.iter().any(|u| u == uid) => {
                version_fate(c, &after, g, version).await?
            }
            Some(_) => continue,
        };
        out.push(device(uid, Some(f.id), fate));
    }
    Ok(out)
}

/// Install `image` on the `chosen` roster servos in one session with the
/// whole fleet in its bootloader at `rate` (an osc rate, not
/// [`super::BaudRate`]). Bootloaders off the roster are reported and left
/// alone. The bus starts and ends in osc mode at its current rate.
/// Per-device failures land in the [`Report`]; `Err` is the link itself,
/// after one try to return the bus to osc mode.
pub async fn update<P: Pipe>(
    c: &mut Client<P>,
    image: &Image,
    chosen: &Chosen,
    rate: BaudRate,
    mut progress: impl FnMut(Progress<'_>),
) -> Result<Report, Error> {
    osc_mode(c)?;
    let fleet = c.info().bps;
    let r = session(c, image, chosen, rate, fleet, &mut progress).await;
    if r.is_err() {
        let _ = c.set_bus(BusMode::Osc, fleet).await;
    }
    r
}

async fn session<P: Pipe>(
    c: &mut Client<P>,
    image: &Image,
    chosen: &Chosen,
    rate: BaudRate,
    fleet: u32,
    progress: &mut impl FnMut(Progress<'_>),
) -> Result<Report, Error> {
    progress(Progress::Phase(Phase::Enter));
    let waiting = enter(c, rate).await?;
    let mut devices = Vec::new();
    if let Chosen::Ids(ids) = chosen {
        for (i, &id) in ids.iter().enumerate() {
            if !ids[..i].contains(&id) && !waiting.iter().any(|f| f.id == id) {
                devices.push(device(&[], Some(id), Fate::NotFound));
            }
        }
    }

    c.set_bus(BusMode::Tinyboot, rate.as_hz()).await?;
    progress(Progress::Phase(Phase::Walk));
    let walk = super::discover(c).await?;
    // Roster servos the walk missed or whose install failed: already judged.
    let mut done: Vec<&[u8]> = Vec::new();
    for f in &waiting {
        if !walk.iter().any(|u| u == chip_uid(f)) {
            done.push(chip_uid(f));
            devices.push(device(chip_uid(f), Some(f.id), Fate::Missing));
        }
    }
    for uid in &walk {
        if !waiting.iter().any(|f| chip_uid(f) == uid) {
            devices.push(device(uid, None, Fate::Waiting));
        }
    }

    progress(Progress::Phase(Phase::Install));
    let mut installed = Vec::new();
    for f in &waiting {
        let uid = chip_uid(f);
        if !chosen.has(f.id) || done.contains(&uid) {
            continue;
        }
        let bytes = |written, total| {
            progress(Progress::Bytes {
                uid,
                written,
                total,
            })
        };
        match super::install(c, uid, image, bytes).await {
            Ok(()) => installed.push(uid.to_vec()),
            Err(e) if per_device(&e) => {
                done.push(uid);
                devices.push(device(uid, Some(f.id), Fate::Failed(e)));
            }
            Err(e) => return Err(e),
        }
    }

    progress(Progress::Phase(Phase::Reset));
    super::reset_all(c).await?;
    c.set_bus(BusMode::Osc, fleet).await?;

    progress(Progress::Phase(Phase::Confirm));
    let back: Vec<Found> = waiting
        .iter()
        .filter(|f| !done.contains(&chip_uid(f)))
        .copied()
        .collect();
    devices.extend(confirm(c, &back, &installed, image.version()).await?);
    Ok(Report { devices })
}

fn osc_mode<P: Pipe>(c: &Client<P>) -> Result<(), Error> {
    use osc_host::link::record::MODE_OSC;
    if c.info().mode != MODE_OSC {
        return Err(Error::Link(LinkError::Rejected(RejectReason::WrongMode)));
    }
    Ok(())
}

fn chip_uid(f: &Found) -> &[u8] {
    &f.uid.0[..CHIP_UID]
}

fn device(uid: &[u8], id: Option<Id>, fate: Fate) -> Device {
    Device {
        uid: uid.to_vec(),
        id,
        fate,
    }
}

/// A verdict on one device, not a broken link.
fn per_device(e: &Error) -> bool {
    !matches!(
        e,
        Error::Pipe(_) | Error::Link(LinkError::Desync(_) | LinkError::Rejected(_))
    )
}

/// A REBOOT whose ack is lost may still have landed: the ping decides.
async fn reboot<P: Pipe>(c: &mut Client<P>, id: Id, rate: BaudRate) -> Result<(), Error> {
    match mgmt::reboot_bootloader(c, id, rate).await {
        Err(e) if !per_device(&e) => Err(e),
        _ => Ok(()),
    }
}

/// The installed servo's version, read from the id its UID answered from
/// after the session, when no other UID shares that id.
async fn version_fate<P: Pipe>(
    c: &mut Client<P>,
    after: &[Found],
    f: &Found,
    version: u16,
) -> Result<Fate, Error> {
    if after.iter().filter(|g| g.id == f.id).count() > 1 {
        return Ok(Fate::Unconfirmed(None));
    }
    Ok(match common::identity(c, f.id).await {
        Ok(i) if i.fw == version => Fate::Confirmed,
        Ok(i) => Fate::Unconfirmed(Some(i.fw)),
        Err(e) if per_device(&e) => Fate::Unconfirmed(None),
        Err(e) => return Err(e),
    })
}
