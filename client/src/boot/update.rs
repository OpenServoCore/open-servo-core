//! The firmware update session: every servo on the wire waits in its
//! bootloader for the whole session (link sec 5), and one broadcast reset
//! ends it, so no app hears a tinyboot byte.

use std::time::Duration;

use osc_protocol::wire::{BaudRate, Id};

use super::{CRC_NS_PER_BYTE, Image, MARGIN};
use crate::client::Client;
use crate::common;
use crate::error::{Error, LinkError, RejectReason};
use crate::mgmt::{self, Found};
use crate::pipe::Pipe;
use crate::session::BusMode;

/// The bootloader's id is the chip's 96-bit ESIG, the osc UID's first 12
/// bytes (tinyboot-ch32 `hal::esig::uid`, servo-ch32 `hal::esig::uid`).
const CHIP_UID: usize = 12;

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
    /// The osc roster walk.
    Roster,
    /// Every roster servo to its bootloader.
    Reboot,
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
    /// (`Some`), or it could not be matched to one id (`None`), as for a
    /// bootloader-only board, which comes up on the factory id.
    Unconfirmed(Option<u16>),
    /// The install stopped. Past the erase the device waits in its
    /// bootloader; before it, the old app runs again.
    Failed(Error),
    /// Chosen, but the bootloader walk did not find it.
    NotFound,
    /// On the roster, silent after its REBOOT, and not in the bootloader
    /// walk.
    Missing,
    /// Reached its bootloader but is not on the roster after the session.
    NotBack,
    /// A bootloader-only board not installed: it has no app to go to.
    Waiting,
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
    /// Roster servos still answering after two REBOOTs. Non-empty means
    /// the session aborted before any install.
    pub stayed_app: Vec<Found>,
}

/// Install `image` on each `chosen` tinyboot id in one session with the
/// whole fleet in its bootloader at `rate` (an osc rate, not
/// [`super::BaudRate`]). The bus starts and ends in osc mode at its
/// current rate. Per-device failures land in the [`Report`]; `Err` is the
/// link itself, after one try to return the bus to osc mode.
pub async fn update<P: Pipe>(
    c: &mut Client<P>,
    image: &Image,
    chosen: &[Vec<u8>],
    rate: BaudRate,
    mut progress: impl FnMut(Progress<'_>),
) -> Result<Report, Error> {
    use osc_host::link::record::MODE_OSC;
    if c.info().mode != MODE_OSC {
        return Err(Error::Link(LinkError::Rejected(RejectReason::WrongMode)));
    }
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
    chosen: &[Vec<u8>],
    rate: BaudRate,
    fleet: u32,
    progress: &mut impl FnMut(Progress<'_>),
) -> Result<Report, Error> {
    progress(Progress::Phase(Phase::Roster));
    let roster = mgmt::discover(c).await?;

    progress(Progress::Phase(Phase::Reboot));
    let mut apps = Vec::new();
    for f in &roster {
        reboot(c, f.id, rate).await?;
    }
    for f in &roster {
        if answers(c, f.id).await? {
            reboot(c, f.id, rate).await?;
            if answers(c, f.id).await? {
                apps.push(*f);
            }
        }
    }
    let mut report = Report {
        devices: Vec::new(),
        stayed_app: apps,
    };
    let waiting: Vec<&Found> = roster
        .iter()
        .filter(|f| !report.stayed_app.contains(f))
        .collect();
    let on_roster = |uid: &[u8]| waiting.iter().find(|f| chip_uid(f) == uid).map(|f| f.id);

    c.set_bus(BusMode::Tinyboot, rate.as_hz()).await?;
    // Roster servos the walk missed, then every install that ended.
    let mut done: Vec<&[u8]> = Vec::new();
    let mut installed: Vec<&[u8]> = Vec::new();
    if report.stayed_app.is_empty() {
        progress(Progress::Phase(Phase::Walk));
        let walk = super::discover(c).await?;
        for f in &waiting {
            if !walk.iter().any(|u| u == chip_uid(f)) {
                done.push(chip_uid(f));
                report
                    .devices
                    .push(device(chip_uid(f), Some(f.id), Fate::Missing));
            }
        }

        progress(Progress::Phase(Phase::Install));
        for (i, uid) in chosen.iter().enumerate() {
            if chosen[..i].contains(uid) {
                continue;
            }
            if !walk.contains(uid) {
                // A roster servo is already reported missing.
                if on_roster(uid).is_none() {
                    report.devices.push(device(uid, None, Fate::NotFound));
                }
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
                Ok(()) => installed.push(uid),
                Err(e) if per_device(&e) => {
                    done.push(uid);
                    report
                        .devices
                        .push(device(uid, on_roster(uid), Fate::Failed(e)));
                }
                Err(e) => return Err(e),
            }
        }
        for uid in &walk {
            let uid = uid.as_slice();
            if on_roster(uid).is_none() && !installed.contains(&uid) && !done.contains(&uid) {
                report.devices.push(device(uid, None, Fate::Waiting));
            }
        }
    }

    progress(Progress::Phase(Phase::Reset));
    super::reset_all(c).await?;
    c.set_bus(BusMode::Osc, fleet).await?;
    c.pause(APP_UP).await;

    progress(Progress::Phase(Phase::Confirm));
    let after = mgmt::discover(c).await?;
    for f in &waiting {
        let uid = chip_uid(f);
        if done.contains(&uid) {
            continue;
        }
        let fate = match after.iter().find(|g| chip_uid(g) == uid) {
            None => Fate::NotBack,
            Some(g) if installed.contains(&uid) => confirm(c, &after, g, image.version()).await?,
            Some(_) => continue,
        };
        report.devices.push(device(uid, Some(f.id), fate));
    }
    for uid in installed {
        if on_roster(uid).is_none() {
            report
                .devices
                .push(device(uid, None, Fate::Unconfirmed(None)));
        }
    }
    Ok(report)
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

/// Whether the ping drew anything: only silence counts as gone.
async fn answers<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<bool, Error> {
    match c.ping(id).await {
        Err(Error::Timeout { .. }) => Ok(false),
        Err(e) if !per_device(&e) => Err(e),
        _ => Ok(true),
    }
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
async fn confirm<P: Pipe>(
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
