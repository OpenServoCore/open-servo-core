//! The firmware update steps `OscClient` runs one call each, in
//! `osc_client::boot::update`'s order: [`enter`] in osc mode, [`list`] into
//! tinyboot, [`install`] per device, [`reset_all`] back to osc, then
//! `discover` and `identity` to confirm. Generic over the pipe, so the fake
//! runs them natively.

use osc_client::boot::{self, Image};
use osc_client::pipe::Pipe;
use osc_client::{BusMode, Client, Error};

use crate::types::{BaudRate, BootRow, Found, InstallVerdict};

pub async fn enter<P: Pipe>(c: &mut Client<P>, rate: BaudRate) -> Result<Vec<Found>, Error> {
    let roster = boot::enter(c, rate.into()).await?;
    Ok(roster.iter().map(Found::from).collect())
}

pub async fn list<P: Pipe>(c: &mut Client<P>, rate: BaudRate) -> Result<Vec<BootRow>, Error> {
    c.set_bus(BusMode::Tinyboot, hz(rate)).await?;
    Ok(boot::list(c).await?.iter().map(BootRow::from).collect())
}

/// A verdict on the device resolves; anything else is the link's `Err`.
pub async fn install<P: Pipe>(
    c: &mut Client<P>,
    uid: &[u8],
    image: &Image,
    progress: impl FnMut(u32, u32),
) -> Result<InstallVerdict, Error> {
    match boot::install(c, uid, image, progress).await {
        Ok(()) => Ok(InstallVerdict::Installed),
        Err(Error::Install(e)) => Ok(e.into()),
        Err(Error::Boot(s)) => Ok(InstallVerdict::Status { status: s.into() }),
        Err(e) => Err(e),
    }
}

/// Resolves once the apps are up: they are in bringup for up to
/// [`boot::APP_UP`] after the reset, and a walk before then misses them.
pub async fn reset_all<P: Pipe>(c: &mut Client<P>, rate: BaudRate) -> Result<(), Error> {
    boot::reset_all(c).await?;
    c.set_bus(BusMode::Osc, hz(rate)).await?;
    c.pause(boot::APP_UP).await;
    Ok(())
}

fn hz(rate: BaudRate) -> u32 {
    osc_client::BaudRate::from(rate).as_hz()
}

#[cfg(all(test, feature = "fake"))]
mod tests {
    use super::*;
    use futures_lite::future::block_on;
    use osc_client::common;
    use osc_client::fake::FakePipe;
    use osc_client::{Id, mgmt};
    use osc_protocol::version::pack_version;

    use crate::types::{BootStatus, FakeFail, FakeServo, ImageCheck, Version};

    fn fleet(servos: Vec<FakeServo>) -> Client<FakePipe> {
        let pipe = crate::fake::pipe(servos).expect("fleet");
        block_on(Client::connect(pipe)).expect("connect")
    }

    fn servo(id: u8) -> FakeServo {
        FakeServo {
            id,
            ..Default::default()
        }
    }

    /// `len` bytes ending in the tinyboot tail `[compat, version]`.
    fn image(len: usize, compat: u32, version: u16) -> Image {
        let mut b: Vec<u8> = (0..len).map(|i| (i * 7) as u8).collect();
        b[len - 6..len - 2].copy_from_slice(&compat.to_le_bytes());
        b[len - 2..].copy_from_slice(&version.to_le_bytes());
        Image::parse(b).expect("image")
    }

    fn fw(c: &mut Client<FakePipe>, id: u8) -> u16 {
        block_on(common::identity(c, Id::new(id)))
            .expect("identity")
            .fw
    }

    fn row<'a>(rows: &'a [BootRow], uid: &str) -> &'a BootRow {
        rows.iter().find(|r| uid.ends_with(&r.uid)).expect("listed")
    }

    /// The dialog's buttons in order: two app servos and a bootloader-only
    /// board listed, the board and one servo installed, every bootloader
    /// reset, and the rediscovered fleet reporting the new version where
    /// it was installed.
    #[test]
    fn the_whole_procedure_installs_and_confirms() {
        let mut board = servo(3);
        board.bootloader = true;
        let mut c = fleet(vec![servo(1), servo(2), board]);
        let old = fw(&mut c, 2);
        let rate = BaudRate::B500000;

        let roster = block_on(enter(&mut c, rate)).expect("enter");
        assert_eq!(roster.iter().map(|f| f.id).collect::<Vec<_>>(), [1, 2]);

        let rows = block_on(list(&mut c, rate)).expect("list");
        assert_eq!(rows.len(), 3);
        let servo_rows: Vec<&BootRow> = roster.iter().map(|f| row(&rows, &f.uid)).collect();
        let board_row = rows
            .iter()
            .find(|r| !servo_rows.contains(r))
            .expect("the board")
            .clone();
        assert_eq!(board_row.uid.len(), 24);
        assert_eq!(board_row.app_version, None);
        assert!(board_row.capacity > 0);

        let v = pack_version(1, 2, 3);
        let img = image(1002, board_row.compat, v);
        for uid in [&board_row.uid, &servo_rows[0].uid] {
            let uid = crate::uid::boot_parse(uid).expect("uid");
            let mut last = (0, 0);
            let verdict = block_on(install(&mut c, &uid, &img, |w, t| last = (w, t)));
            assert_eq!(verdict, Ok(InstallVerdict::Installed));
            assert_eq!(last, (1002, 1002));
        }
        let rows = block_on(list(&mut c, rate)).expect("list again");
        assert_eq!(
            row(&rows, &board_row.uid).app_version,
            Some(Version(1, 2, 3))
        );

        block_on(reset_all(&mut c, BaudRate::B1000000)).expect("reset");
        let found = block_on(mgmt::discover(&mut c)).expect("discover");
        let ids: Vec<u8> = found.iter().map(|f| f.id.as_byte()).collect();
        assert_eq!(ids, [1, 2, 3]);
        assert_eq!([fw(&mut c, 1), fw(&mut c, 2), fw(&mut c, 3)], [v, old, v]);
    }

    #[test]
    fn a_failed_install_resolves_to_its_reason() {
        let mut failing = servo(1);
        failing.fail_next = Some(FakeFail::Write);
        let mut c = fleet(vec![failing]);
        block_on(enter(&mut c, BaudRate::B500000)).expect("enter");
        let rows = block_on(list(&mut c, BaudRate::B500000)).expect("list");
        let uid = crate::uid::boot_parse(&rows[0].uid).expect("uid");
        let compat = rows[0].compat;

        let write_error = InstallVerdict::Status {
            status: BootStatus::WriteError,
        };
        let installs = |c: &mut Client<FakePipe>, img: &Image| {
            block_on(install(c, &uid, img, |_, _| {})).expect("link")
        };
        assert_eq!(installs(&mut c, &image(1002, compat, 7)), write_error);
        assert_eq!(
            installs(&mut c, &image(1002, compat ^ 1, 7)),
            InstallVerdict::Incompatible {
                image: compat ^ 1,
                device: compat,
            }
        );
        assert_eq!(
            installs(&mut c, &image(58_114, compat, 7)),
            InstallVerdict::TooLarge {
                size: 58_114,
                capacity: 58_112,
            }
        );
        assert_eq!(
            block_on(install(
                &mut c,
                &[0x43; 12],
                &image(26, compat, 7),
                |_, _| {}
            )),
            Ok(InstallVerdict::NoDevice)
        );
        assert_eq!(
            installs(&mut c, &image(1002, compat, 7)),
            InstallVerdict::Installed
        );
    }

    #[test]
    fn a_failed_erase_resolves_to_a_write_error() {
        let mut failing = servo(1);
        failing.fail_next = Some(FakeFail::Erase);
        failing.bootloader = true;
        let mut c = fleet(vec![failing]);
        let rows = block_on(list(&mut c, BaudRate::B500000)).expect("list");
        let uid = crate::uid::boot_parse(&rows[0].uid).expect("uid");
        let img = image(26, rows[0].compat, 7);
        assert_eq!(
            block_on(install(&mut c, &uid, &img, |_, _| {})),
            Ok(InstallVerdict::Status {
                status: BootStatus::WriteError,
            })
        );
    }

    #[test]
    fn image_check_reads_the_tail_or_names_the_refusal() {
        let img = image(1002, 0x1234_5678, pack_version(1, 2, 3));
        assert_eq!(
            ImageCheck::from(Image::parse(img.bytes().to_vec())),
            ImageCheck::Ok {
                compat: 0x1234_5678,
                version: Version(1, 2, 3),
                size: 1002,
            }
        );
        let mut elf = b"\x7FELF".to_vec();
        elf.resize(26, 0);
        assert_eq!(ImageCheck::from(Image::parse(elf)), ImageCheck::Elf);
        assert_eq!(
            ImageCheck::from(Image::parse(vec![0; 5])),
            ImageCheck::TooShort { len: 5 }
        );
        assert_eq!(
            ImageCheck::from(Image::parse(vec![0; 24])),
            ImageCheck::Misaligned { len: 24 }
        );
    }
}
