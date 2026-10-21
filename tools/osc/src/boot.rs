//! Firmware update (link sec 5): `osc boot` steps one at a time, for fresh
//! boards and recovery, and `osc flash`, the whole session over the running
//! fleet. `osc tb`: one raw tinyboot exchange (link sec 7) for the bench.

use std::io::Write as _;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::ValueEnum;
use osc_client::blocking::Client;
use osc_client::boot::{self, BootDevice, CHIP_UID, Chosen, Fate, Image, Phase, Progress, Report};
use osc_client::mgmt::Found;
use osc_client::nusb::NusbPipe;
use osc_client::{BaudRate, BusMode, DEFAULT_GUARD, Error, Id, TbExpect, TbReply};

use crate::{hex, parse_hex, parse_rate, rig};

#[derive(clap::Subcommand, Debug)]
pub(crate) enum BootCmd {
    /// Every servo on the roster at the bus rate to its bootloader at
    /// --rate; prints the roster.
    Enter {
        /// Bootloader rate: an osc rate.
        #[arg(long)]
        rate: String,
    },
    /// Every running app at any rate. Exit 0 only when none answers.
    Check,
    /// The bootloader walk and each device's Info.
    List {
        /// Bus rate in tinyboot mode, bps.
        #[arg(long)]
        rate: u32,
    },
    /// Install an image on each named bootloader, in order. No reset.
    Flash {
        /// Raw `.bin` app image (objcopy of the ELF).
        file: PathBuf,
        /// Bus rate in tinyboot mode, bps.
        #[arg(long)]
        rate: u32,
        /// Bootloader ids as `boot list` prints them.
        #[arg(required = true)]
        uids: Vec<String>,
    },
    /// Reset every bootloader to its app at once.
    Reset {
        /// Bus rate in tinyboot mode, bps.
        #[arg(long)]
        rate: u32,
    },
}

#[derive(clap::Args, Debug)]
pub(crate) struct FlashArgs {
    /// Raw `.bin` app image (objcopy of the ELF).
    pub(crate) file: PathBuf,
    /// Bootloader rate for the session: an osc rate.
    #[arg(long)]
    pub(crate) rate: String,
    /// Roster servos to install; every one when none.
    pub(crate) ids: Vec<u8>,
}

/// Generous enough for an erase of the whole app region.
const TB_WINDOW_US: u64 = 2_000_000;

#[derive(clap::Args, Debug)]
pub(crate) struct TbArgs {
    /// Bus rate in tinyboot mode, bps.
    #[arg(long)]
    pub(crate) rate: u32,
    pub(crate) cmd: TbCmd,
    /// Request address (0x-hex or decimal, 24 bits).
    #[arg(long, default_value = "0")]
    pub(crate) addr: String,
    #[arg(long, default_value_t = 0)]
    pub(crate) flags: u8,
    /// Request payload as hex bytes.
    #[arg(long, default_value = "")]
    pub(crate) data: String,
    #[arg(long, value_enum, default_value_t = Expect::One)]
    pub(crate) expect: Expect,
    /// Reply window in us; unused with `--expect none`.
    #[arg(long, default_value_t = TB_WINDOW_US)]
    pub(crate) window_us: u64,
}

#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TbCmd {
    Info,
    Select,
    Erase,
    Write,
    Verify,
    Reset,
}

impl TbCmd {
    fn byte(self) -> u8 {
        (match self {
            TbCmd::Info => boot::Cmd::Info,
            TbCmd::Select => boot::Cmd::Select,
            TbCmd::Erase => boot::Cmd::Erase,
            TbCmd::Write => boot::Cmd::Write,
            TbCmd::Verify => boot::Cmd::Verify,
            TbCmd::Reset => boot::Cmd::Reset,
        }) as u8
    }

    fn name(b: u8) -> String {
        Self::value_variants()
            .iter()
            .find(|c| c.byte() == b)
            .and_then(|c| c.to_possible_value())
            .map_or_else(|| format!("{b:#04x}"), |v| v.get_name().to_string())
    }
}

#[derive(ValueEnum, Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Expect {
    None,
    One,
    Collect,
}

pub(crate) fn flash(a: &FlashArgs, bus: &rig::Bus) -> Result<()> {
    let rate = parse_rate(&a.rate)?;
    let image = read_image(&a.file)?;
    let chosen = if a.ids.is_empty() {
        Chosen::All
    } else {
        Chosen::Ids(a.ids.iter().map(|&i| Id::new(i)).collect())
    };
    let mut c = rig::connect(bus, &[])?;
    println!(
        "image: {} B, fw {}; bootloaders at {}",
        image.bytes().len(),
        image.version(),
        rate.as_hz()
    );
    let mut line = Line::default();
    let report = c.boot_update(&image, &chosen, rate, |p| line.progress(p))?;
    line.end();
    let (lines, ok) = report_lines(&report);
    for l in &lines {
        println!("{l}");
    }
    if !ok {
        bail!("update incomplete");
    }
    Ok(())
}

pub(crate) fn boot(cmd: &BootCmd, bus: &rig::Bus) -> Result<()> {
    match cmd {
        BootCmd::Enter { rate } => {
            let rate = parse_rate(rate)?;
            for f in rig::connect(bus, &[])?.boot_enter(rate)? {
                println!("{}", roster_line(&f));
            }
            println!("bootloaders at {} baud", rate.as_hz());
            Ok(())
        }
        BootCmd::Check => {
            let mut c = Client::connect(NusbPipe::open()?)?;
            c.sync_response_deadline(&[], bus.deadline)?;
            let apps = c.boot_apps()?;
            if apps.is_empty() {
                println!("all quiet: no app answers at any rate");
                return Ok(());
            }
            for (at, f) in &apps {
                println!("{}", app_line(*at, f));
            }
            bail!("{} app(s) answering", apps.len());
        }
        BootCmd::List { rate } => {
            let devices = tinyboot(&mut Client::connect(NusbPipe::open()?)?, *rate, |c| {
                Ok(c.boot_list()?)
            })?;
            if devices.is_empty() {
                println!("no bootloader answers at {rate}");
            }
            for d in &devices {
                println!("{}", list_line(d));
            }
            Ok(())
        }
        BootCmd::Flash { file, rate, uids } => {
            let image = read_image(file)?;
            let uids: Vec<Vec<u8>> = uids
                .iter()
                .map(|u| parse_boot_uid(u))
                .collect::<Result<_>>()?;
            let failed = tinyboot(&mut Client::connect(NusbPipe::open()?)?, *rate, |c| {
                let mut failed = 0;
                for uid in &uids {
                    let mut line = Line::default();
                    let r = c.boot_install(uid, &image, |written, total| {
                        line.progress(Progress::Bytes {
                            uid,
                            written,
                            total,
                        })
                    });
                    line.end();
                    match r {
                        Ok(()) => println!("uid {}: installed", uid_hex(uid)),
                        Err(e @ (Error::Pipe(_) | Error::Link(_))) => return Err(e.into()),
                        Err(e) => {
                            println!("uid {}: FAILED: {e}", uid_hex(uid));
                            failed += 1;
                        }
                    }
                }
                Ok(failed)
            })?;
            if failed > 0 {
                bail!("{failed} install(s) failed");
            }
            Ok(())
        }
        BootCmd::Reset { rate } => {
            tinyboot(&mut Client::connect(NusbPipe::open()?)?, *rate, |c| {
                Ok(c.boot_reset_all()?)
            })?;
            println!("reset sent to every bootloader at {rate}");
            Ok(())
        }
    }
}

/// Run `f` with the bus in tinyboot mode at `rate`, then return it to osc
/// mode at the session's rate whatever `f` returned.
fn tinyboot<T>(
    c: &mut Client<NusbPipe>,
    rate: u32,
    f: impl FnOnce(&mut Client<NusbPipe>) -> Result<T>,
) -> Result<T> {
    let osc = c.info().bps;
    c.set_bus(BusMode::Tinyboot, rate)?;
    let r = f(c);
    let back = c.set_bus(BusMode::Osc, osc);
    let r = r?;
    back?;
    Ok(r)
}

fn read_image(file: &std::path::Path) -> Result<Image> {
    let bytes = std::fs::read(file).with_context(|| format!("read {}", file.display()))?;
    Image::parse(bytes).with_context(|| format!("{}", file.display()))
}

/// A bootloader id as printed (most significant byte first).
fn parse_boot_uid(s: &str) -> Result<Vec<u8>> {
    let mut uid = parse_hex(s)?;
    if uid.is_empty() || uid.len() > 16 {
        bail!("a bootloader id is 1 to 16 hex bytes");
    }
    uid.reverse();
    Ok(uid)
}

/// A running app: its rate, osc id and bootloader id.
fn app_line(at: BaudRate, f: &Found) -> String {
    format!("{:>8}: {}", at.as_hz(), roster_line(f))
}

/// A roster servo: its osc id and bootloader id.
fn roster_line(f: &Found) -> String {
    format!(
        "id {:<3} boot uid {}",
        f.id.as_byte(),
        uid_hex(&f.uid.0[..CHIP_UID])
    )
}

fn list_line(d: &BootDevice) -> String {
    let app = match d.info.app_version {
        0xFFFF => "no app".to_string(),
        v => version(v),
    };
    format!(
        "uid {}  compat {:#010x}  app {app}  boot {}",
        uid_hex(&d.uid),
        d.info.compat,
        version(d.info.boot_version)
    )
}

/// Packed 5.5.6, `0xFFFF` none.
fn version(v: u16) -> String {
    if v == 0xFFFF {
        return "none".into();
    }
    let (major, minor, patch) = boot::unpack_version(v);
    format!("{major}.{minor}.{patch}")
}

/// The progress output: phases on their own lines, each install's bytes on
/// one line rewritten in place.
#[derive(Default)]
struct Line {
    open: bool,
}

impl Line {
    fn progress(&mut self, p: Progress<'_>) {
        match p {
            Progress::Phase(p) => {
                self.end();
                println!("{}", phase(p));
            }
            Progress::Bytes {
                uid,
                written,
                total,
            } => {
                print!("\r  {}: {written}/{total} B", uid_hex(uid));
                let _ = std::io::stdout().flush();
                self.open = true;
                if written == total {
                    self.end();
                }
            }
        }
    }

    fn end(&mut self) {
        if std::mem::take(&mut self.open) {
            println!();
        }
    }
}

fn phase(p: Phase) -> &'static str {
    match p {
        Phase::Enter => "every roster servo to its bootloader",
        Phase::Walk => "bootloader walk",
        Phase::Install => "install",
        Phase::Reset => "every bootloader back to its app",
        Phase::Confirm => "confirm",
    }
}

/// Most significant byte first, as `discover` prints an osc UID.
fn uid_hex(uid: &[u8]) -> String {
    uid.iter().rev().map(|b| format!("{b:02x}")).collect()
}

fn fate(f: &Fate) -> String {
    match f {
        Fate::Confirmed => "installed, version confirmed".into(),
        Fate::Unconfirmed(Some(fw)) => format!("installed, but the app reports fw {fw}"),
        Fate::Unconfirmed(None) => "installed, version unread (no single osc id)".into(),
        Fate::Failed(e) => format!("FAILED: {e}"),
        Fate::NotFound => "NOT FOUND: not on the roster".into(),
        Fate::Missing => "MISSING: silent after its reboot, not in the bootloader walk".into(),
        Fate::NotBack => "NOT BACK: reached its bootloader, absent after the session".into(),
        Fate::Waiting => "WAITING: a bootloader off the roster, left alone (osc boot flash)".into(),
    }
}

/// One line per device, and whether every device the report names is
/// confirmed.
fn report_lines(r: &Report) -> (Vec<String>, bool) {
    let mut lines: Vec<String> = r
        .devices
        .iter()
        .map(|d| {
            let uid = if d.uid.is_empty() {
                "-".into()
            } else {
                uid_hex(&d.uid)
            };
            let id = d.id.map_or("-".into(), |i| i.as_byte().to_string());
            format!("uid {uid:<24}  id {id:<3}  {}", fate(&d.fate))
        })
        .collect();
    let ok = r.devices.iter().all(|d| d.fate == Fate::Confirmed);
    if lines.is_empty() {
        lines.push("nothing installed".into());
    }
    (lines, ok)
}

pub(crate) fn tb(a: &TbArgs) -> Result<()> {
    let addr = parse_u32(&a.addr)?;
    let data = parse_hex(&a.data)?;
    let (expect, window) = match a.expect {
        Expect::None => (TbExpect::None, Duration::ZERO),
        Expect::One => (TbExpect::One, Duration::from_micros(a.window_us)),
        Expect::Collect => (TbExpect::Collect, Duration::from_micros(a.window_us)),
    };
    let mut c = Client::connect(NusbPipe::open()?)?;
    c.set_guard(window + DEFAULT_GUARD);
    let reply = tinyboot(&mut c, a.rate, |c| {
        Ok(c.tb_exchange(expect, window, a.cmd.byte(), addr, a.flags, &data)?)
    })?;
    for l in tb_lines(&reply) {
        println!("{l}");
    }
    Ok(())
}

fn tb_lines(r: &TbReply) -> Vec<String> {
    let mut lines: Vec<String> = r
        .frames
        .iter()
        .map(|f| {
            let status = boot::status(f.status)
                .map_or_else(|| format!("{:#04x}", f.status), |s| format!("{s:?}"));
            format!(
                "slot {:<2} cmd {} status {status} addr {:#08x} flags {:#04x} data {}",
                f.slot,
                TbCmd::name(f.cmd),
                f.addr,
                f.flags,
                hex(&f.data)
            )
        })
        .collect();
    lines.push(format!(
        "{} frame(s), {:?}, garble {}, trailing {}",
        r.frames.len(),
        r.outcome,
        r.garble,
        r.trailing
    ));
    lines
}

fn parse_u32(s: &str) -> Result<u32> {
    Ok(match s.strip_prefix("0x") {
        Some(h) => u32::from_str_radix(h, 16)?,
        None => s.parse()?,
    })
}

#[cfg(test)]
mod tests {
    use clap::Parser;
    use osc_client::boot::Device;
    use osc_client::mgmt::{Found, Uid};
    use osc_client::{Error, Outcome, TbFrame};

    use super::*;
    use crate::{Cli, Cmd};

    fn parse(argv: &[&str]) -> Result<Cmd, clap::Error> {
        Cli::try_parse_from(argv).map(|c| c.cmd)
    }

    #[test]
    fn flash_takes_a_file_a_required_rate_and_optional_ids() {
        let Ok(Cmd::Flash(a)) = parse(&["osc", "flash", "app.bin", "--rate", "500000"]) else {
            panic!("flash parses");
        };
        assert_eq!(
            (a.file, a.rate, a.ids),
            ("app.bin".into(), "500000".into(), vec![])
        );
        let Ok(Cmd::Flash(a)) = parse(&["osc", "flash", "app.bin", "--rate", "1000000", "3", "7"])
        else {
            panic!("flash parses ids");
        };
        assert_eq!(a.ids, [3, 7]);
        let e = parse(&["osc", "flash", "app.bin", "1"]).unwrap_err();
        assert_eq!(e.kind(), clap::error::ErrorKind::MissingRequiredArgument);
    }

    #[test]
    fn tb_takes_a_command_name_and_defaults() {
        let Ok(Cmd::Tb(a)) = parse(&["osc", "tb", "--rate", "115200", "info"]) else {
            panic!("tb parses");
        };
        assert_eq!(
            (a.rate, a.cmd, a.addr.as_str(), a.flags, a.data.as_str()),
            (115_200, TbCmd::Info, "0", 0, "")
        );
        assert_eq!((a.expect, a.window_us), (Expect::One, TB_WINDOW_US));
        let Ok(Cmd::Tb(a)) = parse(&[
            "osc",
            "tb",
            "--rate",
            "500000",
            "select",
            "--data",
            "0000",
            "--expect",
            "collect",
            "--window-us",
            "800",
        ]) else {
            panic!("tb parses options");
        };
        assert_eq!(
            (a.cmd, a.data.as_str(), a.expect, a.window_us),
            (TbCmd::Select, "0000", Expect::Collect, 800)
        );
        let e = parse(&["osc", "tb", "info"]).unwrap_err();
        assert_eq!(e.kind(), clap::error::ErrorKind::MissingRequiredArgument);
        assert!(parse(&["osc", "tb", "--rate", "500000", "boot"]).is_err());
    }

    #[test]
    fn tb_command_bytes_are_tinyboots() {
        let bytes: Vec<u8> = TbCmd::value_variants().iter().map(|c| c.byte()).collect();
        assert_eq!(bytes, [0x00, 0x05, 0x01, 0x02, 0x03, 0x04]);
        assert_eq!(TbCmd::name(0x05), "select");
        assert_eq!(TbCmd::name(0x09), "0x09");
    }

    fn dev(uid: &[u8], id: Option<u8>, fate: Fate) -> Device {
        Device {
            uid: uid.to_vec(),
            id: id.map(Id::new),
            fate,
        }
    }

    #[test]
    fn report_names_each_device_and_its_fate() {
        let (a, b) = ([0x11, 0x22, 0x33], [0xab, 0xcd, 0xef]);
        let r = Report {
            devices: vec![
                dev(&a, Some(1), Fate::Confirmed),
                dev(&b, None, Fate::Unconfirmed(None)),
                dev(&[], Some(7), Fate::NotFound),
                dev(
                    &[0x01],
                    Some(2),
                    Fate::Failed(Error::Boot(boot::Status::WriteError)),
                ),
            ],
        };
        let (lines, ok) = report_lines(&r);
        assert_eq!(
            lines,
            [
                "uid 332211                    id 1    installed, version confirmed",
                "uid efcdab                    id -    installed, version unread (no single osc id)",
                "uid -                         id 7    NOT FOUND: not on the roster",
                "uid 01                        id 2    FAILED: bootloader answered WriteError",
            ]
        );
        assert!(!ok);
        let r = Report {
            devices: vec![dev(&a, Some(1), Fate::Confirmed)],
        };
        assert!(report_lines(&r).1);
    }

    fn found() -> Found {
        let mut uid = [0xee; 16];
        for (i, b) in uid[..12].iter_mut().enumerate() {
            *b = i as u8 + 1;
        }
        Found {
            uid: Uid(uid),
            id: Id::new(2),
        }
    }

    #[test]
    fn boot_steps_parse_with_their_rates() {
        let Ok(Cmd::Boot {
            cmd: BootCmd::Flash { file, rate, uids },
        }) = parse(&[
            "osc", "boot", "flash", "app.bin", "--rate", "500000", "0a0b", "0c",
        ])
        else {
            panic!("boot flash parses");
        };
        assert_eq!(
            (file, rate, uids),
            ("app.bin".into(), 500_000, vec!["0a0b".into(), "0c".into()])
        );
        let no_uid = parse(&["osc", "boot", "flash", "app.bin", "--rate", "500000"]);
        assert_eq!(
            no_uid.unwrap_err().kind(),
            clap::error::ErrorKind::MissingRequiredArgument
        );
        for step in ["enter", "list", "reset"] {
            let e = parse(&["osc", "boot", step]).unwrap_err();
            assert_eq!(
                e.kind(),
                clap::error::ErrorKind::MissingRequiredArgument,
                "{step}"
            );
        }
        assert!(matches!(
            parse(&["osc", "boot", "check"]),
            Ok(Cmd::Boot {
                cmd: BootCmd::Check
            })
        ));
        assert!(matches!(
            parse(&["osc", "boot", "enter", "--rate", "1000000"]),
            Ok(Cmd::Boot { cmd: BootCmd::Enter { rate } }) if rate == "1000000"
        ));
    }

    #[test]
    fn boot_uid_reads_most_significant_byte_first() {
        assert_eq!(parse_boot_uid("0a0b0c").unwrap(), [0x0c, 0x0b, 0x0a]);
        assert!(parse_boot_uid("").is_err());
        assert!(parse_boot_uid(&"00".repeat(17)).is_err());
    }

    #[test]
    fn roster_and_list_lines() {
        assert_eq!(
            roster_line(&found()),
            "id 2   boot uid 0c0b0a090807060504030201"
        );
        assert_eq!(
            app_line(BaudRate::B3000000, &found()),
            " 3000000: id 2   boot uid 0c0b0a090807060504030201"
        );
        let info = |app_version| boot::BootInfo {
            capacity: 58_112,
            erase_size: 256,
            boot_version: 0x0840,
            app_version,
            mode: 0,
            boot_baud: 0xFF,
            compat: 0x1234_abcd,
            max_payload: 64,
        };
        let d = |app_version| BootDevice {
            uid: vec![0x01, 0x02, 0x03],
            info: info(app_version),
        };
        assert_eq!(
            list_line(&d(0x0883)),
            "uid 030201  compat 0x1234abcd  app 1.2.3  boot 1.1.0"
        );
        assert_eq!(
            list_line(&d(0xFFFF)),
            "uid 030201  compat 0x1234abcd  app no app  boot 1.1.0"
        );
    }

    #[test]
    fn tb_prints_each_frame_and_the_garble() {
        let r = TbReply {
            frames: vec![TbFrame {
                slot: 3,
                cmd: 0x05,
                status: 0x01,
                addr: 0x0400,
                flags: 0,
                data: vec![0xde, 0xad],
            }],
            outcome: Outcome::Complete,
            tick: 0,
            garble: 2,
            trailing: false,
            laps: 0,
        };
        assert_eq!(
            tb_lines(&r),
            [
                "slot 3  cmd select status Ok addr 0x000400 flags 0x00 data de ad",
                "1 frame(s), Complete, garble 2, trailing false",
            ]
        );
    }
}
