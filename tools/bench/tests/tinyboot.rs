//! Boot-mode + tinyboot round-trip. Separate binary from the protocol
//! suite so its reboots don't interfere with the shared-bus discipline
//! there. Config via `BENCH_PORT` + `BENCH_BAUD` (see `BusArgs::from_env`).
//!
//! Ported from `tools/dxl-bench/test_system_tinyboot.py` and
//! `tinyboot_frame.py`. Tinyboot frames are hand-rolled here (mirror of
//! `tinyboot/lib/protocol/src/{frame,crc}.rs`) rather than pulled in as a
//! git dep — two frames don't justify the coupling.

use std::thread::sleep;
use std::time::Duration;

use bench::{BOOT_BAUD, Bus, BusArgs, DEFAULT_IDLE_US, build_ping};
use dxl_protocol::types::Id;
use serial_test::serial;

const CONTROL_BASE_ADDR: u16 = 0x0300;
const BOOT_MODE_ADDR: u16 = 0x031C;
const BOOT_MODE_BOOTLOADER: u8 = 1;
const TINYBOOT_MODE_BOOTLOADER: u16 = 0;

/// Tinyboot listens at 3M — mirrors `boot/src/main.rs`'s USART config
/// (`BaudRate::B3000000`). Keep in lockstep with that file.
const TINYBOOT_BAUD: u32 = 3_000_000;

fn bus() -> Bus {
    let args = BusArgs::from_env().expect("parse BENCH_* env");
    Bus::start(args).expect("Bus::start (no pirate/servo attached?)")
}

#[test]
#[serial]
fn reboot_clears_volatile_state() {
    let mut bus = bus();
    let id = Id::new(bus.id());
    let app_baud = bus.baud();

    bus.write_register(id, CONTROL_BASE_ADDR, &[1])
        .expect("write torque_enable=1");
    let pre = bus
        .read_register(id, CONTROL_BASE_ADDR, 1)
        .expect("read torque_enable");
    assert_eq!(pre, vec![1u8], "torque_enable did not persist pre-reboot");

    bus.reboot_chip().expect("reboot");

    // reboot_chip dropped us to BOOT_BAUD + drained the ring; an explicit
    // Ping catches a partial-boot scenario the drain would miss.
    let ping = bus
        .xfer_reply(&build_ping(id).unwrap(), DEFAULT_IDLE_US)
        .expect("post-reboot ping");
    assert!(ping.is_some(), "chip silent post-reboot");

    let post = bus
        .read_register(id, CONTROL_BASE_ADDR, 1)
        .expect("read torque_enable post-reboot");
    assert_eq!(
        post,
        vec![0u8],
        "reboot did not clear volatile torque_enable",
    );

    if app_baud != BOOT_BAUD {
        bus.set_chip_baud(app_baud).expect("restore app baud");
    }
}

#[test]
#[serial]
fn tinyboot_round_trip() {
    let mut bus = bus();
    let id = Id::new(bus.id());
    let app_baud = bus.baud();

    // Set boot mode = bootloader, reboot into it.
    bus.write_register(id, BOOT_MODE_ADDR, &[BOOT_MODE_BOOTLOADER])
        .expect("write BOOT_MODE=bootloader");
    let _ = bus
        .xfer_reply(&bench::build_reboot(id).unwrap(), DEFAULT_IDLE_US)
        .expect("reboot");
    sleep(Duration::from_millis(300));

    bus.pirate_set_baud(TINYBOOT_BAUD)
        .expect("switch pirate to tinyboot baud");

    let info_reply = bus
        .xfer_reply(&info_request(), 500_000)
        .expect("tinyboot info xfer")
        .expect("no Info reply from bootloader");
    let info = parse_info(&info_reply).expect("parse InfoResponse");
    assert_eq!(
        info.mode, TINYBOOT_MODE_BOOTLOADER,
        "expected mode=bootloader(0), got {} — chip is not in tinyboot",
        info.mode,
    );
    assert!(info.capacity > 0, "bogus capacity {}", info.capacity);
    assert!(info.erase_size > 0, "bogus erase_size {}", info.erase_size);

    // Reset back to app (flags=0). Reply may or may not land in the 50ms
    // window before the chip jumps — either is fine.
    let _ = bus.xfer_reply(&reset_request(false), 50_000);
    sleep(Duration::from_millis(500));

    bus.pirate_set_baud(BOOT_BAUD)
        .expect("switch pirate to app baud");
    let ping = bus
        .xfer_reply(&build_ping(id).unwrap(), DEFAULT_IDLE_US)
        .expect("post-reset ping");
    assert!(ping.is_some(), "chip silent post-tinyboot-reset");

    if app_baud != BOOT_BAUD {
        bus.set_chip_baud(app_baud).expect("restore app baud");
    }
}

// ─── tinyboot frame builder + parser ───────────────────────────────────────

const SYNC: [u8; 2] = [0xAA, 0x55];
const CMD_INFO: u8 = 0x00;
const CMD_RESET: u8 = 0x04;
const STATUS_REQUEST: u8 = 0x00;
const STATUS_OK: u8 = 0x01;
const HEADER_LEN: usize = 10;
const CRC_INIT: u16 = 0xFFFF;

/// CRC16-CCITT with init 0xFFFF, poly 0x1021, MSB-first. Matches the
/// tinyboot `crc.rs` reference impl bit-for-bit.
fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = CRC_INIT;
    for &b in bytes {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn build_frame(cmd: u8, status: u8, addr: u32, flags: u8, data: &[u8]) -> Vec<u8> {
    let mut body: Vec<u8> = Vec::with_capacity(HEADER_LEN + data.len() + 2);
    body.extend_from_slice(&SYNC);
    body.push(cmd);
    body.push(status);
    body.push(addr as u8);
    body.push((addr >> 8) as u8);
    body.push((addr >> 16) as u8);
    body.push(flags);
    body.push(data.len() as u8);
    body.push((data.len() >> 8) as u8);
    body.extend_from_slice(data);
    let crc = crc16(&body);
    body.push(crc as u8);
    body.push((crc >> 8) as u8);
    body
}

fn info_request() -> Vec<u8> {
    build_frame(CMD_INFO, STATUS_REQUEST, 0, 0, &[])
}

fn reset_request(bootloader: bool) -> Vec<u8> {
    let flags = if bootloader { 0x01 } else { 0x00 };
    build_frame(CMD_RESET, STATUS_REQUEST, 0, flags, &[])
}

#[derive(Debug)]
struct InfoResponse {
    capacity: u32,
    erase_size: u16,
    #[allow(dead_code)]
    boot_version: u16,
    #[allow(dead_code)]
    app_version: u16,
    mode: u16,
}

fn parse_info(frame: &[u8]) -> anyhow::Result<InfoResponse> {
    let sync_at = frame
        .windows(2)
        .position(|w| w == SYNC)
        .ok_or_else(|| anyhow::anyhow!("SYNC AA55 not found in frame"))?;
    let frame = &frame[sync_at..];
    if frame.len() < HEADER_LEN {
        anyhow::bail!("frame too short: {} bytes", frame.len());
    }
    let header = &frame[..HEADER_LEN];
    let cmd = header[2];
    let status = header[3];
    let length = u16::from_le_bytes([header[8], header[9]]) as usize;
    if cmd != CMD_INFO {
        anyhow::bail!("bootloader reply cmd=0x{cmd:02X}, want INFO");
    }
    if status != STATUS_OK {
        anyhow::bail!("bootloader reply status=0x{status:02X}, want OK");
    }
    if length != 12 {
        anyhow::bail!("InfoData len={length}, want 12");
    }
    if frame.len() < HEADER_LEN + length + 2 {
        anyhow::bail!("frame truncated: {} bytes", frame.len());
    }
    let data = &frame[HEADER_LEN..HEADER_LEN + length];
    let expected = crc16(&frame[..HEADER_LEN + length]);
    let actual = u16::from_le_bytes([frame[HEADER_LEN + length], frame[HEADER_LEN + length + 1]]);
    if expected != actual {
        anyhow::bail!("bootloader CRC mismatch: got 0x{actual:04X}, want 0x{expected:04X}");
    }
    Ok(InfoResponse {
        capacity: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        erase_size: u16::from_le_bytes([data[4], data[5]]),
        boot_version: u16::from_le_bytes([data[6], data[7]]),
        app_version: u16::from_le_bytes([data[8], data[9]]),
        mode: u16::from_le_bytes([data[10], data[11]]),
    })
}
