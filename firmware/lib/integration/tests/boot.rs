//! The fake tinyboot bootloader (`sim::boot`) against tinyboot-core's rules:
//! raw frames in, the device's replies out of its log.

use osc_integration::sim::boot::{self, BOOT_VERSION, CAPACITY, COMPAT, PAGE, SLOT_BITS, UID_LEN};
use osc_integration::sim::{Sim, instruction};
use osc_protocol::wire::{MgmtOp, Opcode};
use osc_servo_core::{BaudRate, Reboot};
use tinyboot_protocol::crc::{CRC_INIT, crc16};
use tinyboot_protocol::{BaudRate as TbBaud, Cmd, ResetFlags, Status, WriteFlags, pack_version};

const UID: [u8; 16] = [
    0x5A, 0x3C, 0x91, 0x07, 0xE2, 0x48, 0xB6, 0x1D, 0x73, 0xC9, 0x0F, 0x84, 0xAA, 0xBB, 0xCC, 0xDD,
];
/// 0.5 M: 96 sim ticks per bit.
const BIT_TICKS: u64 = 96;
const OK: u8 = Status::Ok as u8;

/// A bootloader-only board, waiting at 0.5 M.
fn rig() -> Sim {
    let mut sim = Sim::new(BaudRate::B500000);
    sim.add_servo(1);
    sim.seed_servo_uid(0, UID);
    sim.start_in_bootloader(0);
    sim
}

fn req(cmd: Cmd, addr: u32, flags: u8, data: &[u8]) -> Vec<u8> {
    let a = addr.to_le_bytes();
    boot::encode(&[cmd as u8, 0, a[0], a[1], a[2], flags], data)
}

/// Send `frame`; the replies, each checked to echo its request.
fn ask(sim: &mut Sim, frame: &[u8]) -> Vec<(u8, Vec<u8>)> {
    sim.host_send_raw(frame);
    sim.run();
    let log = sim.boot_log(0);
    log.into_iter()
        .map(|t| {
            let b = t.bytes;
            assert_eq!((b[2], &b[4..8]), (frame[2], &frame[4..8]), "echo");
            (b[3], b[10..b.len() - 2].to_vec())
        })
        .collect()
}

fn one(sim: &mut Sim, cmd: Cmd, addr: u32, flags: u8, data: &[u8]) -> (u8, Vec<u8>) {
    let mut r = ask(sim, &req(cmd, addr, flags, data));
    assert_eq!(r.len(), 1, "{cmd:?} draws one reply");
    r.remove(0)
}

fn select_bits(sim: &mut Sim, seed: u8, bits: u8) -> Vec<(u8, Vec<u8>)> {
    let mut d = vec![seed, bits];
    d.extend_from_slice(&UID[..(bits as usize).div_ceil(8).min(16)]);
    ask(sim, &req(Cmd::Select, 0, 0, &d))
}

fn select(sim: &mut Sim) {
    assert_eq!(select_bits(sim, 0, 96), vec![(OK, UID[..UID_LEN].to_vec())]);
}

/// An image whose tail carries `compat` and `version`.
fn image(len: usize, compat: u32, version: u16) -> Vec<u8> {
    let mut img: Vec<u8> = (0..len).map(|i| (i * 7) as u8).collect();
    img[len - 6..len - 2].copy_from_slice(&compat.to_le_bytes());
    img[len - 2..].copy_from_slice(&version.to_le_bytes());
    img
}

fn info(sim: &mut Sim) -> Vec<u8> {
    let (s, d) = one(sim, Cmd::Info, 0, 0, &[]);
    assert_eq!(s, OK);
    d
}

#[test]
fn every_command_round_trips() {
    let mut sim = rig();
    select(&mut sim);
    let mut want = Vec::new();
    want.extend_from_slice(&(CAPACITY as u32).to_le_bytes());
    want.extend_from_slice(&(PAGE as u16).to_le_bytes());
    want.extend_from_slice(&BOOT_VERSION.to_le_bytes());
    want.extend_from_slice(&[0xFF, 0xFF, 0, 0xFF]);
    want.extend_from_slice(&COMPAT.to_le_bytes());
    want.extend_from_slice(&240u16.to_le_bytes());
    assert_eq!(info(&mut sim), want, "empty app, default rate");

    let v = pack_version(1, 2, 3);
    let img = image(298, COMPAT, v);
    assert_eq!(one(&mut sim, Cmd::Erase, 0, 0, &512u16.to_le_bytes()).0, OK);
    assert_eq!(one(&mut sim, Cmd::Write, 0, 0, &img[..240]).0, OK);
    let flush = WriteFlags::FLUSH.bits();
    assert_eq!(one(&mut sim, Cmd::Write, 240, flush, &img[240..]).0, OK);
    let crc = crc16(CRC_INIT, &img).to_le_bytes().to_vec();
    assert_eq!(one(&mut sim, Cmd::Verify, 298, 0, &[]), (OK, crc));
    assert_eq!(
        info(&mut sim)[8..10],
        v.to_le_bytes(),
        "Info reads the tail"
    );

    // A reset storing 1 M comes back deselected at 1 M.
    let to_boot = (ResetFlags::BOOTLOADER | ResetFlags::BAUD).bits();
    let m1 = TbBaud::B1000000 as u32;
    assert_eq!(one(&mut sim, Cmd::Reset, m1, to_boot, &[]).0, OK);
    sim.set_host_baud(BaudRate::B1000000);
    assert!(ask(&mut sim, &req(Cmd::Info, 0, 0, &[])).is_empty());
    select(&mut sim);
    assert_eq!(one(&mut sim, Cmd::Reset, 0, 0, &[]).0, OK);
    assert!(!sim.in_bootloader(0));
    let fw = sim.servo_table(0, |t| t.config.common.firmware_version);
    assert_eq!(fw, v, "the app reports the image's version");
}

#[test]
fn silent_to_frames_it_cannot_take() {
    let mut sim = rig();
    assert!(
        ask(&mut sim, &req(Cmd::Info, 0, 0, &[])).is_empty(),
        "unselected"
    );
    select(&mut sim);
    let mut bad_crc = req(Cmd::Info, 0, 0, &[]);
    *bad_crc.last_mut().unwrap() ^= 1;
    let unknown = boot::encode(&[6, 0, 0, 0, 0, 0], &[]);
    let long = req(Cmd::Write, 0, 0, &[0; 241]);
    let reply = boot::encode(&[Cmd::Info as u8, OK, 0, 0, 0, 0], &[]);
    for f in [bad_crc, unknown, long, reply] {
        assert!(ask(&mut sim, &f).is_empty(), "{f:02X?}");
    }
    info(&mut sim);
}

#[test]
fn select_matches_by_prefix_and_waits_its_slot() {
    let mut sim = rig();
    assert_eq!(select_bits(&mut sim, 0, 4).len(), 1, "low nibble");
    let mut other = vec![0, 4, UID[0] ^ 0x08];
    assert!(ask(&mut sim, &req(Cmd::Select, 0, 0, &other)).is_empty());
    assert!(
        ask(&mut sim, &req(Cmd::Info, 0, 0, &[])).is_empty(),
        "deselected"
    );
    other[2] = UID[0] ^ 0x10;
    assert_eq!(ask(&mut sim, &req(Cmd::Select, 0, 0, &other)).len(), 1);
    assert!(
        select_bits(&mut sim, 0, 97).is_empty(),
        "longer than the id"
    );
    assert_eq!(select_bits(&mut sim, 0, 0).len(), 1, "empty prefix");

    // The slot: crc16(uid) >> (seed % 16) & 15 units of 96 bit-times.
    let slot = |seed: u8| (crc16(CRC_INIT, &UID[..UID_LEN]) >> (seed & 15)) as i64 & 15;
    let (a, b) = (0..16)
        .flat_map(|a| (0..16).map(move |b| (a, b)))
        .find(|&(a, b)| slot(a) != slot(b))
        .expect("two slots");
    let mut delay = |seed| {
        let mut d = vec![seed, 96];
        d.extend_from_slice(&UID[..UID_LEN]);
        sim.host_send_raw(&req(Cmd::Select, 0, 0, &d));
        sim.run();
        let t = &sim.boot_log(0)[0];
        t.start as i64 - t.request_end as i64
    };
    let (da, db) = (delay(a), delay(b));
    let unit = (SLOT_BITS * BIT_TICKS) as i64;
    assert_eq!(da - db, (slot(a) - slot(b)) * unit);
}

#[test]
fn verify_checks_compat_and_reset_to_app_waits_for_it() {
    let mut sim = rig();
    select(&mut sim);
    let img = image(26, COMPAT ^ 1, 1);
    assert_eq!(one(&mut sim, Cmd::Erase, 0, 0, &256u16.to_le_bytes()).0, OK);
    let flush = WriteFlags::FLUSH.bits();
    assert_eq!(one(&mut sim, Cmd::Write, 0, flush, &img).0, OK);
    let bad = one(&mut sim, Cmd::Verify, 26, 0, &[]).0;
    assert_eq!(bad, Status::Incompatible as u8);
    let reset = one(&mut sim, Cmd::Reset, 0, 0, &[]).0;
    assert_eq!(reset, Status::Unsupported as u8, "updating");
    assert!(sim.in_bootloader(0));
    let to_boot = ResetFlags::BOOTLOADER.bits();
    assert_eq!(one(&mut sim, Cmd::Reset, 0, to_boot, &[]).0, OK);
    assert!(sim.in_bootloader(0));
}

#[test]
fn hooks_fail_power_cycle_and_ignore() {
    let mut sim = rig();
    select(&mut sim);
    sim.fail_next_boot(0, Cmd::Erase, Status::WriteError);
    let erase = |sim: &mut Sim| one(sim, Cmd::Erase, 0, 0, &256u16.to_le_bytes()).0;
    assert_eq!(erase(&mut sim), Status::WriteError as u8);
    assert_eq!(erase(&mut sim), OK);
    sim.power_cycle(0);
    assert!(sim.in_bootloader(0), "mid-update power cut");

    // An app servo: REBOOT to its bootloader at the named rate, or not.
    let mut sim = Sim::new(BaudRate::B1000000);
    sim.add_servo(1);
    sim.add_servo(2);
    sim.set_self_reboot(true);
    sim.ignore_next_boot_reboot(1);
    let to_boot = [MgmtOp::Reboot as u8, Reboot::MODE_BOOTLOADER, 0];
    for id in [1, 2] {
        sim.host_send(&instruction(id, Opcode::Mgmt, 0, &to_boot));
        sim.run();
    }
    assert!(sim.in_bootloader(0));
    assert!(!sim.in_bootloader(1), "ignored");
    sim.power_cycle(0);
    assert!(!sim.in_bootloader(0), "the factory app boots");
}
