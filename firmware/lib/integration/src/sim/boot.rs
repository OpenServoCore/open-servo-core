//! A fake tinyboot bootloader: tinyboot-core's addressable dispatcher rules
//! (protocol.rs, addressing.rs) over the osc-dev-v006 boot build, without
//! its blocking reads. One per servo; while active the app is stopped and
//! the servo's received bytes land here. App flash and META persist across
//! modes, as on the chip.

use osc_servo_core::BaudRate;
use tinyboot_protocol::crc::{CRC_INIT, crc16};
use tinyboot_protocol::{
    BaudRate as TbBaud, Cmd, ResetFlags, Status, WriteFlags, compat_key, pack_version,
};

use super::core::{TICKS_PER_US, bit_ticks, byte_ticks};

/// The osc-dev-v006 boot build: the APP region of its memory.x, the V006
/// page, `run_with_payload::<240>`, its compat name and crate version.
pub const CAPACITY: usize = 58_112;
pub const PAGE: usize = 256;
pub const MAX_PAYLOAD: usize = 240;
pub const COMPAT: u32 = compat_key("osc-dev-v006");
pub const BOOT_VERSION: u16 = pack_version(0, 1, 0);
/// The V006 ESIG, the osc UID's low bytes: what Select matches and answers.
pub const UID_LEN: usize = 12;

// Device time from a request's last byte to its reply's first, V006
// typical. Every frame has its CRC checked or computed.
/// tinyboot-protocol `crc16`, bitwise: ~140 HCLK per byte at 48 MHz (the
/// fetch-model estimate client/src/boot.rs sizes its windows from).
const CRC_NS_PER_BYTE: u64 = 3_000;
/// Write's copy into the page buffer, ~30 HCLK per byte (same source).
const COPY_NS_PER_BYTE: u64 = 700;
/// V006 page erase and program, typical (protocol sec 9.4).
const ERASE_NS: u64 = 2_500_000;
const PROGRAM_NS: u64 = 1_500_000;
/// A META update is an erase and a program (tinyboot-ch32
/// `BootMetaStore::write`).
const META_NS: u64 = ERASE_NS + PROGRAM_NS;
/// One Select slot unit: tinyboot-ch32 `wait_bytes` spins 16 counts of BRR
/// passes through a one-word loop at 6 HCLK on the V006, 96 bit-times.
pub const SLOT_BITS: u64 = 96;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BootState {
    Idle,
    Updating,
    Validating,
}

#[derive(Copy, Clone)]
struct Meta {
    state: BootState,
    crc: u16,
    size: u32,
    /// A tinyboot `BaudRate` index, `0xFF` the build default.
    baud: u8,
}

const META_ERASED: Meta = Meta {
    state: BootState::Idle,
    crc: 0xFFFF,
    size: u32::MAX,
    baud: 0xFF,
};

/// One reply the device put on the wire, sim ticks.
#[derive(Clone, Debug)]
pub struct BootTx {
    pub request_end: u64,
    /// The first byte's start bit.
    pub start: u64,
    pub bytes: Vec<u8>,
}

/// What a request drew: the reply, then a reset (`Some(true)` toward the
/// app) once it has left.
pub struct Reply {
    pub start: u64,
    pub bytes: Vec<u8>,
    pub reset: Option<bool>,
}

pub struct FakeBoot {
    flash: Vec<u8>,
    meta: Meta,
    active: bool,
    rate: BaudRate,
    selected: bool,
    rx: Vec<u8>,
    page: Vec<u8>,
    /// Address of the next sequential write; `None` after a flush.
    next: Option<u32>,
    busy_until: u64,
    pub ignore_reboot: bool,
    pub fail_next: Option<(Cmd, Status)>,
    pub log: Vec<BootTx>,
}

impl Default for FakeBoot {
    /// A servo off the line: a debugger-flashed app, so no CRC in META and
    /// a programmed first word (tinyboot-core `validate_app`).
    fn default() -> Self {
        let mut flash = vec![0xFF; CAPACITY];
        flash[..4].fill(0);
        Self {
            flash,
            meta: META_ERASED,
            active: false,
            rate: BaudRate::B500000,
            selected: false,
            rx: Vec::new(),
            page: Vec::new(),
            next: None,
            busy_until: 0,
            ignore_reboot: false,
            fail_next: None,
            log: Vec::new(),
        }
    }
}

impl FakeBoot {
    /// A bootloader-only board: empty app, erased META.
    pub fn wipe(&mut self) {
        self.flash.fill(0xFF);
        self.meta = META_ERASED;
    }

    pub fn active(&self) -> bool {
        self.active
    }

    pub fn rate(&self) -> BaudRate {
        self.rate
    }

    /// MGMT REBOOT into the bootloader stores its rate first (protocol sec
    /// 9.6).
    pub fn store_baud(&mut self, rate: BaudRate) {
        if let Some(b) = TbBaud::from_hz(rate.as_hz()) {
            self.meta.baud = b as u8;
        }
    }

    /// A reset into the bootloader: deselected, at the META rate. A rate the
    /// sim wire cannot carry runs the build default, 0.5 M.
    pub fn enter(&mut self) -> BaudRate {
        self.rate = TbBaud::from_u8(self.meta.baud)
            .and_then(|b| BaudRate::from_hz(b.hz()))
            .unwrap_or(BaudRate::B500000);
        self.active = true;
        self.selected = false;
        self.rx.clear();
        self.page.clear();
        self.next = None;
        self.rate
    }

    /// tinyboot-core `check_boot_state` without the run mode. Trials are
    /// not modeled: a new image confirms at its first bringup.
    pub fn hands_off(&self) -> bool {
        match self.meta.state {
            BootState::Updating => false,
            _ if self.meta.crc != 0xFFFF => self
                .flash
                .get(..self.meta.size as usize)
                .is_some_and(|app| crc16(CRC_INIT, app) == self.meta.crc),
            _ => self.flash[..4] != [0xFF; 4],
        }
    }

    /// The app runs: its first bringup confirms a new image.
    pub fn leave(&mut self) {
        self.active = false;
        if self.meta.state == BootState::Validating {
            self.meta.state = BootState::Idle;
        }
    }

    /// A verified image's tail version.
    pub fn app_version(&self) -> Option<u16> {
        let end = self.meta.size as usize;
        let v = self.flash.get(end.checked_sub(2)?..end)?;
        Some(u16::from_le_bytes([v[0], v[1]]))
    }

    pub fn on_byte(&mut self, b: u8, now: u64, uid: &[u8]) -> Option<Reply> {
        // A polled USART overruns while the device works or talks.
        if now < self.busy_until {
            return None;
        }
        let rx = &mut self.rx;
        if rx.len() < 2 {
            if rx.len() == 1 && b == 0x55 {
                rx.push(b);
            } else {
                rx.clear();
                if b == 0xAA {
                    rx.push(b);
                }
            }
            return None;
        }
        rx.push(b);
        if rx.len() < 10 {
            return None;
        }
        let len = u16::from_le_bytes([rx[8], rx[9]]) as usize;
        if !Cmd::is_valid(rx[2]) || !Status::is_valid(rx[3]) || len > MAX_PAYLOAD {
            rx.clear();
            return None;
        }
        if rx.len() < 12 + len {
            return None;
        }
        let req = std::mem::take(rx);
        let body = 10 + len;
        if crc16(CRC_INIT, &req[..body]).to_le_bytes() != req[body..]
            || req[3] != Status::Request as u8
        {
            return None;
        }
        let (status, data, ticks, reset) = self.handle(&req[2..10], &req[10..body], uid)?;
        let ns = (body as u64 + 10 + data.len() as u64) * CRC_NS_PER_BYTE;
        let start = now + ticks + ns * TICKS_PER_US / 1000;
        let mut head = [0; 8];
        head.copy_from_slice(&req[2..10]);
        head[1] = status as u8;
        let bytes = encode(&head, &data);
        self.busy_until = start + bytes.len() as u64 * byte_ticks(self.rate);
        self.log.push(BootTx {
            request_end: now,
            start,
            bytes: bytes.clone(),
        });
        Some(Reply {
            start,
            bytes,
            reset,
        })
    }

    /// One clean request: `head` is `cmd status addr(3) flags len(2)`.
    /// Returns the status, the reply data, the work in sim ticks and the
    /// reset after, or `None` to stay silent.
    #[allow(clippy::type_complexity)]
    fn handle(
        &mut self,
        head: &[u8],
        data: &[u8],
        uid: &[u8],
    ) -> Option<(Status, Vec<u8>, u64, Option<bool>)> {
        const CMDS: [Cmd; 6] = [
            Cmd::Info,
            Cmd::Erase,
            Cmd::Write,
            Cmd::Verify,
            Cmd::Reset,
            Cmd::Select,
        ];
        let cmd = CMDS[head[0] as usize];
        let addr = u32::from_le_bytes([head[2], head[3], head[4], 0]);
        let flags = head[5];
        // tinyboot reads past `len` into a stale buffer; the fake reads 0.
        let at = |i: usize| data.get(i).copied().unwrap_or(0);
        let ns = |ns: u64| ns * TICKS_PER_US / 1000;
        let fail = |s| Some((s, Vec::new(), 0, None));

        if cmd == Cmd::Select {
            let (seed, mut rem) = (at(0), at(1) as u32);
            let mut diff = 0u32;
            for (i, &u) in uid.iter().enumerate() {
                let n = rem.min(8);
                rem -= n;
                diff |= (u ^ at(i + 2)) as u32 & (0xFF >> (8 - n));
            }
            self.selected = diff | rem == 0;
            if !self.selected {
                return None;
            }
            let slot = (crc16(CRC_INIT, uid) >> (seed & 15)) as u64 & 15;
            let wait = slot * SLOT_BITS * bit_ticks(self.rate);
            let hash = ns(uid.len() as u64 * CRC_NS_PER_BYTE);
            return Some((Status::Ok, uid.to_vec(), wait + hash, None));
        }
        if !self.selected {
            return None;
        }
        if let Some((c, s)) = self.fail_next
            && c == cmd
        {
            self.fail_next = None;
            return fail(s);
        }
        let state = self.meta.state;
        let a = addr as usize;
        match cmd {
            Cmd::Info => {
                let mut d = Vec::with_capacity(18);
                d.extend_from_slice(&(CAPACITY as u32).to_le_bytes());
                d.extend_from_slice(&(PAGE as u16).to_le_bytes());
                d.extend_from_slice(&BOOT_VERSION.to_le_bytes());
                d.extend_from_slice(&self.app_version().unwrap_or(0xFFFF).to_le_bytes());
                d.extend_from_slice(&[0, self.meta.baud]);
                d.extend_from_slice(&COMPAT.to_le_bytes());
                d.extend_from_slice(&(MAX_PAYLOAD as u16).to_le_bytes());
                Some((Status::Ok, d, 0, None))
            }
            Cmd::Erase => {
                let count = u16::from_le_bytes([at(0), at(1)]) as usize;
                if !a.is_multiple_of(PAGE)
                    || !count.is_multiple_of(PAGE)
                    || count == 0
                    || a + count > CAPACITY
                {
                    return fail(Status::AddrOutOfBounds);
                }
                let mut work = (count / PAGE) as u64 * ERASE_NS;
                if state != BootState::Updating {
                    work += META_NS;
                    if state == BootState::Validating {
                        self.meta.crc = 0xFFFF;
                        self.meta.size = u32::MAX;
                    }
                    self.meta.state = BootState::Updating;
                }
                self.flash[a..a + count].fill(0xFF);
                Some((Status::Ok, Vec::new(), ns(work), None))
            }
            Cmd::Write => {
                if state != BootState::Updating {
                    return fail(Status::Unsupported);
                }
                let aligned = self.next.map_or(a.is_multiple_of(PAGE), |n| addr == n);
                if a + data.len() > CAPACITY || !aligned {
                    return fail(Status::AddrOutOfBounds);
                }
                let mut pos = a + data.len();
                self.page.extend_from_slice(data);
                let mut programs = 0;
                while self.page.len() >= PAGE {
                    let tail = self.page.split_off(PAGE);
                    let at = pos - tail.len() - PAGE;
                    let full = std::mem::replace(&mut self.page, tail);
                    self.program(at, &full);
                    programs += 1;
                }
                if WriteFlags::from_bits_truncate(flags).contains(WriteFlags::FLUSH) {
                    if !self.page.is_empty() {
                        pos -= self.page.len();
                        let part = std::mem::take(&mut self.page);
                        self.program(pos, &part);
                        programs += 1;
                    }
                    self.next = None;
                } else {
                    self.next = Some(pos as u32);
                }
                let work = data.len() as u64 * COPY_NS_PER_BYTE + programs * PROGRAM_NS;
                Some((Status::Ok, Vec::new(), ns(work), None))
            }
            Cmd::Verify => {
                if state != BootState::Updating {
                    return fail(Status::Unsupported);
                }
                if a == 0 || a > CAPACITY {
                    return fail(Status::AddrOutOfBounds);
                }
                // A tagged image ends 2 bytes past a word, its key just
                // before the version.
                let key = a
                    .checked_sub(6)
                    .filter(|k| k.is_multiple_of(4))
                    .map(|k| u32::from_le_bytes(self.flash[k..k + 4].try_into().expect("4")));
                if key != Some(COMPAT) {
                    return fail(Status::Incompatible);
                }
                let crc = crc16(CRC_INIT, &self.flash[..a]);
                self.meta = Meta {
                    state: BootState::Validating,
                    crc,
                    size: addr,
                    ..self.meta
                };
                let work = a as u64 * CRC_NS_PER_BYTE + META_NS;
                Some((Status::Ok, crc.to_le_bytes().to_vec(), ns(work), None))
            }
            _ => {
                let flags = ResetFlags::from_bits_truncate(flags);
                let to_app = !flags.contains(ResetFlags::BOOTLOADER);
                // A reset to the app skips META's check on chips that hand
                // off by run mode, so a half-written image would run.
                if state == BootState::Updating && to_app {
                    return fail(Status::Unsupported);
                }
                let mut work = 0;
                if flags.contains(ResetFlags::BAUD) {
                    self.meta.baud = addr as u8;
                    work = META_NS;
                }
                Some((Status::Ok, Vec::new(), ns(work), Some(to_app)))
            }
        }
    }

    /// NOR program: bits only clear.
    fn program(&mut self, at: usize, bytes: &[u8]) {
        for (f, b) in self.flash[at..at + bytes.len()].iter_mut().zip(bytes) {
            *f &= b;
        }
    }
}

/// One tinyboot frame: sync, `head` (`cmd status addr(3) flags`, a length
/// after it is replaced), `data`, CRC.
pub fn encode(head: &[u8], data: &[u8]) -> Vec<u8> {
    let mut f = vec![0xAA, 0x55];
    f.extend_from_slice(&head[..6]);
    f.extend_from_slice(&(data.len() as u16).to_le_bytes());
    f.extend_from_slice(data);
    let crc = crc16(CRC_INIT, &f);
    f.extend_from_slice(&crc.to_le_bytes());
    f
}
