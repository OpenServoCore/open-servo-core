//! Independent watchdog primitives (RM sec 7). Not in the metapac for this
//! die, so the four 16-bit registers are addressed directly. LSI-clocked:
//! the period rides a 40 kHz RC, and once started only a reset stops it.

const BASE: usize = 0x4000_3000;
const CTLR: *mut u16 = BASE as *mut u16;
const PSCR: *mut u16 = (BASE + 0x04) as *mut u16;
const RLDR: *mut u16 = (BASE + 0x08) as *mut u16;
const STATR: *const u16 = (BASE + 0x0C) as *const u16;

const KEY_UNLOCK: u16 = 0x5555;
const KEY_START: u16 = 0xCCCC;
const KEY_KICK: u16 = 0xAAAA;

/// PVU | RVU: a prescaler/reload write still propagating to the LSI
/// domain; a write landing while set is dropped.
const STATR_BUSY: u16 = 0b11;

/// Propagation takes a few LSI cycles (~100 us); this bound is a backstop,
/// not a wait for a stuck bit.
const UPDATE_SPINS: u32 = 100_000;

/// Nominal LSI rate (RM sec 3.3.3).
pub const LSI_HZ: u32 = 40_000;

/// PR field encodings (RM sec 7.3.2): divisor = 4 << PR.
#[derive(Clone, Copy)]
#[repr(u16)]
pub enum Prescaler {
    Div4 = 0,
    Div8 = 1,
    Div16 = 2,
    Div32 = 3,
    Div64 = 4,
    Div128 = 5,
    Div256 = 6,
}

impl Prescaler {
    pub const fn divisor(self) -> u32 {
        4 << (self as u32)
    }
}

/// Configure and start: period = (reload + 1) x divisor / LSI. `reload`
/// is 12 bits.
pub fn start(pr: Prescaler, reload: u16) {
    // SAFETY: fixed peripheral addresses, 16-bit register width (R16_*).
    unsafe {
        CTLR.write_volatile(KEY_UNLOCK);
        PSCR.write_volatile(pr as u16);
        RLDR.write_volatile(reload & 0x0FFF);
        let mut spins = UPDATE_SPINS;
        while STATR.read_volatile() & STATR_BUSY != 0 && spins > 0 {
            spins -= 1;
        }
        CTLR.write_volatile(KEY_START);
        CTLR.write_volatile(KEY_KICK);
    }
}

/// Reload the counter (one register write; call once per main-loop pass).
#[inline(always)]
pub fn kick() {
    // SAFETY: fixed peripheral address, write-only key register.
    unsafe { CTLR.write_volatile(KEY_KICK) }
}
