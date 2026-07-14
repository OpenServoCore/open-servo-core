//! Flash primitives for the IAP disarm write: standard unlock + halfword
//! program, fast-mode unlock + 256 B page erase (RM sec 32). Exactly what
//! rewriting the loader's signature page needs -- nothing else in this
//! crate touches flash.

use ch32_metapac::FLASH;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

/// Fast-erase granularity on this die.
pub const PAGE_SIZE: usize = 256;

pub fn unlock() {
    FLASH
        .keyr()
        .write_value(ch32_metapac::flash::regs::Keyr(KEY1));
    FLASH
        .keyr()
        .write_value(ch32_metapac::flash::regs::Keyr(KEY2));
    // Fast-mode ops (page_er) sit behind their own lock.
    FLASH
        .modekeyr()
        .write_value(ch32_metapac::flash::regs::Modekeyr(KEY1));
    FLASH
        .modekeyr()
        .write_value(ch32_metapac::flash::regs::Modekeyr(KEY2));
}

pub fn lock() {
    FLASH.ctlr().modify(|w| {
        w.set_flock(true);
        w.set_lock(true);
    });
}

fn wait_idle() {
    while FLASH.statr().read().bsy() {
        core::hint::spin_loop();
    }
}

/// Fast 256 B page erase. `addr` must be page-aligned and the caller holds
/// both unlocks.
pub fn fast_page_erase(addr: u32) {
    wait_idle();
    FLASH.ctlr().modify(|w| w.set_page_er(true));
    FLASH.addr().write(|w| w.set_far(addr));
    FLASH.ctlr().modify(|w| w.set_strt(true));
    wait_idle();
    FLASH.ctlr().modify(|w| w.set_page_er(false));
}

/// Standard 16-bit program at a halfword-aligned address.
pub fn program_halfword(addr: u32, half: u16) {
    wait_idle();
    FLASH.ctlr().modify(|w| w.set_pg(true));
    // SAFETY: flash data write with PG staged; address validity is the
    // caller's contract (page-preserve rewrite inside the erased page).
    unsafe { (addr as *mut u16).write_volatile(half) };
    wait_idle();
    FLASH.ctlr().modify(|w| w.set_pg(false));
}
