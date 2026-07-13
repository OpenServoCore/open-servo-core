const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const FLASH: ch32_metapac::flash::Flash = ch32_metapac::FLASH;

pub const PAGE_SIZE: usize = 256;
const BUF_LOAD_SIZE: usize = 4;

#[inline(always)]
fn wait_busy() {
    while FLASH.statr().read().bsy() {}
    debug_assert!(
        !FLASH.statr().read().wrprterr(),
        "flash write protection error"
    );
    // EOP is W1C -- required after every BUFRST, BUFLOAD, STRT.
    FLASH.statr().modify(|w| w.set_eop(true));
}

fn unlock() {
    FLASH.keyr().write(|w| w.set_keyr(KEY1));
    FLASH.keyr().write(|w| w.set_keyr(KEY2));
    FLASH.modekeyr().write(|w| w.set_modekeyr(KEY1));
    FLASH.modekeyr().write(|w| w.set_modekeyr(KEY2));
}

#[inline(always)]
fn lock() {
    FLASH.ctlr().write(|w| {
        w.set_lock(true);
        w.set_flock(true);
    });
}

/// Erase one 256-byte page (RM sec 18.4.6).
pub fn erase(addr: u32) {
    unlock();
    FLASH.ctlr().write(|w| w.set_fter(true));
    FLASH.addr().write(|w| w.set_far(addr));
    FLASH.ctlr().write(|w| {
        w.set_fter(true);
        w.set_strt(true);
    });
    wait_busy();
    FLASH.ctlr().write(|_| {});
    lock();
}

/// Fast-page write (RM sec 18.4.5) from scattered segments streamed in order --
/// one buffer-load pass, one program cycle; no staging copy. `addr` 4-byte
/// aligned, each segment's len a multiple of 4, the total must not cross a
/// page boundary. Buffer words past the total program as erased (BUFRST
/// leaves them all-1s).
pub fn write(addr: u32, segs: &[&[u8]]) {
    let page_base = addr & !(PAGE_SIZE as u32 - 1);
    let total: usize = segs.iter().map(|s| s.len()).sum();
    debug_assert!(
        (addr as usize & (BUF_LOAD_SIZE - 1)) == 0,
        "write: addr not word-aligned"
    );
    debug_assert!(
        addr + total as u32 <= page_base + PAGE_SIZE as u32,
        "write: crosses page boundary"
    );

    unlock();
    FLASH.ctlr().write(|w| w.set_ftpg(true));
    FLASH.ctlr().write(|w| {
        w.set_ftpg(true);
        w.set_bufrst(true);
    });
    wait_busy();

    let mut buf_addr = addr;
    for seg in segs {
        debug_assert!(
            seg.len().is_multiple_of(BUF_LOAD_SIZE),
            "write: segment not word-aligned"
        );
        let mut ptr = seg.as_ptr() as *const u32;
        for _ in 0..seg.len() / BUF_LOAD_SIZE {
            // Sources may sit at any alignment (a stack header array).
            let word = unsafe { ptr.read_unaligned() };
            unsafe { core::ptr::write_volatile(buf_addr as *mut u32, word) };
            FLASH.ctlr().write(|w| {
                w.set_ftpg(true);
                w.set_bufload(true);
            });
            wait_busy();
            buf_addr += BUF_LOAD_SIZE as u32;
            ptr = unsafe { ptr.add(1) };
        }
    }

    FLASH.addr().write(|w| w.set_far(page_base));
    FLASH.ctlr().write(|w| {
        w.set_ftpg(true);
        w.set_strt(true);
    });
    wait_busy();
    FLASH.ctlr().write(|_| {});
    lock();
}

pub fn boot_mode() -> bool {
    FLASH.statr().read().boot_mode()
}

pub fn set_boot_mode(mode: bool) {
    FLASH.boot_modekeyp().write(|w| w.set_modekeyr(KEY1));
    FLASH.boot_modekeyp().write(|w| w.set_modekeyr(KEY2));
    FLASH.statr().modify(|w| w.set_boot_mode(mode));
}
