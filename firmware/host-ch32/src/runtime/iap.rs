//! The mandatory revert-to-stock path: disarm the resident WCH loader's
//! arm flag and reset. The loader cold-boots, fails its validity gate, and
//! stays in IAP mode (4348:55e0) for `wlink-iap`; its flash-completion
//! path re-arms the flag automatically after the next image lands.
//!
//! The flag is byte 5 of the 6-byte signature at 0x08037F08 ("@WCH" + b4 +
//! b5): 0x5A armed, anything else disarmed (loader disassembly, silicon-
//! proven). 0x5A -> 0xA5 sets bits, so the write needs an erase -- the
//! containing 256 B page is read back, patched, and rewritten. What else
//! shares that page is a bench-dump checklist item; the preserve-rewrite
//! keeps it correct either way.

use crate::hal::{flash, pfic};

const SIG_PAGE: u32 = 0x0803_7F00;
const SIG_OFFSET: usize = 0x08;
const SIG_DISARMED: [u8; 6] = [b'@', b'W', b'C', b'H', 0xFF, 0xA5];

/// Disarm and reset. The caller has already flushed the pipe -- the ack
/// record is the last thing the port delivered.
pub fn enter_bootloader() -> ! {
    critical_section::with(|_| {
        let mut page = [0u8; flash::PAGE_SIZE];
        for (i, b) in page.iter_mut().enumerate() {
            // SAFETY: reading mapped flash.
            *b = unsafe { ((SIG_PAGE as usize + i) as *const u8).read_volatile() };
        }
        page[SIG_OFFSET..SIG_OFFSET + SIG_DISARMED.len()].copy_from_slice(&SIG_DISARMED);

        flash::unlock();
        flash::fast_page_erase(SIG_PAGE);
        for (i, pair) in page.as_chunks::<2>().0.iter().enumerate() {
            flash::program_halfword(SIG_PAGE + 2 * i as u32, u16::from_le_bytes(*pair));
        }
        flash::lock();

        pfic::software_reset()
    })
}
