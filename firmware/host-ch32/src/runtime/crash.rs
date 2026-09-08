//! Crash record in reset-surviving RAM plus the boot snapshot. The two
//! ways the adapter can leave the USB bus silently, a synchronous
//! exception (trap -> reset) and a hang (IWDG reset), both leave their
//! trace here, and INFO carries it to `osc info`.
//!
//! Home: the board's CRASH region, mid-RAM (`osc-crash.x` resolves
//! `_crash` against it; `-Tosc-crash.x` in the board link). Both ends of
//! RAM belong to the resident IAP loader, which runs on every reset (its
//! data at the bottom, its stack at the top), so a record at either end
//! would not survive the loader.
//!
//! Persistence rule: the record lives until the next exception overwrites
//! it; the host never clears it. `seq` counts exceptions since the record
//! was born (0 = none), so a changed `seq` is the host's "new crash" signal.

use core::sync::atomic::{AtomicU8, Ordering};

use osc_host::link::record::{self, Diag};

use crate::hal::{pfic, rcc};

const MAGIC: u32 = 0x5343_4F53;

#[repr(C)]
struct Record {
    magic: u32,
    seq: u32,
    mcause: u32,
    mepc: u32,
    mtval: u32,
    hse_fail: u32,
    /// Main-loop phase, rewritten at every stage boundary: names where a
    /// watchdog reset caught the loop (the exception fields name a trap).
    phase: u32,
}

const BLANK: Record = Record {
    magic: MAGIC,
    seq: 0,
    mcause: 0,
    mepc: 0,
    mtval: 0,
    hse_fail: 0,
    phase: record::PHASE_NONE as u32,
};

/// Boot snapshot (`boot`, pre-IRQ, once): the reset flags this boot found,
/// and the phase the record held when that reset hit.
static BOOT_RESET: AtomicU8 = AtomicU8::new(0);
static BOOT_PHASE: AtomicU8 = AtomicU8::new(record::PHASE_NONE);

/// Same target-gate discipline as the servo's flash-slot symbols: an
/// ungated extern links on the host only by CGU luck, and the host stub
/// lets the record logic run under `cargo test`.
#[cfg(target_arch = "riscv32")]
fn record() -> *mut Record {
    unsafe extern "C" {
        static mut _crash: Record;
    }
    &raw mut _crash
}

#[cfg(not(target_arch = "riscv32"))]
fn record() -> *mut Record {
    use core::cell::SyncUnsafeCell;
    static HOST: SyncUnsafeCell<Record> = SyncUnsafeCell::new(BLANK);
    HOST.get()
}

/// Raw pointer to one field. SAFETY for every volatile access through it:
/// `_crash` is the linker-reserved CRASH region (4-aligned, never aliased
/// by any section), and the writers (pre-IRQ bringup, the main loop, a
/// trap that never returns) never run concurrently.
macro_rules! field {
    ($f:ident) => {
        &raw mut (*record()).$f
    };
}

fn valid() -> bool {
    // SAFETY: see `field!`.
    unsafe { field!(magic).read_volatile() == MAGIC }
}

fn reset_record() {
    // SAFETY: see `field!`.
    unsafe { record().write_volatile(BLANK) }
}

/// Adopt the record (a magic mismatch, first boot or loader clobber,
/// starts it blank), returning the phase the last reset interrupted and
/// marking bringup.
fn adopt() -> u8 {
    if !valid() {
        reset_record();
    }
    // SAFETY: see `field!`.
    unsafe {
        let at = field!(phase).read_volatile() as u8;
        field!(phase).write_volatile(record::PHASE_BRINGUP as u32);
        at
    }
}

/// Exception fields in, `seq` up. Adopts a blank record first: a trap
/// before `boot` must still leave evidence.
fn write_exception(mcause: u32, mepc: u32, mtval: u32) {
    if !valid() {
        reset_record();
    }
    // SAFETY: see `field!`.
    unsafe {
        field!(mcause).write_volatile(mcause);
        field!(mepc).write_volatile(mepc);
        field!(mtval).write_volatile(mtval);
        let seq = field!(seq).read_volatile().wrapping_add(1);
        field!(seq).write_volatile(seq);
    }
}

/// First thing after the PFIC scrub: read and clear the reset-cause flags,
/// adopt the record, snapshot both for INFO.
pub fn boot() {
    let r = rcc::take_reset_flags();
    let mut reset = 0;
    for (set, bit) in [
        (r.pinrstf(), record::RESET_PIN),
        (r.porrstf(), record::RESET_POR),
        (r.sftrstf(), record::RESET_SFT),
        (r.iwdgrstf(), record::RESET_IWDG),
        (r.wwdgrstf(), record::RESET_WWDG),
        (r.lpwrrstf(), record::RESET_LPWR),
    ] {
        if set {
            reset |= bit;
        }
    }
    BOOT_RESET.store(reset, Ordering::Relaxed);
    BOOT_PHASE.store(adopt(), Ordering::Relaxed);
}

/// Mark the main-loop stage (one volatile store).
#[inline(always)]
pub fn phase(p: u8) {
    // SAFETY: see `field!`.
    unsafe { field!(phase).write_volatile(p as u32) }
}

/// The crystal never came ready: count it, mark the phase.
pub fn note_hse_fail() {
    // SAFETY: see `field!`.
    unsafe {
        let n = field!(hse_fail).read_volatile().saturating_add(1);
        field!(hse_fail).write_volatile(n);
    }
    phase(record::PHASE_HSE_FAIL);
}

/// The trap backstop's tail: record the exception, then reset. The loader
/// re-enters the app, bringup re-attaches USB, INFO carries the evidence.
pub fn exception(mcause: u32, mepc: u32, mtval: u32) -> ! {
    write_exception(mcause, mepc, mtval);
    pfic::software_reset()
}

pub fn diag() -> Diag {
    // SAFETY: see `field!`.
    unsafe {
        Diag {
            reset: BOOT_RESET.load(Ordering::Relaxed),
            phase: BOOT_PHASE.load(Ordering::Relaxed),
            crash_seq: field!(seq).read_volatile(),
            mcause: field!(mcause).read_volatile(),
            mepc: field!(mepc).read_volatile(),
            mtval: field!(mtval).read_volatile(),
            hse_fail: field!(hse_fail).read_volatile(),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    /// One static record on the host: tests serialize through a lock and
    /// invalidate it first.
    fn fresh() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // SAFETY: host stub record, serialized by LOCK.
        unsafe { field!(magic).write_volatile(0) };
        g
    }

    #[test]
    fn garbage_adopts_blank() {
        let _g = fresh();
        // SAFETY: host stub record.
        unsafe { field!(seq).write_volatile(77) };
        assert_eq!(adopt(), record::PHASE_NONE);
        let d = diag();
        assert_eq!((d.crash_seq, d.hse_fail), (0, 0));
    }

    #[test]
    fn adopt_reports_phase_at_reset_then_marks_bringup() {
        let _g = fresh();
        adopt();
        phase(record::PHASE_PUMP);
        assert_eq!(adopt(), record::PHASE_PUMP);
        // SAFETY: host stub record.
        let now = unsafe { field!(phase).read_volatile() };
        assert_eq!(now, record::PHASE_BRINGUP as u32);
    }

    #[test]
    fn exceptions_count_and_survive_adopt() {
        let _g = fresh();
        write_exception(5, 0x2a1c, 0x2000_4000);
        write_exception(7, 0x3000, 0x0);
        assert_eq!(adopt(), record::PHASE_NONE, "record kept, not blanked");
        let d = diag();
        assert_eq!(d.crash_seq, 2);
        assert_eq!((d.mcause, d.mepc, d.mtval), (7, 0x3000, 0));
    }

    #[test]
    fn hse_failures_count() {
        let _g = fresh();
        adopt();
        note_hse_fail();
        note_hse_fail();
        assert_eq!(diag().hse_fail, 2);
        assert_eq!(adopt(), record::PHASE_HSE_FAIL);
    }
}
