//! Vector-table backstops, both overriding qingke-rt's weak defaults.
//!
//! Interrupts: this firmware enables no PFIC sources, but the loader's jump
//! is not a reset and qingke-rt's startup turns MIE on: a source the loader
//! left enabled+pending would dispatch before bringup's scrub.
//! `DefaultHandler` retires exactly the offending source and returns,
//! instead of parking in the stock loop.
//!
//! Exceptions: the stock handler parks forever, USB dead, nothing recorded.
//! `ExceptionHandler` (every `__EXCEPTIONS` entry resolves to it) writes
//! mcause/mepc/mtval into the crash record and resets; bringup re-attaches
//! and INFO carries the evidence.

#[cfg(target_arch = "riscv32")]
#[unsafe(export_name = "DefaultHandler")]
extern "riscv-interrupt-m" fn default_handler() {
    let mcause: u32;
    // SAFETY: read-only CSR read.
    unsafe { core::arch::asm!("csrr {0}, mcause", out(reg) mcause) };
    if mcause & 0x8000_0000 != 0 {
        crate::hal::pfic::mask_and_unpend(mcause & 0xFF);
    }
}

#[cfg(target_arch = "riscv32")]
#[unsafe(export_name = "ExceptionHandler")]
extern "C" fn exception_handler() -> ! {
    let (mcause, mepc, mtval): (u32, u32, u32);
    // SAFETY: read-only CSR reads.
    unsafe {
        core::arch::asm!(
            "csrr {0}, mcause",
            "csrr {1}, mepc",
            "csrr {2}, mtval",
            out(reg) mcause,
            out(reg) mepc,
            out(reg) mtval,
        )
    };
    crate::runtime::crash::exception(mcause, mepc, mtval)
}
