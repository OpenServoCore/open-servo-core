//! Vector-table backstop. This firmware enables no PFIC sources, but the
//! loader's jump is not a reset and qingke-rt's startup turns MIE on: a
//! source the loader left enabled+pending would dispatch before bringup's
//! scrub. Overriding qingke-rt's weak `DefaultHandler` retires exactly the
//! offending source and returns, instead of parking in the stock loop.

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
