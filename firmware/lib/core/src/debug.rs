/// Software breakpoint. Emits `ebreak` on riscv32; no-op elsewhere.
///
/// On QingKe V2 (CH32V006), `dcsr.ebreakm` (QingKeV2 Manual §6.1) gates
/// behaviour: 0 = exception #3 via `mtvec` (hang), 1 = enter Debug mode.
/// Debuggers (probe-rs, wlink, OpenOCD) set this on attach.
///
/// To inspect locals at the breakpoint: build with `debug = 2`; note
/// `opt-level = "z"` may show vars as `<optimized out>` — wrap in
/// `core::hint::black_box` or log explicitly.
#[macro_export]
macro_rules! bp {
    () => {
        #[cfg(target_arch = "riscv32")]
        unsafe {
            ::core::arch::asm!("ebreak")
        }
    };
}
