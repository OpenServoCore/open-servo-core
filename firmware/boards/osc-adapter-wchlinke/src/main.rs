#![no_std]
#![no_main]

use panic_halt as _;

osc_host_ch32::install_isrs!();

// The WCH IAP loader's APP-validity gate: word 0x12345678 at image offset
// 0x50 and 0x39373533 at 0x54 (loader disassembly, silicon-proven on this
// board). Those offsets are vector-table entries 20/21 -- the FLASH and RCC
// IRQs, which this firmware never enables -- so the loader's magic and the
// table share the space by WCH's own design. Absolute symbols override the
// device.x PROVIDEs; the cross-build CI step asserts the placement.
core::arch::global_asm!(
    ".global FLASH",
    ".set FLASH, 0x12345678",
    ".global RCC",
    ".set RCC, 0x39373533",
);

#[qingke_rt::entry]
fn main() -> ! {
    osc_host_ch32::runtime::run::run()
}
