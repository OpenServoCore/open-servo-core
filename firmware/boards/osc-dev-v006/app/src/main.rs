//! OSC dev-v006 firmware app — P0 placeholder.
//!
//! Empty entry point. Real bring-up lands in P2 (RCC + GPIO + STAT LED
//! blink) and beyond.

#![no_std]
#![no_main]

use panic_halt as _;

tinyboot_ch32::app::app_version!();

#[qingke_rt::entry]
fn main() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
