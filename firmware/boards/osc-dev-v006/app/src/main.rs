#![no_std]
#![no_main]
#![feature(fn_align)]

use panic_halt as _;

#[cfg(feature = "defmt")]
use defmt_rtt as _;

tinyboot_ch32::app::app_version!(
    compat = include!(concat!(env!("OUT_DIR"), "/board_compat.rs")),
    version = osc_servo_ch32::FIRMWARE_VERSION
);
osc_servo_ch32::install_isrs!();

#[qingke_rt::entry]
fn main() -> ! {
    osc_servo_ch32::log::info!("osc-dev-v006: boot");
    osc_servo_ch32::run!(include!(concat!(env!("OUT_DIR"), "/board_config.rs")))
}
