#![no_std]
#![no_main]

use osc_ch32::{
    board::{BoardConfig, Ch32Board},
    hal::{Pin, delay_cycles},
};
use panic_halt as _;

tinyboot_ch32::app::app_version!();

#[qingke_rt::entry]
fn main() -> ! {
    let board = Ch32Board::new(BoardConfig { stat_led: Pin::PD0 });

    let mut on = false;
    loop {
        board.set_stat_led(on);
        delay_cycles(3_000_000);
        on = !on;
    }
}
