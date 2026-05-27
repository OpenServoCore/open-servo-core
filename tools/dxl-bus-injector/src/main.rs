#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

mod inject;
mod led;
mod listen;
mod proto;
mod usb_cdc;

use panic_halt as _;

use ch32_hal as hal;
use embassy_executor::Spawner;

// ── Wiring (MuseLab nanoCH32V203, V203C8T6 LQFP48)
//
//   PA2   USART2_TX, HDSEL          DXL bus (single-wire)
//   PB11  USART3_RX (listen only)   DXL bus (tap)
//   PA12  USB DP                    host
//   PA11  USB DM                    host
//
// Both USART2 and USART3 are on APB1. With SYSCLK_FREQ_144MHZ_HSI APB1 runs at
// 144 MHz (prescaler DIV1), so BRR = 144_000_000 / 3_000_000 = 48 for the
// 3 Mbaud DXL Fast wire rate.

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    });

    led::init();
    led::on(); // boot indicator — fires once firmware reaches main loop
    inject::init();
    listen::init();

    spawner.must_spawn(usb_cdc::run(p.USBD, p.PA12, p.PA11));
}
