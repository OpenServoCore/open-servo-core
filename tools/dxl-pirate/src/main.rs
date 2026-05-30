#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

mod debug;
mod inject;
mod led;
mod listen;
mod pfic;
mod proto;
mod usb_cdc;

use panic_halt as _;

use ch32_hal as hal;
use embassy_executor::Spawner;

// ── Wiring (MuseLab nanoCH32V203, V203C8T6 LQFP48)
//
//   PA9   USART1_TX, HDSEL          DXL bus (single-wire)
//   PB11  USART3_RX (listen only)   DXL bus (tap)
//   PA12  USB DP                    host
//   PA11  USB DM                    host
//
// USART1 + TIM1 sit on APB2 (DXL bus + fire-schedule timer); USART3 is on
// APB1 (passive listener). With SYSCLK_FREQ_144MHZ_HSE both APB1/2 run at
// 144 MHz (prescaler DIV1), so BRR = 144_000_000 / 3_000_000 = 48 for the
// 3 Mbaud DXL Fast wire rate. The fire path is TIM4 OPM UEV → DMA1_CH7 →
// DMA1_CH4.CFGR (USART1_TX) — no IRQ between the scheduled tick and the
// wire edge. HSE (8 MHz crystal) is required, not HSI: this firmware
// doubles as the master-side cal timebase, and HSI's ±1% trim would
// systematically corrupt slave drift measurements at the ppm scale.

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        ..Default::default()
    });

    led::init();
    led::on(); // boot indicator — fires once firmware reaches main loop
    debug::init();
    inject::init();
    listen::init();
    pfic::set_priorities();

    spawner.must_spawn(led::blink());
    spawner.must_spawn(usb_cdc::run(p.USBD, p.PA12, p.PA11));
}
