#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

mod capture;
mod debug;
mod inject;
mod led;
mod pfic;
mod proto;
mod usb_cdc;

use panic_halt as _;

use ch32_hal as hal;
use embassy_executor::Spawner;

// ── Wiring (MuseLab nanoCH32V203, V203C8T6 LQFP48)
//
//   PB10  USART3_TX (AF OD)        ─► DXL bus
//   PB11  USART3_RX (input pullup) ◄─ DXL bus  (jumpered to PB10)
//   PA12  USB DP                       host
//   PA11  USB DM                       host
//
// USART3 runs full-duplex with PB10 and PB11 bridged externally on the
// bench, putting both halves of the USART block across the single-wire DXL
// line. RX captures our own TX echo (FIRE_COMP autocal) and remote replies;
// HDSEL would internally tri-state RX during TX and is not used.
//
// PA2/PA3 were the original choice but boot-wedge on the MuseLab board:
// PA3 has R10 = 10 kΩ to 3V3 (board-side SD-card CS pull-up). With the bus
// connected idle-high before USB plugs in, current trickles via bus → 1 kΩ
// → PA3 → R10 → chip 3V3 rail and parasitically charges VDD above POR
// threshold, leaving the chip stuck in a half-booted state when USB later
// supplies real power. PB10/PB11 are clean GPIO breakouts with no onboard
// pulls (verified against schematic v1.1).
//
// USART3 + TIM2/TIM3/TIM4 all on APB1 (DXL bus + wire clock + fire timer).
// With SYSCLK_FREQ_144MHZ_HSE both APB1/2 run at 144 MHz (prescaler DIV1),
// so BRR = 144_000_000 / 3_000_000 = 48 for the 3 Mbaud DXL Fast wire
// rate. tick32 (TIM2 low + TIM3 high) ticks at 144 MHz; TIM2_CH3 IC taps
// PB10 (via TIM2_RM=0b10 partial remap) for falling-edge stamping
// (PB10/PB11 share a wire so either pin shows the same edges); TIM4 OPM
// CC2 → DMA1_CH4 → DMA1_CH2 stamps USART3 TX with no IRQ between deadline
// and wire edge.
//
// HSE (8 MHz crystal) is required, not HSI: this firmware doubles as the
// bench timebase, and HSI's ±1% trim would systematically corrupt ppm-
// scale drift measurements of the chips it's measuring.

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        ..Default::default()
    });

    led::init();
    led::on(); // boot indicator
    debug::init();
    inject::init();
    capture::init();
    pfic::set_priorities();

    spawner.must_spawn(led::blink());
    spawner.must_spawn(usb_cdc::run(p.USBD, p.PA12, p.PA11));
}
