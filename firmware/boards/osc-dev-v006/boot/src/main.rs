#![no_std]
#![no_main]

use panic_halt as _;
use tinyboot_ch32_rt as _;

tinyboot_ch32::boot::boot_version!();

use tinyboot_ch32::boot::prelude::*;
use tinyboot_ch32::pac::{
    FLASH, RCC,
    rcc::vals::{Hpre, Pllsrc, Sw},
};

fn init_48mhz_hsi_pll() {
    FLASH.actlr().modify(|w| w.set_latency(2));

    RCC.cfgr0().modify(|w| {
        w.set_pllsrc(Pllsrc::HSI);
        w.set_hpre(Hpre::DIV1);
    });

    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    RCC.cfgr0().modify(|w| w.set_sw(Sw::PLL));
    while RCC.cfgr0().read().sws() != Sw::PLL {}
}

#[unsafe(export_name = "main")]
fn main() -> ! {
    init_48mhz_hsi_pll();

    // Buffered-wire shape (74LVC2G241 + TX_EN) in BOTH app wire modes, on
    // purpose: an unmodified rev B needs it, and a bypassed rev B running
    // the app's direct wire still has the buffer populated — boot RX keeps
    // arriving through it (TX_EN pull-down = receive), boot TX drives the
    // data line through the bypass jumper, and TX_EN toggling just echoes
    // the jumper's level. Do NOT switch to Duplex::Half on tinyboot
    // v0.4.1: its init orders TE before HDSEL, which latches the HDSEL TX
    // signal LOW (bench-measured on the app, osc-ch32 `usart::init_bus`),
    // and its TX pin idles AF push-pull — together a hard-low bus jam for
    // every boot window. A true rev C (buffer removed) needs those two
    // tinyboot fixes before boot-over-UART can work at all.
    let transport = Usart::new(&UsartConfig {
        duplex: Duplex::Full,
        baud: BaudRate::B3000000,
        pclk: 48_000_000,
        mapping: UsartMapping::Usart1Remap3,
        rx_pull: Pull::None,
        tx_en: Some(TxEnConfig {
            pin: Pin::PC2,
            tx_level: Level::High,
        }),
    });
    tinyboot_ch32::boot::run(transport, BootCtl::new());
}
