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

    // Wire modes mirror the app's `half-duplex` feature: default = rev B's
    // 74LVC2G241 buffer + TX_EN (full duplex behind it); the feature =
    // direct single wire (tinyboot's `half-duplex`). Direct is only safe
    // on the shared wire since OpenServoCore/tinyboot#32 (HDSEL-before-TE
    // + open-drain park); v0.4.1 latched the TX output LOW and idled
    // push-pull — a hard-low bus jam for every boot window.
    #[cfg(not(feature = "half-duplex"))]
    let transport = Usart::new(&UsartConfig {
        baud: BaudRate::B3000000,
        pclk: 48_000_000,
        mapping: UsartMapping::Usart1Remap3,
        rx_pull: Pull::None,
        tx_en: Some(TxEnConfig {
            pin: Pin::PC2,
            tx_level: Level::High,
        }),
    });
    #[cfg(feature = "half-duplex")]
    let transport = Usart::new(&UsartConfig {
        baud: BaudRate::B3000000,
        pclk: 48_000_000,
        mapping: UsartMapping::Usart1Remap3,
        tx_en: None,
    });
    tinyboot_ch32::boot::run(transport, BootCtl::new());
}
