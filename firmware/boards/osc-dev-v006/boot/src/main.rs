#![no_std]
#![no_main]

use panic_halt as _;
use tinyboot_ch32_rt as _;

tinyboot_ch32::boot::boot_version!(compat = "osc-dev-v006");

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

    // HDSEL single wire on PC0, open-drain like the app (servo-ch32 init):
    // the servo only pulls low, the host pull-up holds mark. 500k until a
    // host stores another rate in META. Addressable by chip uid, so servos
    // sharing the bus update one at a time. A 240-byte payload keeps a frame
    // at 252 B: the WCH-LinkE serial bridge drops bytes from 3M bursts over
    // 256 B.
    let transport = Usart::new(&UsartConfig {
        baud: BaudRate::B500000,
        pclk: 48_000_000,
        mapping: UsartMapping::Usart1Remap3,
        wire: HalfDuplex { drive: OpenDrain },
        tx_en: NoTxEn,
    });
    tinyboot_ch32::boot::run_with_payload::<240>(
        transport,
        BootCtl::new(),
        Addressable::new(chip_uid),
    );
}
