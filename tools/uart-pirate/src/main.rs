#![no_std]
#![no_main]
#![feature(sync_unsafe_cell)]

mod led;
mod parse;
mod pfic;
mod proto;
mod rcc;
mod rx;
mod tick;
mod time_driver;
mod tx;
mod usb_cdc;
mod usbd;

use panic_halt as _;

use ch32_metapac::{AFIO, EXTEND, GPIOA, RCC};
use embassy_executor::Spawner;

// ── Wiring (MuseLab nanoCH32V203, V203C8T6 LQFP48)
//
//   PB10  USART3_TX (AF OD)        ─► wire
//   PB11  USART3_RX (input pullup) ◄─ wire  (jumpered to PB10)
//   PA12  USB DP                       host
//   PA11  USB DM                       host
//
// USART3 runs full-duplex with PB10 and PB11 bridged externally on the
// bench, putting both halves of the USART block across the same single
// wire. RX captures our own TX echo (TX_COMP autocal) and remote
// replies; HDSEL would internally tri-state RX during TX and is not
// used.
//
// USART3 + TIM2/TIM3/TIM4 all on APB1 (wire UART + wire clock + send
// scheduler). With SYSCLK_FREQ_144MHZ_HSE both APB1/2 run at 144 MHz
// (prescaler DIV1), so BRR = 144_000_000 / 3_000_000 = 48 at 3 Mbaud.
// tick32 (TIM2 low + TIM3 high) ticks at 144 MHz. RX capture is
// DMA-ringed bytes plus break-boundary ticks stamped at the USART3
// FE service (rx::boundary) — no input capture; TIM4 OPM CC2 →
// DMA1_CH4 → DMA1_CH2 stamps USART3 TX with no IRQ between deadline and
// wire edge.
//
// HSE (8 MHz crystal) is required, not HSI: this firmware doubles as the
// bench timebase, and HSI's ±1% trim would systematically corrupt ppm-
// scale drift measurements of the chips it's measuring.

#[qingke_rt::interrupt]
unsafe fn USB_LP_CAN1_RX0() {
    unsafe { usbd::on_usbd_irq() }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    rcc::init_144mhz_hse();
    time_driver::init();

    // PA11/PA12 = USB DM/DP. V20x RM: drive both low push-pull before
    // enabling the USBD block so the host can't see a phantom device
    // during init. Once USBD is enabled, EXTEND.usbdpu pulls D+ high to
    // attach.
    RCC.apb2pcenr().modify(|w| w.set_iopaen(true));
    GPIOA.cfghr().modify(|w| {
        let mut v = w.0;
        // mode=11 (50 MHz), cnf=00 (GP push-pull) → 0b0011
        v &= !(0xF << 12);
        v |= 0b0011 << 12;
        v &= !(0xF << 16);
        v |= 0b0011 << 16;
        w.0 = v;
    });
    GPIOA.bshr().write(|w| {
        w.set_br(11, true);
        w.set_br(12, true);
    });
    let _ = AFIO; // referenced by remap pokes in tx::init_clocks_and_remap; keep import live here

    led::init();
    led::on();
    tick::init();
    tx::init();
    rx::init();
    pfic::set_priorities();

    // Pull D+ high via EXTEND so the host sees us. This is done inside
    // `usbd::Driver::new`, but we touch EXTEND here to keep the import
    // path stable — no-op modify.
    let _ = EXTEND;

    // embassy-executor 0.10's #[task] returns Result<SpawnToken, _>;
    // dispatch each via if-let to honour the no-panic firmware rule.
    if let Ok(t) = led::blink() {
        let _ = spawner.spawn(t);
    }
    if let Ok(t) = usb_cdc::run() {
        let _ = spawner.spawn(t);
    }
}
