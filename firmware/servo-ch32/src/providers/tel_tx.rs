//! TEL-TX provider -- USART2 (remap 5, TX-only on PC4) + DMA1_CH6, the
//! per-fast-tick stream side channel. CH6 is USART2_TX's fixed request
//! channel (RM Table 8-2), which is why the reply-snapshot M2M lives on
//! CH7 (see the ladder in `hal::dma`). No interrupt: `send` runs in the
//! PFIC LOW kernel tick, and a prior frame still in flight simply drops
//! the new one (the Tel driver's drop-on-busy contract) -- completion is
//! the channel having drained by the next tick.

use core::cell::SyncUnsafeCell;

use osc_servo_core::tel::FRAME_BUDGET;

use crate::cfg::chip;
use crate::hal::{
    afio, dma,
    gpio::{self, PinMode},
    rcc, usart,
};

/// Const-folded like the bus BRR (no runtime divide links).
const TEL_BRR: u32 = (crate::hal::clocks::PCLK_HZ + chip::TEL_BAUD / 2) / chip::TEL_BAUD;

/// DMA source; the driver's `bytes` span dies at `send` return, so the
/// frame is staged here for the channel to stream. Single writer: `send`
/// only, from the LOW kernel tick, and only after the prior transfer has
/// drained (the `remaining == 0` gate).
static TEL_BUF: SyncUnsafeCell<[u8; FRAME_BUDGET]> = SyncUnsafeCell::new([0; FRAME_BUDGET]);

/// Production binding to USART2 + DMA1_CH6.
pub struct TelTx;

impl TelTx {
    /// One-shot bring-up (driver-pattern sec 5.5): clock-gate USART2
    /// (PB2PCENR bit 13 -- see `rcc::enable_usart2`), park TX on PC4, fixed
    /// 3 Mbaud, DMAT held on. Inert until the first `send`: the pin idles
    /// mark and CH6 stays disabled.
    pub fn init() {
        rcc::enable_usart2();
        rcc::enable_gpio(chip::TEL_USART_MAPPING.tx_pin().port_index());
        afio::set_usart_remap(
            chip::TEL_USART_MAPPING.peripheral_index(),
            chip::TEL_USART_MAPPING.remap_value(),
        );
        gpio::configure(chip::TEL_USART_MAPPING.tx_pin(), PinMode::AF_PUSH_PULL);
        usart::init_tx_only(chip::TEL_USART_MAPPING.regs(), TEL_BRR);
    }
}

impl osc_servo_drivers::traits::TelTx for TelTx {
    fn send(&mut self, bytes: &[u8]) -> bool {
        if dma::remaining(dma::Channel::CH6) != 0 {
            return false;
        }
        dma::disable(dma::Channel::CH6);
        dma::clear_tc_flag(dma::Channel::CH6);
        let n = bytes.len().min(FRAME_BUDGET);
        // SAFETY: single-writer static (type doc); the prior transfer has
        // drained, so the channel is not reading the buffer.
        let buf = unsafe { &mut *TEL_BUF.get() };
        buf[..n].copy_from_slice(&bytes[..n]);
        let cfg = dma::Config {
            dir: dma::Dir::FROMMEMORY,
            circ: false,
            pinc: false,
            minc: true,
            size: dma::Size::BITS8,
            htie: false,
            tcie: false,
            // LOW, the ladder floor: a deferred beat only delays a
            // background frame (see `hal::dma`).
            pl: dma::Pl::LOW,
        };
        dma::configure(
            dma::Channel::CH6,
            &cfg,
            usart::data_addr(chip::TEL_USART_MAPPING.regs()),
            buf.as_ptr() as u32,
            n as u16,
        );
        dma::enable(dma::Channel::CH6);
        true
    }
}
