//! Break-wake provider (protocol sec 3.4, transport sec 7): TIM2 as an
//! edge-reset quiet timer on the bus pin. HDSEL disables the USART's LIN
//! break detector on this silicon, so the wake is taken from the pin: every
//! wire edge zeroes the counter, and an overflow at 9.25 bit-times of one
//! level is a break when the line sat low (valid data never holds a level
//! past 9 bit-times; the law break holds 10) and idle when it sat high.
//!
//! The overflow's DMA request (DMA1 CH2, TIM2_UP) latches GPIO INDR at that
//! instant; the service classifies on the latched level, never the live
//! pin: entered ~30 ticks after the overflow, it finds the break already
//! over at 1M and above (bringup `tim2_up_break` read the pin low on 0 of
//! 50 breaks). One wake per quiet span: a fire parks the update until the
//! next edge re-arms it, so an idle line raises nothing.

use core::cell::SyncUnsafeCell;

use osc_servo_core::BaudRate;

use crate::cfg::chip;
use crate::hal::clocks::TIM_CLK_HZ;
use crate::hal::timer::tim2;
use crate::hal::{afio, dma, gpio, rcc};

/// The overflow point in quarter bit-times: 37 = 9.25, between the longest
/// data low (9) and the law break's end (10).
const BREAK_QUARTER_BITS: u32 = 37;

/// The bus pin's port INDR at the last overflow, written only by the CH2
/// latch.
static LEVEL: SyncUnsafeCell<u32> = SyncUnsafeCell::new(0);

pub struct BreakWake;

impl BreakWake {
    /// One-shot bring-up (driver-pattern sec 5.5): clock-gate and remap
    /// TIM2, arm the level latch, then start the timer at `reload`.
    pub fn init(reload: u16) {
        rcc::enable_tim2();
        afio::set_tim_remap(2, chip::BREAK_TIM2_MAPPING.remap_value());
        let latch = dma::Config {
            dir: dma::Dir::FROMPERIPHERAL,
            circ: true,
            pinc: false,
            minc: false,
            size: dma::Size::BITS32,
            htie: false,
            tcie: false,
            // VERYHIGH (see `hal::dma`): the level must be the overflow
            // instant's, not one deferred behind another channel's burst.
            pl: dma::Pl::VERYHIGH,
        };
        dma::configure(
            dma::Channel::CH2,
            &latch,
            gpio::input_addr(chip::BUS_USART_MAPPING.tx_pin()),
            LEVEL.get() as u32,
            1,
        );
        dma::enable(dma::Channel::CH2);
        tim2::init_quiet_timer(reload);
    }

    /// TIM2 vector service: `true` when an overflow latched the line low --
    /// a break. Statement order is load-bearing: the fire parks before it
    /// classifies, and an edge between the overflow and the TIF clear was
    /// lost to that clear, so a pin already off the latched level re-arms on
    /// the spot (the counter is counting from that edge).
    #[inline(always)]
    pub fn service() -> bool {
        let pin = chip::BUS_USART_MAPPING.tx_pin();
        let f = tim2::flags();
        if f.uif() && tim2::update_armed() {
            tim2::park();
            // SAFETY: a word read of a static only the latch channel writes.
            let low = gpio::is_low_in(unsafe { LEVEL.get().read_volatile() }, pin);
            if gpio::is_low(pin) != low {
                tim2::rearm();
            }
            return low;
        }
        if f.tif() {
            tim2::rearm();
        }
        false
    }

    /// Retune the overflow point to a new rate; rides every BRR write.
    #[inline(always)]
    pub fn retune(baud: BaudRate) {
        tim2::set_reload(reload_for(baud));
    }

    /// Run `send_break` deaf: the receiver hears nothing of our own TX (F9),
    /// but the detector watches the pin and our own break would read as a
    /// host's (a TEL burst would abort itself on every frame). Listening
    /// again before the first data byte is safe: data never holds a level
    /// 9.25 bit-times.
    #[inline(always)]
    pub fn muted(send_break: impl FnOnce()) {
        tim2::mute();
        send_break();
        tim2::rearm();
    }
}

/// TIM2 reload for 9.25 bit-times at each operational rate. Each arm folds
/// to a literal via `const {}`, as `usart_baud::brr_for` does: no run-time
/// division.
pub const fn reload_for(baud: BaudRate) -> u16 {
    const fn compute(baud_hz: u32) -> u16 {
        (TIM_CLK_HZ / baud_hz * BREAK_QUARTER_BITS / 4) as u16
    }
    match baud {
        BaudRate::B500000 => const { compute(500_000) },
        BaudRate::B1000000 => const { compute(1_000_000) },
        BaudRate::B2000000 => const { compute(2_000_000) },
        BaudRate::B3000000 => const { compute(3_000_000) },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reload_is_nine_and_a_quarter_bits_at_every_rate() {
        assert_eq!(reload_for(BaudRate::B500000), 888);
        assert_eq!(reload_for(BaudRate::B1000000), 444);
        assert_eq!(reload_for(BaudRate::B2000000), 222);
        assert_eq!(reload_for(BaudRate::B3000000), 148);
    }
}
