//! Break-wake provider (protocol sec 3.4, transport sec 7): TIM2 as a
//! low-time counter on the bus pin. HDSEL disables the USART's LIN break
//! detector on this silicon, so the wake is taken from the pin: the counter
//! runs only while the line is low, every rising edge zeroes it (DMA1 CH7,
//! no CPU), and its overflow at 9.25 bit-times of continuous low is a break
//! (valid data never holds the line low past 9 bit-times; the law break
//! holds 10). An idle line and frame data raise nothing.
//!
//! A low that outlasts the service (a break's tail, a rescue pulse, a
//! stuck line) would overflow again every 9.25 bit-times: the service parks
//! the counter past the reload instead, so the next overflow is 65536 ticks
//! of low away, and the rising edge's zero un-parks it. An overflow with no
//! rising edge since the park is the same low again, not a break: one wake
//! per low of any length.

use core::cell::SyncUnsafeCell;

use osc_servo_core::BaudRate;

use crate::cfg::chip;
use crate::hal::clocks::TIM_CLK_HZ;
use crate::hal::timer::tim2;
use crate::hal::{afio, dma, gpio, rcc};
use crate::probe::high_probe;

/// The overflow point in quarter bit-times: 37 = 9.25, between the longest
/// data low (9) and the law break's end (10).
const BREAK_QUARTER_BITS: u32 = 37;

/// The value the rising-edge DMA copies into the counter.
static ZERO: SyncUnsafeCell<u16> = SyncUnsafeCell::new(0);

pub struct BreakWake;

impl BreakWake {
    /// One-shot bring-up (driver-pattern sec 5.5): clock-gate and remap
    /// TIM2, arm the rising-edge zero, then start the timer at `reload`.
    pub fn init(reload: u16) {
        rcc::enable_tim2();
        afio::set_tim_remap(2, chip::BREAK_TIM2_MAPPING.remap_value());
        let zero = dma::Config {
            dir: dma::Dir::FROMMEMORY,
            circ: true,
            pinc: false,
            minc: false,
            size: dma::Size::BITS16,
            htie: false,
            tcie: false,
            // VERYHIGH (see `hal::dma`): the zero must land within a bit
            // of the edge, ahead of a reply's snapshot copy.
            pl: dma::Pl::VERYHIGH,
        };
        dma::configure(
            dma::Channel::CH7,
            &zero,
            tim2::counter_addr(),
            ZERO.get() as u32,
            1,
        );
        dma::enable(dma::Channel::CH7);
        tim2::init_low_timer(reload);
    }

    /// TIM2 vector service: `true` on a break. The update is the only
    /// enabled source; an entry without UIF was pended before a mute.
    /// Statement order is load-bearing: a low still held parks, and a
    /// rising edge between the pin read and the park, whose zero the park
    /// overwrote, shows as the pin high again and is replayed.
    #[inline(always)]
    pub fn service() -> bool {
        let f = tim2::flags();
        if !f.uif() {
            return false;
        }
        tim2::clear_update();
        let pin = chip::BUS_USART_MAPPING.tx_pin();
        if gpio::is_low(pin) {
            tim2::park();
            high_probe(|p| p.parks += 1);
            if !gpio::is_low(pin) {
                tim2::rise();
                high_probe(|p| p.rises += 1);
            }
        }
        let fresh = tim2::rose(f);
        high_probe(|p| if fresh { p.breaks += 1 } else { p.refires += 1 });
        fresh
    }

    /// Retune the overflow point to a new rate; rides every BRR write.
    #[inline(always)]
    pub fn retune(baud: BaudRate) {
        tim2::set_reload(reload_for(baud));
    }

    /// Run `send_break` deaf: the receiver hears nothing of our own TX (F9),
    /// but the detector watches the pin and our own break would read as a
    /// host's (a TEL burst would abort itself on every frame). Listening
    /// again before the first data byte is safe: data never holds the line
    /// low 9.25 bit-times.
    #[inline(always)]
    pub fn muted(send_break: impl FnOnce()) {
        tim2::mute();
        send_break();
        tim2::listen();
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
