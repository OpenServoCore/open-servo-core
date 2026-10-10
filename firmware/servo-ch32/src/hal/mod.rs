mod generated {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}
pub use generated::{Pin, Tim1Mapping, Tim2Mapping, UsartMapping};

pub mod adc;
pub mod afio;
pub mod clocks;
pub mod dma;
pub mod esig;
pub mod exti;
pub mod flash;
pub mod gpio;
pub mod opa;
pub mod pfic;
pub mod rcc;
pub mod systick;
pub mod timer;
pub mod usart;

/// Spins at least `n` HCLK cycles, timed by SysTick.
#[inline(always)]
pub fn delay_cycles(n: u32) {
    spin_until_elapsed(n, systick::ticks);
}

/// Spins until `now` has advanced `n` past its first reading, or for `n`
/// passes. A pass costs more than one HCLK cycle, so the pass bound never ends
/// a running wait early and still ends one whose counter has stopped.
#[inline(always)]
fn spin_until_elapsed(n: u32, now: impl Fn() -> u32) {
    let start = now();
    for _ in 0..n {
        if now().wrapping_sub(start) >= n {
            return;
        }
    }
}

/// Busy-wait for `ms` ms. Reinitializes SYSTICK on entry -- safe to call early.
pub fn delay_ms(ms: u32) {
    use ch32_metapac::SYSTICK;
    use ch32_metapac::systick::vals::Stclk;
    let target = ms.saturating_mul(clocks::SYSTICK_TICKS_PER_MS);
    SYSTICK.cmp().write_value(u32::MAX);
    SYSTICK.cnt().write_value(0);
    SYSTICK.ctlr().write(|w| {
        w.set_ste(true);
        w.set_stclk(Stclk::HCLK);
    });
    while SYSTICK.cnt().read() < target {}
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;

    use super::spin_until_elapsed;

    /// A counter that advances `step` per read, counting its reads.
    struct Counter {
        at: Cell<u32>,
        step: u32,
        reads: Cell<u32>,
    }

    impl Counter {
        fn new(at: u32, step: u32) -> Self {
            Self {
                at: Cell::new(at),
                step,
                reads: Cell::new(0),
            }
        }

        fn read(&self) -> u32 {
            let v = self.at.get();
            self.at.set(v.wrapping_add(self.step));
            self.reads.set(self.reads.get() + 1);
            v
        }
    }

    /// 384 cycles at 12 per pass: 32 passes, not 384.
    #[test]
    fn running_counter_ends_the_wait_at_n_cycles() {
        let c = Counter::new(1_000, 12);
        spin_until_elapsed(384, || c.read());
        assert_eq!(c.reads.get(), 1 + 32);
    }

    #[test]
    fn stopped_counter_ends_the_wait_after_n_passes() {
        let c = Counter::new(1_000, 0);
        spin_until_elapsed(384, || c.read());
        assert_eq!(c.reads.get(), 1 + 384);
    }

    #[test]
    fn wait_spans_the_counter_wrap() {
        let c = Counter::new(u32::MAX - 100, 12);
        spin_until_elapsed(384, || c.read());
        assert_eq!(c.reads.get(), 1 + 32);
    }
}
