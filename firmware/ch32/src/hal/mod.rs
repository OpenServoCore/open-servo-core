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

#[inline(always)]
pub fn delay_cycles(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
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
