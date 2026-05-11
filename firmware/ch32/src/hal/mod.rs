mod generated {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}
pub use generated::{Pin, UsartMapping};

pub mod adc;
pub mod afio;
pub mod dma;
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
