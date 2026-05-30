//! PA7 scope marker for chain-latency analysis. Rising edge = USART3 IDLE ISR
//! entry (i.e. wire-end + IDLE-assertion delay + IRQ entry latency). Falling
//! edge = DMA EN write — the injector's fire kickoff. Pulse width therefore
//! captures only the path between listener IDLE and DMA enable; it should be
//! small and baud-independent. Any baud drift in that width localizes a bug
//! to the INJ side.

use ch32_hal::pac::{GPIOA, RCC};

pub fn init() {
    RCC.apb2pcenr().modify(|w| w.set_iopaen(true));

    // PA7 = GP push-pull, 50 MHz. CFGLR controls PA0..PA7; PA7 sits in
    // bits [31:28]. Mode=11 (50 MHz), CNF=00 (GP push-pull) → 0b0011.
    GPIOA.cfglr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 28);
        v |= 0b0011 << 28;
        w.0 = v;
    });
    clear();
}

#[inline]
pub fn set() {
    GPIOA.bshr().write(|w| w.set_bs(7, true));
}

#[inline]
pub fn clear() {
    GPIOA.bshr().write(|w| w.set_br(7, true));
}
