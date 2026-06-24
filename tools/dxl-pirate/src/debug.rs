//! PA7 scope marker. Currently cleared at each fire kickoff (CC2-stamp arm
//! path) so an external probe can correlate logic-analyzer captures with
//! injector timing.

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
pub fn clear() {
    GPIOA.bshr().write(|w| w.set_br(7, true));
}
