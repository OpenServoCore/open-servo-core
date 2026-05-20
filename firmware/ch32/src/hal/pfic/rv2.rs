use ch32_metapac::PFIC;
use ch32_metapac::pfic::vals::Keycode;

pub use ch32_metapac::Interrupt;

#[inline]
pub fn enable(irq: Interrupt) {
    let n = irq as u16;
    let bit = 1u32 << (n % 32);
    match n / 32 {
        0 => PFIC.ienr1().write(|w| w.0 = bit),
        1 => PFIC.ienr2().write(|w| w.0 = bit),
        2 => PFIC.ienr3().write(|w| w.0 = bit),
        3 => PFIC.ienr4().write(|w| w.0 = bit),
        _ => {}
    }
}

pub fn software_reset() -> ! {
    ch32_metapac::RCC.rstsckr().write(|w| w.0 = 1 << 24);
    PFIC.cfgr().write(|w| {
        w.set_keycode(Keycode(0xBEEF));
        w.set_resetsys(true);
    });
    loop {
        core::hint::spin_loop();
    }
}
