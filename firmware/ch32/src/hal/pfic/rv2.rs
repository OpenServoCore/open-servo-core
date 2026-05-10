use ch32_metapac::pfic::vals::Keycode;

pub fn software_reset() -> ! {
    ch32_metapac::RCC.rstsckr().write(|w| w.0 = 1 << 24);
    ch32_metapac::PFIC.cfgr().write(|w| {
        w.set_keycode(Keycode(0xBEEF));
        w.set_resetsys(true);
    });
    loop {
        core::hint::spin_loop();
    }
}
