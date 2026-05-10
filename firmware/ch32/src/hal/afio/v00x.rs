use ch32_metapac::AFIO;

#[inline]
pub fn set_usart_remap(n: u8, remap: u8) {
    match n {
        1 => AFIO.pcfr1().modify(|w| w.set_usart1_rm(remap)),
        2 => AFIO.pcfr1().modify(|w| w.set_usart2_rm(remap)),
        _ => {}
    }
}

#[inline]
pub fn set_tim_remap(n: u8, remap: u8) {
    match n {
        1 => AFIO.pcfr1().modify(|w| w.set_tim1_rm(remap)),
        2 => AFIO.pcfr1().modify(|w| w.set_tim2_rm(remap)),
        _ => {}
    }
}
