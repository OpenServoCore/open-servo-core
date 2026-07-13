use ch32_metapac::{AFIO, EXTI};

use super::super::Pin;

#[inline]
pub fn configure_falling_edge(pin: Pin) {
    let line = pin.pin_number();
    let port = pin.port_index() as u8;
    AFIO.exticr().modify(|w| w.set_exti(line, port));
    EXTI.ftenr().modify(|w| w.set_tr(line, true));
}

#[inline]
pub fn clear_pending(pin: Pin) {
    let line = pin.pin_number();
    // INTFR is W1C; the default-zero closure leaves other lines' pending
    // bits untouched while a `1` clears `line`.
    EXTI.intfr().write(|w| w.set_if_(line, true));
}

#[inline]
pub fn set_irq(pin: Pin, enable: bool) {
    let line = pin.pin_number();
    EXTI.intenr().modify(|w| w.set_mr(line, enable));
}

#[inline]
pub fn is_pending(pin: Pin) -> bool {
    EXTI.intfr().read().if_(pin.pin_number())
}
