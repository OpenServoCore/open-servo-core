use stm32f3::stm32f301 as pac;

/// Initialize GPIO for LED indicator on PA3
pub fn init_gpio(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;

    // configure PA3 as output for LED
    gpioa.moder.modify(|_, w| w.moder3().output());

    // configure PA3 as push-pull
    gpioa.otyper.modify(|_, w| w.ot3().push_pull());

    // configure PA3 as high speed output
    gpioa.ospeedr.modify(|_, w| w.ospeedr3().high_speed());

    // configure PA3 as no pull-up, no pull-down
    gpioa.pupdr.modify(|_, w| w.pupdr3().floating());

    // set PA3 to low
    gpioa.odr.modify(|_, w| w.odr3().low());
}
