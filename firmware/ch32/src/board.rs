use crate::hal::{
    Pin,
    gpio::{self, Level, PinMode},
    rcc,
};

pub struct BoardConfig {
    pub stat_led: Pin,
}

pub struct Ch32Board {
    stat_led: Pin,
}

impl Ch32Board {
    pub fn new(config: BoardConfig) -> Self {
        rcc::init_48mhz_hsi_pll();
        rcc::enable_gpio(config.stat_led.port_index());
        gpio::configure(config.stat_led, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(config.stat_led, Level::Low);
        Self {
            stat_led: config.stat_led,
        }
    }

    #[inline]
    pub fn set_stat_led(&self, on: bool) {
        gpio::set_level(self.stat_led, if on { Level::High } else { Level::Low });
    }
}
