//! GPIO v3 primitives: per-pin mode/cnf writes and level access. Callers
//! name concrete port+pin; the board pin map (and its rules) lives in
//! `providers::pins`.

use ch32_metapac::gpio::Gpio;
use ch32_metapac::gpio::vals::{Cnf, Mode};

#[derive(Copy, Clone)]
pub struct PinMode {
    mode: Mode,
    cnf: Cnf,
}

impl PinMode {
    pub const INPUT_FLOATING: Self = Self {
        mode: Mode::INPUT,
        cnf: Cnf::FLOATING_IN__OPEN_DRAIN_OUT,
    };
    /// Pull direction comes from ODR: 1 = up, 0 = down.
    pub const INPUT_PULL: Self = Self {
        mode: Mode::INPUT,
        cnf: Cnf::PULL_IN__AF_PUSH_PULL_OUT,
    };
    pub const OUTPUT_2MHZ: Self = Self {
        mode: Mode::OUTPUT_2MHZ,
        cnf: Cnf::ANALOG_IN__PUSH_PULL_OUT,
    };
    pub const OUTPUT_50MHZ: Self = Self {
        mode: Mode::OUTPUT_50MHZ,
        cnf: Cnf::ANALOG_IN__PUSH_PULL_OUT,
    };
    pub const AF_PUSH_PULL_50MHZ: Self = Self {
        mode: Mode::OUTPUT_50MHZ,
        cnf: Cnf::PULL_IN__AF_PUSH_PULL_OUT,
    };
    pub const AF_OPEN_DRAIN_50MHZ: Self = Self {
        mode: Mode::OUTPUT_50MHZ,
        cnf: Cnf::AF_OPEN_DRAIN_OUT,
    };
}

pub fn configure(port: Gpio, pin: usize, m: PinMode) {
    if pin < 8 {
        port.cfglr().modify(|w| {
            w.set_mode(pin, m.mode);
            w.set_cnf(pin, m.cnf);
        });
    } else {
        port.cfghr().modify(|w| {
            w.set_mode(pin - 8, m.mode);
            w.set_cnf(pin - 8, m.cnf);
        });
    }
}

#[inline(always)]
pub fn set_level(port: Gpio, pin: usize, high: bool) {
    port.outdr().modify(|w| w.set_odr(pin, high));
}

#[inline(always)]
pub fn level(port: Gpio, pin: usize) -> bool {
    port.indr().read().idr(pin)
}
