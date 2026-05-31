use super::super::Pin;

#[derive(Copy, Clone)]
pub enum Pull {
    None,
    Up,
    Down,
}

#[derive(Copy, Clone, PartialEq)]
pub enum Level {
    Low,
    High,
}

/// Packed: bits 3:0 = CFGLR nibble (MODE low, CNF high), bits 7/6 = ODR high/low.
/// MODE: INPUT=00, OUTPUT_10MHZ=01. CNF<<2: PP/ANALOG=00, FLOAT/OD=01, PULL/AF_PP=10, AF_OD=11.
#[derive(Copy, Clone)]
pub struct PinMode(u8);

impl PinMode {
    /// MODE=00 + CNF=00. Disconnects the schmitt buffer; required for ADC/OPA
    /// inputs (floating-input keeps the digital buffer hot and clamps high-Z).
    pub const ANALOG: Self = Self(0b0000);
    pub const INPUT_FLOATING: Self = Self(0b0100);
    pub const INPUT_PULL_UP: Self = Self(0b1000 | 0x80);
    pub const INPUT_PULL_DOWN: Self = Self(0b1000 | 0x40);
    pub const OUTPUT_PUSH_PULL: Self = Self(0b0001);
    pub const OUTPUT_OPEN_DRAIN: Self = Self(0b0101);
    pub const AF_PUSH_PULL: Self = Self(0b1001);
    pub const AF_OPEN_DRAIN: Self = Self(0b1101);

    pub fn input_pull(pull: Pull) -> Self {
        match pull {
            Pull::Up => Self::INPUT_PULL_UP,
            Pull::Down => Self::INPUT_PULL_DOWN,
            Pull::None => Self::INPUT_FLOATING,
        }
    }
}

#[inline(always)]
pub fn configure(pin: Pin, mode: PinMode) {
    let regs = pin.gpio_regs();
    let n = pin.pin_number();

    let shift = n * 4;
    let mask = !(0xFu32 << shift);
    let bits = ((mode.0 & 0x0F) as u32) << shift;
    regs.cfglr().modify(|w| w.0 = (w.0 & mask) | bits);

    if mode.0 & 0xC0 != 0 {
        regs.outdr().modify(|w| w.set_odr(n, mode.0 & 0x80 != 0));
    }
}

#[inline]
pub fn set_level(pin: Pin, level: Level) {
    if level == Level::High {
        pin.gpio_regs()
            .bshr()
            .write(|w| w.0 = 1 << pin.pin_number());
    } else {
        pin.gpio_regs().bcr().write(|w| w.0 = 1 << pin.pin_number());
    }
}

#[inline]
pub fn toggle(pin: Pin) {
    // BSHR/BCR are atomic single-bit writes; OUTDR RMW would race a higher-
    // priority ISR's BSHR/BCR on another pin of the same port and stomp it.
    let regs = pin.gpio_regs();
    let mask = 1u32 << pin.pin_number();
    if regs.indr().read().0 & mask != 0 {
        regs.bcr().write(|w| w.0 = mask);
    } else {
        regs.bshr().write(|w| w.0 = mask);
    }
}
