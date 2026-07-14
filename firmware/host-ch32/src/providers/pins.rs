//! System provider: the WCH-LinkE R0-1v3 pin map. One board, so the values
//! live here with their rules rather than behind a wiring schema.
//!
//! PIN RULES (all datasheet- or bench-pinned):
//! - PA13/PA14 (SWD) are NEVER configured -- the stock-probe one-clip
//!   attach depends on it.
//! - PB6/PB7 belong to the USBHS PHY; no GPIO config touches them.
//! - PA5 = 3V3 header rail enable, ACTIVE LOW (CH217K U4), driven low at
//!   boot so the DUT side has power. PA5 shares package pin 2 with PA1
//!   (double bond): PA1 stays an input.
//! - PB12 = 5V rail enable, left OFF (reset state).
//! - PB2 = unbonded die pad (datasheet Note 5): input pull-down to stop
//!   input-buffer leakage.
//! - PC9 = blue LED, active low. PA8 is its package-pin-12 twin (double
//!   bond, one output max): PA8 stays an input.
//! - PC7 = IAP button, input (board 10k pull-up); read-only for us, the
//!   loader's own cold-boot sampler is the brick-recovery path.
//! - PB10 = USART3 TX, the bus wire under HDSEL: AF open-drain at rest
//!   (released; the DUT-side pull-up holds mark). PB11 stays input
//!   pull-up (the header RX pin is unwired in the HDSEL rig).

use ch32_metapac::{GPIOA, GPIOB, GPIOC};

use crate::hal::gpio::{self, PinMode};

pub const BUS_PIN: usize = 10; // PB10

pub struct Pins;

impl Pins {
    pub fn init() {
        // Rail first: LOW = 3V3 on, before anything else so the DUT boots.
        gpio::set_level(GPIOA, 5, false);
        gpio::configure(GPIOA, 5, PinMode::OUTPUT_2MHZ);

        gpio::set_level(GPIOB, 10, true); // idle-high before the AF block takes the pin
        gpio::configure(GPIOB, 10, PinMode::AF_OPEN_DRAIN_50MHZ);
        gpio::set_level(GPIOB, 11, true); // pull-up select
        gpio::configure(GPIOB, 11, PinMode::INPUT_PULL);
        gpio::set_level(GPIOB, 2, false); // pull-down select
        gpio::configure(GPIOB, 2, PinMode::INPUT_PULL);

        gpio::set_level(GPIOC, 9, true); // LED off (active low)
        gpio::configure(GPIOC, 9, PinMode::OUTPUT_2MHZ);
    }
}

#[inline]
pub fn led(on: bool) {
    gpio::set_level(GPIOC, 9, !on);
}

/// DUT 3V3 rail (CH217K U4 EN, ACTIVE LOW -- bench-measured).
pub fn rail_3v3(on: bool) {
    gpio::set_level(GPIOA, 5, !on);
}

/// 5V header rail (CH217K U3 EN = PB12). Active-low is presumed from U4's
/// measured polarity (same part, same topology) -- unverified, nothing is
/// wired to the 5V pin yet.
pub fn rail_5v(on: bool) {
    gpio::set_level(GPIOB, 12, !on);
    gpio::configure(GPIOB, 12, PinMode::OUTPUT_2MHZ);
}

/// Bus wire drive for the TX claim window: push-pull drives both edges at
/// 3M; open-drain hands the wire back to the pull-up.
#[inline]
pub fn bus_drive(push_pull: bool) {
    gpio::configure(
        GPIOB,
        BUS_PIN,
        if push_pull {
            PinMode::AF_PUSH_PULL_50MHZ
        } else {
            PinMode::AF_OPEN_DRAIN_50MHZ
        },
    );
}

/// Rescue pulse: the pin leaves the USART and drives dominant directly.
#[inline]
pub fn bus_hold_low() {
    gpio::set_level(GPIOB, BUS_PIN, false);
    gpio::configure(GPIOB, BUS_PIN, PinMode::OUTPUT_50MHZ);
}

/// End of the rescue pulse: ODR back to mark, pin back to the USART.
#[inline]
pub fn bus_release_from_hold() {
    gpio::set_level(GPIOB, BUS_PIN, true);
    gpio::configure(GPIOB, BUS_PIN, PinMode::AF_OPEN_DRAIN_50MHZ);
}
