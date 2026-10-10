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
//! - PB12 = 5V rail enable, ACTIVE LOW (CH217K U3), driven low at boot --
//!   both rails default ON so a bare adapter powers its bus; the link's
//!   RAILS_BOOT_STATE mirror must match.
//! - PB2 = unbonded die pad (datasheet Note 5): input pull-down to stop
//!   input-buffer leakage.
//! - PC9 = blue LED, active low. PA8 is its package-pin-12 twin (double
//!   bond, one output max): PA8 stays an input.
//! - PC7 = IAP button, input (board 10k pull-up); read-only for us, the
//!   loader's own cold-boot sampler is the brick-recovery path.
//! - PB10 = USART3 TX, the bus wire under HDSEL: AF open-drain from
//!   bringup on, after the USART is enabled. AF modes have no internal
//!   pull, so the bus pull-up at the adapter end (680 ohm on the bench
//!   pigtail) is required: nothing here drives a rising edge. The rescue
//!   pulse is a GPIO open-drain low. PB11 stays input pull-up (the header
//!   RX pin, tied to DATA by the pigtail, unused by firmware).

use ch32_metapac::{GPIOA, GPIOB, GPIOC};

use crate::hal::gpio::{self, PinMode};

pub const BUS_PIN: usize = 10; // PB10

pub struct Pins;

impl Pins {
    pub fn init() {
        // Rails first: LOW = on, before anything else so the DUTs boot.
        gpio::set_level(GPIOA, 5, false);
        gpio::configure(GPIOA, 5, PinMode::OUTPUT_2MHZ);
        gpio::set_level(GPIOB, 12, false);
        gpio::configure(GPIOB, 12, PinMode::OUTPUT_2MHZ);

        gpio::set_level(GPIOB, 10, true); // pull-up select until bus_attach
        gpio::configure(GPIOB, 10, PinMode::INPUT_PULL);
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

/// 5V header rail (CH217K U3 EN = PB12, ACTIVE LOW -- bench-verified by
/// fleet power-cycle: off = ping timeout, on = ping complete).
pub fn rail_5v(on: bool) {
    gpio::set_level(GPIOB, 12, !on);
}

/// The bus pin joins the USART for good: AF open-drain (RM sec 18.5), the
/// pull-up at the bus root sets every rising edge. Call only after the
/// USART is enabled: an AF pin follows the USART's TX output, which idles
/// at mark only once UE and TE are set.
#[inline]
pub fn bus_attach() {
    gpio::set_level(GPIOB, BUS_PIN, true);
    gpio::configure(GPIOB, BUS_PIN, PinMode::AF_OPEN_DRAIN_50MHZ);
}

/// Rescue pulse: the pin leaves the USART and sinks the line directly;
/// [`bus_attach`] ends it.
#[inline]
pub fn bus_hold_low() {
    gpio::set_level(GPIOB, BUS_PIN, false);
    gpio::configure(GPIOB, BUS_PIN, PinMode::OUTPUT_OD_50MHZ);
}
