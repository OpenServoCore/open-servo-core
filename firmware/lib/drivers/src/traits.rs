//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; chip-side adapters implement these over real HAL peripherals
//! (production) or recording mocks (tests).

use osc_core::BaudRate;

use crate::types::Level;

/// Drive a single digital output. Owned by the adapter; the driver holds
/// one `P: DigitalOut` and calls `set` to change the wire state.
pub trait DigitalOut {
    fn set(&mut self, level: Level);
}

/// Free-running monotonic tick counter, used by drivers that schedule by
/// elapsed time. `TICKS_PER_US` describes the rate so the driver can
/// convert µs ↔ ticks without importing chip constants.
pub trait Monotonic {
    const TICKS_PER_US: u32;
    fn ticks(&self) -> u32;
}

/// Single-channel USART baud-rate control. The driver hands a domain-typed
/// `BaudRate`; the chip-side adapter owns the BRR math and any other
/// baud-dependent chip state (e.g., RX edge-capture filter on chips with
/// one) and applies them atomically. `CLOCK_HZ` is the clock that feeds
/// the USART's BRR divisor, which on the chip families this trait is
/// targeted at also ticks the monotonic the driver consumes — the driver
/// uses it for `ticks_per_bit` derivation and ticks↔µs conversion.
pub trait UsartBaud {
    const CLOCK_HZ: u32;
    fn apply_baud(&mut self, baud: BaudRate);
}

/// Trim a clock by integer steps. Constants describe the physics of the
/// trim mechanism so drivers can compute deadbands, thresholds, etc.
/// without importing chip constants.
pub trait ClockTrim {
    /// Inclusive lower / upper bounds for the trim delta.
    const DELTA_MIN: i8;
    const DELTA_MAX: i8;
    /// Base frequency of the trimmed clock, in Hz.
    const HZ: u32;
    /// Frequency shift per trim step, in Hz.
    const STEP_HZ: u32;
    fn apply_delta(&mut self, delta: i8);
}

/// HT/TC interrupt flags for a single DMA channel.
#[derive(Copy, Clone, Default, PartialEq, Eq, Debug)]
pub struct DmaFlags {
    pub ht: bool,
    pub tc: bool,
}

/// One DMA channel's ISR-side surface: read+ack the HT/TC flags and read
/// the remaining-transfer count (NDTR). Drivers that consume a DMA-fed
/// ring borrow one of these through their type parameter; the production
/// adapter binds to a specific channel.
pub trait DmaRing {
    fn read_and_ack(&mut self) -> DmaFlags;
    fn remaining(&self) -> u16;
}

/// What kind of TX this is — passes through `schedule` so the provider can
/// apply variant-specific bias if needed. Carrying the tag means future
/// bench-tuning doesn't reshape the trait if Fast-Last turns out to need a
/// different setup margin from Plain (e.g. catchup-completion slack).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SendKind {
    /// Plain Status reply or Fast Sync/Bulk Read non-Last slot — no chain
    /// participation.
    Plain,
    /// Fast Sync/Bulk Read Last slot — chain-CRC tail folder runs alongside.
    FastLast,
}

/// Schedule a TX at a protocol-prescribed wire deadline. The driver computes
/// the deadline in scheduler ticks (using `TICKS_PER_US` to convert µs ↔
/// ticks); the provider applies chip-specific bias compensation (PFIC +
/// ISR-entry latency, TX_EN OC setup, wrap guard) inside its own body. The
/// trait surface stays free of chip-side knobs.
pub trait DxlTxScheduler {
    /// Tick rate of the chip-side TX-start timer, ticks per µs. Driver uses
    /// this to convert protocol delay (µs / Q8.8 µs) to `deadline_tick`
    /// before calling `schedule`.
    const TICKS_PER_US: u16;

    /// Schedule a wire TX at `deadline_tick`. Provider applies its own
    /// bias / TX_EN setup compensation internally — this method's contract
    /// is "the first wire bit of `byte_count` bytes lands at approximately
    /// `deadline_tick`." Idempotent on re-schedule (overwrites any prior
    /// schedule). `kind` lets the provider apply variant-specific bias.
    ///
    /// `byte_count` is the size of the encoded packet sitting in the
    /// driver-owned TX buffer (codec's `tx_len`); the provider hands it to
    /// the chip-side DMA channel as the transfer count.
    fn schedule(&mut self, deadline_tick: u16, byte_count: u16, kind: SendKind);

    /// Drop any pending TX and return the bus to idle. Idempotent.
    fn cancel(&mut self);

    /// Driver's `on_tx_start` calls this to activate the wire driver — turn
    /// USART transmit on, kick the TX DMA channel. TX_EN is already up via
    /// hardware OC; this finishes the handoff.
    fn handle_start(&mut self);

    /// Driver's `on_tx_complete` calls this to release the wire driver —
    /// drop TX_EN, disable USART TX direction + TC IRQ, disable TX DMA.
    /// Driver body then drains pending config + surfaces any pending reboot.
    fn handle_tx_complete(&mut self);
}
