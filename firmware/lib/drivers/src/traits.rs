//! Driver-owned interfaces. Drivers declare what they need from their
//! environment; chip-side adapters implement these over real HAL peripherals
//! (production) or recording mocks (tests).

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

/// Single-channel USART baud-rate control. `CLOCK_HZ` is the rate that
/// drives the BRR divisor (PCLK on most chips); drivers compute BRR off
/// of it and hand the result here. The adapter writes it to the correct
/// USART instance.
pub trait UsartBaud {
    /// Frequency, in Hz, of the clock that feeds the USART's BRR divisor.
    const CLOCK_HZ: u32;
    fn set_baud(&mut self, brr: u32);
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

/// Arm a TX fire at a protocol-prescribed wire deadline. The driver passes
/// pure intent — the wire-end tick of the last RX byte plus a delay — and
/// the provider applies all chip-specific compensations (PFIC + ISR-entry
/// latency, fine-trim residual, wrap guard, the TX_EN OC setup, etc.)
/// inside its own body. The trait surface stays free of chip-side knobs.
///
/// `wire_end_tick` is the BT-ring tick (free-running timer, 16-bit) at the
/// last bit of the last received byte; `delay_us` (or `delay_q88_us` for
/// chain participation) is what the protocol prescribes from there —
/// typically `RDT` for a Status reply or `RDT + slot_offset` for a Fast
/// slot. Q8.8 µs gives sub-µs resolution at 3 Mbaud where the inter-slot
/// gap is ~3.33 µs.
pub trait DxlTxScheduler {
    /// Single-shot fire at `wire_end_tick + delay_us`. Used for Status
    /// replies and Fast non-Last slots — anything that doesn't fold into
    /// the chain CRC.
    fn schedule(&mut self, wire_end_tick: u16, delay_us: u32);

    /// Fast Last-slot fire — initiates chain-CRC participation. The
    /// provider sets up the predecessor-byte fold (`anchor_bytes` ≈ wire
    /// bytes the post-fire walk must cover) alongside arming the fire
    /// itself; `delay_q88_us` is the slot's wire offset from
    /// `wire_end_tick` carrying Q8.8 µs precision for sub-µs alignment.
    fn schedule_last_slot(&mut self, wire_end_tick: u16, delay_q88_us: u32, anchor_bytes: u32);

    /// Drop any armed fire and return the bus to idle. Idempotent — safe
    /// to call when nothing is armed.
    fn cancel(&mut self);
}
