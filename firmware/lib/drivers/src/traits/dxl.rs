//! DXL-over-UART transport interfaces. Owned by the DXL driver; chip-side
//! providers implement these over real peripherals (production) or
//! recording mocks (tests).

use osc_core::BaudRate;

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

/// Edge-capture DMA channel handle — controller for the DMA channel
/// that lands TIM2_CH4 IC timestamps into the driver-owned `edges`
/// buffer. The buffer itself lives on the driver (`Rx::edges`); this
/// trait owns the channel's ISR-side surface: read+ack HT/TC flags,
/// read remaining-transfer count (NDTR), and toggle HT/TC IE for the
/// Fast Last fold window. The driver borrows one through its type
/// parameter; the production adapter binds to DMA1_CH7.
pub trait EdgeDma {
    fn read_and_ack(&mut self) -> DmaFlags;
    fn remaining(&self) -> u16;

    /// Mask HT/TC interrupt enable on this channel **and** clear any
    /// latched HT/TC IF. Called by the Fast Last scheduler at first-fold
    /// entry (high baud) so the classifier ISR doesn't preempt CC1's body
    /// during the Fast Last window.
    ///
    /// The IF clear is load-bearing, not housekeeping: if HT/TC latches
    /// microseconds before `pause` runs, masking IE alone leaves the IRQ
    /// pending — it would fire and run the classifier ISR the moment CC1
    /// exits, defeating the §10.6 merge. CC1's first fold body has already
    /// done the classifier's work inline, so dropping the latched flag is
    /// correct. Idempotent.
    fn pause(&mut self);

    /// Re-enable HT/TC IRQs on this channel. Called from USART1 TC after
    /// our Fast Last reply finishes. Idempotent.
    fn resume(&mut self);
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
pub trait TxScheduler {
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

/// Long-horizon CMP scheduler for the Fast Last CRC fold pipeline.
///
/// Drives the periodic walks that classify edges, drain the parser, and
/// fold predecessor wire bytes into the running CRC during a Fast Sync /
/// Bulk Read predecessor window. The driver computes wall-clock anchors
/// (one per `BYTES_PER_INTERVAL`-byte step from `t_prior_start` to
/// `final_anchor_tick`) and back-dates EVERY anchor by
/// `FAST_LAST_ENTRY_TICKS` before handing the CMP to the provider — the
/// grid stays rigid even when one body's ISR-entry latency jitters.
///
/// Ticks are HCLK-domain `u32`. On V006 the provider binds to a SysTick
/// CMP rather than a TIM2 CC channel: TIM2's shared prescaler is pinned
/// at PSC=0 for the IC side's 16-tick resolution at 3M, so its 16-bit CNT
/// wraps every 1.365 ms — the Fast Last grid step at low baud (`15 ×
/// byte_ticks`) can exceed that and the fire deadline can be many wraps
/// out. SysTick is 32-bit at HCLK with ~89.5 s horizon, separate IRQ
/// vector from TIM2, and the ~5 µs PFIC-entry jitter is dwarfed by the
/// fold body cost — fine for catchup, which doesn't sit on the TX-fire
/// path. The composite translates parser-derived TIM2 u16 ticks into
/// scheduler u32 at arm time using a boot-captured offset.
pub trait FastLastScheduler {
    /// CC-match → body fold-start latency, in scheduler ticks. Driver
    /// subtracts this from every grid anchor before handing the CMP to
    /// `schedule` so the body's actual fold-start lands on the formula's
    /// intended wall-clock anchor.
    const FAST_LAST_ENTRY_TICKS: u16;

    /// Periodic-walk grid step, in predecessor wire bytes. Each fold body
    /// folds up to `BYTES_PER_INTERVAL` bytes of newly-classified residue
    /// before re-arming the next CMP one grid step ahead.
    const BYTES_PER_INTERVAL: u16;

    /// Pre-start fold residue cap, in predecessor wire bytes. The
    /// final-anchor busy-wait exits at `walk_deadline_tick = t_prior_end −
    /// GUARD_BYTES × byte_ticks`, leaving up to `GUARD_BYTES` predecessor
    /// bytes for the TX-start body's tail to fold inline. At 3M GUARD=1
    /// keeps `patch_crc` ahead of CH4's DMA-prefetch on byte[n − 2].
    const GUARD_BYTES: u16;

    /// Arm a CMP match at `tick`. Driver has already back-dated by
    /// `FAST_LAST_ENTRY_TICKS`. Idempotent on re-schedule (overwrites any
    /// prior CMP).
    fn schedule(&mut self, tick: u32);

    /// Drop any pending CMP and return to idle. Idempotent.
    fn cancel(&mut self);
}
