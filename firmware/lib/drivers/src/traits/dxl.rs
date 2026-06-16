//! DXL-over-UART transport interfaces. Owned by the DXL driver; chip-side
//! providers implement these over real peripherals (production) or
//! recording mocks (tests).

use dxl_protocol::CrcUmts;
use osc_core::BaudRate;

/// Role-shaped bundle of every chip-side leaf interface
/// [`crate::dxl::uart::DxlUart`] consumes. One associated type per leaf
/// trait below so the composite's signature collapses from six type
/// parameters to one while each sub-driver
/// ([`Clock`], [`Codec`], [`FastLast`], [`FastLastCrc`]) stays narrowly
/// typed and still documents exactly what hardware it depends on. Per
/// driver-pattern §5.4.
///
/// Chip-side providers don't implement this trait directly — they
/// implement each leaf trait on their own zero-sized type, and the
/// chip-family crate's `runtime::registry` bundles them into a single
/// `Providers` impl that maps each associated type to the matching ZST.
///
/// [`Clock`]: crate::dxl::uart::clock::Clock
/// [`Codec`]: crate::dxl::uart::codec::Codec
/// [`FastLast`]: crate::dxl::uart::fast_last::FastLast
/// [`FastLastCrc`]: crate::dxl::uart::fast_last_crc::FastLastCrc
pub trait Providers {
    type UsartBaud: UsartBaud;
    type ClockTrim: ClockTrim;
    type EdgeDma: EdgeDma;
    type RxDma: RxDma;
    type TxScheduler: TxScheduler;
    type FastLastScheduler: FastLastScheduler;
    type Crc: CrcUmts;
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

/// RX byte-ring DMA channel — read-only NDTR accessor. The RX DMA channel
/// itself runs unconditionally (USART1 → byte ring); this trait surfaces
/// the remaining-transfer count so the Fast Last fold body can refresh its
/// view of newly-arrived RX bytes from inside the busy-wait without
/// re-entering the chip-side ISR. The driver borrows one through its
/// [`Providers`] bundle; the production adapter binds to DMA1_CH5.
///
/// No flag-drain / mask methods: HT/TC drain for the parser path stays on
/// the chip-side caller via `CodecRx::on_rx_dma_advance`, and the fold
/// path doesn't want either signal — only NDTR.
pub trait RxDma {
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
/// Bulk Read predecessor window.
///
/// The driver works entirely in `(packet_end_tick: u16, offset_ticks: u32)`
/// pairs — a wire-clock anchor plus protocol-derived offsets. The
/// chip-side provider lifts `packet_end_tick` into its own scheduling
/// tick domain once per Fast Last reply (via `set_deadline`) and treats
/// all subsequent `schedule()` / `deadline_passed()` calls as offsets
/// from that anchor. The driver never sees a scheduler-domain tick.
///
/// On V006 the chip-side provider binds to a SysTick CMP rather than a
/// TIM2 CC channel: TIM2's shared prescaler is pinned at PSC=0 for the IC
/// side's 16-tick resolution at 3M, so its 16-bit CNT wraps every
/// 1.365 ms — the Fast Last grid step at low baud (`15 × byte_ticks`) can
/// exceed that and the fire deadline can be many wraps out. SysTick is
/// 32-bit at HCLK with ~89.5 s horizon, separate IRQ vector from TIM2,
/// and the ~5 µs PFIC-entry jitter is dwarfed by the fold body cost.
/// The lift uses the boot-captured TIM2 ↔ SysTick offset (see
/// `docs/dxl-hw-timed-transport.md` §12); both clocks tick at HCLK so the
/// offset is fixed.
pub trait FastLastScheduler {
    /// CMP-match → body fold-start latency, in scheduler ticks. Driver
    /// subtracts this from every grid anchor offset before handing it to
    /// `schedule` so the body's actual fold-start lands on the formula's
    /// intended wall-clock anchor.
    const FAST_LAST_ENTRY_TICKS: u16;

    /// Periodic-walk grid step, in predecessor wire bytes. Each fold body
    /// folds up to `BYTES_PER_INTERVAL` bytes of newly-classified residue
    /// before re-arming the next CMP one grid step ahead.
    const BYTES_PER_INTERVAL: u16;

    /// Pre-start fold residue cap, in predecessor wire bytes. The final
    /// busy-wait exits at `deadline = t_prior_end − GUARD_BYTES × byte_ticks`,
    /// leaving up to `GUARD_BYTES` predecessor bytes for the TX-start
    /// body's tail to fold inline. At 3M GUARD=1 keeps `patch_crc` ahead
    /// of CH4's DMA-prefetch on byte[n − 2].
    const GUARD_BYTES: u16;

    /// Cache the parser-derived `packet_end_tick` (wire-clock value where
    /// the host's request ended) and the busy-wait exit deadline as a tick
    /// offset from it. The provider lifts `packet_end_tick` into its own
    /// scheduling-tick domain once, here, and caches both the lifted
    /// anchor and the lifted deadline. Subsequent `schedule()` /
    /// `deadline_passed()` calls reference the cached anchor.
    fn set_deadline(&mut self, packet_end_tick: u16, deadline_ticks: u32);

    /// Arm the next CMP at `packet_end + offset_ticks` (caller has already
    /// back-dated by `FAST_LAST_ENTRY_TICKS`). Idempotent on re-arm.
    ///
    /// A CMP target that lands in the past — possible at low RDT + small
    /// predecessor counts where back-dating by ENTRY underflows — fires
    /// the IRQ ASAP; the body's first run lands ENTRY ticks late but the
    /// grid step advances cleanly from there.
    fn schedule(&mut self, offset_ticks: u32);

    /// True once the wall clock has passed the deadline staged via
    /// `set_deadline`. Polled by the final-step busy-wait.
    fn deadline_passed(&self) -> bool;

    /// True when the TX DMA channel's read cursor has reached the trailing
    /// CRC slot. Once true, any further `patch_crc` write into
    /// `tx_buf[len-CRC_BYTES..len]` ships too late — placeholder bytes are
    /// already on the wire. Polled by `on_tx_start`'s post-fire fold loop
    /// alongside the predecessor-byte-plateau backstop; whichever fires
    /// first ends the loop.
    fn patch_window_expired(&self) -> bool;

    /// Bump the `crc_patch_deadline_miss` telemetry counter. Called once
    /// per `on_tx_start` exit-via-miss — both the patch-window-expired
    /// route and the plateau (predecessor-bytes-stalled) route feed the
    /// same counter; both ship a placeholder CRC observable to the host as
    /// a bad-CRC packet.
    fn record_patch_deadline_miss(&mut self);

    /// Drop any pending CMP and return to idle. Idempotent.
    fn cancel(&mut self);
}
