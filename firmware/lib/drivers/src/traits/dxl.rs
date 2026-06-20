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
    type TxBus: TxBus;
    type FastLastScheduler: FastLastScheduler;
    type WireClock: WireClock;
    type Crc: CrcUmts;
}

/// 32-bit-horizon wire-clock counter. Same physical clock that ticks
/// `UsartBaud::CLOCK_HZ`; the chip-side ISR layer stamps the low 16 bits
/// (the IC peripheral's native width) into the edge ring, and `now()`
/// returns the same clock lifted to u32 so the driver can store packet-end
/// ticks, skip-deadline anchors, and scheduler deadlines without wrap
/// concerns at any supported baud (~89 s horizon at 48 MHz HCLK well
/// exceeds the longest packet at the lowest baud).
///
/// Contract: the low 16 bits of `now()` equal the modular IC-capture stamp
/// at the same instant. The classifier walks IC stamps in u16 modular
/// (small bit-time windows fit a single wrap) and lifts the output to u32
/// using a current `now()` reading. The chip-side provider is responsible
/// for satisfying the contract (e.g. by reading a u32 SysTick that shares
/// the same HCLK as TIM2's IC).
pub trait WireClock {
    fn now(&self) -> u32;
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

/// Trim a clock by chip-independent ppm corrections. The provider quantizes
/// to its nearest hardware step and clamps to its envelope; the driver-side
/// integrator stays free of chip vocabulary (HSI step Hz, register width).
///
/// `STEP_PPM` and `ENVELOPE_PPM` let the integrator pre-compute its
/// half-step emission threshold and clamp pending corrections without
/// importing chip consts. `apply_ppm` semantics are *absolute relative to
/// factory cal* — the register, not an accumulator, is the state of record;
/// brownout-clean and matches the M3 host-CAL hook.
pub trait ClockTrim {
    /// Smallest correction the provider can represent, in ppm. The
    /// integrator gates its emission threshold at half this and never emits
    /// a correction the provider can't act on.
    const STEP_PPM: u32;
    /// Saturation rails relative to factory cal, in ppm: `(min, max)`. The
    /// integrator clamps pending corrections so it stops emitting once
    /// saturated.
    const ENVELOPE_PPM: (i32, i32);
    /// Apply an absolute correction relative to factory cal, in ppm.
    /// Provider quantizes to the nearest hardware step and clamps.
    fn apply_ppm(&mut self, ppm: i32);
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

/// RX byte-ring DMA channel — NDTR accessor + HT/TC flag ack. The RX DMA
/// channel itself runs unconditionally (USART1 → byte ring); HT/TC fires
/// a publish-only ISR (no parser drain, no codec poll) so the codec's view
/// of `write_seq` stays within `RX_BUF_LEN/2` of the wire regardless of
/// edge-ring cadence. The driver borrows one through its [`Providers`]
/// bundle; the production adapter binds to DMA1_CH5. Always live — no
/// pause/resume; redundant publishes during Fast Last cost ~10 cycles and
/// don't perturb the catchup body's `drain_raw`-driven NDTR refresh.
pub trait RxDma {
    fn remaining(&self) -> u16;

    /// Read and clear HT/TC flags on this channel. Called from the
    /// publish-only ISR before [`Self::remaining`] feeds
    /// `CodecRx::on_rx_dma_advance`. Returned flags are informational;
    /// the publish proceeds regardless of which crossing fired.
    fn read_and_ack(&mut self) -> DmaFlags;

    /// Bump the `edge_anchor_miss` telemetry counter. Called once per
    /// parser Crc event where the classifier had no anchor (interference
    /// / edge loss) and the composite invoked
    /// [`crate::dxl::uart::codec::rx::PollSrc`]-driven fallback for
    /// `packet_end_tick`. Wire-condition floor signal — peer of the
    /// `crc_patch_deadline_miss` counter on [`FastLastScheduler`].
    fn record_edge_anchor_miss(&mut self);
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

/// Schedule a TX at a protocol-prescribed wire deadline. The driver hands
/// an absolute `deadline` in the WireClock u32 domain; the chip-side
/// provider applies its own bias compensation (PFIC + ISR-entry latency,
/// TX_EN OC setup, wrap guard, time-remaining decision tree) without
/// further lifting — the driver already computed the absolute deadline
/// from a `WireClock::now()`-domain anchor.
///
/// Sibling of [`TxBus`] — `TxScheduler` decides *when* the chip transmits;
/// `TxBus` does the wire driving when that moment lands (or when a chain
/// k > 0 reply fires sequence-driven off a SkipComplete event).
pub trait TxScheduler {
    /// Tick rate of the chip-side TX-start timer, ticks per µs. Driver uses
    /// this to convert protocol delay (µs / Q8.8 µs) to ticks before adding
    /// to the wire-clock anchor.
    const TICKS_PER_US: u16;

    /// Schedule a wire TX at the absolute `deadline` (WireClock u32 domain).
    /// Provider applies chip-specific bias + decision tree directly — this
    /// method's contract is "the first wire bit of `byte_count` bytes lands
    /// at approximately `deadline`." Idempotent on re-schedule (overwrites
    /// any prior schedule).
    ///
    /// - `kind == SendKind::Plain` — arm per the chip-side decision tree.
    /// - `kind == SendKind::FastLast` — stash; the composite triggers the
    ///   commit via [`Self::commit_pending`] when the FastLast walk reaches
    ///   its final anchor (chain-CRC catchup co-owns the long-horizon timer
    ///   during the predecessor window, so the scheduler defers).
    ///
    /// `byte_count` is the size of the encoded packet sitting in the
    /// driver-owned TX buffer (codec's `tx_len`); the provider hands it to
    /// the chip-side DMA channel as the transfer count.
    fn schedule(&mut self, deadline: u32, byte_count: u16, kind: SendKind);

    /// Composite signals "the FastLast walk reached its final anchor — commit
    /// the stashed schedule now." Provider runs its time-remaining decision
    /// against the stashed deadline; by construction the caller invokes this
    /// within ~1 byte_time of the wire deadline, so the decision lands in
    /// the direct-arm or software-fire branch. No-op when nothing stashed;
    /// idempotent.
    fn commit_pending(&mut self);

    /// Drop any pending TX schedule. Idempotent. The wire driver itself is
    /// owned by [`TxBus`]; canceling the schedule alone leaves the bus in
    /// whatever state it was in (typically idle — between schedule and the
    /// deadline ISR no wire activity is in flight).
    fn cancel(&mut self);

    /// Long-horizon timer match fired — provider returns `true` if it owned
    /// the deadline (re-runs its decision tree internally) and `false` if
    /// the match belongs to another scheduling consumer (the FastLast walk
    /// grid co-owns the long-horizon timer during a chain-CRC catchup
    /// window). The composite uses the return value to demux which sub-
    /// driver consumes the match.
    fn on_schedule_due(&mut self) -> bool;
}

/// Chip-side control of the half-duplex DXL bus during transmission. Owns
/// the wire-driver state machine — TX_EN gating, TX DMA enable/disable, the
/// "send these bytes now, no deadline involved" path.
///
/// Sibling of [`TxScheduler`]. Two activation sources:
///
/// - The scheduler's deadline ISR — calls [`Self::handle_start`] to take
///   over the bus once the hardware match channel fires.
/// - The Plain Sync / Bulk Read chain reply at slot k > 0
///   (`docs/dxl-streaming-rx.md` §5.2) — calls [`Self::start_now`] from
///   the codec's `PollEvent::SkipComplete` arm matching the chip's
///   immediate predecessor's ID, fires the wire bit immediately with no
///   deadline math.
///
/// In both cases the bus is released via [`Self::handle_tx_complete`] when
/// the chip-side TC IRQ surfaces.
pub trait TxBus {
    /// Stage DMA + USART for `byte_count` outgoing bytes, drive TX_EN
    /// active, and let DMA fetch byte 0 — the first wire bit lands shortly
    /// after this returns. Used when the chip is responding to an observed
    /// wire event with no future deadline, currently the Plain Sync / Bulk
    /// Read chain reply at slot k > 0 per `docs/dxl-streaming-rx.md` §5.2.
    fn start_now(&mut self, byte_count: u16);

    /// Driver's `on_tx_start` calls this from the scheduler's deadline ISR
    /// to take over the bus (enable TX DMA so byte 0 ships). TX_EN is
    /// already up via hardware OC. Not called on the [`Self::start_now`]
    /// path — `start_now` does its own activation inline.
    fn handle_start(&mut self);

    /// Driver's `on_tx_complete` calls this to release the bus — drop
    /// TX_EN, disable USART TX direction + TC IRQ, disable TX DMA. Driver
    /// body then drains pending config + surfaces any pending reboot.
    fn handle_tx_complete(&mut self);
}

/// Long-horizon CMP scheduler for the Fast Last CRC fold pipeline.
///
/// Drives the periodic walks that classify edges, drain the parser, and
/// fold predecessor wire bytes into the running CRC during a Fast Sync /
/// Bulk Read predecessor window.
///
/// The driver works entirely in absolute u32 deadlines (WireClock domain).
/// The chip-side provider applies these directly — no lifting, no
/// per-anchor caching of a separate scheduling-domain tick — because the
/// WireClock contract guarantees a u32 horizon wide enough to span any
/// supported Fast Sync / Bulk Read predecessor window.
///
/// On V006 the chip-side provider binds to a SysTick CMP rather than a
/// TIM2 CC channel: TIM2's shared prescaler is pinned at PSC=0 for the IC
/// side's 16-tick resolution at 3M, so its 16-bit CNT wraps every
/// 1.365 ms — the Fast Last grid step at low baud (`15 × byte_ticks`) can
/// exceed that and the fire deadline can be many wraps out. SysTick is
/// 32-bit at HCLK with ~89.5 s horizon, separate IRQ vector from TIM2,
/// and the ~5 µs PFIC-entry jitter is dwarfed by the fold body cost.
pub trait FastLastScheduler {
    /// CMP-match → body fold-start latency, in scheduler ticks. Driver
    /// subtracts this from every grid anchor before handing it to
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

    /// Cache the busy-wait exit `deadline` (WireClock u32 domain).
    /// Subsequent `deadline_passed()` calls compare against it.
    fn set_deadline(&mut self, deadline: u32);

    /// Arm the next CMP at the absolute `deadline` (caller has already
    /// back-dated by `FAST_LAST_ENTRY_TICKS`). Idempotent on re-arm.
    ///
    /// A CMP target that lands in the past — possible at low RDT + small
    /// predecessor counts where back-dating by ENTRY underflows — fires
    /// the IRQ ASAP; the body's first run lands ENTRY ticks late but the
    /// grid step advances cleanly from there.
    fn schedule(&mut self, deadline: u32);

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
