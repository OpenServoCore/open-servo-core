//! osc-native transport interfaces (`docs/osc-native-protocol.md` §4, §10).
//! Owned by the bus driver; chip-side providers implement these over real
//! peripherals (production) or recording mocks (tests).

use osc_core::BaudRate;

/// Counted circular RX DMA ring, armed once at boot (§4.1). The driver reads
/// the cursor and ring bytes in place; it never drains, and never reloads
/// except through [`Self::rearm`].
pub trait RxRing {
    /// The ring storage. Length must exceed the largest legal frame
    /// ([`crate::bus::FRAME_MAX`]) with lap margin — 512 on V006 (§11).
    fn bytes(&self) -> &[u8];
    /// Index where the next received byte will land (`LEN - NDTR`).
    fn cursor(&self) -> u16;
}

/// One-shot compare on the transport tick domain. `set` arms the compare;
/// the chip ISR it fires calls back into the driver's `on_deadline`.
pub trait Deadline {
    const TICKS_PER_US: u32;
    /// Nominal clock shift per oscillator trim step, ppm — the trim loop's
    /// plant-gain seed (§9.3). The true per-chip value is nonuniform; the
    /// loop measures and replaces it after its first correction.
    const CLOCK_TRIM_STEP_PPM: u32;
    fn now(&self) -> u32;
    fn set(&mut self, at: u32);
    fn cancel(&mut self);
}

/// Hardware CRC engine, DMA-fed — zero CPU per byte (§3.2, F6). Feed spans
/// must be even-LENGTH (the §3.2 fold covers trailing odd bytes) and
/// even-ADDRESSED (F12) — which every source satisfies by construction: the
/// reply buffer head and the snapshot buffer are halfword-aligned, and
/// arbitrary-parity spans reach the engine through [`Self::snapshot`]. Feeds
/// accumulate across calls until [`Self::reset`].
pub trait CrcEngine {
    fn reset(&mut self);
    fn feed(&mut self, span: &[u8]);
    /// Stream `src` into the engine's stable snapshot buffer at byte offset
    /// `off` and return the copy's address. Best-effort and asynchronous:
    /// the copy may still be in flight on return — the engine's transfer
    /// ordering guarantees no downstream consumer (CRC feed, wire arm)
    /// overtakes it. Both the wire and the CRC consume the snapshot, so a
    /// reply's CRC always covers exactly the transmitted bytes (§4.2);
    /// offsets let a caller linearize a ring-wrapped span. Contents stay
    /// valid until the next `snapshot` call over the same range.
    fn snapshot(&mut self, off: u16, src: &[u8]) -> *const u8;
    /// The CRC over everything fed since reset, once the engine has drained
    /// the fed spans; `None` while still busy.
    fn result(&mut self) -> Option<u16>;
}

/// TX side of the half-duplex wire (§4.2): drive discipline + break + one
/// DMA arm at a time. Arm completion surfaces as the chip TC ISR calling
/// the driver's `on_tx_complete`.
pub trait TxWire {
    /// Claim the wire: push-pull drive, then send the break (SBK).
    fn start_frame(&mut self);
    /// Stream one DMA arm. Called once per arm; the next arm is queued from
    /// `on_tx_complete`. UART bytes tolerate the µs-scale re-arm gap (§4.2).
    fn send(&mut self, span: &[u8]);
    /// Release the wire: open-drain, TX DMA off.
    fn release(&mut self);
}

/// Single-channel USART baud control; also the rescue-rate entry (§9.1 —
/// volatile, config register untouched).
pub trait UsartBaud {
    fn apply(&mut self, baud: BaudRate);
}

/// Raw line-level sample of the bus pin, for rescue-break confirmation
/// (§9.1: an ordinary break has risen by wake entry — the detector sets at
/// bit 10 — a rescue low has not).
pub trait LineSense {
    fn is_low(&self) -> bool;
}

/// Role bundle for the `ServoBus` composite (driver-pattern §5.4).
pub trait Providers {
    type Ring: RxRing;
    type Deadline: Deadline;
    type Crc: CrcEngine;
    type Tx: TxWire;
    type Baud: UsartBaud;
    type Line: LineSense;
}

/// Wrap-aware "`a` is at or after `b`" on the u32 tick domain.
#[inline]
pub const fn tick_reached(a: u32, b: u32) -> bool {
    a.wrapping_sub(b) < u32::MAX / 2
}
