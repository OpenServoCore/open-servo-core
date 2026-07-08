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
    fn now(&self) -> u32;
    fn set(&mut self, at: u32);
    fn cancel(&mut self);
}

/// Hardware CRC engine, DMA-fed — zero CPU per byte (§3.2, F6). Spans must
/// be even-LENGTH (the §3.2 fold covers trailing odd bytes); any address is
/// legal — providers whose DMA needs halfword alignment (F12) stage
/// odd-addressed spans through an internal copy (§5). Feeds accumulate
/// across calls until [`Self::reset`].
pub trait CrcEngine {
    fn reset(&mut self);
    fn feed(&mut self, span: &[u8]);
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
/// (§9.1: an ordinary break has risen by FE-ISR entry, a rescue low has not).
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
