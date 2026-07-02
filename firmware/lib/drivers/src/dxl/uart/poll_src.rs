//! Which ISR delivered the current poll. UART-level vocabulary: both the
//! codec (edge parser's packet-end formulas) and the clock (slot-deadline
//! floor) branch on it, so it lives above both sub-drivers rather than
//! inside either one.

/// Which ISR delivered the current poll — DMA1_CH5 byte-ring HT/TC or
/// USART RX IDLE. Threaded through every read that converts an ISR-entry
/// `now` into a packet-relative tick: the edge parser's anchored lift and
/// its fallback, and the clock's per-source slot-RDT floor. The two
/// contexts have different elapsed-time relationships to the last data
/// byte, so the formula downstream must pick the matching one.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PollSrc {
    /// Byte-ring HT/TC poll context. Bytes arrived steadily; the CRC byte
    /// landed in DMA ~now (sub-µs ISR-entry latency aside) so `now` is the
    /// packet-end estimate.
    ByteBatch,
    /// USART line-IDLE poll context. The wire has been quiet for one idle
    /// character (`BITS_PER_FRAME` bit-times) so the last data byte ended
    /// that long ago — back-date `now` by the idle gap.
    LineIdle,
}
