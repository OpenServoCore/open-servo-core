//! Packet-end tick resolution — the drain-source-corrected wire-end
//! estimate the codec attaches to every parser `Crc` event. The codec
//! stashes the most recent drain ISR's `(now, PollSrc)` and resolves it
//! here at Crc so the sink never reaches into the codec's internals.

use super::poll_event::PacketEnd;
use crate::dxl::uart::BITS_PER_FRAME;
use crate::dxl::uart::poll_src::PollSrc;

/// Packet-end estimate at the parser's Crc event — the drain-source-
/// corrected ISR-entry tick (see [`drain_ref`]) less `entry_comp` (the
/// chip's entry-latency compensation, `WireClock::PACKET_END_ENTRY_COMP_TICKS`).
/// `now` / `src` come from the codec's most recent drain-ISR stash.
pub(super) fn resolve(src: PollSrc, now: u32, ticks_per_bit: u16, entry_comp: u32) -> PacketEnd {
    PacketEnd {
        tick: drain_ref(now, src, ticks_per_bit).wrapping_sub(entry_comp),
        src,
    }
}

/// Drain-source-corrected wire-end reference from an ISR-entry `now`:
///
/// - [`PollSrc::ByteBatch`]: returns `now`. The CRC byte landed in DMA
///   ~immediately before the poll, so `now` IS packet-end with negligible
///   ISR-entry offset.
/// - [`PollSrc::LineIdle`]: returns `now − BITS_PER_FRAME · ticks_per_bit`.
///   USART1 IDLE asserts one idle character after the last data byte's
///   stop bit, so back-date by exactly that one-frame interval.
///
/// The one-frame (not two-frame) back-date at Idle is load-bearing at low
/// baud: the exact IDLE elapsed is `2·BITS_PER_FRAME·tpb` (the byte's own
/// 10 bits + the 10-bit idle threshold), but the reference we want is the
/// packet's wire end, which sits one frame before the idle latch — so a
/// single-frame back-date lands `packet_end ≈ stamp + BITS_PER_FRAME·tpb`,
/// centered in the window with symmetric slack against the ns→tick
/// truncation the sim/HCLK rounding can introduce.
///
/// `[[no_idle_timing]]` compliance: IDLE only selects which formula to
/// apply (ByteBatch vs LineIdle); the back-dated `now` is a WireClock
/// reading taken at the ISR entry, not an IDLE-derived measurement of the
/// packet's wire end.
fn drain_ref(now: u32, src: PollSrc, ticks_per_bit: u16) -> u32 {
    match src {
        PollSrc::ByteBatch => now,
        PollSrc::LineIdle => {
            let frame = (BITS_PER_FRAME as u32).wrapping_mul(ticks_per_bit as u32);
            now.wrapping_sub(frame)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 3 Mbaud at HCLK 48 MHz → ticks_per_bit = 16. One byte = 160 ticks.
    const TPB_3M: u16 = 16;
    const BYTE_TICKS_3M: u32 = 10 * TPB_3M as u32;

    #[test]
    fn byte_batch_returns_now_unchanged() {
        assert_eq!(resolve(PollSrc::ByteBatch, 1234, TPB_3M, 0).tick, 1234);
        assert_eq!(
            resolve(PollSrc::ByteBatch, u32::MAX, TPB_3M, 0).tick,
            u32::MAX
        );
    }

    #[test]
    fn line_idle_back_dates_by_one_idle_char() {
        assert_eq!(
            resolve(PollSrc::LineIdle, 1600, TPB_3M, 0).tick,
            1600 - BYTE_TICKS_3M,
        );
    }

    #[test]
    fn entry_comp_subtracts_from_the_estimate() {
        assert_eq!(resolve(PollSrc::ByteBatch, 1000, TPB_3M, 40).tick, 960);
        assert_eq!(
            resolve(PollSrc::LineIdle, 1600, TPB_3M, 40).tick,
            1600 - BYTE_TICKS_3M - 40,
        );
    }

    #[test]
    fn line_idle_wraps_at_u32_boundary() {
        assert_eq!(
            resolve(PollSrc::LineIdle, 5, TPB_3M, 0).tick,
            5u32.wrapping_sub(BYTE_TICKS_3M),
        );
    }

    #[test]
    fn src_round_trips() {
        assert_eq!(
            resolve(PollSrc::LineIdle, 0, TPB_3M, 0).src,
            PollSrc::LineIdle
        );
        assert_eq!(
            resolve(PollSrc::ByteBatch, 0, TPB_3M, 0).src,
            PollSrc::ByteBatch
        );
    }
}
