//! RX-side CRC feed/verify over the ring, plus the anchor's INST accessor.

use osc_protocol::wire::Inst;

use super::ServoBus;
use crate::traits::bus::{CrcEngine, Providers, RxRing};

impl<P: Providers> ServoBus<P> {
    /// Feed the covered span into the CRC engine straight from the ring — no
    /// M2M staging. Even-align the halfword feed by dropping the break's
    /// leading `0x00` when the anchor is odd (an init-0 no-op, so the CRC is
    /// unchanged); the ring base is halfword-aligned (`ring` is
    /// `repr(align(2))`). A ring-wrap splits into two accumulating arms — the
    /// engine sums across them. A trailing odd byte is left for the fold at
    /// verify. Direct feed is safe against the live circular ring: the engine
    /// runs ~8× wire speed, so it drains the covered span long before the write
    /// cursor could lap the ring back onto it.
    #[cfg_attr(target_os = "none", unsafe(link_section = ".highcode"))]
    #[cfg_attr(target_os = "none", inline(never))]
    pub(super) fn crc_feed(&mut self, anchor: u16, footprint: u16) {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let start = anchor + (anchor & 1);
        let end = anchor + footprint as usize - 2;
        let bulk_end = end - ((end - start) & 1);
        self.crc.reset();
        if bulk_end <= len {
            self.crc.feed(&ring[start..bulk_end]);
        } else {
            self.crc.feed(&ring[start..len]);
            self.crc.feed(&ring[..bulk_end - len]);
        }
    }

    /// The covered byte the even-bulk feed left behind, if the span was odd.
    fn crc_tail(&self, anchor: u16, footprint: u16) -> Option<u8> {
        let ring = self.ring.bytes();
        let len = ring.len();
        let anchor = anchor as usize;
        let start = anchor + (anchor & 1);
        let end = anchor + footprint as usize - 2;
        if (end - start) & 1 == 1 {
            Some(ring[(end - 1) % len])
        } else {
            None
        }
    }

    /// Poll the CRC result, fold the trailing odd byte if the feed left one
    /// (§3.2), and compare against the little-endian wire CRC at the
    /// covered-span end. A spin miss counts as a fail — indistinguishable from
    /// a bad frame, and never wedges the wire. Requires a prior
    /// [`Self::crc_feed`] of the same span.
    pub(super) fn crc_verify(&mut self, anchor: u16, footprint: u16) -> bool {
        let ring = self.ring.bytes();
        let len = ring.len();
        let covered = footprint as usize - 2;
        let end = anchor as usize + covered;
        let wire = u16::from_le_bytes([ring[end % len], ring[(end + 1) % len]]);
        let tail = self.crc_tail(anchor, footprint);
        let mut budget = super::super::SPIN_PER_BYTE * covered as u32;
        loop {
            if let Some(v) = self.crc.result() {
                let v = match tail {
                    Some(b) => osc_protocol::crc::osc_crc_continue(v, &[b]),
                    None => v,
                };
                return v == wire;
            }
            if budget == 0 {
                return false;
            }
            budget -= 1;
            core::hint::spin_loop();
        }
    }

    /// Feed then verify the covered span against the wire CRC (the full,
    /// non-front-loaded path).
    pub(super) fn crc_ok(&mut self, anchor: u16, footprint: u16) -> bool {
        if self.ring.bytes().is_empty() || (footprint as usize) < 2 {
            return false;
        }
        self.crc_feed(anchor, footprint);
        self.crc_verify(anchor, footprint)
    }

    /// The INST byte, 3 slots past the anchor (`[0x00][ID][LEN][INST]`).
    pub(super) fn ring_inst(&self, anchor: u16) -> Inst {
        let ring = self.ring.bytes();
        let len = ring.len();
        Inst(ring[(anchor as usize + 3) % len])
    }
}
