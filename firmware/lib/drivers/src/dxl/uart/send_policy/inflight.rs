//! Per-packet wire-state aggregator for an addressed Instruction — the
//! slot-walk cursor and chain-shape accumulators that resolve into a
//! [`ReplyContext`] at the parser's Crc-good event.

use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};
use dxl_protocol::wire::BROADCAST_ID;

use super::fast_shape::{
    PING_STATUS_FRAME_BYTES, compute_fast_position, fast_bytes_before, fast_chain_packet_length,
    fast_first_bytes, fast_middle_bytes,
};
use super::reply_context::ReplyContext;
use crate::dxl::uart::poll_src::PollSrc;

/// Per-packet wire-state aggregator. Lives while the parser is inside an
/// instruction the chip will reply to; consumed at the Crc-good event to
/// derive the [`ReplyContext`] the send path needs. State accumulates from
/// header + per-slot demarcation events; finalized math lives in
/// [`InflightCtx::into_reply_context`].
#[derive(Copy, Clone, Debug)]
pub(super) struct InflightCtx {
    header: InstructionHeader,
    /// Slot-walk cursor. Bumped on each SyncSlot/BulkSlot event; resolves to
    /// the chain's `n_total` at Crc time.
    next_slot_index: u8,
    /// Index of the chip's own slot once identified. Stays `None` for
    /// non-Sync/Bulk variants and for chains the chip is not in.
    slot: Option<u8>,
    /// Wire bytes preceding the chip's slot — accumulated incrementally
    /// during BulkRead / FastBulkRead walks (per-slot length varies);
    /// derived at Crc for SyncRead / FastSyncRead (uniform per-slot length).
    bytes_before: u32,
    /// Per-slot length captured at BulkSlot demarcation just before the chip's
    /// own slot, used to size FastBulkRead's `Middle`/`Last` emission. None for
    /// non-Bulk variants.
    slot_length: Option<u16>,
    /// Sum of per-slot data byte counts across the chain — feeds Fast
    /// chain `packet_length` math for First/Only emissions. Accumulated
    /// during `slot_walk` for FastBulkRead (per-slot length varies); for
    /// FastSyncRead derived in `into_reply_context` from header.length ×
    /// `n_total`.
    chain_data_bytes: u32,
    /// Slot ID seen at the most recent SyncSlot / BulkSlot demarcation
    /// before the chip's own slot lands. Updates per demarcation while
    /// `slot` is still `None`; freezes once `slot` resolves — that's the
    /// chip's chain predecessor for sequence-driven scheduling
    /// (`docs/dxl-streaming-rx.md` §5.2). Stays `None` for slot 0 of any
    /// chain and for non-chain instructions.
    predecessor_id: Option<u8>,
}

impl InflightCtx {
    pub(super) fn new(header: InstructionHeader) -> Self {
        Self {
            header,
            next_slot_index: 0,
            slot: None,
            bytes_before: 0,
            slot_length: None,
            chain_data_bytes: 0,
            predecessor_id: None,
        }
    }

    /// Whether a `packet_end_tick` fallback estimate is safe to use when
    /// the classifier lost anchor. FAST chain ops drive chain-CRC
    /// fold-grid back-dating; a guessed anchor mispatches the CRC by ≥1
    /// byte and corrupts the entire downstream chain — strictly worse
    /// than silence. Single-target replies (Ping / Read / Write /
    /// RegWrite) absorb the ~µs-scale fallback jitter inside RDT slack;
    /// plain Sync/Bulk Read slot k>0 schedules sequence-driven from the
    /// predecessor and ignores `packet_end_tick` entirely.
    pub(super) fn allows_packet_end_fallback(&self) -> bool {
        !matches!(
            self.header,
            InstructionHeader::FastSyncRead { .. } | InstructionHeader::FastBulkRead { .. },
        )
    }

    /// Final ReplyContext at Crc-good. `packet_end_tick` is captured from the
    /// classifier at the same event; `fold_start_cursor` is the codec's
    /// wire-byte cursor at the parser's Crc emit point — the cursor where
    /// the First predecessor reply byte will land (the host's chain
    /// instruction is fully consumed by then, so the next wire byte is the
    /// First servo's `0xFF`).
    pub(super) fn into_reply_context(
        self,
        id: u8,
        rdt_us: u32,
        packet_end_tick: u32,
        fold_start_cursor: u32,
        src: PollSrc,
    ) -> ReplyContext {
        // Plain SyncRead / BulkRead don't appear here: slot 0 of either
        // chain is single-target (slot_offset_bytes = 0 — handled by the
        // catch-all), and slot k > 0 takes the chain-pending path in
        // `ReplyHandle::send_status`, which never reads slot_offset_bytes.
        let (slot_offset_bytes, fast_slot_position, reply_rdt_us) = match (self.header, self.slot) {
            (InstructionHeader::Ping { id: target }, _) if target.as_byte() == BROADCAST_ID => (
                (id as u32) * PING_STATUS_FRAME_BYTES,
                None,
                (crate::dxl::DEFAULT_RDT_2US as u32) * 2,
            ),
            (InstructionHeader::FastSyncRead { length, .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, (n as u32) * (length as u32));
                let position = compute_fast_position(k, n, packet_length);
                let bytes_before = fast_bytes_before(k, length as u32);
                (bytes_before, Some(position), rdt_us)
            }
            (InstructionHeader::FastBulkRead { .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, self.chain_data_bytes);
                let position = compute_fast_position(k, n, packet_length);
                (self.bytes_before, Some(position), rdt_us)
            }
            _ => (0, None, rdt_us),
        };
        let predecessor_id = match (self.header, self.slot) {
            (InstructionHeader::SyncRead { .. } | InstructionHeader::BulkRead { .. }, Some(k))
                if k > 0 =>
            {
                self.predecessor_id
            }
            _ => None,
        };
        ReplyContext {
            packet_end_tick,
            slot_offset_bytes,
            fast_slot_position,
            fold_start_cursor,
            predecessor_id,
            rdt_us: reply_rdt_us,
            src,
        }
    }
}

impl InflightCtx {
    /// Bump slot-walk cursor on a per-slot demarcation event and update
    /// `slot` / `bytes_before` if applicable.
    pub(super) fn on_slot(&mut self, payload: &InstructionPayload, id: u8) {
        let (slot_id, slot_length) = match *payload {
            InstructionPayload::SyncSlot { id, .. } => (id, None),
            InstructionPayload::BulkSlot { id, length, .. } => (id, Some(length)),
            InstructionPayload::WriteDataChunk { .. } => return,
        };
        let k = self.next_slot_index;
        self.next_slot_index = self.next_slot_index.saturating_add(1);
        // Chain-total data byte accumulator runs for every BulkSlot in a Fast
        // Bulk Read — successor slots included — because First/Only's
        // packet_length is the sum across the whole chain.
        if let Some(length) = slot_length
            && let InstructionHeader::FastBulkRead { .. } = self.header
        {
            self.chain_data_bytes = self.chain_data_bytes.saturating_add(length as u32);
        }
        if slot_id.as_byte() == id && self.slot.is_none() {
            self.slot = Some(k);
            self.slot_length = slot_length;
            return;
        }
        if self.slot.is_some() {
            return;
        }
        // Predecessor slot — record the latest candidate (overwriting prior).
        // The chain-fire path for slots k > 0 only reads `predecessor_id` if
        // our own slot lands next; the standing value is always the immediate
        // predecessor when it's read (`docs/dxl-streaming-rx.md` §5.2).
        self.predecessor_id = Some(slot_id.as_byte());
        // Fast Bulk Read needs per-slot wire counts to size the chain CRC fold
        // (FastSlotInfo::bytes_before on Last). Plain Bulk Read takes the
        // chain-pending path on k > 0, so its predecessor sizes don't matter.
        if let Some(length) = slot_length
            && let InstructionHeader::FastBulkRead { .. } = self.header
        {
            let length = length as u32;
            let bytes = if k == 0 {
                fast_first_bytes(length)
            } else {
                fast_middle_bytes(length)
            };
            self.bytes_before += bytes;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::test_support::TEST_ID;
    use dxl_protocol::Id;

    // InflightCtx fallback allowance (per-instruction packet-end policy)

    fn allows_fallback(header: InstructionHeader) -> bool {
        InflightCtx::new(header).allows_packet_end_fallback()
    }

    #[test]
    fn inflight_for_single_target_allows_packet_end_fallback() {
        assert!(allows_fallback(InstructionHeader::Ping {
            id: Id::new(TEST_ID),
        }));
        assert!(allows_fallback(InstructionHeader::Read {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::Write {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::RegWrite {
            id: Id::new(TEST_ID),
            address: 0,
            length: 2,
        }));
    }

    #[test]
    fn inflight_for_plain_sync_and_bulk_allows_packet_end_fallback() {
        assert!(allows_fallback(InstructionHeader::SyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        }));
        assert!(allows_fallback(InstructionHeader::BulkRead {
            id: Id::new(BROADCAST_ID),
        }));
    }

    #[test]
    fn inflight_for_fast_sync_read_disallows_packet_end_fallback() {
        assert!(!allows_fallback(InstructionHeader::FastSyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        }));
    }

    #[test]
    fn inflight_for_fast_bulk_read_disallows_packet_end_fallback() {
        assert!(!allows_fallback(InstructionHeader::FastBulkRead {
            id: Id::new(BROADCAST_ID),
        }));
    }
}
