//! Per-packet wire-state aggregator for an addressed Instruction — the
//! slot-walk cursor and chain-shape accumulators that resolve into a
//! [`ReplyContext`] at the parser's Crc-good event.

use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};
use dxl_protocol::wire::BROADCAST_ID;

use super::fast_shape::{
    PING_REPLY_GUARD_BYTES, PING_STATUS_FRAME_BYTES, compute_fast_position, fast_bytes_before,
    fast_chain_packet_length, fast_first_bytes, fast_successor_bytes,
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
}

// -- events ---------------------------------------------------------------

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
        // The chain-start path for slots k > 0 only reads `predecessor_id` if
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
                fast_successor_bytes(length)
            };
            self.bytes_before += bytes;
        }
    }
}

// -- commands -------------------------------------------------------------

impl InflightCtx {
    /// Final ReplyContext at Crc-good. `packet_end_tick` is the codec's
    /// software packet-end estimate at the same event; `fold_start_cursor`
    /// is the codec's
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
                (id as u32) * (PING_STATUS_FRAME_BYTES + PING_REPLY_GUARD_BYTES),
                None,
                crate::dxl::DEFAULT_RDT_US,
            ),
            (InstructionHeader::FastSyncRead { length, .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, (n as u32) * (length as u32));
                let position = compute_fast_position(k, packet_length);
                let bytes_before = fast_bytes_before(k, length as u32);
                (bytes_before, Some(position), rdt_us)
            }
            (InstructionHeader::FastBulkRead { .. }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, self.chain_data_bytes);
                let position = compute_fast_position(k, packet_length);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxl::uart::test_support::TEST_ID;
    use dxl_protocol::Id;

    // InflightCtx slot walk → ReplyContext derivation

    use dxl_protocol::SlotPosition;

    fn sync_slot(id: u8, index: u8) -> InstructionPayload {
        InstructionPayload::SyncSlot {
            id: Id::new(id),
            index,
        }
    }

    fn bulk_slot(id: u8, index: u8, length: u16) -> InstructionPayload {
        InstructionPayload::BulkSlot {
            id: Id::new(id),
            index,
            address: 0,
            length,
        }
    }

    fn walk(header: InstructionHeader, slots: &[InstructionPayload]) -> InflightCtx {
        let mut ctx = InflightCtx::new(header);
        for s in slots {
            ctx.on_slot(s, TEST_ID);
        }
        ctx
    }

    fn resolve(ctx: InflightCtx) -> ReplyContext {
        ctx.into_reply_context(TEST_ID, 250, 1000, 0, PollSrc::ByteBatch)
    }

    fn fast_sync_read(length: u16) -> InstructionHeader {
        InstructionHeader::FastSyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length,
        }
    }

    #[test]
    fn slot_walk_freezes_predecessor_at_own_slot() {
        let header = InstructionHeader::SyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        };
        let ctx = walk(
            header,
            &[
                sync_slot(0x30, 0),
                sync_slot(0x31, 1),
                sync_slot(TEST_ID, 2),
                sync_slot(0x33, 3),
            ],
        );
        // Plain SyncRead slot k > 0 → the ReplyContext carries the
        // immediate predecessor; the successor slot doesn't overwrite it.
        assert_eq!(resolve(ctx).predecessor_id, Some(0x31));
    }

    #[test]
    fn slot_zero_and_fast_chains_carry_no_predecessor() {
        let sync = InstructionHeader::SyncRead {
            id: Id::new(BROADCAST_ID),
            address: 0,
            length: 2,
        };
        let at_zero = walk(sync, &[sync_slot(TEST_ID, 0), sync_slot(0x31, 1)]);
        assert_eq!(resolve(at_zero).predecessor_id, None);

        // Fast chains start grid-driven, never sequence-driven.
        let fast = walk(
            fast_sync_read(2),
            &[sync_slot(0x30, 0), sync_slot(TEST_ID, 1)],
        );
        assert_eq!(resolve(fast).predecessor_id, None);
    }

    #[test]
    fn fast_sync_read_derives_offset_and_position_from_uniform_length() {
        let ctx = walk(
            fast_sync_read(2),
            &[
                sync_slot(0x30, 0),
                sync_slot(TEST_ID, 1),
                sync_slot(0x33, 2),
            ],
        );
        let reply = resolve(ctx);
        // Slot 1 of 3 follows one First emission: 10 + 2 + crc(2) = 14.
        assert_eq!(reply.slot_offset_bytes, 14);
        assert_eq!(
            reply.fast_slot_position,
            Some(SlotPosition::Successor { crc: 0 })
        );
        assert_eq!(reply.rdt_us, 250);
    }

    #[test]
    fn fast_bulk_read_accumulates_per_slot_lengths() {
        let header = InstructionHeader::FastBulkRead {
            id: Id::new(BROADCAST_ID),
        };
        let ctx = walk(
            header,
            &[
                bulk_slot(0x30, 0, 4),
                bulk_slot(TEST_ID, 1, 2),
                bulk_slot(0x33, 2, 6),
            ],
        );
        let reply = resolve(ctx);
        // First emission for the length-4 predecessor: 10 + 4 + crc(2) = 16.
        assert_eq!(reply.slot_offset_bytes, 16);
        assert_eq!(
            reply.fast_slot_position,
            Some(SlotPosition::Successor { crc: 0 })
        );
    }

    #[test]
    fn fast_last_slot_resolves_last_position() {
        let ctx = walk(
            fast_sync_read(2),
            &[sync_slot(0x30, 0), sync_slot(TEST_ID, 1)],
        );
        let reply = resolve(ctx);
        assert_eq!(
            reply.fast_slot_position,
            Some(SlotPosition::Successor { crc: 0 })
        );
        assert_eq!(reply.slot_offset_bytes, 14);
    }

    #[test]
    fn write_data_chunks_do_not_advance_the_slot_walk() {
        let mut ctx = InflightCtx::new(fast_sync_read(2));
        ctx.on_slot(&sync_slot(0x30, 0), TEST_ID);
        ctx.on_slot(
            &InstructionPayload::WriteDataChunk {
                offset: 0,
                length: 4,
            },
            TEST_ID,
        );
        ctx.on_slot(&sync_slot(TEST_ID, 1), TEST_ID);
        // Own slot still lands at k = 1 — the chunk didn't bump the cursor.
        assert_eq!(resolve(ctx).slot_offset_bytes, 14);
    }

    #[test]
    fn broadcast_ping_positions_by_id_with_the_uniform_default_rdt() {
        let ctx = InflightCtx::new(InstructionHeader::Ping {
            id: Id::new(BROADCAST_ID),
        });
        let reply = resolve(ctx);
        assert_eq!(
            reply.slot_offset_bytes,
            TEST_ID as u32 * (PING_STATUS_FRAME_BYTES + PING_REPLY_GUARD_BYTES)
        );
        assert_eq!(reply.rdt_us, crate::dxl::DEFAULT_RDT_US);
        assert_eq!(reply.fast_slot_position, None);
    }
}
