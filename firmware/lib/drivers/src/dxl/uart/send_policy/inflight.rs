//! Per-packet wire-state aggregator for an addressed Instruction — the
//! slot-walk cursor and chain-shape accumulators that resolve into a
//! [`ReplyContext`]. Built by [`SendPolicy::on_instruction`] as it walks the
//! decoded instruction's typed slot views.
//!
//! [`SendPolicy::on_instruction`]: super::SendPolicy::on_instruction

use super::fast_shape::{
    PING_REPLY_GUARD_BYTES, PING_STATUS_FRAME_BYTES, compute_fast_position, fast_bytes_before,
    fast_chain_packet_length, fast_first_bytes, fast_successor_bytes,
};
use super::reply_context::ReplyContext;
use crate::dxl::uart::poll_src::PollSrc;

/// Chain shape of the addressed instruction — the header facts the slot walk
/// and [`ReplyContext`] derivation branch on, distilled from the decoded
/// instruction so the aggregator carries no borrows.
#[derive(Copy, Clone, Debug)]
pub(super) enum ChainKind {
    /// Single-target or non-chain reply — the catch-all (unicast, plain
    /// Sync/Bulk Write).
    Single,
    /// Broadcast Ping — replies pack by id at the uniform default RDT.
    BroadcastPing,
    /// Plain Sync Read — slot k > 0 defers to its predecessor's Status.
    SyncRead,
    /// Plain Bulk Read — same predecessor-deferred chain path.
    BulkRead,
    /// FAST Sync Read — uniform per-slot `length` sizes the chain fold.
    FastSyncRead { length: u16 },
    /// FAST Bulk Read — per-slot lengths accumulate across the walk.
    FastBulkRead,
}

/// Per-packet wire-state aggregator. Built while
/// [`SendPolicy::on_instruction`] walks an addressed instruction the chip will
/// reply to; consumed by [`InflightCtx::into_reply_context`] to derive the
/// [`ReplyContext`] the send path needs. State accumulates from the chain kind
/// plus per-slot demarcations.
///
/// [`SendPolicy::on_instruction`]: super::SendPolicy::on_instruction
#[derive(Copy, Clone, Debug)]
pub(super) struct InflightCtx {
    kind: ChainKind,
    /// Slot-walk cursor. Bumped per slot; resolves to the chain's `n_total`.
    next_slot_index: u8,
    /// Index of the chip's own slot once identified. Stays `None` for
    /// non-Sync/Bulk variants and for chains the chip is not in.
    slot: Option<u8>,
    /// Wire bytes preceding the chip's slot — accumulated incrementally
    /// during a FastBulkRead walk (per-slot length varies); derived at
    /// resolve time for FastSyncRead (uniform per-slot length).
    bytes_before: u32,
    /// Per-slot length captured at the chip's own slot, used to size
    /// FastBulkRead's `Middle`/`Last` emission. None for non-Bulk variants.
    slot_length: Option<u16>,
    /// Sum of per-slot data byte counts across the chain — feeds Fast
    /// chain `packet_length` math for First/Only emissions. Accumulated
    /// across a FastBulkRead walk (per-slot length varies); for
    /// FastSyncRead derived in `into_reply_context` from the uniform length
    /// × `n_total`.
    chain_data_bytes: u32,
    /// Slot ID seen at the most recent demarcation before the chip's own
    /// slot lands. Updates per demarcation while `slot` is still `None`;
    /// freezes once `slot` resolves — that's the chip's chain predecessor
    /// for sequence-driven scheduling (`docs/dxl-streaming-rx.md` §5.2).
    /// Stays `None` for slot 0 of any chain and for non-chain instructions.
    predecessor_id: Option<u8>,
}

impl InflightCtx {
    pub(super) fn new(kind: ChainKind) -> Self {
        Self {
            kind,
            next_slot_index: 0,
            slot: None,
            bytes_before: 0,
            slot_length: None,
            chain_data_bytes: 0,
            predecessor_id: None,
        }
    }
}

// -- walk -----------------------------------------------------------------

impl InflightCtx {
    /// Advance the slot walk over one demarcation. `slot_length` is `Some`
    /// only for Bulk chains (per-slot register length); `None` for Sync.
    pub(super) fn on_slot(&mut self, slot_id: u8, slot_length: Option<u16>, id: u8) {
        let k = self.next_slot_index;
        self.next_slot_index = self.next_slot_index.saturating_add(1);
        // Chain-total data byte accumulator runs for every FastBulkRead slot —
        // successor slots included — because First/Only's packet_length is the
        // sum across the whole chain.
        if let Some(length) = slot_length
            && matches!(self.kind, ChainKind::FastBulkRead)
        {
            self.chain_data_bytes = self.chain_data_bytes.saturating_add(length as u32);
        }
        if slot_id == id && self.slot.is_none() {
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
        self.predecessor_id = Some(slot_id);
        // Fast Bulk Read needs per-slot wire counts to size the chain CRC fold
        // (bytes_before on Last). Plain Bulk Read takes the chain-pending path
        // on k > 0, so its predecessor sizes don't matter.
        if let Some(length) = slot_length
            && matches!(self.kind, ChainKind::FastBulkRead)
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
    /// Final ReplyContext at parse-complete. `packet_end_tick` is the codec's
    /// software packet-end estimate; `fold_start_cursor` is the codec's
    /// wire-byte cursor at parse-complete — the cursor where the First
    /// predecessor reply byte will land (the host's chain instruction is fully
    /// consumed by then, so the next wire byte is the First servo's `0xFF`).
    pub(super) fn into_reply_context(
        self,
        id: u8,
        rdt_us: u32,
        packet_end_tick: u32,
        fold_start_cursor: u32,
        src: PollSrc,
    ) -> ReplyContext {
        // Plain SyncRead / BulkRead don't appear in the offset arms: slot 0 of
        // either chain is single-target (slot_offset_bytes = 0 — handled by
        // the catch-all), and slot k > 0 takes the chain-pending path in
        // `ReplyHandle::send_status`, which never reads slot_offset_bytes.
        let (slot_offset_bytes, fast_slot_position, reply_rdt_us) = match (self.kind, self.slot) {
            (ChainKind::BroadcastPing, _) => (
                (id as u32) * (PING_STATUS_FRAME_BYTES + PING_REPLY_GUARD_BYTES),
                None,
                crate::dxl::DEFAULT_RDT_US,
            ),
            (ChainKind::FastSyncRead { length }, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, (n as u32) * (length as u32));
                let position = compute_fast_position(k, packet_length);
                let bytes_before = fast_bytes_before(k, length as u32);
                (bytes_before, Some(position), rdt_us)
            }
            (ChainKind::FastBulkRead, Some(k)) => {
                let n = self.next_slot_index;
                let packet_length = fast_chain_packet_length(n, self.chain_data_bytes);
                let position = compute_fast_position(k, packet_length);
                (self.bytes_before, Some(position), rdt_us)
            }
            _ => (0, None, rdt_us),
        };
        let predecessor_id = match (self.kind, self.slot) {
            (ChainKind::SyncRead | ChainKind::BulkRead, Some(k)) if k > 0 => self.predecessor_id,
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

    // InflightCtx slot walk → ReplyContext derivation

    use dxl_protocol::SlotPosition;

    /// Walk `slots` (`(id, per-slot length)`) into a fresh aggregator.
    fn walk(kind: ChainKind, slots: &[(u8, Option<u16>)]) -> InflightCtx {
        let mut ctx = InflightCtx::new(kind);
        for &(slot_id, length) in slots {
            ctx.on_slot(slot_id, length, TEST_ID);
        }
        ctx
    }

    fn resolve(ctx: InflightCtx) -> ReplyContext {
        ctx.into_reply_context(TEST_ID, 250, 1000, 0, PollSrc::ByteBatch)
    }

    #[test]
    fn slot_walk_freezes_predecessor_at_own_slot() {
        let ctx = walk(
            ChainKind::SyncRead,
            &[(0x30, None), (0x31, None), (TEST_ID, None), (0x33, None)],
        );
        // Plain SyncRead slot k > 0 → the ReplyContext carries the
        // immediate predecessor; the successor slot doesn't overwrite it.
        assert_eq!(resolve(ctx).predecessor_id, Some(0x31));
    }

    #[test]
    fn slot_zero_and_fast_chains_carry_no_predecessor() {
        let at_zero = walk(ChainKind::SyncRead, &[(TEST_ID, None), (0x31, None)]);
        assert_eq!(resolve(at_zero).predecessor_id, None);

        // Fast chains start grid-driven, never sequence-driven.
        let fast = walk(
            ChainKind::FastSyncRead { length: 2 },
            &[(0x30, None), (TEST_ID, None)],
        );
        assert_eq!(resolve(fast).predecessor_id, None);
    }

    #[test]
    fn fast_sync_read_derives_offset_and_position_from_uniform_length() {
        let ctx = walk(
            ChainKind::FastSyncRead { length: 2 },
            &[(0x30, None), (TEST_ID, None), (0x33, None)],
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
        let ctx = walk(
            ChainKind::FastBulkRead,
            &[(0x30, Some(4)), (TEST_ID, Some(2)), (0x33, Some(6))],
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
            ChainKind::FastSyncRead { length: 2 },
            &[(0x30, None), (TEST_ID, None)],
        );
        let reply = resolve(ctx);
        assert_eq!(
            reply.fast_slot_position,
            Some(SlotPosition::Successor { crc: 0 })
        );
        assert_eq!(reply.slot_offset_bytes, 14);
    }

    #[test]
    fn broadcast_ping_positions_by_id_with_the_uniform_default_rdt() {
        let reply = resolve(InflightCtx::new(ChainKind::BroadcastPing));
        assert_eq!(
            reply.slot_offset_bytes,
            TEST_ID as u32 * (PING_STATUS_FRAME_BYTES + PING_REPLY_GUARD_BYTES)
        );
        assert_eq!(reply.rdt_us, crate::dxl::DEFAULT_RDT_US);
        assert_eq!(reply.fast_slot_position, None);
    }
}
