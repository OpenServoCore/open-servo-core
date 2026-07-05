//! Transitional bridge from a whole decoded [`Instruction`] to the existing
//! event-driven [`SendPolicy`] surface plus the [`DxlRequest`] the bus hands
//! the single-shot dispatcher.
//!
//! The framer decodes an own/broadcast instruction frame in one shot, but
//! [`SendPolicy`] still speaks the old streaming vocabulary
//! (`on_instruction_header` → `on_slot`* → `on_crc_good`). This module walks
//! the decoded instruction's typed views and replays that call sequence so
//! the reply-gate state (slot index, predecessor, Fast shape, packet-end)
//! resolves exactly as it did under the parser — while also resolving this
//! servo's participation and extracting its own slot's address / length /
//! write data into a [`DxlRequest`].
//!
//! Transitional: deleted when `send_policy` re-vocabularies onto the framer
//! in a later chunk.

use dxl_protocol::streaming::{InstructionHeader, InstructionPayload};
use dxl_protocol::types::packet::Instruction;
use dxl_protocol::unstuff::ByteIter;
use dxl_protocol::wire::BROADCAST_ID;
use dxl_protocol::{Bytes, Id};
use osc_core::{DxlRequest, DxlRequestCtx};

use super::SendPolicy;
use crate::dxl::uart::poll_src::PollSrc;

/// Replay the decoded instruction onto `policy` (staging the reply context)
/// and derive the request this servo dispatches, if it participates. `wr`
/// backs the destuffed write payload for Write-family requests; the returned
/// request borrows it. `None` when this servo is not a target (own id absent
/// from a Sync/Bulk chain) — the reply gate bookkeeping still ran.
pub(crate) fn synthesize<'w>(
    policy: &mut SendPolicy,
    instr: &Instruction<'_>,
    broadcast: bool,
    packet_end_tick: u32,
    fold_start_cursor: u32,
    src: PollSrc,
    wr: &'w mut [u8],
) -> Option<(DxlRequest<'w>, DxlRequestCtx)> {
    let id = policy.id();
    let request = match *instr {
        Instruction::Ping { id: tid } => {
            policy.on_instruction_header(&InstructionHeader::Ping { id: tid });
            Some((DxlRequest::Ping, ctx(broadcast, true, false)))
        }
        Instruction::Read {
            id: tid,
            address,
            length,
        } => {
            header(
                policy,
                InstructionHeader::Read {
                    id: tid,
                    address,
                    length,
                },
            );
            Some((
                DxlRequest::Read { address, length },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::Write {
            id: tid,
            address,
            data,
        } => {
            let n = destuff(&data, wr);
            header(
                policy,
                InstructionHeader::Write {
                    id: tid,
                    address,
                    length: n as u16,
                },
            );
            Some((
                DxlRequest::Write {
                    address,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::RegWrite {
            id: tid,
            address,
            data,
        } => {
            let n = destuff(&data, wr);
            header(
                policy,
                InstructionHeader::RegWrite {
                    id: tid,
                    address,
                    length: n as u16,
                },
            );
            Some((
                DxlRequest::RegWrite {
                    address,
                    data: Bytes::raw(&wr[..n]),
                },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::Action { id: tid } => {
            header(policy, InstructionHeader::Action { id: tid });
            Some((DxlRequest::Action, ctx(broadcast, !broadcast, false)))
        }
        Instruction::Reboot { id: tid } => {
            header(policy, InstructionHeader::Reboot { id: tid });
            Some((DxlRequest::Reboot, ctx(broadcast, !broadcast, false)))
        }
        Instruction::FactoryReset { id: tid, mode } => {
            header(policy, InstructionHeader::FactoryReset { id: tid, mode });
            Some((
                DxlRequest::FactoryReset { mode },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::Clear { id: tid, .. } => {
            header(policy, InstructionHeader::Clear { id: tid, length: 0 });
            Some((
                DxlRequest::Clear {
                    body: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::ControlTableBackup { id: tid, .. } => {
            header(
                policy,
                InstructionHeader::ControlTableBackup { id: tid, length: 0 },
            );
            Some((
                DxlRequest::ControlTableBackup {
                    body: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::Ext {
            id: tid,
            instruction,
            ..
        } => {
            header(
                policy,
                InstructionHeader::Raw {
                    id: tid,
                    instr: instruction,
                    length: 0,
                },
            );
            Some((
                DxlRequest::Ext {
                    instruction,
                    params: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            ))
        }
        Instruction::SyncRead {
            address,
            length,
            ids,
        } => {
            let hit = walk_sync_ids(policy, false, address, length, &ids, id);
            hit.then(|| (DxlRequest::Read { address, length }, ctx(true, true, false)))
        }
        Instruction::FastSyncRead {
            address,
            length,
            ids,
        } => {
            let hit = walk_sync_ids(policy, true, address, length, &ids, id);
            hit.then(|| (DxlRequest::Read { address, length }, ctx(true, true, true)))
        }
        Instruction::SyncWrite {
            address,
            length,
            body,
        } => {
            let n = walk_sync_write(policy, address, length, &body, id, wr);
            n.map(|n| {
                (
                    DxlRequest::Write {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(true, false, false),
                )
            })
        }
        Instruction::BulkRead { body } => {
            let hit = walk_bulk(policy, HdrCtor::BulkRead, &body, id, None);
            hit.map(|(address, length, _)| {
                (DxlRequest::Read { address, length }, ctx(true, true, false))
            })
        }
        Instruction::FastBulkRead { body } => {
            let hit = walk_bulk(policy, HdrCtor::FastBulkRead, &body, id, None);
            hit.map(|(address, length, _)| {
                (DxlRequest::Read { address, length }, ctx(true, true, true))
            })
        }
        Instruction::BulkWrite { body } => {
            let hit = walk_bulk(policy, HdrCtor::BulkWrite, &body, id, Some(&mut *wr));
            hit.map(|(address, _length, n)| {
                (
                    DxlRequest::Write {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(true, false, false),
                )
            })
        }
    };
    // Stage the reply context on the gate exactly as the parser's Crc-good
    // event did — for participants and non-participants alike (a broadcast
    // chain we're absent from still resets stale predecessor waits).
    policy.on_crc_good(Some(packet_end_tick), fold_start_cursor, src);
    request
}

/// Common ctx constructor.
fn ctx(broadcast: bool, may_reply: bool, slot_reply: bool) -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast,
        may_reply,
        slot_reply,
    }
}

/// Feed a non-chain instruction header to the policy.
fn header(policy: &mut SendPolicy, h: InstructionHeader) {
    policy.on_instruction_header(&h);
}

/// Destuff `data` into `wr`, returning the logical byte count (capped at
/// `wr.len()`).
fn destuff(data: &Bytes<'_>, wr: &mut [u8]) -> usize {
    let mut n = 0;
    for b in data.iter() {
        if n >= wr.len() {
            break;
        }
        wr[n] = b;
        n += 1;
    }
    n
}

/// Build the shared Sync header, feed it, then walk the 1-byte-per-slot id
/// list emitting `SyncSlot` demarcations. Returns whether our id is present.
fn walk_sync_ids(
    policy: &mut SendPolicy,
    fast: bool,
    address: u16,
    length: u16,
    ids: &Bytes<'_>,
    id: u8,
) -> bool {
    let header = if fast {
        InstructionHeader::FastSyncRead {
            id: Id::new(BROADCAST_ID),
            address,
            length,
        }
    } else {
        InstructionHeader::SyncRead {
            id: Id::new(BROADCAST_ID),
            address,
            length,
        }
    };
    policy.on_instruction_header(&header);
    let mut hit = false;
    let mut index = 0u8;
    for slot_id in ids.iter() {
        policy.on_slot(&InstructionPayload::SyncSlot {
            id: Id::new(slot_id),
            index,
        });
        if slot_id == id {
            hit = true;
        }
        index = index.wrapping_add(1);
    }
    hit
}

/// Walk a SyncWrite body (`[id, data(length)]*`), emitting `SyncSlot`
/// demarcations and capturing our slot's data into `wr`. Returns the
/// captured byte count when our id is present.
fn walk_sync_write(
    policy: &mut SendPolicy,
    address: u16,
    length: u16,
    body: &Bytes<'_>,
    id: u8,
    wr: &mut [u8],
) -> Option<usize> {
    policy.on_instruction_header(&InstructionHeader::SyncWrite {
        id: Id::new(BROADCAST_ID),
        address,
        length,
    });
    let mut it = body.iter();
    let mut index = 0u8;
    let mut found: Option<usize> = None;
    while let Some(slot_id) = it.next() {
        policy.on_slot(&InstructionPayload::SyncSlot {
            id: Id::new(slot_id),
            index,
        });
        if slot_id == id && found.is_none() {
            found = Some(capture(&mut it, length as usize, wr));
        } else {
            skip_n(&mut it, length as usize);
        }
        index = index.wrapping_add(1);
    }
    found
}

/// Kind of Bulk instruction, so [`walk_bulk`] can build the right header and
/// decide whether to capture write data.
enum InstructionHeaderCtor {
    BulkRead,
    FastBulkRead,
    BulkWrite,
}

use InstructionHeaderCtor as HdrCtor;

/// Walk a Bulk body (`[id, addr16, len16, (data(len) for writes)]*`),
/// emitting `BulkSlot` demarcations. Returns `(address, length, data_len)`
/// for our slot. `wr` is `Some` only for BulkWrite (data capture).
fn walk_bulk(
    policy: &mut SendPolicy,
    ctor: HdrCtor,
    body: &Bytes<'_>,
    id: u8,
    mut wr: Option<&mut [u8]>,
) -> Option<(u16, u16, usize)> {
    let header = match ctor {
        HdrCtor::BulkRead => InstructionHeader::BulkRead {
            id: Id::new(BROADCAST_ID),
        },
        HdrCtor::FastBulkRead => InstructionHeader::FastBulkRead {
            id: Id::new(BROADCAST_ID),
        },
        HdrCtor::BulkWrite => InstructionHeader::BulkWrite {
            id: Id::new(BROADCAST_ID),
        },
    };
    policy.on_instruction_header(&header);
    let mut it = body.iter();
    let mut index = 0u8;
    let mut found: Option<(u16, u16, usize)> = None;
    while let Some(slot_id) = it.next() {
        let (Some(address), Some(length)) = (take_u16(&mut it), take_u16(&mut it)) else {
            break;
        };
        policy.on_slot(&InstructionPayload::BulkSlot {
            id: Id::new(slot_id),
            index,
            address,
            length,
        });
        let is_ours = slot_id == id && found.is_none();
        if let HdrCtor::BulkWrite = ctor {
            if is_ours {
                let n = match wr.as_deref_mut() {
                    Some(buf) => capture(&mut it, length as usize, buf),
                    None => 0,
                };
                found = Some((address, length, n));
            } else {
                skip_n(&mut it, length as usize);
            }
        } else if is_ours {
            found = Some((address, length, 0));
        }
        index = index.wrapping_add(1);
    }
    found
}

/// Copy the next `n` logical bytes into `dst`, returning the count written.
fn capture(it: &mut ByteIter<'_>, n: usize, dst: &mut [u8]) -> usize {
    let mut w = 0;
    for _ in 0..n {
        match it.next() {
            Some(b) if w < dst.len() => {
                dst[w] = b;
                w += 1;
            }
            Some(_) => {}
            None => break,
        }
    }
    w
}

/// Drop the next `n` logical bytes.
fn skip_n(it: &mut ByteIter<'_>, n: usize) {
    for _ in 0..n {
        if it.next().is_none() {
            break;
        }
    }
}

fn take_u16(it: &mut ByteIter<'_>) -> Option<u16> {
    let lo = it.next()?;
    let hi = it.next()?;
    Some(u16::from_le_bytes([lo, hi]))
}
