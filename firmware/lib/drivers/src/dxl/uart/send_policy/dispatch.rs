//! Typed instruction walk — from a whole decoded [`Instruction`] to the
//! [`InflightCtx`] the reply gate resolves plus the [`DxlRequest`] the
//! single-shot dispatcher consumes.
//!
//! The framer decodes an own/broadcast instruction frame in one shot; this
//! walk resolves this servo's participation and, for Sync/Bulk chains, walks
//! the typed slot views to accumulate the chain-shape state (slot index,
//! predecessor, Fast wire layout) while extracting this servo's own slot's
//! address / length / write data into the request.

use dxl_protocol::Bytes;
use dxl_protocol::types::packet::Instruction;
use dxl_protocol::unstuff::ByteIter;
use dxl_protocol::wire::BROADCAST_ID;
use osc_core::{DxlRequest, DxlRequestCtx};

use super::inflight::{ChainKind, InflightCtx};

/// Walk `instr` into its [`InflightCtx`] and the request this servo
/// dispatches, if it participates. `wr` backs the destuffed write payload for
/// Write-family requests; the returned request borrows it. The context is
/// produced for participants and non-participants alike (its slot walk still
/// resolves the chain shape for a chain we're absent from).
pub(super) fn walk<'w>(
    instr: &Instruction<'_>,
    broadcast: bool,
    id: u8,
    wr: &'w mut [u8],
) -> (InflightCtx, Option<(DxlRequest<'w>, DxlRequestCtx)>) {
    match *instr {
        Instruction::Ping { id: tid } => {
            let kind = if tid.as_byte() == BROADCAST_ID {
                ChainKind::BroadcastPing
            } else {
                ChainKind::Single
            };
            (
                InflightCtx::new(kind),
                Some((DxlRequest::Ping, ctx(broadcast, true, false))),
            )
        }
        Instruction::Read {
            address, length, ..
        } => (
            InflightCtx::new(ChainKind::Single),
            Some((
                DxlRequest::Read { address, length },
                ctx(broadcast, !broadcast, false),
            )),
        ),
        Instruction::Write { address, data, .. } => {
            let n = destuff(&data, wr);
            (
                InflightCtx::new(ChainKind::Single),
                Some((
                    DxlRequest::Write {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(broadcast, !broadcast, false),
                )),
            )
        }
        Instruction::RegWrite { address, data, .. } => {
            let n = destuff(&data, wr);
            (
                InflightCtx::new(ChainKind::Single),
                Some((
                    DxlRequest::RegWrite {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(broadcast, !broadcast, false),
                )),
            )
        }
        Instruction::Action { .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((DxlRequest::Action, ctx(broadcast, !broadcast, false))),
        ),
        Instruction::Reboot { .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((DxlRequest::Reboot, ctx(broadcast, !broadcast, false))),
        ),
        Instruction::FactoryReset { mode, .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((
                DxlRequest::FactoryReset { mode },
                ctx(broadcast, !broadcast, false),
            )),
        ),
        Instruction::Clear { .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((
                DxlRequest::Clear {
                    body: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            )),
        ),
        Instruction::ControlTableBackup { .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((
                DxlRequest::ControlTableBackup {
                    body: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            )),
        ),
        Instruction::Ext { instruction, .. } => (
            InflightCtx::new(ChainKind::Single),
            Some((
                DxlRequest::Ext {
                    instruction,
                    params: Bytes::raw(&[]),
                },
                ctx(broadcast, !broadcast, false),
            )),
        ),
        Instruction::SyncRead {
            address,
            length,
            ids,
        } => {
            let mut inflight = InflightCtx::new(ChainKind::SyncRead);
            let hit = walk_sync_ids(&mut inflight, &ids, id);
            let request =
                hit.then(|| (DxlRequest::Read { address, length }, ctx(true, true, false)));
            (inflight, request)
        }
        Instruction::FastSyncRead {
            address,
            length,
            ids,
        } => {
            let mut inflight = InflightCtx::new(ChainKind::FastSyncRead { length });
            let hit = walk_sync_ids(&mut inflight, &ids, id);
            let request =
                hit.then(|| (DxlRequest::Read { address, length }, ctx(true, true, true)));
            (inflight, request)
        }
        Instruction::SyncWrite {
            address,
            length,
            body,
        } => {
            let mut inflight = InflightCtx::new(ChainKind::Single);
            let n = walk_sync_write(&mut inflight, length, &body, id, wr);
            let request = n.map(|n| {
                (
                    DxlRequest::Write {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(true, false, false),
                )
            });
            (inflight, request)
        }
        Instruction::BulkRead { body } => {
            let mut inflight = InflightCtx::new(ChainKind::BulkRead);
            let hit = walk_bulk(&mut inflight, &body, id, None);
            let request = hit.map(|(address, length, _)| {
                (DxlRequest::Read { address, length }, ctx(true, true, false))
            });
            (inflight, request)
        }
        Instruction::FastBulkRead { body } => {
            let mut inflight = InflightCtx::new(ChainKind::FastBulkRead);
            let hit = walk_bulk(&mut inflight, &body, id, None);
            let request = hit.map(|(address, length, _)| {
                (DxlRequest::Read { address, length }, ctx(true, true, true))
            });
            (inflight, request)
        }
        Instruction::BulkWrite { body } => {
            let mut inflight = InflightCtx::new(ChainKind::Single);
            let hit = walk_bulk(&mut inflight, &body, id, Some(&mut *wr));
            let request = hit.map(|(address, _length, n)| {
                (
                    DxlRequest::Write {
                        address,
                        data: Bytes::raw(&wr[..n]),
                    },
                    ctx(true, false, false),
                )
            });
            (inflight, request)
        }
    }
}

/// Common ctx constructor.
fn ctx(broadcast: bool, may_reply: bool, slot_reply: bool) -> DxlRequestCtx {
    DxlRequestCtx {
        broadcast,
        may_reply,
        slot_reply,
    }
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

/// Walk the 1-byte-per-slot id list, advancing the aggregator's slot walk.
/// Returns whether our id is present.
fn walk_sync_ids(inflight: &mut InflightCtx, ids: &Bytes<'_>, id: u8) -> bool {
    let mut hit = false;
    for slot_id in ids.iter() {
        inflight.on_slot(slot_id, None, id);
        if slot_id == id {
            hit = true;
        }
    }
    hit
}

/// Walk a SyncWrite body (`[id, data(length)]*`), advancing the slot walk and
/// capturing our slot's data into `wr`. Returns the captured byte count when
/// our id is present.
fn walk_sync_write(
    inflight: &mut InflightCtx,
    length: u16,
    body: &Bytes<'_>,
    id: u8,
    wr: &mut [u8],
) -> Option<usize> {
    let mut it = body.iter();
    let mut found: Option<usize> = None;
    while let Some(slot_id) = it.next() {
        inflight.on_slot(slot_id, None, id);
        if slot_id == id && found.is_none() {
            found = Some(capture(&mut it, length as usize, wr));
        } else {
            skip_n(&mut it, length as usize);
        }
    }
    found
}

/// Walk a Bulk body (`[id, addr16, len16, (data(len) for writes)]*`),
/// advancing the slot walk. Returns `(address, length, data_len)` for our
/// slot. `wr` is `Some` only for BulkWrite (data capture).
fn walk_bulk(
    inflight: &mut InflightCtx,
    body: &Bytes<'_>,
    id: u8,
    mut wr: Option<&mut [u8]>,
) -> Option<(u16, u16, usize)> {
    let write = wr.is_some();
    let mut it = body.iter();
    let mut found: Option<(u16, u16, usize)> = None;
    while let Some(slot_id) = it.next() {
        let (Some(address), Some(length)) = (take_u16(&mut it), take_u16(&mut it)) else {
            break;
        };
        inflight.on_slot(slot_id, Some(length), id);
        let is_ours = slot_id == id && found.is_none();
        if write {
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
