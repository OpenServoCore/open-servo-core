//! Host-side discovery: the protocol sec 9.2 prefix-tree walk over broadcast ENUM, and
//! the per-baud bus probe built on it. Push-pull UART has no dominant-bit
//! arbitration, so simultaneous ENUM responses arrive as garbage -- and
//! garbage IS the collision signal that drives the descent. Probes ride
//! the adapter's engine (SUBMIT), whose terminal evidence (statuses,
//! garble, trailing energy) is the collision verdict -- the ring counts
//! everything, where a decoded-capture view can lose unanchorable junk.

use std::time::Duration;

use anyhow::{Result, bail};
use osc_protocol::wire::{Id, Inst, MgmtOp, Opcode, ResultCode, UID_LEN};

use crate::wire::Wire;
use crate::{BOOT_BAUD, SUPPORTED_BAUDS};

/// Post-collision settle (protocol sec 8 pacing; the client walk's
/// fleet-measured margin).
const PROBE_SETTLE: Duration = Duration::from_millis(5);

/// What one broadcast ENUM query drew from the bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EnumOutcome {
    /// Nothing answered: empty subtree.
    Silent,
    /// Exactly one servo matched and answered cleanly.
    One { uid: [u8; UID_LEN], id: u8 },
    /// Reply energy that didn't parse: simultaneous responders.
    Collision,
}

/// One broadcast ENUM exchange, classified from the engine's terminal
/// evidence. `prefix` carries `ceil(prefix_len/8)` LSB-first bytes
/// (protocol sec 9.2).
pub fn enum_query(w: &mut Wire, prefix_len: u8, prefix: &[u8]) -> Result<EnumOutcome> {
    let mut payload = vec![MgmtOp::Enum as u8, prefix_len];
    payload.extend_from_slice(prefix);
    let inst = Inst::instruction(Opcode::Mgmt, 0);
    let reply = w.client().exchange(Id::BROADCAST, inst, &payload)?;
    if std::env::var_os("ENUM_TRACE").is_some() {
        eprintln!(
            "-- {} statuses, garble {}, trailing {}",
            reply.statuses.len(),
            reply.garble,
            reply.trailing
        );
    }
    let clean = reply.garble == 0 && !reply.trailing;
    let out = match (reply.statuses.as_slice(), clean) {
        ([], true) => EnumOutcome::Silent,
        ([s], true) if s.result == Some(ResultCode::Ok) && s.payload.len() == UID_LEN => {
            let mut uid = [0u8; UID_LEN];
            uid.copy_from_slice(&s.payload);
            EnumOutcome::One { uid, id: s.id }
        }
        // Garble, trailing energy behind the clean frame, or multiple
        // statuses: overlapping responders.
        _ => EnumOutcome::Collision,
    };
    if out == EnumOutcome::Collision {
        // Parked resolvers need their starve horizon (sec 8) before the
        // next probe reads crisply.
        w.client().pause(PROBE_SETTLE);
    }
    Ok(out)
}

/// Silent is the dangerous verdict (a parked fleet reads as silence, and a
/// pruned subtree is a lost servo): re-probe settled quiet before trusting
/// it -- the client walk's hardening, mirrored.
fn enum_query_settled(w: &mut Wire, prefix_len: u8, prefix: &[u8]) -> Result<EnumOutcome> {
    let mut out = enum_query(w, prefix_len, prefix)?;
    for _ in 0..2 {
        if out != EnumOutcome::Silent {
            return Ok(out);
        }
        w.client().pause(PROBE_SETTLE);
        out = enum_query(w, prefix_len, prefix)?;
    }
    Ok(out)
}

/// One discovered servo.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub uid: [u8; UID_LEN],
    pub id: u8,
}

/// Enumerate every servo at the current baud: DFS over LSB-first bit
/// prefixes, descending one bit on every collision. O(bits * N) exchanges.
pub fn walk(w: &mut Wire) -> Result<Vec<Found>> {
    let mut found = Vec::new();
    let mut stack = vec![(0u8, Vec::new(), false)];
    while let Some((len, prefix, confirmed)) = stack.pop() {
        let out = enum_query_settled(w, len, &prefix)?;
        if std::env::var_os("WALK_TRACE").is_some() {
            eprintln!("walk {len:>3} {prefix:02x?} -> {out:?}");
        }
        match out {
            EnumOutcome::Silent => {}
            // A clean One can be a synchronized-twin superposition reading
            // back as a single frame (protocol sec 9.2) -- confirm it by
            // probing both children once: twins differing at this bit split
            // deterministically; twins agreeing land together in one child
            // where the protocol sec 9.2 slot draw re-rolls (a fresh 1-in-SLOTS shot at
            // unison instead of a permanent hide). A confirmed One -- or one
            // at the full-UID depth, where a lone matcher is structural --
            // is accepted.
            EnumOutcome::One { uid, id } if confirmed || len as usize >= UID_LEN * 8 => {
                found.push(Found { uid, id })
            }
            EnumOutcome::One { uid, .. } => {
                let bit = uid[(len / 8) as usize] >> (len % 8) & 1 == 1;
                let (l, sib) = extend(len, &prefix, !bit);
                let (_, own) = extend(len, &prefix, bit);
                stack.push((l, sib, true));
                stack.push((l, own, true));
            }
            EnumOutcome::Collision if len as usize >= UID_LEN * 8 => {
                bail!("collision at the full 128-bit prefix: duplicate UIDs or persistent garble");
            }
            EnumOutcome::Collision => {
                // Bit-0 subtree explored first (pushed last).
                let (l, one) = extend(len, &prefix, true);
                stack.push((l, one, false));
                let (l, zero) = extend(len, &prefix, false);
                stack.push((l, zero, false));
            }
        }
    }
    Ok(found)
}

/// `prefix` (bit length `len`) extended with one more LSB-first bit.
fn extend(len: u8, prefix: &[u8], bit: bool) -> (u8, Vec<u8>) {
    let mut p = prefix.to_vec();
    if len.is_multiple_of(8) {
        p.push(0);
    }
    if bit {
        p[(len / 8) as usize] |= 1 << (len % 8);
    }
    (len + 1, p)
}

/// Whether anything answers ENUM at the current baud -- a collision counts:
/// garble still means servos are present.
pub fn bus_present(w: &mut Wire) -> Result<bool> {
    Ok(enum_query(w, 0, &[])? != EnumOutcome::Silent)
}

/// Find the bus by ENUM probe: the boot baud first (the common case), then
/// the remaining rates ascending. Leaves the client at the found baud.
pub fn find_bus_baud(w: &mut Wire) -> Result<Option<u32>> {
    let rest = SUPPORTED_BAUDS.iter().copied().filter(|&b| b != BOOT_BAUD);
    for baud in std::iter::once(BOOT_BAUD).chain(rest) {
        w.set_baud(baud)?;
        if bus_present(w)? {
            return Ok(Some(baud));
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extend_grows_the_prefix_lsb_first() {
        // Root -> bit 1 -> bit 0 -> bit 1: prefix bits are uid bit 0, 1, 2.
        let (len, p) = extend(0, &[], true);
        assert_eq!((len, p.as_slice()), (1, [0b001].as_slice()));
        let (len, p) = extend(len, &p, false);
        assert_eq!((len, p.as_slice()), (2, [0b001].as_slice()));
        let (len, p) = extend(len, &p, true);
        assert_eq!((len, p.as_slice()), (3, [0b101].as_slice()));
    }

    #[test]
    fn extend_crosses_byte_boundaries() {
        let mut cur = (0u8, Vec::new());
        for _ in 0..8 {
            cur = extend(cur.0, &cur.1, false);
        }
        assert_eq!(cur.1.len(), 1, "8 bits fit one byte");
        cur = extend(cur.0, &cur.1, true);
        assert_eq!(cur.0, 9);
        assert_eq!(cur.1, [0x00, 0x01], "bit 8 is byte 1 bit 0");
    }
}
