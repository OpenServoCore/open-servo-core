//! Host-side discovery: the §9.2 prefix-tree walk over broadcast ENUM, and
//! the per-baud bus probe built on it. Push-pull UART has no dominant-bit
//! arbitration, so simultaneous ENUM responses arrive as garbage — and
//! garbage IS the collision signal that drives the descent.

use anyhow::{Result, bail};
use osc_protocol::wire::{ResultCode, UID_LEN};

use crate::cli::SETTLE_MS;
use crate::osc::{ExchangeError, build_enum, parse_exchange};
use crate::pirate::Client;
use crate::run::capture;
use crate::{BOOT_BAUD, SUPPORTED_BAUDS};

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

/// One broadcast ENUM exchange, classified. `prefix` carries
/// `ceil(prefix_len/8)` LSB-first bytes (§9.2).
pub fn enum_query(client: &mut Client, prefix_len: u8, prefix: &[u8]) -> Result<EnumOutcome> {
    let wire = build_enum(prefix_len, prefix);
    let (stamps, bit_ticks) = capture(client, &wire, SETTLE_MS)?;
    Ok(match parse_exchange(&stamps, &wire, bit_ticks) {
        Ok(ex)
            if ex.status.result == Some(ResultCode::Ok) && ex.status.payload.len() == UID_LEN =>
        {
            let mut uid = [0u8; UID_LEN];
            uid.copy_from_slice(&ex.status.payload);
            EnumOutcome::One {
                uid,
                id: ex.status.id,
            }
        }
        // A frame that parsed but isn't a clean OK+UID is overlapping
        // responders whose bytes happened to still frame.
        Ok(_) => EnumOutcome::Collision,
        Err(ExchangeError::NoReply) => EnumOutcome::Silent,
        Err(ExchangeError::NoEcho) => bail!("ENUM query never reached the wire (no echo)"),
        Err(_) => EnumOutcome::Collision,
    })
}

/// One discovered servo.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub uid: [u8; UID_LEN],
    pub id: u8,
}

/// Enumerate every servo at the current baud: DFS over LSB-first bit
/// prefixes, descending one bit on every collision. O(bits · N) exchanges.
pub fn walk(client: &mut Client) -> Result<Vec<Found>> {
    let mut found = Vec::new();
    let mut stack = vec![(0u8, Vec::new())];
    while let Some((len, prefix)) = stack.pop() {
        let out = enum_query(client, len, &prefix)?;
        if std::env::var_os("WALK_TRACE").is_some() {
            eprintln!("walk {len:>3} {prefix:02x?} -> {out:?}");
        }
        match out {
            EnumOutcome::Silent => {}
            EnumOutcome::One { uid, id } => found.push(Found { uid, id }),
            EnumOutcome::Collision if len as usize >= UID_LEN * 8 => {
                bail!("collision at the full 128-bit prefix: duplicate UIDs or persistent garble");
            }
            EnumOutcome::Collision => {
                // Bit-0 subtree explored first (pushed last).
                stack.push(extend(len, &prefix, true));
                stack.push(extend(len, &prefix, false));
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

/// Whether anything answers ENUM at the current baud — a collision counts:
/// garble still means servos are present.
pub fn bus_present(client: &mut Client) -> Result<bool> {
    Ok(enum_query(client, 0, &[])? != EnumOutcome::Silent)
}

/// Find the bus by ENUM probe: the boot baud first (the common case), then
/// the remaining rates ascending. Leaves the client at the found baud.
pub fn find_bus_baud(client: &mut Client) -> Result<Option<u32>> {
    let rest = SUPPORTED_BAUDS.iter().copied().filter(|&b| b != BOOT_BAUD);
    for baud in std::iter::once(BOOT_BAUD).chain(rest) {
        client.set_baud(baud)?;
        client.reset()?;
        if bus_present(client)? {
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
        // Root → bit 1 → bit 0 → bit 1: prefix bits are uid bit 0, 1, 2.
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
