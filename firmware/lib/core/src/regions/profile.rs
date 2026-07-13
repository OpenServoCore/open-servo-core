//! PROFILE region (osc-native sec 5.2): span-granular read profiles. A READ or
//! GREAD carrying the PROFILE flag names a slot; the reply streams the slot's
//! spans back-to-back through the copy-once TX path, so scattered telemetry
//! costs the wire one slot byte per cycle instead of a span list.
//!
//! Span word: `[addr:10][count:6]`, `count = 0` = word disabled. Disabled
//! words are SKIPPED, not terminators -- a host can toggle one span with a
//! single 2-byte write. The all-zero boot image is an empty slot. Any u16 is
//! a valid word, so the region carries no field rules; span bounds and the
//! reply ceiling are checked at read time (sec 5.3: `range` / `limit`).

use control_table::{Block, Section};

/// Profile slots in the region.
pub const PROFILE_SLOTS: u8 = 4;
/// Span words per slot.
pub const SPANS_PER_SLOT: usize = 8;

/// Pack a span word. `addr` caps at 10 bits (1023), `count` at 6 bits (63);
/// out-of-range inputs are the caller's bug (host-side builders validate).
#[inline]
pub const fn span_word(addr: u16, count: u8) -> u16 {
    (addr << 6) | (count as u16 & 0x3F)
}

/// Unpack a span word to `(addr, count)`; `None` for a disabled word.
#[inline]
pub const fn span_of(word: u16) -> Option<(u16, u16)> {
    let count = word & 0x3F;
    if count == 0 {
        return None;
    }
    Some((word >> 6, count))
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ProfileSlots {
    pub words: [u16; PROFILE_SLOTS as usize * SPANS_PER_SLOT],
}

#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::PROFILE_BASE_ADDR,
    size = crate::regions::PROFILE_REGION_SIZE,
)]
pub struct ProfileRegs {
    pub slots: ProfileSlots,
}

impl ProfileRegs {
    /// The span words of `slot`; `None` past the last slot (sec 5.3: `range`).
    #[inline]
    pub fn slot_words(&self, slot: u8) -> Option<&[u16; SPANS_PER_SLOT]> {
        if slot >= PROFILE_SLOTS {
            return None;
        }
        let at = slot as usize * SPANS_PER_SLOT;
        self.slots.words[at..at + SPANS_PER_SLOT].try_into().ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn span_word_packs_and_unpacks() {
        assert_eq!(span_of(span_word(0x0200, 8)), Some((0x0200, 8)));
        assert_eq!(span_of(span_word(1023, 63)), Some((1023, 63)));
        assert_eq!(span_of(span_word(1, 1)), Some((1, 1)));
        // count 0 = disabled, at any address.
        assert_eq!(span_of(span_word(0x0200, 0)), None);
        assert_eq!(span_of(0), None);
    }

    #[test]
    fn slot_words_bounds() {
        let r = ProfileRegs::new();
        assert!(r.slot_words(0).is_some());
        assert!(r.slot_words(PROFILE_SLOTS - 1).is_some());
        assert!(r.slot_words(PROFILE_SLOTS).is_none());
    }
}
