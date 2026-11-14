//! The converter's code domain, common to every channel it samples, and its
//! wide codes: the ADC can stick at a code just under a multiple of 256
//! while its input rises through the codes it then skips, so one code
//! stands for a band of inputs. The wide-code map in CALIB (`CalibAdc`) is
//! per board, since the converter is the chip's: slot `k` belongs to code
//! `(k << ADC_WIDE_SHIFT) | ADC_WIDE_MASK` on any channel and holds half
//! the span the converter skips after it, Q4 counts, 0 = no wide code
//! there. A consumer that adds it reads a stuck code as the middle of its
//! band. The kernel adds it on every channel it consumes, where the
//! consumer reads the sample; the raw fields it publishes stay the
//! converter's codes.

use crate::{RegionStorageRaw, Shared};

pub const ADC_BITS: u32 = 12;
pub const ADC_MASK: u16 = (1 << ADC_BITS) - 1;
/// A wide code has its low `ADC_WIDE_SHIFT` bits all set; its slot is
/// `code >> ADC_WIDE_SHIFT`.
pub const ADC_WIDE_SHIFT: u32 = 8;
pub const ADC_WIDE_MASK: u16 = (1 << ADC_WIDE_SHIFT) - 1;
pub const ADC_WIDE_SLOTS: usize = 1 << (ADC_BITS - ADC_WIDE_SHIFT);
/// Fraction bits of a slot and of a Q4 reading.
pub const ADC_Q4_SHIFT: u32 = 4;
/// The most a wide code reads up in whole counts: the largest slot,
/// rounded half up.
pub const ADC_WIDE_MID_MAX: u32 = (u8::MAX as u32 + (1 << (ADC_Q4_SHIFT - 1))) >> ADC_Q4_SHIFT;

/// The wide-code map's slot for `code`, `< ADC_WIDE_SLOTS`.
#[inline(always)]
pub fn wide_slot(code: u16) -> usize {
    ((code & ADC_MASK) >> ADC_WIDE_SHIFT) as usize
}

/// What `map` adds to a reading at `code`, Q4: its slot on a wide code, 0
/// on every other code. The host's mirror of [`Shared::adc_wide_q4`].
#[inline(always)]
pub fn wide_q4(code: u16, map: &[u8; ADC_WIDE_SLOTS]) -> u16 {
    if code & ADC_WIDE_MASK == ADC_WIDE_MASK {
        map[wide_slot(code)] as u16
    } else {
        0
    }
}

impl Shared {
    /// [`wide_q4`] off the CALIB map: one compare on every code, the
    /// volatile byte load on a wide code only. A slot is one byte, so a
    /// host write lands whole at the next sample, as a take would land it.
    #[inline(always)]
    pub fn adc_wide_q4(&self, code: u16) -> u16 {
        if code & ADC_WIDE_MASK != ADC_WIDE_MASK {
            return 0;
        }
        let p = self.table.region_ptr();
        // SAFETY: `wide_slot` keeps the offset inside the static map; a
        // raw-pointer volatile byte read, no `&` across the ISR boundary.
        unsafe {
            (&raw const (*p).calib.adc.adc_wide_mid_q4)
                .cast::<u8>()
                .add(wide_slot(code))
                .read_volatile() as u16
        }
    }

    /// `code` read as the middle of its band, whole counts, the slot
    /// rounded half up; the code itself off a wide code. The fast tick's
    /// form: one compare on every code, and the result stays in a
    /// register's width, so nothing re-masks it.
    #[inline(always)]
    pub fn adc_mid(&self, code: u16) -> u32 {
        const { assert!(ADC_WIDE_MASK == u8::MAX as u16) };
        let mut v = code as u32;
        if code as u8 == u8::MAX {
            v += (self.adc_wide_q4(code) as u32 + (1 << (ADC_Q4_SHIFT - 1))) >> ADC_Q4_SHIFT;
        }
        v
    }

    /// `code` read as the middle of its band, Q4 counts: for a consumer
    /// that carries fraction bits of its own.
    #[inline(always)]
    pub fn adc_mid_q4(&self, code: u16) -> u32 {
        let mut v = (code as u32) << ADC_Q4_SHIFT;
        if code as u8 == u8::MAX {
            v += self.adc_wide_q4(code) as u32;
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RegionStorage;

    /// The kernel's volatile read of the map is the host mirror for every
    /// u16, codes past the converter's span masked into its slots.
    #[test]
    fn kernel_wide_read_matches_wide_q4_exhaustive() {
        let sh = Shared::new();
        let mut map = [0u8; ADC_WIDE_SLOTS];
        for (s, m) in map.iter_mut().enumerate() {
            *m = 255 - 3 * s as u8;
        }
        sh.table.with_mut(|t| t.calib.adc.adc_wide_mid_q4 = map);
        for code in 0..=u16::MAX {
            assert_eq!(sh.adc_wide_q4(code), wide_q4(code, &map), "code {code}");
        }
        assert_eq!(wide_q4(1023, &map), 255 - 9);
        assert_eq!(wide_q4(1022, &map), 0);
        assert_eq!(wide_q4(1024, &map), 0);
        assert_eq!(wide_q4(0x1FFF, &map), wide_q4(0x0FFF, &map));
    }

    /// A map with a different slot on every wide code, rounding both ways
    /// in whole counts: fractions under, at and over the half.
    fn mixed_map() -> [u8; ADC_WIDE_SLOTS] {
        [
            0, 1, 7, 8, 9, 15, 16, 23, 24, 25, 100, 160, 200, 247, 248, 255,
        ]
    }

    /// The whole-count and Q4 readers add the slot at its stuck code and
    /// nowhere else, for every u16: the whole-count one rounds the slot
    /// half up, the Q4 one carries it whole.
    #[test]
    fn the_mid_readers_add_the_slot_at_its_code_alone() {
        let sh = Shared::new();
        let map = mixed_map();
        sh.table.with_mut(|t| t.calib.adc.adc_wide_mid_q4 = map);
        for code in 0..=u16::MAX {
            let slot = wide_q4(code, &map) as u32;
            assert_eq!(sh.adc_mid(code), code as u32 + ((slot + 8) >> 4), "{code}");
            assert_eq!(sh.adc_mid_q4(code), ((code as u32) << 4) + slot, "{code}");
            if code & ADC_WIDE_MASK != ADC_WIDE_MASK {
                assert_eq!(sh.adc_mid(code), code as u32, "{code}");
                assert_eq!(sh.adc_mid_q4(code), (code as u32) << 4, "{code}");
            }
        }
        // half up: 7 sixteenths round away, 8 round to a count
        assert_eq!(sh.adc_mid(0x2ff), 0x2ff);
        assert_eq!(sh.adc_mid(0x3ff), 0x3ff + 1);
        assert_eq!(sh.adc_mid(0x7ff), 0x7ff + 1);
        assert_eq!(sh.adc_mid(0x8ff), 0x8ff + 2);
        assert_eq!(sh.adc_mid(0xfff), 0xfff + ADC_WIDE_MID_MAX);
        assert_eq!(ADC_WIDE_MID_MAX, 16);
    }

    /// The all-zero map reads every code as itself through both readers.
    #[test]
    fn a_zero_map_reads_every_code_as_itself() {
        let sh = Shared::new();
        for code in 0..=u16::MAX {
            assert_eq!(sh.adc_mid(code), code as u32, "{code}");
            assert_eq!(sh.adc_mid_q4(code), (code as u32) << ADC_Q4_SHIFT, "{code}");
        }
    }
}
