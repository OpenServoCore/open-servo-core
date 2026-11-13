//! The converter's code domain, common to every channel it samples, and its
//! wide codes: the ADC can stick at a code just under a multiple of 256
//! while its input rises through the codes it then skips, so one code
//! stands for a band of inputs. The wide-code map in CALIB (`CalibAdc`) is
//! per board, since the converter is the chip's: slot `k` belongs to code
//! `(k << ADC_WIDE_SHIFT) | ADC_WIDE_MASK` on any channel and holds half
//! the span the converter skips after it, Q4 counts, 0 = no wide code
//! there. A consumer that adds it reads a stuck code as the middle of its
//! band.

use crate::{RegionStorageRaw, Shared};

pub const ADC_BITS: u32 = 12;
pub const ADC_MASK: u16 = (1 << ADC_BITS) - 1;
/// A wide code has its low `ADC_WIDE_SHIFT` bits all set; its slot is
/// `code >> ADC_WIDE_SHIFT`.
pub const ADC_WIDE_SHIFT: u32 = 8;
pub const ADC_WIDE_MASK: u16 = (1 << ADC_WIDE_SHIFT) - 1;
pub const ADC_WIDE_SLOTS: usize = 1 << (ADC_BITS - ADC_WIDE_SHIFT);

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
}
