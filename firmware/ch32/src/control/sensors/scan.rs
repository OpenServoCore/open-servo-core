//! Chip-shape ADC scan layout. Two scans per PWM period under center-aligned
//! PWM (peak + trough); UG fires TRGO at CNT=0 before CEN=1 so the trough
//! scan lands at offset 0 and the peak scan lands at `ADC_SCAN_LEN`. Slot
//! indices within a scan reflect the configured RSQR sequence.

use crate::legacy::statics::{ADC_DMA_BUF, ADC_SCAN_LEN};

pub(super) const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
pub(super) const SCAN_TROUGH_OFFSET: usize = 0;

pub(super) const SCAN_IDX_SHUNT_POST: usize = 0;
pub(super) const SCAN_IDX_POS: usize = 1;
pub(super) const SCAN_IDX_NTC: usize = 2;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 3;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 4;
pub(super) const SCAN_IDX_VCAL: usize = 5;

/// Read trough slots before peak; DMA overwrites trough first after TC.
#[inline(always)]
pub(super) fn scan_slot(offset: usize, idx: usize) -> u16 {
    let i = offset + idx;
    debug_assert!(i < 2 * ADC_SCAN_LEN);
    // SAFETY: index bounded above; `ADC_DMA_BUF` is a fixed-length static.
    unsafe { (ADC_DMA_BUF.get() as *const u16).add(i).read_volatile() }
}
