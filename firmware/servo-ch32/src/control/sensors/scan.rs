//! Chip-shape ADC scan layout. Two scans per PWM period under center-aligned
//! PWM (peak + trough); UG fires TRGO at CNT=0 before CEN=1 so the trough
//! scan lands at offset 0 and the peak scan lands at `ADC_SCAN_LEN`. Slot
//! indices within a scan reflect the configured RSQR sequence.
//!
//! Budget at ADCCLK 48 MHz (TCONV = aperture + 12.5, RM sec 9.3; apertures
//! in `cfg::chip`): the shunt slot is 3.5 + 12.5 = 16 cycles = 0.33 us, the
//! other six 7.5 + 12.5 = 20 cycles = 0.42 us each, so a scan is 136 cycles
//! = 2.83 us. vmA's S/H closes 16 + 7.5 = 23.5 cycles = 0.49 us = 24 TIM1
//! ticks after the trigger, vmB's at 43.5 cycles = 0.91 us = 44 ticks. The
//! peak scan's TC lands 2.8 us after its trigger and the trough trigger
//! follows 25 us after it, so the TC ISR has ~22 us to drain the trough
//! slots before slot 0 is rewritten; the peak slots stand until the next
//! peak trigger.

use core::cell::SyncUnsafeCell;

/// In `AdcPins` field order: pos, vmotor.0, vmotor.1, vbus, ntc.
pub(crate) const ADC_SENSOR_COUNT: usize = 5;

/// Slot 0 is the current-sense amplifier output, read on whichever external
/// channel the board routes the OPA output to; then both motor terminals
/// (drive-window-critical, so they convert early), then pos, then Vcal,
/// then the slow board taps (rail, NTC) appended last so the bench-validated
/// terminal window floors keep their meaning. On osc-dev-v006 that is
/// `[IN7/PD4 current, IN5/PD5 vmA, IN6/PD6 vmB, IN3/PD2 pos, IN10/Vcal,
/// IN0/PA2 vbus, IN2/PC4 ntc]`.
pub(crate) const ADC_SCAN_LEN: usize = 7;

/// Two scans per PWM period (peak + trough under center-aligned PWM, RCR=0).
pub(crate) const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

pub(crate) static ADC_DMA_BUF: SyncUnsafeCell<[u16; ADC_DMA_BUF_LEN]> =
    SyncUnsafeCell::new([0; ADC_DMA_BUF_LEN]);

pub(super) const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
pub(super) const SCAN_TROUGH_OFFSET: usize = 0;

pub(super) const SCAN_IDX_SHUNT_POST: usize = 0;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 1;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 2;
pub(super) const SCAN_IDX_POS: usize = 3;
pub(super) const SCAN_IDX_VCAL: usize = 4;
pub(super) const SCAN_IDX_VBUS: usize = 5;
pub(super) const SCAN_IDX_NTC: usize = 6;

/// Read trough slots before peak; DMA overwrites trough first after TC.
#[inline(always)]
pub(super) fn scan_slot(offset: usize, idx: usize) -> u16 {
    let i = offset + idx;
    debug_assert!(i < 2 * ADC_SCAN_LEN);
    // SAFETY: index bounded above; `ADC_DMA_BUF` is a fixed-length static.
    unsafe { (ADC_DMA_BUF.get() as *const u16).add(i).read_volatile() }
}
