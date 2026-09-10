//! Chip-shape ADC scan layout, and the single definition of the scan geometry:
//! sequence, DMA program, and the arm. Bringup seeds it and the shunt burst's
//! restore re-arms from the same statics, so the two can never disagree.
//!
//! Two scans per PWM period under center-aligned PWM (peak + trough); UG fires
//! TRGO at CNT=0 before CEN=1 so the trough scan lands at offset 0 and the peak
//! scan lands at `ADC_SCAN_LEN`. Slot indices within a scan reflect the
//! configured RSQR sequence.
//!
//! Budget at ADCCLK 24 MHz (TCONV = aperture + 12.5, RM sec 9.3; every slot
//! runs `SampleTime::CYCLES15` = a 13.5-cycle aperture, apertures in
//! `cfg::chip`): a conversion is 26 ADCCLK = 52 HCLK = 1.083 us and a 7-slot
//! scan is 182 ADCCLK = 7.58 us. The peak scan's TC lands 7.6 us after its
//! trigger and the trough trigger follows 25 us after that trigger, so the TC
//! ISR has ~17 us to drain the trough slots before slot 0 is rewritten; the
//! peak slots stand until the next peak trigger.

use core::cell::SyncUnsafeCell;

use crate::hal::{adc, dma};

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

/// Board-resolved channel order, seeded once at bringup from the wiring.
static SEQ: SyncUnsafeCell<[adc::Channel; ADC_SCAN_LEN]> =
    SyncUnsafeCell::new([adc::Channel::IN0; ADC_SCAN_LEN]);

pub(super) const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;
pub(super) const SCAN_TROUGH_OFFSET: usize = 0;

pub(super) const SCAN_IDX_SHUNT_POST: usize = 0;
pub(super) const SCAN_IDX_VMOTOR_A: usize = 1;
pub(super) const SCAN_IDX_VMOTOR_B: usize = 2;
pub(super) const SCAN_IDX_POS: usize = 3;
pub(super) const SCAN_IDX_VCAL: usize = 4;
pub(super) const SCAN_IDX_VBUS: usize = 5;
pub(super) const SCAN_IDX_NTC: usize = 6;

/// HIGH, not VERYHIGH: RX (CH5) alone owns the top so an inbound byte's drain
/// outranks everything (see the ladder in `hal::dma`). ADC is the
/// lowest-numbered HIGH channel, so it still wins every HIGH tie and only ever
/// yields to the sparse RX drain.
const DMA_CFG: dma::Config = dma::Config {
    dir: dma::Dir::FROMPERIPHERAL,
    circ: true,
    pinc: false,
    minc: true,
    size: dma::Size::BITS16,
    htie: false,
    tcie: true,
    pl: dma::Pl::HIGH,
};

/// Install the board's channel order and program RSQR from it. Bringup-only,
/// pre-IRQ; sole writer.
pub(crate) fn seed_seq(channels: [adc::Channel; ADC_SCAN_LEN]) {
    // SAFETY: see fn doc -- no reader exists before IRQs enable.
    unsafe { *SEQ.get() = channels };
    adc::set_sequence(seq());
}

/// The frozen slot order, as RSQR must be programmed.
pub(crate) fn seq() -> &'static [adc::Channel] {
    // SAFETY: written only by `seed_seq` pre-IRQ; read-only afterward.
    unsafe { &*SEQ.get() }
}

/// The shunt channel, from the same definition the slot indices name.
pub(crate) fn shunt_channel() -> adc::Channel {
    seq()[SCAN_IDX_SHUNT_POST]
}

/// Point CH1 at `ADC_DMA_BUF` and enable it. Leaves `ADC.CTLR2.DMA` alone:
/// the request source is the caller's to open (a request pulsed at a disabled
/// channel latches on this die and fires at re-enable).
pub(crate) fn arm_dma() {
    dma::configure(
        dma::Channel::CH1,
        &DMA_CFG,
        adc::data_addr(),
        ADC_DMA_BUF.get() as u32,
        ADC_DMA_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH1);
}

/// Read trough slots before peak; DMA overwrites trough first after TC.
#[inline(always)]
pub(super) fn scan_slot(offset: usize, idx: usize) -> u16 {
    let i = offset + idx;
    debug_assert!(i < 2 * ADC_SCAN_LEN);
    // SAFETY: index bounded above; `ADC_DMA_BUF` is a fixed-length static.
    unsafe { (ADC_DMA_BUF.get() as *const u16).add(i).read_volatile() }
}
