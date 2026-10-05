//! Chip-shape ADC scan layout, and the single definition of the scan geometry:
//! sequence, DMA program, and the arm. Bringup seeds it, the shunt burst draws
//! its frame channels from it, and the burst's restore re-arms from the same
//! statics, so none of them can disagree.
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
//!
//! The terminal taps follow the drive direction. The low-side tap converts on
//! a one-channel injected group triggered `LOW_TAP_LEAD_TICKS` before the
//! crest, closing 37 ticks before it and retiring ahead of the crest trigger;
//! the high-side tap takes slot 1 and closes 83 ticks after it, past its
//! post-edge sag. Slot 2 holds the low side again, closing 135 ticks in: the
//! peak copy is unread, the trough copy serves Fast decay. Forward (A high)
//! injects B over `[shunt, A, B, ..]`, Reverse injects A over
//! `[shunt, B, A, ..]`, so the shunt keeps slot 0 at +31 either way.
//!
//! The motor write at the end of a tick notes a flip; the next frame swaps
//! the order, then maps the scans already taken by the old one. That
//! frame's window already ran in the new direction, so for one period the
//! high side is the early sample, still read under its own tap's name.

use core::cell::SyncUnsafeCell;

use osc_servo_core::regions::burst::{FRAME_MAX, chans};

use crate::hal::clocks::{ADCCLK_HZ, TIM_CLK_HZ};
use crate::hal::{adc, dma, timer};

const TICKS_PER_ADCCLK: u16 = (TIM_CLK_HZ / ADCCLK_HZ) as u16;
/// One conversion at the `CYCLES15` aperture: 13.5 + 12.5 ADCCLK.
const CONV_CYCLES: u16 = 26;
/// External trigger to aperture start.
const TRIGGER_DELAY_CYCLES: u16 = 2;
/// Idle between the injected conversion retiring and the crest TRGO. A
/// regular trigger that lands during an injected conversion is postponed (RM
/// sec 9.2.4), which would move every peak slot off its settle-table instant;
/// the margin also covers the 2-ADCCLK group switch.
const INJECTED_GUARD_CYCLES: u16 = 6;
/// How far ahead of the crest TIM1 triggers the low-side tap's injected
/// conversion.
pub(crate) const LOW_TAP_LEAD_TICKS: u16 =
    (CONV_CYCLES + TRIGGER_DELAY_CYCLES + INJECTED_GUARD_CYCLES) * TICKS_PER_ADCCLK;
/// One regular scan, trigger to retire.
const SCAN_TICKS: u16 =
    (TRIGGER_DELAY_CYCLES + ADC_SCAN_LEN as u16 * CONV_CYCLES) * TICKS_PER_ADCCLK;

/// In `AdcPins` field order: pos, vmotor.0, vmotor.1, vbus, ntc.
pub(crate) const ADC_SENSOR_COUNT: usize = 5;

/// Slot 0 is the current-sense amplifier output, read on whichever external
/// channel the board routes the OPA output to; then both motor terminals
/// (drive-window-critical, so they convert early), then pos, then Vcal,
/// then the slow board taps (rail, NTC) appended last so the bench-validated
/// terminal window floors keep their meaning. Each role's channel comes from
/// the board's `BoardWiring` (`current_sense.opa.out`, `sensors`) via
/// `runtime::init::configure_adc_dma_scan`.
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
/// Each tap's slot in `seq()`, the Forward order.
const SCAN_IDX_VMOTOR_A: usize = 1;
const SCAN_IDX_VMOTOR_B: usize = 2;
/// The programmed scan's terminal slots, whichever tap is high.
pub(super) const SCAN_IDX_TAP_HIGH: usize = 1;
pub(super) const SCAN_IDX_TAP_LOW: usize = 2;
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

/// The frozen slot order, as RSQR holds it under Forward.
pub(crate) fn seq() -> &'static [adc::Channel; ADC_SCAN_LEN] {
    // SAFETY: written only by `seed_seq` pre-IRQ; read-only afterward.
    unsafe { &*SEQ.get() }
}

/// Which terminal the bridge drives high: Forward drives A. The values are
/// the swap mask `terminals` applies.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u16)]
pub(crate) enum Direction {
    Forward = 0,
    Reverse = u16::MAX,
}

impl Direction {
    /// `(high, low)` tap channels.
    fn taps(self, seq: &[adc::Channel; ADC_SCAN_LEN]) -> (adc::Channel, adc::Channel) {
        let (a, b) = (seq[SCAN_IDX_VMOTOR_A], seq[SCAN_IDX_VMOTOR_B]);
        match self {
            Direction::Forward => (a, b),
            Direction::Reverse => (b, a),
        }
    }

    /// `(vmotor_a, vmotor_b)` from the high-side and low-side readings.
    /// Masked, not matched: no branch on the tick path.
    #[inline(always)]
    pub(super) fn terminals(self, high: u16, low: u16) -> (u16, u16) {
        let swap = (high ^ low) & self as u16;
        (high ^ swap, low ^ swap)
    }
}

/// The motor write's last drive; Brake, Coast and torque-off leave it. Both
/// cells are touched only from the DMA1 CH1 vector (kernel tick, burst HT/TC).
static DRIVEN: SyncUnsafeCell<Direction> = SyncUnsafeCell::new(Direction::Forward);
/// The order the taps are programmed in; bringup programs Forward.
static PROGRAMMED: SyncUnsafeCell<Direction> = SyncUnsafeCell::new(Direction::Forward);

pub(crate) fn note_drive(d: Direction) {
    // SAFETY: DMA1 CH1 vector only (DRIVEN doc).
    unsafe { *DRIVEN.get() = d };
}

fn driven() -> Direction {
    // SAFETY: DMA1 CH1 vector only (DRIVEN doc).
    unsafe { *DRIVEN.get() }
}

/// The order the frame being drained was sampled under.
#[inline(always)]
pub(super) fn programmed() -> Direction {
    // SAFETY: DMA1 CH1 vector only (DRIVEN doc).
    unsafe { *PROGRAMMED.get() }
}

/// Swap the taps from `order` to the noted drive. The writes leave the
/// samples already taken alone, so the frame reads them after this.
#[inline(always)]
pub(super) fn follow_drive(order: Direction) {
    let d = driven();
    if d != order {
        swap_taps(d);
    }
}

/// Only while the converter is idle between the peak scan and the trough
/// trigger: an RSQR or ISQR write under a live conversion restarts the group
/// (RM sec 9.2.2) and rotates every slot after it. A frame drained outside
/// that gap leaves the swap to the next.
#[cold]
#[inline(never)]
fn swap_taps(d: Direction) {
    critical_section::with(|_| {
        if swap_gap(timer::counting_down(), timer::counter(), timer::period()) {
            program_taps(d);
        }
    });
}

/// Counting down from the crest, a scan's length clear of the peak scan's
/// trigger and of the trough's.
fn swap_gap(down: bool, cnt: u16, arr: u16) -> bool {
    down && (SCAN_TICKS..=arr.saturating_sub(SCAN_TICKS)).contains(&cnt)
}

/// The whole regular sequence, its taps in the noted drive's order, and the
/// injected tap. Converter idle only.
pub(crate) fn program_sequence() {
    adc::set_sequence(seq());
    program_taps(driven());
}

fn program_taps(d: Direction) {
    let (high, low) = d.taps(seq());
    adc::set_injected_channel(low);
    adc::set_slot(SCAN_IDX_TAP_HIGH, high);
    adc::set_slot(SCAN_IDX_TAP_LOW, low);
    // SAFETY: DMA1 CH1 vector only (DRIVEN doc).
    unsafe { *PROGRAMMED.get() = d };
}

/// Scan slot of each `burst::chans` extra, in the frame order the ABI fixes.
const BURST_EXTRAS: [(u8, usize); 3] = [
    (chans::VMOTOR_A, SCAN_IDX_VMOTOR_A),
    (chans::VMOTOR_B, SCAN_IDX_VMOTOR_B),
    (chans::VBUS, SCAN_IDX_VBUS),
];

/// Program RSQR with one burst frame: the shunt, then each extra `mask`
/// selects, under `chans::INTERLEAVE` each after the first behind a shunt
/// slot of its own. Its length is `burst::frame_len(mask)`, at most 6 of the
/// 16 a regular group holds (RM sec 9.2.2).
pub(crate) fn set_burst_sequence(mask: u8) {
    let (frame, len) = burst_frame(mask, seq());
    adc::set_sequence(&frame[..len]);
}

fn burst_frame(mask: u8, seq: &[adc::Channel; ADC_SCAN_LEN]) -> ([adc::Channel; FRAME_MAX], usize) {
    // Every slot starts as the shunt, so a slot skipped below is one.
    let mut frame = [seq[SCAN_IDX_SHUNT_POST]; FRAME_MAX];
    let mut len = 1;
    for (bit, idx) in BURST_EXTRAS {
        if mask & bit != 0 {
            if mask & chans::INTERLEAVE != 0 && len > 1 {
                len += 1;
            }
            frame[len] = seq[idx];
            len += 1;
        }
    }
    (frame, len)
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

/// The low-side tap as the injected group last converted it: the drive
/// window of the period whose peak scan this frame drains.
#[inline(always)]
pub(super) fn low_tap_injected() -> u16 {
    adc::injected_data()
}

#[cfg(test)]
mod tests {
    extern crate std;

    use osc_servo_core::regions::burst::frame_len;

    use super::*;

    /// Trigger at crest - 68, aperture closes at crest - 37 (2 ADCCLK sync +
    /// 13.5 aperture), retires at crest - 12.
    #[test]
    fn low_tap_lead_places_the_injected_sample_before_the_crest() {
        assert_eq!(TICKS_PER_ADCCLK, 2);
        assert_eq!(LOW_TAP_LEAD_TICKS, 68);
        let aperture_ticks = 27;
        let close = LOW_TAP_LEAD_TICKS - TRIGGER_DELAY_CYCLES * TICKS_PER_ADCCLK - aperture_ticks;
        assert_eq!(close, 37);
        let retire = LOW_TAP_LEAD_TICKS - (TRIGGER_DELAY_CYCLES + CONV_CYCLES) * TICKS_PER_ADCCLK;
        assert_eq!(retire, 12);
    }

    const SEQ_FWD: [adc::Channel; ADC_SCAN_LEN] = {
        use adc::Channel::*;
        [IN0, IN1, IN2, IN3, Vcal, IN5, IN6]
    };
    /// A converter that reads each channel as its own number x 100.
    fn read(ch: adc::Channel) -> u16 {
        ch as u16 * 100
    }

    /// Each direction converts the low side early and the high side in slot
    /// 1, and the frame gets each tap back under its own name.
    #[test]
    fn each_direction_maps_both_taps_back_to_their_terminals() {
        let (a, b) = (read(adc::Channel::IN1), read(adc::Channel::IN2));
        for (d, high, low) in [
            (Direction::Forward, adc::Channel::IN1, adc::Channel::IN2),
            (Direction::Reverse, adc::Channel::IN2, adc::Channel::IN1),
        ] {
            let (h, l) = d.taps(&SEQ_FWD);
            assert_eq!((h as u8, l as u8), (high as u8, low as u8), "{d:?}");
            assert_eq!(d.terminals(read(h), read(l)), (a, b), "{d:?}");
        }
    }

    /// The frame after a flip was sampled under the old order: a noted drive
    /// leaves the order the frame maps by alone, so its taps keep their
    /// names; read under the new order they would trade places.
    #[test]
    fn the_straddle_frame_maps_by_the_order_it_was_sampled_under() {
        let (high, low) = programmed().taps(&SEQ_FWD);
        note_drive(Direction::Reverse);
        let mapped = programmed().terminals(read(high), read(low));
        note_drive(Direction::Forward);
        assert_eq!(programmed(), Direction::Forward);
        assert_eq!(mapped, (read(adc::Channel::IN1), read(adc::Channel::IN2)));
        assert_ne!(Direction::Reverse.terminals(read(high), read(low)), mapped);
    }

    /// The swap lands only counting down, a scan clear of the peak scan's
    /// trigger at the crest and of the trough's at zero.
    #[test]
    fn the_swap_waits_for_the_idle_gap() {
        let arr = 1200;
        assert_eq!(SCAN_TICKS, 368);
        assert!(swap_gap(true, arr - SCAN_TICKS, arr));
        assert!(!swap_gap(true, arr - SCAN_TICKS + 1, arr));
        assert!(swap_gap(true, SCAN_TICKS, arr));
        assert!(!swap_gap(true, SCAN_TICKS - 1, arr));
        assert!(!swap_gap(false, arr / 2, arr));
    }

    /// Every mask the field rule admits programs the frame the ABI names:
    /// its length is `frame_len`, the extras keep bit order, and under
    /// `INTERLEAVE` the shunt holds every even slot.
    #[test]
    fn burst_frame_follows_the_mask() {
        use adc::Channel::*;
        let seq = [IN0, IN1, IN2, IN3, Vcal, IN5, IN6];
        let names = |mask: u8| {
            let (frame, len) = burst_frame(mask, &seq);
            assert_eq!(len, frame_len(mask) as usize, "chans {mask:#06b}");
            frame[..len]
                .iter()
                .map(|c| *c as u8)
                .collect::<std::vec::Vec<u8>>()
        };
        for mask in 0..=chans::ALL {
            names(mask);
        }
        let (s, a, b, v) = (0, 1, 2, 5);
        assert_eq!(names(0), [s]);
        assert_eq!(names(chans::VMOTOR_A), [s, a]);
        assert_eq!(names(chans::VMOTOR_A | chans::VMOTOR_B), [s, a, b]);
        assert_eq!(names(chans::EXTRAS), [s, a, b, v]);
        assert_eq!(names(chans::INTERLEAVE), [s]);
        assert_eq!(names(chans::VMOTOR_B | chans::INTERLEAVE), [s, b]);
        assert_eq!(
            names(chans::VMOTOR_A | chans::VMOTOR_B | chans::INTERLEAVE),
            [s, a, s, b]
        );
        assert_eq!(names(chans::ALL), [s, a, s, b, s, v]);
    }
}
