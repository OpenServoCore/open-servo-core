//! Chip-shape ADC scan layout, and the single definition of the scan geometry:
//! sequence, DMA program, and the arm. Bringup seeds it, the shunt burst draws
//! its frame channels from it, and the burst's restore re-arms from the same
//! statics, so none of them can disagree.
//!
//! Two scans per PWM period under center-aligned PWM (peak + trough); UG fires
//! TRGO at CNT=0 before CEN=1 so the trough scan lands at offset 0 and the peak
//! scan lands at `ADC_SCAN_LEN`. Slot indices within a scan reflect the
//! configured RSQR sequence. The DMA lands both straight in the table's tick
//! block (`TelemetryTick`): the trough scan and the peak scan's taps and
//! shunt on unnamed halfwords, the peak scan's slots from `pos` on their
//! fields.
//!
//! Budget at ADCCLK 24 MHz (TCONV = aperture + 12.5, RM sec 9.3; apertures
//! in `cfg::chip`): a slot at `SampleTime::CYCLES15` (a 13.5-cycle aperture)
//! converts in 26 ADCCLK = 52 HCLK = 1.083 us, the Vref slot at `CYCLES43`
//! (41.5) in 54 ADCCLK = 2.25 us, and a 7-slot scan is 210 ADCCLK = 8.75 us.
//!
//! The tick starts on the shunt's own conversion. A two-conversion injected
//! group, triggered `INJECTED_LEAD_TICKS` before the crest, converts the
//! drive's low-side tap, closing 21 ticks before the crest, then the shunt,
//! closing 31 ticks after it; its end (JEOC, `JEOC_TICKS` past the crest) is
//! the kernel's entry, and the shunt is all the motor write reads. The crest
//! TRGO lands during the group and the regular scan is postponed behind it
//! (RM sec 9.2.4), starting `CREST_SCAN_START_TICKS` in: `[high, low, shunt,
//! pos, Vref, rail, NTC]`, the high tap closing 87 ticks in, past its
//! post-edge sag, the pot at +243. The tick waits for that scan after the
//! write. Its low tap (+139) and shunt (+191) are unread. The trough scan
//! takes the same order from its own TRGO, closing the high tap at +31, the
//! low tap at +83 and the shunt, the bias tracker's sample, at +135. Forward
//! (A high) injects B over `[A, B, shunt, ..]`, Reverse injects A over
//! `[B, A, shunt, ..]`.
//!
//! The motor write notes a flip; the first tick whose window gave no reading
//! swaps the order once the crest scan has landed, inside the idle gap ahead
//! of the trough scan, so the next trough scan and injected group sample the
//! noted drive. After a live drive that is the tick of the void window behind
//! the reversal (kernel sec 2.3). A tick that reaches the swap outside the
//! gap leaves it to the next, whose frame was then sampled under the old
//! order while its window ran in the new direction: for one period the high
//! side is the early sample, still read under its own tap's name.

use core::cell::SyncUnsafeCell;
use core::mem::{offset_of, size_of};

use osc_servo_core::regions::TelemetryTick;
use osc_servo_core::regions::burst::{FRAME_MAX, chans};

use crate::hal::clocks::{ADCCLK_HZ, TIM_CLK_HZ};
use crate::hal::{adc, dma, timer};
use crate::runtime::statics::SHARED;

const TICKS_PER_ADCCLK: u16 = (TIM_CLK_HZ / ADCCLK_HZ) as u16;
/// One conversion at the `CYCLES15` aperture: 13.5 + 12.5 ADCCLK.
const CONV_CYCLES: u16 = 26;
/// The Vref slot at the `CYCLES43` aperture: 41.5 + 12.5 ADCCLK.
const VREF_CONV_CYCLES: u16 = 54;
/// External trigger to aperture start.
const TRIGGER_DELAY_CYCLES: u16 = 2;
/// One `CONV_CYCLES` conversion in timer ticks.
pub(crate) const CONV_TICKS: u16 = CONV_CYCLES * TICKS_PER_ADCCLK;
/// How far ahead of the crest TIM1 triggers the injected group: one
/// conversion, the low-side tap's, so the shunt's aperture opens at the
/// trigger delay past the crest, where regular slot 0 opened when the crest
/// TRGO started the scan.
pub(crate) const INJECTED_LEAD_TICKS: u16 = CONV_TICKS;
/// Past the crest, the injected group's end (JEOC): the kernel's trigger.
const JEOC_TICKS: u16 =
    (TRIGGER_DELAY_CYCLES + 2 * CONV_CYCLES) * TICKS_PER_ADCCLK - INJECTED_LEAD_TICKS;
/// The group switch behind the injected group, at ADC prescaler 2 (RM sec
/// 9.2.4).
const GROUP_SWITCH_CYCLES: u16 = 2;
/// Past the crest, where the postponed crest scan opens its first aperture.
/// The RM states the postponement, not this instant.
const CREST_SCAN_START_TICKS: u16 = JEOC_TICKS + GROUP_SWITCH_CYCLES * TICKS_PER_ADCCLK;
/// One regular scan, trigger to retire: every slot but Vref at `CONV_CYCLES`.
const SCAN_TICKS: u16 =
    (TRIGGER_DELAY_CYCLES + (ADC_SCAN_LEN as u16 - 1) * CONV_CYCLES + VREF_CONV_CYCLES)
        * TICKS_PER_ADCCLK;

/// In `AdcPins` field order: pos, vmotor.0, vmotor.1, vbus, ntc.
pub(crate) const ADC_SENSOR_COUNT: usize = 5;

/// Both motor terminals first (drive-window-critical, so they convert
/// early), then the current-sense amplifier output on whichever external
/// channel the board routes the OPA output to, then pos, then Vref, then the
/// slow board taps (rail, NTC). Each role's channel comes from the board's
/// `BoardWiring` (`current_sense.opa.out`, `sensors`) via
/// `runtime::init::configure_adc_dma_scan`.
pub(crate) const ADC_SCAN_LEN: usize = 7;

/// Two scans per PWM period (peak + trough under center-aligned PWM, RCR=0).
pub(crate) const ADC_DMA_BUF_LEN: usize = ADC_SCAN_LEN * 2;

/// Board-resolved channel order, seeded once at bringup from the wiring.
static SEQ: SyncUnsafeCell<[adc::Channel; ADC_SCAN_LEN]> =
    SyncUnsafeCell::new([adc::Channel::IN0; ADC_SCAN_LEN]);

const SCAN_PEAK_OFFSET: usize = ADC_SCAN_LEN;

/// Each tap's slot in `seq()`, the Forward order.
const SCAN_IDX_VMOTOR_A: usize = 0;
const SCAN_IDX_VMOTOR_B: usize = 1;
/// The programmed scan's terminal slots, whichever tap is high.
pub(super) const SCAN_IDX_TAP_HIGH: usize = 0;
pub(super) const SCAN_IDX_TAP_LOW: usize = 1;
pub(super) const SCAN_IDX_SHUNT: usize = 2;
const SCAN_IDX_POS: usize = 3;
const SCAN_IDX_VREF: usize = 4;
const SCAN_IDX_VBUS: usize = 5;
const SCAN_IDX_NTC: usize = 6;

/// The budget's geometry past the injected group's end, in HCLK: the
/// trough's update event a write must beat, the crest scan's end the tick
/// waits for, and the next trough and crest scans' first slot.
const _: () = {
    use osc_servo_core::budget::{
        CREST_SCAN_DONE, NEXT_CREST, NEXT_TROUGH, TICK_PERIOD, WRITE_DEADLINE,
    };
    assert!(TIM_CLK_HZ == crate::hal::clocks::HCLK_HZ);
    let jeoc = JEOC_TICKS as u32;
    let start = CREST_SCAN_START_TICKS as u32;
    let delay = (TRIGGER_DELAY_CYCLES * TICKS_PER_ADCCLK) as u32;
    let conv = CONV_TICKS as u32;
    assert!(WRITE_DEADLINE == TICK_PERIOD / 2 - jeoc);
    assert!(CREST_SCAN_DONE == start + SCAN_TICKS as u32 - delay - jeoc);
    assert!(NEXT_TROUGH == TICK_PERIOD / 2 + delay + conv - jeoc);
    assert!(NEXT_CREST == TICK_PERIOD + start + conv - jeoc);
};

/// The tick block's front holds the DMA buffer: the trough scan and the peak
/// scan's slots ahead of `pos` in `_scan`, each later peak slot on its field.
/// The taps land in sample order, so every field the frame writes lies past
/// the DMA's span and a reader never sees a tap under the other's name.
const _: () = {
    type T = TelemetryTick;
    const fn peak(idx: usize) -> usize {
        (SCAN_PEAK_OFFSET + idx) * size_of::<u16>()
    }
    const SPAN: usize = ADC_DMA_BUF_LEN * size_of::<u16>();
    assert!(offset_of!(T, _scan) == 0);
    assert!(offset_of!(T, pos) == peak(SCAN_IDX_POS));
    assert!(offset_of!(T, vref_raw) == peak(SCAN_IDX_VREF));
    assert!(offset_of!(T, vbus_raw) == peak(SCAN_IDX_VBUS));
    assert!(offset_of!(T, ntc_raw) == peak(SCAN_IDX_NTC));
    assert!(peak(SCAN_IDX_NTC) + size_of::<u16>() == SPAN);
    assert!(offset_of!(T, current_raw) >= SPAN);
    assert!(offset_of!(T, vmotor_a) >= SPAN);
    assert!(offset_of!(T, vmotor_b) >= SPAN);
    assert!(offset_of!(T, current_trough) >= SPAN);
    assert!(offset_of!(T, vmotor_a_trough) >= SPAN);
    assert!(offset_of!(T, vmotor_b_trough) >= SPAN);
};

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
    // the tick enters on the injected group and polls TC; the burst owns
    // the DMA vector
    tcie: false,
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

/// Which terminal the bridge drives high: Forward drives A.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Direction {
    Forward,
    Reverse,
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
    #[inline(always)]
    pub(super) fn terminals(self, high: u16, low: u16) -> (u16, u16) {
        match self {
            Direction::Forward => (high, low),
            Direction::Reverse => (low, high),
        }
    }
}

/// The motor write's last drive; Brake, Coast and torque-off leave it. Both
/// cells are touched only from the ADC vector (kernel tick) and the DMA1 CH1
/// vector (burst HT/TC), both at PFIC HIGH, which never preempt each other.
static DRIVEN: SyncUnsafeCell<Direction> = SyncUnsafeCell::new(Direction::Forward);
/// The order the taps are programmed in; bringup programs Forward.
static PROGRAMMED: SyncUnsafeCell<Direction> = SyncUnsafeCell::new(Direction::Forward);

pub(crate) fn note_drive(d: Direction) {
    // SAFETY: PFIC HIGH vectors only (DRIVEN doc).
    unsafe { *DRIVEN.get() = d };
}

fn driven() -> Direction {
    // SAFETY: PFIC HIGH vectors only (DRIVEN doc).
    unsafe { *DRIVEN.get() }
}

/// The order the frame being drained was sampled under.
#[inline(always)]
pub(super) fn programmed() -> Direction {
    // SAFETY: PFIC HIGH vectors only (DRIVEN doc).
    unsafe { *PROGRAMMED.get() }
}

/// A swap that found the converter busy, run again on the next tick whatever
/// that tick read: one read frame can then be sampled in the old order.
/// Touched only from the ADC vector.
static RETRY: SyncUnsafeCell<bool> = SyncUnsafeCell::new(false);

/// Swap the taps from `order` to the noted drive, when `swap_due`. The
/// writes leave the samples already taken alone, so the frame reads them
/// after this.
#[inline(always)]
pub(super) fn follow_drive(order: Direction, read: bool) {
    let d = driven();
    // SAFETY: the ADC vector only (RETRY doc).
    if swap_due(d, order, read, unsafe { *RETRY.get() }) {
        swap_taps(d);
    }
}

/// On the first tick whose window gave no reading (`read` false). A flip
/// written on a tick that read its window follows a live drive, so the next
/// window is the void one after the reversal (kernel sec 2.3): swapped on
/// that tick, every read frame was sampled in its drive's order. A flip from
/// no drive is written on a tick that read nothing and swaps there.
#[inline(always)]
fn swap_due(driven: Direction, order: Direction, read: bool, retry: bool) -> bool {
    if driven != order {
        core::hint::cold_path();
        !read || retry
    } else {
        false
    }
}

/// Only while the converter is idle between the peak scan and the trough
/// trigger: an RSQR or ISQR write under a live conversion restarts the group
/// (RM sec 9.2.2) and rotates every slot after it. A frame completed outside
/// that gap leaves the swap to the next tick.
#[cold]
#[inline(never)]
#[rustc_align(4)]
fn swap_taps(d: Direction) {
    critical_section::with(|_| {
        let idle = swap_gap(timer::counting_down(), timer::counter(), timer::period());
        if idle {
            program_taps(d);
        }
        // SAFETY: the ADC vector only (RETRY doc).
        unsafe { *RETRY.get() = !idle };
    });
}

/// Counting down from the crest, past the postponed peak scan's end and a
/// conversion clear of the trough's trigger. The swap runs once the peak
/// scan has landed, after the motor write, so it must fit the stretch to
/// the trough, not only its first half.
fn swap_gap(down: bool, cnt: u16, arr: u16) -> bool {
    down && (CONV_TICKS..=arr.saturating_sub(CREST_SCAN_START_TICKS + SCAN_TICKS)).contains(&cnt)
}

/// The whole regular sequence, its taps in the noted drive's order, and the
/// injected pair. Converter idle only.
pub(crate) fn program_sequence() {
    adc::set_sequence(seq());
    program_taps(driven());
}

fn program_taps(d: Direction) {
    let (high, low) = d.taps(seq());
    adc::set_injected_channels(low, seq()[SCAN_IDX_SHUNT]);
    adc::set_slot(SCAN_IDX_TAP_HIGH, high);
    adc::set_slot(SCAN_IDX_TAP_LOW, low);
    // SAFETY: PFIC HIGH vectors only (DRIVEN doc).
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
    let mut frame = [seq[SCAN_IDX_SHUNT]; FRAME_MAX];
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

/// Point CH1 at the tick block and enable it. Leaves `ADC.CTLR2.DMA` alone:
/// the request source is the caller's to open (a request pulsed at a disabled
/// channel latches on this die and fires at re-enable).
pub(crate) fn arm_dma() {
    dma::configure(
        dma::Channel::CH1,
        &DMA_CFG,
        adc::data_addr(),
        SHARED.tick_block() as u32,
        ADC_DMA_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH1);
}

/// Slot `idx` of the trough scan as the DMA landed it. Drained by the frame
/// ahead of the next trough scan.
#[inline(always)]
pub(super) fn trough_slot(tick: *const TelemetryTick, idx: usize) -> u16 {
    debug_assert!(idx < ADC_SCAN_LEN);
    scan_slot(tick, idx)
}

/// Slot `idx` of the peak scan as the DMA landed it, for the slots ahead of
/// `pos`.
#[inline(always)]
pub(super) fn peak_slot(tick: *const TelemetryTick, idx: usize) -> u16 {
    debug_assert!(idx < SCAN_IDX_POS);
    scan_slot(tick, SCAN_PEAK_OFFSET + idx)
}

#[inline(always)]
fn scan_slot(tick: *const TelemetryTick, idx: usize) -> u16 {
    // SAFETY: `tick` is the table's tick block; `idx` lies inside `_scan`
    // (callers); the DMA writes it, so the read is volatile.
    unsafe {
        (&raw const (*tick)._scan)
            .cast::<u16>()
            .add(idx)
            .read_volatile()
    }
}

/// The low-side tap, the injected group's first conversion: the drive
/// window of the period whose peak scan this frame completes.
#[inline(always)]
pub(super) fn low_tap_injected() -> u16 {
    adc::injected_data(0)
}

/// The shunt, the injected group's second conversion, closing 31 ticks past
/// the crest.
#[inline(always)]
pub(super) fn shunt_injected() -> u16 {
    adc::injected_data(1)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use osc_servo_core::regions::burst::frame_len;

    use super::*;

    /// Trigger at crest - 52 (2 ADCCLK sync, then a 13.5-cycle aperture per
    /// conversion): the low tap's aperture closes at crest - 21, the
    /// shunt's at crest + 31, the group ends (JEOC) at crest + 56 and the
    /// postponed regular scan opens at crest + 60, its high tap closing at
    /// +87 and its pot at +243.
    #[test]
    fn the_injected_pair_places_the_shunt_where_slot_0_was() {
        assert_eq!(TICKS_PER_ADCCLK, 2);
        assert_eq!(INJECTED_LEAD_TICKS, 52);
        let aperture_ticks = 27;
        let opens = |k: u16| (TRIGGER_DELAY_CYCLES + k * CONV_CYCLES) * TICKS_PER_ADCCLK;
        let rel = |t: u16| t as i32 - INJECTED_LEAD_TICKS as i32;
        assert_eq!(rel(opens(0) + aperture_ticks), -21);
        assert_eq!(rel(opens(1) + aperture_ticks), 31);
        assert_eq!(rel(opens(2)), JEOC_TICKS as i32);
        assert_eq!((JEOC_TICKS, CREST_SCAN_START_TICKS), (56, 60));
        let closes = |slot: u16| {
            CREST_SCAN_START_TICKS + slot * CONV_CYCLES * TICKS_PER_ADCCLK + aperture_ticks
        };
        assert_eq!(closes(SCAN_IDX_TAP_HIGH as u16), 87);
        assert_eq!(closes(SCAN_IDX_POS as u16), 243);
    }

    const SEQ_FWD: [adc::Channel; ADC_SCAN_LEN] = {
        use adc::Channel::*;
        [IN1, IN2, IN0, IN3, Vref, IN5, IN6]
    };
    /// A converter that reads each channel as its own number x 100.
    fn read(ch: adc::Channel) -> u16 {
        ch as u16 * 100
    }

    /// Each direction converts the low side early and the high side in slot
    /// 0, and the frame gets each tap back under its own name.
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

    const DIRECTIONS: [Direction; 2] = [Direction::Forward, Direction::Reverse];

    fn flipped(d: Direction) -> Direction {
        match d {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        }
    }

    #[test]
    fn no_flip_never_swaps() {
        for d in DIRECTIONS {
            for (read, retry) in [(false, false), (false, true), (true, false), (true, true)] {
                assert!(
                    !swap_due(d, d, read, retry),
                    "{d:?} read {read} retry {retry}"
                );
            }
        }
    }

    /// A flip from no drive is written on a tick that read nothing.
    #[test]
    fn a_flip_on_a_tick_that_read_nothing_swaps() {
        for d in DIRECTIONS {
            assert!(swap_due(d, flipped(d), false, false), "{d:?}");
        }
    }

    /// A flip after a live drive waits for the void window behind it.
    #[test]
    fn a_flip_on_a_tick_that_read_its_window_waits() {
        for d in DIRECTIONS {
            assert!(!swap_due(d, flipped(d), true, false), "{d:?}");
        }
    }

    #[test]
    fn a_pending_retry_swaps_whatever_the_tick_read() {
        for d in DIRECTIONS {
            for read in [false, true] {
                assert!(swap_due(d, flipped(d), read, true), "{d:?} read {read}");
            }
        }
    }

    /// The swap lands only counting down, from the postponed peak scan's
    /// end (484 past the crest, counter 716) to a conversion clear of the
    /// trough's trigger at zero (counter 52).
    #[test]
    fn the_swap_waits_for_the_idle_gap() {
        let arr = 1200;
        assert_eq!(SCAN_TICKS, 424);
        assert!(swap_gap(true, 716, arr));
        assert!(!swap_gap(true, 717, arr));
        assert!(swap_gap(true, 52, arr));
        assert!(!swap_gap(true, 51, arr));
        assert!(!swap_gap(false, arr / 2, arr));
    }

    /// Every mask the field rule admits programs the frame the ABI names:
    /// its length is `frame_len`, the extras keep bit order, and under
    /// `INTERLEAVE` the shunt holds every even slot.
    #[test]
    fn burst_frame_follows_the_mask() {
        use adc::Channel::*;
        let seq = [IN1, IN2, IN0, IN3, Vref, IN5, IN6];
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
