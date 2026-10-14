//! Chip-side sensor acquisition. The ADC DMA lands both scans of each PWM
//! period in the table's tick block (`scan`); the 20 kHz control-loop ISR
//! completes the frame there, where the kernel reads it. No unit conversion
//! here: the servo stays in the counts domain, the host owns engineering
//! units.

pub(crate) mod scan;

use osc_servo_core::regions::TelemetryTick;
use osc_servo_core::{Sensors as SensorsTrait, Shared};

use scan::{
    Direction, SCAN_IDX_SHUNT_POST, SCAN_IDX_TAP_HIGH, SCAN_IDX_TAP_LOW, low_tap_injected,
    peak_slot, trough_slot,
};

#[derive(Default)]
pub struct Ch32Sensors;

impl Ch32Sensors {
    pub const fn new() -> Self {
        Self
    }
}

impl SensorsTrait for Ch32Sensors {
    /// Called from the DMA1 TC ISR, ahead of the next trough scan. Maps by
    /// the order the frame was sampled under, after swapping the order for
    /// a noted drive flip.
    fn land(&mut self, shared: &Shared) {
        let order = scan::programmed();
        scan::follow_drive(order);
        // SAFETY: the tick block of the static table; this vector is the
        // frame's sole CPU writer.
        unsafe { publish(shared.tick_block(), order, low_tap_injected) };
    }
}

/// The frame's fields from the scans in `_scan`: the trough's shunt and taps
/// out of the next trough scan's way, the crest's shunt, and each tap under
/// its terminal's name, the crest's low side from `low` (the injected group).
///
/// # Safety
/// `t` is valid for reads and writes, and no other context writes the
/// frame's fields meanwhile.
#[inline(always)]
unsafe fn publish(t: *mut TelemetryTick, order: Direction, low: impl FnOnce() -> u16) {
    let (a_trough, b_trough) = order.terminals(
        trough_slot(t, SCAN_IDX_TAP_HIGH),
        trough_slot(t, SCAN_IDX_TAP_LOW),
    );
    // SAFETY: fn contract; the DMA leaves these fields alone (`scan`'s
    // layout asserts); volatile per-field accesses.
    unsafe {
        (&raw mut (*t).current_trough).write_volatile(trough_slot(t, SCAN_IDX_SHUNT_POST));
        (&raw mut (*t).vmotor_a_trough).write_volatile(a_trough);
        (&raw mut (*t).vmotor_b_trough).write_volatile(b_trough);
        (&raw mut (*t).current_raw).write_volatile(peak_slot(t, SCAN_IDX_SHUNT_POST));
        let (a, b) = order.terminals(peak_slot(t, SCAN_IDX_TAP_HIGH), low());
        (&raw mut (*t).vmotor_a).write_volatile(a);
        (&raw mut (*t).vmotor_b).write_volatile(b);
    }
}

#[cfg(test)]
mod tests {
    use core::mem::size_of;

    use super::scan::{ADC_DMA_BUF_LEN, ADC_SCAN_LEN};
    use super::*;

    const SHUNT: u16 = 0x0100;
    const A: u16 = 0x0A00;
    const B: u16 = 0x0B00;
    const POS: u16 = 0x0400;

    /// One period's DMA image under `order`: both scans `[shunt, high, low,
    /// pos, ..]`, the crest's samples one count above the trough's.
    fn scans(order: Direction) -> [u16; ADC_DMA_BUF_LEN] {
        let (high, low) = match order {
            Direction::Forward => (A, B),
            Direction::Reverse => (B, A),
        };
        let mut img = [0; ADC_DMA_BUF_LEN];
        for (k, scan) in img.chunks_mut(ADC_SCAN_LEN).enumerate() {
            scan[..4].copy_from_slice(&[SHUNT, high, low, POS].map(|v| v + k as u16));
        }
        img
    }

    /// The DMA's writes: halfwords from the tick block's front.
    fn dma_lands(t: &mut TelemetryTick, img: &[u16; ADC_DMA_BUF_LEN]) {
        const { assert!(size_of::<TelemetryTick>() >= ADC_DMA_BUF_LEN * size_of::<u16>()) };
        let p = (&raw mut *t).cast::<u16>();
        for (i, v) in img.iter().enumerate() {
            // SAFETY: inside the tick block (const assert above).
            unsafe { p.add(i).write_volatile(*v) };
        }
    }

    fn named(t: &TelemetryTick) -> [u16; 4] {
        [t.vmotor_a, t.vmotor_b, t.vmotor_a_trough, t.vmotor_b_trough]
    }

    /// A reader between the crest scan landing and the next frame (about
    /// 6 us of each 50 us tick: the high tap lands NEXT_CREST + one
    /// conversion after the scan TC, the frame ENTRY past the next) reads
    /// the previous frame's taps, each under its own terminal's name, in
    /// either drive direction.
    #[test]
    fn a_scan_landing_leaves_every_published_tap_alone() {
        for order in [Direction::Reverse, Direction::Forward] {
            let mut t = TelemetryTick::default();
            let before = [A - 1, B - 1, A - 2, B - 2];
            [t.vmotor_a, t.vmotor_b, t.vmotor_a_trough, t.vmotor_b_trough] = before;
            dma_lands(&mut t, &scans(order));
            assert_eq!(named(&t), before, "{order:?}");
        }
    }

    /// The frame names each tap by the order its scans were sampled under,
    /// the crest's low side from the injected group, and copies both
    /// shunts; the crest's pos stays where the DMA put it.
    #[test]
    fn the_frame_publishes_each_tap_under_its_terminal() {
        for order in [Direction::Forward, Direction::Reverse] {
            let mut t = TelemetryTick::default();
            dma_lands(&mut t, &scans(order));
            let injected = match order {
                Direction::Forward => B + 1,
                Direction::Reverse => A + 1,
            };
            // SAFETY: a local tick block, this context alone.
            unsafe { publish(&raw mut t, order, || injected) };
            assert_eq!(named(&t), [A + 1, B + 1, A, B], "{order:?}");
            assert_eq!(
                [t.current_raw, t.current_trough, t.pos],
                [SHUNT + 1, SHUNT, POS + 1],
                "{order:?}"
            );
        }
    }
}
