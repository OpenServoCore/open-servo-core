//! Chip-side sensor acquisition. The tick enters on the injected group's
//! end, which holds the shunt; the ADC DMA lands both scans of each PWM
//! period in the table's tick block (`scan`), the crest's while the tick
//! writes the motor. No unit conversion here: the servo stays in the counts
//! domain, the host owns engineering units.
//!
//! At rest (torque off, no drive) the tick block shows the mapping: the
//! shunt, `current_raw` (IDATAR2), reads `current_trough` (a regular slot)
//! and `current_bias_counts`, and both taps read `vmotor_bias_counts`.

pub(crate) mod scan;

use osc_servo_core::regions::TelemetryTick;
use osc_servo_core::{Sensors as SensorsTrait, Shared};

use crate::hal::{dma, systick};
use scan::{
    Direction, SCAN_IDX_SHUNT, SCAN_IDX_TAP_HIGH, SCAN_IDX_TAP_LOW, low_tap_injected, peak_slot,
    shunt_injected, trough_slot,
};

/// The longest wait for the crest scan after the motor write, HCLK past the
/// first look. The scan lands about 480 ticks past the crest and the tick
/// enters at about +100, so a live converter keeps every wait under about
/// 400 even from a write at entry.
const SCAN_WAIT_CAP: u32 = 800;

#[derive(Default)]
pub struct Ch32Sensors;

impl Ch32Sensors {
    pub const fn new() -> Self {
        Self
    }
}

impl SensorsTrait for Ch32Sensors {
    /// At the tick's entry: the shunt from the injected group, and the
    /// trough scan's fields, out of the next trough scan's way.
    #[inline(always)]
    fn land(&mut self, shared: &Shared) {
        // SAFETY: the tick block of the static table; this vector is the
        // frame's sole CPU writer.
        unsafe { publish_entry(shared.tick_block(), scan::programmed(), shunt_injected()) };
    }

    /// After the write: waits for the crest scan, publishes its taps under
    /// the order they were sampled in, then swaps the order for a noted
    /// drive flip, ahead of the trough scan, on a tick whose window gave
    /// no reading (`scan::follow_drive`).
    #[inline(always)]
    fn complete(&mut self, shared: &Shared, read: bool) -> bool {
        if !dma::is_tc_flag(dma::Channel::CH1) && !scan_lands() {
            return false;
        }
        dma::clear_tc_flag(dma::Channel::CH1);
        let order = scan::programmed();
        // SAFETY: as in `land`.
        unsafe { publish_crest(shared.tick_block(), order, low_tap_injected()) };
        scan::follow_drive(order, read);
        true
    }
}

/// Waits for the crest scan's TC, at most `SCAN_WAIT_CAP`; false when it
/// never set. Out of line: only a tick that reaches it early waits, and that
/// tick has the room.
#[cold]
#[inline(never)]
fn scan_lands() -> bool {
    let t0 = systick::ticks();
    while !dma::is_tc_flag(dma::Channel::CH1) {
        if systick::ticks().wrapping_sub(t0) > SCAN_WAIT_CAP {
            return false;
        }
    }
    true
}

/// The trough scan's shunt and taps, each tap under its terminal's name,
/// and the crest's shunt `shunt`.
///
/// # Safety
/// `t` is valid for reads and writes, and no other context writes the
/// frame's fields meanwhile.
#[inline(always)]
unsafe fn publish_entry(t: *mut TelemetryTick, order: Direction, shunt: u16) {
    let (a_trough, b_trough) = order.terminals(
        trough_slot(t, SCAN_IDX_TAP_HIGH),
        trough_slot(t, SCAN_IDX_TAP_LOW),
    );
    // SAFETY: fn contract; the DMA leaves these fields alone (`scan`'s
    // layout asserts); volatile per-field accesses.
    unsafe {
        (&raw mut (*t).current_raw).write_volatile(shunt);
        (&raw mut (*t).current_trough).write_volatile(trough_slot(t, SCAN_IDX_SHUNT));
        (&raw mut (*t).vmotor_a_trough).write_volatile(a_trough);
        (&raw mut (*t).vmotor_b_trough).write_volatile(b_trough);
    }
}

/// The crest's taps: the high side from the scan, the low side `low` from
/// the injected group, each under its terminal's name.
///
/// # Safety
/// As [`publish_entry`].
#[inline(always)]
unsafe fn publish_crest(t: *mut TelemetryTick, order: Direction, low: u16) {
    let (a, b) = order.terminals(peak_slot(t, SCAN_IDX_TAP_HIGH), low);
    // SAFETY: as in `publish_entry`.
    unsafe {
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

    /// One period's DMA image under `order`: both scans `[high, low, shunt,
    /// pos, ..]`, the crest's samples one count above the trough's.
    fn scans(order: Direction) -> [u16; ADC_DMA_BUF_LEN] {
        let (high, low) = match order {
            Direction::Forward => (A, B),
            Direction::Reverse => (B, A),
        };
        let mut img = [0; ADC_DMA_BUF_LEN];
        for (k, scan) in img.chunks_mut(ADC_SCAN_LEN).enumerate() {
            scan[..4].copy_from_slice(&[high, low, SHUNT, POS].map(|v| v + k as u16));
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

    /// A reader between a scan landing and the tick that publishes it reads
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
    /// the crest's low side and shunt from the injected group, the trough's
    /// shunt from its scan; the crest's pos stays where the DMA put it.
    #[test]
    fn the_frame_publishes_each_tap_under_its_terminal() {
        for order in [Direction::Forward, Direction::Reverse] {
            let mut t = TelemetryTick::default();
            dma_lands(&mut t, &scans(order));
            let low = match order {
                Direction::Forward => B + 1,
                Direction::Reverse => A + 1,
            };
            // SAFETY: a local tick block, this context alone.
            unsafe {
                publish_entry(&raw mut t, order, SHUNT + 1);
                publish_crest(&raw mut t, order, low);
            }
            assert_eq!(named(&t), [A + 1, B + 1, A, B], "{order:?}");
            assert_eq!(
                [t.current_raw, t.current_trough, t.pos],
                [SHUNT + 1, SHUNT, POS + 1],
                "{order:?}"
            );
        }
    }
}
