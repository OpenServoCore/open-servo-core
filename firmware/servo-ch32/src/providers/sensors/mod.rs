//! Chip-side sensor acquisition. The ADC DMA lands both scans of each PWM
//! period in the table's tick block (`scan`); the 20 kHz control-loop ISR
//! completes the frame there, where the kernel reads it. No unit conversion
//! here: the servo stays in the counts domain, the host owns engineering
//! units.

pub(crate) mod scan;

use osc_servo_core::{Sensors as SensorsTrait, Shared};

use scan::{
    SCAN_IDX_SHUNT_POST, SCAN_IDX_TAP_HIGH, SCAN_IDX_TAP_LOW, low_tap_injected, trough_slot,
};

#[derive(Default)]
pub struct Ch32Sensors;

impl Ch32Sensors {
    pub const fn new() -> Self {
        Self
    }
}

impl SensorsTrait for Ch32Sensors {
    /// Called from the DMA1 TC ISR, ahead of the next trough scan: the
    /// trough's shunt and taps leave the DMA's way, and the crest's taps
    /// take their terminals' names in place, the high side from its slot
    /// and the low side from the injected group. Maps by the order the frame
    /// was sampled under, after swapping the order for a noted drive flip.
    fn land(&mut self, shared: &Shared) {
        let order = scan::programmed();
        scan::follow_drive(order);
        let t = shared.tick_block();
        let (a_trough, b_trough) = order.terminals(
            trough_slot(t, SCAN_IDX_TAP_HIGH),
            trough_slot(t, SCAN_IDX_TAP_LOW),
        );
        // SAFETY: the tick block of the static table; this vector is the
        // frame's sole CPU writer, and the DMA leaves these fields alone
        // until the next scans; volatile per-field accesses.
        unsafe {
            (&raw mut (*t).current_trough).write_volatile(trough_slot(t, SCAN_IDX_SHUNT_POST));
            (&raw mut (*t).vmotor_a_trough).write_volatile(a_trough);
            (&raw mut (*t).vmotor_b_trough).write_volatile(b_trough);
            let high = (&raw const (*t).vmotor_a).read_volatile();
            let (a, b) = order.terminals(high, low_tap_injected());
            (&raw mut (*t).vmotor_a).write_volatile(a);
            (&raw mut (*t).vmotor_b).write_volatile(b);
        }
    }
}
