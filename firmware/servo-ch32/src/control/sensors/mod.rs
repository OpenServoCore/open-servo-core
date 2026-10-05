//! Chip-side sensor acquisition. The 20 kHz control-loop ISR drains
//! `ADC_DMA_BUF` at the slot indices in `scan` and hands the raw codes to the
//! kernel via `osc_servo_core::Sensors::frame`. No unit conversion here: the
//! servo stays in the counts domain, the host owns engineering units.

pub(crate) mod scan;

use osc_servo_core::{SensorFrame, Sensors as SensorsTrait};

use crate::runtime::statics::read_sample_tick;

use scan::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_TAP_HIGH, SCAN_IDX_TAP_LOW,
    SCAN_IDX_VBUS, SCAN_IDX_VCAL, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, low_tap_injected,
    scan_slot,
};

#[derive(Default)]
pub struct Ch32Sensors;

impl Ch32Sensors {
    pub const fn new() -> Self {
        Self
    }
}

impl SensorsTrait for Ch32Sensors {
    /// Called from DMA1 TC ISR. Peak drives current; trough is diagnostic.
    /// Maps the taps by the order they were sampled under, after swapping
    /// them for a noted drive flip.
    fn frame(&mut self) -> SensorFrame {
        let order = scan::programmed();
        scan::follow_drive(order);
        let current_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_SHUNT_POST);
        let (vmotor_a_trough, vmotor_b_trough) = order.terminals(
            scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_TAP_HIGH),
            scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_TAP_LOW),
        );
        let (vmotor_a, vmotor_b) = order.terminals(
            scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_TAP_HIGH),
            low_tap_injected(),
        );

        SensorFrame {
            tick: read_sample_tick(),
            pos: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_POS),
            current: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_SHUNT_POST),
            current_trough,
            vmotor_a,
            vmotor_a_trough,
            vmotor_b,
            vmotor_b_trough,
            vcal: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VCAL),
            vbus_raw: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VBUS),
            ntc_raw: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_NTC),
        }
    }
}
