//! Chip-side sensor acquisition. The 20 kHz control-loop ISR drains
//! `ADC_DMA_BUF` at the slot indices in `scan` and hands the raw codes to the
//! kernel via `osc_servo_core::Sensors::frame`. No unit conversion here: the
//! servo stays in the counts domain, the host owns engineering units.

pub(crate) mod scan;

use osc_servo_core::{SensorFrame, Sensors as SensorsTrait};

use crate::runtime::statics::read_sample_tick;

use scan::{
    SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A, SCAN_IDX_VMOTOR_B,
    SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, scan_slot,
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
    fn frame(&mut self) -> SensorFrame {
        let current_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_SHUNT_POST);
        let vmotor_a_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_A);
        let vmotor_b_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_B);

        SensorFrame {
            tick: read_sample_tick(),
            pos: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_POS),
            current: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_SHUNT_POST),
            current_trough,
            vmotor_a: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_A),
            vmotor_a_trough,
            vmotor_b: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_B),
            vmotor_b_trough,
            vcal: scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VCAL),
        }
    }
}
