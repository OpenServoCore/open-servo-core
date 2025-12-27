//! Sensor frame construction from ADC DMA buffer.

use open_servo_hw::v2::io::SensorFrame;

use crate::adc_config::idx;
use crate::calibration::adc_to_position;
use crate::resources::ADC_DMA_BUF;

/// Read sensor frame from ADC DMA buffer.
///
/// # Safety
/// Must be called from ADC DMA ISR after transfer complete.
/// The DMA buffer is valid and stable at this point.
#[inline]
pub unsafe fn read_sensor_frame() -> SensorFrame {
    let buf = &ADC_DMA_BUF;

    SensorFrame {
        pos: adc_to_position(buf[idx::POSITION]),
        driver_ok: true, // Stage-0: assume driver is always OK
        ..SensorFrame::default()
    }
}
