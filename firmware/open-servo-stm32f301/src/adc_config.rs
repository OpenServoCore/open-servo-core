//! ADC channel configuration computed at compile time based on enabled features.

use open_servo_hw_utils::adc_dma::AdcDmaBuf;

// Each optional sensor contributes 0, 1, or 2 to the channel count
#[cfg(feature = "current-sense-bus")]
const CH_CURRENT: usize = 1;
#[cfg(not(feature = "current-sense-bus"))]
const CH_CURRENT: usize = 0;

#[cfg(feature = "voltage-sense-motor")]
const CH_MOTOR_VOLTAGE: usize = 2; // V+ and V- terminals
#[cfg(not(feature = "voltage-sense-motor"))]
const CH_MOTOR_VOLTAGE: usize = 0;

#[cfg(feature = "temp-sense-motor")]
const CH_MOTOR_TEMP: usize = 1;
#[cfg(not(feature = "temp-sense-motor"))]
const CH_MOTOR_TEMP: usize = 0;

#[cfg(feature = "temp-sense-mcu")]
const CH_MCU_TEMP: usize = 1;
#[cfg(not(feature = "temp-sense-mcu"))]
const CH_MCU_TEMP: usize = 0;

/// Total ADC channels: vrefint (1) + position (1) + optional sensors
/// Order: vrefint, position, [current], [motor_v+], [motor_v-], [motor_temp], [mcu_temp]
pub const ADC_CHANNELS: usize = 2 + CH_CURRENT + CH_MOTOR_VOLTAGE + CH_MOTOR_TEMP + CH_MCU_TEMP;

/// Static DMA buffer sized for the configured number of channels
pub static ADC_BUF: AdcDmaBuf<ADC_CHANNELS> = AdcDmaBuf::new();

/// Get the DMA target pointer for ADC initialization
pub fn dma_target_ptr() -> *mut u16 {
    ADC_BUF.ptr()
}

/// Read a snapshot of the ADC buffer
pub fn read_buffer() -> [u16; ADC_CHANNELS] {
    ADC_BUF.snapshot()
}
