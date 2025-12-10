use crate::adc_config::{read_buffer, ADC_CHANNELS};

// Compute buffer indices at compile time
// Order: vrefint, position, [current], [motor_v+], [motor_v-], [motor_temp], [mcu_temp]
const IDX_VREFINT: usize = 0;
const IDX_POSITION: usize = 1;

#[cfg(feature = "current-sense-bus")]
const IDX_CURRENT: usize = 2;

// Motor voltage indices depend on whether current-sense is enabled
#[cfg(all(feature = "voltage-sense-motor", feature = "current-sense-bus"))]
const IDX_MOTOR_VPOS: usize = 3;
#[cfg(all(feature = "voltage-sense-motor", not(feature = "current-sense-bus")))]
const IDX_MOTOR_VPOS: usize = 2;

#[cfg(feature = "voltage-sense-motor")]
const IDX_MOTOR_VNEG: usize = IDX_MOTOR_VPOS + 1;

// Motor temp index depends on current + motor voltage
#[cfg(all(feature = "temp-sense-motor", feature = "current-sense-bus", feature = "voltage-sense-motor"))]
const IDX_MOTOR_TEMP: usize = 5;
#[cfg(all(feature = "temp-sense-motor", feature = "current-sense-bus", not(feature = "voltage-sense-motor")))]
const IDX_MOTOR_TEMP: usize = 3;
#[cfg(all(feature = "temp-sense-motor", not(feature = "current-sense-bus"), feature = "voltage-sense-motor"))]
const IDX_MOTOR_TEMP: usize = 4;
#[cfg(all(feature = "temp-sense-motor", not(feature = "current-sense-bus"), not(feature = "voltage-sense-motor")))]
const IDX_MOTOR_TEMP: usize = 2;

// MCU temp is always last (when enabled)
#[cfg(feature = "temp-sense-mcu")]
const IDX_MCU_TEMP: usize = ADC_CHANNELS - 1;

/// Type-safe ADC sample structure.
/// Fields are conditionally compiled based on enabled sensor features.
#[derive(Debug, Clone, Copy)]
pub struct AdcSample {
    /// Internal reference voltage (channel 18) - always enabled
    pub vrefint_raw: u16,

    /// Position potentiometer on PA0 (channel 1) - always enabled
    pub position_raw: u16,

    /// Current sense on PA1 (channel 2) - requires `current-sense-bus` feature
    #[cfg(feature = "current-sense-bus")]
    pub current_raw: u16,

    /// Motor V+ terminal on PA2 (channel 3) - requires `voltage-sense-motor` feature
    #[cfg(feature = "voltage-sense-motor")]
    pub motor_vpos_raw: u16,

    /// Motor V- terminal on PA3 (channel 4) - requires `voltage-sense-motor` feature
    #[cfg(feature = "voltage-sense-motor")]
    pub motor_vneg_raw: u16,

    /// Motor NTC thermistor on PA4 (channel 5) - requires `temp-sense-motor` feature
    #[cfg(feature = "temp-sense-motor")]
    pub motor_temp_raw: u16,

    /// Internal MCU temperature sensor (channel 16) - requires `temp-sense-mcu` feature
    #[cfg(feature = "temp-sense-mcu")]
    pub mcu_temp_raw: u16,
}

impl AdcSample {
    /// Create from raw DMA buffer.
    /// Buffer layout depends on enabled features:
    /// - [0]: vrefint (always)
    /// - [1]: position (always)
    /// - [2]: current (if current-sense-bus)
    /// - [next]: motor_vpos (if voltage-sense-motor)
    /// - [next]: motor_vneg (if voltage-sense-motor)
    /// - [next]: motor_temp (if temp-sense-motor)
    /// - [last]: mcu_temp (if temp-sense-mcu) - always last when enabled
    pub fn from_dma_buffer(buffer: [u16; ADC_CHANNELS]) -> Self {
        AdcSample {
            vrefint_raw: buffer[IDX_VREFINT],
            position_raw: buffer[IDX_POSITION],
            #[cfg(feature = "current-sense-bus")]
            current_raw: buffer[IDX_CURRENT],
            #[cfg(feature = "voltage-sense-motor")]
            motor_vpos_raw: buffer[IDX_MOTOR_VPOS],
            #[cfg(feature = "voltage-sense-motor")]
            motor_vneg_raw: buffer[IDX_MOTOR_VNEG],
            #[cfg(feature = "temp-sense-motor")]
            motor_temp_raw: buffer[IDX_MOTOR_TEMP],
            #[cfg(feature = "temp-sense-mcu")]
            mcu_temp_raw: buffer[IDX_MCU_TEMP],
        }
    }

    /// Read current ADC values from DMA buffer
    pub fn read() -> Self {
        Self::from_dma_buffer(read_buffer())
    }
}
