//! Feature-gated ADC channel configuration.
//!
//! The ADC sequence length depends on enabled features:
//! - Always: VREFINT, position (ch1)
//! - current-sense-bus: current sense (ch2)
//! - voltage-sense-motor: voltage dividers (ch3, ch4)
//! - temp-sense-mcu: internal temp sensor (ch16)
//! - temp-sense-motor: NTC thermistor (ch5)

/// Number of ADC channels in the conversion sequence.
pub const ADC_CHANNEL_COUNT: usize = {
    let base = 2; // VREFINT + position always

    #[cfg(feature = "current-sense-bus")]
    let base = base + 1;

    #[cfg(feature = "voltage-sense-motor")]
    let base = base + 2;

    #[cfg(feature = "temp-sense-mcu")]
    let base = base + 1;

    #[cfg(feature = "temp-sense-motor")]
    let base = base + 1;

    base
};

/// ADC DMA buffer type.
pub type AdcBuffer = [u16; ADC_CHANNEL_COUNT];

/// Index constants for ADC buffer access.
pub mod idx {
    /// VREFINT (internal voltage reference) - always channel 0.
    pub const VREFINT: usize = 0;

    /// Position sensor (PA0, ADC1_IN1) - always channel 1.
    pub const POSITION: usize = 1;

    // Dynamic indices based on features.
    // These are conditionally compiled to avoid unused warnings.

    #[cfg(feature = "current-sense-bus")]
    pub const CURRENT: usize = 2;

    #[cfg(all(feature = "voltage-sense-motor", not(feature = "current-sense-bus")))]
    pub const VOLTAGE_A: usize = 2;
    #[cfg(all(feature = "voltage-sense-motor", not(feature = "current-sense-bus")))]
    pub const VOLTAGE_B: usize = 3;

    #[cfg(all(feature = "voltage-sense-motor", feature = "current-sense-bus"))]
    pub const VOLTAGE_A: usize = 3;
    #[cfg(all(feature = "voltage-sense-motor", feature = "current-sense-bus"))]
    pub const VOLTAGE_B: usize = 4;

    // Motor temp index depends on prior features
    // Base = 2, +1 for current, +2 for voltage
    #[cfg(all(
        feature = "temp-sense-motor",
        not(feature = "current-sense-bus"),
        not(feature = "voltage-sense-motor")
    ))]
    pub const MOTOR_TEMP: usize = 2;

    #[cfg(all(
        feature = "temp-sense-motor",
        feature = "current-sense-bus",
        not(feature = "voltage-sense-motor")
    ))]
    pub const MOTOR_TEMP: usize = 3;

    #[cfg(all(
        feature = "temp-sense-motor",
        not(feature = "current-sense-bus"),
        feature = "voltage-sense-motor"
    ))]
    pub const MOTOR_TEMP: usize = 4;

    #[cfg(all(
        feature = "temp-sense-motor",
        feature = "current-sense-bus",
        feature = "voltage-sense-motor"
    ))]
    pub const MOTOR_TEMP: usize = 5;

    // MCU temp index depends on all prior features
    // Base = 2, +1 for current, +2 for voltage, +1 for motor_temp
    #[cfg(all(
        feature = "temp-sense-mcu",
        not(feature = "current-sense-bus"),
        not(feature = "voltage-sense-motor"),
        not(feature = "temp-sense-motor")
    ))]
    pub const MCU_TEMP: usize = 2;

    #[cfg(all(
        feature = "temp-sense-mcu",
        feature = "current-sense-bus",
        not(feature = "voltage-sense-motor"),
        not(feature = "temp-sense-motor")
    ))]
    pub const MCU_TEMP: usize = 3;

    #[cfg(all(
        feature = "temp-sense-mcu",
        not(feature = "current-sense-bus"),
        feature = "voltage-sense-motor",
        not(feature = "temp-sense-motor")
    ))]
    pub const MCU_TEMP: usize = 4;

    #[cfg(all(
        feature = "temp-sense-mcu",
        feature = "current-sense-bus",
        feature = "voltage-sense-motor",
        not(feature = "temp-sense-motor")
    ))]
    pub const MCU_TEMP: usize = 5;

    #[cfg(all(
        feature = "temp-sense-mcu",
        not(feature = "current-sense-bus"),
        not(feature = "voltage-sense-motor"),
        feature = "temp-sense-motor"
    ))]
    pub const MCU_TEMP: usize = 3;

    #[cfg(all(
        feature = "temp-sense-mcu",
        feature = "current-sense-bus",
        not(feature = "voltage-sense-motor"),
        feature = "temp-sense-motor"
    ))]
    pub const MCU_TEMP: usize = 4;

    #[cfg(all(
        feature = "temp-sense-mcu",
        not(feature = "current-sense-bus"),
        feature = "voltage-sense-motor",
        feature = "temp-sense-motor"
    ))]
    pub const MCU_TEMP: usize = 5;

    #[cfg(all(
        feature = "temp-sense-mcu",
        feature = "current-sense-bus",
        feature = "voltage-sense-motor",
        feature = "temp-sense-motor"
    ))]
    pub const MCU_TEMP: usize = 6;
}
