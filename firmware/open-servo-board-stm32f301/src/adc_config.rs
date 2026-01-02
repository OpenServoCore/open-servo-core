//! Feature-gated ADC channel configuration.
//!
//! The ADC sequence length depends on enabled features:
//! - Always: VREFINT, position (ch1)
//! - current-sense-bus: current sense (ch2)
//! - voltage-sense-motor: voltage dividers (ch3, ch4)
//! - temp-sense-mcu: internal temp sensor (ch16)
//! - temp-sense-motor: NTC thermistor (ch5)

use open_servo_macros::AdcChannels;

#[derive(AdcChannels)]
#[adc_channels(buffer = "AdcBuffer", count = "ADC_CHANNEL_COUNT")]
pub struct Channels<'a> {
    buf: &'a [u16],

    /// VREFINT (internal voltage reference).
    #[channel(name = "VREFINT")]
    _vrefint: (),

    /// Position sensor (PA0, ADC1_IN1).
    #[channel(name = "POSITION")]
    _position: (),

    /// Current sense (PA1, ADC1_IN2).
    #[cfg(feature = "current-sense-bus")]
    #[channel(name = "CURRENT")]
    _current: (),

    /// Motor voltage A (PA2, ADC1_IN3).
    #[cfg(feature = "voltage-sense-motor")]
    #[channel(name = "VOLTAGE_A")]
    _voltage_a: (),

    /// Motor voltage B (PA3, ADC1_IN4).
    #[cfg(feature = "voltage-sense-motor")]
    #[channel(name = "VOLTAGE_B")]
    _voltage_b: (),

    /// Motor temperature NTC (PA4, ADC1_IN5).
    #[cfg(feature = "temp-sense-motor")]
    #[channel(name = "MOTOR_TEMP")]
    _motor_temp: (),

    /// MCU internal temperature sensor (ADC1_IN16).
    #[cfg(feature = "temp-sense-mcu")]
    #[channel(name = "MCU_TEMP")]
    _mcu_temp: (),
}
