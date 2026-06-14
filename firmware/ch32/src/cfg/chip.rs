//! V006-board-fixed constants. Anything in [`super::BoardWiring`] is
//! board-tunable; anything here is determined by the chip + this board's
//! schematic and lives in one place.

use osc_drivers::Level;

use crate::hal::{Pin, Tim1Mapping, Tim2Mapping, UsartMapping, adc, gpio::Pull, opa, timer};

// === DXL transport ===

pub const DXL_USART_MAPPING: UsartMapping = UsartMapping::Usart1Remap3;
pub const DXL_TIM2_MAPPING: Tim2Mapping = Tim2Mapping::Remap2;
pub const DXL_TX_EN_PIN: Pin = DXL_TIM2_MAPPING.ch2_pin();
pub const DXL_TX_EN_LEVEL: Level = Level::High;
pub const DXL_RX_PULL: Pull = Pull::None;

// === OPA current sense ===

pub const CURRENT_SENSE_OPA_INPUT: opa::InputMode = opa::InputMode::Differential {
    pos: opa::PositiveInput::PA2,
    neg: opa::NegativeInput::PA4,
};
pub const CURRENT_SENSE_OPA_OUTPUT: opa::Output = opa::Output::Internal;

// === Motor + STAT (TIM1 Remap7) ===

pub const MOTOR_TIM1_MAPPING: Tim1Mapping = Tim1Mapping::Remap7;
pub const MOTOR_IN1_CH: timer::Channel = timer::Channel::CH3;
pub const MOTOR_IN2_CH: timer::Channel = timer::Channel::CH2;
pub const MOTOR_IN1_PIN: Pin = tim1_channel_pin(MOTOR_TIM1_MAPPING, MOTOR_IN1_CH);
pub const MOTOR_IN2_PIN: Pin = tim1_channel_pin(MOTOR_TIM1_MAPPING, MOTOR_IN2_CH);
pub const MOTOR_PWM_FREQ_HZ: u32 = 20_000;
pub const STAT_LED_PIN: Pin = MOTOR_TIM1_MAPPING.ch4_pin();

const fn tim1_channel_pin(m: Tim1Mapping, c: timer::Channel) -> Pin {
    match c {
        timer::Channel::CH1 => m.ch1_pin(),
        timer::Channel::CH2 => m.ch2_pin(),
        timer::Channel::CH3 => m.ch3_pin(),
        timer::Channel::CH4 => m.ch4_pin(),
    }
}

// === ADC ===

pub const ADC_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES9;

/// ADC channels available as board-configurable sensor inputs on the V006F8P6.
/// Excludes A0 (PA0, claimed by the OPA current-sense input pair).
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum AnalogChannel {
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    A7,
}

impl AnalogChannel {
    pub const fn channel(self) -> adc::Channel {
        match self {
            Self::A1 => adc::Channel::IN1,
            Self::A2 => adc::Channel::IN2,
            Self::A3 => adc::Channel::IN3,
            Self::A4 => adc::Channel::IN4,
            Self::A5 => adc::Channel::IN5,
            Self::A6 => adc::Channel::IN6,
            Self::A7 => adc::Channel::IN7,
        }
    }

    pub const fn pin(self) -> Pin {
        match self {
            Self::A1 => Pin::PA1,
            Self::A2 => Pin::PA2,
            Self::A3 => Pin::PA3,
            Self::A4 => Pin::PA4,
            Self::A5 => Pin::PA5,
            Self::A6 => Pin::PA6,
            Self::A7 => Pin::PA7,
        }
    }
}

/// Free GPIOs on this board available as scratch outputs (DBG, motor DRV_EN).
#[derive(Copy, Clone)]
#[repr(u8)]
pub enum DigitalPin {
    PC3,
    PD0,
}

impl DigitalPin {
    pub const fn pin(self) -> Pin {
        match self {
            Self::PC3 => Pin::PC3,
            Self::PD0 => Pin::PD0,
        }
    }
}
