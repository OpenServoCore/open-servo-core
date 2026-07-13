//! V006-board-fixed constants. Anything in [`super::BoardWiring`] is
//! board-tunable; anything here is determined by the chip + this board's
//! schematic and lives in one place.

use crate::hal::{Pin, Tim1Mapping, UsartMapping, adc, opa, timer};

// === osc-native bus (USART1 on PC0/PC1; direct HDSEL single-wire, or the
// rev B 74LVC2G241 buffered wire by default (`half-duplex` = direct) — see
// providers/tx_wire; the TX_EN pin is board wiring, `BusWiring`) ===

pub const BUS_USART_MAPPING: UsartMapping = UsartMapping::Usart1Remap3;

/// Pin whose input level tracks the bus wire (rescue-break sensing, §9.1):
/// the single-wire pin itself on the direct wire; the buffer's receive
/// output (the USART RX pin) on the buffered wire.
#[cfg(feature = "half-duplex")]
pub const BUS_LINE_PIN: Pin = BUS_USART_MAPPING.tx_pin();
#[cfg(not(feature = "half-duplex"))]
pub const BUS_LINE_PIN: Pin = BUS_USART_MAPPING.rx_pin();

// === OPA current sense ===

pub const CURRENT_SENSE_OPA_INPUT: opa::InputMode = opa::InputMode::Differential {
    pos: opa::PositiveInput::PA2,
    neg: opa::NegativeInput::PA4,
};
pub const CURRENT_SENSE_OPA_OUTPUT: opa::Output = opa::Output::Internal;

// === Motor + STAT (TIM1 Remap8) ===
//
// Remap8, not Remap7: identical pins for every channel this board uses
// (CH2/PC5, CH3/PC6, CH4/PC7 — the schematic's T1Cx_7 pins), but a remap
// places FUNCTIONS, used or not, and Remap7 also puts the complementary
// outputs CH1N/CH2N/CH3N on PC0/PC1/PC2 — double-tenanting the bus pin
// (PC0 = USART1_TX under Usart1Remap3), which the RM forbids. A
// never-enabled OC function's AF signal is its reset state (0), so
// whenever HDSEL released the pin the mux fell through to CH1N's 0 and
// clamped the bus (bench: wire stuck low at idle, rose the instant
// TIM1EN dropped). Remap8 parks the unused CHxN functions on PA3 (analog
// VPOS — digital mux disconnected) and PB0/PB1 (not bonded on TSSOP20).
pub const MOTOR_TIM1_MAPPING: Tim1Mapping = Tim1Mapping::Remap8;
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
