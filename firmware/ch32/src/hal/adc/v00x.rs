use ch32_metapac::ADC;

use crate::hal::Pin;

pub use ch32_metapac::adc::vals::{Extsel, SampleTime};

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    IN0 = 0,
    IN1 = 1,
    IN2 = 2,
    IN3 = 3,
    IN4 = 4,
    IN5 = 5,
    IN6 = 6,
    IN7 = 7,
    Vref = 8,
    OpaOut = 9,
}

impl Channel {
    /// GPIO pin that the channel samples, or `None` for internal channels
    /// (`Vref`, `OpaOut`). The caller is responsible for configuring the
    /// returned pin as analog input (`PinMode::INPUT_FLOATING`) and enabling
    /// its GPIO port clock.
    pub const fn pin(self) -> Option<Pin> {
        match self {
            Channel::IN0 => Some(Pin::PA2),
            Channel::IN1 => Some(Pin::PA1),
            Channel::IN2 => Some(Pin::PC4),
            Channel::IN3 => Some(Pin::PD2),
            Channel::IN4 => Some(Pin::PD3),
            Channel::IN5 => Some(Pin::PD5),
            Channel::IN6 => Some(Pin::PD6),
            Channel::IN7 => Some(Pin::PD4),
            Channel::Vref | Channel::OpaOut => None,
        }
    }
}

/// An ADC input slot: which channel to sample, and how long to hold the
/// sample-and-hold capacitor before conversion. Sample time should be
/// chosen for the source impedance — higher-Z dividers need more cycles
/// for the S/H cap to settle. `Channel::pin().unwrap().pin_number()` is
/// the GPIO; the source impedance is the divider Thevenin equivalent
/// looking out from that pin.
#[derive(Copy, Clone)]
pub struct Input {
    pub channel: Channel,
    pub sample_time: SampleTime,
}

impl Input {
    pub const fn new(channel: Channel, sample_time: SampleTime) -> Self {
        Self {
            channel,
            sample_time,
        }
    }
}

/// `channels.len()` must be 1..=16. Channels are converted in array order.
pub fn set_sequence(channels: &[Channel]) {
    assert!(!channels.is_empty() && channels.len() <= 16);
    for (i, &ch) in channels.iter().enumerate() {
        let v = ch as u8;
        match i {
            0..=5 => ADC.rsqr3().modify(|w| w.set_sq(i, v)),
            6..=11 => ADC.rsqr2().modify(|w| w.set_sq(i - 6, v)),
            12..=15 => ADC.rsqr1().modify(|w| w.set_sq(i - 12, v)),
            _ => unreachable!(),
        }
    }
    ADC.rsqr1().modify(|w| w.set_l((channels.len() - 1) as u8));
}

pub fn set_sample_time(channel: Channel, t: SampleTime) {
    ADC.samptr2().modify(|w| w.set_smp(channel as usize, t));
}

pub fn set_external_trigger(source: Extsel) {
    ADC.ctlr2().modify(|w| {
        w.set_extsel(source);
        w.set_exttrig(true);
    });
}

pub fn set_scan_mode(enable: bool) {
    ADC.ctlr1().modify(|w| w.set_scan(enable));
}

pub fn set_dma(enable: bool) {
    ADC.ctlr2().modify(|w| w.set_dma(enable));
}

pub fn enable() {
    ADC.ctlr2().modify(|w| w.set_adon(true));
}
