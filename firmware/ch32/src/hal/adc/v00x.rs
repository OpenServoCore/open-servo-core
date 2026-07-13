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
    // IN8 is unused on v006 (no external bonding on this package).
    OpaOut = 9,
    /// IN10 = "Vcal" per datasheet block diagram. SMP lives in SAMPTR1.
    Vcal = 10,
}

impl Channel {
    /// `None` for internal channels (OpaOut, Vcal). Caller configures pin as analog input.
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
            Channel::OpaOut | Channel::Vcal => None,
        }
    }
}

/// Sample time should match source Z (divider Thevenin equivalent) for S/H settling.
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

/// `channels.len()` must be 1..=16.
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
    let ch = channel as usize;
    if ch < 10 {
        ADC.samptr2().modify(|w| w.set_smp(ch, t));
    } else {
        // SAMPTR1 covers SMP10..SMP15 (n = 0..6).
        ADC.samptr1().modify(|w| w.set_smp(ch - 10, t));
    }
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

/// CTLR3.ADC_LP. Reset default is set; gates the internal OPA->IN9 path,
/// so it must be cleared before the OpaOut channel reads non-zero.
pub fn set_low_power(enable: bool) {
    ADC.ctlr3().modify(|w| w.set_adc_lp(enable));
}

pub fn enable() {
    ADC.ctlr2().modify(|w| w.set_adon(true));
    crate::hal::delay_cycles(1_000);
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXTERNAL: &[Channel] = &[
        Channel::IN0,
        Channel::IN1,
        Channel::IN2,
        Channel::IN3,
        Channel::IN4,
        Channel::IN5,
        Channel::IN6,
        Channel::IN7,
    ];

    #[test]
    fn external_channels_have_pins() {
        for ch in EXTERNAL {
            assert!(ch.pin().is_some(), "{:?} has no pin", *ch as u8);
        }
    }

    #[test]
    fn internal_channels_have_no_pin() {
        assert!(Channel::OpaOut.pin().is_none());
        assert!(Channel::Vcal.pin().is_none());
    }

    #[test]
    fn external_channels_map_to_distinct_pins() {
        for (i, a) in EXTERNAL.iter().enumerate() {
            for b in &EXTERNAL[i + 1..] {
                assert_ne!(
                    a.pin().unwrap(),
                    b.pin().unwrap(),
                    "channels {:?} and {:?} share a pin",
                    *a as u8,
                    *b as u8
                );
            }
        }
    }
}
