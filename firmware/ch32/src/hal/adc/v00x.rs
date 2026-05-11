use ch32_metapac::ADC;

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
