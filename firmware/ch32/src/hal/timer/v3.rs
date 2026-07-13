use ch32_metapac::TIM1;
use ch32_metapac::timer::vals::{Cms, Mms, Ocm};

use crate::hal::clocks::TIM_CLK_HZ;

#[derive(Copy, Clone)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// Center-aligned period = `2 * arr * (psc + 1)` timer ticks; picks smallest psc that fits ARR in u16.
pub const fn pwm_dividers_from_hz(freq_hz: u32) -> (u16, u16) {
    let denom = 2 * freq_hz;
    let mut psc: u32 = 0;
    loop {
        let arr = TIM_CLK_HZ / (denom * (psc + 1));
        if arr > 0 && arr <= u16::MAX as u32 {
            return (psc as u16, arr as u16);
        }
        psc += 1;
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
}

/// Sets ARPE: CCR/ARR writes take effect at next UEV, not immediately.
pub fn init_center_aligned_pwm(prescaler: u16, period: u16) {
    TIM1.psc().write_value(prescaler);
    TIM1.atrlr().write_value(period);
    TIM1.ctlr1().modify(|w| {
        w.set_cms(Cms::CENTERALIGNED3);
        w.set_arpe(true);
    });
}

pub fn configure_pwm_channel(ch: Channel, polarity: Polarity) {
    let n = (ch as u8 - 1) as usize;
    TIM1.chctlr_output(n / 2).modify(|w| {
        w.set_ocm(n % 2, Ocm::PWMMODE1);
        w.set_ocpe(n % 2, true);
    });
    TIM1.ccer().modify(|w| {
        w.set_cce(n, true);
        w.set_ccp(n, matches!(polarity, Polarity::ActiveLow));
    });
}

#[inline]
pub fn set_duty(ch: Channel, value: u16) {
    TIM1.chcvr((ch as u8 - 1) as usize).write_value(value);
}

/// RCR=0 with center-aligned -> UEV at both peak and trough.
pub fn set_repetition(rcr: u8) {
    TIM1.rptcr().write(|w| w.set_rptcr(rcr));
}

pub fn set_trgo_update() {
    TIM1.ctlr2().modify(|w| w.set_mms(Mms::UPDATE));
}

/// Fires TRGO as a side effect -- arm ADC after, not before.
pub fn force_update_event() {
    TIM1.swevgr().write(|w| w.set_ug(true));
}

pub fn enable_main_output() {
    TIM1.bdtr().modify(|w| w.set_moe(true));
}

pub fn start() {
    TIM1.ctlr1().modify(|w| w.set_cen(true));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn realised_hz(psc: u16, arr: u16) -> u32 {
        TIM_CLK_HZ / (2 * (psc as u32 + 1) * arr as u32)
    }

    #[test]
    fn pwm_dividers_pin_production_freq() {
        let (psc, arr) = pwm_dividers_from_hz(20_000);
        assert_eq!((psc, arr), (0, 1200));
        assert_eq!(realised_hz(psc, arr), 20_000);
    }

    #[test]
    fn pwm_dividers_fits_low_freq() {
        // 50 Hz needs a prescaler; product (psc+1)*arr fits u32.
        let (psc, arr) = pwm_dividers_from_hz(50);
        assert!(arr > 0);
        let realised = realised_hz(psc, arr);
        assert!((45..=55).contains(&realised), "got {realised}");
    }

    #[test]
    fn pwm_dividers_high_freq() {
        let (psc, arr) = pwm_dividers_from_hz(100_000);
        assert_eq!(psc, 0);
        assert_eq!(realised_hz(psc, arr), 100_000);
    }
}
