use ch32_metapac::TIM1;
use ch32_metapac::timer::vals::{Mms, Ocm};

pub use ch32_metapac::timer::vals::Cms;

#[derive(Copy, Clone)]
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// ARPE on, so duty updates take effect at the next update event.
pub fn init_pwm(cms: Cms, prescaler: u16, period: u16) {
    TIM1.psc().write_value(prescaler);
    TIM1.atrlr().write_value(period);
    TIM1.ctlr1().modify(|w| {
        w.set_cms(cms);
        w.set_arpe(true);
    });
}

/// `ch` is 1..=4. PWM mode 1 + output preload, polarity per arg.
pub fn configure_pwm_channel(ch: u8, polarity: Polarity) {
    let n = (ch - 1) as usize;
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
pub fn set_duty(ch: u8, value: u16) {
    TIM1.chcvr((ch - 1) as usize).write_value(value);
}

/// RCR=0 with CENTERALIGNED3 → UEV fires at both peak and trough.
pub fn set_repetition(rcr: u8) {
    TIM1.rptcr().write(|w| w.set_rptcr(rcr));
}

/// TRGO mirrors the update event (peak/trough in CENTERALIGNED3). Used as the
/// ADC external trigger so a scan fires on every UEV.
pub fn set_trgo_update() {
    TIM1.ctlr2().modify(|w| w.set_mms(Mms::UPDATE));
}

/// Pulses UG=1 to latch preloaded CCR/ARR/PSC into shadow registers. Run once
/// after configuring channels and before enabling outputs so the first cycle
/// starts at the configured duty. Also fires TRGO once — call before the ADC
/// is armed.
pub fn force_update_event() {
    TIM1.swevgr().write(|w| w.set_ug(true));
}

/// Required for advanced-timer outputs to drive their pins.
pub fn enable_main_output() {
    TIM1.bdtr().modify(|w| w.set_moe(true));
}

pub fn start() {
    TIM1.ctlr1().modify(|w| w.set_cen(true));
}
