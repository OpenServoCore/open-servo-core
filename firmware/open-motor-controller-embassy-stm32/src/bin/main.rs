#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, InterruptHandler},
    bind_interrupts,
    gpio::OutputType,
    peripherals,
    time::{khz, mhz},
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    Config,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    ADC1 => InterruptHandler<peripherals::ADC1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.adc = AdcClockSource::Pll(AdcPllPrescaler::DIV1);
    }

    let p = embassy_stm32::init(config);

    // initialize PWM
    let mut pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)),
        None,
        Some(PwmPin::new_ch3(p.PA10, OutputType::PushPull)),
        Some(PwmPin::new_ch4(p.PA11, OutputType::PushPull)),
        khz(10),
        CountingMode::CenterAlignedDownInterrupts,
    );
    let channels = pwm.split();
    let mut pwm1 = channels.ch1;
    let mut pwm2 = channels.ch4;
    let mut adc_trigger = channels.ch3;

    pwm1.set_duty_cycle_fully_off();
    pwm1.enable();
    pwm2.set_duty_cycle_fully_off();
    pwm2.enable();

    // used to trigger ADC conversion
    adc_trigger.set_duty_cycle(1);
    adc_trigger.enable();

    let pwm = (pwm1, pwm2);

    // initialize ADC
    let mut adc = Adc::new(p.ADC1, Irqs);
    let mut ts = adc.enable_temperature();

    let result = adc.read(&mut ts);

    loop {
        info!("Hello World!");
        Timer::after_secs(1).await;
    }
}
