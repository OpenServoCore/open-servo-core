#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use open_motor_controller as _; // global logger + panicking-behavior + memory layout
use rtic_time::Monotonic;
use stm32f4xx_hal as hal;

use crate::hal::{
    adc::{
        config::{
            AdcConfig, Continuous, Dma, ExternalTrigger, SampleTime, Scan, Sequence, TriggerMode,
        },
        Adc, Temperature,
    },
    dma::{PeripheralToMemory, Stream0, Transfer},
    gpio::{Output, PC13},
    pac::{self, ADC1, DMA2, TIM1, TIM3},
    prelude::*,
    signature::{VtempCal110, VtempCal30},
    timer::{CenterAlignedMode, MonoTimerUs, PwmChannel},
};

type Mono = MonoTimerUs<TIM3>;

use rtic::app;

#[app(device = crate::pac, dispatchers = [EXTI4], peripherals = true)]
mod app {
    use super::*;

    type AdcTransfer =
        Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut AdcBuffer>;

    pub struct AdcValues {
        current: u16, // mV
        position: u16,
        desired_position: u16,
        temperature: u16,
    }

    type MotorPwm = (PwmChannel<TIM1, 0>, PwmChannel<TIM1, 1>);

    type AdcBuffer = [u16; ADC_BUFFER_SIZE];

    const RCC_HSE_CLOCK_HZ: u32 = 25_000_000;
    const RCC_SYSCLK_HZ: u32 = 72_000_000;

    const PWM_FREQUENCY: u32 = 10_000;
    const PWM_MAX_DUTY: u32 = RCC_SYSCLK_HZ / PWM_FREQUENCY / 2;

    const ADC_BUFFER_SIZE: usize = 4;

    const ISENSE_AIPROPI: f32 = 1500.0; // uA/A
    const ISENSE_RESISTOR_OHM: f32 = 1500.0; // Ohm

    // use this to convert voltage value to current, e.g.:
    // current = adc_value (in mV) * ISENSE_FACTOR will result in mA
    const ISENSE_FACTOR: f32 = ISENSE_AIPROPI / ISENSE_RESISTOR_OHM; // uA / V

    const KP: f32 = 0.0;
    const KI: f32 = 0.0;
    const KD: f32 = 0.0;
    const POS_MAX: u16 = 4000;
    const POS_MIN: u16 = 500;

    // Shared resources go here
    #[shared]
    struct Shared {
        adc_values: AdcValues,
    }

    // Local resources go here
    #[local]
    struct Local {
        motor_pwm: MotorPwm,
        led: PC13<Output>,
        adc_transfer: AdcTransfer,
        adc_buffer: Option<&'static mut AdcBuffer>,
    }

    #[init(local = [
        adc_buffer1: AdcBuffer = [0; ADC_BUFFER_SIZE],
        adc_buffer2: AdcBuffer = [0; ADC_BUFFER_SIZE],
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let rcc = ctx.device.RCC.constrain();

        /*
         * System Clock Configuration
         */
        let clocks = rcc
            .cfgr
            .use_hse(RCC_HSE_CLOCK_HZ.Hz())
            .require_pll48clk()
            .sysclk(RCC_SYSCLK_HZ.Hz())
            .freeze();

        /*
         * RTIC Monotonics
         */
        ctx.device.TIM3.monotonic_us(&mut ctx.core.NVIC, &clocks);

        /*
         * Obtain GPIO handlers
         */
        let gpioa = ctx.device.GPIOA.split();
        let _gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        /*
         * Motor PWM Configuration
         */
        let (mut pwm_manager, (ch1, ch2, ch3, ..)) = ctx.device.TIM1.pwm_hz(
            (PWM_FREQUENCY * 2).Hz(), // Center-aligned mode requires double the frequency
            &clocks,
        );
        let mut pwm1 = ch1.with(gpioa.pa8);
        let mut pwm2 = ch2.with(gpioa.pa9);
        let mut adc_trigger = ch3.with(gpioa.pa10);

        pwm_manager.set_cms(CenterAlignedMode::CenterAligned1);

        defmt::debug!("Max Duty: {}, {}", pwm_manager.get_max_duty(), PWM_MAX_DUTY);

        pwm1.set_duty(0);
        pwm1.enable();

        pwm2.set_duty(0);
        pwm2.enable();

        // Triggers ADC at the center of the PWM cycle with a single pulse
        adc_trigger.set_duty(1);
        adc_trigger.enable();
        let motor_pwm = (pwm1, pwm2);

        /*
         * ADC Configuration
         */
        let adc_config = AdcConfig::default()
            .scan(Scan::Enabled)
            .continuous(Continuous::Single)
            .dma(Dma::Continuous)
            .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_3);

        let mut adc = Adc::adc1(ctx.device.ADC1, true, adc_config);
        adc.enable_temperature_and_vref();

        // PA3 - Current Sensing
        adc.configure_channel(
            &gpioa.pa3.into_analog(),
            Sequence::One,
            SampleTime::Cycles_480,
        );

        // PA4 - Potentiometer on Servo
        adc.configure_channel(
            &gpioa.pa4.into_analog(),
            Sequence::Two,
            SampleTime::Cycles_480,
        );

        // PA5 - Potentiometer nob
        adc.configure_channel(
            &gpioa.pa5.into_analog(),
            Sequence::Three,
            SampleTime::Cycles_480,
        );

        // Temperature sensor
        adc.configure_channel(&Temperature, Sequence::Four, SampleTime::Cycles_480);

        // initialize DMA for ADC
        let dma = hal::dma::StreamsTuple::new(ctx.device.DMA2);
        let config = hal::dma::config::DmaConfig::default()
            .memory_increment(true)
            .double_buffer(false)
            .transfer_complete_interrupt(true);
        let mut adc_transfer =
            Transfer::init_peripheral_to_memory(dma.0, adc, ctx.local.adc_buffer1, None, config);
        let adc_buffer = Some(ctx.local.adc_buffer2);

        /*
         * LED
         */
        let led = gpioc.pc13.into_push_pull_output();

        tick::spawn().ok();

        adc_transfer.start(|_| {});

        (
            Shared {
                adc_values: AdcValues {
                    current: 0,
                    position: 0,
                    desired_position: 0,
                    temperature: 0,
                },
            },
            Local {
                motor_pwm,
                adc_transfer,
                adc_buffer,
                led,
            },
        )
    }

    #[task(
        binds = DMA2_STREAM0,
        local = [
            adc_buffer,
            adc_transfer,
            motor_pwm,
            led,
        ],
        shared = [adc_values],
        priority = 5
    )]
    fn adc_sampling(mut cx: adc_sampling::Context) {
        let adc_sampling::LocalResources {
            adc_buffer,
            adc_transfer,
            led,
            motor_pwm,
            ..
        } = cx.local;

        led.set_low();

        let (buffer, _) = adc_transfer
            .next_transfer(adc_buffer.take().unwrap())
            .unwrap();

        let adc = adc_transfer.peripheral();
        let current = adc.sample_to_millivolts(buffer[0]);
        let position = buffer[1];
        let desired_position = buffer[2];
        let temperature = buffer[3];

        adc_buffer.replace(buffer);

        cx.shared.adc_values.lock(|adc_values| {
            adc_values.current = current;
            adc_values.position = position;
            adc_values.desired_position = desired_position;
            adc_values.temperature = temperature;
        });

        // use PID to control motor
        let error = desired_position as i32 - position as i32;
        let p = (error as f32 * KP) as u32;
        let i = (error as f32 * KI) as u32;



        led.set_high();
    }

    #[task(shared = [adc_values], priority = 1)]
    async fn tick(mut ctx: tick::Context) {
        loop {
            let mut current: u16 = 0;
            let mut position: u16 = 0;
            let mut desired_position: u16 = 0;
            let mut raw_temp: u16 = 0;
            ctx.shared.adc_values.lock(|adc_values| {
                current = adc_values.current;
                position = adc_values.position;
                desired_position = adc_values.desired_position;
                raw_temp = adc_values.temperature;
            });

            let cal30 = VtempCal30::get().read() as f32;
            let cal110 = VtempCal110::get().read() as f32;
            let temperature =
                (110.0 - 30.0) * (raw_temp as f32 - cal30) / (cal110 - cal30) + 30.0;

            let current = ISENSE_FACTOR * f32::from(current);

            defmt::info!(
                "Current: {} mA, Pos: {}, DPos: {}, Temp: {}",
                current,
                position,
                desired_position,
                temperature,
            );

            Mono::delay(50.millis().into()).await;
        }
    }
}
