#![no_main]
#![no_std]

use open_motor_controller as _; // global logger + panicking-behavior + memory layout

use stm32f3xx_hal as hal;

use crate::hal::{
    adc::{
        config::{
            Config, ConversionMode, DmaMode, ExternalTrigger, Resolution, SampleTime, Sequence,
            TriggerMode,
        },
        Adc, CommonAdc, TemperatureSensor,
    },
    pac::TIM1,
    // dma::{PeripheralToMemory, Stream0, Transfer},
    // gpio::{Output, PC13},
    // pac::{self, ADC1, DMA2, TIM1, TIM3},
    prelude::*,
    // signature::{VtempCal110, VtempCal30},
    // timer::{CenterAlignedMode, MonoTimerUs, PwmChannel},
    pwm::tim1,
};

use pid::Pid;

use dwt_systick_monotonic::*;
use rtic::app;

#[app(device = crate::hal::pac, dispatchers = [EXTI4])]
mod app {
    use core::borrow::BorrowMut;

    use stm32f3xx_hal::{
        adc::{config::DataAlignment, Event},
        pac::ADC1,
        pwm::{PwmChannel, Tim1Ch1, Tim1Ch4, WithPins},
    };

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type AppMono = DwtSystick<RCC_SYSCLK_HZ>;

    pub struct AdcValues {
        current: u16, // mV
        position: u16,
        setpoint: u16,
        temperature: u16,
        pwm_duty: f32,
    }

    type MotorPwm = (PwmChannel<Tim1Ch1, WithPins>, PwmChannel<Tim1Ch4, WithPins>);

    const RCC_HSE_CLOCK_HZ: u32 = 16_000_000;
    const RCC_SYSCLK_HZ: u32 = 72_000_000;

    const PWM_FREQUENCY: u32 = 20_000;
    const PWM_MAX_DUTY: u16 = 256;

    const ADC_BUFFER_SIZE: usize = 4;

    const ISENSE_AIPROPI: f32 = 1500.0; // uA/A
    const ISENSE_RESISTOR_OHM: f32 = 1500.0; // Ohm

    // use this to convert voltage value to current, e.g.:
    // current = adc_value (in mV) * ISENSE_FACTOR will result in mA
    const ISENSE_FACTOR: f32 = ISENSE_AIPROPI / ISENSE_RESISTOR_OHM; // uA / V

    const PID_KP: f32 = 40.0;
    const PID_KP_MAX: f32 = PWM_MAX_DUTY as f32;
    const PID_KI: f32 = 0.0;
    const PID_KI_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
    const PID_KD: f32 = 0.0;
    const PID_KD_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
    const PID_MAX_OUTPUT: f32 = (PWM_MAX_DUTY - 1) as f32;

    // Shared resources go here
    #[shared]
    struct Shared {
        adc_values: AdcValues,
    }

    // Local resources go here
    #[local]
    struct Local {
        motor_pwm: MotorPwm,
        pid: Pid<f32>,
        adc: Adc<ADC1>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.constrain();
        let systick = ctx.core.SYST;
        let mut dcb = ctx.core.DCB;
        let dwt = ctx.core.DWT;

        /*
         * System Clock Configuration
         */
        let clocks = rcc
            .cfgr
            .use_hse(RCC_HSE_CLOCK_HZ.Hz().into())
            .sysclk(RCC_SYSCLK_HZ.Hz().into())
            .freeze(&mut flash.acr);

        /*
         * RTIC Monotonics
         */
        let mono = DwtSystick::new(&mut dcb, dwt, systick, RCC_SYSCLK_HZ);

        /*
         * Obtain GPIO handlers
         */
        let mut gpioa = ctx.device.GPIOA.split(&mut rcc.ahb);
        let mut _gpiob = ctx.device.GPIOB.split(&mut rcc.ahb);
        let mut gpioc = ctx.device.GPIOC.split(&mut rcc.ahb);

        /*
         * Motor PWM Configuration
         */
        let (ch1, _ch2, _ch3, ch4) = hal::pwm::tim1(
            ctx.device.TIM1,
            PWM_MAX_DUTY,
            (PWM_FREQUENCY).Hz(),
            &clocks,
        );

        // set TIM1 to center aligned mode 1
        // unsafe {
        //     let tim1 = &*TIM1::ptr();

        //     tim1.cr1.modify(|_, w| w.cms().center_aligned1());
        // }

        let pa8 = gpioa
            .pa8
            .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let pa11 =
            gpioa
                .pa11
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

        let mut pwm1 = ch1.output_to_pa8(pa8);
        let mut pwm2 = ch4.output_to_pa11(pa11);
        pwm1.set_duty(pwm1.get_max_duty());
        pwm2.set_duty(0);
        pwm1.enable();
        pwm2.enable();
        let motor_pwm = (pwm1, pwm2);

        // unsafe {
        //     let tim1 = &*TIM1::ptr();

        //     // set channel 3 as PWM mode 1
        //     tim1.ccmr2_output()
        //         .modify(|_, w| w.oc3m().pwm_mode1().oc3pe().set_bit());

        //     // set channel 3 duty cycle to 1
        //     tim1.ccr3().modify(|_, w| w.ccr().bits(1));

        //     // we don't want to enable output to channel 3, just let it trigger ADC
        //     // tim1.ccer.modify(|_, w| w.cc3e().set_bit());
        // }

        defmt::debug!("Max Duty: {}", PWM_MAX_DUTY);

        /*
         * ADC Configuration
         */
        let adc_config = Config::default()
            .conversion_mode(ConversionMode::Single)
            .resolution(Resolution::Twelve)
            .external_trigger(Some(ExternalTrigger::Tim1Cc3(TriggerMode::FallingEdge)))
            .align(DataAlignment::Right);

        let mut adc_common = CommonAdc::new(ctx.device.ADC1_2, &clocks, &mut rcc.ahb);
        let mut ts = TemperatureSensor::new(&mut adc_common, &mut ctx.device.ADC1);
        let mut adc = Adc::new(ctx.device.ADC1, adc_config, &clocks, &adc_common);

        let mut pa0 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        let mut pa1 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        let mut pa2 = gpioa.pa2.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

        // Potentiometer on Servo
        adc.set_pin_sequence_position(Sequence::One, &mut pa0);
        adc.set_sample_time(&pa0, SampleTime::Cycles181C5);

        // Current Sensor
        adc.set_pin_sequence_position(Sequence::Two, &mut pa1);
        adc.set_sample_time(&pa1, SampleTime::Cycles181C5);

        // Potentiometer nob
        adc.set_pin_sequence_position(Sequence::Three, &mut pa2);
        adc.set_sample_time(&pa2, SampleTime::Cycles181C5);

        // Temperature sensor
        adc.set_pin_sequence_position(Sequence::Four, &mut ts);
        adc.set_sample_time(&ts, SampleTime::Cycles181C5);
        adc.enable_interrupt(Event::EndOfSequence);

        /*
         * PID Controller
         */
        let mut pid = Pid::<f32>::new(0.0, PID_MAX_OUTPUT);
        pid.p(PID_KP, PID_KP_MAX);
        pid.i(PID_KI, PID_KI_MAX);
        pid.d(PID_KD, PID_KD_MAX);

        // tick::spawn().ok();

        (
            Shared {
                adc_values: AdcValues {
                    current: 0,
                    position: 0,
                    setpoint: 0,
                    temperature: 0,
                    pwm_duty: 0.0,
                },
            },
            Local {
                motor_pwm,
                pid,
                adc,
            },
            init::Monotonics(mono),
        )
    }

    // #[task(binds = ADC1_IRQ, shared = [adc_values], local = [adc, motor_pwm, pid])]
    // fn adc_eoc(mut cx: adc_eoc::Context) {
    // let adc_eoc::SharedResources { adc_values, .. } = cx.shared;
    // let adc_eoc::LocalResources { adc, motor_pwm, pid } = cx.local;

    // let address = adc.data_register_address();

    // let current = adc.data_register();
    // let position = adc.read(1);
    // let setpoint = adc.read(2);
    // let raw_temp = adc.read(3);

    // let cal30 = VtempCal30::get().read() as f32;
    // let cal110 = VtempCal110::get().read() as f32;
    // let temperature = (110.0 - 30.0) * (raw_temp as f32 - cal30) / (cal110 - cal30) + 30.0;

    // let current = ISENSE_FACTOR * f32::from(current);

    // let control_output = pid
    //     .setpoint(setpoint as f32)
    //     .next_control_output(position as f32);

    // adc_values.lock(|adc_values| {
    //     adc_values.current = current as u16;
    //     adc_values.position = position as u16;
    //     adc_values.setpoint = setpoint as u16;
    //     adc_values.temperature = raw_temp;
    //     adc_values.pwm_duty = control_output.output;
    // });

    // if control_output.output > 0.0 {
    //     motor_pwm.0.set_duty(control_output.output as u16);
    //     motor_pwm.1.set_duty(0);
    // } else {
    //     motor_pwm.0.set_duty(0);
    //     motor_pwm.1.set_duty(control_output.output.abs() as u16);
    // }
    // }

    // #[task(
    //     binds = DMA2_STREAM0,
    //     local = [
    //         adc_buffer,
    //         adc_transfer,
    //         motor_pwm,
    //         led,
    //         pid,
    //     ],
    //     shared = [adc_values],
    //     priority = 5
    // )]
    // fn adc_sampling(mut cx: adc_sampling::Context) {
    //     let adc_sampling::LocalResources {
    //         adc_buffer,
    //         adc_transfer,
    //         led,
    //         motor_pwm,
    //         pid,
    //         ..
    //     } = cx.local;

    //     led.set_low();

    //     let (buffer, _) = adc_transfer
    //         .next_transfer(adc_buffer.take().unwrap())
    //         .unwrap();

    //     let adc = adc_transfer.peripheral();
    //     let current = adc.sample_to_millivolts(buffer[0]);
    //     let position = buffer[1];
    //     let setpoint = buffer[2];
    //     let temperature = buffer[3];

    //     adc_buffer.replace(buffer);

    //     // use PID to control motor
    //     let control_output = pid
    //         .setpoint(setpoint as f32)
    //         .next_control_output(position as f32);

    //     // update shared ADC values
    //     cx.shared.adc_values.lock(|adc_values| {
    //         adc_values.current = current;
    //         adc_values.position = position;
    //         adc_values.setpoint = setpoint;
    //         adc_values.temperature = temperature;
    //         adc_values.pwm_duty = control_output.output;
    //     });

    //     if control_output.output > 0.0 {
    //         motor_pwm.0.set_duty(control_output.output as u16);
    //         motor_pwm.1.set_duty(0);
    //     } else {
    //         motor_pwm.0.set_duty(0);
    //         motor_pwm.1.set_duty(control_output.output.abs() as u16);
    //     }

    //     led.set_high();
    // }

    // #[task(shared = [adc_values], priority = 1)]
    // fn tick(mut ctx: tick::Context) {
    //     loop {
    //         let mut current: u16 = 0;
    //         let mut position: u16 = 0;
    //         let mut setpoint: u16 = 0;
    //         let mut raw_temp: u16 = 0;
    //         let mut pwm_duty: f32 = 0.0;
    //         ctx.shared.adc_values.lock(|adc_values| {
    //             current = adc_values.current;
    //             position = adc_values.position;
    //             setpoint = adc_values.setpoint;
    //             raw_temp = adc_values.temperature;
    //             pwm_duty = adc_values.pwm_duty;
    //         });

    //         let cal30 = VtempCal30::get().read() as f32;
    //         let cal110 = VtempCal110::get().read() as f32;
    //         let temperature = (110.0 - 30.0) * (raw_temp as f32 - cal30) / (cal110 - cal30) + 30.0;

    //         let current = ISENSE_FACTOR * f32::from(current);

    //         defmt::info!(
    //             "I: {} mA, P: {}, SP: {}, T: {}, D: {}",
    //             current,
    //             position,
    //             setpoint,
    //             temperature,
    //             pwm_duty
    //         );

    //         Mono::delay(50.millis());
    //     }
}
