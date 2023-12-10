#![no_main]
#![no_std]

// Halt on panic
use panic_rtt_target as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f4xx_hal::{
        adc::{
            config::{
                AdcConfig, Clock, Dma, ExternalTrigger, SampleTime, Scan, Sequence, TriggerMode,
            },
            Adc,
        },
        bb,
        dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
        otg_fs::{UsbBus, UsbBusType, USB},
        pac::{ADC1, DMA2, TIM1, TIM2},
        prelude::*,
        timer::{Channel, Channel1, Channel2, ChannelBuilder, PwmHz, Event, CounterHz, Flag},
    };

    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    use bbqueue::BBBuffer;
    use bincode::{Decode, Encode};

    const MONO_HZ: u32 = 84_000_000;
    const PWM_HZ: u32 = 6250;
    const USB_POLL_HZ: u32 = 375;
    const AVERAGE_WINDOW: u32 = PWM_HZ / USB_POLL_HZ;
    const PWM_MAX_DUTY: u32 = MONO_HZ / PWM_HZ - 1;
    const ADC_RESOLUTION: u8 = 12;
    const ADC_MAX_VALUE: u16 = (1 << ADC_RESOLUTION) - 1;
    const ADC_BUFFER_SIZE: usize = 2;
    const SAMPLE_SIZE: usize = 8;
    const UART_TX_BUFFER_SIZE: usize = 256;

    // This is the AIPROPI value from DRV8231A data sheet.
    // In the data sheet, the value is 1500 uA/A, or 1.5 mA/A.
    // For more details, see: https://www.ti.com/lit/ds/symlink/drv8231a.pdf
    // const ISENSE_AIPROPI: f32 = 1500.0; // uA/A
    // const ISENSE_RESISTOR_OHM: f32 = 1500.0; // Ohm
    // const ISENSE_VREF_UV: f32 = 3_300_000.0; // uV
    // const ISENSE_FACTOR: f32 =
    //     ISENSE_VREF_UV * 1000.0 / ADC_MAX_VALUE as f32 / ISENSE_RESISTOR_OHM / ISENSE_AIPROPI;

    // During motor controller drive, high side gate + low side gate is open.,
    // current from 0 to peak took 125 uS = 5Tau
    // Tau = 125 / 5 = 25 uS
    // Low side bridge and high side bridge Rdson = 300 mOhm
    // To calculate L, we use the formula Tau = L/R
    // in this case, R = ( 2 x Rdson ) because the current is going through one high side and one low side gate
    // L = Tau * R = 25 uS * 300 mOhm * 2 = 15 uH
    const MOTOR_INDUCTANCE_UH : u32 = 15;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<MONO_HZ>;

    type DMATransfer =
        Transfer<Stream0<DMA2>, 0, Adc<ADC1>, PeripheralToMemory, &'static mut [u16; 2]>;

    type MotorPwm = PwmHz<TIM1, (ChannelBuilder<TIM1, 0>, ChannelBuilder<TIM1, 1>)>;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        motor_pwm: MotorPwm,

        adc_buffer: Option<&'static mut [u16; ADC_BUFFER_SIZE]>,
        adc_transfer: DMATransfer,

        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_serial: SerialPort<'static, UsbBus<USB>>,
        usb_poll_timer: CounterHz<TIM2>,

        uart_tx_producer: bbqueue::Producer<'static, UART_TX_BUFFER_SIZE>,
        uart_tx_consumer: bbqueue::Consumer<'static, UART_TX_BUFFER_SIZE>,
    }

    #[derive(Encode, Decode, PartialEq, Debug)]
    struct Sample {
        ticks: u32,
        isense: u16,
        pot: u16,
    }

    #[init(local = [
        adc_buffer1: [u16; ADC_BUFFER_SIZE] = [0; ADC_BUFFER_SIZE],
        adc_buffer2: [u16; ADC_BUFFER_SIZE] = [0; ADC_BUFFER_SIZE],
        ep_memory: [u32; 1024] = [0; 1024],
        usb_bus: Option<UsbBusAllocator<UsbBus<USB>>> = None,
        uart_tx_buffer: BBBuffer<UART_TX_BUFFER_SIZE> = BBBuffer::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();

        let dp = cx.device;
        let cp = cx.core;

        let init::LocalResources {
            adc_buffer1,
            adc_buffer2,
            ep_memory,
            usb_bus,
            uart_tx_buffer,
        } = cx.local;

        // initialize clock
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(MONO_HZ.Hz())
            .hclk(MONO_HZ.Hz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .require_pll48clk()
            .freeze();
        rprintln!("[init] Clock initialized");

        // initialize monotonic
        let mut dcb = cp.DCB;
        let dwt = cp.DWT;
        let systick = cp.SYST;
        let mono = DwtSystick::new(&mut dcb, dwt, systick, MONO_HZ);
        rprintln!("[init] Monotonic initialized");

        // initialize GPIOA
        let gpioa = dp.GPIOA.split();

        // initialize USB CDC
        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );

        usb_bus.replace(UsbBusType::new(usb, ep_memory));

        let usb_serial = SerialPort::new(&usb_bus.as_ref().unwrap());
        let usb_dev = UsbDeviceBuilder::new(&usb_bus.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Open Servo Controller")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        rprintln!("[init] USB initialized.");

        let mut usb_poll_timer = dp.TIM2.counter_hz(&clocks);
        usb_poll_timer.start(USB_POLL_HZ.Hz()).unwrap();
        usb_poll_timer.listen(Event::Update);
        rprintln!("[init] USB Timer TIM2 initialized at {} Hz", USB_POLL_HZ);

        //initialize Motor PWM on TIM1
        let channels = (Channel1::new(gpioa.pa8), Channel2::new(gpioa.pa9));
        let mut pwm: PwmHz<TIM1, (ChannelBuilder<TIM1, 0>, ChannelBuilder<TIM1, 1>)> =
            dp.TIM1.pwm_hz(channels, PWM_HZ.Hz() * 2, &clocks);

        unsafe {
            let tim = &*TIM1::ptr();

            // invert the pwm signal for slow decay mode
            tim.ccmr1_output()
                .write(|w| w.oc1m().pwm_mode2().oc2m().pwm_mode2());

            // set center aligned mode and re-enable counter
            tim.cr1.write(|w| w.cms().center_aligned1().cen().set_bit());
        }
        pwm.set_duty(Channel::C1, 0);
        pwm.set_duty(Channel::C2, 0);
        pwm.enable(Channel::C1);
        pwm.enable(Channel::C2);
        rprintln!("[init] Motor PWM initialized");

        // Initialize TIM1 CH3 as PWM to trigger ADC in sync with motor PWM
        unsafe {
            let tim = &*TIM1::ptr();

            // setup pwm for CH3 with preloading
            tim.ccmr2_output()
                .write(|w| w.oc3pe().set_bit().oc3m().pwm_mode1());

            // set duty cycle to 1, so it triggers ADC at the middle of every PWM cycle
            tim.ccr3().write(|w| w.bits(1));

            // enable CH3 PWM
            bb::set(&tim.ccer, 8);
        }
        rprintln!("[init] TIM1 CH3 initialized");

        // Initialize DMA
        let adc_config = AdcConfig::default()
            .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_3)
            .dma(Dma::Continuous)
            .scan(Scan::Enabled)
            .clock(Clock::Pclk2_div_2);
        let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

        // PA3 - Current Sensing
        adc.configure_channel(
            &gpioa.pa3.into_analog(),
            Sequence::One,
            SampleTime::Cycles_84,
        );

        // PA5 - Potentiometer
        adc.configure_channel(
            &gpioa.pa5.into_analog(),
            Sequence::Two,
            SampleTime::Cycles_3,
        );

        // Initialize DMA for ADC
        let dma = StreamsTuple::new(dp.DMA2);
        let config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);
        let mut adc_transfer =
            Transfer::init_peripheral_to_memory(dma.0, adc, adc_buffer2, None, config);
        rprintln!("[init] ADC initialized");

        // delay one second to let usb settle
        adc_transfer.start(|_| {});

        let (uart_tx_producer, uart_tx_consumer) = uart_tx_buffer.try_split().unwrap();
        (
            Shared {},
            Local {
                adc_transfer,
                adc_buffer: Some(adc_buffer1),
                motor_pwm: pwm,
                usb_dev,
                usb_serial,
                uart_tx_producer,
                uart_tx_consumer,
                usb_poll_timer,
            },
            init::Monotonics(mono),
        )
    }

    fn convert_pwm_duty(adc_sample: u16) -> u16 {
        (adc_sample as f32 / ADC_MAX_VALUE as f32 * PWM_MAX_DUTY as f32) as u16
    }

    // fn convert_motor_current(adc_sample: u16) -> f32 {
    //     adc_sample as f32 * ISENSE_FACTOR
    // }

    #[task(
        binds = TIM2, 
        local = [
            uart_tx_consumer, 
            usb_serial, 
            usb_dev, 
            usb_poll_timer,
            read_buffer: [u8; 64] = [0; 64],
        ],
        priority = 3,
    )]
    fn usb_poll(cx: usb_poll::Context) {
        let usb_poll::LocalResources {
            uart_tx_consumer,
            usb_serial,
            usb_dev,
            usb_poll_timer,
            read_buffer,
        } = cx.local;

        usb_dev.poll(&mut [usb_serial]);

        // read and discard for now.
        let _ = usb_serial.read(read_buffer);

        match uart_tx_consumer.read() {
            Ok(g) => {
                let mut bytes_sent = 0;

                match usb_serial.write(g.buf()) {
                    Ok(written_len) => bytes_sent = written_len,
                    Err(UsbError::WouldBlock) => {}
                    Err(e) => panic!("Unable to write to UART: {:?}", e),
                }

                g.release(bytes_sent);
                // rprintln!("bytes sent: {}", bytes_sent);
            }
            Err(bbqueue::Error::InsufficientSize) => {}
            Err(e) => {
                panic!("Error reading from UART TX Buffer: {:?}", e);
            }
        }

        usb_poll_timer.clear_flags(Flag::Update);
    }

    #[task(
        binds = DMA2_STREAM0,
        local = [
            adc_buffer, 
            adc_transfer, 
            motor_pwm, 
            uart_tx_producer,
        ], 
        priority = 5
    )]
    fn adc_sampling(cx: adc_sampling::Context) {
        let ticks = stm32f4xx_hal::pac::DWT::cycle_count();

        let adc_sampling::LocalResources {
            adc_buffer,
            adc_transfer,
            motor_pwm,
            uart_tx_producer,
        } = cx.local;

        let (buffer, _) = adc_transfer
            .next_transfer(adc_buffer.take().unwrap())
            .unwrap();

        let sample = Sample {
            ticks,
            isense: buffer[0],
            pot: buffer[1],
        };

        let duty = convert_pwm_duty(sample.pot);
        motor_pwm.set_duty(Channel::C1, duty);

        match uart_tx_producer.grant_exact(SAMPLE_SIZE) {
            Ok(mut g) => {
                let buf = g.buf();
                buf[0..4].copy_from_slice(&sample.ticks.to_ne_bytes());
                buf[4..6].copy_from_slice(&sample.isense.to_ne_bytes());
                buf[6..8].copy_from_slice(&sample.pot.to_ne_bytes());
                g.commit(SAMPLE_SIZE);
            }
            _ => {}
        }

        *adc_buffer = Some(buffer);

    }
}
