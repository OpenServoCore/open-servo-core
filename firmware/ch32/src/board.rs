use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};
use osc_core::{Board, Capabilities, MotorCmd};

use crate::hal::{
    Pin, Tim1Mapping, Tim2Mapping, adc, afio, delay_cycles, dma,
    gpio::{self, Level, PinMode},
    opa, pfic, rcc, timer,
};
use crate::statics::ADC_DMA_BUF;

const OPA_SETTLE_CYCLES: u32 = 240_000;

pub struct MotorConfig {
    pub tim1: Tim1Mapping,
    pub in1: timer::Channel,
    pub in2: timer::Channel,
    pub drv_en: Pin,
    pub pwm_freq_hz: u32,
    pub polarity: timer::Polarity,
}

pub struct CurrentSenseConfig {
    pub opa_pos: opa::PositiveInput,
    pub opa_gain: opa::Gain,
    pub opa_bias: opa::Bias,
    pub adc_sample_time: adc::SampleTime,
}

pub struct BoardConfig {
    pub stat_led: Pin,
    pub motor: MotorConfig,
    pub current_sense: CurrentSenseConfig,
    pub tim2: Tim2Mapping,
}

pub struct Ch32Board {
    stat_led: Pin,
}

impl Ch32Board {
    pub fn new(cfg: BoardConfig) -> Self {
        let motor = cfg.motor;
        let sense = cfg.current_sense;
        let in1_pin = tim1_channel_pin(motor.tim1, motor.in1);
        let in2_pin = tim1_channel_pin(motor.tim1, motor.in2);
        let opa_pin = sense.opa_pos.pin();

        rcc::init_48mhz_hsi_pll();
        rcc::enable_afio();
        rcc::enable_gpio(cfg.stat_led.port_index());
        rcc::enable_gpio(in1_pin.port_index());
        rcc::enable_gpio(in2_pin.port_index());
        rcc::enable_gpio(motor.drv_en.port_index());
        rcc::enable_gpio(opa_pin.port_index());
        rcc::enable_tim1();
        rcc::enable_adc1();
        rcc::enable_dma1();

        afio::set_tim_remap(1, motor.tim1.remap_value());
        afio::set_tim_remap(2, cfg.tim2.remap_value());

        gpio::configure(cfg.stat_led, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(cfg.stat_led, Level::Low);

        // drv_en LOW = driver disabled (MotorCmd::Disabled boot state).
        gpio::configure(motor.drv_en, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(motor.drv_en, Level::Low);

        gpio::configure(in1_pin, PinMode::AF_PUSH_PULL);
        gpio::configure(in2_pin, PinMode::AF_PUSH_PULL);

        gpio::configure(opa_pin, PinMode::INPUT_FLOATING);

        opa::init_pga_single_ended(
            sense.opa_pos,
            sense.opa_gain,
            sense.opa_bias,
            opa::Output::Internal,
        );
        delay_cycles(OPA_SETTLE_CYCLES);

        adc::set_sample_time(adc::Channel::OpaOut, sense.adc_sample_time);
        adc::set_sequence(&[adc::Channel::OpaOut]);
        adc::set_scan_mode(true);
        adc::set_dma(true);
        adc::set_external_trigger(Extsel::TIM1_TRGO);

        let dma_cfg = dma::Config {
            dir: Dir::FROMPERIPHERAL,
            circ: true,
            pinc: false,
            minc: true,
            size: dma::Size::BITS16,
            tcie: true,
        };
        let paddr = ADC.rdatar().as_ptr() as u32;
        let maddr = ADC_DMA_BUF.get() as u32;
        // 1-channel scan × {peak, trough} = 2 xfers/period → TC once per period.
        dma::configure(dma::Channel::CH1, &dma_cfg, paddr, maddr, 2);
        dma::enable(dma::Channel::CH1);
        adc::enable();

        pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);

        let (psc, arr) = timer::pwm_dividers_from_hz(motor.pwm_freq_hz);
        timer::init_center_aligned_pwm(psc, arr);
        timer::configure_pwm_channel(motor.in1, motor.polarity);
        timer::configure_pwm_channel(motor.in2, motor.polarity);
        timer::set_duty(motor.in1, arr);
        timer::set_duty(motor.in2, arr);
        timer::set_repetition(0);
        timer::set_trgo_update();
        timer::enable_main_output();
        timer::force_update_event();
        timer::start();

        Self {
            stat_led: cfg.stat_led,
        }
    }

    #[inline]
    pub fn set_stat_led(&self, on: bool) {
        gpio::set_level(self.stat_led, if on { Level::High } else { Level::Low });
    }
}

impl Board for Ch32Board {
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }

    fn write_motor(&mut self, _cmd: MotorCmd) {}

    fn pulse_tick_indicator(&mut self) {
        gpio::toggle(self.stat_led);
    }
}

fn tim1_channel_pin(mapping: Tim1Mapping, channel: timer::Channel) -> Pin {
    match channel {
        timer::Channel::CH1 => mapping.ch1_pin(),
        timer::Channel::CH2 => mapping.ch2_pin(),
        timer::Channel::CH3 => mapping.ch3_pin(),
        timer::Channel::CH4 => mapping.ch4_pin(),
    }
}
