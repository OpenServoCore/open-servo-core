use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};

use crate::hal::{
    Pin, Tim1Mapping, adc, afio, delay_cycles, dma,
    gpio::{self, Level, PinMode},
    opa, rcc, timer,
};
use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN, ADC_SENSOR_COUNT};

use super::config::{BoardWiring, CurrentSenseConfig, MotorConfig, Sensors};

const OPA_SETTLE_CYCLES: u32 = 240_000;

// Vref Z_src ~17 kΩ; CYCLES15 is shortest that clears it at fADC=12 MHz.
const VREF_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES15;

fn tim1_channel_pin(mapping: Tim1Mapping, channel: timer::Channel) -> Pin {
    match channel {
        timer::Channel::CH1 => mapping.ch1_pin(),
        timer::Channel::CH2 => mapping.ch2_pin(),
        timer::Channel::CH3 => mapping.ch3_pin(),
        timer::Channel::CH4 => mapping.ch4_pin(),
    }
}

// Order must mirror statics::ADC_SCAN_LEN: pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1.
fn sensor_inputs(s: &Sensors) -> [adc::Input; ADC_SENSOR_COUNT] {
    [
        s.pos, s.ntc, s.vbus, s.vmotor.0, s.vmotor.1, s.enc.0, s.enc.1,
    ]
}

pub(super) fn enable_clocks_and_remaps(w: &BoardWiring) {
    let in1 = tim1_channel_pin(w.motor.tim1, w.motor.in1);
    let in2 = tim1_channel_pin(w.motor.tim1, w.motor.in2);
    let opa_pos_pin = w.current_sense.opa_input.pos().pin();

    rcc::init_48mhz_hsi_pll();
    rcc::enable_afio();
    rcc::enable_gpio(w.stat_led.port_index());
    rcc::enable_gpio(in1.port_index());
    rcc::enable_gpio(in2.port_index());
    rcc::enable_gpio(w.motor.drv_en.port_index());
    rcc::enable_gpio(opa_pos_pin.port_index());
    if let Some(neg_pin) = w.current_sense.opa_input.neg_pin() {
        rcc::enable_gpio(neg_pin.port_index());
    }
    for input in sensor_inputs(&w.sensors) {
        if let Some(pin) = input.channel.pin() {
            rcc::enable_gpio(pin.port_index());
        }
    }
    rcc::enable_tim1();
    rcc::enable_adc1();
    rcc::enable_dma1();

    afio::set_tim_remap(1, w.motor.tim1.remap_value());
    afio::set_tim_remap(2, w.tim2_remap.remap_value());
}

pub(super) fn configure_pins(w: &BoardWiring) {
    let in1 = tim1_channel_pin(w.motor.tim1, w.motor.in1);
    let in2 = tim1_channel_pin(w.motor.tim1, w.motor.in2);
    let opa_pos_pin = w.current_sense.opa_input.pos().pin();

    gpio::configure(w.stat_led, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.stat_led, Level::Low);

    // drv_en LOW = driver disabled (MotorCmd::Disabled boot state).
    gpio::configure(w.motor.drv_en, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.motor.drv_en, Level::Low);

    gpio::configure(in1, PinMode::AF_PUSH_PULL);
    gpio::configure(in2, PinMode::AF_PUSH_PULL);

    gpio::configure(opa_pos_pin, PinMode::INPUT_FLOATING);
    if let Some(neg_pin) = w.current_sense.opa_input.neg_pin() {
        gpio::configure(neg_pin, PinMode::INPUT_FLOATING);
    }
    for input in sensor_inputs(&w.sensors) {
        if let Some(pin) = input.channel.pin() {
            gpio::configure(pin, PinMode::INPUT_FLOATING);
        }
    }
}

pub(super) fn bring_up_analog_chain(cs: &CurrentSenseConfig) {
    opa::init_pga(
        cs.opa_input,
        cs.opa_gain,
        cs.opa_bias,
        opa::Output::Internal,
    );
    delay_cycles(OPA_SETTLE_CYCLES);
}

pub(super) fn configure_adc_dma_scan(sensors: &Sensors, opa_out_sample_time: adc::SampleTime) {
    adc::set_sample_time(adc::Channel::OpaOut, opa_out_sample_time);
    let inputs = sensor_inputs(sensors);
    for input in &inputs {
        adc::set_sample_time(input.channel, input.sample_time);
    }
    adc::set_sample_time(adc::Channel::Vref, VREF_SAMPLE_TIME);

    // [shunt, pos, ntc, vbus, vmotor.0, vmotor.1, enc.0, enc.1, Vref] — 9 slots.
    let mut seq = [adc::Channel::OpaOut; ADC_SCAN_LEN];
    for (i, input) in inputs.iter().enumerate() {
        seq[1 + i] = input.channel;
    }
    seq[ADC_SCAN_LEN - 1] = adc::Channel::Vref;
    adc::set_sequence(&seq);
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
    // Scan × {peak, trough} = ADC_DMA_BUF_LEN xfers/period → TC once per period.
    dma::configure(
        dma::Channel::CH1,
        &dma_cfg,
        paddr,
        maddr,
        ADC_DMA_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH1);
    adc::enable();
}

pub(super) fn start_center_aligned_pwm(m: &MotorConfig) {
    let (psc, arr) = timer::pwm_dividers_from_hz(m.pwm_freq_hz);
    timer::init_center_aligned_pwm(psc, arr);
    timer::configure_pwm_channel(m.in1, m.polarity);
    timer::configure_pwm_channel(m.in2, m.polarity);
    timer::set_duty(m.in1, arr);
    timer::set_duty(m.in2, arr);
    timer::set_repetition(0);
    timer::set_trgo_update();
    timer::enable_main_output();
    timer::force_update_event();
    timer::start();
}
