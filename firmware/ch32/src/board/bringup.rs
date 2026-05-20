use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};

use crate::hal::{
    Pin, Tim1Mapping, adc, afio, delay_ms, dma,
    gpio::{self, Level, PinMode},
    opa, rcc, timer,
};
use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SENSOR_COUNT};

use super::config::{BoardWiring, CurrentSenseConfig, MotorConfig, Sensors};

/// Conservative post-`init_pga` settle.
const OPA_SETTLE_MS: u32 = 1;

// Timing budget at 20 kHz center-aligned PWM (CMS=3), peak+trough triggered:
//   period = 50 µs, half = 25 µs, ADC clk = 12 MHz (HCLK/4).
//   per channel: T_conv = SMP + 12 ADC cycles.
// CYCLES9 → 7 × 21 cyc ≈ 12.25 µs per scan, ~50 % margin under the half-period.
const VCAL_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES9;

fn tim1_channel_pin(mapping: Tim1Mapping, channel: timer::Channel) -> Pin {
    match channel {
        timer::Channel::CH1 => mapping.ch1_pin(),
        timer::Channel::CH2 => mapping.ch2_pin(),
        timer::Channel::CH3 => mapping.ch3_pin(),
        timer::Channel::CH4 => mapping.ch4_pin(),
    }
}

// Order must mirror the scan tail in `configure_adc_dma_scan`: pos, ntc, vbus, vmotor.0, vmotor.1.
fn sensor_inputs(s: &Sensors) -> [adc::Input; ADC_SENSOR_COUNT] {
    [s.pos, s.ntc, s.vbus, s.vmotor.0, s.vmotor.1]
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

    gpio::configure(opa_pos_pin, PinMode::ANALOG);
    if let Some(neg_pin) = w.current_sense.opa_input.neg_pin() {
        gpio::configure(neg_pin, PinMode::ANALOG);
    }
    for input in sensor_inputs(&w.sensors) {
        if let Some(pin) = input.channel.pin() {
            gpio::configure(pin, PinMode::ANALOG);
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
    delay_ms(OPA_SETTLE_MS);
}

pub(super) fn configure_adc_dma_scan(sensors: &Sensors, opa_out_sample_time: adc::SampleTime) {
    // Scan = [IN9/OpaOut, IN7/pos, IN2/ntc, IN5/vmA, IN6/vmB, IN10/Vcal].
    // Slot 0 is the gained OPA output. Vcal trails the scan as the
    // VDD-ratiometric tap.
    adc::set_sample_time(adc::Channel::OpaOut, opa_out_sample_time);
    adc::set_sample_time(sensors.pos.channel, sensors.pos.sample_time);
    adc::set_sample_time(sensors.ntc.channel, sensors.ntc.sample_time);
    adc::set_sample_time(sensors.vmotor.0.channel, sensors.vmotor.0.sample_time);
    adc::set_sample_time(sensors.vmotor.1.channel, sensors.vmotor.1.sample_time);
    adc::set_sample_time(adc::Channel::Vcal, VCAL_SAMPLE_TIME);
    adc::set_low_power(false);

    let seq = [
        adc::Channel::OpaOut,
        sensors.pos.channel,
        sensors.ntc.channel,
        sensors.vmotor.0.channel,
        sensors.vmotor.1.channel,
        adc::Channel::Vcal,
    ];
    adc::set_sequence(&seq);
    adc::set_scan_mode(true);
    adc::set_dma(true);
    adc::set_external_trigger(Extsel::TIM1_TRGO);
    adc::enable();

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
}

/// Returns the configured ARR so the board can rescale `Effort`→duty later.
pub(super) fn start_center_aligned_pwm(m: &MotorConfig) -> u16 {
    let (psc, arr) = timer::pwm_dividers_from_hz(m.pwm_freq_hz);
    timer::init_center_aligned_pwm(psc, arr);
    timer::configure_pwm_channel(m.in1, m.polarity);
    timer::configure_pwm_channel(m.in2, m.polarity);
    // Both channels at 0 → both IN1/IN2 LOW → H-bridge coast on the boot
    // state. drv_en is still LOW so the driver is off regardless.
    timer::set_duty(m.in1, 0);
    timer::set_duty(m.in2, 0);
    timer::set_repetition(0);
    timer::set_trgo_update();
    timer::enable_main_output();
    timer::force_update_event();
    timer::start();
    arr
}
