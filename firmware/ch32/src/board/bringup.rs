use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};
use osc_core::{BaudRate, ConfigDefaults};

use crate::hal::{
    adc, afio, clocks, delay_ms, dma,
    gpio::{self, Level, PinMode},
    opa, pfic, rcc, timer, usart,
};
use crate::statics::{
    ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN, ADC_SENSOR_COUNT, DXL_RX_BUF, DXL_RX_BUF_LEN,
    DXL_TX_BUF, DXL_TX_EN, SHARED,
};

use super::config::{BoardWiring, CurrentSenseConfig, Duplex, DxlBus, MotorConfig, Sensors};

const OPA_SETTLE_MS: u32 = 1;
const VCAL_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES9;

pub(super) struct BringupResult {
    pub(super) shunt_bias_raw: u16,
    pub(super) pwm_arr: u16,
}

pub(super) fn run(wiring: &BoardWiring, defaults: &ConfigDefaults) -> BringupResult {
    enable_clocks_and_remaps(wiring);
    crate::log::debug!("clocks + remaps configured");

    configure_pins(wiring);
    crate::log::debug!("gpio configured");

    bring_up_analog_chain(&wiring.current_sense);
    crate::log::debug!("opa settled");

    let shunt_bias_raw = wiring.current_sense.opa.bias.quiescent_raw();
    configure_adc_dma_scan(&wiring.sensors, wiring.current_sense.adc_sample_time);
    crate::log::debug!(
        "adc/dma scan armed: scan_len={} buf_len={} shunt_bias_raw={}",
        ADC_SCAN_LEN,
        ADC_DMA_BUF_LEN,
        shunt_bias_raw,
    );

    // Sole writer to CONFIG: pre-IRQ, pre-install_kernel.
    SHARED.table.seed_config_defaults(defaults);

    bring_up_dxl(&wiring.dxl, defaults.dxl_baud);
    crate::log::debug!("dxl usart + dma rx armed");

    let pwm_arr = start_center_aligned_pwm(&wiring.motor);
    crate::log::debug!(
        "pwm running ({} Hz, arr={})",
        wiring.motor.pwm_freq_hz,
        pwm_arr
    );

    BringupResult {
        shunt_bias_raw,
        pwm_arr,
    }
}

// Order must mirror the scan tail in `configure_adc_dma_scan`.
fn sensor_inputs(s: &Sensors) -> [adc::Input; ADC_SENSOR_COUNT] {
    [s.pos, s.ntc, s.vbus, s.vmotor.0, s.vmotor.1]
}

fn enable_clocks_and_remaps(w: &BoardWiring) {
    let in1 = w.motor.in1_pin();
    let in2 = w.motor.in2_pin();
    let opa_pos_pin = w.current_sense.opa.input.pos().pin();

    rcc::init_pll();
    rcc::enable_afio();
    rcc::enable_gpio(w.stat_led.port_index());
    rcc::enable_gpio(w.dbg.port_index());
    rcc::enable_gpio(in1.port_index());
    rcc::enable_gpio(in2.port_index());
    rcc::enable_gpio(w.motor.drv_en.port_index());
    rcc::enable_gpio(opa_pos_pin.port_index());
    if let Some(neg_pin) = w.current_sense.opa.input.neg_pin() {
        rcc::enable_gpio(neg_pin.port_index());
    }
    for input in sensor_inputs(&w.sensors) {
        if let Some(pin) = input.channel.pin() {
            rcc::enable_gpio(pin.port_index());
        }
    }
    rcc::enable_gpio(w.dxl.usart.tx_pin().port_index());
    rcc::enable_gpio(w.dxl.usart.rx_pin().port_index());
    if let Some(ref t) = w.dxl.tx_en {
        rcc::enable_gpio(t.pin.port_index());
    }
    rcc::enable_tim1();
    rcc::enable_adc1();
    rcc::enable_dma1();
    rcc::enable_usart1();

    afio::set_tim_remap(1, w.motor.tim1.remap_value());
    afio::set_tim_remap(2, w.tim2_remap.remap_value());
    afio::set_usart_remap(w.dxl.usart.peripheral_index(), w.dxl.usart.remap_value());
}

fn configure_pins(w: &BoardWiring) {
    let in1 = w.motor.in1_pin();
    let in2 = w.motor.in2_pin();
    let opa_pos_pin = w.current_sense.opa.input.pos().pin();

    gpio::configure(w.stat_led, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.stat_led, Level::Low);

    gpio::configure(w.dbg, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.dbg, Level::Low);

    // drv_en LOW = driver disabled (MotorCmd::Disabled boot state).
    gpio::configure(w.motor.drv_en, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.motor.drv_en, Level::Low);

    gpio::configure(in1, PinMode::AF_PUSH_PULL);
    gpio::configure(in2, PinMode::AF_PUSH_PULL);

    gpio::configure(opa_pos_pin, PinMode::ANALOG);
    if let Some(neg_pin) = w.current_sense.opa.input.neg_pin() {
        gpio::configure(neg_pin, PinMode::ANALOG);
    }
    for input in sensor_inputs(&w.sensors) {
        if let Some(pin) = input.channel.pin() {
            gpio::configure(pin, PinMode::ANALOG);
        }
    }

    configure_dxl_pins(&w.dxl);
}

fn configure_dxl_pins(d: &DxlBus) {
    gpio::configure(d.usart.tx_pin(), PinMode::AF_PUSH_PULL);
    if matches!(d.duplex, Duplex::Full) {
        gpio::configure(d.usart.rx_pin(), PinMode::input_pull(d.rx_pull));
    }
    if let Some(ref t) = d.tx_en {
        gpio::configure(t.pin, PinMode::OUTPUT_PUSH_PULL);
        gpio::set_level(t.pin, t.idle_level());
    }
}

fn bring_up_analog_chain(cs: &CurrentSenseConfig) {
    opa::init(&cs.opa);
    delay_ms(OPA_SETTLE_MS);
}

fn configure_adc_dma_scan(sensors: &Sensors, opa_out_sample_time: adc::SampleTime) {
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
    dma::configure(
        dma::Channel::CH1,
        &dma_cfg,
        paddr,
        maddr,
        ADC_DMA_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH1);
}

fn bring_up_dxl(d: &DxlBus, baud: BaudRate) {
    let regs = d.usart.regs();
    let half_duplex = matches!(d.duplex, Duplex::Half);
    usart::init(regs, clocks::PCLK_HZ, baud.as_hz(), half_duplex);

    let dma_cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        tcie: false,
    };
    dma::configure(
        dma::Channel::CH5,
        &dma_cfg,
        usart::data_addr(regs),
        DXL_RX_BUF.get() as u32,
        DXL_RX_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH5);
    usart::set_dma_rx(regs, true);

    let tx_cfg = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: false,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        tcie: false,
    };
    let tx_src = unsafe { (*DXL_TX_BUF.get()).as_ptr() } as u32;
    dma::configure(
        dma::Channel::CH4,
        &tx_cfg,
        usart::data_addr(regs),
        tx_src,
        0,
    );

    // Sole writer; IRQ-only reader unmasks below.
    unsafe { *DXL_TX_EN.get() = d.tx_en };

    usart::set_idle_irq(regs, true);
    pfic::enable(pfic::Interrupt::USART1);
}

/// Returns the configured ARR so `Effort`→duty rescale can avoid a soft-div.
fn start_center_aligned_pwm(m: &MotorConfig) -> u16 {
    let (psc, arr) = timer::pwm_dividers_from_hz(m.pwm_freq_hz);
    timer::init_center_aligned_pwm(psc, arr);
    timer::configure_pwm_channel(m.in1, m.polarity);
    timer::configure_pwm_channel(m.in2, m.polarity);
    timer::set_duty(m.in1, 0);
    timer::set_duty(m.in2, 0);
    timer::set_repetition(0);
    timer::set_trgo_update();
    timer::enable_main_output();
    timer::force_update_event();
    timer::start();
    arr
}
