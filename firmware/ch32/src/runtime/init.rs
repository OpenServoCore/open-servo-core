use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};
use osc_core::ConfigDefaults;
use osc_drivers::Level;

use crate::control::sensors::scan::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN, ADC_SENSOR_COUNT};
use crate::hal::{
    adc, afio, delay_ms, dma,
    gpio::{self, PinMode},
    opa, rcc, systick, timer, usart,
};
use crate::providers::crc::Crc;
use crate::providers::ring::RxRing;
use crate::runtime::Drivers;
use crate::runtime::statics::SHARED;

use crate::cfg::{AdcPins, AnalogChannel, BoardWiring, CurrentSenseConfig, Precomputed, chip};

const OPA_SETTLE_MS: u32 = 1;
const VCAL_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES9;

pub struct BringupResult {
    pub shunt_bias_raw: u16,
}

pub fn bringup(
    wiring: &BoardWiring,
    defaults: &ConfigDefaults,
    pre: &Precomputed,
) -> BringupResult {
    enable_clocks_and_remaps(wiring);
    crate::log::debug!("clocks + remaps configured");

    configure_pins(wiring);
    crate::log::debug!("gpio configured");

    // Sole writer to CONFIG: pre-IRQ, pre-`Drivers::install`. Seeds the
    // dispatcher's view of id/baud/deadline from the board defaults.
    SHARED.table.seed_config_defaults(defaults);

    bring_up_analog_chain(&wiring.current_sense);
    crate::log::debug!("opa settled");

    // SysTick drives both `Monotonic` (LED blinker) and the transport
    // deadline compare. Initialize *after* `bring_up_analog_chain` because
    // `delay_ms` reinitializes SYSTICK on every call; doing it here puts
    // SysTick in a known state (CMP=u32::MAX, CNT=0, STE=on, STIE=off)
    // independent of any further `delay_ms` use.
    systick::init();

    let shunt_bias_raw = wiring.current_sense.bias.quiescent_raw();
    configure_adc_dma_scan(&wiring.sensors);
    crate::log::debug!(
        "adc/dma scan armed: scan_len={} buf_len={} shunt_bias_raw={}",
        ADC_SCAN_LEN,
        ADC_DMA_BUF_LEN,
        shunt_bias_raw,
    );

    bring_up_bus(pre.usart_brr);
    crate::log::debug!("bus usart + rx ring + crc engine armed");

    // Drivers::install runs after the bus peripherals are live: `ServoBus
    // ::new` applies `defaults.baud` to the already-configured BRR.
    // SAFETY: bringup-only, pre-IRQ; sole writer.
    unsafe { Drivers::install(wiring, defaults) };
    crate::log::debug!("drivers installed");

    start_center_aligned_pwm(pre.pwm_psc, pre.pwm_arr);
    crate::log::debug!(
        "pwm running ({} Hz, psc={}, arr={})",
        chip::MOTOR_PWM_FREQ_HZ,
        pre.pwm_psc,
        pre.pwm_arr,
    );

    #[cfg(feature = "defmt")]
    super::diag::dump_init_regs();

    BringupResult { shunt_bias_raw }
}

// Order must mirror the scan tail in `configure_adc_dma_scan`.
fn sensor_channels(s: &AdcPins) -> [AnalogChannel; ADC_SENSOR_COUNT] {
    [s.pos, s.ntc, s.vbus, s.vmotor.0, s.vmotor.1]
}

fn enable_clocks_and_remaps(w: &BoardWiring) {
    let opa_pos_pin = chip::CURRENT_SENSE_OPA_INPUT.pos().pin();

    rcc::init_pll();
    rcc::enable_afio();
    rcc::enable_gpio(chip::STAT_LED_PIN.port_index());
    rcc::enable_gpio(w.dbg.pin().port_index());
    rcc::enable_gpio(chip::MOTOR_IN1_PIN.port_index());
    rcc::enable_gpio(chip::MOTOR_IN2_PIN.port_index());
    rcc::enable_gpio(w.drv_en.pin.pin().port_index());
    rcc::enable_gpio(opa_pos_pin.port_index());
    if let Some(neg_pin) = chip::CURRENT_SENSE_OPA_INPUT.neg_pin() {
        rcc::enable_gpio(neg_pin.port_index());
    }
    for ch in sensor_channels(&w.sensors) {
        rcc::enable_gpio(ch.pin().port_index());
    }
    rcc::enable_gpio(chip::BUS_USART_MAPPING.tx_pin().port_index());
    rcc::enable_gpio(chip::BUS_BUF_DISABLE_PIN.port_index());
    rcc::enable_tim1();
    rcc::enable_adc1();
    rcc::enable_dma1();
    rcc::enable_usart1();

    afio::set_tim_remap(1, chip::MOTOR_TIM1_MAPPING.remap_value());
    afio::set_usart_remap(
        chip::BUS_USART_MAPPING.peripheral_index(),
        chip::BUS_USART_MAPPING.remap_value(),
    );
}

fn configure_pins(w: &BoardWiring) {
    let drv_en_pin = w.drv_en.pin.pin();
    // Boot to the inactive level (driver disabled) before flipping to output.
    gpio::configure(drv_en_pin, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(drv_en_pin, w.drv_en.inactive());

    gpio::configure(chip::MOTOR_IN1_PIN, PinMode::AF_PUSH_PULL);
    gpio::configure(chip::MOTOR_IN2_PIN, PinMode::AF_PUSH_PULL);

    let opa_pos_pin = chip::CURRENT_SENSE_OPA_INPUT.pos().pin();
    gpio::configure(opa_pos_pin, PinMode::ANALOG);
    if let Some(neg_pin) = chip::CURRENT_SENSE_OPA_INPUT.neg_pin() {
        gpio::configure(neg_pin, PinMode::ANALOG);
    }
    for ch in sensor_channels(&w.sensors) {
        gpio::configure(ch.pin(), PinMode::ANALOG);
    }

    configure_bus_pins();
}

fn configure_bus_pins() {
    // PC0 idle: AF open-drain — released, the external bus pull-up holds
    // mark (spike break_framing `pc0_drive`; a bare wire with no pull-up
    // floats low and trips rescue). TxWire flips to AF push-pull for the
    // TX window so data edges never ride the pull-up (§2, F8).
    gpio::configure(chip::BUS_USART_MAPPING.tx_pin(), PinMode::AF_OPEN_DRAIN);
    // The 74LVC2G241 direction buffer is bypassed on this board: park its
    // disable pin LOW once and never touch it again (§2, F7).
    gpio::configure(chip::BUS_BUF_DISABLE_PIN, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(chip::BUS_BUF_DISABLE_PIN, Level::Low);
    // The dedicated RX pin (PC1) is left unconfigured — HDSEL ties RX to the
    // TX pin internally and ignores it.
}

fn bring_up_analog_chain(cs: &CurrentSenseConfig) {
    opa::init(&opa::Config {
        input: chip::CURRENT_SENSE_OPA_INPUT,
        gain: cs.gain,
        bias: cs.bias,
        output: chip::CURRENT_SENSE_OPA_OUTPUT,
    });
    delay_ms(OPA_SETTLE_MS);
}

fn configure_adc_dma_scan(sensors: &AdcPins) {
    adc::set_sample_time(adc::Channel::OpaOut, chip::ADC_SAMPLE_TIME);
    adc::set_sample_time(sensors.pos.channel(), chip::ADC_SAMPLE_TIME);
    adc::set_sample_time(sensors.ntc.channel(), chip::ADC_SAMPLE_TIME);
    adc::set_sample_time(sensors.vmotor.0.channel(), chip::ADC_SAMPLE_TIME);
    adc::set_sample_time(sensors.vmotor.1.channel(), chip::ADC_SAMPLE_TIME);
    adc::set_sample_time(adc::Channel::Vcal, VCAL_SAMPLE_TIME);
    adc::set_low_power(false);

    let seq = [
        adc::Channel::OpaOut,
        sensors.pos.channel(),
        sensors.ntc.channel(),
        sensors.vmotor.0.channel(),
        sensors.vmotor.1.channel(),
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
        htie: false,
        tcie: true,
        // VERYHIGH + lowest channel number = top of the DMA ladder: the ADC
        // drain interleaves through the CH6 staging copy (§5 odd reads)
        // instead of starving behind it.
        pl: dma::Pl::VERYHIGH,
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

/// osc-native transport bring-up: USART1 in single-wire HDSEL mode, the
/// circular RX ring on DMA1_CH5 (armed once), and the SPI-CRC engine. TX arms
/// (DMA1_CH4) are configured per-arm by `TxWire`, so no channel init here.
fn bring_up_bus(brr: u32) {
    let regs = chip::BUS_USART_MAPPING.regs();

    // Arm the RX ring before UE/DMAR come up so the channel is live the
    // moment the first byte's DMA request fires (spike ordering).
    let rx_cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        htie: false,
        tcie: false,
        // VERYHIGH, number 5 < 6: the RX ring wins ties against the CH6
        // staging copy, so inbound bytes never overrun during a copy.
        pl: dma::Pl::VERYHIGH,
    };
    dma::configure(
        dma::Channel::CH5,
        &rx_cfg,
        usart::data_addr(regs),
        RxRing::base_addr(),
        RxRing::LEN as u16,
    );
    dma::enable(dma::Channel::CH5);

    // TE/RE, then HDSEL + RX-DMA + error IRQ, BRR, UE last. No IDLE IRQ.
    usart::init_bus(regs, brr);

    // One-shot SPI-CRC engine setup (clock-gate + config; held live).
    Crc::init();
}

fn start_center_aligned_pwm(psc: u16, arr: u16) {
    timer::init_center_aligned_pwm(psc, arr);
    // Start both channels active-high; the control loop owns CCP per-cycle
    // from the first tick for direction / decay-mode control.
    timer::configure_pwm_channel(chip::MOTOR_IN1_CH, timer::Polarity::ActiveHigh);
    timer::configure_pwm_channel(chip::MOTOR_IN2_CH, timer::Polarity::ActiveHigh);
    timer::set_duty(chip::MOTOR_IN1_CH, 0);
    timer::set_duty(chip::MOTOR_IN2_CH, 0);
    timer::set_repetition(0);
    timer::set_trgo_update();
    timer::enable_main_output();
    timer::force_update_event();
    timer::start();
}
