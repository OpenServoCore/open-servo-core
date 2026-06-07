use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir};
use osc_core::{ConfigDefaults, RegionStorage};

use crate::dxl::statics::{
    DXL_DBG_PIN, DXL_RX_BUF, DXL_RX_BUF_LEN, DXL_RX_PIN, DXL_TX_BUF, DXL_TX_EN, store_baud_derived,
};
use crate::hal::{
    adc, afio, delay_ms, dma, exti,
    gpio::{self, Level, PinMode},
    opa, rcc, timer, usart,
};
use crate::statics::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN, ADC_SENSOR_COUNT, SHARED};

use super::config::{
    AdcPins, BoardWiring, CurrentSenseConfig, Duplex, DxlBus, MotorConfig, Precomputed,
};

const OPA_SETTLE_MS: u32 = 1;
const VCAL_SAMPLE_TIME: adc::SampleTime = adc::SampleTime::CYCLES9;

pub(super) struct BringupResult {
    pub(super) shunt_bias_raw: u16,
}

pub(super) fn run(
    wiring: &BoardWiring,
    defaults: &ConfigDefaults,
    pre: &Precomputed,
) -> BringupResult {
    enable_clocks_and_remaps(wiring);
    rcc::apply_clock_trim_delta(defaults.clock_trim);
    crate::log::debug!(
        "clocks + remaps configured (clock_trim={})",
        defaults.clock_trim
    );

    configure_pins(wiring);
    seed_dbg_pin(wiring.dbg);
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
    SHARED.table.config.with_mut(|c| {
        c.comms.clock_step_ppm = rcc::CLOCK_TRIM_PPM_PER_STEP as u16;
    });

    // Override `ConfigDefaults::dxl_id` with a UID-derived ID so a freshly
    // flashed chip plugged into an existing bus doesn't collide on the
    // default. EEPROM persistence (when landed) layers on top — it loads
    // *after* this and wins if a stored ID is present.
    let id = derive_dxl_id_from_uid();
    crate::log::info!("seed comms.id={} from UID", id);
    SHARED.table.config.with_mut(|c| {
        c.comms.id = id;
    });

    bring_up_dxl(&wiring.dxl, pre.usart_brr);
    crate::log::debug!("dxl usart + dma rx armed");

    start_center_aligned_pwm(&wiring.motor, pre.pwm_psc, pre.pwm_arr);
    crate::log::debug!(
        "pwm running ({} Hz, psc={}, arr={})",
        wiring.motor.pwm_freq_hz,
        pre.pwm_psc,
        pre.pwm_arr,
    );

    BringupResult { shunt_bias_raw }
}

/// XOR-fold the 12-byte chip UID (`ESIG_UNIID1..3` at 0x1FFFF7E8/EC/F0, per
/// RM §19.2) into a single byte, then map to a valid DXL ID in [1, 252] —
/// avoids 0xFD (reserved), 0xFE (broadcast), 0xFF (invalid). Lets identical
/// firmware images on a shared bus seed unique IDs without persistence;
/// EEPROM-stored IDs (when landed) override this in a later boot phase.
fn derive_dxl_id_from_uid() -> u8 {
    const ESIG_UNIID_BASE: *const u32 = 0x1FFFF7E8 as *const u32;
    // SAFETY: ESIG block is ROM-mapped, always present, 4-byte aligned.
    let words = unsafe {
        [
            core::ptr::read_volatile(ESIG_UNIID_BASE),
            core::ptr::read_volatile(ESIG_UNIID_BASE.add(1)),
            core::ptr::read_volatile(ESIG_UNIID_BASE.add(2)),
        ]
    };
    let mut fold: u8 = 0;
    for w in words {
        for shift in (0..32).step_by(8) {
            fold ^= (w >> shift) as u8;
        }
    }
    1u8 + (fold % 252)
}

// Order must mirror the scan tail in `configure_adc_dma_scan`.
fn sensor_inputs(s: &AdcPins) -> [adc::Input; ADC_SENSOR_COUNT] {
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

    // STAT LED defaults solid ON to signal "chip alive"; main loop blinks it
    // on slave-TX activity via `crate::stat_led::poll`.
    gpio::configure(w.stat_led, PinMode::OUTPUT_PUSH_PULL);
    gpio::set_level(w.stat_led, Level::High);
    crate::stat_led::install(w.stat_led);

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

fn configure_adc_dma_scan(sensors: &AdcPins, opa_out_sample_time: adc::SampleTime) {
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
        pl: dma::Pl::LOW,
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

fn bring_up_dxl(d: &DxlBus, brr: u32) {
    let regs = d.usart.regs();
    let half_duplex = matches!(d.duplex, Duplex::Half);
    usart::init(regs, brr, half_duplex);
    store_baud_derived(brr);

    // CH5 (RX snoop) outranks CH4 (TX) so DMA1 arbitration never starves the
    // chain-CRC snoop during TxStreaming, when both channels are servicing
    // USART1 at byte-rate.
    let dma_cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        tcie: false,
        pl: dma::Pl::VERYHIGH,
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
        pl: dma::Pl::HIGH,
    };
    let tx_src = unsafe { (*DXL_TX_BUF.get()).as_ptr() } as u32;
    dma::configure(
        dma::Channel::CH4,
        &tx_cfg,
        usart::data_addr(regs),
        tx_src,
        0,
    );

    // Sole writers; IRQ-only readers unmask below.
    unsafe { *DXL_TX_EN.get() = d.tx_en };
    let rx_pin = d.usart.rx_pin();
    unsafe { *DXL_RX_PIN.get() = Some(rx_pin) };

    usart::set_idle_irq(regs, true);
    // EXTI on the RX pin's falling-edge captures the first byte's start bit
    // for the CALIB cal path (instead of RXNE-after-DMA, which races with
    // V006's DMA-RDR drain at IDLE). on_exti_dxl_rx self-disarms on the
    // first entry; IDLE / TC handlers re-arm.
    exti::configure_falling_edge(rx_pin);
    exti::clear_pending(rx_pin);
    exti::set_irq(rx_pin, true);
}

pub(super) fn seed_dbg_pin(pin: crate::hal::Pin) {
    // SAFETY: called once at bring-up before any ISR can read; chain-CRC
    // ISRs only read.
    unsafe { *DXL_DBG_PIN.get() = Some(pin) };
}

fn start_center_aligned_pwm(m: &MotorConfig, psc: u16, arr: u16) {
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
}
