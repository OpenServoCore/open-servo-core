use super::registry::{DXL_EDGE_BUF_LEN, DXL_RX_BUF_LEN};
use ch32_metapac::{ADC, adc::vals::Extsel, dma::vals::Dir, timer::vals::FilterValue};
use osc_core::{ConfigDefaults, RegionStorage};
use osc_drivers::Level;

use crate::control::sensors::scan::{ADC_DMA_BUF, ADC_DMA_BUF_LEN, ADC_SCAN_LEN, ADC_SENSOR_COUNT};
use crate::hal::{
    adc, afio, delay_ms, dma,
    gpio::{self, PinMode},
    opa, rcc, systick, timer, usart,
};
use crate::providers::usart_baud;
use crate::runtime::Drivers;
use crate::runtime::statics::SHARED;

use crate::cfg::{
    AdcPins, BoardWiring, CurrentSenseConfig, Duplex, DxlUart, MotorConfig, Precomputed, TxEn,
};

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
    rcc::apply_clock_trim_delta(defaults.clock_trim);
    crate::log::debug!(
        "clocks + remaps configured (clock_trim={})",
        defaults.clock_trim
    );

    // Must run before `configure_pins`: TIM2 OC2M=Force-inactive sets the
    // idle level on PC2's CC2 output, so the moment AF mode latches the pin
    // sees the inactive level instead of an indeterminate window. CC3 has no
    // pin output — initialized here so the IRQ stays masked at boot.
    bring_up_tim2_oc(&wiring.dxl);
    configure_pins(wiring);
    // SAFETY: bringup-only, pre-IRQ; sole writer.
    unsafe { Drivers::install(wiring, defaults) };
    crate::log::debug!("gpio configured");

    bring_up_analog_chain(&wiring.current_sense);
    crate::log::debug!("opa settled");

    // SysTick drives `Monotonic` (LED blinker) and the Fast Last CMP
    // scheduler. Initialize *after* `bring_up_analog_chain` because
    // `delay_ms` reinitializes SYSTICK on every call; doing it here puts
    // SysTick in a known state (CMP=u32::MAX, CNT=0, STE=on, STIE=off)
    // independent of any further `delay_ms` use.
    systick::init();

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

    bring_up_dxl(
        &wiring.dxl,
        pre.usart_brr,
        usart_baud::filter_for(defaults.dxl_baud),
    );
    crate::log::debug!("dxl usart + dma rx armed");

    start_center_aligned_pwm(&wiring.motor, pre.pwm_psc, pre.pwm_arr);
    crate::log::debug!(
        "pwm running ({} Hz, psc={}, arr={})",
        wiring.motor.pwm_freq_hz,
        pre.pwm_psc,
        pre.pwm_arr,
    );

    #[cfg(feature = "defmt")]
    super::diag::dump_init_regs();

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
    rcc::enable_tim2();
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

fn configure_dxl_pins(d: &DxlUart) {
    gpio::configure(d.usart.tx_pin(), PinMode::AF_PUSH_PULL);
    if matches!(d.duplex, Duplex::Full) {
        gpio::configure(d.usart.rx_pin(), PinMode::input_pull(d.rx_pull));
    }
    if let Some(ref t) = d.tx_en {
        gpio::configure(t.pin, PinMode::AF_PUSH_PULL);
    }
}

fn bring_up_tim2_oc(d: &DxlUart) {
    let tx_active_high = match d.tx_en {
        Some(TxEn {
            tx_level: Level::High,
            ..
        }) => true,
        Some(TxEn {
            tx_level: Level::Low,
            ..
        }) => false,
        None => true,
    };
    timer::init_tim2_tx_oc_channels(tx_active_high);
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
        htie: false,
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

fn bring_up_dxl(d: &DxlUart, brr: u32, boot_filter: FilterValue) {
    let regs = d.usart.regs();
    let half_duplex = matches!(d.duplex, Duplex::Half);
    usart::init(regs, brr, half_duplex);

    // ADC (CH1) + USART RX/TX channels stay at LOW per
    // `docs/dxl-hw-timed-transport.md` §6, so ET (CH7) is the only DMA
    // channel at VERYHIGH — guaranteeing IC capture writes win arbitration
    // against the byte-rate USART RX snoop.
    let dma_cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        htie: false,
        tcie: false,
        pl: dma::Pl::LOW,
    };
    // SAFETY: see `bring_up_edge_ts_capture`.
    let rx_addr = unsafe { Drivers::dxl_uart() }.rx_buf_addr() as u32;
    dma::configure(
        dma::Channel::CH5,
        &dma_cfg,
        usart::data_addr(regs),
        rx_addr,
        DXL_RX_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH5);
    usart::set_dma_rx(regs, true);

    // DMA1_CH4 source is the driver's TX buffer — M2 (#33) replaces the
    // legacy DXL_TX_BUF static. The channel stays armed but disabled;
    // `DxlTxScheduler` (M3 #5) will enable it per-fire alongside the
    // hardware TX_EN on TIM2_CH2. Until M3, the channel never enables and
    // wire TX is silent by design.
    let tx_cfg = dma::Config {
        dir: Dir::FROMMEMORY,
        circ: false,
        pinc: false,
        minc: true,
        size: dma::Size::BITS8,
        htie: false,
        tcie: false,
        pl: dma::Pl::LOW,
    };
    let tx_src = unsafe { Drivers::dxl_uart() }.tx_buf_addr() as u32;
    dma::configure(
        dma::Channel::CH4,
        &tx_cfg,
        usart::data_addr(regs),
        tx_src,
        0,
    );

    usart::set_idle_irq(regs, true);

    bring_up_edge_ts_capture(boot_filter);
}

/// TIM2_CH4 input capture on the RX pin (falling edge) feeds DMA1_CH7 into
/// the RX driver's edges buffer. The HT/TC ISR walks newly-captured
/// edges through the window classifier; IDLE drains the tail when a packet
/// doesn't fill a half. PL=VeryHigh is the only such channel (ADC + USART
/// RX/TX all sit at LOW per doc §6) so CC4 stores can't be delayed.
fn bring_up_edge_ts_capture(boot_filter: FilterValue) {
    timer::init_tim2_ch4_ic_capture(boot_filter);

    let edge_ts_cfg = dma::Config {
        dir: Dir::FROMPERIPHERAL,
        circ: true,
        pinc: false,
        minc: true,
        size: dma::Size::BITS16,
        htie: true,
        tcie: true,
        pl: dma::Pl::VERYHIGH,
    };
    // SAFETY: `Drivers::install` ran in `run()` before `bring_up_dxl`; the
    // returned address points into the driver's registry cell and stays
    // valid for the lifetime of the program. The driver yields `usize`
    // (chip-agnostic); we narrow to the V006 DMA-MAR width here.
    let edges_addr = unsafe { Drivers::dxl_uart() }.edges_addr() as u32;
    dma::configure(
        dma::Channel::CH7,
        &edge_ts_cfg,
        timer::tim2_ch4_capture_addr(),
        edges_addr,
        DXL_EDGE_BUF_LEN as u16,
    );
    dma::enable(dma::Channel::CH7);
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
