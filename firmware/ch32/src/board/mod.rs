mod bringup;
mod config;
mod convert;
#[cfg(feature = "defmt")]
mod diag;

pub use config::{
    BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, Duplex, DxlBus,
    MotorConfig, NtcCal, Sensors, TxEn,
};

use ch32_metapac::USART1;
use osc_core::{Board, DecayMode, FrameInputs, MotorCmd, RawSamples, SampleFrame};

use crate::hal::{
    Pin, dma,
    gpio::{self, Level},
    timer, usart,
};
use crate::statics::{DXL_TX_BUF, DXL_TX_BUF_LEN, DXL_TX_EN, read_sample_tick};

use bringup::BringupResult;
use convert::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, Scales, VcalLpf, divider_to_mv,
    effort_to_ticks, ntc_to_centi_celsius, pos_to_microrads, scan_slot, shunt_to_milliamps,
    vmotor_diff_mv,
};

pub struct Ch32Board {
    stat_led: Pin,
    dbg: Pin,
    calibration: Calibration,
    shunt_bias_raw: u16,
    scales: Scales,
    vcal_lpf: VcalLpf,
    motor_in1: timer::Channel,
    motor_in2: timer::Channel,
    drv_en: Pin,
    pwm_arr: u16,
}

impl Ch32Board {
    pub fn new(cfg: BoardConfig) -> Self {
        crate::log::info!("Ch32Board::new: start");
        let BoardConfig {
            wiring,
            calibration,
            defaults,
        } = cfg;

        let gain_factor = wiring.current_sense.opa.gain.factor();
        let scales = Scales::new(&calibration, gain_factor);
        crate::log::debug!(
            "scales: gain_factor={} vbus_q32={} vmotor_q32={} shunt_q32={}",
            gain_factor,
            scales.vbus_q32,
            scales.vmotor_q32,
            scales.shunt_q32,
        );

        let stat_led = wiring.stat_led;
        let dbg = wiring.dbg;
        let motor_in1 = wiring.motor.in1;
        let motor_in2 = wiring.motor.in2;
        let drv_en = wiring.motor.drv_en;

        let BringupResult {
            shunt_bias_raw,
            pwm_arr,
        } = bringup::run(&wiring, &defaults);

        #[cfg(feature = "defmt")]
        diag::dump_init_regs();

        crate::log::info!("Ch32Board::new: complete");
        Self {
            stat_led,
            dbg,
            calibration,
            shunt_bias_raw,
            scales,
            vcal_lpf: VcalLpf::new(),
            motor_in1,
            motor_in2,
            drv_en,
            pwm_arr,
        }
    }

    #[inline]
    pub fn set_stat_led(&self, on: bool) {
        gpio::set_level(self.stat_led, if on { Level::High } else { Level::Low });
    }

    /// Pair with `dbg_low` around an ISR body to scope rate × runtime.
    #[inline]
    pub fn dbg_high(&self) {
        gpio::set_level(self.dbg, Level::High);
    }

    #[inline]
    pub fn dbg_low(&self) {
        gpio::set_level(self.dbg, Level::Low);
    }

    pub fn dxl_tx_buf(&mut self) -> &mut heapless::Vec<u8, DXL_TX_BUF_LEN> {
        // SAFETY: caller has `&mut self`; the IRQ-side `.clear()` only runs
        // after a `start_dxl_tx` cycle this method initiated.
        unsafe { &mut *DXL_TX_BUF.get() }
    }

    /// Hands `DXL_TX_BUF` to DMA1 CH4 → USART1.DR and arms the TC IRQ.
    /// Caller must have pushed the outbound frame into `dxl_tx_buf()` first.
    pub fn start_dxl_tx(&mut self) {
        let len = self.dxl_tx_buf().len();
        if len == 0 {
            return;
        }

        if let Some(t) = unsafe { *DXL_TX_EN.get() } {
            gpio::set_level(t.pin, t.tx_level);
        }

        dma::set_count(dma::Channel::CH4, len as u16);
        usart::clear_tc(USART1);
        usart::set_dma_tx(USART1, true);
        usart::set_tc_irq(USART1, true);
        dma::enable(dma::Channel::CH4);
    }
}

impl Board for Ch32Board {
    /// Called from DMA1 TC ISR. Peak drives current; trough is diagnostic.
    fn sample(&mut self, inputs: &FrameInputs) -> SampleFrame {
        let raw_shunt_post_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_vmotor_a_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b_trough = scan_slot(SCAN_TROUGH_OFFSET, SCAN_IDX_VMOTOR_B);

        let raw_shunt_post_peak = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_SHUNT_POST);
        let raw_pos = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_POS);
        let raw_ntc = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_NTC);
        let raw_vmotor_a = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_A);
        let raw_vmotor_b = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VMOTOR_B);
        let raw_vcal = scan_slot(SCAN_PEAK_OFFSET, SCAN_IDX_VCAL);
        let raw_vbus = 0u16;

        let filtered_vcal = self.vcal_lpf.update(raw_vcal);
        let vdd_mv = inputs.vdd_mv as u32;

        let post_peak = shunt_to_milliamps(
            raw_shunt_post_peak,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );
        let post_trough = shunt_to_milliamps(
            raw_shunt_post_trough,
            self.shunt_bias_raw,
            vdd_mv,
            self.scales.shunt_q32,
        );

        SampleFrame {
            tick: read_sample_tick(),
            pos: pos_to_microrads(raw_pos, inputs.pos_min_phys_urad, inputs.pos_max_phys_urad),
            current: post_peak,
            current_post_trough: post_trough,
            temp: ntc_to_centi_celsius(raw_ntc, &self.calibration.ntc),
            vbus: divider_to_mv(raw_vbus, vdd_mv, self.scales.vbus_q32),
            vmotor: vmotor_diff_mv(raw_vmotor_a, raw_vmotor_b, vdd_mv, self.scales.vmotor_q32),
            raw: RawSamples {
                pos: raw_pos,
                current: raw_shunt_post_peak,
                shunt_post_trough: raw_shunt_post_trough,
                temp: raw_ntc,
                vbus: raw_vbus,
                vmotor_a: raw_vmotor_a,
                vmotor_a_trough: raw_vmotor_a_trough,
                vmotor_b: raw_vmotor_b,
                vmotor_b_trough: raw_vmotor_b_trough,
                vcal: raw_vcal,
                vcal_lpf: filtered_vcal,
            },
        }
    }

    // DRV8212P truth table: (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
    // Slow: idle leg HIGH (CCR>ARR holds PWMMODE1 HIGH), drive leg CCR=ARR-ticks.
    // Fast: idle leg LOW  (CCR=0),                       drive leg CCR=ticks.
    fn write_motor(&mut self, cmd: MotorCmd) {
        const STATIC_HIGH: u16 = u16::MAX;
        match cmd {
            MotorCmd::Disabled => {
                gpio::set_level(self.drv_en, Level::Low);
                timer::set_duty(self.motor_in1, 0);
                timer::set_duty(self.motor_in2, 0);
            }
            MotorCmd::Coast => {
                gpio::set_level(self.drv_en, Level::High);
                timer::set_duty(self.motor_in1, 0);
                timer::set_duty(self.motor_in2, 0);
            }
            MotorCmd::Brake => {
                gpio::set_level(self.drv_en, Level::High);
                timer::set_duty(self.motor_in1, STATIC_HIGH);
                timer::set_duty(self.motor_in2, STATIC_HIGH);
            }
            MotorCmd::Drive { duty, decay } => {
                gpio::set_level(self.drv_en, Level::High);
                let ticks = effort_to_ticks(duty.0.unsigned_abs(), self.pwm_arr);
                if ticks == 0 {
                    // Slow decay at zero ticks would lock both legs HIGH = BRAKE; coast instead.
                    timer::set_duty(self.motor_in1, 0);
                    timer::set_duty(self.motor_in2, 0);
                    return;
                }
                let arr_minus = self.pwm_arr.saturating_sub(ticks);
                let (in1, in2) = match (decay, duty.0 >= 0) {
                    (DecayMode::Slow, true) => (STATIC_HIGH, arr_minus),
                    (DecayMode::Slow, false) => (arr_minus, STATIC_HIGH),
                    (DecayMode::Fast, true) => (ticks, 0),
                    (DecayMode::Fast, false) => (0, ticks),
                };
                timer::set_duty(self.motor_in1, in1);
                timer::set_duty(self.motor_in2, in2);
            }
        }
    }

    fn pulse_tick_indicator(&mut self) {
        gpio::toggle(self.stat_led);
    }
}
