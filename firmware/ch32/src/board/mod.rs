mod bringup;
mod config;
mod convert;
#[cfg(feature = "defmt")]
mod diag;

pub use config::{
    AdcPins, BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, Duplex, DxlBus,
    MotorConfig, NtcCal, Precomputed, TxEn,
};

use osc_core::{
    DecayMode, FrameInputs, KernelIo, Motor as MotorTrait, MotorCmd, RawSamples, SampleFrame,
    Sensors as SensorsTrait,
};

use crate::hal::{
    Pin,
    gpio::{self, Level},
    timer,
};
use crate::statics::read_sample_tick;

use bringup::BringupResult;
use convert::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, Scales, VcalLpf, divider_to_mv,
    effort_to_ticks, ntc_to_centi_celsius, pos_to_microrads, scan_slot, shunt_to_milliamps,
    vmotor_diff_mv,
};

pub struct Ch32Sensors {
    calibration: Calibration,
    shunt_bias_raw: u16,
    scales: Scales,
    vcal_lpf: VcalLpf,
}

impl SensorsTrait for Ch32Sensors {
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
}

pub struct Ch32Motor {
    in1: timer::Channel,
    in2: timer::Channel,
    drv_en: Pin,
    pwm_arr: u16,
}

impl MotorTrait for Ch32Motor {
    // DRV8212P truth table: (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
    // Slow: idle leg HIGH (CCR>ARR holds PWMMODE1 HIGH), drive leg CCR=ARR-ticks.
    // Fast: idle leg LOW  (CCR=0),                       drive leg CCR=ticks.
    fn write(&mut self, cmd: MotorCmd) {
        const STATIC_HIGH: u16 = u16::MAX;
        match cmd {
            MotorCmd::Disabled => {
                gpio::set_level(self.drv_en, Level::Low);
                timer::set_duty(self.in1, 0);
                timer::set_duty(self.in2, 0);
            }
            MotorCmd::Coast => {
                gpio::set_level(self.drv_en, Level::High);
                timer::set_duty(self.in1, 0);
                timer::set_duty(self.in2, 0);
            }
            MotorCmd::Brake => {
                gpio::set_level(self.drv_en, Level::High);
                timer::set_duty(self.in1, STATIC_HIGH);
                timer::set_duty(self.in2, STATIC_HIGH);
            }
            MotorCmd::Drive { duty, decay } => {
                gpio::set_level(self.drv_en, Level::High);
                let ticks = effort_to_ticks(duty.0.unsigned_abs(), self.pwm_arr);
                if ticks == 0 {
                    // Slow decay at zero ticks would lock both legs HIGH = BRAKE; coast instead.
                    timer::set_duty(self.in1, 0);
                    timer::set_duty(self.in2, 0);
                    return;
                }
                let arr_minus = self.pwm_arr.saturating_sub(ticks);
                let (in1, in2) = match (decay, duty.0 >= 0) {
                    (DecayMode::Slow, true) => (STATIC_HIGH, arr_minus),
                    (DecayMode::Slow, false) => (arr_minus, STATIC_HIGH),
                    (DecayMode::Fast, true) => (ticks, 0),
                    (DecayMode::Fast, false) => (0, ticks),
                };
                timer::set_duty(self.in1, in1);
                timer::set_duty(self.in2, in2);
            }
        }
    }
}

/// Status LED + scope pin — chip-side debug; not part of `KernelIo`.
pub struct Ch32Dbg {
    stat_led: Pin,
    scope: Pin,
}

impl Ch32Dbg {
    /// Toggled once per kernel tick from the ADC DMA TC handler.
    #[inline]
    pub fn pulse_tick(&self) {
        gpio::toggle(self.stat_led);
    }

    /// Pair with `scope_low` around an ISR body to scope rate × runtime.
    #[inline]
    pub fn scope_high(&self) {
        gpio::set_level(self.scope, Level::High);
    }

    #[inline]
    pub fn scope_low(&self) {
        gpio::set_level(self.scope, Level::Low);
    }
}

pub struct Ch32KernelIo {
    pub sensors: Ch32Sensors,
    pub motor: Ch32Motor,
    pub dbg: Ch32Dbg,
}

impl Ch32KernelIo {
    pub fn new(cfg: BoardConfig, pre: Precomputed) -> Self {
        crate::log::info!("Ch32KernelIo::new: start");
        let BoardConfig {
            wiring,
            calibration,
            defaults,
        } = cfg;

        crate::log::debug!(
            "scales: vbus_q32={} vmotor_q32={} shunt_q32={}",
            pre.scales.vbus_q32,
            pre.scales.vmotor_q32,
            pre.scales.shunt_q32,
        );

        let stat_led = wiring.stat_led;
        let scope = wiring.dbg;
        let in1 = wiring.motor.in1;
        let in2 = wiring.motor.in2;
        let drv_en = wiring.motor.drv_en;

        let BringupResult { shunt_bias_raw } = bringup::run(&wiring, &defaults, &pre);

        #[cfg(feature = "defmt")]
        diag::dump_init_regs();

        crate::log::info!("Ch32KernelIo::new: complete");
        Self {
            sensors: Ch32Sensors {
                calibration,
                shunt_bias_raw,
                scales: pre.scales,
                vcal_lpf: VcalLpf::new(),
            },
            motor: Ch32Motor {
                in1,
                in2,
                drv_en,
                pwm_arr: pre.pwm_arr,
            },
            dbg: Ch32Dbg { stat_led, scope },
        }
    }
}

impl KernelIo for Ch32KernelIo {
    type Sensors = Ch32Sensors;
    type Motor = Ch32Motor;

    fn parts(&mut self) -> (&mut Ch32Sensors, &mut Ch32Motor) {
        (&mut self.sensors, &mut self.motor)
    }
}
