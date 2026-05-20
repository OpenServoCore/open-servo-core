mod bringup;
mod config;
mod convert;
#[cfg(feature = "defmt")]
mod diag;

pub use config::{
    BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, MotorConfig, NtcCal,
    Sensors,
};

use osc_core::{Board, Capabilities, FrameInputs, MotorCmd, RawSamples, SampleFrame};

use crate::hal::{
    Pin,
    gpio::{self, Level},
    timer,
};
use crate::statics::{ADC_SCAN_LEN, read_sample_tick};

use bringup::BringupResult;
use convert::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, Scales, VcalLpf, divider_to_mv,
    ntc_to_centi_celsius, pos_to_microrads, shunt_to_milliamps, vmotor_diff_mv,
    volatile_snapshot_scan,
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

    /// `mag·arr / 32767`, approximated with `>>15` to dodge soft-div in the ISR.
    fn effort_to_ticks(&self, mag: u16) -> u16 {
        let m = mag.min(i16::MAX as u16) as u32;
        let prod = m * self.pwm_arr as u32;
        ((prod + (1 << 14)) >> 15) as u16
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

    /// Called from DMA1 TC ISR. Peak drives current; trough is diagnostic.
    pub fn build_sample_frame(&mut self, inputs: &FrameInputs) -> SampleFrame {
        let scan = volatile_snapshot_scan();
        let peak = &scan[SCAN_PEAK_OFFSET..SCAN_PEAK_OFFSET + ADC_SCAN_LEN];
        let trough = &scan[SCAN_TROUGH_OFFSET..SCAN_TROUGH_OFFSET + ADC_SCAN_LEN];

        let raw_shunt_post_peak = peak[SCAN_IDX_SHUNT_POST];
        let raw_shunt_post_trough = trough[SCAN_IDX_SHUNT_POST];
        let raw_pos = peak[SCAN_IDX_POS];
        let raw_ntc = peak[SCAN_IDX_NTC];
        let raw_vbus = 0u16;
        let raw_vmotor_a = peak[SCAN_IDX_VMOTOR_A];
        let raw_vmotor_a_trough = trough[SCAN_IDX_VMOTOR_A];
        let raw_vmotor_b = peak[SCAN_IDX_VMOTOR_B];
        let raw_vmotor_b_trough = trough[SCAN_IDX_VMOTOR_B];
        let raw_vcal = peak[SCAN_IDX_VCAL];

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

impl Board for Ch32Board {
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }

    // DRV8212P truth table: (1,1)=BRAKE, (0,0)=COAST, (1,0)=fwd, (0,1)=rev.
    // Slow decay: idle leg parked HIGH via CCR>ARR (PWMMODE1 holds HIGH);
    // driving leg's CCR=ARR−ticks dips LOW for the drive window.
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
            MotorCmd::Drive { duty, decay: _ } => {
                gpio::set_level(self.drv_en, Level::High);
                let ticks = self.effort_to_ticks(duty.0.unsigned_abs());
                let drive_ccr = self.pwm_arr.saturating_sub(ticks);
                if duty.0 >= 0 {
                    timer::set_duty(self.motor_in1, STATIC_HIGH);
                    timer::set_duty(self.motor_in2, drive_ccr);
                } else {
                    timer::set_duty(self.motor_in1, drive_ccr);
                    timer::set_duty(self.motor_in2, STATIC_HIGH);
                }
            }
        }
    }

    fn pulse_tick_indicator(&mut self) {
        gpio::toggle(self.stat_led);
    }
}
