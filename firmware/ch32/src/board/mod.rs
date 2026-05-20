mod bringup;
mod config;
mod convert;

pub use config::{
    BoardConfig, BoardWiring, Calibration, CurrentSenseConfig, Divider, MotorConfig, NtcCal,
    Sensors,
};

use osc_core::{Board, Capabilities, FrameInputs, MotorCmd, RawSamples, SampleFrame};

use crate::hal::{
    Pin,
    gpio::{self, Level},
    pfic,
};
use crate::statics::{ADC_SCAN_LEN, SHARED, read_sample_tick};

use bringup::{
    bring_up_analog_chain, configure_adc_dma_scan, configure_pins, enable_clocks_and_remaps,
    start_center_aligned_pwm,
};
use convert::{
    SCAN_IDX_NTC, SCAN_IDX_POS, SCAN_IDX_SHUNT_POST, SCAN_IDX_VCAL, SCAN_IDX_VMOTOR_A,
    SCAN_IDX_VMOTOR_B, SCAN_PEAK_OFFSET, SCAN_TROUGH_OFFSET, Scales, VcalLpf, divider_to_mv,
    ntc_to_centi_celsius, pos_to_microrads, shunt_to_milliamps, vmotor_diff_mv,
    volatile_snapshot_scan,
};

pub struct Ch32Board {
    stat_led: Pin,
    calibration: Calibration,
    /// Nominal OPA quiescent ADC count, derived from the configured `opa::Bias`.
    /// Actual quiescent point on this silicon may drift; treat as a starting offset.
    shunt_bias_raw: u16,
    scales: Scales,
    /// EWMA on the noisy IN10/Vcal channel; updated each frame.
    vcal_lpf: VcalLpf,
}

impl Ch32Board {
    pub fn new(cfg: BoardConfig) -> Self {
        crate::log::info!("Ch32Board::new: start");
        let BoardConfig {
            wiring,
            calibration,
            defaults,
        } = cfg;

        let gain_factor = wiring.current_sense.opa_gain.factor();
        let scales = Scales::new(&calibration, gain_factor);
        let stat_led = wiring.stat_led;
        crate::log::debug!(
            "scales: gain_factor={} vbus_q32={} vmotor_q32={} shunt_q32={}",
            gain_factor,
            scales.vbus_q32,
            scales.vmotor_q32,
            scales.shunt_q32,
        );

        enable_clocks_and_remaps(&wiring);
        crate::log::debug!("clocks + remaps configured");
        configure_pins(&wiring);
        crate::log::debug!("gpio configured");
        bring_up_analog_chain(&wiring.current_sense);
        crate::log::debug!("opa settled");
        let shunt_bias_raw = wiring.current_sense.opa_bias.quiescent_raw();
        configure_adc_dma_scan(&wiring.sensors, wiring.current_sense.adc_sample_time);
        crate::log::debug!(
            "adc/dma scan armed: scan_len={} buf_len={} shunt_bias_raw={}",
            crate::statics::ADC_SCAN_LEN,
            crate::statics::ADC_DMA_BUF_LEN,
            shunt_bias_raw,
        );

        // Sole writer to CONFIG: pre-IRQ, pre-install_kernel.
        SHARED.table.seed_config_defaults(&defaults);

        pfic::enable(pfic::Interrupt::DMA1_CHANNEL1);
        crate::log::debug!("pfic: DMA1_CHANNEL1 enabled");
        start_center_aligned_pwm(&wiring.motor);
        crate::log::debug!("pwm running ({} Hz)", wiring.motor.pwm_freq_hz);

        crate::log::info!("Ch32Board::new: complete");
        Self {
            stat_led,
            calibration,
            shunt_bias_raw,
            scales,
            vcal_lpf: VcalLpf::new(),
        }
    }

    #[inline]
    pub fn set_stat_led(&self, on: bool) {
        gpio::set_level(self.stat_led, if on { Level::High } else { Level::Low });
    }

    /// Called from DMA1 TC ISR. Peak half-scan (ON-window centre) drives the
    /// production current reading; trough half-scan stays exposed as a
    /// current-sense diagnostic.
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

    fn write_motor(&mut self, _cmd: MotorCmd) {}

    fn pulse_tick_indicator(&mut self) {
        gpio::toggle(self.stat_led);
    }
}
