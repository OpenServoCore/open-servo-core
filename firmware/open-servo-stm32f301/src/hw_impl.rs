#[cfg(feature = "current-sense-bus")]
use open_servo_control::MilliAmp;
#[cfg(any(feature = "temp-sense-mcu", feature = "temp-sense-motor"))]
use open_servo_control::DeciC;
use open_servo_control::{mul_div_i32, Adc12, CentiDeg, MilliVolt};
#[cfg(feature = "temp-sense-motor")]
use open_servo_control::ntc_lut_to_deci_celsius;
use open_servo_hw::UartPort;
use stm32f3::stm32f301 as pac;

use crate::adc_sample::AdcSample;
use crate::init::tim::PWM_MAX_DUTY;

// ADC calibration constants
const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;
#[cfg(feature = "temp-sense-mcu")]
const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;
#[cfg(feature = "temp-sense-mcu")]
const TS_CAL2_ADDR: u32 = 0x1FFFF7C2;

pub struct Stm32f301Hw;

impl Stm32f301Hw {
    pub fn new() -> Self {
        Stm32f301Hw
    }

    #[inline(always)]
    fn tim1(&self) -> &pac::tim1::RegisterBlock {
        unsafe { &(*pac::TIM1::ptr()) }
    }

    #[inline(always)]
    fn tim2(&self) -> &pac::tim2::RegisterBlock {
        unsafe { &(*pac::TIM2::ptr()) }
    }

    fn get_adc_sample() -> AdcSample {
        AdcSample::read()
    }

    fn convert_vdda_mv(vref_raw: u16) -> u16 {
        // Check for invalid vref_raw to prevent divide by zero
        if vref_raw == 0 {
            // Return nominal voltage if ADC hasn't read yet
            return 3300;
        }
        let vrefint_cal: u16 = unsafe { core::ptr::read(VREFINT_CAL_ADDR as *const u16) };
        // vdda_mv = 3300 * vrefint_cal / vref_raw
        mul_div_i32(3300, vrefint_cal as i32, vref_raw as i32) as u16
    }

    #[cfg(feature = "temp-sense-mcu")]
    fn convert_temperature_dc(adc_value: u16, vdda_mv: u16) -> DeciC {
        let ts_cal1: u16 = unsafe { core::ptr::read(TS_CAL1_ADDR as *const u16) };
        let ts_cal2: u16 = unsafe { core::ptr::read(TS_CAL2_ADDR as *const u16) };

        // Calibration points: 30°C and 110°C at 3.3V
        // Calculate actual temperature using linear interpolation
        // temp_c = 30 + (adc - ts_cal1) * 80 / (ts_cal2 - ts_cal1) * 3300 / vdda_mv

        // Ensure calibration values are valid
        if ts_cal2 <= ts_cal1 || vdda_mv == 0 {
            // Invalid calibration data or vdda, return room temperature
            return DeciC::from_celsius(25);
        }

        let adc_normalized = mul_div_i32(adc_value as i32, 3300, vdda_mv as i32);
        let cal_diff = (ts_cal2 as i32) - (ts_cal1 as i32);
        let adc_diff = adc_normalized - (ts_cal1 as i32);

        let temp_c = 30 + mul_div_i32(adc_diff, 80, cal_diff);

        // Clamp to reasonable range
        let temp_c = temp_c.clamp(-40, 125);
        DeciC::from_celsius(temp_c as i16)
    }
}

impl Stm32f301Hw {
    #[cfg(feature = "current-sense-bus")]
    pub fn current_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.current_raw
    }

    #[cfg(feature = "current-sense-bus")]
    pub fn current(&self) -> MilliAmp {
        let sample = Self::get_adc_sample();
        MilliAmp::from_ipropi_adc(Adc12::from_raw(sample.current_raw))
    }

    pub fn position_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.position_raw
    }

    pub fn position(&self) -> CentiDeg {
        let raw = self.position_raw();
        CentiDeg::from_pot_adc(Adc12::from_raw(raw))
    }

    /// Read raw VREFINT ADC value (used to calculate VDDA).
    pub fn vdda_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.vrefint_raw
    }

    /// Read VDDA (MCU analog supply voltage) in millivolts.
    ///
    /// Note: This is the MCU logic rail (~3.3V), NOT the motor bus voltage.
    pub fn vdda(&self) -> MilliVolt {
        let sample = Self::get_adc_sample();
        let vdda_mv = Self::convert_vdda_mv(sample.vrefint_raw);
        MilliVolt::from_mv(vdda_mv as i16)
    }

    /// Read raw MCU internal temperature sensor ADC value.
    #[cfg(feature = "temp-sense-mcu")]
    pub fn mcu_temperature_raw(&self) -> Option<u16> {
        let sample = Self::get_adc_sample();
        Some(sample.mcu_temp_raw)
    }

    /// Read MCU internal temperature in deci-Celsius.
    #[cfg(feature = "temp-sense-mcu")]
    pub fn mcu_temperature(&self) -> Option<DeciC> {
        let sample = Self::get_adc_sample();
        let vdda_mv = Self::convert_vdda_mv(sample.vrefint_raw);
        let temp_dc = Self::convert_temperature_dc(sample.mcu_temp_raw, vdda_mv);
        Some(temp_dc)
    }

    /// Read raw motor terminal voltage ADC values.
    /// Returns (V+ raw, V- raw).
    #[cfg(feature = "voltage-sense-motor")]
    pub fn motor_voltage_raw(&self) -> (u16, u16) {
        let sample = Self::get_adc_sample();
        (sample.motor_vpos_raw, sample.motor_vneg_raw)
    }

    /// Read both motor terminal voltages (V+, V-) in millivolts.
    ///
    /// Assumes voltage divider maps 0-5V motor voltage to full ADC range.
    #[cfg(feature = "voltage-sense-motor")]
    pub fn motor_voltage(&self) -> (MilliVolt, MilliVolt) {
        let sample = Self::get_adc_sample();
        // Each terminal: 0-4095 ADC = 0-5000mV
        let vpos_mv = mul_div_i32(sample.motor_vpos_raw as i32, 5000, 4095);
        let vneg_mv = mul_div_i32(sample.motor_vneg_raw as i32, 5000, 4095);
        (MilliVolt::from_mv(vpos_mv as i16), MilliVolt::from_mv(vneg_mv as i16))
    }

    /// Read raw motor temperature NTC ADC value.
    #[cfg(feature = "temp-sense-motor")]
    pub fn motor_temperature_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.motor_temp_raw
    }

    /// Read motor temperature in deci-Celsius.
    ///
    /// Uses lookup table for 10K B3950 NTC with 10K fixed resistor, low-side config:
    /// VCC → 10K fixed → ADC (PA4) → NTC → GND
    #[cfg(feature = "temp-sense-motor")]
    pub fn motor_temperature(&self) -> DeciC {
        /// Lookup table: 10K B3950 NTC, 10K fixed resistor, low-side configuration.
        /// Sorted by descending ADC (ascending temperature).
        const MOTOR_NTC_LUT: [(u16, i16); 13] = [
            (3469, -100), // -10°C
            (3133, 0),    //   0°C
            (2725, 100),  //  10°C
            (2274, 200),  //  20°C
            (2048, 250),  //  25°C
            (1826, 300),  //  30°C
            (1421, 400),  //  40°C
            (1078, 500),  //  50°C
            (810, 600),   //  60°C
            (609, 700),   //  70°C
            (459, 800),   //  80°C
            (351, 900),   //  90°C
            (271, 1000),  // 100°C
        ];

        let raw = self.motor_temperature_raw();
        let dc = ntc_lut_to_deci_celsius(raw, &MOTOR_NTC_LUT);
        DeciC::from_dc(dc)
    }

    pub fn set_pwm(&mut self, duty: i32) {
        let max_duty = PWM_MAX_DUTY as i32;
        let duty = duty.clamp(-max_duty, max_duty);

        if duty > 0 {
            self.tim1().ccr1().write(|w| w.ccr().bits(duty as u16));
            self.tim1().ccr4().write(|w| w.ccr().bits(0));
        } else {
            self.tim1().ccr1().write(|w| w.ccr().bits(0));
            self.tim1().ccr4().write(|w| w.ccr().bits((-duty) as u16));
        }
    }

    pub fn set_enable(&mut self, _en: bool) {
        // No hardware enable pin on this board - motor driver is always enabled
    }

    pub fn now_us(&self) -> u32 {
        // Read TIM2 counter (configured to count microseconds)
        self.tim2().cnt.read().cnt().bits()
    }

    pub fn uart_write(&mut self, _port: UartPort, _buf: &[u8]) {
        // Not implemented for V0
    }

    pub fn uart_read_byte(&mut self, _port: UartPort) -> Option<u8> {
        // Not implemented for V0
        None
    }
}
