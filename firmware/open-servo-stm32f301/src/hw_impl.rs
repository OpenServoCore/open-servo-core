use open_servo_control::{mul_div_i32, Adc12, CentiDeg, DeciC, MilliAmp, MilliVolt};
use open_servo_hw::UartPort;
use open_servo_hw_utils::adc_dma;
use stm32f3::stm32f301 as pac;

use crate::adc_sample::AdcSample;
use crate::init::tim::PWM_MAX_DUTY;

// ADC calibration constants
const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;
const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;
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

    #[inline(always)]
    fn gpioa(&self) -> &pac::gpioa::RegisterBlock {
        unsafe { &(*pac::GPIOA::ptr()) }
    }

    fn get_adc_sample() -> AdcSample {
        let buffer = adc_dma::read_block();
        AdcSample::from_dma_buffer(buffer)
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
    pub fn current_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.current_raw
    }

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

    pub fn voltage_raw(&self) -> u16 {
        let sample = Self::get_adc_sample();
        sample.vrefint_raw // VREF ADC value as proxy for bus voltage
    }

    pub fn voltage(&self) -> MilliVolt {
        let sample = Self::get_adc_sample();
        let vdda_mv = Self::convert_vdda_mv(sample.vrefint_raw);
        MilliVolt::from_mv(vdda_mv as i16)
    }

    pub fn temperature_raw(&self) -> Option<u16> {
        let sample = Self::get_adc_sample();
        Some(sample.temperature_raw)
    }

    pub fn temperature(&self) -> Option<DeciC> {
        let sample = Self::get_adc_sample();
        let vdda_mv = Self::convert_vdda_mv(sample.vrefint_raw);
        let temp_dc = Self::convert_temperature_dc(sample.temperature_raw, vdda_mv);
        Some(temp_dc)
    }

    pub fn motor_temperature(&self) -> Option<u16> {
        // Not implemented for V0 - no motor temp sensor
        None
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

    pub fn set_enable(&mut self, en: bool) {
        // For V0, we can use LED pin (PA3) as enable indicator
        if en {
            self.gpioa().odr.modify(|_, w| w.odr3().high());
        } else {
            self.gpioa().odr.modify(|_, w| w.odr3().low());
        }
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
