use stm32f3::stm32f301 as pac;

use super::init::tim::PWM_MAX_DUTY;
use crate::hw::{adc_dma, Hw, UartPort};

// ADC calibration constants
const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;
const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;
const TS_CAL2_ADDR: u32 = 0x1FFFF7C2;
const ADC_MAX_VALUE: f32 = 4095.0; // 12-bit ADC

// Current sense parameters
const ISENSE_AIPROPI: f32 = 1500.0; // uA/A
const ISENSE_RESISTOR_OHM: f32 = 2200.0; // Ohm
const ISENSE_FACTOR: f32 = ISENSE_AIPROPI / ISENSE_RESISTOR_OHM; // uA / V

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

    fn get_adc_values() -> [u16; 5] {
        adc_dma::read_block()
    }

    fn convert_vdda(vref_raw: u16) -> f32 {
        let vrefint_cal: u16 = unsafe { core::ptr::read(VREFINT_CAL_ADDR as *const u16) };
        3.3 * vrefint_cal as f32 / vref_raw as f32
    }

    fn convert_current(adc_value: u16, vdda: f32) -> f32 {
        let voltage = vdda * adc_value as f32 / ADC_MAX_VALUE;
        voltage * ISENSE_FACTOR / 1000.0 // convert uA to mA
    }

    fn convert_temperature(adc_value: u16, vdda: f32) -> f32 {
        let ts_cal1: u16 = unsafe { core::ptr::read(TS_CAL1_ADDR as *const u16) };
        let ts_cal2: u16 = unsafe { core::ptr::read(TS_CAL2_ADDR as *const u16) };
        let ts_cal1_temp: f32 = 30.0;
        let ts_cal2_temp: f32 = 110.0;
        let ts_cal1_voltage: f32 = 3.3 * ts_cal1 as f32 / ADC_MAX_VALUE;
        let ts_cal2_voltage: f32 = 3.3 * ts_cal2 as f32 / ADC_MAX_VALUE;
        let ts_slope: f32 = (ts_cal2_temp - ts_cal1_temp) / (ts_cal2_voltage - ts_cal1_voltage);
        let ts_offset: f32 = ts_cal1_temp - ts_slope * ts_cal1_voltage;
        let temp_voltage = vdda * adc_value as f32 / ADC_MAX_VALUE;
        ts_slope * temp_voltage + ts_offset
    }
}

impl Hw for Stm32f301Hw {
    fn phase_current(&self) -> u16 {
        let [vref, _pot, isns, _setpoint, _temp] = Self::get_adc_values();
        let vdda = Self::convert_vdda(vref);
        let current_ma = Self::convert_current(isns, vdda);
        current_ma as u16
    }

    fn position(&self) -> u16 {
        let [_vref, pot, _isns, _setpoint, _temp] = Self::get_adc_values();
        pot
    }

    fn bus_voltage(&self) -> u16 {
        let [vref, _pot, _isns, _setpoint, _temp] = Self::get_adc_values();
        let vdda = Self::convert_vdda(vref);
        (vdda * 1000.0) as u16 // Convert to millivolts
    }

    fn temperature(&self) -> Option<u16> {
        let [vref, _pot, _isns, _setpoint, temp] = Self::get_adc_values();
        let vdda = Self::convert_vdda(vref);
        let temp_celsius = Self::convert_temperature(temp, vdda);
        let temp_kelvin = temp_celsius + 273.15;
        Some((temp_kelvin * 10.0) as u16) // Convert to decikelvin
    }

    fn motor_temperature(&self) -> Option<u16> {
        // Not implemented for V0 - no motor temp sensor
        None
    }

    fn set_pwm(&mut self, duty: i32) {
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

    fn set_enable(&mut self, en: bool) {
        // For V0, we can use LED pin (PA3) as enable indicator
        if en {
            self.gpioa().odr.modify(|_, w| w.odr3().high());
        } else {
            self.gpioa().odr.modify(|_, w| w.odr3().low());
        }
    }

    fn now_us(&self) -> u32 {
        // Read TIM2 counter (configured to count microseconds)
        self.tim2().cnt.read().cnt().bits()
    }

    fn uart_write(&mut self, _port: UartPort, _buf: &[u8]) {
        // Not implemented for V0
    }

    fn uart_read_byte(&mut self, _port: UartPort) -> Option<u8> {
        // Not implemented for V0
        None
    }
}
