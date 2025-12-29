//! ADC + DMA configuration.
//!
//! ADC1 is configured for:
//! - Hardware trigger from TIM1_TRGO2 (at PWM center)
//! - DMA circular mode for automatic buffer fill
//! - DMA transfer complete interrupt for control tick

use core::ptr::addr_of;

use stm32f3::stm32f301::{ADC1, ADC1_2, DMA1};

use crate::adc_config::ADC_CHANNEL_COUNT;
use crate::resources::ADC_DMA_BUF;

/// Configure ADC1 and DMA1_CH1 for triggered conversions.
///
/// Does NOT start conversions (ADSTART not set, DMA not enabled).
pub fn configure_adc() {
    // SAFETY: We have exclusive access during init.
    let adc = unsafe { &*ADC1::ptr() };
    let adc_common = unsafe { &*ADC1_2::ptr() };
    let dma = unsafe { &*DMA1::ptr() };

    // ADC clock configuration (in common register)
    // CKMODE = 0b01: Synchronous clock, HCLK/1
    adc_common.ccr.modify(|_, w| w.ckmode().bits(0b01));

    // Enable ADC voltage regulator
    // ADVREGEN = 0b01 to enable
    adc.cr.modify(|_, w| unsafe { w.advregen().bits(0b01) });

    // Wait for voltage regulator startup (~10µs)
    cortex_m::asm::delay(720); // ~10µs at 72MHz

    // ADC calibration
    adc.cr.modify(|_, w| w.adcal().set_bit());
    while adc.cr.read().adcal().bit_is_set() {}

    // Configure ADC
    // Single conversion mode, DMA circular, hardware trigger
    adc.cfgr.modify(|_, w| {
        w.cont().clear_bit(); // Single conversion mode
        w.dmaen().set_bit(); // DMA enabled
        w.dmacfg().set_bit(); // DMA circular mode
                              // External trigger: TIM1_TRGO2
                              // EXTSEL = 0b01010 for TIM1_TRGO2 (check reference manual)
        unsafe { w.extsel().bits(0b01010) };
        w.exten().rising_edge() // Trigger on rising edge
    });

    // Configure sample time for all channels (181.5 cycles)
    // SMPR1 covers channels 1-9, SMPR2 covers channels 10-18
    adc.smpr1.write(|w| unsafe { w.bits(0x3FFFFFFF) }); // All max sample time
    adc.smpr2.write(|w| unsafe { w.bits(0x07FFFFFF) }); // All max sample time

    // Configure conversion sequence
    // SQR1: L = sequence length - 1, SQ1-SQ4
    // SQR2: SQ5-SQ9
    // etc.
    configure_adc_sequence(adc);

    // Configure DMA1 Channel 1 for ADC
    // Disable channel first
    dma.ch1.cr.modify(|_, w| w.en().clear_bit());

    // Peripheral address = ADC1_DR
    dma.ch1
        .par
        .write(|w| unsafe { w.pa().bits(adc.dr.as_ptr() as u32) });

    // Memory address = ADC_DMA_BUF
    // SAFETY: ADC_DMA_BUF is static and valid for the lifetime of the program
    dma.ch1
        .mar
        .write(|w| unsafe { w.ma().bits(addr_of!(ADC_DMA_BUF) as u32) });

    // Number of data items
    dma.ch1
        .ndtr
        .write(|w| w.ndt().bits(ADC_CHANNEL_COUNT as u16));

    // DMA configuration:
    // - Circular mode
    // - Peripheral to memory
    // - Memory increment
    // - 16-bit transfers
    // - Transfer complete interrupt
    dma.ch1.cr.modify(|_, w| {
        w.circ().set_bit(); // Circular mode
        w.dir().clear_bit(); // Peripheral to memory
        w.minc().set_bit(); // Memory increment
        w.pinc().clear_bit(); // Peripheral fixed
        unsafe {
            w.msize().bits(0b01); // 16-bit memory
            w.psize().bits(0b01); // 16-bit peripheral
            w.pl().bits(0b11); // Very high priority
        }
        w.tcie().set_bit() // Transfer complete interrupt
    });
}

/// Configure ADC conversion sequence based on enabled features.
fn configure_adc_sequence(adc: &stm32f3::stm32f301::adc1::RegisterBlock) {
    // Sequence length in SQR1.L
    let len = ADC_CHANNEL_COUNT as u8 - 1;

    // Build sequence based on features
    // Channel mapping:
    // - VREFINT = ch17 (internal)
    // - Position = ch1 (PA0)
    // - Current = ch2 (PA1) if current-sense-bus
    // - VoltageA = ch3 (PA2) if voltage-sense-motor
    // - VoltageB = ch4 (PA3) if voltage-sense-motor
    // - MotorTemp = ch5 (PA4) if temp-sense-motor
    // - McuTemp = ch16 (internal) if temp-sense-mcu

    // For stage-0, just configure basic sequence
    // SQ1 = ch17 (VREFINT)
    // SQ2 = ch1 (position)
    adc.sqr1.write(|w| unsafe {
        w.l().bits(len);
        w.sq1().bits(17); // VREFINT
        w.sq2().bits(1) // Position
    });

    // Additional channels based on features
    #[cfg(feature = "current-sense-bus")]
    adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(2) }); // Current on ch2
}

/// Start ADC DMA operation.
///
/// Enables DMA channel, then starts ADC conversions.
pub fn start_adc_dma() {
    let adc = unsafe { &*ADC1::ptr() };
    let dma = unsafe { &*DMA1::ptr() };

    // Enable DMA channel first
    dma.ch1.cr.modify(|_, w| w.en().set_bit());

    // Enable ADC
    adc.cr.modify(|_, w| w.aden().set_bit());

    // Wait for ADC ready
    while adc.isr.read().adrdy().bit_is_clear() {}

    // Start conversions (hardware triggered, so just arm it)
    adc.cr.modify(|_, w| w.adstart().set_bit());
}
