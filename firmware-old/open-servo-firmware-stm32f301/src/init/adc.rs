//! ADC + DMA configuration.
//!
//! ADC1 is configured for:
//! - Hardware trigger from TIM1_TRGO2 (at PWM center via OC3REF)
//! - DMA circular mode for automatic buffer fill
//! - DMA transfer complete interrupt for control tick

use core::ptr::addr_of;

use cortex_m::asm;
use stm32f3::stm32f301::{ADC1, ADC1_2, DMA1};

use crate::adc_config::ADC_CHANNEL_COUNT;
use crate::resources::ADC_DMA_BUF;

/// Configure ADC1 and DMA1_CH1 for triggered conversions.
///
/// Order matches old open-servo-stm32f301 crate:
/// DMA config → DMA enable → ADC common → ADC CFGR → calibration → sequence → enable
///
/// Does NOT start conversions (ADSTART not set until start_adc_dma).
pub fn configure_adc() {
    // SAFETY: We have exclusive access during init.
    let adc = unsafe { &*ADC1::ptr() };
    let adc_common = unsafe { &*ADC1_2::ptr() };
    let dma = unsafe { &*DMA1::ptr() };

    // === DMA Configuration FIRST (like old crate) ===

    // Disable channel first
    dma.ch1.cr.modify(|_, w| w.en().disabled());

    // Configure DMA1 Channel 1 for ADC
    dma.ch1.cr.modify(|_, w| {
        w.pl()
            .very_high()
            .circ()
            .enabled()
            .dir()
            .from_peripheral()
            .msize()
            .bits16()
            .psize()
            .bits16()
            .tcie()
            .enabled()
            .minc()
            .enabled()
            .pinc()
            .disabled()
    });

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

    // Enable DMA channel BEFORE ADC config (like old crate)
    dma.ch1.cr.modify(|_, w| w.en().enabled());

    // === ADC Common Configuration ===

    // ADC clock configuration (in common register)
    // CKMODE = sync_div1: Synchronous clock, HCLK/1
    // Enable VREFINT (always needed for VDD calculation)
    adc_common.ccr.modify(|_, w| {
        let w = w.ckmode().sync_div1().vrefen().enabled();
        #[cfg(feature = "temp-sense-mcu")]
        let w = w.tsen().enabled();
        w
    });

    // === ADC CFGR (add all fields like old crate) ===

    adc.cfgr.modify(|_, w| {
        w.res()
            .bits12()
            .align()
            .right()
            .cont()
            .single()
            .discen()
            .disabled()
            .dmacfg()
            .circular()
            .dmaen()
            .enabled()
            .ovrmod()
            .preserve()
            .exten()
            .rising_edge()
            .extsel()
            .tim1_trgo2()
    });

    // === ADC Calibration (fix sequence like old crate) ===

    // Enable internal voltage regulator
    // Must go through intermediate state first
    adc.cr.modify(|_, w| w.advregen().intermediate());
    adc.cr.modify(|_, w| w.advregen().enabled());

    // Wait for voltage regulator startup (~10µs at 72MHz = 720 cycles)
    asm::delay(10 * 72);

    // Run calibration (single-ended mode)
    adc.cr
        .modify(|_, w| w.adcaldif().single_ended().adcal().calibration());
    while adc.cr.read().adcal().is_calibration() {}

    // Disable voltage regulator after calibration (like old crate)
    // The internal analog calibration is kept
    adc.cr.modify(|_, w| w.advregen().intermediate());
    adc.cr.modify(|_, w| w.advregen().disabled());

    // Wait 4 ADC clock cycles after calibration before setting ADEN
    asm::delay(4);

    // === Configure ADC sequence ===

    configure_adc_sequence(adc);

    // === Enable ADC ===

    adc.cr.modify(|_, w| w.aden().enabled());
    while adc.isr.read().adrdy().is_not_ready() {}
}

/// Configure ADC sequence slots based on enabled features.
fn configure_adc_sequence(adc: &stm32f3::stm32f301::adc1::RegisterBlock) {
    // Set sequence length
    let len = ADC_CHANNEL_COUNT as u8 - 1;

    // SQ1 = ch18 (VREFINT) - always first
    // SQ2 = ch1 (position) - always second
    adc.sqr1.write(|w| unsafe {
        w.l().bits(len);
        w.sq1().bits(18); // VREFINT on ch18
        w.sq2().bits(1) // Position on ch1
    });

    // Track current sequence slot (1-indexed for SQR registers)
    let mut slot = 3u8;

    // Current sense (ch2) if enabled
    #[cfg(feature = "current-sense-bus")]
    {
        set_sequence_slot(adc, slot, 2);
        slot += 1;
    }

    // Motor voltage sensing (ch3, ch4) if enabled
    #[cfg(feature = "voltage-sense-motor")]
    {
        set_sequence_slot(adc, slot, 3);
        slot += 1;
        set_sequence_slot(adc, slot, 4);
        slot += 1;
    }

    // Motor temperature (ch5) if enabled
    #[cfg(feature = "temp-sense-motor")]
    {
        set_sequence_slot(adc, slot, 5);
        slot += 1;
    }

    // MCU temperature (ch16) if enabled - always last
    #[cfg(feature = "temp-sense-mcu")]
    {
        set_sequence_slot(adc, slot, 16);
    }

    // Suppress unused variable warning when no optional features enabled
    let _ = slot;

    // Configure sample times
    configure_sample_times(adc);
}

/// Set a specific sequence slot to a channel number.
fn set_sequence_slot(adc: &stm32f3::stm32f301::adc1::RegisterBlock, slot: u8, channel: u8) {
    match slot {
        3 => adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(channel) }),
        4 => adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(channel) }),
        5 => adc.sqr2.modify(|_, w| unsafe { w.sq5().bits(channel) }),
        6 => adc.sqr2.modify(|_, w| unsafe { w.sq6().bits(channel) }),
        7 => adc.sqr2.modify(|_, w| unsafe { w.sq7().bits(channel) }),
        8 => adc.sqr2.modify(|_, w| unsafe { w.sq8().bits(channel) }),
        9 => adc.sqr2.modify(|_, w| unsafe { w.sq9().bits(channel) }),
        _ => {}
    }
}

/// Configure sample times for all used channels.
fn configure_sample_times(adc: &stm32f3::stm32f301::adc1::RegisterBlock) {
    // SMPR1 covers channels 1-9
    adc.smpr1.modify(|_, w| {
        w.smp1().cycles181_5() // ch1 position
    });

    #[cfg(feature = "current-sense-bus")]
    adc.smpr1.modify(|_, w| w.smp2().cycles181_5());

    #[cfg(feature = "voltage-sense-motor")]
    {
        adc.smpr1.modify(|_, w| w.smp3().cycles181_5());
        adc.smpr1.modify(|_, w| w.smp4().cycles181_5());
    }

    #[cfg(feature = "temp-sense-motor")]
    adc.smpr1.modify(|_, w| w.smp5().cycles181_5());

    // SMPR2 covers channels 10-18
    adc.smpr2.modify(|_, w| {
        let w = w.smp18().cycles601_5(); // VREFINT (4µs min!)
        #[cfg(feature = "temp-sense-mcu")]
        let w = w.smp16().cycles181_5(); // MCU temp
        w
    });
}

/// Start ADC DMA operation.
///
/// DMA and ADC are already enabled in configure_adc.
/// This just starts ADC conversions.
pub fn start_adc_dma() {
    let adc = unsafe { &*ADC1::ptr() };

    // Start conversions (hardware triggered by TIM1_TRGO2)
    adc.cr.modify(|_, w| w.adstart().start_conversion());
}
