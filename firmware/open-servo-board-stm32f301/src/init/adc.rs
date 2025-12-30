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
    // Enable VREFINT (always needed for VDD calculation)
    adc_common.ccr.modify(|_, w| {
        w.ckmode().bits(0b01);
        w.vrefen().set_bit()
    });

    // Enable internal temperature sensor if feature enabled
    #[cfg(feature = "temp-sense-mcu")]
    adc_common.ccr.modify(|_, w| w.tsen().set_bit());

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

    // Configure sample times per channel based on source impedance
    // SMPR1 covers channels 1-9, SMPR2 covers channels 10-18
    adc.smpr1.modify(|_, w| {
        w.smp1()
            .cycles61_5() // ch1 position (10kΩ pot)
            .smp2()
            .cycles19_5() // ch2 current (1.5kΩ RIPROPI)
            .smp3()
            .cycles181_5() // ch3 motor V+
            .smp4()
            .cycles181_5() // ch4 motor V-
            .smp5()
            .cycles181_5() // ch5 motor temp
    });
    adc.smpr2.modify(|_, w| {
        w.smp16()
            .cycles181_5() // ch16 MCU temp (2.2µs min)
            .smp18()
            .cycles601_5() // ch18 VREFINT (4µs min!)
    });

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
///
/// Sequence order (matches adc_config::idx):
/// - SQ1: ch18 VREFINT (always)
/// - SQ2: ch1 position (always)
/// - SQ3: ch2 current (if current-sense-bus)
/// - SQ4/5: ch3/ch4 motor V+/V- (if voltage-sense-motor)
/// - SQn: ch5 motor temp (if temp-sense-motor)
/// - SQn: ch16 MCU temp (if temp-sense-mcu)
fn configure_adc_sequence(adc: &stm32f3::stm32f301::adc1::RegisterBlock) {
    // Sequence length in SQR1.L (L = number of conversions - 1)
    let len = ADC_CHANNEL_COUNT as u8 - 1;

    // Channel mapping:
    // - VREFINT = ch18 (internal)
    // - Position = ch1 (PA0)
    // - Current = ch2 (PA1) if current-sense-bus
    // - VoltageA = ch3 (PA2) if voltage-sense-motor
    // - VoltageB = ch4 (PA3) if voltage-sense-motor
    // - MotorTemp = ch5 (PA4) if temp-sense-motor
    // - McuTemp = ch16 (internal) if temp-sense-mcu

    // Track current sequence slot (1-indexed for SQR registers)
    let mut slot = 1u8;

    // SQ1 = ch18 (VREFINT) - always first
    // SQ2 = ch1 (position) - always second
    adc.sqr1.write(|w| unsafe {
        w.l().bits(len);
        w.sq1().bits(18); // VREFINT on ch18
        w.sq2().bits(1) // Position on ch1
    });
    slot = 3;

    // Current sense (ch2) if enabled
    #[cfg(feature = "current-sense-bus")]
    {
        adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(2) });
        slot = 4;
    }

    // Motor voltage sensing (ch3, ch4) if enabled
    #[cfg(feature = "voltage-sense-motor")]
    {
        match slot {
            3 => {
                adc.sqr1.modify(|_, w| unsafe {
                    w.sq3().bits(3); // V+ on ch3
                    w.sq4().bits(4) // V- on ch4
                });
            }
            4 => {
                adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(3) }); // V+ on ch3
                adc.sqr2.write(|w| unsafe { w.sq5().bits(4) }); // V- on ch4
            }
            _ => {}
        }
        slot += 2;
    }

    // Motor temperature (ch5) if enabled
    #[cfg(feature = "temp-sense-motor")]
    {
        match slot {
            3 => adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(5) }),
            4 => adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(5) }),
            5 => adc.sqr2.modify(|_, w| unsafe { w.sq5().bits(5) }),
            6 => adc.sqr2.modify(|_, w| unsafe { w.sq6().bits(5) }),
            _ => {}
        }
        slot += 1;
    }

    // MCU temperature (ch16) if enabled - always last
    #[cfg(feature = "temp-sense-mcu")]
    {
        match slot {
            3 => adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(16) }),
            4 => adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(16) }),
            5 => adc.sqr2.modify(|_, w| unsafe { w.sq5().bits(16) }),
            6 => adc.sqr2.modify(|_, w| unsafe { w.sq6().bits(16) }),
            7 => adc.sqr2.modify(|_, w| unsafe { w.sq7().bits(16) }),
            _ => {}
        }
    }

    // Suppress unused variable warning when no optional features enabled
    let _ = slot;
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
