use cortex_m::asm;
use stm32f3::stm32f301 as pac;

use crate::adc_config::{dma_target_ptr, ADC_CHANNELS};

/// Configure ADC sequence slots 3+ based on enabled features.
/// Slots 1-2 are always vrefint and position, configured separately.
/// Order: [current], [motor_v+], [motor_v-], [motor_temp], [mcu_temp]
fn configure_adc_sequence(adc1: &pac::adc1::RegisterBlock) {
    // Track which slot we're configuring (starting at 3)
    let mut _slot: u8 = 3;

    // Current sense (PA1 = ch 2)
    #[cfg(feature = "current-sense-bus")]
    {
        set_sequence_slot(adc1, _slot, 2);
        _slot += 1;
    }

    // Motor V+ (PA2 = ch 3)
    #[cfg(feature = "voltage-sense-motor")]
    {
        set_sequence_slot(adc1, _slot, 3);
        _slot += 1;
    }

    // Motor V- (PA3 = ch 4)
    #[cfg(feature = "voltage-sense-motor")]
    {
        set_sequence_slot(adc1, _slot, 4);
        _slot += 1;
    }

    // Motor temp (PA4 = ch 5)
    #[cfg(feature = "temp-sense-motor")]
    {
        set_sequence_slot(adc1, _slot, 5);
        _slot += 1;
    }

    // MCU temp (internal ch 16) - always last when enabled
    #[cfg(feature = "temp-sense-mcu")]
    {
        set_sequence_slot(adc1, _slot, 16);
        // _slot += 1; // not needed, this is the last one
    }
}

/// Set a specific sequence slot to a channel number.
/// Slots 1-4 are in SQR1, slots 5-9 are in SQR2, etc.
fn set_sequence_slot(adc1: &pac::adc1::RegisterBlock, slot: u8, channel: u8) {
    match slot {
        3 => adc1.sqr1.modify(|_, w| unsafe { w.sq3().bits(channel) }),
        4 => adc1.sqr1.modify(|_, w| unsafe { w.sq4().bits(channel) }),
        5 => adc1.sqr2.modify(|_, w| unsafe { w.sq5().bits(channel) }),
        6 => adc1.sqr2.modify(|_, w| unsafe { w.sq6().bits(channel) }),
        7 => adc1.sqr2.modify(|_, w| unsafe { w.sq7().bits(channel) }),
        8 => adc1.sqr2.modify(|_, w| unsafe { w.sq8().bits(channel) }),
        9 => adc1.sqr2.modify(|_, w| unsafe { w.sq9().bits(channel) }),
        _ => {} // Slots beyond 9 would use SQR3/SQR4, not needed here
    }
}

pub fn init_dma(p: &pac::Peripherals) {
    // enable DMA1 Channel 1
    p.DMA1.ch1.cr.modify(|_, w| {
        w.pl()
            .low() // low priority
            .circ()
            .enabled() // enable circular mode
            .dir()
            .from_peripheral() // read from peripheral
            .msize()
            .bits16() // 16 bit memory size
            .psize()
            .bits16() // 16 bit peripheral size
            .tcie()
            .enabled() // enable transfer complete interrupt
            .teie()
            .enabled() // enable transfer error interrupt
            .mem2mem()
            .disabled() // memory to memory mode disabled
            .minc()
            .enabled() // increment memory address
            .pinc()
            .disabled() // do not increment peripheral address
    });

    p.DMA1
        .ch1
        .par
        .write(|w| unsafe { w.bits(p.ADC1.dr.as_ptr() as u32) }); // set peripheral address to ADC1_DR

    p.DMA1
        .ch1
        .mar
        .write(|w| unsafe { w.bits(dma_target_ptr() as u32) }); // set memory address for DMA

    // set number of data to transfer based on configured channels
    p.DMA1.ch1.ndtr.write(|w| w.ndt().bits(ADC_CHANNELS as u16));

    // enable DMA1 Channel 1
    p.DMA1.ch1.cr.modify(|_, w| w.en().enabled());
}

pub fn init_adc(p: &pac::Peripherals) {
    let adc1 = &p.ADC1;

    /*
     * Configure ADC
     */
    p.ADC1_2.ccr.modify(|_, w| {
        let w = w
            .ckmode()
            .sync_div1() // use synchronous clock mode
            .vrefen()
            .enabled(); // enable VREFINT
        #[cfg(feature = "temp-sense-mcu")]
        let w = w.tsen().enabled(); // enable temperature sensor
        w
    });

    adc1.cfgr.modify(|_, w| {
        w.res()
            .bits12() // set ADC to 12 bit resolution
            .align()
            .right() // set ADC to right aligned
            .cont()
            .single() // set ADC to single conversion mode
            .discen()
            .disabled() // set ADC to disable discontinuous mode
            .dmacfg()
            .circular() // set ADC to circular DMA mode
            .dmaen()
            .enabled() // enable DMA
            .ovrmod()
            .preserve() // set ADC to preserve overrun
            .exten()
            .rising_edge() // set ADC to rising edge trigger
            .extsel()
            .tim1_trgo2() // set ADC to TIM1_TRGO2
    });

    /*
     * ADC calibration
     */
    // enable internal voltage regulator
    adc1.cr.modify(|_, w| w.advregen().intermediate());
    adc1.cr.modify(|_, w| w.advregen().enabled());
    // we need to wait 10 us for the voltage regulator to stabilize
    // we are running at 72 MHz, so 1 us = 72 cycles
    asm::delay(10 * 72);

    // run calibration
    adc1.cr
        .modify(|_, w| w.adcaldif().single_ended().adcal().calibration());
    while adc1.cr.read().adcal().is_calibration() {}

    // disable the internal voltage regulator, the internal analog calibration is kept
    adc1.cr.modify(|_, w| w.advregen().intermediate());
    adc1.cr.modify(|_, w| w.advregen().disabled());

    // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is
    // cleared by hardware
    // ADC clock cycle is set to be 72 MHz / 1 = 72 MHz
    asm::delay(4);

    /*
     * Configure ADC Channels
     */
    // configure GPIO pins as analog input
    let gpioa = &p.GPIOA;
    // PA0 (position) - always enabled
    gpioa.moder.modify(|_, w| w.moder0().analog());
    gpioa.pupdr.modify(|_, w| w.pupdr0().floating());

    // PA1 (current) - only if current-sense-bus enabled
    #[cfg(feature = "current-sense-bus")]
    {
        gpioa.moder.modify(|_, w| w.moder1().analog());
        gpioa.pupdr.modify(|_, w| w.pupdr1().floating());
    }

    // PA2 (motor V+) - only if voltage-sense-motor enabled
    #[cfg(feature = "voltage-sense-motor")]
    {
        gpioa.moder.modify(|_, w| w.moder2().analog());
        gpioa.pupdr.modify(|_, w| w.pupdr2().floating());
    }

    // PA3 (motor V-) - only if voltage-sense-motor enabled
    #[cfg(feature = "voltage-sense-motor")]
    {
        gpioa.moder.modify(|_, w| w.moder3().analog());
        gpioa.pupdr.modify(|_, w| w.pupdr3().floating());
    }

    // PA4 (motor temp) - only if temp-sense-motor enabled
    #[cfg(feature = "temp-sense-motor")]
    {
        gpioa.moder.modify(|_, w| w.moder4().analog());
        gpioa.pupdr.modify(|_, w| w.pupdr4().floating());
    }

    // Configure ADC sequence based on enabled features
    // Order: vrefint, position, [current], [motor_v+], [motor_v-], [motor_temp], [mcu_temp]
    // Set sequence length
    adc1.sqr1.modify(|_, w| {
        w.l().bits((ADC_CHANNELS - 1) as u8) // number of conversions minus 1
    });

    // SQ1 and SQ2 are always set (vrefint, position)
    adc1.sqr1.modify(|_, w| unsafe {
        w.sq1()
            .bits(18) // vrefint (ch 18)
            .sq2()
            .bits(1) // PA0 position (ch 1)
    });

    // Build sequence dynamically - track next slot
    // Slots 3-4 are in SQR1, slots 5-9 are in SQR2, etc.
    configure_adc_sequence(adc1);

    // Set sample time for all used channels
    // PA0 (position) - always
    adc1.smpr1.modify(|_, w| w.smp1().cycles181_5());

    // PA1 (current) - if enabled
    #[cfg(feature = "current-sense-bus")]
    adc1.smpr1.modify(|_, w| w.smp2().cycles181_5());

    // PA2 (motor V+) - if enabled
    #[cfg(feature = "voltage-sense-motor")]
    adc1.smpr1.modify(|_, w| w.smp3().cycles181_5());

    // PA3 (motor V-) - if enabled
    #[cfg(feature = "voltage-sense-motor")]
    adc1.smpr1.modify(|_, w| w.smp4().cycles181_5());

    // PA4 (motor temp) - if enabled
    #[cfg(feature = "temp-sense-motor")]
    adc1.smpr1.modify(|_, w| w.smp5().cycles181_5());

    // Internal channels
    adc1.smpr2.modify(|_, w| {
        let w = w.smp18().cycles601_5(); // VREFINT (ch 18) - 4µs minimum required
        #[cfg(feature = "temp-sense-mcu")]
        let w = w.smp16().cycles181_5(); // MCU Temperature sensor (ch 16)
        w
    });

    /*
     * ENABLE ADC
     */
    adc1.cr.modify(|_, w| w.aden().enabled());
    while adc1.isr.read().adrdy().is_not_ready() {}
}
