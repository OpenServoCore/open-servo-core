use cortex_m::asm;
use stm32f3::stm32f301 as pac;
use open_servo_hw_utils::adc_dma;

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
        .write(|w| unsafe { w.bits(adc_dma::dma_target_ptr() as u32) }); // set memory address for DMA

    // set number of data to transfer to 5 because we are converting 5 channels
    p.DMA1.ch1.ndtr.write(|w| w.ndt().bits(5));

    // enable DMA1 Channel 1
    p.DMA1.ch1.cr.modify(|_, w| w.en().enabled());
}

pub fn init_adc(p: &pac::Peripherals) {
    let adc1 = &p.ADC1;

    /*
     * Configure ADC
     */
    p.ADC1_2.ccr.modify(|_, w| {
        w.ckmode()
            .sync_div1() // use synchronous clock mode
            .tsen()
            .enabled() // enable temperature sensor
            .vrefen()
            .enabled() // enable VREFINT
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
    // configure PA0, PA1, PA2 as analog input
    let gpioa = &p.GPIOA;
    gpioa
        .moder
        .modify(|_, w| w.moder0().analog().moder1().analog().moder2().analog());
    gpioa.pupdr.modify(|_, w| {
        w.pupdr0()
            .floating()
            .pupdr1()
            .floating()
            .pupdr2()
            .floating()
    });

    // configure channels for conversion
    adc1.sqr1.modify(|_, w| unsafe {
        w.l()
            .bits(4) // set number of conversions to 5, this register counts from 0
            .sq1()
            .bits(18) // set vrefint ( ch 18 ) as first conversion
            .sq2()
            .bits(1) // set PA0 ( ch 1 )  as second conversion
            .sq3()
            .bits(2) // set PA1 ( ch 2 )  as third conversion
            .sq4()
            .bits(3) // set PA2 ( ch 3 ) as fourth conversion
    });

    adc1.sqr2.modify(|_, w| unsafe {
        w.sq5().bits(16) // set tempature sensor ( ch 16 ) as first conversion
    });

    // set sample time for channels
    adc1.smpr1.modify(|_, w| {
        w.smp1()
            .cycles181_5() // PA0
            .smp2()
            .cycles181_5() // PA1
            .smp3()
            .cycles181_5() // PA2
    });

    adc1.smpr2.modify(|_, w| {
        w.smp16()
            .cycles181_5() // VREFINT
            .smp18()
            .cycles181_5() // Temperature sensor
    });

    /*
     * ENABLE ADC
     */
    adc1.cr.modify(|_, w| w.aden().enabled());
    while adc1.isr.read().adrdy().is_not_ready() {}
}
