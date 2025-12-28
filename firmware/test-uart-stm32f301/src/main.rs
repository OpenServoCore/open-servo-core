#![no_std]
#![no_main]

//! STM32F411 single-wire UART ping-pong test (MASTER).
//! RX and TX via DMA.

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rtt_init, ChannelMode};
use stm32f4::stm32f411::{interrupt, DMA2, FLASH, GPIOA, RCC, TIM2, USART1};

// HSE 25MHz -> PLL -> 100MHz SYSCLK
const PCLK2_HZ: u32 = 100_000_000;
const BAUD_RATE: u32 = 115_200;
const RX_BUF_SIZE: usize = 64;

// DMA TX buffer
static mut TX_BUF: [u8; 16] = [0; 16];

// DMA RX buffer
static mut RX_BUF: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];

// DMA TX complete flag
static TX_COMPLETE: AtomicBool = AtomicBool::new(false);

// Snapshot of DMA RX start count
static RX_START_NDTR: AtomicUsize = AtomicUsize::new(RX_BUF_SIZE);

// Timer tick flag (1 Hz)
static TIMER_TICK: AtomicBool = AtomicBool::new(false);

defmt::timestamp!("{=u32:us}", {
    // DWT CYCCNT at 100MHz, convert to microseconds
    let cyccnt = cortex_m::peripheral::DWT::cycle_count();
    cyccnt / 100  // 100 cycles per microsecond at 100MHz
});

#[entry]
fn main() -> ! {
    // Enable DWT cycle counter for timestamps
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();

    // Enable debug during sleep modes (keeps SWD alive during WFI)
    const DBGMCU_CR: *mut u32 = 0xE004_2004 as *mut u32;
    unsafe { DBGMCU_CR.write_volatile(0x7) }; // DBG_SLEEP | DBG_STOP | DBG_STANDBY

    let channels = rtt_init! {
        up: {
            0: {
                size: 512,
                mode: ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
    };
    rtt_target::set_defmt_channel(channels.up.0);

    defmt::info!("STM32F411 single-wire MASTER");

    configure_rcc();

    // Verify clock setup
    let rcc = unsafe { &*RCC::ptr() };
    let cfgr = rcc.cfgr.read();
    defmt::info!("SWS={} (2=PLL)", cfgr.sws().bits());

    configure_gpio();
    configure_dma();
    configure_usart();
    configure_timer();

    // Show USART register values
    let usart = unsafe { &*USART1::ptr() };
    defmt::info!("BRR={} (expect 868 for 100MHz/115200)", usart.brr.read().bits());
    defmt::info!("CR1=0x{:04x} CR2=0x{:04x} CR3=0x{:04x}",
        usart.cr1.read().bits(),
        usart.cr2.read().bits(),
        usart.cr3.read().bits());

    configure_nvic();
    start_usart();

    defmt::info!("Mode: single-wire HDSEL on PA9 (open-drain)");
    run_master();
}

fn configure_rcc() {
    let rcc = unsafe { &*RCC::ptr() };
    let flash = unsafe { &*FLASH::ptr() };

    // Enable HSE (25MHz crystal)
    rcc.cr.modify(|_, w| w.hseon().set_bit());
    while rcc.cr.read().hserdy().bit_is_clear() {}

    // Configure PLL: HSE 25MHz / 25 * 200 / 2 = 100MHz
    rcc.pllcfgr.modify(|_, w| unsafe {
        w.pllsrc().hse();
        w.pllm().bits(25);   // VCO input = 25/25 = 1MHz
        w.plln().bits(200);  // VCO = 1 * 200 = 200MHz
        w.pllp().div2();     // SYSCLK = 200/2 = 100MHz
        w.pllq().bits(4)     // USB = 200/4 = 50MHz (not used)
    });

    // Enable PLL
    rcc.cr.modify(|_, w| w.pllon().set_bit());
    while rcc.cr.read().pllrdy().bit_is_clear() {}

    // Set flash latency for 100MHz (3 wait states) + enable acceleration
    flash.acr.modify(|_, w| {
        w.latency().bits(3);
        w.icen().set_bit();    // Instruction cache enable
        w.dcen().set_bit();    // Data cache enable
        w.prften().set_bit()   // Prefetch enable
    });

    // Configure prescalers
    rcc.cfgr.modify(|_, w| {
        w.hpre().div1();   // AHB = 100MHz
        w.ppre1().div2();  // APB1 = 50MHz (max)
        w.ppre2().div1()   // APB2 = 100MHz
    });

    // Switch to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // Enable peripherals
    rcc.ahb1enr.modify(|_, w| {
        w.gpioaen().set_bit();
        w.dma2en().set_bit()
    });
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
    rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());
}

fn configure_gpio() {
    let gpioa = unsafe { &*GPIOA::ptr() };

    // PA9: AF7 open-drain for USART1_TX (HDSEL requires open-drain + pull-up)
    gpioa.moder.modify(|_, w| w.moder9().alternate());
    gpioa.afrh.modify(|_, w| w.afrh9().af7());
    gpioa.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
    gpioa.otyper.modify(|_, w| w.ot9().open_drain());
    gpioa.pupdr.modify(|_, w| w.pupdr9().pull_up());  // Internal pull-up
}

fn configure_dma() {
    let dma = unsafe { &*DMA2::ptr() };

    // Disable TX and RX streams
    dma.st[7].cr.modify(|_, w| w.en().clear_bit());
    dma.st[2].cr.modify(|_, w| w.en().clear_bit());
    while dma.st[7].cr.read().en().bit_is_set() {}
    while dma.st[2].cr.read().en().bit_is_set() {}

    // Clear all flags
    dma.hifcr.write(|w| unsafe { w.bits(0x0F40_0000) }); // Stream 7
    dma.lifcr.write(|w| unsafe { w.bits(0x003D_0000) }); // Stream 2

    let usart_dr = 0x4001_1004 as u32;

    // TX Stream 7: Memory -> USART1_DR, with transfer complete interrupt
    dma.st[7].par.write(|w| unsafe { w.bits(usart_dr) });
    dma.st[7].cr.modify(|_, w| unsafe {
        w.chsel().bits(4);     // Channel 4
        w.dir().bits(0b01);    // Memory to peripheral
        w.minc().set_bit();    // Memory increment
        w.pinc().clear_bit();  // Peripheral fixed
        w.msize().bits(0b00);  // Byte
        w.psize().bits(0b00);  // Byte
        w.circ().clear_bit();  // No circular
        w.pl().bits(0b01);     // Medium priority
        w.tcie().set_bit()     // Transfer complete interrupt
    });

    // RX Stream 2: USART1_DR -> Memory
    dma.st[2].par.write(|w| unsafe { w.bits(usart_dr) });
    dma.st[2].m0ar.write(|w| unsafe { w.bits(RX_BUF.as_ptr() as u32) });
    dma.st[2].ndtr.write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });
    dma.st[2].cr.modify(|_, w| unsafe {
        w.chsel().bits(4);     // Channel 4
        w.dir().bits(0b00);    // Peripheral to memory
        w.minc().set_bit();    // Memory increment
        w.pinc().clear_bit();  // Peripheral fixed
        w.msize().bits(0b00);  // Byte
        w.psize().bits(0b00);  // Byte
        w.circ().clear_bit();  // No circular (will reset manually)
        w.pl().bits(0b01);     // Medium priority
        w.tcie().clear_bit()   // No interrupt (poll NDTR)
    });

    // Enable RX DMA stream
    dma.st[2].cr.modify(|_, w| w.en().set_bit());
}

fn configure_usart() {
    let usart = unsafe { &*USART1::ptr() };

    usart.cr1.modify(|_, w| w.ue().clear_bit());

    let brr = PCLK2_HZ / BAUD_RATE;
    usart.brr.write(|w| unsafe { w.bits(brr) });

    usart.cr1.modify(|_, w| {
        w.m().clear_bit();     // 8 data bits
        w.pce().clear_bit();   // No parity
        w.over8().clear_bit()  // 16x oversampling
    });
    usart.cr2.modify(|_, w| {
        w.stop().bits(0b00);    // 1 stop bit
        w.linen().clear_bit();  // Required for HDSEL
        w.clken().clear_bit()   // Required for HDSEL
    });

    usart.cr1.modify(|_, w| {
        w.te().set_bit();      // Transmitter enable
        w.re().set_bit();      // Receiver enable
        w.rxneie().clear_bit() // No RX interrupt (using DMA)
    });

    usart.cr3.modify(|_, w| {
        w.hdsel().set_bit();   // Half-duplex mode
        w.dmat().set_bit();    // DMA transmit enable
        w.dmar().set_bit();    // DMA receive enable
        w.iren().clear_bit();
        w.scen().clear_bit()
    });
}

fn configure_timer() {
    let tim = unsafe { &*TIM2::ptr() };

    // TIM2 clock = APB1 * 2 = 50MHz * 2 = 100MHz (when APB1 prescaler > 1)
    // PSC = 9999 -> 100MHz / 10000 = 10kHz
    // ARR = 9999 -> 10kHz / 10000 = 1Hz
    tim.psc.write(|w| w.psc().bits(9999));
    tim.arr.write(|w| w.arr().bits(9999));
    tim.dier.modify(|_, w| w.uie().set_bit());  // Update interrupt enable
    tim.cr1.modify(|_, w| w.cen().set_bit());   // Counter enable
}

fn configure_nvic() {
    unsafe {
        // TIM2 (1 Hz tick)
        NVIC::unmask(interrupt::TIM2);
        // DMA2 Stream 7 (TX complete)
        NVIC::unmask(interrupt::DMA2_STREAM7);
        // Note: DMA RX doesn't need interrupt - we poll NDTR
    }
}

fn start_usart() {
    let usart = unsafe { &*USART1::ptr() };
    usart.cr1.modify(|_, w| w.ue().set_bit());
}

// TIM2 interrupt (1 Hz tick)
#[interrupt]
fn TIM2() {
    let tim = unsafe { &*stm32f4::stm32f411::TIM2::ptr() };
    tim.sr.modify(|_, w| w.uif().clear_bit());  // Clear update flag
    TIMER_TICK.store(true, Ordering::SeqCst);
}

// DMA2 Stream 7 interrupt (TX complete)
#[interrupt]
fn DMA2_STREAM7() {
    let dma = unsafe { &*DMA2::ptr() };
    dma.hifcr.write(|w| unsafe { w.bits(0x0F40_0000) });
    TX_COMPLETE.store(true, Ordering::SeqCst);
}


fn dma_tx_start(data: &[u8]) {
    let dma = unsafe { &*DMA2::ptr() };

    let len = data.len().min(16);
    unsafe {
        TX_BUF[..len].copy_from_slice(&data[..len]);
    }

    TX_COMPLETE.store(false, Ordering::SeqCst);

    dma.st[7].cr.modify(|_, w| w.en().clear_bit());
    while dma.st[7].cr.read().en().bit_is_set() {}

    dma.hifcr.write(|w| unsafe { w.bits(0x0F40_0000) });

    dma.st[7].m0ar.write(|w| unsafe { w.bits(TX_BUF.as_ptr() as u32) });
    dma.st[7].ndtr.write(|w| unsafe { w.bits(len as u32) });

    dma.st[7].cr.modify(|_, w| w.en().set_bit());
}

fn dma_tx_wait() {
    while !TX_COMPLETE.load(Ordering::SeqCst) {
        cortex_m::asm::wfi();
    }
    // Also wait for USART TC (transmission complete)
    let usart = unsafe { &*USART1::ptr() };
    while usart.sr.read().tc().bit_is_clear() {}
}

fn rx_reset() {
    let dma = unsafe { &*DMA2::ptr() };

    // Disable RX DMA stream
    dma.st[2].cr.modify(|_, w| w.en().clear_bit());
    while dma.st[2].cr.read().en().bit_is_set() {}

    // Clear RX buffer
    unsafe {
        for b in RX_BUF.iter_mut() {
            *b = 0;
        }
    }

    // Reset DMA counter and memory address
    dma.st[2].m0ar.write(|w| unsafe { w.bits(RX_BUF.as_ptr() as u32) });
    dma.st[2].ndtr.write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

    // Clear any pending flags
    dma.lifcr.write(|w| unsafe { w.bits(0x003D_0000) }); // Stream 2

    // Store starting NDTR
    RX_START_NDTR.store(RX_BUF_SIZE, Ordering::Release);

    // Re-enable RX DMA stream
    dma.st[2].cr.modify(|_, w| w.en().set_bit());
}

fn rx_get_count() -> usize {
    let dma = unsafe { &*DMA2::ptr() };
    let current_ndtr = dma.st[2].ndtr.read().bits() as usize;
    let start_ndtr = RX_START_NDTR.load(Ordering::Acquire);
    start_ndtr.saturating_sub(current_ndtr)
}

fn rx_get_data(buf: &mut [u8]) -> usize {
    let count = rx_get_count().min(buf.len());
    for i in 0..count {
        buf[i] = unsafe { RX_BUF[i] };
    }
    count
}

fn run_master() -> ! {
    let ping = b"ping\n";
    let echo_len = ping.len(); // 5 bytes - we'll strip this as self-echo
    let mut cycle: u32 = 0;
    let mut rx_buf = [0u8; 32];

    loop {
        defmt::info!("--- Cycle {} ---", cycle);

        // Clear RX buffer
        rx_reset();

        // Send ping via DMA
        dma_tx_start(ping);
        dma_tx_wait();
        defmt::info!("TX: ping");

        // Wait for response (echo + pong) with timeout
        // We expect: 5 bytes self-echo ("ping\n") + 5 bytes response ("pong\n")
        let tim = unsafe { &*TIM2::ptr() };
        let start = tim.cnt.read().cnt().bits();
        let mut got_response = false;

        while tim.cnt.read().cnt().bits().wrapping_sub(start) < 10000 { // 1 second timeout
            let count = rx_get_count();
            // Need more than echo_len bytes AND ending with newline
            if count > echo_len {
                let total = rx_get_data(&mut rx_buf);
                if total > 0 && rx_buf[total - 1] == b'\n' {
                    // Strip self-echo, show response
                    defmt::info!("RX: {:a}", &rx_buf[echo_len..total]);
                    got_response = true;
                    break;
                }
            }
        }

        if !got_response {
            // Check what we got
            let count = rx_get_data(&mut rx_buf);
            if count > 0 {
                defmt::warn!("Timeout. RX buffer: {:a}", &rx_buf[..count]);
            } else {
                defmt::warn!("Timeout. No data received.");
            }
        }

        // Wait 1 second before next ping (using TIM2 flag)
        TIMER_TICK.store(false, Ordering::SeqCst);
        while !TIMER_TICK.load(Ordering::SeqCst) {
            cortex_m::asm::wfi();
        }

        cycle = cycle.wrapping_add(1);
    }
}
