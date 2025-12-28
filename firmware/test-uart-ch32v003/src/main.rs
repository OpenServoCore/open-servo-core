#![no_std]
#![no_main]

//! CH32V003 single-wire UART ping-pong test (SLAVE).
//! RX and TX via DMA, polling-based.

use ch32v0::ch32v003::{DMA1, GPIOD, RCC, USART1};
use panic_rtt_target as _;
use portable_atomic::{AtomicUsize, Ordering};
use qingke_rt::entry;
use rtt_target::{rtt_init, ChannelMode};

const SYSCLK_HZ: u32 = 48_000_000;
const PCLK_HZ: u32 = SYSCLK_HZ;
const BAUD_RATE: u32 = 115_200;

const RX_BUF_SIZE: usize = 64;

// DMA TX buffer
static mut TX_BUF: [u8; 16] = [0; 16];

// DMA RX buffer
static mut RX_BUF: [u8; RX_BUF_SIZE] = [0; RX_BUF_SIZE];

// Snapshot of DMA RX start count (for calculating bytes received)
static RX_START_NDTR: AtomicUsize = AtomicUsize::new(RX_BUF_SIZE);

defmt::timestamp!("{=u32}", 0);

#[entry]
fn main() -> ! {
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

    defmt::info!("CH32V003 UART slave ready");

    configure_rcc();
    configure_gpio();
    configure_dma();
    configure_usart();
    start_usart();

    run_slave();
}

fn configure_rcc() {
    let rcc = unsafe { &*RCC::ptr() };

    let flash_actlr = 0x4002_2000 as *mut u32;
    unsafe { flash_actlr.write_volatile(0x01) };

    rcc.cfgr0().modify(|r, w| unsafe {
        let val = r.bits();
        let val = val & !(0b1111 << 4);
        let val = val & !(0b111 << 8);
        w.bits(val)
    });

    rcc.ctlr().modify(|r, w| unsafe {
        w.bits(r.bits() | (1 << 24))
    });

    while (rcc.ctlr().read().bits() & (1 << 25)) == 0 {}

    rcc.cfgr0().modify(|r, w| unsafe {
        let val = r.bits() & !(0b11);
        w.bits(val | 0b10)
    });

    while (rcc.cfgr0().read().bits() >> 2) & 0b11 != 0b10 {}

    rcc.apb2pcenr().modify(|r, w| unsafe {
        w.bits(r.bits() | (1 << 0) | (1 << 5) | (1 << 14))
    });
    rcc.ahbpcenr().modify(|r, w| unsafe {
        w.bits(r.bits() | (1 << 0))
    });
}

fn configure_gpio() {
    let gpiod = unsafe { &*GPIOD::ptr() };

    // PD5: AF open-drain, 50MHz for USART1_TX (HDSEL requires open-drain + pull-up)
    gpiod.cfglr().modify(|r, w| unsafe {
        let val = r.bits();
        let cleared = val & !(0xF << 20);
        w.bits(cleared | (0xF << 20))  // 0xF = AF open-drain 50MHz
    });

    // Enable internal pull-up on PD5
    gpiod.outdr().modify(|r, w| unsafe {
        w.bits(r.bits() | (1 << 5))
    });
}

fn configure_dma() {
    let dma = unsafe { &*DMA1::ptr() };

    // Disable TX and RX channels
    dma.cfgr4().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    dma.cfgr5().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });

    dma.intfcr().write(|w| unsafe { w.bits(0xFFFF_FFFF) });

    let usart_dr = 0x4001_3804 as u32;

    // TX Channel 4: Memory -> USART1_DATAR, with TC interrupt
    dma.paddr4().write(|w| unsafe { w.bits(usart_dr) });
    // Use write(0) first to ensure all bits are cleared (like we did for Channel 5)
    dma.cfgr4().write(|w| unsafe { w.bits(0) });
    dma.cfgr4().modify(|r, w| unsafe {
        let val = r.bits();
        let val = val | (1 << 4);   // DIR=1 (memory to peripheral)
        let val = val | (1 << 7);   // MINC=1
        let val = val | (1 << 12);  // PL=01 (medium)
        let val = val | (1 << 1);   // TCIE=1
        // All other bits (PINC, MSIZE, PSIZE, CIRC, HTIE, TEIE) stay 0
        w.bits(val)
    });

    // RX Channel 5: Configure AND enable, but no DMAR (Step 2b)
    dma.paddr5().write(|w| unsafe { w.bits(usart_dr) });
    dma.maddr5().write(|w| unsafe { w.bits(RX_BUF.as_ptr() as u32) });
    dma.cntr5().write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });
    // Use write(0) first to ensure all bits are cleared (including HTIE, TEIE)
    dma.cfgr5().write(|w| unsafe { w.bits(0) });
    dma.cfgr5().modify(|r, w| unsafe {
        let val = r.bits();
        // DIR=0 (peripheral to memory), MINC=1, PL=01
        // All interrupt enables (TCIE, HTIE, TEIE) stay 0
        let val = val | (1 << 7);   // MINC=1
        let val = val | (1 << 12);  // PL=01 (medium)
        w.bits(val)
    });
    // Enable DMA5 but DMAR stays 0 (USART won't trigger DMA requests)
    dma.cfgr5().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });
}

fn configure_usart() {
    let usart = unsafe { &*USART1::ptr() };

    usart.ctlr1().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 13)) });

    let brr = PCLK_HZ / BAUD_RATE;
    usart.brr().write(|w| unsafe { w.bits(brr) });

    usart.ctlr1().modify(|r, w| unsafe {
        let val = r.bits();
        let val = val & !(1 << 12);  // M=0 (8 data bits)
        let val = val & !(1 << 10);  // PCE=0 (no parity)
        let val = val | (1 << 3);    // TE=1
        let val = val | (1 << 2);    // RE=1
        let val = val & !(1 << 5);   // RXNEIE=0 (no RX interrupt, using DMA)
        w.bits(val)
    });

    usart.ctlr2().modify(|r, w| unsafe {
        w.bits(r.bits() & !(0b11 << 12))
    });

    usart.ctlr3().modify(|r, w| unsafe {
        let val = r.bits();
        let val = val | (1 << 3);   // HDSEL=1
        let val = val | (1 << 7);   // DMAT=1 (DMA TX)
        // NOTE: Do NOT set DMAR here! Must wait until after UE=1 per WCH example
        let val = val & !(1 << 1);
        let val = val & !(1 << 5);
        w.bits(val)
    });
}

fn start_usart() {
    let usart = unsafe { &*USART1::ptr() };

    // Clear any pending RX data before enabling
    let _ = usart.datar().read().bits();

    // Enable USART first (UE=1)
    usart.ctlr1().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 13)) });

    // Step 2c: Enable DMAR after UE=1 (per WCH example order)
    delay(100);
    usart.ctlr3().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 6)) }); // DMAR=1
}

fn dma_tx_start(data: &[u8]) {
    let dma = unsafe { &*DMA1::ptr() };

    let len = data.len().min(16);
    unsafe {
        TX_BUF[..len].copy_from_slice(&data[..len]);
    }

    // Disable channel, clear flags, configure, then enable
    dma.cfgr4().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    dma.intfcr().write(|w| unsafe { w.bits(0xF << 12) });

    dma.maddr4().write(|w| unsafe { w.bits(TX_BUF.as_ptr() as u32) });
    dma.cntr4().write(|w| unsafe { w.bits(len as u32) });

    dma.cfgr4().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });
}

fn dma_tx_wait() {
    let dma = unsafe { &*DMA1::ptr() };
    // Poll for DMA transfer complete (TCIF4 = bit 13)
    while (dma.intfr().read().bits() & (1 << 13)) == 0 {
        nop();
    }
    // Clear the flag
    dma.intfcr().write(|w| unsafe { w.bits(1 << 13) });

    // Wait for USART TC (transmission complete)
    let usart = unsafe { &*USART1::ptr() };
    while (usart.statr().read().bits() & (1 << 6)) == 0 {}
}

fn rx_reset() {
    let dma = unsafe { &*DMA1::ptr() };

    // Disable RX DMA channel
    dma.cfgr5().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    // Wait for DMA to actually stop (critical for avoiding memory corruption)
    while (dma.cfgr5().read().bits() & 1) != 0 {
        nop();
    }

    // Clear RX buffer
    unsafe {
        for b in RX_BUF.iter_mut() {
            *b = 0;
        }
    }

    // Reset DMA counter and memory address
    dma.maddr5().write(|w| unsafe { w.bits(RX_BUF.as_ptr() as u32) });
    dma.cntr5().write(|w| unsafe { w.bits(RX_BUF_SIZE as u32) });

    // Clear any pending flags
    dma.intfcr().write(|w| unsafe { w.bits(0xF << 16) }); // Channel 5 flags

    // Store starting NDTR
    RX_START_NDTR.store(RX_BUF_SIZE, Ordering::Release);

    // Re-enable RX DMA channel
    dma.cfgr5().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 0)) });
}

fn rx_get_count() -> usize {
    let dma = unsafe { &*DMA1::ptr() };
    let current_ndtr = dma.cntr5().read().bits() as usize;
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

fn run_slave() -> ! {
    let mut rx_buf = [0u8; 16];

    loop {
        rx_reset();

        // Wait for data (up to ~10ms timeout)
        for _ in 0..1000 {
            if rx_get_count() >= 5 {
                break;
            }
            delay(10);
        }

        // Check for "ping" and respond with "pong"
        let count = rx_get_data(&mut rx_buf);
        if count >= 4 && &rx_buf[..4] == b"ping" {
            defmt::info!("RX: ping");

            // Record RX count before TX
            let rx_before = rx_get_count();

            delay(1000); // Half-duplex turnaround
            dma_tx_start(b"pong\n");
            dma_tx_wait();

            // Small delay to let any self-echo arrive
            delay(500);

            // Check RX count after TX - did we receive our own "pong"?
            let rx_after = rx_get_count();
            if rx_after > rx_before {
                let new_bytes = rx_after - rx_before;
                defmt::warn!("SELF-ECHO detected: {} new bytes after TX", new_bytes);
            } else {
                defmt::info!("TX: pong (no self-echo)");
            }
        }
    }
}

#[inline(never)]
fn delay(cycles: u32) {
    for _ in 0..cycles {
        nop();
    }
}

#[inline(always)]
fn nop() {
    unsafe { core::arch::asm!("nop") };
}
