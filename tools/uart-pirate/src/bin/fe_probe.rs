//! V203 FE-latch isolation probe (task #26 parked curiosity): does
//! STATR.FE ever latch for a break on this die when NOTHING else
//! touches the USART — no DMA, no interrupts, no LIN, no stray
//! register traffic? The pirate app couldn't answer this: its vector
//! reads STATR on every entry (arming the SR half of the SR→DR clear)
//! and its RX DMA's DATAR access completes the pair, so a latched FE
//! could be retired within nanoseconds of setting.
//!
//! Self-loop over the bench PB10↔PB11 bridge, 250 kbaud on stock
//! 8 MHz HSI, receiver always M=0. Four phases × 50 trials:
//!
//!   0: hardware SBK break (also measures V203 SBK length on a scope)
//!   1: GPIO-timed 10.0-bit low — the protocol-law shape
//!   2: GPIO-timed  9.9-bit low — a fast transmitter's law break
//!   3: GPIO-timed  5.0-bit low — sub-char garble control (no FE due)
//!
//! Per trial three words land in `FE_RESULTS`: STATR after a settle,
//! DATAR, STATR after the DR read. `FE_RESULTS[0]` turns to the magic
//! when the run completes; the host reads everything with
//! `wlink dump <nm FE_RESULTS> <len>`. No console, no USB.

#![no_std]
#![no_main]

use ch32_metapac::{GPIOB, RCC, USART3};
use panic_halt as _;

const PHASES: usize = 4;
const TRIALS: usize = 50;
const WORDS: usize = 3 + PHASES * TRIALS * 3;
const MAGIC: u32 = 0xFEB0_7A11;

#[unsafe(no_mangle)]
static mut FE_RESULTS: [u32; WORDS] = [0; WORDS];

// Core SysTick (V20x STK): CTLR bit0 STE, bit2 STCLK=HCLK.
const STK_CTLR: *mut u32 = 0xE000_F000 as *mut u32;
const STK_CNTL: *mut u32 = 0xE000_F008 as *mut u32;

const HSI_HZ: u32 = 8_000_000;
const BAUD: u32 = 250_000;
const BIT_TICKS: u32 = HSI_HZ / BAUD;

fn now() -> u32 {
    unsafe { STK_CNTL.read_volatile() }
}

fn delay_ticks(n: u32) {
    let t0 = now();
    while now().wrapping_sub(t0) < n {}
}

/// Drive PB10 low as plain GPIO for `tenth_bits`/10 bit-times, then
/// hand back to USART3 AF (TE holds mark) — a break of exact,
/// M-independent length at the M=0 receiver.
fn pulse_low_tenths(tenth_bits: u32) {
    GPIOB.bshr().write(|w| w.set_br(10, true));
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 8);
        v |= 0b0011 << 8; // GP push-pull, latched low
        w.0 = v;
    });
    delay_ticks(BIT_TICKS * tenth_bits / 10);
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 8);
        v |= 0b1011 << 8; // back to AF push-pull (USART mark)
        w.0 = v;
    });
}

#[qingke_rt::entry]
fn main() -> ! {
    unsafe { STK_CTLR.write_volatile(0b101) };
    RCC.apb2pcenr().modify(|w| w.set_iopben(true));
    RCC.apb1pcenr().modify(|w| w.set_usart3en(true));

    // PB11 input pull-up (RX), PB10 AF push-pull (TX, default map).
    GPIOB.bshr().write(|w| w.set_bs(11, true));
    GPIOB.cfghr().modify(|w| {
        let mut v = w.0;
        v &= !(0xF << 12);
        v |= 0b1000 << 12;
        v &= !(0xF << 8);
        v |= 0b1011 << 8;
        w.0 = v;
    });

    USART3.ctlr1().modify(|w| {
        w.set_te(true);
        w.set_re(true);
    });
    USART3.brr().write(|w| w.0 = HSI_HZ / BAUD);
    USART3.ctlr1().modify(|w| w.set_ue(true));

    delay_ticks(BIT_TICKS * 40);

    let base = &raw mut FE_RESULTS as *mut u32;
    let mut idx = 3usize;
    for phase in 0..PHASES {
        for _ in 0..TRIALS {
            // Fresh LBD verdict per trial (rc_w0 clear, the app's
            // clear_lbd_only pattern) — the v2 question: does the break
            // DETECTOR run with LINEN=0, and down to what break length?
            USART3.statr().write(|w| {
                w.0 = u32::MAX;
                w.set_lbd(false);
            });
            match phase {
                0 => {
                    USART3.ctlr1().modify(|w| w.set_sbk(true));
                    delay_ticks(BIT_TICKS * 20);
                }
                1 => pulse_low_tenths(100),
                2 => pulse_low_tenths(99),
                _ => pulse_low_tenths(50),
            }
            // Settle ≥ 3 chars past the event before the one snapshot.
            delay_ticks(BIT_TICKS * 30);
            let s1 = USART3.statr().read().0;
            let dr = USART3.datar().read().0;
            let s2 = USART3.statr().read().0;
            unsafe {
                base.add(idx).write_volatile(s1);
                base.add(idx + 1).write_volatile(dr);
                base.add(idx + 2).write_volatile(s2);
            }
            idx += 3;
            delay_ticks(BIT_TICKS * 50);
        }
    }
    unsafe {
        base.add(1).write_volatile(PHASES as u32);
        base.add(2).write_volatile(TRIALS as u32);
        base.write_volatile(MAGIC);
    }
    loop {
        core::hint::spin_loop();
    }
}
