//! DMA1 primitives -- the four channels this crate claims: CH3 is the
//! USART3 RX ring (circular, armed once, never raises an IRQ), CH2 the
//! USART3 TX arm (one span at a time; completion is the USART TC, never a
//! channel flag), CH1/CH7 the TIM2 CH3/CH4 edge-capture rings (circular,
//! armed once, 16-bit).

use ch32_metapac::DMA1;
pub use ch32_metapac::dma::vals::{Dir, Pl, Size};

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    /// TIM2_CH3 capture ring (wire falling edges).
    CH1 = 1,
    /// USART3_TX arm.
    CH2 = 2,
    /// USART3_RX ring.
    CH3 = 3,
    /// TIM2_CH4 capture ring (wire rising edges).
    CH7 = 7,
}

#[derive(Copy, Clone)]
pub struct Config {
    pub dir: Dir,
    pub circ: bool,
    pub minc: bool,
    pub pl: Pl,
    pub size: Size,
}

pub fn configure(ch: Channel, cfg: &Config, paddr: u32, maddr: u32, count: u16) {
    let n = (ch as u8 - 1) as usize;
    let c = DMA1.ch(n);
    c.par().write_value(paddr);
    c.mar().write_value(maddr);
    c.ndtr().write(|w| w.set_ndt(count));
    c.cr().write(|w| {
        w.set_dir(cfg.dir);
        w.set_circ(cfg.circ);
        w.set_minc(cfg.minc);
        w.set_psize(cfg.size);
        w.set_msize(cfg.size);
        w.set_pl(cfg.pl);
    });
}

// SAFETY: CH2 is touched from MAIN (engine send under CS) and the release
// path; CS keeps the EN RMW atomic. CH1/CH3/CH7 are armed once at bringup.
#[inline(always)]
pub fn enable(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    critical_section::with(|_| {
        DMA1.ch(n).cr().modify(|w| w.set_en(true));
    });
}

#[inline(always)]
pub fn disable(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    critical_section::with(|_| {
        DMA1.ch(n).cr().modify(|w| w.set_en(false));
    });
}

#[inline]
pub fn remaining(ch: Channel) -> u16 {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).ndtr().read().ndt()
}
