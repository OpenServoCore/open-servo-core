//! DMA1 primitives -- only the two channels this crate claims: CH3 is the
//! USART3 RX ring (circular, armed once, never raises an IRQ), CH2 the
//! USART3 TX arm (one span at a time; completion is the USART TC, never a
//! channel flag).

use ch32_metapac::DMA1;
pub use ch32_metapac::dma::vals::{Dir, Pl, Size};

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    /// USART3_TX arm.
    CH2 = 2,
    /// USART3_RX ring.
    CH3 = 3,
}

#[derive(Copy, Clone)]
pub struct Config {
    pub dir: Dir,
    pub circ: bool,
    pub minc: bool,
    pub pl: Pl,
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
        w.set_psize(Size::BITS8);
        w.set_msize(Size::BITS8);
        w.set_pl(cfg.pl);
    });
}

// SAFETY: CH2 is touched from MAIN (engine send under CS) and the release
// path; CS keeps the EN RMW atomic. CH3 is armed once at bringup.
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
