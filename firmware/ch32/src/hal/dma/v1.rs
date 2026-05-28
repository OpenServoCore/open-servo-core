use ch32_metapac::DMA1;

pub use ch32_metapac::dma::vals::{Dir, Size};

#[derive(Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    CH5 = 5,
    CH6 = 6,
    CH7 = 7,
}

#[derive(Copy, Clone)]
pub struct Config {
    pub dir: Dir,
    pub circ: bool,
    pub pinc: bool,
    pub minc: bool,
    /// Applied to both peripheral and memory sides.
    pub size: Size,
    pub tcie: bool,
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
        w.set_pinc(cfg.pinc);
        w.set_minc(cfg.minc);
        w.set_psize(cfg.size);
        w.set_msize(cfg.size);
        w.set_tcie(cfg.tcie);
    });
}

/// Channel must be disabled when called.
pub fn set_count(ch: Channel, count: u16) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).ndtr().write(|w| w.set_ndt(count));
}

#[inline]
pub fn enable(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).cr().modify(|w| w.set_en(true));
}

pub fn disable(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).cr().modify(|w| w.set_en(false));
}

pub fn clear_tc_flag(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ifcr().write(|w| w.set_tcif(n, true));
}

#[inline]
pub fn remaining(ch: Channel) -> u16 {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).ndtr().read().ndt()
}

pub fn is_enabled(ch: Channel) -> bool {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).cr().read().en()
}
