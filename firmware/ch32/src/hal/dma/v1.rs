use ch32_metapac::DMA1;

pub use ch32_metapac::dma::vals::{Dir, Pl, Size};

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
    pub htie: bool,
    pub tcie: bool,
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
        w.set_pinc(cfg.pinc);
        w.set_minc(cfg.minc);
        w.set_psize(cfg.size);
        w.set_msize(cfg.size);
        w.set_htie(cfg.htie);
        w.set_tcie(cfg.tcie);
        w.set_pl(cfg.pl);
    });
}

/// Memory-to-memory byte copy: `src` → `dst`, `count` bytes, started
/// immediately (no peripheral request — MEM2MEM free-runs at bus speed).
/// PAR is the source (DIR = from-peripheral), both sides increment.
pub fn configure_m2m(ch: Channel, src: u32, dst: u32, count: u16, pl: Pl) {
    let n = (ch as u8 - 1) as usize;
    let c = DMA1.ch(n);
    c.par().write_value(src);
    c.mar().write_value(dst);
    c.ndtr().write(|w| w.set_ndt(count));
    // EN in the same CR write (spike-proven shape).
    c.cr().write(|w| {
        w.set_mem2mem(true);
        w.set_dir(Dir::FROMPERIPHERAL);
        w.set_pinc(true);
        w.set_minc(true);
        w.set_psize(Size::BITS8);
        w.set_msize(Size::BITS8);
        w.set_pl(pl);
        w.set_en(true);
    });
}

/// Channel must be disabled when called.
pub fn set_count(ch: Channel, count: u16) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ch(n).ndtr().write(|w| w.set_ndt(count));
}

// SAFETY: see hal/SAFETY.md. CH(n).CR is per-channel; channels driven from
// ≥ 2 priority contexts (the DXL TX channel today, touched from MAIN and
// HIGH ISRs) need the CS to keep EN/TCIE RMW atomic. Single-context channels
// pay the CS cost but stay correct.
// `inline(always)` folds this into the TIM2 CC3 hot path so the wire-driver
// activate sequence stays inside the `.highcode` body — no standalone flash
// fetch per call.
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

#[inline(always)]
pub fn set_tcie(ch: Channel, enable: bool) {
    let n = (ch as u8 - 1) as usize;
    critical_section::with(|_| {
        DMA1.ch(n).cr().modify(|w| w.set_tcie(enable));
    });
}

#[inline(always)]
pub fn set_htie(ch: Channel, enable: bool) {
    let n = (ch as u8 - 1) as usize;
    critical_section::with(|_| {
        DMA1.ch(n).cr().modify(|w| w.set_htie(enable));
    });
}

pub fn clear_tc_flag(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ifcr().write(|w| w.set_tcif(n, true));
}

pub fn clear_ht_flag(ch: Channel) {
    let n = (ch as u8 - 1) as usize;
    DMA1.ifcr().write(|w| w.set_htif(n, true));
}

#[inline]
pub fn is_ht_flag(ch: Channel) -> bool {
    let n = (ch as u8 - 1) as usize;
    DMA1.isr().read().htif(n)
}

#[inline]
pub fn is_tc_flag(ch: Channel) -> bool {
    let n = (ch as u8 - 1) as usize;
    DMA1.isr().read().tcif(n)
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
