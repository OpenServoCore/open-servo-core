//! Bench-only PFIC HIGH load probe (`--features bench`): per transport
//! vector, entries and busy SysTick ticks (sum and max), and the break
//! wake's outcomes. Counters live in a `no_mangle` static so the debug link
//! dumps them by symbol address (`nm` the ELF); call sites stay
//! unconditional and compile to nothing without the feature.

use crate::hal::systick;

#[repr(C)]
pub struct VectorLoad {
    pub entries: u32,
    /// SysTick ticks (HCLK) spent in the body, entry to exit.
    pub busy: u32,
    pub max: u32,
}

impl VectorLoad {
    const ZERO: Self = Self {
        entries: 0,
        busy: 0,
        max: 0,
    };

    /// One body that began at `entry` (a [`stamp`]) ends now.
    #[inline(always)]
    pub fn exit(&mut self, entry: u32) {
        let d = systick::ticks().wrapping_sub(entry);
        self.entries = self.entries.wrapping_add(1);
        self.busy = self.busy.wrapping_add(d);
        self.max = self.max.max(d);
    }
}

#[repr(C)]
pub struct HighProbe {
    pub tim2: VectorLoad,
    pub usart1: VectorLoad,
    pub systick: VectorLoad,
    /// Break-wake fires that latched the line low (breaks) and high (idle).
    pub breaks: u32,
    pub idles: u32,
    /// Re-arms on the edge after a park, and on a pin already off the
    /// latched level at the fire.
    pub edge_rearms: u32,
    pub pin_rearms: u32,
    /// Fires whose level latch had not run: classified on a stale level.
    pub latch_missed: u32,
}

impl HighProbe {
    pub const ZERO: Self = Self {
        tim2: VectorLoad::ZERO,
        usart1: VectorLoad::ZERO,
        systick: VectorLoad::ZERO,
        breaks: 0,
        idles: 0,
        edge_rearms: 0,
        pin_rearms: 0,
        latch_missed: 0,
    };
}

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut HIGH_PROBE: HighProbe = HighProbe::ZERO;

/// Run `f` over the probe -- a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn high_probe(f: impl FnOnce(&mut HighProbe)) {
    // SAFETY: single-hart; every writer runs at PFIC HIGH, so accesses
    // never interleave. The debug link only reads.
    #[cfg(feature = "bench")]
    unsafe {
        f(&mut *core::ptr::addr_of_mut!(HIGH_PROBE));
    }
}

/// A body's entry stamp for [`VectorLoad::exit`]; 0 without `bench`.
#[inline(always)]
pub fn stamp() -> u32 {
    #[cfg(feature = "bench")]
    {
        systick::ticks()
    }
    #[cfg(not(feature = "bench"))]
    {
        0
    }
}
