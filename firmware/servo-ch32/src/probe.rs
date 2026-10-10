//! Bench-only bus load probe (`--features bench`): per transport
//! vector, entries and busy SysTick ticks (sum and max), and the break
//! wake's outcomes. Counters live in a `no_mangle` static so the debug link
//! dumps them by symbol address (`nm` the ELF); call sites stay
//! unconditional and compile to nothing without the feature. The symbol
//! keeps its `HIGH_PROBE` name for the bench dump tools.

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
    /// Break-wake overflows: breaks, and a parked low's re-fires.
    pub breaks: u32,
    pub refires: u32,
    /// Overflows that found the line still low, and of those, the ones whose
    /// rising edge landed between the pin read and the park.
    pub parks: u32,
    pub rises: u32,
}

impl HighProbe {
    pub const ZERO: Self = Self {
        tim2: VectorLoad::ZERO,
        usart1: VectorLoad::ZERO,
        systick: VectorLoad::ZERO,
        breaks: 0,
        refires: 0,
        parks: 0,
        rises: 0,
    };
}

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut HIGH_PROBE: HighProbe = HighProbe::ZERO;

/// Run `f` over the probe -- a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn high_probe(f: impl FnOnce(&mut HighProbe)) {
    // SAFETY: single-hart; every writer runs at the bus level, so accesses
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
