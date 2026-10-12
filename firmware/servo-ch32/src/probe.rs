//! Bench-only budget probe (`--features bench`): every kernel tick and every
//! bus vector body, in HCLK ticks, into the two `budget::probe` records the
//! hardware test `budgets` dumps by symbol (`nm` the ELF) and holds against
//! the budget table. Call sites stay unconditional and compile to nothing
//! without the feature.

use osc_servo_core::budget::probe::BusProbe;
use osc_servo_core::budget::{self, Body};
#[cfg(feature = "bench")]
use osc_servo_core::budget::{Events, Regime, probe::KernelProbe};
use osc_servo_core::kernel::ProbeTags;

#[cfg(feature = "bench")]
use crate::hal::systick;

const _: () = assert!(crate::hal::clocks::HCLK_HZ == budget::CLOCK_HZ);
const _: () = assert!(crate::cfg::chip::MOTOR_PWM_FREQ_HZ == budget::TICK_HZ);

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut KERNEL_PROBE: KernelProbe = KernelProbe::ZERO;

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut BUS_PROBE: BusProbe = BusProbe::ZERO;

/// Run `f` over the bus record; a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn bus_probe(f: impl FnOnce(&mut BusProbe)) {
    // SAFETY: every writer runs at the bus level, whose vectors never
    // preempt each other. The debug link only reads.
    #[cfg(feature = "bench")]
    unsafe {
        f(&mut *core::ptr::addr_of_mut!(BUS_PROBE));
    }
}

/// SysTick and the kernel's busy total at a bus body's edge.
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub struct BusStamp {
    at: u32,
    kernel: u32,
}

/// Read with interrupts masked, so a kernel tick lands in both deltas of a
/// bus body or in neither.
#[inline(always)]
pub fn bus_entry() -> BusStamp {
    #[cfg(feature = "bench")]
    {
        critical_section::with(|_| BusStamp {
            at: systick::ticks(),
            // SAFETY: one-word volatile read of the kernel record.
            kernel: unsafe { (&raw const KERNEL_PROBE.busy).read_volatile() },
        })
    }
    #[cfg(not(feature = "bench"))]
    {
        BusStamp { at: 0, kernel: 0 }
    }
}

/// The bus body that began at `entry` ends now: its time less the kernel
/// ticks inside it.
#[inline(always)]
#[allow(unused_variables)]
pub fn bus_exit(entry: BusStamp, b: Body) {
    #[cfg(feature = "bench")]
    {
        let now = bus_entry();
        let excl = now
            .at
            .wrapping_sub(entry.at)
            .saturating_sub(now.kernel.wrapping_sub(entry.kernel));
        bus_probe(|p| p.body(excl, b));
    }
}

/// A kernel tick's tags from before its body.
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub struct TickTags {
    tags: ProbeTags,
    banks: u32,
}

/// From `Kernel::probe_tags` ahead of `on_tick`.
#[inline(always)]
pub fn tick_before(tags: ProbeTags) -> TickTags {
    TickTags {
        tags,
        banks: osc_servo_drivers::bench::tel_banks(),
    }
}

/// The kernel tick that began at `entry` ends now. `after`: the kernel's
/// tags after the body; `tel`: the TEL burst ran.
#[inline(always)]
#[allow(unused_variables)]
pub fn kernel_exit(entry: u32, before: TickTags, after: ProbeTags, tel: bool) {
    #[cfg(feature = "bench")]
    {
        let exit = systick::ticks();
        let b = before.tags;
        let ev = Events {
            refresh: after.config_gen != b.config_gen,
            tel,
            bank: osc_servo_drivers::bench::tel_banks() != before.banks,
            slow: b.slow_due,
            close: b.slow_due && !b.seated && after.seated,
        };
        let r = if b.closed || after.closed {
            Regime::Closed
        } else {
            Regime::Open
        };
        // SAFETY: the kernel vector is this record's only writer and never
        // preempts itself.
        let k = unsafe { &mut *core::ptr::addr_of_mut!(KERNEL_PROBE) };
        let body = exit.wrapping_sub(entry);
        k.tick(entry, body, b.phase as usize, r, ev, !b.booted);
        // The update itself runs inside any bus body this tick preempted.
        k.ended(exit, systick::ticks());
    }
}

/// A body on the kernel's vector that is no tick (shunt burst events).
#[inline(always)]
#[allow(unused_variables)]
pub fn kernel_other(entry: u32) {
    #[cfg(feature = "bench")]
    {
        let body = systick::ticks().wrapping_sub(entry);
        // SAFETY: as `kernel_exit`.
        unsafe { (*core::ptr::addr_of_mut!(KERNEL_PROBE)).other(body) };
    }
}
