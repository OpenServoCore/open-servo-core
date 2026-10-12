//! Bench-only budget probe (`--features bench`): every kernel tick and every
//! bus vector body, in HCLK ticks, into the two `budget::probe` records the
//! hardware test `budgets` dumps by symbol (`nm` the ELF) and holds against
//! the budget table. Call sites stay unconditional and compile to nothing
//! without the feature.

use osc_servo_core::budget::probe::BusProbe;
use osc_servo_core::budget::{self, Body};
#[cfg(feature = "bench")]
use osc_servo_core::budget::{Events, Regime, probe::KernelProbe};
#[cfg(feature = "bench")]
use osc_servo_core::estimator::thermal::stage;
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

/// SysTick and the kernel's busy total and entries at a bus body's edge.
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub struct BusStamp {
    at: u32,
    kernel: u32,
    entries: u32,
}

/// Read with interrupts masked, so a kernel tick lands in every delta of a
/// bus body or in none.
#[inline(always)]
pub fn bus_entry() -> BusStamp {
    #[cfg(feature = "bench")]
    {
        // SAFETY: one-word volatile reads of the kernel record.
        critical_section::with(|_| unsafe {
            BusStamp {
                at: systick::ticks(),
                kernel: (&raw const KERNEL_PROBE.busy).read_volatile(),
                entries: (&raw const KERNEL_PROBE.entries).read_volatile(),
            }
        })
    }
    #[cfg(not(feature = "bench"))]
    {
        BusStamp {
            at: 0,
            kernel: 0,
            entries: 0,
        }
    }
}

/// The bus body that began at `entry` ends now: its time less the kernel
/// vectors inside it, its wall time and how many it nested.
#[inline(always)]
#[allow(unused_variables)]
pub fn bus_exit(entry: BusStamp, b: Body) {
    #[cfg(feature = "bench")]
    {
        let now = bus_entry();
        let wall = now.at.wrapping_sub(entry.at);
        let excl = wall.saturating_sub(now.kernel.wrapping_sub(entry.kernel));
        let nested = now.entries.wrapping_sub(entry.entries);
        bus_probe(|p| p.body(excl, wall, nested, b));
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
/// tags after the body; `health`: the tick-load counters closed a window.
#[inline(always)]
#[allow(unused_variables)]
pub fn kernel_exit(entry: u32, before: TickTags, after: ProbeTags, health: bool) {
    #[cfg(feature = "bench")]
    {
        let exit = systick::ticks();
        let b = before.tags;
        let ev = Events {
            refresh: after.refresh_staged != b.refresh_staged,
            tel: crate::runtime::statics::TEL_CHANNEL.active(),
            bank: osc_servo_drivers::bench::tel_banks() != before.banks,
            slow: b.slow_due,
            close: b.slow_due && after.therm & stage::INSTALL != 0,
            install: (b.therm & !after.therm) & stage::INSTALL != 0,
            side: (b.therm & !after.therm) & (stage::NTC | stage::DERATE | stage::COLD) != 0,
            health,
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
