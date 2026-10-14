//! Bench-only budget probe (`--features bench`): every kernel tick and every
//! bus vector body, in HCLK ticks, into the two `budget::probe` records the
//! hardware test `budgets` dumps by symbol (`nm` the ELF) and holds against
//! the budget table. Call sites stay unconditional and compile to nothing
//! without the feature. The `bench-telcost` image trades the bus record for
//! the TEL tallies beside the drivers' TEL step record: there the bus
//! probes compile to nothing.

use osc_servo_core::budget::probe::BusProbe;
#[cfg(feature = "bench-telcost")]
use osc_servo_core::budget::probe::TelTally;
use osc_servo_core::budget::{self, Body};
#[cfg(feature = "bench")]
use osc_servo_core::budget::{Events, Regime, probe::KernelProbe};
use osc_servo_core::kernel::{ProbeTags, TICK_HZ};

#[cfg(feature = "bench")]
use crate::hal::systick;

const _: () = assert!(crate::hal::clocks::HCLK_HZ == budget::CLOCK_HZ);
const _: () = assert!(crate::cfg::chip::MOTOR_PWM_FREQ_HZ == TICK_HZ);

#[cfg(feature = "bench")]
#[unsafe(no_mangle)]
pub static mut KERNEL_PROBE: KernelProbe = KernelProbe::ZERO;

#[cfg(all(feature = "bench", not(feature = "bench-telcost")))]
#[unsafe(no_mangle)]
pub static mut BUS_PROBE: BusProbe = BusProbe::ZERO;

#[cfg(feature = "bench-telcost")]
#[unsafe(no_mangle)]
pub static mut TEL_TALLY: TelTally = TelTally::ZERO;

/// Run `f` over the bus record; a no-op without the `bench` feature.
#[inline(always)]
#[allow(unused_variables)]
pub fn bus_probe(f: impl FnOnce(&mut BusProbe)) {
    // SAFETY: every writer runs at the bus level, whose vectors never
    // preempt each other. The debug link only reads.
    #[cfg(all(feature = "bench", not(feature = "bench-telcost")))]
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
    #[cfg(all(feature = "bench", not(feature = "bench-telcost")))]
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
    #[cfg(any(not(feature = "bench"), feature = "bench-telcost"))]
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
    #[cfg(all(feature = "bench", not(feature = "bench-telcost")))]
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
    #[cfg(feature = "bench-telcost")]
    pickups: u32,
}

/// From `Kernel::probe_tags` ahead of `on_tick`.
#[inline(always)]
pub fn tick_before(tags: ProbeTags) -> TickTags {
    TickTags {
        tags,
        banks: osc_servo_drivers::bench::tel_banks(),
        #[cfg(feature = "bench-telcost")]
        pickups: osc_servo_drivers::bench::tel_pickups(),
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
            tel: crate::runtime::statics::TEL_CHANNEL.active(),
            bank: osc_servo_drivers::bench::tel_banks() != before.banks,
            health,
            ..Events::between(b, after)
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
        #[cfg(feature = "bench-telcost")]
        {
            use osc_servo_drivers::bench as tel;
            let first = tel::tel_pickups() != before.pickups;
            let steps = tel::tel_tick_steps();
            let steps = tel::tel_stamping().then_some(steps);
            // SAFETY: as KERNEL_PROBE.
            let t = unsafe { &mut *core::ptr::addr_of_mut!(TEL_TALLY) };
            t.tick(body, b.phase as usize, r, ev, first, steps, !b.booted);
        }
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
