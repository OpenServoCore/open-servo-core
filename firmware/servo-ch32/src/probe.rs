//! Bench-only budget probe (`--features bench`): every kernel tick and every
//! bus vector body, in HCLK ticks, into the `budget::probe` records the
//! hardware test `budgets` dumps by symbol (`nm` the ELF) and holds against
//! the budget table. Call sites stay unconditional and compile to nothing
//! without the feature. The `bench-telcost` image trades the bus record for
//! the TEL tallies beside the drivers' TEL step record: there the bus
//! probes compile to nothing. `bench-window` adds the window record.

use osc_servo_core::budget::probe::BusProbe;
#[cfg(feature = "bench")]
use osc_servo_core::budget::probe::KernelProbe;
#[cfg(feature = "bench-telcost")]
use osc_servo_core::budget::probe::TelTally;
#[cfg(feature = "bench-window")]
use osc_servo_core::budget::probe::WindowProbe;
use osc_servo_core::budget::{self, Body};
use osc_servo_core::kernel::TICK_HZ;

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

#[cfg(feature = "bench-window")]
#[unsafe(no_mangle)]
pub static mut WINDOW_PROBE: WindowProbe = WindowProbe::ZERO;

#[cfg(feature = "bench-telcost")]
#[unsafe(no_mangle)]
pub static mut TEL_TALLY: TelTally = TelTally::ZERO;

/// TEL arms picked up as of the last tick's end, mod 2^8 (one a tick at
/// most).
#[cfg(feature = "bench-telcost")]
static mut TEL_PICKUPS: u8 = 0;

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

/// The kernel tick that entered at `entry` reached its exit stamp now;
/// `health`: the tick-load counters closed a window. The record is updated
/// after the exit stamp, so none of it lands in the tick's body.
#[inline(always)]
#[allow(unused_variables)]
pub fn kernel_exit(entry: u32, health: bool) {
    #[cfg(feature = "bench")]
    {
        let exit = systick::ticks();
        // SAFETY: the kernel vector, after the tick: the kernel's only
        // writer, which never preempts itself.
        let kernel = unsafe { (*crate::runtime::statics::KERNEL.get()).assume_init_ref() };
        let tel = crate::runtime::statics::TEL_CHANNEL.active();
        let banks = osc_servo_drivers::bench::tel_banks();
        // SAFETY: the kernel vector is this record's only writer and never
        // preempts itself.
        let k = unsafe { &mut *core::ptr::addr_of_mut!(KERNEL_PROBE) };
        let body = exit.wrapping_sub(entry);
        #[cfg(feature = "bench-telcost")]
        {
            use osc_servo_drivers::bench as tel;
            let (ev, r, phase, boot) = k.events(kernel.probe_tags(), tel, banks, health);
            let pickups = tel::tel_pickups() as u8;
            // SAFETY: as KERNEL_PROBE; pickups happen only inside a tick.
            let first = pickups != unsafe { core::ptr::replace(&raw mut TEL_PICKUPS, pickups) };
            let steps = tel::tel_tick_steps();
            let steps = tel::tel_stamping().then_some(steps);
            let hook = tel::tel_tick_hook();
            // SAFETY: as KERNEL_PROBE.
            let t = unsafe { &mut *core::ptr::addr_of_mut!(TEL_TALLY) };
            t.tick(body, phase, r, ev, first, steps, hook, boot);
        }
        let booked = k.tick(body, || kernel.probe_tags(), tel, banks, health);
        #[cfg(feature = "bench-window")]
        // SAFETY: as KERNEL_PROBE.
        let w = unsafe { &mut *core::ptr::addr_of_mut!(WINDOW_PROBE) };
        #[cfg(feature = "bench-window")]
        w.tick(entry, body, booked);
        // The update itself runs inside any bus body this tick preempted.
        let end = systick::ticks();
        k.ended(entry, end);
        #[cfg(feature = "bench-window")]
        w.ended(end);
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
