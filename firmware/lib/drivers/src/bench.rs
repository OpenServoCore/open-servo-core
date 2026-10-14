//! Bench-only probes: counter bodies exist under the `bench` feature; call
//! sites stay unconditional in hot paths and compile to nothing without it
//! (the closure is never called and the empty inline erases).
//!
//! Counters live in `no_mangle` statics so the debug link can dump them by
//! symbol address (`nm` the ELF) on a running chip -- no wire traffic, no
//! table space, and they survive test tails that re-center transport state.

pub use osc_servo_core::budget::probe::TelStep;

use crate::traits::bus::Deadline;

#[cfg(feature = "bench")]
static mut TEL_BANKS: u32 = 0;

/// The TEL encoder banked a batch (kernel tick, the only writer).
#[inline(always)]
pub fn tel_banked() {
    // SAFETY: single writer; readers take one word.
    #[cfg(feature = "bench")]
    unsafe {
        let b = &raw mut TEL_BANKS;
        b.write_volatile(b.read_volatile().wrapping_add(1));
    }
}

/// Batches banked so far; 0 without the `bench` feature.
#[inline(always)]
pub fn tel_banks() -> u32 {
    #[cfg(feature = "bench")]
    // SAFETY: one-word volatile read.
    unsafe {
        (&raw const TEL_BANKS).read_volatile()
    }
    #[cfg(not(feature = "bench"))]
    0
}

#[cfg(feature = "bench-telcost")]
#[unsafe(no_mangle)]
pub static mut TEL_COST: osc_servo_core::budget::probe::TelCost =
    osc_servo_core::budget::probe::TelCost::ZERO;

/// A TEL step's opening stamp on `clock`; 0 without `bench-telcost`.
#[inline(always)]
#[allow(unused_variables)]
pub fn tel_stamp(clock: &impl Deadline) -> u32 {
    #[cfg(feature = "bench-telcost")]
    {
        clock.now()
    }
    #[cfg(not(feature = "bench-telcost"))]
    0
}

/// Step `s`, stamped at `from`, ends now.
#[inline(always)]
#[allow(unused_variables)]
pub fn tel_step(clock: &impl Deadline, from: u32, s: TelStep) {
    // SAFETY: the kernel tick is the record's only writer, bringup aside.
    #[cfg(feature = "bench-telcost")]
    unsafe {
        let wall = clock.now().wrapping_sub(from);
        (*core::ptr::addr_of_mut!(TEL_COST)).add(s, wall);
    }
}

/// Price a back-to-back stamp pair. Call once, before interrupts run.
#[inline(always)]
#[allow(unused_variables)]
pub fn tel_calibrate(clock: &impl Deadline) {
    // SAFETY: pre-IRQ bringup, ahead of the record's only other writer.
    #[cfg(feature = "bench-telcost")]
    unsafe {
        let a = clock.now();
        let b = clock.now();
        let cal = &raw mut TEL_COST.cal;
        cal.write_volatile(b.wrapping_sub(a).min(u16::MAX.into()) as u16);
    }
}

/// Arms picked up so far; 0 without `bench-telcost`.
#[inline(always)]
pub fn tel_pickups() -> u32 {
    #[cfg(feature = "bench-telcost")]
    // SAFETY: one-word volatile read.
    unsafe {
        (&raw const TEL_COST.n[TelStep::Pickup as usize]).read_volatile()
    }
    #[cfg(not(feature = "bench-telcost"))]
    0
}

/// The burst picked up last stamps its steps: every other one does, the
/// rest run unstamped for the kernel's TEL cost.
#[inline(always)]
pub fn tel_stamping() -> bool {
    tel_pickups() & 1 != 0
}

/// The running kernel tick's steps, taken: the tick's end calls it.
#[inline(always)]
pub fn tel_tick_steps() -> u32 {
    #[cfg(feature = "bench-telcost")]
    // SAFETY: the kernel tick's end, the record's only writer's context.
    unsafe {
        let t = &raw mut TEL_COST.tick;
        let steps = t.read();
        t.write(0);
        steps
    }
    #[cfg(not(feature = "bench-telcost"))]
    0
}
