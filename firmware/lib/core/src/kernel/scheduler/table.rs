//! The schedule: the tick rate, the decimations, the medium phases, the
//! tick-load close, and the phase table (`Medium::step`).

use crate::Shared;
use crate::kernel::KernelConfig;
use crate::kernel::phases::Medium;
use crate::kernel::phases::fast::{Command, Fast};
use crate::kernel::phases::loops::faults::FaultLatch;

/// Kernel tick rate, one ADC scan per PWM period (the chip asserts it).
pub const TICK_HZ: u32 = 20_000;

/// FAST -> MEDIUM decimation: MED_HZ = tick_hz / DECIM_MED (2 kHz at 20 kHz).
pub const DECIM_MED: u8 = 10;
const _: () = assert!(
    phase::PUBLISH < phase::SPARE_A && phase::SPARE_A < phase::SPEED && phase::SPEED < DECIM_MED
);
/// MEDIUM -> SLOW decimation: SLOW_HZ = MED_HZ / DECIM_SLOW (62.5 Hz).
pub const DECIM_SLOW: u8 = 32;

/// The stages of the medium chain, in data-flow order.
pub mod phase {
    /// Configuration, CONTROL, the edges and the drive kind.
    pub const CONTROL: u8 = 0;
    /// Observer step, speed blend on the boxcar SPEED closed.
    pub const OBSERVER: u8 = 1;
    /// Trajectory and position loop; the hold updates the drive kind.
    pub const TRAJECTORY: u8 = 2;
    /// Limits fold, OpenLoop base duty, stall fault.
    pub const LIMITS: u8 = 3;
    /// Velocity loop or the current clamp; reference and band land.
    pub const VELOCITY: u8 = 4;
    /// Rail estimate, sensor screen, position error timer.
    pub const RAIL: u8 = 5;
    /// The slow block, every DECIM_SLOW periods, and its finish one period
    /// later, with the shunt zero's aging step.
    pub const SLOW: u8 = 6;
    /// The shunt zero's sample; estimates, mode, limits and sensors publish.
    pub const PUBLISH: u8 = 7;
    /// The configuration take after a commit.
    pub const SPARE_A: u8 = 8;
    /// Back-EMF close, on the half this tick's sample ends: the boxcar the
    /// next period's OBSERVER reads.
    pub const SPEED: u8 = 9;
}

/// The phases the kernel holds the TX lane on while a burst streams,
/// beside a SLOW stage's tick: the two heaviest plain ticks carry their row
/// and a head or a bank, never a lane step. `Medium::step` returns the
/// hold; the budget and the DES read this set.
pub const LANE_HELD: u32 = 1 << phase::OBSERVER | 1 << phase::TRAJECTORY;

/// Whether the kernel holds the TX lane on `phase` ([`LANE_HELD`]), a
/// phase under [`DECIM_MED`].
pub const fn lane_holds(phase: usize) -> bool {
    LANE_HELD >> phase & 1 != 0
}

/// The phases a burst may start on: the two plain ticks with the room for
/// the arm's pickup beside a lane step. Never the take's phase.
pub const BURST_START: u32 = 1 << phase::CONTROL | 1 << phase::SPEED;
const _: () = assert!(BURST_START & (LANE_HELD | 1 << phase::SPARE_A | 1 << phase::SLOW) == 0);

/// Whether a burst may start on `phase` ([`BURST_START`]), a phase under
/// [`DECIM_MED`]: one shift, no divide, on the tick a start is asked.
pub const fn burst_starts(phase: usize) -> bool {
    BURST_START >> phase & 1 != 0
}

/// Ticks between two tick-load closes.
pub const HEALTH_CLOSE_TICKS: u32 = 2 * DECIM_MED as u32;

/// The phase every tick-load close lands on: the lightest Closed phase that
/// carries no SLOW stage and no configuration take.
pub const HEALTH_CLOSE_PHASE: u8 = phase::PUBLISH;

/// The most lost ticks one tick-load close counts: a longer hold-off, such
/// as a debug halt, counts exactly this many and resyncs.
pub const HEALTH_LOST_MAX: u16 = 16;

const _: () = assert!(HEALTH_CLOSE_TICKS.is_multiple_of(DECIM_MED as u32));

/// Whether executed tick `tick` closes a tick-load window. The chip counts
/// executed ticks from the boot tick, which runs phase CONTROL, and a shunt
/// burst runs no tick and keeps the count, so tick `tick` runs phase
/// `tick` modulo [`DECIM_MED`]; the first close only sets the mark.
pub const fn health_close_at(tick: u64) -> bool {
    tick % HEALTH_CLOSE_TICKS as u64 == HEALTH_CLOSE_PHASE as u64
}

/// Whether a tick-load close can land on `phase` ([`health_close_at`]).
pub const fn health_close_lands(phase: usize) -> bool {
    phase % DECIM_MED as usize == HEALTH_CLOSE_PHASE as usize
}

impl Medium {
    /// Phase `k` of the medium chain (`phase`), on this tick's sample;
    /// whether the TX lane holds: a SLOW stage ran, or `k` is in
    /// [`LANE_HELD`]. Out of line: inlined, the phases sit
    /// between the fast half and its cold arms and carry every one of
    /// those branches out of reach (v006-performance sec 3). `streaming`: a
    /// TEL burst streams this tick, which holds the dead-man at CONTROL.
    #[allow(clippy::too_many_arguments)]
    #[inline(never)]
    #[rustc_align(4)]
    pub fn step(
        &mut self,
        k: u8,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
        #[cfg(not(feature = "experiment"))] streaming: bool,
    ) -> bool {
        match k {
            #[cfg(not(feature = "experiment"))]
            phase::CONTROL => self.control(
                shared.frame().pos(),
                cfg,
                shared,
                faults,
                fast,
                cmd,
                streaming,
            ),
            #[cfg(feature = "experiment")]
            phase::CONTROL => self.control(cfg, shared, faults, fast, cmd),
            #[cfg(not(feature = "experiment"))]
            phase::OBSERVER => {
                self.observe(shared.frame().pos(), cfg, shared, fast);
                return true;
            }
            // `Kernel::on_tick` takes this phase straight, not through here
            #[cfg(not(feature = "experiment"))]
            phase::TRAJECTORY => {
                return self.trajectory(cfg, faults, cmd, shared.frame().pos());
            }
            #[cfg(not(feature = "experiment"))]
            phase::LIMITS => self.limit(cfg, shared, faults, fast),
            #[cfg(not(feature = "experiment"))]
            phase::VELOCITY => self.velocity(cfg, cmd),
            #[cfg(not(feature = "experiment"))]
            phase::RAIL => self.rail(shared, cfg, faults, fast, cmd),
            #[cfg(not(feature = "experiment"))]
            phase::SLOW => {
                return self.slow(shared.frame().ntc_raw(), cfg, shared, faults, fast, cmd);
            }
            #[cfg(feature = "experiment")]
            phase::SLOW => self.slow(fast),
            #[cfg(feature = "experiment")]
            phase::OBSERVER | phase::TRAJECTORY => return true,
            phase::PUBLISH => {
                fast.track_bias(shared);
                self.publish(shared, faults, fast)
            }
            #[cfg(not(feature = "experiment"))]
            phase::SPEED => self.speed(cfg, fast),
            _ => {}
        }
        false
    }
}
