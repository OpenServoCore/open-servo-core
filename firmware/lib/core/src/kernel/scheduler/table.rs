//! The schedule: the tick rate, the decimations, the medium phases, the
//! tick-load close, and the phase table (`Medium::step`).

use crate::Shared;
use crate::kernel::KernelConfig;
use crate::kernel::phases::Medium;
use crate::kernel::phases::fast::{Command, Fast, Measured};
use crate::kernel::phases::loops::faults::FaultLatch;

/// Kernel tick rate, one ADC scan per PWM period (the chip asserts it).
pub const TICK_HZ: u32 = 20_000;

/// FAST -> MEDIUM decimation: MED_HZ = tick_hz / DECIM_MED (2 kHz at 20 kHz).
pub const DECIM_MED: u8 = 10;
const _: () = assert!(
    phase::PUBLISH < phase::SPARE_A
        && phase::SPARE_A < phase::SPARE_B
        && phase::SPARE_B < DECIM_MED
);
/// MEDIUM -> SLOW decimation: SLOW_HZ = MED_HZ / DECIM_SLOW (62.5 Hz).
pub const DECIM_SLOW: u8 = 32;

/// The stages of the medium chain, in data-flow order.
pub mod phase {
    /// Configuration, CONTROL, the edges and the drive kind.
    pub const CONTROL: u8 = 0;
    /// Back-EMF close, observer step, speed source switch.
    pub const OBSERVER: u8 = 1;
    /// Trajectory and position loop; the hold updates the drive kind.
    pub const TRAJECTORY: u8 = 2;
    /// Limits fold, OpenLoop base duty, stall fault.
    pub const LIMITS: u8 = 3;
    /// Velocity loop or the current clamp; reference and band land.
    pub const VELOCITY: u8 = 4;
    /// Rail estimate, sensor screen, position error timer.
    pub const RAIL: u8 = 5;
    /// The slow block, every DECIM_SLOW periods, and the thermometer's
    /// finish one period later.
    pub const SLOW: u8 = 6;
    /// Estimates, mode, limits and sensors publish.
    pub const PUBLISH: u8 = 7;
    /// The configuration take after a commit; else, the period after the
    /// thermometer's finish, the install of a reference it froze. A stage
    /// whose phase another stage takes waits a period: its pending bit
    /// carries it.
    pub const SPARE_A: u8 = 8;
    /// The NTC the period before a SLOW tick; the period after the finish,
    /// the derate from its temperature and the cold check after an install.
    pub const SPARE_B: u8 = 9;
}

/// Ticks between two tick-load closes.
pub const HEALTH_CLOSE_TICKS: u32 = 2 * DECIM_MED as u32;

/// The phase every tick-load close lands on: the lightest Closed phase that
/// carries no SLOW or thermometer stage and no configuration take.
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
    /// whether a SLOW stage ran.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        k: u8,
        m: &Measured,
        cfg: &KernelConfig,
        shared: &Shared,
        faults: &mut FaultLatch,
        fast: &mut Fast,
        cmd: &mut Command,
    ) -> bool {
        match k {
            phase::CONTROL => self.control(shared.frame().pos(), cfg, shared, faults, fast, cmd),
            phase::OBSERVER => self.observe(shared.frame().pos(), cfg, shared, fast),
            phase::TRAJECTORY => self.trajectory(cfg, faults, cmd),
            phase::LIMITS => self.limit(cfg, shared, faults, fast),
            phase::VELOCITY => self.velocity(cfg, cmd),
            phase::RAIL => self.rail(shared.frame(), cfg, faults, cmd),
            phase::SLOW => return self.slow(m, cfg, faults),
            phase::PUBLISH => self.publish(shared, faults, fast),
            _ => {}
        }
        false
    }
}
