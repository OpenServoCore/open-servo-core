//! The kernel = scheduler + phases. One `on_tick` per PWM period
//! (`scheduler`) carries all three rates over the phases (`phases`): FAST
//! every tick (`phases::fast`: sensor publish, window select, OC trip,
//! back-EMF sums, TEL, current PI, motor write); MEDIUM once per
//! DECIM_MED-tick period, spread over its ticks one phase each (`Medium`,
//! `phase`: CONTROL and the edges, fusion, trajectory and position, limits,
//! velocity, vbus and detectors, slow, publishes); SLOW every DECIM_SLOW
//! periods inside the medium half in two stages a period apart (undervolt
//! and the thermometer's step on the due tick, its temperature and
//! overtemp on the next period's; the thermometer's NTC on the last spare
//! phase of the period before, the derate and the install of a reference
//! it froze on the spare phases of the period after the second stage). The
//! schedule itself is one table (`scheduler::table`); the loops and
//! detectors the phases run live in `phases::loops`. The medium half leaves
//! the fast half a `Command`; the fast measurement reaches the medium phase
//! of the same tick as `Measured`. Only the CONTROL phase reads CONTROL, so
//! a host write (torque, mode, goals) takes effect at the next period's
//! CONTROL phase. The tick index is the sample clock: every executed tick
//! is one period, and nothing reads a wall clock. CONFIG and CALIB reach
//! the tick through the kernel's own copy (`config`), taken whole at
//! SPARE_A after a commit and never under one, so a configuration write
//! takes effect within a medium period, or one more when the tick lands
//! inside the next commit.

mod config;
pub mod phases;
mod scheduler;

pub use config::{KernelConfig, SPAN_END, SPAN_START};
pub use phases::loops::current::CurrentLoop;
pub use phases::loops::limits::{IBand, LimitState};
pub use phases::loops::position::PosOut;
pub use phases::loops::trajectory::TrajGen;
pub use phases::loops::velocity::VelocityLoop;
pub use scheduler::table::{
    DECIM_MED, DECIM_SLOW, HEALTH_CLOSE_PHASE, HEALTH_CLOSE_TICKS, TICK_HZ, health_close_at,
    health_close_lands, phase,
};
pub use scheduler::{Kernel, KernelTiming, ProbeTags};

#[cfg(test)]
mod tests;
