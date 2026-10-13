use super::Kernel;
use super::table::{DECIM_SLOW, phase};
use crate::kernel::phases::fast;
use crate::tel::TelStream;
use crate::traits::ControlIo;

/// What the budget probe reads around a tick: before it, the medium phase
/// it runs, whether the boot tick has run and which SLOW stage it runs; on
/// either side, the count the configuration was taken at (a take moves it),
/// the thermometer's pending stages and whether the drive arm is Closed.
#[derive(Copy, Clone)]
pub struct ProbeTags {
    pub phase: u8,
    pub config_seq: u8,
    pub booted: bool,
    pub slow_due: bool,
    pub slow_finish: bool,
    /// `thermal::stage` bits.
    pub therm: u8,
    pub closed: bool,
}

impl<I: ControlIo, T: TelStream> Kernel<I, T> {
    /// The kernel as the budget probe tags the tick around it.
    pub fn probe_tags(&self) -> ProbeTags {
        ProbeTags {
            phase: self.phase,
            config_seq: self.config_seq,
            booted: self.booted,
            slow_due: self.phase == phase::SLOW && self.medium.slow_periods + 1 >= DECIM_SLOW,
            slow_finish: self.phase == phase::SLOW && self.medium.slow_periods == 0,
            therm: self.medium.thermal.pending(),
            closed: self.faults.mask() == 0 && matches!(self.cmd.drive, fast::Drive::Closed { .. }),
        }
    }
}
