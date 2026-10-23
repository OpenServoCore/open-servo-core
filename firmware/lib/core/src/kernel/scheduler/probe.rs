use super::Kernel;
use super::table::{DECIM_SLOW, phase};
use crate::budget::Regime;
use crate::kernel::phases::fast;
use crate::traits::{ControlIo, TelStream};

/// What the budget probe reads of the kernel between ticks: the medium
/// phase the next tick runs, the count the configuration was taken at (a
/// take moves it), whether the boot tick has run, which SLOW stage the
/// next tick runs, the drive arm's regime and the drive direction check's
/// state. Only the kernel tick moves any of it, so the tags after one tick
/// are the tags before the next.
#[repr(C)]
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct ProbeTags {
    pub phase: u8,
    pub config_seq: u8,
    pub booted: bool,
    pub slow_due: bool,
    pub slow_finish: bool,
    pub regime: Regime,
    /// `DriveCheck::state`.
    pub drive: u8,
}

impl<I: ControlIo, T: TelStream> Kernel<I, T> {
    /// The kernel as the budget probe tags the tick after it.
    pub fn probe_tags(&self) -> ProbeTags {
        ProbeTags {
            phase: self.phase,
            config_seq: self.config_seq,
            booted: self.booted,
            slow_due: self.phase == phase::SLOW && self.medium.slow_periods + 1 >= DECIM_SLOW,
            slow_finish: self.phase == phase::SLOW && self.medium.slow_periods == 0,
            regime: match self.cmd.drive {
                _ if self.faults.mask() != 0 => Regime::Open,
                fast::Drive::Closed { .. } => Regime::Closed,
                fast::Drive::OpenLoop { .. } => Regime::OpenLoop,
                fast::Drive::Off | fast::Drive::Brake => Regime::Open,
            },
            drive: self.medium.drive.state(),
        }
    }
}
