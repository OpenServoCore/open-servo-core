//! Data state: whether the persisted images this servo booted on are its
//! own. `data_flags` (TELEMETRY-MODE) names every image that did not load,
//! as information for the host: no reason refuses a mode. Only the kernel's
//! belt does (`kernel::config::belt`, on the constants a loop would run
//! on).
//!
//! Writers: boot (pre-IRQ) and SAVE (bus dispatch), which retires every
//! reason. The module also runs the main-loop job a position table COMMIT
//! posts (`Shared::data_job_run` + `data_job_publish`): validating the
//! array outlasts the reply deadline, so dispatch only posts, and the job
//! lands the verdict. The publish runs with the bus masked and under a
//! generation check, so a write the verdict depends on landing mid-job
//! leaves the job posted.

use crate::regions::calib::addr::pot::{RAW_MAX, RAW_MIN};
use crate::regions::control::addr::lifecycle::TORQUE_ENABLE;
use crate::{RegionStorage, Shared, pos_lut};

/// Boot found both CONFIG slots erased.
pub const CONFIG_VIRGIN: u8 = 1 << 0;
/// Boot found CONFIG bytes that parse under no version; board defaults
/// run, as on a blank servo.
pub const CONFIG_CORRUPT: u8 = 1 << 1;
pub const CALIB_VIRGIN: u8 = 1 << 2;
pub const CALIB_CORRUPT: u8 = 1 << 3;
/// Boot found a CRC-valid CONFIG image of another layout version: sound
/// bytes that mean nothing here, handled like a fresh servo.
pub const CONFIG_STALE: u8 = 1 << 6;
pub const CALIB_STALE: u8 = 1 << 7;

/// The reasons a successful SAVE retires, every image reason: the
/// persisted table is now the one in RAM.
pub const SAVE_CLEARS: u8 =
    CONFIG_VIRGIN | CONFIG_CORRUPT | CALIB_VIRGIN | CALIB_CORRUPT | CONFIG_STALE | CALIB_STALE;

/// What a commit posts for the main loop (`Shared::data_job_run`).
pub mod job {
    /// A LUT COMMIT: validate the array and land its state.
    pub const LUT_COMMIT: u8 = 1 << 0;
}

/// One run of the posted job, computed unmasked and published masked
/// (`Shared::data_job_publish`).
#[derive(Copy, Clone, Debug)]
pub struct DataJob {
    generation: u16,
    pos_lut_state: u8,
}

/// What boot made of one persisted image's A/B slots.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ImageState {
    Loaded = 0,
    /// Both slots erased.
    Virgin = 1,
    /// Bytes present, none parse under any version.
    Corrupt = 2,
    /// A CRC-valid image of another layout version, nothing newer.
    Stale = 3,
}

impl ImageState {
    const fn flags(self, virgin: u8, corrupt: u8, stale: u8) -> u8 {
        match self {
            ImageState::Loaded => 0,
            ImageState::Virgin => virgin,
            ImageState::Corrupt => corrupt,
            ImageState::Stale => stale,
        }
    }
}

impl Shared {
    /// Boot publish, after both overlays. Pre-IRQ; sole writer.
    pub fn publish_data_state(&self, config: ImageState, calib: ImageState) {
        self.table.with_mut(|t| {
            t.telemetry.mode.data_flags = config.flags(CONFIG_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE)
                | calib.flags(CALIB_VIRGIN, CALIB_CORRUPT, CALIB_STALE);
        });
    }

    /// A committed write `[addr, addr + len)` to what a posted COMMIT's
    /// verdict reads (torque, the stops) moves the generation, so a job in
    /// flight re-judges. Bus dispatch only; one copy behind both commit
    /// sites, O(1).
    #[inline(never)]
    #[rustc_align(4)]
    pub fn data_state_after_commit(&self, addr: u16, len: u16) {
        let end = addr.saturating_add(len);
        let torque = addr <= TORQUE_ENABLE && end > TORQUE_ENABLE;
        let stops = addr < RAW_MAX + 2 && end > RAW_MIN;
        if torque || stops {
            self.data_touch(0, 0);
        }
    }

    pub fn data_job_pending(&self) -> bool {
        self.data_job() != 0
    }

    /// Main loop: run what dispatch posted, unmasked and preemptible. A LUT
    /// COMMIT validates the array against the stops (REJECT_TORQUE if
    /// torque came on since the command, the refusal dispatch would have
    /// given). `None` while nothing is posted.
    pub fn data_job_run(&self) -> Option<DataJob> {
        // generation before job: a post between the two reads moves it
        // and the publish discards this run.
        let generation = self.data_gen();
        if self.data_job() == 0 {
            return None;
        }
        let (torque, raw_min, raw_max) = self.table.with(|t| {
            (
                t.control.lifecycle.torque_enable,
                t.calib.pot.raw_min,
                t.calib.pot.raw_max,
            )
        });
        let pos_lut_state = if torque {
            pos_lut::state::REJECT_TORQUE
        } else {
            self.with_pos_lut(|k| pos_lut::verdict(k, raw_min, raw_max))
        };
        Some(DataJob {
            generation,
            pos_lut_state,
        })
    }

    /// Publish a run, the bus masked on the chip. A generation that moved
    /// since the run leaves the job posted (`false`); the next poll runs it
    /// again.
    pub fn data_job_publish(&self, job: DataJob) -> bool {
        if self.data_gen() != job.generation {
            return false;
        }
        self.table
            .with_mut(|t| t.control.pos_lut.pos_lut_state = job.pos_lut_state);
        self.data_job_done(job::LUT_COMMIT);
        true
    }

    /// Run and publish in one context no write can land in: SAVE (bus
    /// dispatch), the sim's main loop, tests.
    pub fn data_job_service(&self) -> bool {
        self.data_job_run()
            .is_some_and(|j| self.data_job_publish(j))
    }

    /// A successful SAVE retires [`SAVE_CLEARS`]. Bus dispatch only.
    pub fn data_state_saved(&self) {
        self.table
            .with_mut(|t| t.telemetry.mode.data_flags &= !SAVE_CLEARS);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flags(sh: &Shared) -> u8 {
        sh.table.with(|t| t.telemetry.mode.data_flags)
    }

    #[test]
    fn boot_publish_folds_both_images_and_save_retires_them() {
        let sh = Shared::new();
        for (config, calib, want) in [
            (ImageState::Loaded, ImageState::Loaded, 0),
            (
                ImageState::Virgin,
                ImageState::Virgin,
                CONFIG_VIRGIN | CALIB_VIRGIN,
            ),
            (
                ImageState::Corrupt,
                ImageState::Stale,
                CONFIG_CORRUPT | CALIB_STALE,
            ),
            (
                ImageState::Stale,
                ImageState::Corrupt,
                CONFIG_STALE | CALIB_CORRUPT,
            ),
        ] {
            sh.publish_data_state(config, calib);
            assert_eq!(flags(&sh), want);
            sh.data_state_saved();
            assert_eq!(flags(&sh), 0, "SAVE retires every image reason");
        }
    }

    /// Ke unset is the belt's to refuse, not a data-state reason.
    #[test]
    fn a_zero_ke_set_names_no_reason() {
        let sh = Shared::new();
        sh.table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 0;
            t.calib.motor.ke_vpc_q = 0;
        });
        sh.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(flags(&sh), 0);
    }
}
