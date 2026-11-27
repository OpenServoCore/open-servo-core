//! Data state: what boot made of the persisted images, and the main-loop
//! job a position table COMMIT posts. `status_flags` bits 1 and 2
//! (`osc_protocol::table::STATUS_FLAG_*_DEFAULTS`) name each image boot
//! did not load, as information for the host: no bit refuses a mode. Only
//! the kernel's belt does (`kernel::config::belt`, on the constants a loop
//! would run on), and it publishes the modes it refuses in
//! `refused_modes`.
//!
//! Writers: boot (pre-IRQ) and SAVE (bus dispatch), which clears the bits.
//! The job (`Shared::data_job_run` + `data_job_publish`): validating the
//! array outlasts the reply deadline, so dispatch only posts, and the job
//! lands the verdict. The publish runs with the bus masked and under a
//! generation check, so a write the verdict depends on landing mid-job
//! leaves the job posted.

use osc_protocol::table::{STATUS_FLAG_CALIB_DEFAULTS, STATUS_FLAG_CONFIG_DEFAULTS};

use crate::regions::calib::addr::pot::{RAW_MAX, RAW_MIN};
use crate::regions::control::addr::lifecycle::TORQUE_ENABLE;
use crate::{RegionStorage, Shared, pos_lut};

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

/// What boot made of one persisted image's A/B slots. Blank, unreadable
/// and another layout's are one case: no slot parsed, and the defaults
/// already seeded stand.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ImageState {
    Loaded = 0,
    Defaults = 1,
}

impl Shared {
    /// Boot publish, after both overlays. Pre-IRQ; sole writer.
    pub fn publish_data_state(&self, config: ImageState, calib: ImageState) {
        let bits = (config as u8 * STATUS_FLAG_CONFIG_DEFAULTS)
            | (calib as u8 * STATUS_FLAG_CALIB_DEFAULTS);
        self.table
            .with_mut(|t| t.telemetry.common.status_flags |= bits);
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
}

#[cfg(test)]
mod tests {
    use osc_protocol::table::STATUS_SAVE_CLEARS;

    use super::*;

    fn flags(sh: &Shared) -> u8 {
        sh.table.with(|t| t.telemetry.common.status_flags)
    }

    #[test]
    fn boot_publish_names_each_image_on_its_defaults() {
        for (config, calib, want) in [
            (ImageState::Loaded, ImageState::Loaded, 0),
            (
                ImageState::Defaults,
                ImageState::Defaults,
                STATUS_FLAG_CONFIG_DEFAULTS | STATUS_FLAG_CALIB_DEFAULTS,
            ),
            (
                ImageState::Defaults,
                ImageState::Loaded,
                STATUS_FLAG_CONFIG_DEFAULTS,
            ),
            (
                ImageState::Loaded,
                ImageState::Defaults,
                STATUS_FLAG_CALIB_DEFAULTS,
            ),
        ] {
            let sh = Shared::new();
            sh.publish_data_state(config, calib);
            assert_eq!(flags(&sh), want, "{config:?} {calib:?}");
            assert_eq!(want & !STATUS_SAVE_CLEARS, 0, "SAVE clears them");
        }
    }

    /// Ke unset is the belt's to refuse, not a store bit.
    #[test]
    fn a_zero_ke_set_names_no_store_bit() {
        let sh = Shared::new();
        sh.table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 0;
            t.calib.motor.ke_vpc_q = 0;
        });
        sh.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(flags(&sh), 0);
    }
}
