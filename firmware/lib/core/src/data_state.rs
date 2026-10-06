//! Data state: whether the persisted images and the identified set behind
//! the closed loops are this servo's own. `data_flags` (TELEMETRY-MODE)
//! names every reason they are not; the kernel refuses closed loop while
//! one holds and latches `CODE_DATA` on the attempt (kernel/faults.rs), so
//! a virgin servo in OpenLoop shows no ALERT.
//!
//! Writers: boot (pre-IRQ); the HIGH dispatcher (a covered write, a stamp
//! write and a LUT command mark, SAVE checkpoints and retires); and the
//! main loop's job (`Shared::data_job_run` + `data_job_publish`), which
//! runs the recompute a covered write, a stamp write or a LUT COMMIT
//! posted. The recompute is ~600 B of software CRC, longer than the reply
//! deadline, so HIGH only marks `STAMP_MISMATCH` (the refused direction)
//! and posts; the job clears it once the live set matches the stamp
//! again. The publish runs with ISRs masked and under a generation check,
//! so a write landing mid-job leaves the mark standing and the job posted.
//! The kernel reads.

use crate::regions::calib::CalibMotor;
use crate::regions::calib::addr::stamp::PLANT_STAMP;
use crate::regions::control::Mode;
use crate::regions::control::addr::lifecycle::TORQUE_ENABLE;
use crate::{RegionStorage, Shared, pos_lut, stamp};

/// Boot found both CONFIG slots erased.
pub const CONFIG_VIRGIN: u8 = 1 << 0;
/// Boot found CONFIG bytes that parse under no version. Board defaults
/// stand in for a lost tuned config (limits, polarity), so this alone
/// refuses every mode; only FACTORY + reboot clears it.
pub const CONFIG_CORRUPT: u8 = 1 << 1;
pub const CALIB_VIRGIN: u8 = 1 << 2;
pub const CALIB_CORRUPT: u8 = 1 << 3;
/// The last checkpoint's recompute (`stamp` module) differed from
/// `plant_stamp`, or a covered field was written since and its checkpoint
/// has not run yet: the stamp does not describe the live set.
pub const STAMP_MISMATCH: u8 = 1 << 4;
/// `recip_ke_q == 0 || ke_vpc_q == 0` at the last checkpoint.
pub const PLANT_UNSET: u8 = 1 << 5;
/// Boot found a CRC-valid CONFIG image of another layout version: sound
/// bytes that mean nothing here, handled like a fresh servo.
pub const CONFIG_STALE: u8 = 1 << 6;
pub const CALIB_STALE: u8 = 1 << 7;

/// The reasons a successful SAVE retires: the persisted table is now this
/// servo's own. CONFIG_CORRUPT stays, so a blind SAVE cannot bless board
/// defaults as a tuned config.
pub const SAVE_CLEARS: u8 =
    CONFIG_VIRGIN | CALIB_VIRGIN | CALIB_CORRUPT | CONFIG_STALE | CALIB_STALE;

/// The reasons a checkpoint recomputes from the live set.
const CHECKPOINT: u8 = STAMP_MISMATCH | PLANT_UNSET;

/// What a commit posts for the main loop (`Shared::data_job_run`).
pub mod job {
    /// A covered or stamp write: recompute the checkpoint once torque is
    /// off.
    pub const CHECKPOINT: u8 = 1 << 0;
    /// A LUT COMMIT: validate the array, land its state, then checkpoint.
    pub const LUT_COMMIT: u8 = 1 << 1;
}

/// One run of the posted job, computed unmasked and published masked
/// (`Shared::data_job_publish`).
#[derive(Copy, Clone, Debug)]
pub struct DataJob {
    generation: u16,
    done: u8,
    pos_lut_state: Option<u8>,
    flags: u8,
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

/// Entry verdict for `mode` under `flags`. OpenLoop and Current consume
/// neither Ke nor the loop gains above the current loop, and cal/ident
/// drive only those, so every reason but CONFIG_CORRUPT leaves them open.
pub fn allows(mode: Mode, flags: u8) -> bool {
    flags & CONFIG_CORRUPT == 0 && (flags == 0 || matches!(mode, Mode::OpenLoop | Mode::Current))
}

fn plant_flag(motor: &CalibMotor) -> u8 {
    if motor.recip_ke_q == 0 || motor.ke_vpc_q == 0 {
        PLANT_UNSET
    } else {
        0
    }
}

impl Shared {
    /// The checkpoint verdict over the live set: the stamp covers the pot
    /// LUT the kernel applies, the array while `live` and the identity
    /// otherwise.
    fn checkpoint_flags(&self, live: bool) -> u8 {
        self.with_pos_lut(|k| {
            self.table.with(|t| {
                let lut_points = if live { k.first_chunk() } else { None };
                let stamp = if stamp::compute(t, lut_points) == t.calib.stamp.plant_stamp {
                    0
                } else {
                    STAMP_MISMATCH
                };
                plant_flag(&t.calib.motor) | stamp
            })
        })
    }

    fn lut_live(&self) -> bool {
        self.table
            .with(|t| t.control.pos_lut.pos_lut_state == pos_lut::state::LIVE)
    }

    /// Boot publish, after both overlays: the image verdicts plus the
    /// checkpoint over the overlaid set. Pre-IRQ; sole writer.
    pub fn publish_data_state(&self, config: ImageState, calib: ImageState) {
        let flags = self.checkpoint_flags(self.lut_live());
        self.table.with_mut(|t| {
            t.telemetry.mode.data_flags = config.flags(CONFIG_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE)
                | calib.flags(CALIB_VIRGIN, CALIB_CORRUPT, CALIB_STALE)
                | flags;
        });
    }

    /// Synchronous checkpoint: STAMP_MISMATCH and PLANT_UNSET follow the
    /// live set. SAVE (HIGH, a slow op anyway) and boot fixtures only;
    /// the wire-time checkpoints go through the job.
    pub fn data_state_checkpoint(&self) {
        let flags = self.checkpoint_flags(self.lut_live());
        self.table.with_mut(|t| {
            t.telemetry.mode.data_flags = (t.telemetry.mode.data_flags & !CHECKPOINT) | flags;
        });
    }

    /// A committed write `[addr, addr + len)` to a covered field or the
    /// stamp marks the stamp stale at once (a host that dies mid-sequence
    /// leaves the mismatch behind) and posts the checkpoint that clears it
    /// once the live set matches the stamp again; the job holds it while
    /// torque is on. A torque write moves the generation so a job in
    /// flight re-judges under the new torque. Never stops a running loop;
    /// the kernel reads the flags at its next entry. HIGH dispatch only;
    /// one copy behind both commit sites, O(1).
    #[inline(never)]
    pub fn data_state_after_commit(&self, addr: u16, len: u16) {
        let end = addr.saturating_add(len);
        if addr <= TORQUE_ENABLE && end > TORQUE_ENABLE {
            self.data_touch(0, 0);
        }
        let stamp_hit = addr < PLANT_STAMP + 2 && end > PLANT_STAMP;
        if !stamp_hit && !stamp::covers(addr, len) {
            return;
        }
        self.data_touch(job::CHECKPOINT, 0);
        self.table
            .with_mut(|t| t.telemetry.mode.data_flags |= STAMP_MISMATCH);
    }

    pub fn data_job_pending(&self) -> bool {
        self.data_job() != 0
    }

    /// Main loop: run what HIGH posted, unmasked and preemptible. A LUT
    /// COMMIT validates the array against the stops (REJECT_TORQUE if
    /// torque came on since the command, the refusal HIGH would have
    /// given), then the checkpoint runs over the points that verdict makes
    /// effective. `None` while nothing is posted, and while torque holds
    /// back a lone checkpoint (the CRC stays off torque-on paths; the
    /// torque-off write wakes the loop that runs it).
    pub fn data_job_run(&self) -> Option<DataJob> {
        // generation before job: a post between the two reads moves it
        // and the publish discards this run.
        let generation = self.data_gen();
        let done = self.data_job();
        if done == 0 {
            return None;
        }
        let (torque, raw_min, raw_max, state) = self.table.with(|t| {
            (
                t.control.lifecycle.torque_enable,
                t.calib.pot.raw_min,
                t.calib.pot.raw_max,
                t.control.pos_lut.pos_lut_state,
            )
        });
        if torque && done == job::CHECKPOINT {
            return None;
        }
        let pos_lut_state = (done & job::LUT_COMMIT != 0).then(|| {
            if torque {
                pos_lut::state::REJECT_TORQUE
            } else {
                self.with_pos_lut(|k| pos_lut::verdict(k, raw_min, raw_max))
            }
        });
        let live = pos_lut_state.unwrap_or(state) == pos_lut::state::LIVE;
        Some(DataJob {
            generation,
            done,
            pos_lut_state,
            flags: self.checkpoint_flags(live),
        })
    }

    /// Publish a run, ISRs masked on the chip. A generation that moved
    /// since the run leaves the marks standing and the job posted (`false`);
    /// the next poll runs it again.
    pub fn data_job_publish(&self, job: DataJob) -> bool {
        if self.data_gen() != job.generation {
            return false;
        }
        self.table.with_mut(|t| {
            if let Some(s) = job.pos_lut_state {
                t.control.pos_lut.pos_lut_state = s;
            }
            t.telemetry.mode.data_flags = (t.telemetry.mode.data_flags & !CHECKPOINT) | job.flags;
        });
        self.data_job_done(job.done);
        true
    }

    /// Run and publish in one context that nothing preempts: SAVE (HIGH),
    /// the sim's main loop, tests.
    pub fn data_job_service(&self) -> bool {
        self.data_job_run()
            .is_some_and(|j| self.data_job_publish(j))
    }

    /// A successful SAVE retires [`SAVE_CLEARS`]. HIGH dispatch only.
    pub fn data_state_saved(&self) {
        self.table
            .with_mut(|t| t.telemetry.mode.data_flags &= !SAVE_CLEARS);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::calib::addr::motor::RECIP_KE_Q;
    use crate::regions::config::addr::limits::{CURRENT_LIMIT_COUNTS, DRIVE_POLARITY};
    use crate::regions::config::addr::loop_velocity::V_KP_Q88;

    const MODES: [Mode; 4] = [
        Mode::OpenLoop,
        Mode::Current,
        Mode::Velocity,
        Mode::Position,
    ];
    const REASONS: [u8; 8] = [
        CONFIG_VIRGIN,
        CONFIG_CORRUPT,
        CALIB_VIRGIN,
        CALIB_CORRUPT,
        STAMP_MISMATCH,
        PLANT_UNSET,
        CONFIG_STALE,
        CALIB_STALE,
    ];

    fn flags(sh: &Shared) -> u8 {
        sh.table.with(|t| t.telemetry.mode.data_flags)
    }

    fn set_ke(sh: &Shared) {
        sh.table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 3700;
            t.calib.motor.ke_vpc_q = 1150;
        });
    }

    /// The host's stamp over the intended set.
    fn stamp(sh: &Shared) {
        sh.table
            .with_mut(|t| t.calib.stamp.plant_stamp = stamp::compute(t, None));
    }

    #[test]
    fn allows_truth_table() {
        for mode in MODES {
            assert!(allows(mode, 0), "{mode:?} with no reason");
        }
        for reason in REASONS {
            for extra in [0, CALIB_VIRGIN | PLANT_UNSET] {
                let flags = reason | extra;
                let open = flags & CONFIG_CORRUPT == 0;
                assert_eq!(allows(Mode::OpenLoop, flags), open, "{flags:#04x}");
                assert_eq!(allows(Mode::Current, flags), open, "{flags:#04x}");
                assert!(!allows(Mode::Velocity, flags), "{flags:#04x}");
                assert!(!allows(Mode::Position, flags), "{flags:#04x}");
            }
        }
        assert_eq!(
            REASONS.iter().fold(0, |m, r| m | r),
            0xFF,
            "every bit named"
        );
    }

    #[test]
    fn boot_publish_folds_both_images_and_the_checkpoint() {
        let sh = Shared::new();
        sh.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(
            flags(&sh),
            STAMP_MISMATCH | PLANT_UNSET,
            "zero Ke, never stamped"
        );
        set_ke(&sh);
        stamp(&sh);
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
        }
        sh.table.with_mut(|t| t.calib.motor.ke_vpc_q = 0);
        sh.publish_data_state(ImageState::Virgin, ImageState::Loaded);
        assert_eq!(
            flags(&sh),
            CONFIG_VIRGIN | STAMP_MISMATCH | PLANT_UNSET,
            "a covered value moved under the stamp"
        );
        stamp(&sh);
        sh.publish_data_state(ImageState::Virgin, ImageState::Loaded);
        assert_eq!(
            flags(&sh),
            CONFIG_VIRGIN | PLANT_UNSET,
            "a stamped zero-Ke set is consistent, not identified"
        );
    }

    #[test]
    fn save_checkpoint_recomputes_and_success_retires_the_fresh_reasons() {
        let sh = Shared::new();
        sh.publish_data_state(ImageState::Corrupt, ImageState::Stale);
        assert_eq!(
            flags(&sh),
            CONFIG_CORRUPT | CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET
        );
        set_ke(&sh);
        stamp(&sh);
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), CONFIG_CORRUPT | CALIB_STALE);
        sh.data_state_saved();
        assert_eq!(
            flags(&sh),
            CONFIG_CORRUPT,
            "SAVE never blesses a corrupt config"
        );
        sh.table.with_mut(|t| t.calib.motor.recip_ke_q = 0);
        sh.data_state_checkpoint();
        assert_eq!(flags(&sh), CONFIG_CORRUPT | STAMP_MISMATCH | PLANT_UNSET);
    }

    #[test]
    fn covered_write_marks_mismatch_until_a_torque_off_stamp_write() {
        let sh = Shared::new();
        set_ke(&sh);
        stamp(&sh);
        sh.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(flags(&sh), 0);

        // uncovered writes leave the verdict alone
        sh.data_state_after_commit(CURRENT_LIMIT_COUNTS, 2);
        sh.data_state_after_commit(0x180, 4);
        assert_eq!(flags(&sh), 0);

        // a covered write marks at once, whatever the value did
        sh.data_state_after_commit(V_KP_Q88, 2);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        // the stamp write marks and posts; the job verifies: the set still
        // matches the stamp
        sh.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(flags(&sh), STAMP_MISMATCH, "refused until verified");
        assert!(sh.data_job_pending());
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), 0);
        assert!(!sh.data_job_pending());

        // the value moved, the stamp did not
        sh.table.with_mut(|t| t.config.limits.drive_polarity = true);
        sh.data_state_after_commit(DRIVE_POLARITY, 1);
        sh.data_state_after_commit(PLANT_STAMP, 2);
        sh.data_job_service();
        assert_eq!(flags(&sh), STAMP_MISMATCH, "an end-to-end miss");
        stamp(&sh);
        sh.data_state_after_commit(PLANT_STAMP, 2);
        sh.data_job_service();
        assert_eq!(flags(&sh), 0);

        // under torque a stamp write lands unverified
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        sh.table.with_mut(|t| t.calib.motor.recip_ke_q = 0);
        sh.data_state_after_commit(RECIP_KE_Q, 2);
        assert_eq!(flags(&sh), STAMP_MISMATCH, "no checkpoint under torque");
        stamp(&sh);
        sh.data_state_after_commit(PLANT_STAMP, 2);
        assert!(!sh.data_job_service(), "no checkpoint under torque");
        assert_eq!(flags(&sh), STAMP_MISMATCH, "unverified");
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        sh.data_state_after_commit(PLANT_STAMP, 2);
        sh.data_job_service();
        assert_eq!(
            flags(&sh),
            PLANT_UNSET,
            "the torque-off checkpoint sees a stamped zero-Ke set"
        );

        // a write spanning the stamp and a covered field verifies as one
        sh.table.with_mut(|t| t.calib.motor.recip_ke_q = 3700);
        stamp(&sh);
        sh.data_state_after_commit(RECIP_KE_Q, PLANT_STAMP + 2 - RECIP_KE_Q);
        sh.data_job_service();
        assert_eq!(flags(&sh), 0);
    }

    /// The verdict follows the live set: a covered edit marks, and putting
    /// the stamped value back clears, at the next torque-off checkpoint.
    #[test]
    fn restoring_a_covered_value_clears_the_mismatch() {
        let sh = Shared::new();
        set_ke(&sh);
        stamp(&sh);
        sh.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(flags(&sh), 0);

        sh.table.with_mut(|t| t.calib.motor.recip_ke_q = 3800);
        sh.data_state_after_commit(RECIP_KE_Q, 2);
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), STAMP_MISMATCH, "the set differs");
        sh.table.with_mut(|t| t.calib.motor.recip_ke_q = 3700);
        sh.data_state_after_commit(RECIP_KE_Q, 2);
        assert_eq!(flags(&sh), STAMP_MISMATCH, "refused until verified");
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), 0, "the stamped set again");

        // under torque the checkpoint waits for torque off
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = true);
        sh.table.with_mut(|t| t.config.limits.drive_polarity = true);
        sh.data_state_after_commit(DRIVE_POLARITY, 1);
        sh.table
            .with_mut(|t| t.config.limits.drive_polarity = false);
        sh.data_state_after_commit(DRIVE_POLARITY, 1);
        assert!(!sh.data_job_service());
        assert_eq!(flags(&sh), STAMP_MISMATCH);
        sh.table
            .with_mut(|t| t.control.lifecycle.torque_enable = false);
        sh.data_state_after_commit(TORQUE_ENABLE, 1);
        assert!(sh.data_job_service());
        assert_eq!(flags(&sh), 0);
        assert!(!sh.data_job_pending());
    }
}
