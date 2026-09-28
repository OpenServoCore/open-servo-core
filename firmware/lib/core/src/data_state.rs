//! Data state: whether the persisted images and the identified set behind
//! the closed loops are this servo's own. `data_flags` (TELEMETRY-MODE)
//! names every reason they are not; the kernel refuses closed loop while
//! one holds and latches `CODE_DATA` on the attempt (kernel/faults.rs), so
//! a virgin servo in OpenLoop shows no ALERT. Writers: boot (pre-IRQ), then
//! the HIGH dispatcher only (commits, SAVE); the kernel reads.

use crate::regions::ControlTable;
use crate::regions::calib::CalibMotor;
use crate::regions::calib::addr::stamp::PLANT_STAMP;
use crate::regions::control::Mode;
use crate::stamp;
use crate::{ControlTableCell, RegionStorage};

/// Boot found both CONFIG slots erased.
pub const CONFIG_VIRGIN: u8 = 1 << 0;
/// Boot found CONFIG bytes that parse under no version. Board defaults
/// stand in for a lost tuned config (limits, polarity), so this alone
/// refuses every mode; only FACTORY + reboot clears it.
pub const CONFIG_CORRUPT: u8 = 1 << 1;
pub const CALIB_VIRGIN: u8 = 1 << 2;
pub const CALIB_CORRUPT: u8 = 1 << 3;
/// The last checkpoint's recompute (`stamp` module) differed from
/// `plant_stamp`, or a covered field was written since: the stamp no
/// longer describes the live set.
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

/// The checkpoint verdict over the live set. No LUT is applied yet, so
/// the stamp covers the identity table.
fn checkpoint_flags(t: &ControlTable) -> u8 {
    let stamp = if stamp::compute(t, None) == t.calib.stamp.plant_stamp {
        0
    } else {
        STAMP_MISMATCH
    };
    plant_flag(&t.calib.motor) | stamp
}

impl ControlTableCell {
    /// Boot publish, after both overlays: the image verdicts plus the
    /// checkpoint over the overlaid set. Pre-IRQ; sole writer.
    pub fn publish_data_state(&self, config: ImageState, calib: ImageState) {
        self.with_mut(|t| {
            t.telemetry.mode.data_flags = config.flags(CONFIG_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE)
                | calib.flags(CALIB_VIRGIN, CALIB_CORRUPT, CALIB_STALE)
                | checkpoint_flags(t);
        });
    }

    /// Checkpoint: STAMP_MISMATCH and PLANT_UNSET follow the live set. A
    /// torque-off stamp write and SAVE; HIGH dispatch only.
    pub fn data_state_checkpoint(&self) {
        self.with_mut(|t| {
            let flags = checkpoint_flags(t);
            t.telemetry.mode.data_flags = (t.telemetry.mode.data_flags & !CHECKPOINT) | flags;
        });
    }

    /// A committed write `[addr, addr + len)`: a covered field marks the
    /// stamp stale at once (a host that dies mid-sequence leaves the
    /// mismatch behind), and a stamp write is verified only with torque
    /// off - under torque it lands unverified and the mismatch waits for
    /// the next torque-off checkpoint. Never stops a running loop; the
    /// kernel reads the flags at its next entry. HIGH dispatch only; one
    /// copy behind both commit sites.
    #[inline(never)]
    pub fn data_state_after_commit(&self, addr: u16, len: u16) {
        let end = addr.saturating_add(len);
        let stamp_hit = addr < PLANT_STAMP + 2 && end > PLANT_STAMP;
        if !stamp_hit && !stamp::covers(addr, len) {
            return;
        }
        let torque = self.with(|t| t.control.lifecycle.torque_enable);
        if stamp_hit && !torque {
            self.data_state_checkpoint();
        } else {
            self.with_mut(|t| t.telemetry.mode.data_flags |= STAMP_MISMATCH);
        }
    }

    /// A successful SAVE retires [`SAVE_CLEARS`]. HIGH dispatch only.
    pub fn data_state_saved(&self) {
        self.with_mut(|t| t.telemetry.mode.data_flags &= !SAVE_CLEARS);
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

    fn flags(table: &ControlTableCell) -> u8 {
        table.with(|t| t.telemetry.mode.data_flags)
    }

    fn set_ke(table: &ControlTableCell) {
        table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 3700;
            t.calib.motor.ke_vpc_q = 1150;
        });
    }

    /// The host's stamp over the intended set.
    fn stamp(table: &ControlTableCell) {
        table.with_mut(|t| t.calib.stamp.plant_stamp = stamp::compute(t, None));
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
        let table = ControlTableCell::new();
        table.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(
            flags(&table),
            STAMP_MISMATCH | PLANT_UNSET,
            "zero Ke, never stamped"
        );
        set_ke(&table);
        stamp(&table);
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
            table.publish_data_state(config, calib);
            assert_eq!(flags(&table), want);
        }
        table.with_mut(|t| t.calib.motor.ke_vpc_q = 0);
        table.publish_data_state(ImageState::Virgin, ImageState::Loaded);
        assert_eq!(
            flags(&table),
            CONFIG_VIRGIN | STAMP_MISMATCH | PLANT_UNSET,
            "a covered value moved under the stamp"
        );
        stamp(&table);
        table.publish_data_state(ImageState::Virgin, ImageState::Loaded);
        assert_eq!(
            flags(&table),
            CONFIG_VIRGIN | PLANT_UNSET,
            "a stamped zero-Ke set is consistent, not identified"
        );
    }

    #[test]
    fn save_checkpoint_recomputes_and_success_retires_the_fresh_reasons() {
        let table = ControlTableCell::new();
        table.publish_data_state(ImageState::Corrupt, ImageState::Stale);
        assert_eq!(
            flags(&table),
            CONFIG_CORRUPT | CALIB_STALE | STAMP_MISMATCH | PLANT_UNSET
        );
        set_ke(&table);
        stamp(&table);
        table.data_state_checkpoint();
        assert_eq!(flags(&table), CONFIG_CORRUPT | CALIB_STALE);
        table.data_state_saved();
        assert_eq!(
            flags(&table),
            CONFIG_CORRUPT,
            "SAVE never blesses a corrupt config"
        );
        table.with_mut(|t| t.calib.motor.recip_ke_q = 0);
        table.data_state_checkpoint();
        assert_eq!(flags(&table), CONFIG_CORRUPT | STAMP_MISMATCH | PLANT_UNSET);
    }

    #[test]
    fn covered_write_marks_mismatch_until_a_torque_off_stamp_write() {
        let table = ControlTableCell::new();
        set_ke(&table);
        stamp(&table);
        table.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(flags(&table), 0);

        // uncovered writes leave the verdict alone
        table.data_state_after_commit(CURRENT_LIMIT_COUNTS, 2);
        table.data_state_after_commit(0x180, 4);
        assert_eq!(flags(&table), 0);

        // a covered write marks at once, whatever the value did
        table.data_state_after_commit(V_KP_Q88, 2);
        assert_eq!(flags(&table), STAMP_MISMATCH);
        // the stamp write verifies: the set still matches the stamp
        table.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(flags(&table), 0);

        // the value moved, the stamp did not
        table.with_mut(|t| t.config.limits.drive_polarity = true);
        table.data_state_after_commit(DRIVE_POLARITY, 1);
        table.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(flags(&table), STAMP_MISMATCH, "an end-to-end miss");
        stamp(&table);
        table.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(flags(&table), 0);

        // under torque a stamp write lands unverified
        table.with_mut(|t| t.control.lifecycle.torque_enable = true);
        table.with_mut(|t| t.calib.motor.recip_ke_q = 0);
        table.data_state_after_commit(RECIP_KE_Q, 2);
        assert_eq!(flags(&table), STAMP_MISMATCH, "no checkpoint under torque");
        stamp(&table);
        table.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(flags(&table), STAMP_MISMATCH, "unverified");
        table.with_mut(|t| t.control.lifecycle.torque_enable = false);
        table.data_state_after_commit(PLANT_STAMP, 2);
        assert_eq!(
            flags(&table),
            PLANT_UNSET,
            "the torque-off checkpoint sees a stamped zero-Ke set"
        );

        // a write spanning the stamp and a covered field verifies as one
        table.with_mut(|t| t.calib.motor.recip_ke_q = 3700);
        stamp(&table);
        table.data_state_after_commit(RECIP_KE_Q, PLANT_STAMP + 2 - RECIP_KE_Q);
        assert_eq!(flags(&table), 0);
    }
}
