//! Data state: whether the persisted images and the identified set behind
//! the closed loops are this servo's own. `data_flags` (TELEMETRY-MODE)
//! names every reason they are not; the kernel refuses closed loop while
//! one holds and latches `CODE_DATA` on the attempt (kernel/faults.rs), so
//! a virgin servo in OpenLoop shows no ALERT. Writers: boot (pre-IRQ), then
//! the HIGH dispatcher only (SAVE); the kernel reads.

use crate::regions::calib::CalibMotor;
use crate::regions::control::Mode;
use crate::{ControlTableCell, RegionStorage};

/// Boot found both CONFIG slots erased.
pub const CONFIG_VIRGIN: u8 = 1 << 0;
/// Boot found CONFIG bytes that parse under no version. Board defaults
/// stand in for a lost tuned config (limits, polarity), so this alone
/// refuses every mode; only FACTORY + reboot clears it.
pub const CONFIG_CORRUPT: u8 = 1 << 1;
pub const CALIB_VIRGIN: u8 = 1 << 2;
pub const CALIB_CORRUPT: u8 = 1 << 3;
/// Reserved for the plant stamp; nothing sets it yet.
pub const STAMP_MISMATCH: u8 = 1 << 4;
/// `recip_ke_q == 0 || ke_vpc_q == 0` at the last checkpoint (boot, SAVE).
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

impl ControlTableCell {
    /// Boot publish, after both overlays: the image verdicts plus the plant
    /// checkpoint over the overlaid CALIB. Pre-IRQ; sole writer.
    pub fn publish_data_state(&self, config: ImageState, calib: ImageState) {
        self.with_mut(|t| {
            t.telemetry.mode.data_flags = config.flags(CONFIG_VIRGIN, CONFIG_CORRUPT, CONFIG_STALE)
                | calib.flags(CALIB_VIRGIN, CALIB_CORRUPT, CALIB_STALE)
                | plant_flag(&t.calib.motor);
        });
    }

    /// SAVE checkpoint: PLANT_UNSET follows the live set. HIGH dispatch only.
    pub fn data_state_checkpoint(&self) {
        self.with_mut(|t| {
            t.telemetry.mode.data_flags =
                (t.telemetry.mode.data_flags & !PLANT_UNSET) | plant_flag(&t.calib.motor);
        });
    }

    /// A successful SAVE retires [`SAVE_CLEARS`]. HIGH dispatch only.
    pub fn data_state_saved(&self) {
        self.with_mut(|t| t.telemetry.mode.data_flags &= !SAVE_CLEARS);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn boot_publish_folds_both_images_and_the_plant() {
        let table = ControlTableCell::new();
        table.publish_data_state(ImageState::Loaded, ImageState::Loaded);
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            PLANT_UNSET,
            "zero Ke on a loaded table"
        );
        table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 3700;
            t.calib.motor.ke_vpc_q = 1150;
        });
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
            assert_eq!(table.with(|t| t.telemetry.mode.data_flags), want);
        }
        table.with_mut(|t| t.calib.motor.ke_vpc_q = 0);
        table.publish_data_state(ImageState::Virgin, ImageState::Loaded);
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            CONFIG_VIRGIN | PLANT_UNSET
        );
    }

    #[test]
    fn save_checkpoint_recomputes_plant_and_success_retires_the_fresh_reasons() {
        let table = ControlTableCell::new();
        table.publish_data_state(ImageState::Corrupt, ImageState::Stale);
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            CONFIG_CORRUPT | CALIB_STALE | PLANT_UNSET
        );
        table.with_mut(|t| {
            t.calib.motor.recip_ke_q = 3700;
            t.calib.motor.ke_vpc_q = 1150;
        });
        table.data_state_checkpoint();
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            CONFIG_CORRUPT | CALIB_STALE
        );
        table.data_state_saved();
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            CONFIG_CORRUPT,
            "SAVE never blesses a corrupt config"
        );
        table.with_mut(|t| t.calib.motor.recip_ke_q = 0);
        table.data_state_checkpoint();
        assert_eq!(
            table.with(|t| t.telemetry.mode.data_flags),
            CONFIG_CORRUPT | PLANT_UNSET
        );
    }
}
