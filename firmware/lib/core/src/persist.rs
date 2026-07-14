//! Config persistence (osc-native sec 9.4/sec 9.5): the saved-image codec, the
//! boot-time A/B pick + overlay, and the [`ConfigStore`] port the chip's
//! flash provider (or the sim's RAM fake) implements.
//!
//! One image fits a single 256 B fast-erase page:
//!
//!   [0]    magic
//!   [1]    layout version
//!   [2..4] seq u16 LE -- A/B alternation picks the wrapping-newest
//!   [4..6] body len u16 LE (= config + profile)
//!   [6..8] CRC-16/ARC u16 LE over bytes 0..6 ++ 8..IMAGE_LEN
//!   [8..]  config region (128 B) ++ profile region (64 B), raw table bytes
//!
//! The CRC sits in the header (not the tail) so every segment is a whole
//! number of words -- the flash provider streams header/config/profile
//! straight into the page buffer with no byte packing and no staging copy.

use control_table::RegisterMap;
use osc_protocol::crc::{osc_crc, osc_crc_continue};

use crate::ControlTableCell;
use crate::regions::{
    CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE, config,
};

pub const CONFIG_LEN: usize = CONFIG_REGION_SIZE as usize;
pub const PROFILE_LEN: usize = PROFILE_REGION_SIZE as usize;
pub const HEADER_LEN: usize = 8;
pub const IMAGE_LEN: usize = HEADER_LEN + CONFIG_LEN + PROFILE_LEN;

pub const IMAGE_MAGIC: u8 = b'C';
/// Bump on any CONFIG/PROFILE layout change; a mismatched image is ignored
/// (boot keeps board defaults) rather than migrated.
pub const IMAGE_VERSION: u8 = 1;

/// Store failure (erase/program/verify); dispatch answers `hardware` (sec 5.3).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StoreError;

/// sec 9.4 persistence port. Both methods are cold and blocking: on flash,
/// `save` stalls the caller for the erase + program time (ms-scale) -- the
/// torque gate in dispatch is what makes that stall safe.
pub trait ConfigStore: Sync {
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
    ) -> Result<(), StoreError>;
    /// FACTORY (sec 9.5): invalidate both slots -- the erased store IS the
    /// factory state; the follow-up reboot re-seeds board defaults.
    fn wipe(&self) -> Result<(), StoreError>;
}

/// A/B slot names; the store impl maps them to its two pages.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    pub fn other(self) -> Slot {
        match self {
            Slot::A => Slot::B,
            Slot::B => Slot::A,
        }
    }
}

/// Boot-time store state handed to the [`ConfigStore`] impl: the slot the
/// next SAVE programs (the older or invalid one) and the seq it carries.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BootPick {
    pub next_slot: Slot,
    pub next_seq: u16,
    /// A valid image was found and overlaid (false = board defaults stand).
    pub loaded: bool,
}

/// Build the 8-byte header for an image whose body is `config ++ profile`.
pub fn header(
    seq: u16,
    config: &[u8; CONFIG_LEN],
    profile: &[u8; PROFILE_LEN],
) -> [u8; HEADER_LEN] {
    let body_len = (CONFIG_LEN + PROFILE_LEN) as u16;
    let mut h = [0u8; HEADER_LEN];
    h[0] = IMAGE_MAGIC;
    h[1] = IMAGE_VERSION;
    h[2..4].copy_from_slice(&seq.to_le_bytes());
    h[4..6].copy_from_slice(&body_len.to_le_bytes());
    let crc = osc_crc_continue(osc_crc_continue(osc_crc(&h[..6]), config), profile);
    h[6..8].copy_from_slice(&crc.to_le_bytes());
    h
}

/// Assemble a whole image in RAM -- for hosts, fakes, and tests; the chip
/// provider streams the three segments instead (no staging buffer).
pub fn assemble(
    out: &mut [u8; IMAGE_LEN],
    seq: u16,
    config: &[u8; CONFIG_LEN],
    profile: &[u8; PROFILE_LEN],
) {
    out[..HEADER_LEN].copy_from_slice(&header(seq, config, profile));
    out[HEADER_LEN..HEADER_LEN + CONFIG_LEN].copy_from_slice(config);
    out[HEADER_LEN + CONFIG_LEN..].copy_from_slice(profile);
}

/// A validated stored image; region refs borrow the slot bytes in place.
pub struct Image<'a> {
    pub seq: u16,
    pub config: &'a [u8; CONFIG_LEN],
    pub profile: &'a [u8; PROFILE_LEN],
}

impl<'a> Image<'a> {
    /// Validate a slot (extra bytes past `IMAGE_LEN` -- the rest of the page --
    /// are ignored). Magic, version, len, and CRC gate integrity; the
    /// allowed-rules pass gates safety: overlay writes raw bytes into
    /// enum/bool-typed table fields, and an out-of-range discriminant there
    /// is UB, so a CRC-valid image must still never carry one.
    pub fn parse(slot: &'a [u8]) -> Option<Image<'a>> {
        let bytes = slot.get(..IMAGE_LEN)?;
        if bytes[0] != IMAGE_MAGIC || bytes[1] != IMAGE_VERSION {
            return None;
        }
        let seq = u16::from_le_bytes([bytes[2], bytes[3]]);
        if u16::from_le_bytes([bytes[4], bytes[5]]) != (CONFIG_LEN + PROFILE_LEN) as u16 {
            return None;
        }
        let crc = u16::from_le_bytes([bytes[6], bytes[7]]);
        if osc_crc_continue(osc_crc(&bytes[..6]), &bytes[HEADER_LEN..]) != crc {
            return None;
        }
        let img = Image {
            seq,
            config: bytes[HEADER_LEN..HEADER_LEN + CONFIG_LEN].try_into().ok()?,
            profile: bytes[HEADER_LEN + CONFIG_LEN..].try_into().ok()?,
        };
        img.fields_allowed().then_some(img)
    }

    /// Every enum/bool rule whose field lies in a persisted region must hold
    /// an allowed byte. Driven by the table's generated rule data, so new
    /// enum fields are covered without touching this code.
    fn fields_allowed(&self) -> bool {
        <ControlTableCell as RegisterMap>::ALLOWED_RULES
            .iter()
            .all(|r| match self.persisted_byte(r.addr) {
                Some(b) => r.allowed.contains(&b),
                None => true,
            })
    }

    fn persisted_byte(&self, addr: u16) -> Option<u8> {
        if (CONFIG_BASE_ADDR..CONFIG_BASE_ADDR + CONFIG_REGION_SIZE).contains(&addr) {
            Some(self.config[(addr - CONFIG_BASE_ADDR) as usize])
        } else if (PROFILE_BASE_ADDR..PROFILE_BASE_ADDR + PROFILE_REGION_SIZE).contains(&addr) {
            Some(self.profile[(addr - PROFILE_BASE_ADDR) as usize])
        } else {
            None
        }
    }
}

impl ControlTableCell {
    /// Overlay a validated image onto the live table -- raw byte copy,
    /// deliberately bypassing ro masks and field rules (`Image::parse`
    /// already gated the UB-critical bytes; everything else was rule-valid
    /// when saved). The identity block is skipped: model/fw must reflect the
    /// flashed firmware, never a stale saved image. Bringup-only, pre-IRQ;
    /// sole writer (the `seed_config_defaults` contract).
    pub fn overlay_persistent(&self, config: &[u8; CONFIG_LEN], profile: &[u8; PROFILE_LEN]) {
        let skip = config::addr::comms::ID as usize;
        let base = RegisterMap::base(self);
        // SAFETY: base points at the flat map (RegisterMap contract), both
        // regions lie inside it, and the fn doc pins the sole-writer window.
        unsafe {
            core::ptr::copy_nonoverlapping(
                config.as_ptr().add(skip),
                base.add(CONFIG_BASE_ADDR as usize + skip),
                CONFIG_LEN - skip,
            );
            core::ptr::copy_nonoverlapping(
                profile.as_ptr(),
                base.add(PROFILE_BASE_ADDR as usize),
                PROFILE_LEN,
            );
        }
    }
}

/// Boot-time load: parse both slots, overlay the wrapping-newest valid image
/// (board defaults, already seeded, stand when neither validates), and hand
/// back the store state for the impl. Bringup-only, pre-IRQ.
pub fn boot_overlay(table: &ControlTableCell, slot_a: &[u8], slot_b: &[u8]) -> BootPick {
    let a = Image::parse(slot_a);
    let b = Image::parse(slot_b);
    let (img, next_slot) = match (a, b) {
        (Some(a), Some(b)) => {
            if (a.seq.wrapping_sub(b.seq) as i16) > 0 {
                (Some(a), Slot::B)
            } else {
                (Some(b), Slot::A)
            }
        }
        (Some(a), None) => (Some(a), Slot::B),
        (None, b) => (b, Slot::A),
    };
    match img {
        Some(img) => {
            table.overlay_persistent(img.config, img.profile);
            BootPick {
                next_slot,
                next_seq: img.seq.wrapping_add(1),
                loaded: true,
            }
        }
        None => BootPick {
            next_slot: Slot::A,
            next_seq: 1,
            loaded: false,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::config::addr;
    use crate::{ConfigDefaults, RegionStorage};

    /// A plausible table snapshot: pattern data in rule-free fields, valid
    /// (zero) bytes in every enum/bool field.
    fn body() -> ([u8; CONFIG_LEN], [u8; PROFILE_LEN]) {
        let mut config = [0u8; CONFIG_LEN];
        config[addr::comms::ID as usize] = 7;
        config[addr::comms::RESPONSE_DEADLINE_US as usize] = 99;
        config[addr::identity::MODEL_NUMBER as usize] = 0xEE;
        let mut profile = [0u8; PROFILE_LEN];
        profile[0] = 0xAA;
        profile[PROFILE_LEN - 1] = 0x55;
        (config, profile)
    }

    fn image_of(seq: u16) -> [u8; IMAGE_LEN] {
        let (config, profile) = body();
        let mut img = [0u8; IMAGE_LEN];
        assemble(&mut img, seq, &config, &profile);
        img
    }

    #[test]
    fn image_round_trips() {
        let (config, profile) = body();
        let img = image_of(9);
        let parsed = Image::parse(&img).expect("valid image");
        assert_eq!(parsed.seq, 9);
        assert_eq!(parsed.config, &config);
        assert_eq!(parsed.profile, &profile);
    }

    #[test]
    fn parse_rejects_each_gate() {
        let img = image_of(1);
        assert!(Image::parse(&img[..IMAGE_LEN - 1]).is_none(), "short slot");
        for (i, name) in [(0, "magic"), (1, "version"), (4, "len"), (6, "crc")] {
            let mut bad = img;
            bad[i] ^= 0x01;
            assert!(Image::parse(&bad).is_none(), "{name} must gate");
        }
        // Erased flash (all 0xFF) is the common invalid slot.
        assert!(Image::parse(&[0xFF; IMAGE_LEN]).is_none());
        // Extra bytes past IMAGE_LEN (the rest of the page) are ignored.
        let mut page = [0xFF; 256];
        page[..IMAGE_LEN].copy_from_slice(&img);
        assert!(Image::parse(&page).is_some());
    }

    #[test]
    fn parse_rejects_disallowed_enum_byte() {
        // A CRC-valid image planting an out-of-range discriminant must not
        // parse -- overlay would construct an invalid enum (UB).
        let (mut config, profile) = body();
        config[addr::stall::STALL_RESPONSE as usize] = 2;
        let mut img = [0u8; IMAGE_LEN];
        assemble(&mut img, 1, &config, &profile);
        assert!(Image::parse(&img).is_none());
    }

    fn seeded_table() -> crate::ControlTableCell {
        let table = crate::ControlTableCell::new();
        table.seed_config_defaults(&ConfigDefaults {
            id: 1,
            ..Default::default()
        });
        table
    }

    #[test]
    fn boot_overlay_picks_the_newest_and_programs_the_other() {
        let table = seeded_table();
        let newer = {
            let (mut config, profile) = body();
            config[addr::comms::ID as usize] = 8;
            let mut img = [0u8; IMAGE_LEN];
            assemble(&mut img, 6, &config, &profile);
            img
        };
        let pick = boot_overlay(&table, &image_of(5), &newer);
        assert_eq!(
            pick,
            BootPick {
                next_slot: Slot::A,
                next_seq: 7,
                loaded: true
            }
        );
        assert_eq!(table.with(|t| t.config.comms.id), 8);
        assert_eq!(table.with(|t| t.profile.slots.words[0]), 0x00AA);
    }

    #[test]
    fn boot_overlay_seq_compare_wraps() {
        let table = seeded_table();
        let pick = boot_overlay(&table, &image_of(0xFFFF), &image_of(0));
        // 0 is wrapping-newer than 0xFFFF: slot B wins, A is next.
        assert_eq!(pick.next_slot, Slot::A);
        assert_eq!(pick.next_seq, 1);
    }

    #[test]
    fn boot_overlay_single_valid_slot_wins_either_side() {
        for (a, b, next) in [
            (&image_of(3)[..], &[0xFF; IMAGE_LEN][..], Slot::B),
            (&[0xFF; IMAGE_LEN][..], &image_of(3)[..], Slot::A),
        ] {
            let table = seeded_table();
            let pick = boot_overlay(&table, a, b);
            assert_eq!(
                (pick.next_slot, pick.next_seq, pick.loaded),
                (next, 4, true)
            );
            assert_eq!(table.with(|t| t.config.comms.id), 7);
        }
    }

    #[test]
    fn boot_overlay_without_valid_image_keeps_defaults() {
        let table = seeded_table();
        let pick = boot_overlay(&table, &[0xFF; IMAGE_LEN], &[0u8; IMAGE_LEN]);
        assert_eq!(
            pick,
            BootPick {
                next_slot: Slot::A,
                next_seq: 1,
                loaded: false
            }
        );
        assert_eq!(table.with(|t| t.config.comms.id), 1);
    }

    #[test]
    fn overlay_skips_the_identity_block() {
        let table = seeded_table();
        table.with_mut(|t| t.config.identity.model_number = 0x1234);
        boot_overlay(&table, &image_of(1), &[0xFF; IMAGE_LEN]);
        // The image carried 0x00EE at MODEL_NUMBER; the flashed firmware's
        // identity stands.
        assert_eq!(table.with(|t| t.config.identity.model_number), 0x1234);
        assert_eq!(table.with(|t| t.config.comms.id), 7, "comms overlaid");
    }
}
