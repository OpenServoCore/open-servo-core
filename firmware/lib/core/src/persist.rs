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
//!
//! The CALIB region persists as its own image (same header layout, magic
//! `b'K'`, own version and A/B seq) together with its tables: body = the
//! raw 256 B region ++ the position table's 256 host-written points i16 LE
//! (`pos_lut` module; the fixed last point is not stored), so the 776 B
//! image spans four pages per slot. One image, one CRC: calibration and
//! the tables it validates are written and loaded as one unit, so a save
//! never tears between them. Separate from the config image so that
//! identification rewrites calib without touching config, and a config
//! layout bump does not orphan a stored calibration.
//!
//! SAVE stores what the kernel applies: the array while LIVE, the identity
//! otherwise. Boot loads the points beside the calibration and validates
//! them against the stops it just overlaid: a table that fails, or is all
//! zero, is the identity. An image that does not load leaves the identity
//! too, under the CALIB_* reason; the plant stamp (which covers the
//! effective points) is what reports a lost or stale table as
//! `STAMP_MISMATCH`.

use control_table::RegisterMap;
use osc_protocol::crc::{osc_crc, osc_crc_continue};

use crate::data_state::ImageState;
use crate::pos_lut::{self, INTERVALS, state};
use crate::regions::{
    CALIB_BASE_ADDR, CALIB_REGION_SIZE, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, PROFILE_BASE_ADDR,
    PROFILE_REGION_SIZE, config,
};
use crate::{ControlTable, ControlTableCell, RegionStorage, Shared};

pub const CONFIG_LEN: usize = CONFIG_REGION_SIZE as usize;
pub const PROFILE_LEN: usize = PROFILE_REGION_SIZE as usize;
pub const CALIB_LEN: usize = CALIB_REGION_SIZE as usize;
pub const LUT_LEN: usize = 2 * INTERVALS;
pub const HEADER_LEN: usize = 8;
pub const IMAGE_LEN: usize = HEADER_LEN + CONFIG_LEN + PROFILE_LEN;
pub const CALIB_IMAGE_LEN: usize = HEADER_LEN + CALIB_LEN + LUT_LEN;

pub const IMAGE_MAGIC: u8 = b'C';
/// Bump on any CONFIG/PROFILE layout change; a mismatched image boots
/// `ImageState::Stale` (board defaults stand, closed loop gated) rather
/// than migrating.
pub const IMAGE_VERSION: u8 = 5;

pub const CALIB_IMAGE_MAGIC: u8 = b'K';
/// Bump on any CALIB layout or table grid change; independent of
/// [`IMAGE_VERSION`].
pub const CALIB_IMAGE_VERSION: u8 = 3;

/// Store failure (erase/program/verify); dispatch answers `hardware` (sec 5.3).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StoreError;

/// sec 9.4 persistence port. Both methods are cold and blocking: on flash,
/// `save` stalls the caller for the erase + program time (ms-scale) -- the
/// torque gate in dispatch is what makes that stall safe.
pub trait ConfigStore: Sync {
    /// Persist the two images (config+profile, calib+tables) in that
    /// order, each into its own A/B slot pair.
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
        calib: &[u8; CALIB_LEN],
        lut: &[i16; INTERVALS],
    ) -> Result<(), StoreError>;
    /// FACTORY (sec 9.5): invalidate every slot -- the erased store IS the
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
    /// `Loaded` = a valid image was overlaid; anything else leaves board
    /// defaults standing and says why (`data_state`).
    pub state: ImageState,
}

impl BootPick {
    const fn unloaded(state: ImageState) -> Self {
        Self {
            next_slot: Slot::A,
            next_seq: 1,
            state,
        }
    }
}

/// Every byte over the image length erased.
fn erased(slot: &[u8], len: usize) -> bool {
    slot.get(..len)
        .is_some_and(|s| s.iter().all(|&b| b == 0xFF))
}

/// A CRC-valid image of another layout version: sound bytes this firmware
/// cannot read. The version byte is under the CRC, so this is a whole-image
/// verdict, not a header guess; the body length is the header's own, since
/// another layout need not share this one's.
fn stale(slot: &[u8], magic: u8, version: u8) -> bool {
    let Some(h) = slot.get(..HEADER_LEN) else {
        return false;
    };
    let body_len = u16::from_le_bytes([h[4], h[5]]) as usize;
    let Some(bytes) = slot.get(..HEADER_LEN + body_len) else {
        return false;
    };
    bytes[0] == magic
        && bytes[1] != version
        && osc_crc_continue(osc_crc(&bytes[..6]), &bytes[HEADER_LEN..])
            == u16::from_le_bytes([bytes[6], bytes[7]])
}

/// Why neither slot parsed. A stale image beside a rotten one still says
/// the store held a real save once, so stale wins over corrupt. Cold boot
/// path shared by both images: one copy, not one per call site.
#[inline(never)]
fn unloaded_state(a: &[u8], b: &[u8], magic: u8, version: u8, body_len: usize) -> ImageState {
    let len = HEADER_LEN + body_len;
    if erased(a, len) && erased(b, len) {
        ImageState::Virgin
    } else if stale(a, magic, version) || stale(b, magic, version) {
        ImageState::Stale
    } else {
        ImageState::Corrupt
    }
}

/// Wrapping-newest of two validated slot images: the winner overlays, the
/// loser's slot takes the next SAVE.
fn pick_newest<T>(a: Option<T>, b: Option<T>, seq: impl Fn(&T) -> u16) -> (Option<T>, Slot) {
    match (a, b) {
        (Some(a), Some(b)) => {
            if (seq(&a).wrapping_sub(seq(&b)) as i16) > 0 {
                (Some(a), Slot::B)
            } else {
                (Some(b), Slot::A)
            }
        }
        (Some(a), None) => (Some(a), Slot::B),
        (None, b) => (b, Slot::A),
    }
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

/// Build the 8-byte header for a calib image (body = the raw CALIB region
/// ++ the position table points i16 LE).
pub fn calib_header(
    seq: u16,
    calib: &[u8; CALIB_LEN],
    lut_points: &[i16; INTERVALS],
) -> [u8; HEADER_LEN] {
    let mut h = [0u8; HEADER_LEN];
    h[0] = CALIB_IMAGE_MAGIC;
    h[1] = CALIB_IMAGE_VERSION;
    h[2..4].copy_from_slice(&seq.to_le_bytes());
    h[4..6].copy_from_slice(&((CALIB_LEN + LUT_LEN) as u16).to_le_bytes());
    let mut crc = osc_crc_continue(osc_crc(&h[..6]), calib);
    for c in lut_points {
        crc = osc_crc_continue(crc, &c.to_le_bytes());
    }
    h[6..8].copy_from_slice(&crc.to_le_bytes());
    h
}

/// Assemble a whole calib image in RAM - hosts, fakes, and tests; the chip
/// provider streams header + region + array across its four pages instead.
pub fn calib_assemble(
    out: &mut [u8; CALIB_IMAGE_LEN],
    seq: u16,
    calib: &[u8; CALIB_LEN],
    lut_points: &[i16; INTERVALS],
) {
    out[..HEADER_LEN].copy_from_slice(&calib_header(seq, calib, lut_points));
    out[HEADER_LEN..HEADER_LEN + CALIB_LEN].copy_from_slice(calib);
    for (d, c) in out[HEADER_LEN + CALIB_LEN..]
        .as_chunks_mut::<2>()
        .0
        .iter_mut()
        .zip(lut_points)
    {
        *d = c.to_le_bytes();
    }
}

/// A validated stored calib image; the region and point refs borrow the
/// slot bytes.
pub struct CalibImage<'a> {
    pub seq: u16,
    pub calib: &'a [u8; CALIB_LEN],
    pub lut_points: &'a [u8; LUT_LEN],
}

impl<'a> CalibImage<'a> {
    /// Validate a slot (extra bytes past `CALIB_IMAGE_LEN` - the rest of the
    /// fourth page - are ignored). Same gates as [`Image::parse`]; the
    /// allowed-rules pass covers CALIB enum/bool fields if any are ever
    /// added (today the region carries none, so it is vacuously true). The
    /// physics gate on the points (`pos_lut::validate`) runs at load,
    /// against the stops this same image carries.
    pub fn parse(slot: &'a [u8]) -> Option<CalibImage<'a>> {
        let bytes = slot.get(..CALIB_IMAGE_LEN)?;
        if bytes[0] != CALIB_IMAGE_MAGIC || bytes[1] != CALIB_IMAGE_VERSION {
            return None;
        }
        let seq = u16::from_le_bytes([bytes[2], bytes[3]]);
        if u16::from_le_bytes([bytes[4], bytes[5]]) != (CALIB_LEN + LUT_LEN) as u16 {
            return None;
        }
        let crc = u16::from_le_bytes([bytes[6], bytes[7]]);
        if osc_crc_continue(osc_crc(&bytes[..6]), &bytes[HEADER_LEN..]) != crc {
            return None;
        }
        let img = CalibImage {
            seq,
            calib: bytes[HEADER_LEN..HEADER_LEN + CALIB_LEN].try_into().ok()?,
            lut_points: bytes[HEADER_LEN + CALIB_LEN..].try_into().ok()?,
        };
        img.fields_allowed().then_some(img)
    }

    fn fields_allowed(&self) -> bool {
        <ControlTableCell as RegisterMap>::ALLOWED_RULES
            .iter()
            .all(|r| match self.persisted_byte(r.addr) {
                Some(b) => r.allowed.contains(&b),
                None => true,
            })
    }

    fn persisted_byte(&self, addr: u16) -> Option<u8> {
        (CALIB_BASE_ADDR..CALIB_BASE_ADDR + CALIB_REGION_SIZE)
            .contains(&addr)
            .then(|| self.calib[(addr - CALIB_BASE_ADDR) as usize])
    }
}

/// CONFIG ++ CALIB (contiguous from address 0): bit `a % 32` of word
/// `a / 32` set = a field covers address `a`.
const SEALED_END: usize = (CALIB_BASE_ADDR + CALIB_REGION_SIZE) as usize;
const FIELD_WORDS: [u32; SEALED_END / 32] = {
    assert!(CONFIG_BASE_ADDR == 0 && CALIB_BASE_ADDR == CONFIG_REGION_SIZE);
    let mut w = [0u32; SEALED_END / 32];
    let f = &ControlTable::FIELDS;
    let mut n = 0;
    while n < f.len() {
        let mut a = f[n].addr as usize;
        while a < f[n].addr as usize + f[n].width as usize && a < SEALED_END {
            w[a / 32] |= 1 << (a % 32);
            a += 1;
        }
        n += 1;
    }
    w
};

impl ControlTableCell {
    /// SAVE: every CONFIG and CALIB byte no field covers (skip, padding)
    /// to zero, so no earlier image's leftovers are sealed again. Nothing
    /// else writes or reads those bytes.
    #[inline(never)]
    pub fn zero_reserved_persistent(&self) {
        let base = RegisterMap::base(self);
        // black_box keeps the bit loop a loop: folded, it unrolls to one
        // store per reserved byte (+0.6 KB of flash)
        let words = core::hint::black_box(&FIELD_WORDS);
        for a in 0..SEALED_END {
            if words[a / 32] & (1 << (a % 32)) == 0 {
                // SAFETY: a < SEALED_END, inside the flat map (RegisterMap
                // contract); no field aliases the byte.
                unsafe { base.add(a).write(0) };
            }
        }
    }

    /// Overlay a validated image onto the live table -- raw byte copy,
    /// deliberately bypassing ro masks and field rules (`Image::parse`
    /// already gated the UB-critical bytes; everything else was rule-valid
    /// when saved). The identity front (everything before `id`) is skipped:
    /// model/fw must reflect the
    /// flashed firmware, never a stale saved image. Bringup-only, pre-IRQ;
    /// sole writer (the `seed_config_defaults` contract).
    pub fn overlay_persistent(&self, config: &[u8; CONFIG_LEN], profile: &[u8; PROFILE_LEN]) {
        let skip = config::addr::common::ID as usize;
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

    /// Overlay a validated calib image -- whole region, nothing skipped: every
    /// field is data. RO board facts (`CalibSense`, `CalibSenseExt`) must be
    /// re-seeded by install AFTER this overlay so board data always wins over
    /// a stale image. Bringup-only, pre-IRQ; sole writer.
    pub fn overlay_persistent_calib(&self, calib: &[u8; CALIB_LEN]) {
        let base = RegisterMap::base(self);
        // SAFETY: base points at the flat map (RegisterMap contract), the
        // region lies inside it, and the fn doc pins the sole-writer window.
        unsafe {
            core::ptr::copy_nonoverlapping(
                calib.as_ptr(),
                base.add(CALIB_BASE_ADDR as usize),
                CALIB_LEN,
            );
        }
    }
}

/// Boot-time load: parse both slots, overlay the wrapping-newest valid image
/// (board defaults, already seeded, stand when neither validates), and hand
/// back the store state for the impl. Bringup-only, pre-IRQ.
pub fn boot_overlay(table: &ControlTableCell, slot_a: &[u8], slot_b: &[u8]) -> BootPick {
    let (img, next_slot) = pick_newest(Image::parse(slot_a), Image::parse(slot_b), |i| i.seq);
    match img {
        Some(img) => {
            table.overlay_persistent(img.config, img.profile);
            BootPick {
                next_slot,
                next_seq: img.seq.wrapping_add(1),
                state: ImageState::Loaded,
            }
        }
        None => BootPick::unloaded(unloaded_state(
            slot_a,
            slot_b,
            IMAGE_MAGIC,
            IMAGE_VERSION,
            CONFIG_LEN + PROFILE_LEN,
        )),
    }
}

/// Boot-time calib load: same A/B rule as [`boot_overlay`], own slots and
/// seq. The image's points land in the array beside its calibration and go
/// LIVE only if they validate against the stops just overlaid and correct
/// something (the all-zero image SAVE persists for the identity reads
/// IDENTITY); otherwise the array returns to the identity, and the stamp
/// checkpoint reports the loss. Bringup-only, pre-IRQ; caller re-seeds RO
/// board facts after.
pub fn boot_overlay_calib(shared: &Shared, slot_a: &[u8], slot_b: &[u8]) -> BootPick {
    let (img, next_slot) = pick_newest(CalibImage::parse(slot_a), CalibImage::parse(slot_b), |i| {
        i.seq
    });
    match img {
        Some(img) => {
            shared.table.overlay_persistent_calib(img.calib);
            let (raw_min, raw_max) = shared
                .table
                .with(|t| (t.calib.pot.raw_min, t.calib.pot.raw_max));
            let live = shared.with_pos_lut_mut(|k| {
                for (d, s) in k.iter_mut().zip(img.lut_points.as_chunks::<2>().0) {
                    *d = i16::from_le_bytes(*s);
                }
                match pos_lut::validate(k, raw_min, raw_max) {
                    Ok(()) => k.iter().any(|&c| c != 0),
                    Err(_) => {
                        k.fill(0);
                        false
                    }
                }
            });
            shared.table.with_mut(|t| {
                t.control.pos_lut.pos_lut_state = if live { state::LIVE } else { state::IDENTITY }
            });
            BootPick {
                next_slot,
                next_seq: img.seq.wrapping_add(1),
                state: ImageState::Loaded,
            }
        }
        None => BootPick::unloaded(unloaded_state(
            slot_a,
            slot_b,
            CALIB_IMAGE_MAGIC,
            CALIB_IMAGE_VERSION,
            CALIB_LEN + LUT_LEN,
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::config::addr;
    use crate::{ConfigDefaults, CurrentDefaults, RegionStorage};

    /// A plausible table snapshot: pattern data in rule-free fields, valid
    /// (zero) bytes in every enum/bool field.
    fn body() -> ([u8; CONFIG_LEN], [u8; PROFILE_LEN]) {
        let mut config = [0u8; CONFIG_LEN];
        config[addr::common::ID as usize] = 7;
        config[addr::common::RESPONSE_DEADLINE_US as usize] = 99;
        config[addr::common::MODEL_NUMBER as usize] = 0xEE;
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
        config[addr::limits::STALL_RESPONSE as usize] = 2;
        let mut img = [0u8; IMAGE_LEN];
        assemble(&mut img, 1, &config, &profile);
        assert!(Image::parse(&img).is_none());
    }

    fn seed(table: &ControlTableCell) {
        table.seed_config_defaults(
            &ConfigDefaults {
                id: 1,
                ..Default::default()
            },
            &CurrentDefaults::from_sense(33, 15_000, 3300),
            1200,
        );
    }

    fn seeded_table() -> ControlTableCell {
        let table = ControlTableCell::new();
        seed(&table);
        table
    }

    fn seeded_servo() -> Shared {
        let sh = Shared::new();
        seed(&sh.table);
        sh
    }

    #[test]
    fn boot_overlay_picks_the_newest_and_programs_the_other() {
        let table = seeded_table();
        let newer = {
            let (mut config, profile) = body();
            config[addr::common::ID as usize] = 8;
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
                state: ImageState::Loaded
            }
        );
        assert_eq!(table.with(|t| t.config.common.id), 8);
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
                (pick.next_slot, pick.next_seq, pick.state),
                (next, 4, ImageState::Loaded)
            );
            assert_eq!(table.with(|t| t.config.common.id), 7);
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
                state: ImageState::Corrupt
            }
        );
        assert_eq!(table.with(|t| t.config.common.id), 1);
    }

    #[test]
    fn overlay_skips_the_identity_front() {
        let table = seeded_table();
        table.with_mut(|t| t.config.common.model_number = 0x1234);
        boot_overlay(&table, &image_of(1), &[0xFF; IMAGE_LEN]);
        // The image carried 0x00EE at MODEL_NUMBER; the flashed firmware's
        // identity stands.
        assert_eq!(table.with(|t| t.config.common.model_number), 0x1234);
        assert_eq!(table.with(|t| t.config.common.id), 7, "comms overlaid");
    }

    use crate::pos_lut::POINTS;

    const STOPS: (u16, u16) = (209, 3849);

    /// Pattern data in rule-free fields under the mg90-a stops.
    fn calib_body() -> [u8; CALIB_LEN] {
        use crate::regions::calib::addr;
        let mut calib = [0u8; CALIB_LEN];
        calib[(addr::pot::RAW_MIN - CALIB_BASE_ADDR) as usize..][..2]
            .copy_from_slice(&STOPS.0.to_le_bytes());
        calib[(addr::pot::RAW_MAX - CALIB_BASE_ADDR) as usize..][..2]
            .copy_from_slice(&STOPS.1.to_le_bytes());
        calib[(addr::sense::TICK_HZ - CALIB_BASE_ADDR) as usize] = 0x20;
        calib[(addr::motor::R_Q12 - CALIB_BASE_ADDR) as usize..][..2]
            .copy_from_slice(&13800u16.to_le_bytes());
        calib[CALIB_LEN - 1] = 0x55;
        calib
    }

    /// A few points inside the stops, valid against them.
    fn lut_body() -> [i16; INTERVALS] {
        let mut k = [0i16; INTERVALS];
        k[20] = 3;
        k[21] = 5;
        k[100] = -7;
        k
    }

    fn calib_image_of(seq: u16) -> [u8; CALIB_IMAGE_LEN] {
        let mut img = [0u8; CALIB_IMAGE_LEN];
        calib_assemble(&mut img, seq, &calib_body(), &lut_body());
        img
    }

    fn array(sh: &Shared) -> [i16; POINTS] {
        sh.with_pos_lut(|k| *k)
    }

    fn pos_lut_state(sh: &Shared) -> u8 {
        sh.table.with(|t| t.control.pos_lut.pos_lut_state)
    }

    #[test]
    fn calib_image_round_trips() {
        let img = calib_image_of(9);
        let parsed = CalibImage::parse(&img).expect("valid image");
        assert_eq!(parsed.seq, 9);
        assert_eq!(parsed.calib, &calib_body());
        for (bytes, c) in parsed.lut_points.as_chunks::<2>().0.iter().zip(&lut_body()) {
            assert_eq!(i16::from_le_bytes(*bytes), *c);
        }
    }

    #[test]
    fn calib_parse_rejects_each_gate() {
        let img = calib_image_of(1);
        assert!(
            CalibImage::parse(&img[..CALIB_IMAGE_LEN - 1]).is_none(),
            "short slot"
        );
        for (i, name) in [(0, "magic"), (1, "version"), (4, "len"), (6, "crc")] {
            let mut bad = img;
            bad[i] ^= 0x01;
            assert!(CalibImage::parse(&bad).is_none(), "{name} must gate");
        }
        let mut torn = img;
        torn[CALIB_IMAGE_LEN - 1] ^= 0x80;
        assert!(
            CalibImage::parse(&torn).is_none(),
            "the last point is under the crc"
        );
        // A config image in a calib slot fails on magic, not by accident.
        assert!(CalibImage::parse(&image_of(1)).is_none());
        assert!(CalibImage::parse(&[0xFF; CALIB_IMAGE_LEN]).is_none());
        // Extra bytes past CALIB_IMAGE_LEN (rest of the fourth page) ignored.
        let mut pages = [0xFF; 1024];
        pages[..CALIB_IMAGE_LEN].copy_from_slice(&img);
        assert!(CalibImage::parse(&pages).is_some());
    }

    #[test]
    fn calib_boot_overlay_picks_newest_and_lands_fields_and_points() {
        let sh = seeded_servo();
        let newer = {
            let mut calib = calib_body();
            calib[(crate::regions::calib::addr::motor::R_Q12 - CALIB_BASE_ADDR) as usize..][..2]
                .copy_from_slice(&14000u16.to_le_bytes());
            let mut k = lut_body();
            k[100] = -9;
            let mut img = [0u8; CALIB_IMAGE_LEN];
            calib_assemble(&mut img, 6, &calib, &k);
            (img, k)
        };
        let pick = boot_overlay_calib(&sh, &calib_image_of(5), &newer.0);
        assert_eq!(
            pick,
            BootPick {
                next_slot: Slot::A,
                next_seq: 7,
                state: ImageState::Loaded
            }
        );
        assert_eq!(sh.table.with(|t| t.calib.motor.r_q12), 14000);
        assert_eq!(sh.table.with(|t| t.calib.sense.tick_hz), 0x20);
        assert_eq!(pos_lut_state(&sh), state::LIVE);
        assert_eq!(array(&sh)[..INTERVALS], newer.1);
        assert_eq!(array(&sh)[INTERVALS], 0, "the fixed last point");
    }

    #[test]
    fn calib_boot_overlay_seq_compare_wraps() {
        let sh = seeded_servo();
        let pick = boot_overlay_calib(&sh, &calib_image_of(0xFFFF), &calib_image_of(0));
        assert_eq!(pick.next_slot, Slot::A);
        assert_eq!(pick.next_seq, 1);
    }

    #[test]
    fn calib_boot_overlay_single_valid_slot_wins_either_side() {
        for (a, b, next) in [
            (
                &calib_image_of(3)[..],
                &[0xFF; CALIB_IMAGE_LEN][..],
                Slot::B,
            ),
            (
                &[0xFF; CALIB_IMAGE_LEN][..],
                &calib_image_of(3)[..],
                Slot::A,
            ),
        ] {
            let sh = seeded_servo();
            let pick = boot_overlay_calib(&sh, a, b);
            assert_eq!(
                (pick.next_slot, pick.next_seq, pick.state),
                (next, 4, ImageState::Loaded)
            );
            assert_eq!(sh.table.with(|t| t.calib.motor.r_q12), 13800);
            assert_eq!(pos_lut_state(&sh), state::LIVE);
        }
    }

    /// The stops in the image reject its own points (a table left beside a
    /// re-cal): the calibration lands, the identity runs, and the slot
    /// still alternates. The all-zero table SAVE persists for the identity
    /// is sound, loaded, and nothing to apply.
    #[test]
    fn calib_boot_overlay_rejects_points_against_its_stops_and_stays_identity() {
        use crate::regions::calib::addr::pot::RAW_MAX;
        let sh = seeded_servo();
        let mut calib = calib_body();
        calib[(RAW_MAX - CALIB_BASE_ADDR) as usize..][..2].copy_from_slice(&1000u16.to_le_bytes());
        let mut img = [0u8; CALIB_IMAGE_LEN];
        calib_assemble(&mut img, 2, &calib, &lut_body());
        let pick = boot_overlay_calib(&sh, &img, &[0xFF; CALIB_IMAGE_LEN]);
        assert_eq!(
            pick,
            BootPick {
                next_slot: Slot::B,
                next_seq: 3,
                state: ImageState::Loaded
            }
        );
        assert_eq!(sh.table.with(|t| t.calib.pot.raw_max), 1000);
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
        assert_eq!(array(&sh), [0; POINTS]);

        let sh = seeded_servo();
        let mut img = [0u8; CALIB_IMAGE_LEN];
        calib_assemble(&mut img, 5, &calib_body(), &[0; INTERVALS]);
        let pick = boot_overlay_calib(&sh, &[0xFF; CALIB_IMAGE_LEN], &img);
        assert_eq!((pick.next_slot, pick.next_seq), (Slot::A, 6));
        assert_eq!(sh.table.with(|t| t.calib.motor.r_q12), 13800);
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
    }

    #[test]
    fn calib_boot_overlay_without_valid_image_keeps_seeds() {
        let sh = seeded_servo();
        let table = &sh.table;
        table.seed_calib_sense(
            &crate::regions::calib::CalibSense {
                shunt_r_mohm: 0,
                gain_milli: 0,
                vmotor_div_top: 0,
                vmotor_div_bot: 0,
                vdd_mv: 0,
                tick_hz: 20000,
                i_window_min_ticks: 0,
                v_window_min_ticks: 0,
            },
            &crate::regions::calib::CalibSenseExt {
                vbus_div_top_ohm: 0,
                vbus_div_bot_ohm: 0,
                ntc_pullup_ohm: 0,
                ntc_r25_ohm: 0,
                ntc_beta: 0,
                vmotor_bias_nom_counts: 0,
                rail_drop_mv: 0,
            },
        );
        let pick = boot_overlay_calib(&sh, &[0xFF; CALIB_IMAGE_LEN], &[0u8; CALIB_IMAGE_LEN]);
        assert_eq!(
            pick,
            BootPick {
                next_slot: Slot::A,
                next_seq: 1,
                state: ImageState::Corrupt
            }
        );
        assert_eq!(table.with(|t| t.calib.sense.tick_hz), 20000);
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
        assert_eq!(array(&sh), [0; POINTS]);
    }

    #[test]
    fn seeded_identity_survives_a_boot_overlay() {
        // Boot order: seed identity, then overlay a valid saved image. The
        // image's identity front (0x00EE at MODEL_NUMBER) is skipped, so the
        // firmware's stamped identity stands while comms overlay.
        let table = seeded_table();
        table.seed_identity(0x0101, 3);
        boot_overlay(&table, &image_of(1), &[0xFF; IMAGE_LEN]);
        table.with(|t| {
            assert_eq!(t.config.common.model_number, 0x0101);
            assert_eq!(t.config.common.hardware_revision, 3);
            assert_eq!(t.config.common.firmware_version, crate::FIRMWARE_VERSION);
            assert_eq!(t.config.common.id, 7, "comms overlaid");
        });
    }

    /// The image re-sealed under another layout version: CRC-valid, so it
    /// is a real save this firmware cannot read.
    fn reseal_version(img: &mut [u8], version: u8) {
        img[1] = version;
        let crc = osc_crc_continue(osc_crc(&img[..6]), &img[HEADER_LEN..]);
        img[6..8].copy_from_slice(&crc.to_le_bytes());
    }

    #[test]
    fn boot_classifies_erased_as_virgin_and_garbage_as_corrupt() {
        let erased = [0xFF; IMAGE_LEN];
        let zeros = [0u8; IMAGE_LEN];
        let mut torn = image_of(1);
        torn[HEADER_LEN + 3] ^= 0x80;
        for (a, b, want) in [
            (&erased[..], &erased[..], ImageState::Virgin),
            (&erased[..], &zeros[..], ImageState::Corrupt),
            (&torn[..], &erased[..], ImageState::Corrupt),
            (&torn[..], &zeros[..], ImageState::Corrupt),
        ] {
            let table = seeded_table();
            let pick = boot_overlay(&table, a, b);
            assert_eq!(pick, BootPick::unloaded(want));
            assert_eq!(table.with(|t| t.config.common.id), 1, "defaults stand");
        }
        // One valid slot loads whatever sits beside it.
        let table = seeded_table();
        assert_eq!(
            boot_overlay(&table, &torn, &image_of(2)).state,
            ImageState::Loaded
        );
        // The same verdicts for the calib image, a tear in the points
        // included; the identity stands under each.
        let calib_erased = [0xFF; CALIB_IMAGE_LEN];
        let mut calib_torn = calib_image_of(1);
        calib_torn[HEADER_LEN] ^= 0x01;
        let mut lut_torn = calib_image_of(1);
        lut_torn[HEADER_LEN + CALIB_LEN + 40] ^= 0x01;
        for (a, b, want) in [
            (&calib_erased[..], &calib_erased[..], ImageState::Virgin),
            (&calib_torn[..], &calib_erased[..], ImageState::Corrupt),
            (&lut_torn[..], &calib_erased[..], ImageState::Corrupt),
        ] {
            let sh = seeded_servo();
            assert_eq!(boot_overlay_calib(&sh, a, b), BootPick::unloaded(want));
            assert_eq!(pos_lut_state(&sh), state::IDENTITY);
            assert_eq!(array(&sh), [0; POINTS]);
        }
        let sh = seeded_servo();
        assert_eq!(
            boot_overlay_calib(&sh, &lut_torn, &calib_image_of(2)).state,
            ImageState::Loaded,
            "one valid slot loads whatever sits beside it"
        );
        assert_eq!(pos_lut_state(&sh), state::LIVE);
    }

    #[test]
    fn version_bump_is_stale() {
        let mut old = image_of(4);
        reseal_version(&mut old, IMAGE_VERSION.wrapping_add(1));
        assert!(
            Image::parse(&old).is_none(),
            "another version never overlays"
        );
        let erased = [0xFF; IMAGE_LEN];
        let mut torn = image_of(1);
        torn[HEADER_LEN] ^= 0x01;
        for (a, b) in [
            (&old[..], &erased[..]),
            (&erased[..], &old[..]),
            (&old[..], &torn[..]),
        ] {
            let table = seeded_table();
            let pick = boot_overlay(&table, a, b);
            assert_eq!(pick, BootPick::unloaded(ImageState::Stale));
            assert_eq!(table.with(|t| t.config.common.id), 1, "defaults stand");
        }
        // A newer valid image beside a stale one loads as usual.
        let table = seeded_table();
        assert_eq!(
            boot_overlay(&table, &old, &image_of(1)),
            BootPick {
                next_slot: Slot::A,
                next_seq: 2,
                state: ImageState::Loaded
            }
        );
        // A re-sealed CRC is what makes it stale: the same bytes with the
        // old CRC are corrupt.
        let mut unsealed = image_of(4);
        unsealed[1] = IMAGE_VERSION.wrapping_add(1);
        assert_eq!(
            boot_overlay(&seeded_table(), &unsealed, &erased).state,
            ImageState::Corrupt
        );
        // Calib: same rule.
        let mut old_calib = calib_image_of(2);
        reseal_version(&mut old_calib, CALIB_IMAGE_VERSION.wrapping_add(1));
        let sh = seeded_servo();
        assert_eq!(
            boot_overlay_calib(&sh, &old_calib, &[0xFF; CALIB_IMAGE_LEN]).state,
            ImageState::Stale
        );
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
    }

    /// A CALIB image SAVEd by the resistance-anchor firmware (anchor r0
    /// 4170 at 26.5 C, slope 1602, mu 7670 behind it, zeros where the
    /// thermometer block now sits) loads whole: r0, table, kinematics and
    /// stamp land, the anchor's bytes fall in reserved space, and the zero
    /// model reads the sentinel until the host writes one.
    #[test]
    fn anchor_era_calib_image_loads_with_the_thermometer_unset() {
        use crate::estimator::thermal::{Sample, UNSET_CC};
        use crate::estimator::{NtcCfg, ThermCfg, WindingTherm};
        use crate::regions::calib::addr::{kinematics, stamp, winding};
        let mut old = calib_image_of(7);
        let mut put = |addr: u16, v: u16| {
            let at = HEADER_LEN + (addr - CALIB_BASE_ADDR) as usize;
            old[at..at + 2].copy_from_slice(&v.to_le_bytes());
        };
        for (i, v) in [4170, 2650, 1602, 7670].into_iter().enumerate() {
            put(winding::R0_Q12 + 2 * i as u16, v);
        }
        put(kinematics::GEAR_RATIO_CENTI, 30805);
        put(stamp::PLANT_STAMP, 0xBEEF);
        reseal_version(&mut old, 3);
        let sh = seeded_servo();
        assert_eq!(
            boot_overlay_calib(&sh, &old, &[0xFF; CALIB_IMAGE_LEN]).state,
            ImageState::Loaded
        );
        assert_eq!(pos_lut_state(&sh), state::LIVE);
        let th = sh.table.with(|t| {
            assert_eq!(t.calib.winding.r0_q12, 4170);
            assert_eq!(t.calib.motor.r_q12, 13800);
            assert_eq!(t.calib.kinematics.gear_ratio_centi, 30805);
            assert_eq!(t.calib.stamp.plant_stamp, 0xBEEF);
            t.calib.thermal
        });
        let cfg = ThermCfg {
            alpha_q24: th.th_alpha_q24,
            g_q016: th.th_g_q016,
            mu_q016: th.th_mu_q016,
            r_cold_q12: 4170,
            ntc: NtcCfg {
                raw_ref: th.ntc_raw_ref,
                t_ref_cc: th.ntc_t_ref_cc,
                k1_q88: th.ntc_k1_q88,
                k2_q24: th.ntc_k2_q24,
            },
            ..Default::default()
        };
        let seated = Sample {
            v_mean: Some(1200),
            i_meas: Some(400),
            pos_counts: 2000,
            ntc_raw: 2048,
            ..Default::default()
        };
        let mut therm = WindingTherm::new();
        for _ in 0..256 {
            assert_eq!(therm.step(&seated, &cfg), UNSET_CC);
        }
    }

    /// The layout before the tables joined the image: the region alone as
    /// the body, in a slot that then held two pages. Sound bytes of another
    /// length are stale, not corrupt.
    #[test]
    fn previous_calib_layout_is_stale() {
        let mut old = [0xFF; CALIB_IMAGE_LEN];
        old[0] = CALIB_IMAGE_MAGIC;
        old[1] = 2;
        old[2..4].copy_from_slice(&4u16.to_le_bytes());
        old[4..6].copy_from_slice(&(CALIB_LEN as u16).to_le_bytes());
        old[HEADER_LEN..HEADER_LEN + CALIB_LEN].copy_from_slice(&calib_body());
        let crc = osc_crc_continue(osc_crc(&old[..6]), &old[HEADER_LEN..HEADER_LEN + CALIB_LEN]);
        old[6..8].copy_from_slice(&crc.to_le_bytes());
        let sh = seeded_servo();
        assert_eq!(
            boot_overlay_calib(&sh, &old, &[0xFF; CALIB_IMAGE_LEN]),
            BootPick::unloaded(ImageState::Stale)
        );
        assert_eq!(sh.table.with(|t| t.calib.motor.r_q12), 0, "seeds stand");
        assert_eq!(pos_lut_state(&sh), state::IDENTITY);
        // the body length is under the crc: a length claim alone is corrupt
        old[4] ^= 0x01;
        assert_eq!(
            boot_overlay_calib(&seeded_servo(), &old, &[0xFF; CALIB_IMAGE_LEN]).state,
            ImageState::Corrupt
        );
    }
}
