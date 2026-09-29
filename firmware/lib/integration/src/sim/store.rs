//! RAM-backed [`ConfigStore`] fake: page-sized slots holding real images
//! through the core codec, so a rebuilt servo's boot overlay exercises the
//! same parse/pick path the chip takes. `Shared::seed_store` wants
//! `&'static`, so tests obtain instances via [`RamStore::leak`]; the interior
//! `Mutex` keeps the same reference inspectable (and shareable across a
//! rebuilt `Sim`, modeling a reboot with flash intact).

use std::sync::Mutex;

use control_table::RegisterFile;
use osc_protocol::crc::{osc_crc, osc_crc_continue};
use osc_servo_core::persist::{
    self, CALIB_IMAGE_LEN, CALIB_IMAGE_VERSION, CALIB_LEN, CONFIG_LEN, HEADER_LEN, IMAGE_LEN,
    IMAGE_VERSION, PROFILE_LEN, Slot, StoreError,
};
use osc_servo_core::pos_lut::INTERVALS;
use osc_servo_core::regions::{
    CALIB_BASE_ADDR, CALIB_REGION_SIZE, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE, PROFILE_BASE_ADDR,
    PROFILE_REGION_SIZE,
};
use osc_servo_core::{ConfigStore, Shared};

const ERASED: [u8; IMAGE_LEN] = [0xFF; IMAGE_LEN];
const CALIB_ERASED: [u8; CALIB_IMAGE_LEN] = [0xFF; CALIB_IMAGE_LEN];
/// The chip's fast-erase page: what one program cycle lands.
const PAGE: usize = 256;

/// Which persisted image an injection targets.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Kind {
    Config,
    Calib,
}

/// Where the next save loses power.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Tear {
    /// After the CONFIG image lands: the new config beside the old calib.
    AfterConfig,
    /// Partway through the CALIB program: the slot's first page holds the
    /// new image's front, the rest is erased (the cut came after the next
    /// page's erase, before its write).
    MidCalib,
}

fn idx(slot: Slot) -> usize {
    match slot {
        Slot::A => 0,
        Slot::B => 1,
    }
}

/// Re-seal an assembled image under another layout version: CRC-valid, so
/// boot classifies it stale rather than corrupt.
fn reseal_version(img: &mut [u8], version: u8) {
    img[1] = version;
    let crc = osc_crc_continue(osc_crc(&img[..6]), &img[HEADER_LEN..]);
    img[6..8].copy_from_slice(&crc.to_le_bytes());
}

#[derive(Default)]
struct Inner {
    slots: [Option<[u8; IMAGE_LEN]>; 2],
    calib_slots: [Option<[u8; CALIB_IMAGE_LEN]>; 2],
    next_slot: usize,
    next_seq: u16,
    calib_next_slot: usize,
    calib_next_seq: u16,
    fail: bool,
    tear: Option<Tear>,
    saves: usize,
    wipes: usize,
}

pub struct RamStore {
    inner: Mutex<Inner>,
}

impl RamStore {
    pub fn leak() -> &'static RamStore {
        Box::leak(Box::new(RamStore {
            inner: Mutex::new(Inner {
                next_seq: 1,
                calib_next_seq: 1,
                ..Inner::default()
            }),
        }))
    }

    /// Boot-time load, mirroring the chip provider: overlay the newest valid
    /// config and calib images (the position table beside the calib), prime
    /// both A/B states, and publish the data state the verdicts make. Called
    /// by `SimServo::build` before the bus reads the table's comms block; the
    /// caller re-seeds RO calib sense facts after (board data wins).
    pub fn boot_load(&self, shared: &Shared) {
        let mut g = self.inner.lock().unwrap();
        let a = g.slots[0].unwrap_or(ERASED);
        let b = g.slots[1].unwrap_or(ERASED);
        let pick = persist::boot_overlay(&shared.table, &a, &b);
        g.next_slot = idx(pick.next_slot);
        g.next_seq = pick.next_seq;
        let a = g.calib_slots[0].unwrap_or(CALIB_ERASED);
        let b = g.calib_slots[1].unwrap_or(CALIB_ERASED);
        let calib_pick = persist::boot_overlay_calib(shared, &a, &b);
        g.calib_next_slot = idx(calib_pick.next_slot);
        g.calib_next_seq = calib_pick.next_seq;
        shared.publish_data_state(pick.state, calib_pick.state);
    }

    /// A bench SAVE without the wire (sec 9.4): the live CONFIG, PROFILE and
    /// CALIB regions and the position table array land in the store, so the
    /// next boot overlays them back and FACTORY wipes them.
    pub fn save_table(&self, shared: &Shared) {
        let table = &shared.table;
        let config: &[u8; CONFIG_LEN] =
            RegisterFile::read(table, CONFIG_BASE_ADDR, CONFIG_REGION_SIZE)
                .ok()
                .and_then(|s| s.try_into().ok())
                .expect("whole CONFIG region");
        let profile: &[u8; PROFILE_LEN] =
            RegisterFile::read(table, PROFILE_BASE_ADDR, PROFILE_REGION_SIZE)
                .ok()
                .and_then(|s| s.try_into().ok())
                .expect("whole PROFILE region");
        let calib: &[u8; CALIB_LEN] = RegisterFile::read(table, CALIB_BASE_ADDR, CALIB_REGION_SIZE)
            .ok()
            .and_then(|s| s.try_into().ok())
            .expect("whole CALIB region");
        shared.with_pos_lut(|k| {
            let lut = k.first_chunk().expect("the host-written points");
            self.save(config, profile, calib, lut).expect("store save");
        });
    }

    /// Arm every subsequent save/wipe to fail (the chip's readback-verify
    /// miss).
    pub fn set_fail(&self, fail: bool) {
        self.inner.lock().unwrap().fail = fail;
    }

    /// The next save loses power at `at`. One-shot.
    pub fn tear(&self, at: Tear) {
        self.inner.lock().unwrap().tear = Some(at);
    }

    pub fn saves(&self) -> usize {
        self.inner.lock().unwrap().saves
    }

    pub fn wipes(&self) -> usize {
        self.inner.lock().unwrap().wipes
    }

    /// The slot's stored config image, if any (a copy - tests parse it at
    /// leisure).
    pub fn slot(&self, slot: Slot) -> Option<[u8; IMAGE_LEN]> {
        self.inner.lock().unwrap().slots[idx(slot)]
    }

    /// The slot's stored calib image, if any (a copy).
    pub fn calib_slot(&self, slot: Slot) -> Option<[u8; CALIB_IMAGE_LEN]> {
        self.inner.lock().unwrap().calib_slots[idx(slot)]
    }

    /// Plant a calib image (identity tables) directly - tests modeling
    /// out-of-band flash content (a stale image carrying bytes the wire
    /// could never write).
    pub fn inject_calib(&self, slot: Slot, seq: u16, calib: &[u8; CALIB_LEN]) {
        let mut img = [0u8; CALIB_IMAGE_LEN];
        persist::calib_assemble(&mut img, seq, calib, &[0; INTERVALS]);
        self.inner.lock().unwrap().calib_slots[idx(slot)] = Some(img);
    }

    /// Erase both slots of one image: a servo that lost that image alone.
    pub fn erase(&self, kind: Kind) {
        let mut g = self.inner.lock().unwrap();
        match kind {
            Kind::Config => g.slots = [None, None],
            Kind::Calib => g.calib_slots = [None, None],
        }
    }

    /// Overwrite the slot with bytes that parse under no version (flash
    /// rot, a torn program): boot classifies the image corrupt unless the
    /// other slot still holds a valid save.
    pub fn corrupt_slot(&self, kind: Kind, slot: Slot) {
        let mut g = self.inner.lock().unwrap();
        match kind {
            Kind::Config => g.slots[idx(slot)] = Some([0x5A; IMAGE_LEN]),
            Kind::Calib => g.calib_slots[idx(slot)] = Some([0x5A; CALIB_IMAGE_LEN]),
        }
    }

    /// Re-seal the slot's image (an erased slot gets a zero body) under the
    /// next layout version: a save from another firmware, CRC-valid and
    /// unreadable, so boot classifies the image stale.
    pub fn stale_slot(&self, kind: Kind, slot: Slot) {
        let mut g = self.inner.lock().unwrap();
        match kind {
            Kind::Config => {
                let mut img = g.slots[idx(slot)].unwrap_or_else(|| {
                    let mut img = [0u8; IMAGE_LEN];
                    persist::assemble(&mut img, 1, &[0; CONFIG_LEN], &[0; PROFILE_LEN]);
                    img
                });
                reseal_version(&mut img, IMAGE_VERSION.wrapping_add(1));
                g.slots[idx(slot)] = Some(img);
            }
            Kind::Calib => {
                let mut img = g.calib_slots[idx(slot)].unwrap_or_else(|| {
                    let mut img = [0u8; CALIB_IMAGE_LEN];
                    persist::calib_assemble(&mut img, 1, &[0; CALIB_LEN], &[0; INTERVALS]);
                    img
                });
                reseal_version(&mut img, CALIB_IMAGE_VERSION.wrapping_add(1));
                g.calib_slots[idx(slot)] = Some(img);
            }
        }
    }
}

impl ConfigStore for RamStore {
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
        calib: &[u8; CALIB_LEN],
        lut: &[i16; INTERVALS],
    ) -> Result<(), StoreError> {
        let mut g = self.inner.lock().unwrap();
        if g.fail {
            return Err(StoreError);
        }
        let tear = g.tear.take();
        let mut img = [0u8; IMAGE_LEN];
        persist::assemble(&mut img, g.next_seq, config, profile);
        let slot = g.next_slot;
        g.slots[slot] = Some(img);
        g.next_slot ^= 1;
        g.next_seq = g.next_seq.wrapping_add(1);
        if tear == Some(Tear::AfterConfig) {
            return Err(StoreError);
        }
        let mut img = [0u8; CALIB_IMAGE_LEN];
        persist::calib_assemble(&mut img, g.calib_next_seq, calib, lut);
        let slot = g.calib_next_slot;
        if tear == Some(Tear::MidCalib) {
            let mut torn = CALIB_ERASED;
            torn[..PAGE].copy_from_slice(&img[..PAGE]);
            g.calib_slots[slot] = Some(torn);
            return Err(StoreError);
        }
        g.calib_slots[slot] = Some(img);
        g.calib_next_slot ^= 1;
        g.calib_next_seq = g.calib_next_seq.wrapping_add(1);
        g.saves += 1;
        Ok(())
    }

    fn wipe(&self) -> Result<(), StoreError> {
        let mut g = self.inner.lock().unwrap();
        if g.fail {
            return Err(StoreError);
        }
        g.slots = [None, None];
        g.calib_slots = [None, None];
        g.next_slot = 0;
        g.next_seq = 1;
        g.calib_next_slot = 0;
        g.calib_next_seq = 1;
        g.wipes += 1;
        Ok(())
    }
}
