//! RAM-backed [`ConfigStore`] fake: page-sized slots holding real images
//! through the core codec, so a rebuilt servo's boot overlay exercises the
//! same parse/pick path the chip takes. `Shared::seed_store` wants
//! `&'static`, so tests obtain instances via [`RamStore::leak`]; the interior
//! `Mutex` keeps the same reference inspectable (and shareable across a
//! rebuilt `Sim`, modeling a reboot with flash intact).

use std::sync::Mutex;

use osc_servo_core::persist::{
    self, CALIB_IMAGE_LEN, CALIB_LEN, CONFIG_LEN, IMAGE_LEN, PROFILE_LEN, Slot, StoreError,
};
use osc_servo_core::{ConfigStore, ControlTableCell};

const ERASED: [u8; IMAGE_LEN] = [0xFF; IMAGE_LEN];
const CALIB_ERASED: [u8; CALIB_IMAGE_LEN] = [0xFF; CALIB_IMAGE_LEN];

fn idx(slot: Slot) -> usize {
    match slot {
        Slot::A => 0,
        Slot::B => 1,
    }
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
    /// config and calib images and prime both A/B states. Called by
    /// `SimServo::build` before the bus reads the table's comms block; the
    /// caller re-seeds RO calib sense facts after (board data wins).
    pub fn boot_load(&self, table: &ControlTableCell) {
        let mut g = self.inner.lock().unwrap();
        let a = g.slots[0].unwrap_or(ERASED);
        let b = g.slots[1].unwrap_or(ERASED);
        let pick = persist::boot_overlay(table, &a, &b);
        g.next_slot = idx(pick.next_slot);
        g.next_seq = pick.next_seq;
        let a = g.calib_slots[0].unwrap_or(CALIB_ERASED);
        let b = g.calib_slots[1].unwrap_or(CALIB_ERASED);
        let pick = persist::boot_overlay_calib(table, &a, &b);
        g.calib_next_slot = idx(pick.next_slot);
        g.calib_next_seq = pick.next_seq;
    }

    /// Arm every subsequent save/wipe to fail (the chip's readback-verify
    /// miss).
    pub fn set_fail(&self, fail: bool) {
        self.inner.lock().unwrap().fail = fail;
    }

    pub fn saves(&self) -> usize {
        self.inner.lock().unwrap().saves
    }

    pub fn wipes(&self) -> usize {
        self.inner.lock().unwrap().wipes
    }

    /// The slot's stored config image, if any (a copy -- tests parse it at
    /// leisure).
    pub fn slot(&self, slot: Slot) -> Option<[u8; IMAGE_LEN]> {
        self.inner.lock().unwrap().slots[idx(slot)]
    }

    /// The slot's stored calib image, if any (a copy).
    pub fn calib_slot(&self, slot: Slot) -> Option<[u8; CALIB_IMAGE_LEN]> {
        self.inner.lock().unwrap().calib_slots[idx(slot)]
    }

    /// Plant a calib image directly -- tests modeling out-of-band flash
    /// content (a stale image carrying bytes the wire could never write).
    pub fn inject_calib(&self, slot: Slot, seq: u16, calib: &[u8; CALIB_LEN]) {
        let mut img = [0u8; CALIB_IMAGE_LEN];
        persist::calib_assemble(&mut img, seq, calib);
        self.inner.lock().unwrap().calib_slots[idx(slot)] = Some(img);
    }
}

impl ConfigStore for RamStore {
    fn save(
        &self,
        config: &[u8; CONFIG_LEN],
        profile: &[u8; PROFILE_LEN],
        calib: &[u8; CALIB_LEN],
    ) -> Result<(), StoreError> {
        let mut g = self.inner.lock().unwrap();
        if g.fail {
            return Err(StoreError);
        }
        let mut img = [0u8; IMAGE_LEN];
        persist::assemble(&mut img, g.next_seq, config, profile);
        let slot = g.next_slot;
        g.slots[slot] = Some(img);
        g.next_slot ^= 1;
        g.next_seq = g.next_seq.wrapping_add(1);
        let mut img = [0u8; CALIB_IMAGE_LEN];
        persist::calib_assemble(&mut img, g.calib_next_seq, calib);
        let slot = g.calib_next_slot;
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
